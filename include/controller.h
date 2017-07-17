#ifndef controller_H
#define controller_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace controller
{

    using namespace Eigen;

    struct gains
    {
        float p, d, lp;
    };

    struct robot_parameter
    {
        float m, I, l;
    };

    struct perception
    {
        VectorXf goal_vector; 
        float heading_d, dheading_err, heading_err; 
    };

    class Controller
    {

        protected: 
            const double PI;

            // ============== parameters ===============
            const float ARRIVE_RANGE,
                  BOUNDARY_FOLLOWING_THRESH,
                  RUN_AWAY_THRESH,
                  STATE_SWITCH_BUFFER,
                  CLOSE_DIST, // when to slow down
                  DIV_SPEED, 
                  OA_SPEED,
                  NAV_SPEED,
                  U_LIMIT;

            VectorXf sensor_angles; 
            const float sensor_angles_array[7];
            const float sensor_cap;
            const gains oa_gains;
            const gains div_gains;
            const gains stay_gains;
            const gains g2g_gains;
            robot_parameter par;

            // ======================================
            float atan2_angle(float theta)
            {
                return std::atan2(std::sin(theta), std::cos(theta));
            }

            static Matrix2f global2local(const float& theta)
            {
                static float T11, T12, T21, T22;
                T11 = std::cos(theta);
                T12 = std::sin(theta);
                T21 = - std::sin(theta);
                T22 = std::cos(theta);
                Matrix2f T;
                T <<  T11, T12, 
                  T21, T22;
                return T;
            }

            static VectorXf global2local_v(const VectorXf& state)
            {
                VectorXf global_v = state.segment(3, 2);
                Matrix2f T = global2local(state(2));
                VectorXf local_v = T * global_v; 
                return local_v;
            }

            void control_input_cap(VectorXf& u)
            {
                u(0) = u(0) > U_LIMIT ? U_LIMIT : 
                    (u(0) < - U_LIMIT ? -U_LIMIT : u(0));
                u(1) = u(1) > U_LIMIT ? U_LIMIT : 
                    (u(1) < - U_LIMIT ? -U_LIMIT : u(1));
            }
            VectorXf vw2u(const VectorXf& state, 
                    const float v_d, 
                    const float omega_d)
            {
                static float conv_rate_linear = 1.2;
                static float conv_rate_angular = 4;

                float v_actual = std::cos(state(2)) * state(3) 
                    + std::sin(state(2)) * state(4);
                float err_v = v_d - v_actual; 
                float err_omega = omega_d - state(5); 

                float sum_u = conv_rate_linear * par.m * err_v; 
                float diff_u = conv_rate_angular * 2 * err_omega * par.I / par.l; 

                VectorXf u(2); 
                u << 0.5 * (sum_u + diff_u), 
                  0.5 * (sum_u - diff_u);

                control_input_cap(u);
                return u;
            }

            VectorXf controller_oa(VectorXf& state, 
                    VectorXf& point_sensor_readings)
            {
                VectorXf detectionX, detectionY, local_v; 
                detectionX = point_sensor_readings.array() * sensor_angles.array().cos();
                detectionY = point_sensor_readings.array() * sensor_angles.array().sin();
                local_v = global2local_v(state);

                float& v_heading = local_v(0); 
                float& v_lateral = local_v(1);
                float detectionX_sum = detectionX.sum(),
                      detectionY_sum = detectionY.sum();
                float obstacle_angle = std::atan2(detectionY_sum, detectionX_sum); 

                float heading_err, heading_err_inv;
                float& dheading_err = state(5);
                heading_err = obstacle_angle;
                heading_err_inv = heading_err + PI; 
                heading_err_inv = atan2_angle(heading_err_inv); 

                float v_d, omega_d;
                if (std::cos(heading_err) > 0) 
                {
                    v_d = -OA_SPEED;
                    omega_d = oa_gains.p * heading_err + oa_gains.d * dheading_err;
                }
                else
                {
                    v_d = OA_SPEED;
                    omega_d = oa_gains.p * heading_err_inv + oa_gains.d * dheading_err;
                }

                return vw2u(state, v_d, omega_d);
            }

            VectorXf controller_div(VectorXf state, 
                    const VectorXf& point_sensor_readings)
            {
                // gains of this controller, and controller parameters

                VectorXf detectionX, detectionY, local_v; 
                detectionX = point_sensor_readings.array() * sensor_angles.array().cos();
                detectionY = point_sensor_readings.array() * sensor_angles.array().sin();
                local_v = global2local_v(state);
                float& v_heading = local_v(0); 
                float& v_lateral = local_v(1);

                // obstacles ahead
                VectorXf scale_ahead = sensor_angles.array().cos().matrix(); 
                float obs_ahead_x = (detectionX.array() * scale_ahead.array()).sum();
                float obs_ahead_y = (detectionY.array() * scale_ahead.array()).sum();
                VectorXf obs_ahead(2);
                obs_ahead << obs_ahead_x, obs_ahead_y; 
                VectorXf obs_ahead_unit = obs_ahead.array() / obs_ahead.norm();
                float obs_ahead_angle = std::atan2(obs_ahead_y, obs_ahead_x);

                // overall obstacle
                VectorXf obstacle_all(2), obstacle_all_unit(2);
                obstacle_all << detectionX.sum(), detectionY.sum(); 
                obstacle_all_unit = obstacle_all.array() / obstacle_all.norm();

                float heading_err;
                // decide the control input
                if ((obs_ahead_unit.dot(obstacle_all_unit) > 0.86) &&
                        (obs_ahead_angle > 0))
                {
                    heading_err = obs_ahead_angle - PI / 2;
                }
                else
                    heading_err = - obs_ahead_angle;

                heading_err = heading_err - div_gains.lp * v_lateral; 
                heading_err = atan2_angle(heading_err); 
                float dheading_err = state(5); 

                float v_d = DIV_SPEED; 
                float omega_d = div_gains.p * heading_err + div_gains.d * dheading_err; 
                return vw2u(state, v_d, omega_d);
            }

            VectorXf controller_stay(VectorXf& state, 
                    VectorXf& ref, VectorXf point_sensor_readings)
            {
                VectorXf x = state.head(2);
                VectorXf goal_vector = ref - x;
                VectorXf local_v = global2local_v(state); 
                float min_point_sensor_reading = point_sensor_readings.minCoeff(); 
                point_sensor_readings = (point_sensor_readings.array() > sensor_cap)
                    .select(0, sensor_cap - point_sensor_readings.array());

                float& v_heading = local_v(0); 
                float& v_lateral = local_v(1);
                float heading_d = std::atan2(goal_vector(1), goal_vector(0)); 
                float heading_err; 
                heading_err = heading_d - state(2) - stay_gains.lp * v_lateral; 
                heading_err = atan2_angle(heading_err);
                float dheading_err = state(5); 
                float v_d = goal_vector.norm() < CLOSE_DIST ? 
                    NAV_SPEED * goal_vector.norm() / CLOSE_DIST : NAV_SPEED;
                float omega_d;
                if (std::cos(heading_err > 0))
                {
                    float heading_err_inv = heading_err + PI + 2 * stay_gains.lp * v_lateral; 
                    heading_err_inv = atan2_angle(heading_err_inv);
                    v_d = - v_d; 
                    omega_d = stay_gains.p * heading_err_inv + stay_gains.d * dheading_err;
                }
                else
                    omega_d = stay_gains.p * heading_err + stay_gains.d * dheading_err;

                VectorXf u = vw2u(state, v_d, omega_d);

                return u;
            }

            VectorXf controller_g2g(VectorXf& state, 
                    VectorXf& ref, VectorXf point_sensor_readings)
            {
                VectorXf x = state.head(2);
                VectorXf goal_vector = ref - x;
                VectorXf local_v = global2local_v(state); 
                float min_point_sensor_reading = point_sensor_readings.minCoeff(); 
                point_sensor_readings = (point_sensor_readings.array() > sensor_cap)
                    .select(0, sensor_cap - point_sensor_readings.array());

                float& v_heading = local_v(0); 
                float& v_lateral = local_v(1);
                float heading_d = std::atan2(goal_vector(1), goal_vector(0)); 
                float heading_err; 
                heading_err = heading_d - state(2) - g2g_gains.lp * v_lateral; 
                heading_err = atan2_angle(heading_err);
                float dheading_err = state(5); 
                float v_d = goal_vector.norm() < CLOSE_DIST ? 
                    NAV_SPEED * goal_vector.norm() / CLOSE_DIST : NAV_SPEED;
                float omega_d;
                omega_d = g2g_gains.p * heading_err + g2g_gains.d * dheading_err;

                VectorXf u = vw2u(state, v_d, omega_d);
                return u;
            }


        public:

            VectorXf nav_controller(VectorXf& state, 
                    VectorXf& ref, VectorXf point_sensor_readings)
            { 
                enum controller_state 
                {
                    stay_state, 
                    oa_state, 
                    div_state, 
                    nav_state
                };

                static controller_state cstate = nav_state;

                VectorXf u;
                VectorXf x = state.head(2);
                VectorXf goal_vector = ref - x;

                float min_point_sensor_reading = point_sensor_readings.minCoeff(); 

                point_sensor_readings = (point_sensor_readings.array() > sensor_cap)
                    .select(0, sensor_cap - point_sensor_readings.array());

                if (goal_vector.norm() < ARRIVE_RANGE)
                    cstate = stay_state; 
                else if (min_point_sensor_reading < RUN_AWAY_THRESH - STATE_SWITCH_BUFFER) 
                    cstate = oa_state;
                else if (RUN_AWAY_THRESH + STATE_SWITCH_BUFFER <= 
                        min_point_sensor_reading)
                {
                    if (min_point_sensor_reading < BOUNDARY_FOLLOWING_THRESH - 
                            STATE_SWITCH_BUFFER)
                        cstate = div_state; 
                    else if (BOUNDARY_FOLLOWING_THRESH + 
                            STATE_SWITCH_BUFFER <= min_point_sensor_reading) 
                        cstate = nav_state; 
                }

                if (oa_state)
                    u = controller_oa(state, point_sensor_readings);
                else if (div_state)
                    u = controller_div(state, point_sensor_readings);
                else 
                {
                    if (stay_state)
                        u = controller_stay(state, ref, point_sensor_readings);
                    else
                        u = controller_g2g(state, ref, point_sensor_readings);

                }

                return u;
            }

            VectorXf u2pwm(const VectorXf& u)
            {
                static const int PWM_LIMIT = 230;
                int pwm_right = u(0) * PWM_LIMIT / U_LIMIT; 
                int pwm_left = u(1) * PWM_LIMIT / U_LIMIT; 
                VectorXf ret(2);
                ret << pwm_left, pwm_right; 
                return ret;
            }


            Controller(): 
                PI(3.1415926535897931), 
                ARRIVE_RANGE(0.5),
                BOUNDARY_FOLLOWING_THRESH(4), 
                RUN_AWAY_THRESH(2.75), 
                STATE_SWITCH_BUFFER(0.5), 
                CLOSE_DIST(1), 
                DIV_SPEED(0.1), OA_SPEED(0.2), 
                NAV_SPEED(0.2), U_LIMIT(64), 
                sensor_angles_array({-2.2, -1.2, -0.5, 0., 0.5, 1.2, 2.2}),
                sensor_cap(4.8), oa_gains({0.5,0.5,0}),
                div_gains({0.25, 0.25, 3}), 
                stay_gains({0.5, 0.5, 3}),
                g2g_gains({0.5, 0.5, 3}), 
                par({100, 75, 1})
            {
                sensor_angles.resize(7);
                for (int it = 0; it < 7; it ++)
                    sensor_angles(it) = sensor_angles_array[it];
            }


    };

}
#endif
