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
        /*
        p:  proportional gain 
        d:  derivative gain 
        lp: lateral velocity proportional gain
        */
        float p, d, lp;
    };

    struct robot_parameter
    {
        float m, I, l;
    };

    struct obstacle_perception
    {
        /* 
        detX (detY):            x (y) coordinate of detected obstacles
        detX_sum (detY_sum):    summation of all sensors' x (y) coordinates
        */
        VectorXf detX, detY; 
        VectorXf ahead, all, ahead_unit, all_unit;
        float ang, ahead_ang, aheadX, aheadY, detX_sum, detY_sum;
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
            VectorXf obstacle_sensor_scale;
            const float sensor_angles_array[7];
            const float sensor_cap;
            const gains oa_gains;
            const gains div_gains;
            const gains stay_gains;
            const gains g2g_gains;
            robot_parameter par;

            // ============== states (robot and environment) ===============
            obstacle_perception operc; 

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

            void update_obstacle_perception(VectorXf& state, 
                    VectorXf& point_sensor_readings)
            {
                operc.detX = point_sensor_readings.array() * sensor_angles.array().cos();
                operc.detY = point_sensor_readings.array() * sensor_angles.array().sin();
                operc.detX_sum = (point_sensor_readings.array() * sensor_angles.array().cos()).sum();
                operc.detY_sum = (point_sensor_readings.array() * sensor_angles.array().sin()).sum();
                operc.all(0) = operc.detX_sum;
                operc.all(1) = operc.detY_sum;
                operc.all_unit = operc.ahead.array() / operc.ahead.norm();
                operc.ang = std::atan2(operc.detY_sum, operc.detX_sum);

                operc.aheadX = (operc.detX.array() * obstacle_sensor_scale.array()).sum();
                operc.aheadY = (operc.detY.array() * obstacle_sensor_scale.array()).sum();
                operc.ahead(0) = operc.aheadX;
                operc.ahead(1) = operc.aheadY;
                operc.ahead_unit = operc.ahead.array() / operc.ahead.norm();
                operc.ahead_ang = std::atan2(operc.aheadY, operc.aheadX);
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

            VectorXf run_away(VectorXf& state, 
                    VectorXf& point_sensor_readings)
            {
                update_obstacle_perception(state, point_sensor_readings);

                float heading_err, heading_err_inv;
                float& dheading_err = state(5);
                heading_err = operc.ang;
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

            VectorXf divert(VectorXf& state, 
                    VectorXf& point_sensor_readings)
            {
                update_obstacle_perception(state, point_sensor_readings);

                VectorXf local_v; 
                local_v = global2local_v(state);

                float heading_err;
                // decide the control input
                if ((operc.ahead_unit.dot(operc.all_unit) > 0.86) &&
                        (operc.ahead_ang > 0))
                {
                    heading_err = operc.ahead_ang - PI / 2;
                }
                else
                    heading_err = - operc.ahead_ang;

                heading_err = heading_err - div_gains.lp * local_v(1); 
                heading_err = atan2_angle(heading_err); 
                float dheading_err = state(5); 

                float v_d = DIV_SPEED; 
                float omega_d = div_gains.p * heading_err + div_gains.d * dheading_err; 
                return vw2u(state, v_d, omega_d);
            }

            VectorXf stayATgoal(VectorXf& state, 
                    VectorXf& ref)
            {
                VectorXf x = state.head(2);
                VectorXf goal_vector = ref - x;
                VectorXf local_v = global2local_v(state); 

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

            VectorXf go2goal(VectorXf& state, 
                    VectorXf& ref)
            {
                VectorXf x = state.head(2);
                VectorXf goal_vector = ref - x;
                VectorXf local_v = global2local_v(state); 

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

            VectorXf compute_u(VectorXf& state, 
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
                    u = run_away(state, point_sensor_readings);
                else if (div_state)
                    u = divert(state, point_sensor_readings);
                else 
                {
                    if (stay_state)
                        u = stayATgoal(state, ref);
                    else
                        u = go2goal(state, ref);

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
                obstacle_sensor_scale.resize(7); 
                obstacle_sensor_scale = sensor_angles.array().cos().matrix();
                operc.all.resize(2);
                operc.ahead.resize(2);
            }


    };

}
#endif
