#ifndef controller_H
#define controller_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace controller
{
    using namespace Eigen;
    using namespace std;

    enum controller_state 
    {
        stay_state = 0, 
        oa_state = 1, 
        div_state = 2, 
        nav_state = 3
    };
   
    struct gains
    {
        /*
        p:  proportional gain 
        d:  derivative gain 
        lp: lateral velocity proportional gain
        */
        float p, d;
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
        VectorXf ema, raw_ema;
        float ang, ahead_ang, aheadX, aheadY, detX_sum, detY_sum, min;
    };

    struct goal_perception
    {
        VectorXf vec, vec_unit, l_vec, l_vec_unit; 
        float ang, vnorm;
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
                  U_LIMIT,
                  EMA_K,
                  GOAL_P;

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
            goal_perception gperc;
            controller_state cstate;

            // ======================================
            float atan2_angle(float theta)
            {
                return atan2(sin(theta), cos(theta));
            }

            Matrix2f rotation(const float& theta)
            {
                static float T11, T12, T21, T22;
                T11 = cos(theta);
                T12 = -sin(theta);
                T21 = sin(theta);
                T22 = cos(theta); 
                Matrix2f T; 
                T << T11, T12, 
                  T21, T22;
                return T;
            }

            Matrix2f global2local(const float& theta)
            {
                static float T11, T12, T21, T22;
                T11 = cos(theta);
                T12 = sin(theta);
                T21 = - sin(theta);
                T22 = cos(theta);
                Matrix2f T;
                T <<  T11, T12, 
                  T21, T22;
                return T;
            }

            VectorXf global2local_v(const VectorXf& state)
            {
                VectorXf global_v = state.segment(3, 2);
                Matrix2f T = global2local(state(2));
                VectorXf local_v = T * global_v; 
                return local_v;
            }

            void update_obstacle_perception(VectorXf& state, 
                    VectorXf& point_sensor_readings)
            {
                for (int it = 0; it < 7; it ++)
                {
                    if (operc.raw_ema(it) >= point_sensor_readings(it))
                        operc.raw_ema(it) = point_sensor_readings(it);
                    else
                        operc.raw_ema(it) =EMA_K * operc.raw_ema(it) + (1 - EMA_K) * 
                            point_sensor_readings(it); 
                }

                operc.min = operc.raw_ema.minCoeff(); 

                operc.ema = (operc.raw_ema.array() > sensor_cap)
                    .select(0, sensor_cap - operc.raw_ema.array());

                operc.detX = operc.ema.array() * sensor_angles.array().cos();
                operc.detY = operc.ema.array() * sensor_angles.array().sin();
                operc.detX_sum = operc.detX.array().sum();
                operc.detY_sum = operc.detY.array().sum();
                operc.all(0) = operc.detX_sum;
                operc.all(1) = operc.detY_sum;
                operc.all_unit = operc.ahead.array() / operc.ahead.norm();
                operc.ang = atan2(operc.detY_sum, operc.detX_sum);

                operc.aheadX = (operc.detX.array() * obstacle_sensor_scale.array()).sum();
                operc.aheadY = (operc.detY.array() * obstacle_sensor_scale.array()).sum();
                operc.ahead(0) = operc.aheadX;
                operc.ahead(1) = operc.aheadY;
                operc.ahead_unit = operc.ahead.array() / operc.ahead.norm();
                operc.ahead_ang = atan2(operc.aheadY, operc.aheadX);

            }

            void update_goal_perception(VectorXf& state, 
                    VectorXf& ref)
            {
                gperc.vec = ref - state.head(2);
                gperc.vec_unit = gperc.vec.array() / gperc.vec.norm(); 
                gperc.ang = atan2(gperc.vec_unit(1), gperc.vec_unit(0)) - state(2);
                gperc.vnorm = gperc.vec.norm();
                Matrix2f T = global2local(state(2));
                gperc.l_vec = T * gperc.vec;
                gperc.l_vec_unit = T * gperc.vec_unit;
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

                float err_omega = omega_d - state(5); 

                float sum_u = conv_rate_linear * par.m * v_d; 
                float diff_u = conv_rate_angular * 2 * err_omega * par.I / par.l; 

                VectorXf u(2); 
                u << 0.5 * (sum_u + diff_u), 
                  0.5 * (sum_u - diff_u);

                control_input_cap(u);
                return u;
            }

            VectorXf run_away(VectorXf& state)
            {
                float heading_err, heading_err_inv;
                float dheading_err = state(5);
                heading_err = operc.ang;
                heading_err_inv = heading_err + PI; 
                heading_err_inv = atan2_angle(heading_err_inv); 

                float v_d, omega_d;
                if (cos(heading_err) > 0) 
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

            VectorXf divert(VectorXf& state)
            {
                // local heading vector computed using sensor info
                VectorXf heading_div(2);
                if (operc.ahead_unit.dot(operc.all_unit) > 0.86)
                {
                    if (operc.ahead_ang > 0)
                        heading_div = rotation(-PI/2) * operc.ahead_unit;
                    else 
                        heading_div = rotation(PI/2) * operc.ahead_unit;
                }
                else
                {
                    heading_div(0) = operc.ahead_unit(0);
                    heading_div(1) = -operc.ahead_unit(1);
                }

                // blending. vectors all in lcoal frame.
                if (gperc.l_vec_unit.dot(operc.all_unit) >= 0)
                {
                    VectorXf heading_d = gperc.l_vec_unit * GOAL_P 
                        + heading_div * (1 - GOAL_P); 
                    float heading_d_ang = atan2(heading_d(1), heading_d(0)); 

                    float heading_err;
                    heading_err = heading_d_ang - state(2); 
                    heading_err = atan2_angle(heading_err);
                    float dheading_err = state(5);

                    float v_d = DIV_SPEED; 
                    float omega_d = div_gains.p * heading_err + div_gains.d * dheading_err; 

                    return vw2u(state, v_d, omega_d);
                }
                else
                    return go2goal(state);
            }

            VectorXf stayATgoal(VectorXf& state) 
            {
                float heading_err = gperc.ang;
                float dheading_err = state(5); 
                float v_d = gperc.vnorm < CLOSE_DIST ? 
                    NAV_SPEED * gperc.vnorm / CLOSE_DIST : NAV_SPEED;
                float omega_d;
                if (cos(heading_err < 0))
                {
                    float heading_err_inv = heading_err + PI; 
                    heading_err_inv = atan2_angle(heading_err_inv);
                    v_d = - v_d; 
                    omega_d = stay_gains.p * heading_err_inv + stay_gains.d * dheading_err;
                }
                else
                    omega_d = stay_gains.p * heading_err + stay_gains.d * dheading_err;

                VectorXf u = vw2u(state, v_d, omega_d);

                return u;
            }

            VectorXf go2goal(VectorXf& state)
            {
                float heading_err = gperc.ang; 
                float dheading_err = state(5); 
                float v_d = gperc.vec.norm() < CLOSE_DIST ? 
                    NAV_SPEED * gperc.vec.norm() / CLOSE_DIST : NAV_SPEED;
                float omega_d;
                omega_d = g2g_gains.p * heading_err + g2g_gains.d * dheading_err;

                VectorXf u = vw2u(state, v_d, omega_d);

                return u;
            }


        public:

            VectorXf compute_u(VectorXf& state, 
                    VectorXf ref, VectorXf point_sensor_readings)
            { 
                VectorXf u;

                // update robot's perception
                update_obstacle_perception(state, point_sensor_readings);
                update_goal_perception(state, ref);

                if (gperc.vnorm < ARRIVE_RANGE)
                    cstate = stay_state; 
                else if (operc.min < RUN_AWAY_THRESH - STATE_SWITCH_BUFFER) 
                    cstate = oa_state;
                else if (RUN_AWAY_THRESH + STATE_SWITCH_BUFFER <= 
                        operc.min)
                {
                    if (operc.min < BOUNDARY_FOLLOWING_THRESH - 
                            STATE_SWITCH_BUFFER)
                        cstate = div_state; 
                    else if (BOUNDARY_FOLLOWING_THRESH + 
                            STATE_SWITCH_BUFFER <= operc.min) 
                        cstate = nav_state; 
                }
                cout << cstate << endl;


                if (cstate == oa_state)
                    u = run_away(state);
                else if (cstate == div_state)
                    u = divert(state);
                else if (cstate == stay_state)
                    u = stayATgoal(state);
                else if (cstate == nav_state)
                    u = go2goal(state);

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
                RUN_AWAY_THRESH(1.5), 
                STATE_SWITCH_BUFFER(0.25), 
                CLOSE_DIST(1), 
                DIV_SPEED(0.1), OA_SPEED(0.2), 
                NAV_SPEED(0.2), U_LIMIT(64), 
                sensor_angles_array({2.2689, 1.2217, 0.5236, 0., -0.5236, -1.2217, -2.2689}),
                sensor_cap(4.8), oa_gains({0.5,0.5}),
                div_gains({0.25, 0.25}), 
                stay_gains({0.5, 0.5}),
                g2g_gains({0.5, 0.5}), 
                par({100, 75, 1}), 
                EMA_K(0.99),
                GOAL_P(0.25),
                cstate(nav_state)
            {
                sensor_angles.resize(7);
                operc.raw_ema.resize(7);
                operc.ema.resize(7);
                for (int it = 0; it < 7; it ++)
                {
                    operc.raw_ema(it) = 5;
                    sensor_angles(it) = sensor_angles_array[it];
                }

                obstacle_sensor_scale.resize(7); 
                obstacle_sensor_scale = sensor_angles.array().cos().matrix();

                operc.all.resize(2);
                operc.all_unit.resize(2);
                operc.ahead.resize(2);
                operc.ahead_unit.resize(2);

                gperc.vec.resize(2);
                gperc.vec_unit.resize(2);
                gperc.l_vec.resize(2);
                gperc.l_vec_unit.resize(2);
            }


    };

}
#endif
