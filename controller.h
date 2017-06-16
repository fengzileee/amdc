#ifndef controller_H
#define controller_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace controller
{

using namespace Eigen;

static double PI = 3.1415926535897931;

// ============== parameters ===============
static const float ARRIVE_RANGE = 0.25,
             BOUNDARY_FOLLOWING_THRESH = 0.7,
             RUN_AWAY_THRESH = 0.4,
             CLOSE_DIST = 1., // when to slow down
             DIV_SPEED = 0.1, 
             OA_SPEED = 0.2,
             NAV_SPEED = 0.2,
             U_LIMIT = 1;

static const float sensor_angles_array[7] = {-2.2, -1.2, -0.5, 0., 0.5, 1.2, 2.2};

static const VectorXf sensor_angles(7); 
static const int sensor_num = sensor_angles.size();
static const float sensor_cap = 0.9;

static const struct
{
    const float m; // mass
    const float I; // inertia
    const float l; // distance between the propellers
} robot_para = 
{
    1, 
    1, 
    1
};

// ======================================
float atan2_angle(float theta)
{
    return std::atan2(std::sin(theta), std::cos(theta));
}

static Matrix2f global2local(const float& theta)
{
    float T11, T12, T21, T22;
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


VectorXf controller_vw2u(const VectorXf& state, 
        const float v_d, 
        const float omega_d)
{
    static float conv_rate_linear = 1.2;
    static float conv_rate_angular = 4;

    float v_actual = std::cos(state(2)) * state(3) 
        + std::sin(state(2)) * state(4);
    float err_v = v_d - v_actual; 
    float err_omega = omega_d - state(5); 

    float sum_u = conv_rate_linear * robot_para.m * err_v; 
    float diff_u = conv_rate_angular * 2 * err_omega * robot_para.I / robot_para.l; 

    VectorXf u(2); 
    u << 0.5 * (sum_u + diff_u), 
      0.5 * (sum_u - diff_u);

    control_input_cap(u);
    return u;
}

VectorXf controller_oa(VectorXf& state, 
        VectorXf& point_sensor_readings)
{
    // gains of this controller
    static const float k_p_turn = 0.5,
                 k_d_turn = 0.5; 

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
        omega_d = k_p_turn * heading_err + k_d_turn * dheading_err;
    }
    else
    {
        v_d = OA_SPEED;
        omega_d = k_p_turn * heading_err_inv + k_d_turn * dheading_err;
    }

    return controller_vw2u(state, v_d, omega_d);
}

VectorXf controller_div(VectorXf state, 
        VectorXf& point_sensor_readings)
{
    // gains of this controller, and controller parameters
    static const float k_p_turn = 0.25, 
                 k_d_turn = 0.25,
                 k_lateral_p = 4; 

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

    // obstacles aside 
    /*
    VectorXf scale_side = sensor_angles.array().sin();
    float obs_side_x = (detectionX.array() * scale_side.array()).sum();
    float obs_side_y = (detectionY.array() * scale_side.array()).sum();
    VectorXf obs_side(2);
    obs_side << obs_side_x, obs_side_y;
    float obs_side_angle = std::atan2(obs_side_y, obs_side_y);
    float obs_side_norm = obs_side.norm();
    */

    // overall obstacle
    VectorXf obstacle_all(2), obstacle_all_unit(2);
    obstacle_all << detectionX.sum(), detectionY.sum(); 
    obstacle_all_unit = obstacle_all.array() / obstacle_all.norm();

    float heading_err;
    // decide the control input
    if (obs_ahead_unit.dot(obstacle_all_unit) > 0.86)
    {
        // if (obs_side_norm < side_activate_thresh)
        // {
            if (obs_ahead_angle > 0)
                heading_err = obs_ahead_angle - PI / 2;
            else
                heading_err = obs_ahead_angle + PI / 2;
        /*
        }
        else
        {
            if (obs_side_angle > 0)
                heading_err = obs_ahead_angle - PI / 2;
            else
                heading_err = obs_ahead_angle + PI / 2;
        }
        */
    }
    else
        heading_err = - obs_ahead_angle;

    heading_err = heading_err - k_lateral_p * v_lateral; 
    heading_err = atan2_angle(heading_err); 
    float dheading_err = state(5); 

    float v_d = DIV_SPEED; 
    float omega_d = k_p_turn * heading_err + k_d_turn * dheading_err; 
    return controller_vw2u(state, v_d, omega_d);
}

VectorXf nav_controller(VectorXf& state, 
        VectorXf& ref, VectorXf point_sensor_readings)
{
    static VectorXf sensor_angles(sensor_num);
    float* sensor_angles_array = sensor_angles.data();
    static const float k_p_turn = 0.5, 
                 k_d_turn = 0.5, 
                 k_lateral_p = 4;

    VectorXf u;
    VectorXf x = state.head(2);
    VectorXf goal_vector = ref - x;

    float min_point_sensor_reading = point_sensor_readings.minCoeff(); 
    bool boundary_following_check, run_away_check, stay_at_goal_check;
    run_away_check = min_point_sensor_reading < RUN_AWAY_THRESH;
    boundary_following_check = min_point_sensor_reading < BOUNDARY_FOLLOWING_THRESH; 
    stay_at_goal_check = goal_vector.norm() < ARRIVE_RANGE;

    point_sensor_readings = (point_sensor_readings.array() > sensor_cap)
        .select(0, point_sensor_readings);

    if (run_away_check)
        u = controller_oa(state, sensor_angles);
    else if (boundary_following_check)
        u = controller_div(state, point_sensor_readings);
    else 
    {
        VectorXf local_v = global2local_v(state); 
        float& v_heading = local_v(0); 
        float& v_lateral = local_v(1);
        float heading_d = std::atan2(goal_vector(1), goal_vector(0)); 
        float heading_err; 
        heading_err = heading_d - state(2) - k_lateral_p * v_lateral; 
        heading_err = atan2_angle(heading_err);
        float dheading_err = state(5); 
        float v_d = goal_vector.norm() < CLOSE_DIST ? goal_vector.norm()/5 : NAV_SPEED;
        float omega_d;
        if (stay_at_goal_check)
        {
            // stay at goal controller
            if (std::cos(heading_err > 0))
            {
                float heading_err_inv = heading_err + PI + 2 * k_lateral_p * v_lateral; 
                heading_err_inv = atan2_angle(heading_err_inv);
                v_d = - v_d; 
                omega_d = k_p_turn * heading_err_inv + k_d_turn * dheading_err;
            }
        }
        else
            // go2goal
            omega_d = k_p_turn * heading_err + k_d_turn * dheading_err;

        u = controller_vw2u(state, v_d, omega_d);
    }

    return u;
}

}
#endif
