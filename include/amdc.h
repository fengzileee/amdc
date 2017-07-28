#ifndef amdc__H
#define amdc__H

#include <queue>
#include <Eigen/Dense>
#include <cmath>

#include "geometry_msgs/PointStamped.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "controller.h" 
#include "kalman.h"

struct PropellerCommand
{
    int16_t left_spd;
    int16_t right_spd;
    bool update;
};

struct ServoCommand
{
    int16_t left_angle;
    int16_t right_angle;
    bool update;
};

class Amdc
{
public:
    std::queue<Eigen::VectorXf> goals;
    Eigen::VectorXf state;
    Eigen::VectorXf range_raw, range; //raw and kf ultrasonic readings
    Eigen::VectorXf debris_coord;
    controller::Controller controller_nav;
    controller::Controller_VS controller_vs;
    KF kf[7]; 
    PropellerCommand propeller_cmd;
    ServoCommand servo_cmd;
    bool remote_controlled;

    Amdc()
    {
        state.resize(6);
        range.resize(7);
        range_raw.resize(7);
        debris_coord.resize(2);
    }

    void stateUpdateCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // ================================================
        // =    Update the state of the robot through a ROS
        // =        message with type nav_msgs/Odometry.
        // ================================================

        // =========== position: x, y, theta =============
        state(0) = msg -> pose.pose.position.x; // x
        state(1) = msg -> pose.pose.position.y; // y
        // rotation about z
        // float qx = msg -> pose.pose.orientation.x;
        // float qy = msg -> pose.pose.orientation.y;
        // float qz = msg -> pose.pose.orientation.z;
        // float qw = msg -> pose.pose.orientation.w;
        //state(2) = std::atan2(2 * (qw * qz + qx * qy), 
                //1 - 2 * (qy * qy + qz * qz));
        // ========== velocity: time derivative of position ============
        state(3) = msg -> twist.twist.linear.x; 
        state(4) = msg -> twist.twist.linear.y; 
        state(5) = msg -> twist.twist.angular.z; 
    }

    void imuMagFusedCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // We use the orientation from fused reading from IMU and magnetometer
        // instead of the KF node.
        float qx = msg -> orientation.x;
        float qy = msg -> orientation.y;
        float qz = msg -> orientation.z;
        float qw = msg -> orientation.w;
        state(2) = std::atan2(2 * (qw * qz + qx * qy), 
                1 - 2 * (qy * qy + qz * qz));
    }

    /**
     * Callback function that adds a new goal that was published on 
     * /target_gps_odometry_odom.
     * \param in   ROS PointStamped message
     */
    void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        Eigen::VectorXf goal(2);
        goal << msg->point.x, msg->point.y;
        goals.push(goal);
    }

};

extern Amdc amdc_s;

void update_state_machine();

#endif
