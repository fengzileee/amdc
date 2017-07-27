#ifndef amdc__H
#define amdc__H

#include <queue>
#include <Eigen/Dense>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

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
    Eigen::VectorXf range;
    Eigen::VectorXf debris_coord;
    PropellerCommand propeller_cmd;
    ServoCommand servo_cmd;
    bool remote_controlled;

    Amdc()
    {
        state.resize(6);
        range.resize(7);
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
        float qx = msg -> pose.pose.orientation.x;
        float qy = msg -> pose.pose.orientation.y;
        float qz = msg -> pose.pose.orientation.z;
        float qw = msg -> pose.pose.orientation.w;
        state(2) = std::atan2(2 * (qw * qz + qx * qy), 
                1 - 2 * (qy * qy + qz * qz));
        // ========== velocity: time derivative of position ============
        state(3) = msg -> twist.twist.linear.x; 
        state(4) = msg -> twist.twist.linear.y; 
        state(5) = msg -> twist.twist.angular.z; 
    }

    void vision_callback(const geometry_msgs::Point::ConstPtr& msg)
    {
        ROS_ASSERT(msg->x >= 0 && msg->y >= 0);
        amdc_s.debris_coord(0) = msg->x;
        amdc_s.debris_coord(1) = msg->y;
    }

};

extern Amdc amdc_s;

void update_state_machine();

#endif
