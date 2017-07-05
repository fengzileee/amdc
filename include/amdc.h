#ifndef amdc__H
#define amdc__H

#include <queue>
#include <Eigen/Dense>
#include <cmath>

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
        state(0) = msg -> PoseWithCovariance.pose.position.x; // x
        state(1) = msg -> PoseWithCovariance.pose.position.y; // y
        // rotation about z
        float qx = msg -> PoseWithCovariance.pose.orientation.x;
        float qy = msg -> PoseWithCovariance.pose.orientation.y;
        float qz = msg -> PoseWithCovariance.pose.orientation.z;
        float qw = msg -> PoseWithCovariance.pose.orientation.w;
        state(2) = std::atan2(2 * (qw * qz + qx * qy), 
                1 - 2 * (qy * qy + qz * qz));
        // ========== velocity: time derivative of position ============
        state(3) = msg -> TwistWithCovariance.twist.linear.x; 
        state(4) = msg -> TwistWithCovariance.twist.linear.y; 
        state(5) = msg -> TwistWithCovariance.twist.angular.z; 
    }

};

extern Amdc amdc_s;

void update_state_machine();

#endif
