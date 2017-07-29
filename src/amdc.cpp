#include <Eigen/Dense>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"

#include "amdc.h"
#include "amdc/PropellerCmd.h"
#include "sensor_util.h"

Amdc amdc_s;

// hardware device handlers
ultrasonic_handler ultrasonic[7];

ros::Publisher propeller_pub;
amdc::PropellerCmd propeller_msg;

ros::Subscriber rc_cond;
ros::Subscriber rc_cmd;

ros::Subscriber vision_subscriber;

void init_ultrasonic(ros::NodeHandle nh)
{
    for (int i = 0; i < 7; ++i)
    {
        ultrasonic[i].id = i;
        ultrasonic[i].subscribe(i, nh, &amdc_s);
    }
}

void remote_controlled_callback(const std_msgs::Bool::ConstPtr& msg)
{
    amdc_s.remote_controlled = msg->data;

    ROS_INFO_COND(amdc_s.remote_controlled, "remote controller attached");
    ROS_INFO_COND(!amdc_s.remote_controlled, "remote controller detached");
}

void rc_propeller_cmd_callback(const amdc::PropellerCmd& msg)
{
    amdc_s.propeller_cmd.left_spd = msg.left_pwm;
    amdc_s.propeller_cmd.right_spd = msg.right_pwm;
    amdc_s.propeller_cmd.update = true;
}

void init_remote_controller(ros::NodeHandle nh)
{
    rc_cond = nh.subscribe("remote_controlled", 1, 
            remote_controlled_callback);
    rc_cmd = nh.subscribe("rc_propeller_cmd", 1, 
            rc_propeller_cmd_callback);
}

void init_vision(ros::NodeHandle nh)
{
    vision_subscriber = nh.subscribe("debris_coord", 1, 
            &Amdc::visionCallback, &amdc_s);
}

void init_propeller(ros::NodeHandle nh)
{
    propeller_pub = nh.advertise<amdc::PropellerCmd>("propeller_cmd", 1000);
}

void init_servo(ros::NodeHandle nh)
{
    // TODO
    // add publisher here
}

void update_actuators()
{
    if (amdc_s.propeller_cmd.update == true)
    {
        propeller_msg.left_pwm = amdc_s.propeller_cmd.left_spd;
        propeller_msg.right_pwm = amdc_s.propeller_cmd.right_spd;
        propeller_msg.left_enable = 1;
        propeller_msg.right_enable = 1;

        propeller_pub.publish(propeller_msg);
        amdc_s.propeller_cmd.update = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "computer_node");
    ros::NodeHandle nh;

    // initialise amdc states
    amdc_s.state << 0,0,0,0,0,0;
    amdc_s.range << 6,6,6,6,6,6,6;

    Eigen::VectorXf goal(2);
    goal << 4,4;
    amdc_s.goals.push(goal);

    // initialise all publisher and subscriber and sensor data
    init_ultrasonic(nh);
    init_remote_controller(nh);
    init_propeller(nh);
    init_servo(nh);
    init_vision(nh);

    ros::Subscriber amdc_state_update_sub = nh.subscribe("odometry_gps", 
            1, &Amdc::stateUpdateCallback, &amdc_s);

    ros::Subscriber amdc_goal_sub = nh.subscribe("target_gps_odometry_odom", 
            1, &Amdc::goalCallback, &amdc_s);

    ros::Subscriber imu_mag_fused_sub = nh.subscribe("imu_mag_fused", 
            1, &Amdc::imuMagFusedCallback, &amdc_s);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (!amdc_s.remote_controlled)
            update_state_machine();
        update_actuators();

        // handle callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
