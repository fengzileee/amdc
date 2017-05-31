#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_util.h"

// sensors
ultrasonic_handler ultrasonic[7];
imu_handler imu;

void init_ultrasonic(ros::NodeHandle nh)
{
    for (int i = 1; i <= 7; ++i)
    {
        ultrasonic[i - 1].subscribe(i, nh);
        ultrasonic[i - 1].advertise(i, nh);
    }
}

void init_imu(ros::NodeHandle nh)
{
    // TODO
    // add subscriber here
    imu.subscribe(nh);
    imu.advertise(nh);
}

void init_gps(ros::NodeHandle nh)
{
    // TODO
    // add subscriber here
}

void init_vision(ros::NodeHandle nh)
{
    // TODO
    // add subscriber here
}

void init_propeller(ros::NodeHandle nh)
{
    // TODO
    // add publisher here
}

void init_servo(ros::NodeHandle nh)
{
    // TODO
    // add publisher here
}

void do_stuff()
{
    // TODO
    // this is probably where we're going to do the computations?
    float a = ultrasonic[0].distance + 1;
}

void publish_propeller_cmd()
{
    // TODO
}

void publish_servo_cmd()
{
    // TODO
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "computer_node");
    ros::NodeHandle nh;

    // initialise all publisher and subscriber and sensor data
    init_ultrasonic(nh);
    init_imu(nh);
    init_gps(nh);
    init_propeller(nh);
    init_servo(nh);
    init_vision(nh);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        do_stuff();
        publish_propeller_cmd();
        publish_servo_cmd();

        // handle callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
