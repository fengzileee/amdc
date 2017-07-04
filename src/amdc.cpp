#include <Eigen/Dense>

#include "ros/ros.h"
#include "ros/console.h"
#include "amdc.h"
#include "sensor_util.h"

Amdc amdc_s;

// hardware device handlers
ultrasonic_handler ultrasonic[7];

void init_ultrasonic(ros::NodeHandle nh)
{
    for (int i = 0; i < 7; ++i)
    {
        ultrasonic[i].id = i;
        ultrasonic[i].subscribe(i, nh, &amdc_s);
    }
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

void update_actuators()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "computer_node");
    ros::NodeHandle nh;

    //XXX XXX
    Eigen::VectorXf goal(2);
    goal << 10, 0;
    amdc_s.goals.push(goal);
    amdc_s.state << 0,0,0,0,0,0;
    amdc_s.range << 6,6,6,6,6,6,6;

    // initialise all publisher and subscriber and sensor data
    init_ultrasonic(nh);
    init_propeller(nh);
    init_servo(nh);
    init_vision(nh);

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
