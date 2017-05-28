#ifndef sensor_util_H
#define sensor_util_H

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Int16.h"
#include <sstream>

const struct
{
    const char *topic;
    const char *frame_id;
} ultrasonic_info[] =
{ {"ultrasonic_left",       "ultrasonic_left_frame"      },
  {"ultrasonic_top_left",   "ultrasonic_top_left_frame"  },
  {"ultrasonic_top_right",  "ultrasonic_top_right_frame" },
  {"ultrasonic_right",      "ultrasonic_right_frame"     },
  {"ultrasonic_btm_right",  "ultrasonic_btm_right_frame" },
  {"ultrasonic_btm",        "ultrasonic_btm_frame"       },
  {"ultrasonic_btm_left",   "ultrasonic_btm_left_frame"  } };

// TODO
// maybe create a base class?
class ultrasonic_handler
{
    private:
        int id;
        ros::Subscriber sub;
        ros::Publisher pub;
        sensor_msgs::Range range_msg;

    public:
        // processed proximity value from sensor (in meters)
        float distance;

        ultrasonic_handler()
        {
            range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
            range_msg.field_of_view = 0.2;
            range_msg.min_range = 0.03;
            range_msg.max_range = 6.00;
        }

        void callback(const std_msgs::Int16::ConstPtr&);
        void advertise(int id, ros::NodeHandle nh)
        {
            this->id = id;
            pub = nh.advertise<sensor_msgs::Range>(
                        ultrasonic_info[id - 1].topic,
                        1000);

            range_msg.header.frame_id = ultrasonic_info[id - 1].frame_id;
        }
        void subscribe(int id, ros::NodeHandle nh)
        {
            this->id = id;
            std::ostringstream topic;
            topic << "u" << id;
            sub = nh.subscribe(topic.str(),
                               1000,
                               &ultrasonic_handler::callback,
                               this);
        }
};

void ultrasonic_handler::callback(const std_msgs::Int16::ConstPtr& msg)
{
    distance = (float) msg->data / 100;
    ROS_DEBUG_STREAM("id " << id << ", distance (m): " << distance);

    // publish as Range msg for visualisation in rviz
    range_msg.header.stamp = ros::Time::now();
    range_msg.range = distance;
    pub.publish(range_msg);
}

#endif
