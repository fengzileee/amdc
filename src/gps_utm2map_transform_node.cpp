#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"

#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class GPSTrans
{
    public:
        GPSTrans() :
            tf2_(buffer_),  target_frame_("map"),
            tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
        {
            point_sub_.subscribe(n_, "target_gps_odometry_utm", 10);
            point_pub_ = n_.advertise<nav_msgs::Odometry>("target_gps_odometry_map", 10);
            tf2_filter_.registerCallback( boost::bind(&GPSTrans::msgCallback, this, _1) );
        }

        //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
        void msgCallback(const nav_msgs::OdometryConstPtr& odom_ptr) 
        {
            nav_msgs::Odometry odom_out;
            try 
            {
                buffer_.transform(*odom_ptr, odom_out, target_frame_);
                point_pub_.publish(odom_out);

                ROS_INFO("point of reference point in frame of robot map Position(x:%f y:%f)\n", 
                        odom_out.pose.pose.position.x,
                        odom_out.pose.pose.position.y);
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
            }
        }

    private:
        std::string target_frame_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener tf2_;
        ros::NodeHandle n_;
        message_filters::Subscriber<nav_msgs::Odometry> point_sub_;
        ros::Publisher point_pub_;
        tf2_ros::MessageFilter<nav_msgs::Odometry> tf2_filter_;

};


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "GPS_utm2map_transform_node"); //Init ROS
    GPSTrans gt; //Construct class
    ros::spin(); // Run until interupted 
    return 0;
};

;
