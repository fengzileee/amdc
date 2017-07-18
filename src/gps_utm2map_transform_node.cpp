#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"

#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class GPSTrans
{
    public:
        GPSTrans() :
            tf2_(buffer_),  target_frame_("odom"),
            tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
        {
            point_sub_.subscribe(n_, "target_gps_point_utm", 10);
            point_pub_ = n_.advertise<geometry_msgs::PointStamped>("target_gps_odometry_odom", 10);
            tf2_filter_.registerCallback( boost::bind(&GPSTrans::msgCallback, this, _1) );
        }

        //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
        void msgCallback(const geometry_msgs::PointStampedConstPtr& odom_ptr) 
        {
            geometry_msgs::PointStamped odom_out;
            try 
            {
                buffer_.transform(*odom_ptr, odom_out, target_frame_);
                point_pub_.publish(odom_out);

                ROS_INFO("point of reference point in frame of robot map Position(x:%f y:%f)\n", 
                        odom_out.point.x,
                        odom_out.point.y);
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
        message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
        ros::Publisher point_pub_;
        tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;

};


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "GPS_utm2map_transform_node"); //Init ROS
    GPSTrans gt; //Construct class
    ros::spin(); // Run until interupted 
    return 0;
};

;
