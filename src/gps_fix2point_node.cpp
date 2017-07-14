#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"

class SubPub
{
    public:
        SubPub()
        {
            pub = nh.advertise<geometry_msgs::PointStamped>("target_gps_point_utm", 10);
            sub = nh.subscribe("gps_utm_odometry", 10, 
                    &SubPub::fix2point_callback, this);
        }


        void fix2point_callback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            geometry_msgs::PointStamped point;
            point.header.stamp = ros::Time();
            point.header.frame_id = "utm"; 
            point.point = msg -> pose.pose.position;
            pub.publish(point);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_fix2point_node"); 

    SubPub subpub;

    ros::spin();

    return 0;
}
