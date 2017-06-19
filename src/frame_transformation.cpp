#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "transformation_broadcaster"); 

    static tf2_ros::StaticTransformBroadcaster tf_odom2base_br; 
    geometry_msgs::TransformStamped tf_odom2base;

    tf_odom2base.header.stamp = ros::Time::now(); 
    tf_odom2base.header.frame_id = "odom"; 
    tf_odom2base.child_frame_id = "base_link";
    // TODO: transformation from imu_frame to base_link 

    tf_odom2base_br.sendTransform(tf_odom2base);

    ros::spin();
    return 0;
}
