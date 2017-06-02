#ifndef sensor_util_H
#define sensor_util_H

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Vector3.h"
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

// topic name:  "imu_data"    "mag_data" 
// frame id:    "imu_frame"   "mag_frame"
class imu_handler
{
  private:
    ros::Subscriber sub;
    ros::Publisher pub_imu, pub_mag;
    // conversion factor
    // 0.061, 4.35, 6842 are the default factors given in the data sheet
    // imu, p15: https://www.pololu.com/file/0J1087/LSM6DS33.pdf
    // magnetometer, p8: https://www.pololu.com/file/0J1089/LIS3MDL.pdf
    static const double linacc_cf = 0.061, 
          angvel_cf = 4.35 * 3.14159265359 / 180,
          magfel_cf = 1. / (10000 * 6842);
    
  public:
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg; 

    imu_handler()
    {
      imu_msg.orientation_covariance = 
        { 0, 0, 0, 
          0, 0, 0, 
          0, 0, 0};
      mag_msg.magnetic_field_covariance = 
        { 0, 0, 0, 
          0, 0, 0, 
          0, 0, 0};
    }

    void callback(const std_msgs::Int16MultiArray::ConstPtr&);

    void advertise(ros::NodeHandle nh)
    {
      pub_imu = nh.advertise<sensor_msgs::Imu>("imu_data", 1000);
      imu_msg.header.frame_id = "imu_frame";

      pub_mag = nh.advertise<sensor_msgs::MagneticField>("mag_data",1000);
      mag_msg.header.frame_id = "mag_frame";
    }

    void subscribe(ros::NodeHandle nh)
    {
      sub = nh.subscribe("im", 1000, &imu_handler::callback, this);
    }
};

void imu_handler::callback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
  imu_msg.linear_acceleration.x = msg -> data[0] * linacc_cf;
  imu_msg.linear_acceleration.y = msg -> data[1] * linacc_cf;
  imu_msg.linear_acceleration.z = msg -> data[2] * linacc_cf;
  imu_msg.angular_velocity.x = msg -> data[3] * angvel_cf;
  imu_msg.angular_velocity.y = msg -> data[4] * angvel_cf;
  imu_msg.angular_velocity.z = msg -> data[5] * angvel_cf;
  mag_msg.magnetic_field.x = msg -> data[6] * magfel_cf;
  mag_msg.magnetic_field.y = msg -> data[7] * magfel_cf;
  mag_msg.magnetic_field.z = msg -> data[8] * magfel_cf;

  ROS_DEBUG_STREAM("imu,  linear acceleration (m/s2): " 
      << imu_msg.linear_acceleration);
  ROS_DEBUG_STREAM("imu, angular velocities (rad/s): " 
      << imu_msg.angular_velocity); 
  ROS_DEBUG_STREAM("magnetometer, field (Tesla): " 
      << mag_msg.magnetic_field);

  imu_msg.header.stamp = ros::Time::now();
  pub_imu.publish(imu_msg); 

  mag_msg.header.stamp = ros::Time::now();
  pub_mag.publish(mag_msg);
}

class gps_handler
{
  private:
    ros::Subscriber sub;
    ros::Publisher pub; 
    sensor_msgs::NavSatFix gps_msg;

  public:

    gps_handler()
    {
      gps_msg.position_covariance_type = 0; // covariance unkonwn
    }

    void callback(const std_msgs::Float32MultiArray::ConstPtr&); 

    void advertise(ros::NodeHandle nh)
    {
      pub = nh.advertise("gps_data", 1000);
      gps_msg.header.frame_id = "gps_frame";
    }

    void subscribe(ros::NodeHandle nh)
    {
      sub = nh.subscribe("gp", 1000, &gps_handler::callback, this);
    }
};

void gps_handler::callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  gps_msg.latitude = msg -> data[0]; 
  gps_msg.longitude = msg -> data[1];
  gps_msg.altitude =  msg -> data[2];
  gps_msg.status.status = msg -> data[3];
  gps_msg.status.service = msg -> data[4];

  ROS_DEBUG_STREAM("gps (lat[deg], long[deg], alt[m]), status, service: " 
      << gps_msg.latitude << ", " 
      << gps_msg.longitude << ", "
      << gps_msg.altitude << ", " 
      << gps_msg.status.status << ", "
      << gps_msg.status.service);

  gps_msg.header.stamp = ros::Time::now();
  pub.publish(gps_msg);
}

#endif
