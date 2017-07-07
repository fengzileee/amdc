#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_util.h"

#include <iostream>

extern "C"
{
#include "serial_comm.h"
}

using namespace std;

// sensors
ultrasonic_handler ultrasonic[7];
imu_handler imu; // pub: imu_data
gps_handler gps; // pub: gps_data
propeller_handler propeller; // pub: propeller_feedback

void init_ultrasonic(ros::NodeHandle nh)
{
    for (int i = 0; i < 7; ++i)
    {
        ultrasonic[i].id = i;
        ultrasonic[i].advertise(i, nh);
    }
}

void init_imu(ros::NodeHandle nh)
{
    imu.advertise(nh);
}

void init_gps(ros::NodeHandle nh)
{
    gps.advertise(nh);
}

void init_propeller(ros::NodeHandle nh)
{
    propeller.subscribe(nh);
    propeller.advertise(nh);
}

int checksum(uint8_t *buf, int sz)
{
    int lrc = 0;

    for (int i = 0; i < sz; ++i)
        lrc += buf[i];

    lrc = -lrc & 0xff;
    return lrc;
}

void send_propeller_command()
{
    uint8_t buf[8];

    if (propeller.update == false)
        return;

    buf[0] = 'A';
    buf[1] = propeller.out_msg.left_pwm & 0xff;
    buf[2] = (propeller.out_msg.left_pwm >> 8) & 0xff;
    buf[3] = propeller.out_msg.right_pwm & 0xff;
    buf[4] = (propeller.out_msg.right_pwm >> 8) & 0xff;
    buf[5] = propeller.out_msg.left_enable;
    buf[6] = propeller.out_msg.right_enable;

    // checksum
    buf[7] = 0;
    for (int i = 1; i <= 6; ++i)
    {
        buf[7] += buf[i];
    }
    buf[7] = -buf[7];

    serial_write(buf, sizeof buf);
    propeller.update = false;
}

void process_serial_data(uint8_t *buf, int ctr)
{
    static int state = 0;
    static int msg_len = 0;
    static uint8_t data_buf[80];

    static int msg_count = 0;
    static int bad_msg = 0;

    for (int i = 0; i < ctr; ++i)
    {
        switch (state)
        {
            case 0: // initial state
                state = buf[i] == 'A' ? state + 1 : state;
                break;
            case 1: // in header state
                state = buf[i] == 'z' ? state + 1 : state - 1;
                break;
            case 2: // in message state
                if (buf[i] != 'A')
                {
                    data_buf[msg_len++] = buf[i];
                }
                else
                {
                    ++state;
                }
                break;
            case 3: // check end of message state
                if (buf[i] == 'z')
                {
                    // checksum
                    int expected_lrc = checksum(data_buf, msg_len - 1);
                    int received_lrc = data_buf[msg_len - 1];
                    int msg_sz = data_buf[0];
                    uint8_t *actual_data_buf = data_buf + 1;

                    if (expected_lrc != received_lrc)
                    {
                        ROS_DEBUG_STREAM("Bad checksum: " << received_lrc
                                << ", expected: " << expected_lrc);
                        ++bad_msg;
                    }
                    else if (msg_sz == 5)
                    {
                        int addr = actual_data_buf[0];
                        ultrasonic[addr - 2].process_sensor_msg(actual_data_buf);
                    }
                    else if (msg_sz == 9)
                    {
                        propeller.process_sensor_msg(actual_data_buf);
                    }
                    else if (msg_sz == 19)
                    {
                        imu.process_sensor_msg(actual_data_buf);
                    }
                    else if (msg_sz == 21)
                    {
                        gps.process_sensor_msg(actual_data_buf);
                    }
                    else
                    {
                        ROS_WARN_STREAM("Unknown message size: " << msg_sz);
                    }

                    msg_len = 0;
                    ++msg_count;

                }
                else
                {
                    data_buf[msg_len++] = 'A';
                    data_buf[msg_len++] = 'z';
                }
                --state;
                break;
        }
    }
    ROS_DEBUG_STREAM(" error rate: " << (float) bad_msg / msg_count);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " serial_port" << endl;
    }

    ros::init(argc, argv, "custom_rosserial");
    ros::NodeHandle nh;

    begin_serial(argv[1], 5); // timeout of 5s

    init_ultrasonic(nh);
    init_imu(nh);
    init_gps(nh);
    init_propeller(nh);

    uint8_t buf[8192];
    
    while (ros::ok())
    {
        int recv = serial_read(buf);
        if (recv > 1024)
        {
            ROS_WARN_STREAM("received " << recv
                    << " bytes. buffer getting full");
        }
        process_serial_data(buf, recv);
        ros::spinOnce();
        send_propeller_command();
    }

    return 0;
}

