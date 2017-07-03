#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_util.h"

#include <iostream>

extern "C"
{
#include "serial_comm.h"
}

using namespace std;

enum state { HEADER1, HEADER2, SIZE, DATA };

// sensors
ultrasonic_handler ultrasonic[7];
imu_handler imu; // pub: imu_data
gps_handler gps; // pub: gps_data
propeller_handler propeller; // pub: propeller_feedback

void init_ultrasonic(ros::NodeHandle nh)
{
    for (int i = 1; i <= 7; ++i)
    {
        ultrasonic[i - 1].advertise(i, nh);
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

int checksum(void *buffer, int sz)
{
    unsigned char *buf = (unsigned char *)buffer;
    int lrc = sz;

    for (int i = 0; i < sz - 1; ++i)
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

void get_serial_data()
{
    static enum state sm = HEADER1;

    static unsigned char data_buf[80];
    static int msg_sz;

    unsigned char buf;
    int bytes_recv;

    switch (sm)
    {
    case HEADER1:

        bytes_recv = serial_read(&buf, sizeof buf);
        if (bytes_recv == -1)
        {
            sm = HEADER1;
        }
        else if (bytes_recv < 1)
        {
            cerr << "Timeout: unable to receive any message.\n";
            sm = HEADER1;
        }
        else if (buf != 'A')
        {
            cerr << "Received " << buf << " instead of A\n";
            sm = HEADER1;
        }
        else
        {
            sm = HEADER2;
        }
        break;

    case HEADER2:

        bytes_recv = serial_read(&buf, sizeof buf);
        if (bytes_recv == -1)
        {
            sm = HEADER1;
        }
        else if (bytes_recv < 1)
        {
            cerr << "Timeout: unable to receive any message.\n";
            sm = HEADER1;
        }
        else if (buf != 'z')
        {
            cerr << "Received " << buf << " instead of z\n";
            sm = HEADER1;
        }
        else
        {
            sm = SIZE;
        }
        break;

    case SIZE:

        // XXX
        //cerr << "in size\n";
        bytes_recv = serial_read(&buf, sizeof buf);
        if (bytes_recv == -1)
        {
            sm = HEADER1;
        }
        else if (bytes_recv < 1)
        {
            cerr << "Timeout: unable to read msg size.\n";
            sm = HEADER1;
        }
        else
        {
            msg_sz = buf;
            if (msg_sz < 1)
            {
                cerr << "Received size 0 data.\n";
                sm = HEADER1;
            }
            else if (msg_sz != 5   // ultrasonic
                  && msg_sz != 9   // propeller
                  && msg_sz != 19  // imu+mag
                  && msg_sz != 21) // gps
            {
                cerr << "Unknown message size: " << msg_sz << endl;
                sm = HEADER1;
            }
            else
            {
                sm = DATA;
            }
        }
        break;

    case DATA:

        // XXX
        //cerr << "in data\n";
        static int sum_of_bytes = 0;
        bytes_recv = serial_read(data_buf + sum_of_bytes, msg_sz - sum_of_bytes);
        if (bytes_recv == -1)
        {
            sm = HEADER1;
        }
        else if (bytes_recv < 1)
        {
            cerr << "Timeout: unable to read msg data.\n";
            sm = HEADER1;
        }
        else if (bytes_recv + sum_of_bytes < msg_sz)
        {
            sum_of_bytes += bytes_recv;
        }
        else
        {
            int expected_lrc = checksum(data_buf, msg_sz);
            int received_lrc = data_buf[msg_sz - 1];

            if (expected_lrc != received_lrc)
            {
                cerr << "Bad checksum: " << received_lrc << endl;
            }
            else if (msg_sz == 5)
            {
                int addr = data_buf[0];
                ultrasonic[addr - 2].process_sensor_msg(data_buf);
            }
            else if (msg_sz == 9)
            {
                propeller.process_sensor_msg(data_buf);
            }
            else if (msg_sz == 19)
            {
                imu.process_sensor_msg(data_buf);
            }
            else if (msg_sz == 21)
            {
                gps.process_sensor_msg(data_buf);
            }
            else
            {
                cerr << "Unknown message size: " << msg_sz << endl;
            }
            sum_of_bytes = 0;
            sm = HEADER1;
        }
        break;
    }
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
    
    while (ros::ok())
    {
        get_serial_data();
        ros::spinOnce();
        send_propeller_command();
    }

    return 0;
}

