#include <ros/console.h>
#include <iostream>

extern "C"
{
#include "serial_comm.h"
}

using namespace std;

unsigned char data_buf[80];
int msg_sz;

void handle_imu_mag()
{
    short *buf = (short *)data_buf;
    int lin_x = buf[0];
    int lin_y = buf[1];
    int lin_z = buf[2];
    int ang_x = buf[3];
    int ang_y = buf[4];
    int ang_z = buf[5];
    int mag_x = buf[6];
    int mag_y = buf[7];
    int mag_z = buf[8];
    cout << "imu\n";
    cout << "lin: " << lin_x << "\t" << lin_y << "\t" << lin_z << endl;
    cout << "ang: " << ang_x << "\t" << ang_y << "\t" << ang_z << endl;
    cout << "mag: " << mag_x << "\t" << mag_y << "\t" << mag_z << endl;
}

void handle_ultrasonic()
{
    int addr = data_buf[0];
    int distance = data_buf[1];
    cout << "ultrasonic " << addr << ": " << distance << endl;
}

int checksum()
{
    int lrc = msg_sz;

    for (int i = 0; i < msg_sz - 1; ++i)
        lrc += data_buf[i];

    lrc = -lrc & 0xff;
    return lrc;
}

int main(int argc, char **argv)
{
    enum state { HEADER1, HEADER2, SIZE, DATA };
    enum state sm = HEADER1;
    unsigned char buf;
    int bytes_recv;
    int sum_of_bytes = 0;

    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " serial_port" << endl;
    }

    begin_serial(argv[1], 5); // timeout of 5s

    while (1)
    {
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
                else if (msg_sz != 3 && msg_sz != 19)
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
                int expected_lrc = checksum();
                int received_lrc = data_buf[msg_sz - 1];

                if (expected_lrc != received_lrc)
                {
                    cerr << "Bad checksum: " << received_lrc << endl;
                }
                else if (msg_sz == 3)
                {
                    handle_ultrasonic();
                }
                else if (msg_sz == 19)
                {
                    handle_imu_mag();
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
}
