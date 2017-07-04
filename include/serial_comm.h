// code adapted from:
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define handle_error(func) do { perror(func); exit(-1); } while (0);

static int fd;

int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
        handle_error("tcgetattr");

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        handle_error("tcsetattr");
    return 0;
}

void set_blocking(int fd, int should_block, int timeout)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
        handle_error("tcgetattr");

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = timeout;            // timeout = timeout * 100ms

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        handle_error("tcsetattr");
}

void begin_serial(const char *port, int timeout)
{
    fd = open (port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
        handle_error("open");

    set_interface_attribs(fd, B115200, 0); // 115200 bps, 8n1 (no parity)
    set_blocking(fd, 0, timeout * 10);     // set no blocking
}

int serial_read(void *buf) 
{
    int b;

    if ( (b = read(fd, buf, 4096)) == -1) 
    {   
        perror("read");
    }   
    return b;
}


int serial_write(void *buf, int sz)
{
    int b;
    if ( (b = write(fd, buf, sz)) == -1)
    {
        perror("write");
    }
    return b;
}
