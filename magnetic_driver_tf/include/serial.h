#include <fcntl.h>
#include <termios.h>

static int SerialOpen(std::string serial_dev, int baud_rate, int data_bits, char parity, int stop_bits)
{
    int fd = open(serial_dev.c_str(), O_RDWR|O_NOCTTY);//|O_NDELAY
    if (fd < 0)
    {
        printf("failure: open magnetic serial port %s, error message: %s\n", serial_dev.c_str(), strerror(errno));
        return fd;
    }
    else if (fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("failure: set fcntl file %s\n", serial_dev.c_str());
        close(fd);
        return -1;
    }

    struct termios newtio, oldtio;
    if (0 != tcgetattr(fd, &oldtio))
    {
        printf("failure: get attr %s\n", serial_dev.c_str());
        close(fd);
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (baud_rate)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 38400:
        cfsetispeed(&newtio, B38400);
        cfsetospeed(&newtio, B38400);
        break;
    case 57600:
        cfsetispeed(&newtio, B57600);
        cfsetospeed(&newtio, B57600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        printf("failure: set %s baud rate %d\n", serial_dev.c_str(), baud_rate);
        close(fd);
        return -1;
    }

    switch (data_bits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    default:
        printf("failure: set %s data bits %d\n", serial_dev.c_str(), data_bits);
        close(fd);
        return -1;
    }

    switch (parity)
    {
    case 'O':                     //奇校验
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':                     //偶校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':                     //无校验
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        printf("failure: set %s parity %c\n", serial_dev.c_str(), parity);
        close(fd);
        return -1;
    }

    switch (stop_bits)
    {
    case 1:
        newtio.c_cflag &=  ~CSTOPB;
        break;
    case 2:
        newtio.c_cflag |=  CSTOPB;
        break;
    default:
        printf("failure: set %s stop bits %d\n", serial_dev.c_str(), stop_bits);
        close(fd);
        return -1;
    }

    newtio.c_cc[VTIME] = 1;//VTIME单位百毫秒
    newtio.c_cc[VMIN] = 10;
    if (0 != (tcsetattr(fd, TCSANOW, &newtio)))
    {
        printf("failure: set attr %s\n", serial_dev.c_str());
        close(fd);
        return -1;
    }

    tcflush(fd, TCIFLUSH);
    printf("success: setup serial port %s", serial_dev.c_str());

    return fd;
}
