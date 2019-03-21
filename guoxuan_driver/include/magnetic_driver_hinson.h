#ifndef MAGNETIC_DRIVER
#define MAGNETIC_DRIVER

#include <ros/ros.h>
#include <fcntl.h>
#include <termios.h>

#include <std_msgs/Header.h>

using namespace std;
class MagneticDriverHinson
{
public:
    MagneticDriverHinson();
    ~MagneticDriverHinson();

    void Run();

private:
    ros::NodeHandle nh_;
    ros::Publisher magnetic_pub_;
    ros::Subscriber task_switch_sub_;

    string serial_dev_;
    int serial_fd_;

    bool run_task_;

    int magnetic_point_;
    int length_read_magnetic_;
    unsigned char buf_send_point_io_[7];

    void InitBufSend();
    void SerialClose();
    void TaskSwitchCallback(const std_msgs::HeaderPtr &task_switch_msg);
    unsigned int GetCrcCheck(unsigned char *cBuffer, unsigned int iBufLen);

    void ReadMagneticSend();
    void ReadMagneticReturn(int length_read);
};

#endif
