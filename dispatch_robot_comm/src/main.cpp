#include <dispatch_robot_comm/dispatch_robot_comm.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dispatch_robot_comm");
    Dispatch dispatch;

    dispatch.Run();
    return EXIT_SUCCESS;
}
