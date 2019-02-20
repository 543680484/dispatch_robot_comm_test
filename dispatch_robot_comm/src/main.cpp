#include <dispatch_robot_comm/dispatch_robot_comm.h>

inline std::string get_now_time()
{
    const int max_buf_size = 64;
    char buf[max_buf_size];
    std::fill(buf, buf+max_buf_size, 0);
    auto now = std::time(nullptr);
    auto r = std::strftime(buf, max_buf_size, "%F-%H", std::localtime(&now));
    if (!r) throw std::runtime_error("time format error");
    return buf;
}

int main(int argc, char **argv)
{
    // Console Logger
    auto console_logger = std::make_shared<lynx::ConsoleLogger>();
    lynx::Log::Instance()->add_handle(console_logger);
    // File Logger (by open time)
    auto file = "dispatch_logs/" + get_now_time() + ".txt";// 在~/.ros/dispatch_logs/文件夹下
    auto file_logger = std::make_shared<lynx::FileLogger>(file);
    lynx::Log::Instance()->add_handle(file_logger);

    ros::init(argc, argv, "dispatch_robot_comm");
    Dispatch dispatch;

    dispatch.Run();
    return EXIT_SUCCESS;
}
