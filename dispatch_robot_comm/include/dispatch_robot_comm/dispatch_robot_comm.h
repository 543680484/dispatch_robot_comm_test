#ifndef DISPATCH_ROBOT_COMM
#define DISPATCH_ROBOT_COMM

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <yocs_msgs/Trajectory.h>
#include <yocs_msgs/TrajectoryList.h>
#include <yocs_msgs/NavigationControl.h>
#include <yocs_msgs/NavigationControlStatus.h>
#include <sensor_msgs/BatteryState.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include <json/json.h>

#include <dispatch_robot_comm/logger.h>

#define BUFFERSIZE              1024
using namespace std;

class Dispatch
{
public:
    Dispatch();
    ~Dispatch();

    void Run();

protected:
    ros::NodeHandle nh_;

    ros::Subscriber robot_pose_sub_;
    ros::Subscriber battery_sub_;
    ros::Subscriber navigation_control_status_sub_;
    ros::Subscriber navigation_control_sub_;

    ros::Publisher trajectories_add_pub_;
    ros::Publisher trajectories_remove_pub_;
    ros::Publisher navigation_control_pub_;

    std::string server_ip_;
    int server_port_;
    int socket_fd_;

    int dev_id_;
    long long int time_stamp_recv_sec_;
    int time_stamp_recv_usec_;
    int dispatch_comm_id_;
    const string dispatch_trajectory_name_prefix_;
    const string dispatch_waypoint_name_prefix_;

    yocs_msgs::TrajectoryList trajectory_list_msg_;
    geometry_msgs::Pose2D agv_pose2d_;

    bool trajectorie_finished_;
    bool traj_exist_in_waypoints_;
    bool first_magnetic_nav_point_;
    bool get_new_move_;
    bool get_hand_shake_from_dispatch_;
    bool recv_dispatch_cancel_task_;

    string comm_type_;
    string proc_name_;
    string waypoint_name_trajectorie_finished_;

    bool need_hand_shake_;

    double magnetic_back_offset_;
    double battery_percentage_;

    enum task_state
    {
        IDLE,
        PUB_TRAJECTORIE_ADD,
        PUB_NAVIGATION_CONTROL,
        WAIT_TASK_FINISH,
        WAIT_DISPATCH_HAND_SHAKE
    };

    task_state task_state_;

    bool ConnectServer(string ip_str, int port );
    int SetNonBlock ( const int sockfd, const struct sockaddr* serv_addr,
                      const socklen_t socklen, const int nsec, const int usec );
    int AGVWrite(int fd, const char *buffer,int length);

    void RobotPoseCallBack(const geometry_msgs::PoseConstPtr &robot_pose_msg);
    void BatteryCallBack(const sensor_msgs::BatteryStatePtr &battery_msg);
    void NavigationControlStatusCallBack(const yocs_msgs::NavigationControlStatusConstPtr &navigation_control_msg_ptr);
    void AgvCancelTaskCallBack(const yocs_msgs::NavigationControlConstPtr &navigation_control_msg);

    void TrajectorieAddPub();
    void TrajectorieRemovePub(string trajectories_name);
    void NavigationControlPub();
    void NavigationControlPub( string trajectories_name );
    void NavigationControlPubDispatchCancel();

    bool MsgFromDispatchParse(string str_complete);
    void MsgFromDispatchSplicing( const string& str_recv, vector<string>& msg_complete );
    void MsgToDispatch(string dev_type, string& msg_agv_to_dispatch);
    void ExecuteTask();

    bool ChangeNavMode( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData );
    bool ChangeNavModeExist( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData );
    void MagneticTrackLocation( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData );
    bool MotorAction( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData );
    void TurnRound( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData );
    void OdometryGoalong( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData );
    bool NavigationAction(yocs_msgs::Trajectory& trajectory_msg, const Json::Value& MoveData );
    bool NavigationActionLastPoint(yocs_msgs::Trajectory& trajectory_msg, const Json::Value& MoveData );
    void ChargeStart( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData );
    void ChargeOver( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData );
    void MagneticTest( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData );
    void DispatchCancelTask();
    void ResetTask();
};

#endif
