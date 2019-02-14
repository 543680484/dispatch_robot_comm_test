#include <dispatch_robot_comm/dispatch_robot_comm.h>
#include <glog/logging.h>

#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)

Dispatch::Dispatch()
    :nh_()
    ,robot_pose_sub_()
    ,navigation_control_status_sub_()
    ,trajectories_add_pub_()
    ,trajectories_remove_pub_()
    ,navigation_control_pub_()
    ,task_state_(IDLE )
    ,task_status_(0)
    ,dispatch_comm_id_(0)
    ,time_stamp_recv_sec_(0)
    ,time_stamp_recv_usec_(0)
    ,need_hand_shake_(false)
    ,trajectorie_finished_(false)
    ,get_hand_shake_from_dispatch_(false)
    ,dispatch_trajectory_name_prefix_("dispatch_trajectory_")
    ,dispatch_waypoint_name_prefix_("dispatch_waypoint_")
    ,comm_type_("")
    ,socket_fd_()
    ,m_InforList_()
    ,m_lastPlus_()
    ,answer_comm_id_(0)
    ,agv_move_result_(0)
    ,agv_proc_state_("Execute")
    ,trajectory_list_msg_()
    ,traj_exist_in_waypoints_(false)
    ,first_magnetic_nav_point_(true)
    ,proc_name_()
    ,agv_pose2d_()
    ,get_new_move_(false)
    ,str_all_("")
    ,waypoint_name_trajectorie_finished_("")
{
    robot_pose_sub_ = nh_.subscribe("/robot_pose", 1, &Dispatch::RobotPoseCallBack, this );
    battery_sub_ = nh_.subscribe("/battery", 1, &Dispatch::BatteryCallBack, this );
    navigation_control_status_sub_ = nh_.subscribe("/nav_ctrl_status", 3, &Dispatch::NavigationControlStatusCallback, this );

    trajectories_add_pub_ = nh_.advertise<yocs_msgs::Trajectory>("/trajectory_add",1);
    trajectories_remove_pub_ = nh_.advertise<yocs_msgs::Trajectory>("/trajectory_remove",1);
    navigation_control_pub_ = nh_.advertise<yocs_msgs::NavigationControl>("/nav_ctrl",1);

    nh_.param( "server_ip", server_ip_, (string)"192.168.1.104" );
    nh_.param( "server_port", server_port_, 10001 );
    nh_.param( "dev_id", dev_id_, 1 );
    nh_.param( "magnetic_back_offset", magnetic_back_offset_, 0.08 );
    nh_.param( "comm_mode", comm_mode_, (string)"need_finished_hand_shake" );
}

Dispatch::~Dispatch(){}

bool Dispatch::ConnectServer (string ip_str, int port )
{
    cout << "ip: " << ip_str << " port: " << port << endl;

    struct sockaddr_in server_addr;

    if ( ( socket_fd_ = socket ( AF_INET, SOCK_STREAM, 0 ) ) < 0 )
    {
        //ROS_ERROR ( "create socket error: %s(errno:%d)\n)", strerror ( errno ), errno );
        LOG(ERROR) << "create socket error: " << strerror(errno) << "errno:" << errno;
        return false;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if ( inet_pton ( AF_INET, ip_str.c_str(), &server_addr.sin_addr ) <= 0 )
    {
        //ROS_ERROR ( "inet_pton error: %s(errno:%d))\n", strerror ( errno ), errno );
        LOG(ERROR) << "create socket error: " << strerror(errno) << "errno:" << errno;
        return false;
    }

    if ( SetNonBlock ( socket_fd_, ( struct sockaddr* ) &server_addr, sizeof ( server_addr ), 0, 1000000 ) == 0 )
    {
        //ROS_INFO ( "igv connected!" );
        LOG(INFO) << "igv connected";
        return true;
    }

    return false;
}

int Dispatch::SetNonBlock ( const int sockfd, const struct sockaddr* serv_addr, const socklen_t socklen,
                                   const int nsec, const int usec )
{
    int flags, n, error = 0;
    socklen_t len;
    fd_set rset, wset;
    struct timeval tval;

    flags = fcntl ( sockfd, F_GETFL, 0 );
    fcntl ( sockfd, F_SETFL, flags | O_NONBLOCK );
    if ( ( n = connect ( sockfd, serv_addr, socklen ) ) < 0 )
    {
        if ( errno != EINPROGRESS )
        {
            return -1;
        }
    }

    if ( n == 0 )
    {
        goto GotoTest;
    }
    FD_ZERO ( &rset );
    FD_SET ( sockfd, &rset );
    wset = rset;
    tval.tv_sec = nsec;
    tval.tv_usec = usec;

    if ( ( n = select ( sockfd + 1, &rset, &wset, NULL, &tval ) ) == 0 )
    {
        close ( sockfd );
        errno = ETIMEDOUT;
        return -1;
    }
    if ( FD_ISSET ( sockfd, &rset ) || FD_ISSET ( sockfd, &wset ) )
    {
        len = sizeof ( error );
        if ( getsockopt ( sockfd, SOL_SOCKET, SO_ERROR, &error, &len ) < 0 )
        {
            return -1;
        }
    }
    else
    {
       //ROS_DEBUG ( "select error: sockfd  not set" );
        LOG(WARNING) << "select error: sockfd  not set";
    }
GotoTest:
    fcntl ( sockfd, F_SETFL, flags );
    if ( error )
    {
        close ( sockfd );
        errno = error;
        return -1;
    }
    return 0;
}

void Dispatch::TrajectorieAddPub()
{
    trajectories_add_pub_.publish(trajectory_list_msg_.trajectories[0]);
}

void Dispatch::NavigationControlPub()
{
    yocs_msgs::NavigationControl navigation_control_msg;

    navigation_control_msg.control = yocs_msgs::NavigationControl::START;
    navigation_control_msg.goal_name = trajectory_list_msg_.trajectories[0].name;

    navigation_control_pub_.publish(navigation_control_msg);
}

void Dispatch::NavigationControlPub( string trajectories_name )
{
    yocs_msgs::NavigationControl navigation_control_msg;

    navigation_control_msg.control = yocs_msgs::NavigationControl::START;
    navigation_control_msg.goal_name = trajectories_name;

    navigation_control_pub_.publish(navigation_control_msg);
}

void Dispatch::TrajectorieRemovePub()
{
    yocs_msgs::Trajectory trajectory_msg;
    trajectory_msg.name = trajectory_list_msg_.trajectories[0].name;

    trajectories_remove_pub_.publish(trajectory_msg);
}

void Dispatch::NavigationControlStatusCallback(const yocs_msgs::NavigationControlStatusConstPtr &navigation_control_msg_ptr)
{
    task_status_ = static_cast<int>(navigation_control_msg_ptr->status);

    if ( task_status_ == yocs_msgs::NavigationControlStatus::COMPLETED )
    {
        trajectorie_finished_ = true;
        //waypoint_name_trajectorie_finished_ = navigation_control_msg_ptr->waypoint_name;
        //ROS_INFO ("call back trajectorie finished: %s", waypoint_name_trajectorie_finished_.c_str());
        LOG(INFO) <<"call back trajectorie finished:"<< waypoint_name_trajectorie_finished_.c_str();
    }
}

void Dispatch::RobotPoseCallBack(const geometry_msgs::PoseConstPtr &robot_pose_msg)
{
    agv_pose2d_.x = robot_pose_msg->position.x;
    agv_pose2d_.y = robot_pose_msg->position.y;
    agv_pose2d_.theta = RadianToAngle(tf::getYaw(robot_pose_msg->orientation));
}

void Dispatch::BatteryCallBack(const sensor_msgs::BatteryStatePtr &battery_msg)
{
    battery_percentage_ = battery_msg->percentage;
}

bool Dispatch::NavigationAction(yocs_msgs::Trajectory& trajectory_msg, const Json::Value& MoveData )
{
    int point_num = MoveData["Points_Num"].asInt();

    Json::Value arrayObj = MoveData["Points_Array"];

    int points_array_size = arrayObj.size();

    string str_time_stamp_usec = boost::lexical_cast<string>(time_stamp_recv_usec_);
    trajectory_msg.name = dispatch_trajectory_name_prefix_ + str_time_stamp_usec;

    if ( points_array_size > 0 )
    {
        trajectory_msg.waypoints.resize(points_array_size);
        for ( int i = 0; i < points_array_size; ++i )
        {
            if ( i == (points_array_size - 1) )
            {
                trajectory_msg.waypoints[i].header.frame_id = "{\"close_enough\":\"0.0\",\"failure_mode\":\"LOOP\",\"frame_id\":\"map\",\"goal_timeout\":\"0\",\"mark\":\"false\",\"type\":\"goal\"}";
            }
            else
            {
                trajectory_msg.waypoints[i].header.frame_id = "{\"close_enough\":\"0.05\",\"failure_mode\":\"LOOP\",\"frame_id\":\"map\",\"goal_timeout\":\"0\",\"mark\":\"false\",\"type\":\"goal\"}";
            }
            trajectory_msg.waypoints[i].name = dispatch_waypoint_name_prefix_ + str_time_stamp_usec;
            trajectory_msg.waypoints[i].pose.position.x = arrayObj[i]["X"].asDouble();
            trajectory_msg.waypoints[i].pose.position.y = arrayObj[i]["Y"].asDouble();
            trajectory_msg.waypoints[i].pose.position.z = 0.0;

            tf::Quaternion q = tf::createQuaternionFromYaw( AngleToRadian(arrayObj[i]["a"].asDouble()) );
            trajectory_msg.waypoints[i].pose.orientation.x = q.getX();
            trajectory_msg.waypoints[i].pose.orientation.y = q.getY();
            trajectory_msg.waypoints[i].pose.orientation.z = q.getZ();
            trajectory_msg.waypoints[i].pose.orientation.w = q.getW();
//            trajectory_msg.waypoints[i].close_enough = 0.0;
//            trajectory_msg.waypoints[i].goal_timeout = 0.0;
//            trajectory_msg.waypoints[i].failure_mode = "";
        }
    }
    else
    {
        //ROS_ERROR("no move point");
        LOG(ERROR) << "no move point";
        return false;
    }

    return true;
}

bool Dispatch::NavigationActionLastPoint(yocs_msgs::Trajectory& trajectory_msg, const Json::Value& MoveData )
{
    int point_num = MoveData["Points_Num"].asInt();

    Json::Value arrayObj = MoveData["Points_Array"];

    int points_array_size = arrayObj.size();

    string str_time_stamp_usec = boost::lexical_cast<string>(time_stamp_recv_usec_);

    trajectory_msg.name = dispatch_trajectory_name_prefix_ + str_time_stamp_usec;

    if ( points_array_size > 0 )
    {
        trajectory_msg.waypoints.resize(1);

        trajectory_msg.waypoints[0].header.frame_id = "{\"close_enough\":\"0.0\",\"failure_mode\":\"LOOP\",\"frame_id\":\"map\",\"goal_timeout\":\"0\",\"mark\":\"false\",\"type\":\"goal\"}";
        trajectory_msg.waypoints[0].name = dispatch_waypoint_name_prefix_ + str_time_stamp_usec;
        trajectory_msg.waypoints[0].pose.position.x = arrayObj[points_array_size-1]["X"].asDouble();
        trajectory_msg.waypoints[0].pose.position.y = arrayObj[points_array_size-1]["Y"].asDouble();
        trajectory_msg.waypoints[0].pose.position.z = 0.0;

        tf::Quaternion q = tf::createQuaternionFromYaw( AngleToRadian(arrayObj[points_array_size-1]["a"].asDouble()) );
        trajectory_msg.waypoints[0].pose.orientation.x = q.getX();
        trajectory_msg.waypoints[0].pose.orientation.y = q.getY();
        trajectory_msg.waypoints[0].pose.orientation.z = q.getZ();
        trajectory_msg.waypoints[0].pose.orientation.w = q.getW();
    }
    else
    {
        //ROS_ERROR("no move point");
        LOG(ERROR) << "no move point";
        return false;
    }

    return true;
}

bool Dispatch::ChangeNavModeExist(yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    string nav_mode = procData["Proc_Para_1"].asString();

    if ( nav_mode == "MAGNET" )
    {
        first_magnetic_nav_point_ = true;
        trajectory_msg.name = "traj_change_to_magnetic";
        traj_exist_in_waypoints_ = true;

    }
    else if ( nav_mode == "SLAM" )
    {
        trajectory_msg.name = "traj_change_to_dwa";
        traj_exist_in_waypoints_ = true;
    }
    else
    {
        //ROS_ERROR("undefined nav mode: %s", nav_mode.c_str());
        LOG(ERROR) << "undefined nav mode: "<< nav_mode.c_str();

        return false;
    }

    return true;
}

bool Dispatch::ChangeNavMode(yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    string nav_mode = procData["Proc_Para_1"].asString();

    if ( nav_mode == "MAGNET" )
    {
        first_magnetic_nav_point_ = true;

        trajectory_msg.name = "traj_change_to_magnetic";
        trajectory_msg.waypoints.resize(5);

        trajectory_msg.waypoints[0].name = "magnetic_driver_hinson_on";
        trajectory_msg.waypoints[0].header.frame_id = "{\"goal_timeout\":\"2\",\"type\":\"shell\"}";

        trajectory_msg.waypoints[1].name = "magnetic_tf_pub_on";
        trajectory_msg.waypoints[1].header.frame_id = "{\"goal_timeout\":\"2\",\"type\":\"shell\"}";

        trajectory_msg.waypoints[2].name = "change_bz";
        trajectory_msg.waypoints[2].header.frame_id = "{\"node\":\"/move_base\",\"param\":\"base_local_planner\",\"type\":\"dynparam\",\"value\":\"bz_local_planner/BZPlannerROS\"}";

        trajectory_msg.waypoints[3].name = "move_base_controller_frequency_up";
        trajectory_msg.waypoints[3].header.frame_id = "{\"node\":\"/move_base\",\"param\":\"controller_frequency\",\"type\":\"dynparam\",\"value\":\"10\"}";

        trajectory_msg.waypoints[4].name = "bz_local_planner_min_vel_x";
        trajectory_msg.waypoints[4].header.frame_id = "{\"node\":\"/move_base/BZPlannerROS\",\"param\":\"min_vel_x\",\"type\":\"dynparam\",\"value\":\"0.05\"}";
    }
    else if ( nav_mode == "SLAM" )
    {
        trajectory_msg.name = "traj_change_to_dwa";
        trajectory_msg.waypoints.resize(4);

        trajectory_msg.waypoints[0].name = "move_base_controller_frequency_up";
        trajectory_msg.waypoints[0].header.frame_id = "{\"node\":\"/move_base\",\"param\":\"controller_frequency\",\"type\":\"dynparam\",\"value\":\"5\"}";

        trajectory_msg.waypoints[1].name = "magnetic_driver_hinson_off";
        trajectory_msg.waypoints[1].header.frame_id = "{\"goal_timeout\":\"2\",\"type\":\"shell\"}";

        trajectory_msg.waypoints[2].name = "magnetic_tf_pub_off";
        trajectory_msg.waypoints[2].header.frame_id = "{\"goal_timeout\":\"2\",\"type\":\"shell\"}";

        trajectory_msg.waypoints[3].name = "change_dwa";
        trajectory_msg.waypoints[3].header.frame_id = "{\"node\":\"/move_base\",\"param\":\"base_local_planner\",\"type\":\"dynparam\",\"value\":\"dwa_local_planner/DWAPlannerROS\"}";
    }
    else
    {
        //ROS_ERROR("undefined nav mode: %s", nav_mode.c_str());
        LOG(ERROR) << "undefined nav mode: "<< nav_mode.c_str();
        return false;
    }

    return true;
}

void Dispatch::MagneticTest( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    double pose[3][8] = {
                                   { 0.702,  1.27,  1.332, 1.905, 3.858, 4.426, 4.485, 5.052  },
                                   { 0.554, 1.06,  1.184,  1.695,   1.74,  2.248, 2.368, 2.876  },
                                   { 0.65,  1.132, 1.273, 1.76,  1.76,  2.245,  2.388, 2.88 }
                        };

    int place = boost::lexical_cast<int>(procData["Proc_Para_1"].asString());
    double current_offset = boost::lexical_cast<double>(procData["Proc_Para_2"].asString());

    string str_time_stamp_usec = boost::lexical_cast<string>(time_stamp_recv_usec_);
    trajectory_msg.name = dispatch_trajectory_name_prefix_ + str_time_stamp_usec;

    trajectory_msg.waypoints.resize(16);
    for ( int i = 0; i < 16; i = i+2 )
    {
        trajectory_msg.waypoints[i].header.frame_id = "{\"type\":\"local_marker\"}";
        trajectory_msg.waypoints[i].name = dispatch_waypoint_name_prefix_ + str_time_stamp_usec;
        trajectory_msg.waypoints[i].pose.position.x = pose[place][i/2] + current_offset;
        trajectory_msg.waypoints[i].pose.position.y = 0;
        trajectory_msg.waypoints[i].pose.position.z = 0;
        cout << "MAGNETICTRACK_LOCATION x: " << trajectory_msg.waypoints[i].pose.position.x << endl;

        trajectory_msg.waypoints[i].pose.orientation.x = 0;
        trajectory_msg.waypoints[i].pose.orientation.y = 0;
        trajectory_msg.waypoints[i].pose.orientation.z = 0;
        trajectory_msg.waypoints[i].pose.orientation.w = 1;

        trajectory_msg.waypoints[i+1].header.frame_id = "{\"close_enough\":\"0.0\",\"failure_mode\":\"NONE\",\"goal_timeout\":\"6\",\"type\":\"timer\"}";
        trajectory_msg.waypoints[i+1].name = "sleep6s";
    }
}

void Dispatch::MagneticTrackLocation( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    static double last_offset = 0.0;
    double current_offset = boost::lexical_cast<double>(procData["Proc_Para_1"].asString());
    double back_offset = 0.0;

    if ( first_magnetic_nav_point_ )
    {
        first_magnetic_nav_point_ = false;
    }
    else
    {
        if ( last_offset >= 0 )
        {
            if ( (current_offset - last_offset) < -0.001 )
            {
                back_offset = -magnetic_back_offset_;
            }
        }
        else
        {
            if ( (current_offset - last_offset) > 0.001 )
            {
                back_offset = magnetic_back_offset_;
            }
        }
    }

    last_offset = current_offset;

    string str_time_stamp_usec = boost::lexical_cast<string>(time_stamp_recv_usec_);
    trajectory_msg.name = dispatch_trajectory_name_prefix_ + str_time_stamp_usec;

    trajectory_msg.waypoints.resize(1);
    trajectory_msg.waypoints[0].header.frame_id = "{\"type\":\"local_marker\"}";
    trajectory_msg.waypoints[0].name = dispatch_waypoint_name_prefix_ + str_time_stamp_usec;
    trajectory_msg.waypoints[0].pose.position.x = current_offset + back_offset;
    trajectory_msg.waypoints[0].pose.position.y = 0;
    trajectory_msg.waypoints[0].pose.position.z = 0;
    cout << "MAGNETICTRACK_LOCATION x: " << trajectory_msg.waypoints[0].pose.position.x << endl;

    trajectory_msg.waypoints[0].pose.orientation.x = 0;
    trajectory_msg.waypoints[0].pose.orientation.y = 0;
    trajectory_msg.waypoints[0].pose.orientation.z = 0;
    trajectory_msg.waypoints[0].pose.orientation.w = 1;
}

bool Dispatch::RollerMotor( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    string roller_num = procData["Proc_Para_1"].asString();
    string roller_action = procData["Proc_Para_2"].asString();
    if ( roller_num == "0" )
    {
        if ( roller_action == "LOAD" )
        {
            trajectory_msg.name = "traj_agv_load_motor0";
            traj_exist_in_waypoints_ = true;
        }
        else if ( roller_action == "UNLOAD" )
        {
            trajectory_msg.name = "traj_agv_unload_motor0";
            traj_exist_in_waypoints_ = true;
        }
        else
        {
            //ROS_ERROR("undefined nav mode: %s", roller_action.c_str());
            LOG(ERROR) << "undefined nav mode: "<< roller_action.c_str();
            return false;
        }
    }
    else if ( roller_num == "1" )
    {
        if ( roller_action == "LOAD" )
        {
            trajectory_msg.name = "traj_agv_load_motor1";
            traj_exist_in_waypoints_ = true;
        }
        else if ( roller_action == "UNLOAD" )
        {
            trajectory_msg.name = "traj_agv_unload_motor1";
            traj_exist_in_waypoints_ = true;
        }
        else
        {
            //ROS_ERROR("undefined nav mode: %s", roller_action.c_str());
            LOG(ERROR) << "undefined nav mode: "<< roller_action.c_str();
            return false;
        }
    }
    else if ( roller_num == "2" )
    {
        if ( roller_action == "LOAD" )
        {
            trajectory_msg.name = "traj_agv_load_motor2";
            traj_exist_in_waypoints_ = true;
        }
        else if ( roller_action == "UNLOAD" )
        {
            trajectory_msg.name = "traj_agv_unload_motor2";
            traj_exist_in_waypoints_ = true;
        }
        else
        {
            //ROS_ERROR("undefined nav mode: %s", roller_action.c_str());
            LOG(ERROR) << "undefined nav mode: "<< roller_action.c_str();

            return false;
        }
    }
    else
    {
        //ROS_ERROR("undefined nav mode: %s", roller_num.c_str());
        LOG(ERROR) << "undefined nav mode: "<< roller_num.c_str();
        return false;
    }

    return true;
}

bool Dispatch::LocatingPin(yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    string pin_num = procData["Proc_Para_1"].asString();
    string pin_action = procData["Proc_Para_2"].asString();
    if ( pin_num == "0" )
    {
        if ( pin_action == "LOAD" )
        {
            trajectory_msg.name = "traj_agv_load_motor0";
            traj_exist_in_waypoints_ = true;
        }
        else if ( pin_action == "UNLOAD" )
        {
            trajectory_msg.name = "traj_agv_unload_motor0";
            traj_exist_in_waypoints_ = true;
        }
        else
        {
            //ROS_ERROR("undefined nav mode: %s", pin_action.c_str());
            LOG(ERROR) << "undefined nav mode: "<< pin_action.c_str();

            return false;
        }
    }
    else if ( pin_num == "1" )
    {
        if ( pin_action == "LOAD" )
        {
            trajectory_msg.name = "traj_agv_load_motor1";
            traj_exist_in_waypoints_ = true;
        }
        else if ( pin_action == "UNLOAD" )
        {
            trajectory_msg.name = "traj_agv_unload_motor1";
            traj_exist_in_waypoints_ = true;
        }
        else
        {
            //ROS_ERROR("undefined nav mode: %s", pin_action.c_str());
            LOG(ERROR) << "undefined nav mode: "<< pin_action.c_str();

            return false;
        }
    }
    else if ( pin_num == "2" )
    {
        if ( pin_action == "LOAD" )
        {
            trajectory_msg.name = "traj_agv_load_motor2";
            traj_exist_in_waypoints_ = true;
        }
        else if ( pin_action == "UNLOAD" )
        {
            trajectory_msg.name = "traj_agv_unload_motor2";
            traj_exist_in_waypoints_ = true;
        }
        else
        {
            //ROS_ERROR("undefined nav mode: %s", pin_action.c_str());
            LOG(ERROR) << "undefined nav mode: "<< pin_action.c_str();
            return false;
        }
    }
    else
    {
        //ROS_ERROR("undefined nav mode: %s", pin_num.c_str());
        LOG(ERROR) << "undefined nav mode: " << pin_num.c_str();
        return false;
    }

    return true;
}

void Dispatch::TurnRound( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    double angle = boost::lexical_cast<double>(procData["Proc_Para_1"].asString());

    string str_time_stamp_usec = boost::lexical_cast<string>(time_stamp_recv_usec_);
    trajectory_msg.name = dispatch_trajectory_name_prefix_ + str_time_stamp_usec;

    trajectory_msg.waypoints.resize(1);
    trajectory_msg.waypoints[0].header.frame_id = "{\"close_enough\":\"0.0\",\"failure_mode\":\"LOOP\",\"frame_id\":\"base_link\",\"goal_timeout\":\"0\",\"type\":\"goal\"}";
    trajectory_msg.waypoints[0].name = dispatch_waypoint_name_prefix_ + str_time_stamp_usec;
    trajectory_msg.waypoints[0].pose.position.x = 0;
    trajectory_msg.waypoints[0].pose.position.y = 0;
    trajectory_msg.waypoints[0].pose.position.z = 0;

    tf::Quaternion q = tf::createQuaternionFromYaw( AngleToRadian(angle) );
    trajectory_msg.waypoints[0].pose.orientation.x = q.getX();
    trajectory_msg.waypoints[0].pose.orientation.y = q.getY();
    trajectory_msg.waypoints[0].pose.orientation.z = q.getZ();
    trajectory_msg.waypoints[0].pose.orientation.w = q.getW();
}

void Dispatch::OdometryGoalong( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    double offset = boost::lexical_cast<double>(procData["Proc_Para_1"].asString());

    string str_time_stamp_usec = boost::lexical_cast<string>(time_stamp_recv_usec_);
    trajectory_msg.name = dispatch_trajectory_name_prefix_ + str_time_stamp_usec;

    trajectory_msg.waypoints.resize(1);
    trajectory_msg.waypoints[0].header.frame_id = "{\"close_enough\":\"0.0\",\"failure_mode\":\"LOOP\",\"frame_id\":\"base_link\",\"goal_timeout\":\"0\",\"type\":\"goal\"}";
    trajectory_msg.waypoints[0].name = dispatch_waypoint_name_prefix_ + str_time_stamp_usec;
    trajectory_msg.waypoints[0].pose.position.x = offset;
    trajectory_msg.waypoints[0].pose.position.y = 0;
    trajectory_msg.waypoints[0].pose.position.z = 0;
    cout << "ODOMETRY_GOALONG x: " << trajectory_msg.waypoints[0].pose.position.x << endl;

    trajectory_msg.waypoints[0].pose.orientation.x = 0;
    trajectory_msg.waypoints[0].pose.orientation.y = 0;
    trajectory_msg.waypoints[0].pose.orientation.z = 0;
    trajectory_msg.waypoints[0].pose.orientation.w = 1;
}

void Dispatch::ChargeStart( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    trajectory_msg.name = "charging_on_sleep";
    traj_exist_in_waypoints_ = true;
}

void Dispatch::ChargeOver( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    trajectory_msg.name = "charging_off_sleep";
    traj_exist_in_waypoints_ = true;
}

bool Dispatch::DeserializedJson(std::string strValue)
{
    Json::Reader reader;
    Json::Value json_object;

    if(reader.parse(strValue, json_object))
    {
        Json::Value dev_data = json_object["Dev_Data"];
        //ROS_INFO("recv data time");
        LOG(INFO) << "recv data time";
        cout << "dev_data:" << dev_data << endl;

        if ( dev_data.empty() )
        {
            //ROS_ERROR("dev data is empty");
            LOG(ERROR) << "dev data is empty";
            return false;
        }

        need_hand_shake_ = true;

        long long int time_stamp_sec = dev_data["TimeStamp_sec"].asInt64();
        int time_stamp_usec = dev_data["TimeStamp_usec"].asInt();

        if ( time_stamp_recv_sec_ == time_stamp_sec && time_stamp_recv_usec_ == time_stamp_usec )
        {
            //ROS_WARN("recv same command !!!");
            LOG(WARNING) << "recv same command !!!";
            return false;
        }

        time_stamp_recv_sec_ = time_stamp_sec;
        time_stamp_recv_usec_ = time_stamp_usec;
        dispatch_comm_id_ = dev_data["Comm_ID"].asInt();

        string comm_type = dev_data["Comm_Type"].asString();

        if ( comm_type == "MOVE" && comm_type_ == "MOVE" )
        {
            get_new_move_ = true;
        }

        comm_type_ = comm_type;

        yocs_msgs::Trajectory trajectory_msg;

        if ( "MOVE" == comm_type_ )
        {
            Json::Value MoveData = dev_data["MOVE"];

            if ( MoveData.empty() )
            {
                //ROS_ERROR("move empty");
                LOG(ERROR) << "move empty";
                return false;
            }
//            if ( !NavigationAction(trajectory_msg, MoveData) )
            if ( !NavigationActionLastPoint(trajectory_msg, MoveData) )
            {
                return false;
            }
        }
        else if ( "PROC" == comm_type_ )
        {
            Json::Value procData = dev_data["PROC"];

            if ( procData.empty() )
            {
                //ROS_ERROR("proc empty");
                LOG(ERROR) << "proc empty";
                return false;
            }

            proc_name_ = procData["Proc_Name"].asString();
            if ( proc_name_ == "CHANGE_NAV_MODE" )
            {
//                if ( !ChangeNavMode(trajectory_msg, procData) )
                if ( !ChangeNavModeExist(trajectory_msg, procData) )
                {
                    return false;
                }
            }
            else if ( proc_name_ == "MAGNETICTRACK_LOCATION" )
            {
                MagneticTrackLocation(trajectory_msg, procData);
            }
            else if ( proc_name_ == "ROLLER_MOTOR" )
            {
                if ( !RollerMotor(trajectory_msg, procData) )
                {
                    return false;
                }
            }
            else if ( proc_name_ == "LOCATINGPIN" )
            {
                if ( !LocatingPin(trajectory_msg, procData) )
                {
                    return false;
                }
            }

            else if ( proc_name_ == "TURNROUND" )
            {
                TurnRound(trajectory_msg, procData);
            }
            else if ( proc_name_ == "ODOMETRY_GOALONG" )
            {
                OdometryGoalong(trajectory_msg, procData);
            }
            else if ( proc_name_ == "CHARGE_START" )
            {
                ChargeStart(trajectory_msg, procData);
            }
            else if ( proc_name_ == "CHARGE_OVER" )
            {
                ChargeOver(trajectory_msg, procData);
            }
            else if ( proc_name_ == "CANCEL_COMMAND" )
            {
                //todo
            }
            else if ( "TEST" == proc_name_ )
            {
                MagneticTest(trajectory_msg, procData);
            }
            else
            {
                //ROS_ERROR("undefined proc name: %s", proc_name_.c_str());
                LOG(ERROR) << "undefined proc name: "<< comm_type_.c_str();

                return false;
            }
        }
        else if ( "HANDSHAKE_MOVE_END" == comm_type_ || "HANDSHAKE_PROC_END" == comm_type_ )
        {
            get_hand_shake_from_dispatch_ = true;
            return true;
        }
        else
        {
            //ROS_ERROR("undefined comm type: %s", comm_type_.c_str());
            LOG(ERROR) << "undefined comm type: "<< comm_type_.c_str();
            return false;
        }

        //need_hand_shake_ = true;
        trajectory_list_msg_.trajectories.push_back(trajectory_msg);

        return true;
    }
    else
    {
        //ROS_ERROR("parse error");
        LOG(ERROR) << "parse error";
        return false;
    }
}

void Dispatch::MsgAGVToDispatch(string dev_type, string& msg_agv_to_dispatch)
{
    struct timeval time_value;
    gettimeofday(&time_value,NULL);

    Json::Value DeviceInfor;
    DeviceInfor["Dev_Type"] = dev_type;
    DeviceInfor["Dev_ID"] = dev_id_;

    Json::Value AGVInfor;
    AGVInfor["TimeStamp_sec"] = static_cast<long long int>(time_value.tv_sec);
    AGVInfor["TimeStamp_usec"] = static_cast<int>(time_value.tv_usec);

    AGVInfor["Map_ID"] = 1;
    AGVInfor["X"] = agv_pose2d_.x;
    AGVInfor["Y"] = agv_pose2d_.y;
    AGVInfor["a"] = agv_pose2d_.theta;

//    static ros::Time start_time = ros::Time::now();
//    if ( ros::Time::now() - start_time > ros::Duration(240.0) )
//    {
//        AGVInfor["Cell_Quan"] = 0.2;
//    }
//    else
//    {
//        AGVInfor["Cell_Quan"] = 0.98;
//    }

    AGVInfor["Cell_Quan"] = battery_percentage_;
    AGVInfor["ERROR"] = 1;

    if( answer_comm_id_ == 0 && !need_hand_shake_ ) //一般上报数据
    {
        AGVInfor["FeedBack_Type"] = "";
        AGVInfor["Comm_ID"] = answer_comm_id_;
    }
    else //握手或任务完成
    {
        AGVInfor["FeedBack_Type"] = comm_type_;
        AGVInfor["Comm_ID"] = dispatch_comm_id_;

        if ( comm_type_ == "MOVE" )
        {
            Json::Value AGVMove;
            AGVMove["Move_Result"] = agv_move_result_;
            AGVInfor["MOVE"] = AGVMove;
        }
        else if ( comm_type_ == "PROC" )
        {
            Json::Value AGVProc;
            AGVProc["Proc_Name"] = proc_name_;
            AGVProc["Proc_State"] = agv_proc_state_;
            AGVProc["Proc_Result"] = 1.0;
            AGVInfor["PROC"] = AGVProc;
        }

        if ( need_hand_shake_ ) //握手
        {
            need_hand_shake_ = false;
            //ROS_INFO ("hand_shake AGVInfor[Comm_ID]: %d", dispatch_comm_id_);
            LOG(INFO) << "hand_shake AGVInfor[Comm_ID]: " << dispatch_comm_id_;
        }
        else //任务完成，状态复位
        {
            //ROS_INFO ("MsgAGVToDispatch trajectorie finished");
            LOG(INFO) << "MsgAGVToDispatch trajectorie finished";
            answer_comm_id_ = 0;
            agv_move_result_ = task_status_;
            agv_proc_state_ = "Execute";
        }
    }

    DeviceInfor["Dev_Data"] = AGVInfor;

    msg_agv_to_dispatch = DeviceInfor.toStyledString();
    msg_agv_to_dispatch += 0x0d;
    msg_agv_to_dispatch += 0x0a;
}

void Dispatch::ExecuteTask()
{
    static ros::Time trajectories_add_time;
    static ros::Time task_finish_time;

    //cout << "state_machine_: " << state_machine_ << endl;
    if ( task_state_ == IDLE )
    {
        if ( !trajectory_list_msg_.trajectories.empty() )
        {
            cout << "trajectory_list_msg_.trajectories[0].name: " << trajectory_list_msg_.trajectories[0].name << endl;
            task_state_ = traj_exist_in_waypoints_ ? PUB_NAVIGATION_CONTROL : PUB_TRAJECTORIE_ADD;
        }
    }
    else if ( task_state_ == PUB_TRAJECTORIE_ADD )
    {
        trajectories_add_time = ros::Time::now();
        TrajectorieAddPub();
        task_state_ = PUB_NAVIGATION_CONTROL;
    }
    else if ( task_state_ == PUB_NAVIGATION_CONTROL )
    {
        if ( ros::Time::now() - trajectories_add_time > ros::Duration(0.3) )
        {
            NavigationControlPub();
            task_state_ = WAIT_MOVE_FINISH;
        }
    }
    else if ( task_state_ == WAIT_MOVE_FINISH )
    {
        if ( trajectorie_finished_ )
        {
            if ( comm_type_ != "MOVE" )
            {
                //get_hand_shake_from_dispatch_ = false;
                if ( !traj_exist_in_waypoints_ )
                {
                    TrajectorieRemovePub();
                }

                trajectory_list_msg_.trajectories.erase(trajectory_list_msg_.trajectories.begin());
                //ROS_INFO ("state machine trajectorie finished: %d", dispatch_comm_id_);
                LOG(INFO) << "state machine trajectorie finished: " << dispatch_comm_id_;

                if ( comm_mode_ == "need_finished_hand_shake" )
                {
                    task_state_ = WAIT_DISPATCH_HAND_SHAKE;
                    task_finish_time = ros::Time::now();
                }
                else
                {
                    task_state_ = IDLE;
                }

                traj_exist_in_waypoints_ = false;
                trajectorie_finished_ = false;
                get_new_move_ = false;

                answer_comm_id_ = dispatch_comm_id_;
                agv_move_result_ = yocs_msgs::NavigationControlStatus::COMPLETED;
                agv_proc_state_ = "Finish";
            }
            else if ( waypoint_name_trajectorie_finished_ == trajectory_list_msg_.trajectories[0].waypoints[0].name )
            {
                if ( !traj_exist_in_waypoints_ )
                {
                    TrajectorieRemovePub();
                }

                trajectory_list_msg_.trajectories.erase(trajectory_list_msg_.trajectories.begin());
                //ROS_INFO ("state machine trajectorie finished: %d", dispatch_comm_id_);
                LOG(INFO) << "state machine trajectorie finished: " << dispatch_comm_id_;

                if ( comm_mode_ == "need_finished_hand_shake" )
                {
                    task_state_ = WAIT_DISPATCH_HAND_SHAKE;
                    task_finish_time = ros::Time::now();
                }
                else
                {
                    task_state_ = IDLE;
                }

                traj_exist_in_waypoints_ = false;
                trajectorie_finished_ = false;
                get_new_move_ = false;

                answer_comm_id_ = dispatch_comm_id_;
                agv_move_result_ = yocs_msgs::NavigationControlStatus::COMPLETED;
                agv_proc_state_ = "Finish";
            }
        }
        if ( get_new_move_ )
        {
            if ( !traj_exist_in_waypoints_ )
            {
                TrajectorieRemovePub();
            }

            trajectory_list_msg_.trajectories.erase(trajectory_list_msg_.trajectories.begin());
            //ROS_INFO ("get new move, state machine to IDLE");
            LOG(INFO) << "get new move, state machine to IDLE";

            task_state_ = IDLE;
            traj_exist_in_waypoints_ = false;
            trajectorie_finished_ = false;

            get_new_move_ = false;
        }
    }
    else if ( task_state_ == WAIT_DISPATCH_HAND_SHAKE )
    {
        if ( get_hand_shake_from_dispatch_ )
        {
            task_state_ = IDLE;
            get_hand_shake_from_dispatch_ = false;
        }
        else
        {
            if ( ros::Time::now() - task_finish_time > ros::Duration(3.0) )
            {
                task_finish_time = ros::Time::now();
                answer_comm_id_ = dispatch_comm_id_;
                agv_move_result_ = yocs_msgs::NavigationControlStatus::COMPLETED;
                agv_proc_state_ = "Finish";
            }
        }
    }
    else
    {
        cout << "undefined state_machine_: " << task_state_ << endl;
    }
}

inline int AGVWrite(int fd, const char *buffer,int length)
{
    int bytes_left;
    int written_bytes;
    const char *ptr;

    ptr=buffer;
    bytes_left=length;
    while(bytes_left>0)
    {
        written_bytes=write(fd,ptr,bytes_left);
        if(written_bytes<=0)
        {
            if(errno==EINTR)
            {
                written_bytes=0;
                //ROS_FATAL("INTERRUPT ERROR!!!");
                LOG(FATAL) << "INTERRUPT ERROR!!!";
            }
            else
                return(-1);
        }
        bytes_left-=written_bytes;
        ptr+=written_bytes;
    }
    return(0);
}


void Dispatch::Run()
{
    char buffer[BUFFERSIZE];
    bool connect_state = false;

    google::InitGoogleLogging("KLH");

    google::SetLogDestination(google::GLOG_FATAL, "../log/log_fatal_"); // 设置 google::FATAL 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::GLOG_ERROR, "../log/log_error_"); //设置 google::ERROR 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::GLOG_WARNING, "../log/log_warning_"); //设置 google::WARNING 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::GLOG_INFO, "../log/log_info_"); //设置 google::INFO 级别的日志存储路径和文件名前缀
 /*   sleep(20);*/
//    ros::spinOnce();
//    trajectorie_finished_ = false;

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();

        if ( connect_state == false )
        {
            connect_state = ConnectServer ( server_ip_, server_port_ );
            if ( connect_state )
            {
                cout << "socket_fd_: " << socket_fd_ << endl;
                //set timeout
                struct timeval timeout = {1, 0};
                //int ret_send = setsockopt( socket_fd_external_server, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout) );
                int ret_recv = setsockopt ( socket_fd_, SOL_SOCKET, SO_RCVTIMEO, ( const char* ) &timeout, sizeof ( timeout ) );
            }
            else
            {
                //ROS_ERROR ( "setup connect to server failed! will retry in 1s" );
                LOG(ERROR) << "setup connect to server failed! will retry in 1s";
                close(socket_fd_);
                ros::Duration ( 1.0 ).sleep();
                continue;
            }
        }

        memset(buffer, 0, BUFFERSIZE);

        int recv_length = recv(socket_fd_, buffer, BUFFERSIZE, MSG_DONTWAIT);
        //cout << "recv_length: " << recv_length << endl;

        if (recv_length > 0)
        {
            string str_recv = buffer;

            //ReceiveData(buffer, recv_length);
            ReceiveDataSplicing(str_recv);

            if ( m_InforList_.empty() )
            {
                ROS_WARN("m_InforList_ is empty");
                loop_rate.sleep();
                continue;
            }

            if ( DeserializedJson(m_InforList_[0]) )
            {
                m_InforList_.erase(m_InforList_.begin());
            }

//            if ( !DeserializedJson(m_InforList_[0]) )
//            {
//                loop_rate.sleep();
//                continue;
//            }
//            m_InforList_.erase(m_InforList_.begin());
        }
        else if ( recv_length == -1 )
        {
            // cout << "no recv data" << endl;
        }
        else if ( recv_length == 0 )
        {
            //ROS_ERROR ( "disconnented from server! will retry in 1s" );
            LOG(ERROR) << "disconnented from server! will retry in 1s";
            connect_state = false;
            close(socket_fd_);
            ros::Duration ( 1.0 ).sleep();
            continue;
        }

        ExecuteTask();

        string msg_agv_to_dispatch;
        MsgAGVToDispatch("AGV", msg_agv_to_dispatch);

        int write_length = AGVWrite(socket_fd_, msg_agv_to_dispatch.c_str(), msg_agv_to_dispatch.size());

        if (write_length < 0)
        {
            //ROS_ERROR("CONNECTION CLOSED");
            LOG(ERROR) << "CONNECTION CLOSED";
            connect_state = false;
            close(socket_fd_);
            ros::Duration ( 1.0 ).sleep();
            continue;
        }

        loop_rate.sleep();
    }
    close ( socket_fd_ );
}

void Dispatch::ReceiveDataSplicing( const string& str_recv )
{
    string str_end = "\r\n";

    str_all_ += str_recv;

    while ( str_all_.size() > 0  )
    {
        int index_end = str_all_.find(str_end);
        if ( index_end != string::npos )
        {
            string one_times = str_all_.substr( 0, index_end );
            m_InforList_.push_back(one_times);
            str_all_.erase( 0, index_end + 2 );
        }
        else
        {
            break;
        }
    }
}

void Dispatch::ReceiveData(char* p_buffer_start, int buffer_size)
{
    std::string str_all;

    char* pStart = p_buffer_start;
    char* pOver = NULL;
    char* pTail = NULL;

    for(int i = 0; i < (buffer_size-1); i++)
    {
        char* p1 = (p_buffer_start + i);
        char* p2 = (p_buffer_start + i + 1);

        if( (0x0d == *p1) && (0x0a == *p2) )
        {
            pOver = p2;

            char  buf1[BUFFERSIZE];
            memset(buf1, 0, BUFFERSIZE);
            memcpy(buf1, pStart, (pOver - pStart - 1));

            std::string  tempStr = buf1;
            str_all = m_lastPlus_ + tempStr;
            m_lastPlus_.clear();
            m_InforList_.push_back(str_all);

            if(i < (buffer_size-2))
                pStart = pOver + 1;
        }
        else
        {
            //last char
            if( i == (buffer_size-2))
            {
                char  buf1[BUFFERSIZE];
                memset(buf1, 0, BUFFERSIZE);
                memcpy(buf1, pStart, ( (p_buffer_start + buffer_size -1)  - pStart + 1));

                std::string  tempStr = buf1;
                m_lastPlus_ += tempStr;
            }
        }

        if( (0x00 == *p1) && (0x00 == *p2) )
        {
            pTail = p1;

            if( pStart != pTail )
            {
                char  buf1[BUFFERSIZE];
                memset(buf1, 0, BUFFERSIZE);
                memcpy(buf1, pStart, (pTail - pStart + 1));

                std::string  tempStr = buf1;
                m_lastPlus_ += tempStr;
            }

            break;
        }
    }
}
