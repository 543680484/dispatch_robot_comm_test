#include <dispatch_robot_comm/dispatch_robot_comm.h>

#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)

Dispatch::Dispatch()
    :nh_()
    ,robot_pose_sub_()
    ,navigation_control_status_sub_()
    ,trajectories_add_pub_()
    ,trajectories_remove_pub_()
    ,navigation_control_pub_()
    ,task_state_(IDLE)
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
    ,trajectory_list_msg_()
    ,traj_exist_in_waypoints_(false)
    ,first_magnetic_nav_point_(true)
    ,proc_name_()
    ,agv_pose2d_()
    ,get_new_move_(false)
    ,battery_percentage_(1.0)
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
}

Dispatch::~Dispatch(){}

bool Dispatch::ConnectServer (string ip_str, int port )
{
    cout << "ip: " << ip_str << " port: " << port << endl;

    struct sockaddr_in server_addr;

    if ( ( socket_fd_ = socket ( AF_INET, SOCK_STREAM, 0 ) ) < 0 )
    {
        LERROR("create socket error: %s(errno:%d)\n)", strerror ( errno ), errno);
        return false;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if ( inet_pton ( AF_INET, ip_str.c_str(), &server_addr.sin_addr ) <= 0 )
    {
        LERROR( "inet_pton error: %s(errno:%d))\n", strerror ( errno ), errno );
        return false;
    }

    if ( SetNonBlock ( socket_fd_, ( struct sockaddr* ) &server_addr, sizeof ( server_addr ), 0, 1000000 ) == 0 )
    {
        LINFO( "AGV connected!" );
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
        LDEBUG( "select error: sockfd  not set" );
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
    LINFO("NavigationControlPub: ", trajectory_list_msg_.trajectories[0].name);

    navigation_control_pub_.publish(navigation_control_msg);
}

void Dispatch::NavigationControlPub( string trajectories_name )
{
    yocs_msgs::NavigationControl navigation_control_msg;

    navigation_control_msg.control = yocs_msgs::NavigationControl::START;
    navigation_control_msg.goal_name = trajectories_name;

    navigation_control_pub_.publish(navigation_control_msg);
}

void Dispatch::TrajectorieRemovePub(string trajectories_name)
{
    yocs_msgs::Trajectory trajectory_msg;
    trajectory_msg.name = trajectories_name;
    LINFO("TrajectorieRemovePub: ", trajectory_msg.name);

    trajectories_remove_pub_.publish(trajectory_msg);
}

void Dispatch::NavigationControlStatusCallback(const yocs_msgs::NavigationControlStatusConstPtr &navigation_control_msg_ptr)
{
    string waypoint_name_finished = navigation_control_msg_ptr->waypoint_name;

    if ( navigation_control_msg_ptr->status == yocs_msgs::NavigationControlStatus::COMPLETED )
    {
        if (comm_type_ != "MOVE")
        {
            trajectorie_finished_ = true;
            LINFO("trajectorie finished_PROC: ", waypoint_name_finished.c_str());
        }
        else if ( waypoint_name_finished == trajectory_list_msg_.trajectories[0].waypoints[0].name )
        {
            trajectorie_finished_ = true;
            LINFO("trajectorie finished_MOVE: ", waypoint_name_finished.c_str());
        }
        else
        {
            LWARN("trajectorie finished_not_current: ", waypoint_name_finished.c_str());
        }
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
        LWARN("no move point");
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
        LWARN("no move point");
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
        LERROR("undefined nav mode: ", nav_mode.c_str());
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
        LERROR("undefined nav mode: ", nav_mode.c_str());
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
//    cout << "MAGNETICTRACK_LOCATION x: " << trajectory_msg.waypoints[0].pose.position.x << endl;
    LINFO("MAGNETICTRACK_LOCATION x: ", trajectory_msg.waypoints[0].pose.position.x);

    trajectory_msg.waypoints[0].pose.orientation.x = 0;
    trajectory_msg.waypoints[0].pose.orientation.y = 0;
    trajectory_msg.waypoints[0].pose.orientation.z = 0;
    trajectory_msg.waypoints[0].pose.orientation.w = 1;
}

bool Dispatch::MotorAction( yocs_msgs::Trajectory& trajectory_msg, const Json::Value& procData )
{
    string motor_num = procData["Proc_Para_1"].asString();
    string motor_action = procData["Proc_Para_2"].asString();
    if ( motor_num == "0" )
    {
        if ( motor_action == "LOAD" )
        {
            trajectory_msg.name = "traj_agv_load_motor0";
            traj_exist_in_waypoints_ = true;
        }
        else if ( motor_action == "UNLOAD" )
        {
            trajectory_msg.name = "traj_agv_unload_motor0";
            traj_exist_in_waypoints_ = true;
        }
        else
        {
            LERROR("undefined mode: ", motor_action.c_str());
            return false;
        }
    }
    else if ( motor_num == "1" )
    {
        if ( motor_action == "LOAD" )
        {
            trajectory_msg.name = "traj_agv_load_motor1";
            traj_exist_in_waypoints_ = true;
        }
        else if ( motor_action == "UNLOAD" )
        {
            trajectory_msg.name = "traj_agv_unload_motor1";
            traj_exist_in_waypoints_ = true;
        }
        else
        {
            LERROR("undefined mode: ", motor_action.c_str());
            return false;
        }
    }
    else if ( motor_num == "2" )
    {
        if ( motor_action == "LOAD" )
        {
            trajectory_msg.name = "traj_agv_load_motor2";
            traj_exist_in_waypoints_ = true;
        }
        else if ( motor_action == "UNLOAD" )
        {
            trajectory_msg.name = "traj_agv_unload_motor2";
            traj_exist_in_waypoints_ = true;
        }
        else
        {
            LERROR("undefined mode: ", motor_action.c_str());
            return false;
        }
    }
    else
    {
        LERROR("undefined nav mode: ", motor_num.c_str());
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

bool Dispatch::MsgFromDispatchParse(string str_complete)
{
    Json::Reader reader;
    Json::Value json_object;

    if(reader.parse(str_complete, json_object))
    {
        Json::Value dev_data = json_object["Dev_Data"];

        if ( dev_data.empty() )
        {
            LERROR("dev data is empty");
            return false;
        }

        long long int time_stamp_sec = dev_data["TimeStamp_sec"].asInt64();
        int time_stamp_usec = dev_data["TimeStamp_usec"].asInt();

        dispatch_comm_id_ = dev_data["Comm_ID"].asInt();
        int resend_times = dev_data["Time"].asInt();

        string comm_type = dev_data["Comm_Type"].asString();
        if ( comm_type == "MOVE" || comm_type == "PROC" )
        {
            need_hand_shake_ = true;
        }
        else
        {
            need_hand_shake_ = false;
        }

        if ( time_stamp_recv_sec_ == time_stamp_sec && time_stamp_recv_usec_ == time_stamp_usec )
        {
            LWARN("recv same command !!!");
            return false;
        }

        time_stamp_recv_sec_ = time_stamp_sec;
        time_stamp_recv_usec_ = time_stamp_usec;

        if ( comm_type == "MOVE" && comm_type_ == "MOVE" )
        {
            if ( IDLE != task_state_ )
            {
                get_new_move_ = true;
                LINFO("get_new_move_ = true， task_state_ is: ", task_state_);
            }
        }

        comm_type_ = comm_type;

        yocs_msgs::Trajectory trajectory_msg;

        if ( "MOVE" == comm_type_ )
        {
            Json::Value MoveData = dev_data["MOVE"];

            if ( MoveData.empty() )
            {
                LERROR("movedata empty");
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
                LERROR("proc empty");
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
                if ( !MotorAction(trajectory_msg, procData) )
                {
                    return false;
                }
            }
            else if ( proc_name_ == "LOCATINGPIN" )
            {
                if ( !MotorAction(trajectory_msg, procData) )
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
                LERROR("undefined proc name: ", proc_name_.c_str());
                return false;
            }
        }
        else if ( "FB_RT_MOVE" == comm_type_ || "FB_RT_PROC" == comm_type_ )
        {
            get_hand_shake_from_dispatch_ = true;
            return true;
        }
        else
        {
            LERROR("undefined comm type: ", comm_type_.c_str());
            return false;
        }

        trajectory_list_msg_.trajectories.push_back(trajectory_msg);
        return true;
    }
    else
    {
        LERROR("parse msg from dispatch error");
        return false;
    }
}

void Dispatch::MsgToDispatch(string dev_type, string& msg_agv_to_dispatch)
{
    struct timeval time_value;
    gettimeofday(&time_value,NULL);

    Json::Value DeviceInfor;
    DeviceInfor["Dev_Type"] = dev_type;
    DeviceInfor["Dev_ID"] = dev_id_;

    Json::Value AGVInfor;
    AGVInfor["TimeStamp_sec"] = static_cast<long long int>(time_value.tv_sec);
    //AGVInfor["TimeStamp_usec"] = static_cast<int>(time_value.tv_usec);

    AGVInfor["Map_ID"] = 1;
    AGVInfor["X"] = agv_pose2d_.x;
    AGVInfor["Y"] = agv_pose2d_.y;
    AGVInfor["a"] = agv_pose2d_.theta;

    AGVInfor["Cell_Quan"] = battery_percentage_;
    AGVInfor["ERROR"] = 1;

    //一般状态，握手，任务完成
    if ( trajectorie_finished_ )
    {
        AGVInfor["Comm_ID"] = dispatch_comm_id_;
        AGVInfor["Data_Type"] = "RT_" + comm_type_;

        if ( comm_type_ == "PROC" )
        {
            Json::Value AGVProc;
            AGVProc["Proc_Name"] = proc_name_;
            AGVProc["Proc_Result"] = 1.0;
            AGVInfor["RT_PROC"] = AGVProc;
        }
        trajectorie_finished_ = false;
        LINFO("trajectorie_finished to dispatch");
    }
    else if ( need_hand_shake_ )
    {
        AGVInfor["Comm_ID"] = dispatch_comm_id_;
        AGVInfor["Data_Type"] = "FB_" + comm_type_;

        if ( comm_type_ == "PROC" )
        {
            Json::Value AGVProc;
            AGVProc["Proc_Name"] = proc_name_;
            AGVInfor["FB_PROC"] = AGVProc;
        }
        need_hand_shake_ = false;
    }
    else
    {
        AGVInfor["Comm_ID"] = 0;
        AGVInfor["Data_Type"] = "NORMAL";
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
    static vector<string> trajectories_to_remove;

    if ( task_state_ == IDLE )
    {
        if ( !trajectory_list_msg_.trajectories.empty() )
        {
            LINFO("start task: ", trajectory_list_msg_.trajectories[0].name);
            task_state_ = traj_exist_in_waypoints_ ? PUB_NAVIGATION_CONTROL : PUB_TRAJECTORIE_ADD;
            get_new_move_ = false;
        }
        if ( !trajectories_to_remove.empty() && (ros::Time::now() - trajectories_add_time > ros::Duration(15.0)))
        {
            TrajectorieRemovePub(trajectories_to_remove[0]);
            LINFO("trajectories_to_remove.erase: ", trajectories_to_remove[0]);
            trajectories_to_remove.erase(trajectories_to_remove.begin());
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
            task_state_ = WAIT_TASK_FINISH;
        }
    }
    else if ( task_state_ == WAIT_TASK_FINISH )
    {
        if ( trajectorie_finished_ || get_new_move_ )
        {
            LINFO("get_new_move_: ", get_new_move_);
            if ( !traj_exist_in_waypoints_ )
            {
                trajectories_to_remove.push_back(trajectory_list_msg_.trajectories[0].name);
                LINFO("trajectories_to_remove.push_back: ", trajectory_list_msg_.trajectories[0].name);
            }

            LINFO("trajectory_list_msg_.trajectories.erase: ", trajectory_list_msg_.trajectories[0].name);
            trajectory_list_msg_.trajectories.erase(trajectory_list_msg_.trajectories.begin());

            if (trajectorie_finished_)
            {
                task_state_ = WAIT_DISPATCH_HAND_SHAKE;
                task_finish_time = ros::Time::now();
            }
            else
            {
                task_state_ = IDLE;
            }

            traj_exist_in_waypoints_ = false;
            get_new_move_ = false;
            LINFO("get_new_move_ = false");
        }
    }
    else if ( task_state_ == WAIT_DISPATCH_HAND_SHAKE )
    {
        if ( get_hand_shake_from_dispatch_ || get_new_move_ )
        {
            task_state_ = IDLE;
            LINFO("get_hand_shake_from_dispatch_: ", get_hand_shake_from_dispatch_);
            get_hand_shake_from_dispatch_ = false;
        }
        else
        {
            if ( ros::Time::now() - task_finish_time > ros::Duration(3.0) )
            {
                trajectorie_finished_ = true;//重发
                task_finish_time = ros::Time::now();
                LINFO("resend trajectorie finished to dispatch");
            }
        }
    }
    else
    {
        LINFO("undefined state_machine_: ", task_state_);
    }
}

int Dispatch::AGVWrite(int fd, const char *buffer,int length)
{
    int bytes_left;
    int written_bytes;
    const char *ptr;

    ptr=buffer;
    bytes_left=length;
    while(bytes_left>0)
    {
        written_bytes=write(fd,ptr,bytes_left);
        if(written_bytes==0)
        {
            return -1;
        }
        else if (written_bytes<0)
        {
            if(errno==EINTR)
            {
                written_bytes=0;
                LWARN("INTERRUPT ERROR!!!");
            }
            else if (errno==EAGAIN || errno==EWOULDBLOCK)
            {
                LWARN("EAGAIN or EWOULDBLOCK ERROR!!!");
                return (0);
            }
            else
            {
                return(-1);
            }
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
    vector<string> msg_complete_vector;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if ( connect_state == false )
        {
            connect_state = ConnectServer ( server_ip_, server_port_ );
            if ( connect_state )
            {
                LINFO("socket_fd_: ", socket_fd_);
                //set timeout
                struct timeval timeout = {1, 0};
                //int ret_send = setsockopt( socket_fd_external_server, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout) );
                int ret_recv = setsockopt ( socket_fd_, SOL_SOCKET, SO_RCVTIMEO, ( const char* ) &timeout, sizeof ( timeout ) );
            }
            else
            {
                LERROR("setup connect to server failed! will retry in 2s");
                close(socket_fd_);
                ros::Duration ( 2.0 ).sleep();
                continue;
            }
        }

        memset(buffer, 0, BUFFERSIZE);

        int recv_length = recv(socket_fd_, buffer, BUFFERSIZE, MSG_DONTWAIT);

        if (recv_length > 0)
        {
            string str_recv = buffer;
            LINFO("str recv:", str_recv);
            MsgFromDispatchSplicing(str_recv, msg_complete_vector);

            if ( !msg_complete_vector.empty() )
            {
                MsgFromDispatchParse(msg_complete_vector[0]);
                msg_complete_vector.erase(msg_complete_vector.begin());
            }
            else
            {
                LWARN("msg_complete_vector is empty");
            }
        }
        else if ( recv_length == -1 )
        {
            // cout << "no recv data" << endl;
        }
        else if ( recv_length == 0 )
        {
            LERROR( "disconnented from server! will retry in 1s" );
            connect_state = false;
            close(socket_fd_);
            ros::Duration ( 1.0 ).sleep();
            continue;
        }

        ros::spinOnce();
        ExecuteTask();

        string msg_agv_to_dispatch;
        MsgToDispatch("AGV", msg_agv_to_dispatch);

        //int write_length = write(socket_fd_, msg_agv_to_dispatch.c_str(), msg_agv_to_dispatch.size());
        int write_length = AGVWrite(socket_fd_, msg_agv_to_dispatch.c_str(), msg_agv_to_dispatch.size());

        if (write_length < 0)
        {
            LERROR("agv write to dispatch ERROR");
            connect_state = false;
            close(socket_fd_);
            ros::Duration ( 1.0 ).sleep();
            continue;
        }

        loop_rate.sleep();
    }
    close ( socket_fd_ );
}

void Dispatch::MsgFromDispatchSplicing( const string& str_recv, vector<string>& msg_complete )
{
    static string str_all;
    const string msg_end = "\r\n";

    str_all += str_recv;

    while ( str_all.size() > 0  )
    {
        int index_end = str_all.find(msg_end);
        if ( index_end != string::npos )
        {
            string one_times = str_all.substr( 0, index_end );
            msg_complete.push_back(one_times);
            str_all.erase( 0, index_end + 2 );
        }
        else
        {
            break;
        }
    }
}
