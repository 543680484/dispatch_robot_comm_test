#include <magnetic_tf_pub.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
#define SIGN(n) (n <= 0?( n < 0 ?-1:0):1)
#define RadianToAngle(X) ((X)*180.0/M_PI)
#define AngleToRadian(X) ((X)*M_PI/180.0)

MagneticTfPub::MagneticTfPub()
    : nh_()
    , task_switch_sub_()
    , magnetic_head_sub_()
    , magnetic_tail_sub_()
    , magnetic_detect_status_pub_()
    , magnetic_head_pose2d_()
    , magnetic_tail_pose2d_()
    , magnetic_head_is_forward_(true)
    , magnetic_tail_is_forward_(true)
    , count_detected_tail_marker_(true)
    , count_detected_head_marker_(true)
    , can_detected_tail_marker_(false)
    , can_detected_head_marker_(false)
    , detected_tail_marker_times_(0)
    , detected_head_marker_times_(0)
    , param_test_(0.0)
    , head_or_tail_(NON)
    , run_task_(false)
    , need_recod_marker_pose_in_odom_(true)
    , head_detected_magnetic_(false)
    , tail_detected_magnetic_(false)
    , signal_for_head_out_track_(0)
    , signal_for_tail_out_track_(0)
    , odometry_marker_start_()
    , odometry_marker_end_()
    , get_marker_middle_pose_(false)
//    , need_set_bzp_min_vel_x_(true)
{
    odom_sub_ = nh_.subscribe("/odom", 1, &MagneticTfPub::OdomCallback, this);
    task_switch_sub_ = nh_.subscribe("/task_switch", 1, &MagneticTfPub::TaskSwitchCallback, this );
    magnetic_head_sub_ = nh_.subscribe("/magnetic_head_data", 1, &MagneticTfPub::MagneticHeadCallback, this);
    magnetic_tail_sub_ = nh_.subscribe("/magnetic_tail_data", 1, &MagneticTfPub::MagneticTailCallback, this);

    magnetic_detect_status_pub_ = nh_.advertise<std_msgs::Bool>("/magnetic_detected", 1);

    ros::NodeHandle nh_p("~");
    double distance_magnetic_head_to_robot_center;
    double distance_magnetic_tail_to_robot_center;
    
    nh_p.param("magnetic_head_is_forward", magnetic_head_is_forward_, true);
    nh_p.param("magnetic_tail_is_forward", magnetic_tail_is_forward_, true);
    nh_p.param("distance_magnetic_head_to_robot_center", distance_magnetic_head_to_robot_center, 0.27);
    nh_p.param("distance_magnetic_tail_to_robot_center", distance_magnetic_tail_to_robot_center, 0.27);
    nh_p.param("points_num_consider_detected_magnetic_marker", points_num_consider_detected_magnetic_marker_, 8);
    nh_p.param("fake_pose_y_out_track_cm", fake_pose_y_out_track_cm_, 5.0);
    nh_p.param("param_test", param_test_, 1.0);
    nh_p.param("output_odometry_from_marker", output_odometry_from_marker_, false);

    magnetic_head_pose2d_.x = distance_magnetic_head_to_robot_center;
    magnetic_tail_pose2d_.x = -distance_magnetic_tail_to_robot_center;
}

MagneticTfPub::~MagneticTfPub()
{}

void MagneticTfPub::TaskSwitchCallback(const std_msgs::HeaderPtr &task_switch_msg)
{
    size_t found_node = task_switch_msg->frame_id.find("magnetic_tf_pub");
    if ( found_node != std::string::npos || task_switch_msg->frame_id.empty() )
    {
        if ( task_switch_msg->seq == 0 )
        {
            cout << "stop run" << endl;
            StopRun();
        }
        else
        {
            cout << "start run" << endl;
            run_task_ = true;
            ResetParam();
        }
    }
}

void MagneticTfPub::MagneticHeadCallback(const std_msgs::Int32MultiArrayPtr magnetic_head_msg)
{
    int num_detected_magnetic = 0;

    magnetic_head_pose2d_.y = GetPoseYInRobotBase(magnetic_head_is_forward_, magnetic_head_msg, signal_for_head_out_track_, num_detected_magnetic);

    if ( num_detected_magnetic == 0 ) //未检测到磁条
    {
        head_detected_magnetic_ = false;
    }
    else //检测到磁条
    {
        head_detected_magnetic_ = true;
        signal_for_head_out_track_ = SIGN(magnetic_head_pose2d_.y);

        if ( num_detected_magnetic > points_num_consider_detected_magnetic_marker_ )
        {
            can_detected_head_marker_ = true;
            if ( head_or_tail_ == NON )
            {
                head_or_tail_ = HEAD;
            }
        }
        else
        {
            can_detected_head_marker_ = false;
        }
    }
}

void MagneticTfPub::MagneticTailCallback(const std_msgs::Int32MultiArrayPtr magnetic_tail_msg)
{
    int num_detected_magnetic = 0;

    magnetic_tail_pose2d_.y = GetPoseYInRobotBase(magnetic_tail_is_forward_, magnetic_tail_msg, signal_for_tail_out_track_, num_detected_magnetic);

    if ( num_detected_magnetic == 0 ) //未检测到磁条
    {
        tail_detected_magnetic_ = false;
    }
    else //检测到磁条
    {
        tail_detected_magnetic_ = true;
        signal_for_tail_out_track_ = SIGN(magnetic_tail_pose2d_.y);

        if ( num_detected_magnetic > points_num_consider_detected_magnetic_marker_ )
        {
            can_detected_tail_marker_ = true;
            if ( head_or_tail_ == NON )
            {
                head_or_tail_ = TAIL;
            }
        }
        else
        {
            can_detected_tail_marker_ = false;
        }
    }
}

void MagneticTfPub::OdomCallback(const nav_msgs::OdometryConstPtr odometry_msg)
{
//    odometry_recoder_ = *odometry_msg;
    odometry_recoder_.pose.pose.position.x = odometry_msg->pose.pose.position.x;
    odometry_recoder_.pose.pose.position.y = odometry_msg->pose.pose.position.y;
}

void MagneticTfPub::ResetParam()
{
    head_or_tail_ = NON;

    need_recod_marker_pose_in_odom_ = true;

    count_detected_tail_marker_ = true;
    count_detected_head_marker_ = true;

    signal_for_head_out_track_ = 0;
    signal_for_tail_out_track_ = 0;

    detected_tail_marker_times_ = 0;
    detected_head_marker_times_ = 0;

    get_marker_middle_pose_ = false;
//    need_set_bzp_min_vel_x_ = true;
}

void MagneticTfPub::StopRun()
{
    run_task_ = false;

    ResetParam();
}

double MagneticTfPub::GetPoseYInRobotBase(bool magnetic_is_forward, const std_msgs::Int32MultiArrayPtr magnetic_msg, int get_signal, int& num_detected_magnetic)
{
    double pose_y_sum = 0.0;

    int signal = 0;
    if ( magnetic_is_forward )
    {
        signal = -1;
    }
    else
    {
        signal = 1;
    }

    int magnetic_msg_size = magnetic_msg->data.size();

    float magnetic_data_middle_index = (magnetic_msg_size - 1) / 2.0;

    for ( int i = 0; i < magnetic_msg_size; ++i )
    {
        if (magnetic_msg->data[i] != 0)
        {
            ++num_detected_magnetic;
            pose_y_sum += signal * (magnetic_data_middle_index - i);
        }
    }

    double pose_y_cm = 0.0;
    // static int get_signal = 0;

    if ( num_detected_magnetic == 0 ) //未检测到磁条
    {
        // pose_y_cm = get_signal * fake_pose_y_out_track_cm_;
        pose_y_cm = 0.0;
    }
    else //检测到磁条
    {
        pose_y_cm = (pose_y_sum / num_detected_magnetic);

        // get_signal = SIGN(pose_y_cm);
    }

    return pose_y_cm * 0.01; //from cm to m
}

double MagneticTfPub::FakeYForControl(double magnetic_y_cm)
{
    double fake_y_for_control_m = magnetic_y_cm * fabs(magnetic_y_cm) * 0.01;// * 2;
    return fake_y_for_control_m;
}

void MagneticTfPub::SendTransform( double x, double y, double theta )
{
    tf::Quaternion q;
    tf::Transform transform;
    static tf::TransformBroadcaster br;

    q.setRPY(0.0, 0.0, theta);
    transform.setRotation(q);
    transform.setOrigin( tf::Vector3(x, y, 0.0) );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "magnetic_center"));
}

//计算base_link与磁条的垂足
void MagneticTfPub::GetMagneticPoseInRobot(double& magnetic_tf_x, double& magnetic_tf_y, double& magnetic_tf_theta)
{
    //由两点求直线方程的一般式AX+BY+C=0, A=Y2-Y1, B=X1-X2, C=X2*Y1-X1*Y2;
    //求已知点到已知直线的垂足( (B*B*X-A*B*Y-A*C)/(A*A+B*B), (A*A*Y-A*B*X-B*C)/(A*A+B*B) )

    double dx = magnetic_head_pose2d_.x - magnetic_tail_pose2d_.x;
    double dy = magnetic_head_pose2d_.y - magnetic_tail_pose2d_.y;

    double A = dy;
    double B = -dx;
    double C = magnetic_head_pose2d_.x * magnetic_tail_pose2d_.y - magnetic_tail_pose2d_.x * magnetic_head_pose2d_.y;
    double Apow2_add_Bpow2 = A*A + B*B;

    magnetic_tf_x = -A*C/Apow2_add_Bpow2;
    magnetic_tf_y = -B*C/Apow2_add_Bpow2;
    magnetic_tf_theta = atan2(dy, dx);

    //cout << "magnetic_tf_x: " << magnetic_tf_x << endl;
    //cout << "magnetic_tf_y: " << magnetic_tf_y << endl;
    //cout << "magnetic_tf_theta: " << magnetic_tf_theta << endl;
}

double MagneticTfPub::GetMarkerInOdometryDiff()
{
    static double marker_in_odometry_x = 0.0;
    static double marker_in_odometry_y = 0.0;

    if ( need_recod_marker_pose_in_odom_ )//首次检测到磁条标记，记录odom
    {
        ROS_ERROR("-------------------get magnetic marker----------------------");
        marker_in_odometry_x = (odometry_marker_start_.x + odometry_marker_end_.x) / 2.0; //odometry_recoder_.pose.pose.position.x;
        marker_in_odometry_y = (odometry_marker_start_.y + odometry_marker_end_.y) / 2.0; //odometry_recoder_.pose.pose.position.y;

        need_recod_marker_pose_in_odom_ = false;
    }

    double dx = odometry_recoder_.pose.pose.position.x - marker_in_odometry_x;
    double dy = odometry_recoder_.pose.pose.position.y - marker_in_odometry_y;

    double marker_in_odometry_diff = sqrt( pow(dx,2) + pow(dy,2) );

    return marker_in_odometry_diff;
}

void MagneticTfPub::MagneticDetectStatusPub( bool magnetic_detected )
{
    std_msgs::Bool magnetic_detected_msg;
    magnetic_detected_msg.data = magnetic_detected;
    magnetic_detect_status_pub_.publish(magnetic_detected_msg);
}

void MagneticTfPub::Run()
{
    double magnetic_tf_theta = 0.0;
    double magnetic_tf_x = 0.0;
    double magnetic_tf_y = 0.0;

    int no_detected_magnetic_times = 0;
    int no_detected_tail_marker_times = 0;
    int no_detected_head_marker_times = 0;

    const static int rate_hz = 100;

    ros::Rate r(rate_hz);
    while ( ros::ok() )
    {
        ros::spinOnce();

        if ( !run_task_ )
        {
            SendTransform(0.0, 0.0, 0.0);
            r.sleep();
            continue;
        }

        if ( !head_detected_magnetic_ && !tail_detected_magnetic_ ) //前后均未检测到磁条
        {
            MagneticDetectStatusPub(false);
            ROS_WARN("both head and tail do not detected magnetic");
            ++ no_detected_magnetic_times;
            if ( no_detected_magnetic_times > 5*rate_hz )
            {
                StopRun();
            }

            SendTransform(0.0, 0.0, 0.0);
            r.sleep();
            continue;
        }
        else
        {
            MagneticDetectStatusPub(true);

            no_detected_magnetic_times = 0;
            int sign_by_orientation = 0;

            if ( head_or_tail_ == HEAD )
            {
                if ( can_detected_head_marker_ )
                {
                    if ( count_detected_head_marker_ )
                    {
                        detected_head_marker_times_ ++;
                        count_detected_head_marker_ = false;
                        if ( !get_marker_middle_pose_ )
                        {
                            cout << "get_head_marker_middle_pose_" << endl;
                            odometry_marker_start_.x = odometry_recoder_.pose.pose.position.x;
                            odometry_marker_start_.y = odometry_recoder_.pose.pose.position.y;
                        }
                    }
                    if ( no_detected_head_marker_times > 0 )
                    {
                        ROS_ERROR("no_detected_head_marker_times: %d", no_detected_head_marker_times);
                    }
                    no_detected_head_marker_times = 0;
                }
                else
                {
                    ++ no_detected_head_marker_times;
                    if ( no_detected_head_marker_times > 10 )
                    {
                        if ( !get_marker_middle_pose_ )
                        {
                            odometry_marker_end_.x = odometry_recoder_.pose.pose.position.x;
                            odometry_marker_end_.y = odometry_recoder_.pose.pose.position.y;
                            get_marker_middle_pose_ = true;
                        }
                        if ( no_detected_head_marker_times > 300 )
                        {
                            count_detected_head_marker_ = true;
                        }
                    }
                }
                sign_by_orientation = pow(-1, detected_head_marker_times_-1);
            }
            else if ( head_or_tail_ == TAIL )
            {                
                if ( can_detected_tail_marker_ )
                {
                    if ( count_detected_tail_marker_ )
                    {
                        detected_tail_marker_times_ ++;
                        count_detected_tail_marker_ = false;
                        if ( !get_marker_middle_pose_ )
                        {
                            cout << "get_tail_marker_middle_pose_" << endl;
                            odometry_marker_start_.x = odometry_recoder_.pose.pose.position.x;
                            odometry_marker_start_.y = odometry_recoder_.pose.pose.position.y;
                        }
                    }
                    if ( no_detected_tail_marker_times > 0 )
                    {
                        ROS_ERROR("no_detected_tail_marker_times: %d", no_detected_tail_marker_times);
                    }
                    no_detected_tail_marker_times = 0;
                }
                else
                {
                    ++ no_detected_tail_marker_times;
                    if ( no_detected_tail_marker_times > 10 )
                    {
                        if ( !get_marker_middle_pose_ )
                        {
                            odometry_marker_end_.x = odometry_recoder_.pose.pose.position.x;
                            odometry_marker_end_.y = odometry_recoder_.pose.pose.position.y;
                            get_marker_middle_pose_ = true;
                        }
                        if ( no_detected_tail_marker_times > 300 )
                        {
                            count_detected_tail_marker_ = true;
                        }
                    }
                }
                sign_by_orientation = pow(-1, detected_tail_marker_times_);
            }
            else //NON
            {}

            //cout << detected_head_marker_times_  << " : " << sign_by_orientation << endl;

            double marker_in_odometry_diff = 0.0;

            if ( head_or_tail_ != NON && get_marker_middle_pose_ )
            {
                marker_in_odometry_diff = sign_by_orientation * GetMarkerInOdometryDiff();//param_test_
//                if ( need_set_bzp_min_vel_x_ )
//                {
//                    int system_rtn = system("rosrun dynamic_reconfigure dynparam set /move_base/BZPlannerROS min_vel_x 0.02");
//                    need_set_bzp_min_vel_x_ = false;
//                }
            }

            GetMagneticPoseInRobot(magnetic_tf_x, magnetic_tf_y, magnetic_tf_theta);

            if ( output_odometry_from_marker_ )
            {
                cout << "odometry_from_marker: " << marker_in_odometry_diff*cos(magnetic_tf_theta) << endl;
            }
            double magnetic_tf_x_pub = magnetic_tf_x - marker_in_odometry_diff*cos(magnetic_tf_theta);
            double magnetic_tf_y_pub = magnetic_tf_y - marker_in_odometry_diff*sin(magnetic_tf_theta);

            SendTransform(magnetic_tf_x_pub, magnetic_tf_y_pub, magnetic_tf_theta);
        }

        r.sleep();
    }
}

int main(int argc, char** argv )
{
    ros::init(argc, argv, "magnetic_tf_pub");
    MagneticTfPub magnetic_tf_pub;
    magnetic_tf_pub.Run();
    return 0;
};
