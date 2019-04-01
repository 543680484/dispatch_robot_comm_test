#include <velocity_and_light_control.h>

VelocityAndLightControl::VelocityAndLightControl()
:ph_("~")
,nh_()
,cmd_vel_rectified_pub_()
,joy_feedback_pub_()
,robotsound_pub_()
,sound_player_pub_()
,cmd_vel_sub_()
,joy_sub_()
,battery_sub_()
,joy_()
,robot_in_move_(false)
,joy_estop_laser_trigged_(false)
,trigged_status_()
{
    InitParam();

    cmd_vel_rectified_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_rectified", 10, true);
    if (use_joy_feedback_)
    {
        joy_feedback_pub_ =  nh_.advertise<sensor_msgs::JoyFeedback>("joy_feedback", 10, true);
    }
    sound_player_pub_ = nh_.advertise<sound_play::SoundRequest>("robotsound", 10, true);

    joy_sub_ =  nh_.subscribe<sensor_msgs::Joy>("joy", 10, &VelocityAndLightControl::JoyCallback, this);
    cmd_vel_sub_ =  nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &VelocityAndLightControl::CmdVelSub, this);
    battery_sub_ = nh_.subscribe("/battery", 1, &VelocityAndLightControl::BatteryCallBack, this);

    joy_.axes.resize(JOYX_AXES);
    joy_.buttons.resize(JOYX_BUTTONS);
    trigged_status_.resize(BATTERY_LOW+1);

    ShowHelp();
}

void VelocityAndLightControl::InitParam()
{
    ph_.param("use_joy_feedback", use_joy_feedback_, false);
    double twist_factor;
    ph_.param("twist_factor", twist_factor, 1.0);
    double linear_factor;
    ph_.param("linear_factor", linear_factor, twist_factor);
    ph_.param("linear_x_factor", linear_x_factor_, linear_factor);
    double angular_factor;
    ph_.param("angular_factor", angular_factor, twist_factor);
    ph_.param("angular_z_factor", angular_z_factor_, angular_factor);

    ph_.param("volume", volume_, 15);
}

void VelocityAndLightControl::SoundPlayerPub( int sound )
{
    sound_play::SoundRequest msg_sound_player;
    msg_sound_player.sound = sound;//音乐(0x00~0xFF,0x00停止)
    msg_sound_player.command = volume_;//音量(0x00~0x1c 0~28)
    sound_player_pub_.publish(msg_sound_player);
}

void VelocityAndLightControl::CmdVelRectifiedPubStop()
{
    robot_in_move_ = false;

    geometry_msgs::Twist msg_zero;
    msg_zero.linear.x = 0;
    msg_zero.angular.z = 0;
    cmd_vel_rectified_pub_.publish(msg_zero);
}

void VelocityAndLightControl::CmdVelRectifiedPub(geometry_msgs::Twist msg_twist)
{
    if ( fabs(msg_twist.linear.x) > 0.01 || fabs(msg_twist.angular.z) > 0.01 )
    {
        robot_in_move_ = true;
        if ( fabs(msg_twist.angular.z) > 0.06 )
        {
            volume_ = 28;
        }
        else
        {
            volume_ = 20;
        }
    }
    else
    {
        robot_in_move_ = false;
    }

    cmd_vel_rectified_pub_.publish(msg_twist);
}

void VelocityAndLightControl::BatteryCallBack(const sensor_msgs::BatteryStatePtr &battery_msg)
{
    if ( battery_msg->percentage < 0.2 )
    {
        trigged_status_[BATTERY_LOW] = true;
    }
    else
    {
        trigged_status_[BATTERY_LOW] = false;
    }
}

bool VelocityAndLightControl::JoyFeedbackArrayPub(sensor_msgs::Joy& joy)
{
    bool is_timestamp = false;
    sensor_msgs::JoyFeedbackArray joy_feedback_array;
    for (int i = 0; i < JOYX_BUTTONS; i++)
    {
        if (joy_.buttons[i] != joy.buttons[i])
        {
            //cout << "joy_feedback_array_pub i: " << i << "  " << joy.buttons[i] << endl;
            sensor_msgs::JoyFeedback joy_feedback;
            joy_feedback.type = 1;
            joy_feedback.id = i;
            joy_feedback.intensity = joy.buttons[i];
            joy_feedback_array.array.push_back(joy_feedback);
        }
    }
    if (joy_feedback_array.array.size())
    {
        //cout << "joy_feedback_array_pub joy_feedback_array.array.size: " << joy_feedback_array.array.size() << endl;
        if (use_joy_feedback_)
        {
            for (int i = 0; i < joy_feedback_array.array.size(); i++)
            {
                joy_feedback_pub_.publish(joy_feedback_array.array[i]);
            }
        }
    }

    return is_timestamp;
}

void VelocityAndLightControl::CmdVelSub(const geometry_msgs::Twist::ConstPtr& msg)
{
    //if (joy_estop_laser_trigged_ && msg->linear.x < 0)//后雷达触发且后退
    if (joy_estop_laser_trigged_)//后雷达触发
    {
        CmdVelRectifiedPubStop();//停车
    }
    else
    {
        CmdVelRectifiedPub(*msg);
    }
}

void VelocityAndLightControl::LightControl( const sensor_msgs::Joy &joy )
{
    sensor_msgs::JoyFeedback joy_feedback;
    joy_feedback.type = 1; //type = 1 即 enum_intensity选择buttons
    joy_feedback.id = stick; //id = stick(10)，即enum_int_buttons选择stick
    //joy_feedback.intensity,是给id指向的那个地方赋值

    int intensity = 0;
    if (joy.buttons[estop] > 0) //急停或后雷达
    {
        intensity = 1; //车体黄灯

        trigged_status_[ESTOP] = true;
    }
    else
    {
        intensity = 0;

        trigged_status_[ESTOP] = false;
    }

    if ((joy.buttons[dump] & 0x04) || (joy.buttons[load] & 0x04)) //辊道1上料或下料
    {
        intensity = intensity | 0x04; //辊道1红灯
    }
    else
    {
        intensity = intensity | 0x02; //辊道1绿灯
    }
    if ((joy.buttons[dump] & 0x08) || (joy.buttons[load] & 0x08)) //辊道2上料或下料
    {
        intensity = intensity | 0x10;//0001 0000 辊道2红灯
    }
    else
    {
        intensity = intensity | 0x08;//0000 1000 辊道2绿灯
    }
    joy_feedback.intensity = intensity;
    joy_feedback_pub_.publish(joy_feedback);
}

void VelocityAndLightControl::JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    sensor_msgs::Joy joy;
    if (JOYD_AXES == (JOYENUM)msg->axes.size() && JOYD_BUTTONS == (JOYENUM)msg->buttons.size()) {
        joy.axes.resize(JOYX_AXES);
        joy.axes[JOYX_0X] = msg->axes[JOYD_0X];
        joy.axes[JOYX_0Y] = msg->axes[JOYD_0Y];
        joy.axes[JOYX_1X] = msg->axes[JOYD_1X];
        joy.axes[JOYX_1Y] = msg->axes[JOYD_1Y];
        joy.axes[JOYX_2X] = msg->axes[JOYD_2X];
        joy.axes[JOYX_2Y] = msg->axes[JOYD_2Y];
        joy.axes[JOYX_LT] = 1 - msg->buttons[JOYD_LT];
        joy.axes[JOYX_RT] = 1 - msg->buttons[JOYD_RT];
        joy.buttons.resize(JOYX_BUTTONS);
        joy.buttons[JOYX_A] = msg->buttons[JOYD_A];
        joy.buttons[JOYX_B] = msg->buttons[JOYD_B];
        joy.buttons[JOYX_X] = msg->buttons[JOYD_X];
        joy.buttons[JOYX_Y] = msg->buttons[JOYD_Y];
        joy.buttons[JOYX_LB] = msg->buttons[JOYD_LB];
        joy.buttons[JOYX_RB] = msg->buttons[JOYD_RB];
        joy.buttons[JOYX_BACK] = msg->buttons[JOYD_BACK];
        joy.buttons[JOYX_START] = msg->buttons[JOYD_START];
        joy.buttons[JOYX_LSTICK] = msg->buttons[JOYD_LSTICK];
        joy.buttons[JOYX_RSTICK] = msg->buttons[JOYD_RSTICK];
        joy.buttons[JOYX_LOGO] = 0;
    } else if (JOYX_AXES == (JOYENUM)msg->axes.size() && JOYX_BUTTONS == (JOYENUM)msg->buttons.size()) {
        joy.axes = msg->axes;
        joy.buttons = msg->buttons;
    } else {
        ROS_ERROR("unknown joy array size");
        return;
    }

    static bool get_different_buttons = false;
    for (int i = 0; i < JOYX_BUTTONS; i++)
    {
        if (joy_.buttons[i] != joy.buttons[i])
        {
            get_different_buttons = true;
            break;
        }
    }

    if ( get_different_buttons )
    {
        get_different_buttons = false;
        joy_estop_laser_trigged_ = joy.buttons[estop] & 0x04;//后雷达触发
        LightControl(joy);
    }

    if ( joy.buttons[modex] == 2 ) //遥控模式下
    {
        static bool get_joy_move = false;
        if ( joy.axes[linear_x] || joy.axes[angular_z] )//运动控制
        {
            get_joy_move = true;

            if ( !joy.buttons[estop] )
            {
                geometry_msgs::Twist msg_cmd_vel_rectified;
                msg_cmd_vel_rectified.linear.x = joy.axes[linear_x] * linear_x_factor_;
                msg_cmd_vel_rectified.angular.z = joy.axes[angular_z] * angular_z_factor_;
                // cout << "linear.x: " << msg_cmd_vel_rectified.linear.x << endl;
                // cout << "angular.z: " << msg_cmd_vel_rectified.angular.z << endl;
                CmdVelRectifiedPub(msg_cmd_vel_rectified);
            }
        }
        else
        {
            if ( get_joy_move )
            {
                get_joy_move = false;
                CmdVelRectifiedPubStop();
            }
        }
        if ( joy.buttons[dump] || joy.buttons[load] )//上下料控制
        {
            JoyFeedbackArrayPub(joy);
        }
    }

    joy_ = joy;

    SoundPlayerControl();
}

void VelocityAndLightControl::SoundPlayerControl()
{
    static SOUND sound_current = UNDEFINED;
    static SOUND sound_last = UNDEFINED;

    if ( robot_in_move_ )//运动时的优先级：ESTOP 前者没有时播放MOVE
    {
        if ( trigged_status_[ESTOP] )//也是有可能的，万一哪里出了问题呢
        {
            sound_current = ESTOP;
        }
        else
        {
            sound_current = MOVE;
        }
    }
    else//静止时的优先级：ESTOP BATTERY_LOW，前两者没有时停止播放STOP_PLAYER
    {
        if ( trigged_status_[BATTERY_LOW] )
        {
            sound_current = BATTERY_LOW;
        }
        else if ( trigged_status_[ESTOP] )
        {
            sound_current = ESTOP;
        }
        else
        {
            sound_current = STOP_PLAYER;
        }
    }

    if ( sound_current != sound_last )
    {
        sound_last = sound_current;
        SoundPlayerPub(sound_current);
    }
}

void VelocityAndLightControl::ShowHelp()
{
    puts("------------------------------");
    puts("--  joystick teleoperation  --");
    puts("------------------------------");
    puts("  [ enable ]      [ enable ]  ");
    puts("  [lift down]    [ lift up ]  ");
    puts("   _______          _______   ");
    puts("  /   _   \\________/       \\  ");
    puts(" |  _| |_  < >  < >  (help) | ");
    puts(" | |_only| < >  < >  (*)(#) | ");
    puts(" |   |_| ___      ___(stop) | ");
    puts(" |      /   \\    /   \\      | ");
    puts(" |      \\___/    \\___/      | ");
    puts(" |      z vel    y vel      | ");
    puts("  \\_______/        \\_______/  ");
    puts("------------------------------");
    puts("--  *  :  cancel, nav_ctrl  --");
    puts("--  #  :  shell/cmd_string  --");
    puts("------------------------------");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_and_light_control");
    VelocityAndLightControl velocity_and_light_control;

    ros::spin();

    return EXIT_SUCCESS;
}
