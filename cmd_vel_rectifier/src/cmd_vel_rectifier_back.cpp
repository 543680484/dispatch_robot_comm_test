
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <sound_play/SoundRequest.h>
#include <std_msgs/String.h>

class cmd_vel_rectifier
{
public:
    cmd_vel_rectifier();

private:
    void init_param();

    void cmd_vel_sub(const geometry_msgs::Twist::ConstPtr& msg);
    void joy_sub(const sensor_msgs::Joy::ConstPtr& msg);
    void joy_feedback_info(sensor_msgs::Joy& joy);
    void joy_feedback_warn(sensor_msgs::Joy& joy);
    bool joy_feedback_array_pub(sensor_msgs::Joy& joy);
    void show_help();
    
private:
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher joy_feedback_pub_;
    ros::Publisher joy_feedback_array_pub_;
    ros::Publisher robotsound_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber joy_sub_;
    ros::Time joy_timestamp_;

    sensor_msgs::Joy joy_;

    bool joy_lt_trigged_;
    bool joy_rt_trigged_;
    bool joy_estop_trigged_;

    bool use_joy_feedback_;
    double linear_x_factor_;
    double linear_y_factor_;
    double linear_z_factor_;
    double angular_x_factor_;
    double angular_y_factor_;
    double angular_z_factor_;

    enum JOYENUM
    {
        JOYD_0Y      = 0,
        JOYD_0X,
        JOYD_1Y,
        JOYD_1X,
        JOYD_2Y,
        JOYD_2X,
        JOYD_AXES,
        JOYD_X       = 0,
        JOYD_A,
        JOYD_B,
        JOYD_Y,
        JOYD_LB,
        JOYD_RB,
        JOYD_LT,
        JOYD_RT,
        JOYD_BACK,
        JOYD_START,
        JOYD_LSTICK,
        JOYD_RSTICK,
        JOYD_BUTTONS,
        JOYX_0Y      = 0,
        JOYX_0X,
        JOYX_LT,
        JOYX_1Y,
        JOYX_1X,
        JOYX_RT,
        JOYX_2Y,
        JOYX_2X,
        JOYX_AXES,
        JOYX_A       = 0,
        JOYX_B,
        JOYX_X,
        JOYX_Y,
        JOYX_LB,
        JOYX_RB,
        JOYX_BACK,
        JOYX_START,
        JOYX_LOGO,
        JOYX_LSTICK,
        JOYX_RSTICK,
        JOYX_BUTTONS,
        linear_y     = 0,
        linear_x,
        joy_lt,
        angular_z,
        linear_z,
        joy_rt,
        angular_y,
        angular_x,
        enable       = 0,
        buzzer,
        dump,
        load,
        joy_lb,
        joy_rb,
        estop,
        start,
        modex,
        pedal,
        stick,
    };
};

cmd_vel_rectifier::cmd_vel_rectifier()
:ph_("~")
{
    init_param();

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_rectified", 100, true);
    if (use_joy_feedback_)
    {
        joy_feedback_pub_ =  nh_.advertise<sensor_msgs::JoyFeedback>("joy_feedback", 100, true);
    }
    joy_feedback_array_pub_ =  nh_.advertise<sensor_msgs::JoyFeedbackArray>("joy_feedback_array", 100, true);
    robotsound_pub_ = nh_.advertise<sound_play::SoundRequest>("robotsound", 100, true);
    cmd_vel_sub_ =  nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &cmd_vel_rectifier::cmd_vel_sub, this);
    joy_sub_ =  nh_.subscribe<sensor_msgs::Joy>("joy", 1000, &cmd_vel_rectifier::joy_sub, this);

    joy_timestamp_ = ros::Time::now();

    joy_.axes.resize(JOYX_AXES);
    joy_.buttons.resize(JOYX_BUTTONS);

    joy_lt_trigged_ = false;
    joy_rt_trigged_ = false;
    joy_estop_trigged_ = false;

    show_help();
}

void cmd_vel_rectifier::init_param()
{
    ph_.param("use_joy_feedback", use_joy_feedback_, false);

    double twist_factor;
    ph_.param("twist_factor", twist_factor, 1.0);
    double linear_factor;
    ph_.param("linear_factor", linear_factor, twist_factor);
    ph_.param("linear_x_factor", linear_x_factor_, linear_factor);
    ph_.param("linear_y_factor", linear_y_factor_, linear_factor);
    ph_.param("linear_z_factor", linear_z_factor_, linear_factor);
    double angular_factor;
    ph_.param("angular_factor", angular_factor, twist_factor);
    ph_.param("angular_x_factor", angular_x_factor_, angular_factor);
    ph_.param("angular_y_factor", angular_y_factor_, angular_factor);
    ph_.param("angular_z_factor", angular_z_factor_, angular_factor);
}

void cmd_vel_rectifier::joy_feedback_info(sensor_msgs::Joy& joy)
{
    joy.buttons[joy_lb] = (joy.axes[angular_z] > +0.2);
    joy.buttons[joy_rb] = (joy.axes[angular_z] < -0.2);
    joy.buttons[estop] = (joy.axes[linear_x] < 0);
}

void cmd_vel_rectifier::joy_feedback_warn(sensor_msgs::Joy& joy)
{
    joy.buttons[joy_lb] = 1;
    joy.buttons[joy_rb] = 1;
}

bool cmd_vel_rectifier::joy_feedback_array_pub(sensor_msgs::Joy& joy)
{
    bool is_timestamp = false;
    sensor_msgs::JoyFeedbackArray joy_feedback_array;
    for (int i = 0; i < JOYX_BUTTONS; i++)
    {
        if (joy_.buttons[i] != joy.buttons[i])
        {
            sensor_msgs::JoyFeedback joy_feedback;
            joy_feedback.type = 1;
            joy_feedback.id = i;
            joy_feedback.intensity = joy.buttons[i];
            joy_feedback_array.array.push_back(joy_feedback);
            if (joy_lb != i && joy_rb != i && estop != i)
            {
                is_timestamp = true;
            }

            if ((int)buzzer == i)
            {
                std::string arg = "sound.ogg";
                std::string path_bringup = getenv("PATH_BRINGUP");
                if ("" != path_bringup)
                {
                    arg = path_bringup + "/param/" + arg;
                }
                if (joy.buttons[i])
                {
                    sound_play::SoundRequest msg;
                    msg.sound = -2;
                    msg.command = 2;
//                    msg.volume = 1.0;
                    msg.arg = arg;
                    robotsound_pub_.publish(msg);
                }
                else
                {
                    sound_play::SoundRequest msg;
                    msg.sound = -2;
                    msg.command = 0;
//                    msg.volume = 1.0;
                    msg.arg = arg;
                    robotsound_pub_.publish(msg);
                }
            }
        }
        // if ((int)start != i && joy.buttons[i]) //TODO:???
        // {
        //     is_timestamp = true;
        // }
    }
    if (joy_feedback_array.array.size())
    {
        joy_feedback_array_pub_.publish(joy_feedback_array);
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

void cmd_vel_rectifier::cmd_vel_sub(const geometry_msgs::Twist::ConstPtr& msg)
{
    double timespan = (ros::Time::now() - joy_timestamp_).toSec();
    if (timespan > 2.0)
    {
        sensor_msgs::Joy joy = joy_;
        if (timespan > 5.0)
        {

            if (joy_estop_trigged_ && msg->linear.x < 0) //后雷达触发且后退
            {
                geometry_msgs::Twist msg_zero;
                cmd_vel_pub_.publish(msg_zero);//停车
                joy_feedback_warn(joy);//亮黄灯
            }
            else
            {
                joy.axes[linear_x] = msg->linear.x / linear_x_factor_;
                joy.axes[linear_y] = msg->linear.y / linear_y_factor_;
                joy.axes[linear_z] = msg->linear.z / linear_z_factor_;
                joy.axes[angular_x] = msg->angular.x / angular_x_factor_;
                joy.axes[angular_y] = msg->angular.y / angular_y_factor_;
                joy.axes[angular_z] = msg->angular.z / angular_z_factor_;
                cmd_vel_pub_.publish(* msg);
                joy_feedback_info(joy);
            }
        }
        else
        {
            joy_feedback_warn(joy);
        }
        joy_feedback_array_pub(joy);
    }
}

void cmd_vel_rectifier::joy_sub(const sensor_msgs::Joy::ConstPtr& msg)
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

    if (!joy_lt_trigged_)
    {
        if (joy.axes[joy_lt])
        {
            joy_lt_trigged_ = true;
        }
        else
        {
            joy.axes[joy_lt] += 1;
        }
    }
    if (!joy_rt_trigged_)
    {
        if (joy.axes[joy_rt])
        {
            joy_rt_trigged_ = true;
        }
        else
        {
            joy.axes[joy_rt] += 1;
        }
    }
    // joy_estop_trigged_ = joy.buttons[estop] & 0x04;
    joy_estop_trigged_ = 4 & 0x04;
    joy.buttons[joy_lb] = joy_.buttons[joy_lb];
    joy.buttons[joy_rb] = joy_.buttons[joy_rb];
    joy.buttons[estop] = joy_.buttons[estop];

    bool is_timestamp = false;
    if (fabs(joy.axes[joy_lt]) != 1.0 || fabs(joy.axes[joy_rt]) != 1.0 && !joy.buttons[pedal] && !joy.buttons[stick])
    {
        for (int i = 0; i < JOYX_AXES; i++)
        {
            if (joy_.axes[i] != joy.axes[i])
            {
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = joy.axes[linear_x] * linear_x_factor_;
                cmd_vel.linear.y = joy.axes[linear_y] * linear_y_factor_;
                cmd_vel.linear.z = joy.axes[linear_z] * linear_z_factor_;
                cmd_vel.angular.x = joy.axes[angular_x] * angular_x_factor_;
                cmd_vel.angular.y = joy.axes[angular_y] * angular_y_factor_;
                cmd_vel.angular.z = joy.axes[angular_z] * angular_z_factor_;
                cmd_vel_pub_.publish(cmd_vel);
                joy_feedback_info(joy);
                is_timestamp = true;
                break;
            }
        }
        if (joy.axes[linear_x] || joy.axes[linear_y] || joy.axes[linear_z] || joy.axes[angular_x] || joy.axes[angular_y] || joy.axes[angular_z])
        {
            is_timestamp = true;
        }
    }
    else
    {
        for (int i = 0; i < JOYX_AXES; i++)
        {
            if (joy_.axes[i] != joy.axes[i])
            {
                geometry_msgs::Twist cmd_vel;
                cmd_vel_pub_.publish(cmd_vel);
                joy_feedback_info(joy);
                is_timestamp = true;
                break;
            }
        }
    }

    // joy.buttons[start] = (!joy.buttons[estop] && !joy.buttons[start]);
    // if (joy.buttons[start])
    // {
    //     joy_feedback_warn(joy);
    // }
    if (joy_feedback_array_pub(joy) || is_timestamp)
    {
        joy_timestamp_ = ros::Time::now();
        // ROS_ERROR("haha0");
    }
    // if (joy.buttons[start])
    // {
    //     ros::spinOnce();
    //     sleep(1);
    //     int ret = system("sudo shutdown -h now");
    // }
    joy_ = joy;
}

void cmd_vel_rectifier::show_help()
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
    ros::init(argc, argv, "cmd_vel_rectifier");
    cmd_vel_rectifier cmd_vel_rectifier;

    ros::spin();
}
