/*喇叭音频：
1.注意安全请避让
2.设备故障请处理
3.当前电量低请充电
4.电量过低，停止运行
5.超速运行注意安全
6.开始充电
7.结束充电
8.设备对接中*/

#ifndef VELOCITY_AND_LIGHT_CONTROL
#define VELOCITY_AND_LIGHT_CONTROL

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <sound_play/SoundRequest.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>

using namespace std;

class VelocityAndLightControl
{
public:
    VelocityAndLightControl();

private:
    void InitParam();
    void ShowHelp();
    void CmdVelSub(const geometry_msgs::Twist::ConstPtr& msg);
    void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    bool JoyFeedbackArrayPub(sensor_msgs::Joy& joy);
    void SoundPlayerPub(int sound);
    void BatteryCallBack(const sensor_msgs::BatteryStatePtr &battery_msg);
    void CmdVelRectifiedPubStop();
    void CmdVelRectifiedPub(geometry_msgs::Twist msg_twist);
    void SoundPlayerControl();
    void LightControl(const sensor_msgs::Joy &joy);

private:
    ros::NodeHandle ph_;
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_rectified_pub_;
    ros::Publisher joy_feedback_pub_;
    ros::Publisher robotsound_pub_;
    ros::Publisher sound_player_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber battery_sub_;
    sensor_msgs::Joy joy_;

    bool robot_in_move_;
    bool joy_estop_laser_trigged_;

    bool use_joy_feedback_;
    double linear_x_factor_;
    double angular_z_factor_;

    int volume_;

    vector<bool> trigged_status_;

    enum SOUND
    {
        STOP_PLAYER = 0,  //0.停止播放
        MOVE,             //1.注意安全请避让
        ESTOP,            //2.设备故障请处理
        BATTERY_LOW,      //3.当前电量低请充电
        UNDEFINED         //4.未定义状态
    };

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
        linear_y     = 0,  //axes
        linear_x,          //无线遥控器：前进1.0，后退-1.0
        joy_lt,//enable
        angular_z,         //无线遥控器：左转1.0，右转-1.0
        linear_z,
        joy_rt,//enable
        angular_y,
        angular_x,
        enable       = 0, //buttons
        buzzer,
        dump,//lift_down  //无线遥控器：辊道1,2下料(1,2) //中间继电器：辊道1,2下料
        load,//lift_up    //无线遥控器：辊道1,2上料(1,2) //中间继电器：辊道1,2上料
        joy_lb,           //辊道限位传感器：辊道1,2下料到位
        joy_rb,           //辊道限位传感器：辊道1,2上料到位
        estop,            //急停，安全触边，后激光雷达
        start,
        modex,            //模式切换：手推，遥控
        pedal,            //叉车踏板
        stick,            //(车体黄色指示灯亮，辊道1绿1红2绿2红)
    };
};

#endif
