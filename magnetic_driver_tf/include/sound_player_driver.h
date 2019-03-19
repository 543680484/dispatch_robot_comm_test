#ifndef SOUND_PLAYER_DRIVER
#define SOUND_PLAYER_DRIVER

#include <ros/ros.h>
#include <sound_play/SoundRequest.h>
#include <serial.h>

using namespace std;
class SoundPlayerDriver
{
public:
    SoundPlayerDriver();
    ~SoundPlayerDriver();

    void Run();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sound_player_sub_;

    string serial_dev_;
    int serial_fd_;
    int length_send_recv_;
    bool get_new_sound_;

    unsigned char buf_send_[7];

    void SerialClose();
    void SoundCallback( const sound_play::SoundRequestConstPtr &sound_msg );
    unsigned char GetCheckCodeXor(unsigned char buf_data[], unsigned int xor_length);

    void SoundCmdSend();
    void SoundCmdReturn();
};

#endif
