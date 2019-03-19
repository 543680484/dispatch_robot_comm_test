#include <sound_player_driver.h>

SoundPlayerDriver::SoundPlayerDriver()
    :nh_("~")
    ,serial_dev_()
    ,serial_fd_()
    ,sound_player_sub_()
    ,get_new_sound_(false)
    ,length_send_recv_(7)
    ,buf_send_{0x01,0x51,0x00,0x00,0x00,0x00,0x02}//首码 功能码 音乐 备用 音量 异或值 结束码
{
    nh_.param<string>( "serial_dev", serial_dev_, "/dev/ttyUSB0" );

    sound_player_sub_ = nh_.subscribe("/robotsound", 1, &SoundPlayerDriver::SoundCallback, this );
}

SoundPlayerDriver::~SoundPlayerDriver()
{
}

void SoundPlayerDriver::SoundCallback( const sound_play::SoundRequestConstPtr &sound_msg )
{
    get_new_sound_ = true;
    buf_send_[2] = sound_msg->sound; //音乐
    buf_send_[4] = sound_msg->command; //音量
    buf_send_[5] = GetCheckCodeXor(buf_send_, length_send_recv_-2); //异或值
}

unsigned char SoundPlayerDriver::GetCheckCodeXor(unsigned char buf_data[], unsigned int xor_length)
{
    unsigned char check_code = 0;
    for ( int i = 0; i < xor_length; ++i )
    {
        check_code = check_code ^ buf_data[i];
    }
    return check_code;
}

void SoundPlayerDriver::SerialClose()
{
    close(serial_fd_);
}

void SoundPlayerDriver::SoundCmdSend()
{
    // for (int i=0; i<length_send_recv_; ++i)
    // {
    //     printf("send %d, %x\n", i, buf_send_[i]);
    // }
    int length_real_send = write(serial_fd_, buf_send_, length_send_recv_);
    if ( length_real_send != length_send_recv_ )
    {
        ROS_ERROR("write data failed");
        tcflush(serial_fd_, TCOFLUSH);
        return;
    }
}

void SoundPlayerDriver::SoundCmdReturn()
{
    unsigned char buf_recv[length_send_recv_];
    memset(buf_recv, 0, sizeof(buf_recv));

    int length_real_read = read(serial_fd_, buf_recv, 2*length_send_recv_);

    // for (int i=0; i<length_real_read; ++i)
    // {
    //     printf("recv %d, %x\n", i, buf_recv[i]);
    // }

    if( length_real_read == length_send_recv_ )
    {
        unsigned int check_result = GetCheckCodeXor(buf_recv, length_send_recv_-2);

        if ( buf_recv[length_send_recv_-2] != check_result )
        {
            ROS_WARN("get a wrong magnetic check code");
        }
    }
    else
    {
        ROS_ERROR("%s read data failed, %d", serial_dev_.c_str() ,length_real_read);
        tcflush(serial_fd_, TCOFLUSH);
    }
}

void SoundPlayerDriver::Run()
{
    serial_fd_ = SerialOpen(serial_dev_, 9600, 8, 'N', 1, length_send_recv_);
    while ( serial_fd_ <= 0 && ros::ok() )
    {
        SerialClose();
        ros::Duration(1.0).sleep();
        serial_fd_ = SerialOpen(serial_dev_, 9600, 8, 'N', 1, length_send_recv_);
    }

    ros::Rate r(2);
    while (ros::ok())
    {
        ros::spinOnce();

        if ( get_new_sound_ )
        {
            get_new_sound_ = false;

            SoundCmdSend();

            ros::Duration(0.4).sleep();

            SoundCmdReturn();
        }

        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sound_player_driver");
    SoundPlayerDriver sound_player_driver;
    sound_player_driver.Run();
    return EXIT_SUCCESS;
}
