#include <std_msgs/Int32MultiArray.h>
#include <bitset>

#include <serial.h>
#include <magnetic_driver_hinson.h>

/*8检测点接收数据：标识符（5Byte）+ 数据（1Byte） + 校验（2Byte） = 8Byte  /52 4D 67 73 77 + 数据（1Byte）  + 校验（2Byte）
  16检测点接收数据：标识符（5Byte）+ 数据（3Byte） + 校验（2Byte） = 10Byte /52 4D 67 73 77 + 数据（3Byte）  + 校验（2Byte）
*/

MagneticDriverHinson::MagneticDriverHinson()
    :nh_("~")
    ,task_switch_sub_()
    ,magnetic_pub_()
    ,serial_dev_()
    ,serial_fd_()
    ,magnetic_point_()
    ,run_task_(false)
    ,length_read_magnetic_(7)
    ,buf_send_point_io_{0x52,0x4D,0x67,0x73,0x77,0x00,0x00} //8位和16位相同，发送数据：标识符（5Byte）+ 校验（2Byte） = 7Byte  /52 4D 67 73 77 /5E(低位) 69(高位)
{
    string pub_topic_name;
    nh_.param<string>( "pub_topic_name", pub_topic_name, "/magnetic_head_data" );

    nh_.param<string>( "serial_dev", serial_dev_, "/dev/ttyUSB0" );
    nh_.param<int>( "magnetic_point", magnetic_point_, 16 );

    task_switch_sub_ = nh_.subscribe("/task_switch", 1, &MagneticDriverHinson::TaskSwitchCallback, this );
    magnetic_pub_ = nh_.advertise<std_msgs::Int32MultiArray>(pub_topic_name,1);
}

MagneticDriverHinson::~MagneticDriverHinson()
{
}

void MagneticDriverHinson::InitBufSend()
{
    unsigned int check_result = GetCrcCheck(buf_send_point_io_, length_read_magnetic_-2);

    buf_send_point_io_[length_read_magnetic_-2] = check_result & 0xFF;
    buf_send_point_io_[length_read_magnetic_-1] = (check_result>>8) & 0xFF;
}

void MagneticDriverHinson::TaskSwitchCallback(const std_msgs::HeaderPtr &task_switch_msg)
{
    size_t found_node = task_switch_msg->frame_id.find("magnetic_driver_hinson");
    if ( found_node != std::string::npos || task_switch_msg->frame_id.empty() )
    {
        if ( task_switch_msg->seq == 0 )
        {
            run_task_ = false;
        }
        else
        {
            run_task_ = true;
        }
    }
}
   
void MagneticDriverHinson::SerialClose()
{
    close(serial_fd_);
}

unsigned int MagneticDriverHinson::GetCrcCheck(unsigned char *buffer, unsigned int buffer_length)
{
    unsigned int i, j;
    unsigned int wCrc = 0xffff;
    unsigned int wPolynom = 0xA001;
    for (i = 0; i < buffer_length; i++)
    {
        wCrc ^= buffer[i];
        for (j = 0; j < 8; j++)
        {
            if (wCrc &0x0001)
            {
                wCrc = (wCrc >> 1) ^ wPolynom;
            }
            else
            {
                wCrc = wCrc >> 1;
            }
        }
    }
    return wCrc;
}

void MagneticDriverHinson::ReadMagneticSend()
{
//    for (int i=0; i<length_read_magnetic_; ++i)
//    {
//        printf("send %d, %x\n", i, buf_send_point_io_[i]);
//    }
    int length_real_send = write(serial_fd_, buf_send_point_io_, length_read_magnetic_);
    if ( length_real_send != length_read_magnetic_ )
    {
        ROS_ERROR("write data failed");
        tcflush(serial_fd_, TCOFLUSH);
        return;
    }
}

void MagneticDriverHinson::ReadMagneticReturn(int length_read)
{
    unsigned char buf_recv[length_read];
    memset(buf_recv, 0, sizeof(buf_recv));

    std_msgs::Int32MultiArray magnetic_data_msg;//存放磁条检测结果

    int length_real_read = read(serial_fd_, buf_recv, 2*length_read);
//    for (int i=0; i<length_read_magnetic8_io_reply_; ++i)
//    {
//        printf("recv %d, %x\n", i, buf_recv[i]);
//    }

    if( length_real_read == length_read )
    {
        //计算校验码
        unsigned int check_result = GetCrcCheck(buf_recv, length_read-2);
        unsigned char check_result_h = check_result & 0xFF;
        unsigned char check_result_l = (check_result>>8) & 0xFF;

        if ( buf_recv[length_read-2] == check_result_h && buf_recv[length_read-1] == check_result_l )
        {
            int result_byte = length_read - 5 - 2; //5符号，2校验

            if ( result_byte == 1 ) //8个点
            {
                std::bitset<8> bit_0_7 = buf_recv[5];
                for ( int i = 0; i < 8; ++i )
                {
                    magnetic_data_msg.data.push_back(bit_0_7[i]);
                }
            }
            else //16个点
            {
                std::bitset<8> bit_0_7 = buf_recv[5];//buf_recv[5]第1~7位有效,第0位无效
                std::bitset<8> bit_8_15 = buf_recv[6];//buf_recv[6]第0~7位有效
                std::bitset<8> bit_16 = buf_recv[7];//buf_recv[7]第0位有效

                for ( int i = 1; i < 17; ++i ) //因为buf_recv[5]第0位无效，i从1开始
                {
                    if ( i < 8 )
                    {
                        magnetic_data_msg.data.push_back(bit_0_7[i]);
                    }
                    else if ( i < 16 )
                    {
                        magnetic_data_msg.data.push_back(bit_8_15[i-8]);
                    }
                    else
                    {
                        magnetic_data_msg.data.push_back(bit_16[0]);
                    }
                }
            }

            magnetic_pub_.publish(magnetic_data_msg);
        }
        else
        {
            ROS_WARN("get a wrong magnetic crc code");
        }
    }
    else
    {
        ROS_ERROR("%s read data failed, %d", serial_dev_.c_str() ,length_real_read);
        tcflush(serial_fd_, TCOFLUSH);
    }
}

void MagneticDriverHinson::Run()
{
    InitBufSend();

    int length_read = 0;

    if ( magnetic_point_ == 8 )
    {
        length_read = 8;
    }
    else if ( magnetic_point_ == 16 )
    {
        length_read = 10;
    }
    else
    {
        ROS_ERROR("undefied magnetic type :%d", magnetic_point_);
        SerialClose();
        return;
    }

    serial_fd_ = SerialOpen(serial_dev_, 115200, 8, 'N', 1, length_read);
    while ( serial_fd_ <= 0 && ros::ok() )
    {
        SerialClose();
        ros::Duration(1.0).sleep();
        serial_fd_ = SerialOpen(serial_dev_, 115200, 8, 'N', 1, length_read);
    }

    ros::Rate r(100);
    while (ros::ok())
    {
        ros::spinOnce();
        if ( !run_task_ )
        {
            r.sleep();
            continue;
        }

        ReadMagneticSend();

        ros::Duration(0.002).sleep();

        ReadMagneticReturn(length_read);

        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "magnetic_driver_hinson");
    MagneticDriverHinson magnetic_driver_hinson;
    magnetic_driver_hinson.Run();
    return EXIT_SUCCESS;
}
