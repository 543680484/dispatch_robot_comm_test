#include <ros/ros.h>

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
#include <errno.h>

#include <json/json.h>

#define MAXLINE 1024
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_sdk_server");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.2);
    char buff[4096];

    int  listenfd, connfd;
    struct sockaddr_in  servaddr;

    if( (listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 ){
        printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
        return 0;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(10001);

    if( bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1){
        printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);
        return 0;
    }

    if( listen(listenfd, 1) == -1){
        printf("listen socket error: %s(errno: %d)\n",strerror(errno),errno);
        return 0;
    }

    int comm_id = 0;

    while(ros::ok())
    {
        static bool connect_state = false;

        while ( !connect_state )
        {
            printf("======waiting for client's request======\n");
            connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
            if( connfd == -1)
            {
                printf("accept socket error: %s(errno: %d)",strerror(errno),errno);
                sleep(1);
                connect_state = false;
            }
            else
            {
                cout << "connected !!!" << endl;
                connect_state = true;
            }
        }
//        n = recv(connfd, buff, MAXLINE, 0);

        cout << "---------------------------" << endl;
        cout << "please input c for continue" << endl;
        cout << "please input q for quit" << endl;
        cout << "---------------------------" << endl;

        std::string operate;
        cin >> operate;

        if ( operate == "c" )
        {
            std::string place;
            cout << "please input place, then enter" << endl;
            cin >> place;
            std::string distance;
            cout << "please input distance, then enter" << endl;
            cin >> distance;

            struct timeval time_value;
            gettimeofday(&time_value,NULL);

            Json::Value DeviceInfor;
            DeviceInfor["Dev_Type"] = "Core";
            DeviceInfor["Dev_ID"] = 1;

            Json::Value AGVInfor;
            AGVInfor["TimeStamp_sec"] = static_cast<long long int>(time_value.tv_sec);
            AGVInfor["TimeStamp_usec"] = static_cast<int>(time_value.tv_usec);

            ++comm_id;
            AGVInfor["AGV_ID"] = 2;
            AGVInfor["Comm_ID"] = comm_id;
            AGVInfor["Comm_Type"] = "PROC";

            Json::Value AGVProc;
            AGVProc["Proc_Name"] = "TEST";//"MAGNETICTRACK_LOCATION";
            AGVProc["Proc_Para_1"] = place;
            AGVProc["Proc_Para_2"] = distance;
            AGVInfor["PROC"] = AGVProc;

            DeviceInfor["Dev_Data"] = AGVInfor;

            std::string msg_agv_to_dispatch = DeviceInfor.toStyledString();
            msg_agv_to_dispatch += 0x0d;
            msg_agv_to_dispatch += 0x0a;

            int write_length = write(connfd, msg_agv_to_dispatch.c_str(), msg_agv_to_dispatch.size());

            if (write_length < 0)
            {
                ROS_ERROR("server write to client ERROR");
                connect_state = false;
                close(connfd);
                close(listenfd);
                ros::Duration ( 1.0 ).sleep();
                continue;
            }
        }
        else if ( operate == "q" )
        {
            close(connfd);
            close(listenfd);
            sleep(1);
            break;
        }

//        int n = 200;
//        while ( n > 100 )
//        {
//            cout << "read: " << n << endl;
//            n = recv(connfd, buff, 2048, 0);
//    //      buff[n] = '\0';
//    //      printf("recv msg from client: %s\n", buff);
//    //      close(connfd);
//        }


        loop_rate.sleep();
    }
    close(connfd);
    close(listenfd);
    return 0;
}
