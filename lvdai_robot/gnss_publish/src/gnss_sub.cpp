// Step 1:  Include Library Headers:
//发布gnss的数据到gnss_data
#include <ros/ros.h> 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <math.h> 
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "gnss_publish/gnss_gphcd.h"
#include <serial/serial.h>
#include "std_msgs/Header.h"
#include <nmea_msgs/Sentence.h>
#include <tf/transform_broadcaster.h>
#include<iostream>


#define DELAY_TIME 0.001	//每次通讯等待时间
#define LOOP_RATE 200		//每秒调用200次回调函数
#define GPS_PUB_RART 10	//信息测量频率
#define RESENT_TIMES 20	//多少次没有接受到就重发
#define BaudRate 115200	//通讯波特率
#define ERROR_TIME_MOST 15	//测试用，记录等待时间超过本时间的次数
#define get_mag 1		//未接受到数据时是否堵塞
using   namespace   std; 
const int QueueSize=500;
serial::Serial ros_ser4;//串口4，GNSS
int error_time_most = 0;
int real_resent_time = 0;

 static void GNSSCallback(const nmea_msgs::Sentence &msg)
{
    cout<<"收到"<<endl;
    cout<<"msg.header.frame_id:"<<msg.header.frame_id<<endl;
    cout<<"msg.header.stamp:"<<msg.header.stamp<<endl;
    cout<<"msg.header.seq:"<<msg.header.seq<<endl;
    cout<<"sentence:"<<msg.sentence<<endl;
}

int main(int argc, char** argv)
{
    // Step 2: Initialization:
    ros::init(argc, argv, "gnss_sub");     
    ros::NodeHandle n("~");
    ros::Subscriber client_send = n.subscribe("/nmea_sentence", 10, GNSSCallback);


    ros::Time::init();
    ros::Rate loop_rate(GPS_PUB_RART); 

    ros::spin();

    return 0;
}




