#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/UInt8MultiArray.h"
#include <iostream>  
using   namespace   std; 

serial::Serial ros_ser;
//回调函数
//void callback(const std_msgs::String::ConstPtr& msg)
//{
//     ROS_INFO_STREAM("Write to serial port" << msg->data);
//     ros_ser.write(msg->data);
//}

int main (int argc, char** argv)
{
     ros::init(argc, argv, "my_serial_node");
     ros::NodeHandle n;
     //订阅主题command
     //ros::Subscriber command_sub = n.subscribe("command", 1000, callback);
     //发布主题sensor
     ros::Publisher sensor_pub = n.advertise<std_msgs::UInt8MultiArray>("sensor", 1000);
     uint8_t s_buffer[100];
     memset(s_buffer,0,sizeof(s_buffer));
     try
     {
         ros_ser.setPort("/dev/ttyS0");
         //ros_ser.setBaudrate(9600);
         ros_ser.setBaudrate(115200);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser.setTimeout(to);
         ros_ser.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }


     if(ros_ser.isOpen())
     {
         ROS_INFO_STREAM("Serial Port opened");
     }
     else
     {
         return -1;
     }

     ros::Rate loop_rate(2);
      int j=0;
      size_t size=0;
        
     while(ros::ok())
     {
       size=0;
        j=0;
        uint8_t buffer[1024];
        memset(buffer,0,sizeof(buffer));
        memset(s_buffer,0,sizeof(s_buffer));
        
      //  ros::spinOnce();
        s_buffer[0] = 0x77;
        s_buffer[1] = 0x04;
        s_buffer[2] = 0x00;
        s_buffer[3] = 0x59;
        s_buffer[4] = 0x5D;
        ros_ser.write(s_buffer,5);
       // ros_ser.write(s_buffer,5);    //向串口发送指令
        // for(int i=0;i<65535;i++)
        // {
        //      size = ros_ser.available();
             
        // }
        while(size==0)
        {
             size = ros_ser.available();
            //  Sleep(1);//延时1ms
             ros::Duration(0.002).sleep();
             j++;
             printf("j = %d",j);
             if(j>10);
             
             {
                  ros_ser.write(s_buffer,5);    //向串口发送指令
                //  ros_ser.write(s_buffer,5);    //向串口发送指令
                  j=0;
             }
        }
       
        size = ros_ser.available();
        ROS_INFO("1111111111111111111111111111111111111111");
         if(size)
         {
             ROS_INFO_STREAM("Reading from serial port");
             std_msgs::UInt8MultiArray serial_data;
             //获取串口数据
             serial_data.data.resize(1024);
            // serial_data.data = ros_ser.read(ros_ser.available()){};
             ros_ser.read(buffer,size);
            ROS_INFO("%d",size);

             for(int i=0; i<48; i++)
            {
                serial_data.data[i] = buffer[i];
                //16进制的方式打印到屏幕
                std::cout << std::hex << (serial_data.data[i] & 0xff) << " ";
                //std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
             //ROS_INFO_STREAM("Read: " << serial_data.data);
             //将串口数据发布到主题sensor
            
             sensor_pub.publish(serial_data);
            for(int i=0; i<size; i++)
            {
                serial_data.data[i] = 0;
            }
             memset(s_buffer,0,sizeof(s_buffer));
             memset(buffer,0,sizeof(buffer));
         }
        //  if(size>48)
        //     {
        //         j++;
        //         ROS_INFO("j= %d",j);

        //     }
        // ros::spinOnce();
          loop_rate.sleep();
     }
 }
