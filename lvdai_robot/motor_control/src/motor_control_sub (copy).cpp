#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/UInt8MultiArray.h"
#include <iostream>  
#include <geometry_msgs/Twist.h>

#define WheelD 0.3	//轮子半径
#define DELAY_TIME 0.01	//每次通讯等待时间
#define BaudRate 115200	//通讯波特率
#define get_mag 0		//未接受到数据时是否堵塞
#define add1 1		//未接受到数据时是否堵塞
#define add2 2		//未接受到数据时是否堵塞
using   namespace   std; 
serial::Serial ros_ser1;//串口1，左电机
serial::Serial ros_ser2;//串口2，右电机

class MotorControl
{   
public:
	unsigned char speedstr_l[8];
	unsigned char speedstr_r[8];
	unsigned short crc16;
    float leftVel;
    float rightVel;
	unsigned short rrpm;
	unsigned short lrpm;
    void cmd_velCallback(const geometry_msgs::Twist& cmdvel_);
	void sendmsg(unsigned char *s_buffer1,unsigned char *s_buffer2,unsigned char len);
	unsigned short getCRC16(unsigned char *ptr,unsigned char len);
 //   void SubscribeAndPublish();
    MotorControl()
    {
        leftVel=0.0;
        rightVel=0.0;
		speedstr_l[0]=add1;//驱动器1地址
		speedstr_l[1]=0x06;//功能码
		speedstr_l[2]=0x00;//
		speedstr_l[3]=0x6C;//设置成电机速度寄存器
		speedstr_r[0]=add2;//驱动器2地址
		speedstr_r[1]=0x06;//功能码
		speedstr_r[2]=0x00;//
		speedstr_r[3]=0x6C;//设置成电机速度寄存器
     //   pub_=n_.advertise<std_msgs::UInt8MultiArray>("PubSpeed",25);
        sub_=n_.subscribe("cmd_vel",1,&MotorControl::cmd_velCallback,this);
    }
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
};


void MotorControl::cmd_velCallback(const geometry_msgs::Twist& cmdvel_)
{
	//将中心点的速度转换成左右轮速度
	leftVel=cmdvel_.linear.x-cmdvel_.angular.z*2/5;
	rightVel=cmdvel_.linear.x+cmdvel_.angular.z*2/5;

	std::cout <<"leftvel"<< fixed <<(leftVel) << endl;
	std::cout <<"(leftVel*60/WheelD)"<< (leftVel*60/WheelD) << endl;
	
	lrpm=(unsigned short)(leftVel*60/WheelD);
	rrpm=(unsigned short)(rightVel*60/WheelD);
	std::cout << std::hex << (lrpm & 0xffff) << endl;

	speedstr_l[4]=((lrpm>>8)&0xff);//crc高
	speedstr_l[5]=lrpm&0xff;//crc低
	crc16=getCRC16(speedstr_l,6);
	speedstr_l[6]=((crc16>>8)&0xff);//crc高
	speedstr_l[7]=crc16&0xff;//crc低

	speedstr_r[4]=((rrpm>>8)&0xff);//crc高
	speedstr_r[5]=rrpm&0xff;//crc低
	crc16=getCRC16(speedstr_r,6);
	speedstr_r[6]=((crc16>>8)&0xff);//crc高
	speedstr_r[7]=crc16&0xff;//crc低

	sendmsg(speedstr_l,speedstr_r,8);

}

unsigned short MotorControl::getCRC16(unsigned char *ptr,unsigned char len)
{
	unsigned char i;
	unsigned short crc = 0xFFFF;
	if(len==0)
	{
		len = 1;
	}
	while(len--)
	{
		crc ^= *ptr;
		for(i=0; i<8; i++)
		{
			if(crc&1)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		ptr++;
		}
	}
	//  std::cout << std::hex << (crc & 0xffff) << endl;
	//  std::cout << std::hex << ((crc>>8) & 0xff )<< endl;
	return(crc);
}

void MotorControl::sendmsg(unsigned char *s_buffer1,unsigned char *s_buffer2,unsigned char len)
{
	unsigned char buffer1[256];
	unsigned char buffer2[256];
	size_t size1=0;
	size_t size2=0;
	int j=0;
	ros_ser1.write(s_buffer1,len);
	ros_ser2.write(s_buffer2,len);
	if(get_mag)
		while(size1==0||size2==0)
		{
			//Sleep(1);//延时1ms
			ros::Duration(DELAY_TIME).sleep();
			size1 = ros_ser1.available();
			size2 = ros_ser2.available();
			j++;
			printf("j = %d /n",j);
			if(j>10&&size1==0);
			{
				ros_ser1.write(s_buffer1,len);    //向串口发送指令
			}
			if(j>10&&size2==0);
			{
				ros_ser2.write(s_buffer2,len);    //向串口发送指令
			}
		}
	else
		ros::Duration(DELAY_TIME).sleep();

	ROS_INFO_STREAM("Reading from serial port");
	ros_ser1.read(buffer1,size1);
	ros_ser2.read(buffer2,size2);
	ROS_INFO("l  : %d",size1);
	ROS_INFO("r  : %d",size2);
	for(int i=0; i<size1; i++)
	{
		//16进制的方式打印到屏幕
		std::cout << std::hex << (buffer1[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
		for(int i=0; i<size2; i++)
	{
		//16进制的方式打印到屏幕
		std::cout << std::hex << (buffer2[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
}




unsigned short getCRC16(unsigned char *ptr,unsigned char len)
{
	unsigned char i;
	unsigned short crc = 0xFFFF;
	if(len==0)
	{
		len = 1;
	}
	while(len--)
	{
		crc ^= *ptr;
		for(i=0; i<8; i++)
		{
			if(crc&1)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		ptr++;
		}
	}
	//  std::cout << std::hex << (crc & 0xffff) << endl;
	//  std::cout << std::hex << ((crc>>8) & 0xff )<< endl;
	return(crc);
}

void sendmsg(unsigned char *s_buffer1,unsigned char *s_buffer2,unsigned char len)
{
	unsigned char buffer1[256];
	unsigned char buffer2[256];
	size_t size1=0;
	size_t size2=0;
	int j=0;
	ros_ser1.write(s_buffer1,len);
	s_buffer2[1]=add2;  //修改地址
	ros_ser2.write(s_buffer2,len);
	if(get_mag)
		while(size1==0||size2==0)
		{
			//Sleep(1);//延时1ms
			ros::Duration(DELAY_TIME).sleep();
			size1 = ros_ser1.available();
			size2 = ros_ser2.available();
			j++;
			printf("j = %d /n",j);
			if(j>10&&size1==0);
			{
				ros_ser1.write(s_buffer1,len);    //向串口发送指令
			}
			if(j>10&&size2==0);
			{
				ros_ser2.write(s_buffer2,len);    //向串口发送指令
			}
		}
	else
		ros::Duration(DELAY_TIME).sleep();

	ROS_INFO_STREAM("Reading from serial port");
	ros_ser1.read(buffer1,size1);
	ros_ser2.read(buffer2,size2);
	ROS_INFO("l  : %d",size1);
	ROS_INFO("r  : %d",size2);
	for(int i=0; i<size1; i++)
	{
		//16进制的方式打印到屏幕
		std::cout << std::hex << (buffer1[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
		for(int i=0; i<size2; i++)
	{
		//16进制的方式打印到屏幕
		std::cout << std::hex << (buffer2[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
}


void motor_control_init()
{
	unsigned char speedstr[8];
	unsigned short crc16;
//设置成网络操作模式
	printf("设置成网络操作模式");
	 speedstr[0]=add1;//驱动器地址
	 speedstr[1]=0x06;//功能码
	 speedstr[2]=0x00;//
	 speedstr[3]=0x33;//操作模式地址
	 speedstr[4]=0x00;//
	 speedstr[5]=0x00;//设置成网络模式
	 crc16=getCRC16(speedstr ,6);
	 speedstr[6]=((crc16>>8)&0xff);//crc高
	 speedstr[7]=crc16&0xff;//crc低
	 sendmsg(speedstr,speedstr,8);
//设置成速度控制模式
	printf("设置成速度控制模式");
 	 speedstr[2]=0x00;//
	 speedstr[3]=0x34;//操作模式地址
	 speedstr[4]=0x00;//
	 speedstr[5]=0x01;//设置成速度模式
	 crc16=getCRC16(speedstr ,6);
	 speedstr[6]=((crc16>>8)&0xff);//crc高
	 speedstr[7]=crc16&0xff;//crc低
	 sendmsg(speedstr,speedstr,8);
//设置电机电流
	printf("设置电机电流");
 	 speedstr[2]=0x00;//
	 speedstr[3]=0x2B;//操作模式地址
	 speedstr[4]=0x00;//
	 speedstr[5]=0x0A;//设置成速度模式
	 crc16=getCRC16(speedstr ,6);
	 speedstr[6]=((crc16>>8)&0xff);//crc高
	 speedstr[7]=crc16&0xff;//crc低
	 sendmsg(speedstr,speedstr,8);
//设置电机加速度
	printf("设置电机加速度");
 	 speedstr[2]=0x00;//
	 speedstr[3]=0x58;//操作模式地址
	 speedstr[4]=0x00;//
	 speedstr[5]=0x05;//设置成速度模式
	 crc16=getCRC16(speedstr ,6);
	 speedstr[6]=((crc16>>8)&0xff);//crc高
	 speedstr[7]=crc16&0xff;//crc低
	 sendmsg(speedstr,speedstr,8);
//设置电机减速度
	printf("设置电机减速度");
 	 speedstr[2]=0x00;//
	 speedstr[3]=0x59;//操作模式地址
	 speedstr[4]=0x00;//
	 speedstr[5]=0x05;//设置成速度模式
	 crc16=getCRC16(speedstr ,6);
	 speedstr[6]=((crc16>>8)&0xff);//crc高
	 speedstr[7]=crc16&0xff;//crc低
	 sendmsg(speedstr,speedstr,8);

//使能电机
	printf("使能电机");
 	 speedstr[2]=0x00;//
	 speedstr[3]=0x35;//操作模式地址
	 speedstr[4]=0x00;//
	 speedstr[5]=0x01;//设置成速度模式
	 crc16=getCRC16(speedstr ,6);
	 speedstr[6]=((crc16>>8)&0xff);//crc高
	 speedstr[7]=crc16&0xff;//crc低
	 sendmsg(speedstr,speedstr,8);
}


int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "motor_control_sub");
//	ros::NodeHandle n;
	ros::Time::init();

	try
     {
		 //串口1，左电机
         ros_ser1.setPort("/dev/ttyS0");
         ros_ser1.setBaudrate(BaudRate);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser1.setTimeout(to);
         ros_ser1.open();
		 //串口2，右电机
		 ros_ser2.setPort("/dev/ttyS1");
         ros_ser2.setBaudrate(BaudRate);
       //  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser2.setTimeout(to);
         ros_ser2.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }

     if(ros_ser1.isOpen())
     {
         ROS_INFO_STREAM("Serial_1 Port opened");
     }
     else
     {
         return -1;
     }
	  if(ros_ser2.isOpen())
     {
         ROS_INFO_STREAM("Serial_1 Port opened");
     }
     else
     {
         return -1;
     }
	motor_control_init();
	//   for(int i=0; i<8; i++)
    //         {  
    //             //16进制的方式打印到屏幕
    //             std::cout << std::hex << (speedstr[i] & 0xff) << " ";
    //             //std::cout << std::hex << (buffer[i] & 0xff) << " ";
    //         }
	// 		std::cout << endl;

    MotorControl motor_control;
   
    ros::spin();
	return 0;
}