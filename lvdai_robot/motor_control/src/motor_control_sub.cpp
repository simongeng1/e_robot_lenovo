#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include <iostream>  
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


#define WheelD 0.05	//轮子半径
#define DELAY_TIME 0.001	//每次通讯等待时间
#define LOOP_RATE 200		//每秒调用回调函数次数
#define SPEED_PUB_RART 5	//速度信息测量频率
#define RESENT_TIMES 5		//多少次没有接受到就重发
#define BaudRate 115200	//通讯波特率
#define get_mag 1		//未接受到数据时是否堵塞
#define add1 1		//左电机地址
#define add2 2		//右电机地址
#define width 1		//车身宽度



using   namespace   std; 
serial::Serial ros_ser1;//串口1，左电机
serial::Serial ros_ser2;//串口2，右电机


//////////////////////////////////////////////////////////////////
// 速度获取类
/////////////////////////////////////////////////////////////////
class GetSpeed
{
public:
	unsigned char speedstr_l[8];
	unsigned char speedstr_r[8];
	double speed_l_real;
	double speed_r_real;
//	nav_msgs::Odometry odom;
	tf::TransformBroadcaster odom_broadcaster;
	double x;
	double y ;
	double th ;
	double vx ;
 	double vy ;
 	double vth;

	ros::Time current_time, last_time;
	unsigned short crc16;
    float leftVel;
    float rightVel;
	unsigned short getCRC16(unsigned char *ptr,unsigned char len);
	void send_get_speed();
	GetSpeed()
    {
		x = 0.0;
		y = 0.0;
		th = 0.0;
		vx = 0;
 		vy = 0;
 		vth = 0;
        leftVel=0.0;
        rightVel=0.0;
		speedstr_l[0]=add1;//驱动器1地址
		speedstr_l[1]=0x03;//功能码
		speedstr_l[2]=0x00;//
		speedstr_l[3]=0xAF;//设置成电机速度寄存器
		speedstr_l[4]=0x00;//
		speedstr_l[5]=0x01;//寄存器数量
		crc16=getCRC16(speedstr_l ,6);
		speedstr_l[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
		speedstr_l[6]=crc16&0xff;//crc高

		speedstr_r[0]=add2;//驱动器2地址
		speedstr_r[1]=0x03;//功能码
		speedstr_r[2]=0x00;//
		speedstr_r[3]=0xAF;//设置成电机速度寄存器
		speedstr_r[4]=0x00;//
		speedstr_r[5]=0x01;//寄存器数量
		crc16=getCRC16(speedstr_r ,6);
		speedstr_r[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
		speedstr_r[6]=crc16&0xff;//crc高
        pub_=n2_.advertise<nav_msgs::Odometry>("odom", 50);
    }
private:
  ros::NodeHandle n2_; 
  ros::Publisher pub_;
  //ros::Subscriber sub_;
};


unsigned short GetSpeed::getCRC16(unsigned char *ptr,unsigned char len)
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
		}
		ptr++;
	}
	//  std::cout << std::hex << (crc & 0xffff) << endl;
	//  std::cout << std::hex << ((crc>>8) & 0xff )<< endl;
	return(crc);
}
void GetSpeed::send_get_speed()
{
	unsigned char buffer1[256];
	unsigned char buffer2[256];
	unsigned short crcresult1;
	unsigned short crcresult2;
	size_t size1=0;
	size_t size2=0;
	int j=0;
	ros_ser1.write(speedstr_l,8);
	ros_ser2.write(speedstr_r,8);
	if(get_mag)
	{
		while(size1==0||size2==0)
		{
			
			//Sleep(1);//延时1ms
			ros::Duration(DELAY_TIME).sleep();
			size1 = ros_ser1.available();
			size2 = ros_ser2.available();
			j++; 
			printf("j = %d \n",j);
			if(size1>0)
				std::cout <<  "left is ok " << endl;
			if(size2>0)
				std::cout <<  "right is ok " << endl;
			if((j%RESENT_TIMES)==0&&size1==0)
			{	std::cout << "left" << endl;
				ros_ser1.write(speedstr_l,8);    //向串口发送指令
				
			}
			if((j%RESENT_TIMES)==0&&size2==0)
			{
				std::cout <<  "right" << endl;
				ros_ser2.write(speedstr_r,8);    //向串口发送指令
			}
			if(j >(RESENT_TIMES*10+1))
			{
				ROS_INFO("Speed_Get send error");
				return;
			}
		}
	}
	else
		ros::Duration(DELAY_TIME).sleep();
	last_time = current_time;
	current_time = ros::Time::now();	//记录接受到消息时的时间

	ROS_INFO_STREAM("GETSPEED Reading from serial port");
	ros_ser1.read(buffer1,size1);
	ros_ser2.read(buffer2,size2);
//	ROS_INFO("l  : %d",size1);
//	ROS_INFO("r  : %d",size2);
	std::cout <<"left_real_speed:";
	//	return;
	for(int i=0; i<size2||i<8; i++)
	{
		
		//16进制的方式打印到屏幕
		std::cout <<std::hex << (buffer1[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
	std::cout <<"right_real_speed:";
		for(int i=0; i<size2||i<8; i++)
	{
		//16进制的方式打印到屏幕
		
		std::cout << std::hex << (buffer2[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
	std::cout << endl;
	if(size1 == 7&&size2 == 7)
	{
		crcresult1 = getCRC16(buffer1,5);
		crcresult2 = getCRC16(buffer2,5);
	//std::cout << std::hex << (crcresult1&0xff) << "GAO"<<((crcresult1>>8) & 0xff) << " "<<endl;
	//	std::cout <<"speed_l_1111111111111real = "<< std::hex << (speed_l_real & 0xffff) << " "<<"speed_r_real = "<<(speed_r_real & 0xffff)<<endl;
		if((buffer1[5] == (crcresult1&0xff))&&(buffer1[6] == (crcresult1>>8)&0xff)&&
		   (buffer2[5] == (crcresult2&0xff))&&(buffer2[6] == (crcresult2>>8)&0xff))
		{
			speed_l_real=(double)(buffer1[3]*256+buffer1[4]);
			speed_r_real=(double)(buffer2[3]*256+buffer2[4]);
			speed_l_real= speed_l_real*WheelD/60;
			speed_r_real= speed_r_real*WheelD/60;
			std::cout <<"speed_l_real = "<< std::hex <<  speed_l_real <<"   speed_r_real = "<<speed_r_real<<endl;
			vx = (speed_l_real+speed_r_real)/2;
			vth = atan((speed_r_real-speed_l_real)/width);
			std::cout <<"VX = "<<  vx <<"   VY = "<<vy<<"   Vth"<<vth<<endl;
			double dt = (current_time - last_time).toSec();
			double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
			double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
			double delta_th = vth * dt;
			x += delta_x;
			y += delta_y;
			th += delta_th;
			//since all odometry is 6DOF we'll need a quaternion created from yaw
			//为了兼容二维和三维的功能包，让消息结构更加通用，里程计的偏航角需要转换成四元数才能发布
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "odom";

			//set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;
			std::cout <<"X = "<<  x <<"   Y = "<<y<<"   th"<<th<<endl;
			//set the velocity
			odom.child_frame_id = "base_link";
			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.angular.z = vth;

			// std::cout <<"speed_l_real = "<< std::hex << (speed_l_real & 0xffff) << " "<<"speed_r_real = "<<(speed_r_real & 0xffff)<<endl;
			// speed.data[0] =(speed_l_real&0xffff);
			// speed.data[0] =(speed_r_real&0xffff);
			pub_.publish(odom);
		}
		else
			return;
	}
	else
		return;

}



//////////////////////////////////////////////////////////////////
// 电机控制类
/////////////////////////////////////////////////////////////////

class MotorControl
{   
public:
	unsigned char speedstr_l[8];
	unsigned char speedstr_r[8];
	unsigned char speed_l_real[4];
	unsigned char speed_r_real[4];
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
		speedstr_l[3]=0xAE;//设置成电机速度寄存器
		speedstr_r[0]=add2;//驱动器2地址
		speedstr_r[1]=0x06;//功能码
		speedstr_r[2]=0x00;//
		speedstr_r[3]=0xAE;//设置成电机速度寄存器
     //   pub_=n_.advertise<std_msgs::UInt16MultiArray>("PubSpeed",25);
        sub_=n_.subscribe("cmd_vel",1,&MotorControl::cmd_velCallback,this);
    }
	 ~MotorControl()
	 {
	 printf("关闭电机");
 	 speedstr_l[2]=0x00;//
	 speedstr_l[3]=0x28;//操作模式地址
	 speedstr_l[4]=0x00;//
	 speedstr_l[5]=0x00;//设置成关闭电机模式
	 crc16=getCRC16(speedstr_l ,6);
	 speedstr_l[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	 speedstr_l[6]=crc16&0xff;//crc高
	 speedstr_r[2]=0x00;//
	 speedstr_r[3]=0x28;//操作模式地址
	 speedstr_r[4]=0x00;//
	 speedstr_r[5]=0x00;//设置成关闭电机模式
	 crc16=getCRC16(speedstr_r ,6);
	 speedstr_r[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	 speedstr_r[6]=crc16&0xff;//crc高
	 sendmsg(speedstr_l,speedstr_r,8);

	 }
private:
  ros::NodeHandle n_; 
 // ros::Publisher pub_;
  ros::Subscriber sub_;
};



void MotorControl::cmd_velCallback(const geometry_msgs::Twist& cmdvel_)
{
	//将中心点的速度转换成左右轮速度
	leftVel=cmdvel_.linear.x-tan(cmdvel_.angular.z)*width/2;
	rightVel=cmdvel_.linear.x+tan(cmdvel_.angular.z)*width/2;

	std::cout <<"leftvel"<< fixed <<(leftVel) << endl;
	std::cout <<"(leftVel*60/WheelD)"<< (leftVel*60/WheelD) << endl;
	
	lrpm=(unsigned short)(leftVel*60/WheelD);
	rrpm=(unsigned short)(rightVel*60/WheelD);
	std::cout << std::hex << (lrpm & 0xffff) << endl;

	speedstr_l[4]=((lrpm>>8)&0xff);//crc高
	speedstr_l[5]=lrpm&0xff;//crc低
	crc16=getCRC16(speedstr_l,6);
	speedstr_l[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	 speedstr_l[6]=crc16&0xff;//crc高

	speedstr_r[4]=((rrpm>>8)&0xff);//crc高
	speedstr_r[5]=rrpm&0xff;//crc低
	crc16=getCRC16(speedstr_r,6);
	speedstr_r[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	speedstr_r[6]=crc16&0xff;//crc高

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
		}
		ptr++;
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
			
			if((j%RESENT_TIMES)==0&&size1==0)
			{	std::cout << "left" << endl;
				ros_ser1.write(s_buffer1,len);    //向串口发送指令
				printf("j = %d /n",j);
			}
			if((j%RESENT_TIMES)==0&&size2==0)
			{
				std::cout <<  "right" << endl;
				ros_ser2.write(s_buffer2,len);    //向串口发送指令
				printf("j = %d /n",j);
			}
			if(j >(RESENT_TIMES*10+1))
			{
				ROS_INFO("MotorControl send error");
				return;
			}
		}
	else
		ros::Duration(DELAY_TIME).sleep();

	//ROS_INFO_STREAM("MotorControl Reading from serial port");
	ros_ser1.read(buffer1,size1);
	ros_ser2.read(buffer2,size2);
	// ROS_INFO("l  : %d",size1);
	// ROS_INFO("r  : %d",size2);
	// std::cout <<"left_ctrl:";
	// for(int i=0; i<size1; i++)
	// {
	// 	//16进制的方式打印到屏幕
	// 	std::cout << std::hex << (buffer1[i] & 0xff) << " ";
	// 	//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	// }
	// std::cout <<"right_ctrl:";
	// 	for(int i=0; i<size2; i++)
	// {
	// 	//16进制的方式打印到屏幕
	// 	std::cout << std::hex << (buffer2[i] & 0xff) << " ";
	// 	//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	// }
	// std::cout <<endl;
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
			{
				crc >>= 1;
			}
		}
		ptr++;
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
	ros_ser2.write(s_buffer2,len);
	std::cout <<"com1发送了";
	for(int i=0; i<len; i++)
	{
		//16进制的方式打印到屏幕
		std::cout << std::hex << (s_buffer1[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
	std::cout <<"com2发送了";
		for(int i=0; i<len; i++)
	{
		//16进制的方式打印到屏幕
		std::cout << std::hex << (s_buffer2[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
	std::cout <<endl;

	if(get_mag)
		while(size1==0||size2==0)
		{
			//Sleep(1);//延时1ms
			ros::Duration(DELAY_TIME).sleep();
			size1 = ros_ser1.available();
			size2 = ros_ser2.available();
			j++;
			printf("j = %d \n",j);
			std::cout <<"left resent!!";
			if(size1>0)
				std::cout <<  "left is ok " << endl;
			if(size2>0)
				std::cout <<  "roght is ok " << endl;
			if((j%RESENT_TIMES)==0&&size1==0)
			{
				std::cout <<  "left" << endl;
				ros_ser1.write(s_buffer1,len);    //向串口发送指令
				
			}
			std::cout <<"right resent !";
			if((j%RESENT_TIMES)==0&&size2==0)
			{
				std::cout <<  "right" << endl;
				ros_ser2.write(s_buffer2,len);    //向串口发送指令
			}
			if(j >(RESENT_TIMES*10+1))
			{
				ROS_INFO("MotorInit send error");
				return;
			}
		}
	else
		ros::Duration(DELAY_TIME).sleep();

	ROS_INFO_STREAM("Reading from serial port");
	ros_ser1.read(buffer1,size1);
	ros_ser2.read(buffer2,size2);
	std::cout << "l接受了";
	for(int i=0; i<size1; i++)
	{
		//16进制的方式打印到屏幕
		std::cout << std::hex << (buffer1[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
	std::cout <<"r接受了"<<endl;
	
		for(int i=0; i<size2; i++)
	{
		//16进制的方式打印到屏幕
		std::cout << std::hex << (buffer2[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
	std::cout <<endl;
	std::cout <<endl;

}


void motor_control_init()
{
	unsigned char speedstr_l[8];
	unsigned char speedstr_r[8];
	unsigned short crc16;
//设置成网络操作模式
	printf("设置成网络操作模式");
	 speedstr_l[0]=add1;//驱动器地址
	 speedstr_l[1]=0x06;//功能码
	 speedstr_l[2]=0x00;//
	 speedstr_l[3]=0x24;//操作模式地址
	 speedstr_l[4]=0x00;//
	 speedstr_l[5]=0x00;//设置成网络模式
	 crc16=getCRC16(speedstr_l ,6);
	 speedstr_l[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	 speedstr_l[6]=crc16&0xff;//crc高

	 speedstr_r[0]=add2;//驱动器地址
	 speedstr_r[1]=0x06;//功能码
	 speedstr_r[2]=0x00;//
	 speedstr_r[3]=0x24;//操作模式地址
	 speedstr_r[4]=0x00;//
	 speedstr_r[5]=0x00;//设置成网络模式
	 crc16=getCRC16(speedstr_r ,6);
	 speedstr_r[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	 speedstr_r[6]=crc16&0xff;//crc高
	 sendmsg(speedstr_l,speedstr_r,8);
    //设置成速度控制模式
	 printf("设置成速度控制模式");
 	 speedstr_l[2]=0x00;//
	 speedstr_l[3]=0x25;//操作模式地址
	 speedstr_l[4]=0x00;//
	 speedstr_l[5]=0x01;//设置成速度模式
	 crc16=getCRC16(speedstr_l ,6);
	 speedstr_l[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	 speedstr_l[6]=crc16&0xff;//crc高

	 speedstr_r[2]=0x00;//
	 speedstr_r[3]=0x25;//操作模式地址
	 speedstr_r[4]=0x00;//
	 speedstr_r[5]=0x01;//设置成速度模式
	 crc16=getCRC16(speedstr_r ,6);
	 speedstr_r[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	 speedstr_r[6]=crc16&0xff;//crc高
	 sendmsg(speedstr_l,speedstr_r,8);


//使能电机
	printf("使能电机");
 	 speedstr_l[2]=0x00;//
	 speedstr_l[3]=0x28;//操作模式地址
	 speedstr_l[4]=0x00;//
	 speedstr_l[5]=0x01;//设置开启电机
	 crc16=getCRC16(speedstr_l ,6);
	 speedstr_l[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	 speedstr_l[6]=crc16&0xff;//crc高

	 speedstr_r[2]=0x00;//
	 speedstr_r[3]=0x28;//操作模式地址
	 speedstr_r[4]=0x00;//
	 speedstr_r[5]=0x01;//设置开启电机
	 crc16=getCRC16(speedstr_r ,6);
	 speedstr_r[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	 speedstr_r[6]=crc16&0xff;//crc高
	 sendmsg(speedstr_l,speedstr_r ,8);
	
    //  speedstr[2]=0x00;//
	//  speedstr[3]=0xAE;//速度寄存器地址
	//  speedstr[4]=0x00;//
	//  speedstr[5]=0x0A;//设置速度100
	//  crc16=getCRC16(speedstr ,6);
	//  speedstr[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	//  speedstr[6]=crc16&0xff;//crc高
	//  sendmsg(speedstr,speedstr,8);

	//  ros::Duration(3).sleep();

	//  speedstr[4]=0x00;//
	//  speedstr[5]=0x00;//设置速度0
	//  crc16=getCRC16(speedstr ,6);
	//  speedstr[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	//  speedstr[6]=crc16&0xff;//crc高
	//  sendmsg(speedstr,speedstr,8);

	//  printf("关闭电机");
 	//  speedstr[2]=0x00;//
	//  speedstr[3]=0x28;//操作模式地址
	//  speedstr[4]=0x00;//
	//  speedstr[5]=0x00;//设置关闭电机
	//  crc16=getCRC16(speedstr ,6);
	//  speedstr[7]=((crc16>>8)&0xff);//crc低，我已不知道为啥时这样，但是这样的结果正确
	//  speedstr[6]=crc16&0xff;//crc高
	//  sendmsg(speedstr,speedstr,8);
}

void MySigintHandler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
	ROS_INFO("shutting down!");
	ros::shutdown();
    exit(0);
}

int main(int argc, char *argv[])
{
	int xunhuan = 0;
	ros::init(argc, argv, "motor_control_sub");
//	ros::NodeHandle n;
	ros::Time::init();
	ros::Rate loop_rate(LOOP_RATE); 
	signal(SIGINT, MySigintHandler);
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
         ROS_INFO_STREAM("Serial_2 Port opened");
     }
     else
     {
         return -1;
     }
	motor_control_init();

    MotorControl motor_control;
	GetSpeed get_speed;
	get_speed.current_time = ros::Time::now();
  	get_speed.last_time = ros::Time::now();


   while(ros::ok())
   {
	   // std::cout <<"一个循环"<<endl ;
	   if(xunhuan > LOOP_RATE /SPEED_PUB_RART )
	   {
		   std::cout <<"一个循环"<<endl ;
			get_speed.send_get_speed();
			xunhuan =0;
	   }
		
	    //循环等待回调函数
		xunhuan++;
        ros::spinOnce();
        //按照循环频率延时
        loop_rate.sleep();
   }

	return 0;
}