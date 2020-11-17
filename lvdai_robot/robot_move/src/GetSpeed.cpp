#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <ros/ros.h>  
#include <string.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/UInt8MultiArray.h"
class GetSpeed
{   
public:
    float leftVel;
    float rightVel;
    char leftstr[4];
    char rightstr[4];
    char speedstr[4];
    void cmd_velCallback(const geometry_msgs::Twist& cmdvel_);
 //   void SubscribeAndPublish();
    GetSpeed()
    {
        leftVel=0.0;
        rightVel=0.0;
        memset(leftstr,0,sizeof(leftstr));
        memset(rightstr,0,sizeof(rightstr));
        pub_=n_.advertise<std_msgs::UInt8MultiArray>("PubSpeed",25);
        sub_=n_.subscribe("cmd_vel",20,&GetSpeed::cmd_velCallback,this);
    }
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
};
// void GetSpeed::SubscribeAndPublish()
// {
//      pub_=n_.advertise<std_msgs::String>("PubSpeed",25);
//     sub_=n_.subscribe("cmd_vel",20,&GetSpeed::cmd_velCallback,this);
// }

void GetSpeed::cmd_velCallback(const geometry_msgs::Twist& cmdvel_)
{
    //将中心点的速度转换成左右轮速度
    leftVel=cmdvel_.linear.x-cmdvel_.angular.z*5/2;
    rightVel=cmdvel_.linear.x+cmdvel_.angular.z*5/2;
    ROS_INFO("I heard left_speed: [%f],right_speed: [%f]",leftVel,rightVel);
    int Flagl=1;
    int Flagr=1;
    memset(speedstr,0,sizeof(speedstr));
    if(leftVel*100>0)
    {
        Flagl=1;
    }
    else
    {
        Flagl=-1;
    } 
    int n=leftVel*100*Flagl;
    int unitPlace=n/1%10;
    int tenPlace=n/10%10;
    int hundredPlace=n/100%10;
    speedstr[1]=(tenPlace<<4)+unitPlace;
    speedstr[0]+=hundredPlace;
    //ROS_INFO("lala:%c %c %c",a,b,c);
    if(leftVel>=0)
    {
        speedstr[0]&=0x0f;
    }
    else
    {
        speedstr[0]&=0x0f;
         speedstr[0]|=0x10;
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(rightVel>0)
    {
        Flagr=1;
    }
    else
    {
        Flagr=-1;
    } 
    int w=rightVel*100*Flagr;
    int unit=w/1%10;
    int ten=w/10%10;
    int hundred=w/100%10;
    speedstr[3]=(ten<<4)+unit;
    speedstr[2]+=hundred;
    if(rightVel>=0)
    {
        speedstr[2]&=0x0f;
    }
    else
    {
       speedstr[2]&=0x0f;
         speedstr[2]|=0x10;
    }

 for(int i=0; i<4; i++)
           {
               //16进制的方式打印到屏幕
               std::cout << std::hex << (speedstr[i] & 0xff) << " ";
           }
             std::cout << std::endl;
           
    //        //发布消息

  //  speedstr[4]='\0';
  
      std_msgs::UInt8MultiArray msg;

    // memcpy( msg.data , speedstr , 4);

      msg.data.resize(4);
   for(int i=0; i<4; i++)
           {
               //16进制的方式打印到屏幕
               
               msg.data[i] = speedstr[i];
               std::cout << std::hex << (msg.data[i] & 0xff) << " ";
           }
               std::cout << std::endl;   
    //        //发布消息
    //  std::cout<<msg.data<<std::endl;
     pub_.publish(msg);
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"GetSpeed");
    ros::NodeHandle n1;
    GetSpeed speed;
   // ros::Subscriber sub_=n1.subscribe("cmd_vel",20,&GetSpeed::cmd_velCallback,&speed);
    // ROS_INFO("%d",speed.leftstr);
   // ros::Publisher pub_=n2.advertise<std_msgs::String>("PubSpeed",25);   ///////////////////////////修改
    //ros::Rate loop_rate(25);  
    ros::spin();
    return 0;
}
        
    // while(ros::ok())
    // {
    //   std_msgs::String msg;
    //    if(strlen(speed.speedstr)==4)
    //    {
    //        for(int i=0; i<6; i++)
    //         {
    //             msg.data[i]=0;
    //         }
    //        msg.data=speed.speedstr;
           
    //        //发布消息
    //        std::cout<<msg.data<<std::endl;
          
    //        pub_.publish(msg);
    //    }
    //    else{
    //         for(int i=0; i<4; i++)
    //         {
    //             msg.data[i]=0;
    //          //   std::cout << std::hex << (msg.data[i] & 0xff) << " ";
    //         }
    //        std::cout<<msg.data<<std::endl;
    //        ROS_INFO("%s",msg.data);
    //         pub_.publish(msg);

    //   }
    //     //for(int i=0; i<6; i++)
    //      //   {
    //      //       //16进制的方式打印到屏幕
    //      //       std::cout << std::hex << (msg.data[i] & 0xff) << " ";
    //      //   }
    //           //  std::cout << std::endl;
    //    //循环等待回调函数
    //    ros::spinOnce();
    //    //按照循环频率延时
    //    loop_rate.sleep();
    // }
  
