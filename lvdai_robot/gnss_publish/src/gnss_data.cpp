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

#define DELAY_TIME 0.001	//每次通讯等待时间
#define LOOP_RATE 200		//每秒调用200次回调函数
#define GPS_PUB_RART 50	//信息测量频率
#define RESENT_TIMES 20	//多少次没有接受到就重发
#define BaudRate 115200	//通讯波特率
#define ERROR_TIME_MOST 15	//测试用，记录等待时间超过本时间的次数
#define get_mag 1		//未接受到数据时是否堵塞
using   namespace   std; 

serial::Serial ros_ser4;//串口4，GNSS
int error_time_most = 0;
int real_resent_time = 0;



class Listener 
{
private:    //私有的，类自己的成员函数访问，不能被对象访问

    ros::Subscriber client_send;
    ros::Time current_time;
    int more_sent_flog ;
    
public:
    unsigned char topic_message[256];
    unsigned char GNSS_data_save[256];
    gnss_publish::gnss_gphcd gnss_msg;
    std_msgs::Header header;
    ros::NodeHandle n_;//所有均可以访问
    void callback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
 //   unsigned char* getMessageValue();
    bool bFlag;
    int get_GNSS();
    int GNSS_checksum(unsigned char buff[],int len);
    int count_data(unsigned char buffer[],int size);
   // float BCDtoDec(unsigned char abc[],int length,int position);
   
    Listener()
    {
        memset(topic_message, 0, 256); 
        header.frame_id='frame';
        header.seq = 1;
        bFlag=0;
        client_send = n_.subscribe("/get_messages", 10, &Listener::callback, this);
    }
};



void Listener::callback(const std_msgs::UInt8MultiArray::ConstPtr& msg) 
{

    memset(topic_message, 0, 256);  //赋值函数，给某一块内存空间赋值，第一个参数时数组名或指针
                                    //第二个时要填充的值，第三个参数是要填充的字节数
   // memcpy(topic_message, msg->data,52);    //该函数不检查null字符（即将null字符当作普通字符处理），意味着将复制num个字符才结束，
                                                    //strcpy遇到\0就结束复制了
                

    for(int i=0; i<52; i++)
    {
        //16进制的方式打印到屏幕
        topic_message[i]= msg->data[i];
     //   std::cout << std::hex << (msg->data.c_str()[i] & 0xff) << " ";
        //std::cout << std::hex << (buffer[i] & 0xff) << " ";
    }
 //   std::cout << std::endl;

     for(int i=0; i<52; i++)
    {
        //16进制的方式打印到屏幕
        std::cout << std::hex << (topic_message[i] & 0xff) << " ";
        
    }
    ROS_INFO("**********************************************************************");

 //   bzero(topic_message,256);


}
int Listener::GNSS_checksum(unsigned char buff[],int len)
{
    int checksum=0;     
    for(int i=0;i<len;i++)     
    {     
      checksum+=buff[i];     
    }     
    printf("checksum=%x/n",checksum);     
    checksum=(checksum>>16)+(checksum & 0xffff);     
    checksum+=(checksum>>16);     
    checksum=0xffff-checksum;     
    printf("checksum=%04x/n",checksum);
    return  0;

}


int  Listener::count_data(unsigned char buffer[],int size)
{
    string GPHCD,GPSDay, GPSMonth,GPSYear, GPSUTC,Heading;
    string Pitch, Reserved,VTG,Lat,Lon, Alt, X, Y,Z,QF,SatNo,xx;
    int num=0;
    int flag=999;


    // if(GNSS_checksum(buffer,130)==0)//如果校验不通过则跳过
    // {
    //     return 0;
    // }

    for(int i=0; i<size-2; i++)
    {
        if(buffer[i]==',')
        {
            num++;
        }
        else if(buffer[i]=='*')
        {
            num =17;
        }
        else 
        {
            if(num==0)
            {
                GPHCD+=buffer[i];   
            }
            else if(num==1)
            {
                GPSDay+=buffer[i];
            }
            else if(num==2)
            {
                GPSMonth+=buffer[i];
            }
            else if(num==3)
            {
                GPSYear+=buffer[i];
            }
            else if(num==4)
            {
                GPSUTC+=buffer[i];
            }
            else if(num==5)
            {
                Heading+=buffer[i];
            }
            else if(num==6)
            {
                Pitch+=buffer[i];
            }
            else if(num==7)
            {
                Reserved+=buffer[i];
            }
            else if(num==8)
            {
                VTG+=buffer[i];
            }
            else if(num==9)
            {
                Lat+=buffer[i];
            }
            else if(num==10)
            {
                Lon+=buffer[i];
            }
            else if(num==11)
            {
                Alt+=buffer[i];
            }
            else if(num==12)
            {
                X+=buffer[i];
            }
            else if(num==13)
            {
                Y+=buffer[i];
            }
            else if(num==14)
            {
                Z+=buffer[i];
            }
            else if(num==15)
            {
                QF+=buffer[i];
            }
            else if(num==16)
            {
                SatNo+=buffer[i];
            }
            else if(num==17)
            {
                xx+=buffer[i];
            }
        }
    }

// flag = GNSS_checksum(buffer,size);
// cout<<"flag" << flag<<endl;

    gnss_msg.header=header;
    gnss_msg.GPSDay=GPSDay;
    gnss_msg.GPSMonth=GPSMonth;
    gnss_msg.GPSYear=GPSYear;
    gnss_msg.GPSUTC=GPSUTC;
    gnss_msg.Heading=Heading;
    gnss_msg.Pitch=Pitch;
    gnss_msg.Reserved=Reserved;
    gnss_msg.VTG=VTG;
    gnss_msg.Lat=Lat;
    gnss_msg.Lon=Lon;
    gnss_msg.Alt=Alt;
    gnss_msg.X=X;
    gnss_msg.Y=Y;
    gnss_msg.Z=Z;
    gnss_msg.QF=QF;
    gnss_msg.SatNo=SatNo;
    gnss_msg.xx=xx;
    ROS_INFO("GPSDay:%s",gnss_msg.GPSDay.c_str());
    ROS_INFO("GPSMonth:%s",gnss_msg.GPSMonth.c_str());
    ROS_INFO("GPSYear:%s",gnss_msg.GPSYear.c_str());
    ROS_INFO("GPSUTC:%s",gnss_msg.GPSUTC.c_str());
    ROS_INFO("Heading:%s",gnss_msg.Heading.c_str());
    ROS_INFO("Pitch:%s",gnss_msg.Pitch.c_str());
    ROS_INFO("Reserved:%s",gnss_msg.Reserved.c_str());
    ROS_INFO("VTG:%s",gnss_msg.VTG.c_str());
    ROS_INFO("Lat:%s",gnss_msg.Lat.c_str());
    ROS_INFO("Lon:%s",gnss_msg.Lon.c_str());
    ROS_INFO("Alt:%s",gnss_msg.Alt.c_str());
    ROS_INFO("X:%s",gnss_msg.X.c_str());
    ROS_INFO("Y:%s",gnss_msg.Y.c_str());
    ROS_INFO("Z:%s",gnss_msg.Z.c_str());
    ROS_INFO("QF:%s",gnss_msg.QF.c_str());
    ROS_INFO("SatNo:%s",gnss_msg.SatNo.c_str());
    ROS_INFO("xx:%s",gnss_msg.xx.c_str());
    return 1;

}

int Listener::get_GNSS()
{
    unsigned char buffer[256];
    unsigned char buffer_1[256];
    int  sum;
   	char size=0;
    char size_1=0;
	int j=0;
    sum = 0;
    size = ros_ser4.available();
    while(size==0)
    {
        
        //Sleep(1);//延时1ms
        ros::Duration(DELAY_TIME).sleep();
        size = ros_ser4.available();
        j++; 
        //printf("j = %d \n",j);
        if(size!=0)
            std::cout <<  "GNSS get data! " << endl;

        if(j >(RESENT_TIMES*1000+1))
        {
            ROS_INFO("gnss send error");
            error_time_most++;
            return 0;
        }  
    }
	current_time = ros::Time::now();	//记录接受到消息时的时间
	ROS_INFO_STREAM("gnss Reading from serial port");
    ros::Duration(DELAY_TIME*10).sleep();
    
    size = ros_ser4.available();
	ros_ser4.read(buffer,size);
	ROS_INFO("gnss SIZE  : %d",size);
	std::cout <<"gnss_DATA:";
	//	return;
	for(int i=0; i<size; i++)
	{
		//16进制的方式打印到屏幕
		std::cout << buffer[i] ;
	}
 //   if(buffer[0]=='$' && buffer[1] == 'G' && buffer[2] == 'P' && buffer[3] == 'H' && buffer[4] == 'C' && buffer[5] == 'D' && size>=132){
    if(buffer[0]=='$' && buffer[1] == 'G' && buffer[2] == 'P' && buffer[3] == 'H' && buffer[4] == 'C' && buffer[5] == 'D'){
        if(size>132&&buffer[132]=='$')
        {
            for(int i = 132;i<size;i++)
                GNSS_data_save[i-48]=buffer[i];
            more_sent_flog = size - 132;
        }
        return count_data(buffer,size);

    }
    
    else    
    {
        ros::Duration(DELAY_TIME).sleep();
        ros::Duration(DELAY_TIME).sleep();
        size = ros_ser4.available();
        ros_ser4.read(buffer,size);
        more_sent_flog =0;
        return 0;
    }
}


// unsigned char* Listener::getMessageValue() 
// {
//     return topic_message;
// }

int main(int argc, char** argv)
{
    // Step 2: Initialization:
    ros::init(argc, argv, "gnss_data");     
    std::string frame_id;
    ros::NodeHandle n("~");
    Listener listener;
   // ros::Subscriber client_send = nh2.subscribe("/get_messages", 10, &Listener::callback, &listener);

    ros::Publisher gnss_pub = n.advertise<gnss_publish::gnss_gphcd>("gnss_data", 1);  //定义要发送topic的函数
    n.param<std::string> ("frame_id", frame_id, "gnss");

    listener.gnss_msg.header.frame_id = frame_id;
    //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，然后进行传递
                ///////////////////////////////////////////////
    ros::Time::init();
    ros::Rate loop_rate(GPS_PUB_RART); 
    try
     {
		 //串口3，gnss
         ros_ser4.setPort("/dev/ttyS2"); //tty3对应com3口，tty2对应com4
         ros_ser4.setBaudrate(BaudRate);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser4.setTimeout(to);
         ros_ser4.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }

     if(ros_ser4.isOpen())
     {
         ROS_INFO_STREAM("Serial_4 Port opened");
     }
     else
     {
         return -1;
     }
    int  error_time=0;
    int  send_time=0;

    while(ros::ok())
    {

    
        if(listener.get_GNSS())
        {
            gnss_pub.publish(listener.gnss_msg);
            send_time++;
        }
        else
        {
            error_time++;
           // while (1);
        }
        

    
    }
    return 0;
}




