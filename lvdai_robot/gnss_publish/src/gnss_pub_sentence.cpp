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
#define IF_TEST 1		//是否测试模式，1为测试模式
using   namespace   std; 
const int QueueSize=500;
serial::Serial ros_ser4;//串口4，GNSS
int error_time_most = 0;
int real_resent_time = 0;

 
class CirQueue
{
	public:
		CirQueue()                             //构造函数，置空队列
		{
			front=rear=0;
		}
		~CirQueue(){cout<<"destory";}    //析构函数
		void EnQueue(int x);                   //元素x入队
		char DeQueue();                         //队头元素出队
		char GetQueue();                       //获取队头元素，不删除队头
		bool Empty()                          //判断队列是否为空
		{
			if(front==rear)
				return 1;
			else
				return 0;
		}
	private:
		char data[QueueSize];                  //存放队列的数组
		int front,rear;                      //头指针与尾指针
};
 
void CirQueue::EnQueue(int x)
{
	if((rear+1)%QueueSize==front)             //判断队列是否已满
		cout<<"queue is full,can't put "<<x<<" into it"<<endl;
	else
	{
		rear=(rear+1)%QueueSize;             //移动尾指针指向下一个空间
		data[rear]=x;                        //元素x入队
	}
}

 
char CirQueue::DeQueue()                    //队头元素出栈     
{
	if(Empty())                            //判断队列是否为空
    {
        cout<<"queue is empty"<<endl;
        return '|' ;
    }
		
	else
	{
		front=(front+1)%QueueSize;        //移动队头指针指向下一个空间，即被删元素所在位置
		return data[front];               //返回被删除的元素的值
	}
}
 
char CirQueue::GetQueue()
{
	if(Empty())
	{
        cout<<"queue is empty"<<endl;
        return '|' ;
    }
	else
	{
		return data[(front+1)%QueueSize];
	}
}
class Listener 
{
private:    //私有的，类自己的成员函数访问，不能被对象访问

    ros::Time current_time;
    int more_sent_flog ;
    CirQueue Q;
    std::string msg;
    bool chinge_flog;
public:
    unsigned char topic_message[256];
    unsigned char GNSS_data_save[256];
    nmea_msgs::Sentence gnss_msg;
    std_msgs::Header header;
    ros::NodeHandle n_;//所有均可以访问

 //   unsigned char* getMessageValue();
    int get_GNSS();
    int GNSS_checksum(unsigned char buff[],int len);
   // float BCDtoDec(unsigned char abc[],int length,int position);
    Listener()
    {
        memset(topic_message, 0, 256); 
        header.frame_id="gps";
        header.seq = 1;
        chinge_flog = 1;
        more_sent_flog=0;
    }
};


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


int Listener::get_GNSS()
{
    uint8_t buffer[256];
    int  sum;
   	char size=0;
    char size_1=0;
	int j=0;
    sum = 0;
    uint8_t buffer_[]={'2','3','5','6','7','a','s','d','f'};
    uint8_t buffer_2[]="abcd123\n999";
    more_sent_flog ++;
    std::cout << "*********************" <<more_sent_flog <<endl;
    if(IF_TEST)
    {
        if(more_sent_flog%5 !=0)
        {
            if(chinge_flog)
            {
                current_time = ros::Time::now();	//记录接受到消息时的时间
                chinge_flog = 0;
                for(int i = 0 ; i<sizeof(buffer_);i++)
                Q.EnQueue(buffer_[i]);
            }
            else
            {
                for(int i = 0 ; i<sizeof(buffer_2);i++)
                Q.EnQueue(buffer_2[i]);
            }
        }
    }
    else
    {
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

            if(j >(RESENT_TIMES*100+1))
            {
                ROS_INFO("gnss get error");
                error_time_most++;
                return 0;
            }  
        }
        if(chinge_flog)
        {
            current_time = ros::Time::now();	//记录接受到消息时的时间
            chinge_flog = 0;
        }
        
        ROS_INFO_STREAM("gnss Reading from serial port");
        ros::Duration(DELAY_TIME*5).sleep();
        
        size = ros_ser4.available();
        ros_ser4.read(buffer,size);
        for(int i = 0 ; i<sizeof(buffer);i++)
                Q.EnQueue(buffer[i]);
    }
    while (!Q.Empty())
    {
       // std::cout << "运行到了" <<endl;
        char read_Q = Q.DeQueue();
        uint8_t a[] = "\n";
        msg += read_Q;
        cout<<"msg:"<<msg<<endl;
        if(read_Q == a[0])
        {
            gnss_msg.sentence=msg;
            cout<<"gnss_msg.sentence:"<<gnss_msg.sentence<<endl;

        //  strncp(gnss_ptr->sentence,buffer,size);
            header.stamp =current_time;
            header.seq++;
            gnss_msg.header =header;
            std::cout << "到了" <<endl;
            msg.clear();
            std::cout << "msg" <<msg <<endl ;
            // memset(msg, 0, sizeof(msg));
            chinge_flog =1;
            return 1;
        }   
    }
    return 0;
	
}



int main(int argc, char** argv)
{
    // Step 2: Initialization:
    ros::init(argc, argv, "gnss_pub_sentence");     
    std::string frame_id;
    ros::NodeHandle n("~");
    Listener listener;
   // ros::Subscriber client_send = nh2.subscribe("/get_messages", 10, &Listener::callback, &listener);

    ros::Publisher gnss_pub = n.advertise<nmea_msgs::Sentence>("/nmea_sentence", 100);  //定义要发送topic的函数
    n.param<std::string> ("frame_id", frame_id, "gps");
    cout << frame_id<<endl;
    listener.gnss_msg.header.frame_id = frame_id;
    cout << listener.gnss_msg.header.frame_id<<endl;
    ros::Time::init();
    ros::Rate loop_rate(GPS_PUB_RART); 
    try
     {
		 //串口3，gnss
         ros_ser4.setPort("/dev/ttyS2"); //ttys3对应com3口，ttys2对应com4
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
            cout<<"运行发布"<<endl;
            gnss_pub.publish(listener.gnss_msg);
            cout << listener.gnss_msg.header.frame_id<<endl;
            send_time++;
        }
        else
        {
            error_time++;
           // while (1);
        }
        loop_rate.sleep();
    }
    return 0;
}




