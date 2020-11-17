// Step 1:  Include Library Headers:
//发布imu的数据到IMU_data
#include <ros/ros.h> 
#include <sensor_msgs/Imu.h>
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
#include <serial/serial.h>
#define DELAY_TIME 0.001	//每次通讯等待时间
#define LOOP_RATE 200		//每秒调用1k次回调函数
#define IMU_PUB_RART 50	//IMU信息测量频率
#define RESENT_TIMES 20	//多少次没有接受到就重发
#define BaudRate 115200	//通讯波特率
#define ERROR_TIME_MOST 15	//测试用，记录等待时间超过本时间的次数
#define get_mag 1		//未接受到数据时是否堵塞
using   namespace   std; 

serial::Serial ros_ser3;//串口3，IMU
int error_time_most = 0;
int real_resent_time = 0;
float BCDtoDec(unsigned char bcd[],int length,int position) 
{
    // char newbcd[125];
    // int b=0;
  

    // for(int a=0;a<length*2;a+=2)
    // {
    //     newbcd[a+1]=bcd[position+b] & 0x0f;
    //     newbcd[a]=(bcd[position+b]>>4) & 0x0f;
    //     b++;
    // }
    //将压缩的bdc码转换成数字
    int i=0;
    float dec = 0,tem,tem1;
    for ( i = 0; i < length; i++) 
    {
        //除了符号位，其他的进行计算
         tem = ((bcd[i+position] >> 4) & 0x0F) * 10 + (bcd[i+position] & 0x0F);;//减去0的ASCII,就是数字本身了
        tem1=1;
        for(int j=i;j>0;j--)
            tem1=0.01*tem1;
        dec+=tem * tem1;//缩小所在位
    }
    if ( dec >= 10 ) //判断符号
        dec = -1*( dec - 10 );
    ROS_INFO("%f",dec);
    return dec;
}

class Listener 
{
private:    //私有的，类自己的成员函数访问，不能被对象访问
    
    float orientation_x,orientation_y,orientation_z,orientation_w;      //四元数位姿
    float linear_acceleration_x,linear_acceleration_y,linear_acceleration_z;//线加速度
    float angular_velocity_x,angular_velocity_y,angular_velocity_z;    //角速度
    float roll,pitch,yall;       //姿态角
    ros::Subscriber client_send;
    ros::Time current_time;
    int more_sent_flog ;
public:
    unsigned char topic_message[256];
    unsigned char imu_data_save[256];
    unsigned char imustr_cmd[6];
    sensor_msgs::Imu imu_data;
    ros::NodeHandle n_;//所有均可以访问
    void callback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
 //   unsigned char* getMessageValue();
    bool bFlag;
    int send_get_IMU(unsigned char len);
    int get_IMU();
    void  count_data(unsigned char buffer[]);
   // float BCDtoDec(unsigned char abc[],int length,int position);
    Listener()
    {
         memset(topic_message, 0, 256); 
         memset(imu_data_save, 0, 256); 
         more_sent_flog =0;
         imustr_cmd[0]=0x77;//驱动器1地址
         imustr_cmd[1]=0x04;//功能码
         imustr_cmd[2]=0x00;//
         imustr_cmd[3]=0x59;//设置成电机速度寄存器
         imustr_cmd[4]=0x5D;//
         imustr_cmd[5]=0x00;//寄存器数量
         
         orientation_x=0;
         orientation_y=0;
         orientation_z=0;
         orientation_w=0;
         linear_acceleration_x=0;
         linear_acceleration_y=0;
         linear_acceleration_z=0;
         angular_velocity_x=0;
         angular_velocity_y=0;
         angular_velocity_z=0; 
         roll=0;
         pitch=0;
         yall=0;
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
    orientation_x=BCDtoDec(topic_message,4,31);//四元数，10 87 06 35代表-0.870635，共8个数，代表四元数中的一位
    orientation_y=BCDtoDec(topic_message,4,35);
    orientation_z=BCDtoDec(topic_message,4,39);
    orientation_w=BCDtoDec(topic_message,4,43);
    ROS_INFO("*************************");
    pitch=BCDtoDec(topic_message,3,4)*100;    //姿态角
    roll=BCDtoDec(topic_message,3,7)*100;
    yall=BCDtoDec(topic_message,3,10)*100;
    ROS_INFO("*************************");
    linear_acceleration_x=BCDtoDec(topic_message,3,13)*9.8;//线加速度，10 01 07代表-0.0107g，故需要乘g
    linear_acceleration_y=BCDtoDec(topic_message,3,16)*9.8;
    linear_acceleration_z=BCDtoDec(topic_message,3,19)*9.8;
    ROS_INFO("*************************");
    angular_velocity_x=BCDtoDec(topic_message,3,22)*100;//给定的真值表中，角速度10 00 13代表-0.13，与其他情况不一样，故乘以100
    angular_velocity_y=BCDtoDec(topic_message,3,25)*100;
    angular_velocity_z=BCDtoDec(topic_message,3,28)*100;    
   
 //   bzero(topic_message,256);

    imu_data.header.stamp = ros::Time::now();
    //    imu_data.header.stamp = listener.header.stamp;
    imu_data.orientation.x = orientation_x;
    imu_data.orientation.y = orientation_y;
    imu_data.orientation.z = orientation_z;
    imu_data.orientation.w = orientation_w;
    //线加速度
    imu_data.linear_acceleration.x = linear_acceleration_x; 
    imu_data.linear_acceleration.y = linear_acceleration_y;
    imu_data.linear_acceleration.z = linear_acceleration_z;
    //角速度
    imu_data.angular_velocity.x = angular_velocity_x; 
    imu_data.angular_velocity.y = angular_velocity_y; 
    imu_data.angular_velocity.z = angular_velocity_z;

    bFlag=1;
    ROS_INFO("%f  %f  %f",pitch,roll,yall);

}

int Listener::send_get_IMU(unsigned char len)
{
    unsigned char buffer[256];
    unsigned char imustr_cmd_000[]={0,0,0,0,0};
	unsigned char buffer_1[256];
   	char size=0;
    char size_1=0;
	int j=0;
    ros_ser3.write(imustr_cmd_000,5);
	ros_ser3.write(imustr_cmd,len);

    size = ros_ser3.available();
   // ros_ser3.write(imustr_cmd,len);
    for(int i=0; i<len; i++)
	{
		
		//16进制的方式打印到屏幕
		std::cout <<std::hex << (imustr_cmd[i] & 0xff) << " ";
		//std::cout << std::hex << (buffer[i] & 0xff) << " ";
	}
	if(get_mag)
	{
		while(size==0)
		{
			
			//Sleep(1);//延时1ms
			ros::Duration(DELAY_TIME).sleep();
			size = ros_ser3.available();
			j++; 
			printf("j = %d \n",j);
			if(size!=0)
				std::cout <<  "IMU is ok " << endl;
			if((j%RESENT_TIMES)==0&&size==0)
			{	std::cout << "IMU DATA" << endl;
                ros_ser3.write(imustr_cmd_000,5);
				ros_ser3.write(imustr_cmd,len);    //向串口发送指令
               // ros_ser3.write(imustr_cmd,len);    //向串口发送指令
				
			}
			if(j >(RESENT_TIMES*10+1))
			{
				ROS_INFO("IMU send error");
                error_time_most++;
				return 0;
			}  
		}
        if(j>ERROR_TIME_MOST)
            error_time_most++;
        if(j >(RESENT_TIMES+1))
            real_resent_time++;
	}
	else
		ros::Duration(DELAY_TIME).sleep();
	current_time = ros::Time::now();	//记录接受到消息时的时间
	ROS_INFO_STREAM("IMU Reading from serial port");
    ros::Duration(DELAY_TIME).sleep();
    ros::Duration(DELAY_TIME).sleep();
    size = ros_ser3.available();
	ros_ser3.read(buffer,size);
	ROS_INFO("IMU SIZE  : %d",size);
	std::cout <<"IMU_DATA:";
	//	return;
	for(int i=0; i<size; i++)
	{
		//16进制的方式打印到屏幕
		std::cout <<std::hex << (buffer[i] & 0xff) << " ";
	}
    if((buffer[0]==0x77 && buffer[1] == 0x2f && buffer[2] == 0x00 && buffer[3] == 0x59)&&size>=48){
       
        count_data(buffer);
        return 1;
    }
    else if((buffer[0]==0x77 && buffer[1] == 0x2f && buffer[2] == 0x00 && buffer[3] == 0x59)&&size<48)
    {
        ros::Duration(DELAY_TIME).sleep();
        ros::Duration(DELAY_TIME).sleep();
        size_1 = ros_ser3.available();
        ros_ser3.read(buffer_1,size_1);
        if((size+size_1)==48)
        {
            for(int i = 0;i<size_1;i++)
            {
                buffer[size+i] = buffer_1[i];
            }
            count_data(buffer);
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else    
    {
        ros::Duration(DELAY_TIME).sleep();
        ros::Duration(DELAY_TIME).sleep();
        ros::Duration(DELAY_TIME).sleep();
        ros::Duration(DELAY_TIME).sleep();
        size = ros_ser3.available();
        ros_ser3.read(buffer,size);
        return 0;
    }
    
}

int Listener::get_IMU()
{
    unsigned char buffer[256];
    unsigned char buffer_1[256];
    int  sum;
   	char size=0;
    char size_1=0;
	int j=0;
    sum = 0;

    size = ros_ser3.available();
	if(get_mag)
	{
		while(size==0)
		{
			
			//Sleep(1);//延时1ms
			ros::Duration(DELAY_TIME).sleep();
			size = ros_ser3.available();
			j++; 
			//printf("j = %d \n",j);
			if(size!=0)
				std::cout <<  "IMU is ok " << endl;

			if(j >(RESENT_TIMES*10+1))
			{
				ROS_INFO("IMU send error");
                error_time_most++;
				return 0;
			}  
		}
        // if(j>ERROR_TIME_MOST)
        //     error_time_most++;
        // if(j >(RESENT_TIMES+1))
        //     real_resent_time++;
	}
	else
		ros::Duration(DELAY_TIME).sleep();

	current_time = ros::Time::now();	//记录接受到消息时的时间
	ROS_INFO_STREAM("IMU Reading from serial port");
    ros::Duration(DELAY_TIME).sleep();
    ros::Duration(DELAY_TIME).sleep();
    ros::Duration(DELAY_TIME).sleep();
    ros::Duration(DELAY_TIME).sleep();  //大约4ms，断短了错误数据多
    size = ros_ser3.available();
	ros_ser3.read(buffer,size);
	ROS_INFO("IMU SIZE  : %d",size);
	std::cout <<"IMU_DATA:";
	//	return;
	for(int i=0; i<size; i++)
	{
		//16进制的方式打印到屏幕
		std::cout <<std::hex << (buffer[i] & 0xff) << " ";
	}
    if((buffer[0]==0x77 && buffer[1] == 0x2f && buffer[2] == 0x00 && buffer[3] == 0x59)&&size>=48){
        if(size>48&&buffer[48]==0x77)
        {
            for(int i = 48;i<size;i++)
                imu_data_save[i-48]=buffer[i];
            more_sent_flog = size - 48;
        }
        for(int i = 1; i<47 ; i++)
            {
                sum+=buffer[i];
            }
        if((sum&0xff)==buffer[47])
        {
                count_data(buffer);
                return 1;
        }
        else
        {
            return 0;
        }     
    }
    else if((buffer[0]==0x77 && buffer[1] == 0x2f && buffer[2] == 0x00 && buffer[3] == 0x59)&&size<48)
    {
        j=0;
        ros::Duration(DELAY_TIME).sleep();
        size_1 = ros_ser3.available();
        
        while ((size+size_1)<48&&j<6)
        {
            ros::Duration(DELAY_TIME).sleep();
            size_1 = ros_ser3.available();
            j++;
        }
        ROS_INFO("IMU SIZE_1  : %d",size_1);
        ros_ser3.read(buffer_1,size_1);
        for(int i=0; i<size_1; i++)
	    {
		//16进制的方式打印到屏幕
		std::cout <<std::hex << (buffer_1[i] & 0xff) << " ";
	    }
        if((size+size_1)>=48)
        {
            for(int i = 0;i<size_1;i++)
            {
                buffer[size+i] = buffer_1[i];
            }
            for(int i = 1; i<47 ; i++)
            {
                sum+=buffer[i];
            }
            cout<<hex << "sum&0xff  " <<(sum&0xff) << "   buffer[47]    " <<(buffer[47] & 0xff)<<endl;
            if((sum&0xff)==buffer[47])
            {
                count_data(buffer);
                ROS_INFO("出现不完整数据，请判断是否正常----------------------------------------------------------------");
                real_resent_time++;
                return 1;
            }
            else
            {
                return 0;
            }  
        }
        else
        {
            return 0;
        }
    }
    else if((more_sent_flog+size)==48&&more_sent_flog!=0)
    {
        for(int i = 0;i<size_1;i++)
        {
            imu_data_save[more_sent_flog+i] = buffer_1[i];
        }
        for(int i = 1; i<48 ; i++)
        {
            sum+=imu_data_save[i];
        }
        cout<<hex << "sum&0xff  " <<(sum&0xff) << "   buffer[47]    " <<(buffer[47] & 0xff)<<endl;
        if((sum&0xff)==buffer[47])
        {
            count_data(imu_data_save);
            ROS_INFO("出现不完整数据，请判断是否正常++++++++++++++++++++++++++++++++++++++++++++");
            more_sent_flog=0;
            memset(imu_data_save, 0, 256); 
            return 0;
        }
        else
        {
            return 0;
        }

    }
    else    
    {
        ros::Duration(DELAY_TIME).sleep();
        ros::Duration(DELAY_TIME).sleep();
        size = ros_ser3.available();
        ros_ser3.read(buffer,size);
        more_sent_flog =0;
        return 0;
    }
}

void   Listener::count_data(unsigned char buffer[])
{
     ROS_INFO("**********************************************************************");
        orientation_x=BCDtoDec(buffer,4,31);//四元数，10 87 06 35代表-0.870635，共8个数，代表四元数中的一位
        orientation_y=BCDtoDec(buffer,4,35);
        orientation_z=BCDtoDec(buffer,4,39);
        orientation_w=BCDtoDec(buffer,4,43);
        ROS_INFO("*************************");
        pitch=BCDtoDec(buffer,3,4)*100;    //姿态角
        roll=BCDtoDec(buffer,3,7)*100;
        yall=BCDtoDec(buffer,3,10)*100;
        ROS_INFO("*************************");
        linear_acceleration_x=BCDtoDec(buffer,3,13)*9.8;//线加速度，10 01 07代表-0.0107g，故需要乘g
        linear_acceleration_y=BCDtoDec(buffer,3,16)*9.8;
        linear_acceleration_z=BCDtoDec(buffer,3,19)*9.8;
        ROS_INFO("*************************");
        angular_velocity_x=BCDtoDec(buffer,3,22)*100;//给定的真值表中，角速度10 00 13代表-0.13，与其他情况不一样，故乘以100
        angular_velocity_y=BCDtoDec(buffer,3,25)*100;
        angular_velocity_z=BCDtoDec(buffer,3,28)*100;    
    
    //   bzero(topic_message,256);

        imu_data.header.stamp =current_time;
        //    imu_data.header.stamp = listener.header.stamp;
        imu_data.orientation.x = orientation_x;
        imu_data.orientation.y = orientation_y;
        imu_data.orientation.z = orientation_z;
        imu_data.orientation.w = orientation_w;
        //线加速度
        imu_data.linear_acceleration.x = linear_acceleration_x; 
        imu_data.linear_acceleration.y = linear_acceleration_y;
        imu_data.linear_acceleration.z = linear_acceleration_z;
        //角速度
        imu_data.angular_velocity.x = angular_velocity_x; 
        imu_data.angular_velocity.y = angular_velocity_y; 
        imu_data.angular_velocity.z = angular_velocity_z;
        ROS_INFO("%f  %f  %f",pitch,roll,yall);
}



// unsigned char* Listener::getMessageValue() 
// {
//     return topic_message;
// }

int main(int argc, char** argv)
{
    // Step 2: Initialization:
    ros::init(argc, argv, "imu");     
    std::string frame_id;
    ros::NodeHandle n("~");
    Listener listener;
   // ros::Subscriber client_send = nh2.subscribe("/get_messages", 10, &Listener::callback, &listener);

    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("imu_data", 1);  //定义要发送topic的函数
    n.param<std::string> ("frame_id", frame_id, "imu_link");

    listener.imu_data.header.frame_id = frame_id;
    //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，然后进行传递
                ///////////////////////////////////////////////
    ros::Time::init();
    ros::Rate loop_rate(IMU_PUB_RART); 
    try
     {
		 //串口3，imu
         ros_ser3.setPort("/dev/ttyS3"); //tty3对应com3口，tty2对应com4
         ros_ser3.setBaudrate(BaudRate);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser3.setTimeout(to);
         ros_ser3.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }

     if(ros_ser3.isOpen())
     {
         ROS_INFO_STREAM("Serial_1 Port opened");
     }
     else
     {
         return -1;
     }
    int  error_time=0;
    int  send_time=0;
    //listener.send_get_IMU(5);

    while(ros::ok())
    {
        //if(listener.send_get_IMU(5))
        if(listener.get_IMU())
        {
            IMU_pub.publish(listener.imu_data);
            send_time++;
        }
        else
        {
            error_time++;
           // while (1);
        }
        
        cout<<"****************************error_time****************************" <<error_time<<endl;
        cout<<"****************************send_time****************************" <<send_time<<endl;
        cout<<"****************************ERROR_TIME_MOST****************************" <<error_time_most<<endl;
        cout<<"****************************REAL_RESENT_TIMES****************************" <<real_resent_time<<endl;
        //循环等待回调函数

        if(error_time>=1)
        {
            if(listener.get_IMU())
            {
                IMU_pub.publish(listener.imu_data);
                send_time++;
            }
            else
            {
                error_time++;
            //  while (1);
            }
              cout<<"****************************error_time****************************" <<error_time<<endl;
        cout<<"****************************send_time****************************" <<send_time<<endl;
        cout<<"****************************ERROR_TIME_MOST****************************" <<error_time_most<<endl;
        cout<<"****************************REAL_RESENT_TIMES****************************" <<real_resent_time<<endl;   
            if(listener.get_IMU())
            {
                IMU_pub.publish(listener.imu_data);
                send_time++;
            }
            else
            {
                error_time++;
            //  while (1);
            }
             cout<<"****************************error_time****************************" <<error_time<<endl;
        cout<<"****************************send_time****************************" <<send_time<<endl;
        cout<<"****************************ERROR_TIME_MOST****************************" <<error_time_most<<endl;
        cout<<"****************************REAL_RESENT_TIMES****************************" <<real_resent_time<<endl;
            if(listener.get_IMU())
            {
                IMU_pub.publish(listener.imu_data);
                send_time++;
            }
            else
            {
                error_time++;
            //  while (1);
            }
             cout<<"****************************error_time****************************" <<error_time<<endl;
        cout<<"****************************send_time****************************" <<send_time<<endl;
        cout<<"****************************ERROR_TIME_MOST****************************" <<error_time_most<<endl;
        cout<<"****************************REAL_RESENT_TIMES****************************" <<real_resent_time<<endl;
             while (1);
        }
        ros::spinOnce();
      


        //按照循环频率延时
       // loop_rate.sleep();
    }
    return 0;
}
