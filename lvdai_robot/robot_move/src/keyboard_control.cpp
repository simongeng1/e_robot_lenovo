#include <termios.h>  
#include <signal.h>  
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
#include <boost/thread/thread.hpp>  
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>  

#define KEYCODE_W 0x77  
#define KEYCODE_A 0x61  
#define KEYCODE_S 0x73  
#define KEYCODE_D 0x64  
#define KEYCODE_A_CAP 0x41  
#define KEYCODE_D_CAP 0x44  
#define KEYCODE_S_CAP 0x53  
#define KEYCODE_W_CAP 0x57  

class SmartCarKeyboardTeleopNode  
{  
  private:  
      double walk_vel_;  //线速度
      double run_vel_;   //线加速度
      double yaw_rate_;  //角速度
      double yaw_rate_run_;   //角加速度
  
      geometry_msgs::Twist cmdvel_;  
      ros::NodeHandle n_;  
      ros::Publisher pub_; 
 
  public:  
      SmartCarKeyboardTeleopNode()  
      {  
          pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);   //发布消息
          ros::NodeHandle n_private("~");  
          n_private.param("walk_vel", walk_vel_, 1.42);  //0.5
          n_private.param("run_vel", run_vel_, 2.48);    //1.0
          n_private.param("yaw_rate", yaw_rate_, 1.02);  //1.0	
          n_private.param("yaw_rate_run", yaw_rate_run_, 2.04);   //1.5
      }  

     ~SmartCarKeyboardTeleopNode(){};  
     void keyboardLoop();  

     void stopRobot()   //急停
     {  
          cmdvel_.linear.x = 0;  
          cmdvel_.angular.z =0; 

           
          pub_.publish(cmdvel_);  
     }  
};  

SmartCarKeyboardTeleopNode* tbk;  
int kfd = 0; 
struct termios cooked, raw;  
bool done;  

int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  
    SmartCarKeyboardTeleopNode tbk;  

    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));   //创建线程
    ros::spin();     //消息回调处理函数（执行到此不再继续执行下面的代码）
    t.interrupt();    //中断线程
    t.join();    //join()将阻塞调用新线程直到boost::thread所代表的线程执行完成。
    tbk.stopRobot();  
    tcsetattr(kfd,TCSANOW,&cooked);   //设置终端参数

    return(0);  
}  

void SmartCarKeyboardTeleopNode::keyboardLoop()  
{  
    char c;  
    double max_tv = walk_vel_;  
    double max_rv = yaw_rate_;  
    bool dirty = false;  
    int speed = 0;  
    int turn = 0; 

    // get the console in raw mode  
    tcgetattr(kfd, &cooked);  //从终端获取参数
    memcpy(&raw, &cooked, sizeof(struct termios));   //内存拷贝
    raw.c_lflag &=~ (ICANON | ECHO);   //c_lflag:本地模式标志,控制终端编辑功能 ICANON:使用标准输入模式 ECHO:显示输入字符
    raw.c_cc[VEOL] = 1;    //附加的End-of-file字符
    raw.c_cc[VEOF] = 2;    //End-of-file字符
    tcsetattr(kfd, TCSANOW, &raw);  

    puts("Reading from keyboard");  
    puts("Use WASD keys to control the robot");  
    puts("Press Shift to move faster");  

    struct pollfd ufd;  
    ufd.fd = kfd;    //文件描述符
    ufd.events = POLLIN;    //等待的事件(可读)

    for(;;)  
    {  
        boost::this_thread::interruption_point();  

        // get the next event from the keyboard  
        int num;  

        if ((num = poll(&ufd, 1, 250)) < 0)     //等待250ms,如果小于零代表有错误
        {  
            perror("poll():");     //将上一个函数发生错误的原因输出到标准设备,poll():错误
            return;  
        }  
        else if(num > 0)  
        {  
            if(read(kfd, &c, 1) < 0)  
            {  
                perror("read():");  
                return;  
            }  
        }  
        else  
        {  
            if (dirty == true)    //按一下动一下
            {  
                stopRobot();  
                dirty = false; 
            }  
            continue;  
        }  

        switch(c) 
        {  
            case KEYCODE_W:  
                max_tv = walk_vel_;  
                speed = 1;  
                turn = 0;  
                dirty = true;
                break;  

            case KEYCODE_S:  
                max_tv = walk_vel_;  
                speed = -1;  
                turn = 0;  
                dirty = true;  
                break;  

            case KEYCODE_A:  
                max_rv = yaw_rate_;   
                speed = 0;  
                turn = 1;  
                dirty = true;  
                break;  

            case KEYCODE_D:  
                max_rv = yaw_rate_;  
                speed = 0;  
                turn = -1;  
                dirty = true;  
                break;  

            case KEYCODE_W_CAP:  
                max_tv = run_vel_;  
                speed = 1;  
                turn = 0;  
                dirty = true;  
                break;  

            case KEYCODE_S_CAP:  
                max_tv = run_vel_;  
                speed = -1;  
                turn = 0;  
                dirty = true;  
                break;  

            case KEYCODE_A_CAP:  
                max_rv = yaw_rate_run_;  
                speed = 0;  
                turn = 1;  
                dirty = true;  
                break;  

            case KEYCODE_D_CAP:  
                max_rv = yaw_rate_run_;  
                speed = 0;  
                turn = -1;  
                dirty = true;  
                break;                

            default:  
                max_tv = walk_vel_;  
                max_rv = yaw_rate_;  
                speed = 0;  
                turn = 0;  
                dirty = false;  
        }  
        cmdvel_.linear.x = speed * max_tv;  
        cmdvel_.angular.z = turn * max_rv;
        pub_.publish(cmdvel_);  
    }  

}