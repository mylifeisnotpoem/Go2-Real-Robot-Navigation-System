#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <chrono>
#include <unistd.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_SPACE 0x20

class KeyboardControl
{
public:
    KeyboardControl();
    void keyLoop();
    void publishLoop();
    ~KeyboardControl() {
        running_ = false;
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }
    }

private:
    ros::NodeHandle nh_;
    double linear_x, linear_y, angular_z;
    double linear_scale, angular_scale;
    ros::Publisher twist_pub_;
    std::thread publish_thread_;
    bool running_;
};

KeyboardControl::KeyboardControl():
    linear_x(0),
    linear_y(0),
    angular_z(0)
{
    nh_.param("linear_scale", linear_scale, 0.5);
    nh_.param("angular_scale", angular_scale, 0.5);
    
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    running_ = true;
    
    // 启动发布线程
    publish_thread_ = std::thread(&KeyboardControl::publishLoop, this);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void KeyboardControl::publishLoop()
{
    ros::Rate loop_rate(50);  // 50Hz的发布频率
    
    while (running_ && ros::ok())
    {
        geometry_msgs::Twist twist;
        twist.linear.x = linear_x;
        twist.linear.y = linear_y;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = angular_z;
        
        twist_pub_.publish(twist);
        loop_rate.sleep();
    }
}

void KeyboardControl::keyLoop()
{
    char c;
    bool dirty = false;

    // 获取控制台设置
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("控制方法:");
    puts("---------------------------");
    puts("W/S : 前进/后退");
    puts("A/D : 左移/右移");
    puts("Q/E : 左转/右转");
    puts("空格 : 停止");
    puts("CTRL-C : 退出");
    puts("---------------------------");

    while (ros::ok())
    {
        // 获取下一个事件
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        linear_x = linear_y = angular_z = 0;
        
        switch(c)
        {
            case KEYCODE_W:
                linear_x = linear_scale;
                dirty = true;
                break;
            case KEYCODE_S:
                linear_x = -linear_scale;
                dirty = true;
                break;
            case KEYCODE_A:
                linear_y = linear_scale;
                dirty = true;
                break;
            case KEYCODE_D:
                linear_y = -linear_scale;
                dirty = true;
                break;
            case KEYCODE_Q:
                angular_z = angular_scale;
                dirty = true;
                break;
            case KEYCODE_E:
                angular_z = -angular_scale;
                dirty = true;
                break;
            case KEYCODE_SPACE:
                linear_x = linear_y = angular_z = 0;
                dirty = true;
                break;
        }

        if(dirty == true)
        {
            ROS_INFO("发送速度命令 - linear_x: %.2f, linear_y: %.2f, angular_z: %.2f", 
                     linear_x, linear_y, angular_z);
            dirty = false;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_control_node");
    KeyboardControl keyboard_control;

    signal(SIGINT, quit);

    keyboard_control.keyLoop();
    
    return(0);
} 