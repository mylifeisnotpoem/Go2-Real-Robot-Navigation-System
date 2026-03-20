/*** 
 * @Author: YYJ
 * @Date: 2025-04-24 14:57:34
 * @LastEditTime: 2025-05-20 16:37:51
 * @LastEditors: YYJ
 * @Description: 
 */
#include <ros/ros.h>
#include "pgo/pgo.h"
#include <csignal> 

volatile sig_atomic_t g_request_shutdown = 0;
// 信号处理函数
void signalHandler(int sig) {
    ROS_INFO("Received shutdown signal, saving point cloud...");    
    g_request_shutdown = 1;
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pgo_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    PGO pgo(nh, private_nh);

    // 注册信号处理函数
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    ros::Rate rate(100);
    while (ros::ok() && !g_request_shutdown)
    {
        /* code */
        ros::spinOnce();

        pgo.Run();
        
        rate.sleep();
    }
    if(g_request_shutdown) {
        pgo.saveOptimizedCloud();
        ROS_INFO("Point cloud saved successfully.");
    } else {
        ROS_INFO("Node terminated without shutdown signal.");
    }
    ros::shutdown();
    return 0;
}