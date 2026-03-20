#include "path_smoother/path_smoother.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "path_smoother_node");
    
    ROS_INFO("启动路径平滑节点...");
    
    try
    {
        path_smoother::PathSmoother smoother;
        
        if (!smoother.initialize())
        {
            ROS_ERROR("路径平滑器初始化失败");
            return -1;
        }
        
        ROS_INFO("路径平滑节点启动成功");
        smoother.spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("路径平滑节点运行异常: %s", e.what());
        return -1;
    }
    
    ROS_INFO("路径平滑节点已退出");
    return 0;
}
