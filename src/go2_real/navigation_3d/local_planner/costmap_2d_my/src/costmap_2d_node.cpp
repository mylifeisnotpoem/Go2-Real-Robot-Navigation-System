#include "local_planner/costmap_2d.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_2d_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    ROS_INFO("Starting CostMap2D node...");
    
    try
    {
        // 创建代价地图对象
        local_planner::CostMap2D costmap(nh, private_nh);
        
        // 初始化
        if (!costmap.initialize())
        {
            ROS_ERROR("Failed to initialize costmap");
            return -1;
        }
        
        ROS_INFO("CostMap2D node started successfully");
        
        // 运行ROS循环
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("CostMap2D node failed: %s", e.what());
        return -1;
    }
    
    ROS_INFO("CostMap2D node shutting down");
    return 0;
}
