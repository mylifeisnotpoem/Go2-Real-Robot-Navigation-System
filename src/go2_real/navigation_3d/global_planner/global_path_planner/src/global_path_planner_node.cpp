#include "global_path_planner/global_path_planner.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    
    ros::init(argc, argv, "global_path_planner_node");
    
    global_path_planner::GlobalPathPlanner planner;
    
    if (!planner.initialize())
    {
        ROS_ERROR("全局路径规划器初始化失败");
        return -1;
    }
    
    ROS_INFO("全局路径规划器节点启动");
    planner.spin();
    
    return 0;
}
