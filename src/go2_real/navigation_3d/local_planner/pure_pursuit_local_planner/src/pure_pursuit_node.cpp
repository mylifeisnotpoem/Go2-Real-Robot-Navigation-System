#include "pure_pursuit_local_planner/pure_pursuit_local_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_pursuit_local_planner_node");
    ros::NodeHandle nh("~");
    
    pure_pursuit_local_planner::PurePursuitLocalPlanner planner;
    
    if (!planner.initialize(nh))
    {
        ROS_ERROR("Failed to initialize Pure Pursuit Local Planner");
        return -1;
    }
    
    ROS_INFO("Pure Pursuit Local Planner initialized successfully");
    
    ros::spin();
    
    return 0;
}
