#include "dual_radar_traversable_extractor/dual_radar_traversable_extractor.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "dual_radar_traversable_extractor_node");
    
    try {
        DualRadarTraversableExtractor extractor;
        
        ROS_INFO("双雷达可行区域提取节点启动");
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("节点运行异常: %s", e.what());
        return -1;
    }
    
    ROS_INFO("双雷达可行区域提取节点结束");
    return 0;
}
