#ifndef PATH_SMOOTHER_H
#define PATH_SMOOTHER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

namespace path_smoother
{

class PathSmoother
{
public:
    PathSmoother();
    ~PathSmoother();
    
    bool initialize();
    void spin();

private:
    // ROS组件
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber raw_path_sub_;
    ros::Publisher smoothed_path_pub_;
    ros::Publisher visualization_pub_;
    tf::TransformListener tf_listener_;
    
    // 参数
    std::string global_frame_;
    double smoothing_factor_;          // 平滑强度因子 (0.0-1.0)
    int smoothing_iterations_;         // 平滑迭代次数
    double neighbor_radius_;           // 邻域半径
    double min_point_distance_;        // 最小点间距（用于路径点过滤）
    double max_deviation_;             // 最大允许偏差
    bool enable_visualization_;
    bool enable_point_filtering_;      // 是否启用路径点过滤
    
    // 数据
    nav_msgs::Path last_raw_path_;
    nav_msgs::Path last_smoothed_path_;
    bool has_path_;
    
    // 回调函数
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    
    // 路径平滑核心方法
    nav_msgs::Path smoothPath(const nav_msgs::Path& raw_path);
    nav_msgs::Path weightedNeighborhoodSmooth(const nav_msgs::Path& path);
    nav_msgs::Path filterPathPoints(const nav_msgs::Path& path);
    
    // 权重计算
    double calculateWeight(double distance, double radius);
    std::vector<double> getNeighborWeights(int center_idx, 
                                         const std::vector<geometry_msgs::Point>& points);
    
    // 工具方法
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::Point& from, 
                                                   const geometry_msgs::Point& to);
    bool validateSmoothedPath(const nav_msgs::Path& original_path, 
                             const nav_msgs::Path& smoothed_path);
    
    // 可视化方法
    void publishVisualization(const nav_msgs::Path& raw_path, 
                             const nav_msgs::Path& smoothed_path);
    
    // 路径质量评估
    double calculatePathLength(const nav_msgs::Path& path);
    double calculateMaxDeviation(const nav_msgs::Path& original_path, 
                                const nav_msgs::Path& smoothed_path);
    double calculatePathSmoothness(const nav_msgs::Path& path);
};

} // namespace path_smoother

#endif // PATH_SMOOTHER_H
