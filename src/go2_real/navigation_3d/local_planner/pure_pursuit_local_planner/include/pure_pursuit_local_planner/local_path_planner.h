#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <cmath>
#include "pure_pursuit_local_planner/astar_planner.h"

namespace pure_pursuit_local_planner
{

class LocalPathPlanner
{
public:
    LocalPathPlanner();
    ~LocalPathPlanner();

    // 初始化
    bool initialize(ros::NodeHandle& nh,
                   std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    // 设置全局路径
    void setGlobalPath(const nav_msgs::Path& global_path);

    // 更新代价地图
    void updateCostmap(const nav_msgs::OccupancyGrid& costmap);

    // 获取局部路径
    nav_msgs::Path getLocalPath() const;

    // 检查是否有有效的局部路径
    bool hasValidLocalPath() const;

    // 停止规划器
    void stop();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher local_path_pub_;
    ros::Publisher goal_marker_pub_;

    // A*规划器
    std::unique_ptr<AStarPlanner> astar_planner_;

    // 线程管理
    std::thread planning_thread_;
    std::atomic<bool> running_;
    mutable std::mutex path_mutex_;
    mutable std::mutex costmap_mutex_;
    mutable std::mutex global_path_mutex_;

    // 数据
    nav_msgs::Path global_path_;
    nav_msgs::Path local_path_;
    nav_msgs::OccupancyGrid costmap_;
    bool global_path_received_;
    bool costmap_received_;
    bool local_path_valid_;
    int last_goal_index_;              // 记录上一次目标点索引，防止倒退

    // 参数
    double planning_frequency_;       // 规划频率 (Hz)
    double lookahead_distance_;      // 前瞻距离 (m)
    double min_lookahead_distance_; // 最小前瞻距离 (m)
    double max_lookahead_distance_; // 最大前瞻距离 (m)
    std::string global_frame_;       // 全局坐标系
    std::string robot_frame_;        // 机器人坐标系
    double transform_tolerance_;     // TF变换容忍度

    // MISSING MEMBERS ADDED HERE:
    // TF Buffer (needed for getRobotPose)
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Path stability members
    int path_stability_counter_;
    double path_similarity_threshold_;
    nav_msgs::Path previous_local_path_;

    // 主要函数
    void planningLoop();
    bool computeLocalPath();

    // 辅助函数
    bool getRobotPose(geometry_msgs::PoseStamped& robot_pose);
    geometry_msgs::Point findGoalPoint(const geometry_msgs::PoseStamped& robot_pose);
    int findClosestPointOnGlobalPath(const geometry_msgs::PoseStamped& robot_pose);
    bool isPointInObstacle(const geometry_msgs::Point& point);
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);

    // 可视化
    void publishGoalMarker(const geometry_msgs::Point& goal_point);

    // MISSING HELPER FUNCTIONS ADDED HERE:
    bool shouldKeepPreviousPath(const nav_msgs::Path& new_path);
    nav_msgs::Path smoothPath(const nav_msgs::Path& path);
    double calculatePathSimilarity(const nav_msgs::Path& path1, const nav_msgs::Path& path2);
    
    int findProjectionOnPath(const geometry_msgs::PoseStamped& robot_pose, const nav_msgs::Path& path);
    int findBestPathPoint(const geometry_msgs::PoseStamped& robot_pose, const nav_msgs::Path& path);
    
    double calculateDistance3D(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    double pointToLineDistance(const geometry_msgs::Point& point, 
                              const geometry_msgs::Point& line_start, 
                              const geometry_msgs::Point& line_end);
    geometry_msgs::Point projectPointOnLine(const geometry_msgs::Point& point,
                                           const geometry_msgs::Point& line_start,
                                           const geometry_msgs::Point& line_end);
};

} // namespace pure_pursuit_local_planner

#endif // LOCAL_PATH_PLANNER_H