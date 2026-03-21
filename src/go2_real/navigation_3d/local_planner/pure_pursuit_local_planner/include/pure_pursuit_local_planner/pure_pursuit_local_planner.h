#ifndef PURE_PURSUIT_LOCAL_PLANNER_H
#define PURE_PURSUIT_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <dynamic_reconfigure/server.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>

#include "pure_pursuit_local_planner/obstacle_avoidance.h"
#include "pure_pursuit_local_planner/path_tracker.h"
#include "pure_pursuit_local_planner/local_path_planner.h"
#include "pure_pursuit_local_planner/front_obstacle_detector.h"

namespace pure_pursuit_local_planner
{

/**
 * @brief 导航状态枚举
 */
enum class NavigationState
{
    NORMAL,                       // 正常导航状态
    SMALL_OBSTACLE_AVOIDING,      // 小障碍物避障状态（势场避障绕行）
    LARGE_OBSTACLE_REPLANNING,     // 大障碍物重规划状态（A*重规划绕行）
    LARGE_OBSTACLE_STOPPED,        // 大障碍物停止状态（等待障碍物移除）
    NAVIGATION_FAILED             // 导航失败状态（无法通过大障碍物）
};

struct PlannerConfig
{
    // Pure Pursuit parameters
    double lookahead_distance_min;      // 最小前瞻距离
    double lookahead_distance_max;      // 最大前瞻距离
    double goal_tolerance;              // 目标点容忍距离
    
    // Speed parameters
    double max_linear_velocity;         // 最大线速度 (X方向)
    double max_linear_velocity_y;       // Y方向最大线速度 (全向机器人)
    double max_angular_velocity;        // 最大角速度
    double min_linear_velocity;         // 最小线速度
    double acceleration_limit;          // 加速度限制
    double deceleration_limit;          // 减速度限制
    
    // Obstacle avoidance parameters
    double obstacle_detection_range;    // 障碍物检测范围
    double safety_distance;             // 安全距离
    double avoidance_weight;            // 避障权重
    double path_following_weight;       // 路径跟踪权重
    
    // Control parameters
    double control_frequency;           // 控制频率
    double transform_tolerance;         // TF变换容忍时间

    // Orientation adjustment parameters
    double yaw_tolerance;               // 角度容忍度
    double yaw_adjustment_gain;         // 角度调整增益
    
    // Local planning parameters
    bool enable_local_replanning;       // 是否启用A*局部重规划
    bool use_smoothed_path;             // 是否使用平滑后的路径

    // Dynamic obstacle avoidance parameters
    int large_obstacle_failure_threshold;  // 大障碍物失败阈值（计数达到此值则停止）
    double recovery_check_duration;        // 恢复检测持续时间（检测不到大障碍物的时间）
    double large_obstacle_reset_timeout;   // 大障碍物状态复位超时时间（防止不同地点障碍物累加）
    bool enable_dynamic_obstacle_avoidance; // 是否启用动态障碍物避障功能

    // Near-goal safety stop parameters
    bool near_goal_obstacle_stop_enable;   // 是否启用近目标大障碍物急停
    double near_goal_obstacle_stop_distance; // 近目标急停触发距离

    // Frame IDs
    std::string global_frame;           // 全局坐标系
    std::string robot_frame;            // 机器人坐标系
};

class PurePursuitLocalPlanner
{
public:
    PurePursuitLocalPlanner();
    ~PurePursuitLocalPlanner();
    
    /**
     * @brief 初始化局部规划器
     * @param nh ROS节点句柄
     * @return 初始化是否成功
     */
    bool initialize(ros::NodeHandle& nh);
    
    /**
     * @brief 设置全局路径
     * @param global_path 全局路径
     * @return 设置是否成功
     */
    bool setGlobalPath(const nav_msgs::Path& global_path);
    
    /**
     * @brief 计算控制指令
     * @param cmd_vel 输出的控制指令
     * @return 计算是否成功
     */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    
    /**
     * @brief 判断是否到达目标点
     * @return 是否到达目标点
     */
    bool isGoalReached();
    
    /**
     * @brief 获取机器人在全局坐标系下的位姿
     * @param robot_pose 输出的机器人位姿
     * @return 获取是否成功
     */
    bool getRobotPose(geometry_msgs::PoseStamped& robot_pose);

private:
    ros::Subscriber emergency_stop_sub_;
    bool emergency_stop_active_ = false;
    ros::Subscriber stairs_flag_sub_;      // 楼梯检测标志订阅器
    bool is_on_stairs_ = false;          // 是否正在爬楼梯
    ros::Time last_stair_state_change_;    // 最后一次楼梯状态改变的时间

    ros::Subscriber pointcloud_sub_;       // 点云订阅器（楼梯模式用）

    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher lookahead_pub_;
    ros::Publisher path_pub_;
    ros::Publisher local_trajectory_pub_;    // 局部轨迹发布器
    ros::Publisher velocity_arrow_pub_;      // 速度箭头发布器
    ros::Publisher nav_state_pub_;           // 导航状态发布器
    ros::Subscriber global_path_sub_;
    ros::Subscriber clicked_point_sub_;
    ros::Subscriber costmap_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber smoothed_local_path_sub_;  // 新增：平滑后局部路径订阅器
    ros::Subscriber target_points_sub_;        // 新增：目标点订阅器
    ros::Timer control_timer_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 配置参数
    PlannerConfig config_;
    
    // 路径和状态
    nav_msgs::Path global_path_;
    nav_msgs::Path smoothed_local_path_;                 // 新增：平滑后的局部路径
    bool smoothed_local_path_received_;                  // 新增：平滑后局部路径接收标志
    bool path_received_;
    bool goal_reached_;
    bool yaw_adjusting_;                                 // 新增：是否正在调整朝向
    geometry_msgs::Pose target_pose_;                    // 新增：目标位姿
    bool target_pose_received_;                          // 新增：目标位姿接收标志
    int current_path_index_;
    size_t path_progress_index_;                         // 路径进度索引，防止回头
    geometry_msgs::PointStamped last_lookahead_point_;  // 上一次的前瞻点
    bool has_valid_lookahead_;                           // 是否有有效的前瞻点
    geometry_msgs::Point near_goal_reference_point_;    // 近目标急停参考点（来自/clicked_point或/target_points）
    bool near_goal_reference_received_;                  // 近目标急停参考点是否可用

    // 动态障碍物避障状态
    NavigationState nav_state_;                          // 当前导航状态
    int large_obstacle_failure_count_;                   // 大障碍物失败计数器
    int large_obstacle_replan_count_;                  // 大障碍物重规划计数器
    ros::Time last_large_obstacle_time_;                // 最后一次检测到大障碍物的时间
    ros::Time last_state_change_time_;                   // 最后一次状态改变的时间

    // 组件
    std::unique_ptr<ObstacleAvoidance> obstacle_avoidance_;
    std::unique_ptr<PathTracker> path_tracker_;
    std::unique_ptr<LocalPathPlanner> local_path_planner_;  // 新增：局部路径规划器
    std::unique_ptr<FrontObstacleDetector> front_obstacle_detector_;  // 新增：前方障碍物检测器
    
    // 控制相关
    geometry_msgs::Twist last_cmd_vel_;
    ros::Time last_control_time_;
    
    /**
     * @brief 加载参数
     */
    void loadParameters();
    
    /**
     * @brief 控制回调函数
     */
    void controlCallback(const ros::TimerEvent& event);
    
    /**
     * @brief 全局路径回调函数
     */
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);

    /**
     * @brief clicked_point 回调函数（用于近目标急停参考点）
     */
    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    
    /**
     * @brief 代价地图回调函数
     */
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    
    /**
     * @brief 激光雷达回调函数
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief emergency_stop回调函数
     */
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);

    /**
     * @brief 楼梯检测标志回调函数
     */
    void stairsFlagCallback(const std_msgs::Bool::ConstPtr& msg);

        /**
         * @brief 点云回调函数（转发给前方障碍物检测器）
         */
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    /**
     * @brief 平滑后局部路径回调函数
     */
    void smoothedLocalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    
    /**
     * @brief 目标点回调函数
     */
    void targetPointsCallback(const geometry_msgs::Pose::ConstPtr& msg);
    
    /**
     * @brief 计算前瞻点
     * @param robot_pose 机器人当前位姿
     * @param lookahead_point 输出的前瞻点
     * @return 计算是否成功
     */
    bool computeLookaheadPoint(const geometry_msgs::PoseStamped& robot_pose,
                               geometry_msgs::PointStamped& lookahead_point);
    
    /**
     * @brief 计算Pure Pursuit控制指令
     * @param robot_pose 机器人当前位姿
     * @param lookahead_point 前瞻点
     * @param cmd_vel 输出的控制指令
     * @return 计算是否成功
     */
    bool computePurePursuitCommand(const geometry_msgs::PoseStamped& robot_pose,
                                   const geometry_msgs::PointStamped& lookahead_point,
                                   geometry_msgs::Twist& cmd_vel);
    
    /**
     * @brief 应用速度限制
     * @param cmd_vel 控制指令
     */
    void applyVelocityLimits(geometry_msgs::Twist& cmd_vel);
    
    /**
     * @brief 发布可视化信息
     */
    void publishVisualization(const geometry_msgs::PoseStamped& robot_pose,
                              const geometry_msgs::PointStamped& lookahead_point);
    
    /**
     * @brief 发布局部轨迹可视化
     */
    void publishLocalTrajectory(const geometry_msgs::PoseStamped& robot_pose,
                                const geometry_msgs::Twist& cmd_vel);
    
    /**
     * @brief 发布速度箭头可视化
     */
    void publishVelocityArrow(const geometry_msgs::PoseStamped& robot_pose,
                              const geometry_msgs::Twist& cmd_vel);
    
    /**
     * @brief 生成局部轨迹预测路径
     */
    nav_msgs::Path generateLocalTrajectory(const geometry_msgs::PoseStamped& robot_pose,
                                           const geometry_msgs::Twist& cmd_vel,
                                           double prediction_time = 2.0,
                                           double dt = 0.1);
    
    /**
     * @brief 计算两点间距离
     */
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    
    /**
     * @brief 标准化角度到[-π, π]
     */
    double normalizeAngle(double angle);
    
    /**
     * @brief 计算圆与线段的交点
     * @param circle_x 圆心x坐标
     * @param circle_y 圆心y坐标
     * @param radius 圆半径
     * @param line_start 线段起点
     * @param line_end 线段终点
     * @return 交点列表
     */
    std::vector<geometry_msgs::Point> computeCircleLineIntersection(
        double circle_x, double circle_y, double radius,
        const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end);
    
    /**
     * @brief 计算点在路径上的进度
     * @param segment_index 线段索引
     * @param point 点坐标
     * @return 累积路径长度
     */
    double calculatePathProgress(int segment_index, const geometry_msgs::Point& point);
    
    /**
     * @brief 更新路径进度索引（防止回头）
     * @param robot_pose 机器人当前位姿
     */
    void updatePathProgress(const geometry_msgs::PoseStamped& robot_pose);
    
    /**
     * @brief 检查是否已达到目标朝向
     * @param robot_pose 机器人当前位姿
     * @return 是否达到目标朝向
     */
    bool isTargetOrientationReached(const geometry_msgs::PoseStamped& robot_pose);
    
    /**
     * @brief 计算朝向调整控制指令
     * @param robot_pose 机器人当前位姿
     * @param cmd_vel 输出的控制指令
     * @return 计算是否成功
     */
    bool computeOrientationAdjustmentCommand(const geometry_msgs::PoseStamped& robot_pose,
                                             geometry_msgs::Twist& cmd_vel);

    /**
     * @brief 更新导航状态
     * @param new_state 新的导航状态
     */
    void updateNavigationState(NavigationState new_state);

    /**
     * @brief 尝试绕开大障碍物（A*重规划）
     * @return 重规划是否成功
     */
    bool attemptReplanAroundLargeObstacle();

    /**
     * @brief 处理动态障碍物避障逻辑
     * @param robot_pose 机器人当前位姿
     * @param cmd_vel 输出的控制指令
     * @return 是否需要停止执行导航（返回true表示停止）
     */
    bool handleDynamicObstacleAvoidance(const geometry_msgs::PoseStamped& robot_pose,
                                       geometry_msgs::Twist& cmd_vel);
};

} // namespace pure_pursuit_local_planner

#endif // PURE_PURSUIT_LOCAL_PLANNER_H
