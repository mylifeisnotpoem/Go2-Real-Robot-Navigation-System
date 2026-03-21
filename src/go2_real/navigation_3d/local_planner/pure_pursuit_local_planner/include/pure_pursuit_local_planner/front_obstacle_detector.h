#ifndef FRONT_OBSTACLE_DETECTOR_H
#define FRONT_OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <vector>
#include <cmath>
#include <algorithm>

namespace pure_pursuit_local_planner
{

// [新增] 定义障碍物类型枚举，用于决策层区分处理逻辑
enum class ObstacleType {
    NONE,   // 无障碍：正常行驶
    SMALL,  // 小障碍：尝试绕行（势场法或局部避障）
    LARGE   // 大障碍：停车等待或后退
};

class FrontObstacleDetector
{
public:
    FrontObstacleDetector();
    ~FrontObstacleDetector();

    /**
     * @brief 初始化障碍物检测器
     * @param nh ROS节点句柄
     * @param tf_buffer TF变换缓冲区
     * @return 是否初始化成功
     */
    bool initialize(ros::NodeHandle& nh, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    /**
     * @brief 更新代价地图
     * @param costmap 代价地图
     */
    void updateCostmap(const nav_msgs::OccupancyGrid& costmap);

    /**
     * @brief 检测前方障碍物并调整速度 (保留旧接口以兼容)
     * @param cmd_vel 输入速度指令
     * @param robot_pose 机器人当前位姿
     * @return 调整后的速度指令
     */
    geometry_msgs::Twist adjustVelocityForObstacles(const geometry_msgs::Twist& cmd_vel,
                                                    const geometry_msgs::PoseStamped& robot_pose);

    /**
     * @brief [新增] 核心接口：检测并返回障碍物类型
     * 用于 PurePursuitLocalPlanner 进行高级决策（停车/绕行/后退）
     * @param robot_pose 机器人当前位姿
     * @return ObstacleType (NONE, SMALL, LARGE)
     */
    ObstacleType detectObstacleType(const geometry_msgs::PoseStamped& robot_pose);

    /**
     * @brief 检查目标点落脚区域是否被占据
     * @param goal_point 目标点
     * @return true 表示目标点附近区域被占据
     */
    bool isGoalRegionOccupied(const geometry_msgs::Point& goal_point);

    /**
     * @brief [修改] 发布检测区域可视化
     * @param robot_pose 机器人当前位姿
     * @param obstacle_type 障碍物类型 (用于显示不同颜色: 绿/黄/红)
     */
    void publishVisualization(const geometry_msgs::PoseStamped& robot_pose, ObstacleType obstacle_type);

    /**
     * @brief 更新点云数据（用于楼梯模式下的直接检测）
     * @param cloud 3D点云数据
     */
    void updatePointCloud(const sensor_msgs::PointCloud2& cloud);

    /**
     * @brief 楼梯模式专用：直接用点云检测前方障碍物
     * 绕过costmap，避免楼梯场景下滤波导致检测失效
     * @param robot_pose 机器人当前位姿
     * @return ObstacleType (NONE 或 LARGE，楼梯模式下不区分大小)
     */
    ObstacleType detectObstacleFromPointCloud(const geometry_msgs::PoseStamped& robot_pose);

    /**
     * @brief 是否处于下楼梯场景（/stairs_scene = -1）
     */
    bool isDownstairsScene() const { return stairs_scene_state_ == -1; }

private:
    void stairsSceneCallback(const std_msgs::Int32::ConstPtr& msg);

    /**
     * @brief 检测指定区域内是否有障碍物
     * @param robot_pose 机器人位姿
     * @return 是否检测到障碍物
     */
    bool detectObstacleInFrontArea(const geometry_msgs::PoseStamped& robot_pose);

    /**
     * @brief 将世界坐标转换为代价地图坐标
     * @param world_x 世界坐标X
     * @param world_y 世界坐标Y
     * @param map_x 输出地图坐标X
     * @param map_y 输出地图坐标Y
     * @return 是否转换成功
     */
    bool worldToMap(double world_x, double world_y, int& map_x, int& map_y);

    /**
     * @brief 获取地图指定位置的代价值
     * @param map_x 地图坐标X
     * @param map_y 地图坐标Y
     * @return 代价值，-1表示越界
     */
    int getCostAtMapCoordinate(int map_x, int map_y);

    /**
     * @brief 生成检测区域的角点
     * @param robot_pose 机器人位姿
     * @param corners 输出的角点坐标
     */
    void generateDetectionAreaCorners(const geometry_msgs::PoseStamped& robot_pose,
                                     std::vector<std::pair<double, double>>& corners);

    /**
     * @brief 检测扇形区域内是否有障碍物
     * @param robot_pose 机器人位姿
     * @return 是否检测到障碍物
     */
    bool detectObstacleInSectorArea(const geometry_msgs::PoseStamped& robot_pose);

    /**
     * @brief 生成扇形检测区域的边界点
     * @param robot_pose 机器人位姿
     * @param sector_points 输出的扇形边界点
     */
    void generateSectorAreaPoints(const geometry_msgs::PoseStamped& robot_pose,
                                 std::vector<std::pair<double, double>>& sector_points);

    /**
     * @brief 检查点是否在扇形区域内
     * @param point_x 点的X坐标
     * @param point_y 点的Y坐标
     * @param robot_x 机器人X坐标
     * @param robot_y 机器人Y坐标
     * @param robot_yaw 机器人朝向
     * @return 是否在扇形内
     */
    bool isPointInSector(double point_x, double point_y, double robot_x, double robot_y, double robot_yaw);

    // ROS相关
    ros::NodeHandle nh_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    ros::Publisher detection_area_pub_;   // 检测区域可视化发布器
    ros::Subscriber stairs_scene_sub_;    // 上下楼梯场景订阅器（1上楼梯，-1下楼梯，0平地）

    // 代价地图
    nav_msgs::OccupancyGrid costmap_;
    bool has_costmap_;

    // 参数
    double detection_width_;     // 检测区域宽度 (m)
    double detection_length_;    // 检测区域长度 (m)
    double sector_radius_;       // 扇形半径 (m)
    double sector_angle_;        // 扇形角度 (度)
    int obstacle_threshold_;     // 障碍物阈值 (代价值)
    
    // [新增] 大障碍物判定比例阈值
    double large_obstacle_ratio_; 
    
    double linear_velocity_factor_;  // 线速度衰减因子
    double angular_velocity_factor_; // 角速度增强因子
    std::string detection_shape_;    // 检测形状 ("rectangle" 或 "sector")
    double near_goal_check_radius_;  // 近目标占据检测半径
    double near_goal_occupied_ratio_threshold_; // 近目标占据比例阈值
    
    // 调试和可视化
    bool debug_mode_;
    bool enable_visualization_;      // 是否启用可视化

    // 点云直接检测（楼梯模式）
    sensor_msgs::PointCloud2 latest_cloud_;
    bool has_pointcloud_;
    double stair_detection_range_;       // 楼梯模式检测距离 (m)，默认1.2
    double stair_detection_width_;       // 楼梯模式检测宽度 (m)，默认0.8
    double stair_detection_height_min_;  // 点云高度下限（相对base_link，m），默认0.15
    double stair_detection_height_max_;  // 点云高度上限（相对base_link，m），默认1.5
    double downstairs_detection_range_;      // 下楼梯模式检测距离 (m)
    double downstairs_detection_width_;      // 下楼梯模式检测宽度 (m)
    double downstairs_detection_height_min_; // 下楼梯模式点云高度下限 (m)
    double downstairs_detection_height_max_; // 下楼梯模式点云高度上限 (m)
    int stair_obstacle_point_threshold_; // 判定为障碍物的最少点数，默认10
    int stairs_scene_state_ = 0;         // 楼梯场景：1上楼梯，-1下楼梯，0平地
};

} // namespace pure_pursuit_local_planner

#endif // FRONT_OBSTACLE_DETECTOR_H
