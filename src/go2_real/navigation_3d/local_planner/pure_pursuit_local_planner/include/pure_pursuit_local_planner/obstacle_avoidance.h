#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <cmath>
#include <algorithm>

namespace pure_pursuit_local_planner
{

/**
 * @brief 人工势场参数结构体
 */
struct PotentialFieldParams
{
    double attractive_gain;       // 吸引力增益
    double repulsive_gain;       // 排斥力增益
    double influence_radius;     // 障碍物影响半径
    double safety_radius;        // 安全半径
    double force_limit;          // 最大力限制
    double angular_gain;         // 角速度调整增益
    double speed_factor_gain;    // 速度因子增益
    
    // Y方向避障参数
    double y_force_threshold;     // Y方向力阈值
    double y_velocity_gain;       // Y方向速度增益
    double y_smoothing_factor;    // Y方向速度平滑因子
    double y_dead_zone;           // Y方向速度死区
    
    // 角速度调整参数
    double angular_smoothing_factor;  // 角速度平滑因子
    
    // 双侧障碍物检测参数
    double bilateral_threshold;   // 双侧障碍物检测阈值
    double bilateral_damping;     // 双侧障碍物时的阻尼系数
};

/**
 * @brief 基于人工势场法的障碍物避障类
 */
class ObstacleAvoidance
{
public:
    ObstacleAvoidance();
    ~ObstacleAvoidance();
    
    /**
     * @brief 初始化障碍物避障模块
     * @param nh ROS节点句柄
     * @return 初始化是否成功
     */
    bool initialize(ros::NodeHandle& nh);
    
    /**
     * @brief 更新代价地图
     * @param costmap 代价地图数据
     */
    void updateCostmap(const nav_msgs::OccupancyGrid& costmap);
    
    /**
     * @brief 更新激光扫描数据
     * @param scan 激光扫描数据
     */
    void updateLaserScan(const sensor_msgs::LaserScan& scan);
    
    /**
     * @brief 使用人工势场法计算避障速度
     * @param original_cmd 原始控制指令
     * @param robot_pose 机器人当前位姿
     * @param target_point 目标点（前瞻点）
     * @return 修正后的控制指令
     */
    geometry_msgs::Twist computePotentialFieldAvoidance(
        const geometry_msgs::Twist& original_cmd,
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::PointStamped& target_point);
    
    /**
     * @brief 检查路径是否安全
     * @param robot_pose 机器人当前位姿
     * @param cmd_vel 控制指令
     * @param prediction_time 预测时间
     * @return 路径是否安全
     */
    bool isPathSafe(const geometry_msgs::PoseStamped& robot_pose,
                    const geometry_msgs::Twist& cmd_vel,
                    double prediction_time = 1.0);
    
    /**
     * @brief 发布势场可视化信息
     * @param robot_pose 机器人位姿
     * @param attractive_force 吸引力
     * @param repulsive_force 排斥力
     */
    void publishVisualization(const geometry_msgs::PoseStamped& robot_pose,
                             const geometry_msgs::Vector3& attractive_force,
                             const geometry_msgs::Vector3& repulsive_force);

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher force_markers_pub_;     // 势场力可视化发布器
    ros::Publisher field_markers_pub_;     // 势场网格可视化发布器
    ros::Publisher front_detection_pub_;   // 前方检测区域可视化发布器
    
    // 数据
    nav_msgs::OccupancyGrid current_costmap_;
    bool costmap_received_;
    
    sensor_msgs::LaserScan current_scan_;
    bool scan_received_;
    
    // 势场参数
    PotentialFieldParams field_params_;
    double costmap_threshold_;      // 代价地图阈值
    
    /**
     * @brief 加载参数
     */
    void loadParameters();
    
    /**
     * @brief 计算吸引力
     * @param robot_pose 机器人当前位姿
     * @param target_point 目标点
     * @return 吸引力向量
     */
    geometry_msgs::Vector3 computeAttractiveForce(
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::PointStamped& target_point);
    
    /**
     * @brief 计算排斥力
     * @param robot_pose 机器人当前位姿
     * @return 排斥力向量
     */
    geometry_msgs::Vector3 computeRepulsiveForce(
        const geometry_msgs::PoseStamped& robot_pose);
    
    /**
     * @brief 将力向量转换为速度指令
     * @param total_force 总力向量
     * @param original_cmd 原始控制指令
     * @param robot_pose 机器人当前位姿
     * @return 修正后的控制指令
     */
    geometry_msgs::Twist convertForceToVelocity(
        const geometry_msgs::Vector3& total_force,
        const geometry_msgs::Twist& original_cmd,
        const geometry_msgs::PoseStamped& robot_pose);
    
    /**
     * @brief 获取机器人当前偏航角
     * @param robot_pose 机器人位姿
     * @return 偏航角（弧度）
     */
    double getRobotYaw(const geometry_msgs::PoseStamped& robot_pose);
    
    /**
     * @brief 限制力的大小
     * @param force 力向量
     * @param max_force 最大力限制
     * @return 限制后的力向量
     */
    geometry_msgs::Vector3 limitForce(const geometry_msgs::Vector3& force, double max_force);
    
    /**
     * @brief 检测机器人两侧是否都存在障碍物
     * @param robot_pose 机器人位姿
     * @param repulsive_force 排斥力
     * @return 是否为双侧障碍物情况
     */
    bool detectBilateralObstacles(const geometry_msgs::PoseStamped& robot_pose,
                                  const geometry_msgs::Vector3& repulsive_force);
    
    /**
     * @brief 检测机器人正前方是否有障碍物
     * @param robot_pose 机器人位姿
     * @return 正前方是否有障碍物
     */
    bool checkFrontObstacle(const geometry_msgs::PoseStamped& robot_pose);
    
    /**
     * @brief 创建力向量可视化标记
     * @param robot_pose 机器人位姿
     * @param attractive_force 吸引力
     * @param repulsive_force 排斥力
     * @return 可视化标记数组
     */
    visualization_msgs::MarkerArray createForceMarkers(
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::Vector3& attractive_force,
        const geometry_msgs::Vector3& repulsive_force);
    
    /**
     * @brief 创建势场网格可视化标记
     * @param robot_pose 机器人位姿
     * @return 可视化标记数组
     */
    visualization_msgs::MarkerArray createFieldMarkers(
        const geometry_msgs::PoseStamped& robot_pose);
    
    /**
     * @brief 发布前方检测区域可视化
     * @param robot_pose 机器人位姿
     */
    void publishFrontDetectionVisualization(const geometry_msgs::PoseStamped& robot_pose);
};

} // namespace pure_pursuit_local_planner

#endif // OBSTACLE_AVOIDANCE_H
