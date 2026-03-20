#ifndef LOCAL_PLANNER_COSTMAP_2D_H
#define LOCAL_PLANNER_COSTMAP_2D_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <vector>
#include <memory>
#include <mutex>
#include <Eigen/Dense>

namespace local_planner
{

// 代价值定义
static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

class CostMap2D
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param private_nh 私有节点句柄
     */
    CostMap2D(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    /**
     * @brief 析构函数
     */
    ~CostMap2D();

    /**
     * @brief 初始化代价地图
     * @return 是否成功初始化
     */
    bool initialize();

    /**
     * @brief 更新代价地图
     */
    void updateCostMap();

    /**
     * @brief 发布代价地图
     */
    void publishCostMap();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher costmap_pub_;
    
    // TF相关
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 激光数据回调
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    
    // 参数
    std::string global_frame_;      // 全局坐标系
    std::string robot_base_frame_;  // 机器人基座坐标系
    std::string scan_topic_;        // 激光话题名
    
    // 代价地图参数
    double resolution_;             // 分辨率 (m/pixel)
    int width_;                     // 宽度 (像素)
    int height_;                    // 高度 (像素)
    double origin_x_;               // 原点x坐标
    double origin_y_;               // 原点y坐标
    double origin_z_;               // 原点z坐标
    
    // 障碍物参数
    double max_obstacle_range_;     // 最大障碍物检测距离
    double min_obstacle_range_;     // 最小障碍物检测距离
    double inflation_radius_;      // 膨胀半径
    
    // 时间衰减参数
    double obstacle_max_age_;       // 障碍物最大存在时间 (秒)
    double decay_rate_;             // 衰减速率 (1/秒)
    bool use_time_decay_;           // 是否使用时间衰减
    
    // 俯仰角检测参数
    bool enable_pitch_detection_;   // 是否启用俯仰角检测
    double max_pitch_angle_;        // 最大俯仰角 (度)
    
    // 俯仰角状态管理
    bool pitch_exceeded_state_;     // 当前俯仰角超限状态
    ros::Time last_normal_pitch_time_;  // 最后一次俯仰角正常的时间
    double pitch_recovery_time_;    // 俯仰角恢复正常的等待时间 (秒)
    
    // 代价地图数据
    std::vector<unsigned char> costmap_;
    std::vector<ros::Time> obstacle_timestamps_;  // 障碍物时间戳
    nav_msgs::OccupancyGrid costmap_msg_;
    
    // 互斥锁
    std::mutex costmap_mutex_;
    
    // 辅助函数
    
    /**
     * @brief 初始化代价地图数据
     */
    void initializeCostMap();
    
    /**
     * @brief 将激光数据转换为点云
     * @param scan 激光数据
     * @param cloud 输出点云
     */
    void scanToPointCloud(const sensor_msgs::LaserScan& scan, 
                         pcl::PointCloud<pcl::PointXYZ>& cloud);
    
    /**
     * @brief 将点云转换到地图坐标系
     * @param cloud_in 输入点云
     * @param cloud_out 输出点云
     * @param target_frame 目标坐标系
     * @param stamp 时间戳
     */
    bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                           pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                           const std::string& target_frame,
                           const ros::Time& stamp);
    
    /**
     * @brief 清除代价地图 (简单清除)
     */
    void clearCostMap();
    
    /**
     * @brief 使用时间衰减机制清理过时障碍物
     */
    void decayObstacles();
    
    /**
     * @brief 射线追踪清理自由空间 (基于激光扫描数据)
     * @param scan 激光扫描数据
     * @param robot_x 机器人x坐标
     * @param robot_y 机器人y坐标
     */
    void raytraceFreespace(const sensor_msgs::LaserScan& scan, double robot_x, double robot_y);
    
    /**
     * @brief 清理射线路径上的障碍物
     * @param ray_cells 射线经过的网格单元
     * @param obstacle_detected 是否检测到障碍物
     */
    void clearRayPath(const std::vector<std::pair<int, int>>& ray_cells, bool obstacle_detected);
    
    /**
     * @brief Bresenham直线算法用于射线追踪
     * @param x0 起点x坐标
     * @param y0 起点y坐标  
     * @param x1 终点x坐标
     * @param y1 终点y坐标
     * @param cells 输出的网格单元列表
     */
    void bresenhamLine(int x0, int y0, int x1, int y1, 
                      std::vector<std::pair<int, int>>& cells);
    
    /**
     * @brief 在代价地图上标记障碍物
     * @param cloud 点云数据
     */
    void markObstacles(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    
    /**
     * @brief 膨胀障碍物
     */
    void inflateObstacles();
    
    /**
     * @brief 世界坐标转换为地图坐标
     * @param wx 世界x坐标
     * @param wy 世界y坐标
     * @param mx 地图x坐标 (输出)
     * @param my 地图y坐标 (输出)
     * @return 是否在地图范围内
     */
    bool worldToMap(double wx, double wy, int& mx, int& my);
    
    /**
     * @brief 地图坐标转换为世界坐标
     * @param mx 地图x坐标
     * @param my 地图y坐标
     * @param wx 世界x坐标 (输出)
     * @param wy 世界y坐标 (输出)
     */
    void mapToWorld(int mx, int my, double& wx, double& wy);
    
    /**
     * @brief 设置代价值
     * @param mx 地图x坐标
     * @param my 地图y坐标
     * @param cost 代价值
     */
    void setCost(int mx, int my, unsigned char cost);
    
    /**
     * @brief 获取代价值
     * @param mx 地图x坐标
     * @param my 地图y坐标
     * @return 代价值
     */
    unsigned char getCost(int mx, int my);
    
    /**
     * @brief 获取机器人当前位置
     * @param x 机器人x坐标 (输出)
     * @param y 机器人y坐标 (输出)
     * @param z 机器人z坐标 (输出)
     * @param yaw 机器人朝向 (输出)
     * @return 是否成功获取位置
     */
    bool getRobotPose(double& x, double& y, double& z, double& yaw);
    
    /**
     * @brief 更新代价地图原点位置（以机器人为中心）
     */
    void updateMapOrigin();
    
    /**
     * @brief 滚动地图数据
     * @param offset_x x方向偏移量（像素）
     * @param offset_y y方向偏移量（像素）
     */
    void rollMap(int offset_x, int offset_y);
    
    /**
     * @brief 计算两点间的距离
     * @param x1 点1的x坐标
     * @param y1 点1的y坐标
     * @param x2 点2的x坐标
     * @param y2 点2的y坐标
     * @return 距离
     */
    double distance(double x1, double y1, double x2, double y2);
    
    /**
     * @brief 检查机器人俯仰角是否超过阈值
     * @return true如果俯仰角超过阈值，false否则
     */
    bool isPitchAngleExceeded();
    
    /**
     * @brief 加载参数
     */
    void loadParameters();
};

} // namespace local_planner

#endif // LOCAL_PLANNER_COSTMAP_2D_H
