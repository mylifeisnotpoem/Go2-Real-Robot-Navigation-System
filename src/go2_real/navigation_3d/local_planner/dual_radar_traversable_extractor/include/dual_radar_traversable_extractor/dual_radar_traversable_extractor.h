#ifndef DUAL_RADAR_TRAVERSABLE_EXTRACTOR_H_
#define DUAL_RADAR_TRAVERSABLE_EXTRACTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <deque>
#include <mutex>

// 添加PatchWorkpp支持（参考PGO）
#include "ground_seg/patchworkpp/patchworkpp.hpp"

using PointType = pcl::PointXYZI;

class DualRadarTraversableExtractor {
public:
    DualRadarTraversableExtractor();
    ~DualRadarTraversableExtractor();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber lidar_sub_;       // 激光雷达点云订阅
    ros::Subscriber radar_sub_;       // 毫米波雷达点云订阅
    ros::Subscriber odom_sub_;        // 里程计订阅（可选，用于同步）
    
    // 发布者
    ros::Publisher ground_cloud_pub_;          // 地面点云
    ros::Publisher obstacle_cloud_pub_;        // 障碍物点云
    ros::Publisher combined_cloud_pub_;        // 融合点云
    
    // TF相关
    // TF转换处理
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // PatchWorkpp地面分割器（参考PGO实现）
    std::shared_ptr<PatchWorkpp<PointType>> patchworkpp_;
    
    // 数据缓存
    std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_queue_;
    std::deque<sensor_msgs::PointCloud2::ConstPtr> radar_queue_;
    std::deque<double> lidar_time_queue_;
    std::deque<double> radar_time_queue_;
    
    std::mutex lidar_mutex_;
    std::mutex radar_mutex_;
    
    // 参数
    std::string lidar_topic_;
    std::string radar_topic_;
    std::string odom_topic_;
    std::string base_frame_;
    std::string lidar_frame_;
    std::string radar_frame_;
    
    bool use_dual_radar_;             // 是否使用双雷达模式
    bool use_patchworkpp_;            // 是否使用PatchWorkpp地面分割
    double refined_height_threshold_; // 精细高度过滤阈值（参考PGO）
    double time_sync_threshold_;      // 时间同步阈值
    double voxel_leaf_size_;          // 体素滤波器叶子大小
    double height_threshold_;         // 地面高度阈值
    double robot_radius_;             // 机器人半径
    double max_range_;                // 最大处理距离
    bool use_statistical_filter_;    // 是否使用统计滤波
    int statistical_filter_k_;       // 统计滤波邻居数
    double statistical_filter_std_;  // 统计滤波标准差倍数
    
    // 障碍物点云聚类滤波参数
    bool use_obstacle_clustering_;   // 是否启用障碍物点云聚类滤波
    double cluster_tolerance_;       // 聚类容忍距离
    int min_cluster_size_;          // 最小聚类大小
    int max_cluster_size_;          // 最大聚类大小
    
    // 地面点云膨胀参数
    bool use_ground_expansion_;       // 是否启用地面点云膨胀
    double expansion_radius_;         // 膨胀半径
    double expansion_height_;         // 膨胀高度范围（上下各多少米）
    
    // 滤波器
    pcl::VoxelGrid<PointType> voxel_filter_;
    pcl::StatisticalOutlierRemoval<PointType> stat_filter_;
    
    // 变换矩阵缓存
    Eigen::Matrix4d lidar_to_base_matrix_;   // laser_link到base_link的变换（用于激光雷达点云）
    Eigen::Matrix4d radar_to_base_matrix_;   // radar frame到base_link的变换
    bool lidar_transform_ready_;             // lidar到base_link变换是否就绪
    bool radar_transform_ready_;             // radar到base_link变换是否就绪
    
    // 回调函数
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void radarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    // 主处理函数
    void processData();
    
    // 工具函数
    void loadParameters();
    bool getTransform(const std::string& from_frame, const std::string& to_frame, 
                     Eigen::Matrix4d& transform_matrix);
    
    pcl::PointCloud<PointType>::Ptr 
    transformPointCloud(const pcl::PointCloud<PointType>::Ptr& cloud,
                       const Eigen::Matrix4d& transform);
    
    pcl::PointCloud<PointType>::Ptr 
    combinePointClouds(const pcl::PointCloud<PointType>::Ptr& lidar_cloud,
                      const pcl::PointCloud<PointType>::Ptr& radar_cloud);
    
    pcl::PointCloud<PointType>::Ptr 
    preprocessPointCloud(const pcl::PointCloud<PointType>::Ptr& input_cloud);
    
    // 障碍物点云聚类滤波函数
    pcl::PointCloud<PointType>::Ptr
    filterObstacleCloud(const pcl::PointCloud<PointType>::Ptr& obstacle_cloud);
    
    bool segmentGround(const pcl::PointCloud<PointType>::Ptr& input_cloud,
                      pcl::PointCloud<PointType>::Ptr& ground_cloud,
                      pcl::PointCloud<PointType>::Ptr& obstacle_cloud);
    
    // 地面点云膨胀函数
    void expandGroundCloud(const pcl::PointCloud<PointType>::Ptr& original_cloud,
                          pcl::PointCloud<PointType>::Ptr& ground_cloud,
                          pcl::PointCloud<PointType>::Ptr& obstacle_cloud);
    
    // 发布函数
    void publishResults(const pcl::PointCloud<PointType>::Ptr& combined_cloud,
                       const pcl::PointCloud<PointType>::Ptr& ground_cloud,
                       const pcl::PointCloud<PointType>::Ptr& obstacle_cloud,
                       const std_msgs::Header& header);
};

#endif // DUAL_RADAR_TRAVERSABLE_EXTRACTOR_H_
