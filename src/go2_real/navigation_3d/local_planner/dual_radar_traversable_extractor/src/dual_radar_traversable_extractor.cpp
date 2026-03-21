#include "dual_radar_traversable_extractor/dual_radar_traversable_extractor.h"
#include <cmath>
#include <algorithm>

DualRadarTraversableExtractor::DualRadarTraversableExtractor()
    : private_nh_("~"), lidar_transform_ready_(false), radar_transform_ready_(false),
      tf_buffer_(nullptr), tf_listener_(nullptr)
{
    // 加载参数
    loadParameters();

    // 初始化TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10.0));
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // 初始化变换矩阵
    lidar_to_base_matrix_ = Eigen::Matrix4d::Identity();
    radar_to_base_matrix_ = Eigen::Matrix4d::Identity();

    // 初始化订阅者
    lidar_sub_ = nh_.subscribe(lidar_topic_, 1,
                               &DualRadarTraversableExtractor::lidarCallback, this);

    // 根据参数决定是否订阅雷达数据
    if (use_dual_radar_)
    {
        radar_sub_ = nh_.subscribe(radar_topic_, 1,
                                   &DualRadarTraversableExtractor::radarCallback, this);
        ROS_INFO("双雷达模式启用，订阅毫米波雷达: %s", radar_topic_.c_str());
    }
    else
    {
        ROS_INFO("单雷达模式，仅使用激光雷达数据");
    }

    // 初始化发布者
    ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
    obstacle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1);
    combined_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);

    // 配置滤波器
    voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

    if (use_statistical_filter_)
    {
        stat_filter_.setMeanK(statistical_filter_k_);
        stat_filter_.setStddevMulThresh(statistical_filter_std_);
    }

    // 初始化PatchWorkpp地面分割器（参考PGO实现）
    if (use_patchworkpp_)
    {
        // 创建PatchWorkpp专用的NodeHandle，指向配置子命名空间
        ros::NodeHandle patchworkpp_nh(private_nh_, "patchworkpp");
        patchworkpp_ = std::make_shared<PatchWorkpp<PointType>>(&patchworkpp_nh);
        ROS_INFO("PatchWorkpp地面分割器初始化完成");
    }

    ROS_INFO("DualRadarTraversableExtractor 初始化完成");
    ROS_INFO("订阅激光雷达: %s", lidar_topic_.c_str());
    if (use_dual_radar_)
    {
        ROS_INFO("订阅毫米波雷达: %s", radar_topic_.c_str());
    }
    ROS_INFO("基坐标系: %s", base_frame_.c_str());
}

DualRadarTraversableExtractor::~DualRadarTraversableExtractor()
{
}

void DualRadarTraversableExtractor::loadParameters()
{
    // 话题参数
    private_nh_.param<std::string>("lidar_topic", lidar_topic_, "/cloud_registered_body_1");
    private_nh_.param<std::string>("radar_topic", radar_topic_, "/radar_points");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");

    // 双雷达模式参数
    private_nh_.param("use_dual_radar", use_dual_radar_, false); // 默认使用单雷达模式

    // PatchWorkpp地面分割参数（参考PGO配置）
    private_nh_.param("use_patchworkpp", use_patchworkpp_, true);                  // 默认使用PatchWorkpp
    private_nh_.param("refined_height_threshold", refined_height_threshold_, 0.1); // 精细高度过滤

    // 坐标系参数
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
    private_nh_.param<std::string>("radar_frame", radar_frame_, "radar");

    // 处理参数
    private_nh_.param("time_sync_threshold", time_sync_threshold_, 0.05);
    private_nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.05);
    private_nh_.param("height_threshold", height_threshold_, 0.1); // 参考PGO的0.1米
    private_nh_.param("robot_radius", robot_radius_, 0.3);
    private_nh_.param("max_range", max_range_, 10.0);

    // 滤波参数
    private_nh_.param("use_statistical_filter", use_statistical_filter_, true);
    private_nh_.param("statistical_filter_k", statistical_filter_k_, 50);
    private_nh_.param("statistical_filter_std", statistical_filter_std_, 1.0);

    // 地面点云膨胀参数
    private_nh_.param("use_ground_expansion", use_ground_expansion_, true);    // 默认启用地面膨胀
    private_nh_.param("expansion_radius", expansion_radius_, 0.5);           // 默认膨胀半径0.5米
    private_nh_.param("expansion_height", expansion_height_, 0.3);           // 默认膨胀高度范围0.3米

    // 障碍物点云聚类滤波参数
    private_nh_.param("use_obstacle_clustering", use_obstacle_clustering_, true);  // 默认启用障碍物聚类滤波
    private_nh_.param("cluster_tolerance", cluster_tolerance_, 0.1);             // 默认聚类容忍距离0.1米
    private_nh_.param("min_cluster_size", min_cluster_size_, 10);                // 默认最小聚类大小10点
    private_nh_.param("max_cluster_size", max_cluster_size_, 25000);             // 默认最大聚类大小25000点

    ROS_INFO("参数加载完成:");
    ROS_INFO("  激光雷达话题: %s", lidar_topic_.c_str());
    if (use_dual_radar_)
    {
        ROS_INFO("  毫米波雷达话题: %s", radar_topic_.c_str());
        ROS_INFO("  双雷达模式: 启用");
    }
    else
    {
        ROS_INFO("  双雷达模式: 禁用（仅使用激光雷达）");
    }
    ROS_INFO("  PatchWorkpp地面分割: %s", use_patchworkpp_ ? "启用" : "禁用");
    ROS_INFO("  体素滤波器叶子大小: %.3f", voxel_leaf_size_);
    ROS_INFO("  地面高度阈值: %.3f", height_threshold_);
    ROS_INFO("  精细高度过滤阈值: %.3f", refined_height_threshold_);
    ROS_INFO("  机器人半径: %.3f", robot_radius_);
    ROS_INFO("  最大处理距离: %.1f", max_range_);
    ROS_INFO("  地面点云膨胀: %s", use_ground_expansion_ ? "启用" : "禁用");
    if (use_ground_expansion_)
    {
        ROS_INFO("  膨胀半径: %.3f 米", expansion_radius_);
        ROS_INFO("  膨胀高度范围: %.3f 米", expansion_height_);
    }
    ROS_INFO("  障碍物点云聚类滤波: %s", use_obstacle_clustering_ ? "启用" : "禁用");
    if (use_obstacle_clustering_)
    {
        ROS_INFO("  聚类容忍距离: %.3f 米", cluster_tolerance_);
        ROS_INFO("  最小聚类大小: %d 点", min_cluster_size_);
        ROS_INFO("  最大聚类大小: %d 点", max_cluster_size_);
    }
}

void DualRadarTraversableExtractor::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_queue_.push_back(msg);
    lidar_time_queue_.push_back(msg->header.stamp.toSec());

    // 保持队列大小
    while (lidar_queue_.size() > 10)
    {
        lidar_queue_.pop_front();
        lidar_time_queue_.pop_front();
    }

    // 获取激光雷达到base_link的变换（用于地面分割）
    if (!lidar_transform_ready_)
    {
        lidar_transform_ready_ = getTransform(lidar_frame_, base_frame_, lidar_to_base_matrix_);
    }

    // 处理数据
    processData();
}

void DualRadarTraversableExtractor::radarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (!use_dual_radar_)
    {
        return; // 如果未启用双雷达模式，直接返回
    }

    std::lock_guard<std::mutex> lock(radar_mutex_);
    radar_queue_.push_back(msg);
    radar_time_queue_.push_back(msg->header.stamp.toSec());

    // 保持队列大小
    while (radar_queue_.size() > 10)
    {
        radar_queue_.pop_front();
        radar_time_queue_.pop_front();
    }

    // 获取雷达到base_link的变换
    if (!radar_transform_ready_)
    {
        radar_transform_ready_ = getTransform(radar_frame_, base_frame_, radar_to_base_matrix_);
    }
}

void DualRadarTraversableExtractor::processData()
{
    // 检查数据队列
    if (lidar_queue_.empty())
    {
        return;
    }

    // 获取最新的激光雷达数据
    sensor_msgs::PointCloud2::ConstPtr lidar_msg = lidar_queue_.back();
    double lidar_time = lidar_time_queue_.back();

    // 转换激光雷达点云
    pcl::PointCloud<PointType>::Ptr lidar_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*lidar_msg, *lidar_cloud);

    if (lidar_cloud->empty())
    {
        ROS_WARN("接收到空的激光雷达点云");
        return;
    }

    // 将激光雷达点云从laser_link变换到base_link坐标系（用于正确的地面分割）
    if (lidar_transform_ready_)
    {
        lidar_cloud = transformPointCloud(lidar_cloud, lidar_to_base_matrix_);
        ROS_DEBUG("激光雷达点云已变换到base_link坐标系");
    }
    else
    {
        ROS_WARN("激光雷达变换矩阵未就绪，跳过变换");
    }

    // 寻找时间同步的雷达数据（仅在双雷达模式下）
    pcl::PointCloud<PointType>::Ptr radar_cloud(new pcl::PointCloud<PointType>);
    bool radar_data_found = false;

    if (use_dual_radar_ && !radar_queue_.empty() && radar_transform_ready_)
    {
        // 查找时间最接近的雷达数据
        double min_time_diff = std::numeric_limits<double>::max();
        size_t best_idx = 0;

        for (size_t i = 0; i < radar_time_queue_.size(); ++i)
        {
            double time_diff = std::abs(radar_time_queue_[i] - lidar_time);
            if (time_diff < min_time_diff)
            {
                min_time_diff = time_diff;
                best_idx = i;
            }
        }

        if (min_time_diff < time_sync_threshold_)
        {
            pcl::fromROSMsg(*radar_queue_[best_idx], *radar_cloud);
            if (!radar_cloud->empty())
            {
                radar_cloud = transformPointCloud(radar_cloud, radar_to_base_matrix_);
                radar_data_found = true;
                ROS_DEBUG("找到同步的雷达数据，时间差: %.3f秒", min_time_diff);
            }
        }
    }

    // 合并点云
    pcl::PointCloud<PointType>::Ptr combined_cloud;
    if (use_dual_radar_ && radar_data_found)
    {
        combined_cloud = combinePointClouds(lidar_cloud, radar_cloud);
        ROS_DEBUG("合并点云: 激光雷达 %zu 点, 毫米波雷达 %zu 点",
                  lidar_cloud->points.size(), radar_cloud->points.size());
    }
    else
    {
        combined_cloud = lidar_cloud;
        if (use_dual_radar_)
        {
            ROS_DEBUG("未找到同步的雷达数据，仅使用激光雷达数据: %zu 点", lidar_cloud->points.size());
        }
        else
        {
            ROS_DEBUG("单雷达模式，使用激光雷达数据: %zu 点", lidar_cloud->points.size());
        }
    }

    // 预处理点云
    pcl::PointCloud<PointType>::Ptr preprocessed_cloud = preprocessPointCloud(combined_cloud);

    // 地面分割（使用PGO的简化算法）
    pcl::PointCloud<PointType>::Ptr ground_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr obstacle_cloud(new pcl::PointCloud<PointType>);

    if (!segmentGround(preprocessed_cloud, ground_cloud, obstacle_cloud))
    {
        ROS_WARN("地面分割失败");
        return;
    }

    // 地面点云膨胀（如果启用）
    if (use_ground_expansion_)
    {
        expandGroundCloud(preprocessed_cloud, ground_cloud, obstacle_cloud);
    }

    // 障碍物点云聚类滤波（如果启用）
    if (use_obstacle_clustering_)
    {
        obstacle_cloud = filterObstacleCloud(obstacle_cloud);
    }

    // 发布结果
    publishResults(combined_cloud, ground_cloud, obstacle_cloud, lidar_msg->header);
}

bool DualRadarTraversableExtractor::getTransform(const std::string &from_frame,
                                                 const std::string &to_frame,
                                                 Eigen::Matrix4d &transform_matrix)
{
    try
    {
        geometry_msgs::TransformStamped transform =
            tf_buffer_->lookupTransform(to_frame, from_frame, ros::Time(0), ros::Duration(1.0));

        // 提取平移和旋转
        Eigen::Vector3d translation(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        Eigen::Quaterniond rotation(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z);
        rotation.normalize();

        // 构造变换矩阵
        Eigen::Isometry3d tf_matrix = Eigen::Isometry3d::Identity();
        tf_matrix.linear() = rotation.toRotationMatrix();
        tf_matrix.translation() = translation;
        transform_matrix = tf_matrix.matrix();

        ROS_INFO("获取变换矩阵: %s -> %s", from_frame.c_str(), to_frame.c_str());
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("无法获取变换 %s -> %s: %s",
                 from_frame.c_str(), to_frame.c_str(), ex.what());
        return false;
    }
}

pcl::PointCloud<PointType>::Ptr
DualRadarTraversableExtractor::transformPointCloud(const pcl::PointCloud<PointType>::Ptr &cloud,
                                                   const Eigen::Matrix4d &transform)
{
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);

    for (const auto &point : cloud->points)
    {
        Eigen::Vector4d p_src(point.x, point.y, point.z, 1.0);
        Eigen::Vector4d p_dst = transform * p_src;

        PointType transformed_point;
        transformed_point.x = p_dst.x();
        transformed_point.y = p_dst.y();
        transformed_point.z = p_dst.z();
        transformed_point.intensity = point.intensity;

        transformed_cloud->points.push_back(transformed_point);
    }

    transformed_cloud->width = transformed_cloud->points.size();
    transformed_cloud->height = 1;
    transformed_cloud->is_dense = true;

    return transformed_cloud;
}

pcl::PointCloud<PointType>::Ptr
DualRadarTraversableExtractor::combinePointClouds(const pcl::PointCloud<PointType>::Ptr &lidar_cloud,
                                                  const pcl::PointCloud<PointType>::Ptr &radar_cloud)
{
    pcl::PointCloud<PointType>::Ptr combined_cloud(new pcl::PointCloud<PointType>);

    // 添加激光雷达点云
    for (const auto &point : lidar_cloud->points)
    {
        combined_cloud->points.push_back(point);
    }

    // 添加雷达点云
    for (const auto &point : radar_cloud->points)
    {
        combined_cloud->points.push_back(point);
    }

    combined_cloud->width = combined_cloud->points.size();
    combined_cloud->height = 1;
    combined_cloud->is_dense = true;

    return combined_cloud;
}

pcl::PointCloud<PointType>::Ptr
DualRadarTraversableExtractor::preprocessPointCloud(const pcl::PointCloud<PointType>::Ptr &input_cloud)
{
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

    // 距离滤波
    for (const auto &point : input_cloud->points)
    {
        double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance <= max_range_)
        {
            filtered_cloud->points.push_back(point);
        }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    if (filtered_cloud->empty())
    {
        return filtered_cloud;
    }

    // 体素滤波
    voxel_filter_.setInputCloud(filtered_cloud);
    pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>);
    voxel_filter_.filter(*downsampled_cloud);

    // 统计滤波（可选）
    if (use_statistical_filter_ && downsampled_cloud->points.size() > statistical_filter_k_)
    {
        stat_filter_.setInputCloud(downsampled_cloud);
        pcl::PointCloud<PointType>::Ptr stat_filtered_cloud(new pcl::PointCloud<PointType>);
        stat_filter_.filter(*stat_filtered_cloud);
        return stat_filtered_cloud;
    }

    return downsampled_cloud;
}

bool DualRadarTraversableExtractor::segmentGround(const pcl::PointCloud<PointType>::Ptr &input_cloud,
                                                  pcl::PointCloud<PointType>::Ptr &ground_cloud,
                                                  pcl::PointCloud<PointType>::Ptr &obstacle_cloud)
{
    if (input_cloud->empty())
    {
        return false;
    }

    // 使用PatchWorkpp进行高级地面分割（可检测台阶/楼梯）
    if (use_patchworkpp_ && patchworkpp_)
    {
        try
        {
            // 使用PatchWorkpp进行地面分割
            double time_taken = 0.0;
            patchworkpp_->estimate_ground(*input_cloud, *ground_cloud, *obstacle_cloud, time_taken);

            ROS_DEBUG("PatchWorkpp地面分割完成: 地面点 %zu, 障碍物点 %zu, 耗时 %.3f ms",
                      ground_cloud->points.size(), obstacle_cloud->points.size(), time_taken * 1000.0);

            return !ground_cloud->empty();
        }
        catch (const std::exception &e)
        {
            ROS_WARN("PatchWorkpp分割失败，回退到简单高度阈值方法: %s", e.what());
            // 回退到简单方法
        }
    }

    // 使用PGO的简化地面分割方法：基于高度阈值的分割（作为备用方案）
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    // 计算点云的高度范围
    for (const auto &point : input_cloud->points)
    {
        if (point.z < min_z)
            min_z = point.z;
        if (point.z > max_z)
            max_z = point.z;
    }

    // 如果点云高度差太小，可能都是地面点
    if (max_z - min_z < 0.1)
    {
        *ground_cloud = *input_cloud;
        obstacle_cloud->clear();
        ROS_DEBUG("点云高度差很小 (%.3f)，全部视为地面点", max_z - min_z);
        return true;
    }

    // 基于高度阈值进行分割
    double ground_threshold = min_z + height_threshold_;

    for (const auto &point : input_cloud->points)
    {
        if (point.z <= ground_threshold)
        {
            ground_cloud->points.push_back(point);
        }
        else
        {
            obstacle_cloud->points.push_back(point);
        }
    }

    // 设置点云属性
    ground_cloud->width = ground_cloud->points.size();
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    obstacle_cloud->width = obstacle_cloud->points.size();
    obstacle_cloud->height = 1;
    obstacle_cloud->is_dense = true;

    // 参考PGO，对地面点云进行进一步的高度过滤
    if (!ground_cloud->empty())
    {
        pcl::PointCloud<PointType> pc_ground_filtered;
        pcl::PointCloud<PointType> elevated_points;

        // 计算地面点云的平均高度
        double avg_height = 0.0;
        for (const auto &pt : ground_cloud->points)
        {
            avg_height += pt.z;
        }
        avg_height /= ground_cloud->points.size();

        // 相对于平均高度进行过滤
        double filter_threshold = avg_height + height_threshold_;

        // 遍历原始地面点云
        for (const auto &pt : ground_cloud->points)
        {
            if (pt.z > filter_threshold)
            {
                elevated_points.push_back(pt); // 保存需要移除的高点
            }
            else
            {
                pc_ground_filtered.push_back(pt); // 保留有效地面点
            }
        }

        // 更新地面点云（过滤后）
        ground_cloud->clear();
        ground_cloud->points = pc_ground_filtered.points;
        ground_cloud->width = ground_cloud->points.size();
        ground_cloud->height = 1;
        ground_cloud->is_dense = true;

        // 将高点合并到障碍物点云
        for (const auto &pt : elevated_points.points)
        {
            obstacle_cloud->points.push_back(pt);
        }
        obstacle_cloud->width = obstacle_cloud->points.size();

        ROS_DEBUG("高度过滤完成: 过滤掉 %zu 个高点, 平均高度 %.3f",
                  elevated_points.points.size(), avg_height);
    }

    ROS_DEBUG("地面分割完成: 地面点 %zu, 障碍物点 %zu, 地面高度阈值 %.3f",
              ground_cloud->points.size(), obstacle_cloud->points.size(), ground_threshold);

    return !ground_cloud->empty();
}

void DualRadarTraversableExtractor::publishResults(const pcl::PointCloud<PointType>::Ptr &combined_cloud,
                                                   const pcl::PointCloud<PointType>::Ptr &ground_cloud,
                                                   const pcl::PointCloud<PointType>::Ptr &obstacle_cloud,
                                                   const std_msgs::Header &header)
{
    // 创建新的header，设置frame_id为base_link（因为点云已转换到base_link坐标系）
    std_msgs::Header base_link_header = header;
    base_link_header.frame_id = base_frame_;

    // 发布合并后的点云
    if (!combined_cloud->empty() && combined_cloud_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 combined_msg;
        pcl::toROSMsg(*combined_cloud, combined_msg);
        combined_msg.header = base_link_header;
        combined_cloud_pub_.publish(combined_msg);
    }

    // 发布地面点云
    if (!ground_cloud->empty() && ground_cloud_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_cloud, ground_msg);
        ground_msg.header = base_link_header;
        ground_cloud_pub_.publish(ground_msg);
    }

    // 发布障碍物点云
    if (!obstacle_cloud->empty() && obstacle_cloud_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
        obstacle_msg.header = base_link_header;
        obstacle_cloud_pub_.publish(obstacle_msg);
    }

    ROS_DEBUG("发布完成: 合并点云 %zu 点, 地面点云 %zu 点, 障碍物点云 %zu 点",
              combined_cloud->points.size(), ground_cloud->points.size(), obstacle_cloud->points.size());
}

void DualRadarTraversableExtractor::expandGroundCloud(const pcl::PointCloud<PointType>::Ptr& original_cloud,
                                                      pcl::PointCloud<PointType>::Ptr& ground_cloud,
                                                      pcl::PointCloud<PointType>::Ptr& obstacle_cloud)
{
    if (ground_cloud->empty() || obstacle_cloud->empty())
    {
        ROS_DEBUG("地面点云或障碍物点云为空，跳过膨胀");
        return;
    }

    // 记录原始数据大小
    size_t original_ground_size = ground_cloud->points.size();
    size_t original_obstacle_size = obstacle_cloud->points.size();

    // 创建新的点云容器
    pcl::PointCloud<PointType>::Ptr expanded_ground_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr filtered_obstacle_cloud(new pcl::PointCloud<PointType>);

    // 首先将原有地面点云加入到膨胀后的地面点云中
    *expanded_ground_cloud = *ground_cloud;

    // 遍历障碍物点云中的每个点，检查是否在地面点云附近
    for (const auto& obstacle_point : obstacle_cloud->points)
    {
        bool is_near_ground = false;

        // 检查该障碍物点是否在任意地面点的膨胀范围内
        for (const auto& ground_point : ground_cloud->points)
        {
            // 计算水平距离
            double horizontal_distance = sqrt(pow(obstacle_point.x - ground_point.x, 2) + 
                                             pow(obstacle_point.y - ground_point.y, 2));
            
            // 计算垂直距离
            double vertical_distance = fabs(obstacle_point.z - ground_point.z);

            // 如果在膨胀范围内，则将该点视为地面点
            if (horizontal_distance <= expansion_radius_ && vertical_distance <= expansion_height_)
            {
                is_near_ground = true;
                break;
            }
        }

        // 根据是否接近地面来分类点
        if (is_near_ground)
        {
            expanded_ground_cloud->points.push_back(obstacle_point);
        }
        else
        {
            filtered_obstacle_cloud->points.push_back(obstacle_point);
        }
    }

    // 更新点云属性
    expanded_ground_cloud->width = expanded_ground_cloud->points.size();
    expanded_ground_cloud->height = 1;
    expanded_ground_cloud->is_dense = true;

    filtered_obstacle_cloud->width = filtered_obstacle_cloud->points.size();
    filtered_obstacle_cloud->height = 1;
    filtered_obstacle_cloud->is_dense = true;

    // 替换原有点云
    *ground_cloud = *expanded_ground_cloud;
    *obstacle_cloud = *filtered_obstacle_cloud;

    // 计算膨胀后的数据统计
    size_t expanded_points = ground_cloud->points.size() - original_ground_size;
    
    ROS_DEBUG("地面点云膨胀完成:");
    ROS_DEBUG("  原始地面点: %zu -> %zu (新增 %zu 点)",
              original_ground_size, ground_cloud->points.size(), expanded_points);
    ROS_DEBUG("  原始障碍物点: %zu -> %zu (移除 %zu 点)",
              original_obstacle_size, obstacle_cloud->points.size(), expanded_points);
    ROS_DEBUG("  膨胀参数: 半径 %.3f 米, 高度范围 %.3f 米",
              expansion_radius_, expansion_height_);
}

pcl::PointCloud<PointType>::Ptr 
DualRadarTraversableExtractor::filterObstacleCloud(const pcl::PointCloud<PointType>::Ptr& obstacle_cloud)
{
    if (obstacle_cloud->empty())
    {
        ROS_DEBUG("障碍物点云为空，跳过聚类滤波");
        return obstacle_cloud;
    }

    // 记录原始点云大小
    size_t original_size = obstacle_cloud->points.size();

    // 创建KdTree对象用于搜索
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(obstacle_cloud);

    // 创建聚类提取对象
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(cluster_tolerance_);  // 聚类容忍距离
    ec.setMinClusterSize(min_cluster_size_);     // 最小聚类大小
    ec.setMaxClusterSize(max_cluster_size_);     // 最大聚类大小
    ec.setSearchMethod(tree);
    ec.setInputCloud(obstacle_cloud);
    ec.extract(cluster_indices);

    // 创建滤波后的点云
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

    // 合并所有有效聚类
    for (const auto& cluster : cluster_indices)
    {
        for (const auto& idx : cluster.indices)
        {
            filtered_cloud->points.push_back(obstacle_cloud->points[idx]);
        }
    }

    // 设置点云属性
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    // 统计信息
    size_t filtered_size = filtered_cloud->points.size();
    size_t removed_points = original_size - filtered_size;

    ROS_DEBUG("障碍物点云聚类滤波完成:");
    ROS_DEBUG("  原始点云: %zu 点", original_size);
    ROS_DEBUG("  滤波后点云: %zu 点", filtered_size);
    ROS_DEBUG("  移除孤立点: %zu 点", removed_points);
    ROS_DEBUG("  有效聚类数: %zu 个", cluster_indices.size());
    ROS_DEBUG("  聚类参数: 容忍距离 %.3f 米, 最小大小 %d 点, 最大大小 %d 点",
              cluster_tolerance_, min_cluster_size_, max_cluster_size_);

    return filtered_cloud;
}
