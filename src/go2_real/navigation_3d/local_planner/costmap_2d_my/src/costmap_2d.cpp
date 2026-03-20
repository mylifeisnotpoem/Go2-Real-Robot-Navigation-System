#include "local_planner/costmap_2d.h"
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>

namespace local_planner
{

CostMap2D::CostMap2D(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), tf_listener_(tf_buffer_),
      pitch_exceeded_state_(false), last_normal_pitch_time_(ros::Time::now())
{
    // 加载参数
    loadParameters();
    
    ROS_INFO("CostMap2D initialized with parameters:");
    ROS_INFO("  Resolution: %.2f m/pixel", resolution_);
    ROS_INFO("  Size: %d x %d pixels", width_, height_);
    ROS_INFO("  Max obstacle range: %.2f m", max_obstacle_range_);
    ROS_INFO("  Inflation radius: %.2f m", inflation_radius_);
    
    if (enable_pitch_detection_)
    {
        ROS_INFO("  Pitch detection: ENABLED (max angle: %.1f degrees, recovery time: %.1f seconds)", 
                 max_pitch_angle_, pitch_recovery_time_);
    }
    else
    {
        ROS_INFO("  Pitch detection: DISABLED");
    }
}

CostMap2D::~CostMap2D()
{
}

bool CostMap2D::initialize()
{
    // 初始化订阅器和发布器
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &CostMap2D::scanCallback, this);
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
    
    // 初始化代价地图
    initializeCostMap();
    
    ROS_INFO("CostMap2D node initialized successfully");
    ROS_INFO("Subscribed to: %s", scan_topic_.c_str());
    ROS_INFO("Publishing costmap to: costmap");
    
    return true;
}

void CostMap2D::loadParameters()
{
    // 坐标系参数
    private_nh_.param<std::string>("global_frame", global_frame_, "map");
    private_nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
    private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    
    // 地图参数
    private_nh_.param<double>("resolution", resolution_, 0.05);  // 5cm分辨率
    private_nh_.param<int>("width", width_, 400);               // 20m x 20m @ 5cm = 400x400
    private_nh_.param<int>("height", height_, 400);
    
    // 障碍物检测参数
    private_nh_.param<double>("max_obstacle_range", max_obstacle_range_, 10.0);
    private_nh_.param<double>("min_obstacle_range", min_obstacle_range_, 0.1);
    private_nh_.param<double>("inflation_radius", inflation_radius_, 0.5);
    
    // 时间衰减参数
    private_nh_.param<double>("obstacle_max_age", obstacle_max_age_, 1.0);        // 障碍物最大存在2秒
    private_nh_.param<double>("decay_rate", decay_rate_, 0.80);                   // 每秒衰减50%
    private_nh_.param<bool>("use_time_decay", use_time_decay_, false);             // 默认启用时间衰减
    
    // 俯仰角检测参数
    private_nh_.param<bool>("enable_pitch_detection", enable_pitch_detection_, true);  // 是否启用俯仰角检测
    private_nh_.param<double>("max_pitch_angle", max_pitch_angle_, 20.0);               // 最大俯仰角度数
    private_nh_.param<double>("pitch_recovery_time", pitch_recovery_time_, 2.0);       // 俯仰角恢复等待时间（秒）
}

void CostMap2D::initializeCostMap()
{
    // 分配内存
    costmap_.resize(width_ * height_);
    obstacle_timestamps_.resize(width_ * height_);  // 初始化时间戳数组
    
    // 初始化为自由空间
    std::fill(costmap_.begin(), costmap_.end(), FREE_SPACE);
    
    // 初始化时间戳为当前时间
    ros::Time current_time = ros::Time::now();
    std::fill(obstacle_timestamps_.begin(), obstacle_timestamps_.end(), current_time);
    
    // 初始化OccupancyGrid消息
    costmap_msg_.header.frame_id = global_frame_;
    costmap_msg_.info.resolution = resolution_;
    costmap_msg_.info.width = width_;
    costmap_msg_.info.height = height_;
    costmap_msg_.data.resize(width_ * height_);
    
    // 初始化原点（稍后会根据机器人位置更新）
    origin_x_ = -(width_ * resolution_) / 2.0;
    origin_y_ = -(height_ * resolution_) / 2.0;
    origin_z_ = 0.0;  // 初始化为0，在第一次updateMapOrigin时会更新为机器人的实际Z坐标
    
    costmap_msg_.info.origin.position.x = origin_x_;
    costmap_msg_.info.origin.position.y = origin_y_;
    costmap_msg_.info.origin.position.z = origin_z_;
    costmap_msg_.info.origin.orientation.w = 1.0;
    
    ROS_INFO("Costmap initialized: %dx%d @ %.2fm resolution", width_, height_, resolution_);
}

void CostMap2D::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    
    // 更新地图原点（以机器人为中心）
    updateMapOrigin();
    
    // 检查俯仰角是否超过阈值
    bool pitch_exceeded = isPitchAngleExceeded();
    
    // 获取机器人位置
    double robot_x, robot_y, robot_z, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_z, robot_yaw))
    {
        ROS_WARN_THROTTLE(1.0, "Failed to get robot pose for scan processing");
        return;
    }
    
    if (pitch_exceeded)
    {
        // 俯仰角超过阈值，清空代价地图中的所有障碍物
        clearCostMap();
        ROS_DEBUG("Pitch angle exceeded, costmap cleared");
    }
    else
    {
        // 俯仰角正常，进行正常的障碍物处理
        // 步骤1: 先进行射线追踪清理（重要：在标记新障碍物之前清理）
        raytraceFreespace(*scan_msg, robot_x, robot_y);
        
        // 步骤2: 将激光数据转换为点云并标记新的障碍物
        pcl::PointCloud<pcl::PointXYZ> scan_cloud;
        scanToPointCloud(*scan_msg, scan_cloud);
        
        // 变换到全局坐标系
        pcl::PointCloud<pcl::PointXYZ> global_cloud;
        if (transformPointCloud(scan_cloud, global_cloud, global_frame_, scan_msg->header.stamp))
        {
            // 标记新检测到的障碍物
            markObstacles(global_cloud);
            
            // 步骤3: 膨胀障碍物
            inflateObstacles();
            
            // 步骤4: 使用时间衰减机制清理过时障碍物（可选，通常射线清理就够了）
            if (use_time_decay_) {
                decayObstacles();
            }
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "Failed to transform scan to global frame");
        }
    }
    
    // 无论俯仰角是否超过阈值，都要发布代价地图
    publishCostMap();
}

void CostMap2D::scanToPointCloud(const sensor_msgs::LaserScan& scan, 
                                pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    cloud.clear();
    cloud.header.frame_id = scan.header.frame_id;
    pcl_conversions::toPCL(scan.header.stamp, cloud.header.stamp);
    
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        float range = scan.ranges[i];
        
        // 过滤无效数据
        if (range < scan.range_min || range > scan.range_max || 
            range < min_obstacle_range_ || range > max_obstacle_range_)
        {
            continue;
        }
        
        // 计算点的坐标
        float angle = scan.angle_min + i * scan.angle_increment;
        pcl::PointXYZ point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        point.z = 0.0;  // 激光扫描在传感器坐标系的z=0平面
        
        cloud.points.push_back(point);
    }
    
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
}

bool CostMap2D::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                                   pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                                   const std::string& target_frame,
                                   const ros::Time& stamp)
{
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            target_frame, cloud_in.header.frame_id, stamp, ros::Duration(0.1));
        
        cloud_out.clear();
        cloud_out.header.frame_id = target_frame;
        cloud_out.header.stamp = cloud_in.header.stamp;
        
        for (const auto& point : cloud_in.points)
        {
            geometry_msgs::PointStamped point_in, point_out;
            point_in.header.frame_id = cloud_in.header.frame_id;
            point_in.point.x = point.x;
            point_in.point.y = point.y;
            point_in.point.z = point.z;
            
            tf2::doTransform(point_in, point_out, transform);
            
            pcl::PointXYZ transformed_point;
            transformed_point.x = point_out.point.x;
            transformed_point.y = point_out.point.y;
            transformed_point.z = point_out.point.z;
            
            cloud_out.points.push_back(transformed_point);
        }
        
        cloud_out.width = cloud_out.points.size();
        cloud_out.height = 1;
        cloud_out.is_dense = true;
        
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Transform failed: %s", ex.what());
        return false;
    }
}

void CostMap2D::clearCostMap()
{
    std::fill(costmap_.begin(), costmap_.end(), FREE_SPACE);
}

void CostMap2D::markObstacles(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    ros::Time current_time = ros::Time::now();
    
    for (const auto& point : cloud.points)
    {
        int mx, my;
        if (worldToMap(point.x, point.y, mx, my))
        {
            int index = my * width_ + mx;
            setCost(mx, my, LETHAL_OBSTACLE);
            
            // 更新障碍物时间戳
            if (index >= 0 && index < static_cast<int>(obstacle_timestamps_.size())) {
                obstacle_timestamps_[index] = current_time;
            }
        }
    }
}

void CostMap2D::inflateObstacles()
{
    if (inflation_radius_ <= 0)
        return;
    
    // 计算膨胀半径（像素）
    int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    
    // 复制原始代价地图
    std::vector<unsigned char> original_costmap = costmap_;
    
    // 对每个致命障碍物进行膨胀
    for (int mx = 0; mx < width_; ++mx)
    {
        for (int my = 0; my < height_; ++my)
        {
            if (original_costmap[my * width_ + mx] == LETHAL_OBSTACLE)
            {
                // 在该点周围进行膨胀
                for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
                {
                    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
                    {
                        int nx = mx + dx;
                        int ny = my + dy;
                        
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_)
                        {
                            double dist = distance(mx, my, nx, ny) * resolution_;
                            if (dist <= inflation_radius_)
                            {
                                unsigned char current_cost = getCost(nx, ny);
                                if (current_cost != LETHAL_OBSTACLE)
                                {
                                    // 计算连续的梯度代价值
                                    // 距离越近，代价值越高
                                    double normalized_dist = dist / inflation_radius_;  // 0.0 到 1.0
                                    unsigned char gradient_cost = static_cast<unsigned char>(
                                        LETHAL_OBSTACLE - normalized_dist * (LETHAL_OBSTACLE - FREE_SPACE));
                                    
                                    // 确保代价值在合理范围内
                                    gradient_cost = std::max(static_cast<unsigned char>(FREE_SPACE + 1), gradient_cost);
                                    gradient_cost = std::min(static_cast<unsigned char>(LETHAL_OBSTACLE - 1), gradient_cost);
                                    
                                    // 只在代价值更高时更新
                                    if (gradient_cost > current_cost)
                                    {
                                        setCost(nx, ny, gradient_cost);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void CostMap2D::updateMapOrigin()
{
    // 滚动窗口式局部代价地图
    double robot_x, robot_y, robot_z, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_z, robot_yaw))
        return;
    
    // 计算新的地图原点
    double new_origin_x = robot_x - (width_ * resolution_) / 2.0;
    double new_origin_y = robot_y - (height_ * resolution_) / 2.0;
    double new_origin_z = robot_z;  // Z坐标直接跟随机器人
    
    // 检查是否需要移动地图
    double dx = new_origin_x - origin_x_;
    double dy = new_origin_y - origin_y_;
    double dz = new_origin_z - origin_z_;
    
    // 如果移动距离超过一个像素，就需要滚动地图
    if (abs(dx) >= resolution_ || abs(dy) >= resolution_)
    {
        // 计算像素偏移量
        int offset_x = static_cast<int>(round(dx / resolution_));
        int offset_y = static_cast<int>(round(dy / resolution_));
        
        // 滚动地图数据
        rollMap(offset_x, offset_y);
        
        // 更新原点
        origin_x_ = new_origin_x;
        origin_y_ = new_origin_y;
        
        ROS_DEBUG("Map rolled by (%d, %d) pixels to origin (%.2f, %.2f, %.2f)", 
                 offset_x, offset_y, origin_x_, origin_y_, origin_z_);
    }
    
    // 无论是否滚动地图，都更新Z坐标
    origin_z_ = new_origin_z;
    
    // 更新消息中的原点信息
    costmap_msg_.info.origin.position.x = origin_x_;
    costmap_msg_.info.origin.position.y = origin_y_;
    costmap_msg_.info.origin.position.z = origin_z_;
}

bool CostMap2D::getRobotPose(double& x, double& y, double& z, double& yaw)
{
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            global_frame_, robot_base_frame_, ros::Time(0), ros::Duration(0.1));
        
        x = transform.transform.translation.x;
        y = transform.transform.translation.y;
        z = transform.transform.translation.z;  // 添加Z坐标
        
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw);
        
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "Failed to get robot pose: %s", ex.what());
        return false;
    }
}

bool CostMap2D::worldToMap(double wx, double wy, int& mx, int& my)
{
    mx = static_cast<int>((wx - origin_x_) / resolution_);
    my = static_cast<int>((wy - origin_y_) / resolution_);
    
    return (mx >= 0 && mx < width_ && my >= 0 && my < height_);
}

void CostMap2D::mapToWorld(int mx, int my, double& wx, double& wy)
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

void CostMap2D::setCost(int mx, int my, unsigned char cost)
{
    if (mx >= 0 && mx < width_ && my >= 0 && my < height_)
    {
        costmap_[my * width_ + mx] = cost;
    }
}

unsigned char CostMap2D::getCost(int mx, int my)
{
    if (mx >= 0 && mx < width_ && my >= 0 && my < height_)
    {
        return costmap_[my * width_ + mx];
    }
    return NO_INFORMATION;
}

double CostMap2D::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

bool CostMap2D::isPitchAngleExceeded()
{
    if (!enable_pitch_detection_)
    {
        return false;  // 功能未启用，认为俯仰角正常
    }
    
    bool current_pitch_exceeded = false;  // 当前检测到的俯仰角是否超限
    
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            global_frame_, robot_base_frame_, ros::Time(0), ros::Duration(0.1));
        
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 将弧度转换为度数
        double pitch_degrees = pitch * 180.0 / M_PI;
        
        // 检查当前俯仰角绝对值是否超过阈值
        current_pitch_exceeded = std::abs(pitch_degrees) > max_pitch_angle_;
        
        ros::Time current_time = ros::Time::now();
        
        if (current_pitch_exceeded)
        {
            // 当前俯仰角超限
            if (!pitch_exceeded_state_)
            {
                // 从正常状态转为超限状态
                pitch_exceeded_state_ = true;
                ROS_WARN("Robot pitch angle (%.1f deg) exceeds threshold (%.1f deg), activating costmap clearing mode", 
                         pitch_degrees, max_pitch_angle_);
            }
            else
            {
                ROS_DEBUG("Robot pitch angle (%.1f deg) still exceeds threshold (%.1f deg), maintaining clearing mode", 
                         pitch_degrees, max_pitch_angle_);
            }
        }
        else
        {
            // 当前俯仰角正常
            if (pitch_exceeded_state_)
            {
                // 处于超限状态，但当前俯仰角正常
                // 检查是否已经连续正常足够长时间
                if (last_normal_pitch_time_.isZero())
                {
                    // 第一次检测到正常状态，记录时间
                    last_normal_pitch_time_ = current_time;
                    ROS_INFO("Robot pitch angle (%.1f deg) returned to normal, starting recovery timer (%.1f seconds)", 
                             pitch_degrees, pitch_recovery_time_);
                }
                else
                {
                    // 计算连续正常的时间
                    double normal_duration = (current_time - last_normal_pitch_time_).toSec();
                    
                    if (normal_duration >= pitch_recovery_time_)
                    {
                        // 连续正常时间足够，解除超限状态
                        pitch_exceeded_state_ = false;
                        last_normal_pitch_time_ = ros::Time(0);  // 重置
                        ROS_INFO("Robot pitch angle stable for %.1f seconds, deactivating costmap clearing mode", 
                                 normal_duration);
                    }
                    else
                    {
                        ROS_DEBUG("Robot pitch angle (%.1f deg) normal for %.1f/%.1f seconds, waiting for recovery", 
                                 pitch_degrees, normal_duration, pitch_recovery_time_);
                    }
                }
            }
            else
            {
                // 正常状态且当前也正常，保持正常状态
                ROS_DEBUG("Robot pitch angle: %.1f deg (normal)", pitch_degrees);
            }
        }
        
        // 如果当前检测到超限，但之前已经在恢复状态中，需要重置恢复计时器
        if (current_pitch_exceeded && pitch_exceeded_state_ && !last_normal_pitch_time_.isZero())
        {
            last_normal_pitch_time_ = ros::Time(0);  // 重置恢复计时器
            ROS_DEBUG("Pitch angle exceeded again during recovery, resetting recovery timer");
        }
        
        return pitch_exceeded_state_;  // 返回当前的状态，而不是瞬时检测结果
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "Failed to get robot pose for pitch detection: %s", ex.what());
        return pitch_exceeded_state_;  // 获取姿态失败，保持当前状态
    }
}

void CostMap2D::updateCostMap()
{
    // 这个函数可以用于周期性更新，目前主要通过scan回调更新
}

void CostMap2D::decayObstacles()
{
    ros::Time current_time = ros::Time::now();
    
    for (int i = 0; i < width_ * height_; ++i)
    {
        if (costmap_[i] > FREE_SPACE && costmap_[i] <= LETHAL_OBSTACLE)
        {
            // 计算障碍物年龄
            double age = (current_time - obstacle_timestamps_[i]).toSec();
            
            if (age > obstacle_max_age_)
            {
                // 超过最大年龄，完全清除
                costmap_[i] = FREE_SPACE;
            }
            else if (age > 0)
            {
                // 基于时间进行衰减
                double decay_factor = exp(-decay_rate_ * age);
                
                if (costmap_[i] == LETHAL_OBSTACLE)
                {
                    // 致命障碍物逐渐衰减为膨胀障碍物
                    if (decay_factor < 0.8)  // 衰减到80%时开始降级
                    {
                        unsigned char new_cost = static_cast<unsigned char>(
                            FREE_SPACE + (INSCRIBED_INFLATED_OBSTACLE - FREE_SPACE) * decay_factor);
                        costmap_[i] = std::max(static_cast<unsigned char>(FREE_SPACE), new_cost);
                    }
                }
                else
                {
                    // 膨胀障碍物直接衰减
                    unsigned char new_cost = static_cast<unsigned char>(costmap_[i] * decay_factor);
                    costmap_[i] = std::max(static_cast<unsigned char>(FREE_SPACE), new_cost);
                }
            }
        }
    }
}

void CostMap2D::raytraceFreespace(const sensor_msgs::LaserScan& scan, double robot_x, double robot_y)
{
    int robot_mx, robot_my;
    if (!worldToMap(robot_x, robot_y, robot_mx, robot_my))
    {
        ROS_WARN_THROTTLE(5.0, "Robot position outside costmap bounds for raytracing");
        return;
    }
    
    // 获取传感器在全局坐标系中的位置
    geometry_msgs::PointStamped sensor_origin_local, sensor_origin_global;
    sensor_origin_local.header = scan.header;
    sensor_origin_local.point.x = 0.0;
    sensor_origin_local.point.y = 0.0;
    sensor_origin_local.point.z = 0.0;
    
    try
    {
        tf2::doTransform(sensor_origin_local, sensor_origin_global,
                        tf_buffer_.lookupTransform(global_frame_, scan.header.frame_id, 
                                                 scan.header.stamp, ros::Duration(0.1)));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "Failed to get sensor origin for raytracing: %s", ex.what());
        return;
    }
    
    int sensor_mx, sensor_my;
    if (!worldToMap(sensor_origin_global.point.x, sensor_origin_global.point.y, sensor_mx, sensor_my))
    {
        ROS_WARN_THROTTLE(5.0, "Sensor position outside costmap bounds");
        return;
    }
    
    // 对每条激光束进行射线追踪
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        float range = scan.ranges[i];
        float angle = scan.angle_min + i * scan.angle_increment;
        
        // 处理有效射线
        if (range >= scan.range_min && range <= scan.range_max)
        {
            // 在传感器坐标系中计算终点
            geometry_msgs::PointStamped ray_end_local, ray_end_global;
            ray_end_local.header = scan.header;
            
            // 使用实际测量距离，但限制在我们的处理范围内
            float effective_range = std::min(range, static_cast<float>(max_obstacle_range_));
            ray_end_local.point.x = effective_range * cos(angle);
            ray_end_local.point.y = effective_range * sin(angle);
            ray_end_local.point.z = 0.0;
            
            try
            {
                tf2::doTransform(ray_end_local, ray_end_global,
                                tf_buffer_.lookupTransform(global_frame_, scan.header.frame_id, 
                                                         scan.header.stamp, ros::Duration(0.1)));
            }
            catch (tf2::TransformException& ex)
            {
                continue; // 跳过变换失败的射线
            }
            
            int end_mx, end_my;
            if (worldToMap(ray_end_global.point.x, ray_end_global.point.y, end_mx, end_my))
            {
                // 执行射线追踪清理
                std::vector<std::pair<int, int>> ray_cells;
                bresenhamLine(sensor_mx, sensor_my, end_mx, end_my, ray_cells);
                
                // 如果射线在有效障碍物检测范围内击中了东西，标记检测到障碍物
                bool obstacle_hit = (range <= max_obstacle_range_ && range >= min_obstacle_range_);
                clearRayPath(ray_cells, obstacle_hit);
            }
        }
        else if (std::isinf(range) || range > scan.range_max)
        {
            // 对于无穷大或超出范围的射线，清理到最大检测距离
            geometry_msgs::PointStamped ray_end_local, ray_end_global;
            ray_end_local.header = scan.header;
            ray_end_local.point.x = max_obstacle_range_ * cos(angle);
            ray_end_local.point.y = max_obstacle_range_ * sin(angle);
            ray_end_local.point.z = 0.0;
            
            try
            {
                tf2::doTransform(ray_end_local, ray_end_global,
                                tf_buffer_.lookupTransform(global_frame_, scan.header.frame_id, 
                                                         scan.header.stamp, ros::Duration(0.1)));
                
                int end_mx, end_my;
                if (worldToMap(ray_end_global.point.x, ray_end_global.point.y, end_mx, end_my))
                {
                    std::vector<std::pair<int, int>> ray_cells;
                    bresenhamLine(sensor_mx, sensor_my, end_mx, end_my, ray_cells);
                    clearRayPath(ray_cells, false); // 没有检测到障碍物
                }
            }
            catch (tf2::TransformException& ex)
            {
                continue;
            }
        }
        // 对于range < scan.range_min的射线，不进行处理（通常是无效数据）
    }
}

// Bresenham直线算法用于射线追踪
void CostMap2D::bresenhamLine(int x0, int y0, int x1, int y1, 
                             std::vector<std::pair<int, int>>& cells)
{
    cells.clear();
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true)
    {
        cells.push_back(std::make_pair(x, y));
        
        if (x == x1 && y == y1)
            break;
            
        int e2 = 2 * err;
        
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }
}

void CostMap2D::clearRayPath(const std::vector<std::pair<int, int>>& ray_cells, bool obstacle_detected)
{
    // 官方costmap的射线清理机制：
    // 1. 无条件清理射线穿过的所有区域（除了最终的障碍物点）
    // 2. 不考虑障碍物年龄，射线穿过就清理
    
    // 确定清理范围：如果检测到障碍物，不清理最后一个点（障碍物点本身）
    size_t clear_end = ray_cells.size();
    if (obstacle_detected && ray_cells.size() > 1)
    {
        clear_end = ray_cells.size() - 1; // 保留障碍物点
    }
    
    // 清理射线路径上的所有点（从传感器位置开始，但不包括传感器位置本身）
    for (size_t i = 1; i < clear_end; ++i)
    {
        int mx = ray_cells[i].first;
        int my = ray_cells[i].second;
        
        if (mx >= 0 && mx < width_ && my >= 0 && my < height_)
        {
            int index = my * width_ + mx;
            
            // 无条件清理射线穿过的区域（官方costmap行为）
            // 只要射线穿过，就设置为自由空间
            costmap_[index] = FREE_SPACE;
            
            // 更新时间戳
            if (index >= 0 && index < static_cast<int>(obstacle_timestamps_.size())) {
                obstacle_timestamps_[index] = ros::Time::now();
            }
        }
    }
}

void CostMap2D::publishCostMap()
{
    // 更新时间戳
    costmap_msg_.header.stamp = ros::Time::now();
    
    // 转换代价值到OccupancyGrid格式 (0-100)
    for (size_t i = 0; i < costmap_.size(); ++i)
    {
        if (costmap_[i] == NO_INFORMATION)
        {
            costmap_msg_.data[i] = -1;  // 未知
        }
        else if (costmap_[i] == LETHAL_OBSTACLE)
        {
            costmap_msg_.data[i] = 100;  // 致命障碍物
        }
        else if (costmap_[i] == FREE_SPACE)
        {
            costmap_msg_.data[i] = 0;    // 自由空间
        }
        else
        {
            // 梯度代价值：将内部代价值 (1-253) 映射到 OccupancyGrid 的 (1-99)
            int occupancy_value = static_cast<int>((costmap_[i] * 99.0) / 253.0);
            occupancy_value = std::max(1, std::min(99, occupancy_value));
            costmap_msg_.data[i] = occupancy_value;
        }
    }
    
    // 发布代价地图
    costmap_pub_.publish(costmap_msg_);
}

void CostMap2D::rollMap(int offset_x, int offset_y)
{
    // 创建新的地图数据
    std::vector<unsigned char> new_costmap(width_ * height_, FREE_SPACE);
    std::vector<ros::Time> new_timestamps(width_ * height_, ros::Time::now());
    
    // 复制旧数据到新位置
    for (int old_y = 0; old_y < height_; ++old_y)
    {
        for (int old_x = 0; old_x < width_; ++old_x)
        {
            // 计算在新地图中的位置
            int new_x = old_x - offset_x;
            int new_y = old_y - offset_y;
            
            // 检查新位置是否在地图范围内
            if (new_x >= 0 && new_x < width_ && new_y >= 0 && new_y < height_)
            {
                int old_index = old_y * width_ + old_x;
                int new_index = new_y * width_ + new_x;
                
                new_costmap[new_index] = costmap_[old_index];
                new_timestamps[new_index] = obstacle_timestamps_[old_index];
            }
        }
    }
    
    // 替换地图数据
    costmap_ = std::move(new_costmap);
    obstacle_timestamps_ = std::move(new_timestamps);
}

} // namespace local_planner
