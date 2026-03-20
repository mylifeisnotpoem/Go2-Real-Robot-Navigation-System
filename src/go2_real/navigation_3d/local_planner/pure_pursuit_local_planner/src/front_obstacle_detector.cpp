#include "pure_pursuit_local_planner/front_obstacle_detector.h"
#include <algorithm>
#include <cmath>

namespace pure_pursuit_local_planner
{

FrontObstacleDetector::FrontObstacleDetector()
    : has_costmap_(false)
    , detection_width_(0.3)      // 0.3m 宽度
    , detection_length_(0.5)     // 0.5m 长度
    , sector_radius_(0.8)        // 0.8m 扇形半径
    , sector_angle_(60.0)        // 60度扇形角度（正负30度）
    , obstacle_threshold_(50)    // 代价值阈值
    , large_obstacle_ratio_(0.5) // [新增] 大障碍物判定比例阈值
    , linear_velocity_factor_(0.4)   // 线速度乘以0.4
    , angular_velocity_factor_(2.0)  // 角速度乘以2
    , detection_shape_("rectangle")  // 默认使用矩形
    , near_goal_check_radius_(0.25)
    , near_goal_occupied_ratio_threshold_(0.2)
    , debug_mode_(false)
    , enable_visualization_(true)
    , has_pointcloud_(false)
    , stair_detection_range_(1.2)
    , stair_detection_width_(0.8)
    , stair_detection_height_min_(0.15)
    , stair_detection_height_max_(1.5)
    , downstairs_detection_range_(1.5)
    , downstairs_detection_width_(0.8)
    , downstairs_detection_height_min_(0.05)
    , downstairs_detection_height_max_(1.5)
    , stair_obstacle_point_threshold_(10)
{
}

FrontObstacleDetector::~FrontObstacleDetector()
{
}

bool FrontObstacleDetector::initialize(ros::NodeHandle& nh, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
    nh_ = nh;
    tf_buffer_ = tf_buffer;

    // 加载参数
    nh_.param("front_obstacle_detector/detection_width", detection_width_, 0.3);
    nh_.param("front_obstacle_detector/detection_length", detection_length_, 0.5);
    nh_.param("front_obstacle_detector/sector_radius", sector_radius_, 0.8);
    nh_.param("front_obstacle_detector/sector_angle", sector_angle_, 60.0);
    nh_.param("front_obstacle_detector/obstacle_threshold", obstacle_threshold_, 50);
    // [新增] 加载大障碍物阈值参数
    nh_.param("front_obstacle_detector/large_obstacle_ratio", large_obstacle_ratio_, 0.5);
    nh_.param("front_obstacle_detector/linear_velocity_factor", linear_velocity_factor_, 0.4);
    nh_.param("front_obstacle_detector/angular_velocity_factor", angular_velocity_factor_, 2.0);
    nh_.param("front_obstacle_detector/detection_shape", detection_shape_, std::string("rectangle"));
    nh_.param("front_obstacle_detector/near_goal_check_radius", near_goal_check_radius_, 0.25);
    nh_.param("front_obstacle_detector/near_goal_occupied_ratio_threshold", near_goal_occupied_ratio_threshold_, 0.2);
    nh_.param("front_obstacle_detector/debug_mode", debug_mode_, false);
    nh_.param("front_obstacle_detector/enable_visualization", enable_visualization_, true);

    // 楼梯模式点云检测参数
    nh_.param("front_obstacle_detector/stair_detection_range", stair_detection_range_, 1.2);
    nh_.param("front_obstacle_detector/stair_detection_width", stair_detection_width_, 0.8);
    nh_.param("front_obstacle_detector/stair_detection_height_min", stair_detection_height_min_, 0.15);
    nh_.param("front_obstacle_detector/stair_detection_height_max", stair_detection_height_max_, 1.5);
    nh_.param("front_obstacle_detector/downstairs_detection_range", downstairs_detection_range_, 1.5);
    nh_.param("front_obstacle_detector/downstairs_detection_width", downstairs_detection_width_, 0.8);
    nh_.param("front_obstacle_detector/downstairs_detection_height_min", downstairs_detection_height_min_, 0.05);
    nh_.param("front_obstacle_detector/downstairs_detection_height_max", downstairs_detection_height_max_, 1.5);
    nh_.param("front_obstacle_detector/stair_obstacle_point_threshold", stair_obstacle_point_threshold_, 10);
    stairs_scene_sub_ = nh_.subscribe("/stairs_scene", 1, &FrontObstacleDetector::stairsSceneCallback, this);

    // 初始化发布器
    if (enable_visualization_)
    {
        detection_area_pub_ = nh_.advertise<visualization_msgs::Marker>("front_detection_area", 1);
    }

    ROS_INFO("Front Obstacle Detector initialized with parameters:");
    if (detection_shape_ == "rectangle")
    {
        ROS_INFO("  - Detection shape: Rectangle (%.2fm x %.2fm)", detection_width_, detection_length_);
    }
    else if (detection_shape_ == "sector")
    {
        ROS_INFO("  - Detection shape: Sector (radius: %.2fm, angle: %.1f°)", sector_radius_, sector_angle_);
    }
    ROS_INFO("  - Obstacle threshold: %d", obstacle_threshold_);
    ROS_INFO("  - Large obstacle ratio: %.2f", large_obstacle_ratio_); // [新增]
    ROS_INFO("  - Linear velocity factor: %.2f", linear_velocity_factor_);
    ROS_INFO("  - Angular velocity factor: %.2f", angular_velocity_factor_);
    ROS_INFO("  - Near-goal occupancy check: radius=%.2fm, ratio_threshold=%.2f",
             near_goal_check_radius_, near_goal_occupied_ratio_threshold_);
    ROS_INFO("  - Visualization enabled: %s", enable_visualization_ ? "true" : "false");

    ROS_INFO("  - Stair mode detection range: %.2fm, width: %.2fm", stair_detection_range_, stair_detection_width_);
    ROS_INFO("  - Stair mode height filter: [%.2f, %.2f]m, point threshold: %d",
             stair_detection_height_min_, stair_detection_height_max_, stair_obstacle_point_threshold_);
    ROS_INFO("  - Downstairs override range: %.2fm, width: %.2fm, height filter: [%.2f, %.2f]m",
             downstairs_detection_range_, downstairs_detection_width_,
             downstairs_detection_height_min_, downstairs_detection_height_max_);

    return true;
}

void FrontObstacleDetector::stairsSceneCallback(const std_msgs::Int32::ConstPtr& msg)
{
    stairs_scene_state_ = msg->data;
    if (debug_mode_)
    {
        ROS_INFO_THROTTLE(1.0, "[StairDebug][Scene] /stairs_scene=%d (1:up, -1:down, 0:flat)",
            stairs_scene_state_);
    }
}

void FrontObstacleDetector::updateCostmap(const nav_msgs::OccupancyGrid& costmap)
{
    costmap_ = costmap;
    has_costmap_ = true;
}

void FrontObstacleDetector::updatePointCloud(const sensor_msgs::PointCloud2& cloud)
{
    latest_cloud_ = cloud;
    has_pointcloud_ = true;

    if (debug_mode_)
    {
        ROS_INFO_THROTTLE(1.0, "[StairDebug][Cloud] Received cloud frame=%s stamp=%.3f points(width=%u,height=%u)",
            cloud.header.frame_id.c_str(), cloud.header.stamp.toSec(),
            cloud.width, cloud.height);
    }
}

// [修改] 这个旧接口保留，但内部可视化调用适配了新逻辑
geometry_msgs::Twist FrontObstacleDetector::adjustVelocityForObstacles(const geometry_msgs::Twist& cmd_vel,
                                                                       const geometry_msgs::PoseStamped& robot_pose)
{
    geometry_msgs::Twist adjusted_cmd_vel = cmd_vel;

    // 如果没有代价地图，直接返回原始速度
    if (!has_costmap_)
    {
        if (debug_mode_)
        {
            ROS_WARN_THROTTLE(2.0, "No costmap available for obstacle detection");
        }
        return adjusted_cmd_vel;
    }

    // 检测前方区域是否有障碍物
    bool obstacle_detected = false;
    if (detection_shape_ == "sector")
    {
        obstacle_detected = detectObstacleInSectorArea(robot_pose);
    }
    else // 默认使用矩形
    {
        obstacle_detected = detectObstacleInFrontArea(robot_pose);
    }

    // 发布可视化 [修改] 适配新的枚举类型
    if (enable_visualization_)
    {
        // 如果旧接口检测到障碍物，暂时默认标记为 SMALL 类型进行可视化
        publishVisualization(robot_pose, obstacle_detected ? ObstacleType::SMALL : ObstacleType::NONE);
    }

    if (obstacle_detected)
    {
        const bool stair_scene_active = (stairs_scene_state_ != 0);  // 1:上楼梯, -1:下楼梯
        const double active_angular_velocity_factor = stair_scene_active ? 1.0 : angular_velocity_factor_;

        // 调整速度：线速度按参数衰减；楼梯场景下角速度系数强制为1.0
        adjusted_cmd_vel.linear.x *= linear_velocity_factor_;
        adjusted_cmd_vel.linear.y *= linear_velocity_factor_;
        adjusted_cmd_vel.angular.z *= active_angular_velocity_factor;

        if (debug_mode_)
        {
            ROS_WARN_THROTTLE(0.5, "Obstacle detected in front area! Adjusting velocity:");
            ROS_WARN_THROTTLE(0.5, "  Original: linear_x=%.3f, angular_z=%.3f", 
                             cmd_vel.linear.x, cmd_vel.angular.z);
            ROS_WARN_THROTTLE(0.5, "  Adjusted: linear_x=%.3f, angular_z=%.3f (ang_factor=%.2f, stairs_scene=%d)",
                             adjusted_cmd_vel.linear.x, adjusted_cmd_vel.angular.z,
                             active_angular_velocity_factor, stairs_scene_state_);
        }
    }
    else if (debug_mode_)
    {
        ROS_DEBUG_THROTTLE(1.0, "No obstacles detected in front area");
    }

    return adjusted_cmd_vel;
}

// [新增] 核心分类逻辑实现
ObstacleType FrontObstacleDetector::detectObstacleType(const geometry_msgs::PoseStamped& robot_pose)
{
    if (!has_costmap_) return ObstacleType::NONE;

    int obstacle_cells = 0;
    int total_cells = 0;

    // 根据形状选择检测逻辑
    // 为了不破坏原有函数结构，这里复用了核心计数逻辑
    if (detection_shape_ == "sector")
    {
        double robot_x = robot_pose.pose.position.x;
        double robot_y = robot_pose.pose.position.y;
        double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

        // 计算边界
        double min_x = robot_x - sector_radius_;
        double max_x = robot_x + sector_radius_;
        double min_y = robot_y - sector_radius_;
        double max_y = robot_y + sector_radius_;
        
        int min_map_x, min_map_y, max_map_x, max_map_y;
        if (worldToMap(min_x, min_y, min_map_x, min_map_y) && worldToMap(max_x, max_y, max_map_x, max_map_y))
        {
             min_map_x = std::max(0, min_map_x);
             min_map_y = std::max(0, min_map_y);
             max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
             max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

             for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
                for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
                    double world_x = costmap_.info.origin.position.x + map_x * costmap_.info.resolution;
                    double world_y = costmap_.info.origin.position.y + map_y * costmap_.info.resolution;
                    if (isPointInSector(world_x, world_y, robot_x, robot_y, robot_yaw)) {
                        int cost = getCostAtMapCoordinate(map_x, map_y);
                        if (cost >= 0) {
                            total_cells++;
                            if (cost >= obstacle_threshold_) obstacle_cells++;
                        }
                    }
                }
             }
        }
    }
    else // Rectangle
    {
        std::vector<std::pair<double, double>> corners;
        generateDetectionAreaCorners(robot_pose, corners);
        
        double min_x = std::min({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        double max_x = std::max({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        double min_y = std::min({corners[0].second, corners[1].second, corners[2].second, corners[3].second});
        double max_y = std::max({corners[0].second, corners[1].second, corners[2].second, corners[3].second});

        int min_map_x, min_map_y, max_map_x, max_map_y;
        if (worldToMap(min_x, min_y, min_map_x, min_map_y) && worldToMap(max_x, max_y, max_map_x, max_map_y))
        {
            min_map_x = std::max(0, min_map_x);
            min_map_y = std::max(0, min_map_y);
            max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
            max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

            for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
                for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
                    int cost = getCostAtMapCoordinate(map_x, map_y);
                    if (cost >= 0) {
                        total_cells++;
                        if (cost >= obstacle_threshold_) obstacle_cells++;
                    }
                }
            }
        }
    }

    // 计算占比并分类
    double ratio = (total_cells > 0) ? static_cast<double>(obstacle_cells) / total_cells : 0.0;
    
    ObstacleType type = ObstacleType::NONE;
    if (ratio > large_obstacle_ratio_) {
        type = ObstacleType::LARGE;
    } else if (ratio > 0.1) {
        type = ObstacleType::SMALL;
    }

    if (enable_visualization_) {
        publishVisualization(robot_pose, type);
    }
    
    if (debug_mode_) {
        ROS_INFO_THROTTLE(0.5, "Obstacle Ratio: %.2f, Type: %s", ratio, 
            (type == ObstacleType::LARGE ? "LARGE" : (type == ObstacleType::SMALL ? "SMALL" : "NONE")));
    }

    return type;
} // <--- Added missing closing brace here

bool FrontObstacleDetector::isGoalRegionOccupied(const geometry_msgs::Point& goal_point)
{
    if (!has_costmap_)
    {
        ROS_WARN_THROTTLE(1.0,
            "Near-goal occupancy check skipped: no costmap, goal=(%.3f, %.3f, %.3f)",
            goal_point.x, goal_point.y, goal_point.z);
        return false;
    }

    int center_map_x = 0;
    int center_map_y = 0;
    if (!worldToMap(goal_point.x, goal_point.y, center_map_x, center_map_y))
    {
        if (debug_mode_)
        {
            ROS_WARN_THROTTLE(1.0, "Near-goal occupancy check skipped: goal point outside costmap");
        }
        ROS_WARN_THROTTLE(1.0,
            "Near-goal occupancy check skipped: goal outside costmap, goal=(%.3f, %.3f, %.3f)",
            goal_point.x, goal_point.y, goal_point.z);
        return false;
    }

    const double resolution = costmap_.info.resolution;
    const int cell_radius = std::max(1, static_cast<int>(std::ceil(near_goal_check_radius_ / resolution)));

    int occupied_cells = 0;
    int total_cells = 0;
    bool has_nearest_occupied = false;
    double nearest_occupied_dist = std::numeric_limits<double>::max();
    int nearest_map_x = 0;
    int nearest_map_y = 0;
    int nearest_cost = -1;
    double nearest_world_x = 0.0;
    double nearest_world_y = 0.0;

    for (int dx = -cell_radius; dx <= cell_radius; ++dx)
    {
        for (int dy = -cell_radius; dy <= cell_radius; ++dy)
        {
            const double dist = std::hypot(dx * resolution, dy * resolution);
            if (dist > near_goal_check_radius_)
            {
                continue;
            }

            const int map_x = center_map_x + dx;
            const int map_y = center_map_y + dy;
            const int cost = getCostAtMapCoordinate(map_x, map_y);
            if (cost < 0)
            {
                continue;
            }

            total_cells++;
            if (cost >= obstacle_threshold_)
            {
                occupied_cells++;

                if (dist < nearest_occupied_dist)
                {
                    has_nearest_occupied = true;
                    nearest_occupied_dist = dist;
                    nearest_map_x = map_x;
                    nearest_map_y = map_y;
                    nearest_cost = cost;
                    nearest_world_x = costmap_.info.origin.position.x + (map_x + 0.5) * resolution;
                    nearest_world_y = costmap_.info.origin.position.y + (map_y + 0.5) * resolution;
                }
            }
        }
    }

    if (total_cells == 0)
    {
        ROS_WARN_THROTTLE(1.0,
            "Near-goal occupancy check skipped: no valid cells around goal=(%.3f, %.3f), radius=%.2f",
            goal_point.x, goal_point.y, near_goal_check_radius_);
        return false;
    }

    const double occupied_ratio = static_cast<double>(occupied_cells) / total_cells;
    const bool occupied = occupied_ratio >= near_goal_occupied_ratio_threshold_;

    if (has_nearest_occupied)
    {
        ROS_INFO_THROTTLE(0.5,
            "Near-goal occupancy detail: goal=(%.3f, %.3f) center_cell=(%d,%d) radius=%.2f occupied=%d/%d ratio=%.2f threshold=%.2f nearest_occ_cell=(%d,%d) nearest_occ_world=(%.3f, %.3f) nearest_occ_dist=%.3f cost=%d result=%s",
            goal_point.x, goal_point.y, center_map_x, center_map_y, near_goal_check_radius_,
            occupied_cells, total_cells, occupied_ratio, near_goal_occupied_ratio_threshold_,
            nearest_map_x, nearest_map_y, nearest_world_x, nearest_world_y, nearest_occupied_dist, nearest_cost,
            occupied ? "OCCUPIED" : "CLEAR");
    }
    else
    {
        ROS_INFO_THROTTLE(0.5,
            "Near-goal occupancy detail: goal=(%.3f, %.3f) center_cell=(%d,%d) radius=%.2f occupied=%d/%d ratio=%.2f threshold=%.2f nearest_occ=NONE result=%s",
            goal_point.x, goal_point.y, center_map_x, center_map_y, near_goal_check_radius_,
            occupied_cells, total_cells, occupied_ratio, near_goal_occupied_ratio_threshold_,
            occupied ? "OCCUPIED" : "CLEAR");
    }

    if (debug_mode_)
    {
        ROS_INFO_THROTTLE(0.5,
            "Near-goal occupancy: goal=(%.3f, %.3f), occupied=%d/%d, ratio=%.2f, threshold=%.2f, result=%s",
            goal_point.x, goal_point.y, occupied_cells, total_cells, occupied_ratio,
            near_goal_occupied_ratio_threshold_, occupied ? "OCCUPIED" : "CLEAR");
    }

    return occupied;
}

ObstacleType FrontObstacleDetector::detectObstacleFromPointCloud(
    const geometry_msgs::PoseStamped& robot_pose)
{
    const bool is_downstairs = (stairs_scene_state_ == -1);
    const double active_stair_detection_range = is_downstairs ? downstairs_detection_range_ : stair_detection_range_;
    const double active_stair_detection_width = is_downstairs ? downstairs_detection_width_ : stair_detection_width_;
    const double active_stair_detection_height_min = is_downstairs ? downstairs_detection_height_min_ : stair_detection_height_min_;
    const double active_stair_detection_height_max = is_downstairs ? downstairs_detection_height_max_ : stair_detection_height_max_;

    if (debug_mode_)
    {
        ROS_INFO_THROTTLE(1.0, "[StairDebug][SceneActive] stairs_scene=%d is_downstairs=%d",
            stairs_scene_state_, is_downstairs ? 1 : 0);
        ROS_INFO_THROTTLE(1.0, "[StairDebug][ParamProfile] %s",
            is_downstairs ? "downstairs_override" : "default_stair_params");
        ROS_INFO_THROTTLE(1.0, "[StairDebug][DetectStart] robot_pose map=(%.3f, %.3f, %.3f) yaw=%.3f "
            "range=(0.05, %.3f) half_width=%.3f z_range=(%.3f, %.3f) threshold_points=%d",
            robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z,
            tf2::getYaw(robot_pose.pose.orientation),
            active_stair_detection_range, active_stair_detection_width / 2.0,
            active_stair_detection_height_min, active_stair_detection_height_max,
            stair_obstacle_point_threshold_);
    }

    if (!has_pointcloud_)
    {
        ROS_WARN_THROTTLE(2.0, "Stair mode: No point cloud data available");
        return ObstacleType::NONE;
    }

    // 检查点云时间戳是否太旧（超过1秒认为无效）
    double cloud_age = (ros::Time::now() - latest_cloud_.header.stamp).toSec();
    if (cloud_age > 1.0)
    {
        ROS_WARN_THROTTLE(2.0, "Stair mode: Point cloud data too old (%.1fs)", cloud_age);
        return ObstacleType::NONE;
    }

    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_z = robot_pose.pose.position.z;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    // cos/sin 预计算
    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);
    double half_width = active_stair_detection_width / 2.0;

    int obstacle_points = 0;
    int total_points_in_range = 0;

    // 遍历点云
    try
    {
        // 获取 点云坐标系 -> base_link 的变换
        geometry_msgs::TransformStamped cloud_to_base;
        if (tf_buffer_)
        {
            cloud_to_base = tf_buffer_->lookupTransform(
                "base_link", latest_cloud_.header.frame_id,
                ros::Time(0), ros::Duration(0.1));
        }
        else
        {
            ROS_WARN_THROTTLE(2.0, "Stair mode: TF buffer not available");
            return ObstacleType::NONE;
        }

        double tx = cloud_to_base.transform.translation.x;
        double ty = cloud_to_base.transform.translation.y;
        double tz = cloud_to_base.transform.translation.z;

        // 获取旋转四元数
        tf2::Quaternion q_tf;
        tf2::fromMsg(cloud_to_base.transform.rotation, q_tf);
        tf2::Matrix3x3 rot(q_tf);

        if (debug_mode_)
        {
            ROS_INFO_THROTTLE(1.0, "[StairDebug][TF] cloud_frame=%s -> base_link tx=%.3f ty=%.3f tz=%.3f",
                latest_cloud_.header.frame_id.c_str(), tx, ty, tz);
        }

        // 遍历点云中的每个点
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(latest_cloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(latest_cloud_, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(latest_cloud_, "z");

        int point_idx = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_idx)
        {
            float px = *iter_x;
            float py = *iter_y;
            float pz = *iter_z;

            // 跳过无效点
            if (std::isnan(px) || std::isnan(py) || std::isnan(pz))
                continue;

            // 变换到 base_link 坐标系
            double bx = rot[0][0] * px + rot[0][1] * py + rot[0][2] * pz + tx;
            double by = rot[1][0] * px + rot[1][1] * py + rot[1][2] * pz + ty;
            double bz = rot[2][0] * px + rot[2][1] * py + rot[2][2] * pz + tz;

            const bool x_ok = (bx > 0.05 && bx < active_stair_detection_range);
            const bool y_ok = (std::abs(by) < half_width);
            const bool z_ok = (bz > active_stair_detection_height_min && bz < active_stair_detection_height_max);

            if (x_ok && y_ok)
            {
                total_points_in_range++;
            }

            if (debug_mode_)
            {
                ROS_INFO_THROTTLE(1.0, "[StairDebug][Point] idx=%d raw=(%.3f, %.3f, %.3f) base=(%.3f, %.3f, %.3f) "
                    "x_ok=%d y_ok=%d z_ok=%d in_box=%d",
                    point_idx, px, py, pz, bx, by, bz,
                    x_ok ? 1 : 0, y_ok ? 1 : 0, z_ok ? 1 : 0,
                    (x_ok && y_ok && z_ok) ? 1 : 0);
            }

            // 在 base_link 坐标系下检查：
            // X > 0 且 X < detection_range → 前方
            // |Y| < half_width → 宽度范围内
            // height_min < Z < height_max → 高度过滤（排除地面和过高的点）
            if (x_ok && y_ok && z_ok)
            {
                obstacle_points++;

                if (debug_mode_)
                {
                    ROS_INFO_THROTTLE(1.0, "[StairDebug][Hit] idx=%d obstacle_points=%d", point_idx, obstacle_points);
                }
            }
        }
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(2.0, "Stair mode: TF lookup failed: %s", ex.what());
        return ObstacleType::NONE;
    }

    // 判定结果
    ObstacleType result = ObstacleType::NONE;
    if (obstacle_points >= stair_obstacle_point_threshold_)
    {
        result = ObstacleType::LARGE;  // 楼梯模式下统一判定为 LARGE（直接停车）
    }

    if (debug_mode_)
    {
        ROS_INFO_THROTTLE(1.0, "[StairDebug][Summary] scene=%d downstairs=%d in_range_xy=%d obstacle_points=%d "
            "threshold=%d result=%s",
            stairs_scene_state_, is_downstairs ? 1 : 0, total_points_in_range, obstacle_points,
            stair_obstacle_point_threshold_,
            (result == ObstacleType::LARGE ? "OBSTACLE" : "CLEAR"));
    }

    if (debug_mode_)
    {
        ROS_INFO_THROTTLE(0.5, "Stair pointcloud detection: %d points in box → %s",
            obstacle_points,
            (result == ObstacleType::LARGE ? "OBSTACLE" : "CLEAR"));
    }

    // 可视化
    if (enable_visualization_)
    {
        publishVisualization(robot_pose, result);
    }

    return result;
}

bool FrontObstacleDetector::detectObstacleInFrontArea(const geometry_msgs::PoseStamped& robot_pose)
{
    // 生成检测区域的四个角点（机器人前方矩形区域）
    std::vector<std::pair<double, double>> corners;
    generateDetectionAreaCorners(robot_pose, corners);

    // 确定检测区域的边界
    double min_x = std::min({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
    double max_x = std::max({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
    double min_y = std::min({corners[0].second, corners[1].second, corners[2].second, corners[3].second});
    double max_y = std::max({corners[0].second, corners[1].second, corners[2].second, corners[3].second});

    // 转换为地图坐标
    int min_map_x, min_map_y, max_map_x, max_map_y;
    if (!worldToMap(min_x, min_y, min_map_x, min_map_y) ||
        !worldToMap(max_x, max_y, max_map_x, max_map_y))
    {
        if (debug_mode_)
        {
            ROS_WARN_THROTTLE(1.0, "Detection area outside of costmap bounds");
        }
        return false;
    }

    // 确保地图坐标在有效范围内
    min_map_x = std::max(0, min_map_x);
    min_map_y = std::max(0, min_map_y);
    max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
    max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

    // 检查区域内的代价值
    int obstacle_cells = 0;
    int total_cells = 0;

    for (int map_x = min_map_x; map_x <= max_map_x; ++map_x)
    {
        for (int map_y = min_map_y; map_y <= max_map_y; ++map_y)
        {
            int cost = getCostAtMapCoordinate(map_x, map_y);
            if (cost >= 0)  // 有效的代价值
            {
                total_cells++;
                if (cost >= obstacle_threshold_)
                {
                    obstacle_cells++;
                }
            }
        }
    }

    // 如果障碍物占比超过阈值，认为检测到障碍物
    double obstacle_ratio = (total_cells > 0) ? static_cast<double>(obstacle_cells) / total_cells : 0.0;
    bool obstacle_detected = obstacle_ratio > 0.1;  // 10%的区域有障碍物就触发

    if (debug_mode_ && obstacle_detected)
    {
        ROS_WARN_THROTTLE(0.5, "Obstacle detection: %d/%d cells above threshold (%.1f%%)", 
                         obstacle_cells, total_cells, obstacle_ratio * 100.0);
    }

    return obstacle_detected;
}

bool FrontObstacleDetector::worldToMap(double world_x, double world_y, int& map_x, int& map_y)
{
    if (!has_costmap_)
        return false;

    // 转换世界坐标到地图坐标
    map_x = static_cast<int>((world_x - costmap_.info.origin.position.x) / costmap_.info.resolution);
    map_y = static_cast<int>((world_y - costmap_.info.origin.position.y) / costmap_.info.resolution);

    // 检查边界
    return (map_x >= 0 && map_x < static_cast<int>(costmap_.info.width) &&
            map_y >= 0 && map_y < static_cast<int>(costmap_.info.height));
}

int FrontObstacleDetector::getCostAtMapCoordinate(int map_x, int map_y)
{
    if (!has_costmap_ || 
        map_x < 0 || map_x >= static_cast<int>(costmap_.info.width) ||
        map_y < 0 || map_y >= static_cast<int>(costmap_.info.height))
    {
        return -1;  // 越界
    }

    int index = map_y * costmap_.info.width + map_x;
    if (index >= 0 && index < static_cast<int>(costmap_.data.size()))
    {
        return costmap_.data[index];
    }

    return -1;
}

void FrontObstacleDetector::generateDetectionAreaCorners(const geometry_msgs::PoseStamped& robot_pose,
                                                        std::vector<std::pair<double, double>>& corners)
{
    corners.clear();

    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    // 计算机器人前方矩形区域的四个角点
    // 矩形：宽度0.3m，长度0.5m，位于机器人前方
    
    double half_width = detection_width_ / 2.0;  // 0.15m
    double length = detection_length_;           // 0.5m

    // 机器人坐标系下的四个角点（机器人前方的矩形）
    std::vector<std::pair<double, double>> local_corners = {
        {0.0, -half_width},         // 左后角 (机器人位置)
        {0.0, half_width},          // 右后角 (机器人位置)
        {length, half_width},       // 右前角
        {length, -half_width}       // 左前角
    };

    // 转换到世界坐标系
    for (const auto& local_corner : local_corners)
    {
        double local_x = local_corner.first;
        double local_y = local_corner.second;

        // 旋转变换
        double world_x = robot_x + local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw);
        double world_y = robot_y + local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw);

        corners.push_back({world_x, world_y});
    }

    if (debug_mode_)
    {
        ROS_DEBUG_THROTTLE(1.0, "Detection area corners in world frame:");
        for (size_t i = 0; i < corners.size(); ++i)
        {
            ROS_DEBUG_THROTTLE(1.0, "  Corner %zu: (%.3f, %.3f)", i, corners[i].first, corners[i].second);
        }
    }
}

// [修改] 参数改为 ObstacleType 以支持颜色区分
void FrontObstacleDetector::publishVisualization(const geometry_msgs::PoseStamped& robot_pose, ObstacleType obstacle_type)
{
    if (!enable_visualization_)
        return;

    visualization_msgs::Marker marker;
    marker.header.frame_id = robot_pose.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "front_obstacle_detection";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;

    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    if (detection_shape_ == "sector")
    {
        // 扇形可视化 - 使用 LINE_STRIP
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.02; // 线条宽度

        // 生成扇形边界点
        std::vector<std::pair<double, double>> sector_points;
        generateSectorAreaPoints(robot_pose, sector_points);

        // 添加边界点到marker
        for (const auto& point : sector_points)
        {
            geometry_msgs::Point p;
            p.x = point.first;
            p.y = point.second;
            p.z = robot_pose.pose.position.z + 0.1;
            marker.points.push_back(p);
        }

        // 闭合扇形（连接最后一点到第一点）
        if (!marker.points.empty())
        {
            marker.points.push_back(marker.points[0]);
        }
    }
    else
    {
        // 矩形可视化 - 使用 CUBE
        marker.type = visualization_msgs::Marker::CUBE;

        // 计算检测区域中心点位置（机器人前方 detection_length_/2 距离处）
        double center_offset = detection_length_ / 2.0;
        marker.pose.position.x = robot_x + center_offset * std::cos(robot_yaw);
        marker.pose.position.y = robot_y + center_offset * std::sin(robot_yaw);
        marker.pose.position.z = robot_pose.pose.position.z + 0.1; // 稍微抬高以便可视化

        // 设置朝向（与机器人朝向一致）
        marker.pose.orientation = robot_pose.pose.orientation;

        // 设置尺寸
        marker.scale.x = detection_length_;  // 长度
        marker.scale.y = detection_width_;   // 宽度
        marker.scale.z = 0.05;              // 高度（薄片状）
    }

    // [修改] 设置颜色：根据障碍物类型显示颜色
    marker.color.a = 0.5; // 半透明
    
    switch (obstacle_type) {
        case ObstacleType::LARGE:
            marker.color.r = 1.0; // 红色 - 大障碍/停车
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            break;
        case ObstacleType::SMALL:
            marker.color.r = 1.0; // 黄色 - 小障碍/绕行
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            break;
        case ObstacleType::NONE:
        default:
            marker.color.r = 0.0; // 绿色 - 安全
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            break;
    }

    // 设置生存时间
    marker.lifetime = ros::Duration(0.2); // 200ms后自动消失

    detection_area_pub_.publish(marker);

    if (debug_mode_)
    {
        std::string type_str;
        if(obstacle_type == ObstacleType::LARGE) type_str = "RED (Large)";
        else if(obstacle_type == ObstacleType::SMALL) type_str = "YELLOW (Small)";
        else type_str = "GREEN (Clear)";

        ROS_DEBUG_THROTTLE(1.0, "Published %s detection area visualization: %s", 
                          detection_shape_.c_str(), type_str.c_str());
    }
}

bool FrontObstacleDetector::detectObstacleInSectorArea(const geometry_msgs::PoseStamped& robot_pose)
{
    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    // 计算扇形边界框
    double min_x = robot_x - sector_radius_;
    double max_x = robot_x + sector_radius_;
    double min_y = robot_y - sector_radius_;
    double max_y = robot_y + sector_radius_;

    // 转换为地图坐标
    int min_map_x, min_map_y, max_map_x, max_map_y;
    if (!worldToMap(min_x, min_y, min_map_x, min_map_y) ||
        !worldToMap(max_x, max_y, max_map_x, max_map_y))
    {
        if (debug_mode_)
        {
            ROS_WARN_THROTTLE(1.0, "Sector detection area outside of costmap bounds");
        }
        return false;
    }

    // 确保地图坐标在有效范围内
    min_map_x = std::max(0, min_map_x);
    min_map_y = std::max(0, min_map_y);
    max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
    max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

    // 检查扇形区域内的代价值
    int obstacle_cells = 0;
    int total_cells = 0;

    for (int map_x = min_map_x; map_x <= max_map_x; ++map_x)
    {
        for (int map_y = min_map_y; map_y <= max_map_y; ++map_y)
        {
            // 转换地图坐标回世界坐标
            double world_x = costmap_.info.origin.position.x + map_x * costmap_.info.resolution;
            double world_y = costmap_.info.origin.position.y + map_y * costmap_.info.resolution;

            // 检查点是否在扇形内
            if (isPointInSector(world_x, world_y, robot_x, robot_y, robot_yaw))
            {
                int cost = getCostAtMapCoordinate(map_x, map_y);
                if (cost >= 0)  // 有效的代价值
                {
                    total_cells++;
                    if (cost >= obstacle_threshold_)
                    {
                        obstacle_cells++;
                    }
                }
            }
        }
    }

    // 如果障碍物占比超过阈值，认为检测到障碍物
    double obstacle_ratio = (total_cells > 0) ? static_cast<double>(obstacle_cells) / total_cells : 0.0;
    bool obstacle_detected = obstacle_ratio > 0.1;  // 10%的区域有障碍物就触发

    if (debug_mode_ && obstacle_detected)
    {
        ROS_WARN_THROTTLE(0.5, "Sector obstacle detection: %d/%d cells above threshold (%.1f%%)", 
                         obstacle_cells, total_cells, obstacle_ratio * 100.0);
    }

    return obstacle_detected;
}

void FrontObstacleDetector::generateSectorAreaPoints(const geometry_msgs::PoseStamped& robot_pose,
                                                    std::vector<std::pair<double, double>>& sector_points)
{
    sector_points.clear();

    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    // 扇形参数
    double half_angle = (sector_angle_ * M_PI / 180.0) / 2.0;  // 转换为弧度的半角
    double start_angle = robot_yaw - half_angle;
    double end_angle = robot_yaw + half_angle;

    // 添加扇形中心点（机器人位置）
    sector_points.push_back({robot_x, robot_y});

    // 生成扇形弧线上的点
    int num_arc_points = 20;  // 弧线分段数
    for (int i = 0; i <= num_arc_points; ++i)
    {
        double angle = start_angle + (end_angle - start_angle) * i / num_arc_points;
        double arc_x = robot_x + sector_radius_ * std::cos(angle);
        double arc_y = robot_y + sector_radius_ * std::sin(angle);
        sector_points.push_back({arc_x, arc_y});
    }

    if (debug_mode_)
    {
        ROS_DEBUG_THROTTLE(1.0, "Generated sector with %zu points, radius: %.2fm, angle: %.1f°", 
                          sector_points.size(), sector_radius_, sector_angle_);
    }
}

bool FrontObstacleDetector::isPointInSector(double point_x, double point_y, 
                                           double robot_x, double robot_y, double robot_yaw)
{
    // 计算点到机器人的距离
    double dx = point_x - robot_x;
    double dy = point_y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 检查距离是否在扇形半径内
    if (distance > sector_radius_)
        return false;

    // 计算点相对于机器人的角度
    double point_angle = std::atan2(dy, dx);
    double angle_diff = point_angle - robot_yaw;

    // 标准化角度差到 [-π, π]
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    // 检查角度是否在扇形范围内
    double half_angle = (sector_angle_ * M_PI / 180.0) / 2.0;  // 转换为弧度的半角
    return std::abs(angle_diff) <= half_angle;
}

} // namespace pure_pursuit_local_planner

