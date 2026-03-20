#include "pure_pursuit_local_planner/pure_pursuit_local_planner.h"
#include <limits>

namespace pure_pursuit_local_planner
{

    PurePursuitLocalPlanner::PurePursuitLocalPlanner()
        : path_received_(false), goal_reached_(false), current_path_index_(0), has_valid_lookahead_(false),
          path_progress_index_(0), smoothed_local_path_received_(false), yaw_adjusting_(false),
          target_pose_received_(false), nav_state_(NavigationState::NORMAL), large_obstacle_failure_count_(0),
          large_obstacle_replan_count_(0), is_on_stairs_(false), near_goal_reference_received_(false)
    {
    }

    PurePursuitLocalPlanner::~PurePursuitLocalPlanner()
    {
    }

    bool PurePursuitLocalPlanner::initialize(ros::NodeHandle &nh)
    {
        nh_ = nh;

        // 加载参数
        loadParameters();

        // 初始化TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化组件
        obstacle_avoidance_ = std::make_unique<ObstacleAvoidance>();
        if (!obstacle_avoidance_->initialize(nh_))
        {
            ROS_ERROR("Failed to initialize obstacle avoidance");
            return false;
        }

        path_tracker_ = std::make_unique<PathTracker>();
        if (!path_tracker_->initialize(nh_))
        {
            ROS_ERROR("Failed to initialize path tracker");
            return false;
        }

        // 只有在启用局部重规划时才初始化局部路径规划器
        if (config_.enable_local_replanning)
        {
            local_path_planner_ = std::unique_ptr<LocalPathPlanner>(new LocalPathPlanner());
            if (!local_path_planner_->initialize(nh_, tf_buffer_))
            {
                ROS_ERROR("Failed to initialize local path planner");
                return false;
            }
            ROS_INFO("Local path planner (A* replanning) enabled");
        }
        else
        {
            ROS_INFO("Local path planner (A* replanning) disabled");
        }

        // 初始化前方障碍物检测器
        front_obstacle_detector_ = std::make_unique<FrontObstacleDetector>();
        if (!front_obstacle_detector_->initialize(nh_, tf_buffer_))
        {
            ROS_ERROR("Failed to initialize front obstacle detector");
            return false;
        }

        // 初始化发布器
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        lookahead_pub_ = nh_.advertise<visualization_msgs::Marker>("lookahead_point", 1);
        path_pub_ = nh_.advertise<nav_msgs::Path>("local_path", 1);
        local_trajectory_pub_ = nh_.advertise<nav_msgs::Path>("local_trajectory", 1);
        velocity_arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("velocity_arrow", 1);
        nav_state_pub_ = nh_.advertise<std_msgs::Bool>("/nav_state", 1);

        // 初始化订阅器
        global_path_sub_ = nh_.subscribe("/global_path", 1,
                         &PurePursuitLocalPlanner::globalPathCallback, this);
        clicked_point_sub_ = nh_.subscribe("/clicked_point", 1,
                         &PurePursuitLocalPlanner::clickedPointCallback, this);
        costmap_sub_ = nh_.subscribe("/local_costmap", 1,
                         &PurePursuitLocalPlanner::costmapCallback, this);
        // scan_sub_ = nh_.subscribe("/scan", 1,
        //                           &PurePursuitLocalPlanner::scanCallback, this);
        smoothed_local_path_sub_ = nh_.subscribe("/pure_pursuit_local_planner/smoothed_local_path", 1,
                            &PurePursuitLocalPlanner::smoothedLocalPathCallback, this);
        target_points_sub_ = nh_.subscribe("/target_points", 1,
                          &PurePursuitLocalPlanner::targetPointsCallback, this);
        emergency_stop_sub_ = nh_.subscribe("/emergency_stop", 1, &PurePursuitLocalPlanner::emergencyStopCallback, this);
        stairs_flag_sub_ = nh_.subscribe("/stairs_flag", 1, &PurePursuitLocalPlanner::stairsFlagCallback, this);

            // 点云订阅器（楼梯模式下直接用点云检测障碍物）
            pointcloud_sub_ = nh_.subscribe("/livox/lidar", 1,
                      &PurePursuitLocalPlanner::pointCloudCallback, this);

        // 初始化控制定时器
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.control_frequency),
                                         &PurePursuitLocalPlanner::controlCallback, this);

        last_control_time_ = ros::Time::now();

        ROS_INFO("Pure Pursuit Local Planner initialized");
        return true;
    }

    void PurePursuitLocalPlanner::emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        emergency_stop_active_ = msg->data;
    }

    void PurePursuitLocalPlanner::stairsFlagCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        // 检查楼梯状态是否改变
        if (is_on_stairs_ != msg->data)
        {
            last_stair_state_change_ = ros::Time::now();
            is_on_stairs_ = msg->data;

            if (is_on_stairs_)
            {
                ROS_INFO("Stairs detected - entering stair climbing mode");
                ROS_INFO("When on stairs: obstacles will cause STOP instead of avoidance");
            }
            else
            {
                ROS_INFO("Stairs cleared - exiting stair climbing mode");
            }
        }
        else
        {
            // 状态未改变，更新时间戳
            last_stair_state_change_ = ros::Time::now();
        }
    }

    void PurePursuitLocalPlanner::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (front_obstacle_detector_)
        {
            front_obstacle_detector_->updatePointCloud(*msg);
        }
    }

    void PurePursuitLocalPlanner::loadParameters()
    {
        // Pure Pursuit parameters
        nh_.param("lookahead_distance_min", config_.lookahead_distance_min, 1.0);
        nh_.param("lookahead_distance_max", config_.lookahead_distance_max, 3.0);
        nh_.param("goal_tolerance", config_.goal_tolerance, 0.2);

        // Speed parameters
        nh_.param("max_linear_velocity", config_.max_linear_velocity, 1.0);
        nh_.param("max_linear_velocity_y", config_.max_linear_velocity_y, 1.0);
        nh_.param("max_angular_velocity", config_.max_angular_velocity, 1.0);
        nh_.param("min_linear_velocity", config_.min_linear_velocity, 0.1);
        nh_.param("acceleration_limit", config_.acceleration_limit, 0.5);
        nh_.param("deceleration_limit", config_.deceleration_limit, 0.8);

        // Obstacle avoidance parameters
        nh_.param("obstacle_detection_range", config_.obstacle_detection_range, 2.0);
        nh_.param("safety_distance", config_.safety_distance, 0.3);

        // Control parameters
        nh_.param("control_frequency", config_.control_frequency, 20.0);
        nh_.param("transform_tolerance", config_.transform_tolerance, 0.1);

        // Orientation adjustment parameters
        nh_.param("yaw_tolerance", config_.yaw_tolerance, 0.1);
        nh_.param("yaw_adjustment_gain", config_.yaw_adjustment_gain, 1.0);

        // Local planning parameters
        nh_.param("enable_local_replanning", config_.enable_local_replanning, false);
        nh_.param("use_smoothed_path", config_.use_smoothed_path, false);

        // Dynamic obstacle avoidance parameters
        nh_.param("dynamic_obstacle_avoidance/large_obstacle_failure_threshold",
                  config_.large_obstacle_failure_threshold, 3);
        nh_.param("dynamic_obstacle_avoidance/recovery_check_duration",
                  config_.recovery_check_duration, 5.0);
        nh_.param("dynamic_obstacle_avoidance/large_obstacle_reset_timeout",
                  config_.large_obstacle_reset_timeout, 30.0);
        nh_.param("dynamic_obstacle_avoidance/enable_dynamic_obstacle_avoidance",
                  config_.enable_dynamic_obstacle_avoidance, true);
        nh_.param("near_goal_obstacle_stop_enable",
                  config_.near_goal_obstacle_stop_enable, false);
        nh_.param("near_goal_obstacle_stop_distance",
                  config_.near_goal_obstacle_stop_distance, 1.0);
        ROS_INFO("Near-goal obstacle stop config: enable=%s, distance=%.2f m",
                 config_.near_goal_obstacle_stop_enable ? "true" : "false",
                 config_.near_goal_obstacle_stop_distance);

        // Frame IDs
        nh_.param<std::string>("global_frame", config_.global_frame, "map");
        nh_.param<std::string>("robot_frame", config_.robot_frame, "base_link");
    }

    bool PurePursuitLocalPlanner::setGlobalPath(const nav_msgs::Path &global_path)
    {
        if (global_path.poses.empty())
        {
            ROS_WARN("Received empty global path");
            return false;
        }

        global_path_ = global_path;
        path_received_ = true;
        goal_reached_ = false;
        yaw_adjusting_ = false;        // 新增：重置朝向调整状态
        current_path_index_ = 0;
        path_progress_index_ = 0;     // 新增：重置路径进度，防止回头
        has_valid_lookahead_ = false; // 重置前瞻点状态

        // 重置动态障碍物避障状态
        nav_state_ = NavigationState::NORMAL;
        large_obstacle_failure_count_ = 0;
        large_obstacle_replan_count_ = 0;
        ROS_INFO("Dynamic obstacle avoidance state reset to NORMAL");

        // 发布导航开始状态
        std_msgs::Bool nav_state_msg;
        nav_state_msg.data = false;
        nav_state_pub_.publish(nav_state_msg);

        // 将路径传递给路径跟踪器
        if (path_tracker_)
        {
            path_tracker_->setPath(global_path_);
        }

        // 只有在启用局部重规划时才将路径传递给局部路径规划器
        if (config_.enable_local_replanning && local_path_planner_)
        {
            local_path_planner_->setGlobalPath(global_path_);
            ROS_INFO("Global path passed to local planner for A* replanning");
        }
        else
        {
            ROS_INFO("Local replanning disabled, using global path directly");
        }

        ROS_INFO("Global path set with %zu poses", global_path_.poses.size());
        return true;
    }

    bool PurePursuitLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!path_received_)
        {
            ROS_WARN_THROTTLE(1.0, "No global path received");
            cmd_vel = geometry_msgs::Twist();
            return false;
        }

        if (goal_reached_)
        {
            cmd_vel = geometry_msgs::Twist();
            return true;
        }

        // 获取机器人当前位姿
        geometry_msgs::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose))
        {
            ROS_WARN_THROTTLE(1.0, "Failed to get robot pose");
            cmd_vel = geometry_msgs::Twist();
            return false;
        }

        // 近目标安全增强：检测到大障碍物时，优先急停
        if (config_.near_goal_obstacle_stop_enable &&
            front_obstacle_detector_ &&
            near_goal_reference_received_)
        {
            double distance_to_goal = calculateDistance(robot_pose.pose.position, near_goal_reference_point_);
            if (distance_to_goal < config_.near_goal_obstacle_stop_distance)
            {
                bool goal_region_occupied = front_obstacle_detector_->isGoalRegionOccupied(near_goal_reference_point_);
                ROS_INFO_THROTTLE(1.0,
                    "Near-target occupancy check active: robot=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) ref_dist=%.2f m threshold=%.2f m goal_region_occupied=%s",
                    robot_pose.pose.position.x,
                    robot_pose.pose.position.y,
                    robot_pose.pose.position.z,
                    near_goal_reference_point_.x,
                    near_goal_reference_point_.y,
                    near_goal_reference_point_.z,
                    distance_to_goal,
                    config_.near_goal_obstacle_stop_distance,
                    goal_region_occupied ? "true" : "false");
                if (goal_region_occupied)
                {
                    cmd_vel = geometry_msgs::Twist();
                    ROS_WARN_THROTTLE(0.5,
                        "Near-target safety stop: goal region occupied, robot=(%.3f, %.3f, %.3f), goal=(%.3f, %.3f, %.3f), ref_dist=%.2f m",
                        robot_pose.pose.position.x,
                        robot_pose.pose.position.y,
                        robot_pose.pose.position.z,
                        near_goal_reference_point_.x,
                        near_goal_reference_point_.y,
                        near_goal_reference_point_.z,
                        distance_to_goal);
                    return true;
                }
            }
        }
        else if (config_.near_goal_obstacle_stop_enable && !near_goal_reference_received_)
        {
            ROS_WARN_THROTTLE(2.0,
                              "Near-target obstacle stop enabled, but no reference point from /clicked_point or /target_points; skipping near-target stop");
        }

        // 检查是否到达目标点
        if (isGoalReached())
        {
            // 如果有目标朝向且还未到达目标朝向，则进行朝向调整
            if (target_pose_received_ && !isTargetOrientationReached(robot_pose))
            {
                if (!yaw_adjusting_)
                {
                    yaw_adjusting_ = true;
                    ROS_INFO("Goal position reached! Starting orientation adjustment...");
                }
                
                // 计算朝向调整控制指令
                if (computeOrientationAdjustmentCommand(robot_pose, cmd_vel))
                {
                    return true;
                }
                else
                {
                    cmd_vel = geometry_msgs::Twist();
                    return false;
                }
            }
            else
            {
                cmd_vel = geometry_msgs::Twist();
                goal_reached_ = true;
                yaw_adjusting_ = false;
                
                // 发布导航完成状态
                std_msgs::Bool nav_state_msg;
                nav_state_msg.data = true;
                nav_state_pub_.publish(nav_state_msg);
                
                ROS_INFO("Goal reached!");
                return true;
            }
        }

        // emergency stop 逻辑
        if (emergency_stop_active_)
        {
            cmd_vel = geometry_msgs::Twist();
            ROS_WARN_THROTTLE(1.0, "Emergency stop active, cmd_vel forced to zero");
            return true;
        }

        // 计算前瞻点
        geometry_msgs::PointStamped lookahead_point;
        if (!computeLookaheadPoint(robot_pose, lookahead_point))
        {
            ROS_WARN_THROTTLE(1.0, "Failed to compute lookahead point");
            cmd_vel = geometry_msgs::Twist();
            return false;
        }

        // 计算Pure Pursuit控制指令
        if (!computePurePursuitCommand(robot_pose, lookahead_point, cmd_vel))
        {
            ROS_WARN_THROTTLE(1.0, "Failed to compute pure pursuit command");
            cmd_vel = geometry_msgs::Twist();
            return false;
        }

        // 小障碍物势场避障（仅在 SMALL_OBSTACLE_AVOIDING 且非楼梯模式下启用）
        // 楼梯模式下由点云停障逻辑主导，避免势场持续降速导致“挪不动”
        const bool stair_mode_active_for_pf = front_obstacle_detector_ &&
                                              (is_on_stairs_ || front_obstacle_detector_->isDownstairsScene());
        if (nav_state_ == NavigationState::SMALL_OBSTACLE_AVOIDING &&
            obstacle_avoidance_ &&
            !stair_mode_active_for_pf)
        {
            cmd_vel = obstacle_avoidance_->computePotentialFieldAvoidance(cmd_vel, robot_pose, lookahead_point);
        }

        // 前方障碍物检测和速度调整
        // 楼梯模式下仅抑制“角速度放大”，保留线速度减速，避免转向过猛
        if (front_obstacle_detector_)
        {
            cmd_vel = front_obstacle_detector_->adjustVelocityForObstacles(cmd_vel, robot_pose);
        }

        // 动态障碍物避障逻辑（小障碍重规划/大障碍停止）
        if (config_.enable_dynamic_obstacle_avoidance && front_obstacle_detector_)
        {
            if (handleDynamicObstacleAvoidance(robot_pose, cmd_vel))
            {
                // 返回true表示需要停止执行导航
                cmd_vel = geometry_msgs::Twist();  // 停止运动
                return true;
            }
        }

        // 应用速度限制
        const double pre_limit_linear_x = cmd_vel.linear.x;
        const double pre_limit_angular_z = cmd_vel.angular.z;
        applyVelocityLimits(cmd_vel);
        ROS_INFO_THROTTLE(1.0,
            "[VelDiag][CmdVel] nav_state=%d, stairs_flag=%d, pre_limit(vx=%.3f,wz=%.3f) -> final(vx=%.3f,wz=%.3f)",
            static_cast<int>(nav_state_),
            is_on_stairs_ ? 1 : 0,
            pre_limit_linear_x,
            pre_limit_angular_z,
            cmd_vel.linear.x,
            cmd_vel.angular.z);

        // 发布可视化信息
        publishVisualization(robot_pose, lookahead_point);

        // 发布局部轨迹和速度可视化
        publishLocalTrajectory(robot_pose, cmd_vel);
        publishVelocityArrow(robot_pose, cmd_vel);

        last_cmd_vel_ = cmd_vel;
        last_control_time_ = ros::Time::now();

        return true;
    }

    bool PurePursuitLocalPlanner::isGoalReached()
    {
        if (!path_received_ || global_path_.poses.empty())
            return false;

        geometry_msgs::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose))
            return false;

        const auto &goal_pose = global_path_.poses.back();
        double distance = calculateDistance(robot_pose.pose.position, goal_pose.pose.position);

        return distance < config_.goal_tolerance;
    }

    bool PurePursuitLocalPlanner::getRobotPose(geometry_msgs::PoseStamped &robot_pose)
    {
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                config_.global_frame, config_.robot_frame, ros::Time(0),
                ros::Duration(config_.transform_tolerance));

            robot_pose.header.frame_id = config_.global_frame;
            robot_pose.header.stamp = transform.header.stamp;
            robot_pose.pose.position.x = transform.transform.translation.x;
            robot_pose.pose.position.y = transform.transform.translation.y;
            robot_pose.pose.position.z = transform.transform.translation.z;
            robot_pose.pose.orientation = transform.transform.rotation;

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "Failed to get robot pose: %s", ex.what());
            return false;
        }
    }

    void PurePursuitLocalPlanner::controlCallback(const ros::TimerEvent &event)
    {
        geometry_msgs::Twist cmd_vel;
        if (computeVelocityCommands(cmd_vel))
        {
            cmd_vel_pub_.publish(cmd_vel);
        }
    }

    void PurePursuitLocalPlanner::globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        setGlobalPath(*msg);
    }

    void PurePursuitLocalPlanner::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        near_goal_reference_point_ = msg->point;
        near_goal_reference_received_ = true;
        ROS_INFO_THROTTLE(1.0, "Near-goal reference updated from /clicked_point: (%.3f, %.3f, %.3f)",
            near_goal_reference_point_.x,
            near_goal_reference_point_.y,
            near_goal_reference_point_.z);
    }

    void PurePursuitLocalPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        if (obstacle_avoidance_)
        {
            obstacle_avoidance_->updateCostmap(*msg);
        }
        
        // 只有在启用局部重规划时才更新局部路径规划器的代价地图
        if (config_.enable_local_replanning && local_path_planner_)
        {
            local_path_planner_->updateCostmap(*msg);
        }

        // 更新前方障碍物检测器的代价地图
        if (front_obstacle_detector_)
        {
            front_obstacle_detector_->updateCostmap(*msg);
        }
    }

    void PurePursuitLocalPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // 人工势场方法不使用激光雷达数据，仅保留回调函数以避免编译错误
        // 可以在这里添加其他激光雷达相关的处理逻辑（如果需要）
    }

    void PurePursuitLocalPlanner::smoothedLocalPathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        // 只有在启用平滑路径时才处理
        if (config_.use_smoothed_path)
        {
            smoothed_local_path_ = *msg;
            smoothed_local_path_received_ = !msg->poses.empty();
            
            if (smoothed_local_path_received_)
            {
                ROS_DEBUG("Received smoothed local path with %zu poses", msg->poses.size());
            }
        }
        else
        {
            ROS_DEBUG("Smoothed path callback received but smoothed path usage is disabled");
        }
    }

    void PurePursuitLocalPlanner::targetPointsCallback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        target_pose_ = *msg;
        target_pose_received_ = true;
        near_goal_reference_point_ = target_pose_.position;
        near_goal_reference_received_ = true;
        ROS_DEBUG("Received target pose: position(%.3f, %.3f, %.3f), orientation(%.3f, %.3f, %.3f, %.3f)",
                  target_pose_.position.x, target_pose_.position.y, target_pose_.position.z,
                  target_pose_.orientation.x, target_pose_.orientation.y, 
                  target_pose_.orientation.z, target_pose_.orientation.w);
    }

    bool PurePursuitLocalPlanner::computeLookaheadPoint(const geometry_msgs::PoseStamped &robot_pose,
                                                        geometry_msgs::PointStamped &lookahead_point)
    {
        // 根据参数决定路径选择优先级
        nav_msgs::Path path_to_use;
        bool use_local_path = false;
        std::string path_type;
        
        if (config_.enable_local_replanning)
        {
            // 启用局部重规划时的优先级：平滑后局部路径 > 原始局部路径 > 全局路径
            if (config_.use_smoothed_path && smoothed_local_path_received_ && !smoothed_local_path_.poses.empty())
            {
                path_to_use = smoothed_local_path_;
                use_local_path = true;
                path_type = "smoothed local path";
                ROS_DEBUG("Using smoothed local path for lookahead point computation");
            }
            else if (local_path_planner_ && local_path_planner_->hasValidLocalPath())
            {
                path_to_use = local_path_planner_->getLocalPath();
                use_local_path = true;
                path_type = "original local path";
                ROS_DEBUG("Using original local path for lookahead point computation");
            }
            else if (!global_path_.poses.empty())
            {
                path_to_use = global_path_;
                use_local_path = false;
                path_type = "global path";
                ROS_DEBUG("Using global path for lookahead point computation (fallback)");
            }
            else
            {
                return false;
            }
        }
        else
        {
            // 禁用局部重规划时，直接使用全局路径
            if (!global_path_.poses.empty())
            {
                path_to_use = global_path_;
                use_local_path = false;
                path_type = "global path";
                ROS_DEBUG("Local replanning disabled, using global path directly");
            }
            else
            {
                return false;
            }
        }

        if (path_to_use.poses.empty())
            return false;

        // 计算前瞻距离
        double current_velocity = std::hypot(last_cmd_vel_.linear.x, last_cmd_vel_.linear.y);

        // 计算速度因子：将当前速度映射到 [0, 1] 范围
        double velocity_factor = 0.0;
        if (config_.max_linear_velocity > 0.0)
        {
            velocity_factor = std::min(current_velocity / config_.max_linear_velocity, 1.0);
        }

        // 根据速度因子在最小和最大前瞻距离之间插值
        double lookahead_distance = config_.lookahead_distance_min +
                                    velocity_factor * (config_.lookahead_distance_max - config_.lookahead_distance_min);

        // 机器人当前位置
        double robot_x = robot_pose.pose.position.x;
        double robot_y = robot_pose.pose.position.y;

        // 对于全局路径，需要更新路径进度防止回头；对于局部路径则不需要
        size_t start_index = 0;
        if (!use_local_path)
        {
            // 使用全局路径时，更新路径进度防止回头
            updatePathProgress(robot_pose);
            start_index = path_progress_index_;
        }

        // 查找以机器人为中心、前瞻距离为半径的圆与路径的交点
        std::vector<int> intersection_indices;
        std::vector<geometry_msgs::Point> intersection_points;

        for (size_t i = start_index; i < path_to_use.poses.size() - 1; ++i)
        {
            const auto &p1 = path_to_use.poses[i].pose.position;
            const auto &p2 = path_to_use.poses[i + 1].pose.position;

            // 计算线段与圆的交点
            std::vector<geometry_msgs::Point> segment_intersections =
                computeCircleLineIntersection(robot_x, robot_y, lookahead_distance, p1, p2);

            for (const auto &intersection : segment_intersections)
            {
                intersection_indices.push_back(i);
                intersection_points.push_back(intersection);
            }
        }

        // 如果有交点，选择最靠近终点的交点
        if (!intersection_points.empty())
        {
            int best_index = 0;
            double max_progress = -1.0;

            for (size_t i = 0; i < intersection_points.size(); ++i)
            {
                // 对于局部路径，直接使用索引作为进度；对于全局路径，计算累积距离
                double progress;
                if (use_local_path)
                {
                    progress = intersection_indices[i];  // 局部路径直接使用索引
                }
                else
                {
                    progress = calculatePathProgress(intersection_indices[i], intersection_points[i]);
                }
                
                if (progress > max_progress)
                {
                    max_progress = progress;
                    best_index = i;
                }
            }

            lookahead_point.header.frame_id = config_.global_frame;
            lookahead_point.header.stamp = ros::Time::now();
            lookahead_point.point = intersection_points[best_index];

            // 更新当前路径索引
            current_path_index_ = intersection_indices[best_index];

            // 保存当前前瞻点作为下次的备选
            last_lookahead_point_ = lookahead_point;
            has_valid_lookahead_ = true;

            return true;
        }

        // 如果没有交点，选择从当前进度开始的最远可见点作为前瞻点
        if (use_local_path)
        {
            // 对于局部路径，选择路径终点或距离起点一定距离的点
            size_t target_index = std::min(start_index + 10, path_to_use.poses.size() - 1);
            
            const auto &target_pose = path_to_use.poses[target_index];
            lookahead_point.header.frame_id = config_.global_frame;
            lookahead_point.header.stamp = ros::Time::now();
            lookahead_point.point = target_pose.pose.position;

            current_path_index_ = target_index;

            // 保存当前前瞻点
            last_lookahead_point_ = lookahead_point;
            has_valid_lookahead_ = true;

            ROS_DEBUG("No intersection found on %s, using target point at index %zu", path_type.c_str(), target_index);
            return true;
        }
        else if (path_progress_index_ < global_path_.poses.size())
        {
            // 对于全局路径，选择距离当前进度一定距离的点，或者路径终点
            size_t target_index = std::min(path_progress_index_ + 10, global_path_.poses.size() - 1);

            const auto &target_pose = global_path_.poses[target_index];
            lookahead_point.header.frame_id = config_.global_frame;
            lookahead_point.header.stamp = ros::Time::now();
            lookahead_point.point = target_pose.pose.position;

            current_path_index_ = target_index;

            // 保存当前前瞻点
            last_lookahead_point_ = lookahead_point;
            has_valid_lookahead_ = true;

            ROS_DEBUG("No intersection found on %s, using target point at index %zu", path_type.c_str(), target_index);
            return true;
        }

        return false;
    }

    bool PurePursuitLocalPlanner::computePurePursuitCommand(const geometry_msgs::PoseStamped &robot_pose,
                                                            const geometry_msgs::PointStamped &lookahead_point,
                                                            geometry_msgs::Twist &cmd_vel)
    {
        // 计算机器人到前瞻点的向量
        double dx = lookahead_point.point.x - robot_pose.pose.position.x;
        double dy = lookahead_point.point.y - robot_pose.pose.position.y;
        double distance = std::hypot(dx, dy);
        printf("distance_robot_point: %f\r\n", distance);

        if (distance < 1e-6)
        {
            cmd_vel = geometry_msgs::Twist();
            return false;
        }

        // 获取机器人朝向
        double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

        // 计算目标线速度（基于机器人方向和前瞻点方向的点乘）
        double target_velocity = config_.max_linear_velocity;
        if (path_tracker_)
        {
            target_velocity = path_tracker_->computeTargetVelocity(robot_pose, lookahead_point, config_.max_linear_velocity);
        }

        // 计算距离比例因子：距离前瞻点越近，速度越小
        // 使用当前前瞻距离范围进行归一化
        double min_distance_factor = 0.3;  // 最小速度比例（距离很近时）
        double max_distance_factor = 1.0;  // 最大速度比例（距离较远时）
        
        // 将距离映射到 [min_distance_factor, max_distance_factor] 范围
        double distance_factor = min_distance_factor + 
            (max_distance_factor - min_distance_factor) * 
            std::min(distance / config_.lookahead_distance_max, 1.0);
        
        // 应用距离比例因子
        target_velocity *= distance_factor;

        // 计算X方向的速度分量（Y方向设为0）
        cmd_vel.linear.x = target_velocity;
        ROS_INFO_THROTTLE(1.0,
            "[VelDiag][PurePursuit] lookahead_dist=%.3f m, lookahead_max=%.3f m, distance_factor=%.3f, v_after_dist=%.3f",
            distance,
            config_.lookahead_distance_max,
            distance_factor,
            target_velocity);
        cmd_vel.linear.y = 0.0; // 固定设为0，不使用全向运动
        cmd_vel.linear.z = 0.0;

        // 确保最小速度（仅对前进方向）
        // if (std::abs(cmd_vel.linear.x) > 0 && std::abs(cmd_vel.linear.x) < config_.min_linear_velocity)
        // {
        //     cmd_vel.linear.x = std::copysign(config_.min_linear_velocity, cmd_vel.linear.x);
        // }

        // 计算角速度（可选：让机器人保持朝向前瞻点的方向）
        // 对于全向机器人，可以选择保持当前朝向或调整朝向
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = target_yaw - robot_yaw;

        // 标准化角度
        while (yaw_error > M_PI)
            yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI)
            yaw_error += 2.0 * M_PI;

        // 角速度控制（可调节系数）
        double angular_gain = 1.0; // 可以作为参数配置
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = angular_gain * yaw_error;

        // 限制角速度
        cmd_vel.angular.z = std::max(cmd_vel.angular.z, -config_.max_angular_velocity);
        cmd_vel.angular.z = std::min(cmd_vel.angular.z, config_.max_angular_velocity);

        return true;
    }

    void PurePursuitLocalPlanner::applyVelocityLimits(geometry_msgs::Twist &cmd_vel)
    {
        double dt = (ros::Time::now() - last_control_time_).toSec();
        if (dt <= 0.0)
            return;

        // 限制X方向线速度
        cmd_vel.linear.x = std::max(cmd_vel.linear.x, -config_.max_linear_velocity);
        cmd_vel.linear.x = std::min(cmd_vel.linear.x, config_.max_linear_velocity);

        // 限制Y方向线速度
        cmd_vel.linear.y = std::max(cmd_vel.linear.y, -config_.max_linear_velocity_y);
        cmd_vel.linear.y = std::min(cmd_vel.linear.y, config_.max_linear_velocity_y);

        // 限制加速度（X方向）
        double velocity_diff_x = cmd_vel.linear.x - last_cmd_vel_.linear.x;
        double max_velocity_change_x = (velocity_diff_x > 0) ? config_.acceleration_limit * dt : config_.deceleration_limit * dt;

        if (std::abs(velocity_diff_x) > max_velocity_change_x)
        {
            cmd_vel.linear.x = last_cmd_vel_.linear.x +
                               std::copysign(max_velocity_change_x, velocity_diff_x);
        }

        // 限制加速度（Y方向）
        double velocity_diff_y = cmd_vel.linear.y - last_cmd_vel_.linear.y;
        double max_velocity_change_y = (velocity_diff_y > 0) ? config_.acceleration_limit * dt : config_.deceleration_limit * dt;

        if (std::abs(velocity_diff_y) > max_velocity_change_y)
        {
            cmd_vel.linear.y = last_cmd_vel_.linear.y +
                               std::copysign(max_velocity_change_y, velocity_diff_y);
        }

        // 限制角速度
        cmd_vel.angular.z = std::max(cmd_vel.angular.z, -config_.max_angular_velocity);
        cmd_vel.angular.z = std::min(cmd_vel.angular.z, config_.max_angular_velocity);
    }

    void PurePursuitLocalPlanner::publishVisualization(const geometry_msgs::PoseStamped &robot_pose,
                                                       const geometry_msgs::PointStamped &lookahead_point)
    {
        // 发布前瞻点标记
        visualization_msgs::Marker marker;
        marker.header.frame_id = config_.global_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "pure_pursuit";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = lookahead_point.point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        lookahead_pub_.publish(marker);

        // 发布局部路径
        if (path_tracker_)
        {
            path_tracker_->publishVisualization(robot_pose, lookahead_point, current_path_index_);
        }
    }

    void PurePursuitLocalPlanner::publishLocalTrajectory(const geometry_msgs::PoseStamped &robot_pose,
                                                         const geometry_msgs::Twist &cmd_vel)
    {
        nav_msgs::Path trajectory = generateLocalTrajectory(robot_pose, cmd_vel);
        local_trajectory_pub_.publish(trajectory);
    }

    void PurePursuitLocalPlanner::publishVelocityArrow(const geometry_msgs::PoseStamped &robot_pose,
                                                       const geometry_msgs::Twist &cmd_vel)
    {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = config_.global_frame;
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "velocity";
        arrow.id = 0;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        // 设置箭头起点
        arrow.pose.position = robot_pose.pose.position;
        arrow.pose.position.z += 0.2; // 稍微抬高避免与地面重叠

        // 全向机器人：计算实际速度方向（结合X和Y方向速度）
        double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

        // 将机器人坐标系下的速度转换到全局坐标系
        double global_vel_x = cmd_vel.linear.x * std::cos(robot_yaw) - cmd_vel.linear.y * std::sin(robot_yaw);
        double global_vel_y = cmd_vel.linear.x * std::sin(robot_yaw) + cmd_vel.linear.y * std::cos(robot_yaw);

        // 计算实际运动方向
        double arrow_yaw = std::atan2(global_vel_y, global_vel_x);

        tf2::Quaternion q;
        q.setRPY(0, 0, arrow_yaw);
        arrow.pose.orientation = tf2::toMsg(q);

        // 根据速度大小设置箭头长度
        double velocity_magnitude = std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);
        double arrow_length = std::max(0.2, velocity_magnitude * 0.5); // 最小长度0.2m

        arrow.scale.x = arrow_length; // 箭头长度
        arrow.scale.y = 0.1;          // 箭头宽度
        arrow.scale.z = 0.1;          // 箭头高度

        // 根据速度大小设置颜色（绿色到红色渐变）
        arrow.color.a = 0.8;
        double max_combined_velocity = std::hypot(config_.max_linear_velocity, config_.max_linear_velocity_y);
        double speed_ratio = velocity_magnitude / max_combined_velocity;
        arrow.color.r = speed_ratio;
        arrow.color.g = 1.0 - speed_ratio;
        arrow.color.b = 0.0;

        velocity_arrow_pub_.publish(arrow);
    }

    nav_msgs::Path PurePursuitLocalPlanner::generateLocalTrajectory(const geometry_msgs::PoseStamped &robot_pose,
                                                                    const geometry_msgs::Twist &cmd_vel,
                                                                    double prediction_time,
                                                                    double dt)
    {
        nav_msgs::Path trajectory;
        trajectory.header.frame_id = config_.global_frame;
        trajectory.header.stamp = ros::Time::now();

        // 获取机器人当前状态
        double x = robot_pose.pose.position.x;
        double y = robot_pose.pose.position.y;
        double theta = tf2::getYaw(robot_pose.pose.orientation);

        double linear_vel = cmd_vel.linear.x;
        double angular_vel = cmd_vel.angular.z;

        // 生成预测轨迹点
        int num_steps = static_cast<int>(prediction_time / dt);
        for (int i = 0; i <= num_steps; ++i)
        {
            double t = i * dt;

            geometry_msgs::PoseStamped pose;
            pose.header = trajectory.header;
            pose.header.stamp = ros::Time::now() + ros::Duration(t);

            if (std::abs(angular_vel) < 1e-6)
            {
                // 直线运动
                pose.pose.position.x = x + linear_vel * t * std::cos(theta);
                pose.pose.position.y = y + linear_vel * t * std::sin(theta);
                pose.pose.position.z = robot_pose.pose.position.z;

                tf2::Quaternion q;
                q.setRPY(0, 0, theta);
                pose.pose.orientation = tf2::toMsg(q);
            }
            else
            {
                // 圆弧运动
                double radius = linear_vel / angular_vel;
                double delta_theta = angular_vel * t;

                pose.pose.position.x = x + radius * (std::sin(theta + delta_theta) - std::sin(theta));
                pose.pose.position.y = y - radius * (std::cos(theta + delta_theta) - std::cos(theta));
                pose.pose.position.z = robot_pose.pose.position.z;

                tf2::Quaternion q;
                q.setRPY(0, 0, theta + delta_theta);
                pose.pose.orientation = tf2::toMsg(q);
            }

            trajectory.poses.push_back(pose);
        }

        return trajectory;
    }

    double PurePursuitLocalPlanner::calculateDistance(const geometry_msgs::Point &p1,
                                                      const geometry_msgs::Point &p2)
    {
        return std::hypot(std::hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
    }

    double PurePursuitLocalPlanner::normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    std::vector<geometry_msgs::Point> PurePursuitLocalPlanner::computeCircleLineIntersection(
        double circle_x, double circle_y, double radius,
        const geometry_msgs::Point &line_start, const geometry_msgs::Point &line_end)
    {
        std::vector<geometry_msgs::Point> intersections;

        // 线段向量
        double dx = line_end.x - line_start.x;
        double dy = line_end.y - line_start.y;

        // 从线段起点到圆心的向量
        double fx = line_start.x - circle_x;
        double fy = line_start.y - circle_y;

        // 求解二次方程 a*t^2 + b*t + c = 0
        double a = dx * dx + dy * dy;
        double b = 2.0 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - radius * radius;

        double discriminant = b * b - 4.0 * a * c;

        if (discriminant < 0.0 || a < 1e-8)
        {
            // 无交点或线段长度为0
            return intersections;
        }

        double sqrt_discriminant = std::sqrt(discriminant);
        double t1 = (-b - sqrt_discriminant) / (2.0 * a);
        double t2 = (-b + sqrt_discriminant) / (2.0 * a);

        // 检查交点是否在线段上 (0 <= t <= 1)
        if (t1 >= 0.0 && t1 <= 1.0)
        {
            geometry_msgs::Point intersection;
            intersection.x = line_start.x + t1 * dx;
            intersection.y = line_start.y + t1 * dy;
            intersection.z = line_start.z;
            intersections.push_back(intersection);
        }

        if (t2 >= 0.0 && t2 <= 1.0 && std::abs(t2 - t1) > 1e-8)
        {
            geometry_msgs::Point intersection;
            intersection.x = line_start.x + t2 * dx;
            intersection.y = line_start.y + t2 * dy;
            intersection.z = line_start.z;
            intersections.push_back(intersection);
        }

        return intersections;
    }

    double PurePursuitLocalPlanner::calculatePathProgress(int segment_index, const geometry_msgs::Point &point)
    {
        if (segment_index < 0 || segment_index >= static_cast<int>(global_path_.poses.size() - 1))
            return -1.0;

        // 计算累积路径长度到该段的起点
        double cumulative_length = 0.0;
        for (int i = 0; i < segment_index; ++i)
        {
            cumulative_length += calculateDistance(global_path_.poses[i].pose.position,
                                                   global_path_.poses[i + 1].pose.position);
        }

        // 计算点在当前段上的距离
        const auto &segment_start = global_path_.poses[segment_index].pose.position;
        double segment_progress = calculateDistance(segment_start, point);

        return cumulative_length + segment_progress;
    }

    void PurePursuitLocalPlanner::updatePathProgress(const geometry_msgs::PoseStamped &robot_pose)
    {
        if (global_path_.poses.empty())
            return;

        double robot_x = robot_pose.pose.position.x;
        double robot_y = robot_pose.pose.position.y;

        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = path_progress_index_;

        // 从当前进度开始向前搜索最近点（允许少量向前搜索）
        size_t search_start = path_progress_index_;
        size_t search_end = std::min(path_progress_index_ + 20, global_path_.poses.size());

        for (size_t i = search_start; i < search_end; ++i)
        {
            const auto &waypoint = global_path_.poses[i].pose.position;
            double distance = std::hypot(robot_x - waypoint.x, robot_y - waypoint.y);

            if (distance < min_distance)
            {
                min_distance = distance;
                closest_index = i;
            }
        }

        // 只允许进度向前更新，防止回头
        if (closest_index > path_progress_index_)
        {
            path_progress_index_ = closest_index;
            ROS_DEBUG("Path progress updated to index %zu", path_progress_index_);
        }
    }

    bool PurePursuitLocalPlanner::isTargetOrientationReached(const geometry_msgs::PoseStamped &robot_pose)
    {
        if (!target_pose_received_)
            return true;

        double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
        double target_yaw = tf2::getYaw(target_pose_.orientation);
        
        double yaw_error = std::abs(normalizeAngle(target_yaw - robot_yaw));
        
        return yaw_error < config_.yaw_tolerance;
    }

    bool PurePursuitLocalPlanner::computeOrientationAdjustmentCommand(const geometry_msgs::PoseStamped &robot_pose,
                                                                     geometry_msgs::Twist &cmd_vel)
    {
        if (!target_pose_received_)
            return false;

        double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
        double target_yaw = tf2::getYaw(target_pose_.orientation);
        
        double yaw_error = normalizeAngle(target_yaw - robot_yaw);
        
        // 设置线速度为0，只调整角速度
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        
        // 计算角速度，使用比例控制
        cmd_vel.angular.z = config_.yaw_adjustment_gain * yaw_error;
        
        // 限制角速度
        cmd_vel.angular.z = std::max(cmd_vel.angular.z, -config_.max_angular_velocity);
        cmd_vel.angular.z = std::min(cmd_vel.angular.z, config_.max_angular_velocity);
        
        ROS_DEBUG("Orientation adjustment: robot_yaw=%.3f, target_yaw=%.3f, yaw_error=%.3f, angular_z=%.3f",
                  robot_yaw, target_yaw, yaw_error, cmd_vel.angular.z);
        
        return true;
    }

    void PurePursuitLocalPlanner::updateNavigationState(NavigationState new_state)
    {
        if (nav_state_ != new_state)
        {
            const char* state_names[] = {"NORMAL", "SMALL_OBSTACLE_AVOIDING", "LARGE_OBSTACLE_REPLANNING",
                                       "LARGE_OBSTACLE_STOPPED", "NAVIGATION_FAILED"};
            ROS_INFO("Navigation state changed: %s -> %s",
                     state_names[static_cast<int>(nav_state_)],
                     state_names[static_cast<int>(new_state)]);
            nav_state_ = new_state;
            last_state_change_time_ = ros::Time::now();
        }
    }

    bool PurePursuitLocalPlanner::attemptReplanAroundLargeObstacle()
    {
        // 检查是否启用了局部重规划
        if (!config_.enable_local_replanning || !local_path_planner_)
        {
            ROS_WARN("Local replanning not enabled or not available, cannot replan around large obstacle");
            return false;
        }

        // 增加重规划尝试计数
        large_obstacle_replan_count_++;

        // 获取当前机器人位姿
        geometry_msgs::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose))
        {
            ROS_WARN("Failed to get robot pose for replanning");
            return false;
        }

        // LocalPathPlanner 会在后台线程中自动重新规划
        // 我们只需要检查它是否产生了有效的局部路径
        ROS_INFO("Checking A* replanning status (attempt %d/%d)",
                 large_obstacle_replan_count_, config_.large_obstacle_failure_threshold);

        // 检查是否有有效的局部路径
        bool replan_success = local_path_planner_->hasValidLocalPath();

        if (replan_success)
        {
            ROS_INFO("A* replanning succeeded: valid local path available");
        }
        else
        {
            ROS_WARN("A* replanning failed: no valid local path available");
        }

        return replan_success;
    }

    bool PurePursuitLocalPlanner::handleDynamicObstacleAvoidance(const geometry_msgs::PoseStamped& robot_pose,
                                                                geometry_msgs::Twist& cmd_vel)
    {
        // 检测前方障碍物类型
        ObstacleType obstacle_type = front_obstacle_detector_->detectObstacleType(robot_pose);

        ros::Time current_time = ros::Time::now();

        // ===== 爬楼梯时的特殊处理 =====
            // ========== 爬楼梯特殊处理：用点云直接检测，不依赖costmap ==========
            // 兼容 /stairs_flag 抖动：只要 /stairs_scene=-1（下楼梯）也强制进入楼梯停障模式
            const bool downstairs_scene_active = front_obstacle_detector_ && front_obstacle_detector_->isDownstairsScene();
            if (is_on_stairs_ || downstairs_scene_active)
            {
                if (downstairs_scene_active && !is_on_stairs_)
                {
                    ROS_WARN_THROTTLE(1.0,
                                      "Downstairs scene active (/stairs_scene=-1) while /stairs_flag=false, forcing stair STOP mode");
                }
                // 楼梯模式下不使用 SMALL_OBSTACLE_AVOIDING 势场，避免残留状态持续降速
                if (nav_state_ == NavigationState::SMALL_OBSTACLE_AVOIDING)
                {
                    updateNavigationState(NavigationState::NORMAL);
                    ROS_INFO_THROTTLE(1.0, "Stair mode active: reset nav_state SMALL_OBSTACLE_AVOIDING -> NORMAL");
                }
                ObstacleType stair_obstacle = ObstacleType::NONE;
                if (front_obstacle_detector_)
                {
                    // 使用点云直接检测，绕过costmap
                    stair_obstacle = front_obstacle_detector_->detectObstacleFromPointCloud(robot_pose);
                }

                if (stair_obstacle != ObstacleType::NONE)
                {
                    cmd_vel = geometry_msgs::Twist();  // 停车
                    ROS_WARN_THROTTLE(1.0, "Stair mode (pointcloud): Obstacle detected, STOPPING");
                    return true;
                }

                // 无障碍物，继续爬楼梯
                ROS_DEBUG_THROTTLE(2.0, "Stair mode (pointcloud): Path clear");
                return false;
            }
        // ===== 爬楼梯处理结束 =====

        switch (nav_state_)
        {
            case NavigationState::NORMAL:
            {
                // 正常状态：根据障碍物类型决定后续动作
                if (obstacle_type == ObstacleType::SMALL)
                {
                    // 遇到小障碍物，进入避障状态（势场避障绕行）
                    updateNavigationState(NavigationState::SMALL_OBSTACLE_AVOIDING);
                    ROS_INFO("Small obstacle detected, entering potential field avoidance mode");
                }
                else if (obstacle_type == ObstacleType::LARGE)
                {
                    // 遇到大障碍物，进入重规划状态
                    last_large_obstacle_time_ = current_time;  // 记录大障碍物首次检测时间
                    updateNavigationState(NavigationState::LARGE_OBSTACLE_REPLANNING);
                    ROS_INFO("Large obstacle detected, entering A* replanning mode");
                }
                break;
            }

            case NavigationState::SMALL_OBSTACLE_AVOIDING:
            {
                // 小障碍物势场避障状态
                if (obstacle_type == ObstacleType::NONE)
                {
                    // 障碍物已绕开，回到正常状态
                    updateNavigationState(NavigationState::NORMAL);
                    ROS_INFO("Small obstacle avoided, returning to normal navigation");
                }
                else if (obstacle_type == ObstacleType::LARGE)
                {
                    // 避障过程中发现大障碍物，进入重规划状态
                    last_large_obstacle_time_ = current_time;  // 记录大障碍物检测时间
                    updateNavigationState(NavigationState::LARGE_OBSTACLE_REPLANNING);
                    ROS_INFO("Large obstacle detected during avoidance, entering A* replanning mode");
                }
                // 如果仍然是小障碍物，继续势场避障（保持当前状态）
                break;
            }

            case NavigationState::LARGE_OBSTACLE_REPLANNING:
            {
                // 大障碍物重规划状态
                if (obstacle_type == ObstacleType::NONE)
                {
                    // 重规划成功，无障碍物，回到正常状态
                    updateNavigationState(NavigationState::NORMAL);
                    ROS_INFO("A* replanning successful, returning to normal navigation");
                }
                else if (obstacle_type == ObstacleType::LARGE)
                {
                    // 重规划失败，仍有大障碍物
                    // 只在调用 attemptReplanAroundLargeObstacle() 时才计数一次
                    if (attemptReplanAroundLargeObstacle())
                    {
                        // 重规划成功，但检测器仍报告障碍（可能是检测延迟）
                        // 稍等一会再检查
                        ROS_DEBUG("Replanning succeeded, but obstacle still detected");
                        break;
                    }

                    // 重规划失败，增加失败计数
                    large_obstacle_failure_count_++;
                    ROS_WARN("A* replanning failed! Failure count: %d/%d",
                             large_obstacle_failure_count_, config_.large_obstacle_failure_threshold);

                    if (large_obstacle_failure_count_ >= config_.large_obstacle_failure_threshold)
                    {
                        // 达到阈值，进入停止状态
                        updateNavigationState(NavigationState::LARGE_OBSTACLE_STOPPED);
                        ROS_ERROR("Large obstacle failure threshold (%d) reached, stopping navigation",
                                  config_.large_obstacle_failure_threshold);
                    }
                }
                break;
            }

            case NavigationState::LARGE_OBSTACLE_STOPPED:
            {
                // 大障碍物停止状态
                cmd_vel = geometry_msgs::Twist();  // 强制停止

                if (obstacle_type == ObstacleType::NONE)
                {
                    // 检测不到障碍物，检查持续时间
                    double time_since_last_obstacle = (current_time - last_large_obstacle_time_).toSec();
                    if (time_since_last_obstacle >= config_.recovery_check_duration)
                    {
                        // 5秒内一直没检测到大障碍物，恢复导航
                        large_obstacle_failure_count_ = 0;  // 重置计数器
                        large_obstacle_replan_count_ = 0;  // 重置重规划计数器
                        updateNavigationState(NavigationState::NORMAL);
                        ROS_INFO("Large obstacle removed for %.1f seconds, resuming navigation",
                                 time_since_last_obstacle);
                    }
                    else
                    {
                        ROS_DEBUG_THROTTLE(1.0, "Waiting %.1f more seconds to confirm obstacle removal...",
                                          config_.recovery_check_duration - time_since_last_obstacle);
                    }
                }
                else
                {
                    // 仍然检测到大障碍物，更新时间戳
                    last_large_obstacle_time_ = current_time;

                    // 检查30秒超时，防止不同地点障碍物累加
                    double time_in_stop_state = (current_time - last_state_change_time_).toSec();
                    if (time_in_stop_state >= config_.large_obstacle_reset_timeout)
                    {
                        // 30秒超时，强制复位
                        ROS_WARN("Large obstacle reset timeout (%.1f seconds) reached, forcing reset",
                                 time_in_stop_state);
                        large_obstacle_failure_count_ = 0;  // 重置计数器
                        large_obstacle_replan_count_ = 0;  // 重置重规划计数器
                        updateNavigationState(NavigationState::NORMAL);
                    }
                }
                return true;  // 返回true表示停止执行后续导航逻辑
            }

            case NavigationState::NAVIGATION_FAILED:
            {
                // 导航失败状态，完全停止
                cmd_vel = geometry_msgs::Twist();
                ROS_ERROR("Navigation failed, stopped");
                return true;
            }

            default:
                break;
        }

        return false;  // 继续执行正常的导航逻辑
    }

} // namespace pure_pursuit_local_planner

