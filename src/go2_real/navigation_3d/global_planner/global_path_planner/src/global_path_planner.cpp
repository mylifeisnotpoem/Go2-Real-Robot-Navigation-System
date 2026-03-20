#include "global_path_planner/global_path_planner.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <queue>
#include <limits>

namespace global_path_planner
{

    GlobalPathPlanner::GlobalPathPlanner()
        : has_current_pose_(false), has_goal_(false), has_free_space_cloud_(false)
    {
        // 获取参数
        ros::NodeHandle nh("~");
        nh.param<std::string>("topology_map_file", map_file_path_,
                              "/home/server/WS_ROS1/ws_pcl_map/src/topology_rviz_plugin/map/topology_map.txt");
        nh.param<std::string>("global_frame", global_frame_, "map");
        nh.param<std::string>("robot_frame", robot_frame_, "base_link");
        nh.param<std::string>("odom_topic", odom_topic_, "/Odometry");
        nh.param<double>("connection_distance_threshold", connection_distance_threshold_, 5.0);
        nh.param<int>("interpolation_points", interpolation_points_, 20);

        // 可行性检查相关参数
        nh.param<double>("free_space_radius", free_space_radius_, 1.0);
        nh.param<double>("edge_proximity_threshold", edge_proximity_threshold_, 0.8);
        nh.param<double>("line_of_sight_resolution", line_of_sight_resolution_, 0.2);
        nh.param<bool>("enable_feasibility_check", enable_feasibility_check_, true);
        nh.param<bool>("enable_direct_connection", enable_direct_connection_, true);
        nh.param<double>("direct_connection_threshold", direct_connection_threshold_, 3.0);
        
        // 边合法性检查相关参数
        nh.param<double>("robot_height", robot_height_, 0.35);
        nh.param<int>("edge_sample_count", edge_sample_count_, 100);
        nh.param<double>("safety_sphere_radius", safety_sphere_radius_, 0.3);
        nh.param<double>("valid_point_threshold", valid_point_threshold_, 0.9);
        nh.param<std::string>("free_space_topic", free_space_topic_, "/free_space_cloud");
        nh.param<bool>("enable_temp_connection_feasibility_check", enable_temp_connection_feasibility_check_, false);
        
        // 初始化PCL相关对象
        free_space_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);

        ROS_INFO("全局路径规划器构造完成");
        ROS_INFO("节点名: %s, 私有命名空间: %s",
                 ros::this_node::getName().c_str(), nh.getNamespace().c_str());
        ROS_INFO("地图文件: %s", map_file_path_.c_str());
        ROS_INFO("全局坐标系: %s", global_frame_.c_str());
        ROS_INFO("机器人坐标系: %s", robot_frame_.c_str());
        ROS_INFO("直接连接: %s, 阈值: %.2f m",
                 enable_direct_connection_ ? "启用" : "禁用",
                 direct_connection_threshold_);
        ROS_INFO("临时节点连接距离阈值: %.2f m, 插值点数: %d",
                 connection_distance_threshold_, interpolation_points_);
        ROS_INFO("机器狗高度: %.2f m", robot_height_);
        ROS_INFO("安全球半径: %.2f m", safety_sphere_radius_);
        ROS_INFO("有效点阈值: %.1f%%", valid_point_threshold_ * 100);
        ROS_INFO("临时连接可行域检查: %s", enable_temp_connection_feasibility_check_ ? "启用" : "禁用");
    }

    GlobalPathPlanner::~GlobalPathPlanner()
    {
        ROS_INFO("全局路径规划器已清理并关闭");
    }

    bool GlobalPathPlanner::initialize()
    {
        ROS_INFO("初始化全局路径规划器...");

        // 创建发布器
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1, true);

        // 注释掉可视化发布器以避免RViz崩溃
        // visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("topology_visualization", 1, true);

        // 创建订阅器（输入源）：
        // 1) odom_sub_: 实时更新机器人当前位姿（起点）
        // 2) goal_sub_: 来自 RViz clicked_point 的目标点
        // 3) target_points_sub_: 来自上层任务系统/UI 的目标位姿
        // 4) free_space_cloud_sub_: 用于边合法性检查的可行域点云
        // 说明：goal_sub_ 与 target_points_sub_ 都会触发“收到目标后尝试规划并发布 /global_path”。
        odom_sub_ = nh_.subscribe(odom_topic_, 1, &GlobalPathPlanner::odomCallback, this);
        goal_sub_ = nh_.subscribe("clicked_point", 1, &GlobalPathPlanner::goalCallback, this);
        target_points_sub_ = nh_.subscribe("/target_points", 1, &GlobalPathPlanner::targetPointsCallback, this);
        free_space_cloud_sub_ = nh_.subscribe(free_space_topic_, 1, &GlobalPathPlanner::freeSpaceCloudCallback, this);

        // 加载拓扑地图
        if (!loadTopologyMap())
        {
            ROS_ERROR("加载拓扑地图失败");
            return false;
        }

        ROS_INFO("全局路径规划器初始化完成，加载了 %lu 个节点和 %lu 条边",
                 nodes_.size(), edges_.size());
        return true;
    }

    void GlobalPathPlanner::spin()
    {
        ros::spin();
    }

    bool GlobalPathPlanner::loadTopologyMap()
    {
        ROS_INFO("开始加载拓扑地图文件: %s", map_file_path_.c_str());

        std::ifstream file(map_file_path_.c_str());
        if (!file.is_open())
        {
            ROS_ERROR("无法打开拓扑地图文件: %s", map_file_path_.c_str());
            return false;
        }

        std::string line;
        bool reading_edges = false;

        while (std::getline(file, line))
        {
            // 移除前后空格
            line.erase(0, line.find_first_not_of(" \t\r\n"));
            line.erase(line.find_last_not_of(" \t\r\n") + 1);

            // 跳过空行和注释
            if (line.empty() || line[0] == '#')
            {
                if (line.find("Topology Edges") != std::string::npos)
                {
                    reading_edges = true;
                    ROS_INFO("开始读取边数据");
                }
                continue;
            }

            std::istringstream iss(line);

            if (!reading_edges)
            {
                // 读取节点: ID X Y Z 标签
                TopologyNode node;
                if (iss >> node.id >> node.x >> node.y >> node.z)
                {
                    // 读取剩余部分作为标签
                    std::string remaining;
                    std::getline(iss, remaining);
                    if (!remaining.empty() && remaining[0] == ' ')
                    {
                        remaining = remaining.substr(1);
                    }
                    node.label = remaining.empty() ? "node_" + std::to_string(node.id) : remaining;

                    nodes_.push_back(node);
                    node_map_[node.id] = node;

                    ROS_DEBUG("读取节点: ID=%d, 位置=(%.2f,%.2f,%.2f)",
                              node.id, node.x, node.y, node.z);
                }
            }
            else
            {
                // 读取边: ID 起始ID 结束ID 权重 类型 双向
                TopologyEdge edge;
                std::string type;
                int bidirectional_int;

                if (iss >> edge.id >> edge.from_id >> edge.to_id >> edge.weight >> type >> bidirectional_int)
                {
                    edge.bidirectional = (bidirectional_int == 1);
                    edges_.push_back(edge);

                    ROS_DEBUG("读取边: ID=%d, %d->%d, 权重=%.2f, 双向=%s",
                              edge.id, edge.from_id, edge.to_id, edge.weight,
                              edge.bidirectional ? "是" : "否");
                }
            }
        }

        file.close();

        ROS_INFO("拓扑地图加载完成: %lu 个节点, %lu 条边", nodes_.size(), edges_.size());
        return true;
    }

    void GlobalPathPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 回调职责：
        // 将里程计位姿统一变换到 global_frame_，并更新 current_position_。
        // 后续任意目标回调（clicked_point/target_points）都会使用这里缓存的起点进行规划。

        // Step 1) 从 Odometry 构造 PoseStamped，便于 tf 变换。
        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;

        try
        {
            // Step 2) 变换到全局坐标系（通常是 map）。
            tf_listener_.transformPose(global_frame_, pose_in, pose_out);

            // Step 3) 更新内部状态：当前位置 + 起点有效标志。
            current_position_.x = pose_out.pose.position.x;
            current_position_.y = pose_out.pose.position.y;
            current_position_.z = pose_out.pose.position.z;
            has_current_pose_ = true;

            ROS_DEBUG("更新当前位置: (%.2f, %.2f, %.2f)",
                      current_position_.x, current_position_.y, current_position_.z);
        }
        catch (tf::TransformException &ex)
        {
            // TF 异常时不更新 current_position_，保留上一次有效值。
            ROS_WARN("坐标变换失败: %s", ex.what());
        }
    }

    void GlobalPathPlanner::goalCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        // 回调职责：
        // 处理 RViz 点击目标（clicked_point），转换到 global_frame_，
        // 更新 goal_position_ 后，若已有起点则立即触发一次全局规划。
        ROS_INFO("收到点击目标点");

        // Step 1) 将点击点坐标转换到全局坐标系。
        geometry_msgs::PointStamped goal_in_global;
        try
        {
            tf_listener_.transformPoint(global_frame_, *msg, goal_in_global);

            // Step 2) 更新内部目标状态。
            goal_position_.x = goal_in_global.point.x;
            goal_position_.y = goal_in_global.point.y;
            goal_position_.z = goal_in_global.point.z;
            has_goal_ = true;

            ROS_INFO("目标点位置: (%.2f, %.2f, %.2f)",
                     goal_position_.x, goal_position_.y, goal_position_.z);

            // Step 3) 若起点有效则立即规划并发布；
            // 否则仅缓存目标，等待里程计回调更新起点后再规划。
            if (has_current_pose_)
            {
                nav_msgs::Path global_path = planGlobalPath();
                if (!global_path.poses.empty())
                {
                    global_path_pub_.publish(global_path);
                    ROS_INFO("发布全局路径，包含 %lu 个路径点", global_path.poses.size());
                }
                else
                {
                    ROS_WARN("无法规划路径");
                }
            }
            else
            {
                ROS_WARN("尚未接收到当前位置信息");
            }
        }
        catch (tf::TransformException &ex)
        {
            // 目标变换失败时，不覆盖已有目标状态。
            ROS_ERROR("目标点坐标变换失败: %s", ex.what());
        }
    }

    void GlobalPathPlanner::targetPointsCallback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        // 回调职责：
        // 处理上层模块发送的目标位姿（/target_points）。
        // 与 clicked_point 不同，这里消息通常已在期望坐标系中，直接使用 position。
        ROS_INFO("收到目标点位姿");

        // Step 1) 缓存目标位置并标记目标有效。
        goal_position_.x = msg->position.x;
        goal_position_.y = msg->position.y;
        goal_position_.z = msg->position.z;
        has_goal_ = true;

        ROS_INFO("目标点位置: (%.2f, %.2f, %.2f)",
                 goal_position_.x, goal_position_.y, goal_position_.z);
        ROS_INFO("目标点朝向: (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                 msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

        // Step 2) 若起点有效则立即规划并发布。
        if (has_current_pose_)
        {
            nav_msgs::Path global_path = planGlobalPath();
            if (!global_path.poses.empty())
            {
                global_path_pub_.publish(global_path);
                ROS_INFO("发布全局路径，包含 %lu 个路径点", global_path.poses.size());
            }
            else
            {
                ROS_WARN("无法规划路径");
            }
        }
        else
        {
            ROS_WARN("尚未接收到当前位置信息");
        }
    }

    nav_msgs::Path GlobalPathPlanner::planGlobalPath()
    {
        // 入口说明：
        // 使用“原拓扑图 + 起终点临时节点 + 临时连接边”构建临时图，
        // 在临时图上执行 Dijkstra，最后将拓扑节点序列插值为 nav_msgs/Path。
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = global_frame_;

        if (!has_current_pose_ || !has_goal_)
        {
            ROS_WARN("缺少起点或终点信息");
            return path;
        }

        ROS_INFO("开始规划从 (%.2f, %.2f, %.2f) 到 (%.2f, %.2f, %.2f) 的路径",
                 current_position_.x, current_position_.y, current_position_.z,
                 goal_position_.x, goal_position_.y, goal_position_.z);

        try
        {
            // 创建临时拓扑图，包含起点和终点作为临时节点
            std::vector<TopologyNode> temp_nodes;
            std::vector<TopologyEdge> temp_edges;

            // 预分配内存以避免频繁重新分配
            temp_nodes.reserve(nodes_.size() + 2);
            temp_edges.reserve(edges_.size() + 10); // 额外空间用于临时连接

            // 复制原始节点和边
            temp_nodes = nodes_;
            temp_edges = edges_;

            // 添加起点作为临时节点
            int start_node_id = temp_nodes.size();
            TopologyNode start_node;
            start_node.id = start_node_id;
            start_node.x = current_position_.x;
            start_node.y = current_position_.y;
            start_node.z = current_position_.z;
            start_node.label = "start_temp";
            temp_nodes.push_back(start_node);

            // 添加终点作为临时节点
            int goal_node_id = temp_nodes.size();
            TopologyNode goal_node;
            goal_node.id = goal_node_id;
            goal_node.x = goal_position_.x;
            goal_node.y = goal_position_.y;
            goal_node.z = goal_position_.z;
            goal_node.label = "goal_temp";
            temp_nodes.push_back(goal_node);

            // 为起点和终点节点创建连接到最近节点的边，增加连接数量以避免折返路线
            addTemporaryConnections(temp_nodes, temp_edges, start_node_id, current_position_);
            addTemporaryConnections(temp_nodes, temp_edges, goal_node_id, goal_position_);

            // 检查起点和终点距离，如果很近且可行，直接连接
            double direct_distance = calculateDistance(current_position_, goal_position_);
            ROS_INFO("起点到终点直线距离: %.2f 米", direct_distance);
            
            // 创建临时节点用于边合法性检查
            TopologyNode start_temp_node, goal_temp_node;
            start_temp_node.id = start_node_id;
            start_temp_node.x = current_position_.x;
            start_temp_node.y = current_position_.y;
            start_temp_node.z = current_position_.z;
            start_temp_node.label = "start_temp";
            
            goal_temp_node.id = goal_node_id;
            goal_temp_node.x = goal_position_.x;
            goal_temp_node.y = goal_position_.y;
            goal_temp_node.z = goal_position_.z;
            goal_temp_node.label = "goal_temp";
            
            // 检查直接连接的可行性
            bool direct_connection_feasible = true;
            if (enable_temp_connection_feasibility_check_)
            {
                direct_connection_feasible = isEdgeLegal(start_temp_node, goal_temp_node);
            }

            ROS_INFO("直连判定: enable=%s, distance=%.2f m, threshold=%.2f m, feasible=%s",
                     enable_direct_connection_ ? "true" : "false",
                     direct_distance,
                     direct_connection_threshold_,
                     direct_connection_feasible ? "true" : "false");
            
            if (enable_direct_connection_ && 
                direct_distance <= direct_connection_threshold_ && 
                direct_connection_feasible)
            {
                // 创建起点到终点的直接连接
                TopologyEdge direct_edge;
                direct_edge.id = temp_edges.size();
                direct_edge.from_id = start_node_id;
                direct_edge.to_id = goal_node_id;
                direct_edge.weight = direct_distance * 0.9; // 给直接连接一个小的奖励
                direct_edge.bidirectional = true;
                temp_edges.push_back(direct_edge);
                
                ROS_INFO("添加起点到终点的直接连接，距离: %.2f 米", direct_distance);
            }
            else
            {
                if (!enable_direct_connection_)
                {
                    ROS_INFO("未添加直连边: 直接连接功能已禁用");
                }
                else if (direct_distance > direct_connection_threshold_)
                {
                    ROS_INFO("未添加直连边: 起点到终点距离 %.2f 米超过阈值 %.2f 米",
                             direct_distance, direct_connection_threshold_);
                }
                else
                {
                    if (enable_temp_connection_feasibility_check_)
                    {
                        ROS_INFO("未添加直连边: 起点到终点直接连接不可行（合法性检查失败）");
                    }
                    else
                    {
                        ROS_WARN("未添加直连边: 判定条件未满足，但日志原因落入了未预期分支");
                    }
                }
            }

            ROS_DEBUG("临时拓扑图创建完成：%lu 个节点，%lu 条边", temp_nodes.size(), temp_edges.size());

            // 在临时拓扑图中进行Dijkstra搜索
            std::vector<int> topology_path = dijkstraSearchInTempGraph(temp_nodes, temp_edges, start_node_id, goal_node_id);

            if (topology_path.empty())
            {
                ROS_WARN("未找到可行路径");
                return path;
            }

            // 构建完整路径
            path = buildCompletePathFromTempGraph(temp_nodes, topology_path);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("路径规划过程中发生异常: %s", e.what());
            return nav_msgs::Path(); // 返回空路径
        }
        catch (...)
        {
            ROS_ERROR("路径规划过程中发生未知异常");
            return nav_msgs::Path(); // 返回空路径
        }

        return path;
    }

    void GlobalPathPlanner::addTemporaryConnections(std::vector<TopologyNode> &temp_nodes,
                                                    std::vector<TopologyEdge> &temp_edges,
                                                    int temp_node_id,
                                                    const geometry_msgs::Point &position)
    {
        // 创建临时节点用于边合法性检查
        TopologyNode temp_node;
        temp_node.id = temp_node_id;
        temp_node.x = position.x;
        temp_node.y = position.y;
        temp_node.z = position.z;
        temp_node.label = "temp";

        ROS_INFO("开始为临时节点 %d 查找连接", temp_node_id);

        // 第一步：找到距离最近的第一个合法节点
        int nearest_legal_node_id = -1;
        double min_distance = std::numeric_limits<double>::max();

        // 按距离排序所有节点
        std::vector<std::pair<double, int>> sorted_nodes;
        for (size_t i = 0; i < nodes_.size(); ++i)
        {
            geometry_msgs::Point node_pos;
            node_pos.x = nodes_[i].x;
            node_pos.y = nodes_[i].y;
            node_pos.z = nodes_[i].z;
            
            double dist = calculateDistance(position, node_pos);
            sorted_nodes.push_back(std::make_pair(dist, nodes_[i].id));
        }
        
        // 按距离排序
        std::sort(sorted_nodes.begin(), sorted_nodes.end());

        // 找到第一个合法节点
        for (const auto& node_pair : sorted_nodes)
        {
            int node_id = node_pair.second;
            
            // 找到对应的节点
            auto it = std::find_if(nodes_.begin(), nodes_.end(), 
                                   [node_id](const TopologyNode& n) { return n.id == node_id; });
            
            if (it != nodes_.end())
            {
                // 根据参数控制是否进行边合法性检查
                bool is_legal = true;
                if (enable_temp_connection_feasibility_check_)
                {
                    is_legal = isEdgeLegal(temp_node, *it);
                }
                
                if (is_legal)
                {
                    nearest_legal_node_id = node_id;
                    min_distance = node_pair.first;
                    ROS_INFO("找到第一个合法节点: %d，距离: %.2f", node_id, min_distance);
                    break;
                }
                else
                {
                    ROS_DEBUG("节点 %d 连接非法，继续查找", node_id);
                }
            }
        }

        if (nearest_legal_node_id == -1)
        {
            ROS_ERROR("临时节点 %d 无法找到任何合法连接节点", temp_node_id);
            return;
        }

        // 第二步：收集第一层和第二层候选节点
        std::set<int> candidate_nodes;
        candidate_nodes.insert(nearest_legal_node_id); // 第一层：最近的合法节点

        // 查找第二层：与第一个合法节点相邻的所有节点
        for (const auto& edge : edges_)
        {
            if (edge.from_id == nearest_legal_node_id)
            {
                candidate_nodes.insert(edge.to_id);
                ROS_DEBUG("添加第二层候选节点: %d (通过边 %d->%d)", edge.to_id, edge.from_id, edge.to_id);
            }
            else if (edge.bidirectional && edge.to_id == nearest_legal_node_id)
            {
                candidate_nodes.insert(edge.from_id);
                ROS_DEBUG("添加第二层候选节点: %d (通过双向边 %d<->%d)", edge.from_id, edge.from_id, edge.to_id);
            }
        }

        ROS_INFO("第一个合法节点: %d, 第二层候选节点总数: %lu", nearest_legal_node_id, candidate_nodes.size() - 1);

        // 第三步：对所有候选节点进行合法性检查并创建连接
        int actual_connections = 0;
        for (int candidate_id : candidate_nodes)
        {
            // 找到候选节点
            auto it = std::find_if(nodes_.begin(), nodes_.end(), 
                                   [candidate_id](const TopologyNode& n) { return n.id == candidate_id; });
            
            if (it != nodes_.end())
            {
                // 根据参数控制是否进行边合法性检查
                bool is_legal = true;
                if (enable_temp_connection_feasibility_check_)
                {
                    is_legal = isEdgeLegal(temp_node, *it);
                }
                
                if (is_legal)
                {
                    // 计算连接代价
                    geometry_msgs::Point node_pos;
                    node_pos.x = it->x;
                    node_pos.y = it->y;
                    node_pos.z = it->z;
                    
                    double cost = calculateConnectionCost(position, node_pos);

                    // 创建双向连接
                    TopologyEdge edge;
                    edge.id = temp_edges.size();
                    edge.from_id = temp_node_id;
                    edge.to_id = candidate_id;
                    edge.weight = cost;
                    edge.bidirectional = true;
                    temp_edges.push_back(edge);

                    actual_connections++;
                    ROS_INFO("创建合法连接: 临时节点%d -> 节点%d, 代价: %.2f", temp_node_id, candidate_id, cost);
                }
                else
                {
                    ROS_DEBUG("候选节点 %d 连接非法，跳过", candidate_id);
                }
            }
        }

        if (actual_connections == 0)
        {
            ROS_ERROR("临时节点 %d 无法创建任何合法连接，尝试强制连接到最近合法节点", temp_node_id);
            
            // 强制连接到第一个合法节点作为备选
            auto it = std::find_if(nodes_.begin(), nodes_.end(), 
                                   [nearest_legal_node_id](const TopologyNode& n) { return n.id == nearest_legal_node_id; });
            
            if (it != nodes_.end())
            {
                geometry_msgs::Point node_pos;
                node_pos.x = it->x;
                node_pos.y = it->y;
                node_pos.z = it->z;
                
                double cost = calculateDistance(position, node_pos);

                TopologyEdge edge;
                edge.id = temp_edges.size();
                edge.from_id = temp_node_id;
                edge.to_id = nearest_legal_node_id;
                edge.weight = cost;
                edge.bidirectional = true;
                temp_edges.push_back(edge);

                actual_connections = 1;
                ROS_WARN("强制创建备选连接: 临时节点%d -> 节点%d", temp_node_id, nearest_legal_node_id);
            }
        }

        ROS_INFO("临时节点 %d 成功连接到 %d 个节点", temp_node_id, actual_connections);
    }

    std::vector<int> GlobalPathPlanner::findTopologyPath(int start_node_id, int goal_node_id)
    {
        if (start_node_id == goal_node_id)
        {
            return std::vector<int>{start_node_id};
        }

        return dijkstraSearch(start_node_id, goal_node_id);
    }

    std::vector<int> GlobalPathPlanner::dijkstraSearch(int start_id, int goal_id)
    {
        // distances: 起点到每个节点的当前最短已知距离 g(n)
        // previous : 最短路径树中的前驱节点，用于最后回溯出完整路径
        std::map<int, double> distances;
        std::map<int, int> previous;

        // 最小堆（按距离从小到大弹出）
        // 元素格式: (当前距离, 节点ID)
        // 使用 greater 让 priority_queue 表现为小顶堆。
        std::priority_queue<std::pair<double, int>,
                            std::vector<std::pair<double, int>>,
                            std::greater<std::pair<double, int>>>
            pq;

        // Step 1) 初始化：
        // 所有节点距离置为 +inf，起点距离为 0 并入堆。
        for (const auto &node : nodes_)
        {
            distances[node.id] = std::numeric_limits<double>::infinity();
        }
        distances[start_id] = 0.0;
        pq.push(std::make_pair(0.0, start_id));

        // Step 2) 主循环：
        // 每次取出当前“距离最小”的未确定节点，尝试用它去松弛相邻边。
        while (!pq.empty())
        {
            int current_id = pq.top().second;
            double current_dist = pq.top().first;
            pq.pop();

            // 提前终止：Dijkstra 中，当终点第一次以最小距离出堆时，
            // 已经得到其最短路，可以直接结束。
            if (current_id == goal_id)
            {
                break;
            }

            // 由于没有 decrease-key，堆中可能存在“旧条目”：
            // 如果当前弹出的距离大于已记录最短距离，说明它已过期，跳过即可。
            if (current_dist > distances[current_id])
            {
                continue;
            }

            // Step 2.1) 枚举与 current_id 相连的边，识别邻居节点。
            // 当前实现通过遍历 edges_ 找邻接关系（非邻接表实现）。
            for (const auto &edge : edges_)
            {
                int neighbor_id = -1;
                if (edge.from_id == current_id)
                {
                    neighbor_id = edge.to_id;
                }
                else if (edge.bidirectional && edge.to_id == current_id)
                {
                    neighbor_id = edge.from_id;
                }

                if (neighbor_id != -1 && node_map_.find(neighbor_id) != node_map_.end())
                {
                    // Step 2.2) 松弛操作（Relaxation）：
                    // 若“经 current 到 neighbor”的路径更短，则更新最短距离与前驱。
                    double edge_weight = edge.weight;
                    double new_distance = distances[current_id] + edge_weight;

                    if (new_distance < distances[neighbor_id])
                    {
                        distances[neighbor_id] = new_distance;
                        previous[neighbor_id] = current_id;
                        pq.push(std::make_pair(new_distance, neighbor_id));
                    }
                }
            }
        }

        // Step 3) 路径重构：
        // 若终点距离仍为 +inf，表示不可达，返回空路径。
        std::vector<int> path;
        if (distances[goal_id] == std::numeric_limits<double>::infinity())
        {
            return path;
        }

        // 从终点沿 previous 反向回溯到起点，再 reverse 得到正向路径。
        int current = goal_id;
        while (current != start_id)
        {
            path.push_back(current);
            current = previous[current];
        }
        path.push_back(start_id);

        std::reverse(path.begin(), path.end());

        return path;
    }

    nav_msgs::Path GlobalPathPlanner::buildCompletePath(const geometry_msgs::Point &start,
                                                        const geometry_msgs::Point &goal,
                                                        const std::vector<int> &topology_path)
    {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = global_frame_;

        std::vector<geometry_msgs::Point> waypoints;

        // 1. 添加起点
        waypoints.push_back(start);

        // 2. 添加拓扑路径点
        for (int node_id : topology_path)
        {
            if (node_map_.find(node_id) != node_map_.end())
            {
                const TopologyNode &node = node_map_[node_id];
                geometry_msgs::Point point;
                point.x = node.x;
                point.y = node.y;
                point.z = node.z;
                waypoints.push_back(point);
            }
        }

        // 3. 添加终点
        waypoints.push_back(goal);

        // 4. 插值生成完整路径
        for (size_t i = 0; i < waypoints.size() - 1; ++i)
        {
            std::vector<geometry_msgs::Point> segment =
                interpolatePath(waypoints[i], waypoints[i + 1], interpolation_points_);

            for (const auto &point : segment)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position = point;

                // 计算朝向
                if (i < waypoints.size() - 2 || &point != &segment.back())
                {
                    geometry_msgs::Point next_point;
                    if (&point == &segment.back())
                    {
                        next_point = waypoints[i + 2];
                    }
                    else
                    {
                        auto it = std::find_if(segment.begin(), segment.end(),
                                               [&point](const geometry_msgs::Point &p)
                                               {
                                                   return p.x == point.x && p.y == point.y && p.z == point.z;
                                               });
                        if (it != segment.end() && std::next(it) != segment.end())
                        {
                            next_point = *std::next(it);
                        }
                        else
                        {
                            next_point = waypoints[i + 1];
                        }
                    }
                    pose.pose.orientation = calculateOrientation(point, next_point);
                }
                else
                {
                    pose.pose.orientation.w = 1.0;
                }

                path.poses.push_back(pose);
            }
        }

        return path;
    }

    int GlobalPathPlanner::findNearestNode(const geometry_msgs::Point &point)
    {
        int nearest_id = -1;
        double min_distance = std::numeric_limits<double>::max();

        for (const auto &node : nodes_)
        {
            geometry_msgs::Point node_pos;
            node_pos.x = node.x;
            node_pos.y = node.y;
            node_pos.z = node.z;

            double dist = calculateDistance(point, node_pos);
            if (dist < min_distance)
            {
                min_distance = dist;
                nearest_id = node.id;
            }
        }

        return nearest_id;
    }

    TopologyEdge GlobalPathPlanner::findNearestEdge(const geometry_msgs::Point &point)
    {
        TopologyEdge nearest_edge;
        double min_distance = std::numeric_limits<double>::max();
        bool found = false;

        for (const auto &edge : edges_)
        {
            if (node_map_.find(edge.from_id) != node_map_.end() &&
                node_map_.find(edge.to_id) != node_map_.end())
            {
                const TopologyNode &from_node = node_map_[edge.from_id];
                const TopologyNode &to_node = node_map_[edge.to_id];

                geometry_msgs::Point from_point;
                from_point.x = from_node.x;
                from_point.y = from_node.y;
                from_point.z = from_node.z;

                geometry_msgs::Point to_point;
                to_point.x = to_node.x;
                to_point.y = to_node.y;
                to_point.z = to_node.z;

                double dist = pointToSegmentDistance(point, from_point, to_point);
                if (dist < min_distance)
                {
                    min_distance = dist;
                    nearest_edge = edge;
                    found = true;
                }
            }
        }

        if (!found)
        {
            ROS_WARN("未找到最近的边");
        }

        return nearest_edge;
    }

    geometry_msgs::Point GlobalPathPlanner::findNearestPointOnEdge(const geometry_msgs::Point &point,
                                                                   const TopologyEdge &edge)
    {
        geometry_msgs::Point result = point;

        if (node_map_.find(edge.from_id) != node_map_.end() &&
            node_map_.find(edge.to_id) != node_map_.end())
        {
            const TopologyNode &from_node = node_map_[edge.from_id];
            const TopologyNode &to_node = node_map_[edge.to_id];

            geometry_msgs::Point from_point;
            from_point.x = from_node.x;
            from_point.y = from_node.y;
            from_point.z = from_node.z;

            geometry_msgs::Point to_point;
            to_point.x = to_node.x;
            to_point.y = to_node.y;
            to_point.z = to_node.z;

            // 计算点在线段上的投影
            double dx = to_point.x - from_point.x;
            double dy = to_point.y - from_point.y;
            double dz = to_point.z - from_point.z;

            double segment_length_squared = dx * dx + dy * dy + dz * dz;

            if (segment_length_squared > 1e-6)
            {
                double px = point.x - from_point.x;
                double py = point.y - from_point.y;
                double pz = point.z - from_point.z;

                double t = (px * dx + py * dy + pz * dz) / segment_length_squared;
                t = std::max(0.0, std::min(1.0, t)); // 限制在线段范围内

                result.x = from_point.x + t * dx;
                result.y = from_point.y + t * dy;
                result.z = from_point.z + t * dz;
            }
            else
            {
                result = from_point;
            }
        }

        return result;
    }

    double GlobalPathPlanner::pointToSegmentDistance(const geometry_msgs::Point &point,
                                                     const geometry_msgs::Point &seg_start,
                                                     const geometry_msgs::Point &seg_end)
    {
        // 计算线段向量
        double dx = seg_end.x - seg_start.x;
        double dy = seg_end.y - seg_start.y;
        double dz = seg_end.z - seg_start.z;

        double segment_length_squared = dx * dx + dy * dy + dz * dz;

        if (segment_length_squared < 1e-6)
        {
            // 线段退化为点
            return calculateDistance(point, seg_start);
        }

        // 计算点到起点的向量
        double px = point.x - seg_start.x;
        double py = point.y - seg_start.y;
        double pz = point.z - seg_start.z;

        // 计算投影参数 t
        double t = (px * dx + py * dy + pz * dz) / segment_length_squared;
        t = std::max(0.0, std::min(1.0, t)); // 限制在 [0, 1] 范围内

        // 计算投影点
        geometry_msgs::Point projection;
        projection.x = seg_start.x + t * dx;
        projection.y = seg_start.y + t * dy;
        projection.z = seg_start.z + t * dz;

        return calculateDistance(point, projection);
    }

    std::vector<geometry_msgs::Point> GlobalPathPlanner::interpolatePath(
        const geometry_msgs::Point &start,
        const geometry_msgs::Point &end,
        int num_points)
    {
        std::vector<geometry_msgs::Point> points;

        if (num_points <= 0)
        {
            return points;
        }

        for (int i = 0; i <= num_points; ++i)
        {
            double t = static_cast<double>(i) / num_points;

            geometry_msgs::Point point;
            point.x = start.x + t * (end.x - start.x);
            point.y = start.y + t * (end.y - start.y);
            point.z = start.z + t * (end.z - start.z);

            points.push_back(point);
        }

        return points;
    }

    double GlobalPathPlanner::calculateDistance(const geometry_msgs::Point &p1,
                                                const geometry_msgs::Point &p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    geometry_msgs::Quaternion GlobalPathPlanner::calculateOrientation(
        const geometry_msgs::Point &from,
        const geometry_msgs::Point &to)
    {
        geometry_msgs::Quaternion quat;

        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double yaw = std::atan2(dy, dx);

        quat.x = 0.0;
        quat.y = 0.0;
        quat.z = std::sin(yaw / 2.0);
        quat.w = std::cos(yaw / 2.0);

        return quat;
    }

    std::vector<int> GlobalPathPlanner::dijkstraSearchInTempGraph(const std::vector<TopologyNode> &temp_nodes,
                                                                  const std::vector<TopologyEdge> &temp_edges,
                                                                  int start_id, int goal_id)
    {
        // distances: 临时图中 start_id 到各节点的当前最短已知距离
        // previous : 最短路径树前驱，用于最终回溯路径
        std::map<int, double> distances;
        std::map<int, int> previous;

        // 小顶堆，按 (距离, 节点ID) 升序弹出
        // 每次扩展当前最“便宜”的候选节点。
        std::priority_queue<std::pair<double, int>,
                            std::vector<std::pair<double, int>>,
                            std::greater<std::pair<double, int>>>
            pq;

        // Step 1) 初始化：
        // 临时图所有节点距离设为 +inf，起点设为 0 并入堆。
        for (const auto &node : temp_nodes)
        {
            distances[node.id] = std::numeric_limits<double>::infinity();
        }
        distances[start_id] = 0.0;
        pq.push(std::make_pair(0.0, start_id));

        // Step 2) Dijkstra 主循环：
        // 弹出当前最小距离节点，尝试松弛所有相邻边。
        while (!pq.empty())
        {
            int current_id = pq.top().second;
            double current_dist = pq.top().first;
            pq.pop();

            // 早停：终点第一次以最小距离出堆时，其最短路已经确定。
            if (current_id == goal_id)
            {
                break;
            }

            // 跳过过期堆条目（无 decrease-key 导致的重复入堆）。
            if (current_dist > distances[current_id])
            {
                continue;
            }

            // Step 2.1) 在临时边集中找与 current_id 相连的邻居。
            // 当前实现为边数组线性扫描，便于复用临时图结构。
            for (const auto &edge : temp_edges)
            {
                int neighbor_id = -1;
                if (edge.from_id == current_id)
                {
                    neighbor_id = edge.to_id;
                }
                else if (edge.bidirectional && edge.to_id == current_id)
                {
                    neighbor_id = edge.from_id;
                }

                if (neighbor_id != -1)
                {
                    // Step 2.2) 松弛：
                    // 如果经由 current_id 到 neighbor_id 的路径更短，则更新并重新入堆。
                    double edge_weight = edge.weight;
                    double new_distance = distances[current_id] + edge_weight;

                    if (new_distance < distances[neighbor_id])
                    {
                        distances[neighbor_id] = new_distance;
                        previous[neighbor_id] = current_id;
                        pq.push(std::make_pair(new_distance, neighbor_id));
                    }
                }
            }
        }

        // Step 3) 路径重构：
        // 若终点距离仍是 +inf，说明不可达，返回空路径。
        std::vector<int> path;
        if (distances[goal_id] == std::numeric_limits<double>::infinity())
        {
            ROS_WARN("在临时拓扑图中未找到从 %d 到 %d 的路径", start_id, goal_id);
            return path;
        }

        // 从 goal 反向沿 previous 回溯到 start，再 reverse 成正向路径。
        // 这里额外检查 previous 是否存在，避免异常图结构导致死循环或越界。
        int current = goal_id;
        while (current != start_id)
        {
            path.push_back(current);
            if (previous.find(current) == previous.end())
            {
                ROS_ERROR("路径重构失败：节点 %d 没有前驱节点", current);
                return std::vector<int>();
            }
            current = previous[current];
        }
        path.push_back(start_id);

        std::reverse(path.begin(), path.end());

        ROS_INFO("在临时拓扑图中找到路径，包含 %lu 个节点", path.size());
        return path;
    }

    nav_msgs::Path GlobalPathPlanner::buildCompletePathFromTempGraph(const std::vector<TopologyNode> &temp_nodes,
                                                                     const std::vector<int> &topology_path)
    {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = global_frame_;

        if (topology_path.empty())
        {
            ROS_WARN("拓扑路径为空，无法构建完整路径");
            return path;
        }

        // 创建节点ID到节点的映射
        std::map<int, TopologyNode> temp_node_map;
        for (const auto &node : temp_nodes)
        {
            temp_node_map[node.id] = node;
        }

        std::vector<geometry_msgs::Point> waypoints;

        // 将拓扑路径中的所有节点转换为路径点
        for (int node_id : topology_path)
        {
            if (temp_node_map.find(node_id) != temp_node_map.end())
            {
                const TopologyNode &node = temp_node_map[node_id];
                geometry_msgs::Point point;
                point.x = node.x;
                point.y = node.y;
                point.z = node.z;
                waypoints.push_back(point);
            }
            else
            {
                ROS_WARN("节点 %d 在临时节点映射中不存在", node_id);
            }
        }

        if (waypoints.size() < 2)
        {
            ROS_WARN("路径点不足，无法构建完整路径");
            return path;
        }

        // 插值生成完整路径
        for (size_t i = 0; i < waypoints.size() - 1; ++i)
        {
            std::vector<geometry_msgs::Point> segment =
                interpolatePath(waypoints[i], waypoints[i + 1], interpolation_points_);

            for (size_t j = 0; j < segment.size(); ++j)
            {
                // 避免重复添加相同的点（除了第一个段）
                if (i == 0 || j != 0)
                {
                    geometry_msgs::PoseStamped pose;
                    pose.header = path.header;
                    pose.pose.position = segment[j];

                    // 计算朝向
                    if (j < segment.size() - 1)
                    {
                        pose.pose.orientation = calculateOrientation(segment[j], segment[j + 1]);
                    }
                    else if (i < waypoints.size() - 2)
                    {
                        pose.pose.orientation = calculateOrientation(segment[j], waypoints[i + 2]);
                    }
                    else
                    {
                        // 最后一个点，保持前一个点的朝向
                        if (!path.poses.empty())
                        {
                            pose.pose.orientation = path.poses.back().pose.orientation;
                        }
                        else
                        {
                            pose.pose.orientation.w = 1.0;
                        }
                    }

                    path.poses.push_back(pose);
                }
            }
        }

        ROS_INFO("构建完整路径完成，包含 %lu 个路径点", path.poses.size());
        return path;
    }

    void GlobalPathPlanner::freeSpaceCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        try
        {
            // 转换ROS消息到PCL点云
            pcl::fromROSMsg(*msg, *free_space_cloud_);
            
            // 设置KdTree的输入点云
            if (!free_space_cloud_->empty())
            {
                kdtree_->setInputCloud(free_space_cloud_);
                has_free_space_cloud_ = true;
                
                ROS_DEBUG("更新可行域点云，包含 %lu 个点", free_space_cloud_->size());
            }
            else
            {
                ROS_WARN("接收到空的可行域点云");
                has_free_space_cloud_ = false;
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("处理可行域点云时发生异常: %s", e.what());
            has_free_space_cloud_ = false;
        }
    }

    bool GlobalPathPlanner::isEdgeLegal(const TopologyNode& node1, const TopologyNode& node2)
    {
        if (!has_free_space_cloud_ || free_space_cloud_->empty())
        {
            ROS_WARN("没有可行域点云数据，无法进行边合法性检查");
            return false;
        }
        
        // 将节点转换为Point并减去机器狗高度
        geometry_msgs::Point point1, point2;
        point1.x = node1.x;
        point1.y = node1.y;
        point1.z = node1.z - robot_height_;
        
        point2.x = node2.x;
        point2.y = node2.y;
        point2.z = node2.z - robot_height_;
        
        ROS_DEBUG("检查边合法性: 节点%d(%.2f,%.2f,%.2f) -> 节点%d(%.2f,%.2f,%.2f)", 
                 node1.id, point1.x, point1.y, point1.z,
                 node2.id, point2.x, point2.y, point2.z);
        
        // 对边进行采样
        std::vector<geometry_msgs::Point> sample_points = sampleEdgePoints(point1, point2, edge_sample_count_);
        
        if (sample_points.empty())
        {
            ROS_WARN("边采样失败");
            return false;
        }
        
        int valid_points = 0;
        int total_points = sample_points.size();
        
        // 检查每个采样点
        for (const auto& sample_point : sample_points)
        {
            if (isPointInFreeSpaceWithRadius(sample_point, safety_sphere_radius_))
            {
                valid_points++;
            }
        }
        
        // 计算有效点比例
        double valid_ratio = static_cast<double>(valid_points) / total_points;
        
        ROS_DEBUG("边合法性检查结果: %d/%d 个点有效 (%.1f%%), 阈值: %.1f%%", 
                 valid_points, total_points, valid_ratio * 100, valid_point_threshold_ * 100);
        
        return valid_ratio >= valid_point_threshold_;
    }

    std::vector<geometry_msgs::Point> GlobalPathPlanner::sampleEdgePoints(const geometry_msgs::Point& start, 
                                                                          const geometry_msgs::Point& end, 
                                                                          int sample_count)
    {
        std::vector<geometry_msgs::Point> sample_points;
        
        if (sample_count <= 0)
        {
            ROS_WARN("采样点数量必须大于0");
            return sample_points;
        }
        
        sample_points.reserve(sample_count);
        
        // 均匀采样
        for (int i = 0; i < sample_count; ++i)
        {
            double t = static_cast<double>(i) / (sample_count - 1);  // 从0到1的参数
            
            geometry_msgs::Point sample_point;
            sample_point.x = start.x + t * (end.x - start.x);
            sample_point.y = start.y + t * (end.y - start.y);
            sample_point.z = start.z + t * (end.z - start.z);
            
            sample_points.push_back(sample_point);
        }
        
        ROS_DEBUG("生成 %lu 个边采样点", sample_points.size());
        return sample_points;
    }

    bool GlobalPathPlanner::isPointInFreeSpaceWithRadius(const geometry_msgs::Point& point, double radius)
    {
        if (!has_free_space_cloud_ || free_space_cloud_->empty() || !kdtree_)
        {
            return false;
        }
        
        try
        {
            // 将geometry_msgs::Point转换为PCL点
            pcl::PointXYZ search_point;
            search_point.x = point.x;
            search_point.y = point.y;
            search_point.z = point.z;
            
            // 使用KdTree进行半径搜索
            std::vector<int> point_indices;
            std::vector<float> point_distances;
            
            int found_points = kdtree_->radiusSearch(search_point, radius, point_indices, point_distances);
            
            if (found_points > 0)
            {
                ROS_DEBUG("在半径 %.2f 内找到 %d 个可行域点", radius, found_points);
                return true;
            }
            else
            {
                ROS_DEBUG("在半径 %.2f 内未找到可行域点", radius);
                return false;
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("半径搜索时发生异常: %s", e.what());
            return false;
        }
    }

    double GlobalPathPlanner::calculateConnectionCost(const geometry_msgs::Point& from, 
                                                      const geometry_msgs::Point& to)
    {
        // 基础距离代价
        double distance_cost = calculateDistance(from, to);
        
        // 简单的线性代价函数，距离越远代价越高
        return distance_cost;
    }

    bool GlobalPathPlanner::isConnectionFeasible(const geometry_msgs::Point& from, 
                                                 const geometry_msgs::Point& to)
    {
        // 1. 检查距离是否合理（不能太远）
        double distance = calculateDistance(from, to);
        if (distance > connection_distance_threshold_)
        {
            return false;
        }

        // 2. 如果禁用可行性检查，只检查距离
        if (!enable_feasibility_check_)
        {
            return true;
        }

        // 3. 检查两点间是否有视线（简单的直线障碍物检测）
        if (!checkLineOfSight(from, to))
        {
            return false;
        }

        return true;
    }

    bool GlobalPathPlanner::checkLineOfSight(const geometry_msgs::Point& from, 
                                             const geometry_msgs::Point& to)
    {
        // 简化的视线检查：沿直线采样多个点，检查是否都在自由空间
        int num_samples = static_cast<int>(calculateDistance(from, to) / line_of_sight_resolution_);
        num_samples = std::max(5, std::min(num_samples, 20)); // 限制采样点数量

        for (int i = 1; i < num_samples; ++i)
        {
            double t = static_cast<double>(i) / num_samples;
            geometry_msgs::Point sample_point;
            sample_point.x = from.x + t * (to.x - from.x);
            sample_point.y = from.y + t * (to.y - from.y);
            sample_point.z = from.z + t * (to.z - from.z);

            if (!isPointInFreeSpace(sample_point))
            {
                ROS_DEBUG("视线检查失败：采样点 (%.2f, %.2f, %.2f) 不在自由空间", 
                         sample_point.x, sample_point.y, sample_point.z);
                return false;
            }
        }

        return true;
    }

    bool GlobalPathPlanner::isPointInFreeSpace(const geometry_msgs::Point& point)
    {
        // 简化的自由空间检查
        // 这里可以集成点云地图或栅格地图进行更精确的检查
        
        // 检查点是否距离现有拓扑节点足够近（假设拓扑节点都在自由空间）
        for (const auto& node : nodes_)
        {
            geometry_msgs::Point node_pos;
            node_pos.x = node.x;
            node_pos.y = node.y;
            node_pos.z = node.z;
            
            double dist = calculateDistance(point, node_pos);
            if (dist < free_space_radius_) // 使用配置的自由空间半径
            {
                return true;
            }
        }

        // 也可以检查点是否距离任何拓扑边足够近
        for (const auto& edge : edges_)
        {
            if (node_map_.find(edge.from_id) != node_map_.end() &&
                node_map_.find(edge.to_id) != node_map_.end())
            {
                const TopologyNode& from_node = node_map_[edge.from_id];
                const TopologyNode& to_node = node_map_[edge.to_id];
                
                geometry_msgs::Point from_point, to_point;
                from_point.x = from_node.x;
                from_point.y = from_node.y;
                from_point.z = from_node.z;
                
                to_point.x = to_node.x;
                to_point.y = to_node.y;
                to_point.z = to_node.z;
                
                double dist_to_edge = pointToSegmentDistance(point, from_point, to_point);
                if (dist_to_edge < edge_proximity_threshold_) // 使用配置的边接近阈值
                {
                    return true;
                }
            }
        }

        // 默认：如果距离所有拓扑元素都太远，认为可能不在自由空间
        ROS_DEBUG("点 (%.2f, %.2f, %.2f) 可能不在自由空间", point.x, point.y, point.z);
        return false;
    }
} // namespace global_path_planner
