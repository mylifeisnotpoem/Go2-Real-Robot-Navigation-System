#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <unitree_go/WirelessController.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <signal.h>
#include <chrono>
#include <iomanip>

struct TopologyNode
{
    int id;
    double x, y, z;
    std::string label;
    int path_index;  // 新增：记录该节点对应路径中的索引

    TopologyNode(int _id, double _x, double _y, double _z, const std::string &_label, int _path_index = -1)
        : id(_id), x(_x), y(_y), z(_z), label(_label), path_index(_path_index) {}
};

struct TopologyEdge
{
    int id;
    int start_node_id;
    int end_node_id;
    double weight;
    std::string type;
    bool bidirectional;

    TopologyEdge(int _id, int _start, int _end, double _weight,
                 const std::string &_type, bool _bidirectional)
        : id(_id), start_node_id(_start), end_node_id(_end),
          weight(_weight), type(_type), bidirectional(_bidirectional) {}
};

struct TargetPoint
{
    int id;
    double x, y, z;
    double qx, qy, qz, qw; // 四元数
    std::string label;
    int path_index;  // 新增：记录该目标点对应路径中的索引

    TargetPoint(int _id, double _x, double _y, double _z,
                double _qx, double _qy, double _qz, double _qw,
                const std::string &_label, int _path_index = -1)
        : id(_id), x(_x), y(_y), z(_z), qx(_qx), qy(_qy), qz(_qz), qw(_qw), label(_label), path_index(_path_index) {}
};

class TopologyMapGenerator
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber wireless_controller_sub_;
    ros::Subscriber topology_node_set_sub_;
    ros::Publisher nav_set_point_pub_;

    std::vector<TopologyNode> nodes_;
    std::vector<TopologyEdge> edges_;
    std::vector<TargetPoint> target_points_;  // 新增：目标点容器

    int node_interval_;
    int path_point_counter_;
    int next_node_id_;
    int next_edge_id_;
    std::string output_file_path_;
    std::string target_points_file_path_;  // 新增：目标点文件路径
    std::string path_topic_;
    std::string edge_type_;
    bool auto_bidirectional_;

    nav_msgs::Path last_path_;
    bool path_received_;
    int total_path_points_processed_;  // 新增：记录已处理的总路径点数

    // 手动节点生成相关变量
    int manual_node_counter_;
    std::chrono::steady_clock::time_point last_manual_trigger_time_;
    const int manual_trigger_delay_ms_ = 2000; // 2秒延时

    // 中文数字映射（已废弃，标签自动生成）

public:
    TopologyMapGenerator() : nh_("~"),
                             path_point_counter_(0),
                             next_node_id_(0),
                             next_edge_id_(0),
                             path_received_(false),
                             total_path_points_processed_(0),
                             manual_node_counter_(0),
                             last_manual_trigger_time_(std::chrono::steady_clock::now())
    {

        // 读取参数
        nh_.param<int>("node_interval", node_interval_, 50);
        nh_.param<std::string>("output_file", output_file_path_, "map.txt");
        nh_.param<std::string>("path_topic", path_topic_, "/path");
        nh_.param<std::string>("edge_type", edge_type_, "auto");
        nh_.param<bool>("auto_bidirectional", auto_bidirectional_, true);

        // 设置目标点文件路径（基于输出文件路径）
        size_t last_dot = output_file_path_.find_last_of('.');
        if (last_dot != std::string::npos)
        {
            target_points_file_path_ = output_file_path_.substr(0, last_dot) + "_target_points.txt";
        }
        else
        {
            target_points_file_path_ = output_file_path_ + "_target_points.txt";
        }

        // 订阅路径话题
        path_sub_ = nh_.subscribe(path_topic_, 10,
                                  &TopologyMapGenerator::pathCallback, this);

        // 订阅无线控制器话题
        wireless_controller_sub_ = nh_.subscribe("/wireless_controller", 10,
                                                 &TopologyMapGenerator::wirelessControllerCallback, this);

        // 订阅拓扑节点设置话题
        topology_node_set_sub_ = nh_.subscribe("/topology_node_set", 10,
                                               &TopologyMapGenerator::topologyNodeSetCallback, this);

        // 初始化导航设置点发布器
        nav_set_point_pub_ = nh_.advertise<std_msgs::Int32>("/nav_set_point", 10);

        ROS_INFO("拓扑地图生成器已启动");
        ROS_INFO("参数配置:");
        ROS_INFO("  - 节点间隔: %d 个路径点", node_interval_);
        ROS_INFO("  - 输出文件: %s", output_file_path_.c_str());
        ROS_INFO("  - 目标点文件: %s", target_points_file_path_.c_str());
        ROS_INFO("  - 路径话题: %s", path_topic_.c_str());
        ROS_INFO("  - 边类型: %s", edge_type_.c_str());
        ROS_INFO("  - 自动双向: %s", auto_bidirectional_ ? "是" : "否");
        ROS_INFO("  - 手动节点生成: 按键值32触发，2秒防重复延时");
        ROS_INFO("  - 话题节点生成: /topology_node_set 话题触发 (bool=true)，2秒防重复延时");
        ROS_INFO("  - 导航设置点发布器: /nav_set_point (一号=1, 二号=2, ...)");
    }

    void wirelessControllerCallback(const unitree_go::WirelessController::ConstPtr &msg)
    {
        // 检查是否是目标按键值
        if (msg->keys != 32)
        {
            return;
        }

        // 检查防重复触发延时
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_trigger = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           current_time - last_manual_trigger_time_)
                                           .count();

        if (time_since_last_trigger < manual_trigger_delay_ms_)
        {
            ROS_DEBUG("手动节点生成被延时阻止，距离上次触发仅 %ld ms", time_since_last_trigger);
            return;
        }

        // 不再限制节点数量

        // 获取当前位置
        geometry_msgs::PoseStamped current_pose;
        if (!getCurrentPose(current_pose))
        {
            ROS_WARN("无法获取当前位置，跳过手动节点生成");
            return;
        }

    // 自动生成标签：第N号
    std::string label = std::to_string(manual_node_counter_ + 1) + "号";

        // 获取当前在路径中的索引（使用路径最后一个点的索引）
        int current_path_index = path_received_ && !last_path_.poses.empty() ? 
                                 (int)last_path_.poses.size() - 1 : -1;

        // 创建新节点
        TopologyNode new_node(
            next_node_id_,
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z,
            label,
            current_path_index);  // 记录路径索引

        nodes_.push_back(new_node);

        ROS_INFO("手动生成新节点 ID: %d, 位置: (%.6f, %.6f, %.6f), 标签: %s, 路径索引: %d",
                 next_node_id_, new_node.x, new_node.y, new_node.z, label.c_str(), current_path_index);

        // 创建对应的目标点（包含完整的位姿信息）
        TargetPoint new_target_point(
            manual_node_counter_,  // 使用手动节点计数器作为目标点ID
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z,
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w,
            label,
            current_path_index);  // 记录路径索引

        target_points_.push_back(new_target_point);

        ROS_INFO("创建目标点 ID: %d, 位置: (%.6f, %.6f, %.6f), 四元数: (%.6f, %.6f, %.6f, %.6f), 标签: %s",
                 manual_node_counter_, new_target_point.x, new_target_point.y, new_target_point.z,
                 new_target_point.qx, new_target_point.qy, new_target_point.qz, new_target_point.qw,
                 label.c_str());

        // 如果不是第一个节点，创建与前一个节点的连接
        if (next_node_id_ > 0)
        {
            const TopologyNode &prev_node = nodes_[nodes_.size() - 2];

            // 计算两节点间距离作为权重
            double dx = new_node.x - prev_node.x;
            double dy = new_node.y - prev_node.y;
            double dz = new_node.z - prev_node.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            // 创建边
            TopologyEdge new_edge(
                next_edge_id_,
                prev_node.id,
                new_node.id,
                distance,
                "manual",
                auto_bidirectional_);

            edges_.push_back(new_edge);

            ROS_INFO("创建手动边 ID: %d, 连接节点 %d -> %d, 权重: %.6f",
                     next_edge_id_, prev_node.id, new_node.id, distance);

            next_edge_id_++;
        }

        next_node_id_++;
        manual_node_counter_++;
        last_manual_trigger_time_ = current_time;

        // 发布导航设置点 (一号=1, 二号=2, ...)
        publishNavSetPoint(manual_node_counter_);

    ROS_INFO("手动节点生成完成，下个节点将标记为: %s",
         (std::to_string(manual_node_counter_ + 1) + "号").c_str());
    }

    void topologyNodeSetCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        // 仅在收到 true 值时触发节点生成
        if (!msg->data)
        {
            return;
        }

        // 检查防重复触发延时
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_trigger = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           current_time - last_manual_trigger_time_)
                                           .count();

        if (time_since_last_trigger < manual_trigger_delay_ms_)
        {
            ROS_DEBUG("手动节点生成被延时阻止，距离上次触发仅 %ld ms", time_since_last_trigger);
            return;
        }

        // 不再限制节点数量

        // 获取当前位置
        geometry_msgs::PoseStamped current_pose;
        if (!getCurrentPose(current_pose))
        {
            ROS_WARN("无法获取当前位置，跳过手动节点生成");
            return;
        }

    // 自动生成标签：第N号
    std::string label = std::to_string(manual_node_counter_ + 1) + "号";

        // 获取当前在路径中的索引（使用路径最后一个点的索引）
        int current_path_index = path_received_ && !last_path_.poses.empty() ? 
                                 (int)last_path_.poses.size() - 1 : -1;

        // 创建新节点
        TopologyNode new_node(
            next_node_id_,
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z,
            label,
            current_path_index);  // 记录路径索引

        nodes_.push_back(new_node);

        ROS_INFO("通过话题生成新节点 ID: %d, 位置: (%.6f, %.6f, %.6f), 标签: %s, 路径索引: %d",
                 next_node_id_, new_node.x, new_node.y, new_node.z, label.c_str(), current_path_index);

        // 创建对应的目标点（包含完整的位姿信息）
        TargetPoint new_target_point(
            manual_node_counter_,  // 使用手动节点计数器作为目标点ID
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z,
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w,
            label,
            current_path_index);  // 记录路径索引

        target_points_.push_back(new_target_point);

        ROS_INFO("创建目标点 ID: %d, 位置: (%.6f, %.6f, %.6f), 四元数: (%.6f, %.6f, %.6f, %.6f), 标签: %s",
                 manual_node_counter_, new_target_point.x, new_target_point.y, new_target_point.z,
                 new_target_point.qx, new_target_point.qy, new_target_point.qz, new_target_point.qw,
                 label.c_str());

        // 如果不是第一个节点，创建与前一个节点的连接
        if (next_node_id_ > 0)
        {
            const TopologyNode &prev_node = nodes_[nodes_.size() - 2];

            // 计算两节点间距离作为权重
            double dx = new_node.x - prev_node.x;
            double dy = new_node.y - prev_node.y;
            double dz = new_node.z - prev_node.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            // 创建边
            TopologyEdge new_edge(
                next_edge_id_,
                prev_node.id,
                new_node.id,
                distance,
                "topic",  // 标记为通过话题触发
                auto_bidirectional_);

            edges_.push_back(new_edge);

            ROS_INFO("创建话题触发边 ID: %d, 连接节点 %d -> %d, 权重: %.6f",
                     next_edge_id_, prev_node.id, new_node.id, distance);

            next_edge_id_++;
        }

        next_node_id_++;
        manual_node_counter_++;
        last_manual_trigger_time_ = current_time;

        // 发布导航设置点 (一号=1, 二号=2, ...)
        publishNavSetPoint(manual_node_counter_);

    ROS_INFO("话题触发节点生成完成，下个节点将标记为: %s",
         (std::to_string(manual_node_counter_ + 1) + "号").c_str());
    }

    bool getCurrentPose(geometry_msgs::PoseStamped &pose)
    {
        // 如果有路径数据，使用最新的路径点
        if (path_received_ && !last_path_.poses.empty())
        {
            pose = last_path_.poses.back();
            return true;
        }

        // 如果没有路径数据但有已生成的节点，使用最后一个节点的位置
        if (!nodes_.empty())
        {
            const TopologyNode &last_node = nodes_.back();
            pose.pose.position.x = last_node.x;
            pose.pose.position.y = last_node.y;
            pose.pose.position.z = last_node.z;
            pose.pose.orientation.w = 1.0; // 默认朝向
            return true;
        }

        // 都没有数据，返回原点
        ROS_WARN("没有位置数据，使用原点作为默认位置");
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        return true;
    }

    void publishNavSetPoint(int point_number)
    {
        std_msgs::Int32 msg;
        msg.data = point_number;
        nav_set_point_pub_.publish(msg);

    ROS_INFO("发布导航设置点: %d (%s)", point_number,
         (std::to_string(point_number) + "号").c_str());
    }

    void updateNodesFromOptimizedPath(const nav_msgs::Path& optimized_path)
    {
        if (optimized_path.poses.empty() || nodes_.empty())
        {
            return;
        }

        ROS_INFO("开始根据优化路径更新 %d 个拓扑节点位置", (int)nodes_.size());

        // 更新所有拓扑节点的位置
        for (auto& node : nodes_)
        {
            if (node.path_index >= 0 && node.path_index < (int)optimized_path.poses.size())
            {
                const auto& optimized_pose = optimized_path.poses[node.path_index];
                
                double old_x = node.x, old_y = node.y, old_z = node.z;
                node.x = optimized_pose.pose.position.x;
                node.y = optimized_pose.pose.position.y;
                node.z = optimized_pose.pose.position.z;

                double displacement = std::sqrt(
                    std::pow(node.x - old_x, 2) + 
                    std::pow(node.y - old_y, 2) + 
                    std::pow(node.z - old_z, 2)
                );

                ROS_DEBUG("节点 %d (%s) 位置更新: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f), 位移: %.3fm",
                         node.id, node.label.c_str(), old_x, old_y, old_z, 
                         node.x, node.y, node.z, displacement);
            }
            else
            {
                ROS_WARN("节点 %d (%s) 的路径索引 %d 超出优化路径范围 [0, %d]",
                        node.id, node.label.c_str(), node.path_index, (int)optimized_path.poses.size() - 1);
            }
        }

        // 更新所有目标点的位置
        for (auto& target_point : target_points_)
        {
            if (target_point.path_index >= 0 && target_point.path_index < (int)optimized_path.poses.size())
            {
                const auto& optimized_pose = optimized_path.poses[target_point.path_index];
                
                double old_x = target_point.x, old_y = target_point.y, old_z = target_point.z;
                target_point.x = optimized_pose.pose.position.x;
                target_point.y = optimized_pose.pose.position.y;
                target_point.z = optimized_pose.pose.position.z;
                target_point.qx = optimized_pose.pose.orientation.x;
                target_point.qy = optimized_pose.pose.orientation.y;
                target_point.qz = optimized_pose.pose.orientation.z;
                target_point.qw = optimized_pose.pose.orientation.w;

                double displacement = std::sqrt(
                    std::pow(target_point.x - old_x, 2) + 
                    std::pow(target_point.y - old_y, 2) + 
                    std::pow(target_point.z - old_z, 2)
                );

                ROS_DEBUG("目标点 %d (%s) 位置更新: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f), 位移: %.3fm",
                         target_point.id, target_point.label.c_str(), old_x, old_y, old_z, 
                         target_point.x, target_point.y, target_point.z, displacement);
            }
        }

        // 重新计算所有边的权重
        updateEdgeWeights();

        ROS_INFO("拓扑节点位置更新完成");
    }

    void updateEdgeWeights()
    {
        for (auto& edge : edges_)
        {
            // 查找起始和结束节点
            auto start_node_it = std::find_if(nodes_.begin(), nodes_.end(),
                [&edge](const TopologyNode& node) { return node.id == edge.start_node_id; });
            
            auto end_node_it = std::find_if(nodes_.begin(), nodes_.end(),
                [&edge](const TopologyNode& node) { return node.id == edge.end_node_id; });

            if (start_node_it != nodes_.end() && end_node_it != nodes_.end())
            {
                double old_weight = edge.weight;
                
                // 重新计算距离
                double dx = end_node_it->x - start_node_it->x;
                double dy = end_node_it->y - start_node_it->y;
                double dz = end_node_it->z - start_node_it->z;
                edge.weight = std::sqrt(dx * dx + dy * dy + dz * dz);

                ROS_DEBUG("边 %d 权重更新: %.3f -> %.3f", edge.id, old_weight, edge.weight);
            }
        }
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (msg->poses.empty())
        {
            return;
        }

        // 检查是否是路径优化（路径长度变化或路径被重置）
        bool path_optimized = false;
        if (path_received_ && !last_path_.poses.empty())
        {
            // 如果新路径比之前处理的总点数还少，说明发生了优化
            if ((int)msg->poses.size() < total_path_points_processed_)
            {
                path_optimized = true;
                ROS_INFO("检测到路径优化：新路径长度 %d < 已处理点数 %d", 
                        (int)msg->poses.size(), total_path_points_processed_);
            }
            // 或者路径的早期部分发生了显著变化
            else
            {
                int check_points = std::min(50, std::min((int)msg->poses.size(), (int)last_path_.poses.size()));
                double total_displacement = 0.0;
                
                for (int i = 0; i < check_points; ++i)
                {
                    const auto& new_pose = msg->poses[i].pose.position;
                    const auto& old_pose = last_path_.poses[i].pose.position;
                    
                    total_displacement += std::sqrt(
                        std::pow(new_pose.x - old_pose.x, 2) +
                        std::pow(new_pose.y - old_pose.y, 2) +
                        std::pow(new_pose.z - old_pose.z, 2)
                    );
                }
                
                double avg_displacement = total_displacement / check_points;
                if (avg_displacement > 0.1)  // 平均位移超过10cm认为是优化
                {
                    path_optimized = true;
                    ROS_INFO("检测到路径优化：前%d个点平均位移 %.3fm", check_points, avg_displacement);
                }
            }
        }

        last_path_ = *msg;
        path_received_ = true;

        // 如果检测到路径优化，更新所有现有节点的位置
        if (path_optimized && !nodes_.empty())
        {
            updateNodesFromOptimizedPath(*msg);
        }

        // 检查是否需要生成新节点（仅在路径继续增长时）
        if ((int)msg->poses.size() > total_path_points_processed_)
        {
            // 计算需要处理的新增路径点
            int new_points_start = total_path_points_processed_;
            int new_points_count = (int)msg->poses.size() - total_path_points_processed_;
            
            for (int i = 0; i < new_points_count; ++i)
            {
                int current_point_index = new_points_start + i;
                
                if (path_point_counter_ % node_interval_ == 0)
                {
                    // 获取对应的位置点
                    const auto &latest_pose = msg->poses[current_point_index];

                    // 创建新节点，记录其在路径中的索引
                    TopologyNode new_node(
                        next_node_id_,
                        latest_pose.pose.position.x,
                        latest_pose.pose.position.y,
                        latest_pose.pose.position.z,
                        "node_",
                        current_point_index);  // 记录路径索引

                    nodes_.push_back(new_node);

                    ROS_INFO("生成新节点 ID: %d, 位置: (%.6f, %.6f, %.6f), 路径索引: %d",
                             next_node_id_, new_node.x, new_node.y, new_node.z, current_point_index);

                    // 如果不是第一个节点，创建与前一个节点的连接
                    if (next_node_id_ > 0)
                    {
                        const TopologyNode &prev_node = nodes_[nodes_.size() - 2];

                        // 计算两节点间距离作为权重
                        double dx = new_node.x - prev_node.x;
                        double dy = new_node.y - prev_node.y;
                        double dz = new_node.z - prev_node.z;
                        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                        // 创建边
                        TopologyEdge new_edge(
                            next_edge_id_,
                            prev_node.id,
                            new_node.id,
                            distance,
                            edge_type_,
                            auto_bidirectional_);

                        edges_.push_back(new_edge);

                        ROS_INFO("创建边 ID: %d, 连接节点 %d -> %d, 权重: %.6f",
                                 next_edge_id_, prev_node.id, new_node.id, distance);

                        next_edge_id_++;
                    }

                    next_node_id_++;
                }

                path_point_counter_++;
            }
            
            total_path_points_processed_ = (int)msg->poses.size();
        }

        // 每100个点输出一次统计信息
        if (path_point_counter_ % 100 == 0)
        {
            ROS_INFO("已处理 %d 个路径点, 生成 %d 个节点, %d 条边",
                     path_point_counter_, (int)nodes_.size(), (int)edges_.size());
        }
    }

    void saveTopologyMap()
    {
        if (nodes_.empty())
        {
            ROS_WARN("没有生成任何节点，不保存文件");
            return;
        }

        std::ofstream file(output_file_path_);
        if (!file.is_open())
        {
            ROS_ERROR("无法打开输出文件: %s", output_file_path_.c_str());
            return;
        }

        // 写入文件头
        file << "# 拓扑地图文件 (UTF-8编码)" << std::endl;
        file << "# 格式: 节点ID X Y Z 标签 路径索引" << std::endl;
        file << "# Topology Nodes" << std::endl;

        // 写入节点
        for (const auto &node : nodes_)
        {
            file << node.id << " "
                 << std::fixed << std::setprecision(6)
                 << node.x << " " << node.y << " " << node.z << " "
                 << node.label << " " << node.path_index << std::endl;
        }

        // 写入边的格式说明
        file << "# 格式: 边ID 起始节点ID 结束节点ID 权重 类型 双向" << std::endl;
        file << "# Topology Edges" << std::endl;

        // 写入边
        for (const auto &edge : edges_)
        {
            file << edge.id << " "
                 << edge.start_node_id << " " << edge.end_node_id << " "
                 << std::fixed << std::setprecision(6) << edge.weight << " "
                 << edge.type << " " << (edge.bidirectional ? 1 : 0) << std::endl;
        }

        file.close();

        ROS_INFO("拓扑地图已保存到: %s", output_file_path_.c_str());
        ROS_INFO("总计: %d 个节点 (其中 %d 个手动节点), %d 条边",
                 (int)nodes_.size(), manual_node_counter_, (int)edges_.size());
    }

    void saveTargetPoints()
    {
        if (target_points_.empty())
        {
            ROS_WARN("没有生成任何目标点，不保存目标点文件");
            return;
        }

        std::ofstream file(target_points_file_path_);
        if (!file.is_open())
        {
            ROS_ERROR("无法打开目标点输出文件: %s", target_points_file_path_.c_str());
            return;
        }

        // 写入文件头
        file << "# 目标点文件 (UTF-8编码)" << std::endl;
        file << "# 格式: 目标点ID X Y Z QX QY QZ QW 标签 路径索引" << std::endl;
        file << "# Target Points" << std::endl;

        // 写入目标点
        for (const auto &target_point : target_points_)
        {
            file << target_point.id << " "
                 << std::fixed << std::setprecision(6)
                 << target_point.x << " " << target_point.y << " " << target_point.z << " "
                 << target_point.qx << " " << target_point.qy << " " 
                 << target_point.qz << " " << target_point.qw << " "
                 << target_point.label << " " << target_point.path_index << std::endl;
        }

        file.close();

        ROS_INFO("目标点已保存到: %s", target_points_file_path_.c_str());
        ROS_INFO("总计: %d 个目标点", (int)target_points_.size());
    }

    void addFinalNodeIfNeeded()
    {
        if (!path_received_ || last_path_.poses.empty())
        {
            return;
        }

        // 如果路径点数不是interval的整数倍，添加最后一个点作为节点
        if (path_point_counter_ % node_interval_ != 0)
        {
            const auto &final_pose = last_path_.poses.back();

            TopologyNode final_node(
                next_node_id_,
                final_pose.pose.position.x,
                final_pose.pose.position.y,
                final_pose.pose.position.z,
                "node_",
                (int)last_path_.poses.size() - 1);  // 记录路径索引

            nodes_.push_back(final_node);

            // 如果有前一个节点，创建连接
            if (next_node_id_ > 0)
            {
                const TopologyNode &prev_node = nodes_[nodes_.size() - 2];

                double dx = final_node.x - prev_node.x;
                double dy = final_node.y - prev_node.y;
                double dz = final_node.z - prev_node.z;
                double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                TopologyEdge final_edge(
                    next_edge_id_,
                    prev_node.id,
                    final_node.id,
                    distance,
                    edge_type_,
                    auto_bidirectional_);

                edges_.push_back(final_edge);
            }

            ROS_INFO("添加最终节点 ID: %d, 位置: (%.6f, %.6f, %.6f), 路径索引: %d",
                     next_node_id_, final_node.x, final_node.y, final_node.z, final_node.path_index);
        }
    }

    void shutdown()
    {
        ROS_INFO("正在关闭拓扑地图生成器...");
        addFinalNodeIfNeeded();
        
        // 在保存前进行最后一次优化更新
        if (path_received_ && !last_path_.poses.empty() && !nodes_.empty())
        {
            ROS_INFO("执行最终位置优化...");
            updateNodesFromOptimizedPath(last_path_);
        }
        
        saveTopologyMap();
        saveTargetPoints();  // 新增：保存目标点文件
    }
};

// 全局指针，用于信号处理
TopologyMapGenerator *g_generator = nullptr;

void signalHandler(int signum)
{
    ROS_INFO("接收到关闭信号 (%d), 保存拓扑地图...", signum);
    if (g_generator)
    {
        g_generator->shutdown();
    }
    ros::shutdown();
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "topology_map_generator");

    TopologyMapGenerator generator;
    g_generator = &generator;

    // 注册信号处理器
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ROS_INFO("拓扑地图生成器正在运行，按 Ctrl+C 停止并保存地图");

    ros::spin();

    // 正常退出时也保存地图
    generator.shutdown();

    return 0;
}
