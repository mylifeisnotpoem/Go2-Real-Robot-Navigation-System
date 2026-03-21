#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <state_machine_msg/keywords.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <csignal> // 用于信号处理
#include <map>     // 用于建立标签映射表

class GoalLoopNode
{
public:
    GoalLoopNode() : is_navigating_(false), current_goal_id_(0), goal_reached_threshold_(0.5)
    {
        setlocale(LC_ALL, "");
        // 初始化ROS节点
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // 从参数服务器获取参数
        private_nh.param<std::string>("key_nodes_file_path", key_nodes_file_path_, "");
        private_nh.param<std::string>("map_frame", map_frame_, "map");
        private_nh.param<double>("goal_reached_threshold", goal_reached_threshold_, 0.5); // 默认0.5米

        if (key_nodes_file_path_.empty())
        {
            ROS_ERROR("未指定关键节点地图文件路径，请通过参数指定");
            ros::shutdown();
            return;
        }

        // 读取关键节点地图文件
        if (readKeyNodesMap())
        {
            ROS_INFO("成功读取关键节点地图文件，共%d个节点", (int)label_to_pose_map_.size());
            printLabelMappings();
        }
        else
        {
            ROS_ERROR("无法读取关键节点地图文件: %s", key_nodes_file_path_.c_str());
            ros::shutdown();
            return;
        }

        // 创建发布者和订阅者
        goal_pub_ = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 10);
        key_command_sub_ = nh.subscribe("/keywords_command", 10, &GoalLoopNode::keyCommandCallback, this);
        odom_sub_ = nh.subscribe("/robot_odom", 10, &GoalLoopNode::odomCallback, this);

        // 创建定时器用于检测是否到达目标点
        timer_ = nh.createTimer(ros::Duration(0.2), &GoalLoopNode::timerCallback, this); // 0.2秒检测一次
        timer_.stop();                                                                   // 初始时停止定时器

        ROS_INFO("循环导航节点初始化完成");
    }

private:
    bool readKeyNodesMap()
    {
        std::ifstream file(key_nodes_file_path_);
        if (!file.is_open())
        {
            ROS_ERROR("无法打开关键节点地图文件: %s", key_nodes_file_path_.c_str());
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            // 跳过注释行和空行
            if (line.empty() || line[0] == '#')
                continue;

            std::stringstream ss(line);
            std::string value;
            std::vector<std::string> row;

            // 按空格分割
            while (ss >> value)
            {
                row.push_back(value);
            }

            // 检查行格式: 节点ID X Y Z 标签
            if (row.size() >= 5)
            {
                try
                {
                    int node_id = std::stoi(row[0]);
                    double x = std::stod(row[1]);
                    double y = std::stod(row[2]);
                    double z = std::stod(row[3]);
                    std::string label = row[4];

                    // 建立标签到坐标的映射
                    geometry_msgs::Point point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    
                    label_to_pose_map_[label] = point;
                    id_to_label_map_[node_id] = label;
                    
                    ROS_DEBUG("读取节点: ID=%d, 标签=%s, 坐标=(%.2f, %.2f, %.2f)", 
                             node_id, label.c_str(), x, y, z);
                }
                catch (const std::exception &e)
                {
                    ROS_WARN("解析关键节点行时出错: %s - %s", line.c_str(), e.what());
                }
            }
            else
            {
                ROS_WARN("关键节点行格式错误，应为5列(ID,X,Y,Z,标签)，但发现%d列: %s", 
                        (int)row.size(), line.c_str());
            }
        }

        file.close();
        return !label_to_pose_map_.empty();
    }

    void printLabelMappings()
    {
        ROS_INFO("=== 关键节点标签映射表 ===");
        for (const auto &pair : label_to_pose_map_)
        {
            ROS_INFO("标签: %s -> 坐标: (%.2f, %.2f, %.2f)", 
                    pair.first.c_str(), pair.second.x, pair.second.y, pair.second.z);
        }
        ROS_INFO("========================");
    }

    void keyCommandCallback(const state_machine_msg::keywords::ConstPtr &msg)
    {
        ROS_INFO("收到关键词命令: %s, 目标点: %d", msg->command.c_str(), msg->target_point);

        // 首先检查命令中的关键词是否存在于标签映射表中
        bool found_label_match = false;
        std::string matched_label = "";
        
        // 遍历标签映射表，检查是否有标签与命令匹配
        for (const auto &pair : label_to_pose_map_)
        {
            // 检查标签是否在命令中出现
            if (msg->command.find(pair.first) != std::string::npos)
            {
                matched_label = pair.first;
                found_label_match = true;
                break;
            }
        }
        
        // 如果在txt文件中找到了匹配的标签，则进行导航
        if (found_label_match)
        {
            auto it = label_to_pose_map_.find(matched_label);
            if (it != label_to_pose_map_.end())
            {
                ROS_INFO("找到标签匹配: %s -> 坐标(%.2f, %.2f, %.2f)", 
                        matched_label.c_str(), it->second.x, it->second.y, it->second.z);
                
                // 如果当前正在导航，先停止当前导航
                if (is_navigating_)
                {
                    geometry_msgs::PointStamped empty_goal;
                    empty_goal.header.frame_id = map_frame_;
                    empty_goal.header.stamp = ros::Time::now();
                    goal_pub_.publish(empty_goal);
                    ROS_INFO("停止当前导航");
                    ros::Duration(0.5).sleep();
                }

                is_navigating_ = false;
                timer_.stop();
                
                // 发布标签对应的3D坐标点
                publishLabelGoal(matched_label, it->second);
            }
            return;
        }

        // 如果没有找到标签匹配，检查是否是系统命令
        else if (msg->command == "导航开始")
        {
            if (!is_navigating_ && !label_to_pose_map_.empty())
            {
                is_navigating_ = true;
                current_goal_id_ = 0;
                ROS_INFO("开始循环导航");
                // 开始从第一个标签点循环导航
                auto it = label_to_pose_map_.begin();
                publishLabelGoal(it->first, it->second);
                timer_.start();
                return;
            }
            else if (label_to_pose_map_.empty())
            {
                ROS_WARN("没有可用的导航点，无法开始导航");
                return;
            }
            else
            {
                ROS_INFO("导航已经开始，忽略命令");
                return;
            }
        }
        else if (msg->command == "导航终止")
        {
            if (is_navigating_)
            {
                is_navigating_ = false;
                timer_.stop();
                ROS_INFO("导航终止");

                // 发送空目标以停止机器人
                geometry_msgs::PointStamped empty_goal;
                empty_goal.header.frame_id = map_frame_;
                empty_goal.header.stamp = ros::Time::now();
                goal_pub_.publish(empty_goal);
                ROS_INFO("发送空目标停止机器人");
                return;
            }
            else
            {
                ROS_INFO("导航未开始，忽略命令");
                return;
            }
        }
        else if (msg->command == "回家")
        {
            ROS_INFO("收到回家命令，导航回原点位置(0,0,0)");

            // 如果当前正在导航，先停止当前导航
            if (is_navigating_)
            {
                is_navigating_ = false;
                timer_.stop();

                // 发送空目标以停止当前导航
                geometry_msgs::PointStamped empty_goal;
                empty_goal.header.frame_id = map_frame_;
                empty_goal.header.stamp = ros::Time::now();
                goal_pub_.publish(empty_goal);
                ROS_INFO("停止当前导航");

                // 等待短暂时间让机器人停下
                ros::Duration(0.5).sleep();
            }

            // 创建一个位于原点(0,0)的导航目标
            geometry_msgs::PointStamped home_goal;
            home_goal.header.frame_id = map_frame_;
            home_goal.header.stamp = ros::Time::now();
            home_goal.point.x = 0.0;
            home_goal.point.y = 0.0;
            home_goal.point.z = 0.0;

            goal_pub_.publish(home_goal);
            ROS_INFO("发布回家导航目标: 原点(0.0, 0.0, 0.0)");
            return;

        }
        else
        {
            // 如果没有找到标签匹配，也不是系统命令，输出警告
            ROS_WARN("未找到匹配的标签或系统命令，命令: %s", msg->command.c_str());
            return;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 保存当前位置，可用于距离检测或其他功能
        current_position_ = msg->pose.pose.position;
    }

    void timerCallback(const ros::TimerEvent &)
    {
        if (is_navigating_ && !label_to_pose_map_.empty())
        {
            // 检查是否已到达当前目标点
            if (isReachedCurrentGoal())
            {
                // 到达当前目标点，切换到下一个目标点
                current_goal_id_ = (current_goal_id_ + 1) % label_to_pose_map_.size();
                
                auto it = label_to_pose_map_.begin();
                std::advance(it, current_goal_id_);
                
                ROS_INFO("已到达目标点，循环导航到下一点: %s", it->first.c_str());
                publishLabelGoal(it->first, it->second);
            }
        }
    }

    bool isReachedCurrentGoal()
    {
        if (label_to_pose_map_.empty() || current_goal_id_ >= label_to_pose_map_.size())
        {
            return false;
        }

        // 获取当前目标点的坐标
        auto it = label_to_pose_map_.begin();
        std::advance(it, current_goal_id_);
        
        // 计算当前位置与目标点之间的距离
        double dx = current_position_.x - it->second.x;
        double dy = current_position_.y - it->second.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 判断是否在阈值范围内
        if (distance <= goal_reached_threshold_)
        {
            ROS_INFO("已到达目标点 %s, 距离: %.2f米", it->first.c_str(), distance);
            return true;
        }

        return false;
    }

    void publishGoal(int goal_id)
    {
        if (label_to_pose_map_.empty() || goal_id >= label_to_pose_map_.size())
        {
            ROS_WARN("目标点ID %d 超出范围", goal_id);
            return;
        }

        // 获取指定索引的目标点
        auto it = label_to_pose_map_.begin();
        std::advance(it, goal_id);

        geometry_msgs::PointStamped goal_msg;
        goal_msg.header.frame_id = map_frame_;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.point.x = it->second.x;
        goal_msg.point.y = it->second.y;
        goal_msg.point.z = it->second.z;

        goal_pub_.publish(goal_msg);
        ROS_INFO("发布导航目标: %s -> 坐标(%.2f, %.2f, %.2f)", 
                it->first.c_str(), it->second.x, it->second.y, it->second.z);
    }

    void publishLabelGoal(const std::string &label, const geometry_msgs::Point &point)
    {
        geometry_msgs::PointStamped goal_msg;
        goal_msg.header.frame_id = map_frame_;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.point = point;

        goal_pub_.publish(goal_msg);
        ROS_INFO("发布标签目标: %s -> 坐标(%.2f, %.2f, %.2f)", 
                label.c_str(), point.x, point.y, point.z);
    }

    // 成员变量
    std::string key_nodes_file_path_;
    std::string map_frame_;
    std::map<std::string, geometry_msgs::Point> label_to_pose_map_;  // 标签到坐标的映射
    std::map<int, std::string> id_to_label_map_;                     // ID到标签的映射
    ros::Publisher goal_pub_;
    ros::Subscriber key_command_sub_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;
    bool is_navigating_;
    int current_goal_id_;
    geometry_msgs::Point current_position_;
    double goal_reached_threshold_; // 目标点到达判定阈值（单位：米）
};

// 信号处理器
void signalHandler(int sig)
{
    // 发送空目标以停止机器人
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 1);

    geometry_msgs::PointStamped empty_goal;
    empty_goal.header.frame_id = "map";
    empty_goal.header.stamp = ros::Time::now();

    // 循环发布几次，确保消息被接收
    for (int i = 0; i < 3; i++)
    {
        goal_pub.publish(empty_goal);
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("节点终止，已发送停止命令");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_loop_node");

    // 注册信号处理器
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    GoalLoopNode goal_loop_node;
    ros::spin();
    return 0;
}
