#include "user_interface/navigation_task_state_machine.h"
#include <sstream>
#include <algorithm>
#include <locale>
#include <dirent.h>
#include <sys/stat.h>

namespace user_interface
{

    NavigationTaskStateMachine::NavigationTaskStateMachine()
        : current_state_(NavigationTaskState::IDLE), previous_state_(NavigationTaskState::IDLE), current_task_index_(0), current_wait_delay_(0), nav_state_(false), nav_state_received_(false), target_point_published_(false), last_task_signal_(NavigationTaskSignal::STOP), state_machine_frequency_(10.0)
    {
        ROS_INFO("NavigationTaskStateMachine 构造函数");
    }

    NavigationTaskStateMachine::~NavigationTaskStateMachine()
    {
        ROS_INFO("NavigationTaskStateMachine 析构函数");
    }

    bool NavigationTaskStateMachine::initialize(ros::NodeHandle &nh)
    {
        nh_ = nh;

        try
        {
            // 加载参数
            loadParameters();

            // 初始化发布器
            task_action_pub_ = nh_.advertise<std_msgs::Int32>("/task_action", 10);
            target_points_pub_ = nh_.advertise<geometry_msgs::Pose>("/target_points", 10);

            // 初始化订阅器
            nav_state_sub_ = nh_.subscribe("/nav_state", 10,
                                           &NavigationTaskStateMachine::navStateCallback, this);
            nav_task_signal_sub_ = nh_.subscribe("/nav_task_signal", 10,
                                                 &NavigationTaskStateMachine::navTaskSignalCallback, this);

            // 初始化状态机定时器
            state_machine_timer_ = nh_.createTimer(ros::Duration(1.0 / state_machine_frequency_),
                                                   &NavigationTaskStateMachine::stateMachineTimerCallback, this);

            // 加载任务数据
            if (!loadAllTaskLists())
            {
                ROS_ERROR("加载任务列表失败");
                return false;
            }

            if (!loadTargetPoints())
            {
                ROS_ERROR("加载目标点失败");
                return false;
            }

            // 校验任务列表
            if (!validateTaskList())
            {
                ROS_ERROR("任务列表校验失败");
                return false;
            }

            ROS_INFO("NavigationTaskStateMachine 初始化成功");
            ROS_INFO("加载了 %zu 个任务文件和 %zu 个目标点", task_lists_map_.size(), target_points_map_.size());

            return true;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("NavigationTaskStateMachine 初始化失败: %s", e.what());
            return false;
        }
    }

    void NavigationTaskStateMachine::loadParameters()
    {
        // 获取配置文件路径
        nh_.param<std::string>("task_folder_path", task_folder_path_,
                               "$(find user_interface)/config/task");
        nh_.param<std::string>("target_points_file", target_points_file_,
                               "$(find user_interface)/config/generated_map_target_points.txt");
        nh_.param<double>("state_machine_frequency", state_machine_frequency_, 10.0);

        ROS_INFO("任务文件夹路径: %s", task_folder_path_.c_str());
        ROS_INFO("目标点文件: %s", target_points_file_.c_str());
        ROS_INFO("状态机频率: %.1f Hz", state_machine_frequency_);
    }

    bool NavigationTaskStateMachine::loadAllTaskLists()
    {
        task_lists_map_.clear();
        
        DIR* dir = opendir(task_folder_path_.c_str());
        if (!dir)
        {
            ROS_ERROR("无法打开任务文件夹: %s", task_folder_path_.c_str());
            return false;
        }

        struct dirent* entry;
        int task_file_count = 0;
        
        while ((entry = readdir(dir)) != NULL)
        {
            std::string filename = entry->d_name;
            
            // 跳过 "." 和 ".." 以及隐藏文件
            if (filename[0] == '.')
                continue;
            
            // 只处理 .txt 文件
            if (filename.length() < 4 || filename.substr(filename.length() - 4) != ".txt")
                continue;
            
            std::string full_path = task_folder_path_ + "/" + filename;
            
            // 检查是否为普通文件
            struct stat st;
            if (stat(full_path.c_str(), &st) != 0 || !S_ISREG(st.st_mode))
                continue;
            
            // 从文件加载任务列表
            std::vector<TaskItem> task_list;
            if (loadTaskListFromFile(full_path, task_list))
            {
                // 去掉 .txt 后缀作为任务名称
                std::string task_name = filename.substr(0, filename.length() - 4);
                task_lists_map_[task_name] = task_list;
                task_file_count++;
                ROS_INFO("成功加载任务文件: %s (%zu 个任务)", task_name.c_str(), task_list.size());
            }
        }
        
        closedir(dir);
        
        if (task_lists_map_.empty())
        {
            ROS_ERROR("未找到任何有效的任务文件");
            return false;
        }
        
        ROS_INFO("成功加载 %d 个任务文件", task_file_count);
        return true;
    }

    bool NavigationTaskStateMachine::loadTaskListFromFile(const std::string& file_path, std::vector<TaskItem>& task_list)
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            ROS_WARN("无法打开任务列表文件: %s", file_path.c_str());
            return false;
        }

        task_list.clear();
        std::string line;
        int line_number = 0;

        while (std::getline(file, line))
        {
            line_number++;

            // 跳过注释行和空行
            if (line.empty() || line[0] == '#')
                continue;

            std::istringstream iss(line);
            TaskItem task;

            if (!(iss >> task.target_id >> task.wait_delay >> task.publish_data))
            {
                ROS_WARN("文件 %s 第 %d 行格式错误: %s", file_path.c_str(), line_number, line.c_str());
                continue;
            }

            task_list.push_back(task);
            ROS_DEBUG("加载任务: ID=%d, 延时=%d秒, 发布数据=%d",
                      task.target_id, task.wait_delay, task.publish_data);
        }

        file.close();

        if (task_list.empty())
        {
            ROS_WARN("任务列表文件 %s 为空", file_path.c_str());
            return false;
        }

        return true;
    }

    bool NavigationTaskStateMachine::loadTargetPoints()
    {
        std::ifstream file(target_points_file_);
        if (!file.is_open())
        {
            ROS_ERROR("无法打开目标点文件: %s", target_points_file_.c_str());
            return false;
        }

        target_points_map_.clear();
        std::string line;
        int line_number = 0;

        while (std::getline(file, line))
        {
            line_number++;

            // 跳过注释行和空行
            if (line.empty() || line[0] == '#')
                continue;

            std::istringstream iss(line);
            TargetPoint point;

            if (!(iss >> point.id >> point.x >> point.y >> point.z >>
                  point.qx >> point.qy >> point.qz >> point.qw >> point.label))
            {
                ROS_WARN("目标点文件第 %d 行格式错误: %s", line_number, line.c_str());
                continue;
            }

            target_points_map_[point.id] = point;
            ROS_DEBUG("加载目标点: ID=%d, 位置=(%.3f,%.3f,%.3f), 标签=%s",
                      point.id, point.x, point.y, point.z, point.label.c_str());
        }

        file.close();

        if (target_points_map_.empty())
        {
            ROS_ERROR("目标点列表为空");
            return false;
        }

        ROS_INFO("成功加载 %zu 个目标点", target_points_map_.size());
        return true;
    }

    bool NavigationTaskStateMachine::validateTaskList()
    {
        bool validation_passed = true;

        for (const auto& task_pair : task_lists_map_)
        {
            const std::string& task_name = task_pair.first;
            const std::vector<TaskItem>& task_list = task_pair.second;
            
            for (const auto &task : task_list)
            {
                if (target_points_map_.find(task.target_id) == target_points_map_.end())
                {
                    ROS_ERROR("任务 %s 中的目标点ID %d 在目标点文件中不存在", 
                             task_name.c_str(), task.target_id);
                    validation_passed = false;
                }
            }
        }

        if (validation_passed)
        {
            ROS_INFO("所有任务列表校验通过");
        }
        else
        {
            ROS_ERROR("任务列表校验失败，请检查任务列表中的目标点ID");
        }

        return validation_passed;
    }

    void NavigationTaskStateMachine::navStateCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        nav_state_ = msg->data;
        nav_state_received_ = true;

        ROS_DEBUG("收到导航状态: %s", nav_state_ ? "已到达" : "未到达");
    }

    void NavigationTaskStateMachine::navTaskSignalCallback(const std_msgs::String::ConstPtr &msg)
    {
        std::string signal = msg->data;
        
        ROS_INFO("收到任务信号: %s", signal.c_str());
        
        // 处理控制命令
        if (signal == "STOP")
        {
            ROS_INFO("停止任务");
            if (current_state_ != NavigationTaskState::IDLE)
            {
                previous_state_ = current_state_;
                current_state_ = NavigationTaskState::STOPPED;
                logStateTransition(previous_state_, current_state_);
            }
            return;
        }
        else if (signal == "CONTINUE")
        {
            ROS_INFO("继续执行任务");
            if (current_state_ == NavigationTaskState::STOPPED)
            {
                // 如果当前任务的目标点还没发布，需要重置标志
                if (!nav_state_received_)
                {
                    target_point_published_ = false;
                }
                previous_state_ = current_state_;
                current_state_ = NavigationTaskState::RUNNING;
                logStateTransition(previous_state_, current_state_);
            }
            return;
        }
        else if (signal == "GO_HOME")
        {
            ROS_INFO("回家");
            previous_state_ = current_state_;
            current_state_ = NavigationTaskState::GOING_HOME;
            logStateTransition(previous_state_, current_state_);
            // 立即发布回家目标点 (0, 0, 0)
            publishHomeTargetPoint();
            return;
        }
        
        // 否则，将信号作为任务名称处理
        auto it = task_lists_map_.find(signal);
        if (it != task_lists_map_.end())
        {
            ROS_INFO("开始执行任务: %s (%zu 个任务点)", signal.c_str(), it->second.size());
            
            if (current_state_ == NavigationTaskState::IDLE ||
                current_state_ == NavigationTaskState::STOPPED)
            {
                current_task_name_ = signal;
                current_task_list_ = it->second;
                current_task_index_ = 0;
                nav_state_received_ = false;
                target_point_published_ = false;
                previous_state_ = current_state_;
                current_state_ = NavigationTaskState::RUNNING;
                logStateTransition(previous_state_, current_state_);
            }
            else
            {
                ROS_WARN("当前状态不允许开始新任务，请先停止当前任务");
            }
        }
        else
        {
            ROS_WARN("未找到任务: %s", signal.c_str());
            ROS_INFO("可用的任务有:");
            for (const auto& task_pair : task_lists_map_)
            {
                ROS_INFO("  - %s", task_pair.first.c_str());
            }
        }
    }

    void NavigationTaskStateMachine::stateMachineTimerCallback(const ros::TimerEvent &event)
    {
        processStateMachine();
    }

    void NavigationTaskStateMachine::processStateMachine()
    {
        switch (current_state_)
        {
        case NavigationTaskState::IDLE:
            // 空闲状态，等待启动信号
            break;

        case NavigationTaskState::RUNNING:
        {
            if (current_task_index_ >= static_cast<int>(current_task_list_.size()))
            {
                // 所有任务完成，回到空闲状态
                ROS_INFO("任务 %s 完成，回到空闲状态", current_task_name_.c_str());
                resetStateMachine();
                break;
            }

            const TaskItem &current_task = getCurrentTask();

            if (!nav_state_received_)
            {
                // 还没有收到导航状态，只在第一次发布目标点
                if (!target_point_published_)
                {
                    publishTargetPoint(current_task.target_id);
                    target_point_published_ = true;
                    ROS_INFO("发布目标点 ID=%d, 标签=%s",
                             current_task.target_id,
                             target_points_map_[current_task.target_id].label.c_str());
                }
                else
                {
                    // 目标点已发布，等待导航状态反馈
                    ROS_DEBUG_THROTTLE(5.0, "等待导航到目标点 ID=%d", current_task.target_id);
                }
            }
            else if (nav_state_)
            {
                // 已到达目标点，发布任务动作并开始等待
                publishTaskAction(current_task.publish_data);
                ROS_INFO("到达目标点 ID=%d, 发布动作数据=%d",
                         current_task.target_id, current_task.publish_data);

                if (current_task.wait_delay > 0)
                {
                    // 需要等待，切换到等待状态
                    current_wait_delay_ = current_task.wait_delay;
                    wait_start_time_ = ros::Time::now();
                    previous_state_ = current_state_;
                    current_state_ = NavigationTaskState::WAITING;
                    logStateTransition(previous_state_, current_state_);
                    ROS_INFO("开始等待 %d 秒", current_wait_delay_);
                }
                else
                {
                    // 不需要等待，直接移动到下一个任务
                    moveToNextTask();
                }
            }
        }
        break;

        case NavigationTaskState::WAITING:
        {
            // 检查等待时间是否结束
            ros::Duration elapsed = ros::Time::now() - wait_start_time_;
            if (elapsed.toSec() >= current_wait_delay_)
            {
                ROS_INFO("等待时间结束，继续下一个任务");
                moveToNextTask();
                previous_state_ = current_state_;
                current_state_ = NavigationTaskState::RUNNING;
                logStateTransition(previous_state_, current_state_);
            }
            else
            {
                ROS_DEBUG("等待中... 剩余时间: %.1f 秒",
                          current_wait_delay_ - elapsed.toSec());
            }
        }
        break;

        case NavigationTaskState::STOPPED:
            // 停止状态，等待继续或其他信号
            break;

        case NavigationTaskState::GOING_HOME:
        {
            if (nav_state_received_ && nav_state_)
            {
                // 已到达家位置，重置状态机
                ROS_INFO("已到达家位置，重置状态机");
                resetStateMachine();
            }
        }
        break;
        }
    }

    void NavigationTaskStateMachine::publishTargetPoint(int target_id)
    {
        geometry_msgs::Pose target_msg;

        auto it = target_points_map_.find(target_id);
        if (it != target_points_map_.end())
        {
            const TargetPoint &point = it->second;
            target_msg.position.x = point.x;
            target_msg.position.y = point.y;
            target_msg.position.z = point.z;
            target_msg.orientation.x = point.qx;
            target_msg.orientation.y = point.qy;
            target_msg.orientation.z = point.qz;
            target_msg.orientation.w = point.qw;

            ROS_INFO("发布目标点: ID=%d, 位置=(%.3f,%.3f,%.3f), 标签=%s",
                     target_id, point.x, point.y, point.z, point.label.c_str());
        }
        else
        {
            ROS_ERROR("目标点ID %d 不存在", target_id);
            return;
        }

        target_points_pub_.publish(target_msg);
    }

    void NavigationTaskStateMachine::publishHomeTargetPoint()
    {
        geometry_msgs::Pose target_msg;

        // 回家点 (0, 0, 0)
        target_msg.position.x = 0.0;
        target_msg.position.y = 0.0;
        target_msg.position.z = 0.0;
        target_msg.orientation.x = 0.0;
        target_msg.orientation.y = 0.0;
        target_msg.orientation.z = 0.0;
        target_msg.orientation.w = 1.0;

        target_points_pub_.publish(target_msg);
        ROS_INFO("发布回家目标点: (0, 0, 0)");
    }

    void NavigationTaskStateMachine::publishTaskAction(int action_data)
    {
        std_msgs::Int32 msg;
        msg.data = action_data;
        task_action_pub_.publish(msg);

        ROS_DEBUG("发布任务动作: %d", action_data);
    }

    void NavigationTaskStateMachine::resetStateMachine()
    {
        previous_state_ = current_state_;
        current_state_ = NavigationTaskState::IDLE;
        current_task_index_ = 0;
        nav_state_received_ = false;
        nav_state_ = false;
        target_point_published_ = false;
        current_wait_delay_ = 0;

        logStateTransition(previous_state_, current_state_);
        ROS_INFO("状态机已重置到空闲状态");
    }

    void NavigationTaskStateMachine::moveToNextTask()
    {
        current_task_index_++;
        nav_state_received_ = false;
        nav_state_ = false;
        target_point_published_ = false;

        if (current_task_index_ >= static_cast<int>(current_task_list_.size()))
        {
            ROS_INFO("任务 %s 的所有任务点已完成", current_task_name_.c_str());
        }
        else
        {
            ROS_INFO("移动到下一个任务点，索引: %d/%zu", current_task_index_, current_task_list_.size());
        }
    }

    bool NavigationTaskStateMachine::isLastTask() const
    {
        return current_task_index_ >= static_cast<int>(current_task_list_.size()) - 1;
    }

    const TaskItem &NavigationTaskStateMachine::getCurrentTask() const
    {
        static TaskItem default_task;
        if (current_task_index_ >= 0 && current_task_index_ < static_cast<int>(current_task_list_.size()))
        {
            return current_task_list_[current_task_index_];
        }
        return default_task;
    }

    void NavigationTaskStateMachine::logStateTransition(NavigationTaskState from_state, NavigationTaskState to_state)
    {
        ROS_INFO("状态转换: %s -> %s",
                 stateToString(from_state).c_str(),
                 stateToString(to_state).c_str());
    }

    std::string NavigationTaskStateMachine::stateToString(NavigationTaskState state) const
    {
        switch (state)
        {
        case NavigationTaskState::IDLE:
            return "IDLE";
        case NavigationTaskState::RUNNING:
            return "RUNNING";
        case NavigationTaskState::WAITING:
            return "WAITING";
        case NavigationTaskState::STOPPED:
            return "STOPPED";
        case NavigationTaskState::GOING_HOME:
            return "GOING_HOME";
        default:
            return "UNKNOWN";
        }
    }

    void NavigationTaskStateMachine::run()
    {
        ROS_INFO("导航任务状态机开始运行");
        ros::spin();
    }

} // namespace user_interface