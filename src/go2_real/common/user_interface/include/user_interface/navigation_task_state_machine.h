#ifndef NAVIGATION_TASK_STATE_MACHINE_H
#define NAVIGATION_TASK_STATE_MACHINE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>

namespace user_interface
{

/**
 * @brief 导航任务状态枚举
 */
enum class NavigationTaskState
{
    IDLE,           // 空闲状态
    RUNNING,        // 运行状态
    WAITING,        // 等待状态（到达目标点后的延时等待）
    STOPPED,        // 停止状态
    GOING_HOME      // 回家状态
};

/**
 * @brief 导航任务信号枚举
 */
enum class NavigationTaskSignal
{
    START_LOOP = 1,    // 开始循环导航
    STOP = 2,          // 停止任务
    CONTINUE = 3,      // 继续执行任务
    GO_HOME = 4        // 回家
};

/**
 * @brief 任务项结构体
 */
struct TaskItem
{
    int target_id;           // 目标点ID
    int wait_delay;          // 等待延时（秒）
    int publish_data;        // 抵达后发布的数据
    
    TaskItem() : target_id(0), wait_delay(0), publish_data(0) {}
    TaskItem(int id, int delay, int data) : target_id(id), wait_delay(delay), publish_data(data) {}
};

/**
 * @brief 目标点结构体
 */
struct TargetPoint
{
    int id;                    // 目标点ID
    double x, y, z;           // 位置坐标
    double qx, qy, qz, qw;    // 四元数朝向
    std::string label;        // 标签
    
    TargetPoint() : id(0), x(0), y(0), z(0), qx(0), qy(0), qz(0), qw(1) {}
};

/**
 * @brief 导航任务状态机类
 */
class NavigationTaskStateMachine
{
public:
    /**
     * @brief 构造函数
     */
    NavigationTaskStateMachine();

    /**
     * @brief 析构函数
     */
    ~NavigationTaskStateMachine();

    /**
     * @brief 初始化状态机
     * @param nh ROS节点句柄
     * @return 成功返回true，失败返回false
     */
    bool initialize(ros::NodeHandle& nh);

    /**
     * @brief 运行状态机主循环
     */
    void run();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber nav_state_sub_;        // 导航状态订阅器
    ros::Subscriber nav_task_signal_sub_;  // 导航任务信号订阅器
    ros::Publisher task_action_pub_;       // 任务动作发布器
    ros::Publisher target_points_pub_;     // 目标点发布器
    ros::Timer state_machine_timer_;       // 状态机定时器

    // 状态机相关
    NavigationTaskState current_state_;    // 当前状态
    NavigationTaskState previous_state_;   // 前一个状态
    
    // 任务相关
    std::map<std::string, std::vector<TaskItem>> task_lists_map_;  // 任务列表映射表（文件名->任务列表）
    std::vector<TaskItem> current_task_list_;  // 当前使用的任务列表
    std::string current_task_name_;        // 当前任务名称
    std::map<int, TargetPoint> target_points_map_;  // 目标点映射表
    int current_task_index_;               // 当前任务索引
    ros::Time wait_start_time_;            // 等待开始时间
    int current_wait_delay_;               // 当前等待延时
    
    // 导航状态
    bool nav_state_;                       // 导航状态（是否到达目标点）
    bool nav_state_received_;              // 是否接收到导航状态
    bool target_point_published_;          // 当前任务的目标点是否已发布
    NavigationTaskSignal last_task_signal_; // 最后一个任务信号
    
    // 配置文件路径
    std::string task_folder_path_;         // 任务文件夹路径
    std::string target_points_file_;       // 目标点文件路径
    
    // 控制参数
    double state_machine_frequency_;       // 状态机运行频率

    /**
     * @brief 加载参数
     */
    void loadParameters();

    /**
     * @brief 从文件夹加载所有任务列表
     * @return 成功返回true，失败返回false
     */
    bool loadAllTaskLists();
    
    /**
     * @brief 从单个文件加载任务列表
     * @param file_path 任务文件路径
     * @param task_list 输出的任务列表
     * @return 成功返回true，失败返回false
     */
    bool loadTaskListFromFile(const std::string& file_path, std::vector<TaskItem>& task_list);

    /**
     * @brief 从文件加载目标点
     * @return 成功返回true，失败返回false
     */
    bool loadTargetPoints();

    /**
     * @brief 校验任务列表中的目标点ID是否都存在
     * @return 校验通过返回true，失败返回false
     */
    bool validateTaskList();

    /**
     * @brief 导航状态回调函数
     * @param msg 导航状态消息
     */
    void navStateCallback(const std_msgs::Bool::ConstPtr& msg);

    /**
     * @brief 导航任务信号回调函数
     * @param msg 导航任务信号消息（字符串格式）
     */
    void navTaskSignalCallback(const std_msgs::String::ConstPtr& msg);

    /**
     * @brief 状态机定时器回调函数
     * @param event 定时器事件
     */
    void stateMachineTimerCallback(const ros::TimerEvent& event);

    /**
     * @brief 处理状态机逻辑
     */
    void processStateMachine();

    /**
     * @brief 发布目标点
     * @param target_id 目标点ID
     */
    void publishTargetPoint(int target_id);

    /**
     * @brief 发布回家目标点 (0, 0, 0)
     */
    void publishHomeTargetPoint();

    /**
     * @brief 发布任务动作
     * @param action_data 动作数据
     */
    void publishTaskAction(int action_data);

    /**
     * @brief 重置状态机到空闲状态
     */
    void resetStateMachine();

    /**
     * @brief 切换到下一个任务
     */
    void moveToNextTask();

    /**
     * @brief 检查是否为最后一个任务
     * @return 是最后一个任务返回true，否则返回false
     */
    bool isLastTask() const;

    /**
     * @brief 获取当前任务项
     * @return 当前任务项的引用，如果索引无效则返回默认任务项
     */
    const TaskItem& getCurrentTask() const;

    /**
     * @brief 状态转换日志输出
     * @param from_state 从哪个状态
     * @param to_state 到哪个状态
     */
    void logStateTransition(NavigationTaskState from_state, NavigationTaskState to_state);

    /**
     * @brief 状态枚举转字符串
     * @param state 状态枚举值
     * @return 状态字符串
     */
    std::string stateToString(NavigationTaskState state) const;
};

} // namespace user_interface

#endif // NAVIGATION_TASK_STATE_MACHINE_H