#include <ros/ros.h>
#include "user_interface/navigation_task_state_machine.h"
#include <signal.h>

/**
 * @brief 信号处理函数
 */
void signalHandler(int signum)
{
    ROS_INFO("收到信号 %d，正在关闭导航任务状态机节点...", signum);
    ros::shutdown();
}

/**
 * @brief 导航任务状态机节点主函数
 */
int main(int argc, char** argv)
{
    // 设置中文编码支持
    setlocale(LC_ALL, "");
    
    // 初始化ROS节点
    ros::init(argc, argv, "navigation_task_state_machine_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 注册信号处理函数
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    ROS_INFO("启动导航任务状态机节点");
    
    try
    {
        // 创建状态机对象
        user_interface::NavigationTaskStateMachine state_machine;
        
        // 初始化状态机
        if (!state_machine.initialize(private_nh))
        {
            ROS_ERROR("导航任务状态机初始化失败");
            return -1;
        }
        
        ROS_INFO("导航任务状态机节点初始化完成");
        ROS_INFO("等待导航任务信号...");
        ROS_INFO("任务信号说明:");
        ROS_INFO("  1 - 开始循环导航");
        ROS_INFO("  2 - 停止任务");
        ROS_INFO("  3 - 继续执行任务");
        ROS_INFO("  4 - 回家");
        
        // 运行状态机
        state_machine.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("导航任务状态机节点运行异常: %s", e.what());
        return -1;
    }
    
    ROS_INFO("导航任务状态机节点正常退出");
    return 0;
}