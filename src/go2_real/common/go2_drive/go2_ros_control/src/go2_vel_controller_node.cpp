#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>  // 添加TwistStamped头文件
#include <std_msgs/Int32.h>  // 添加Int32消息头文件
#include <std_msgs/Bool.h>   // 添加Bool消息头文件
#include <unitree_go/WirelessController.h>  // 添加遥控器消息头文件
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>  // 添加遥控器DDS消息头文件
#include <state_machine_msg/keywords.h>  // 添加keywords消息头文件
#include <memory>
#include <string>
#include <mutex>
#include <cmath>
#include <chrono>  // 添加时间相关头文件

#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"

// 定义这个宏以使用TwistStamped，注释掉则使用Twist
//#define USE_TWIST_STAMPED

// 根据宏定义选择消息类型
#ifdef USE_TWIST_STAMPED
    #define VELOCITY_MSG_TYPE geometry_msgs::TwistStamped
    #define VELOCITY_DATA(msg) msg->twist
    #define TOPIC_CMD_VEL "/cmd_vel"  // TwistStamped的主题
#else
    #define VELOCITY_MSG_TYPE geometry_msgs::Twist
    #define VELOCITY_DATA(msg) (*msg)
    #define TOPIC_CMD_VEL "/cmd_vel"  // Twist的主题
#endif

// 遥控器按键联合体
typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

// 单个按键类
class Button
{
public:
    Button() : pressed(false), on_press(false), on_release(false), 
               last_press_time(0), debounce_delay(300) {}  // 默认300ms防抖

    void update(bool state)
    {
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_epoch = current_time.time_since_epoch();
        auto current_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch).count();
        
        // 检测按键按下事件
        bool new_press = state && !pressed;
        
        // 防抖处理：只有在防抖延迟时间后才允许新的按下事件
        if (new_press && (current_time_ms - last_press_time) >= debounce_delay)
        {
            on_press = true;
            last_press_time = current_time_ms;
        }
        else
        {
            on_press = false;
        }
        
        // 按键释放事件不需要防抖
        on_release = !state && pressed;
        pressed = state;
    }
    
    // 设置防抖延迟时间（毫秒）
    void setDebounceDelay(int delay_ms)
    {
        debounce_delay = delay_ms;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
    
private:
    long long last_press_time;  // 上次按下的时间戳（毫秒）
    int debounce_delay;         // 防抖延迟时间（毫秒）
};

// 游戏手柄类
class Gamepad
{
public:
    Gamepad() 
    {
        // 为不同类型的按键设置不同的防抖延迟
        // 功能按键设置较长的防抖延迟
        start.setDebounceDelay(500);    // START按键500ms防抖
        select.setDebounceDelay(500);   // SELECT按键500ms防抖
        
        // 肩部按键设置中等防抖延迟
        L1.setDebounceDelay(300);
        R1.setDebounceDelay(300);
        L2.setDebounceDelay(300);
        R2.setDebounceDelay(300);
        
        // 面部按键设置较短防抖延迟
        A.setDebounceDelay(200);
        B.setDebounceDelay(200);
        X.setDebounceDelay(200);
        Y.setDebounceDelay(200);
        
        // 方向键设置较短防抖延迟
        up.setDebounceDelay(200);
        down.setDebounceDelay(200);
        left.setDebounceDelay(200);
        right.setDebounceDelay(200);
        
        // F1, F2功能键设置较长防抖延迟
        F1.setDebounceDelay(400);
        F2.setDebounceDelay(400);
    }

    void Update(unitree_go::msg::dds_::WirelessController_ &key_msg)
    {
        // 更新摇杆值，带平滑和死区处理
        lx = lx * (1 - smooth) + (std::fabs(key_msg.lx()) < dead_zone ? 0.0 : key_msg.lx()) * smooth;
        rx = rx * (1 - smooth) + (std::fabs(key_msg.rx()) < dead_zone ? 0.0 : key_msg.rx()) * smooth;
        ry = ry * (1 - smooth) + (std::fabs(key_msg.ry()) < dead_zone ? 0.0 : key_msg.ry()) * smooth;
        ly = ly * (1 - smooth) + (std::fabs(key_msg.ly()) < dead_zone ? 0.0 : key_msg.ly()) * smooth;

        // 更新按键状态
        key.value = key_msg.keys();

        R1.update(key.components.R1);
        L1.update(key.components.L1);
        start.update(key.components.start);
        select.update(key.components.select);
        R2.update(key.components.R2);
        L2.update(key.components.L2);
        F1.update(key.components.F1);
        F2.update(key.components.F2);
        A.update(key.components.A);
        B.update(key.components.B);
        X.update(key.components.X);
        Y.update(key.components.Y);
        up.update(key.components.up);
        right.update(key.components.right);
        down.update(key.components.down);
        left.update(key.components.left);
    }

    float smooth = 0.2;      // 平滑参数
    float dead_zone = 0.05;  // 死区参数

    float lx = 0.;
    float rx = 0.;
    float ry = 0.;
    float ly = 0.;

    Button R1, L1, start, select, R2, L2, F1, F2;
    Button A, B, X, Y, up, right, down, left;

private:
    xKeySwitchUnion key;
};

class Go2VelController
{
public:
    Go2VelController(ros::NodeHandle &nh, const std::string &network_interface)
        : nh_(nh), sport_client_(), cmd_vel_enabled_(true),  // 默认启用cmd_vel控制
          last_publish_time_(std::chrono::steady_clock::now()),  // 初始化发布时间
          filtered_vx_(0.0), filtered_vy_(0.0), filtered_vyaw_(0.0),  // 初始化滤波器状态
          filter_alpha_(0.3)  // 默认滤波器参数，可通过参数调整
    {
        try
        {
            ROS_INFO("Initializing Go2VelController...");
            
            // 读取滤波器参数
            ros::NodeHandle private_nh("~");
            private_nh.param("velocity_filter_alpha", filter_alpha_, 0.3);
            ROS_INFO("Velocity filter alpha: %.3f", filter_alpha_);

            // 初始化Go2 SDK
            sport_client_.SetTimeout(10.0f);
            sport_client_.Init();
            ROS_INFO("Go2 SDK initialized successfully.");

            // 初始化状态订阅
            suber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
            suber_->InitChannel(std::bind(&Go2VelController::stateCallback, this, std::placeholders::_1), 1);
            ROS_INFO("State subscriber initialized.");

            // 初始化遥控器订阅
            joystick_suber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
            joystick_suber_->InitChannel(std::bind(&Go2VelController::joystickCallback, this, std::placeholders::_1), 1);
            ROS_INFO("Joystick subscriber initialized.");

            // 等待1秒以获取稳定的状态
            ros::Duration(1.0).sleep();

            // 初始化ROS订阅器 - 使用宏定义的主题名称
            cmd_vel_sub_ = nh_.subscribe(TOPIC_CMD_VEL, 1, &Go2VelController::cmdVelCallback, this);
            ROS_INFO("ROS subscriber initialized on topic: %s", TOPIC_CMD_VEL);
            
            // 初始化紧急停止订阅器
            emergency_stop_sub_ = nh_.subscribe("/emergency_stop", 1, &Go2VelController::emergencyStopCallback, this);
            ROS_INFO("Emergency stop subscriber initialized on topic: /emergency_stop");
            
            // 初始化ROS发布器
            wireless_controller_pub_ = nh_.advertise<unitree_go::WirelessController>("/wireless_controller", 10);
            ROS_INFO("Wireless controller publisher initialized on topic: /wireless_controller");
            
            // 初始化cmd_vel状态发布器
            go2_control_status_pub_ = nh_.advertise<std_msgs::Int32>("/go2_control_status", 10);
            ROS_INFO("Cmd_vel status publisher initialized on topic: /go2_control_status");
            

            // 确保机器狗处于站立状态
            //sport_client_.SwitchGait(0); // 0:idle; 1:tort; 2:tort running; 3:climb stair; 4:tort obstacle
            sport_client_.StandUp();
            ros::Duration(3.0).sleep();

            sport_client_.BalanceStand();
            ros::Duration(1.0).sleep();


            ROS_INFO("Robot is now standing.");

            // 显示当前使用的消息类型
            #ifdef USE_TWIST_STAMPED
                ROS_INFO("Using TwistStamped messages for velocity control");
            #else
                ROS_INFO("Using Twist messages for velocity control");
            #endif

            // 发布初始cmd_vel状态（默认启用），导航模式
            publishCmdVelStatus(0);

            ROS_INFO("Go2 velocity controller initialized successfully.");
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Initialization error: %s", e.what());
            throw;
        }
    }

    ~Go2VelController()
    {
        try
        {
            sport_client_.StopMove();
            // sport_client_.SwitchGait(0);
            // sport_client_.StandDown();
        }
        catch (const std::exception &e)
        {
            ROS_WARN("Error during shutdown: %s", e.what());
        }
    }

    void cmdVelCallback(const typename VELOCITY_MSG_TYPE::ConstPtr &msg)
    {
        try
        {
            // 如果cmd_vel被禁用，则忽略所有速度命令
            if (!cmd_vel_enabled_) {
                return;
            }
            
            // 从消息中提取线速度和角速度 - 使用宏定义访问数据
            double vx = VELOCITY_DATA(msg).linear.x;    // 前进速度
            double vy = VELOCITY_DATA(msg).linear.y;    // 侧向速度
            double vyaw = VELOCITY_DATA(msg).angular.z; // 旋转速度

            #ifdef USE_TWIST_STAMPED
                ROS_DEBUG("Received TwistStamped with timestamp: %.6f", msg->header.stamp.toSec());
            #endif

            // 速度限制
            vx = std::max(-1.0, std::min(1.0, vx));
            vy = std::max(-0.3, std::min(0.3, vy));
            vyaw = std::max(-0.8, std::min(0.8, vyaw));

            // 低通滤波器 - 指数移动平均
            // filtered = alpha * new_value + (1 - alpha) * old_filtered
            filtered_vx_ = filter_alpha_ * vx + (1.0 - filter_alpha_) * filtered_vx_;
            filtered_vy_ = filter_alpha_ * vy + (1.0 - filter_alpha_) * filtered_vy_;
            filtered_vyaw_ = filter_alpha_ * vyaw + (1.0 - filter_alpha_) * filtered_vyaw_;

            // 使用Go2 SDK控制机器狗移动
            // 如果滤波后的速度都很小,则跳过控制
            const double speed_threshold = 0.01;
            if (std::abs(filtered_vx_) < speed_threshold && 
                std::abs(filtered_vy_) < speed_threshold && 
                std::abs(filtered_vyaw_) < speed_threshold) {
                return;
            }
            
            sport_client_.Move(filtered_vx_, filtered_vy_, filtered_vyaw_);
            
            ROS_DEBUG("Velocity - Raw: [%.3f, %.3f, %.3f], Filtered: [%.3f, %.3f, %.3f]", 
                     vx, vy, vyaw, filtered_vx_, filtered_vy_, filtered_vyaw_);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error in cmdVelCallback: %s", e.what());
        }
    }

    void emergencyStopCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        try
        {
            bool emergency_stop = msg->data;
            
            if (emergency_stop)
            {
                // 紧急停止激活：切换到遥控模式
                ROS_WARN("Emergency stop ACTIVATED - switching to remote control mode");
                cmd_vel_enabled_ = false;  // 禁用cmd_vel控制
                sport_client_.StopMove();  // 立即停止机器人
                resetVelocityFilter();     // 重置滤波器状态
                publishCmdVelStatus(1);    // 发布遥控模式状态
            }
            else
            {
                // 紧急停止解除：切换到导航模式
                ROS_INFO("Emergency stop DEACTIVATED - switching to navigation mode");
                cmd_vel_enabled_ = true;   // 启用cmd_vel控制
                publishCmdVelStatus(0);    // 发布导航模式状态
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error in emergencyStopCallback: %s", e.what());
        }
    }

    void stateCallback(const void *message)
    {
        try
        {
            state_ = *(unitree_go::msg::dds_::LowState_ *)message;
            // ROS_DEBUG("Position: [%.2f, %.2f, %.2f]",
            //           state_.position()[0], state_.position()[1], state_.position()[2]);
            // ROS_DEBUG("IMU RPY: [%.2f, %.2f, %.2f]",
            //           state_.imu_state().rpy()[0], state_.imu_state().rpy()[1], state_.imu_state().rpy()[2]);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error in stateCallback: %s", e.what());
        }
    }

    void joystickCallback(const void *message)
    {
        try
        {
            std::lock_guard<std::mutex> lock(joystick_mutex_);
            joystick_msg_ = *(unitree_go::msg::dds_::WirelessController_ *)message;
            
            // 更新游戏手柄状态
            gamepad_.Update(joystick_msg_);
            
            // 发布ROS消息
            publishWirelessController();
            
            // 处理特殊按键（可选功能）
            handleGamepadCommands();
            
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error in joystickCallback: %s", e.what());
        }
    }

    void publishWirelessController()
    {
        // 防止过于频繁地发布消息（限制发布频率）
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_publish = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_publish_time_).count();
        
        // 限制发布频率为最多每50ms一次（20Hz）
        const int publish_interval_ms = 50;
        
        // 检查是否有按键按下事件或者达到了发布间隔
        bool has_key_event = gamepad_.start.on_press || gamepad_.select.on_press ||
                            gamepad_.L1.on_press || gamepad_.R1.on_press ||
                            gamepad_.L2.on_press || gamepad_.R2.on_press ||
                            gamepad_.A.on_press || gamepad_.B.on_press ||
                            gamepad_.X.on_press || gamepad_.Y.on_press ||
                            gamepad_.up.on_press || gamepad_.down.on_press ||
                            gamepad_.left.on_press || gamepad_.right.on_press ||
                            gamepad_.F1.on_press || gamepad_.F2.on_press;
        
        // 只有在有按键事件或达到发布间隔时才发布消息
        if (has_key_event || time_since_last_publish >= publish_interval_ms)
        {
            unitree_go::WirelessController ros_msg;
            
            // 填充摇杆数据
            ros_msg.lx = gamepad_.lx;
            ros_msg.ly = gamepad_.ly;
            ros_msg.rx = gamepad_.rx;
            ros_msg.ry = gamepad_.ry;
            
            // 填充按键数据
            ros_msg.keys = joystick_msg_.keys();
            
            // 发布消息
            wireless_controller_pub_.publish(ros_msg);
            
            // 更新最后发布时间
            last_publish_time_ = current_time;
            
            // 如果有按键事件，记录日志
            if (has_key_event)
            {
                ROS_DEBUG("Published wireless controller data with key event - keys:%d", ros_msg.keys);
            }
            else
            {
                ROS_DEBUG("Published wireless controller data - lx:%.2f, ly:%.2f, rx:%.2f, ry:%.2f, keys:%d", 
                          ros_msg.lx, ros_msg.ly, ros_msg.rx, ros_msg.ry, ros_msg.keys);
            }
        }
    }

    void handleGamepadCommands()
    {
        // 示例：使用START按键切换cmd_vel控制状态
        if (gamepad_.start.on_press)
        {
            cmd_vel_enabled_ = !cmd_vel_enabled_;
            if (cmd_vel_enabled_)
            {
                ROS_INFO("Gamepad: cmd_vel control ENABLED");
                publishCmdVelStatus(0);  // 启用时发布0，导航模式
            }
            else
            {
                sport_client_.StopMove();
                resetVelocityFilter();  // 重置滤波器状态
                ROS_INFO("Gamepad: cmd_vel control DISABLED and robot stopped");
                publishCmdVelStatus(1);  // 禁用时发布1,遥控模式
            }
        }
        
        
        // 示例：使用L1+R1组合键紧急停止
        if (gamepad_.L1.pressed && gamepad_.R1.pressed)
        {
            sport_client_.StopMove();
            resetVelocityFilter();  // 重置滤波器状态
            cmd_vel_enabled_ = false;
            ROS_WARN("Gamepad: EMERGENCY STOP - L1+R1 pressed");
        }

    }

    void resetVelocityFilter()
    {
        filtered_vx_ = 0.0;
        filtered_vy_ = 0.0;
        filtered_vyaw_ = 0.0;
        ROS_DEBUG("Velocity filter reset to zero");
    }

    void publishCmdVelStatus(int status)
    {
        std_msgs::Int32 status_msg;
        status_msg.data = status;
        go2_control_status_pub_.publish(status_msg);
        ROS_DEBUG("Published cmd_vel status: %d", status);
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber emergency_stop_sub_;  // 紧急停止订阅器
    ros::Subscriber keywords_sub_;  // 关键词命令订阅器
    ros::Publisher wireless_controller_pub_;  // 遥控器数据发布器
    ros::Publisher go2_control_status_pub_;  // cmd_vel控制状态发布器
    
    unitree::robot::go2::SportClient sport_client_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> suber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_suber_;
    
    unitree_go::msg::dds_::LowState_ state_;
    unitree_go::msg::dds_::WirelessController_ joystick_msg_;
    
    Gamepad gamepad_;
    std::mutex joystick_mutex_;
    
    bool cmd_vel_enabled_;  // 控制cmd_vel是否生效的标志
    std::chrono::steady_clock::time_point last_publish_time_;  // 上次发布消息的时间
    
    // 速度滤波器状态变量
    double filtered_vx_;   // 滤波后的前进速度
    double filtered_vy_;   // 滤波后的侧向速度
    double filtered_vyaw_; // 滤波后的旋转速度
    double filter_alpha_;  // 低通滤波器参数 (0-1, 越小越平滑)
};

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "go2_vel_controller");
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // 从ROS参数服务器获取网络接口，默认值为"enp46s0"
        std::string network_interface;
        private_nh.param<std::string>("interface", network_interface, "enp46s0");
        
        ROS_INFO("Use network interface: %s", network_interface.c_str());
        
        ROS_INFO("Initializing Channel Factory...");
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface.c_str());

        ROS_INFO("Creating Go2VelController...");
        Go2VelController controller(nh, network_interface);

        ROS_INFO("Starting ROS spin...");
        ros::spin();

        return 0;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Fatal error: %s", e.what());
        return -1;
    }
}
