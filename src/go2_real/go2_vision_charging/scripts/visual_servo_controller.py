#!/usr/bin/env python3
"""
视觉伺服控制器 - 修正版
功能：
1. 接收base_link坐标系下的充电底座位姿
2. 使用PID控制器计算速度指令
3. 实现厘米级精准对接
4. 检测稳定状态并锁定

关键修改：
- 直接使用base_link坐标系下的充电目标（无需坐标转换）
- 控制指令在机器人本体坐标系下
"""
import rospy
import tf.transformations as tf_trans
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

class VisualServoController:
    def __init__(self):
        rospy.init_node('visual_servo_controller', anonymous=True)

        # ================= 参数配置 =================
        # 几何偏移（已在transformer中处理，这里保留用于兼容性）
        self.offset_z = rospy.get_param('~geometry/offset_z', -1.00)
        self.offset_x = rospy.get_param('~geometry/offset_x', 0.0)
        self.offset_y = rospy.get_param('~geometry/offset_y', -0.616)
        self.target_dock_dist = rospy.get_param('~geometry/target_dock_dist', 0.0)
        
        # PID参数
        self.k_v_x = rospy.get_param('~pid/k_p_x', 0.5)
        self.k_v_y = rospy.get_param('~pid/k_p_y', 0.8)
        self.k_w_z = rospy.get_param('~pid/k_p_z', 1.0)
        self.k_i_x = rospy.get_param('~pid/k_i_x', 0.05)
        self.k_i_y = rospy.get_param('~pid/k_i_y', 0.05)
        self.k_i_z = rospy.get_param('~pid/k_i_z', 0.05)
        self.k_d_x = rospy.get_param('~pid/k_d_x', 0.1)
        self.k_d_y = rospy.get_param('~pid/k_d_y', 0.1)
        self.k_d_z = rospy.get_param('~pid/k_d_z', 0.1)
        
        # PID状态变量
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_yaw = 0.0
        self.integral_limit = 0.5
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_yaw = 0.0
        self.last_time = rospy.Time.now()

        # 速度限制
        self.max_v_x = rospy.get_param('~limits/max_v_x', 0.15)
        self.max_v_y = rospy.get_param('~limits/max_v_y', 0.10)
        self.max_w_z = rospy.get_param('~limits/max_w_z', 0.3)
        
        # 容差
        self.dist_tolerance = rospy.get_param('~tolerance/dist', 0.07)
        self.yaw_tolerance = rospy.get_param('~tolerance/yaw', 0.04)
        self.lat_tolerance = rospy.get_param('~tolerance/lateral', 0.1)

        # 超时保护
        self.last_aruco_time = None
        self.aruco_timeout = rospy.get_param('~timeout/aruco_lost', 2.0)

        # 速度滤波
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        self.filter_alpha = rospy.get_param('~filter/alpha', 0.3)

        # 稳定性判定
        self.stable_count = 0
        self.stable_threshold = 15
        self.is_locked = False

        # ================= 通信接口 =================
        # 🔧 关键修改：订阅base_link坐标系下的充电目标
        self.aruco_sub = rospy.Subscriber(
            "/charging_goal_base",  # 从transformer节点接收已转换的位姿
            PoseStamped, 
            self.callback
        )
        self.vel_pub = rospy.Publisher("/servo_cmd_vel", Twist, queue_size=10)
        self.lock_pub = rospy.Publisher("/docking_locked", Bool, queue_size=10)

        rospy.loginfo("=" * 60)
        rospy.loginfo("视觉伺服控制器已启动")
        rospy.loginfo(f"控制坐标系: base_link (机器人本体坐标系)")
        rospy.loginfo(f"PID参数: Kp=({self.k_v_x:.2f}, {self.k_v_y:.2f}, {self.k_w_z:.2f})")
        rospy.loginfo(f"速度限制: vx={self.max_v_x:.2f}m/s, vy={self.max_v_y:.2f}m/s, w={self.max_w_z:.2f}rad/s")
        rospy.loginfo(f"收敛容差: dist={self.dist_tolerance*100:.1f}cm, lat={self.lat_tolerance*100:.1f}cm, yaw={math.degrees(self.yaw_tolerance):.1f}°")
        rospy.loginfo("=" * 60)

    def callback(self, msg):
        """
        接收base_link坐标系下的充电底座位姿，计算控制指令
        
        输入坐标系定义（base_link）：
            +X: 机器人前方
            +Y: 机器人左侧
            +Z: 机器人上方
            
        msg.pose.position:
            x: 充电底座在机器人前方的距离（正值=前方，负值=后方）
            y: 充电底座在机器人左侧的距离（正值=左侧，负值=右侧）
            z: 充电底座在机器人上方的距离（通常用于验证，不控制）
        """
        # 如果已锁定，停止控制
        if self.is_locked:
            cmd = Twist()
            self.vel_pub.publish(cmd)
            self.lock_pub.publish(Bool(data=True))
            return
        
        # 更新最后接收时间
        self.last_aruco_time = rospy.Time.now()
        
        # ========== 提取位姿信息 ==========
        # 已经在base_link坐标系，直接使用
        dock_x = msg.pose.position.x  # 前后距离（正=前方）
        dock_y = msg.pose.position.y  # 左右偏移（正=左侧）
        dock_z = msg.pose.position.z  # 高度（用于验证）
        
        # ========== 计算控制误差 ==========
        # 前后距离误差（期望距离为target_dock_dist，通常为0）
        error_dist = dock_x - self.target_dock_dist  # 正值=需要前进
        
        # 左右偏移误差
        error_lat = dock_y  # 正值=充电口在左侧，需要向左移动
        
        # 姿态误差（从四元数提取yaw角）
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_trans.euler_from_quaternion(orientation_list)
        error_yaw = yaw + 1.5707 # 角度误差（正值=需要逆时针旋转）

        # ========== 时间和导数计算 ==========
        current_time = rospy.Time.now()
        dt = max((current_time - self.last_time).to_sec(), 0.001)  # 防止除零
        
        # 计算误差导数（用于D项）
        derivative_x = (error_dist - self.last_error_x) / dt
        derivative_y = (error_lat - self.last_error_y) / dt
        derivative_yaw = (error_yaw - self.last_error_yaw) / dt
        
        # 更新历史误差
        self.last_error_x = error_dist
        self.last_error_y = error_lat
        self.last_error_yaw = error_yaw
        self.last_time = current_time

        # ========== 积分累加（用于I项） ==========
        self.integral_x += error_dist * dt
        self.integral_y += error_lat * dt
        self.integral_yaw += error_yaw * dt
        
        # 积分抗饱和
        self.integral_x = np.clip(self.integral_x, -self.integral_limit, self.integral_limit)
        self.integral_y = np.clip(self.integral_y, -self.integral_limit, self.integral_limit)
        self.integral_yaw = np.clip(self.integral_yaw, -self.integral_limit, self.integral_limit)

        # ========== PID控制计算 ==========
        cmd = Twist()
        
        # 前后速度控制（X轴）
        if abs(error_dist) > self.dist_tolerance:
            cmd.linear.x = (self.k_v_x * error_dist + 
                           self.k_i_x * self.integral_x + 
                           self.k_d_x * derivative_x)
        else:
            cmd.linear.x = 0.0  # 距离已满足，停止前进
        
        # 左右速度控制（Y轴）
        # 注意符号：error_lat>0表示充电口在左侧，需要向左移动（+Y方向）
        # 但机器人左移对应负的Y速度指令，所以加负号
        cmd.linear.y = (-self.k_v_y * error_lat - 
                       self.k_i_y * self.integral_y - 
                       self.k_d_y * derivative_y)
        
        # 旋转速度控制（Z轴）
        cmd.angular.z = (-self.k_w_z * error_yaw - 
                        self.k_i_z * self.integral_yaw - 
                        self.k_d_z * derivative_yaw)

        # ========== 速度限幅 ==========
        cmd.linear.x = np.clip(cmd.linear.x, -self.max_v_x, self.max_v_x)
        cmd.linear.y = np.clip(cmd.linear.y, -self.max_v_y, self.max_v_y)
        cmd.angular.z = np.clip(cmd.angular.z, -self.max_w_z, self.max_w_z)

        # ========== 低通滤波（平滑速度指令） ==========
        cmd.linear.x = self.filter_alpha * cmd.linear.x + (1 - self.filter_alpha) * self.last_cmd_vx
        cmd.linear.y = self.filter_alpha * cmd.linear.y + (1 - self.filter_alpha) * self.last_cmd_vy
        cmd.angular.z = self.filter_alpha * cmd.angular.z + (1 - self.filter_alpha) * self.last_cmd_wz
        
        # 更新历史速度
        self.last_cmd_vx = cmd.linear.x
        self.last_cmd_vy = cmd.linear.y
        self.last_cmd_wz = cmd.angular.z

        # ========== 稳定性判定 ==========
        is_converged = (abs(error_dist) < self.dist_tolerance and  
                       abs(error_lat) < self.lat_tolerance and 
                       abs(error_yaw) < self.yaw_tolerance)
        
        if is_converged:
            self.stable_count += 1
        else:
            self.stable_count = 0
        
        # 连续稳定才锁定（避免误判）
        if self.stable_count >= self.stable_threshold:
            # 停止所有运动
            cmd.linear.x = 0
            cmd.linear.y = 0
            cmd.angular.z = 0
            
            # 清零PID状态
            self.integral_x = 0
            self.integral_y = 0
            self.integral_yaw = 0
            self.last_error_x = 0.0
            self.last_error_y = 0.0
            self.last_error_yaw = 0.0
            
            # 设置锁定标志
            self.is_locked = True
            self.lock_pub.publish(Bool(data=True))
            
            rospy.loginfo("=" * 60)
            rospy.loginfo(f"✓ 对接完成并锁定！")
            rospy.loginfo(f"  最终误差: dist={error_dist*100:.2f}cm, lat={error_lat*100:.2f}cm, yaw={math.degrees(error_yaw):.2f}°")
            rospy.loginfo(f"  稳定帧数: {self.stable_count}")
            rospy.loginfo("=" * 60)
        else:
            # 未锁定，发布未锁定信号
            self.lock_pub.publish(Bool(data=False))
            
            # 日志输出（限流0.5Hz）
            rospy.loginfo_throttle(0.5, 
                f"距离:{dock_x:.3f}m | 横偏:{error_lat:.3f}m | 角偏:{math.degrees(error_yaw):.1f}° | "
                f"稳定:{self.stable_count}/{self.stable_threshold} | "
                f"速度 vx:{cmd.linear.x:.2f} vy:{cmd.linear.y:.2f} w:{cmd.angular.z:.2f}")

        # 发布速度指令
        self.vel_pub.publish(cmd)

if __name__ == '__main__':
    try:
        controller = VisualServoController()
        rate = rospy.Rate(10)  # 10Hz控制频率
        
        while not rospy.is_shutdown():
            # 超时检测（二维码丢失保护）
            if controller.last_aruco_time is not None:
                time_since_last = (rospy.Time.now() - controller.last_aruco_time).to_sec()
                
                # 如果超时且未锁定，停止运动并重置状态
                if time_since_last > controller.aruco_timeout and not controller.is_locked:
                    stop_cmd = Twist()
                    controller.vel_pub.publish(stop_cmd)
                    
                    rospy.logwarn_throttle(2, 
                        f"⚠ 二维码丢失 {time_since_last:.1f}秒，已停止运动")
                    
                    # 重置PID状态（防止积分饱和）
                    controller.integral_x = 0.0
                    controller.integral_y = 0.0
                    controller.integral_yaw = 0.0
                    controller.last_error_x = 0.0
                    controller.last_error_y = 0.0
                    controller.last_error_yaw = 0.0
                    controller.stable_count = 0
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("视觉伺服控制器已关闭")