#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool  
from enum import Enum

class ChargingState(Enum):
    IDLE = 0
    SEARCHING = 1
    ALIGNING = 2
    RETREATING = 3
    DOCKED = 4
    FAILED = 5

class ChargingStateMachine:
    def __init__(self):
        rospy.init_node('charging_state_machine', anonymous=True)
        
        self.state = ChargingState.IDLE
        self.retry_count = 0
        self.max_retries = 3
        
        self.search_timeout = rospy.get_param('~timeout/search', 10.0)
        self.align_timeout = rospy.get_param('~timeout/align', 30.0)
        self.state_start_time = None
        
        self.aruco_detected = False
        self.last_aruco_time = None
        self.aruco_lost_threshold = rospy.get_param('~timeout/aruco_lost', 1.0)
        
        self.docking_locked = False  # 修改：监听控制器的锁定状态
        
        self.cmd_vel_pub = rospy.Publisher("/charging_cmd_vel", Twist, queue_size=10)
        self.state_pub = rospy.Publisher("/charging_state", String, queue_size=10)
        
        self.aruco_sub = rospy.Subscriber("/aruco_poses", PoseStamped, self.aruco_callback)
        self.servo_cmd_sub = rospy.Subscriber("/servo_cmd_vel", Twist, self.servo_cmd_callback)
        self.lock_sub = rospy.Subscriber("/docking_locked", Bool, self.lock_callback)  
        
        self.servo_cmd = Twist()
        
        rospy.loginfo("充电状态机启动 | 当前状态: IDLE")
    
    def aruco_callback(self, msg):
        self.aruco_detected = True
        self.last_aruco_time = rospy.Time.now()
    
    def servo_cmd_callback(self, msg):
        self.servo_cmd = msg
    
    def lock_callback(self, msg):
        """新增：接收控制器的锁定信号"""
        self.docking_locked = msg.data
    
    def check_aruco_lost(self):
        if self.last_aruco_time is None:
            return True
        return (rospy.Time.now() - self.last_aruco_time).to_sec() > self.aruco_lost_threshold
    
    def check_timeout(self):
        if self.state_start_time is None:
            return False
        
        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
        
        if self.state == ChargingState.SEARCHING:
            return elapsed > self.search_timeout
        elif self.state == ChargingState.ALIGNING:
            return elapsed > self.align_timeout
        
        return False
    
    def transition_to(self, new_state):
        rospy.loginfo(f"状态转换: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_start_time = rospy.Time.now()
        self.state_pub.publish(new_state.name)
    
    def state_idle(self):
        pass
    
    def state_searching(self):
        cmd = Twist()
        
        if self.aruco_detected and not self.check_aruco_lost():
            rospy.loginfo("✓ 检测到二维码，开始对接")
            self.transition_to(ChargingState.ALIGNING)
            return
        
        if self.check_timeout():
            rospy.logwarn("✗ 搜索超时")
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                self.transition_to(ChargingState.FAILED)
            else:
                rospy.loginfo(f"重试搜索 ({self.retry_count}/{self.max_retries})")
                self.state_start_time = rospy.Time.now()
            return
        
        cmd.angular.z = 0.3
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo_throttle(2, "正在搜索二维码...")
    
    def state_aligning(self):
        # 修改：等待控制器发送锁定信号
        if self.docking_locked:
            rospy.loginfo("✓ 对接完成！")
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.transition_to(ChargingState.DOCKED)
            return
        
        if self.check_aruco_lost():
            rospy.logwarn("二维码丢失，后退重试")
            self.transition_to(ChargingState.RETREATING)
            return
        
        if self.check_timeout():
            rospy.logwarn("✗ 对接超时")
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                self.transition_to(ChargingState.FAILED)
            else:
                rospy.loginfo(f"重试对接 ({self.retry_count}/{self.max_retries})")
                self.transition_to(ChargingState.SEARCHING)
            return
        
        self.cmd_vel_pub.publish(self.servo_cmd)
    
    def state_docked(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo_throttle(5, "对接完成，等待充电...")
    
    def state_failed(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.logerr_throttle(5, f"充电失败！已重试{self.max_retries}次")
    
    def state_retreating(self):
        cmd = Twist()
        cmd.linear.x = -0.2
        self.cmd_vel_pub.publish(cmd)
    
        if (rospy.Time.now() - self.state_start_time).to_sec() > 2.5:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.transition_to(ChargingState.SEARCHING)
        
    def run(self):
        rate = rospy.Rate(10)
        
        rospy.sleep(1.0)
        self.transition_to(ChargingState.SEARCHING)
        
        while not rospy.is_shutdown():
            if self.docking_locked and self.state != ChargingState.DOCKED:
                rospy.loginfo("✓ 检测到锁定信号，直接切换到DOCKED")
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                self.transition_to(ChargingState.DOCKED)
            
            if self.state == ChargingState.IDLE:
                self.state_idle()
            elif self.state == ChargingState.SEARCHING:
                self.state_searching()
            elif self.state == ChargingState.ALIGNING:
                self.state_aligning()
            elif self.state == ChargingState.RETREATING:  
                self.state_retreating()
            elif self.state == ChargingState.DOCKED:
                self.state_docked()
            elif self.state == ChargingState.FAILED:
                self.state_failed()
            
            rate.sleep()

if __name__ == '__main__':
    try:
        machine = ChargingStateMachine()
        machine.run()
    except rospy.ROSInterruptException:
        pass
