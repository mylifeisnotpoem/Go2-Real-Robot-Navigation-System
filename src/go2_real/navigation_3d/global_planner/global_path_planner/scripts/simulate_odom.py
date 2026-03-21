#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math

class OdomSimulator:
    def __init__(self):
        rospy.init_node('odom_simulator', anonymous=True)
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.tf_br = tf.TransformBroadcaster()
        
        self.x = 0  # 起始x坐标，接近拓扑地图中的某个点
        self.y = 0   # 起始y坐标
        self.z = 0  # 起始z坐标
        self.th = 0.0   # 朝向角度
        
        self.vx = 0.0   # x方向速度
        self.vy = 0.0   # y方向速度
        self.vth = 0.0  # 角速度
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        rospy.loginfo("里程计模拟器启动，初始位置: x=%.2f, y=%.2f, z=%.2f", self.x, self.y, self.z)
    
    def publish_odom(self):
        current_time = rospy.Time.now()
        
        # 计算时间差
        dt = (current_time - self.last_time).to_sec()
        
        # 更新位置（这里保持静止，只发布固定位置）
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 创建四元数
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # 发布 tf 变换
        self.tf_br.sendTransform(
            (self.x, self.y, self.z),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )
        
        # 创建并发布里程计消息
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # 设置位置
        odom.pose.pose = Pose(
            Point(self.x, self.y, self.z),
            Quaternion(*odom_quat)
        )
        
        # 设置速度
        odom.twist.twist = Twist(
            Vector3(self.vx, self.vy, 0),
            Vector3(0, 0, self.vth)
        )
        
        # 设置协方差（可选）
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[14] = 0.01 # z
        odom.pose.covariance[21] = 0.01 # roll
        odom.pose.covariance[28] = 0.01 # pitch
        odom.pose.covariance[35] = 0.01 # yaw
        
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[7] = 0.01  # vy
        odom.twist.covariance[35] = 0.01 # vyaw
        
        # 发布里程计消息
        self.odom_pub.publish(odom)
        
        self.last_time = current_time
    
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            self.publish_odom()
            rate.sleep()

if __name__ == '__main__':
    try:
        simulator = OdomSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
