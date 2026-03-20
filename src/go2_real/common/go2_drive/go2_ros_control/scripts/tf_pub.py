#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动态TF发布器 - 使用滑动条调整base_link到laser_link的变换
"""

import rospy
import tf2_ros
import tkinter as tk
from tkinter import ttk
from geometry_msgs.msg import TransformStamped
import tf.transformations as tf_trans
import math
import threading

class DynamicTFPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('dynamic_tf_publisher', anonymous=True)
        
        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 初始值（从URDF中获取）
        self.x = 0.30
        self.y = 0.0
        self.z = 0.15
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # 创建GUI
        self.setup_gui()
        
        # 启动TF发布线程
        self.running = True
        self.tf_thread = threading.Thread(target=self.publish_tf_loop)
        self.tf_thread.daemon = True
        self.tf_thread.start()
        
    def setup_gui(self):
        """设置GUI界面"""
        self.root = tk.Tk()
        self.root.title("动态TF发布器 - base_link到laser_link变换")
        self.root.geometry("500x400")
        
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 标题
        title_label = ttk.Label(main_frame, text="base_link → laser_link 变换参数", 
                               font=("Arial", 14, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # 位置参数 (xyz)
        position_frame = ttk.LabelFrame(main_frame, text="位置参数 (m)", padding="10")
        position_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # X轴滑动条
        ttk.Label(position_frame, text="X:").grid(row=0, column=0, sticky=tk.W)
        self.x_var = tk.DoubleVar(value=self.x)
        self.x_scale = ttk.Scale(position_frame, from_=-1.0, to=1.0, 
                                variable=self.x_var, orient=tk.HORIZONTAL, length=200,
                                command=self.update_x)
        self.x_scale.grid(row=0, column=1, padx=(10, 10))
        self.x_label = ttk.Label(position_frame, text=f"{self.x:.4f}")
        self.x_label.grid(row=0, column=2, sticky=tk.W)
        
        # Y轴滑动条
        ttk.Label(position_frame, text="Y:").grid(row=1, column=0, sticky=tk.W)
        self.y_var = tk.DoubleVar(value=self.y)
        self.y_scale = ttk.Scale(position_frame, from_=-1.0, to=1.0, 
                                variable=self.y_var, orient=tk.HORIZONTAL, length=200,
                                command=self.update_y)
        self.y_scale.grid(row=1, column=1, padx=(10, 10))
        self.y_label = ttk.Label(position_frame, text=f"{self.y:.4f}")
        self.y_label.grid(row=1, column=2, sticky=tk.W)
        
        # Z轴滑动条
        ttk.Label(position_frame, text="Z:").grid(row=2, column=0, sticky=tk.W)
        self.z_var = tk.DoubleVar(value=self.z)
        self.z_scale = ttk.Scale(position_frame, from_=-1.0, to=1.0, 
                                variable=self.z_var, orient=tk.HORIZONTAL, length=200,
                                command=self.update_z)
        self.z_scale.grid(row=2, column=1, padx=(10, 10))
        self.z_label = ttk.Label(position_frame, text=f"{self.z:.4f}")
        self.z_label.grid(row=2, column=2, sticky=tk.W)
        
        # 姿态参数 (rpy)
        orientation_frame = ttk.LabelFrame(main_frame, text="姿态参数 (rad)", padding="10")
        orientation_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Roll滑动条
        ttk.Label(orientation_frame, text="Roll:").grid(row=0, column=0, sticky=tk.W)
        self.roll_var = tk.DoubleVar(value=self.roll)
        self.roll_scale = ttk.Scale(orientation_frame, from_=-math.pi, to=math.pi, 
                                   variable=self.roll_var, orient=tk.HORIZONTAL, length=200,
                                   command=self.update_roll)
        self.roll_scale.grid(row=0, column=1, padx=(10, 10))
        self.roll_label = ttk.Label(orientation_frame, text=f"{self.roll:.4f}")
        self.roll_label.grid(row=0, column=2, sticky=tk.W)
        
        # Pitch滑动条
        ttk.Label(orientation_frame, text="Pitch:").grid(row=1, column=0, sticky=tk.W)
        self.pitch_var = tk.DoubleVar(value=self.pitch)
        self.pitch_scale = ttk.Scale(orientation_frame, from_=-math.pi, to=math.pi, 
                                    variable=self.pitch_var, orient=tk.HORIZONTAL, length=200,
                                    command=self.update_pitch)
        self.pitch_scale.grid(row=1, column=1, padx=(10, 10))
        self.pitch_label = ttk.Label(orientation_frame, text=f"{self.pitch:.4f}")
        self.pitch_label.grid(row=1, column=2, sticky=tk.W)
        
        # Yaw滑动条
        ttk.Label(orientation_frame, text="Yaw:").grid(row=2, column=0, sticky=tk.W)
        self.yaw_var = tk.DoubleVar(value=self.yaw)
        self.yaw_scale = ttk.Scale(orientation_frame, from_=-math.pi, to=math.pi, 
                                  variable=self.yaw_var, orient=tk.HORIZONTAL, length=200,
                                  command=self.update_yaw)
        self.yaw_scale.grid(row=2, column=1, padx=(10, 10))
        self.yaw_label = ttk.Label(orientation_frame, text=f"{self.yaw:.4f}")
        self.yaw_label.grid(row=2, column=2, sticky=tk.W)
        
        # 控制按钮
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, columnspan=3, pady=(20, 0))
        
        reset_button = ttk.Button(button_frame, text="重置到初始值", command=self.reset_values)
        reset_button.pack(side=tk.LEFT, padx=(0, 10))
        
        # 状态显示
        self.status_label = ttk.Label(main_frame, text="状态: TF发布中...", 
                                     foreground="green")
        self.status_label.grid(row=4, column=0, columnspan=3, pady=(10, 0))
        
        # 配置列权重
        main_frame.columnconfigure(1, weight=1)
        position_frame.columnconfigure(1, weight=1)
        orientation_frame.columnconfigure(1, weight=1)
        
    def update_x(self, value):
        """更新X坐标"""
        self.x = float(value)
        self.x_label.config(text=f"{self.x:.4f}")
        
    def update_y(self, value):
        """更新Y坐标"""
        self.y = float(value)
        self.y_label.config(text=f"{self.y:.4f}")
        
    def update_z(self, value):
        """更新Z坐标"""
        self.z = float(value)
        self.z_label.config(text=f"{self.z:.4f}")
        
    def update_roll(self, value):
        """更新Roll角度"""
        self.roll = float(value)
        self.roll_label.config(text=f"{self.roll:.4f}")
        
    def update_pitch(self, value):
        """更新Pitch角度"""
        self.pitch = float(value)
        self.pitch_label.config(text=f"{self.pitch:.4f}")
        
    def update_yaw(self, value):
        """更新Yaw角度"""
        self.yaw = float(value)
        self.yaw_label.config(text=f"{self.yaw:.4f}")
        
    def reset_values(self):
        """重置到初始值"""
        self.x = 0.30
        self.y = 0.0
        self.z = 0.15
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # 更新滑动条
        self.x_var.set(self.x)
        self.y_var.set(self.y)
        self.z_var.set(self.z)
        self.roll_var.set(self.roll)
        self.pitch_var.set(self.pitch)
        self.yaw_var.set(self.yaw)
        
        # 更新标签
        self.x_label.config(text=f"{self.x:.4f}")
        self.y_label.config(text=f"{self.y:.4f}")
        self.z_label.config(text=f"{self.z:.4f}")
        self.roll_label.config(text=f"{self.roll:.4f}")
        self.pitch_label.config(text=f"{self.pitch:.4f}")
        self.yaw_label.config(text=f"{self.yaw:.4f}")
        
    def publish_tf_loop(self):
        """TF发布循环"""
        rate = rospy.Rate(50)  # 50Hz发布频率
        
        while not rospy.is_shutdown() and self.running:
            try:
                # 创建变换消息
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "base_link"
                transform.child_frame_id = "laser_link"
                
                # 设置位置
                transform.transform.translation.x = self.x
                transform.transform.translation.y = self.y
                transform.transform.translation.z = self.z
                
                # 将RPY转换为四元数
                quaternion = tf_trans.quaternion_from_euler(self.roll, self.pitch, self.yaw)
                transform.transform.rotation.x = quaternion[0]
                transform.transform.rotation.y = quaternion[1]
                transform.transform.rotation.z = quaternion[2]
                transform.transform.rotation.w = quaternion[3]
                
                # 发布变换
                self.tf_broadcaster.sendTransform(transform)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logwarn(f"TF发布错误: {e}")
                rate.sleep()
                
    def run(self):
        """运行GUI主循环"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            rospy.signal_shutdown("GUI关闭")

if __name__ == "__main__":
    try:
        publisher = DynamicTFPublisher()
        rospy.loginfo("动态TF发布器已启动")
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("动态TF发布器已关闭")
    except Exception as e:
        rospy.logerr(f"启动失败: {e}")
