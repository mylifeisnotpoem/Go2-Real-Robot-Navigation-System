#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
导航任务GUI控制界面
提供可视化的导航控制界面，包括目标点显示和控制按钮
"""

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import sys
import threading
import datetime
import os
import rospkg
import tkinter as tk
from tkinter import ttk

try:
    # Python 2/3 兼容性
    import tkinter as tk
    from tkinter import ttk, messagebox
except ImportError:
    import Tkinter as tk
    import ttk
    import tkMessageBox as messagebox

class NavigationControlGUI:
    def __init__(self):
        """初始化GUI控制界面"""
        
        # 初始化ROS节点
        rospy.init_node('navigation_control_gui', anonymous=True)
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("Go2 机器人导航控制界面")
        self.root.geometry("500x400")
        self.root.resizable(True, True)
        
        # 设置窗口图标和样式
        self.setup_styles()
        
        # 初始化变量
        self.current_target = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.emergency_stop_state = False
        self.nav_state = False
        self.last_target_update = "未接收到目标点"
        self.available_tasks = []  # 可用的任务列表
        self.selected_task = tk.StringVar()  # 选中的任务
        
        # 加载可用的任务文件
        self.load_available_tasks()
        
        # 创建ROS发布器
        self.nav_task_signal_pub = rospy.Publisher('/nav_task_signal', String, queue_size=10)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)
        
        # 创建ROS订阅器
        self.target_points_sub = rospy.Subscriber('/target_points', Pose, self.target_points_callback)
        self.nav_state_sub = rospy.Subscriber('/nav_state', Bool, self.nav_state_callback)
        
        # 创建GUI界面
        self.create_widgets()
        
        # 启动ROS消息处理线程
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # 启动GUI更新定时器
        self.update_display()
        
        rospy.loginfo("导航控制GUI界面已启动")
    
    def load_available_tasks(self):
        """加载config/task目录下的所有任务文件"""
        try:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('user_interface')
            task_folder = os.path.join(package_path, 'config', 'task')
            
            if not os.path.exists(task_folder):
                rospy.logwarn("任务文件夹不存在: %s", task_folder)
                self.available_tasks = []
                return
            
            # 获取所有.txt文件
            files = [f[:-4] for f in os.listdir(task_folder) 
                    if f.endswith('.txt') and os.path.isfile(os.path.join(task_folder, f))]
            
            self.available_tasks = sorted(files)
            rospy.loginfo("找到 %d 个任务文件: %s", len(self.available_tasks), ', '.join(self.available_tasks))
            
            # 设置默认选中第一个任务
            if self.available_tasks:
                self.selected_task.set(self.available_tasks[0])
            
        except Exception as e:
            rospy.logerr("加载任务文件列表失败: %s", str(e))
            self.available_tasks = []
    
    def setup_styles(self):
        """设置GUI样式"""
        style = ttk.Style()
        
        # 配置按钮样式
        style.configure('Start.TButton', foreground='white', background='green', font=('Arial', 12, 'bold'))
        style.configure('Stop.TButton', foreground='white', background='red', font=('Arial', 12, 'bold'))
        style.configure('Home.TButton', foreground='white', background='blue', font=('Arial', 12, 'bold'))
        style.configure('Emergency.TButton', foreground='white', background='orange', font=('Arial', 12, 'bold'))
    
    def create_widgets(self):
        """创建GUI组件"""
        
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # 标题
        title_label = tk.Label(main_frame, text="Go2 机器人导航控制", 
                              font=('Arial', 16, 'bold'), fg='navy')
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # 状态显示区域
        status_frame = ttk.LabelFrame(main_frame, text="机器人状态", padding="10")
        status_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 20))
        status_frame.columnconfigure(1, weight=1)
        
        # 当前目标点显示
        tk.Label(status_frame, text="当前目标点:", font=('Arial', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, pady=2)
        self.target_label = tk.Label(status_frame, text="等待目标点...", 
                                    font=('Arial', 10), fg='blue', wraplength=300)
        self.target_label.grid(row=0, column=1, sticky=(tk.W, tk.E), pady=2, padx=(10, 0))
        
        # 导航状态显示
        tk.Label(status_frame, text="导航状态:", font=('Arial', 10, 'bold')).grid(row=1, column=0, sticky=tk.W, pady=2)
        self.nav_status_label = tk.Label(status_frame, text="未知", 
                                        font=('Arial', 10), fg='gray')
        self.nav_status_label.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=2, padx=(10, 0))
        
        # 紧急停止状态显示
        tk.Label(status_frame, text="紧急停止:", font=('Arial', 10, 'bold')).grid(row=2, column=0, sticky=tk.W, pady=2)
        self.emergency_status_label = tk.Label(status_frame, text="关闭", 
                                              font=('Arial', 10), fg='green')
        self.emergency_status_label.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=2, padx=(10, 0))
        
        # 最后更新时间
        tk.Label(status_frame, text="最后更新:", font=('Arial', 10, 'bold')).grid(row=3, column=0, sticky=tk.W, pady=2)
        self.update_time_label = tk.Label(status_frame, text="-", 
                                         font=('Arial', 10), fg='gray')
        self.update_time_label.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=2, padx=(10, 0))
        
        # 任务选择区域
        task_select_frame = ttk.LabelFrame(main_frame, text="任务选择", padding="10")
        task_select_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 20))
        task_select_frame.columnconfigure(1, weight=1)
        
        tk.Label(task_select_frame, text="选择任务:", font=('Arial', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, padx=5)
        
        # 创建任务选择下拉框
        if self.available_tasks:
            self.task_combobox = ttk.Combobox(task_select_frame, textvariable=self.selected_task, 
                                             values=self.available_tasks, state='readonly', 
                                             font=('Arial', 10), width=30)
        else:
            self.task_combobox = ttk.Combobox(task_select_frame, textvariable=self.selected_task, 
                                             values=['无可用任务'], state='readonly', 
                                             font=('Arial', 10), width=30)
            self.selected_task.set('无可用任务')
        
        self.task_combobox.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5, pady=5)
        
        # 控制按钮区域
        control_frame = ttk.LabelFrame(main_frame, text="导航控制", padding="10")
        control_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 20))
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        control_frame.columnconfigure(2, weight=1)
        
        # 开始导航按钮
        self.start_button = tk.Button(control_frame, text="开始导航", 
                                     command=self.start_navigation,
                                     bg='green', fg='white', font=('Arial', 12, 'bold'),
                                     height=2, relief=tk.RAISED)
        self.start_button.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))
        
        # 紧急停止按钮
        self.emergency_button = tk.Button(control_frame, text="紧急停止", 
                                         command=self.toggle_emergency_stop,
                                         bg='red', fg='white', font=('Arial', 12, 'bold'),
                                         height=2, relief=tk.RAISED)
        self.emergency_button.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))
        
        # 回家按钮
        self.home_button = tk.Button(control_frame, text="回家", 
                                    command=self.go_home,
                                    bg='blue', fg='white', font=('Arial', 12, 'bold'),
                                    height=2, relief=tk.RAISED)
        self.home_button.grid(row=0, column=2, padx=5, pady=5, sticky=(tk.W, tk.E))
        
        # 任务控制按钮区域
        task_frame = ttk.LabelFrame(main_frame, text="任务控制", padding="10")
        task_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 20))
        task_frame.columnconfigure(0, weight=1)
        task_frame.columnconfigure(1, weight=1)
        
        # 停止任务按钮
        self.stop_task_button = tk.Button(task_frame, text="停止任务", 
                                         command=self.stop_task,
                                         bg='orange', fg='white', font=('Arial', 11, 'bold'),
                                         height=1, relief=tk.RAISED)
        self.stop_task_button.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))
        
        # 继续任务按钮
        self.continue_task_button = tk.Button(task_frame, text="继续任务", 
                                             command=self.continue_task,
                                             bg='purple', fg='white', font=('Arial', 11, 'bold'),
                                             height=1, relief=tk.RAISED)
        self.continue_task_button.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))
        
        # 日志显示区域
        log_frame = ttk.LabelFrame(main_frame, text="操作日志", padding="10")
        log_frame.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        main_frame.rowconfigure(5, weight=1)
        
        # 创建日志文本框和滚动条
        self.log_text = tk.Text(log_frame, height=8, wrap=tk.WORD, font=('Consolas', 9))
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # 添加初始日志
        self.add_log("导航控制GUI界面已启动")
        self.add_log("等待ROS消息...")
        
        # 状态栏
        self.status_bar = tk.Label(self.root, text="就绪", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.grid(row=1, column=0, sticky=(tk.W, tk.E))
        
        # 确保根窗口行的配置
        self.root.rowconfigure(1, weight=0)
    
    def add_log(self, message):
        """添加日志消息"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_message)
        self.log_text.see(tk.END)
        
        # 限制日志行数
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 100:
            self.log_text.delete("1.0", "2.0")
    
    def target_points_callback(self, msg):
        """目标点回调函数"""
        self.current_target["x"] = msg.position.x
        self.current_target["y"] = msg.position.y
        self.current_target["z"] = msg.position.z
        
        import datetime
        self.last_target_update = datetime.datetime.now().strftime("%H:%M:%S")
        
        rospy.logdebug("收到目标点: (%.3f, %.3f, %.3f)", 
                      self.current_target["x"], self.current_target["y"], self.current_target["z"])
    
    def nav_state_callback(self, msg):
        """导航状态回调函数"""
        self.nav_state = msg.data
        rospy.logdebug("收到导航状态: %s", "已到达" if self.nav_state else "导航中")
    
    def start_navigation(self):
        """开始导航"""
        selected_task = self.selected_task.get()
        
        if not selected_task or selected_task == '无可用任务':
            messagebox.showwarning("警告", "请先选择一个任务！")
            return
        
        msg = String()
        msg.data = selected_task
        self.nav_task_signal_pub.publish(msg)
        
        self.add_log(f"开始执行任务: {selected_task}")
        self.status_bar.config(text=f"已发送任务信号: {selected_task}")
        rospy.loginfo("开始执行任务: %s", selected_task)
    
    def toggle_emergency_stop(self):
        """切换紧急停止状态"""
        self.emergency_stop_state = not self.emergency_stop_state
        
        msg = Bool()
        msg.data = self.emergency_stop_state
        self.emergency_stop_pub.publish(msg)
        
        status_text = "激活" if self.emergency_stop_state else "关闭"
        self.add_log(f"紧急停止: {status_text}")
        self.status_bar.config(text=f"紧急停止已{status_text}")
        
        # 更新按钮颜色
        if self.emergency_stop_state:
            self.emergency_button.config(bg='darkred', text="解除停止")
        else:
            self.emergency_button.config(bg='red', text="紧急停止")
        
        rospy.loginfo("紧急停止状态: %s", status_text)
    
    def go_home(self):
        """回家"""
        msg = String()
        msg.data = "GO_HOME"
        self.nav_task_signal_pub.publish(msg)
        
        self.add_log("发送回家信号")
        self.status_bar.config(text="已发送回家信号")
        rospy.loginfo("发送回家信号")
    
    def stop_task(self):
        """停止任务"""
        msg = String()
        msg.data = "STOP"
        self.nav_task_signal_pub.publish(msg)
        
        self.add_log("发送停止任务信号")
        self.status_bar.config(text="已发送停止任务信号")
        rospy.loginfo("发送停止任务信号")
    
    def continue_task(self):
        """继续任务"""
        msg = String()
        msg.data = "CONTINUE"
        self.nav_task_signal_pub.publish(msg)
        
        self.add_log("发送继续任务信号")
        self.status_bar.config(text="已发送继续任务信号")
        rospy.loginfo("发送继续任务信号")
    
    def update_display(self):
        """更新显示信息"""
        try:
            # 更新目标点显示
            target_text = f"X: {self.current_target['x']:.3f}, Y: {self.current_target['y']:.3f}, Z: {self.current_target['z']:.3f}"
            self.target_label.config(text=target_text)
            
            # 更新导航状态显示
            if self.nav_state:
                self.nav_status_label.config(text="已到达目标点", fg='green')
            else:
                self.nav_status_label.config(text="导航中", fg='blue')
            
            # 更新紧急停止状态显示
            if self.emergency_stop_state:
                self.emergency_status_label.config(text="激活", fg='red')
            else:
                self.emergency_status_label.config(text="关闭", fg='green')
            
            # 更新最后更新时间
            self.update_time_label.config(text=self.last_target_update)
            
        except Exception as e:
            rospy.logwarn("更新显示时出错: %s", str(e))
        
        # 每100ms更新一次
        self.root.after(100, self.update_display)
    
    def ros_spin(self):
        """ROS消息处理线程"""
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr("ROS消息处理出错: %s", str(e))
    
    def on_closing(self):
        """窗口关闭事件处理"""
        if messagebox.askokcancel("退出", "确定要退出导航控制界面吗？"):
            rospy.loginfo("导航控制GUI界面正在关闭...")
            rospy.signal_shutdown("GUI界面关闭")
            self.root.destroy()
    
    def run(self):
        """运行GUI界面"""
        try:
            # 设置窗口关闭事件
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            
            # 启动GUI主循环
            rospy.loginfo("GUI界面开始运行")
            self.root.mainloop()
            
        except Exception as e:
            rospy.logerr("GUI运行出错: %s", str(e))
            messagebox.showerror("错误", f"GUI运行出错: {str(e)}")

def main():
    """主函数"""
    try:
        # 创建并运行GUI
        gui = NavigationControlGUI()
        gui.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被ROS中断")
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr("程序运行出错: %s", str(e))
        print(f"错误: {str(e)}")

if __name__ == '__main__':
    main()