#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
导航任务命令行控制器
提供简单的命令行界面来控制导航任务
"""

import rospy
from std_msgs.msg import String, Bool, String
from geometry_msgs.msg import Pose
import sys
import threading
import time

class NavigationControlCLI:
    def __init__(self):
        """初始化命令行控制器"""
        
        # 初始化ROS节点
        rospy.init_node('navigation_control_cli', anonymous=True)
        
        # 初始化变量
        self.current_target = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.emergency_stop_state = False
        self.nav_state = False
        self.target_received = False
        
        # 创建ROS发布器
        self.nav_task_signal_pub = rospy.Publisher('/nav_task_signal', String, queue_size=10)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)
        
        # 创建ROS订阅器
        self.target_points_sub = rospy.Subscriber('/target_points', Pose, self.target_points_callback)
        self.nav_state_sub = rospy.Subscriber('/nav_state', Bool, self.nav_state_callback)
        
        # 启动ROS消息处理线程
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        rospy.loginfo("导航控制命令行界面已启动")
    
    def target_points_callback(self, msg):
        """目标点回调函数"""
        self.current_target["x"] = msg.position.x
        self.current_target["y"] = msg.position.y
        self.current_target["z"] = msg.position.z
        self.target_received = True
        
        rospy.loginfo("收到目标点: (%.3f, %.3f, %.3f)", 
                      self.current_target["x"], self.current_target["y"], self.current_target["z"])
    
    def nav_state_callback(self, msg):
        """导航状态回调函数"""
        self.nav_state = msg.data
        status_text = "已到达目标点" if self.nav_state else "导航中"
        rospy.loginfo("导航状态: %s", status_text)
    
    def send_task_signal(self, signal):
        """发送任务信号"""
        signal_names = {1: "二教学楼巡检任务", 2: "STOP", 3: "CONTINUE", 4: "GO_HOME"}
        
        msg = String()
        msg.data = signal_names.get(signal, str(signal))
        self.nav_task_signal_pub.publish(msg)
        
        rospy.loginfo("发送任务信号: %d (%s)", signal, signal_names.get(signal, "未知"))
    
    def toggle_emergency_stop(self):
        """切换紧急停止状态"""
        self.emergency_stop_state = not self.emergency_stop_state
        
        msg = Bool()
        msg.data = self.emergency_stop_state
        self.emergency_stop_pub.publish(msg)
        
        status_text = "激活" if self.emergency_stop_state else "关闭"
        rospy.loginfo("紧急停止: %s", status_text)
    
    def show_status(self):
        """显示当前状态"""
        print("\n=== 机器人导航状态 ===")
        if self.target_received:
            print(f"当前目标点: X={self.current_target['x']:.3f}, Y={self.current_target['y']:.3f}, Z={self.current_target['z']:.3f}")
        else:
            print("当前目标点: 未接收到目标点")
        
        nav_status = "已到达目标点" if self.nav_state else "导航中"
        print(f"导航状态: {nav_status}")
        
        emergency_status = "激活" if self.emergency_stop_state else "关闭"
        print(f"紧急停止: {emergency_status}")
        print("========================\n")
    
    def print_help(self):
        """打印帮助信息"""
        print("\n=== 导航控制命令 ===")
        print("1 或 s - 开始循环导航")
        print("2 或 stop - 停止任务")
        print("3 或 c - 继续执行任务")
        print("4 或 h - 回家")
        print("e - 切换紧急停止状态")
        print("status - 显示当前状态")
        print("help - 显示此帮助信息")
        print("q 或 quit - 退出程序")
        print("===================\n")
    
    def ros_spin(self):
        """ROS消息处理线程"""
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr("ROS消息处理出错: %s", str(e))
    
    def run(self):
        """运行命令行界面"""
        print("\n=== Go2 机器人导航控制界面 ===")
        print("等待ROS消息...")
        time.sleep(2)  # 等待ROS连接建立
        
        self.print_help()
        
        try:
            while not rospy.is_shutdown():
                try:
                    # Python 2/3 兼容性
                    if sys.version_info[0] >= 3:
                        cmd = input("请输入命令 (输入 help 查看帮助): ").strip().lower()
                    else:
                        cmd = raw_input("请输入命令 (输入 help 查看帮助): ").strip().lower()
                    
                    if cmd in ['q', 'quit', 'exit']:
                        print("退出导航控制界面...")
                        break
                    elif cmd in ['1', 's', 'start']:
                        self.send_task_signal(1)
                    elif cmd in ['2', 'stop']:
                        self.send_task_signal(2)
                    elif cmd in ['3', 'c', 'continue']:
                        self.send_task_signal(3)
                    elif cmd in ['4', 'h', 'home']:
                        self.send_task_signal(4)
                    elif cmd == 'e':
                        self.toggle_emergency_stop()
                    elif cmd == 'status':
                        self.show_status()
                    elif cmd == 'help':
                        self.print_help()
                    elif cmd == '':
                        continue
                    else:
                        print(f"未知命令: {cmd}")
                        print("输入 'help' 查看可用命令")
                
                except KeyboardInterrupt:
                    print("\n\n程序被用户中断，正在退出...")
                    break
                except EOFError:
                    print("\n\n检测到EOF，正在退出...")
                    break
                except Exception as e:
                    print(f"输入处理出错: {str(e)}")
        
        except Exception as e:
            rospy.logerr("命令行界面运行出错: %s", str(e))
        
        rospy.signal_shutdown("命令行界面关闭")

def main():
    """主函数"""
    try:
        # 创建并运行命令行界面
        cli = NavigationControlCLI()
        cli.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被ROS中断")
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr("程序运行出错: %s", str(e))
        print(f"错误: {str(e)}")

if __name__ == '__main__':
    main()