#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PointStamped
from unitree_go.msg import WirelessController

class PointPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('point_publisher', anonymous=True)
        
        # 预定义的目标点 (x, y, z坐标)
        self.target_points = {
            4096: {"x": 0.0, "y": 0.0, "z": 0.0, "name": "目标点1"},
            8192: {"x": 56.427, "y": -4.479, "z": -6.792, "name": "目标点2"},
            16384: {"x": 24.053, "y": 46.684, "z": -0.368, "name": "目标点3"},
            32768: {"x": -12.831, "y": 1.47, "z": -4.05, "name": "目标点4"}
        }
 

        # 参数配置
        self.frame_id = rospy.get_param('~frame_id', 'map')  # 目标点的坐标系
        self.debounce_time = rospy.get_param('~debounce_time', 0.5)  # 防抖时间（秒）
        
        # 防抖机制
        self.last_key_press_time = {}  # 记录每个按键的上次按下时间
        self.last_key_state = {}       # 记录每个按键的上次状态
        
        # 初始化按键状态
        for key in self.target_points.keys():
            self.last_key_press_time[key] = 0.0
            self.last_key_state[key] = False
        
        # 发布器
        self.target_point_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
        
        # 订阅器
        rospy.Subscriber('/wireless_controller', WirelessController, self.wireless_controller_callback)
        
        rospy.loginfo("Point Publisher节点已启动")
        rospy.loginfo("监听按键: 4096, 8192, 16384, 32768")
        rospy.loginfo("目标坐标系: %s", self.frame_id)
        rospy.loginfo("防抖时间: %.1f秒", self.debounce_time)

    def wireless_controller_callback(self, msg):
        """处理无线控制器消息，带防抖机制"""
        try:
            current_time = time.time()
            
            # 检查每个目标按键
            for key in self.target_points.keys():
                # 检查当前按键是否被按下
                key_pressed = (msg.keys & key) != 0
                
                # 获取上次状态
                last_state = self.last_key_state.get(key, False)
                last_press_time = self.last_key_press_time.get(key, 0.0)
                
                # 检测按键按下事件（从未按下到按下的状态变化）
                if key_pressed and not last_state:
                    # 检查防抖时间
                    if current_time - last_press_time >= self.debounce_time:
                        target_info = self.target_points[key]
                        rospy.loginfo(f"检测到按键 {key}，发送{target_info['name']}: ({target_info['x']}, {target_info['y']}, {target_info['z']})")
                        
                        # 发送预定义的目标点
                        self.publish_target_point(
                            target_info['x'], 
                            target_info['y'], 
                            target_info['z'],
                            target_info['name']
                        )
                        
                        # 更新按键按下时间
                        self.last_key_press_time[key] = current_time
                    else:
                        # 防抖期间，忽略按键
                        remaining_time = self.debounce_time - (current_time - last_press_time)
                        rospy.logdebug(f"按键 {key} 在防抖期间，剩余时间: {remaining_time:.2f}秒")
                
                # 更新按键状态
                self.last_key_state[key] = key_pressed
                
        except Exception as e:
            rospy.logerr(f"处理无线控制器消息时出错: {e}")

    def publish_target_point(self, x, y, z, name="目标点"):
        """发布预定义的目标点"""
        try:
            # 创建PointStamped消息
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = self.frame_id
            point_msg.point.x = x
            point_msg.point.y = y
            point_msg.point.z = z
            
            # 发布目标点
            self.target_point_pub.publish(point_msg)
            
            rospy.loginfo(f"已发布{name}到clicked_point话题")
            
        except Exception as e:
            rospy.logerr(f"发布目标点时出错: {e}")

    def reset_debounce_state(self, key=None):
        """重置防抖状态"""
        try:
            if key is None:
                # 重置所有按键
                for k in self.target_points.keys():
                    self.last_key_press_time[k] = 0.0
                    self.last_key_state[k] = False
                rospy.loginfo("已重置所有按键的防抖状态")
            elif key in self.target_points:
                # 重置指定按键
                self.last_key_press_time[key] = 0.0
                self.last_key_state[key] = False
                rospy.loginfo(f"已重置按键 {key} 的防抖状态")
            else:
                rospy.logwarn(f"按键 {key} 不在预定义列表中")
        except Exception as e:
            rospy.logerr(f"重置防抖状态时出错: {e}")

    def set_debounce_time(self, debounce_time):
        """动态设置防抖时间"""
        if debounce_time >= 0:
            self.debounce_time = debounce_time
            rospy.loginfo(f"防抖时间已设置为: {debounce_time:.1f}秒")
        else:
            rospy.logwarn("防抖时间必须为非负数")

    def update_target_point(self, key, x, y, z, name):
        """更新指定按键对应的目标点"""
        if key in self.target_points:
            self.target_points[key] = {"x": x, "y": y, "z": z, "name": name}
            rospy.loginfo(f"已更新按键{key}的目标点: {name} ({x}, {y}, {z})")
        else:
            rospy.logwarn(f"按键{key}不在预定义列表中")

    def get_target_points_info(self):
        """获取所有目标点信息"""
        info = "当前配置的目标点:\n"
        for key, point in self.target_points.items():
            info += f"  按键{key}: {point['name']} ({point['x']}, {point['y']}, {point['z']})\n"
        return info

    def run(self):
        """运行节点"""
        rospy.loginfo("Point Publisher节点正在运行...")
        rospy.loginfo(self.get_target_points_info())
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PointPublisher()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Point Publisher节点已退出")
