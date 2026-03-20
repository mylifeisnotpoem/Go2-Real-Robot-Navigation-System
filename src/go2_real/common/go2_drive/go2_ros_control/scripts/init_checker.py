#!/usr/bin/env python3

import rospy
import message_filters
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan, PointCloud2

class InitChecker:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('init_checker', anonymous=True)
        
        # 创建发布者，发布初始化标志
        self.init_flag_pub = rospy.Publisher('/init_flag', Bool, queue_size=10)
        
        # 初始化标志，记录每个关键话题是否已收到数据
        self.topics_ready = {
            'odometry': False,
            'map': False,
            'tf': False,
            'scan': False,
            'cloud': False
        }
        
        # 初始化计时器，用于定期检查并发布状态
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self.check_and_publish)
        
        # 创建订阅者，监听各个关键话题
        rospy.Subscriber('/Odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_callback)
        
        rospy.loginfo("初始化检查节点已启动，正在等待关键话题数据...")
    
    def odometry_callback(self, msg):
        if not self.topics_ready['odometry']:
            self.topics_ready['odometry'] = True
            rospy.loginfo("已接收到 Odometry 数据")
    
    def map_callback(self, msg):
        if not self.topics_ready['map']:
            self.topics_ready['map'] = True
            rospy.loginfo("已接收到 Map 数据")
    
    def tf_callback(self, msg):
        if not self.topics_ready['tf']:
            self.topics_ready['tf'] = True
            rospy.loginfo("已接收到 TF 数据")
    
    def scan_callback(self, msg):
        if not self.topics_ready['scan']:
            self.topics_ready['scan'] = True
            rospy.loginfo("已接收到 Scan 数据")
    
    def cloud_callback(self, msg):
        if not self.topics_ready['cloud']:
            self.topics_ready['cloud'] = True
            rospy.loginfo("已接收到 PointCloud2 数据")
    
    def check_and_publish(self, event=None):
        """检查所有关键话题是否都已收到数据，如果都收到了，则发布初始化成功标志"""
        # 检查是否所有话题都已经收到数据
        all_ready = all(self.topics_ready.values())
        
        # 创建消息
        msg = Bool()
        msg.data = all_ready
        
        # 发布消息
        self.init_flag_pub.publish(msg)
        
        # 如果所有话题都已准备就绪，且是第一次检测到，则打印日志
        if all_ready and not hasattr(self, 'has_reported_all_ready'):
            rospy.loginfo("所有关键话题已准备就绪，初始化成功！发布 init_flag=True")
            self.has_reported_all_ready = True
            
            # 发布一次成功消息
            self.publish_success_flag()
            
            # 短暂延迟，确保消息被发布出去
            rospy.loginfo("初始化成功，等待0.5秒确保消息发布完成...")
            rospy.sleep(0.5)
            
            # 初始化成功后，关闭整个节点
            rospy.loginfo("初始化检查完成，关闭初始化检查节点")
            rospy.signal_shutdown("初始化检查完成")
        
        # 打印当前状态
        if not all_ready and hasattr(self, 'last_report_time') and rospy.Time.now() - self.last_report_time > rospy.Duration(5.0):
            missing_topics = [topic for topic, ready in self.topics_ready.items() if not ready]
            rospy.loginfo(f"等待以下话题的数据: {', '.join(missing_topics)}")
            self.last_report_time = rospy.Time.now()
        elif not hasattr(self, 'last_report_time'):
            self.last_report_time = rospy.Time.now()
    
    def publish_success_flag(self, event=None):
        """在初始化成功后，定期发布成功标志"""
        msg = Bool()
        msg.data = True
        self.init_flag_pub.publish(msg)
        rospy.logdebug("持续发布初始化成功标志")

if __name__ == '__main__':
    try:
        checker = InitChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
