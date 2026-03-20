#!/usr/bin/env python3

import json
import logging
import os
import rospy
import sys
import time
from sensor_msgs.msg import JointState, PointCloud2
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import PoseStamped
from unitree_go.msg import LowState
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Go2DriverNode:
    def __init__(self):
        rospy.init_node('go2_driver_node')

        # 获取网卡名称参数
        self.interface = rospy.get_param('~interface', os.getenv('GO2_INTERFACE', 'eth0'))
        # 初始化 CycloneDDS - 参照官方程序，直接使用默认初始化
        try:
            # 直接使用默认配置初始化 CycloneDDS，忽略 ROS 传入的命令行参数
            rospy.loginfo("Initializing CycloneDDS with default config")
            ChannelFactoryInitialize(0, self.interface)
            rospy.loginfo("CycloneDDS initialized successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize CycloneDDS: {e}")
            raise
        
        # 创建发布器
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.pointcloud_pub = rospy.Publisher('lidar/points', PointCloud2, queue_size=10)
        
        # 初始化关节状态
        self.joint_state = JointState()
        self.joint_state.name = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]
        self.joint_state.position = [0.0] * 12
        
        # 使用 CycloneDDS 方式订阅底层数据
        self.low_state = None
        self.scan = None
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)
        # self.lidar_subscriber = ChannelSubscriber("rt/utlidar/cloud_base", PointCloud2_)
        self.lidar_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
        self.lidar_subscriber.Init(self.LidarMessageHandler, 10)


        rospy.loginfo(f"Go2DriverNode initialized with interface: {self.interface}")

    def LidarMessageHandler(self, msg: PointCloud2_):
        """处理 CycloneDDS Lidar 数据回调"""
        self.scan = msg
        self.publish_pointcloud(msg)

    def LowStateMessageHandler(self, msg: LowState_):
        """处理 CycloneDDS 底层状态回调"""
        self.low_state = msg
        self.publish_joint_state_cyclonedds(msg)
        
    def publish_joint_state_cyclonedds(self, msg):
        """发布关节状态"""
        now = rospy.Time.now()
        
        # 更新关节状态
        self.joint_state.header.stamp = now
        self.joint_state.position = [
            msg.motor_state[3].q, msg.motor_state[4].q, msg.motor_state[5].q,  # FL
            msg.motor_state[0].q, msg.motor_state[1].q, msg.motor_state[2].q,  # FR
            msg.motor_state[9].q, msg.motor_state[10].q, msg.motor_state[11].q,  # RL
            msg.motor_state[6].q, msg.motor_state[7].q, msg.motor_state[8].q,  # RR
        ]
        
        # 发布关节状态
        self.joint_pub.publish(self.joint_state)
        
    def publish_pointcloud(self, msg: PointCloud2_):
        """发布点云数据，使用雷达坐标系 'radar'"""
        # 创建ROS点云消息
        ros_pointcloud = PointCloud2()
        
        # 复制原始点云数据
        ros_pointcloud.header.stamp = rospy.Time.now()
        # ros_pointcloud.header.frame_id = 'base_link'  # 设置坐标系为"radar"
        ros_pointcloud.header.frame_id = 'radar'  # 设置坐标系为"radar"
        
        # 复制点云数据字段
        ros_pointcloud.height = msg.height
        ros_pointcloud.width = msg.width
        ros_pointcloud.fields = msg.fields
        ros_pointcloud.is_bigendian = msg.is_bigendian
        ros_pointcloud.point_step = msg.point_step
        ros_pointcloud.row_step = msg.row_step
        ros_pointcloud.data = msg.data
        ros_pointcloud.is_dense = msg.is_dense
        
        # 发布点云数据
        self.pointcloud_pub.publish(ros_pointcloud)
        rospy.logdebug("点云数据已发布，frame_id: radar")

def main():
    try:
        node = Go2DriverNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Node terminated by keyboard interrupt")
    except Exception as e:
        rospy.logerr(f"Fatal error in node execution: {e}")
    finally:
        rospy.signal_shutdown("Node terminated")

if __name__ == '__main__':
    main()
