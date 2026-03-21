#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import sys
from geometry_msgs.msg import TransformStamped
import tf.transformations as tf_trans

class TFMatrixCalculator:
    def __init__(self):
        """初始化TF监听器"""
        rospy.init_node('tf_matrix_calculator', anonymous=True)
        
        # 创建tf2缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 等待tf树建立
        rospy.sleep(1.0)
        
    def get_transform_matrix(self, source_frame, target_frame, timeout=5.0):
        """
        获取从source_frame到target_frame的变换矩阵
        
        Args:
            source_frame (str): 源坐标系名称
            target_frame (str): 目标坐标系名称
            timeout (float): 超时时间（秒）
            
        Returns:
            numpy.ndarray: 4x4齐次变换矩阵，如果失败返回None
        """
        try:
            # 获取变换
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout)
            )
            
            # 提取平移和旋转
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # 转换为齐次变换矩阵
            trans_matrix = tf_trans.translation_matrix([
                translation.x, 
                translation.y, 
                translation.z
            ])
            
            rot_matrix = tf_trans.quaternion_matrix([
                rotation.x,
                rotation.y, 
                rotation.z,
                rotation.w
            ])
            
            # 组合变换矩阵
            homogeneous_matrix = np.dot(trans_matrix, rot_matrix)
            
            return homogeneous_matrix
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"获取变换失败: {e}")
            return None
    
    def print_matrix_info(self, matrix, source_frame, target_frame):
        """
        打印变换矩阵的详细信息
        
        Args:
            matrix (numpy.ndarray): 4x4齐次变换矩阵
            source_frame (str): 源坐标系
            target_frame (str): 目标坐标系
        """
        if matrix is None:
            print(f"无法获取从 {source_frame} 到 {target_frame} 的变换")
            return
            
        print(f"\n从 {source_frame} 到 {target_frame} 的齐次变换矩阵:")
        print("=" * 60)
        np.set_printoptions(precision=4, suppress=True)
        print(matrix)
        np.set_printoptions()  # 重置为默认设置
        
        # 提取平移部分
        translation = matrix[:3, 3]
        print(f"\n平移向量 (x, y, z): [{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}]")
        
        # 提取旋转部分并转换为欧拉角
        rotation_matrix = matrix[:3, :3]
        euler_angles = tf_trans.euler_from_matrix(matrix, 'rxyz')
        
        print(f"欧拉角 (roll, pitch, yaw): [{np.degrees(euler_angles[0]):.3f}°, {np.degrees(euler_angles[1]):.3f}°, {np.degrees(euler_angles[2]):.3f}°]")
        
        # 转换为四元数
        quaternion = tf_trans.quaternion_from_matrix(matrix)
        print(f"四元数 (x, y, z, w): [{quaternion[0]:.6f}, {quaternion[1]:.6f}, {quaternion[2]:.6f}, {quaternion[3]:.6f}]")
        
    def calculate_transforms(self, frame_a, frame_b):
        """
        计算两个坐标系之间的双向变换矩阵
        
        Args:
            frame_a (str): 坐标系A的名称
            frame_b (str): 坐标系B的名称
        """
        print(f"正在计算坐标系 '{frame_a}' 和 '{frame_b}' 之间的变换矩阵...")
        
        # 获取A到B的变换矩阵
        matrix_a_to_b = self.get_transform_matrix(frame_a, frame_b)
        
        # 获取B到A的变换矩阵
        matrix_b_to_a = self.get_transform_matrix(frame_b, frame_a)
        
        # 打印结果
        self.print_matrix_info(matrix_a_to_b, frame_a, frame_b)
        self.print_matrix_info(matrix_b_to_a, frame_b, frame_a)
        
        # 验证逆矩阵关系
        if matrix_a_to_b is not None and matrix_b_to_a is not None:
            print(f"\n验证逆矩阵关系:")
            print("=" * 60)
            
            # 计算逆矩阵
            inverse_check = np.dot(matrix_a_to_b, matrix_b_to_a)
            identity_matrix = np.eye(4)
            
            # 检查是否接近单位矩阵
            is_identity = np.allclose(inverse_check, identity_matrix, atol=1e-6)
            print(f"A到B * B到A ≈ 单位矩阵: {is_identity}")
            
            if not is_identity:
                print("矩阵乘积:")
                print(inverse_check)
        
        return matrix_a_to_b, matrix_b_to_a
    
    def list_available_frames(self):
        """列出当前可用的tf坐标系"""
        try:
            # 获取所有可用的坐标系
            frames = self.tf_buffer.all_frames_as_string()
            print("当前可用的坐标系:")
            print("=" * 40)
            print(frames)
        except Exception as e:
            rospy.logerr(f"获取坐标系列表失败: {e}")

def main():
    """主函数"""
    try:
        calculator = TFMatrixCalculator()
        
        # 检查命令行参数
        if len(sys.argv) == 1:
            # 没有参数，显示帮助信息和可用坐标系
            print("TF变换矩阵计算器")
            print("=" * 50)
            print("用法:")
            print("  python tf_matrix_calculator.py <frame_a> <frame_b>")
            print("  例如: python tf_matrix_calculator.py base_link camera_link")
            print("")
            
            calculator.list_available_frames()
            
        elif len(sys.argv) == 3:
            # 有两个参数，计算变换矩阵
            frame_a = sys.argv[1]
            frame_b = sys.argv[2]
            
            calculator.calculate_transforms(frame_a, frame_b)
            
        else:
            print("错误: 请提供正确的参数")
            print("用法: python tf_matrix_calculator.py <frame_a> <frame_b>")
            sys.exit(1)
            
    except rospy.ROSInterruptException:
        print("程序被中断")
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        rospy.logerr(f"程序运行出错: {e}")

if __name__ == '__main__':
    main()
