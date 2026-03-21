#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import json
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

# ================= 配置区域 =================
# 目标 IP (接收端的 IP 地址)
# 如果是发给宿主机，通常是宿主机的局域网 IP (如 192.168.1.x)
# 或者 Docker 网桥网关 (如 172.17.0.1)
TARGET_IP = "127.0.0.1"

# 目标端口 (接收端监听的端口)
TARGET_PORT = 5005
# ===========================================

# 全局 socket，供回调与收包线程共用
sock = None

def charging_cmd_vel_callback(msg):
    """
    回调：收到 /charging_cmd_vel (Twist) 时触发
    """
    try:
        data_payload = {
            "t": "charging_cmd_vel",
            "v": round(msg.linear.x, 3),
            "w": round(msg.angular.z, 3)
        }
        message = json.dumps(data_payload).encode('utf-8')
        sock.sendto(message, (TARGET_IP, TARGET_PORT))
    except Exception as e:
        rospy.logerr("UDP Send Error (charging_cmd_vel): %s", e)


def charging_state_callback(msg):
    """
    回调：收到 /charging_state (String) 时触发（例如：SEARCHING）
    """
    try:
        message = msg.data.encode('utf-8')
        sock.sendto(message, (TARGET_IP, TARGET_PORT))
    except Exception as e:
        rospy.logerr("UDP Send Error (charging_state): %s", e)


def cmd_vel_callback(msg):
    """
    回调函数：收到 ROS /cmd_vel 消息时触发
    """
    try:
        data_payload = {
            "v": round(msg.linear.x, 3),
            "w": round(msg.angular.z, 3)
        }
        message = json.dumps(data_payload).encode('utf-8')
        sock.sendto(message, (TARGET_IP, TARGET_PORT))
    except Exception as e:
        rospy.logerr("UDP Send Error: %s", e)


def inspection_task_json_new_callback(msg):
    """
    回调：收到 /weilai/robot_id/inspection_task_json_new 时，按协议发往宿主机 5005
    """
    try:
        data_payload = {"t": "task_new", "data": msg.data}
        message = json.dumps(data_payload, ensure_ascii=False).encode('utf-8')
        sock.sendto(message, (TARGET_IP, TARGET_PORT))
    except Exception as e:
        rospy.logerr("UDP Send Error (task_new): %s", e)


def recv_task_done_loop(pub_done):
    """
    在另一线程中从同一 socket 收包；若 JSON 含 "done" 则发布到 inspection_task_done
    """
    sock.settimeout(0.2)
    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(4096)
            payload = json.loads(data.decode('utf-8'))
            if 'done' in payload:
                pub_done.publish(Int32(data=int(payload['done'])))
        except socket.timeout:
            continue
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            rospy.logdebug("recv_task_done parse skip: %s", e)
            continue
        except Exception as e:
            if not rospy.is_shutdown():
                rospy.logerr("recv_task_done_loop: %s", e)
            break


if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rospy.init_node('cmd_vel_to_udp_sender', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback, queue_size=1)
    rospy.Subscriber(
        "/weilai/robot_id/inspection_task_json_new",
        String,
        inspection_task_json_new_callback,
        queue_size=1
    )

    pub_done = rospy.Publisher(
        "/weilai/robot_id/inspection_task_done",
        Int32,
        queue_size=1
    )

    rospy.Subscriber("/charging_cmd_vel", Twist, charging_cmd_vel_callback, queue_size=1)
    rospy.Subscriber("/charging_state", String, charging_state_callback, queue_size=1)

    recv_thread = threading.Thread(target=recv_task_done_loop, args=(pub_done,))
    recv_thread.daemon = True
    recv_thread.start()

    rospy.loginfo("UDP Bridge Started. Targeting %s:%d (cmd_vel + inspection_task)", TARGET_IP, TARGET_PORT)

    rospy.spin()

    sock.close()
