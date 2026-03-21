#!/bin/bash
echo "正在恢复导航模式..."

# 停止充电节点
echo "1. 停止充电系统..."
rosnode kill /charging_state_machine 2>/dev/null
rosnode kill /visual_servo_controller 2>/dev/null
sleep 1

# 启用导航速度（ROS2）
echo "2. 启用导航速度指令..."
ros2 topic pub /inspecting_state std_msgs/msg/Bool '{data: true}' -r1 &
ROS2_PID=$!
sleep 2
kill $ROS2_PID

echo "✓ 已恢复导航模式"
echo "可以重启导航规划器: roslaunch go2_navigation pure_pursuit.launch"