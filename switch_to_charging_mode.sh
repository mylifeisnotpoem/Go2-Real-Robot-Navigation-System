#!/bin/bash
echo "正在切换到充电模式..."

# 停止导航节点
echo "1. 停止导航系统..."
rosnode kill /pure_pursuit_local_planner 2>/dev/null
rosnode kill /keyboard_control_node 2>/dev/null
sleep 1

# 禁用导航速度（ROS2）
echo "2. 禁用导航速度指令..."
ros2 topic pub /inspecting_state std_msgs/msg/Bool '{data: false}' -r1 &
ROS2_PID=$!
sleep 2
kill $ROS2_PID

# 检查cmd_vel发布者
echo "3. 验证速度指令权限..."
PUBLISHERS=$(rostopic info /charging_cmd_vel | grep Publishers -A 10)
echo "$PUBLISHERS"

if echo "$PUBLISHERS" | grep -q "charging_state_machine"; then
    echo "✓ 充电系统已获得控制权"
    echo "✓ 可以开始充电流程"
else
    echo "⚠ 警告: 充电系统未正确连接"
fi

echo "完成! 请将机器人移至充电座前方"