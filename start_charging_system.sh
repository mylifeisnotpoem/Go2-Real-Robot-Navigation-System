#!/bin/bash

# GO2视觉充电系统 - 一键启动脚本
# 功能：自动启动底层运控、SLAM、视觉充电系统

set -e  # 遇到错误立即退出

echo "=========================================="
echo "  GO2视觉充电系统 - 自动启动"
echo "=========================================="


# ==================== 步骤2: 启动SLAM系统 ====================
echo ""
echo "[2/4] 启动Fast-LIO SLAM系统..."
source ~/ws_nav/devel/setup.bash
roslaunch fast_lio_slam mapping_mid360.launch &
SLAM_PID=$!
sleep 5

# 验证SLAM是否正常
if rostopic list | grep -q "/cloud_registered"; then
    echo "✓ SLAM系统启动成功"
else
    echo "✗ SLAM系统启动失败"
    kill $SLAM_PID 2>/dev/null
    exit 1
fi

# ==================== 步骤3: 启动视觉充电系统 ====================
echo ""
echo "[3/4] 启动视觉充电系统..."
roslaunch go2_vision_charging auto_charge_demo.launch &
CHARGING_PID=$!
sleep 3

# 验证充电系统是否正常
if rostopic list | grep -q "/charging_state"; then
    echo "✓ 视觉充电系统启动成功"
else
    echo "✗ 视觉充电系统启动失败"
    kill $CHARGING_PID 2>/dev/null
    exit 1
fi

# ==================== 步骤4: 切换到充电模式 ====================
echo ""
echo "[4/4] 切换到充电模式..."

# 停止导航节点（如果在运行）
rosnode kill /pure_pursuit_local_planner 2>/dev/null || true
rosnode kill /keyboard_control_node 2>/dev/null || true
sleep 1

# 禁用导航速度指令
ros2 topic pub /inspecting_state std_msgs/msg/Bool '{data: false}' -r1 &
ROS2_PUB_PID=$!
sleep 2
kill $ROS2_PUB_PID

# 验证charging_cmd_vel控制权
echo ""
echo "=========================================="
echo "验证系统状态..."
echo "=========================================="
PUBLISHERS=$(rostopic info /charging_cmd_vel | grep Publishers -A 10)
echo "$PUBLISHERS"

if echo "$PUBLISHERS" | grep -q "charging_state_machine"; then
    echo ""
    echo "=========================================="
    echo "✓✓✓ 所有系统启动成功 ✓✓✓"
    echo "=========================================="
    echo "充电系统已获得控制权"
    echo "请将机器人移至充电座前方 (0.5-2.5米)"
    echo ""
    echo "监控命令:"
    echo "  - 查看充电状态: rostopic echo /charging_state"
    echo "  - 查看相机图像: rosrun rqt_image_view rqt_image_view"
    echo "  - 查看充电目标: rostopic echo /charging_goal_base"
    echo "=========================================="
else
    echo ""
    echo "⚠ 警告: 充电系统未完全获得控制权"
    echo "请检查是否还有其他节点在发布/cmd_vel"
fi

# 保持脚本运行（否则后台进程会被终止）
echo ""
echo "按 Ctrl+C 停止所有系统"
wait