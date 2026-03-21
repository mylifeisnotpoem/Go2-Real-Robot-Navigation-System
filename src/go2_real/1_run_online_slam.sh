#!/bin/bash
set -e

export DISPLAY=:0

PID_DRIVE=""
PID_VOXEL=""
PID_TOPO=""

stop_node() {
	local name="$1"
	local pid="$2"
	local grace="${3:-5}"

	if [ -z "$pid" ] || ! kill -0 "$pid" >/dev/null 2>&1; then
		return
	fi

	echo "Stopping ${name} (PID=${pid})..."
	kill -SIGINT "$pid"

	for _ in $(seq 1 "$grace"); do
		if ! kill -0 "$pid" >/dev/null 2>&1; then
			echo "${name} stopped cleanly."
			return
		fi
		sleep 1
	done

	echo "${name} did not exit in ${grace}s, forcing shutdown..."
	kill -SIGTERM "$pid"
}

cleanup() {
	echo "正在关闭所有节点..."
	stop_node "Topology Generator" "$PID_TOPO" 5
	stop_node "Voxel-SLAM" "$PID_VOXEL" 10
	stop_node "Drive" "$PID_DRIVE" 5
}

trap cleanup INT TERM EXIT

# 自动 source 环境
source /root/ws_nav/devel/setup.bash

echo "[START] Voxel-SLAM System"

# 0. 启动机器人驱动 (后台)
echo "Starting robot drive..."
roslaunch go2_ros_control common_robot_slam.launch > /dev/null 2>&1 &
PID_DRIVE=$!
sleep 2

# 1. 启动 Voxel-SLAM (后台)
echo "Starting Voxel-SLAM..."
roslaunch voxel_slam vxlm_mid360.launch rviz:=false &
PID_VOXEL=$!
sleep 3

# 2. 启动 IMU Filter (可选，Voxel-SLAM内部已处理)
# echo "Starting IMU Filter..."
# roslaunch imu_filter_madgwick imu_filter_madgwick.launch > /dev/null 2>&1 &
# PID_IMU=$!
# sleep 1

# 3. 启动拓扑地图生成器 (后台)
echo "Starting Topology Generator..."
roslaunch topology_map_generator topology_map_generator.launch > /dev/null 2>&1 &
PID_TOPO=$!
sleep 1

echo "所有后台节点已启动！"
echo "-----------------------------------"
echo "Voxel-SLAM 特殊命令："
echo "  - 完成建图后执行全局优化: rosparam set finish true"
echo "-----------------------------------"
echo "正在启动键盘控制..."

# 4. 启动键盘控制 (前台运行)
rosrun user_interface common_keyboard_node

# 清理由 trap 负责
exit 0
