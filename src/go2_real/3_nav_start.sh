#!/bin/bash
set -e
export DISPLAY=:0

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "${PROJECT_DIR}/../.." && pwd )"
LOG_ROOT_DIR="$(dirname "$0")/logs"
LOG_SLOT_FILE="${LOG_ROOT_DIR}/.run_slot"
MAX_LOG_RUNS=10
MAX_SINGLE_LOG_SIZE=$((500 * 1024 * 1024))
RUN_LOG_TS="$(date '+%Y%m%d_%H%M%S')"

mkdir -p "${LOG_ROOT_DIR}"

if [ -f "${LOG_SLOT_FILE}" ]; then
    RUN_LOG_SLOT="$(cat "${LOG_SLOT_FILE}")"
else
    RUN_LOG_SLOT=0
fi

if ! [[ "${RUN_LOG_SLOT}" =~ ^[0-9]+$ ]]; then
    RUN_LOG_SLOT=0
fi

RUN_LOG_SLOT=$((RUN_LOG_SLOT % MAX_LOG_RUNS))
RUN_LOG_SLOT_PADDED=$(printf '%02d' "${RUN_LOG_SLOT}")

for old_dir in "${LOG_ROOT_DIR}/slot_${RUN_LOG_SLOT_PADDED}_"*; do
    if [ -e "${old_dir}" ]; then
        rm -rf "${old_dir}"
    fi
done

RUN_LOG_DIR="${LOG_ROOT_DIR}/slot_${RUN_LOG_SLOT_PADDED}_${RUN_LOG_TS}"
mkdir -p "${RUN_LOG_DIR}"
echo $(((RUN_LOG_SLOT + 1) % MAX_LOG_RUNS)) > "${LOG_SLOT_FILE}"

sanitize_log() {
    sed -u -r \
        -e 's/\x1B\[[0-9;?]*[ -/]*[@-~]//g' \
        -e 's/\x1B\][^\a]*(\a|\x1B\\)//g'
}

write_log_with_cap() {
    local logfile="$1"
    local current_size=0

    if [ -f "${logfile}" ]; then
        current_size=$(wc -c < "${logfile}" 2>/dev/null || echo 0)
    fi

    while IFS= read -r line || [ -n "${line}" ]; do
        local line_size=$(( ${#line} + 1 ))
        if [ $((current_size + line_size)) -gt "${MAX_SINGLE_LOG_SIZE}" ]; then
            : > "${logfile}"
            current_size=0
        fi
        printf '%s\n' "${line}" >> "${logfile}"
        current_size=$((current_size + line_size))
    done
}

STARTUP_LOG="${RUN_LOG_DIR}/startup.log"
exec > >(sanitize_log | tee >(write_log_with_cap "${STARTUP_LOG}")) 2>&1

echo "[START] Fast-LIO2 Navigation"
source "${WORKSPACE_DIR}/devel/setup.bash"

PIDS=()

cleanup() {
    if [ ${#PIDS[@]} -gt 0 ]; then
        echo "Stopping background nodes..."
        kill "${PIDS[@]}" 2>/dev/null || true
    fi
}

trap cleanup EXIT

echo "Logs will be written to: ${RUN_LOG_DIR}"
echo "Startup log: ${STARTUP_LOG}"

launch_bg() {
    local name="$1"
    shift

    echo "Starting ${name}..."
    "$@" > >(sanitize_log | write_log_with_cap "${RUN_LOG_DIR}/${name// /_}.log") 2>&1 &

    PIDS+=($!)
}

launch_bg "robot navigation bringup" roslaunch go2_ros_control common_robot_nav.launch
sleep 6



launch_bg "Livox point cloud" roslaunch livox2pointcloud livox2pointcloud.launch
sleep 3

# 启动IMU滤波 + 楼梯状态发布（/stairs_flag）
launch_bg "IMU stairs detector" roslaunch imu_filter_madgwick imu_filter_madgwick.launch
sleep 2




# ========== HDL 全局定位（用于获取初始位姿） ==========
echo "=========================================="
echo "Starting HDL Global Localization..."
echo "=========================================="

# 定义路径
PCD_INPUT="/root/ws_nav/src/go2_real/navigation_3d/fast_lio_nav/open3d_loc/map/aft_pgo_cloud.pcd"

## 这里其实是下采样原地图，并不是free，，名字懒得改了
PCD_OUTPUT="/root/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pcd/voxel_free_space_downsampled.pcd"

# 检查输入文件是否存在
if [ ! -f "${PCD_INPUT}" ]; then
    echo "ERROR: Input PCD file not found: ${PCD_INPUT}"
    exit 1
fi

# 执行降采样并检查结果
# 这里下采样的值可以根据地图大小调整
echo "Downsampling PCD file..."
if pcl_voxel_grid "${PCD_INPUT}" "${PCD_OUTPUT}" -leaf 0.2,0.2,0.2; then  
    echo "Downsampling successful!"
    ls -lh "${PCD_OUTPUT}"
else
    echo "ERROR: Failed to downsample PCD file"
    exit 1
fi

# 验证输出文件
if [ ! -f "${PCD_OUTPUT}" ]; then
    echo "ERROR: Downsampled PCD file was not created"
    exit 1
fi

MAP_PCD_PATH="${PCD_OUTPUT}"
echo "Loading global map from: ${MAP_PCD_PATH}"

# 发布全局地图点云
launch_bg "Global map publisher" rosrun pcl_ros pcd_to_pointcloud ${MAP_PCD_PATH} 0.5 _frame_id:=map cloud_pcd:=/globalmap

# 等待地图topic可用
echo "Waiting for global map topic..."
TIMEOUT=30
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if rostopic list | grep -q "/globalmap"; then
        echo "Global map topic is available!"
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

if [ $ELAPSED -ge $TIMEOUT ]; then
    echo "ERROR: Global map topic not available after ${TIMEOUT}s"
    exit 1
fi

# 等待地图数据真正发布（检查消息数量）
echo "Waiting for map data to be published..."
sleep 3
MAP_MSG_COUNT=$(rostopic echo /globalmap -n 1 --noarr 2>/dev/null | wc -l)
if [ "$MAP_MSG_COUNT" -eq 0 ]; then
    echo "WARNING: No messages received on /globalmap"
fi

# 启动自动初始位姿节点
launch_bg "Auto initial pose" roslaunch hdl_global_localization auto_initialpose.launch \
    map_topic:=/globalmap \
    cloud_topic:=/livox/pointcloud2 \
    auto_mode:=false \
    max_candidates:=1

# 等待所需的两个服务都就绪
echo "Waiting for HDL services to be ready..."
SERVICES_TO_CHECK=(
    "/hdl_global_localization/query"
    "/auto_initialpose/trigger"
)

for service in "${SERVICES_TO_CHECK[@]}"; do
    TIMEOUT=30
    ELAPSED=0
    echo "Checking service: ${service}"
    while [ $ELAPSED -lt $TIMEOUT ]; do
        if rosservice list | grep -q "${service}"; then
            echo "  ✓ ${service} ready!"
            break
        fi
        sleep 1
        ELAPSED=$((ELAPSED + 1))
    done
    
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "ERROR: Service ${service} not ready after ${TIMEOUT}s"
        exit 1
    fi
done

# 确保Livox点云数据可用
echo "Checking Livox point cloud availability..."
if timeout 15 rostopic echo /livox/pointcloud2 -n 1 --noarr 2>/dev/null | grep -q "header"; then
    echo "Livox point cloud is publishing!"
else
    echo "WARNING: Livox point cloud not detected, continue startup and relocalize later"
fi

# 额外等待，确保所有节点完全初始化
echo "Waiting for nodes to fully initialize..."
sleep 5

# 执行全局定位
LOCALIZATION_SUCCESS=false
AUTO_POSE_LOG="${RUN_LOG_DIR}/auto_initialpose.py.log"

# 触发全局定位（Trigger 无参数）
RESP=$(rosservice call /auto_initialpose/trigger 2>&1)
RET=$?

# 将本次 trigger 返回同时输出到终端并追加到独立日志（按 python 脚本名）
{
    echo "[$(date '+%F %T')] rosservice call /auto_initialpose/trigger"
    echo "$RESP"
} | sanitize_log | tee >(write_log_with_cap "$AUTO_POSE_LOG")

if [ $RET -eq 0 ]; then
    echo "Trigger service returned:"
    echo "$RESP"

    # 先根据 Trigger 的返回判断是否成功
    if echo "$RESP" | grep -q "success: True"; then
        echo "✓ Global localization triggered and reported success."
        LOCALIZATION_SUCCESS=true
    else
        echo "WARNING: Trigger returned success=False"
        # 继续往下检查 /initialpose 是否有发布（有些情况下服务返回false但仍可能发布）
    fi

    # 等待定位结果（可选：如果你觉得服务返回已经足够，就可以把这段删掉）
    echo "Waiting for localization to complete..."
    sleep 2

    # 检查是否成功发布了初始位姿
    echo "Checking if initial pose was published..."
    if timeout 5 rostopic echo /initialpose -n 1 --noarr 2>/dev/null | grep -q "pose"; then
        echo "✓ Initial pose received on /initialpose."
        LOCALIZATION_SUCCESS=true
    else
        echo "WARNING: Did not receive /initialpose within 5s"
    fi

    if [ "$LOCALIZATION_SUCCESS" = false ]; then
        echo "WARNING: Global localization may not have completed successfully"
        echo "Please check if the robot's initial position matches the map"
    fi
else
    echo "ERROR: Failed to call /auto_initialpose/trigger"
    echo "$RESP"
    # exit 1
fi


echo "=========================================="
echo "HDL Global Localization setup complete"
echo "Continuing with navigation startup..."
echo "=========================================="


launch_bg "3D localization" roslaunch open3d_loc localization_3d_go2.launch
sleep 1



launch_bg "pointcloud to laserscan" roslaunch pointcloud_to_laserscan sample_node.launch
sleep 1
# launch_bg "local costmap" roslaunch costmap_2d local_costmap.launch
# sleep 1
launch_bg "local planner costmap" roslaunch local_planner costmap_2d.launch
sleep 1
launch_bg "path smoother" roslaunch path_smoother test_with_smoother.launch
sleep 1
launch_bg "pure pursuit" roslaunch pure_pursuit_local_planner pure_pursuit_local_planner.launch
sleep 1
launch_bg "dual radar extractor" roslaunch dual_radar_traversable_extractor dual_radar_extractor.launch
sleep 3
launch_bg "target points visualizer" roslaunch topology_rviz_plugin target_points_visualizer.launch
sleep 1
launch_bg "navigation state machine" roslaunch user_interface navigation_task_state_machine.launch
sleep 1
launch_bg "photograph service" roslaunch user_interface photograph.launch
sleep 1

# === 新增：启动 UDP cmd_vel 发送程序 ===
# 使用绝对路径确保能找到文件，并在后台运行
launch_bg "UDP cmd_vel sender" python3 "/root/ws_nav/src/go2_real/common/go2_drive/go2_ros_control/scripts/cmd_vel_udp.py"
sleep 1
# ====================================



echo "All background nodes are up. Launching navigation GUI (foreground)..."
python3 "${PROJECT_DIR}/common/user_interface/scripts/navigation_control_cli.py"
