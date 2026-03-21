#!/bin/bash
set -e
export DISPLAY=:0
echo "[START] Voxel-SLAM Map Tools"

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "${PROJECT_DIR}/../.." && pwd )"

source "${WORKSPACE_DIR}/devel/setup.bash"

# Voxel-SLAM 配置
VOXEL_SLAM_SAVE_PATH="/root/ws_nav/maps/mid360_init"
MERGED_MAP_FILE="/root/ws_nav/maps/voxel_merged/voxel_merged.pcd"

copy_map() {
    local src="$1"
    local dest="$2"
    echo "Copying $(basename "$src") -> $(dirname "$dest")"
    mkdir -p "$(dirname "$dest")"
    cp "$src" "$dest"
}

merge_voxel_maps() {
    echo "Merging Voxel-SLAM point cloud files..."
    mkdir -p "$(dirname "$MERGED_MAP_FILE")"
    
    if [ ! -d "$VOXEL_SLAM_SAVE_PATH" ]; then
        echo "错误: Voxel-SLAM 地图目录不存在: $VOXEL_SLAM_SAVE_PATH"
        echo "请先运行 Voxel-SLAM 并完成建图 (rosparam set finish true)"
        exit 1
    fi
    
    local pcd_count=$(find "$VOXEL_SLAM_SAVE_PATH" -name "*.pcd" 2>/dev/null | wc -l)
    if [ "$pcd_count" -eq 0 ]; then
        echo "错误: 未找到任何 PCD 文件"
        echo "提示: 确保 Voxel-SLAM 运行时设置 is_save_map: 1"
        exit 1
    fi
    
    echo "找到 $pcd_count 个 PCD 文件，正在合并..."
    
    # 使用 combine_pcds 合并所有点云（需要位姿文件）
    local POSE_FILE="$VOXEL_SLAM_SAVE_PATH/alidarState.txt"
    local COMBINE_PCDS="${PROJECT_DIR}/combine_pcds"
    
    if [ ! -f "$COMBINE_PCDS" ]; then
        echo "错误: combine_pcds 工具不存在: $COMBINE_PCDS"
        exit 1
    fi
    
    if [ ! -f "$POSE_FILE" ]; then
        echo "错误: 位姿文件不存在: $POSE_FILE"
        echo "提示: 确保 Voxel-SLAM 保存了位姿文件"
        exit 1
    fi
    
    echo "使用 combine_pcds 工具合并点云..."
    echo "位姿文件: $POSE_FILE"
    echo "PCD文件夹: $VOXEL_SLAM_SAVE_PATH"
    
    cd "$(dirname "$MERGED_MAP_FILE")"
    "$COMBINE_PCDS" "$POSE_FILE" "$VOXEL_SLAM_SAVE_PATH"
    
    if [ -f "combined.pcd" ]; then
        mv combined.pcd "$(basename "$MERGED_MAP_FILE")"
        echo "✓ 点云合并成功: $MERGED_MAP_FILE"
    else
        echo "✗ 合并失败"
        exit 1
    fi
}


extract_ground_from_map() {
    local input_pcd="$1"
    local output_pcd="$2"
    
    echo ""
    echo "========================================"
    echo "提取可行区域（地面点云）..."
    echo "方法: 参考 PGO 的 height_threshold 过滤"
    echo "========================================"
    
    # 参考 PGO 的地面分割逻辑：
    # 1. Patchwork++ 初步分割地面
    # 2. height_threshold = 0.1m 过滤高点
    # 这里使用 Z 值范围等效实现该逻辑
    local Z_MIN=-0.5  # 包含轻微凹陷的地面
    local Z_MAX=0.3   # 过滤高于30cm的障碍物（比PGO的0.1m更宽松）
    
    if command -v pcl_passthrough_filter &> /dev/null; then
        echo "使用 PCL PassThrough 滤波器（Z轴范围: ${Z_MIN}m ~ ${Z_MAX}m）"
        echo "输入: $input_pcd"
        echo "输出: $output_pcd"
        
        pcl_passthrough_filter "$input_pcd" "$output_pcd" -field z -min "$Z_MIN" -max "$Z_MAX"
        
        if [ $? -eq 0 ] && [ -f "$output_pcd" ]; then
            echo "✓ 可行区域提取成功: $(basename "$output_pcd")"
            return 0
        else
            echo "✗ PCL PassThrough 滤波失败"
            echo "  尝试备用方案..."
        fi
    fi
    
    # 备用方案：如果PCL工具不可用，直接复制全局地图
    echo "⚠ PCL工具不可用，使用全局地图作为可行区域"
    echo "  建议安装: sudo apt-get install pcl-tools"
    cp "$input_pcd" "$output_pcd"
    echo "! 注意: 未进行地面分割，建议安装PCL工具后重新处理"
    return 0
}


# 1. 合并 Voxel-SLAM 的多个 PCD 文件
merge_voxel_maps

# 2. 提取可行区域（参考 PGO 的地面分割方法）
FREE_SPACE_FILE="/root/ws_nav/maps/voxel_merged/voxel_free_space.pcd"
extract_ground_from_map "$MERGED_MAP_FILE" "$FREE_SPACE_FILE"

# 3. 复制合并后的地图到定位模块和建图模块
echo ""
echo "复制地图到定位模块..."
copy_map "$MERGED_MAP_FILE" \
          "${PROJECT_DIR}/navigation_3d/fast_lio_nav/open3d_loc/map/aft_pgo_cloud.pcd"
copy_map "$MERGED_MAP_FILE" \
          "${PROJECT_DIR}/slam_3d/PGO/PCD/aft_pgo_cloud.pcd"

# 4. 复制可行区域到全局规划器（与 PGO 保持一致）
echo "复制可行区域到全局规划器..."
copy_map "$FREE_SPACE_FILE" \
          "${PROJECT_DIR}/navigation_3d/global_planner/pcd_map/pcd/aft_pgo_free_space.pcd"

# 5. 复制拓扑地图目标点文件
copy_map "${PROJECT_DIR}/navigation_3d/global_planner/topology_rviz_plugin/map/generated_map_target_points.txt" \
          "${PROJECT_DIR}/common/user_interface/config/generated_map_target_points.txt"

# 6. 处理拓扑图
echo "Processing topology graph..."
cd "${PROJECT_DIR}/navigation_3d/global_planner/topology_rviz_plugin/scripts"
python3 topology_connect.py --threshold 0.8 --stats
cd - > /dev/null

echo ""
echo "拓扑图处理完成。"
echo "========================================"
echo "Voxel-SLAM 地图处理完成！"
echo "全局地图: $MERGED_MAP_FILE"
echo "可行区域: $FREE_SPACE_FILE"
echo "========================================"
echo ""
echo "正在启动拓扑编辑器 (前台运行)..."
roslaunch topology_rviz_plugin topology_tool_demo.launch