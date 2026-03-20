#!/bin/bash
set -e

echo "[START] Fast-LIO2 Map Tools"

# ===== 1. 参数处理 =====
# 第一个参数作为输出基目录
OUTPUT_BASE_DIR="$1"
echo "目标目录: ${OUTPUT_BASE_DIR}"
if [ -z "$OUTPUT_BASE_DIR" ]; then
  echo "用法: $0 <output_dir>"
  echo "示例: $0 /data/maps"
  exit 1
fi

# ===== 2. 路径计算 =====
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "${PROJECT_DIR}/../.." && pwd )"

source "${WORKSPACE_DIR}/devel/setup.bash"

# ===== 3. 输出目录（带时间戳） =====
CURRENT_DATE="$(date +'%Y-%m-%d_%H-%M-%S')"
MAP_DIR="${OUTPUT_BASE_DIR}"

echo "输出目录: ${MAP_DIR}"
mkdir -p "${MAP_DIR}"

# ===== 4. 执行转换 =====
"${PROJECT_DIR}/common/bin/pcd_to_pgm_cpp" \
  --pcd "${PROJECT_DIR}/slam_3d/PGO/PCD/aft_pgo_cloud.pcd" \
  --out "${MAP_DIR}" \
  --resolution 0.05 \
  --z-min -0.5 \
  --z-max 0.5 \
  --radius 0.08 \
  --min-neighbors 10 \
  --min-points-per-cell 8 \
  --inflate 0

echo "PCD 转 PGM 处理完成。"

