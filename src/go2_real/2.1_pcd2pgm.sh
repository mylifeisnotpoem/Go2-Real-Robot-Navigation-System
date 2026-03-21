#!/bin/bash
set -e

echo "[START] Fast-LIO2 Map Tools"

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "${PROJECT_DIR}/../.." && pwd )"

source "${WORKSPACE_DIR}/devel/setup.bash"

CURRENT_DATE="$(date +'%Y-%m-%d_%H-%M-%S')"
echo "当前日期字符串: ${CURRENT_DATE}"
MAP_DIR="${HOME}/maps/${CURRENT_DATE}"
mkdir -p "${MAP_DIR}"
${PROJECT_DIR}/common/bin/pcd_to_pgm_cpp --pcd "${PROJECT_DIR}/slam_3d/PGO/PCD/aft_pgo_cloud.pcd" --out "${MAP_DIR}" --resolution 0.05 --z-min -0.5 --z-max 0.5 --radius 0.08 --min-neighbors 10 --min-points-per-cell 8 --inflate 0

echo "PCD 转 PGM 处理完成。"