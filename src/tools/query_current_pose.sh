#!/bin/bash
set -e

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "${PROJECT_DIR}/../.." && pwd )"
SETUP_BASH="${WORKSPACE_DIR}/devel/setup.bash"

if [ ! -f "${SETUP_BASH}" ]; then
    echo "ROS environment not found: ${SETUP_BASH}" >&2
    exit 1
fi

source "${SETUP_BASH}"

if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "Usage: $0 [topic]"
    echo "  Default topic: \${POSE_TOPIC:-/baselink2map_kalman}"
    echo "Example: $0 /baselink2map"
    exit 0
fi

TOPIC="${POSE_TOPIC:-/baselink2map_kalman}"
if [ -n "$1" ]; then
    TOPIC="$1"
fi

POS=$(rostopic echo -n1 "${TOPIC}/pose/pose/position" | awk '/x:/{x=$2}/y:/{y=$2}/z:/{z=$2}END{if(x==""||y==""||z==""){exit 1}else{print x" "y" "z}}')
echo "${POS}"

