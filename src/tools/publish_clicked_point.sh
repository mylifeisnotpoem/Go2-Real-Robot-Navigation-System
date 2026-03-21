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

if [ "$#" -lt 3 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "Usage: $0 <x> <y> <z> [frame_id]"
    echo "Environment:"
    echo "  FRAME_ID       Override default frame id (default: map)"
    echo "  CLICKED_TOPIC  Override topic name (default: /clicked_point)"
    exit 1
fi

TOPIC="${CLICKED_TOPIC:-/clicked_point}"
FRAME_ID="${FRAME_ID:-map}"

X="$1"
Y="$2"
Z="$3"

if [ -n "$4" ]; then
    FRAME_ID="$4"
fi

echo "Publishing ${TOPIC} point (${X}, ${Y}, ${Z}) in frame ${FRAME_ID}..."
rostopic pub -1 "${TOPIC}" geometry_msgs/PointStamped "{header: {frame_id: \"${FRAME_ID}\"}, point: {x: ${X}, y: ${Y}, z: ${Z}}}"

