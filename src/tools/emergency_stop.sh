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

if [ $# -lt 1 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "Usage: $0 <on|off>"
    echo "  on  -> emergency stop TRUE"
    echo "  off -> emergency stop FALSE"
    exit 0
fi

case "$1" in
    on|true)
        state=true
        ;;
    off|false)
        state=false
        ;;
    *)
        echo "Unknown option: $1" >&2
        exit 1
        ;;
esac

rostopic pub -1 /emergency_stop std_msgs/Bool "data: ${state}"
