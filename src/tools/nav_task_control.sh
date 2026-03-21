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
    echo "Usage: $0 <start|pause|resume|home|task NAME>"
    echo "  start       -> data:1  (start loop nav)"
    echo "  pause/stop  -> data:2  (pause/stop task)"
    echo "  resume      -> data:3  (continue task)"
    echo "  home        -> data:4  (go home)"
    echo "  task NAME   -> publish String NAME to /nav_task_signal"
    exit 0
fi

cmd="$1"
case "${cmd}" in
    start|start_loop)
        signal=1
        ;;
    pause|stop|cancel)
        signal=2
        ;;
    resume|continue)
        signal=3
        ;;
    home|go_home)
        signal=4
        ;;
    task)
        if [ -z "$2" ]; then
            echo "task name required" >&2
            exit 1
        fi
        rostopic pub -1 /nav_task_signal std_msgs/String "data: '$2'"
        exit 0
        ;;
    *)
        echo "Unknown command: ${cmd}" >&2
        exit 1
        ;;
esac

rostopic pub -1 /nav_task_signal std_msgs/Int32 "data: ${signal}"
