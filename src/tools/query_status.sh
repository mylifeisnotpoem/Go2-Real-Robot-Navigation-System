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

NAV_STATE_TOPIC="${NAV_STATE_TOPIC:-/nav_state}"
CONF_TOPIC="${CONF_TOPIC:-/localization_3d_confidence}"
DELAY_TOPIC="${DELAY_TOPIC:-/localization_3d_delay_ms}"

nav_state=$(rostopic echo -n1 "${NAV_STATE_TOPIC}" 2>/dev/null | awk '/data:/{print $2; exit}')
conf=$(rostopic echo -n1 "${CONF_TOPIC}" 2>/dev/null | awk '/data:/{print $2; exit}')
delay=$(rostopic echo -n1 "${DELAY_TOPIC}" 2>/dev/null | awk '/data:/{print $2; exit}')

echo "nav_state=${nav_state}"
echo "localization_confidence=${conf}"
echo "localization_delay_ms=${delay}"
