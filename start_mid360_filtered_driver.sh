#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-65}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found" >&2
  exit 1
fi

if [ -f "${SCRIPT_DIR}/../../install/setup.bash" ]; then
  # Normal layout: <workspace>/src/livox_ros_driver2_filtered
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/../../install/setup.bash"
elif [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  # Standalone layout, useful for quick local tests.
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/install/setup.bash"
else
  echo "ERROR: no install/setup.bash found. Build this workspace first." >&2
  echo "       Example: cd ${SCRIPT_DIR} && ./build.sh ${ROS_DISTRO}" >&2
  exit 1
fi

echo "Starting filtered Livox MID360 driver"
echo "  ROS_DISTRO=${ROS_DISTRO}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "  launch=livox_ros_driver2 msg_MID360_launch.py"
echo "  output topic=/livox/lidar (embedded self-filter enabled in launch)"

exec ros2 launch livox_ros_driver2 msg_MID360_launch.py "$@"
