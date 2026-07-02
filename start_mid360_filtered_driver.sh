#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPECTED_PREFIX="${SCRIPT_DIR}/install/livox_ros_driver2"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-65}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# Keep this fork isolated from previously sourced workspaces.
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH

source /opt/ros/humble/setup.bash

if [ ! -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  echo "ERROR: ${SCRIPT_DIR}/install/setup.bash not found" >&2
  echo "Run: cd ${SCRIPT_DIR} && ./build.sh" >&2
  exit 1
fi
source "${SCRIPT_DIR}/install/setup.bash"

PACKAGE_PREFIX="$(ros2 pkg prefix livox_ros_driver2)"
if [ "${PACKAGE_PREFIX}" != "${EXPECTED_PREFIX}" ]; then
  echo "ERROR: wrong livox_ros_driver2 package selected" >&2
  echo "  expected: ${EXPECTED_PREFIX}" >&2
  echo "  actual:   ${PACKAGE_PREFIX}" >&2
  echo "Clean and rebuild this fork:" >&2
  echo "  cd ${SCRIPT_DIR} && rm -rf build install log && ./build.sh" >&2
  exit 1
fi

echo "Starting filtered Livox MID360 driver"
echo "  ROS_DISTRO=humble"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "  package_prefix=${PACKAGE_PREFIX}"
echo "  topic=/livox/lidar"

exec ros2 launch "${SCRIPT_DIR}/launch_ROS2/msg_MID360_launch.py" "$@"
