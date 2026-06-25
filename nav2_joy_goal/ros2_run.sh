#!/usr/bin/env bash
# ros2_run.sh — convenience wrapper that sources the right setup.bash
# before running a ros2 command. Without this, a fresh shell can only
# see /opt/ros/humble and the nav2_joy package is invisible.
#
# Usage:
#   ./nav2_joy/ros2_run.sh pkg executables nav2_joy
#   ./nav2_joy/ros2_run.sh launch nav2_joy nav2_joy_simulation_launch.py terminal:=xterm
#   ./nav2_joy/ros2_run.sh topic list
#   ./nav2_joy/ros2_run.sh run nav2_joy joy_to_twist
#
# You can also source it from another script:
#   source /home/howard/code/navigation2/nav2_joy/ros2_run.sh --noexec

set -eo pipefail

WS_DIR="/home/howard/code/navigation2"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [ "${1:-}" = "--noexec" ]; then
  shift
  if [ -f "${ROS_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
  fi
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_DIR}/install/setup.bash"
  fi
  return 0 2>/dev/null || exit 0
fi

if [ $# -eq 0 ]; then
  cat <<EOF
Usage: $0 <ros2 subcommand> [args...]

Examples:
  $0 pkg executables nav2_joy
  $0 launch nav2_joy nav2_joy_simulation_launch.py terminal:=xterm
  $0 topic list
EOF
  exit 1
fi

if [ ! -f "${ROS_SETUP}" ]; then
  echo "ERROR: ${ROS_SETUP} not found." >&2
  exit 1
fi
# shellcheck disable=SC1090
source "${ROS_SETUP}"

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${WS_DIR}/install/setup.bash"
else
  echo "WARN: ${WS_DIR}/install/setup.bash not found." >&2
  echo "      Did you run: ./nav2_joy/build_nav2_joy.sh" >&2
fi

exec ros2 "$@"
