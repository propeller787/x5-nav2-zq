#!/usr/bin/env bash
# 一键启动 MPPI house SLAM demo, 含死机后的环境清理.
set -eo pipefail

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# 死机后 GPU 驱动可能暂时不可用; 软件渲染保证 Gazebo 能启动.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

NAV2_WS="/home/howard/code/navigation2"
LAUNCH_FILE="${NAV2_WS}/nav2_mppi_controller/launch/mppi_house_slam_demo.launch.py"

echo "[run_mppi_house_slam_demo] cleaning stale Gazebo/ROS demo processes..."
for sig in INT TERM; do
  pkill -"$sig" -f "mppi_house_slam_demo.launch.py" 2>/dev/null || true
done
sleep 1
if pgrep -x gzserver >/dev/null || pgrep -x gzclient >/dev/null; then
  echo "[run_mppi_house_slam_demo] killing leftover gzserver/gzclient..."
  killall -9 gzserver gzclient 2>/dev/null || true
  sleep 1
fi

source /opt/ros/humble/setup.bash
source "${NAV2_WS}/install/setup.bash"
ros2 daemon stop 2>/dev/null || true
ros2 daemon start

echo "[run_mppi_house_slam_demo] launching demo..."
exec ros2 launch "${LAUNCH_FILE}" "$@"
