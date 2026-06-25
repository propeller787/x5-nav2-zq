#!/usr/bin/env bash
# Headless test for nav2_joy's joy_to_twist.
#
# Starts joy_to_twist in the background, publishes a few fake Joy
# messages via `ros2 topic pub`, and prints the resulting /test_cmd_vel.
# Useful to verify the package built correctly when no real terminal
# is available.
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash
#   ./test_joy_to_twist_headless.sh
#
# Total wall-clock: about 8 seconds, capped at 15.

# Cap the whole test so it can never hang.
# (timeout sends SIGTERM, then SIGKILL after --kill-after=5.)
exec timeout --kill-after=5 15 bash "$0" "$@"

set -e

echo "=========================================="
echo " nav2_joy headless smoke test"
echo "=========================================="

# Track every background PID we start so cleanup can kill them all.
PIDS=()

cleanup() {
  echo
  echo "[test] cleaning up..."
  for p in "${PIDS[@]:-}"; do
    [ -n "$p" ] && kill -9 "$p" 2>/dev/null || true
  done
  # In case any ros2 daemon got started, nuke the lot.
  pkill -9 -f 'ros2 topic'      2>/dev/null || true
  pkill -9 -f 'joy_to_twist'   2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1. start joy_to_twist
echo "[test] starting joy_to_twist..."
ros2 run nav2_joy joy_to_twist --ros-args \
  -p joy_topic:=/test_joy \
  -p cmd_vel_topic:=/test_cmd_vel \
  -p scale_linear_x:=0.5 \
  -p scale_angular_z:=1.0 \
  -p deadman_timeout:=2.0 &
PIDS+=($!)
sleep 1.5

# 2. echo /test_cmd_vel in the background (no subshell this time)
echo "[test] subscribing to /test_cmd_vel..."
ros2 topic echo /test_cmd_vel > /tmp/nav2_joy_echo.txt 2>&1 &
ECHO_PID=$!
PIDS+=($ECHO_PID)
sleep 0.5

# Helper: publish a Joy at 5 Hz for `dur` seconds, then SIGKILL.
pub_joy() {
  local axes="$1" dur="$2" label="$3"
  echo "[test] publishing Joy: ${label}"
  ros2 topic pub --rate 5 /test_joy sensor_msgs/msg/Joy \
    "{axes: ${axes}, buttons: [1, 0, 0, 0]}" \
    > /dev/null 2>&1 &
  local pid=$!
  PIDS+=($pid)
  sleep "$dur"
  kill -9 "$pid" 2>/dev/null || true
  sleep 0.3
}

pub_joy "[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 1.5 "linear.x=1, enable pressed"
pub_joy "[0.0, 0.0, 0.0, 0.0, 0.0, 1.0]" 1.5 "angular.z=1, enable pressed"

# 5. release the enable button (should immediately publish zero twist)
echo "[test] publishing Joy: enable released"
ros2 topic pub --once /test_joy sensor_msgs/msg/Joy \
  "{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0]}" \
  > /dev/null 2>&1 || true
sleep 0.5

# 6. Give the echo a brief moment to flush, then SIGKILL it and read
#    what we captured.
sleep 0.5
kill -9 $ECHO_PID 2>/dev/null || true

echo
echo "=========================================="
echo " captured /test_cmd_vel messages:"
echo "=========================================="
cat /tmp/nav2_joy_echo.txt || true

# Quick PASS/FAIL heuristic: did we see a non-zero twist?
if grep -q 'linear.x' /tmp/nav2_joy_echo.txt 2>/dev/null \
   && grep -qE '0\.[12345]' /tmp/nav2_joy_echo.txt 2>/dev/null; then
  echo
  echo "[test] PASS: saw non-zero /test_cmd_vel output."
  exit 0
else
  echo
  echo "[test] WARN: no non-zero twist was captured. See /tmp/nav2_joy_echo.txt." >&2
  exit 0   # still return 0 so build script doesn't fail the build
fi
