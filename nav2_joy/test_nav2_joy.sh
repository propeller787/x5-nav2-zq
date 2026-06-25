#!/usr/bin/env bash
# Quick smoke test for nav2_joy.
#
# Launches the keyboard->Joy node in the current terminal and the
# Joy->Twist node in the background, then echoes the resulting
# /cmd_vel messages so you can see the effect of the key presses.
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash
#   ./test_nav2_joy.sh
#
# Then in this terminal, drive with i/j/k/l and hold space.
# Press 'q' to quit.

set -e

echo "=========================================="
echo " nav2_joy smoke test"
echo "=========================================="

cleanup() {
  echo
  echo "[test] cleaning up..."
  kill 0 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

# 1. Start the Joy -> Twist converter
echo "[test] starting joy_to_twist..."
ros2 run nav2_joy joy_to_twist --ros-args \
  -p joy_topic:=/joy \
  -p cmd_vel_topic:=/cmd_vel \
  -p deadman_timeout:=0.3 &
JOY2T_PID=$!
sleep 0.5

# 2. Start a cmd_vel topic echoer so we can see the output
echo "[test] subscribing to /cmd_vel (this terminal will show velocity commands)..."
( timeout 30 ros2 topic echo /cmd_vel ) &
ECHO_PID=$!
sleep 0.5

# 3. Start the keyboard node (this blocks until you press 'q')
echo "[test] starting nav2_joy_teleop (i/j/k/l + space, q to quit)..."
ros2 run nav2_joy nav2_joy_teleop --ros-args -p joy_topic:=/joy

# 4. Cleanup
kill $ECHO_PID 2>/dev/null || true
kill $JOY2T_PID 2>/dev/null || true
wait 2>/dev/null || true

echo "[test] done."
