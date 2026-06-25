#!/usr/bin/env python3
# Copyright (c) 2026, nav2_joy_goal authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Dual-mode keyboard controller: Nav2 goal OR direct cmd_vel teleop.

A single xterm. The two modes use disjoint key sets so they never
collide. Press `m` to switch between them.

GOAL mode (default):
  w       target 1.0m forward
  s       target 0.5m backward
  a / d   target 0.5m forward + heading +/- 30 deg
  q / e   target 1.0m forward + heading +/- 30 deg
  c       cancel current NavigateToPose goal

TELEOP mode (direct /cmd_vel, hold-to-move with auto-decay):
  i / k   forward / backward full speed
  j / l   rotate left / right full speed
  u / o   forward + rotate diagonal
  w / x   forward / backward 0.75
  a / d   rotate +/- 0.75
  , / .   forward / backward 0.25
  s       stop
  space   deadman (robot only moves while space is held)

Common:
  m       switch mode
  Q / Esc quit
"""

import math
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


STEP_XY = 0.5                  # m per goal keypress
STEP_YAW = math.radians(30.0)  # rad per goal keypress
DECAY = 0.6                    # velocity decay per 100ms tick in teleop
DEAD_THRESHOLD = 0.02          # |v| below this snaps to 0


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


class KeyboardGoalSetter(Node):
    def __init__(self):
        super().__init__('keyboard_goal_setter')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose_target', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._current_handle = None
        self._mode = 'goal'         # 'goal' or 'teleop'
        self._twist = Twist()
        self._deadman = False

    # ---------------------------------------------------------------- goal
    def _get_robot_pose(self):
        t = self._tf.lookup_transform(
            'map', 'base_link', rclpy.time.Time(),
            timeout=Duration(seconds=1.0))
        return (
            t.transform.translation.x,
            t.transform.translation.y,
            yaw_from_quat(t.transform.rotation),
        )

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('goal rejected')
            return
        self._current_handle = handle
        handle.get_result_async().add_done_callback(
            lambda _: setattr(self, '_current_handle', None))

    def _send_goal(self, x, y, yaw):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('navigate_to_pose not available')
            return
        if self._current_handle is not None:
            self._current_handle.cancel_goal_async()
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation = quat_from_yaw(yaw)
        self._goal_pub.publish(msg)
        future = self._client.send_goal_async(NavigateToPose.Goal(pose=msg))
        future.add_done_callback(self._on_goal_response)
        print(f'\r  -> x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+.0f}deg    ',
              end='', flush=True)

    def _handle_goal_key(self, key):
        if key == 'c':
            if self._current_handle is not None:
                self._current_handle.cancel_goal_async()
                self._current_handle = None
                print('\r  -> cancelled                    ', end='', flush=True)
            return
        try:
            x, y, yaw = self._get_robot_pose()
        except Exception as e:
            self.get_logger().warn(f'no TF: {e}')
            return
        if key == 'w':
            x += STEP_XY * math.cos(yaw) * 2.0; y += STEP_XY * math.sin(yaw) * 2.0
        elif key == 's':
            x -= STEP_XY * math.cos(yaw); y -= STEP_XY * math.sin(yaw)
        elif key == 'a':
            x += STEP_XY * math.cos(yaw); y += STEP_XY * math.sin(yaw)
            yaw += STEP_YAW
        elif key == 'd':
            x += STEP_XY * math.cos(yaw); y += STEP_XY * math.sin(yaw)
            yaw -= STEP_YAW
        elif key == 'q':
            x += STEP_XY * math.cos(yaw) * 2.0; y += STEP_XY * math.sin(yaw) * 2.0
            yaw += STEP_YAW
        elif key == 'e':
            x += STEP_XY * math.cos(yaw) * 2.0; y += STEP_XY * math.sin(yaw) * 2.0
            yaw -= STEP_YAW
        else:
            return
        self._send_goal(x, y, yaw)

    # -------------------------------------------------------------- teleop
    def _handle_teleop_key(self, key):
        if key == ' ':
            self._deadman = not self._deadman
            if not self._deadman:
                self._twist = Twist()
            print(f'\r  -> deadman {"ON " if self._deadman else "OFF"}                ',
                  end='', flush=True)
            return
        if not self._deadman:
            return
        v = self._twist
        if key == 'i':   v.linear.x =  1.0
        elif key == 'k': v.linear.x = -1.0
        elif key == 'j': v.angular.z =  1.0
        elif key == 'l': v.angular.z = -1.0
        elif key == 'u': v.linear.x = 0.5; v.angular.z =  0.5
        elif key == 'o': v.linear.x = 0.5; v.angular.z = -0.5
        elif key == 'w': v.linear.x =  0.75
        elif key == 'x': v.linear.x = -0.75
        elif key == 'a': v.angular.z =  0.75
        elif key == 'd': v.angular.z = -0.75
        elif key == ',': v.linear.x =  0.25
        elif key == '.': v.linear.x = -0.25
        elif key == 's': self._twist = Twist()
        else: return
        print(f'\r  -> vx={v.linear.x:+.2f} wz={v.angular.z:+.2f}                 ',
              end='', flush=True)

    def _decay_twist(self):
        if not self._deadman or self._mode != 'teleop':
            return
        self._twist.linear.x *= DECAY
        self._twist.angular.z *= DECAY
        if abs(self._twist.linear.x)  < DEAD_THRESHOLD: self._twist.linear.x  = 0.0
        if abs(self._twist.angular.z) < DEAD_THRESHOLD: self._twist.angular.z = 0.0

    # -------------------------------------------------------------- common
    def _print_mode(self):
        print(f'\n[MODE: {self._mode.upper()}]')
        if self._mode == 'goal':
            print('  w 1.0m fwd  s 0.5m back')
            print('  a/d fwd 0.5m + heading +/-30deg')
            print('  q/e fwd 1.0m + heading +/-30deg  c cancel')
        else:
            print('  space deadman (hold to enable), then:')
            print('  i/k fwd/back  j/l rotate  u/o diagonal')
            print('  w/x f +/-0.75  a/d r +/-0.75  ,/. f +/-0.25  s stop')

    def run(self):
        print('m: switch mode (GOAL <-> TELEOP)   Q/Esc: quit')
        try:
            self._get_robot_pose()
        except Exception:
            print('Waiting for TF map->base_link...')
        self._print_mode()

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while rclpy.ok():
                r, _, _ = select.select([fd], [], [], 0.1)
                if r:
                    key = sys.stdin.read(1)
                    if key in ('Q', '\x1b'):
                        print('\nquit')
                        break
                    if key == 'm':
                        self._mode = 'teleop' if self._mode == 'goal' else 'goal'
                        self._twist = Twist()
                        self._deadman = False
                        if self._current_handle is not None:
                            self._current_handle.cancel_goal_async()
                            self._current_handle = None
                        self._print_mode()
                        continue
                    if self._mode == 'goal':
                        self._handle_goal_key(key)
                    else:
                        self._handle_teleop_key(key)
                else:
                    self._decay_twist()
                self._cmd_vel_pub.publish(self._twist)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = KeyboardGoalSetter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
