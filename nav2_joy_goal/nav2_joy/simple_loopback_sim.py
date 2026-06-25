#!/usr/bin/env python3
# Copyright (c) 2026, nav2_joy authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Minimal loopback simulator for Nav2.

This is a simple Python replacement for nav2_loopback_sim when the
C++ package is not available (e.g., ROS 2 Humble apt doesn't ship it).

It provides:
- /clock (rosgraph_msgs/Clock)
- /odom (nav_msgs/Odometry) - integrates /cmd_vel
- /scan (sensor_msgs/LaserScan) - fake scan data
- TF: odom -> base_link, base_link -> base_scan
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class SimpleLoopbackSim(Node):
    """Minimal loopback simulator."""

    def __init__(self):
        super().__init__('simple_loopback_sim')

        # Declare parameters
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('scan_frame', 'base_scan')
        # 【修复】loopback 模式下没有 AMCL, 手动发布 map -> odom 的 static TF,
        # 避免 costmap 等不到 map 坐标系. 关闭 publish_map_to_odom 可禁用.
        self.declare_parameter('publish_map_to_odom', True)
        self.declare_parameter('map_frame', 'map')

        update_rate = self.get_parameter('update_rate').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.publish_map_to_odom = self.get_parameter('publish_map_to_odom').value
        self.map_frame = self.get_parameter('map_frame').value

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_cmd_vel = None
        self.last_update_time = None

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.clock_pub = self.create_publisher(Clock, '/clock', qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos
        )

        # Timer for simulation loop
        self.timer = self.create_timer(1.0 / update_rate, self.update)

        self.get_logger().info(
            f'SimpleLoopbackSim started: rate={update_rate}Hz, '
            f'base_frame={self.base_frame}, odom_frame={self.odom_frame}, '
            f'map_frame={self.map_frame}, publish_map_to_odom={self.publish_map_to_odom}'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Store the latest cmd_vel."""
        self.last_cmd_vel = msg

    def update(self):
        """Main simulation loop."""
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        # Initialize last_update_time
        if self.last_update_time is None:
            self.last_update_time = now_sec
            return

        dt = now_sec - self.last_update_time
        self.last_update_time = now_sec

        # Integrate cmd_vel
        if self.last_cmd_vel is not None:
            vx = self.last_cmd_vel.linear.x
            vy = self.last_cmd_vel.linear.y
            vtheta = self.last_cmd_vel.angular.z

            # Update position (in odom frame)
            self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
            self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
            self.theta += vtheta * dt

            # Normalize theta to [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish clock
        clock_msg = Clock()
        clock_msg.clock = now.to_msg()
        self.clock_pub.publish(clock_msg)

        # Publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Quaternion from yaw
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom_msg.twist.twist.linear.x = self.last_cmd_vel.linear.x if self.last_cmd_vel else 0.0
        odom_msg.twist.twist.linear.y = self.last_cmd_vel.linear.y if self.last_cmd_vel else 0.0
        odom_msg.twist.twist.angular.z = self.last_cmd_vel.angular.z if self.last_cmd_vel else 0.0

        self.odom_pub.publish(odom_msg)

        # Publish TF: odom -> base_link
        t_odom = TransformStamped()
        t_odom.header.stamp = now.to_msg()
        t_odom.header.frame_id = self.odom_frame
        t_odom.child_frame_id = self.base_frame
        t_odom.transform.translation.x = self.x
        t_odom.transform.translation.y = self.y
        t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation.z = math.sin(self.theta / 2.0)
        t_odom.transform.rotation.w = math.cos(self.theta / 2.0)

        # Publish TF: base_link -> base_scan
        t_scan = TransformStamped()
        t_scan.header.stamp = now.to_msg()
        t_scan.header.frame_id = self.base_frame
        t_scan.child_frame_id = self.scan_frame
        t_scan.transform.translation.x = 0.0
        t_scan.transform.translation.y = 0.0
        t_scan.transform.translation.z = 0.1  # Slightly above base
        t_scan.transform.rotation.w = 1.0  # No rotation

        # 【修复】map -> odom 的 static TF, loopback 模式下没有 AMCL, 需要这个
        # 才能让 costmap/rviz 的 map 坐标系生效
        transforms = [t_odom, t_scan]
        if self.publish_map_to_odom:
            t_map = TransformStamped()
            t_map.header.stamp = now.to_msg()
            t_map.header.frame_id = self.map_frame
            t_map.child_frame_id = self.odom_frame
            t_map.transform.translation.x = 0.0
            t_map.transform.translation.y = 0.0
            t_map.transform.translation.z = 0.0
            t_map.transform.rotation.w = 1.0
            transforms.append(t_map)

        self.tf_broadcaster.sendTransform(transforms)

        # Publish fake scan (empty scan, no obstacles)
        scan_msg = LaserScan()
        scan_msg.header.stamp = now.to_msg()
        scan_msg.header.frame_id = self.scan_frame
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.pi / 180.0  # 1 degree resolution
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.12
        scan_msg.range_max = 3.5
        # Fill with max range (no obstacles)
        num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
        scan_msg.ranges = [scan_msg.range_max] * num_readings
        scan_msg.intensities = [0.0] * num_readings

        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLoopbackSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
