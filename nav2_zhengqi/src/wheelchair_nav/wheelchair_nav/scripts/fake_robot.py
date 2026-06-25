#!/usr/bin/env python3
"""
fake_robot.py
=============
用于 sandbox 等无 Gazebo 环境的简易机器人仿真节点。

订阅 /cmd_vel (Twist)，模拟差分驱动运动学：
    x_dot   = vx * cos(theta)
    y_dot   = vx * sin(theta)
    theta_dot = wz

发布：
    - /odom (Odometry)
    - /tf (odom -> base_footprint)
    - /scan (LaserScan) - 模拟简易环境（房间 + 障碍物）
    - /clock (Clock) - 推进 ROS 时间，让 use_sim_time 节点工作

可被 Nav2 替代 Gazebo + 真实激光雷达进行端到端验证。

本节点 use_sim_time=False，独立 system clock 运行，不受 /clock 影响。
"""

import math
import numpy as np
import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster


class FakeRobot(Node):
    def __init__(self):
        # 用 wall clock 跑, 不依赖 /clock 推进
        super().__init__('fake_robot', automatically_declare_parameters_from_overrides=True)

        # ============== 参数 ==============
        if not self.has_parameter('cmd_vel_topic'):
            self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        if not self.has_parameter('odom_topic'):
            self.declare_parameter('odom_topic', '/odom')
        if not self.has_parameter('scan_topic'):
            self.declare_parameter('scan_topic', '/scan')
        if not self.has_parameter('clock_topic'):
            self.declare_parameter('clock_topic', '/clock')
        if not self.has_parameter('publish_clock'):
            self.declare_parameter('publish_clock', True)
        if not self.has_parameter('odom_frame'):
            self.declare_parameter('odom_frame', 'odom')
        if not self.has_parameter('base_frame'):
            self.declare_parameter('base_frame', 'base_footprint')
        if not self.has_parameter('initial_x'):
            self.declare_parameter('initial_x', 0.0)
        if not self.has_parameter('initial_y'):
            self.declare_parameter('initial_y', 0.0)
        if not self.has_parameter('initial_yaw'):
            self.declare_parameter('initial_yaw', 0.0)
        if not self.has_parameter('update_rate'):
            self.declare_parameter('update_rate', 50.0)
        if not self.has_parameter('simulate_obstacles'):
            self.declare_parameter('simulate_obstacles', True)

        self.cmd_vel_topic  = self.get_parameter('cmd_vel_topic').value
        self.odom_topic     = self.get_parameter('odom_topic').value
        self.scan_topic     = self.get_parameter('scan_topic').value
        self.clock_topic    = self.get_parameter('clock_topic').value
        self.publish_clock  = self.get_parameter('publish_clock').value
        self.odom_frame     = self.get_parameter('odom_frame').value
        self.base_frame     = self.get_parameter('base_frame').value
        self.simulate_obs   = self.get_parameter('simulate_obstacles').value

        # ============== 状态 ==============
        self.x     = float(self.get_parameter('initial_x').value)
        self.y     = float(self.get_parameter('initial_y').value)
        self.theta = float(self.get_parameter('initial_yaw').value)
        self.vx    = 0.0
        self.wz    = 0.0

        # ============== 独立 wall clock (不被 use_sim_time 影响) ==============
        # 必须在 self.last_cmd_wall 之前定义, 因为 _now_wall() 会调用 self.wall_clock
        self.wall_clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME)
        self.last_cmd_wall = self._now_wall()

        # ============== 订阅 ==============
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_callback, 10)

        # ============== 发布 ==============
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # /clock 用 TRANSIENT_LOCAL 持久化 (供首次订阅者拉取)
        clock_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.clock_pub = self.create_publisher(Clock, self.clock_topic, clock_qos) \
            if self.publish_clock else None

        # ============== 定时器 ==============
        rate = self.get_parameter('update_rate').value
        period = 1.0 / rate
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info(
            f'fake_robot ready: odom={self.odom_topic} scan={self.scan_topic} '
            f'clock={"yes" if self.publish_clock else "no"} '
            f'init=({self.x:.2f},{self.y:.2f},{math.degrees(self.theta):.1f}deg)')

    def _now_wall(self):
        return self.wall_clock.now()

    def cmd_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z
        self.last_cmd_wall = self._now_wall()

    def update(self):
        # ============== 用 wall clock 推进 ==============
        now_wall = self._now_wall()
        rate = self.get_parameter('update_rate').value
        dt = 1.0 / rate
        now_msg = now_wall.to_msg()

        # 0.5s 没收到 cmd_vel 就停
        dt_since_cmd = (now_wall.nanoseconds - self.last_cmd_wall.nanoseconds) / 1e9
        if dt_since_cmd > 0.5:
            self.vx = 0.0
            self.wz = 0.0

        # ============== 差分驱动运动学 ==============
        self.x     += self.vx * math.cos(self.theta) * dt
        self.y     += self.vx * math.sin(self.theta) * dt
        self.theta += self.wz * dt
        # 归一化 theta 到 [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ============== 发布 /clock ==============
        if self.clock_pub is not None:
            clock = Clock()
            clock.clock = now_msg
            self.clock_pub.publish(clock)

        # ============== 发布 /odom ==============
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        # 协方差
        odom.pose.covariance[0]  = 0.05   # x
        odom.pose.covariance[7]  = 0.05   # y
        odom.pose.covariance[35] = 0.05   # yaw
        self.odom_pub.publish(odom)

        # ============== 发布 /tf ==============
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # ============== 发布 /scan ==============
        self.scan_pub.publish(self.make_scan(now_msg))

    def make_scan(self, stamp):
        """生成模拟激光雷达数据。
        环境: 10m x 10m 房间 + 3 个障碍物。
        """
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = -math.pi
        scan.angle_max =  math.pi
        scan.angle_increment = math.radians(1.0)
        scan.time_increment  = 0.0
        scan.scan_time       = 0.1
        scan.range_min       = 0.05
        scan.range_max       = 5.0

        n = 360
        ranges = np.full(n, 5.0)

        if not self.simulate_obs:
            scan.ranges = ranges.tolist()
            return scan

        ROOM_HALF = 5.0
        OBSTACLES = [
            ( 1.5,  1.5, 0.3),
            (-1.0,  2.0, 0.4),
            ( 2.0, -1.0, 0.3),
        ]

        for i in range(n):
            angle = -math.pi + i * math.radians(1.0)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            min_r = 5.0
            # 4 面墙
            if abs(cos_a) > 1e-3:
                r = (ROOM_HALF - self.x) / cos_a
                if r > 0 and r < min_r:
                    y_at = self.y + r * sin_a
                    if -ROOM_HALF <= y_at <= ROOM_HALF:
                        min_r = r
                r = (-ROOM_HALF - self.x) / cos_a
                if r > 0 and r < min_r:
                    y_at = self.y + r * sin_a
                    if -ROOM_HALF <= y_at <= ROOM_HALF:
                        min_r = r
            if abs(sin_a) > 1e-3:
                r = (ROOM_HALF - self.y) / sin_a
                if r > 0 and r < min_r:
                    x_at = self.x + r * cos_a
                    if -ROOM_HALF <= x_at <= ROOM_HALF:
                        min_r = r
                r = (-ROOM_HALF - self.y) / sin_a
                if r > 0 and r < min_r:
                    x_at = self.x + r * cos_a
                    if -ROOM_HALF <= x_at <= ROOM_HALF:
                        min_r = r

            # 圆形障碍物
            for ox, oy, orad in OBSTACLES:
                dx = ox - self.x
                dy = oy - self.y
                f = dx * cos_a + dy * sin_a
                d2 = dx*dx + dy*dy - f*f
                r2 = orad * orad
                if d2 <= r2:
                    root = math.sqrt(max(0, r2 - d2))
                    t1 = f - root
                    t2 = f + root
                    if t1 > 0 and t1 < min_r:
                        min_r = t1
                    elif t2 > 0 and t2 < min_r:
                        min_r = t2

            ranges[i] = max(scan.range_min, min(min_r, scan.range_max))

        scan.ranges = ranges.tolist()
        return scan


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
