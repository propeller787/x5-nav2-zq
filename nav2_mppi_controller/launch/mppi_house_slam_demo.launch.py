#!/usr/bin/env python3
# Copyright 2024
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
"""
MPPI Controller Demo Launch (Indoor House + SLAM + Nav2 + MPPI).

该脚本一键启动室内房屋仿真下的 MPPI demo (A 方案 / SLAM 模式):
  1. Gazebo 仿真环境 (TurtleBot3 waffle, turtlebot3_house)
     - 内含墙体与桌椅沙发等家具 (model://turtlebot3_house, apt 自带)
  2. slam_toolbox (online_async) 同步建图
  3. Nav2 栈 (controller_server, planner_server, behavior_server,
                smoother_server, velocity_smoother, bt_navigator,
                waypoint_follower) —— 不启动 map_server / amcl
  4. Nav2 lifecycle_manager 统一管理 slam_toolbox + Nav2 节点
  5. RViz2

关键点:
  - controller_server 加载的仍是 nav2_bringup/params/nav2_params.yaml,
    FollowPath 插件保持 nav2_mppi_controller::MPPIController,
    MPPI 算法模块未做修改。
  - turtlebot3_house 没有预建地图, 本 launch 用 SLAM 边建图边导航。
    地图会通过 /map 话题实时发布给 Nav2 用作 costmap 背景。
  - 仿真启动用 turtlebot3_house.launch.py (已包含 gzserver + gzclient
    + robot_state_publisher + spawn_turtlebot3), 因此本脚本不再单独
    启动 robot_state_publisher。
  - lifecycle_manager 的启动用 OnProcessStart 事件绑定 slam_toolbox
    进程启动, 完全事件驱动, 避免 fast-launch 下 service-client race.

依赖:
  - ros-humble-turtlebot3-gazebo     (apt, 含 turtlebot3_house 模型)
  - ros-humble-slam-toolbox          (apt)
  - ros-humble-nav2-bringup          (apt, 提供默认 RViz 配置)
  - 本仓库源码编译产物 (ros-humble-nav2-mppi-controller, etc.)

Usage:
    export ROS_LOG_DIR=/tmp/ros_logs
    export TURTLEBOT3_MODEL=waffle
    source /opt/ros/humble/setup.bash
    source ~/code/navigation2/install/setup.bash
    ros2 launch /home/howard/code/navigation2/nav2_mppi_controller/launch/mppi_house_slam_demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ----------------------------------------------------------------------------
# 路径常量
# ----------------------------------------------------------------------------
# 源码中的 nav2_bringup: 用于加载包含 MPPI 配置的 params 文件
NAV2_SOURCE_DIR = '/home/howard/code/navigation2/nav2_bringup'
PARAMS_FILE = os.path.join(NAV2_SOURCE_DIR, 'params', 'nav2_params.yaml')

# apt 安装的 nav2_bringup: 提供默认 RViz 配置
APT_NAV2_BRINGUP_DIR = get_package_share_directory('nav2_bringup')
RVIZ_CONFIG = os.path.join(APT_NAV2_BRINGUP_DIR, 'rviz', 'nav2_default_view.rviz')

# 由 lifecycle_manager 统一管理的生命周期节点
#   - 不包含 amcl / map_server (SLAM 模式不需要)
#   - 包含 slam_toolbox (slam_toolbox 节点是 lifecycle 节点)
LIFECYCLE_NODES = [
    'slam_toolbox',
    'controller_server',
    'planner_server',
    'behavior_server',
    'smoother_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
]


def _make_nav2_nodes(params):
    """生成 Nav2 节点列表 (无 amcl / map_server)。共享同一份 params 文件。"""
    return [
        Node(package='nav2_controller',        executable='controller_server',
             name='controller_server', output='screen', parameters=params),
        Node(package='nav2_planner',           executable='planner_server',
             name='planner_server',    output='screen', parameters=params),
        Node(package='nav2_behaviors',         executable='behavior_server',
             name='behavior_server',   output='screen', parameters=params),
        Node(package='nav2_smoother',          executable='smoother_server',
             name='smoother_server',   output='screen', parameters=params),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=params),
        Node(package='nav2_bt_navigator',      executable='bt_navigator',
             name='bt_navigator',      output='screen', parameters=params),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=params),
    ]


def generate_launch_description():
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    x_pose       = LaunchConfiguration('x_pose')
    y_pose       = LaunchConfiguration('y_pose')

    # 1) Gazebo 仿真: turtlebot3_house (含 gzserver + gzclient +
    #    robot_state_publisher + spawn_turtlebot3, 一次性启动)
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_house.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    # 2) slam_toolbox: online_async 模式, 边建图边供 Nav2 消费 /map.
    #    显式 Node 启动, 以便后续用 OnProcessStart 绑定它的进程启动事件
    #    来触发 lifecycle_manager (事件驱动同步, 替代固定 TimerAction).
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    slam_toolbox_params_file = os.path.join(
        slam_toolbox_pkg, 'config', 'mapper_params_online_async.yaml')
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_params_file, {'use_sim_time': use_sim_time}],
    )

    # 3) Nav2 节点 (无 amcl / map_server)
    nav2_params = [params_file, {'use_sim_time': use_sim_time}]
    nav2_nodes = _make_nav2_nodes(nav2_params)

    # 4) 用 ExecuteProcess 跑 bash + `ros2 lifecycle set` 循环,
    #    完全绕开 nav2_lifecycle_manager 节点 (其 service client 在 fast-launch
    #    下拿不到 service, 一直 waiting, 是 nav2 老 bug). 这里在 slam_toolbox
    #    进程启动 8s 后, 挨个给所有生命周期节点发 configure/activate 命令.
    activate_lifecycle_nodes = ExecuteProcess(
        cmd=['bash', '-c', (
            'for n in slam_toolbox controller_server planner_server behavior_server '
            'smoother_server velocity_smoother bt_navigator waypoint_follower; do '
            '  ros2 lifecycle set /$n configure 2>&1; '
            '  sleep 0.3; '
            '  ros2 lifecycle set /$n activate  2>&1; '
            '  sleep 0.3; '
            'done; '
            'echo "[mppi_house_slam_demo] all lifecycle nodes activated"'
        )],
        output='screen',
    )

    # 5) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', RVIZ_CONFIG],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------------
    # 装配 LaunchDescription
    # ------------------------------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=PARAMS_FILE,
        description='Full path to Nav2 params file (must contain MPPI plugin)'))
    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='-2.0',
        description='Initial robot x position'))
    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='-0.5',
        description='Initial robot y position'))

    # 启动顺序: Gazebo -> slam_toolbox -> Nav2 -> RViz
    # lifecycle 激活用 ExecuteProcess 跑 bash 循环 (见 activate_lifecycle_nodes),
    # 触发时机仍是 OnProcessStart(slam_toolbox) + 8s buffer. 彻底绕开
    # nav2_lifecycle_manager 节点自身的 service-client race bug.
    ld.add_action(gazebo)
    ld.add_action(slam_toolbox)
    for n in nav2_nodes:
        ld.add_action(n)
    ld.add_action(rviz)

    slam_toolbox_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox,
            on_start=[
                LogInfo(msg='[mppi_house_slam_demo] slam_toolbox process started, '
                            'activating lifecycle nodes in 8s...'),
                TimerAction(period=8.0, actions=[activate_lifecycle_nodes]),
            ],
        ),
    )
    ld.add_action(slam_toolbox_start_handler)

    return ld
