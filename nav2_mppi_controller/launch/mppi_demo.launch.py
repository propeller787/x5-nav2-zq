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
MPPI Controller Demo Launch (TurtleBot3 + Gazebo + Nav2 + MPPI).

该脚本一键启动完整的 MPPI 控制器 demo：
  1. Gazebo 仿真环境 (TurtleBot3 waffle, turtlebot3_world)
  2. robot_state_publisher (发布 TF 树)
  3. Nav2 完整栈 (map_server, AMCL, controller_server, planner_server,
                 behavior_server, smoother_server, velocity_smoother,
                 bt_navigator, waypoint_follower)
  4. Nav2 lifecycle_manager (自动配置 + 激活所有 Nav2 节点)
  5. RViz2 (Nav2 默认视图)

关键点:
  - controller_server 加载的是源码中 nav2_bringup/params/nav2_params.yaml，
    该文件已把 FollowPath 插件配置为 nav2_mppi_controller::MPPIController。
  - 地图使用 apt 安装的 nav2_bringup 中自带的 turtlebot3_world.yaml
    (Gazebo 仿真世界与该地图对应)。

依赖:
  - ros-humble-turtlebot3-gazebo     (apt)
  - ros-humble-nav2-bringup          (apt, 提供地图与默认 RViz 配置)
  - 本仓库源码编译产物 (ros-humble-nav2-mppi-controller, etc.)

Usage:
    export ROS_LOG_DIR=/tmp/ros_logs
    export TURTLEBOT3_MODEL=waffle
    source /opt/ros/humble/setup.bash
    source ~/code/navigation2/install/setup.bash   # 源码编译后的 workspace
    ros2 launch /home/howard/code/navigation2/nav2_mppi_controller/launch/mppi_demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ----------------------------------------------------------------------------
# 路径常量
# ----------------------------------------------------------------------------
# 源码中的 nav2_bringup: 用于加载包含 MPPI 配置的 params 文件
NAV2_SOURCE_DIR = '/home/howard/code/navigation2/nav2_bringup'
PARAMS_FILE = os.path.join(NAV2_SOURCE_DIR, 'params', 'nav2_params.yaml')

# apt 安装的 nav2_bringup: 提供与 Gazebo 世界对应的预建地图
APT_NAV2_BRINGUP_DIR = get_package_share_directory('nav2_bringup')
MAP_FILE = os.path.join(APT_NAV2_BRINGUP_DIR, 'maps', 'turtlebot3_world.yaml')
RVIZ_CONFIG = os.path.join(APT_NAV2_BRINGUP_DIR, 'rviz', 'nav2_default_view.rviz')

# 由 lifecycle_manager 统一管理的 Nav2 节点
NAV2_NODES = [
    'map_server',
    'amcl',
    'controller_server',
    'planner_server',
    'behavior_server',
    'smoother_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
]


def _make_nav2_nodes(params, map_server_params=None):
    """生成 Nav2 节点列表。共享同一份 params 文件。

    Args:
        params: 通用 Nav2 参数 (params_file + use_sim_time)。
        map_server_params: map_server 专用参数 (额外注入 yaml_filename)。
            若为 None 则使用 params。
    """
    if map_server_params is None:
        map_server_params = params
    return [
        Node(package='nav2_map_server',        executable='map_server',
             name='map_server',        output='screen', parameters=map_server_params),
        Node(package='nav2_amcl',              executable='amcl',
             name='amcl',              output='screen', parameters=params),
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
    map_yaml_file = LaunchConfiguration('map')
    params_file   = LaunchConfiguration('params_file')
    x_pose        = LaunchConfiguration('x_pose')
    y_pose        = LaunchConfiguration('y_pose')

    # 1) Gazebo 仿真
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    # 2) robot_state_publisher (从 URDF 发布 TF)
    urdf_path = os.path.join(tb3_gazebo_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
    )

    # 3) Nav2 节点 (共享同一份 params)
    #    map_server 需要 yaml_filename 参数覆盖 (params 文件里该字段为空,
    #    默认由 launch 传入)
    nav2_common_params = [params_file, {'use_sim_time': use_sim_time}]
    map_server_params = [
        params_file,
        {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file},
    ]
    nav2_nodes = _make_nav2_nodes(nav2_common_params, map_server_params=map_server_params)

    # 4) lifecycle_manager: 自动配置并激活所有 Nav2 节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': NAV2_NODES}],
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

    # 声明启动参数
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=MAP_FILE,
        description='Full path to map yaml file (AMCL)'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=PARAMS_FILE,
        description='Full path to Nav2 params file (must contain MPPI plugin)'))
    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='-2.0',
        description='Initial robot x position'))
    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='-0.5',
        description='Initial robot y position'))

    # 启动顺序: Gazebo -> robot_state_publisher -> Nav2 -> lifecycle_manager -> RViz
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    for n in nav2_nodes:
        ld.add_action(n)
    ld.add_action(lifecycle_manager)
    ld.add_action(rviz)

    return ld
