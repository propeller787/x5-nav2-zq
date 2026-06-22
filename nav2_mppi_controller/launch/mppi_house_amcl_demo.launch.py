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
MPPI Controller Demo Launch (Indoor House + AMCL + Nav2 + MPPI).

与 mppi_demo.launch.py 的差别:
  - Gazebo 世界: turtlebot3_world (9 柱子空旷世界)
                → turtlebot3_house   (含桌椅沙发床的室内场景)
  - 定位模式:   AMCL 用预建地图 (而非 SLAM 边建图边定位)
  - MPPI 算法: 完全不变, FollowPath 仍为 nav2_mppi_controller::MPPIController

用法:
  ros2 launch /home/howard/code/navigation2/nav2_mppi_controller/launch/mppi_house_amcl_demo.launch.py

如果用其它位置保存的地图, 可传参:
  ros2 launch ... mppi_house_amcl_demo.launch.py map:=/path/to/your_map.yaml

依赖:
  - ros-humble-turtlebot3-gazebo   (apt, 含 turtlebot3_house 模型)
  - ros-humble-nav2-bringup        (apt, 提供 RViz 默认配置)
  - 本仓库源码编译产物 (ros-humble-nav2-mppi-controller, etc.)
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
NAV2_SOURCE_DIR = '/home/howard/code/navigation2/nav2_bringup'
PARAMS_FILE = os.path.join(NAV2_SOURCE_DIR, 'params', 'nav2_params.yaml')

# 默认地图: 用户在 SLAM 模式下保存的 turtlebot3_house_map
DEFAULT_MAP_FILE = '/home/howard/map/tmp/turtlebot3_house_map.yaml'

# RViz 用 apt 安装的 nav2_bringup 默认视图
APT_NAV2_BRINGUP_DIR = get_package_share_directory('nav2_bringup')
RVIZ_CONFIG = os.path.join(APT_NAV2_BRINGUP_DIR, 'rviz', 'nav2_default_view.rviz')

# 由 lifecycle 统一管理的 Nav2 节点 (无 slam_toolbox, 走 AMCL)
LIFECYCLE_NODES = [
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


def _make_nav2_nodes(common_params, map_server_params):
    """生成 Nav2 节点列表。map_server 用专用 params (含 yaml_filename)。"""
    return [
        Node(package='nav2_map_server',        executable='map_server',
             name='map_server',        output='screen', parameters=map_server_params),
        Node(package='nav2_amcl',              executable='amcl',
             name='amcl',              output='screen', parameters=common_params),
        Node(package='nav2_controller',        executable='controller_server',
             name='controller_server', output='screen', parameters=common_params),
        Node(package='nav2_planner',           executable='planner_server',
             name='planner_server',    output='screen', parameters=common_params),
        Node(package='nav2_behaviors',         executable='behavior_server',
             name='behavior_server',   output='screen', parameters=common_params),
        Node(package='nav2_smoother',          executable='smoother_server',
             name='smoother_server',   output='screen', parameters=common_params),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common_params),
        Node(package='nav2_bt_navigator',      executable='bt_navigator',
             name='bt_navigator',      output='screen', parameters=common_params),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=common_params),
    ]


def generate_launch_description():
    use_sim_time  = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file   = LaunchConfiguration('params_file')
    x_pose        = LaunchConfiguration('x_pose')
    y_pose        = LaunchConfiguration('y_pose')

    # 1) Gazebo 仿真: turtlebot3_house (室内 + 家具, 与保存的地图对应)
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_house.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    # 2) Nav2 节点 (含 map_server, 走 AMCL 定位)
    #    map_server 需要 yaml_filename 参数覆盖 (params 文件里该字段为空)
    nav2_common_params = [params_file, {'use_sim_time': use_sim_time}]
    map_server_params = [
        params_file,
        {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file},
    ]
    nav2_nodes = _make_nav2_nodes(nav2_common_params, map_server_params)

    # 3) 用 ExecuteProcess 跑 bash + `ros2 lifecycle set` 循环,
    #    激活所有 Nav2 lifecycle 节点. 这里没有 slam_toolbox, 理论上用
    #    nav2_lifecycle_manager 也能成功, 但 ExecuteProcess 方案统一且稳.
    activate_lifecycle_nodes = ExecuteProcess(
        cmd=['bash', '-c', (
            'for n in map_server amcl controller_server planner_server behavior_server '
            'smoother_server velocity_smoother bt_navigator waypoint_follower; do '
            # 先看状态再转换, 跳过 unconfigured/inactive/error 中间态
            '  s=$(ros2 lifecycle get /$n 2>/dev/null | grep -oE "(unconfigured|inactive|active|finalized|error)\\s*\\[[0-9]+\\]" | head -1); '
            '  echo "[lcm] /$n current state: ${s:-unknown}"; '
            '  case "$s" in '
            '    *unconfigured*) ros2 lifecycle set /$n configure 2>&1; sleep 0.5; ;& '
            '    *inactive*)     ros2 lifecycle set /$n activate  2>&1; sleep 0.5; ;& '
            '  esac; '
            'done; '
            'echo "[mppi_house_amcl_demo] all lifecycle nodes activated"'
        )],
        output='screen',
    )

    # 4) RViz
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
        'map', default_value=DEFAULT_MAP_FILE,
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

    # 启动顺序: Gazebo -> Nav2 -> RViz
    # lifecycle 激活延 15s + 智能状态机. 6s 不够: bt_navigator 等 TF/topic 全
    # ready 再创建 service client, 此时 `configure` 才会被 node 接受, 否则
    # transition 静默失败 -> 节点留在 unconfigured, action server registered
    # 但 service client 为空 -> 所有 goal 被 REJECT.
    ld.add_action(gazebo)
    for n in nav2_nodes:
        ld.add_action(n)
    ld.add_action(rviz)
    ld.add_action(TimerAction(period=15.0, actions=[
        LogInfo(msg='[mppi_house_amcl_demo] activating lifecycle nodes (smart state machine)...'),
        activate_lifecycle_nodes,
    ]))

    return ld
