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
  6. turtlebot3_house/model.sdf 引用了 mailbox / cafe_table /
     first_2015_trash_can / table_marble 4 个外部模型. 废弃的
     models.gazebosim.org 已 EOL, Gazebo 拉取超时会让 world 加载阻塞 100+ 秒.
     本 launch 把本仓库的 models/ 目录追加到 GAZEBO_MODEL_PATH, 走本地模型.

依赖:
  - ros-humble-turtlebot3-gazebo     (apt, 含 turtlebot3_house 模型)
  - ros-humble-slam-toolbox          (apt)
  - ros-humble-nav2-bringup          (apt, 提供默认 RViz 配置)
  - 本仓库源码编译产物 (ros-humble-nav2-mppi-controller, etc.)
  - 本仓库自带的 nav2_mppi_controller/models/  (4 个 ~3MB 的 SDF 模型)

关键点:
  - controller_server 加载的仍是 nav2_bringup/params/nav2_params.yaml,
    FollowPath 插件保持 nav2_mppi_controller::MPPIController,
    MPPI 算法模块未做修改。
  - turtlebot3_house 没有预建地图, 本 launch 用 SLAM 边建图边导航。
    地图会通过 /map 话题实时发布给 Nav2 用作 costmap 背景。
  - 仿真分拆启动 gzserver → spawn → (15s 后) gzclient/Nav2/RViz,
    避免死机后 GPU 恢复慢导致 spawn 超时.
  - lifecycle 激活在 spawn 成功后 25s 触发 (等 Nav2 节点就绪).

Usage:
    /home/howard/code/navigation2/nav2_mppi_controller/scripts/run_mppi_house_slam_demo.sh

    # 或手动:
    export ROS_LOG_DIR=/tmp/ros_logs
    export TURTLEBOT3_MODEL=waffle
    export LIBGL_ALWAYS_SOFTWARE=1
    source /opt/ros/humble/setup.bash
    source ~/code/navigation2/install/setup.bash
    ros2 launch /home/howard/code/navigation2/nav2_mppi_controller/launch/mppi_house_slam_demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.event_handlers import OnProcessExit
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

# 本仓库内自带的 Gazebo 模型 (mailbox / cafe_table / first_2015_trash_can /
# table_marble), 用于补全 turtlebot3_house 的 model:// 引用, 避免走废弃的
# models.gazebosim.org. 追加到 GAZEBO_MODEL_PATH, 保留默认 ~/.gazebo/models.
# 注意: nav2_mppi_controller 不一定被 colcon 编译进 install/, 这里直接用
# 源码绝对路径, 与 NAV2_SOURCE_DIR 风格保持一致.
PACKAGE_MODELS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'models')

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
    params_file  = LaunchConfiguration('nav2_params_file')
    x_pose       = LaunchConfiguration('x_pose')
    y_pose       = LaunchConfiguration('y_pose')

    # 1) Gazebo 仿真: 分拆启动以便给 spawn 更长超时 (默认 30s 在 GPU
    #    恢复/多 GUI 并发时不够, 会导致 /spawn_entity 超时、机器人未生成).
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    launch_file_dir = os.path.join(tb3_gazebo_dir, 'launch')
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    world = os.path.join(tb3_gazebo_dir, 'worlds', 'turtlebot3_house.world')
    model_sdf = os.path.join(
        tb3_gazebo_dir, 'models', f'turtlebot3_{tb3_model}', 'model.sdf')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        # 避免父 launch 的 params_file 透传到 gzserver (会加载 nav2_params,
        # 拖慢 Gazebo 初始化).
        launch_arguments={'world': world, 'params_file': ''}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', tb3_model,
            '-file', model_sdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-timeout', '180',
        ],
        output='screen',
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
    ros_setup = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/howard/code/navigation2/install/setup.bash'
    )
    activate_lifecycle_nodes = ExecuteProcess(
        cmd=['bash', '-c', (
            f'{ros_setup} && '
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

    # 延迟启动 GUI 和 Nav2, 让 gzserver 先独占 GPU 完成世界加载.
    delayed_stack = TimerAction(
        period=15.0,
        actions=[
            gzclient,
            slam_toolbox,
            *nav2_nodes,
            rviz,
        ],
    )

    # ------------------------------------------------------------------------
    # 装配 LaunchDescription
    # ------------------------------------------------------------------------
    ld = LaunchDescription()

    # # 死机后 Intel GPU 驱动可能暂时不可用; 软件渲染可避免 gzserver 卡死.
    # 正常情况，还是GPU渲染快，所以这里注释掉软件渲染.
    # ld.add_action(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'))
    ld.add_action(SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model))

    # 优先用本仓库自带的模型 (mailbox / cafe_table 等), 再叠加系统默认路径,
    # 避免 Gazebo 在 turtlebot3_house 加载时去拉废弃的 models.gazebosim.org.
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = PACKAGE_MODELS_DIR
    if existing_model_path:
        new_model_path = new_model_path + ':' + existing_model_path
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path))

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'))
    ld.add_action(DeclareLaunchArgument(
        'nav2_params_file', default_value=PARAMS_FILE,
        description='Full path to Nav2 params file (must contain MPPI plugin)'))
    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='-2.0',
        description='Initial robot x position'))
    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='-0.5',
        description='Initial robot y position'))

    # 启动顺序: gzserver + spawn 先启动; 15s 后再启 GUI/Nav2 (避免 GPU 争抢).
    # lifecycle 在 spawn 成功后 10s 触发.
    ld.add_action(gzserver)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(delayed_stack)

    spawn_success_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                LogInfo(msg='[mppi_house_slam_demo] robot spawned, '
                            'activating lifecycle nodes in 25s...'),
                TimerAction(period=25.0, actions=[activate_lifecycle_nodes]),
            ],
        ),
    )
    ld.add_action(spawn_success_handler)

    return ld
