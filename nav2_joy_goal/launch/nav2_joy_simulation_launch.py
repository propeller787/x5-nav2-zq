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

"""All-in-one Nav2 simulation driven by the keyboard, no Gazebo required.

Brings up the loopback simulator + a minimal robot description + Nav2 +
the ``nav2_joy`` keyboard teleop in a single command. The robot moves
in software-only mode (``simple_loopback_sim`` integrates ``cmd_vel`` to
fake odometry), so no Gazebo / RViz window manager is required.

This is a self-contained launch file: it does NOT depend on
``nav2_minimal_tb3_sim`` (or any other robot description package). A
tiny inline URDF is published by ``robot_state_publisher`` so that the
TF tree is complete and RViz / Nav2 have something to render.

This is the recommended entry point for trying the package out.

Usage:
  ros2 launch nav2_joy nav2_joy_simulation_launch.py

A new xterm will pop up showing the keyboard help banner. Use i/j/k/l
to drive the robot, hold space as a dead-man switch, q to quit.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter

from _nav2_joy_launch_helpers import LaunchConfigAsBool

MINIMAL_URDF = """<?xml version="1.0"?>
<robot name="nav2_joy_loopback">
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.2"/>
      </geometry>
    </visual>
  </link>
  <joint name="base_scan_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_scan"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
  <link name="base_scan"/>
</robot>"""


_TERMINAL_PREFIXES = {
    'xterm': 'xterm -fa Monospace -fs 10 -hold -e',
    'gnome-terminal': 'gnome-terminal --',
    'konsole': 'konsole -e',
    'none': None,
}


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_joy_dir = get_package_share_directory('nav2_joy_goal')

    default_map = os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml')
    # 仓库内自定义的 nav2_joy_simulation_params.yaml 复制自 nav2_params.yaml,
    # 只在 local_costmap 加回 static_layer / track_unknown_space / footprint /
    # obstacle_max_range, 让 simple_loopback_sim 不要跑出地图边界; 主 yaml
    # 保持 apt 默认, mppi_*_demo.launch.py 跑 SLAM 不会被这些参数拖慢.
    default_params = os.path.join(nav2_joy_dir, 'config', 'nav2_joy_simulation_params.yaml')
    default_joy_teleop_params = os.path.join(nav2_joy_dir, 'config', 'joy_teleop.yaml')
    default_graph = os.path.join(bringup_dir, 'graphs', 'turtlebot3_graph.geojson')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    joy_teleop_params = LaunchConfiguration('joy_teleop_params')
    graph_file = LaunchConfiguration('graph')
    use_rviz = LaunchConfigAsBool('use_rviz')
    use_composition = LaunchConfigAsBool('use_composition')
    autostart = LaunchConfigAsBool('autostart')
    use_respawn = LaunchConfigAsBool('use_respawn')
    rviz_config = LaunchConfiguration('rviz_config')
    terminal = LaunchConfiguration('terminal')

    simple_loopback_sim = Node(
        package='nav2_joy_goal',
        executable='simple_loopback_sim.py',
        name='simple_loopback_sim',
        output='screen',
        parameters=[{
            'update_rate': 100.0,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_frame': 'base_scan',
            'publish_map_to_odom': True,
            'map_frame': 'map',
        }],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': MINIMAL_URDF,
        }],
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments=[
            ('map', map_file),
            ('graph', graph_file),
            ('use_sim_time', 'True'),
            ('params_file', params_file),
            ('autostart', autostart),
            ('use_composition', use_composition),
            ('use_respawn', use_respawn),
            ('use_localization', 'False'),
            ('use_keepout_zones', 'False'),
            ('use_speed_zones', 'False'),
            ('container_name', 'nav2_container'),
        ],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments=[
            ('namespace', ''),
            ('use_sim_time', 'True'),
            ('rviz_config', rviz_config),
        ],
    )

    def _resolve_keyboard_node(context):
        terminal_value = context.perform_substitution(terminal)
        prefix = _TERMINAL_PREFIXES.get(terminal_value)
        return [
            Node(
                package='nav2_joy_goal',
                executable='keyboard_goal_setter.py',
                name='keyboard_goal_setter',
                output='screen',
                prefix=prefix,
                parameters=[{'use_sim_time': True}],
            )
        ]

    goal_setter_node = OpaqueFunction(function=_resolve_keyboard_node)

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Full path to the map yaml file'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 params file'),
        DeclareLaunchArgument(
            'joy_teleop_params', default_value=default_joy_teleop_params,
            description='Full path to the joy_teleop yaml file'),
        DeclareLaunchArgument(
            'graph', default_value=default_graph,
            description='Route graph file'),
        DeclareLaunchArgument(
            'use_rviz', default_value='True',
            description='Start RViz'),
        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Load Nav2 nodes inside a composable container'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically lifecycle-configure / activate Nav2'),
        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Respawn crashed nodes'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
            description='RViz config to load'),
        DeclareLaunchArgument(
            'terminal', default_value='xterm', choices=list(_TERMINAL_PREFIXES.keys()),
            description='Terminal wrapper for the keyboard node'),

        simple_loopback_sim,
        robot_state_publisher,
        bringup_cmd,
        rviz_cmd,
        goal_setter_node,
    ])
