"""The control base alone: the manager that owns the board.

    ros2 launch rupsha_control bringup.launch.py
    ros2 launch rupsha_control bringup.launch.py url:=tcp:127.0.0.1:5760   # SITL

Everything above it (vision, localization, the executive) is optional and
launched by rupsha_planner/rupsha.launch.py; the rover can always be driven
with just this and `ros2 run rupsha_planner rupsha`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('rupsha_control'), 'config', 'manager.yaml')
    url = LaunchConfiguration('url')
    return LaunchDescription([
        DeclareLaunchArgument('url', default_value='/dev/serial/by-id/usb-ArduPilot_Pixhawk1*'),
        DeclareLaunchArgument('extnav_uplink', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='rupsha_control', executable='manager', name='rupsha_manager',
             parameters=[config, {
                 'url': url,
                 'extnav_uplink': ParameterValue(LaunchConfiguration('extnav_uplink'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             # The Pi can boot before the Pixhawk: the manager exits on LinkDown
             # after 30 s and comes back, rather than leaving no control base.
             respawn=True, respawn_delay=2.0,
             output='screen', emulate_tty=True),
    ])
