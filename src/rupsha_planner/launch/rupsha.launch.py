"""The whole rover: control base, then the layers that sit on it.

    ros2 launch rupsha_planner rupsha.launch.py
    ros2 launch rupsha_planner rupsha.launch.py vision:=false localization:=false

Order is the architecture: the manager owns the board and is useful alone;
sensors, vision, localization and the executive only ever reach the board through it
(/rupsha/move), so any of them can be left out and the rest still runs.
In simulation use rupsha_sim/sim.launch.py, which starts the same nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, name, condition=None, **args):
    path = os.path.join(get_package_share_directory(pkg), 'launch', name)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=args.items(), condition=condition)


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('url', default_value='/dev/serial/by-id/usb-ArduPilot_Pixhawk1*'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('vision', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('executive', default_value='true'),
        _include('rupsha_control', 'bringup.launch.py', url=arg('url')),
        _include('rupsha_sensors', 'sensors.launch.py', IfCondition(arg('sensors'))),
        _include('rupsha_vision', 'vision.launch.py', IfCondition(arg('vision'))),
        _include('rupsha_localization', 'localization.launch.py', IfCondition(arg('localization'))),
        Node(package='rupsha_planner', executable='executive', name='executive',
             output='screen', emulate_tty=True, condition=IfCondition(arg('executive'))),
    ])
