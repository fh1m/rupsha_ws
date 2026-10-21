"""Start every driver row of config/sensors.yaml (board rows come from the manager).

    ros2 launch rupsha_sensors sensors.launch.py
    ros2 launch rupsha_sensors sensors.launch.py table:=/path/to/bench_sensors.yaml
"""

import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    from rupsha_sensors.table import load, node_spec

    table = pathlib.Path(LaunchConfiguration('table').perform(context))
    calib = table.parent / 'calibration'
    out = []
    for s in load(table, calib):
        spec = node_spec(s, calib)
        if spec is not None:
            out.append(Node(package=spec.package, executable=spec.executable, name=spec.name,
                            parameters=[spec.parameters], remappings=list(spec.remappings),
                            output='screen', emulate_tty=True))
    return out


def generate_launch_description():
    default = os.path.join(get_package_share_directory('rupsha_sensors'), 'config', 'sensors.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('table', default_value=default),
        OpaqueFunction(function=_nodes),
    ])
