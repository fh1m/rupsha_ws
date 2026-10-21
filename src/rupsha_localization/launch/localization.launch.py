"""Shadow estimator. `extnav_enabled:=true` opts in to the ExtNav candidate topic."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rupsha_localization')
    params = os.path.join(share, 'config', 'localization.yaml')
    extnav = LaunchConfiguration('extnav_enabled')
    return LaunchDescription([
        DeclareLaunchArgument('extnav_enabled', default_value='false',
                              description='Publish on the ExtNav odometry topic when the gate passes'),
        Node(
            package='rupsha_localization',
            executable='localization',
            name='localization',
            output='screen',
            parameters=[params, {'extnav_enabled': ParameterValue(extnav, value_type=bool)}],
        ),
    ])
