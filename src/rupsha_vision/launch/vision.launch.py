"""Vision node with the package defaults; `config:=` swaps the whole file,
`model_path:=` the one value most often changed on the bench."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('rupsha_vision'),
                                  'config', 'vision.yaml')
    config = LaunchConfiguration('config')
    model_path = LaunchConfiguration('model_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Overriding with '' would clobber a model set in the config file, so the
    # override is applied only when a path was actually given.
    has_model = PythonExpression(["'", model_path, "' != ''"])

    def node(condition, extra):
        return Node(package='rupsha_vision', executable='vision', name='vision',
                    output='screen', emulate_tty=True, condition=condition,
                    parameters=[config, {'use_sim_time': use_sim_time}, *extra])

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('model_path', default_value='',
                              description="ONNX model; '' keeps the config's value"),
        node(IfCondition(has_model), [{'model_path': model_path}]),
        node(UnlessCondition(has_model), []),
    ])
