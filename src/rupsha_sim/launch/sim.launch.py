"""Gazebo Harmonic + ArduRover SITL (JSON lockstep) + the Rupsha stack.

    ros2 launch rupsha_sim sim.launch.py                       # flat_ci, GUI
    ros2 launch rupsha_sim sim.launch.py world:=playpen headless:=true

Starts: gz sim (world), the contract bridge, ArduRover SITL --model JSON (talks
to the ArduPilotPlugin on UDP 9002), and -- unless disabled -- the manager on
tcp:127.0.0.1:5760, vision, localization and the executive. The stack nodes are the
same binaries as on the rover; only `url` differs.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    from rupsha_sim import contract, model_gen
    from rupsha_sim.sitl import Sitl

    share = get_package_share_directory('rupsha_sim')
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    world_file = os.path.join(share, 'worlds', f'{world}.sdf')
    import yaml
    with open(os.path.join(share, 'scenarios', f'{world}.yaml')) as f:
        home = yaml.safe_load(f)['home']

    gz_args = ['gz', 'sim', '-r', world_file]
    if headless:
        gz_args[2:2] = ['-s', '--headless-rendering']
    sitl = Sitl(model='JSON', home=(home['lat'], home['lon'], home['alt'], home['heading']),
                log_dir=os.path.expanduser('~/.ros/rupsha_sitl'))
    sitl.dir.mkdir(parents=True, exist_ok=True)
    return [
        ExecuteProcess(cmd=gz_args, output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sim_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                          'use_sim_time': True}], output='screen'),
        ExecuteProcess(cmd=sitl.command(), cwd=str(sitl.dir), output='log'),
        TimerAction(period=8.0, actions=[
            Node(package='rupsha_control', executable='manager', name='rupsha_manager',
                 parameters=[{'url': contract.SITL_URL,
                              'camera_forward_m': float(model_gen.CAMERA_XYZ[0])}], output='screen',
                 condition=IfCondition(LaunchConfiguration('manager'))),
            # Sim time: the bridge stamps images in Gazebo's clock, so on wall time every
            # frame's latency is NaN and every detection's age unknown (B-33).
            Node(package='rupsha_vision', executable='vision', name='vision', output='screen',
                 parameters=[{'use_sim_time': True}],
                 condition=IfCondition(LaunchConfiguration('vision'))),
            Node(package='rupsha_localization', executable='localization', name='localization',
                 parameters=[os.path.join(get_package_share_directory('rupsha_localization'),
                                          'config', 'localization.yaml')],
                 output='screen', condition=IfCondition(LaunchConfiguration('localization'))),
            Node(package='rupsha_planner', executable='executive', name='executive',
                 output='screen', condition=IfCondition(LaunchConfiguration('executive'))),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='flat_ci'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('manager', default_value='true'),
        DeclareLaunchArgument('vision', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('executive', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
