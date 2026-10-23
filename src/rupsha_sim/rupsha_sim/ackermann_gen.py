"""The Ackermann plant: one driven axle, two steer hinges, camera, IMU, GPS.

No lidar and no ArduPilot plugin. The controller YAML path is a token the
launch replaces with the installed file, because gz_ros2_control does not
expand $(find ...) itself.
"""

from __future__ import annotations

import math
import pathlib
import random
import struct
import zlib

from . import chassis, contract

# Replaced at launch with the installed meshes directory. Gazebo does not
# expand package:// when the model is spawned from a file.
MESH = '__RUPSHA_MESH__'
# Their CAD is millimetres, +Y forward, origin at the plate centre.
# The shell link is that frame. The joint below turns it into ours:
# +X forward, rear axle at the origin, plate 5 mm off the ground.
SHELL_JOINT = '0.065 0 -0.035'
SHELL_RPY = '0 0 -1.5708'

PKG = pathlib.Path(__file__).resolve().parent.parent
NAME = 'rupsha_ackermann'

# Chassis centre is the rear axle. Spawned at z=SPAWN_Z so the wheels sit on the ground.
SPAWN_Z = 0.04
# Camera pitched up. Positive pitch in this URDF looks down, same as the skid SDF.
CAMERA_PITCH_DEG = -17.0
# One post, east of a robot facing +x, so the camera has something to see.
POST_EAST_M = 2.0
POST_TAG = 3


def _inertia_box(m, x, y, z):
    return (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12)


def _inertia_cyl(m, r, length):
    axial = 0.5 * m * r * r
    perp = m * (3 * r * r + length * length) / 12
    return perp, perp, axial


def _mesh(filename, rgba, material, scale='0.001 0.001 0.001'):
    # urdfdom drops the whole visual, and with it the link's collision, when
    # the material has no name. A dropped wheel collision leaves the robot sitting.
    return (
        f'    <visual><geometry><mesh filename="{MESH}/{filename}" scale="{scale}"/></geometry>'
        f'<material name="{material}"><color rgba="{rgba}"/></material></visual>\n'
    )


def _fixed(name, body, xyz='0 0 0', rpy='0 0 0', parent='chassis') -> str:
    return f"""  <link name="{name}">
{body}    <inertial><mass value="0.001"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>
  </link>
  <joint name="{name}_joint" type="fixed">
    <origin xyz="{xyz}" rpy="{rpy}"/>
    <parent link="{parent}"/><child link="{name}"/>
  </joint>
"""


def _shell_links() -> str:
    # One link per colour. Gazebo paints a whole link, so the white shell,
    # the orange neck, the blue head, and the black gearbox stay separate.
    groups = {
        'shell_white': (['bottom_chassis.stl', 'middle_plate.stl', 'top_cover.stl'], '0.93 0.93 0.90 1'),
        'shell_orange': (['camera_neck.stl'], '0.95 0.42 0.08 1'),
        'shell_blue': (['camera_head.stl'], '0.16 0.32 0.62 1'),
        'shell_black': ([
            'diff_left.stl', 'diff_right.stl', 'servo_mount.stl', 'motor_mount.stl',
            'gearbox_left_a.stl', 'gearbox_left_b.stl', 'gearbox_right.stl',
            'shaft_lock_left.stl', 'shaft_lock_right.stl', 'axle_support.stl',
        ], '0.08 0.08 0.09 1'),
    }
    out = []
    for name, (files, rgba) in groups.items():
        body = ''.join(_mesh(f, rgba, name) for f in files)
        out.append(_fixed(name, body, SHELL_JOINT, SHELL_RPY))
    # Four M3 posts from the top of the bottom plate to the underside of the
    # middle plate. Their CAD puts that gap at 50 mm, inset from the plate corners.
    posts = [(-0.020, 0.038), (-0.020, -0.038), (0.122, 0.038), (0.122, -0.038)]
    brass = ''.join(
        f'    <visual><origin xyz="{x:.3f} {y:.3f} -0.007"/>'
        f'<geometry><cylinder radius="0.003" length="0.050"/></geometry>'
        f'<material name="brass"><color rgba="0.72 0.58 0.28 1"/></material></visual>\n'
        for x, y in posts
    )
    out.append(_fixed('shell_brass', brass))
    return ''.join(out)


def _paint() -> str:
    paints = {
        'shell_white': '0.93 0.93 0.91',
        'shell_orange': '0.95 0.40 0.06',
        'shell_blue': '0.15 0.32 0.70',
        'shell_black': '0.08 0.08 0.09',
        'shell_brass': '0.74 0.60 0.30',
        'rear_left_wheel': '0.94 0.94 0.92',
        'rear_right_wheel': '0.94 0.94 0.92',
        'front_left_wheel': '0.94 0.94 0.92',
        'front_right_wheel': '0.94 0.94 0.92',
    }
    lines = []
    for link, rgb in paints.items():
        lines.append(
            f'  <gazebo reference="{link}"><material>'
            f'<ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>'
            f'<specular>0.20 0.20 0.18 1</specular></material></gazebo>'
        )
    return '\n'.join(lines)


def _wheel_visual() -> str:
    # Their wheel STL is in metres, axle along Z, radius 22.5 mm. Scale it to the plant.
    scale = chassis.WHEEL_RADIUS_M / 0.0225
    drop = -0.0085 * scale / 2
    return (
        f'    <visual><origin xyz="0 0 {drop:.5f}"/>'
        f'<geometry><mesh filename="{MESH}/wheel.stl" scale="{scale:.5f} {scale:.5f} {scale:.5f}"/>'
        f'</geometry><material name="wheel_white"><color rgba="0.95 0.95 0.93 1"/></material></visual>\n'
    )


def _png(width, height, rgb) -> bytes:
    """One uncompressed RGB PNG. Deterministic, no image library."""
    def chunk(tag, data):
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', crc)
    rows = []
    for y in range(height):
        row = bytearray(b'\x00')
        for x in range(width):
            row.extend(rgb(x, y))
        rows.append(bytes(row))
    raw = b''.join(rows)
    header = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    body = chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b'')
    return b'\x89PNG\r\n\x1a\n' + body


def floor_png() -> bytes:
    rng = random.Random(23)

    def rgb(x, y):
        n = rng.random()
        v = int(168 + 18 * math.sin(x / 17) * math.sin(y / 23) + 10 * n)
        # A faint joint every 32 px, so the floor reads as a workshop slab.
        if x % 64 == 0 or y % 64 == 0:
            v = max(120, v - 28)
        return (v, v - 2, v - 6)
    return _png(256, 256, rgb)


def model_urdf() -> str:
    ix, iy, iz = _inertia_box(1.5, chassis.LENGTH_M, chassis.WIDTH_M, chassis.HEIGHT_M)
    wx, wy, wz = _inertia_cyl(0.05, chassis.WHEEL_RADIUS_M, 0.008)
    half = chassis.TRACK_M / 2
    pitch = math.radians(CAMERA_PITCH_DEG)
    # Box sits mostly forward of the rear axle, 3.5 cm of it behind, matching their len_offs.
    box_x = chassis.LENGTH_M / 2 - 0.035
    wheel_z = chassis.WHEEL_RADIUS_M - SPAWN_Z
    wheel = _wheel_visual()
    shell = _shell_links()
    paint = _paint()
    return f"""<?xml version="1.0"?>
<!-- GENERATED by rupsha_sim.ackermann_gen -- do not edit.
     wheelbase {chassis.WHEELBASE_M} steer_limit {chassis.STEER_LIMIT_RAD:.6f}
     ticks_per_metre {chassis.TPM} -->
<robot name="{NAME}">
  <link name="base_link"/>
  <link name="chassis">
    <collision>
      <origin xyz="{box_x:.4f} 0 0"/>
      <geometry><box size="{chassis.LENGTH_M} {chassis.WIDTH_M} {chassis.HEIGHT_M}"/></geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="{ix:.6e}" ixy="0" ixz="0" iyy="{iy:.6e}" iyz="0" izz="{iz:.6e}"/>
    </inertial>
  </link>
  <joint name="chassis_joint" type="fixed">
    <parent link="base_link"/><child link="chassis"/>
  </joint>
{shell}
  <link name="camera_link">
    <inertial><mass value="0.02"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>
  <joint name="camera_joint" type="fixed">
    <origin xyz="0.172 0 0.070" rpy="0 {pitch:.5f} 0"/>
    <parent link="chassis"/><child link="camera_link"/>
  </joint>

  <link name="rear_left_wheel">
    <collision><geometry><cylinder radius="{chassis.WHEEL_RADIUS_M}" length="0.008"/></geometry></collision>
{wheel}
    <inertial><mass value="0.05"/>
      <inertia ixx="{wx:.6e}" ixy="0" ixz="0" iyy="{wy:.6e}" iyz="0" izz="{wz:.6e}"/></inertial>
  </link>
  <joint name="rear_left_wheel_joint" type="continuous">
    <origin xyz="0 {half:.4f} {wheel_z:.4f}" rpy="-1.5708 0 0"/>
    <parent link="chassis"/><child link="rear_left_wheel"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="rear_right_wheel">
    <collision><geometry><cylinder radius="{chassis.WHEEL_RADIUS_M}" length="0.008"/></geometry></collision>
{wheel}
    <inertial><mass value="0.05"/>
      <inertia ixx="{wx:.6e}" ixy="0" ixz="0" iyy="{wy:.6e}" iyz="0" izz="{wz:.6e}"/></inertial>
  </link>
  <joint name="rear_right_wheel_joint" type="continuous">
    <origin xyz="0 {-half:.4f} {wheel_z:.4f}" rpy="-1.5708 0 0"/>
    <parent link="chassis"/><child link="rear_right_wheel"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_steering">
    <inertial><mass value="0.02"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>
  <joint name="left_wheel_steering_joint" type="revolute">
    <origin xyz="{chassis.WHEELBASE_M} {half:.4f} {wheel_z:.4f}"/>
    <parent link="chassis"/><child link="left_steering"/>
    <axis xyz="0 0 1"/>
    <limit effort="5" lower="-{chassis.STEER_LIMIT_RAD:.6f}" upper="{chassis.STEER_LIMIT_RAD:.6f}" velocity="3"/>
  </joint>
  <link name="right_steering">
    <inertial><mass value="0.02"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>
  <joint name="right_wheel_steering_joint" type="revolute">
    <origin xyz="{chassis.WHEELBASE_M} {-half:.4f} {wheel_z:.4f}"/>
    <parent link="chassis"/><child link="right_steering"/>
    <axis xyz="0 0 1"/>
    <limit effort="5" lower="-{chassis.STEER_LIMIT_RAD:.6f}" upper="{chassis.STEER_LIMIT_RAD:.6f}" velocity="3"/>
  </joint>

  <link name="front_left_wheel">
    <collision><geometry><cylinder radius="{chassis.WHEEL_RADIUS_M}" length="0.008"/></geometry></collision>
{wheel}
    <inertial><mass value="0.05"/>
      <inertia ixx="{wx:.6e}" ixy="0" ixz="0" iyy="{wy:.6e}" iyz="0" izz="{wz:.6e}"/></inertial>
  </link>
  <joint name="front_left_wheel_joint" type="continuous">
    <origin xyz="0 0 0" rpy="-1.5708 0 0"/>
    <parent link="left_steering"/><child link="front_left_wheel"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="front_right_wheel">
    <collision><geometry><cylinder radius="{chassis.WHEEL_RADIUS_M}" length="0.008"/></geometry></collision>
{wheel}
    <inertial><mass value="0.05"/>
      <inertia ixx="{wx:.6e}" ixy="0" ixz="0" iyy="{wy:.6e}" iyz="0" izz="{wz:.6e}"/></inertial>
  </link>
  <joint name="front_right_wheel_joint" type="continuous">
    <origin xyz="0 0 0" rpy="-1.5708 0 0"/>
    <parent link="right_steering"/><child link="front_right_wheel"/>
    <axis xyz="0 0 1"/>
  </joint>

  <ros2_control name="GazeboSystem" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/><state_interface name="position"/>
    </joint>
    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/><state_interface name="position"/>
    </joint>
    <joint name="left_wheel_steering_joint">
      <command_interface name="position"/><state_interface name="position"/>
    </joint>
    <joint name="right_wheel_steering_joint">
      <command_interface name="position"/><state_interface name="position"/>
    </joint>
    <joint name="front_left_wheel_joint">
      <state_interface name="position"/><state_interface name="velocity"/>
    </joint>
    <joint name="front_right_wheel_joint">
      <state_interface name="position"/><state_interface name="velocity"/>
    </joint>
  </ros2_control>

  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>{contract.CONTROLLERS_TOKEN}</parameters>
      <position_proportional_gain>1.0</position_proportional_gain>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_topic>{contract.GZ_ODOMETRY}</odom_topic>
      <dimensions>3</dimensions>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>
  </gazebo>
  <gazebo reference="camera_link">
    <sensor name="front_camera" type="camera">
      <always_on>1</always_on><update_rate>{contract.CAMERA_RATE_HZ}</update_rate>
      <topic>{contract.GZ_CAMERA_FRONT}</topic>
      <camera_info_topic>{contract.GZ_CAMERA_FRONT_INFO}</camera_info_topic>
      <camera><horizontal_fov>{contract.CAMERA_HFOV_RAD}</horizontal_fov>
        <image><width>{contract.CAMERA_WIDTH}</width><height>{contract.CAMERA_HEIGHT}</height>
          <format>R8G8B8</format></image>
        <clip><near>0.05</near><far>30</far></clip></camera>
    </sensor>
  </gazebo>
  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on><update_rate>200</update_rate>
      <topic>{contract.GZ_IMU}</topic>
    </sensor>
    <sensor name="navsat_sensor" type="navsat">
      <always_on>1</always_on><update_rate>10</update_rate>
      <topic>{contract.GZ_NAVSAT}</topic>
    </sensor>
  </gazebo>
  <gazebo reference="rear_left_wheel"><mu1>1.2</mu1><mu2>1.2</mu2></gazebo>
  <gazebo reference="rear_right_wheel"><mu1>1.2</mu1><mu2>1.2</mu2></gazebo>
  <gazebo reference="front_left_wheel"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>
  <gazebo reference="front_right_wheel"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>
{paint}
</robot>
"""


def world_sdf() -> str:
    return f"""<?xml version="1.0"?>
<!-- GENERATED by rupsha_sim.ackermann_gen -- do not edit. -->
<sdf version="1.9">
  <world name="ackermann_ci">
    <physics name="1ms" type="ignore"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>-35.363262</latitude_deg><longitude_deg>149.165237</longitude_deg>
      <elevation>584.0</elevation><heading_deg>0</heading_deg>
    </spherical_coordinates>
    <scene><ambient>0.85 0.85 0.84 1</ambient><background>0.62 0.66 0.70 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 30 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse><specular>0.15 0.15 0.13 1</specular>
      <direction>-0.4 0.25 -0.85</direction></light>
    <light type="directional" name="fill"><cast_shadows>false</cast_shadows><pose>0 0 8 0 0 0</pose>
      <diffuse>0.35 0.38 0.42 1</diffuse><specular>0 0 0 1</specular>
      <direction>0.5 -0.4 -0.8</direction></light>
    <model name="ground"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        <material><ambient>0.75 0.74 0.72 1</ambient><diffuse>0.78 0.77 0.74 1</diffuse>
          <pbr><metal>
            <albedo_map>model://rupsha_ackermann/materials/textures/floor.png</albedo_map>
            <roughness>1</roughness><metalness>0</metalness>
          </metal></pbr>
        </material></visual>
    </link></model>
    <include><uri>model://aruco_post_{POST_TAG}</uri><name>post_{POST_TAG}</name>
      <pose>{POST_EAST_M} 0 0 0 0 0</pose></include>
  </world>
</sdf>
"""


def controllers_yaml() -> str:
    return f"""# GENERATED by rupsha_sim.ackermann_gen -- do not edit.
controller_manager:
  ros__parameters:
    update_rate: {chassis.CONTROL_HZ}

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    drive_velocity:
      type: velocity_controllers/JointGroupVelocityController

    steer_position:
      type: position_controllers/JointGroupPositionController

drive_velocity:
  ros__parameters:
    joints:
      - rear_left_wheel_joint
      - rear_right_wheel_joint

steer_position:
  ros__parameters:
    joints:
      - left_wheel_steering_joint
      - right_wheel_steering_joint
"""


def outputs() -> dict:
    return {
        PKG / 'models' / NAME / 'model.urdf': model_urdf(),
        PKG / 'worlds' / 'ackermann_ci.sdf': world_sdf(),
        PKG / 'config' / 'ackermann_bridge.yaml': contract.ackermann_bridge_yaml(),
        PKG / 'config' / 'ackermann_controllers.yaml': controllers_yaml(),
        PKG / 'models' / NAME / 'materials' / 'textures' / 'floor.png': floor_png(),
    }
