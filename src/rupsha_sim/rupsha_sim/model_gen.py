"""Generate the rupsha_skid Gazebo model from one table of numbers.

A frame-agnostic 6WD skid-steer stand-in until the real chassis is measured:
the geometry below is a plausible URC-class rover, not Rupsha. Every number that
the ArduPilot side also depends on (wheel radius -> plugin multiplier ->
rupsha_gazebo.parm) lives here once, and test_sim_contract_drift checks the
generated files are fresh.

IMU: ArduPilotPlugin rotates the *pose* into FRD/NED but forwards the IMU
sensor's gyro and accel untouched, so the sensor itself must be mounted
x-forward/y-right/z-down (rolled 180 deg). Left z-up, yaw rate reaches the
board negated: the steering rate loop becomes positive feedback and the rover
spins at full output while the attitude still looks right.

Channels: SERVO1 = ThrottleLeft (73) -> plugin channel 0, SERVO3 = ThrottleRight
(74) -> plugin channel 2, as rupsha_baseline.parm sets them.
"""

from __future__ import annotations

import math
import pathlib

from . import contract

PKG = pathlib.Path(__file__).resolve().parent.parent
NAME = 'rupsha_skid'

CHASSIS = (0.70, 0.46, 0.16)       # x, y, z (m)
CHASSIS_MASS = 14.0
CHASSIS_Z = 0.20                   # chassis centre height
WHEEL_R = 0.10
WHEEL_W = 0.07
WHEEL_MASS = 0.6
WHEEL_X = (0.25, 0.0, -0.25)
WHEEL_Y = 0.285
WHEEL_MU = 0.9
MAX_WHEEL_RADPS = 20.0             # +-20 rad/s at full throttle -> +-2 m/s
CAMERA_XYZ = (0.33, 0.0, 0.42)
# Positive pitch in this SDF looks down. The post tag's centre is ~0.88 m up and
# the camera is ~0.44 m up, so at the 1.5 m standoff the tag is ~17 deg above the
# horizon. Nose-down put it on the top edge of the frame by the time the approach
# stopped, and the verify window then saw only the pole.
CAMERA_PITCH_DEG = -17.0


def _inertia_box(m, x, y, z):
    return (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12)


def _inertia_cyl(m, r, h):
    ixx = m * (3 * r * r + h * h) / 12
    return (ixx, ixx, m * r * r / 2)


def _inertial(m, ixx, iyy, izz):
    return (f'<inertial><mass>{m}</mass><inertia><ixx>{ixx:.5f}</ixx><iyy>{iyy:.5f}</iyy>'
            f'<izz>{izz:.5f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


def wheels():
    for side, sy in (('left', 1), ('right', -1)):
        for k, x in enumerate(WHEEL_X):
            yield f'{side}_{k}', side, x, sy * WHEEL_Y


def model_sdf() -> str:
    cx, cy, cz = CHASSIS
    links = [f"""
    <link name="base_link">
      <pose>0 0 {CHASSIS_Z} 0 0 0</pose>
      {_inertial(CHASSIS_MASS, *_inertia_box(CHASSIS_MASS, cx, cy, cz))}
      <collision name="collision"><geometry><box><size>{cx} {cy} {cz}</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>{cx} {cy} {cz}</size></box></geometry>
        <material><ambient>0.15 0.35 0.55 1</ambient><diffuse>0.15 0.35 0.55 1</diffuse></material></visual>
      <sensor name="imu_sensor" type="imu"><pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on><update_rate>400</update_rate></sensor>
    </link>
    <link name="camera_link">
      <pose relative_to="base_link">{CAMERA_XYZ[0]} {CAMERA_XYZ[1]} {CAMERA_XYZ[2] - CHASSIS_Z} 0 {math.radians(CAMERA_PITCH_DEG):.5f} 0</pose>
      {_inertial(0.05, 1e-5, 1e-5, 1e-5)}
      <visual name="visual"><geometry><box><size>0.04 0.09 0.03</size></box></geometry></visual>
      <sensor name="front_camera" type="camera">
        <always_on>1</always_on><update_rate>{contract.CAMERA_RATE_HZ}</update_rate>
        <topic>{contract.GZ_CAMERA_FRONT}</topic>
        <camera_info_topic>{contract.GZ_CAMERA_FRONT_INFO}</camera_info_topic>
        <camera><horizontal_fov>{contract.CAMERA_HFOV_RAD}</horizontal_fov>
          <image><width>{contract.CAMERA_WIDTH}</width><height>{contract.CAMERA_HEIGHT}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>60</far></clip></camera>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>"""]
    joints, controls = [], []
    for name, side, x, y in wheels():
        links.append(f"""
    <link name="wheel_{name}">
      <pose relative_to="base_link">{x} {y} {WHEEL_R - CHASSIS_Z} -1.5707963 0 0</pose>
      {_inertial(WHEEL_MASS, *_inertia_cyl(WHEEL_MASS, WHEEL_R, WHEEL_W))}
      <collision name="collision"><geometry><cylinder><radius>{WHEEL_R}</radius><length>{WHEEL_W}</length></cylinder></geometry>
        <surface><friction><ode><mu>{WHEEL_MU}</mu><mu2>{WHEEL_MU}</mu2></ode></friction></surface></collision>
      <visual name="visual"><geometry><cylinder><radius>{WHEEL_R}</radius><length>{WHEEL_W}</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
    </link>""")
        # Wheel frame is rolled -90 deg about x, so its z axis points to +y (left).
        # Positive spin about +y rolls the rover forward: axis = 0 0 1 in the wheel frame.
        joints.append(f"""
    <joint name="wheel_{name}_joint" type="revolute"><parent>base_link</parent><child>wheel_{name}</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>""")
        channel = 0 if side == 'left' else 2
        controls.append(f"""
      <control channel="{channel}">
        <jointName>wheel_{name}_joint</jointName>
        <useForce>0</useForce>
        <multiplier>{2 * MAX_WHEEL_RADPS}</multiplier>
        <offset>-0.5</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
        <type>VELOCITY</type>
      </control>""")
    plugin = f"""
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>base_link::imu_sensor</imuName>{''.join(controls)}
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>{contract.MODEL}</robot_base_frame>
      <odom_topic>{contract.GZ_ODOMETRY}</odom_topic>
      <dimensions>3</dimensions>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>"""
    return (f'<?xml version="1.0"?>\n<!-- GENERATED by rupsha_sim.model_gen -- do not edit. -->\n'
            f'<sdf version="1.9">\n  <model name="{NAME}">{"".join(links)}{"".join(joints)}{plugin}\n  </model>\n</sdf>\n')


def model_config() -> str:
    return (f'<?xml version="1.0"?>\n<model>\n  <name>{NAME}</name>\n  <version>0.1</version>\n'
            f'  <sdf version="1.9">model.sdf</sdf>\n  <author><name>Muhammad Fahim Faisal</name></author>\n'
            f'  <description>Rupsha 6WD skid-steer stand-in (generated)</description>\n</model>\n')


def outputs() -> dict:
    d = PKG / 'models' / NAME
    return {d / 'model.sdf': model_sdf(), d / 'model.config': model_config()}
