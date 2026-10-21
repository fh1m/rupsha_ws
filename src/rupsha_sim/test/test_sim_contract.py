"""The sim's generated files, its ROS contract and its SITL tuning cannot drift.

Fast and simulator-free: every number checked here is recomputed from the one
table it came from (model_gen.py, contract.py, the manifest), so a change in one
place that is not carried to the others fails here rather than as a rover that
spins in Gazebo.
"""

import math
import pathlib
import xml.etree.ElementTree as ET

import pytest
import yaml

from rupsha_control.manifest import Topics
from rupsha_sim import contract, model_gen, worldgen

PKG = pathlib.Path(__file__).resolve().parent.parent
GAZEBO_PARM = PKG / 'config' / 'ardupilot' / 'rupsha_gazebo.parm'
MEASURED_SCRUB = 0.85        # 185 deg/s measured vs 217 ideal at RC 1800 (2026-10-21)


def parm(path):
    out = {}
    for line in path.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if line:
            k, v = line.replace(',', ' ').split()[:2]
            out[k] = float(v)
    return out


def test_generated_files_are_fresh():
    stale = []
    for path, content in worldgen.outputs().items():
        if not worldgen.same_content(path, content):
            stale.append(str(path.relative_to(PKG)))
    assert not stale, f'run python3 -m rupsha_sim.worldgen: {stale}'


@pytest.mark.parametrize('name', sorted(worldgen.WORLDS))
def test_worldgen_is_deterministic(name):
    spec = worldgen.WORLDS[name]
    assert worldgen.scenario(name, **spec) == worldgen.scenario(name, **spec)
    assert worldgen.world_sdf(worldgen.scenario(name, **spec)) == worldgen.world_sdf(
        worldgen.scenario(name, **spec))


def _detect_ids(img):
    import cv2
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if tuple(int(x) for x in cv2.__version__.split('.')[:2]) >= (4, 7):
        _, ids, _ = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters()).detectMarkers(img)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(img, d, parameters=cv2.aruco.DetectorParameters_create())
    return [] if ids is None else sorted(int(i) for i in ids.ravel())


def test_marker_textures_are_the_tags_they_claim():
    import cv2
    import numpy as np
    pngs = sorted(PKG.glob('models/aruco_post_*/materials/textures/marker_*.png'))
    assert pngs
    for png in pngs:
        tag = int(png.stem.split('_')[1])
        img = cv2.imdecode(np.frombuffer(png.read_bytes(), np.uint8), cv2.IMREAD_GRAYSCALE)
        assert _detect_ids(img) == [tag], png.name


def test_bridge_carries_the_manifest_topics():
    rows = {r['gz_topic_name']: r for r in yaml.safe_load((PKG / 'config' / 'bridge.yaml').read_text())}
    assert rows[contract.GZ_CAMERA_FRONT]['ros_topic_name'] == Topics.CAMERA_FRONT
    assert rows[contract.GZ_CAMERA_FRONT_INFO]['ros_topic_name'] == Topics.CAMERA_FRONT_INFO
    assert rows[contract.GZ_ODOMETRY]['ros_topic_name'] == contract.GROUND_TRUTH_ODOMETRY
    assert all(r['direction'] == 'GZ_TO_ROS' for r in rows.values())


def _model():
    return ET.fromstring(model_gen.model_sdf().split('\n', 2)[2])


def test_model_publishes_where_the_contract_says():
    cam = _model().find(".//sensor[@type='camera']")
    assert cam.findtext('topic') == contract.GZ_CAMERA_FRONT
    assert cam.findtext('camera_info_topic') == contract.GZ_CAMERA_FRONT_INFO
    assert int(cam.findtext('camera/image/width')) == contract.CAMERA_WIDTH
    assert float(cam.findtext('camera/horizontal_fov')) == pytest.approx(contract.CAMERA_HFOV_RAD)


def test_imu_is_mounted_frd():
    """ArduPilotPlugin forwards gyro/accel unrotated; a z-up IMU negates yaw rate
    and the steering loop runs away (seen 2026-10-21: full-output spin)."""
    m = _model()
    imu = m.find(".//sensor[@type='imu']")
    roll, pitch, yaw = (float(x) for x in imu.findtext('pose').split()[3:])
    assert imu.find('pose').get('degrees') == 'true'
    assert (roll, pitch, yaw) == (180.0, 0.0, 0.0)
    link = next(lk for lk in m.iter('link') if lk.find(".//sensor[@type='imu']") is not None)
    assert m.find('.//plugin/imuName').text == f"{link.get('name')}::{imu.get('name')}"


def test_wheel_channels_match_the_baseline_servo_functions():
    controls = _model().findall('.//plugin/control')
    by_side = {}
    for c in controls:
        by_side.setdefault(c.findtext('jointName').split('_')[1], set()).add(int(c.get('channel')))
    assert by_side == {'left': {0}, 'right': {2}}       # SERVO1 = 73 left, SERVO3 = 74 right


def test_sitl_cruise_pair_matches_the_model():
    p = parm(GAZEBO_PARM)
    full_mps = model_gen.MAX_WHEEL_RADPS * model_gen.WHEEL_R
    assert p['CRUISE_SPEED'] == pytest.approx(p['CRUISE_THROTTLE'] / 100 * full_mps, rel=0.02)


def test_sitl_steering_feedforward_matches_the_model():
    p = parm(GAZEBO_PARM)
    full_mps = model_gen.MAX_WHEEL_RADPS * model_gen.WHEEL_R
    max_rate = 2 * full_mps / (2 * model_gen.WHEEL_Y) * MEASURED_SCRUB
    assert p['ATC_STR_RAT_FF'] == pytest.approx(1 / max_rate, rel=0.1)
    assert p['ATC_STR_RAT_MAX'] < math.degrees(max_rate)


def test_sitl_has_one_imu_like_the_board():
    p = parm(GAZEBO_PARM)
    assert p['INS_ENABLE_MASK'] == 1
    assert all(p[f'INS_ACC2OFFS_{a}'] == 0 and p[f'INS_ACC2SCAL_{a}'] == 1 for a in 'XYZ')
