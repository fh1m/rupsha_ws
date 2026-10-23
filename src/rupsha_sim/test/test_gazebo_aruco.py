"""ArUco range from the simulated camera, against Gazebo's geometry.

The rover drives toward post 3 of flat_ci and stops at three distances. At each
stop the front camera's frames go through rupsha_vision's detector, and the
range it reports is compared with the distance from the camera to the centre of
the tag face it sees, computed from Gazebo's model pose and the world file --
never from the EKF or from the detector's own pose.

    source install/setup.bash && pytest -m gazebo -s src/rupsha_sim/test/test_gazebo_aruco.py
"""

import math
import statistics
import time

import numpy as np
import pytest
import yaml

from rupsha_sim import contract, gazebo, worldgen
from rupsha_sim.sitl import find_ardupilot

pytestmark = [
    pytest.mark.gazebo,
    pytest.mark.skip(reason='ArduRover is not part of this stack. The ArUco bars stay in the ledger as the previous body.'),
    pytest.mark.skipif(find_ardupilot() is None, reason='ArduRover SITL not built'),
    pytest.mark.skipif(gazebo.available() is not None, reason=str(gazebo.available())),
]

TAG_ID = 3
# a 20 cm tag decodes reliably from ~20 px; 6 m (16 px) is the edge, 8 m (11 px) never (measured-bars §3)
STATIONS_M = (4.5, 3.0)
FRAMES = 10


def scenario():
    return yaml.safe_load((worldgen.PKG / 'scenarios' / 'flat_ci.yaml').read_text())


def post(tag_id):
    p = next(p for p in scenario()['posts'] if p['tag_id'] == tag_id)
    return p['east'], p['north']


def frames(n, timeout=10.0):
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node
    got = []
    node = Node()
    node.subscribe(Image, f'/{contract.GZ_CAMERA_FRONT}', got.append)
    deadline = time.monotonic() + timeout
    while len(got) < n and time.monotonic() < deadline:
        time.sleep(0.02)
    return [np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3) for m in got[:n]]


def test_aruco_range_matches_gazebo_geometry(gz_rover):
    import cv2

    from rupsha_control import geo
    from rupsha_vision.aruco import ArucoTagDetector
    from rupsha_vision.camera_model import PinholeCamera
    fc, r = gz_rover
    pe, pn = post(TAG_ID)
    home = scenario()['home']                   # the world's geodetic origin
    home_lat, home_lon = home['lat'], home['lon']
    camera = PinholeCamera.from_fov(contract.CAMERA_WIDTH, contract.CAMERA_HEIGHT,
                                    math.degrees(contract.CAMERA_HFOV_RAD))
    detector = ArucoTagDetector(marker_size_m=worldgen.MARKER_M)
    e0, n0, _ = gazebo.truth()
    away = np.array([e0 - pe, n0 - pn]) / math.hypot(e0 - pe, n0 - pn)
    rows = []
    for d in STATIONS_M:
        se, sn = pe + d * away[0], pn + d * away[1]
        lat, lon = geo.offset(home_lat, home_lon, sn, se)
        assert r.run('goto_gnss', lat=lat, lon=lon, radius=0.5, timeout=60).success
        e, n, _ = gazebo.truth()
        facing = math.degrees(math.atan2(pe - e, pn - n)) % 360.0
        assert r.run('turn_to', heading=facing, timeout=20).success
        time.sleep(1.0)
        e, n, h = gazebo.truth()
        truth_m = gazebo.true_range(e, n, h, pe, pn)
        ranges = []
        sides = []
        for img in frames(FRAMES):
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            hits = [o for o in detector.detect(gray, camera) if o.tag_id == TAG_ID and o.has_pose]
            ranges += [o.range_m for o in hits]
            sides += [o.side_px for o in hits]
        seen = len(ranges) / FRAMES
        est = statistics.median(ranges) if ranges else math.nan
        rows.append((d, truth_m, est, seen))
        print(f'station {d:.1f} m: truth {truth_m:.3f} m, aruco {est:.3f} m, '
              f'error {est - truth_m:+.3f} m ({(est - truth_m) / truth_m:+.1%}), '
              f'tag {statistics.median(sides) if sides else math.nan:.1f} px, seen in {seen:.0%} of frames')
    for d, truth_m, est, seen in rows:
        assert seen >= 0.8, f'tag {TAG_ID} seen in {seen:.0%} of frames at {d} m'
        assert abs(est - truth_m) <= 0.05 * truth_m + 0.05, \
            f'at {d} m: aruco {est:.3f} m vs truth {truth_m:.3f} m'
