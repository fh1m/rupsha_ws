"""The whole stack runs a marker leg in Gazebo and ends at the post, by Gazebo's truth.

The SITL launch is gone. This test is the record of one run on the previous
skid body. It does not start ArduRover. The leg was B-32's: the GNSS point 6 m short of post 3 in flat_ci, so
the first sighting is at the decode edge. The rover must search, approach and verify,
and the score is where /sim/ground_truth/odometry says the rover stopped, never
/rupsha/state or the executive's own word.

    source install/setup.bash && GZ_VERSION=harmonic pytest -m gazebo -s \\
        src/rupsha_sim/test/test_gazebo_marker_leg.py
"""

import math
import os
import signal
import subprocess
import time

import pytest
import yaml

from rupsha_sim import contract, gazebo, worldgen

pytestmark = [
    pytest.mark.gazebo,
    pytest.mark.skip(reason='ArduRover is not part of this stack. The 1.69 m leg stays in the ledger as the previous body.'),
]

TAG_ID = 3
SHORT_OF_POST_M = 6.0         # B-32: the first sighting is at the 6 m decode edge
LEG_BUDGET_S = 420.0
# The executive's defaults: verified when the tag reads <= standoff + slack.
STANDOFF_M, SLACK_M = 1.5, 1.0
CLEAR_OF_POST_M = 0.8         # body centre to post centre: closer is a collision


def scenario():
    return yaml.safe_load((worldgen.PKG / 'scenarios' / 'flat_ci.yaml').read_text())


def test_marker_leg_ends_at_the_post_by_truth(tmp_path):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    from rupsha_control import geo
    from rupsha_control.manifest import AutonomyState, Topics
    from rupsha_interfaces.msg import AutonomyStatus, Detections, RupshaState
    from rupsha_planner.client import RosPort

    sc = scenario()
    home = sc['home']
    post = next(p for p in sc['posts'] if p['tag_id'] == TAG_ID)
    pe, pn = post['east'], post['north']
    d = math.hypot(pe, pn)
    leg_n, leg_e = pn * (1 - SHORT_OF_POST_M / d), pe * (1 - SHORT_OF_POST_M / d)
    leg_lat, leg_lon = geo.offset(home['lat'], home['lon'], leg_n, leg_e)

    log = open(tmp_path / 'launch.log', 'w')
    env = dict(os.environ, GZ_VERSION=os.environ.get('GZ_VERSION', 'harmonic'))
    launch = subprocess.Popen(['ros2', 'launch', 'rupsha_sim', 'sitl.launch.py', 'headless:=true'],
                              stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    port = None
    try:
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node('marker_leg_test')
        track, status, board = [], {}, {}

        def on_truth(m):
            p = m.pose.pose.position
            track.append((p.x, p.y, gazebo.heading_from_quaternion(m.pose.pose.orientation),
                          time.monotonic()))

        node.create_subscription(Odometry, contract.GROUND_TRUTH_ODOMETRY, on_truth, 10)
        node.create_subscription(AutonomyStatus, Topics.AUTONOMY_STATUS,
                                 lambda m: status.update(state=m.state, detail=m.detail), 10)
        node.create_subscription(RupshaState, Topics.STATE, lambda m: board.update(m=m), 10)
        seen = []
        node.create_subscription(
            Detections, Topics.DETECTIONS,
            lambda m: seen.extend((time.monotonic(), d.tag_id, d.bearing_deg, d.range_m) for d in m.detections),
            qos_profile_sensor_data)
        port = RosPort(node, connect_timeout=120.0)

        deadline = time.monotonic() + 180.0
        armed = None
        while time.monotonic() < deadline:
            m = board.get('m')
            if m is not None and m.prearm_ok and m.ekf_ok and m.gps_fix >= 3:
                armed = port.run('arm')
                if armed.success:
                    break
            time.sleep(2.0)
        assert armed is not None and armed.success, \
            f'never armed: {armed.message if armed else board.get("m")} (log {tmp_path})'
        assert _wait(lambda: 'state' in status, 60.0), 'the executive never published its status'

        assert port.run('add_marker_leg', lat=leg_lat, lon=leg_lon, tag_id=TAG_ID, radius=20.0).success
        assert port.run('start_autonomy').success
        start = time.monotonic()
        _wait(lambda: status.get('state') != int(AutonomyState.IDLE), 30.0)
        _wait(lambda: status.get('state') in (int(AutonomyState.IDLE), int(AutonomyState.STUCK)),
              LEG_BUDGET_S)
        took = time.monotonic() - start
        assert track, 'no ground truth from Gazebo'
        # Kept for the next person who has to explain a failure.
        (tmp_path / 'truth.csv').write_text(
            't,east,north,heading\n' + ''.join(f'{t:.2f},{e:.3f},{n:.3f},{h:.1f}\n' for e, n, h, t in track[::5]))
        (tmp_path / 'detections.csv').write_text(
            't,tag_id,bearing_deg,range_m\n' + ''.join(f'{t:.2f},{i},{b:.1f},{r:.2f}\n' for t, i, b, r in seen))
        e, n, h, _ = track[-1]
        final = gazebo.true_range(e, n, h, pe, pn)
        closest = min(math.hypot(te - pe, tn - pn) for te, tn, _, _ in track)
        print(f'leg: {status.get("detail")!r} in {took:.0f} s; truth: camera {final:.2f} m from the '
              f'tag face, body closest {closest:.2f} m to the post')
        assert 'all 1 legs complete' in status.get('detail', ''), status
        assert final <= STANDOFF_M + SLACK_M, f'stopped {final:.2f} m from the tag by truth'
        assert closest >= CLEAR_OF_POST_M, f'drove to {closest:.2f} m of the post centre'
    finally:
        if port is not None:
            port.run('disable_autonomy')
            port.run('disarm')
            port.close()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
        log.close()


def _wait(pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.5)
    return False
