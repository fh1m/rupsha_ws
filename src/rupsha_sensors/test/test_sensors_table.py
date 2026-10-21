import pathlib

import pytest

from rupsha_control.manifest import Topics
from rupsha_sensors.sensors_check import ABSENT, FAIL, PASS, evaluate, verdict
from rupsha_sensors.table import TableError, load, node_spec

CAMERA = """
sensors:
  front_camera:
    driver: {driver}
    topic: CAMERA_FRONT
    info_topic: CAMERA_FRONT_INFO
    frame_id: camera_front_optical
    device: /dev/video0
    width: 640
    height: 480
    fps: 15
    min_hz: 10
    {extra}
"""


def table(tmp_path, text):
    p = tmp_path / 'sensors.yaml'
    p.write_text(text)
    return p


def test_shipped_table_loads_and_names_manifest_topics():
    sensors = load()
    assert {s.topic for s in sensors} >= {Topics.CAMERA_FRONT, Topics.BOARD_IMU}
    assert all(s.min_hz > 0 for s in sensors)


def test_unknown_topic_is_refused(tmp_path):
    with pytest.raises(TableError, match='not a manifest Topics'):
        load(table(tmp_path, CAMERA.format(driver='v4l2_camera', extra='').replace(
            'topic: CAMERA_FRONT\n', 'topic: /camera/image_raw\n')))


def test_unknown_driver_is_refused(tmp_path):
    with pytest.raises(TableError, match='driver'):
        load(table(tmp_path, CAMERA.format(driver='gstreamer', extra='')))


def test_missing_calibration_file_is_refused(tmp_path):
    with pytest.raises(TableError, match='calibration'):
        load(table(tmp_path, CAMERA.format(driver='v4l2_camera', extra='calibration: nope.yaml')))


def test_disabled_rows_are_skipped(tmp_path):
    assert load(table(tmp_path, CAMERA.format(driver='v4l2_camera', extra='enabled: false'))) == []


@pytest.mark.parametrize('driver,package', [('v4l2_camera', 'v4l2_camera'), ('camera_ros', 'camera_ros')])
def test_camera_drivers_publish_on_the_manifest_topics(tmp_path, driver, package):
    (s,) = load(table(tmp_path, CAMERA.format(driver=driver, extra='')))
    spec = node_spec(s, tmp_path)
    assert spec.package == package
    targets = dict(spec.remappings)
    assert set(targets.values()) == {Topics.CAMERA_FRONT, Topics.CAMERA_FRONT_INFO}


def test_board_rows_launch_nothing():
    board = [s for s in load() if s.driver == 'board']
    assert board and all(node_spec(s, pathlib.Path('.')) is None for s in board)


def test_check_separates_absent_from_slow():
    sensors = [s for s in load() if s.driver != 'board'] + [s for s in load() if s.driver == 'board']
    cam, imu, fix = sensors[0], sensors[1], sensors[2]
    stamps = {cam.topic: [i / 15 for i in range(30)],          # 15 Hz
              imu.topic: [i / 2 for i in range(6)]}            # 2 Hz, needs 8
    rows = {name: v for v, name, _ in evaluate([cam, imu, fix], stamps, 2.0)}
    assert rows == {cam.name: PASS, imu.name: FAIL, fix.name: ABSENT}
    assert verdict(evaluate([cam], stamps, 2.0)) == 0
    assert verdict(evaluate([cam, fix], stamps, 2.0)) == 1
    assert verdict([]) == 1
