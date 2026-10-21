"""Pixel -> angle, checked against points projected through the same camera."""
import math

import cv2
import numpy as np
import pytest

from rupsha_vision.camera_model import PinholeCamera


def _project(cam, xyz):
    img, _ = cv2.projectPoints(np.asarray([xyz], np.float64), np.zeros(3), np.zeros(3),
                               cam.K, cam.D)
    return img.reshape(2)


def test_from_fov_puts_the_frame_edge_at_half_the_fov():
    cam = PinholeCamera.from_fov(640, 480, 70.0)
    assert cam.fx == pytest.approx(320.0 / math.tan(math.radians(35.0)))
    assert cam.fx == cam.fy and cam.calibrated
    assert cam.bearing_deg(640.0) == pytest.approx(35.0)
    assert cam.bearing_deg(0.0) == pytest.approx(-35.0)
    assert cam.bearing_deg(cam.cx) == 0.0


@pytest.mark.parametrize('deg', [-30.0, -5.0, 0.0, 12.5, 33.0])
def test_bearing_recovers_the_angle_of_a_projected_point(deg):
    cam = PinholeCamera.from_fov(640, 480, 70.0)
    x = 4.0 * math.tan(math.radians(deg))
    u, v = _project(cam, [x, 0.3, 4.0])
    assert cam.bearing_deg(u, v) == pytest.approx(deg, abs=1e-9)


def test_bearing_through_distortion_undistorts_first():
    """A barrel-distorted pixel read as pinhole is degrees off at the edge."""
    cam = PinholeCamera.from_k_d([500, 0, 320, 0, 500, 240, 0, 0, 1],
                                 [-0.3, 0.1, 0.0, 0.0, 0.0], 640, 480)
    u, v = _project(cam, [math.tan(math.radians(30.0)), 0.2, 1.0])
    assert cam.bearing_deg(u, v) == pytest.approx(30.0, abs=1e-3)
    naive = math.degrees(math.atan((u - cam.cx) / cam.fx))
    assert abs(naive - 30.0) > 1.0


def test_distort_and_undistort_px_are_inverses():
    cam = PinholeCamera.from_k_d([500, 0, 320, 0, 500, 240, 0, 0, 1],
                                 [-0.3, 0.1, 0.001, -0.001, 0.0], 640, 480)
    pts = np.array([[10.0, 20.0], [320.0, 240.0], [600.0, 400.0]])
    np.testing.assert_allclose(cam.distort_px(cam.undistort_px(pts)), pts, atol=1e-3)


def test_camera_info_dict_ros2_field_names():
    cam = PinholeCamera.from_camera_info({
        'width': 640, 'height': 480, 'distortion_model': 'plumb_bob',
        'k': [450.0, 0, 321.0, 0, 451.0, 239.0, 0, 0, 1], 'd': [0.0] * 5})
    assert (cam.fx, cam.fy, cam.cx, cam.cy) == (450.0, 451.0, 321.0, 239.0)
    assert cam.calibrated and cam.dist == ()


def test_camera_info_calibration_yaml_layout():
    cam = PinholeCamera.from_camera_info({
        'image_width': 640, 'image_height': 480,
        'camera_matrix': {'rows': 3, 'cols': 3,
                          'data': [450.0, 0, 320, 0, 450.0, 240, 0, 0, 1]},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'data': [-0.1, 0.01, 0, 0, 0]}})
    assert cam.calibrated and cam.dist[0] == -0.1


def test_camera_info_object_with_attributes():
    class Info:
        width, height, distortion_model = 320, 240, ''
        k = (200.0, 0, 160, 0, 200.0, 120, 0, 0, 1)
        d = ()
    assert PinholeCamera.from_camera_info(Info()).calibrated


@pytest.mark.parametrize('k', [[0.0] * 9, []])
def test_zero_or_empty_k_is_uncalibrated_not_a_guess(k):
    cam = PinholeCamera.from_camera_info({'width': 640, 'height': 480, 'k': k})
    assert not cam.calibrated
    assert math.isnan(cam.bearing_deg(400.0))


def test_uncalibrated_has_no_angles():
    cam = PinholeCamera.uncalibrated(640, 480)
    assert not cam.calibrated and not cam.has_intrinsics
    assert math.isnan(cam.bearing_deg(10.0))
    assert all(math.isnan(x) for x in cam.normalized(10.0, 10.0))


def test_fisheye_is_refused_not_read_as_pinhole():
    with pytest.raises(ValueError, match='equidistant'):
        PinholeCamera.from_camera_info({
            'width': 640, 'height': 480, 'distortion_model': 'equidistant',
            'k': [300.0, 0, 320, 0, 300.0, 240, 0, 0, 1], 'd': [0.1, 0, 0, 0]})


def test_odd_distortion_length_is_refused():
    with pytest.raises(ValueError, match='distortion'):
        PinholeCamera.from_k_d([300.0, 0, 320, 0, 300.0, 240, 0, 0, 1], [0.1, 0.2, 0.3],
                               640, 480)


def test_scaled_to_preserves_angles():
    cam = PinholeCamera.from_fov(1280, 960, 70.0)
    half = cam.scaled_to(640, 480)
    assert half.bearing_deg(500.0) == pytest.approx(cam.bearing_deg(1000.0))


@pytest.mark.parametrize('bad', [0.0, 180.0, -10.0])
def test_from_fov_refuses_impossible_fov(bad):
    with pytest.raises(ValueError):
        PinholeCamera.from_fov(640, 480, bad)
