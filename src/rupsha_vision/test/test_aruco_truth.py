"""ArUco against rendered scenes whose answer is known by construction.

The scene is a DICT_4X4_50 marker painted through a 640x480, 70 deg pinhole at
a pose chosen here (`rupsha_vision.synthetic`: cv2.projectPoints places the
corners, warpPerspective paints them, nothing from the detector involved).
The detector is then asked what it sees.

TOLERANCES (stated, and what was measured when they were set):
    id            exact
    centre        1 px      measured worst 0.11 px over these poses
    bearing       0.5 deg   measured worst 0.013 deg
    range         3 %       measured worst 0.6 % (2 m and 5 m, up to 35 deg yaw)
"""
import math

import numpy as np
import pytest

from rupsha_vision.aruco import ArucoTagDetector, dictionary_id, quad_centre
from rupsha_vision.camera_model import PinholeCamera
from rupsha_vision.synthetic import render_tag, rotation

CAM = PinholeCamera.from_fov(640, 480, 70.0)
TAG_M = 0.20
CENTRE_TOL_PX = 1.0
BEARING_TOL_DEG = 0.5
RANGE_TOL = 0.03

# (lateral x at 5 m, height y, yaw, pitch). Lateral offset scales with range so
# the far poses stay in frame; 45 deg yaw at 5 m is outside what the tag
# resolves (13 px foreshortened side) and is tested separately as a miss.
POSES = [(0.0, 0.0, 0.0, 0.0),
         (0.0, 0.1, 0.0, 0.0),
         (0.6, 0.0, 25.0, 0.0),
         (-1.0, 0.2, -35.0, 10.0),
         (0.9, 0.3, 10.0, 20.0),
         (1.5, 0.0, 0.0, 0.0),
         (-1.5, -0.3, -20.0, -10.0)]


def _one(obs):
    assert len(obs) == 1, f'expected exactly one tag, got {len(obs)}'
    return obs[0]


@pytest.mark.parametrize('distance', [2.0, 5.0])
@pytest.mark.parametrize('x,y,yaw,pitch', POSES)
@pytest.mark.parametrize('noise', [0.0, 3.0])
def test_rendered_tag_recovers_id_centre_bearing_range(distance, x, y, yaw, pitch,
                                                       noise):
    tvec = [x * distance / 5.0, y, distance]
    img, truth = render_tag(CAM, 7, TAG_M, rotation(yaw, pitch), tvec,
                            noise_sigma=noise, seed=11)
    o = _one(ArucoTagDetector(marker_size_m=TAG_M).detect(img, CAM))

    assert o.tag_id == truth.tag_id
    assert math.hypot(o.u - truth.u, o.v - truth.v) <= CENTRE_TOL_PX
    assert abs(o.bearing_deg - truth.bearing_deg) <= BEARING_TOL_DEG
    assert abs(o.range_m - truth.range_m) <= RANGE_TOL * truth.range_m
    assert o.has_pose and o.refined
    np.testing.assert_allclose(o.pose.tvec, tvec, atol=RANGE_TOL * distance)


@pytest.mark.parametrize('tag_id', [0, 1, 17, 49])
def test_every_id_decodes_as_itself(tag_id):
    img, _ = render_tag(CAM, tag_id, TAG_M, rotation(15.0), [0.2, 0.0, 3.0])
    assert _one(ArucoTagDetector().detect(img, CAM)).tag_id == tag_id


def test_bearing_sign_is_positive_right():
    img, truth = render_tag(CAM, 3, TAG_M, rotation(), [1.0, 0.0, 3.0])
    o = _one(ArucoTagDetector().detect(img, CAM))
    assert truth.bearing_deg > 15.0
    assert o.bearing_deg == pytest.approx(truth.bearing_deg, abs=BEARING_TOL_DEG)


def test_uncalibrated_camera_keeps_pixels_but_range_is_nan():
    img, truth = render_tag(CAM, 5, TAG_M, rotation(20.0), [0.5, 0.0, 2.0])
    o = _one(ArucoTagDetector().detect(img, PinholeCamera.uncalibrated(640, 480)))
    assert o.tag_id == 5
    assert math.hypot(o.u - truth.u, o.v - truth.v) <= CENTRE_TOL_PX
    assert math.isnan(o.range_m) and math.isnan(o.bearing_deg)
    assert o.pose is None and not o.has_pose and math.isnan(o.pose_ambiguity)


def test_nominal_fov_gives_bearing_but_never_range():
    """A datasheet FOV is good enough to steer by, not to range by."""
    nominal = PinholeCamera.from_fov(640, 480, 70.0, calibrated=False)
    img, truth = render_tag(CAM, 5, TAG_M, rotation(), [0.5, 0.0, 2.0])
    o = _one(ArucoTagDetector().detect(img, nominal))
    assert o.bearing_deg == pytest.approx(truth.bearing_deg, abs=BEARING_TOL_DEG)
    assert math.isnan(o.range_m)


@pytest.mark.parametrize('x,yaw', [(1.2, 20.0), (-1.0, -30.0)])
def test_barrel_distorted_lens_off_axis(x, yaw):
    """Truth is set in the ideal camera; the image is what a k1 = -0.25 lens
    with the same K would record. Reading it as the ideal pinhole is degrees
    off this far from the axis -- checked, so the test is known to exercise
    the distortion path rather than pass around it."""
    from rupsha_vision.synthetic import distort_image
    ideal = PinholeCamera(400.0, 400.0, 320.0, 240.0, 640, 480)
    lens = PinholeCamera.from_k_d(ideal.K.reshape(-1), [-0.25, 0.06, 0.0, 0.0, 0.0],
                                  640, 480)
    img, truth = render_tag(ideal, 9, TAG_M, rotation(yaw), [x, 0.3, 2.0])
    img = distort_image(img, lens)
    o = _one(ArucoTagDetector().detect(img, lens))
    assert o.bearing_deg == pytest.approx(truth.bearing_deg, abs=BEARING_TOL_DEG)
    assert o.range_m == pytest.approx(truth.range_m, rel=RANGE_TOL)
    naive = _one(ArucoTagDetector().detect(img, ideal))
    assert abs(naive.bearing_deg - truth.bearing_deg) > 1.0


def test_marker_size_scales_range_linearly():
    """The printed size is the metric scale: telling the detector the tag is
    twice as big must double the range, and nothing else may supply it."""
    img, _ = render_tag(CAM, 2, TAG_M, rotation(10.0), [0.0, 0.0, 3.0])
    r1 = _one(ArucoTagDetector(marker_size_m=TAG_M).detect(img, CAM)).range_m
    r2 = _one(ArucoTagDetector(marker_size_m=2 * TAG_M).detect(img, CAM)).range_m
    assert r2 == pytest.approx(2.0 * r1, rel=1e-6)


def test_tag_too_small_is_a_miss_not_a_guess():
    img, _ = render_tag(CAM, 7, TAG_M, rotation(45.0), [0.3, 0.0, 5.0])
    assert ArucoTagDetector().detect(img, CAM) == []


def test_empty_scene_detects_nothing():
    blank = np.full((480, 640), 255, np.uint8)
    assert ArucoTagDetector().detect(blank, CAM) == []


def test_image_and_camera_size_must_agree():
    with pytest.raises(ValueError, match='640x480'):
        ArucoTagDetector().detect(np.zeros((240, 320), np.uint8), CAM)


def test_quad_centre_is_the_projective_centre_not_the_mean():
    """Under strong tilt the corner mean is pixels away from the centre."""
    _, truth = render_tag(CAM, 1, TAG_M, rotation(60.0), [0.0, 0.0, 0.6])
    u, v = quad_centre(truth.corners)
    assert math.hypot(u - truth.u, v - truth.v) < 1e-6
    m = truth.corners.mean(axis=0)
    assert math.hypot(m[0] - truth.u, m[1] - truth.v) > 1.0


@pytest.mark.parametrize('bad', [11, 17, 0])
def test_min_side_outside_12_to_16_is_refused(bad):
    with pytest.raises(ValueError, match='min_side_px'):
        ArucoTagDetector(min_side_px=bad)


@pytest.mark.parametrize('bad', [0.0, -0.2, float('nan')])
def test_non_positive_marker_size_is_refused(bad):
    with pytest.raises(ValueError, match='marker_size_m'):
        ArucoTagDetector(marker_size_m=bad)


def test_unknown_dictionary_is_refused():
    with pytest.raises(ValueError, match='DICT_4X4_51'):
        dictionary_id('DICT_4X4_51')
    assert dictionary_id('4X4_50') == dictionary_id('DICT_4X4_50')


def test_detector_settings_are_the_specified_ones():
    import cv2
    det = ArucoTagDetector(min_side_px=12)
    p = det.parameters
    if not det.legacy:
        p = det._find.__self__.getDetectorParameters()
    assert det.unsupported_params == ()
    assert p.useAruco3Detection is True
    assert p.minSideLengthCanonicalImg == 12
    assert p.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_SUBPIX
