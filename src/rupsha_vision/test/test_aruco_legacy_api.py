"""The pre-4.7 path (module-level detectMarkers) sees what ArucoDetector sees.

cv2 4.13 still ships `cv2.aruco.detectMarkers`, so the legacy path is forced
here and run for real against the same rendered scenes, with truth from the
renderer. On a 4.6 box the module picks that path by itself and the
new-API comparisons skip.
"""
import math

import cv2
import pytest

from rupsha_vision import aruco
from rupsha_vision.aruco import ArucoTagDetector
from rupsha_vision.camera_model import PinholeCamera
from rupsha_vision.synthetic import render_tag, rotation

CAM = PinholeCamera.from_fov(640, 480, 70.0)
TAG_M = 0.20
RANGE_TOL = 0.03
# Both paths feed the same edge refinement, so beyond the detector's coarse
# corners nothing differs; agreement is far tighter than the truth tolerance.
AGREE_REL = 1e-3

HAS_NEW_API = hasattr(cv2.aruco, 'ArucoDetector') and aruco.opencv_version() >= (4, 7)
needs_new_api = pytest.mark.skipif(not HAS_NEW_API, reason=f'cv2 {cv2.__version__} '
                                   f'has no usable ArucoDetector')

SCENES = [(3, 2.0, 0.0, 0.0, 0.0),
          (17, 2.0, 0.6, 25.0, 0.0),
          (0, 5.0, -1.0, -35.0, 10.0),
          (49, 5.0, 0.9, 10.0, 20.0),
          (7, 3.5, 1.2, 0.0, -10.0)]


def _render(tag_id, distance, x, yaw, pitch):
    return render_tag(CAM, tag_id, TAG_M, rotation(yaw, pitch),
                      [x * distance / 5.0, 0.1, distance], noise_sigma=2.0, seed=tag_id)


@pytest.mark.parametrize('tag_id,distance,x,yaw,pitch', SCENES)
def test_legacy_path_recovers_the_truth(tag_id, distance, x, yaw, pitch):
    img, truth = _render(tag_id, distance, x, yaw, pitch)
    det = ArucoTagDetector(marker_size_m=TAG_M, legacy=True)
    assert det.legacy
    (o,) = det.detect(img, CAM)
    assert o.tag_id == truth.tag_id
    assert math.hypot(o.u - truth.u, o.v - truth.v) <= 1.0
    assert abs(o.range_m - truth.range_m) <= RANGE_TOL * truth.range_m


@needs_new_api
@pytest.mark.parametrize('tag_id,distance,x,yaw,pitch', SCENES)
def test_legacy_and_new_paths_agree(tag_id, distance, x, yaw, pitch):
    img, _ = _render(tag_id, distance, x, yaw, pitch)
    new = ArucoTagDetector(marker_size_m=TAG_M, legacy=False).detect(img, CAM)
    old = ArucoTagDetector(marker_size_m=TAG_M, legacy=True).detect(img, CAM)
    assert [o.tag_id for o in old] == [o.tag_id for o in new] == [tag_id]
    assert old[0].range_m == pytest.approx(new[0].range_m, rel=AGREE_REL)
    assert old[0].bearing_deg == pytest.approx(new[0].bearing_deg, abs=0.01)


def test_module_switch_selects_the_legacy_path(monkeypatch):
    monkeypatch.setattr(aruco, 'LEGACY_API', True)
    det = ArucoTagDetector()
    assert det.legacy
    img, _ = _render(5, 2.0, 0.3, 15.0, 0.0)
    assert [o.tag_id for o in det.detect(img, CAM)] == [5]


def test_legacy_parameters_carry_the_specified_settings():
    det = ArucoTagDetector(min_side_px=14, legacy=True)
    p = det.parameters
    for name, want in (('useAruco3Detection', True),
                       ('minSideLengthCanonicalImg', 14),
                       ('cornerRefinementMethod', cv2.aruco.CORNER_REFINE_SUBPIX)):
        if name in det.unsupported_params:
            assert not hasattr(p, name)
        else:
            assert getattr(p, name) == want


@pytest.mark.skipif(HAS_NEW_API, reason='ArucoDetector is available here')
def test_new_path_is_refused_where_cv2_lacks_it():
    with pytest.raises(RuntimeError, match='ArucoDetector'):
        ArucoTagDetector(legacy=False)
