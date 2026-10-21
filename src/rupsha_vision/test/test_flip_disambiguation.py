"""The IPPE flip: two poses that fit nearly equally, and only one is the post.

Ground truth here is the marker NORMAL, known by construction. Two levels:

1. Constructed corners. The corners are blended 45 / 55 between the true
   pose's projection and its mirror's, so IPPE ranks the MIRROR first with an
   ambiguity above the threshold -- the trap, set deliberately. A tracker that
   has seen the true normal must pick the second solution; one with no
   history, or with expired history, must take the first.
2. Rendered frames at 5 m with heavy pixel noise, where the raw best solution
   flips on its own. The tracker must at least halve the flipped frames
   (measured: 11 -> 4 of 60). It cannot remove them: a frame where noise makes
   the mirror CLEARLY better (ambiguity below the threshold) wins outright and
   re-seeds the memory, by design.
"""
import math

import cv2
import numpy as np
import pytest

from rupsha_vision.aruco import (
    ArucoTagDetector,
    FlipTracker,
    PoseSolution,
    square_object_points,
)
from rupsha_vision.camera_model import PinholeCamera
from rupsha_vision.synthetic import project, render_tag, rotation

CAM = PinholeCamera.from_fov(640, 480, 70.0)
TAG_M = 0.20
OBJ = square_object_points(TAG_M)
SAME_DEG = 2.0            # "this is that normal"


def _angle(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def _trap(yaw=25.0):
    rv, tv = rotation(yaw, 10.0), np.array([0.4, 0.1, 5.0])
    true_corners = project(CAM, OBJ, rv, tv)
    det = ArucoTagDetector(marker_size_m=TAG_M)
    sols, _ = det.solve_pose(true_corners, CAM)
    mirror = sols[1]
    trap = 0.45 * true_corners + 0.55 * project(CAM, OBJ, mirror.rvec, mirror.tvec)
    normal = cv2.Rodrigues(rv)[0][:, 2]
    return det, true_corners, trap, normal, mirror.normal


@pytest.mark.parametrize('yaw', [15.0, 25.0, 35.0])
def test_the_trap_is_real(yaw):
    """Without this the other tests prove nothing: the mirror must rank first
    and be ambiguous, and the two normals must be far apart."""
    det, _, trap, normal, mirror_normal = _trap(yaw)
    sols, amb = det.solve_pose(trap, CAM)
    assert amb > det.tracker.ambiguity_threshold
    assert _angle(sols[0].normal, mirror_normal) < SAME_DEG
    assert _angle(sols[1].normal, normal) < SAME_DEG
    assert _angle(normal, mirror_normal) > 20.0


@pytest.mark.parametrize('yaw', [15.0, 25.0, 35.0])
def test_history_picks_the_consistent_solution(yaw):
    det, clean, trap, normal, _ = _trap(yaw)
    tr = det.tracker
    sols, amb = det.solve_pose(clean, CAM)
    assert amb < tr.ambiguity_threshold            # the seeding frame is clear
    tr.choose(7, sols, amb, 0.0)

    sols, amb = det.solve_pose(trap, CAM)
    idx, overrode = tr.choose(7, sols, amb, 0.1)
    assert overrode and idx == 1
    assert _angle(sols[idx].normal, normal) < SAME_DEG


def test_no_history_takes_the_lower_error_solution():
    det, _, trap, _, mirror_normal = _trap()
    sols, amb = det.solve_pose(trap, CAM)
    idx, overrode = det.tracker.choose(7, sols, amb, 0.0)
    assert (idx, overrode) == (0, False)
    assert _angle(sols[idx].normal, mirror_normal) < SAME_DEG


def test_history_expires_after_memory_s():
    det, clean, trap, _, _ = _trap()
    tr = det.tracker
    tr.choose(7, *det.solve_pose(clean, CAM), 0.0)
    sols, amb = det.solve_pose(trap, CAM)
    idx, overrode = tr.choose(7, sols, amb, tr.memory_s + 0.01)
    assert (idx, overrode) == (0, False)


def test_history_is_per_tag_id():
    det, clean, trap, _, _ = _trap()
    tr = det.tracker
    tr.choose(7, *det.solve_pose(clean, CAM), 0.0)
    sols, amb = det.solve_pose(trap, CAM)
    assert tr.choose(8, sols, amb, 0.1) == (0, False)


def test_a_stamp_going_backwards_drops_the_memory():
    """A looped bag or a clock step: the memory belongs to another timeline."""
    det, clean, trap, _, _ = _trap()
    tr = det.tracker
    tr.choose(7, *det.solve_pose(clean, CAM), 10.0)
    sols, amb = det.solve_pose(trap, CAM)
    assert tr.choose(7, sols, amb, 9.0) == (0, False)


def test_a_clear_frame_is_never_overridden():
    """Below the threshold the measurement outranks the memory, even when the
    memory disagrees -- this is what corrects a coin flip that was held."""
    det, clean, _, normal, _ = _trap()
    tr = det.tracker
    sols, amb = det.solve_pose(clean, CAM)
    # Seed with the WRONG normal.
    tr.choose(7, [PoseSolution(sols[1].rvec, sols[1].tvec, 0.0)], 0.0, 0.0)
    idx, overrode = tr.choose(7, sols, amb, 0.1)
    assert (idx, overrode) == (0, False)
    assert _angle(sols[0].normal, normal) < SAME_DEG


def test_end_to_end_tracker_halves_flips_on_noisy_frames():
    rv, near, far = rotation(10.0, 5.0), [0.3, 0.1, 2.5], [0.3, 0.1, 5.0]
    det = ArucoTagDetector(marker_size_m=TAG_M)
    img, truth = render_tag(CAM, 7, TAG_M, rv, near)
    o = det.detect(img, CAM, stamp_s=0.0)[0]
    assert _angle(o.pose.normal, truth.normal) < 5.0

    raw = tracked = seen = 0
    for k in range(60):
        img, truth = render_tag(CAM, 7, TAG_M, rv, far, noise_sigma=12.0, seed=k)
        obs = det.detect(img, CAM, stamp_s=0.05 * (k + 1))
        if not obs:
            continue
        o = obs[0]
        seen += 1
        raw += _angle(o.solutions[0].normal, truth.normal) > 10.0
        tracked += _angle(o.pose.normal, truth.normal) > 10.0
    assert seen >= 55
    assert raw >= 5, 'the scene no longer produces flips; the test proves nothing'
    assert 2 * tracked <= raw


def test_tracker_refuses_nonsense_settings():
    with pytest.raises(ValueError):
        FlipTracker(ambiguity_threshold=0.0)
    with pytest.raises(ValueError):
        FlipTracker(ambiguity_threshold=1.5)
    with pytest.raises(ValueError):
        FlipTracker(memory_s=0.0)


def test_quaternion_matches_the_rotation():
    from rupsha_vision.aruco import rvec_to_quaternion_xyzw
    for rv in (rotation(), rotation(30, 10, 5), rotation(-170, 0, 0), rotation(0, 89, 0)):
        x, y, z, w = rvec_to_quaternion_xyzw(rv)
        assert math.isclose(x * x + y * y + z * z + w * w, 1.0, abs_tol=1e-9)
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(R, cv2.Rodrigues(rv)[0], atol=1e-9)
