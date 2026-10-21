"""Render a marker through a known camera at a known pose. Ground truth by construction.

The truth tests need a scene whose answer is known independently of the code
under test, so the marker is placed with cv2.projectPoints (the camera model)
and painted with warpPerspective (no detector involved). Rendering is
supersampled and area-averaged so the edges carry the sub-pixel information a
real lens would, rather than a hard staircase the corner refiner snaps to.

Supersampling is 16x by default because the edge position is quantised to
1/supersample px: 4x was measured to misplace edges by up to 1/8 px, which is
1.5 % of range on the 18 px side of a 0.20 m tag at 5 m.

PIXEL CONVENTION. Integer coordinates are pixel CENTRES (cv2's and the
detector's convention). A generated marker of N px has its outer edge at
-0.5 and N - 0.5, and a supersampled pixel centre x_hi maps to
(x_hi - (s - 1) / 2) / s in the output image.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .aruco import marker_image, predefined_dictionary, square_object_points
from .camera_model import PinholeCamera


@dataclass(frozen=True)
class TagTruth:
    tag_id: int
    corners: np.ndarray       # (4, 2) projected outer corners, TL TR BR BL
    u: float                  # projected marker centre
    v: float
    bearing_deg: float        # atan2(x, z) of the centre in the optical frame
    range_m: float            # |tvec|
    normal: np.ndarray        # marker +z in the camera frame


def rotation(yaw_deg: float = 0.0, pitch_deg: float = 0.0,
             roll_deg: float = 0.0) -> np.ndarray:
    """rvec for a marker facing the camera, then turned about the camera's
    y (yaw), x (pitch) and z (roll) axes. Facing = marker +z toward the camera."""
    face = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    cy, sy = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    cp, sp = np.cos(np.radians(pitch_deg)), np.sin(np.radians(pitch_deg))
    cr, sr = np.cos(np.radians(roll_deg)), np.sin(np.radians(roll_deg))
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    rvec, _ = cv2.Rodrigues(Ry @ Rx @ Rz @ face)
    return rvec.reshape(3)


def project(camera: PinholeCamera, pts: np.ndarray, rvec, tvec) -> np.ndarray:
    img, _ = cv2.projectPoints(np.asarray(pts, np.float64),
                               np.asarray(rvec, np.float64).reshape(3, 1),
                               np.asarray(tvec, np.float64).reshape(3, 1),
                               camera.K, camera.D)
    return img.reshape(-1, 2)


def distort_image(ideal: np.ndarray, camera: PinholeCamera) -> np.ndarray:
    """What `camera` (with distortion) sees of a scene rendered through the same
    K without it: every output pixel samples the ideal image at its
    undistorted position."""
    h, w = ideal.shape[:2]
    xs, ys = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    src = camera.undistort_px(np.column_stack([xs.ravel(), ys.ravel()]))
    mx = src[:, 0].reshape(h, w).astype(np.float32)
    my = src[:, 1].reshape(h, w).astype(np.float32)
    return cv2.remap(ideal, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def render_tag(camera: PinholeCamera, tag_id: int, size_m: float,
               rvec, tvec, *, dictionary: str = 'DICT_4X4_50',
               supersample: int = 16, marker_px: int = 600,
               background: int = 255, noise_sigma: float = 0.0,
               seed: int = 0) -> tuple[np.ndarray, TagTruth]:
    """Grey image of one marker at (rvec, tvec) and its ground truth.

    Distortion is not rendered (the warp is a homography); pass an
    undistorted camera.
    """
    if camera.dist:
        raise ValueError('render_tag paints a homography; use a camera without '
                         'distortion')
    marker = marker_image(predefined_dictionary(dictionary), tag_id, marker_px)
    obj = square_object_points(size_m)
    corners = project(camera, obj, rvec, tvec)
    centre = project(camera, np.zeros((1, 3)), rvec, tvec)[0]

    # Only the marker's bounding box is supersampled: at 16x a full 640x480
    # frame is 78 MB, and everything outside the box is background anyway.
    s = int(supersample)
    n = float(marker_px)
    x0 = max(int(np.floor(corners[:, 0].min())) - 2, 0)
    y0 = max(int(np.floor(corners[:, 1].min())) - 2, 0)
    x1 = min(int(np.ceil(corners[:, 0].max())) + 3, camera.width)
    y1 = min(int(np.ceil(corners[:, 1].max())) + 3, camera.height)
    img = np.full((camera.height, camera.width), int(background), np.uint8)
    if x1 > x0 and y1 > y0:
        src = np.array([[-0.5, -0.5], [n - 0.5, -0.5], [n - 0.5, n - 0.5],
                        [-0.5, n - 0.5]], np.float32)
        dst = ((corners - [x0, y0]) * s + (s - 1) / 2.0).astype(np.float32)
        H = cv2.getPerspectiveTransform(src, dst)
        big = cv2.warpPerspective(marker, H, ((x1 - x0) * s, (y1 - y0) * s),
                                  flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT,
                                  borderValue=int(background))
        img[y0:y1, x0:x1] = cv2.resize(big, (x1 - x0, y1 - y0),
                                       interpolation=cv2.INTER_AREA)
    if noise_sigma > 0.0:
        rng = np.random.default_rng(seed)
        img = np.clip(img + rng.normal(0.0, noise_sigma, img.shape), 0, 255
                      ).astype(np.uint8)

    t = np.asarray(tvec, np.float64).reshape(3)
    R, _ = cv2.Rodrigues(np.asarray(rvec, np.float64).reshape(3, 1))
    truth = TagTruth(int(tag_id), corners, float(centre[0]), float(centre[1]),
                     float(np.degrees(np.arctan2(t[0], t[2]))),
                     float(np.linalg.norm(t)), R[:, 2].copy())
    return img, truth
