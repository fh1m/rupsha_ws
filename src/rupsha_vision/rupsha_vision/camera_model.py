"""Pinhole camera: the one place a pixel becomes an angle.

THREE WAYS TO KNOW A CAMERA, AND THEY ARE NOT EQUALLY TRUE
----------------------------------------------------------
``from_camera_info``  a calibration (or a simulator, whose intrinsics are
                      exact). `calibrated=True`: bearings and metric range.
``from_fov``          an ideal pinhole of a given horizontal FOV. Exact for a
                      sim camera, so `calibrated=True` by default; pass
                      `calibrated=False` when the FOV is a datasheet number for
                      a real lens -- bearings are then coarse and range is NaN.
``uncalibrated``      nothing known but the image size. fx/fy are NaN, so
                      bearing and range are NaN. Pixels are still pixels.

An all-zero K in a CameraInfo means the driver has no calibration. That is
read as `uncalibrated`, never as a guessed FOV: a guessed FOV that silently
stands in for a calibration produces ranges that are wrong and look right.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

# Models whose coefficient vector cv2's pinhole functions consume directly.
# `equidistant` (fisheye) needs cv2.fisheye and a different PnP path; feeding
# its 4 coefficients to the pinhole model would be wrong without failing.
_PINHOLE_MODELS = ('', 'plumb_bob', 'rational_polynomial')

# cv2.undistortPoints stops after 5 fixed-point iterations, which was measured
# to leave 0.06 px at the corner of a k1 = -0.3 lens -- a third of the corner
# budget at 5 m. Iterate to convergence instead.
_UNDISTORT_CRITERIA = (3, 100, 1e-12)       # COUNT | EPS


def _undistort(pts: np.ndarray, K: np.ndarray, D: np.ndarray, P=None) -> np.ndarray:
    return cv2.undistortPointsIter(pts, K, D, None, P, _UNDISTORT_CRITERIA)


@dataclass(frozen=True)
class PinholeCamera:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist: tuple[float, ...] = field(default_factory=tuple)
    calibrated: bool = True

    # ------------------------------------------------------------------ #
    @classmethod
    def from_fov(cls, width: int, height: int, hfov_deg: float, *,
                 calibrated: bool = True) -> PinholeCamera:
        if not (0.0 < hfov_deg < 180.0):
            raise ValueError(f'hfov_deg must be in (0, 180), got {hfov_deg}')
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        # Square pixels; principal point at width/2, height/2, which is what
        # gz-sim publishes in its own CameraInfo.
        return cls(fx, fx, width / 2.0, height / 2.0, int(width), int(height),
                   (), bool(calibrated))

    @classmethod
    def uncalibrated(cls, width: int, height: int) -> PinholeCamera:
        nan = float('nan')
        return cls(nan, nan, width / 2.0, height / 2.0, int(width), int(height),
                   (), False)

    @classmethod
    def from_k_d(cls, K: Sequence[float], D: Sequence[float],
                 width: int, height: int) -> PinholeCamera:
        k = np.asarray(K, dtype=float).reshape(-1)
        if k.size != 9:
            raise ValueError(f'K must have 9 entries (row-major 3x3), got {k.size}')
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if not (np.isfinite([fx, fy, cx, cy]).all() and fx > 0 and fy > 0):
            return cls.uncalibrated(width, height)
        d = tuple(float(x) for x in np.asarray(D, dtype=float).reshape(-1))
        if len(d) not in (0, 4, 5, 8, 12, 14):
            raise ValueError(f'{len(d)} distortion coefficients is not a cv2 '
                             f'pinhole model (expected 0, 4, 5, 8, 12 or 14)')
        if not any(d):
            d = ()
        return cls(float(fx), float(fy), float(cx), float(cy),
                   int(width), int(height), d, True)

    @classmethod
    def from_camera_info(cls, info: Any) -> PinholeCamera:
        """From a sensor_msgs/CameraInfo or a dict with the same field names.

        Accepts `k`/`d` (ROS 2 field names) or `K`/`D`, and also the
        camera_calibration_parsers YAML layout (`camera_matrix: {data: [...]}`).
        """
        get = (info.get if isinstance(info, Mapping)
               else lambda key, default=None: getattr(info, key, default))

        def first(*keys):
            for key in keys:
                v = get(key, None)
                if v is not None:
                    return v
            return None

        def data(v):
            return v.get('data') if isinstance(v, Mapping) else v

        width = first('width', 'image_width')
        height = first('height', 'image_height')
        if width is None or height is None:
            raise ValueError('camera info has no width/height')
        model = str(first('distortion_model') or '')
        if model not in _PINHOLE_MODELS:
            raise ValueError(f'distortion_model {model!r} is not supported by '
                             f'the pinhole path (supported: plumb_bob, '
                             f'rational_polynomial)')
        K = data(first('k', 'K', 'camera_matrix'))
        D = data(first('d', 'D', 'distortion_coefficients'))
        if K is None or len(K) == 0:
            return cls.uncalibrated(int(width), int(height))
        return cls.from_k_d(list(K), list(D) if D is not None else [],
                            int(width), int(height))

    # ------------------------------------------------------------------ #
    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], dtype=np.float64)

    @property
    def D(self) -> np.ndarray:
        return np.asarray(self.dist if self.dist else (0.0,) * 5, dtype=np.float64)

    @property
    def has_intrinsics(self) -> bool:
        """fx/fy known (calibrated OR a nominal FOV). Bearings are computable."""
        return math.isfinite(self.fx) and math.isfinite(self.fy)

    def normalized(self, u: float, v: float) -> tuple[float, float]:
        """Undistorted normalized image coordinates (x/z, y/z). NaN without intrinsics."""
        if not self.has_intrinsics:
            return float('nan'), float('nan')
        if not self.dist:
            return (u - self.cx) / self.fx, (v - self.cy) / self.fy
        pt = np.array([[[u, v]]], dtype=np.float64)
        x, y = _undistort(pt, self.K, self.D)[0, 0]
        return float(x), float(y)

    def undistort_px(self, pts: np.ndarray) -> np.ndarray:
        """(N, 2) raw pixels -> (N, 2) pixels of the same camera without distortion."""
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        if not self.dist or not self.has_intrinsics:
            return pts.copy()
        return _undistort(pts.reshape(-1, 1, 2), self.K, self.D,
                          self.K).reshape(-1, 2)

    def distort_px(self, pts: np.ndarray) -> np.ndarray:
        """Inverse of `undistort_px`."""
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        if not self.dist or not self.has_intrinsics:
            return pts.copy()
        xy = np.column_stack([(pts[:, 0] - self.cx) / self.fx,
                              (pts[:, 1] - self.cy) / self.fy,
                              np.ones(len(pts))])
        img, _ = cv2.projectPoints(xy, np.zeros(3), np.zeros(3), self.K, self.D)
        return img.reshape(-1, 2)

    def bearing_deg(self, u: float, v: float | None = None) -> float:
        """Horizontal angle of pixel column `u` from the optical axis, + right.

        With distortion the row matters too, so pass `v` when it is known; it
        defaults to the principal row.
        """
        x, _ = self.normalized(u, self.cy if v is None else v)
        return math.degrees(math.atan(x))

    def elevation_deg(self, u: float, v: float) -> float:
        """Vertical angle of a pixel from the optical axis, + DOWN (image y)."""
        x, y = self.normalized(u, v)
        return math.degrees(math.atan2(y, math.hypot(1.0, x)))

    def scaled_to(self, width: int, height: int) -> PinholeCamera:
        """The same lens at another resolution (a driver that publishes CameraInfo
        for the sensor's native size while streaming a downscaled image)."""
        if (width, height) == (self.width, self.height):
            return self
        sx, sy = width / self.width, height / self.height
        return PinholeCamera(self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy,
                             int(width), int(height), self.dist, self.calibrated)

