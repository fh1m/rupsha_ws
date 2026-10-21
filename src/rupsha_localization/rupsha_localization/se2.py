"""SE(2) for a ground rover: the group the shadow filter lives on.

CONVENTION, fixed here and nowhere else. A pose is a 3x3 homogeneous matrix

    T = [ R  p ]     R in SO(2), p in R^2
        [ 0  1 ]

and T maps BODY coordinates to WORLD coordinates (T_wb). World is the local
ENU tangent plane (x east, y north), body is REP-103 FLU (x forward, y left),
so yaw is measured counter-clockwise from EAST. The board reports yaw
clockwise from NORTH; `yaw_from_compass_deg` is the one place that converts.

A tangent vector is ordered tau = (rho_x, rho_y, theta) -- translation first,
as in Sola, Deray & Atchuthan 2018, "A micro Lie theory for state estimation
in robotics", whose closed forms are used below.

Pure numpy. No ROS.
"""
from __future__ import annotations

import math

import numpy as np

# Below this angle sin(t)/t and (1-cos t)/t are evaluated by their series, not
# by dividing two small numbers. A numerical boundary, not a tuning knob.
_SMALL_ANGLE = 1e-9


def wrap_pi(a: float) -> float:
    """Angle into [-pi, pi). NaN stays NaN -- absent is not zero."""
    a = float(a)
    if not math.isfinite(a):
        return float('nan')
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_compass_deg(compass_deg: float) -> float:
    """Board yaw (deg, clockwise from north, NED) -> ENU yaw (rad, CCW from east)."""
    return wrap_pi(math.pi / 2.0 - math.radians(float(compass_deg)))


def compass_deg_from_yaw(yaw: float) -> float:
    """Inverse of `yaw_from_compass_deg`, into [0, 360)."""
    yaw = float(yaw)
    if not math.isfinite(yaw):
        return float('nan')
    return math.degrees(math.pi / 2.0 - yaw) % 360.0


def rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def make(x: float, y: float, theta: float) -> np.ndarray:
    T = np.eye(3)
    T[:2, :2] = rot(theta)
    T[0, 2], T[1, 2] = x, y
    return T


def yaw_of(T: np.ndarray) -> float:
    return math.atan2(T[1, 0], T[0, 0])


def inverse(T: np.ndarray) -> np.ndarray:
    R, p = T[:2, :2], T[:2, 2]
    out = np.eye(3)
    out[:2, :2] = R.T
    out[:2, 2] = -R.T @ p
    return out


def compose(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """A * B. Re-orthonormalised: a long product of rotations drifts off SO(2)."""
    C = A @ B
    th = math.atan2(C[1, 0], C[0, 0])
    C[:2, :2] = rot(th)
    C[2, :] = (0.0, 0.0, 1.0)
    return C


def _v_matrix(theta: float) -> np.ndarray:
    """The left Jacobian of SO(2) acting on translation: p = V(theta) rho."""
    if abs(theta) < _SMALL_ANGLE:
        a = 1.0 - theta * theta / 6.0
        b = theta / 2.0
    else:
        a = math.sin(theta) / theta
        b = (1.0 - math.cos(theta)) / theta
    return np.array([[a, -b], [b, a]])


def exp(tau) -> np.ndarray:
    tau = np.asarray(tau, dtype=float).reshape(3)
    T = np.eye(3)
    T[:2, :2] = rot(tau[2])
    T[:2, 2] = _v_matrix(tau[2]) @ tau[:2]
    return T


def log(T: np.ndarray) -> np.ndarray:
    theta = math.atan2(T[1, 0], T[0, 0])
    rho = np.linalg.solve(_v_matrix(theta), T[:2, 2])
    return np.array([rho[0], rho[1], theta])


def adjoint(T: np.ndarray) -> np.ndarray:
    """Ad_T with T exp(tau) T^-1 = exp(Ad_T tau), for tau = (rho, theta)."""
    Ad = np.eye(3)
    Ad[:2, :2] = T[:2, :2]
    Ad[0, 2] = T[1, 2]
    Ad[1, 2] = -T[0, 2]
    return Ad
