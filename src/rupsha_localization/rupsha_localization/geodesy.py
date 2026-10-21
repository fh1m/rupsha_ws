"""WGS84 geodetic <-> local ENU tangent plane, through ECEF.

WHY NOT EQUIRECTANGULAR. x = R*cos(lat0)*dlon, y = R*dlat treats the Earth as
a sphere of one radius and the plane as curving with it. Over a URC course
(order 1-2 km) the meridian/prime-vertical radius difference alone is ~0.7%
-- metres at 1 km -- and the flat approximation also hides the ~0.3 m drop of
the surface below the tangent plane at 2 km. A real tangent plane costs
nothing extra on a Pi, so this module does the exact transform.

ENU here is the tangent plane at the ORIGIN: x east, y north, z up, all in
metres. The filter uses (x, y) only; z is kept so round trips are exact.

Pure math. No ROS, no numpy needed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
WGS84_B = WGS84_A * (1.0 - WGS84_F)
_EP2 = (WGS84_A ** 2 - WGS84_B ** 2) / WGS84_B ** 2


def _require_finite(**kw) -> None:
    for k, v in kw.items():
        if not math.isfinite(float(v)):
            raise ValueError(f'geodesy: {k} is absent ({v})')


def geodetic_to_ecef(lat_deg: float, lon_deg: float, h: float = 0.0) -> Tuple[float, float, float]:
    _require_finite(lat=lat_deg, lon=lon_deg, h=h)
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    s, c = math.sin(lat), math.cos(lat)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    return ((N + h) * c * math.cos(lon),
            (N + h) * c * math.sin(lon),
            (N * (1.0 - WGS84_E2) + h) * s)


def ecef_to_geodetic(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Bowring's initial value, then Newton on latitude; sub-micrometre in 3 passes."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    theta = math.atan2(z * WGS84_A, p * WGS84_B)
    lat = math.atan2(z + _EP2 * WGS84_B * math.sin(theta) ** 3,
                     p - WGS84_E2 * WGS84_A * math.cos(theta) ** 3)
    for _ in range(3):
        s = math.sin(lat)
        N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
        h = p / math.cos(lat) - N if abs(lat) < math.radians(89.0) else z / s - N * (1.0 - WGS84_E2)
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * N / (N + h)))
    s = math.sin(lat)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    h = p / math.cos(lat) - N if abs(lat) < math.radians(89.0) else z / s - N * (1.0 - WGS84_E2)
    return math.degrees(lat), math.degrees(lon), h


@dataclass(frozen=True)
class LocalTangentPlane:
    """ENU frame anchored at (lat0, lon0, h0)."""
    lat0: float
    lon0: float
    h0: float = 0.0

    def __post_init__(self):
        _require_finite(lat0=self.lat0, lon0=self.lon0, h0=self.h0)

    def _rot(self):
        la, lo = math.radians(self.lat0), math.radians(self.lon0)
        sl, cl, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
        # Rows are the E, N, U unit vectors expressed in ECEF.
        return ((-so, co, 0.0),
                (-sl * co, -sl * so, cl),
                (cl * co, cl * so, sl))

    def to_enu(self, lat_deg: float, lon_deg: float, h: float = None) -> Tuple[float, float, float]:
        h = self.h0 if h is None else h
        x, y, z = geodetic_to_ecef(lat_deg, lon_deg, h)
        x0, y0, z0 = geodetic_to_ecef(self.lat0, self.lon0, self.h0)
        d = (x - x0, y - y0, z - z0)
        return tuple(sum(r[i] * d[i] for i in range(3)) for r in self._rot())

    def to_geodetic(self, e: float, n: float, u: float = 0.0) -> Tuple[float, float, float]:
        _require_finite(e=e, n=n, u=u)
        R = self._rot()
        x0, y0, z0 = geodetic_to_ecef(self.lat0, self.lon0, self.h0)
        enu = (e, n, u)
        dx = sum(R[k][0] * enu[k] for k in range(3))
        dy = sum(R[k][1] * enu[k] for k in range(3))
        dz = sum(R[k][2] * enu[k] for k in range(3))
        return ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)

    def to_geodetic_on_plane(self, e: float, n: float) -> Tuple[float, float]:
        """(lat, lon) of a point on the ELLIPSOID below (e, n) at the origin height.

        A ground rover is on the surface, not on the tangent plane; at 2 km the
        two differ by ~0.3 m vertically, which moves lat/lon by millimetres.
        Solved by two fixed-point passes on the vertical.
        """
        u = 0.0
        for _ in range(3):
            lat, lon, h = self.to_geodetic(e, n, u)
            u -= (h - self.h0)
        return lat, lon
