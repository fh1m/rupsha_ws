"""Tangent-plane geodesy: round trips, and the two effects equirectangular misses."""
import math

import pytest

from rupsha_localization.geodesy import (
    WGS84_A,
    WGS84_E2,
    LocalTangentPlane,
    ecef_to_geodetic,
    geodetic_to_ecef,
)

# Hanksville, Utah (URC) and Dhaka: a mid-latitude and a low-latitude origin.
ORIGINS = [(38.4064, -110.7919, 1350.0), (23.7272, 90.3988, 10.0)]


@pytest.mark.parametrize('lat0,lon0,h0', ORIGINS)
@pytest.mark.parametrize('dist', [1.0, 2000.0])
def test_round_trip_enu_geodetic_enu(lat0, lon0, h0, dist):
    plane = LocalTangentPlane(lat0, lon0, h0)
    for bearing in range(0, 360, 30):
        b = math.radians(bearing)
        e, n, u = dist * math.sin(b), dist * math.cos(b), -0.2
        lat, lon, h = plane.to_geodetic(e, n, u)
        e2, n2, u2 = plane.to_enu(lat, lon, h)
        assert math.dist((e, n, u), (e2, n2, u2)) < 1e-6


@pytest.mark.parametrize('lat0,lon0,h0', ORIGINS)
def test_ecef_round_trip(lat0, lon0, h0):
    lat, lon, h = ecef_to_geodetic(*geodetic_to_ecef(lat0, lon0, h0))
    assert lat == pytest.approx(lat0, abs=1e-11)
    assert lon == pytest.approx(lon0, abs=1e-11)
    assert h == pytest.approx(h0, abs=1e-6)


def test_two_km_north_matches_the_meridian_arc():
    lat0 = 38.4064
    plane = LocalTangentPlane(lat0, -110.7919, 0.0)
    phi = math.radians(lat0)
    M = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * math.sin(phi) ** 2) ** 1.5
    dlat = math.degrees(2000.0 / M)
    e, n, u = plane.to_enu(lat0 + dlat, -110.7919, 0.0)
    assert abs(e) < 1e-6
    assert n == pytest.approx(2000.0, abs=0.01)
    # The ellipsoid falls away below the tangent plane: ~d^2 / (2R) = 0.31 m.
    assert -0.33 < u < -0.30


def test_on_plane_inverse_lands_on_the_surface():
    plane = LocalTangentPlane(38.4064, -110.7919, 1350.0)
    lat, lon = plane.to_geodetic_on_plane(1500.0, -1200.0)
    e, n, _ = plane.to_enu(lat, lon, 1350.0)
    assert math.hypot(e - 1500.0, n + 1200.0) < 1e-3


def test_absent_input_is_refused():
    with pytest.raises(ValueError):
        LocalTangentPlane(float('nan'), 0.0)
    with pytest.raises(ValueError):
        LocalTangentPlane(10.0, 10.0).to_enu(float('nan'), 10.0)
