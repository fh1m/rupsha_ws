"""Shadow-versus-board verdicts. ABSENT must never read as OK."""
import math

import pytest

from rupsha_localization import health
from rupsha_localization.geodesy import LocalTangentPlane

NAN = float('nan')
LAT, LON = 38.4064, -110.7919


def _offset(e, n):
    lat, lon, _ = LocalTangentPlane(LAT, LON).to_geodetic(e, n, 0.0)
    return lat, lon


def _cmp(s_lat, s_lon, s_yaw, b_lat=LAT, b_lon=LON, b_yaw=10.0):
    return health.compare(s_lat, s_lon, s_yaw, b_lat, b_lon, b_yaw, 5.0, 15.0)


def test_agreement_is_ok_with_measured_divergence():
    lat, lon = _offset(3.0, 3.0)
    v = _cmp(lat, lon, 12.0)
    assert v.status == health.OK
    assert v.position_m == pytest.approx(math.hypot(3.0, 3.0), abs=1e-3)
    assert v.yaw_deg == pytest.approx(2.0)


def test_position_divergence():
    lat, lon = _offset(6.0, 0.0)
    v = _cmp(lat, lon, 10.0)
    assert v.status == health.DIVERGED
    assert v.position_m == pytest.approx(6.0, abs=1e-3)
    assert 'position' in v.detail


def test_yaw_divergence_and_wrap():
    assert _cmp(LAT, LON, 40.0).status == health.DIVERGED
    v = _cmp(LAT, LON, 359.0, b_yaw=1.0)
    assert v.status == health.OK and v.yaw_deg == pytest.approx(-2.0)


@pytest.mark.parametrize('field', range(6))
def test_any_nan_is_absent(field):
    args = [LAT, LON, 10.0, LAT, LON, 10.0]
    args[field] = NAN
    v = health.compare(*args, 5.0, 15.0)
    assert v.status == health.ABSENT
    assert math.isnan(v.position_m) and math.isnan(v.yaw_deg)


def test_none_is_absent_too():
    assert _cmp(None, LON, 10.0).status == health.ABSENT
