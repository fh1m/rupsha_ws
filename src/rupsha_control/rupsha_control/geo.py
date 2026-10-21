"""Small-distance geodesy for verbs. Haversine is exact enough under 10 km, which
is two orders of magnitude past any leg this rover drives; the shadow estimator
in rupsha_localization uses a full ECEF tangent plane instead."""

import math

EARTH_R = 6371008.8


def distance_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    """Initial bearing from point 1 to point 2, 0..360 from north."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


def offset(lat, lon, north_m, east_m):
    """Point `north_m`/`east_m` away. Flat-earth; fine for the leg lengths here."""
    dlat = north_m / EARTH_R
    dlon = east_m / (EARTH_R * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def heading_error(target_deg: float, current_deg: float) -> float:
    """Signed shortest turn from current to target, positive = clockwise."""
    return wrap180(target_deg - current_deg)
