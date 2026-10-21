"""Route a straight leg around circular keep-out zones.

Deliberately small: a local north/east frame, circles, and a detour waypoint
placed beside each circle the segment cuts, recursing on both halves. It is the
planner a Pi 4 can run every leg without a map. The roadmap replaces it with
weighted A* over a traversability grid (MRDT's cost 1 + beta*10*(1-s)^2) once a
DEM or an elevation map exists; the interface (start, goal, obstacles) -> points
stays.
"""

from __future__ import annotations

import math

from rupsha_control import geo

MARGIN_M = 0.8
MAX_DEPTH = 6


def to_local(lat0, lon0, lat, lon):
    n = math.copysign(geo.distance_m(lat0, lon0, lat, lon0), lat - lat0)
    e = math.copysign(geo.distance_m(lat, lon0, lat, lon), lon - lon0)
    return n, e


def _closest_on_segment(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return (ax + t * dx, ay + t * dy), t


def _blocking(a, b, obstacles, margin):
    """The obstacle the segment a->b cuts nearest to a, or None."""
    best = None
    for c, r in obstacles:
        q, t = _closest_on_segment(c, a, b)
        if math.dist(q, c) < r + margin and 0.0 < t < 1.0:
            if best is None or t < best[0]:
                best = (t, c, r, q)
    return best


def plan_local(a, b, obstacles, margin=MARGIN_M, depth=0):
    """Waypoints after `a`, ending at `b`, avoiding (centre, radius) circles."""
    if depth > MAX_DEPTH:
        return [b]
    hit = _blocking(a, b, obstacles, margin)
    if hit is None:
        return [b]
    _, c, r, q = hit
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / L, dx / L                  # left of travel
    side = (q[0] - c[0]) * nx + (q[1] - c[1]) * ny
    if side < 0:
        nx, ny = -nx, -ny
    clear = (r + margin) * 1.25
    w = (c[0] + nx * clear, c[1] + ny * clear)
    return plan_local(a, w, obstacles, margin, depth + 1) + plan_local(w, b, obstacles, margin, depth + 1)


def plan(lat0, lon0, lat1, lon1, obstacles_ll, margin=MARGIN_M):
    """Lat/lon waypoints from (lat0, lon0) to (lat1, lon1). obstacles_ll: [(lat, lon, r)]."""
    obs = [(to_local(lat0, lon0, la, lo), r) for la, lo, r in obstacles_ll]
    pts = plan_local((0.0, 0.0), to_local(lat0, lon0, lat1, lon1), obs, margin)
    out = [geo.offset(lat0, lon0, n, e) for n, e in pts[:-1]]
    return out + [(lat1, lon1)]
