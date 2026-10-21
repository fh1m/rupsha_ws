"""The autonomy executive: queued legs -> verbs, ROS-free.

States are MissouriMRDT Autonomy_Software's (manifest enum AutonomyState) and so
is the shape: a leg queue the operator fills (`add_*_leg`), `start_autonomy`,
and a state machine that can be interrupted by Reversing/Stuck and resumed from
the state it saved. What is ours is the boundary: the executive never touches
the board. Every action is a verb through a port, so the same code runs against
a FakeRover in tests and against /rupsha/move on the vehicle.

A leg succeeds only after VERIFYING: GNSS averaged over a window for position
legs, a majority of frames inside standoff for marker/object legs. Arriving is
not the same as being there, and the scorer is written before the run.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field

from rupsha_control import geo
from rupsha_control.manifest import AutonomyState, Display, LegType, Outcome

from . import routing
from .stuck import StuckConfig, StuckDetector, ladder

S = AutonomyState


@dataclass
class Leg:
    kind: LegType
    lat: float
    lon: float
    radius: float = 3.0
    tag_id: int = -1
    target_name: str = ''

    def describe(self) -> str:
        what = {LegType.POSITION: 'position', LegType.MARKER: f'marker {self.tag_id}',
                LegType.OBJECT: f'object {self.target_name}'}[self.kind]
        return f'{what} @ {self.lat:.6f},{self.lon:.6f}'


@dataclass
class ExecConfig:
    nav_radius_m: float = 2.0            # arrival radius for marker/object legs' GNSS point
    nav_timeout_s: float = 300.0
    search_speed: float = 0.6
    search_timeout_s: float = 240.0
    approach_standoff_m: float = 1.5
    approach_speed: float = 0.5
    approach_timeout_s: float = 60.0
    verify_position_s: float = 3.0
    verify_target_s: float = 1.5
    verify_hit_fraction: float = 0.6
    verify_target_slack_m: float = 1.0
    arrive_dwell_s: float = 3.0
    max_nav_attempts: int = 3
    max_reacquire: int = 2
    backoff_m: float = 2.0               # reverse this far if the post is in view at leg start
    stuck: StuckConfig = field(default_factory=StuckConfig)


@dataclass
class ExecResult:
    ok: bool
    legs_done: int
    message: str


class Executive:

    def __init__(self, port, cfg: ExecConfig | None = None, on_status=None):
        self.port = port
        self.cfg = cfg or ExecConfig()
        self.on_status = on_status
        self.legs: list[Leg] = []
        self.obstacles: list[tuple] = []           # (lat, lon, radius)
        self.learned: list[tuple] = []             # stuck points, routed around
        self.state = S.IDLE
        self.saved_state = S.IDLE
        self.display = Display.OFF
        self.leg_index = -1
        self.detail = ''
        self.distance_to_goal = math.nan
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._rung = 0
        self._last_stuck = None
        self.history: list = []                    # (state name, detail)

    # ── operator verbs (manifest runs_on: executive) ─────────────────────────
    def add_position_leg(self, lat, lon, radius):
        return self._add(Leg(LegType.POSITION, lat, lon, radius))

    def add_marker_leg(self, lat, lon, tag_id, radius):
        return self._add(Leg(LegType.MARKER, lat, lon, radius, tag_id=tag_id))

    def add_object_leg(self, lat, lon, target_name, radius):
        if not target_name:
            return False, 'object leg needs target_name (the detector class)'
        return self._add(Leg(LegType.OBJECT, lat, lon, radius, target_name=target_name))

    def add_obstacle(self, lat, lon, radius):
        if radius <= 0:
            return False, 'obstacle radius must be > 0'
        with self._lock:
            self.obstacles.append((lat, lon, radius))
        return True, f'{len(self.obstacles)} obstacles'

    def clear_legs(self):
        if self.running:
            return False, 'autonomy running; disable_autonomy first'
        with self._lock:
            self.legs.clear()
        return True, 'legs cleared'

    def clear_obstacles(self):
        with self._lock:
            self.obstacles.clear()
            self.learned.clear()
        return True, 'obstacles cleared'

    def _add(self, leg: Leg):
        if leg.lat == 0.0 and leg.lon == 0.0:
            return False, 'leg at 0,0 is almost certainly unset fields'
        if self.running:
            return False, 'autonomy running; legs are frozen until it stops'
        with self._lock:
            self.legs.append(leg)
        return True, f'leg {len(self.legs)}: {leg.describe()}'

    @property
    def running(self) -> bool:
        return self.state != S.IDLE and self.state != S.STUCK

    def request_stop(self) -> None:
        self._stop.set()

    # ── status ───────────────────────────────────────────────────────────────
    def _set(self, state: S, detail: str = '') -> None:
        self.state = state
        self.detail = detail
        self.history.append((state.name, detail))
        if self.on_status:
            self.on_status(self)

    def _led(self, display: Display) -> None:
        self.display = display
        self.port.run('led', target_name=display.name.lower())

    # ── the run ──────────────────────────────────────────────────────────────
    def run(self) -> ExecResult:
        """Run every queued leg. Blocking; call from a worker thread."""
        self._stop.clear()
        if not self.legs:
            return ExecResult(False, 0, 'no legs queued')
        self._led(Display.AUTONOMY)
        done = 0
        try:
            for i, leg in enumerate(list(self.legs)):
                self.leg_index = i
                ok, why = self._run_leg(leg)
                if not ok:
                    self.port.run('stop')
                    final = S.STUCK if self.state == S.STUCK else S.IDLE
                    self._set(final, f'leg {i + 1} failed: {why}')
                    self._led(Display.OFF)
                    return ExecResult(False, done, self.detail)
                done += 1
                self._led(Display.ARRIVED)
                self.port.run('stop')
                self.port.sleep(self.cfg.arrive_dwell_s)
                if i + 1 < len(self.legs):
                    self._led(Display.AUTONOMY)
            self._set(S.IDLE, f'all {done} legs complete')
            return ExecResult(True, done, self.detail)
        finally:
            self.leg_index = -1

    def _stopped(self) -> str | None:
        return 'disabled by operator' if self._stop.is_set() else None

    def _run_leg(self, leg: Leg):
        cfg = self.cfg
        self._rung = 0
        if leg.kind != LegType.POSITION and self._target_in_view(leg):
            self._set(S.REVERSING, 'target already in view at leg start: backing off')
            self.port.run('move_back', duration=cfg.backoff_m / cfg.stuck.reverse_speed,
                          speed=cfg.stuck.reverse_speed)
        state = S.NAVIGATING
        nav_attempts = reacquire = 0
        while True:
            if (why := self._stopped()):
                return False, why
            if state == S.NAVIGATING:
                nav_attempts += 1
                if nav_attempts > cfg.max_nav_attempts:
                    return False, f'could not reach {leg.describe()}'
                radius = leg.radius if leg.kind == LegType.POSITION else cfg.nav_radius_m
                outcome = self._navigate(leg, radius)
                if outcome == 'stuck':
                    if not self._recover():
                        return False, 'stuck: recovery ladder exhausted'
                    continue
                if outcome != 'arrived':
                    if (why := self._stopped()):
                        return False, why
                    continue
                state = {LegType.POSITION: S.VERIFYING_POSITION,
                         LegType.MARKER: S.SEARCH_PATTERN,
                         LegType.OBJECT: S.SEARCH_PATTERN}[leg.kind]
                if leg.kind != LegType.POSITION and self._target_in_view(leg):
                    state = self._approach_state(leg)
            elif state == S.VERIFYING_POSITION:
                self._set(state, leg.describe())
                if self._verify_position(leg):
                    return True, 'position verified'
                state = S.NAVIGATING
            elif state == S.SEARCH_PATTERN:
                self._set(state, f'searching {leg.radius:.0f} m for {leg.describe()}')
                res = self._watched('search_spiral', radius=leg.radius, speed=cfg.search_speed,
                                    target_name=leg.target_name, tag_id=leg.tag_id,
                                    timeout=cfg.search_timeout_s)
                if res == 'stuck':
                    if not self._recover():
                        return False, 'stuck while searching'
                    continue
                if res.outcome != Outcome.SUCCEEDED:
                    return False, f'search: {res.message}'
                state = self._approach_state(leg)
            elif state in (S.APPROACHING_MARKER, S.APPROACHING_OBJECT):
                self._set(state, leg.describe())
                res = self._watched('vision_approach', target_name=leg.target_name,
                                    tag_id=leg.tag_id, standoff=cfg.approach_standoff_m,
                                    speed=cfg.approach_speed, timeout=cfg.approach_timeout_s)
                if res == 'stuck':
                    if not self._recover():
                        return False, 'stuck while approaching'
                    continue
                if res.outcome == Outcome.SUCCEEDED:
                    state = S.VERIFYING_MARKER if leg.kind == LegType.MARKER else S.VERIFYING_OBJECT
                    continue
                reacquire += 1
                if reacquire > cfg.max_reacquire:
                    return False, f'approach: {res.message}'
                state = S.SEARCH_PATTERN
            elif state in (S.VERIFYING_MARKER, S.VERIFYING_OBJECT):
                self._set(state, leg.describe())
                if self._verify_target(leg):
                    return True, 'target verified'
                reacquire += 1
                if reacquire > cfg.max_reacquire:
                    return False, 'target never verified inside standoff'
                state = self._approach_state(leg)

    @staticmethod
    def _approach_state(leg: Leg) -> S:
        return S.APPROACHING_MARKER if leg.kind == LegType.MARKER else S.APPROACHING_OBJECT

    # ── navigation with stuck watch ──────────────────────────────────────────
    def _watched(self, verb, **kw):
        """Run a verb with the stuck detector and the operator stop watching it."""
        det = StuckDetector(self.cfg.stuck, self.port.now())
        stuck = [False]

        def watch():
            if self._stop.is_set():
                return 'stop'
            t = self.port.state()
            if det.update(self.port.now(), t.groundspeed, t.yaw_rate_dps):
                stuck[0] = True
                return 'stuck'
            return None
        res = self.port.run(verb, watch=watch, **kw)
        return 'stuck' if stuck[0] else res

    def _navigate(self, leg: Leg, radius: float) -> str:
        t = self.port.state()
        if not math.isfinite(t.lat):
            self.port.sleep(1.0)
            return 'nofix'
        avoid = self.obstacles + self.learned
        waypoints = routing.plan(t.lat, t.lon, leg.lat, leg.lon, avoid)
        for k, (la, lo) in enumerate(waypoints):
            last = k == len(waypoints) - 1
            self.distance_to_goal = geo.distance_m(t.lat, t.lon, leg.lat, leg.lon)
            self._set(S.NAVIGATING, f'{leg.describe()} via {len(waypoints) - 1} detours'
                      if len(waypoints) > 1 else leg.describe())
            res = self._watched('goto_gnss', lat=la, lon=lo,
                                radius=radius if last else max(1.0, radius / 2),
                                timeout=self.cfg.nav_timeout_s)
            if res == 'stuck':
                return 'stuck'
            if res.outcome != Outcome.SUCCEEDED:
                return res.outcome.name.lower()
        return 'arrived'

    def _recover(self) -> bool:
        """Climb the ladder from where we are; True once the rover has moved out."""
        cfg = self.cfg.stuck
        self.saved_state = self.state
        t = self.port.state()
        here = (t.lat, t.lon)
        if self._last_stuck and geo.distance_m(*self._last_stuck, *here) > cfg.same_place_m:
            self._rung = 0
        self._last_stuck = here
        if math.isfinite(t.yaw_deg):
            ahead = geo.offset(t.lat, t.lon, cfg.obstacle_ahead_m * math.cos(math.radians(t.yaw_deg)),
                               cfg.obstacle_ahead_m * math.sin(math.radians(t.yaw_deg)))
            self.learned.append((*ahead, cfg.obstacle_radius_m))
        while True:
            steps = ladder(cfg, self._rung)
            if steps is None:
                self._set(S.STUCK, 'recovery ladder exhausted')
                return False
            self._set(S.REVERSING, f'stuck: rung {self._rung + 1}')
            self._rung += 1
            for verb, kw in steps:
                if self._stopped():
                    return False
                self.port.run(verb, **kw)
            t = self.port.state()
            if geo.distance_m(*here, t.lat, t.lon) >= cfg.escape_m:
                self._set(self.saved_state, 'resumed after recovery')
                return True

    # ── verification ─────────────────────────────────────────────────────────
    def _verify_position(self, leg: Leg) -> bool:
        lats, lons = [], []
        end = self.port.now() + self.cfg.verify_position_s
        while self.port.now() < end:
            t = self.port.state()
            if math.isfinite(t.lat):
                lats.append(t.lat)
                lons.append(t.lon)
            self.port.sleep(0.2)
        if not lats:
            return False
        d = geo.distance_m(sum(lats) / len(lats), sum(lons) / len(lons), leg.lat, leg.lon)
        self.distance_to_goal = d
        return d <= leg.radius

    def _matches(self, leg: Leg, d) -> bool:
        if leg.kind == LegType.MARKER:
            return d.label == 'tag' and (leg.tag_id < 0 or d.tag_id == leg.tag_id)
        return d.label == leg.target_name

    def _target_in_view(self, leg: Leg) -> bool:
        now = self.port.now()
        return any(self._matches(leg, d) and now - d.stamp < 0.5 for d in self.port.detections())

    def _verify_target(self, leg: Leg) -> bool:
        limit = self.cfg.approach_standoff_m + self.cfg.verify_target_slack_m
        hits = samples = 0
        end = self.port.now() + self.cfg.verify_target_s
        while self.port.now() < end:
            samples += 1
            now = self.port.now()
            if any(self._matches(leg, d) and now - d.stamp < 0.5 and d.range_m <= limit
                   for d in self.port.detections()):
                hits += 1
            self.port.sleep(0.1)
        return samples > 0 and hits / samples >= self.cfg.verify_hit_fraction
