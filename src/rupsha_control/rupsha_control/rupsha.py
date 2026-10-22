"""The Rupsha verb facade: one method per manager verb in manifest/rupsha.yaml.

Reflexes on the board, thinking here. `board` verbs send one target and watch the
board drive it; `host` verbs stream velocity / yaw-rate at their manifest rate and
check abort every tick. Every verb returns a VerbResult; none raises for a
condition an operator can cause (no fix, not armed, target lost) -- those are
outcomes. A raise is a bug.

Abort is cooperative: `request_abort()` sets a flag every loop polls, and the
loop's own exit path zeroes the setpoint. The manager clears the flag when it
CLAIMS a non-bypass verb (see rover_manager_node), never from inside a verb:
mongla lost a stop that way, cleared on the very next line after it was raised.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field

from . import geo
from .fc.base import CommandRejected, Telemetry
from .manifest import VERBS, Display, Firmware, Outcome

NAN = math.nan

# Gains and bounds for host loops. Starting values, not measured on a vehicle.
TURN_KP = 1.5                 # deg/s of yaw rate per deg of heading error
TURN_TOL_DEG = 3.0
TURN_SETTLE_S = 0.5
APPROACH_YAW_KP = 1.2         # deg/s per deg of bearing
APPROACH_TURN_FIRST_DEG = 20.0
APPROACH_CONFIRM_FRAMES = 3
DETECTION_FRESH_S = 0.5
TARGET_LOST_S = 2.0
SPIRAL_SPACING_M = 4.0        # ring spacing ~ the camera's reliable tag range
SPIRAL_MIN_RADIUS_M = 1.0
BOARD_POLL_HZ = 5.0
GPS_FIX_MIN = 3

# Where the target is, not where it was last seen (MRDT's first fallback, sota §9).
# A 20 cm tag decodes reliably to 4.5 m on the 640x480 70 deg camera (measured-bars §3):
# inside that the pixels steer; outside it the board drives to the estimated position.
APPROACH_SERVO_RANGE_M = 4.5
# The tag flickers inside 2 m on this camera (Gazebo, 2026-10-22: centered at 1.71 m,
# then gone for 2 s). A sighting this close to the standoff is the arrival.
APPROACH_NEAR_M = 0.25
RETARGET_M = 0.3              # re-send a local target only when it moved this far
POSE_HISTORY_S = 3.0
POSE_MATCH_S = 0.15           # a stamp further than this outside the history has no pose
ESTIMATE_FIXES = 10
ESTIMATE_TTL_S = 30.0
SEARCH_CONFIRM_HITS = 3       # distinct frames before the search hands over
SEARCH_CONFIRM_RANGE_M = 4.0  # close to this range of the estimate to collect them
SEARCH_CONFIRM_S = 15.0


@dataclass(frozen=True)
class _Target:
    """A `_stream` setpoint that is a local position for the board, not a velocity."""
    north: float
    east: float


@dataclass
class _Estimate:
    """A target's position in the local frame: the 1/range^2-weighted mean of its fixes."""
    fixes: deque = field(default_factory=lambda: deque(maxlen=ESTIMATE_FIXES))
    updated: float = NAN

    def add(self, north, east, range_m, now):
        self.fixes.append((north, east, 1.0 / max(range_m, 0.5) ** 2))
        self.updated = now

    @property
    def position(self):
        w = sum(f[2] for f in self.fixes)
        return (sum(f[0] * f[2] for f in self.fixes) / w,
                sum(f[1] * f[2] for f in self.fixes) / w)


@dataclass
class VerbResult:
    outcome: Outcome
    message: str = ''
    final_value: float = NAN
    error_value: float = NAN
    elapsed_s: float = NAN

    @property
    def success(self) -> bool:
        return self.outcome == Outcome.SUCCEEDED


class Rupsha:

    def __init__(self, fc, *, vision=None, clock=time.monotonic, sleep=time.sleep,
                 led_sink=None, log=None):
        self.fc = fc
        self.vision = vision or (lambda: [])
        self.clock = clock
        self.sleep = sleep
        self.led_sink = led_sink
        self.log = log
        self.display = Display.OFF
        # The board's WP_RADIUS. In GUIDED the board declares arrival and stops at
        # its own radius, so a verb radius smaller than it can never be met (SITL:
        # `drive` sat 2.0 m short for 30 s against WP_RADIUS=2). NaN = unread.
        self.board_wp_radius = NAN
        # Camera ahead of the pose the board reports (body x, m). Geometry, not a
        # reading: the Gazebo model's 0.33 m is set by sim.launch.py; the real rover's
        # is unmeasured until the chassis exists.
        self.camera_forward_m = 0.0
        self._poses: deque = deque()     # (clock, north, east, yaw_deg), sampled every loop tick
        self._estimates: dict = {}       # (target_name, tag_id) -> _Estimate
        self._sent_target = None
        self._abort = threading.Event()
        self.progress = None          # optional callable(phase, fraction, line)

    # ── abort ────────────────────────────────────────────────────────────────
    def request_abort(self) -> None:
        self._abort.set()

    def clear_abort(self) -> None:
        self._abort.clear()

    @property
    def aborted(self) -> bool:
        return self._abort.is_set()

    def run(self, verb: str, **kwargs) -> VerbResult:
        """Call a verb with manifest defaults, the way the manager does. For the DSL and tests."""
        spec = VERBS[verb]
        args = dict(spec['defaults'])
        args.update(kwargs)
        if not spec['bypass_busy']:
            self.clear_abort()
        return getattr(self, verb)(**args)

    # ── helpers ──────────────────────────────────────────────────────────────
    def _tel(self) -> Telemetry:
        return self.fc.telemetry()

    def _report(self, phase, fraction=NAN, line=''):
        if self.progress:
            self.progress(phase, fraction, line)

    def _ready_to_move(self) -> VerbResult | None:
        tel = self._tel()
        if not tel.link_alive:
            return VerbResult(Outcome.NOT_READY, 'no board heartbeat')
        if not tel.armed:
            return VerbResult(Outcome.NOT_READY, 'disarmed -- arm first')
        return None

    def _ensure_guided(self) -> VerbResult | None:
        tel = self._tel()
        if tel.mode == 'GUIDED':
            return None
        try:
            self.fc.set_mode('GUIDED', 5.0)
        except CommandRejected as exc:
            hint = '' if tel.ekf_ok else ' -- EKF has no absolute position yet'
            return VerbResult(Outcome.DENIED, f'GUIDED refused: {exc}{hint}', elapsed_s=0.0)
        return None

    def _zero(self) -> None:
        try:
            self.fc.velocity_body(0.0, 0.0)
        except Exception as exc:                    # noqa: BLE001 -- best effort on the way out
            if self.log:
                self.log.warning(f'[VERB ] zero setpoint failed: {exc}')

    def _done(self, outcome, msg, t0, final=NAN, err=NAN) -> VerbResult:
        return VerbResult(outcome, msg, final, err, self.clock() - t0)

    # ── meta / board verbs ───────────────────────────────────────────────────
    def arm(self, timeout: float) -> VerbResult:
        t0 = self.clock()
        tel = self._tel()
        if not tel.link_alive:
            return self._done(Outcome.NOT_READY, 'no board heartbeat', t0)
        fw = tuple(tel.firmware)
        if not fw:
            return self._done(Outcome.NOT_READY, 'firmware version not read yet', t0)
        if fw < Firmware.MIN_VERSION:
            floor = '.'.join(map(str, Firmware.MIN_VERSION))
            return self._done(Outcome.DENIED, f'firmware {fw} below the {floor} floor', t0)
        try:
            self.fc.arm(timeout)
        except CommandRejected as exc:
            return self._done(Outcome.DENIED, f'arm refused: {exc}', t0)
        return self._done(Outcome.SUCCEEDED, 'armed', t0)

    def disarm(self, timeout: float) -> VerbResult:
        t0 = self.clock()
        self._zero()
        try:
            self.fc.disarm(timeout)
        except CommandRejected as exc:
            return self._done(Outcome.FAILED, f'disarm refused: {exc}', t0)
        return self._done(Outcome.SUCCEEDED, 'disarmed', t0)

    def stop(self) -> VerbResult:
        t0 = self.clock()
        self._zero()
        try:
            self.fc.set_mode('HOLD', 5.0)
        except CommandRejected as exc:
            return self._done(Outcome.FAILED, f'HOLD refused: {exc}', t0)
        return self._done(Outcome.SUCCEEDED, 'holding', t0)

    def abort(self) -> VerbResult:
        self.request_abort()
        r = self.stop()
        r.message = 'aborted; ' + r.message
        return r

    def set_mode(self, target_name: str, timeout: float) -> VerbResult:
        t0 = self.clock()
        if not target_name:
            return self._done(Outcome.DENIED, 'set_mode needs target_name', t0)
        try:
            self.fc.set_mode(target_name, timeout)
        except CommandRejected as exc:
            return self._done(Outcome.DENIED, str(exc), t0)
        return self._done(Outcome.SUCCEEDED, f'mode {target_name.upper()}', t0)

    def hold(self) -> VerbResult:
        return self.set_mode('HOLD', 5.0)

    def _watch_arrival(self, distance_fn, radius, timeout, t0, label) -> VerbResult:
        """Poll the board's progress toward a target it is driving itself."""
        if math.isfinite(self.board_wp_radius) and radius < self.board_wp_radius:
            label = f'{label} (radius {radius:g} -> board WP_RADIUS {self.board_wp_radius:g})'
            radius = self.board_wp_radius + 0.25
        start = distance_fn()
        period = 1.0 / BOARD_POLL_HZ
        while True:
            if self.aborted:
                self._zero()
                return self._done(Outcome.ABORTED, f'{label}: aborted', t0, final=distance_fn())
            tel = self._tel()
            if not tel.link_alive:
                return self._done(Outcome.LOST, f'{label}: board link lost', t0)
            if tel.mode != 'GUIDED':
                return self._done(Outcome.FAILED, f'{label}: board left GUIDED ({tel.mode})',
                                  t0, final=distance_fn())
            d = distance_fn()
            if math.isfinite(d) and d <= radius:
                return self._done(Outcome.SUCCEEDED, f'{label}: arrived {d:.2f} m out',
                                  t0, final=d, err=d)
            if self.clock() - t0 > timeout:
                self.fc.set_mode('HOLD', 5.0)
                return self._done(Outcome.TIMEOUT, f'{label}: {d:.1f} m short after {timeout:.0f} s',
                                  t0, final=d, err=d)
            if math.isfinite(start) and start > 0 and math.isfinite(d):
                self._report(label, max(0.0, 1.0 - d / start), f'{d:.1f} m to go')
            self.sleep(period)

    def goto_gnss(self, lat: float, lon: float, radius: float, timeout: float) -> VerbResult:
        t0 = self.clock()
        if (blocked := self._ready_to_move()):
            return blocked
        tel = self._tel()
        if tel.gps_fix < GPS_FIX_MIN or not math.isfinite(tel.lat):
            return self._done(Outcome.NOT_READY, f'no 3D fix (fix_type {tel.gps_fix})', t0)
        if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0.0 and lon == 0.0):
            return self._done(Outcome.DENIED, f'bad target {lat}, {lon}', t0)
        if (blocked := self._ensure_guided()):
            return blocked
        self.fc.target_global(lat, lon)

        def dist():
            t = self._tel()
            return geo.distance_m(t.lat, t.lon, lat, lon) if math.isfinite(t.lat) else NAN
        return self._watch_arrival(dist, radius, timeout, t0, 'goto_gnss')

    def goto_local(self, x: float, y: float, radius: float, timeout: float) -> VerbResult:
        t0 = self.clock()
        if (blocked := self._ready_to_move()):
            return blocked
        if not math.isfinite(self._tel().north_m):
            return self._done(Outcome.NOT_READY, 'no local position (EKF origin unset?)', t0)
        if (blocked := self._ensure_guided()):
            return blocked
        self.fc.target_local(x, y)

        def dist():
            t = self._tel()
            return math.hypot(t.north_m - x, t.east_m - y)
        return self._watch_arrival(dist, radius, timeout, t0, 'goto_local')

    def drive(self, distance: float, radius: float, timeout: float) -> VerbResult:
        t0 = self.clock()
        if (blocked := self._ready_to_move()):
            return blocked
        tel = self._tel()
        if not (math.isfinite(tel.north_m) and math.isfinite(tel.yaw_deg)):
            return self._done(Outcome.NOT_READY, 'no local position or yaw', t0)
        if distance == 0.0:
            return self._done(Outcome.DENIED, 'drive needs a non-zero distance', t0)
        if (blocked := self._ensure_guided()):
            return blocked
        yaw = math.radians(tel.yaw_deg)
        tn = tel.north_m + distance * math.cos(yaw)
        te = tel.east_m + distance * math.sin(yaw)
        self.fc.target_body_offset(distance)

        def dist():
            t = self._tel()
            return math.hypot(t.north_m - tn, t.east_m - te)
        return self._watch_arrival(dist, radius, timeout, t0, 'drive')

    # ── host-streamed verbs ──────────────────────────────────────────────────
    def _stream(self, verb, duration, setpoint_fn, t0) -> VerbResult:
        """Stream `setpoint_fn(elapsed) -> (vx, yaw_rate) | _Target | VerbResult` at the verb's rate.

        A `_Target` hands the board a local position and lets it drive (its S-curves,
        its speed loop); it is re-sent only when it moves RETARGET_M. A position target
        does not expire like a velocity stream, so the exit path still zeroes it.
        """
        period = 1.0 / VERBS[verb]['stream_hz']
        self._sent_target = None
        try:
            while True:
                if self.aborted:
                    return self._done(Outcome.ABORTED, f'{verb}: aborted', t0)
                tel = self._tel()
                if not tel.link_alive:
                    return self._done(Outcome.LOST, f'{verb}: board link lost', t0)
                self._record_pose(tel)
                elapsed = self.clock() - t0
                if duration is not None and elapsed >= duration:
                    return self._done(Outcome.SUCCEEDED, f'{verb}: {elapsed:.1f} s', t0)
                sp = setpoint_fn(elapsed)
                if isinstance(sp, VerbResult):
                    return sp
                if isinstance(sp, _Target):
                    last = self._sent_target
                    if last is None or math.hypot(sp.north - last.north, sp.east - last.east) > RETARGET_M:
                        self.fc.target_local(sp.north, sp.east)
                        self._sent_target = sp
                        if self.log:
                            self.log.info(f'[VERB ] {verb}: target N {sp.north:.2f} E {sp.east:.2f} '
                                          f'from N {tel.north_m:.2f} E {tel.east_m:.2f}')
                else:
                    self._sent_target = None
                    self.fc.velocity_body(*sp)
                self.sleep(period)
        finally:
            self._zero()

    def _host_preflight(self):
        if (blocked := self._ready_to_move()):
            return blocked
        return self._ensure_guided()

    def move_forward(self, duration: float, speed: float) -> VerbResult:
        t0 = self.clock()
        if duration <= 0:
            return self._done(Outcome.DENIED, 'duration must be > 0', t0)
        if (blocked := self._host_preflight()):
            return blocked
        return self._stream('move_forward', duration, lambda _: (abs(speed), 0.0), t0)

    def move_back(self, duration: float, speed: float) -> VerbResult:
        t0 = self.clock()
        if duration <= 0:
            return self._done(Outcome.DENIED, 'duration must be > 0', t0)
        if (blocked := self._host_preflight()):
            return blocked
        return self._stream('move_back', duration, lambda _: (-abs(speed), 0.0), t0)

    def pivot(self, duration: float, yaw_rate: float) -> VerbResult:
        t0 = self.clock()
        if duration <= 0:
            return self._done(Outcome.DENIED, 'duration must be > 0', t0)
        if (blocked := self._host_preflight()):
            return blocked
        return self._stream('pivot', duration, lambda _: (0.0, yaw_rate), t0)

    def arc(self, duration: float, speed: float, yaw_rate: float) -> VerbResult:
        t0 = self.clock()
        if duration <= 0:
            return self._done(Outcome.DENIED, 'duration must be > 0', t0)
        if (blocked := self._host_preflight()):
            return blocked
        return self._stream('arc', duration, lambda _: (speed, yaw_rate), t0)

    def turn_to(self, heading: float, yaw_rate: float, timeout: float) -> VerbResult:
        t0 = self.clock()
        if (blocked := self._host_preflight()):
            return blocked
        if not math.isfinite(self._tel().yaw_deg):
            return self._done(Outcome.NOT_READY, 'no yaw from the board', t0)
        target = heading % 360.0
        limit = abs(yaw_rate)
        settled_since = [None]

        def step(elapsed):
            err = geo.heading_error(target, self._tel().yaw_deg)
            if abs(err) <= TURN_TOL_DEG:
                settled_since[0] = settled_since[0] if settled_since[0] is not None else elapsed
                if elapsed - settled_since[0] >= TURN_SETTLE_S:
                    return self._done(Outcome.SUCCEEDED, f'turn_to {target:.0f}: {err:+.1f} deg',
                                      t0, final=self._tel().yaw_deg, err=err)
            else:
                settled_since[0] = None
            if elapsed > timeout:
                return self._done(Outcome.TIMEOUT, f'turn_to {target:.0f}: {err:+.1f} deg left',
                                  t0, final=self._tel().yaw_deg, err=err)
            return 0.0, max(-limit, min(limit, TURN_KP * err))
        return self._stream('turn_to', None, step, t0)

    def turn(self, angle: float, yaw_rate: float, timeout: float) -> VerbResult:
        yaw = self._tel().yaw_deg
        if not math.isfinite(yaw):
            return VerbResult(Outcome.NOT_READY, 'no yaw from the board', elapsed_s=0.0)
        return self.turn_to((yaw + angle) % 360.0, yaw_rate, timeout)

    # ── vision verbs ─────────────────────────────────────────────────────────
    def _match(self, target_name: str, tag_id: int):
        """Freshest detection that matches, or None. Empty name + tag_id -1 = any tag.

        A NaN stamp is not fresh: an observation whose age is unknown is absent.
        """
        now = self.clock()
        best = None
        for d in self.vision():
            if not now - d.stamp <= DETECTION_FRESH_S:
                continue
            if tag_id >= 0:
                if d.tag_id != tag_id:
                    continue
            elif target_name:
                if d.label != target_name:
                    continue
            elif d.label != 'tag':
                continue
            if best is None or d.confidence > best.confidence:
                best = d
        return best

    def _record_pose(self, tel: Telemetry) -> None:
        if not (math.isfinite(tel.north_m) and math.isfinite(tel.east_m) and math.isfinite(tel.yaw_deg)):
            return
        now = self.clock()
        self._poses.append((now, tel.north_m, tel.east_m, tel.yaw_deg))
        while self._poses and now - self._poses[0][0] > POSE_HISTORY_S:
            self._poses.popleft()

    def _pose_at(self, t: float):
        """(north, east, yaw_deg) at clock time `t`, interpolated; None outside the history."""
        if not math.isfinite(t) or not self._poses:
            return None
        first, last = self._poses[0], self._poses[-1]
        if t < first[0] - POSE_MATCH_S or t > last[0] + POSE_MATCH_S:
            return None
        if t <= first[0]:
            return first[1:]
        if t >= last[0]:
            return last[1:]
        for a, b in zip(self._poses, list(self._poses)[1:], strict=False):
            if a[0] <= t <= b[0]:
                f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                yaw = (a[3] + f * geo.heading_error(b[3], a[3])) % 360.0
                return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), yaw
        return None

    def _observe(self, key, hit) -> None:
        """Fold one frame's detection into the target's local-frame estimate.

        The rover pose is taken at the IMAGE time, not now: a bearing from a frame
        0.4 s old, applied to a rover that has pivoted since, lands metres off.
        """
        pose = self._pose_at(hit.stamp)
        if pose is None or not (math.isfinite(hit.range_m) and math.isfinite(hit.bearing_deg)):
            return
        n, e, yaw = pose
        y = math.radians(yaw)
        cn, ce = n + self.camera_forward_m * math.cos(y), e + self.camera_forward_m * math.sin(y)
        h = math.radians(yaw + hit.bearing_deg)
        est = self._estimates.setdefault(key, _Estimate())
        fn, fe = cn + hit.range_m * math.cos(h), ce + hit.range_m * math.sin(h)
        est.add(fn, fe, hit.range_m, self.clock())
        if self.log:
            en, ee = est.position
            self.log.info(f'[TAG  ] {key}: {hit.range_m:.2f} m {hit.bearing_deg:+.1f} deg from '
                          f'N {n:.2f} E {e:.2f} yaw {yaw:.0f} -> fix N {fn:.2f} E {fe:.2f}, '
                          f'estimate N {en:.2f} E {ee:.2f} ({len(est.fixes)})')

    def tag_estimate(self, target_name: str, tag_id: int):
        """(north, east) of the target in the local frame, or None when there is none."""
        est = self._estimates.get((target_name, tag_id))
        if est is None or not est.fixes or not self.clock() - est.updated <= ESTIMATE_TTL_S:
            return None
        return est.position

    def _toward(self, key, short_m: float):
        """Setpoint toward the estimate: a local target `short_m` short of it, or a pivot
        to face it when the rover is already that close. None when there is no estimate
        or no local position. Second value: distance from the rover to the estimate."""
        est = self.tag_estimate(*key)
        tel = self._tel()
        if est is None or not (math.isfinite(tel.north_m) and math.isfinite(tel.yaw_deg)):
            return None, NAN
        dn, de = est[0] - tel.north_m, est[1] - tel.east_m
        d = math.hypot(dn, de)
        err = geo.heading_error(math.degrees(math.atan2(de, dn)), tel.yaw_deg)
        if d <= short_m + RETARGET_M:
            if abs(err) <= TURN_TOL_DEG:
                return (0.0, 0.0), d
            return (0.0, max(-30.0, min(30.0, APPROACH_YAW_KP * err))), d
        return _Target(est[0] - short_m * dn / d, est[1] - short_m * de / d), d

    def _camera_range(self, key) -> float:
        """Metres from the camera to the estimate, or NaN when either is unknown."""
        est = self.tag_estimate(*key)
        tel = self._tel()
        if est is None or not (math.isfinite(tel.north_m) and math.isfinite(tel.yaw_deg)):
            return NAN
        y = math.radians(tel.yaw_deg)
        cn = tel.north_m + self.camera_forward_m * math.cos(y)
        ce = tel.east_m + self.camera_forward_m * math.sin(y)
        return math.hypot(est[0] - cn, est[1] - ce)

    def search_spiral(self, radius: float, speed: float, target_name: str, tag_id: int,
                      timeout: float) -> VerbResult:
        t0 = self.clock()
        if (blocked := self._host_preflight()):
            return blocked
        key = (target_name, tag_id)
        # A retry keeps the estimate the failed approach just built. Wiping it is what
        # sent the rover spiraling 10 m past the post (B-32, second Gazebo run).
        s = {'prev': 0.0, 'spiral_s': 0.0, 'hits': 0, 'frame': NAN, 'confirm_since': None}

        def step(elapsed):
            dt, s['prev'] = elapsed - s['prev'], elapsed
            hit = self._match(target_name, tag_id)
            if hit is not None and hit.stamp != s['frame']:
                s['frame'] = hit.stamp
                s['hits'] += 1
                self._observe(key, hit)
                if s['confirm_since'] is None:
                    s['confirm_since'] = elapsed
            if s['hits'] >= SEARCH_CONFIRM_HITS:
                return self._done(Outcome.SUCCEEDED,
                                  f'search: saw {hit.label} {hit.tag_id} in {s["hits"]} frames, '
                                  f'last at {hit.bearing_deg:+.0f} deg',
                                  t0, final=hit.range_m, err=hit.bearing_deg)
            if elapsed > timeout:
                return self._done(Outcome.TIMEOUT, 'search: timed out', t0)
            held = self._estimates.get(key)
            if held is not None and len(held.fixes) >= SEARCH_CONFIRM_HITS:
                sp, _ = self._toward(key, SEARCH_CONFIRM_RANGE_M)
                if sp is not None:
                    return sp
            if s['confirm_since'] is not None:
                # One frame at the decode edge is not a find: close in on where it
                # was and look again. Nothing more in SEARCH_CONFIRM_S: it was not there.
                if elapsed - s['confirm_since'] <= SEARCH_CONFIRM_S:
                    sp, _ = self._toward(key, SEARCH_CONFIRM_RANGE_M)
                    if sp is not None:
                        return sp
                s['confirm_since'], s['hits'] = None, 0
                self._estimates.pop(key, None)
            s['spiral_s'] += dt
            r = math.sqrt(SPIRAL_SPACING_M * abs(speed) * s['spiral_s'] / math.pi)
            if r > radius:
                return self._done(Outcome.FAILED, f'search: nothing inside {radius:.0f} m', t0)
            return abs(speed), math.degrees(abs(speed) / max(SPIRAL_MIN_RADIUS_M, r))
        return self._stream('search_spiral', None, step, t0)

    def vision_approach(self, target_name: str, tag_id: int, standoff: float, speed: float,
                        timeout: float) -> VerbResult:
        """Close on a target to `standoff`.

        Inside APPROACH_SERVO_RANGE_M, steer on the pixels. Otherwise, drive the board
        to a point short of the target's estimated position (MRDT's first fallback).
        With no estimate (no local position), hold the last bearing for half the
        last-seen gap (B-30), the only fallback left.
        """
        t0 = self.clock()
        if (blocked := self._host_preflight()):
            return blocked
        key = (target_name, tag_id)
        state = {'last_seen': self.clock(), 'close': 0, 'coast': 0.0, 'blind_m': 0.0, 'prev': 0.0,
                 'frame': NAN, 'saw': False}

        def step(elapsed):
            hit = self._match(target_name, tag_id)
            now = self.clock()
            dt, state['prev'] = elapsed - state['prev'], elapsed
            if hit is not None:
                state['saw'] = True
                if not math.isfinite(hit.range_m):
                    return self._done(Outcome.FAILED,
                                      'approach: no metric range (camera uncalibrated or size unknown)', t0)
                state['last_seen'] = now
                if hit.stamp != state['frame']:
                    state['frame'] = hit.stamp
                    self._observe(key, hit)
            have_estimate = self.tag_estimate(*key) is not None
            if hit is not None and (hit.range_m <= APPROACH_SERVO_RANGE_M or not have_estimate):
                state['arrived'] = None
                return servo(hit, elapsed)
            state['close'] = 0
            if elapsed > timeout:
                return self._done(Outcome.TIMEOUT, 'approach: timed out', t0)
            cam = self._camera_range(key)
            # The board stops WP_RADIUS short of a position target, so the point we hand it
            # is that much closer to the tag. `there` is the camera, not the board's arrival.
            if have_estimate and math.isfinite(cam) and cam <= standoff + APPROACH_NEAR_M:
                if state['saw']:
                    return self._done(Outcome.SUCCEEDED,
                                      f'approach: {cam:.2f} m from the estimate, tag seen this pass',
                                      t0, final=cam, err=cam - standoff)
                if now - state['last_seen'] > TARGET_LOST_S:
                    return self._done(Outcome.LOST,
                                      f'approach: not seen at its estimated position '
                                      f'for {TARGET_LOST_S:.0f} s', t0)
            if have_estimate:
                sp, _ = self._toward(key, max(0.3, standoff - self._wp_radius()))
                if sp is not None:
                    return sp
            # Near the detection limit the tag flickers, and closing in is what brings
            # it back: hold the last bearing for up to half the last-seen gap.
            state['blind_m'] -= state['coast'] * dt
            if state['blind_m'] > 0.0:
                return state['coast'], 0.0
            if now - state['last_seen'] > TARGET_LOST_S:
                return self._done(Outcome.LOST, f'approach: target unseen {TARGET_LOST_S:.0f} s', t0)
            return 0.0, 0.0

        def servo(hit, elapsed):
            gap = hit.range_m - (standoff + APPROACH_NEAR_M)
            if gap <= 0:
                state['close'] += 1
                if state['close'] >= APPROACH_CONFIRM_FRAMES:
                    return self._done(Outcome.SUCCEEDED, f'approach: {hit.range_m:.2f} m',
                                      t0, final=hit.range_m, err=gap)
                return 0.0, 0.0
            state['close'] = 0
            state['coast'] = 0.0
            if elapsed > timeout:
                return self._done(Outcome.TIMEOUT, f'approach: {gap:.1f} m short', t0,
                                  final=hit.range_m, err=gap)
            yaw_rate = max(-30.0, min(30.0, APPROACH_YAW_KP * hit.bearing_deg))
            if abs(hit.bearing_deg) > APPROACH_TURN_FIRST_DEG:
                return 0.0, yaw_rate
            v = min(abs(speed), 0.5 * gap + 0.1)
            state['coast'], state['blind_m'] = v, 0.5 * gap
            return v, yaw_rate
        return self._stream('vision_approach', None, step, t0)

    def _wp_radius(self) -> float:
        return self.board_wp_radius if math.isfinite(self.board_wp_radius) else 0.0

    # ── settings ─────────────────────────────────────────────────────────────
    def set_ekf_source(self, target_name: str) -> VerbResult:
        t0 = self.clock()
        sets = {'gps': 1, 'extnav': 2}
        if target_name not in sets:
            return self._done(Outcome.DENIED, f'source must be one of {sorted(sets)}', t0)
        if target_name == 'extnav':
            try:
                viso = self.fc.get_param('VISO_TYPE')
            except CommandRejected as exc:
                return self._done(Outcome.FAILED, f'VISO_TYPE unreadable: {exc}', t0)
            if viso <= 0:
                return self._done(Outcome.DENIED,
                                  'VISO_TYPE=0: load rupsha_extnav.parm and reboot first', t0)
        try:
            self.fc.set_ekf_source(sets[target_name])
        except CommandRejected as exc:
            return self._done(Outcome.DENIED, str(exc), t0)
        return self._done(Outcome.SUCCEEDED, f'EKF source set {sets[target_name]} ({target_name})', t0)

    def set_origin(self, lat: float, lon: float) -> VerbResult:
        t0 = self.clock()
        if lat == 0.0 and lon == 0.0:
            return self._done(Outcome.DENIED, 'origin 0,0 is almost certainly unset fields', t0)
        self.fc.set_origin(lat, lon)
        return self._done(Outcome.SUCCEEDED, f'origin {lat:.7f}, {lon:.7f}', t0)

    def led(self, target_name: str) -> VerbResult:
        t0 = self.clock()
        try:
            state = Display[target_name.upper()]
        except KeyError:
            return self._done(Outcome.DENIED, f'led: one of {[d.name.lower() for d in Display]}', t0)
        self.display = state
        if self.led_sink:
            self.led_sink(state)
        return self._done(Outcome.SUCCEEDED, f'display {state.name}', t0)

    def wait(self, duration: float) -> VerbResult:
        t0 = self.clock()
        while self.clock() - t0 < duration:
            if self.aborted:
                return self._done(Outcome.ABORTED, 'wait: aborted', t0)
            self.sleep(min(0.1, duration))
        return self._done(Outcome.SUCCEEDED, f'waited {duration:.1f} s', t0)
