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
from dataclasses import dataclass

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
        """Stream `setpoint_fn(elapsed) -> (vx, yaw_rate) | VerbResult` at the verb's rate."""
        period = 1.0 / VERBS[verb]['stream_hz']
        try:
            while True:
                if self.aborted:
                    return self._done(Outcome.ABORTED, f'{verb}: aborted', t0)
                if not self._tel().link_alive:
                    return self._done(Outcome.LOST, f'{verb}: board link lost', t0)
                elapsed = self.clock() - t0
                if duration is not None and elapsed >= duration:
                    return self._done(Outcome.SUCCEEDED, f'{verb}: {elapsed:.1f} s', t0)
                sp = setpoint_fn(elapsed)
                if isinstance(sp, VerbResult):
                    return sp
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
        """Freshest detection that matches, or None. Empty name + tag_id -1 = any tag."""
        now = self.clock()
        best = None
        for d in self.vision():
            if now - d.stamp > DETECTION_FRESH_S:
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

    def search_spiral(self, radius: float, speed: float, target_name: str, tag_id: int,
                      timeout: float) -> VerbResult:
        t0 = self.clock()
        if (blocked := self._host_preflight()):
            return blocked

        def step(elapsed):
            hit = self._match(target_name, tag_id)
            if hit is not None:
                return self._done(Outcome.SUCCEEDED,
                                  f'search: saw {hit.label} {hit.tag_id} at {hit.bearing_deg:+.0f} deg',
                                  t0, final=hit.range_m, err=hit.bearing_deg)
            r = math.sqrt(SPIRAL_SPACING_M * abs(speed) * elapsed / math.pi)
            if r > radius:
                return self._done(Outcome.FAILED, f'search: nothing inside {radius:.0f} m', t0)
            if elapsed > timeout:
                return self._done(Outcome.TIMEOUT, 'search: timed out', t0)
            return abs(speed), math.degrees(abs(speed) / max(SPIRAL_MIN_RADIUS_M, r))
        return self._stream('search_spiral', None, step, t0)

    def vision_approach(self, target_name: str, tag_id: int, standoff: float, speed: float,
                        timeout: float) -> VerbResult:
        t0 = self.clock()
        if (blocked := self._host_preflight()):
            return blocked
        state = {'last_seen': self.clock(), 'close': 0}

        def step(elapsed):
            hit = self._match(target_name, tag_id)
            now = self.clock()
            if hit is None:
                if now - state['last_seen'] > TARGET_LOST_S:
                    return self._done(Outcome.LOST, f'approach: target unseen {TARGET_LOST_S:.0f} s', t0)
                return 0.0, 0.0
            state['last_seen'] = now
            if not math.isfinite(hit.range_m):
                return self._done(Outcome.FAILED,
                                  'approach: no metric range (camera uncalibrated or size unknown)', t0)
            gap = hit.range_m - standoff
            if gap <= 0:
                state['close'] += 1
                if state['close'] >= APPROACH_CONFIRM_FRAMES:
                    return self._done(Outcome.SUCCEEDED, f'approach: {hit.range_m:.2f} m',
                                      t0, final=hit.range_m, err=gap)
                return 0.0, 0.0
            state['close'] = 0
            if elapsed > timeout:
                return self._done(Outcome.TIMEOUT, f'approach: {gap:.1f} m short', t0,
                                  final=hit.range_m, err=gap)
            yaw_rate = max(-30.0, min(30.0, APPROACH_YAW_KP * hit.bearing_deg))
            if abs(hit.bearing_deg) > APPROACH_TURN_FIRST_DEG:
                return 0.0, yaw_rate
            return min(abs(speed), 0.5 * gap + 0.1), yaw_rate
        return self._stream('vision_approach', None, step, t0)

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
