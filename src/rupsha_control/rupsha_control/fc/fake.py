"""A kinematic stand-in for ArduRover: same boundary, simulated time.

Not a simulator of ArduPilot -- a unicycle that obeys the GUIDED contract closely
enough to referee the facade and the executive: position targets are driven at
cruise speed with a turn-then-go controller, velocity setpoints expire after 3 s
like the real board, HOLD stops, disarm stops. Time only advances when the code
under test sleeps, so a 120 s leg runs in milliseconds and runs the same way twice.

Fault hooks (`stuck`, `link_alive`, `refuse_arm`, `slip`) let tests stage the
cases that matter instead of hoping the fake wanders into them.
"""

from __future__ import annotations

import math

from .. import geo
from ..manifest import Firmware
from .base import CommandRejected, FlightController, Telemetry

GUIDED_SETPOINT_TIMEOUT_S = 3.0
SIM_DT = 0.02


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


class FakeRover(FlightController):

    def __init__(self, lat=-35.363262, lon=149.165237, yaw_deg=0.0, cruise=1.0,
                 max_yaw_rate=60.0, firmware=Firmware.MIN_VERSION, clock=None):
        self.clock = clock or FakeClock()
        self.origin = (lat, lon)
        self.north, self.east = 0.0, 0.0
        self.yaw = yaw_deg
        self.v = 0.0
        self.w = 0.0
        self.cruise = cruise
        self.max_yaw_rate = max_yaw_rate
        self.armed = False
        self.mode = 'HOLD'
        self.firmware = tuple(firmware)
        self.gps_fix = 3
        self.link_alive = True
        self.refuse_arm = ''
        self.stuck = False          # wheels turn, body does not move
        self.rocks = []             # (north, east, radius): motion INTO one is blocked
        self.actual_v = 0.0
        self.params = {'VISO_TYPE': 0.0}
        self.ekf_source = 1
        self.sent = []              # (kind, args) log of every command
        self._pos_target = None     # (north, east)
        self._heading_target = None
        self._vel = None            # (vx, w, stamp)
        self.heartbeats = 0
        self.odometry = []

    # ── time ─────────────────────────────────────────────────────────────────
    def sleep(self, dt: float) -> None:
        end = self.clock.t + dt
        while self.clock.t < end - 1e-12:
            step = min(SIM_DT, end - self.clock.t)
            self._integrate(step)
            self.clock.t += step

    def _integrate(self, dt: float) -> None:
        v, w = self._command()
        self.v, self.w = v, w
        if self.stuck:
            v = 0.0
        self.yaw = (self.yaw + w * dt) % 360.0
        y = math.radians(self.yaw)
        n = self.north + v * math.cos(y) * dt
        e = self.east + v * math.sin(y) * dt
        if any(math.hypot(n - rn, e - re) < rr
               and math.hypot(n - rn, e - re) < math.hypot(self.north - rn, self.east - re)
               for rn, re, rr in self.rocks):
            v = 0.0                    # moving into a rock: wheels spin, body stays
        else:
            self.north, self.east = n, e
        self.actual_v = v

    def _command(self):
        if not self.armed or self.mode != 'GUIDED':
            return 0.0, 0.0
        if self._vel is not None:
            vx, w, stamp = self._vel
            if self.clock.t - stamp > GUIDED_SETPOINT_TIMEOUT_S:
                self._vel = None
                return 0.0, 0.0
            return vx, w
        if self._heading_target is not None:
            err = geo.heading_error(self._heading_target, self.yaw)
            return 0.0, max(-self.max_yaw_rate, min(self.max_yaw_rate, 2.0 * err))
        if self._pos_target is not None:
            dn, de = self._pos_target[0] - self.north, self._pos_target[1] - self.east
            d = math.hypot(dn, de)
            if d < 0.2:
                return 0.0, 0.0
            err = geo.heading_error(math.degrees(math.atan2(de, dn)), self.yaw)
            w = max(-self.max_yaw_rate, min(self.max_yaw_rate, 2.0 * err))
            v = self.cruise * max(0.0, math.cos(math.radians(err))) * min(1.0, d)
            return v, w
        return 0.0, 0.0

    # ── boundary ─────────────────────────────────────────────────────────────
    @property
    def latlon(self):
        return geo.offset(self.origin[0], self.origin[1], self.north, self.east)

    def telemetry(self) -> Telemetry:
        if not self.link_alive:
            return Telemetry(link_alive=False, stamp=self.clock.t)
        lat, lon = self.latlon if self.gps_fix >= 3 else (math.nan, math.nan)
        return Telemetry(link_alive=True, armed=self.armed, mode=self.mode, lat=lat, lon=lon,
                         yaw_deg=self.yaw, yaw_rate_dps=self.w, groundspeed=abs(self.actual_v),
                         vx_body=self.actual_v, gps_fix=self.gps_fix,
                         gps_sats=12 if self.gps_fix >= 3 else 0, ekf_ok=True, prearm_ok=True,
                         north_m=self.north, east_m=self.east, stamp=self.clock.t,
                         firmware=self.firmware)

    def send_heartbeat(self) -> None:
        self.heartbeats += 1

    def arm(self, timeout: float = 10.0) -> None:
        self.sent.append(('arm', ()))
        if self.refuse_arm:
            raise CommandRejected(f'command 400 FAILED ({self.refuse_arm})')
        self.armed = True

    def disarm(self, timeout: float = 10.0, force: bool = False) -> None:
        self.sent.append(('disarm', ()))
        self.armed = False
        self._clear()

    def set_mode(self, name: str, timeout: float = 5.0) -> None:
        self.sent.append(('set_mode', (name,)))
        name = name.upper()
        if name not in ('MANUAL', 'ACRO', 'STEERING', 'HOLD', 'LOITER', 'AUTO', 'RTL',
                        'SMART_RTL', 'GUIDED'):
            raise CommandRejected(f'unknown mode {name!r}')
        self.mode = name
        self._clear()

    def _clear(self):
        self._pos_target = self._heading_target = self._vel = None

    def _require_guided(self, kind):
        if self.mode != 'GUIDED':
            self.sent.append((kind + '_ignored', ()))
            return False
        return True

    def target_global(self, lat: float, lon: float) -> None:
        self.sent.append(('target_global', (lat, lon)))
        if self._require_guided('target_global'):
            n = geo.distance_m(self.origin[0], self.origin[1], lat, self.origin[1])
            n = math.copysign(n, lat - self.origin[0])
            e = geo.distance_m(lat, self.origin[1], lat, lon)
            e = math.copysign(e, lon - self.origin[1])
            self._clear()
            self._pos_target = (n, e)

    def target_local(self, north_m: float, east_m: float) -> None:
        self.sent.append(('target_local', (north_m, east_m)))
        if self._require_guided('target_local'):
            self._clear()
            self._pos_target = (north_m, east_m)

    def target_body_offset(self, forward_m: float) -> None:
        self.sent.append(('target_body_offset', (forward_m,)))
        if self._require_guided('target_body_offset'):
            y = math.radians(self.yaw)
            self._clear()
            self._pos_target = (self.north + forward_m * math.cos(y),
                                self.east + forward_m * math.sin(y))

    def velocity_body(self, vx: float, yaw_rate_dps: float) -> None:
        self.sent.append(('velocity_body', (vx, yaw_rate_dps)))
        if self._require_guided('velocity_body'):
            self._pos_target = self._heading_target = None
            self._vel = (vx, max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate_dps)),
                         self.clock.t)

    def heading(self, heading_deg: float) -> None:
        self.sent.append(('heading', (heading_deg,)))
        if self._require_guided('heading'):
            self._clear()
            self._heading_target = heading_deg % 360.0

    def set_ekf_source(self, source_set: int) -> None:
        self.sent.append(('set_ekf_source', (source_set,)))
        self.ekf_source = source_set

    def set_origin(self, lat: float, lon: float) -> None:
        self.sent.append(('set_origin', (lat, lon)))
        self.origin = (lat, lon)

    def get_param(self, name: str, timeout: float = 3.0) -> float:
        if name not in self.params:
            raise CommandRejected(f'param {name}: no PARAM_VALUE')
        return self.params[name]

    def set_param(self, name: str, value: float, timeout: float = 3.0) -> None:
        self.params[name] = float(value)

    def send_odometry(self, **kw) -> None:
        self.odometry.append(kw)

    def close(self) -> None:
        pass
