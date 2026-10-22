"""ArduRover over MAVLink. The ONLY module in the workspace that imports pymavlink.

THREADING RULE (mongla's, unchanged): exactly one thread calls `recv_match` --
the reader started by `start()`. Every other method reads the reader's cache and
sends under `_tx_lock`. Two readers would steal each other's COMMAND_ACKs.

What the board closes and what we close: position targets (GUIDED) are planned
and driven on the board with its own S-curves (GUID_OPTIONS bit 6). Velocity and
yaw-rate setpoints are host-streamed, and the board stops the rover 3 s after the
last one (Rover/mode_guided.cpp), which is the host-loss reflex we rely on.
"""

from __future__ import annotations

import math
import os
import threading
import time
from collections import deque

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil  # noqa: E402

from ..manifest import Companion, RoverMode  # noqa: E402
from .base import NAN, CommandRejected, FlightController, LinkDown, Telemetry  # noqa: E402

# A board HEARTBEAT older than this means the link is dead. ArduRover sends 1 Hz.
LINK_STALE_S = 2.0

MAV_RESULT = {0: 'ACCEPTED', 1: 'TEMP_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED',
              4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED'}

# ── SET_POSITION_TARGET type_mask ────────────────────────────────────────────
# ⚠ Rover tests each group with `type_mask & GROUP` (GCS_MAVLink_Rover.cpp), so
# setting ANY one bit of a group ignores the WHOLE group. The commonly quoted
# velocity+yaw-rate mask 1511 sets VZ_IGNORE and the board then drops the
# velocity entirely. Every group below is either fully set or fully clear.
_POS = 0b000000000111
_VEL = 0b000000111000
_ACC = 0b000111000000      # must always be set: Rover refuses any non-ignored accel
_YAW = 0b010000000000
_YAW_RATE = 0b100000000000
MASK_POSITION = _VEL | _ACC | _YAW | _YAW_RATE               # 3576
MASK_VEL_YAWRATE = _POS | _ACC | _YAW                        # 1479
MASK_YAWRATE_ONLY = _POS | _VEL | _ACC | _YAW                # 1535
MASK_HEADING_ONLY = _POS | _VEL | _ACC | _YAW_RATE           # 2559

FRAME_LOCAL_NED = 1
FRAME_GLOBAL_RELATIVE_ALT_INT = 6
FRAME_BODY_OFFSET_NED = 9

# Streams the manager reads, Hz. Requested by MAV_CMD_SET_MESSAGE_INTERVAL so they
# do not depend on the SRx_/MAVx_ parameters a factory reset would put back to 1.
STREAMS_HZ = {
    'ATTITUDE': 20, 'RAW_IMU': 20, 'GLOBAL_POSITION_INT': 10, 'LOCAL_POSITION_NED': 10,
    'VFR_HUD': 10, 'GPS_RAW_INT': 5, 'SYS_STATUS': 2, 'EKF_STATUS_REPORT': 2,
    'SERVO_OUTPUT_RAW': 5,
}

_EKF_NAV_FLAGS = (mavutil.mavlink.EKF_ATTITUDE | mavutil.mavlink.EKF_VELOCITY_HORIZ
                  | mavutil.mavlink.EKF_POS_HORIZ_ABS)
_MODE_NAMES = {m.value: m.name for m in RoverMode}


def parse_version(flight_sw_version: int) -> tuple:
    """AUTOPILOT_VERSION.flight_sw_version -> (major, minor, patch)."""
    v = int(flight_sw_version)
    return ((v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF)


class ArduRover(FlightController):

    def __init__(self, url: str, baud: int = 115200, log=None,
                 sysid: int = Companion.SYSID, compid: int = Companion.COMPID):
        self.url = url
        self.log = log
        self._conn = mavutil.mavlink_connection(
            url, baud=baud, source_system=sysid, source_component=compid,
            autoreconnect=True)
        self._tx_lock = threading.Lock()
        self._lock = threading.Lock()
        self._msgs: dict = {}            # type -> (monotonic, msg)
        self._acks: dict = {}            # command id -> (monotonic, result)
        self._params: dict = {}          # name -> (monotonic, value)
        self._texts: deque = deque(maxlen=50)
        self._target = (1, 1)
        self._firmware: tuple = ()
        self._stop = threading.Event()
        self._reader = None
        self._listeners = []             # callables(msg) run on the reader thread
        self.reader_errors = 0

    # ── lifecycle ────────────────────────────────────────────────────────────
    def wait_board(self, timeout: float = 10.0) -> None:
        """Block until an autopilot HEARTBEAT arrives (not a GCS's). Before start()."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.send_heartbeat()
            hb = self._conn.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if hb is None:
                continue
            if hb.type == mavutil.mavlink.MAV_TYPE_GCS or \
                    hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue
            self._target = (hb.get_srcSystem(), hb.get_srcComponent())
            self._store(hb)
            return
        raise LinkDown(f'no autopilot HEARTBEAT on {self.url} within {timeout:.0f} s')

    def start(self) -> None:
        self._reader = threading.Thread(target=self._read_loop, name='mav-reader', daemon=True)
        self._reader.start()
        for name, hz in STREAMS_HZ.items():
            self._set_interval(name, hz)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)

    def add_listener(self, fn) -> None:
        self._listeners.append(fn)

    def close(self) -> None:
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=1.0)
        self._conn.close()

    # ── reader ───────────────────────────────────────────────────────────────
    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.2)
            except Exception:                    # noqa: BLE001 -- a serial hiccup must not kill the reader
                self.reader_errors += 1
                time.sleep(0.05)
                continue
            if msg is None or msg.get_type() == 'BAD_DATA':
                continue
            if (msg.get_srcSystem(), msg.get_srcComponent()) != self._target:
                continue
            self._store(msg)
            for fn in self._listeners:
                try:
                    fn(msg)
                except Exception:                # noqa: BLE001 -- a listener bug must not kill the reader
                    self.reader_errors += 1

    def _store(self, msg) -> None:
        now = time.monotonic()
        t = msg.get_type()
        with self._lock:
            self._msgs[t] = (now, msg)
            if t == 'COMMAND_ACK':
                self._acks[msg.command] = (now, msg.result)
            elif t == 'PARAM_VALUE':
                self._params[msg.param_id] = (now, msg.param_value)
            elif t == 'STATUSTEXT':
                self._texts.append((now, msg.text))
            elif t == 'AUTOPILOT_VERSION':
                self._firmware = parse_version(msg.flight_sw_version)

    def _latest(self, t: str, max_age: float = math.inf):
        with self._lock:
            entry = self._msgs.get(t)
        if entry is None or time.monotonic() - entry[0] > max_age:
            return None
        return entry[1]

    def last_rx(self) -> float:
        """Monotonic time of the newest message from the board; -inf before the first."""
        with self._lock:
            return max((ts for ts, _ in self._msgs.values()), default=-math.inf)

    def statustexts(self, since: float = 0.0) -> list:
        with self._lock:
            return [txt for ts, txt in self._texts if ts >= since]

    # ── telemetry ────────────────────────────────────────────────────────────
    def telemetry(self) -> Telemetry:
        tel = Telemetry(firmware=self._firmware)
        # Alive on any message from the board, not HEARTBEAT alone: it is 1 Hz in the
        # board's time, so a sim running at 0.4x real time spaces it 2.5 s apart.
        tel.link_alive = time.monotonic() - self.last_rx() <= LINK_STALE_S
        hb = self._latest('HEARTBEAT') if tel.link_alive else None
        if hb is not None:
            tel.armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            tel.mode = _MODE_NAMES.get(hb.custom_mode, f'MODE_{hb.custom_mode}')
        stale = 2.0
        gp = self._latest('GLOBAL_POSITION_INT', stale)
        if gp is not None and (gp.lat or gp.lon):
            tel.lat, tel.lon = gp.lat * 1e-7, gp.lon * 1e-7
            tel.alt_m = gp.relative_alt * 1e-3
        att = self._latest('ATTITUDE', stale)
        if att is not None:
            tel.yaw_deg = math.degrees(att.yaw) % 360.0
            tel.yaw_rate_dps = math.degrees(att.yawspeed)
        vfr = self._latest('VFR_HUD', stale)
        if vfr is not None:
            tel.groundspeed = float(vfr.groundspeed)
        lp = self._latest('LOCAL_POSITION_NED', stale)
        if lp is not None:
            tel.north_m, tel.east_m = float(lp.x), float(lp.y)
            if att is not None:
                tel.vx_body = lp.vx * math.cos(att.yaw) + lp.vy * math.sin(att.yaw)
        ss = self._latest('SYS_STATUS', 5.0)
        if ss is not None:
            if ss.voltage_battery not in (0, 0xFFFF):
                tel.battery_v = ss.voltage_battery * 1e-3
            bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
            if ss.onboard_control_sensors_present & bit:
                tel.prearm_ok = bool(ss.onboard_control_sensors_health & bit)
        gps = self._latest('GPS_RAW_INT', 5.0)
        if gps is not None:
            tel.gps_fix = int(gps.fix_type)
            tel.gps_sats = int(gps.satellites_visible) if gps.satellites_visible != 255 else -1
        ekf = self._latest('EKF_STATUS_REPORT', 5.0)
        if ekf is not None:
            tel.ekf_ok = (ekf.flags & _EKF_NAV_FLAGS) == _EKF_NAV_FLAGS
        texts = self.statustexts()
        tel.last_text = texts[-1] if texts else ''
        last = self.last_rx()
        tel.stamp = last if math.isfinite(last) else NAN
        return tel

    def raw(self, msg_type: str, max_age: float = 2.0):
        """Latest cached message of a type, for the IMU/servo publishers."""
        return self._latest(msg_type, max_age)

    @property
    def firmware(self) -> tuple:
        return self._firmware

    # ── sending ──────────────────────────────────────────────────────────────
    def _send(self, fn, *args) -> None:
        with self._tx_lock:
            fn(*args)

    def send_heartbeat(self) -> None:
        self._send(self._conn.mav.heartbeat_send,
                   mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                   mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                   mavutil.mavlink.MAV_STATE_ACTIVE)

    def _command_long(self, command: int, *params) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self._send(self._conn.mav.command_long_send, *self._target, command, 0, *p)

    def command(self, command: int, *params, timeout: float = 3.0) -> int:
        """COMMAND_LONG and wait for its ACK on the reader's cache. Raises on non-ACCEPTED."""
        with self._lock:
            self._acks.pop(command, None)
        sent = time.monotonic()
        self._command_long(command, *params)
        deadline = sent + timeout
        while time.monotonic() < deadline:
            with self._lock:
                ack = self._acks.get(command)
            if ack is not None and ack[0] >= sent:
                if ack[1] == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    time.sleep(0.02)
                    continue
                if ack[1] != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    texts = self.statustexts(since=sent)
                    why = f' ({texts[-1]})' if texts else ''
                    raise CommandRejected(
                        f'command {command} {MAV_RESULT.get(ack[1], ack[1])}{why}')
                return ack[1]
            time.sleep(0.02)
        raise CommandRejected(f'command {command}: no COMMAND_ACK within {timeout:.1f} s')

    def _set_interval(self, msg_name: str, hz: float) -> None:
        msg_id = getattr(mavutil.mavlink, f'MAVLINK_MSG_ID_{msg_name}')
        self._command_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def _request_message(self, msg_id: int) -> None:
        self._command_long(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, msg_id)

    def _wait_heartbeat(self, pred, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            hb = self._latest('HEARTBEAT', LINK_STALE_S)
            if hb is not None and pred(hb):
                return True
            time.sleep(0.05)
        return False

    def arm(self, timeout: float = 10.0) -> None:
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, timeout=timeout)
        armed = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if not self._wait_heartbeat(lambda hb: hb.base_mode & armed, timeout):
            raise CommandRejected('arm ACKed but the HEARTBEAT never showed armed')

    def disarm(self, timeout: float = 10.0, force: bool = False) -> None:
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                     21196 if force else 0, timeout=timeout)
        armed = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if not self._wait_heartbeat(lambda hb: not hb.base_mode & armed, timeout):
            raise CommandRejected('disarm ACKed but the HEARTBEAT still shows armed')

    def set_mode(self, name: str, timeout: float = 5.0) -> None:
        try:
            number = RoverMode[name.upper()].value
        except KeyError:
            raise CommandRejected(f'unknown mode {name!r}; one of {[m.name for m in RoverMode]}') from None
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, number, timeout=timeout)
        if not self._wait_heartbeat(lambda hb: hb.custom_mode == number, timeout):
            raise CommandRejected(f'{name} ACKed but the HEARTBEAT never showed it')

    def target_global(self, lat: float, lon: float) -> None:
        self._send(self._conn.mav.set_position_target_global_int_send,
                   0, *self._target, FRAME_GLOBAL_RELATIVE_ALT_INT, MASK_POSITION,
                   int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0,
                   0, 0, 0, 0, 0, 0, 0, 0)

    def _local(self, frame: int, mask: int, x=0.0, y=0.0, vx=0.0, yaw=0.0, yaw_rate=0.0):
        self._send(self._conn.mav.set_position_target_local_ned_send,
                   0, *self._target, frame, mask, x, y, 0.0, vx, 0.0, 0.0,
                   0, 0, 0, yaw, yaw_rate)

    def target_local(self, north_m: float, east_m: float) -> None:
        self._local(FRAME_LOCAL_NED, MASK_POSITION, x=north_m, y=east_m)

    def target_body_offset(self, forward_m: float) -> None:
        self._local(FRAME_BODY_OFFSET_NED, MASK_POSITION, x=forward_m)

    def velocity_body(self, vx: float, yaw_rate_dps: float) -> None:
        if vx == 0.0:
            # A pivot is sent as yaw-rate-only so the board's log shows the intent;
            # Rover takes speed from |v| and direction from sign(vx) otherwise.
            self._local(FRAME_BODY_OFFSET_NED, MASK_YAWRATE_ONLY,
                        yaw_rate=math.radians(yaw_rate_dps))
            return
        self._local(FRAME_BODY_OFFSET_NED, MASK_VEL_YAWRATE, vx=vx,
                    yaw_rate=math.radians(yaw_rate_dps))

    def heading(self, heading_deg: float) -> None:
        """Pivot to an absolute heading; the board closes the heading loop."""
        self._local(FRAME_LOCAL_NED, MASK_HEADING_ONLY, yaw=math.radians(heading_deg))

    def set_ekf_source(self, source_set: int) -> None:
        self.command(mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET, int(source_set))

    def set_origin(self, lat: float, lon: float) -> None:
        self._send(self._conn.mav.set_gps_global_origin_send, self._target[0],
                   int(round(lat * 1e7)), int(round(lon * 1e7)), 0, int(time.time() * 1e6))

    def get_param(self, name: str, timeout: float = 3.0) -> float:
        sent = time.monotonic()
        self._send(self._conn.mav.param_request_read_send, *self._target,
                   name.encode(), -1)
        deadline = sent + timeout
        while time.monotonic() < deadline:
            with self._lock:
                got = self._params.get(name)
            if got is not None and got[0] >= sent:
                return float(got[1])
            time.sleep(0.02)
        raise CommandRejected(f'param {name}: no PARAM_VALUE within {timeout:.1f} s')

    def fetch_all_params(self, timeout: float = 30.0) -> dict:
        """PARAM_REQUEST_LIST, gathered until the count stops growing."""
        with self._lock:
            self._params.clear()
        self._send(self._conn.mav.param_request_list_send, *self._target)
        deadline = time.monotonic() + timeout
        last, quiet_since = -1, time.monotonic()
        while time.monotonic() < deadline:
            time.sleep(0.25)
            with self._lock:
                n = len(self._params)
            if n != last:
                last, quiet_since = n, time.monotonic()
            elif n and time.monotonic() - quiet_since > 2.0:
                break
        with self._lock:
            return {k: float(v) for k, (_, v) in self._params.items()}

    def set_param(self, name: str, value: float, timeout: float = 3.0) -> None:
        sent = time.monotonic()
        self._send(self._conn.mav.param_set_send, *self._target, name.encode(),
                   float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = sent + timeout
        while time.monotonic() < deadline:
            with self._lock:
                got = self._params.get(name)
            if got is not None and got[0] >= sent:
                if not math.isclose(got[1], value, rel_tol=1e-6, abs_tol=1e-6):
                    raise CommandRejected(f'param {name}: set {value}, board kept {got[1]}')
                return
            time.sleep(0.02)
        raise CommandRejected(f'param {name}: no PARAM_VALUE echo within {timeout:.1f} s')

    def set_servo(self, channel: int, pwm: int) -> None:
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, channel, pwm)

    def send_odometry(self, *, time_usec: int, x: float, y: float, z: float, q,
                      vx: float, vy: float, vz: float, pos_var: float, yaw_var: float,
                      reset_counter: int = 0, quality: int = 50) -> None:
        """MAVLink ODOMETRY for EK3 ExtNav (source set 2). frame LOCAL_FRD, child BODY_FRD."""
        cov = [math.nan] * 21
        cov[0] = cov[6] = cov[11] = pos_var
        cov[20] = yaw_var
        self._send(self._conn.mav.odometry_send, time_usec,
                   mavutil.mavlink.MAV_FRAME_LOCAL_FRD, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                   x, y, z, q, vx, vy, vz, math.nan, math.nan, math.nan,
                   cov, [math.nan] * 21, reset_counter,
                   mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION, quality)

    def reboot(self) -> None:
        self._command_long(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
