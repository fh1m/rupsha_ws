"""The board boundary: what the facade may ask of a flight controller.

Duck-typed on purpose (mongla's fc/base.py, same shape). `ArduRover` is the real
one; `test/fake_fc.py` is the one the facade's tests drive. Anything that needs a
MAVLink detail the boundary does not expose is a missing method here, not a
pymavlink import somewhere else.

Units at this boundary are SI and degrees, positive yaw rate = clockwise seen from
above (NED), headings 0..360 from north. Absent values are NaN; tri-state flags
are Optional[bool] with None = "the board has not said".
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

NAN = math.nan


@dataclass
class Telemetry:
    """One snapshot of the board's cached state. Never blocks to build."""
    link_alive: bool = False
    armed: Optional[bool] = None
    mode: str = ''
    lat: float = NAN
    lon: float = NAN
    alt_m: float = NAN
    yaw_deg: float = NAN
    yaw_rate_dps: float = NAN
    groundspeed: float = NAN
    vx_body: float = NAN
    battery_v: float = NAN
    gps_fix: int = -1
    gps_sats: int = -1
    ekf_ok: Optional[bool] = None
    prearm_ok: Optional[bool] = None
    # Local NED from LOCAL_POSITION_NED, relative to the EKF origin.
    north_m: float = NAN
    east_m: float = NAN
    last_text: str = ''
    # Monotonic time this snapshot's newest field arrived; NaN before any.
    stamp: float = NAN
    firmware: tuple = ()
    extra: dict = field(default_factory=dict)


class CommandRejected(RuntimeError):
    """The board answered COMMAND_ACK with something other than ACCEPTED."""


class LinkDown(RuntimeError):
    """No board HEARTBEAT inside the staleness window."""


class FlightController:
    """Interface only. Every method returns promptly; nothing here loops on motion."""

    def telemetry(self) -> Telemetry:
        raise NotImplementedError

    def send_heartbeat(self) -> None:
        raise NotImplementedError

    def arm(self, timeout: float) -> None:
        raise NotImplementedError

    def disarm(self, timeout: float, force: bool = False) -> None:
        raise NotImplementedError

    def set_mode(self, name: str, timeout: float) -> None:
        raise NotImplementedError

    def target_global(self, lat: float, lon: float) -> None:
        """GUIDED position target; the board plans and drives to it."""
        raise NotImplementedError

    def target_local(self, north_m: float, east_m: float) -> None:
        raise NotImplementedError

    def target_body_offset(self, forward_m: float) -> None:
        raise NotImplementedError

    def velocity_body(self, vx: float, yaw_rate_dps: float) -> None:
        """One GUIDED velocity + yaw-rate setpoint. The board stops after 3 s without one."""
        raise NotImplementedError

    def set_ekf_source(self, source_set: int) -> None:
        raise NotImplementedError

    def set_origin(self, lat: float, lon: float) -> None:
        raise NotImplementedError

    def get_param(self, name: str, timeout: float = 3.0) -> float:
        raise NotImplementedError

    def set_param(self, name: str, value: float, timeout: float = 3.0) -> None:
        raise NotImplementedError

    def send_odometry(self, **kw) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError
