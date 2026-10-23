# The board contract

The ESP32 firmware is written from scratch against this page. Lazygo's
sketch is the cautionary read, not the starting point: a 12.5 Hz speed
loop, a two-byte map that cannot carry a path, parking turns living on
the microcontroller, and a BNO055 whose Euler angles they then had to
unwrap by hand.

The split is Mongla's, from [the shift](../the-shift.md). One core does
the sensors, the speed loop, and the motor, and nothing that can wait is
allowed on it. The Pi can stall on a camera frame. The wheels still stop.

A hobby servo is a 50 Hz pulse. This is not Mongla's 500 Hz DShot loop,
and 500 Hz is not a number this page claims. The inner loop is pinned to
one core and aimed at **200 Hz**. That rate is a design target. It is
unmeasured until a scope on the real board shows it.

## Who does what

**Core 1, the loop.** Encoder count, one BNO085 sample, the speed PI,
the steering pulse, and the stop. No serial parsing, no GPS sentences,
no OLED, no mission.

**Core 0, everything that can wait.** The framed link to the Pi, the
M8N parse, the OLED. A late GPS sentence cannot delay core 1.

**The Pi 5.** Missions, the camera, detections, the path. It sends a
speed and a steer. It does not send a parking manoeuvre. Their ESP32
ran `moveLeft` and `moveRight` as firmware commands. A new mission here
is a node, not a flash.

## The link

Pi to board:

| Field | Unit | Meaning |
|---|---|---|
| `speed_mms` | mm/s | signed. Positive is forward. Zero is stop. |
| `steer_rad` | rad | bicycle angle, positive left. The board clamps it to the servo. |
| `camera_pulse_us` | µs | camera servo pulse. 0 leaves it where it is. |
| `heartbeat` | count | changes while the Pi is alive. |

Board to Pi, at 50 Hz:

| Field | Meaning |
|---|---|
| `encoder_ticks` | signed count. 1873 ticks is one metre of drive-axle travel, their calibration, kept because it is the motor we are bolting on. |
| `orientation` | quaternion from the BNO085 gyro-integrated rotation vector. Not an Euler triple. |
| `latitude_deg`, `longitude_deg`, `altitude_m`, `gps_fix` | the M8N. `gps_fix` 0 is no fix. 3 is a 3D fix. |
| `battery_v` | pack voltage. |
| `link_fresh` | true only while core 1 has seen a live heartbeat inside 0.5 s. |

The bytes on the wire are a frame: a length, a message id, the payload,
a CRC. Lazygo's keys 15 and 16, with values squeezed into 50..250, are
not this protocol. They capped a lidar range at 200 cm and lost the
millimetres. That map is not being extended.

If the heartbeat stops, core 1 sets speed to zero and centres the steer.
It does not hold a heading and it does not continue a leg. `link_fresh`
goes false.

## Heading and position

Heading is the BNO085 gyro-integrated rotation vector. Mongla turns the
magnetometer off because thrusters and battery current pull it around.
This chassis has a VNH2SP30 and a 3S pack on the same plates. The
magnetometer stays on the chip and stays out of the heading until a
bench, with the motor running, shows it is stable.

Position outdoors is the M8N. Position between fixes is the encoder arc
times the heading, the same accumulation their odometry function does
(`ds`, average heading, `ds·cos`, `ds·sin`) and nothing else. They also
double-integrate linear acceleration into `positionXY` and then do not
send it. That second integrator is not in this contract.

The BNO055 is not on this board. Its fusion offsets cannot be cleared
cleanly, the Euler output is not the chip's axes past a modest tilt,
and its I2C misbehaves on ESP32. The 085 is the part.

## What the simulator has to match

`board_sim` speaks this contract and nothing else the planner needs.
Ticks come from the rear axle at 1873 per metre. The quaternion comes
from the simulated IMU. The fix comes from the simulated navsat. The
battery in sim is the nominal 11.1 V of a 3S pack, not a measured cell.
A silent command for 0.5 s is a dead link: speed zero, steer centred,
`link_fresh` false.

The real serial node, when the board exists, replaces `board_sim`.
The planner does not change. There is no PlatformIO tree in the repo
until that sim node and its headless test agree with this page.
