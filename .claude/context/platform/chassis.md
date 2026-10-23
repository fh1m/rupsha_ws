# The chassis — Lazygo's body, measured from their repo, not copied

The body being built is Rakibul Islam's 2025 WRO Future Engineers chassis
(team LazyGo, robot LazyBot). At the 2025 international final in Singapore
that chassis placed 6th in Future Engineers, Bangladesh's best place at that
final, and a full score on the robot runs was a first for a team from
Bangladesh. The source that was read is
[A-N-M-Noor/LazyGo_WRO2025](https://github.com/A-N-M-Noor/LazyGo_WRO2025):
the README, the Onshape notes in `models/README.md`, `lazyBot.xacro`,
`control.xacro`, the ESP32 firmware, and `lazybridge.py`. None of that
code is in this repo. What that result used, and this stack does not, is the
lidar disparity extender, the lap counter, and parking on the ESP32. Those
were enough for three laps. A longer mission has to live on the Pi, which is
why the firmware contract stops at speed, steer, and the sensors. The chassis
itself is not here yet. Every length
below is their number, and the turn radius is still unmeasured on our copy.

## What is on it

Three plates on M3 standoffs, printed on a Bambu Lab P1S. The footprint is
20 cm by 10 cm.

- Bottom plate, 5 mm off the ground, 3 mm thick. Steering servo, 3S LiPo,
  the one drive motor, and a LEGO generation-3 differential.
- Middle plate, 36 mm above the bottom. Raspberry Pi 5, the 5 V 20 A buck,
  the VNH2SP30 motor driver underneath. Their LiDAR used to rest on this
  plate. Ours does not.
- Top, 30 mm above the middle. Their homemade through-hole PCB, and a cover
  15 mm above it that held the LiDAR upside down at 5 cm and a camera pan
  servo. We keep the cover and the camera. The LiDAR mount stays empty.

Wheelbase in their xacro is 0.12 m. The CAD note says 11.5 cm. Wheel radius
0.02 m. Rear track (`wheel_width` in the xacro) 0.094 m. Chassis box in the
xacro is 0.20 by 0.10 by 0.07 m.

## The drive

One 25GA370 encoder motor, not one motor per wheel. A 20-tooth printed gear
drives the differential at 1:1.2. The encoder is calibrated at **1873 ticks
per metre of travel** and reports the average of the two driven wheels.
Their speed loop on the ESP32 is a PI at 12.5 Hz (`kp` 0.04, `ki` 0.35,
80 ms, minimum PWM 18, 18 kHz motor PWM). That loop is not ours. The rate
is in [firmware-contract.md](firmware-contract.md).

Steering is one PS-1171MG servo. Pulse 950 µs (right) to 2250 µs (left),
centre 1600 µs. Their sim bridge caps the command at 30°. The xacro joint
limit is ±45°. A turn inside the 15 cm the CAD note claims, on a 0.12 m
wheelbase, wants about 39°. Those three numbers disagree. The Gazebo plant
uses the xacro: wheelbase 0.12 m, hinges limited to ±45°. A bicycle angle
of 45° would ask the inner hinge for about 59°, past that stop, so the
board caps the bicycle command near 36° and the inner hinge lands on the
limit. The real radius is unmeasured until this chassis is on the ground.

Front wheel hubs take a 625 bearing, inserted at a 7 mm pause in the print.
The camera is a Logitech C270 on a second PS-1171MG, pulse 500 to 2340 µs,
centre 1420 µs, angle about 19° to 161°.

## What we add, and what we leave off

- **GPS.** A u-blox M8N, which their robot does not have. It is the
  outdoor position fix. It is not the heading source.
- **IMU.** Their board is a BNO055 at I2C 0x29 in IMUPLUS, with offsets
  baked into the sketch. Ours is a BNO085. The reason is in the firmware
  contract: the 055's fusion cannot be turned off cleanly, and its I2C
  is a known problem on ESP32.
- **No LiDAR.** Their open-round controller is a disparity extender on an
  RPLidar C1 at 10 Hz. That is a wall follower for one WRO track. It is
  not the planner, and the sensor is not on this body.
- **No sonars.** `vehicle.yaml` in their repo still lists five sonars and
  four proximity sensors. The built robot does not carry them.

## Power, so the next board does not repeat their brownouts

3S LiPo, 12.6 V full, 11.1 V nominal, about 9.9 V empty. Two bucks,
because a servo inrush reset the ESP32 and throttled the Pi when they
shared a rail:

- 5.1 V from the 20 A module for the Pi, and for the ESP32.
- 5 V from an SCT2650 5 A module for the servos, the OLED, and the buzzer.
- The motor takes pack voltage through the VNH2SP30. The driver does not
  need the 5 V rail.

## The computers

The previous Rupsha body was a Pixhawk 2.4.8 and a Pi 4, with ArduRover
closing the rate loops. This body is not that.

- A custom ESP32 board is the reflexes: encoder, BNO085, M8N, the one
  motor, the steering servo. The contract is
  [firmware-contract.md](firmware-contract.md). The firmware is written
  here, not taken from their sketch.
- A Pi 5 with the Hailo-8 AI HAT is the thinker, the same split Mongla
  uses. The hat is not in the simulator. Vision in Gazebo is the camera
  through the existing detection message. A Hailo node is a later swap
  behind that message, once the hat is on a bench.

Their `vehicle.yaml` length and width are in centimetres of a different
sketch and are not the model.
