# Authors

**Rupsha** is written and maintained by:

**Muhammad Fahim Faisal** — author.
Architecture, control integration, autonomy, perception, localization, the simulator, and this
repository. Also the author of [Mongla](https://github.com/fh1m/mongla_ws), whose fundamentals
this stack is built on.
<fh1m.dev@gmail.com> · [fh1m.github.io](https://fh1m.github.io/) · [@fh1m](https://github.com/fh1m)

## Ideas we learned from

- The [Missouri S&T Mars Rover Design Team](https://github.com/MissouriMRDT) —
  `Autonomy_Software`, `RoveComm_Base`, `DriveBoard_Software`, `Smart_MoCo`, `RoveSoSimulator`.
  The single manifest, first-class legs, the explicit autonomy state machine, the stuck
  ladder and verify-before-success are their ideas, re-implemented here in our own code; none
  of their code is vendored.

## Third-party work

- [ArduPilot](https://ardupilot.org) (GPLv3) — the firmware on the Pixhawk and in SITL; not
  vendored, pinned at tag `Rover-4.7.2`.
- [`ardupilot_gazebo`](https://github.com/ArduPilot/ardupilot_gazebo) (LGPLv3) — fetched by
  `rupsha.repos` into the gitignored `src/external/`; not vendored.
- Built on ROS 2, Gazebo, MAVLink/pymavlink, OpenCV and ONNX Runtime, each under its own licence.

Rupsha itself is MIT licensed — see [LICENSE](LICENSE).
