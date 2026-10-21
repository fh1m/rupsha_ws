# rupsha_localization

**State: BUILT** — truth tests on known trajectories; not yet run against a simulator.

A shadow right-invariant EKF on SE(2) with ICR skid-steer odometry and slip gating. It
cross-checks the board's EKF3 and can only advise; the ExtNav uplink to EKF3 is opt-in and off.
The package's own page is the authority:
[`src/rupsha_localization/README.md`](../../../src/rupsha_localization/README.md).
