# rupsha_sensors — one table of sensors

**State: BUILT.** No camera has been plugged into the rover yet.

[`config/sensors.yaml`](../../../src/rupsha_sensors/config/sensors.yaml) lists every sensor once:
its driver, the manifest topic it publishes on, and the minimum rate below which it counts as
failing. Today: the front camera (`v4l2_camera`, 640×480 at 15 fps, ≥ 10 Hz), the board IMU
(relayed by the manager, ≥ 8 Hz) and the board GNSS (≥ 4 Hz). Calibration files go in
`config/calibration/`; a camera without one says `calibration: null`, and pose-dependent
consumers treat it as uncalibrated rather than guessing intrinsics.

`table.py` validates the table at load — topics must be `Topics` attributes, drivers must be
known (`v4l2_camera`, `camera_ros`, `board`), calibration files must exist — and turns each
entry into a node spec with remaps onto the manifest topics. `sensors.launch.py` starts them.

`ros2 run rupsha_sensors sensors_check` subscribes to every topic in the table, measures rates,
and prints PASS / FAIL / ABSENT per sensor. ABSENT (no message at all) is never PASS.

Tests: `test_sensors_table.py` — validation failures, remaps, and the verdict logic on synthetic
rates.
