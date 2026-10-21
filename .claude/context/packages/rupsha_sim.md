# rupsha_sim — simulators, worlds, and the hardware chain

**State: BENCH** (SITL, Gazebo lockstep, Pi-in-the-loop). The full story:
[platform/sim-and-hitl.md](../platform/sim-and-hitl.md).

| Module | What it does |
|---|---|
| `sitl.py` | starts ArduRover SITL with the baseline (and the Gazebo overlay), waits for readiness, optional `--serial0` dial-out |
| `gazebo.py` | `GzServer`: headless `gz sim` with its own resource paths in a process group; `truth()` from `/model/rupsha/odometry` |
| `model_gen.py` | generates the `rupsha_skid` SDF from one table (wheels, track, IMU mounted FRD) |
| `worldgen.py` | seeded scenario → SDF worlds and deterministic ArUco textures |
| `contract.py` | the fixed sim contract: `/clock`, `/sim/<cam>/…`, `/sim/ground_truth/…` |
| `pil.py` | Pi-in-the-loop: SITL dialling the Pi plus a SIMSTATE truth recorder; `score` |

**Generated, never hand-edited:** `models/`, `worlds/`, `scenarios/`, `config/bridge.yaml`.

Launch: `ros2 launch rupsha_sim sim.launch.py world:=playpen headless:=true` (arguments
`manager`, `vision`, `localization`, `executive`).

Tests: `test_sim_contract.py` (generated files fresh and deterministic, tags decode to their id,
bridge carries the manifest topics, IMU FRD, wheel channels, cruise pair vs model) ·
`test_sitl_scenarios.py` (`-m sitl`) · `test_gazebo_lockstep.py` (`-m gazebo`).
