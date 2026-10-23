# rupsha_sim — simulators, worlds, and the hardware chain

**State: BENCH.** The default plant is Ackermann. SITL and the skid model are the previous
body. The full story: [platform/sim-and-hitl.md](../platform/sim-and-hitl.md).

| Module | What it does |
|---|---|
| `chassis.py` | the Ackermann table: wheelbase, track, 1873 ticks per metre, hinge angles, rear rates |
| `ackermann_gen.py` | generates the Ackermann URDF, `ackermann_ci.sdf`, the bridge and the controllers |
| `board_sim.py` | stands in for the ESP32. Speed and steer in, ticks, quaternion and GPS out |
| `sitl.py` | starts ArduRover SITL for the previous body |
| `gazebo.py` | `truth()` from `/model/rupsha/odometry` |
| `model_gen.py` | generates the previous `rupsha_skid` SDF |
| `worldgen.py` | seeded scenarios, and it calls the Ackermann generator |
| `contract.py` | `/clock`, the camera, ground truth, and the Ackermann bridge rows |
| `pil.py` | Pi-in-the-loop against the previous body |

**Generated, never hand-edited:** `models/`, `worlds/`, `scenarios/`, `config/bridge.yaml`,
`config/ackermann_bridge.yaml`, `config/ackermann_controllers.yaml`.

Launch: `ros2 launch rupsha_sim sim.launch.py headless:=true` for this chassis.
The ArduRover SITL launch is deleted.

Tests: `test_ackermann_math.py` · `test_gazebo_ackermann.py` (`-m gazebo`, no ArduRover) ·
`test_sim_contract.py` · `test_sitl_scenarios.py` (`-m sitl`) · `test_gazebo_lockstep.py`.
