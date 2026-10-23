# The book — every context document, in reading order

Start at [`CLAUDE.md`](../../CLAUDE.md). Then read in this order; each document assumes the ones
above it. `test_doc_drift.py` fails if a document exists here and is not listed, or a link breaks.

## 1. Why it is built this way

- [the-shift.md](the-shift.md) — reflexes on the board, thinking on the Pi
- [sota/README.md](sota/README.md) — the research behind each choice, with sources

## 2. Where it stands

- [HANDOFF.md](HANDOFF.md) — start here if you were told to continue the work
- [ROADMAP.md](ROADMAP.md) — the one status file
- [capability-map.md](capability-map.md) — every capability, its evidence, its state
- [measured-bars.md](measured-bars.md) — every number, and what measured it
- [BUGS.md](BUGS.md) — the single defect register

## 3. The platform

- [platform/chassis.md](platform/chassis.md) — the Ackermann body being built, from Lazygo's measurements
- [platform/firmware-contract.md](platform/firmware-contract.md) — what the ESP32 will do, and what it will not
- [platform/ardurover-pixhawk.md](platform/ardurover-pixhawk.md) — the previous board, its record, its traps
- [platform/sim-and-hitl.md](platform/sim-and-hitl.md) — SITL, Gazebo, board-, Pi-in-the-loop, Sim-on-Hardware
- [platform/box-and-pi.md](platform/box-and-pi.md) — the dev box and the Pi
- [platform/records/](platform/records/) — firmware identities and parameter dumps, as captured

## 4. The packages

- [packages/rupsha_interfaces.md](packages/rupsha_interfaces.md)
- [packages/rupsha_control.md](packages/rupsha_control.md)
- [packages/rupsha_planner.md](packages/rupsha_planner.md)
- [packages/rupsha_sensors.md](packages/rupsha_sensors.md)
- [packages/rupsha_vision.md](packages/rupsha_vision.md)
- [packages/rupsha_localization.md](packages/rupsha_localization.md)
- [packages/rupsha_sim.md](packages/rupsha_sim.md)

## 5. Reference

- [reference/commands.md](reference/commands.md) — every verb and its fields (generated from the manifest)
