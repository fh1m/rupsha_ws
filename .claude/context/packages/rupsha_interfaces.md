# rupsha_interfaces — the messages

**State: BUILT.** Most files here are **generated** by `tools/gen_manifest.py` from
[`manifest/rupsha.yaml`](../../../manifest/rupsha.yaml). Edit the manifest, regenerate, never edit
the output; `test_manifest_outputs_are_fresh` fails CI otherwise.

| File | Generated | What |
|---|---|---|
| `action/Move.action` | yes | one action for every verb: verb name + the union of verb fields → `outcome`, `success`, `message`, `final_value` / `error_value` (NaN when the verb has none), `elapsed_s`; feedback carries phase and progress |
| `msg/RupshaState.msg` | yes | the board's state as the manager sees it; NaN for absent |
| `msg/AutonomyStatus.msg` | yes | executive state, display, leg index, distance to goal |
| `msg/Detection.msg`, `msg/Detections.msg` | no | vision output: class or tag id, bearing, range, pose, confidence |

The verb reference generated from the same manifest: [`reference/commands.md`](../reference/commands.md).
