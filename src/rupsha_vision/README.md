# rupsha_vision -- the front camera, read honestly

One node, `vision`. It takes `/rupsha/camera/front/image_raw` (+ `camera_info`),
runs whichever detectors the executive's current state calls for, and publishes
`rupsha_interfaces/Detections` on `/rupsha/detections` with the IMAGE's capture
stamp.

    image_raw --(best effort, depth 1)--> mailbox --worker--> [aruco] [objects] --> /rupsha/detections
    camera_info ---------------------------------> PinholeCamera (calibrated or not)
    /rupsha/autonomy_status (AutonomyState) -----> scheduler: which detectors run

## Rules this package keeps

- **NaN means absent, never 0.** No calibration: `range_m` is NaN. No fx: `bearing_deg`
  is NaN. A decoded tag is `label "tag"` with `confidence` 1.0, as `Detection.msg`
  defines; the consumers select tags by that label. A Detection without a metric
  solve has a NaN `pose`.
- **Pixel-native first.** `u, v, width_px, height_px` and `tag_id` need nothing but the
  image and are always filled. Bearing needs fx; range needs a calibration AND a known
  physical size.
- **Refuse loudly.** A configured `model_path` that does not exist stops the node with
  the export command in the error. An empty `model_path` is allowed and logged as
  `OBJECT DETECTION DISABLED` at start, and again the first time each phase asks for
  objects. A fisheye `camera_info`, an unknown dictionary, an AutonomyState with no
  vision phase, or a model / class-names mismatch all raise. None of them is guessed.
- **Newest frame only.** A one-slot mailbox. A slow detector costs dropped frames
  (counted and logged every 10 s), never a backlog.
- **Nothing scheduled, nothing published.** An empty Detections means "looked, saw
  nothing", so the node says nothing when it did not look.

## Capabilities and their honest state

`BUILT` = written and passing synthetic truth tests on the dev box. `BENCH` = run
against a real camera on the bench. `FIELD` = run on the rover outdoors. `BLOCKED` = cannot
run yet, and why.

| Capability | State |
|---|---|
| Newest-frame mailbox, drop count, stale age | BUILT |
| Pinhole model from CameraInfo / K,D / FOV; bearing through distortion | BUILT |
| ArUco 4x4_50 id, centre, bbox, bearing (Aruco3, min side 16 px) | BUILT |
| ArUco metric pose (IPPE square, both solutions, ambiguity ratio) | BUILT |
| Per-tag flip disambiguation from the previous normal | BUILT |
| ONNX YOLO, classic `[1,4+nc,N]` + NMS and end-to-end `[1,N,6]` | BUILT (synthetic tensors and a constant ONNX graph; no real model run) |
| Coarse object range `fy*H/h_px` for configured class heights | BUILT |
| Mission-phase scheduling from AutonomyState | BUILT |
| `vision` node | BLOCKED: see below |

Nothing here is BENCH or FIELD yet.

**BLOCKED.** The node cannot start in this workspace yet, for two reasons outside this package:
1. `rupsha_interfaces` is not built, so `Detection`/`Detections`/`AutonomyStatus` do
   not import.
2. The generated `rupsha_control/manifest.py` does not import (`SyntaxError: cannot
   assign to False`). PyYAML 1.1 reads `Display: {OFF: 0}` in `manifest/rupsha.yaml` as
   `{False: 0}`, and the generator then writes `False = 0`. Quote the key (`'OFF'`) or make
   `tools/gen_manifest.py` stringify the keys, then regenerate.

`test/test_vision_node.py` skips until both import. The node logic was smoke-tested
against stand-in message classes and a fixed copy of the manifest. That covered the stamp
copy, NaN range without camera_info, the scheduler driving publication, refusal of a bad
state ordinal, and drop counting.

## What the truth tests measured

They render a 0.20 m tag through a 640x480, 70 deg pinhole at a pose chosen in the test
(`rupsha_vision/synthetic.py`: `projectPoints` + `warpPerspective`, 16x supersampled).

| | Tolerance | Measured worst (2 m and 5 m, up to 35 deg yaw, noise 0 or 3 grey levels) |
|---|---|---|
| centre | 1 px | 0.11 px |
| bearing | 0.5 deg | 0.013 deg |
| range | 3 % | 0.6 % |

On a k1 = -0.25 lens, 1 m off-axis: bearing within 0.05 deg and range within 0.3 %. Reading
the same image as an ideal pinhole is off by 1.5-2.3 deg.

**Corner refinement is done here, not by cv2.** With `useAruco3Detection`, cv2 4.13's
`CORNER_REFINE_SUBPIX` was measured to leave the corners where `CORNER_REFINE_NONE` puts
them: 0.7-0.85 px inside the edge, which reads +1.5 % range at 2 m. `cornerSubPix`
afterwards halves that and stays biased. The corners are re-derived from edge-line fits
(see `aruco.py`).

**Limits measured, not fixed:**
- A 0.20 m tag at 5 m and 45 deg yaw (about 13 px foreshortened) is not detected.
- At 5 m with heavy pixel noise (sigma 12), the raw best IPPE solution is the mirror pose in
  11 of 60 frames, and the tracker brings that to 4. It cannot reach 0. When noise makes
  the mirror clearly better (ambiguity below 0.6), the measurement wins over the memory,
  by design.

## Parameters (`config/vision.yaml`)

`enabled` (the only one that can change at runtime), `dictionary`, `marker_size_m`,
`min_side_px` (12..16), `ambiguity_threshold`, `pose_memory_s`, `model_path` (`''` =
objects off), `class_names_path` (`''` = from the ONNX metadata), `input_size`,
`conf_threshold`, `iou_threshold`, `class_heights_m` (`'name:height_m,...'`),
`fallback_hfov_deg` (coarse bearings without calibration; range stays NaN),
`phase_without_status` (what runs before the executive speaks), `image_topic`,
`camera_info_topic`.

    ros2 launch rupsha_vision vision.launch.py model_path:=/path/yolo11n.onnx
    yolo export model=yolo11n.pt format=onnx imgsz=320     # or yolo26n.pt (NMS-free)

## Compute upgrade path

1. **Now: Pi 4 CPU.** ArUco is about 3 ms per 640x480 frame on the dev box, so expect several
   times that on a Pi 4. YOLO11n/YOLO26n runs at 320 input through ONNX Runtime on the CPU
   (ncnn is the other candidate on the Pi). The scheduler exists so both never have to run
   on every frame.
2. **OAK-D.** The network runs on the camera's Myriad X and only boxes come over USB3. The Pi
   keeps ArUco and the pose solve. The decoders here stay valid. Only `ObjectDetector`'s
   session changes.
3. **Jetson Orin.** TensorRT (`TensorrtExecutionProvider` is available but deliberately
   not in the default provider list, because the first engine build takes minutes).

Not options: **Hailo** needs PCIe, and a Pi 4 has none (that is a Pi 5 / CM4 path).
**Coral** Edge TPU is end-of-life.

## Tests

    cd ~/tmp/rupsha_ws && python3 -m pytest src/rupsha_vision -q

ROS-free except `test_vision_node.py`, which skips without rclpy, cv_bridge, a built
`rupsha_interfaces` and an importable manifest.
