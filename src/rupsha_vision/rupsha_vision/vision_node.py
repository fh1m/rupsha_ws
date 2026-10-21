"""`vision`: front camera -> scheduled detectors -> /rupsha/detections.

THREADING. The image callback does one thing: drop the message in the
mailbox. A worker thread takes the newest frame, runs whatever the mission
phase schedules, and publishes. The executor is never blocked by a detector,
so camera_info and autonomy_status keep arriving while YOLO runs, and a slow
frame costs drops (counted) rather than a backlog (silent lag).

STAMPS. `Detections.header.stamp` is the IMAGE's capture stamp, copied, never
`now()`: every consumer ages a detection by it, and a publish-time stamp makes
a 200 ms-old observation look fresh. `latency_ms` is capture -> publish on
this node's clock, NaN when the stamp is not in that clock's domain.

NOTHING SCHEDULED, NOTHING PUBLISHED. An empty Detections means "looked and
saw nothing". When the phase runs no detectors, nothing was looked for, and
publishing an empty list would tell the executive the field is clear.

CAMERA MODEL. From camera_info when it has a K; until then, and whenever K is
all zeros, the camera is uncalibrated: pixels and ids are published, bearing
is NaN (or coarse, from `fallback_hfov_deg` if set) and range is NaN.

rclpy, cv_bridge, rupsha_interfaces and the manifest are imported inside this
module only; every core it drives is ROS-free.
"""
from __future__ import annotations

import threading
import time

import cv2

from . import scheduler
from .aruco import ArucoTagDetector, TagObservation, rvec_to_quaternion_xyzw
from .camera_model import PinholeCamera
from .detector import ObjectDetector, ObjectObservation, parse_class_heights
from .latency import LatencyWindow, e2e_ms
from .mailbox import FrameMailbox

_NAN = float('nan')
TAG_LABEL = 'tag'
_STATS_PERIOD_S = 10.0


def _manifest():
    try:
        from rupsha_control.manifest import AutonomyState, Topics
    except Exception as e:      # SyntaxError from a bad generation is the known case
        raise RuntimeError(
            f'rupsha_control.manifest does not import ({type(e).__name__}: {e}). '
            f'Regenerate it with tools/gen_manifest.py from manifest/rupsha.yaml; '
            f'vision will not guess topic names or AutonomyState ordinals.') from e
    return AutonomyState, Topics


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _fill_pose_absent(pose) -> None:
    for obj, keys in ((pose.position, 'xyz'), (pose.orientation, 'xyzw')):
        for k in keys:
            setattr(obj, k, _NAN)


def tag_to_msg(Detection, t: TagObservation):
    d = Detection()
    d.source = scheduler.ARUCO
    # Detection.msg: label "tag", confidence 1.0 for a decoded tag. The executive,
    # the facade and the DSL select tags by this label.
    d.label = TAG_LABEL
    d.tag_id = int(t.tag_id)
    d.confidence = 1.0
    d.u, d.v = float(t.u), float(t.v)
    d.width_px, d.height_px = float(t.width_px), float(t.height_px)
    d.bearing_deg = float(t.bearing_deg)
    d.range_m = float(t.range_m)
    d.pose_ambiguity = float(t.pose_ambiguity)
    if t.has_pose:
        p = d.pose
        p.position.x, p.position.y, p.position.z = (float(x) for x in t.pose.tvec)
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = rvec_to_quaternion_xyzw(t.pose.rvec)
    else:
        _fill_pose_absent(d.pose)
    return d


def object_to_msg(Detection, o: ObjectObservation):
    d = Detection()
    d.source = scheduler.OBJECTS
    d.label = o.label
    d.tag_id = -1
    d.confidence = float(o.confidence)
    d.u, d.v = float(o.u), float(o.v)
    d.width_px, d.height_px = float(o.width_px), float(o.height_px)
    d.bearing_deg = float(o.bearing_deg)
    d.range_m = float(o.range_m)
    # A box range has no orientation and no lateral solve worth a Pose.
    d.pose_ambiguity = _NAN
    _fill_pose_absent(d.pose)
    return d


def _make_node_class():
    from cv_bridge import CvBridge
    from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    from rupsha_interfaces.msg import AutonomyStatus, Detection, Detections

    AutonomyState, Topics = _manifest()

    # BEST_EFFORT/VOLATILE subscribers are compatible with every publisher
    # QoS; a RELIABLE or TRANSIENT_LOCAL request against a best-effort or
    # volatile publisher connects to nothing and says so only once, at start.
    mailbox_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
    detections_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)

    class VisionNode(Node):

        def __init__(self) -> None:
            super().__init__('vision')
            ro = ParameterDescriptor(read_only=True)
            p = self.declare_parameter
            self._enabled = bool(p('enabled', True).value)
            dictionary = p('dictionary', 'DICT_4X4_50', ro).value
            marker_size_m = float(p('marker_size_m', 0.20, ro).value)
            min_side_px = int(p('min_side_px', 16, ro).value)
            ambiguity = float(p('ambiguity_threshold', 0.6, ro).value)
            memory_s = float(p('pose_memory_s', 1.0, ro).value)
            model_path = str(p('model_path', '', ro).value)
            class_names_path = str(p('class_names_path', '', ro).value)
            input_size = int(p('input_size', 320, ro).value)
            conf = float(p('conf_threshold', 0.35, ro).value)
            iou = float(p('iou_threshold', 0.45, ro).value)
            heights = parse_class_heights(str(p('class_heights_m', '', ro).value))
            self._fallback_hfov = float(p('fallback_hfov_deg', 0.0, ro).value)
            self._phase_without_status = str(p('phase_without_status', 'search', ro).value)
            image_topic = str(p('image_topic', Topics.CAMERA_FRONT, ro).value)
            info_topic = str(p('camera_info_topic', Topics.CAMERA_FRONT_INFO, ro).value)
            self.add_on_set_parameters_callback(self._on_params)

            scheduler.check_tables([s.name for s in AutonomyState])
            scheduler.detectors_for_phase(self._phase_without_status)
            self._AutonomyState = AutonomyState

            self._aruco = ArucoTagDetector(dictionary, marker_size_m, min_side_px,
                                           ambiguity, memory_s)
            if self._aruco.unsupported_params:
                self.get_logger().warning(
                    f'cv2 {cv2.__version__} ArUco parameters lack '
                    f'{list(self._aruco.unsupported_params)}; running without them')
            self._objects: ObjectDetector | None = None
            if model_path:
                # A configured path that does not load is fatal: the operator
                # asked for objects and must not get a node that silently
                # sees none.
                self._objects = ObjectDetector(
                    model_path, class_names_path=class_names_path,
                    input_size=input_size, conf_threshold=conf,
                    iou_threshold=iou, class_heights_m=heights)
                self.get_logger().info(
                    f'objects: {model_path} ({self._objects.layout or "dynamic"} '
                    f'output, {len(self._objects.class_names)} classes, '
                    f'{self._objects.providers[0]})')
            else:
                self.get_logger().warning(
                    'OBJECT DETECTION DISABLED: model_path is empty. Phases that '
                    'schedule objects will run ArUco only. Export a model with '
                    '`yolo export model=yolo11n.pt format=onnx imgsz=320`.')
            self._available = frozenset({scheduler.ARUCO} |
                                        ({scheduler.OBJECTS} if self._objects else set()))

            self._state_name: str | None = None
            self._run = frozenset()
            self._warned_missing = set()
            self._set_schedule(None)

            self._bridge = CvBridge()
            self._Detection, self._Detections = Detection, Detections
            self._camera_info: PinholeCamera | None = None
            self._warned_uncalibrated = False
            self._mailbox = FrameMailbox()
            self._compute = LatencyWindow()
            self._e2e = LatencyWindow()
            self._processed = 0
            self._last_stats = time.monotonic()

            self._pub = self.create_publisher(Detections, Topics.DETECTIONS,
                                              detections_qos)
            self.create_subscription(Image, image_topic, self._on_image, mailbox_qos)
            self.create_subscription(CameraInfo, info_topic, self._on_info, mailbox_qos)
            self.create_subscription(AutonomyStatus, Topics.AUTONOMY_STATUS,
                                     self._on_status, mailbox_qos)

            self._stop = threading.Event()
            self._worker = threading.Thread(target=self._loop, name='vision-worker',
                                            daemon=True)
            self._worker.start()
            self.get_logger().info(
                f'vision up: {image_topic} -> {Topics.DETECTIONS}, {dictionary} '
                f'{marker_size_m:.3f} m, min_side {min_side_px} px, cv2 {cv2.__version__} '
                f'{"legacy detectMarkers" if self._aruco.legacy else "ArucoDetector"}')

        # ------------------------------------------------------------ #
        def _on_params(self, params):
            for prm in params:
                if prm.name == 'enabled':
                    self._enabled = bool(prm.value)
                    self.get_logger().info(f'enabled = {self._enabled}')
            return SetParametersResult(successful=True)

        def _set_schedule(self, state_name: str | None) -> None:
            if state_name is None:
                phase = self._phase_without_status
                wanted = scheduler.detectors_for_phase(phase)
                run, missing = wanted & self._available, wanted - self._available
                label = f'no autonomy_status yet -> phase {phase!r}'
            else:
                phase = scheduler.phase_for_state(state_name)
                run, missing = scheduler.schedule(state_name, self._available)
                label = f'{state_name} -> phase {phase!r}'
            if run != self._run or state_name != self._state_name:
                self.get_logger().info(f'{label}: running {sorted(run) or "nothing"}')
            # Once per (phase, detector): the warning matters most the first time
            # an object leg reaches its approach, not only at start-up.
            for m in sorted(missing):
                if (phase, m) not in self._warned_missing:
                    self._warned_missing.add((phase, m))
                    self.get_logger().warning(f'{label} wants {m!r}, which is not '
                                              f'available on this node')
            self._state_name, self._run = state_name, run

        def _on_status(self, msg) -> None:
            try:
                name = self._AutonomyState(int(msg.state)).name
                if name != self._state_name:
                    self._set_schedule(name)
            except (ValueError, KeyError) as e:
                self.get_logger().error(
                    f'autonomy_status state {msg.state}: {e}; keeping the '
                    f'previous schedule', throttle_duration_sec=5.0)

        def _on_info(self, msg) -> None:
            try:
                cam = PinholeCamera.from_camera_info(msg)
            except ValueError as e:
                self.get_logger().error(f'camera_info refused: {e}',
                                        throttle_duration_sec=10.0)
                return
            prev = self._camera_info
            self._camera_info = cam      # one reference swap; the worker reads it whole
            if cam.calibrated and (prev is None or not prev.calibrated):
                self.get_logger().info(
                    f'camera_info: fx {cam.fx:.1f} fy {cam.fy:.1f} '
                    f'{cam.width}x{cam.height}, metric range enabled')

        def _on_image(self, msg) -> None:
            self._mailbox.put(msg, msg.header.stamp)

        # ------------------------------------------------------------ #
        def _camera_for(self, w: int, h: int) -> PinholeCamera:
            cam = self._camera_info
            if cam is not None and cam.calibrated:
                if abs(cam.width * h - cam.height * w) <= max(w, h):
                    return cam.scaled_to(w, h)
                self.get_logger().error(
                    f'camera_info is {cam.width}x{cam.height} but images are '
                    f'{w}x{h} (different aspect); treating as uncalibrated',
                    throttle_duration_sec=10.0)
            if not self._warned_uncalibrated:
                self._warned_uncalibrated = True
                self.get_logger().warning(
                    'no calibrated camera_info: range_m is NaN'
                    + (f', bearings from a nominal {self._fallback_hfov:.1f} deg FOV'
                       if self._fallback_hfov > 0 else ', bearings NaN'))
            if self._fallback_hfov > 0.0:
                return PinholeCamera.from_fov(w, h, self._fallback_hfov, calibrated=False)
            return PinholeCamera.uncalibrated(w, h)

        def _loop(self) -> None:
            while not self._stop.is_set():
                item = self._mailbox.get(timeout=0.5)
                if item is None:
                    continue
                try:
                    self._process(item[0])
                # One bad frame must not kill the only thread that publishes.
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error(f'frame failed: {type(e).__name__}: {e}',
                                            throttle_duration_sec=2.0)
                self._maybe_stats()

        def _process(self, msg) -> None:
            run = self._run
            if not self._enabled or not run:
                return
            t0 = time.perf_counter()
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = img.shape[:2]
            cam = self._camera_for(w, h)
            stamp_s = _stamp_s(msg.header.stamp)
            dets: list = []
            if scheduler.ARUCO in run:
                dets += [tag_to_msg(self._Detection, t)
                         for t in self._aruco.detect(img, cam, stamp_s)]
            if scheduler.OBJECTS in run and self._objects is not None:
                dets += [object_to_msg(self._Detection, o)
                         for o in self._objects.detect(img, cam)]
            self._compute.add((time.perf_counter() - t0) * 1e3)

            out = self._Detections()
            out.header = msg.header
            out.image_width, out.image_height = int(w), int(h)
            out.detections = dets
            now_s = self.get_clock().now().nanoseconds * 1e-9
            out.latency_ms = float(e2e_ms(stamp_s, now_s))
            self._e2e.add(out.latency_ms)
            self._pub.publish(out)
            self._processed += 1

        def _maybe_stats(self) -> None:
            now = time.monotonic()
            if now - self._last_stats < _STATS_PERIOD_S:
                return
            self._last_stats = now
            c, e = self._compute.summary(), self._e2e.summary()
            mb = self._mailbox
            self.get_logger().info(
                f'frames in {mb.puts} processed {self._processed} dropped {mb.drops}; '
                f'compute mean {c["mean"]:.1f} p95 {c["p95"]:.1f} ms; '
                f'capture->publish mean {e["mean"]:.1f} p95 {e["p95"]:.1f} ms '
                f'({self._e2e.nan_count} unstampable); run {sorted(self._run)}')

        def shutdown(self) -> None:
            self._stop.set()
            self._mailbox.close()
            self._worker.join(timeout=2.0)

    return VisionNode


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = _make_node_class()()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
