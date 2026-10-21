"""ONNX YOLO objects: letterbox, run, decode, map back. No model, no detector.

TWO OUTPUT LAYOUTS, TOLD APART BY SHAPE
---------------------------------------
classic   YOLOv8 / YOLO11   [1, 4 + nc, N]  cx, cy, w, h, per-class scores.
                                            Needs NMS (cv2.dnn.NMSBoxes, per
                                            class).
end2end   YOLO26 / YOLOv10  [1, N, 6]       x1, y1, x2, y2, score, class.
                                            NMS-free by construction; running
                                            NMS again would only cost time.
Both are in letterboxed INPUT pixels and are mapped back to the source image
by the exact inverse of the letterbox that produced them.

A MISSING MODEL IS AN ERROR, not an empty list. A detector that returns no
detections because its weights are absent is indistinguishable, downstream,
from one that is looking at an empty field -- and an object leg would then
search its full radius for a thing it could never have seen.

RANGE IS A COARSE ESTIMATE AND OFF BY DEFAULT. With a calibrated camera and a
configured physical height H for the class, range = fy * H / h_px. It assumes
the object is upright, unoccluded, fully in frame and that the box hugs it;
YOLO boxes are loose by a few pixels, which is ~10 % at 20 px tall. A box that
touches the top or bottom of the image is truncated, would over-range, and
gets NaN. Anything else without a height: NaN.
"""
from __future__ import annotations

import ast
import math
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import cv2
import numpy as np

from .camera_model import PinholeCamera

DEFAULT_INPUT_SIZE = 320
DEFAULT_CONF = 0.35
DEFAULT_IOU = 0.45
LETTERBOX_FILL = 114        # Ultralytics' pad value; the model was trained on it
EXPORT_HINT = 'yolo export model=yolo11n.pt format=onnx imgsz=320'
# TensorRT is left out of the default order on purpose: its first session
# builds an engine for minutes, and a node that hangs at start looks dead.
DEFAULT_PROVIDERS = ('CUDAExecutionProvider', 'CPUExecutionProvider')

_NAN = float('nan')


# --------------------------------------------------------------------------- #
#  Configuration parsing
# --------------------------------------------------------------------------- #
def parse_class_heights(spec: str) -> dict[str, float]:
    """'person:1.7,mallet:0.3' -> {'person': 1.7, 'mallet': 0.3}. '' -> {}."""
    out: dict[str, float] = {}
    for item in (s.strip() for s in (spec or '').split(',')):
        if not item:
            continue
        name, sep, h = item.rpartition(':')
        name = name.strip()
        try:
            height = float(h)
        except ValueError:
            height = _NAN
        if not sep or not name or not (math.isfinite(height) and height > 0.0):
            raise ValueError(f'class_heights_m entry {item!r} is not '
                             f'"name:height_m" with height > 0')
        out[name] = height
    return out


def load_class_names(path: str) -> list[str]:
    """Names from a .txt (one per line), or a .yaml holding a list, an
    {id: name} map, or an Ultralytics dataset file's `names:` key."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f'class names file not found: {path}')
    if path.endswith(('.yaml', '.yml')):
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f)
        if isinstance(data, Mapping) and 'names' in data:
            data = data['names']
        return _names_from(data, path)
    with open(path) as f:
        names = [ln.strip() for ln in f if ln.strip() and not ln.startswith('#')]
    if not names:
        raise ValueError(f'class names file {path} is empty')
    return names


def _names_from(data, where: str) -> list[str]:
    if isinstance(data, (list, tuple)):
        names = [str(x) for x in data]
    elif isinstance(data, Mapping):
        ids = sorted(int(k) for k in data)
        if ids != list(range(len(ids))):
            raise ValueError(f'class ids in {where} are not 0..n-1: {ids}')
        names = [str(data[k] if k in data else data[str(k)]) for k in ids]
    else:
        raise TypeError(f'cannot read class names from {where}: '
                        f'{type(data).__name__}')
    if not names:
        raise ValueError(f'no class names in {where}')
    return names


# --------------------------------------------------------------------------- #
#  Letterbox
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Letterbox:
    """The transform `letterbox()` applied. Box coordinates are pixel EDGES
    (a box spanning the whole image is [0, 0, w, h]), so the map is a pure
    scale and offset per axis."""
    sx: float        # input px per source px, x
    sy: float        # input px per source px, y
    pad_x: int       # left pad, input px
    pad_y: int       # top pad, input px
    src_w: int
    src_h: int
    size: int

    def to_source(self, xyxy: np.ndarray) -> np.ndarray:
        """(N, 4) input-pixel boxes -> source pixels, clipped to the image."""
        b = np.asarray(xyxy, dtype=np.float64).reshape(-1, 4).copy()
        b[:, [0, 2]] = ((b[:, [0, 2]] - self.pad_x) / self.sx).clip(0.0, self.src_w)
        b[:, [1, 3]] = ((b[:, [1, 3]] - self.pad_y) / self.sy).clip(0.0, self.src_h)
        return b

    def to_input(self, xyxy: np.ndarray) -> np.ndarray:
        b = np.asarray(xyxy, dtype=np.float64).reshape(-1, 4).copy()
        b[:, [0, 2]] = b[:, [0, 2]] * self.sx + self.pad_x
        b[:, [1, 3]] = b[:, [1, 3]] * self.sy + self.pad_y
        return b


def letterbox(image: np.ndarray, size: int = DEFAULT_INPUT_SIZE
              ) -> tuple[np.ndarray, Letterbox]:
    """Resize preserving aspect into a size x size canvas, centred.

    The inverse uses the per-axis scale the resize ACTUALLY applied (resized
    size / source size), not the nominal ratio: the rounded sizes differ from
    it by up to half a pixel, and a box mapped back with the nominal ratio
    inherits that error.
    """
    h, w = image.shape[:2]
    r = min(size / h, size / w)
    nw, nh = max(1, round(w * r)), max(1, round(h * r))
    resized = image if (nw, nh) == (w, h) else cv2.resize(
        image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas = np.full((size, size) + image.shape[2:], LETTERBOX_FILL, dtype=image.dtype)
    canvas[py:py + nh, px:px + nw] = resized
    return canvas, Letterbox(nw / w, nh / h, px, py, w, h, int(size))


# --------------------------------------------------------------------------- #
#  Decoders
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class RawDetections:
    xyxy: np.ndarray     # (M, 4) input px
    score: np.ndarray    # (M,)
    cls: np.ndarray      # (M,) int

    @staticmethod
    def empty() -> RawDetections:
        return RawDetections(np.zeros((0, 4)), np.zeros(0), np.zeros(0, int))


def decode_classic(out: np.ndarray, conf: float = DEFAULT_CONF,
                   iou: float = DEFAULT_IOU) -> RawDetections:
    """[1, 4 + nc, N] -> boxes after per-class NMS."""
    a = np.asarray(out, dtype=np.float32)
    if a.ndim != 3 or a.shape[0] != 1 or a.shape[1] < 5:
        raise ValueError(f'classic YOLO output must be [1, 4+nc, N], got {a.shape}')
    p = a[0].T                                   # (N, 4 + nc)
    scores_all = p[:, 4:]
    cls = scores_all.argmax(axis=1)
    score = scores_all[np.arange(len(p)), cls]
    keep = score >= conf
    if not keep.any():
        return RawDetections.empty()
    p, cls, score = p[keep], cls[keep], score[keep]
    cx, cy, w, h = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
    xyxy = np.column_stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])
    kept: list[int] = []
    for c in np.unique(cls):
        idx = np.nonzero(cls == c)[0]
        boxes = [[float(x1), float(y1), float(x2 - x1), float(y2 - y1)]
                 for x1, y1, x2, y2 in xyxy[idx]]
        sel = cv2.dnn.NMSBoxes(boxes, score[idx].astype(float).tolist(),
                               float(conf), float(iou))
        kept.extend(int(idx[i]) for i in np.asarray(sel).reshape(-1))
    kept.sort(key=lambda i: -float(score[i]))
    k = np.array(kept, dtype=int)
    return RawDetections(xyxy[k].astype(np.float64), score[k].astype(np.float64),
                         cls[k].astype(int))


def decode_end2end(out: np.ndarray, conf: float = DEFAULT_CONF) -> RawDetections:
    """[1, N, 6] (x1, y1, x2, y2, score, cls) -> boxes above `conf`. No NMS."""
    a = np.asarray(out, dtype=np.float32)
    if a.ndim != 3 or a.shape[0] != 1 or a.shape[2] != 6:
        raise ValueError(f'end-to-end YOLO output must be [1, N, 6], got {a.shape}')
    p = a[0]
    p = p[p[:, 4] >= conf]
    p = p[np.argsort(-p[:, 4])]
    return RawDetections(p[:, :4].astype(np.float64), p[:, 4].astype(np.float64),
                         np.rint(p[:, 5]).astype(int))


def output_layout(shape: Sequence[int], num_classes: int) -> str:
    """'classic' or 'end2end' for an output of `shape`; refuses if neither or both."""
    if len(shape) != 3 or shape[0] != 1:
        raise ValueError(f'YOLO output must be rank 3 with batch 1, got {tuple(shape)}')
    classic = shape[1] == 4 + num_classes
    e2e = shape[2] == 6
    if classic and e2e:
        raise ValueError(f'output {tuple(shape)} with {num_classes} classes fits '
                         f'both layouts; cannot tell them apart')
    if classic:
        return 'classic'
    if e2e:
        return 'end2end'
    raise ValueError(f'output {tuple(shape)} is neither [1, 4+{num_classes}, N] '
                     f'(YOLOv8/11) nor [1, N, 6] (YOLO26/v10 end-to-end)')


def decode(out: np.ndarray, num_classes: int, conf: float = DEFAULT_CONF,
           iou: float = DEFAULT_IOU) -> RawDetections:
    if output_layout(np.shape(out), num_classes) == 'classic':
        return decode_classic(out, conf, iou)
    return decode_end2end(out, conf)


# --------------------------------------------------------------------------- #
#  Detector
# --------------------------------------------------------------------------- #
@dataclass
class ObjectObservation:
    label: str
    class_id: int
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float
    u: float
    v: float
    width_px: float
    height_px: float
    bearing_deg: float = _NAN
    range_m: float = _NAN        # coarse pinhole estimate; see module doc


def pinhole_range_m(camera: PinholeCamera, height_m: float, y1: float, y2: float
                    ) -> float:
    h_px = y2 - y1
    if not camera.calibrated or not (h_px > 0.0) or not (height_m > 0.0):
        return _NAN
    # Truncated by the frame: the visible height is a lower bound, so the
    # range would be an over-estimate with no way to tell by how much.
    if y1 <= 0.5 or y2 >= camera.height - 0.5:
        return _NAN
    return float(camera.fy * height_m / h_px)


class ObjectDetector:

    def __init__(self, model_path: str, class_names: Sequence[str] | None = None,
                 class_names_path: str = '', input_size: int = DEFAULT_INPUT_SIZE,
                 conf_threshold: float = DEFAULT_CONF,
                 iou_threshold: float = DEFAULT_IOU,
                 class_heights_m: Mapping[str, float] | None = None,
                 providers: Sequence[str] = DEFAULT_PROVIDERS) -> None:
        if not model_path or not os.path.isfile(model_path):
            raise FileNotFoundError(
                f'ONNX model not found: {model_path!r}. Export one with '
                f'`{EXPORT_HINT}` (or yolo26n.pt) and set model_path to the '
                f'.onnx file; set model_path to \'\' to run without objects.')
        if not (0.0 < conf_threshold < 1.0):
            raise ValueError(f'conf_threshold must be in (0, 1), got {conf_threshold}')
        try:
            import onnxruntime as ort
        except ImportError as e:
            raise ImportError('onnxruntime is required for object detection '
                              '(pip install onnxruntime, or onnxruntime-gpu)') from e

        available = set(ort.get_available_providers())
        chosen = [p for p in providers if p in available]
        if 'CPUExecutionProvider' in available and 'CPUExecutionProvider' not in chosen:
            chosen.append('CPUExecutionProvider')
        if not chosen:
            raise RuntimeError(f'none of {list(providers)} is available in this '
                               f'onnxruntime ({sorted(available)})')
        self.session = ort.InferenceSession(model_path, providers=chosen)
        self.providers = self.session.get_providers()
        self.model_path = model_path

        inp = self.session.get_inputs()[0]
        self._input_name = inp.name
        shape = list(inp.shape)
        static = [d for d in shape[2:] if isinstance(d, int)]
        if static and any(d != input_size for d in static):
            raise ValueError(f'{model_path} was exported for input {shape[2:]}, '
                             f'but input_size is {input_size}; re-export with '
                             f'imgsz={input_size} or change input_size')
        self.input_size = int(input_size)

        if class_names is not None:
            names = list(class_names)
        elif class_names_path:
            names = load_class_names(class_names_path)
        else:
            names = self._names_from_metadata()
        if not names:
            raise ValueError(f'no class names for {model_path}: the model carries '
                             f'none in its metadata and no class_names_path was given')
        self.class_names = names

        out_shape = self.session.get_outputs()[0].shape
        if all(isinstance(d, int) for d in out_shape):
            self.layout = output_layout(out_shape, len(names))
        else:
            self.layout = ''          # dynamic: decided on the first output
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.class_heights_m = dict(class_heights_m or {})
        unknown = sorted(set(self.class_heights_m) - set(names))
        if unknown:
            raise ValueError(f'class_heights_m names {unknown} are not classes of '
                             f'{model_path} ({names})')

    def _names_from_metadata(self) -> list[str]:
        meta = self.session.get_modelmeta().custom_metadata_map or {}
        raw = meta.get('names')
        if not raw:
            return []
        return _names_from(ast.literal_eval(raw), f'{self.model_path} metadata')

    def preprocess(self, bgr: np.ndarray) -> tuple[np.ndarray, Letterbox]:
        if bgr.ndim == 2:
            bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)
        img, lb = letterbox(bgr, self.input_size)
        x = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        return np.ascontiguousarray(x.transpose(2, 0, 1)[None]), lb

    def detect(self, bgr: np.ndarray, camera: PinholeCamera) -> list[ObjectObservation]:
        x, lb = self.preprocess(bgr)
        out = self.session.run(None, {self._input_name: x})[0]
        if not self.layout:
            self.layout = output_layout(out.shape, len(self.class_names))
        if self.layout == 'classic':
            raw = decode_classic(out, self.conf_threshold, self.iou_threshold)
        else:
            raw = decode_end2end(out, self.conf_threshold)
        return self.observations(raw, lb, camera)

    def observations(self, raw: RawDetections, lb: Letterbox,
                     camera: PinholeCamera) -> list[ObjectObservation]:
        boxes = lb.to_source(raw.xyxy)
        obs: list[ObjectObservation] = []
        for (x1, y1, x2, y2), s, c in zip(boxes, raw.score, raw.cls, strict=True):
            if not (0 <= c < len(self.class_names)):
                # An id the names list does not cover is a model / names
                # mismatch; labelling it would be inventing a class.
                raise ValueError(f'class id {c} outside the {len(self.class_names)} '
                                 f'names configured for {self.model_path}')
            if x2 - x1 <= 0.0 or y2 - y1 <= 0.0:
                continue
            label = self.class_names[c]
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            rng = (pinhole_range_m(camera, self.class_heights_m[label], y1, y2)
                   if label in self.class_heights_m else _NAN)
            obs.append(ObjectObservation(label, int(c), float(s), float(x1),
                                         float(y1), float(x2), float(y2), u, v,
                                         float(x2 - x1), float(y2 - y1),
                                         camera.bearing_deg(u, v), rng))
        return obs
