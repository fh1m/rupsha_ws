"""ArUco posts: id, pixels, bearing -- and a metric pose only when it is earned.

WHAT COMES OUT, IN ORDER OF HOW MUCH IT CAN BE TRUSTED
------------------------------------------------------
1. `tag_id`, the corners, the centre `u, v` and the bbox size. Pixel-native;
   need nothing but the image.
2. `bearing_deg`. Needs fx (a calibration, or a nominal FOV).
3. `range_m` and the pose. Needs a CALIBRATED camera AND the printed marker
   size. Otherwise NaN -- never a guess dressed as a measurement.

THE FLIP. A planar square seen from far away has two poses that reproject
almost equally well: the true one and its mirror about the line of sight
(IPPE returns both). Range is nearly identical between them; the marker
NORMAL is not, and a consumer that steers to face the post would oscillate
between two headings. `pose_ambiguity` = err_best / err_second says how close
the call was (1.0 = indistinguishable). Above `ambiguity_threshold` the
`FlipTracker` picks the solution whose normal is closest to the last accepted
one for that id, so the answer is at least temporally consistent; below it the
lower-error solution wins outright and re-seeds the tracker.

DETECTOR SETTINGS. `useAruco3Detection` with `minSideLengthCanonicalImg` 16
(12..16 allowed): Aruco3's speed-up is a search over a canonical scale, and
the canonical side bounds the smallest marker it can still decode. The URC
post's 0.20 m tag is ~18 px across at 5 m on a 640 px, 70 deg camera, so the
floor matters.

CORNERS. Sub-pixel corners are what make 5 m range usable at all -- a 0.2 px
corner error on an 18 px side is already 1 % of range. `CORNER_REFINE_SUBPIX`
is set, but with `useAruco3Detection` on cv2 4.13 it was measured to leave the
corners exactly where CORNER_REFINE_NONE puts them: 0.7-0.85 px inside the true
edge, +1.5 % range at 2 m. `cv2.cornerSubPix` run afterwards halves that but
stays biased on an L-shaped corner. So the corners are re-derived from the four
EDGES: the 50 % crossing of the white-to-black profile is found at several
points along each edge, a line is fitted through them, and adjacent lines are
intersected. Measured on rendered tags: <= 0.25 px corner error at 2-5 m.
The fit runs in undistorted pixels, where the edges are straight.
"""
from __future__ import annotations

import math
import re
from collections.abc import Sequence
from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera_model import PinholeCamera

DEFAULT_DICTIONARY = 'DICT_4X4_50'        # URC posts
DEFAULT_MARKER_SIZE_M = 0.20              # URC: black square, outer edge
MIN_SIDE_RANGE = (12, 16)
DEFAULT_AMBIGUITY_THRESHOLD = 0.6
DEFAULT_MEMORY_S = 1.0

DICTIONARIES = {name: getattr(cv2.aruco, name)
                for name in dir(cv2.aruco) if name.startswith('DICT_')}

_NAN = float('nan')


def opencv_version() -> tuple[int, int]:
    major, minor = (int(x) for x in re.findall(r'\d+', cv2.__version__)[:2])
    return major, minor


# cv2 4.6 (Ubuntu's python3-opencv) has no usable ArucoDetector -- it segfaults --
# so below 4.7 detection goes through the module-level detectMarkers.
LEGACY_API = opencv_version() < (4, 7) or not hasattr(cv2.aruco, 'ArucoDetector')

# Applied on both API paths; a name the installed params object lacks is
# skipped and reported in `ArucoTagDetector.unsupported_params`.
_SPECIFIED_PARAMS = ('useAruco3Detection', 'minSideLengthCanonicalImg',
                     'cornerRefinementMethod')


def dictionary_id(name: str) -> int:
    key = name if name.startswith('DICT_') else 'DICT_' + name
    if key not in DICTIONARIES:
        raise ValueError(f'unknown ArUco dictionary {name!r}; one of '
                         f'{sorted(DICTIONARIES)}')
    return DICTIONARIES[key]


def predefined_dictionary(name: str, legacy: bool = LEGACY_API):
    did = dictionary_id(name)
    if legacy and hasattr(cv2.aruco, 'Dictionary_get'):
        return cv2.aruco.Dictionary_get(did)
    return cv2.aruco.getPredefinedDictionary(did)


def detector_parameters(legacy: bool = LEGACY_API):
    if legacy and hasattr(cv2.aruco, 'DetectorParameters_create'):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


def marker_image(dictionary, tag_id: int, side_px: int) -> np.ndarray:
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, int(tag_id), int(side_px))
    return cv2.aruco.drawMarker(dictionary, int(tag_id), int(side_px))


def square_object_points(size_m: float) -> np.ndarray:
    """Marker corners in the marker frame, in ArUco's corner order (TL, TR, BR,
    BL) -- the order SOLVEPNP_IPPE_SQUARE requires."""
    h = size_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
                    dtype=np.float64)


def quad_centre(corners: np.ndarray) -> tuple[float, float]:
    """Intersection of the diagonals: the projective image of the square's
    centre. The corner mean is not, under perspective, and the error grows
    with tilt."""
    p0, p1, p2, p3 = (np.asarray(c, dtype=np.float64) for c in corners)
    d1, d2 = p2 - p0, p3 - p1
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denom) < 1e-12:
        m = np.mean(corners, axis=0)
        return float(m[0]), float(m[1])
    t = ((p1[0] - p0[0]) * d2[1] - (p1[1] - p0[1]) * d2[0]) / denom
    c = p0 + t * d1
    return float(c[0]), float(c[1])


# Edge refinement. Samples avoid the outer 12 % of each edge, where the
# neighbouring edge's blur bends the profile.
_EDGE_SAMPLES = 12
_EDGE_SPAN = (0.12, 0.88)
_PROFILE_STEP_PX = 0.25
# Grey levels between the white quiet zone and the black border below which an
# edge is not measurable (shadow, glare, motion blur).
MIN_EDGE_CONTRAST = 16.0
# A refined corner further than this fraction of the side from the detector's
# is a fit to something else (an occluder, a neighbouring marker): keep the
# detector's corners instead.
_MAX_REFINE_SHIFT_FRAC = 0.1


def _fit_line(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    m = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - m)
    return m, vt[0]


def refine_corners(gray: np.ndarray, corners: np.ndarray,
                   camera: PinholeCamera) -> np.ndarray | None:
    """Corners from edge-line intersections, or None if any edge is unmeasurable."""
    c = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    g = gray if gray.dtype == np.float32 else gray.astype(np.float32)
    side = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
    # The profile must stay inside the black border cell (1/6 of a 4x4 marker's
    # side, 1/7 of a 5x5's), or the data bits inside it move the 50 % level.
    reach = max(1.5, min(6.0, 0.7 * side / 7.0))
    offs = np.arange(-reach, reach + 1e-9, _PROFILE_STEP_PX)
    ts = np.linspace(_EDGE_SPAN[0], _EDGE_SPAN[1], _EDGE_SAMPLES)
    centre = c.mean(axis=0)
    lines = []
    for i in range(4):
        a, b = c[i], c[(i + 1) % 4]
        d = (b - a) / max(np.linalg.norm(b - a), 1e-9)
        n = np.array([-d[1], d[0]])
        if np.dot(centre - (a + b) / 2.0, n) < 0.0:
            n = -n                                    # n points into the marker
        base = a[None, :] + ts[:, None] * (b - a)[None, :]
        q = base[:, None, :] + offs[None, :, None] * n[None, None, :]
        prof = cv2.remap(g, q[..., 0].astype(np.float32), q[..., 1].astype(np.float32),
                         cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        pts = []
        for p, row in zip(base, prof, strict=True):
            hi, lo = float(row[:3].mean()), float(row[-3:].mean())
            if hi - lo < MIN_EDGE_CONTRAST:
                continue
            mid = 0.5 * (hi + lo)
            below = np.nonzero(row < mid)[0]
            if below.size == 0 or below[0] == 0:
                continue
            j = int(below[0])
            f = (row[j - 1] - mid) / (row[j - 1] - row[j])
            pts.append(p + (offs[j - 1] + f * _PROFILE_STEP_PX) * n)
        if len(pts) < 4:
            return None
        lines.append(_fit_line(camera.undistort_px(np.array(pts))))
    out = []
    for i in range(4):
        (p1, d1), (p2, d2) = lines[(i - 1) % 4], lines[i]
        A = np.array([d1, -d2]).T
        if abs(np.linalg.det(A)) < 1e-9:
            return None
        s = np.linalg.solve(A, p2 - p1)
        out.append(p1 + s[0] * d1)
    refined = camera.distort_px(np.array(out))
    if np.abs(refined - c).max() > _MAX_REFINE_SHIFT_FRAC * side + 1.0:
        return None
    return refined


def rvec_to_quaternion_xyzw(rvec: np.ndarray) -> tuple[float, float, float, float]:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(1.0 + tr)
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return float(x), float(y), float(z), float(w)


# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class PoseSolution:
    rvec: np.ndarray            # (3,) marker -> camera optical frame
    tvec: np.ndarray            # (3,) m, marker centre in the optical frame
    reproj_err_px: float

    @property
    def normal(self) -> np.ndarray:
        """Marker +z in the camera frame (points back toward the camera)."""
        R, _ = cv2.Rodrigues(self.rvec.reshape(3, 1))
        return R[:, 2].copy()


@dataclass
class TagObservation:
    tag_id: int
    corners: np.ndarray         # (4, 2) px, TL TR BR BL
    u: float
    v: float
    width_px: float
    height_px: float
    bearing_deg: float = _NAN
    range_m: float = _NAN
    pose: PoseSolution | None = None
    pose_ambiguity: float = _NAN
    disambiguated: bool = False  # the tracker overrode the lower-error solution
    refined: bool = False        # corners came from the edge fit, not the detector
    solutions: tuple[PoseSolution, ...] = field(default_factory=tuple)

    @property
    def has_pose(self) -> bool:
        return self.pose is not None and math.isfinite(self.range_m)

    @property
    def side_px(self) -> float:
        c = self.corners
        return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4])
                              for i in range(4)]))


# --------------------------------------------------------------------------- #
@dataclass
class _Track:
    normal: np.ndarray
    stamp_s: float


class FlipTracker:
    """Per-tag memory of the last accepted marker normal.

    Time is the FRAME's stamp, not a wall clock read here, so a replayed bag
    and a slow pipeline expire state by the scene's own time.
    """

    def __init__(self, ambiguity_threshold: float = DEFAULT_AMBIGUITY_THRESHOLD,
                 memory_s: float = DEFAULT_MEMORY_S) -> None:
        if not (0.0 < ambiguity_threshold <= 1.0):
            raise ValueError(f'ambiguity_threshold must be in (0, 1], got '
                             f'{ambiguity_threshold}')
        if not (memory_s > 0.0):
            raise ValueError(f'memory_s must be > 0, got {memory_s}')
        self.ambiguity_threshold = float(ambiguity_threshold)
        self.memory_s = float(memory_s)
        self._tracks: dict[int, _Track] = {}

    def previous(self, tag_id: int, stamp_s: float) -> np.ndarray | None:
        tr = self._tracks.get(tag_id)
        if tr is None:
            return None
        age = stamp_s - tr.stamp_s
        # A stamp that goes backwards is a bag loop or a clock step: the memory
        # belongs to another timeline.
        if not (0.0 <= age <= self.memory_s):
            del self._tracks[tag_id]
            return None
        return tr.normal

    def choose(self, tag_id: int, solutions: Sequence[PoseSolution],
               ambiguity: float, stamp_s: float) -> tuple[int, bool]:
        """`(index into solutions, overrode_best)`. `solutions` is best-first."""
        idx, overrode = 0, False
        prev = self.previous(tag_id, stamp_s)
        if (prev is not None and len(solutions) > 1
                and math.isfinite(ambiguity)
                and ambiguity > self.ambiguity_threshold):
            dots = [float(np.dot(s.normal, prev)) for s in solutions]
            idx = int(np.argmax(dots))
            overrode = idx != 0
        # Seeded even from an ambiguous frame with no history: a coin flip that
        # is then held is consistent, and the next clear frame corrects it.
        self._tracks[tag_id] = _Track(solutions[idx].normal, float(stamp_s))
        return idx, overrode

    def forget(self) -> None:
        self._tracks.clear()


# --------------------------------------------------------------------------- #
class ArucoTagDetector:

    def __init__(self, dictionary: str = DEFAULT_DICTIONARY,
                 marker_size_m: float = DEFAULT_MARKER_SIZE_M,
                 min_side_px: int = 16,
                 ambiguity_threshold: float = DEFAULT_AMBIGUITY_THRESHOLD,
                 memory_s: float = DEFAULT_MEMORY_S,
                 legacy: bool | None = None) -> None:
        lo, hi = MIN_SIDE_RANGE
        if not (lo <= int(min_side_px) <= hi) or int(min_side_px) != min_side_px:
            raise ValueError(f'min_side_px must be an integer in [{lo}, {hi}], '
                             f'got {min_side_px}')
        if not (math.isfinite(marker_size_m) and marker_size_m > 0.0):
            raise ValueError(f'marker_size_m must be > 0, got {marker_size_m}')
        self.dictionary_name = dictionary
        self.marker_size_m = float(marker_size_m)
        self.min_side_px = int(min_side_px)

        self.legacy = LEGACY_API if legacy is None else bool(legacy)
        if not self.legacy and not hasattr(cv2.aruco, 'ArucoDetector'):
            raise RuntimeError(f'cv2 {cv2.__version__} has no cv2.aruco.ArucoDetector; '
                               f'use the legacy detectMarkers path (legacy=True)')
        params = detector_parameters(self.legacy)
        wanted = {'useAruco3Detection': True,
                  'minSideLengthCanonicalImg': self.min_side_px,
                  'cornerRefinementMethod': cv2.aruco.CORNER_REFINE_SUBPIX}
        unsupported = []
        for name in _SPECIFIED_PARAMS:
            if hasattr(params, name):
                setattr(params, name, wanted[name])
            else:
                unsupported.append(name)
        self.unsupported_params: tuple[str, ...] = tuple(unsupported)
        self.parameters = params
        dic = predefined_dictionary(dictionary, self.legacy)
        if self.legacy:
            self._find = lambda gray: cv2.aruco.detectMarkers(gray, dic, parameters=params)
        else:
            detector = cv2.aruco.ArucoDetector(dic, params)
            self._find = detector.detectMarkers
        self._obj = square_object_points(self.marker_size_m)
        self.tracker = FlipTracker(ambiguity_threshold, memory_s)

    def solve_pose(self, corners: np.ndarray, camera: PinholeCamera
                   ) -> tuple[tuple[PoseSolution, ...], float]:
        """Both IPPE solutions best-first, and err_best / err_second.

        Empty and NaN unless the camera is calibrated.
        """
        if not camera.calibrated:
            return (), _NAN
        img = np.asarray(corners, dtype=np.float64).reshape(4, 1, 2)
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            self._obj, img, camera.K, camera.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        sols = []
        for i in range(int(n)):
            t = np.asarray(tvecs[i], dtype=np.float64).reshape(3)
            if not (np.isfinite(t).all() and t[2] > 0.0):
                continue
            err = float(np.asarray(errs).reshape(-1)[i]) if errs is not None else _NAN
            sols.append(PoseSolution(np.asarray(rvecs[i], dtype=np.float64).reshape(3),
                                     t, err))
        sols.sort(key=lambda s: s.reproj_err_px)
        if not sols:
            return (), _NAN
        if len(sols) == 1:
            return tuple(sols), 0.0
        e0, e1 = sols[0].reproj_err_px, sols[1].reproj_err_px
        if e1 <= 1e-12:
            # Both fit exactly: nothing in this frame distinguishes them.
            amb = 1.0
        else:
            amb = e0 / e1
        return tuple(sols), float(amb)

    def detect(self, image: np.ndarray, camera: PinholeCamera,
               stamp_s: float = 0.0) -> list[TagObservation]:
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if (gray.shape[1], gray.shape[0]) != (camera.width, camera.height):
            raise ValueError(f'image is {gray.shape[1]}x{gray.shape[0]} but the '
                             f'camera model is {camera.width}x{camera.height}')
        corners_list, ids = self._find(gray)[:2]
        out: list[TagObservation] = []
        if ids is None:
            return out
        gray_f = gray.astype(np.float32)
        for c, tid in zip(corners_list, ids.reshape(-1), strict=True):
            c = np.asarray(c, dtype=np.float64).reshape(4, 2)
            r = refine_corners(gray_f, c, camera)
            if r is not None:
                c = r
            u, v = quad_centre(c)
            lo, hi = c.min(axis=0), c.max(axis=0)
            obs = TagObservation(int(tid), c, u, v, float(hi[0] - lo[0]),
                                 float(hi[1] - lo[1]),
                                 bearing_deg=camera.bearing_deg(u, v),
                                 refined=r is not None)
            sols, amb = self.solve_pose(c, camera)
            if sols:
                idx, overrode = self.tracker.choose(obs.tag_id, sols, amb, stamp_s)
                chosen = sols[idx]
                obs.pose = chosen
                obs.solutions = sols
                obs.pose_ambiguity = amb
                obs.disambiguated = overrode
                obs.range_m = float(np.linalg.norm(chosen.tvec))
            out.append(obs)
        return out
