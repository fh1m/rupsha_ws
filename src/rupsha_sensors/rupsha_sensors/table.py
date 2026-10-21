"""config/sensors.yaml -> validated rows -> the driver nodes to launch.

Pure (no rclpy), so the table is checked in CI on a machine with no camera:
a topic that is not in the manifest, a driver we do not know how to start, or
a calibration file that does not exist fails here, not on the rover.
"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass, field

import yaml

from rupsha_control.manifest import Topics

PKG = pathlib.Path(__file__).resolve().parent.parent
DRIVERS = ('v4l2_camera', 'camera_ros', 'board')
CAMERA_DRIVERS = ('v4l2_camera', 'camera_ros')


class TableError(ValueError):
    pass


@dataclass(frozen=True)
class Sensor:
    name: str
    driver: str
    topic: str
    min_hz: float
    info_topic: str | None = None
    frame_id: str = ''
    device: str = ''
    width: int = 0
    height: int = 0
    fps: float = 0.0
    calibration: str | None = None
    extra: dict = field(default_factory=dict)


def _topic(name: str, key: str, row: str) -> str:
    value = getattr(Topics, key, None) if isinstance(key, str) else None
    if not isinstance(value, str):
        raise TableError(f'{row}: {name} {key!r} is not a manifest Topics attribute')
    return value


def load(path: pathlib.Path | str | None = None, calib_dir: pathlib.Path | None = None) -> list[Sensor]:
    path = pathlib.Path(path or PKG / 'config' / 'sensors.yaml')
    calib_dir = calib_dir or path.parent / 'calibration'
    doc = yaml.safe_load(path.read_text()) or {}
    out = []
    for name, row in (doc.get('sensors') or {}).items():
        if not row.get('enabled', True):
            continue
        driver = row.get('driver')
        if driver not in DRIVERS:
            raise TableError(f'{name}: driver {driver!r} not one of {DRIVERS}')
        if not row.get('min_hz', 0) > 0:
            raise TableError(f'{name}: min_hz must be > 0')
        cal = row.get('calibration')
        if cal and not (calib_dir / cal).exists():
            raise TableError(f'{name}: calibration {calib_dir / cal} does not exist')
        if driver in CAMERA_DRIVERS:
            for key in ('info_topic', 'frame_id', 'width', 'height', 'fps'):
                if not row.get(key):
                    raise TableError(f'{name}: camera rows need {key}')
            if row['fps'] < row['min_hz']:
                raise TableError(f'{name}: fps {row["fps"]} below its own min_hz {row["min_hz"]}')
        known = {f for f in Sensor.__dataclass_fields__} - {'name', 'extra'}
        out.append(Sensor(
            name=name, driver=driver, topic=_topic('topic', row.get('topic'), name),
            min_hz=float(row['min_hz']),
            info_topic=_topic('info_topic', row['info_topic'], name) if row.get('info_topic') else None,
            frame_id=row.get('frame_id', ''), device=str(row.get('device', '')),
            width=int(row.get('width', 0)), height=int(row.get('height', 0)),
            fps=float(row.get('fps', 0)), calibration=cal,
            extra={k: v for k, v in row.items() if k not in known | {'enabled'}}))
    return out


@dataclass(frozen=True)
class NodeSpec:
    package: str
    executable: str
    name: str
    parameters: dict
    remappings: tuple


def node_spec(s: Sensor, calib_dir: pathlib.Path) -> NodeSpec | None:
    """The driver node for one row, or None when the manager already publishes it."""
    info_url = f'file://{calib_dir / s.calibration}' if s.calibration else ''
    if s.driver == 'v4l2_camera':
        return NodeSpec(
            'v4l2_camera', 'v4l2_camera_node', s.name,
            {'video_device': s.device, 'image_size': [s.width, s.height],
             'time_per_frame': [1, int(round(s.fps))], 'camera_frame_id': s.frame_id,
             'camera_info_url': info_url, 'output_encoding': 'rgb8'},
            (('image_raw', s.topic), ('camera_info', s.info_topic)))
    if s.driver == 'camera_ros':
        return NodeSpec(
            'camera_ros', 'camera_node', s.name,
            {'camera': s.device or 0, 'width': s.width, 'height': s.height,
             'frame_id': s.frame_id, 'camera_info_url': info_url, 'format': 'RGB888',
             'FrameDurationLimits': [int(1e6 / s.fps)] * 2},
            (('~/image_raw', s.topic), ('~/camera_info', s.info_topic)))
    return None
