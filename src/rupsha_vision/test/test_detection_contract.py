"""What vision publishes must be what the planner and the facade select on (Detection.msg)."""
import math
import pathlib
import re
from types import SimpleNamespace

from rupsha_vision import vision_node
from rupsha_vision.aruco import TagObservation

ROOT = pathlib.Path(__file__).resolve().parents[3]
MSG = ROOT / 'src/rupsha_interfaces/msg/Detection.msg'
CONSUMERS = (
    ROOT / 'src/rupsha_planner/rupsha_planner/executive.py',
    ROOT / 'src/rupsha_planner/rupsha_planner/dsl.py',
    ROOT / 'src/rupsha_control/rupsha_control/rupsha.py',
)


def _detection():
    xyz = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    quat = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    return SimpleNamespace(pose=SimpleNamespace(position=xyz, orientation=quat))


def _tag():
    return TagObservation(tag_id=3, corners=None, u=320.0, v=240.0, width_px=30.0, height_px=30.0,
                          bearing_deg=1.0)


def test_tag_label_and_confidence_follow_the_message():
    text = MSG.read_text()
    label = re.search(r'label\s+#\s*"(\w+)"', text).group(1)
    assert 'confidence' in text and '1.0 for a decoded tag' in text
    d = vision_node.tag_to_msg(_detection, _tag())
    assert d.label == label
    assert d.confidence == 1.0
    assert d.tag_id == 3 and math.isnan(d.range_m) and math.isnan(d.pose.position.x)


def test_every_consumer_selects_tags_by_the_published_label():
    for path in CONSUMERS:
        assert f"'{vision_node.TAG_LABEL}'" in path.read_text(), path.name
