"""The shipped config names only parameters the node declares, with the cores' defaults.

A key in vision.yaml that the node never declares is silently ignored by
rclpy; a default that differs from the core's means the bench and the rover
run different detectors from the same code.
"""
import pathlib
import re

import pytest

from rupsha_vision import aruco, detector

yaml = pytest.importorskip('yaml')
PKG = pathlib.Path(__file__).resolve().parents[1]


def _params():
    return yaml.safe_load((PKG / 'config' / 'vision.yaml').read_text())['vision']['ros__parameters']


def test_every_config_key_is_declared_by_the_node():
    declared = set(re.findall(r"\bp\('([a-z_]+)'", (PKG / 'rupsha_vision' / 'vision_node.py').read_text()))
    assert set(_params()) <= declared, set(_params()) - declared


def test_config_defaults_match_the_cores():
    p = _params()
    assert p['dictionary'] == aruco.DEFAULT_DICTIONARY
    assert p['marker_size_m'] == aruco.DEFAULT_MARKER_SIZE_M
    assert p['ambiguity_threshold'] == aruco.DEFAULT_AMBIGUITY_THRESHOLD
    assert p['pose_memory_s'] == aruco.DEFAULT_MEMORY_S
    assert aruco.MIN_SIDE_RANGE[0] <= p['min_side_px'] <= aruco.MIN_SIDE_RANGE[1]
    assert p['input_size'] == detector.DEFAULT_INPUT_SIZE
    assert p['conf_threshold'] == detector.DEFAULT_CONF
    assert p['iou_threshold'] == detector.DEFAULT_IOU
    assert p['model_path'] == ''
    # Typed as the node declares them: a 0 here would make rclpy reject 0.0 later.
    assert isinstance(p['fallback_hfov_deg'], float)
    assert isinstance(p['marker_size_m'], float)
