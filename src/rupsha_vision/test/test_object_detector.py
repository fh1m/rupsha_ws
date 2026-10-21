"""YOLO decoding and letterboxing against tensors and images whose truth is set here.

No real model is needed. The decoders get synthetic output tensors; the full
`ObjectDetector` path (onnxruntime session, letterbox, decode, map back,
bearing, range) gets a tiny ONNX graph that ignores its pixels and emits a
known tensor -- so what is under test is everything AROUND the network, which
is the part this package owns.
"""
import math

import numpy as np
import pytest

from rupsha_vision import detector as D
from rupsha_vision.camera_model import PinholeCamera

CAM = PinholeCamera.from_fov(640, 480, 70.0)


# --------------------------------------------------------------------------- #
#  Letterbox
# --------------------------------------------------------------------------- #
def test_letterbox_640x480_to_320_is_half_scale_with_40px_bands():
    img, lb = D.letterbox(np.zeros((480, 640, 3), np.uint8), 320)
    assert img.shape == (320, 320, 3)
    assert (lb.sx, lb.sy, lb.pad_x, lb.pad_y) == (0.5, 0.5, 0, 40)
    assert (img[:40] == D.LETTERBOX_FILL).all() and (img[280:] == D.LETTERBOX_FILL).all()


@pytest.mark.parametrize('w,h', [(640, 480), (1280, 720), (333, 500), (320, 320),
                                 (1000, 1)])
def test_letterbox_inverse_is_exact(w, h):
    _, lb = D.letterbox(np.zeros((h, w), np.uint8), 320)
    boxes = np.array([[0, 0, w, h], [w * 0.1, h * 0.2, w * 0.7, h * 0.9],
                      [1.25, 0.5, 3.75, h]])
    np.testing.assert_allclose(lb.to_source(lb.to_input(boxes)), boxes, atol=1e-9)


@pytest.mark.parametrize('w,h,box', [(640, 480, (100, 50, 300, 250)),
                                     (1280, 720, (400, 100, 1000, 600)),
                                     (333, 500, (20, 60, 200, 420))])
def test_a_painted_rectangle_maps_back_to_where_it_was_painted(w, h, box):
    """Paint the box, letterbox the image, find the box in the network's input
    by its pixels, map back: it must land on the painted edges to within one
    input pixel (the resample blurs each edge by at most that)."""
    src = np.zeros((h, w), np.uint8)
    x1, y1, x2, y2 = box
    src[y1:y2, x1:x2] = 255
    img, lb = D.letterbox(src, 320)
    ys, xs = np.nonzero(img > 127)
    found = np.array([[xs.min(), ys.min(), xs.max() + 1, ys.max() + 1]], float)
    back = lb.to_source(found)[0]
    tol = 1.0 / min(lb.sx, lb.sy)
    np.testing.assert_allclose(back, box, atol=tol)


def test_to_source_clips_to_the_image():
    _, lb = D.letterbox(np.zeros((480, 640), np.uint8), 320)
    out = lb.to_source([[-10, 0, 330, 330]])[0]
    np.testing.assert_allclose(out, [0, 0, 640, 480])


# --------------------------------------------------------------------------- #
#  Decoders
# --------------------------------------------------------------------------- #
def _classic(rows, nc, n=2100):
    """rows: (cx, cy, w, h, cls, score). Every other anchor is background."""
    out = np.zeros((1, 4 + nc, n), np.float32)
    for i, (cx, cy, w, h, c, s) in enumerate(rows):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4 + c, i] = s
    return out


def test_classic_decode_nms_within_a_class_only():
    out = _classic([(100, 100, 40, 40, 0, 0.90),     # kept
                    (102, 101, 40, 40, 0, 0.80),     # same class, IoU ~0.9: suppressed
                    (101, 100, 40, 40, 1, 0.70),     # other class, same place: kept
                    (250, 200, 20, 60, 2, 0.60),     # kept
                    (50, 50, 10, 10, 1, 0.20)],      # below conf
                   nc=3)
    r = D.decode_classic(out, conf=0.35, iou=0.45)
    assert r.cls.tolist() == [0, 1, 2]
    np.testing.assert_allclose(r.score, [0.9, 0.7, 0.6], rtol=1e-6)
    np.testing.assert_allclose(r.xyxy, [[80, 80, 120, 120], [81, 80, 121, 120],
                                        [240, 170, 260, 230]], atol=1e-4)


def test_classic_decode_keeps_overlapping_boxes_below_iou():
    out = _classic([(100, 100, 40, 40, 0, 0.9), (130, 100, 40, 40, 0, 0.8)], nc=1)
    r = D.decode_classic(out, conf=0.35, iou=0.45)      # IoU = 10/70 = 0.14
    assert len(r.score) == 2


def test_classic_decode_nothing_above_threshold_is_empty():
    r = D.decode_classic(_classic([(10, 10, 5, 5, 0, 0.1)], nc=2), conf=0.35)
    assert r.xyxy.shape == (0, 4) and r.score.size == 0


def test_end2end_decode_is_conf_filter_only():
    out = np.zeros((1, 300, 6), np.float32)
    out[0, 0] = (10, 20, 50, 80, 0.55, 2)
    out[0, 1] = (11, 21, 51, 81, 0.95, 2)       # overlapping, same class: NOT suppressed
    out[0, 2] = (5, 5, 9, 9, 0.30, 0)           # below conf
    r = D.decode_end2end(out, conf=0.35)
    np.testing.assert_allclose(r.score, [0.95, 0.55], rtol=1e-6)
    assert r.cls.tolist() == [2, 2]
    np.testing.assert_allclose(r.xyxy, [[11, 21, 51, 81], [10, 20, 50, 80]])


def test_layout_is_told_apart_by_shape():
    assert D.output_layout((1, 84, 2100), 80) == 'classic'
    assert D.output_layout((1, 300, 6), 80) == 'end2end'
    assert D.output_layout((1, 7, 2100), 3) == 'classic'


@pytest.mark.parametrize('shape,nc', [((1, 6, 6), 2), ((1, 85, 2100), 80),
                                      ((2, 84, 2100), 80), ((84, 2100), 80)])
def test_layout_refuses_shapes_it_cannot_place(shape, nc):
    with pytest.raises(ValueError):
        D.output_layout(shape, nc)


# --------------------------------------------------------------------------- #
#  Configuration
# --------------------------------------------------------------------------- #
def test_parse_class_heights():
    assert D.parse_class_heights('') == {}
    assert D.parse_class_heights(' mallet:0.3, water bottle : 0.25 ,') == {
        'mallet': 0.3, 'water bottle': 0.25}


@pytest.mark.parametrize('bad', ['mallet', 'mallet:', ':0.3', 'mallet:abc',
                                 'mallet:0', 'mallet:-1', 'mallet:nan'])
def test_parse_class_heights_refuses_malformed(bad):
    with pytest.raises(ValueError, match='class_heights_m'):
        D.parse_class_heights(bad)


def test_class_names_from_txt_and_yaml(tmp_path):
    (tmp_path / 'n.txt').write_text('mallet\n\n# comment\nbottle\n')
    (tmp_path / 'l.yaml').write_text('[mallet, bottle]\n')
    (tmp_path / 'd.yaml').write_text('names:\n  0: mallet\n  1: bottle\n')
    for f in ('n.txt', 'l.yaml', 'd.yaml'):
        assert D.load_class_names(str(tmp_path / f)) == ['mallet', 'bottle']
    (tmp_path / 'gap.yaml').write_text('names: {0: a, 2: c}\n')
    with pytest.raises(ValueError, match='0..n-1'):
        D.load_class_names(str(tmp_path / 'gap.yaml'))


def test_pinhole_range_truth_and_refusals():
    """An object H tall at Z projects to fy * H / Z px; the estimate inverts it."""
    H, Z = 0.5, 4.0
    h = CAM.fy * H / Z
    y1 = 200.0
    assert D.pinhole_range_m(CAM, H, y1, y1 + h) == pytest.approx(Z, rel=1e-12)
    assert math.isnan(D.pinhole_range_m(CAM, H, 0.0, h)), 'truncated at the top'
    assert math.isnan(D.pinhole_range_m(CAM, H, 480 - h, 480.0)), 'truncated at the bottom'
    assert math.isnan(D.pinhole_range_m(PinholeCamera.uncalibrated(640, 480), H,
                                        y1, y1 + h))
    nominal = PinholeCamera.from_fov(640, 480, 70.0, calibrated=False)
    assert math.isnan(D.pinhole_range_m(nominal, H, y1, y1 + h))


# --------------------------------------------------------------------------- #
#  The detector, end to end through onnxruntime
# --------------------------------------------------------------------------- #
def test_missing_model_is_a_loud_error_naming_path_and_export(tmp_path):
    path = str(tmp_path / 'yolo11n.onnx')
    with pytest.raises(FileNotFoundError) as e:
        D.ObjectDetector(path)
    assert path in str(e.value)
    assert 'yolo export model=yolo11n.pt format=onnx imgsz=320' in str(e.value)
    with pytest.raises(FileNotFoundError):
        D.ObjectDetector('')


def _const_model(path, out, names=None, size=320):
    """ONNX graph: images[1,3,size,size] -> out (constant), touching the input
    so the runtime cannot prune it."""
    onnx = pytest.importorskip('onnx')
    pytest.importorskip('onnxruntime')
    from onnx import TensorProto, helper, numpy_helper
    out = np.asarray(out, np.float32)
    nodes = [
        helper.make_node('Constant', [], ['c'], value=numpy_helper.from_array(out)),
        helper.make_node('ReduceMean', ['images'], ['m'], axes=[1, 2, 3], keepdims=0),
        helper.make_node('Mul', ['m', 'zero'], ['m0']),
        helper.make_node('Add', ['c', 'm0'], ['output0']),
    ]
    graph = helper.make_graph(
        nodes, 'const',
        [helper.make_tensor_value_info('images', TensorProto.FLOAT, [1, 3, size, size])],
        [helper.make_tensor_value_info('output0', TensorProto.FLOAT, list(out.shape))],
        [numpy_helper.from_array(np.zeros(1, np.float32), 'zero')])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)])
    model.ir_version = 8
    if names is not None:
        helper.set_model_props(model, {'names': repr(dict(enumerate(names)))})
    onnx.save(model, path)
    return path


def _src_box_to_input(x1, y1, x2, y2):
    # 640x480 -> 320: scale 0.5, 40 px top band (asserted by the first test).
    return x1 * 0.5, y1 * 0.5 + 40.0, x2 * 0.5, y2 * 0.5 + 40.0


def test_classic_model_end_to_end_recovers_box_label_bearing_range(tmp_path):
    H, Z = 0.5, 4.0
    truth = (100.0, 150.0, 140.0, 150.0 + CAM.fy * H / Z)      # source px
    ix1, iy1, ix2, iy2 = _src_box_to_input(*truth)
    cx, cy, w, h = (ix1 + ix2) / 2, (iy1 + iy2) / 2, ix2 - ix1, iy2 - iy1
    out = _classic([(cx, cy, w, h, 1, 0.9),
                    (cx + 0.5, cy, w, h, 1, 0.6),              # duplicate -> NMS
                    (200.0, 150.0, 30.0, 30.0, 0, 0.5)], nc=2)
    path = _const_model(str(tmp_path / 'm.onnx'), out, names=['mallet', 'bottle'])
    det = D.ObjectDetector(path, class_heights_m={'bottle': H}, providers=('CPUExecutionProvider',))
    assert det.layout == 'classic' and det.class_names == ['mallet', 'bottle']

    obs = det.detect(np.zeros((480, 640, 3), np.uint8), CAM)
    assert [o.label for o in obs] == ['bottle', 'mallet']
    b = obs[0]
    np.testing.assert_allclose([b.x1, b.y1, b.x2, b.y2], truth, atol=1e-3)
    assert b.bearing_deg == pytest.approx(CAM.bearing_deg((truth[0] + truth[2]) / 2), abs=1e-6)
    assert b.range_m == pytest.approx(Z, rel=1e-4)
    assert math.isnan(obs[1].range_m), 'no height configured for mallet'


def test_end2end_model_end_to_end(tmp_path):
    out = np.zeros((1, 300, 6), np.float32)
    out[0, 0] = (*_src_box_to_input(320, 200, 400, 300), 0.8, 0)
    path = _const_model(str(tmp_path / 'm.onnx'), out)
    det = D.ObjectDetector(path, class_names=['mallet'], providers=('CPUExecutionProvider',))
    assert det.layout == 'end2end'
    (o,) = det.detect(np.zeros((480, 640, 3), np.uint8), CAM)
    np.testing.assert_allclose([o.x1, o.y1, o.x2, o.y2], [320, 200, 400, 300], atol=1e-3)
    assert o.confidence == pytest.approx(0.8) and math.isnan(o.range_m)


def test_model_without_names_is_refused(tmp_path):
    path = _const_model(str(tmp_path / 'm.onnx'), np.zeros((1, 300, 6)))
    with pytest.raises(ValueError, match='class names'):
        D.ObjectDetector(path)


def test_input_size_mismatch_is_refused(tmp_path):
    path = _const_model(str(tmp_path / 'm.onnx'), np.zeros((1, 300, 6)), names=['a'],
                        size=640)
    with pytest.raises(ValueError, match='imgsz=320'):
        D.ObjectDetector(path, input_size=320)


def test_class_id_beyond_the_names_is_refused(tmp_path):
    out = np.zeros((1, 300, 6), np.float32)
    out[0, 0] = (10, 50, 60, 90, 0.9, 5)
    path = _const_model(str(tmp_path / 'm.onnx'), out, names=['a', 'b'])
    det = D.ObjectDetector(path, providers=('CPUExecutionProvider',))
    with pytest.raises(ValueError, match='class id 5'):
        det.detect(np.zeros((480, 640, 3), np.uint8), CAM)


def test_height_for_an_unknown_class_is_refused(tmp_path):
    path = _const_model(str(tmp_path / 'm.onnx'), np.zeros((1, 300, 6)), names=['a'])
    with pytest.raises(ValueError, match='not classes'):
        D.ObjectDetector(path, class_heights_m={'b': 1.0})


def test_preprocess_is_rgb_chw_unit_range():
    det = D.ObjectDetector.__new__(D.ObjectDetector)
    det.input_size = 320
    bgr = np.zeros((480, 640, 3), np.uint8)
    bgr[..., 0] = 255                                   # pure blue
    x, _ = det.preprocess(bgr)
    assert x.shape == (1, 3, 320, 320) and x.dtype == np.float32
    assert x[0, 2, 160, 160] == 1.0 and x[0, 0, 160, 160] == 0.0
    assert x[0, 0, 0, 0] == pytest.approx(D.LETTERBOX_FILL / 255.0)
