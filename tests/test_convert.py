"""kinect_console.convert: message -> browser data, no ROS needed."""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from kinect_console import convert


def mono16(arr, pad=0):
    h, w = arr.shape
    rows = np.zeros((h, w + pad), "<u2")
    rows[:, :w] = arr
    return SimpleNamespace(height=h, width=w, step=(w + pad) * 2, encoding="mono16",
                           data=rows.tobytes())


def decode(jpeg):
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_UNCHANGED)


def test_image_u16_honours_row_padding():
    a = np.arange(12, dtype=np.uint16).reshape(3, 4)
    assert (convert.image_u16(mono16(a, pad=2)) == a).all()


def test_depth_colormap_near_warm_far_cool_invalid_black():
    d = np.zeros((10, 30), np.uint16)
    d[:, 10:20] = 600      # near
    d[:, 20:] = 4400       # far
    bgr = convert.depth_u16_to_bgr(d)
    assert (bgr[:, :10] == 0).all()                  # no reading stays black
    near, far = bgr[5, 15].astype(int), bgr[5, 25].astype(int)
    assert near[2] > near[0] and far[0] > far[2]     # red-ish near, blue-ish far


def test_depth_and_ir_to_jpeg():
    d = np.random.default_rng(0).integers(500, 4500, (424, 512)).astype(np.uint16)
    for fn in (convert.depth_to_jpeg, convert.ir_to_jpeg):
        img = decode(fn(mono16(d)))
        assert img.shape[:2] == (424, 512)


def test_ir_auto_gain_fills_range():
    ir = np.full((50, 50), 400, np.uint16)
    ir[0, 0] = 60000
    u8 = convert.ir_u16_to_u8(ir)
    assert u8.max() == 255 and u8[10, 10] == 255     # 99th percentile -> white


def test_bgr_to_jpeg_rgb_and_bgra():
    img = np.zeros((4, 6, 3), np.uint8)
    img[..., 2] = 200                                  # red in BGR
    msg = SimpleNamespace(height=4, width=6, step=18, encoding="bgr8", data=img.tobytes())
    out = decode(convert.bgr_to_jpeg(msg))
    assert out[2, 2, 2] > 150 and out[2, 2, 0] < 60
    bgra = np.dstack([img, np.full((4, 6), 255, np.uint8)])
    msg = SimpleNamespace(height=4, width=6, step=24, encoding="bgra8", data=bgra.tobytes())
    assert decode(convert.bgr_to_jpeg(msg)).shape == (4, 6, 3)


def test_passthrough_jpeg_is_untouched():
    assert convert.passthrough_jpeg(SimpleNamespace(data=b"\xff\xd8abc")) == b"\xff\xd8abc"


def test_bad_message_returns_none():
    bad = SimpleNamespace(height=10, width=10, step=20, encoding="mono16", data=b"short")
    assert convert.depth_to_jpeg(bad) is None and convert.ir_to_jpeg(bad) is None


def test_depth_at_median_and_holes():
    d = np.zeros((20, 20), np.uint16)
    d[5:15, 5:15] = 1500
    d[10, 10] = 0                                      # dropped pixel
    assert convert.depth_at(d, 10, 10) == 1500
    assert convert.depth_at(d, 0, 0, radius=2) is None


def test_channel_levels_from_rclpy_style_bytes():
    t = np.arange(1600) / 16000
    pcm = np.stack([np.sin(2 * np.pi * 440 * t) * a for a in (0.5, 0.25, 0.1, 0.0)], 1)
    raw = (pcm * 32767).astype("<i2").tobytes()
    as_list = [bytes([b]) for b in raw]               # how rclpy hands ByteMultiArray.data over
    for data in (raw, as_list):
        lv = convert.channel_levels(data, 4)
        assert lv[0] == pytest.approx(0.5 / np.sqrt(2), rel=0.02)
        assert lv[1] == pytest.approx(0.25 / np.sqrt(2), rel=0.02)
        assert lv[3] == 0.0
