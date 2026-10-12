"""Explicit letterbox inversion and all-frame no-detection records."""

from __future__ import annotations

import hashlib
import sys
from pathlib import Path
from types import SimpleNamespace as Obj

import numpy as np
import pytest
import torch


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.postprocess_detections import (
    _observation, inference_due, letterbox_rgb_to_bgr, unletterbox_xyxy,
)


class _Boxes:
    def __init__(self, box=None):
        self.xyxy = torch.empty((0, 4)) if box is None else torch.tensor([box])
        self.conf = torch.empty((0,)) if box is None else torch.tensor([0.8])

    def __len__(self):
        return len(self.conf)


class _Model:
    def __init__(self, box=None):
        self.box = box
        self.kwargs = None

    def predict(self, **kwargs):
        self.kwargs = kwargs
        return [Obj(orig_shape=(736, 736), boxes=_Boxes(self.box))]


def _image_index():
    pixels = np.zeros((720, 1280, 3), dtype=np.uint8)
    pixels[0, 0] = [255, 0, 0]
    raw = pixels.tobytes()
    image = Obj(
        data=raw, width=1280, height=720, encoding="rgb8",
        header=Obj(stamp=Obj(sec=1, nanosec=250_000_000)),
    )
    index = {
        "run_id": "G0-P0-R01", "attempt_id": "a01", "simulation_epoch": 0,
        "camera_id": "left", "frame_id": "left:0:1250000000:000001",
        "pair_id": None, "image_timestamp_ns": 1_250_000_000,
        "receive_timestamp_ns": 2_000_000_000,
        "receive_time_domain": "rosbag2_record_utc_ns",
        "original_image_reference": "raw_images_or_lossless_bag/left/left_0.db3#message_id=1",
        "image_pixels_sha256": hashlib.sha256(raw).hexdigest(),
    }
    config = {"sha256": "a" * 64, "confidence": 0.1, "nms_iou": 0.7,
              "inference_imgsz_px": 736, "inference_max_det": 1}
    return image, index, config


def test_letterbox_is_explicit_and_inverse_returns_original_pixels():
    rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
    rgb[360, 640] = [255, 0, 0]
    input_bgr, transform = letterbox_rgb_to_bgr(rgb, 736)
    assert input_bgr.shape == (736, 736, 3)
    assert transform["scale"] == pytest.approx(0.575)
    assert (transform["pad_left_px"], transform["pad_top_px"],
            transform["pad_right_px"], transform["pad_bottom_px"]) == (0, 161, 0, 161)
    assert input_bgr[0, 0].tolist() == [114, 114, 114]
    assert unletterbox_xyxy([57.5, 247.25, 115.0, 304.75], transform) == pytest.approx(
        [100.0, 150.0, 200.0, 250.0]
    )


def test_detected_and_missing_frames_keep_source_identity_and_nulls():
    image, index, config = _image_index()
    model = _Model([57.5, 247.25, 115.0, 304.75])
    detected = _observation(index, image, model, config, "cpu", "8.4.127")
    assert detected["detection_status"] == "DETECTED"
    assert detected["bbox_xyxy_px"] == pytest.approx([100.0, 150.0, 200.0, 250.0])
    assert detected["bbox_center_uv_px"] == pytest.approx([150.0, 200.0])
    assert detected["image_timestamp_ns"] == 1_250_000_000
    assert detected["original_image_reference"] == index["original_image_reference"]
    assert model.kwargs["iou"] == 0.7 and model.kwargs["imgsz"] == 736

    missing = _observation(index, image, _Model(), config, "cpu", "8.4.127")
    assert missing["detection_status"] == "NOT_DETECTED"
    assert missing["bbox_xyxy_px"] is None
    assert missing["bbox_center_uv_px"] is None
    assert missing["confidence"] is None


def test_rate_limiter_tracks_a_fixed_phase_through_camera_timestamp_jitter():
    next_due = None
    inferred = []
    for timestamp_ns in (0, 48_000_000, 100_000_000, 148_000_000, 200_000_000, 248_000_000):
        due, next_due = inference_due(timestamp_ns, next_due, 50_000_000)
        if due:
            inferred.append(timestamp_ns)
    assert inferred == [0, 48_000_000, 100_000_000, 148_000_000, 200_000_000, 248_000_000]


def test_rate_limiter_selects_every_other_frame_when_camera_is_twice_as_fast():
    next_due = None
    inferred = []
    for timestamp_ns in (0, 25_000_000, 50_000_000, 75_000_000, 100_000_000, 125_000_000):
        due, next_due = inference_due(timestamp_ns, next_due, 50_000_000)
        if due:
            inferred.append(timestamp_ns)
    assert inferred == [0, 50_000_000, 100_000_000]
