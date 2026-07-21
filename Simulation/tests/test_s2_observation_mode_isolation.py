from collections import deque
from pathlib import Path
import sys
import threading

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

import stereo_ground_sim.stereo_distance_node as node_module
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


def _node(mode, hz, path, disabled):
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.observation_mode = mode
    node.yolo_target_hz = hz
    node.yolo_model_path = path
    node.yolo_runtime_disabled = disabled
    node.resolved_yolo_device = None
    node._left_cb = lambda _message: None
    node._right_cb = lambda _message: None
    return node


@pytest.mark.parametrize("mode", ("bbox_yolo", "ideal_reference_point"))
def test_observation_mode_validation_accepts_only_exact_supported_values(mode):
    assert StereoDistanceNode._validate_observation_mode(mode) == mode


@pytest.mark.parametrize(
    "mode",
    ("", "BBOX_YOLO", "bbox_yolo ", "ideal", "unknown"),
)
def test_observation_mode_validation_rejects_aliases_and_silent_fallbacks(mode):
    with pytest.raises(ValueError, match="observation_mode"):
        StereoDistanceNode._validate_observation_mode(mode)


@pytest.mark.parametrize(
    ("mode", "hz", "path", "disabled", "load", "subscribe"),
    (
        ("bbox_yolo", 40.0, "/tmp/model.pt", False, True, True),
        ("bbox_yolo", 40.0, "", False, False, False),
        ("bbox_yolo", 0.0, "/tmp/model.pt", False, False, False),
        ("bbox_yolo", 40.0, "/tmp/model.pt", True, False, False),
        ("ideal_reference_point", 40.0, "", False, False, True),
    ),
)
def test_loader_and_subscription_preconditions_by_mode(
    mode, hz, path, disabled, load, subscribe
):
    node = _node(mode, hz, path, disabled)
    assert node._should_load_yolo_model() is load
    assert node._should_subscribe_camera_images() is subscribe


@pytest.mark.parametrize(
    ("mode", "hz", "path", "disabled", "expected_initialized"),
    (
        ("bbox_yolo", 40.0, "/tmp/model.pt", False, True),
        ("bbox_yolo", 40.0, "", False, False),
        ("bbox_yolo", 0.0, "/tmp/model.pt", False, False),
        ("bbox_yolo", 40.0, "/tmp/model.pt", True, False),
        ("ideal_reference_point", 40.0, "", False, False),
    ),
)
def test_yolo_runtime_initialization_call_counts(
    mode, hz, path, disabled, expected_initialized
):
    node = _node(mode, hz, path, disabled)
    calls = {"resolve": 0, "load": 0}

    def resolve():
        calls["resolve"] += 1
        return "cuda:0"

    def load():
        calls["load"] += 1

    node._resolve_yolo_device = resolve
    node._load_yolo_model = load

    assert node._initialize_yolo_runtime_if_enabled() is expected_initialized
    expected_count = int(expected_initialized)
    assert calls == {"resolve": expected_count, "load": expected_count}


@pytest.mark.parametrize(
    ("mode", "hz", "path", "disabled", "expected_topics"),
    (
        ("bbox_yolo", 40.0, "/tmp/model.pt", False, 2),
        ("bbox_yolo", 40.0, "", False, 0),
        ("bbox_yolo", 0.0, "/tmp/model.pt", False, 0),
        ("bbox_yolo", 40.0, "/tmp/model.pt", True, 0),
        ("ideal_reference_point", 40.0, "", False, 2),
    ),
)
def test_image_subscription_creation_call_counts(
    mode, hz, path, disabled, expected_topics
):
    node = _node(mode, hz, path, disabled)
    topics = []

    def create_subscription(_message_type, topic, _callback, _qos):
        topics.append(topic)
        return topic

    node.create_subscription = create_subscription

    assert node._create_camera_image_subscriptions_if_enabled(object()) is bool(
        expected_topics
    )
    assert topics == (["/left_camera/image", "/right_camera/image"] if expected_topics else [])
    assert len(node.image_subscriptions) == expected_topics


@pytest.mark.parametrize(
    ("mode", "hz", "path", "disabled", "expected_starts"),
    (
        ("bbox_yolo", 40.0, "/tmp/model.pt", False, 1),
        ("bbox_yolo", 40.0, "", False, 0),
        ("bbox_yolo", 0.0, "/tmp/model.pt", False, 0),
        ("bbox_yolo", 40.0, "/tmp/model.pt", True, 0),
        ("ideal_reference_point", 40.0, "", False, 1),
    ),
)
def test_worker_start_call_counts(
    monkeypatch, mode, hz, path, disabled, expected_starts
):
    node = _node(mode, hz, path, disabled)
    starts = []

    class FakeThread:
        def __init__(self, *, target, daemon):
            self.target = target
            self.daemon = daemon

        def start(self):
            starts.append(self.target)

    monkeypatch.setattr(node_module.threading, "Thread", FakeThread)

    assert node._start_yolo_worker_if_enabled() is bool(expected_starts)
    assert len(starts) == expected_starts
    assert (node.yolo_thread is not None) is bool(expected_starts)


@pytest.mark.parametrize(
    ("mode", "expected_warmups"),
    (("bbox_yolo", 1), ("ideal_reference_point", 0)),
)
def test_worker_warmup_is_bbox_only(mode, expected_warmups):
    node = _node(mode, 40.0, "/tmp/model.pt", False)
    node.yolo_condition = threading.Condition()
    node.measurement_pair_queue = deque()
    node.gt_alignment_ready_queue = deque()
    node.drain_requested = True
    node.drain_complete = False
    node.shutdown_event = threading.Event()
    warmups = []
    node._warm_yolo_model = lambda: warmups.append(mode)

    node._yolo_worker_loop()

    assert len(warmups) == expected_warmups
    assert node.drain_complete is True
