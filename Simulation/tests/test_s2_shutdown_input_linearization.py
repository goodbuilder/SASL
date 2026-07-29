from __future__ import annotations

import sys
import threading
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

import stereo_ground_sim.stereo_distance_node as node_module
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode
from stereo_ground_sim.time_alignment import (
    GroundTruthPositionBuffer,
    PendingPairGroundTruthAligner,
    SimulationEpochTracker,
    StereoImagePairer,
)


class Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, _message):
        self.warnings.append(str(_message))


def _node() -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.lock = threading.Lock()
    node.yolo_condition = threading.Condition(node.lock)
    node.accepting_input = True
    node.input_rejected_after_shutdown_count = 0
    node.simulation_epoch = SimulationEpochTracker()
    node.simulation_epoch.observe_clock(10)
    node.image_pairer = StereoImagePairer()
    node.gt_position_buffer = GroundTruthPositionBuffer()
    node.pending_gt_aligner = PendingPairGroundTruthAligner(node.gt_position_buffer)
    node.measurement_pair_queue = deque()
    node.gt_alignment_ready_queue = deque()
    node.pairs_by_pair_id = {}
    node.measurement_queue_capacity = 32
    node.measurement_overrun_invalid = False
    node.measurement_queue_overflow_count = 0
    node.measurement_accepted_count = 0
    node.measurement_queue_depth_sample_count = 0
    node.measurement_queue_depth_sum = 0
    node.measurement_queue_depth_min = None
    node.measurement_queue_depth_max = 0
    node.stale_pre_reset_count = 0
    node.image_reorder_lead_limit_ns = 20_000_000
    node.logger = Logger()
    node.get_logger = lambda: node.logger
    return node


def _image_message(stamp_ns: int):
    sec, nanosec = divmod(int(stamp_ns), 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_callback_parsed_before_shutdown_is_rejected_inside_condition(monkeypatch, side) -> None:
    node = _node()
    parsed = threading.Event()

    def parsed_stamp(_message):
        parsed.set()
        return 10

    monkeypatch.setattr(node_module, "image_stamp_ns", parsed_stamp)
    callback = node._left_cb if side == "left" else node._right_cb

    with node.yolo_condition:
        thread = threading.Thread(target=callback, args=(SimpleNamespace(),))
        thread.start()
        assert parsed.wait(timeout=1.0)
        node.accepting_input = False
        node.shutdown_drain_status = "SUCCESS"

    thread.join(timeout=1.0)
    assert not thread.is_alive()
    assert node.shutdown_drain_status == "SUCCESS"
    assert node.image_pairer.left_received_count == 0
    assert node.image_pairer.right_received_count == 0
    assert node.image_pairer.paired_count == 0
    assert node.measurement_accepted_count == 0
    assert node.measurement_pair_queue == deque()
    assert node.pending_gt_aligner._pending == deque()
    assert node.input_rejected_after_shutdown_count == 1


def test_image_one_clock_tick_ahead_is_accepted() -> None:
    node = _node()

    node._left_cb(_image_message(10 + 16_000_000))

    assert node.image_pairer.left_received_count == 1
    assert node.stale_pre_reset_count == 0
    assert node.logger.warnings == []


def test_image_beyond_reorder_tolerance_is_rejected() -> None:
    node = _node()

    node._left_cb(_image_message(10 + 21_000_000))

    assert node.image_pairer.left_received_count == 0
    assert node.stale_pre_reset_count == 1
    assert "by 21000000 ns; ignored" in node.logger.warnings[-1]
