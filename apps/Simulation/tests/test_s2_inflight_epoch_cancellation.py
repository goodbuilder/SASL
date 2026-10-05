from __future__ import annotations

import threading
import sys
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import (
    SimulationEpochTracker,
    StereoImagePair,
)
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


class Logger:
    def warning(self, _message):
        pass


def test_completed_old_epoch_inference_is_discarded_at_commit_linearization_point() -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.lock = threading.Lock()
    node.yolo_condition = threading.Condition(node.lock)
    node.simulation_epoch = SimulationEpochTracker()
    node.simulation_epoch.observe_clock(100)
    node.inflight_pair_id = 7
    node.inflight_pair_epoch = 0
    node.yolo_inference_in_progress = True
    node.discard_measurement_commits = False
    node.old_epoch_result_discard_count = 0
    node.accepted_pair_discard_count = 0
    node.measurement_results_by_pair_id = {}
    node.get_logger = lambda: Logger()
    pair = StereoImagePair(7, 100, 100, 100, 0, "exact", object(), object(), 0)

    barrier = threading.Barrier(2)
    released = threading.Event()

    def inference():
        barrier.wait()
        released.wait(2.0)
        with node.yolo_condition:
            node._commit_pair_measurement_if_current(pair, {"finished": 0.0})

    worker = threading.Thread(target=inference)
    worker.start()
    barrier.wait()
    with node.yolo_condition:
        event = node.simulation_epoch.observe_clock(1)
        assert event.reset
    released.set()
    worker.join(2.0)

    assert not worker.is_alive()
    assert node.old_epoch_result_discard_count == 1
    assert node.measurement_results_by_pair_id == {}
    assert node.inflight_pair_id is None
    assert node.inflight_pair_epoch is None


def test_current_epoch_result_can_reach_commit_body() -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.lock = threading.Lock()
    node.yolo_condition = threading.Condition(node.lock)
    node.simulation_epoch = SimpleNamespace(epoch=1)
    node.inflight_pair_id = 8
    node.inflight_pair_epoch = 1
    node.yolo_inference_in_progress = True
    node.discard_measurement_commits = False
    node.old_epoch_result_discard_count = 0
    node.accepted_pair_discard_count = 0
    node.measurement_processed_count = 0
    node._drop_pair_state_locked = lambda *_args: None
    pair = StereoImagePair(8, 1, 1, 1, 0, "exact", object(), object(), 1)
    with node.yolo_condition:
        assert node._commit_pair_measurement_if_current(
            pair, {"left_image": None, "right_image": None}
        ) is None
    assert node.old_epoch_result_discard_count == 0
    assert node.measurement_processed_count == 0
    assert node.accepted_pair_discard_count == 1
