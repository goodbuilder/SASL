"""Nominal 1 ms source-frame pairing and unmatched preservation."""

from __future__ import annotations

import sys
from pathlib import Path


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.bag_index import pair_frames


def _frame(side, epoch, stamp, number):
    return {
        "camera_id": side,
        "frame_id": f"{side}:{epoch}:{stamp}:{number}",
        "simulation_epoch": epoch,
        "image_timestamp_ns": stamp,
        "pair_id": None,
    }


def test_pairing_never_reuses_source_frame_and_keeps_outside_frames_unmatched():
    left = [_frame("left", 0, 1_000_000_000, 1),
            _frame("left", 0, 1_000_000_000, 2),
            _frame("left", 0, 2_000_000_000, 3),
            _frame("left", 1, 10_000_000, 4)]
    right = [_frame("right", 0, 1_000_000_000, 1),
             _frame("right", 0, 1_000_900_000, 2),
             _frame("right", 0, 2_001_000_001, 3),
             _frame("right", 0, 10_000_000, 4)]
    pairs = pair_frames(left, right)
    assert len(pairs) == 2
    assert [abs(pair["pair_delta_ns"]) for pair in pairs] == [0, 900_000]
    assert len({pair["left_frame_id"] for pair in pairs}) == len(pairs)
    assert len({pair["right_frame_id"] for pair in pairs}) == len(pairs)
    assert left[2]["pair_id"] is None and left[3]["pair_id"] is None
    assert right[2]["pair_id"] is None and right[3]["pair_id"] is None


def test_exact_one_millisecond_pair_is_inclusive():
    left = [_frame("left", 0, 1_000_000_000, 1)]
    right = [_frame("right", 0, 1_001_000_000, 1)]
    pairs = pair_frames(left, right)
    assert len(pairs) == 1
    assert pairs[0]["pair_delta_ns"] == 1_000_000
