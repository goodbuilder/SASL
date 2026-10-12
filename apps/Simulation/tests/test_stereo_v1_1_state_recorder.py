"""Raw GT and PX4 state JSONL preserve distinct frames and clock domains."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace as Obj


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.state_recorder import StateRecordWriter, _expected_stop_exception


def _gt(stamp_ns: int):
    return Obj(
        header=Obj(frame_id="gazebo_world_enu", stamp=Obj(sec=stamp_ns // 1_000_000_000,
                                                               nanosec=stamp_ns % 1_000_000_000)),
        pose=Obj(position=Obj(x=2.0, y=415.0, z=1.0),
                 orientation=Obj(x=0.0, y=0.0, z=0.0, w=1.0)),
    )


def test_separate_raw_files_clock_epoch_and_gt_gap_flag(tmp_path: Path):
    writer = StateRecordWriter(tmp_path)
    writer.record_clock(1_000_000_000, 50)
    writer.record_gt(_gt(1_000_000_000), 51)
    writer.record_clock(1_020_000_000, 70)
    writer.record_gt(_gt(1_020_000_000), 71)
    writer.record_clock(1_060_000_000, 110)
    writer.record_gt(_gt(1_060_000_000), 111)
    writer.record_px4_local(Obj(
        timestamp=1_000_000, x=-3.0, y=4.0, z=-5.0,
        vx=0.1, vy=0.2, vz=-0.3, heading=3.14,
        xy_valid=True, z_valid=True, xy_reset_counter=4, z_reset_counter=2,
    ), 112)
    writer.record_px4_attitude(Obj(timestamp=1_000_001, q=[1.0, 0.0, 0.0, 0.0]), 113)
    writer.record_camera("left", '{"frame":"optical_rdf","side":"left"}', 114)
    writer.record_camera("right", '{"frame":"optical_rdf","side":"right"}', 115)
    result = writer.close()
    assert result["status"] == "STATE_CAPTURE_COMPLETE"
    assert result["gt_gap_ge_40ms_count"] == 1
    assert result["max_gt_gap_ns"] == 40_000_000
    gt = json.loads((tmp_path / "ground_truth.jsonl").read_text().splitlines()[0])
    local = json.loads((tmp_path / "px4_local_position.jsonl").read_text().splitlines()[0])
    attitude = json.loads((tmp_path / "px4_attitude.jsonl").read_text().splitlines()[0])
    assert gt["position_world_enu_m"] == [2.0, 415.0, 1.0]
    assert gt["quaternion_order"] == "xyzw"
    assert local["position_local_ned_m"] == [-3.0, 4.0, -5.0]
    assert local["axis_contract"] == "px4_local_ned:north,east,down"
    assert attitude["quaternion_order"] == "wxyz"
    camera = json.loads((tmp_path / "camera_truth.json").read_text())
    assert set(camera["camera_payloads"]) == {"left", "right"}


def test_second_state_recorder_cannot_overwrite_raw_gt(tmp_path: Path):
    writer = StateRecordWriter(tmp_path)
    writer.record_clock(10, 10)
    writer.close()
    previous = (tmp_path / "ground_truth.jsonl").read_bytes()
    try:
        StateRecordWriter(tmp_path)
    except FileExistsError:
        pass
    else:
        raise AssertionError("second writer must reject the existing attempt")
    assert (tmp_path / "ground_truth.jsonl").read_bytes() == previous


def test_sigint_ros_external_shutdown_is_expected_only_after_stop_request():
    class ExternalShutdownException(Exception):
        pass
    class RCLError(Exception):
        pass

    assert _expected_stop_exception(ExternalShutdownException(), True)
    assert _expected_stop_exception(KeyboardInterrupt(), True)
    assert _expected_stop_exception(RCLError("failed to initialize wait set: context is not valid"), True)
    assert not _expected_stop_exception(ExternalShutdownException(), False)
    assert not _expected_stop_exception(RCLError("unrelated ROS writer error"), True)
    assert not _expected_stop_exception(RuntimeError("writer failed"), True)
