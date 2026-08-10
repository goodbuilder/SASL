from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.pose_source_provider import PoseSourceProvider


def test_attitude_is_normalized_and_preserves_wxyz_frd_to_ned_contract():
    provider = PoseSourceProvider()
    sample = provider.store_attitude(
        "left",
        SimpleNamespace(q=(2.0, 0.0, 0.0, 0.0), timestamp_sample=7, timestamp=6, quat_reset_counter=3),
        epoch=2,
        simulation_timestamp_ns=100,
        received_ns=200,
    )
    assert sample.valid is True
    assert sample.quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert sample.timestamp_us == 7
    assert sample.simulation_epoch == 2


def test_invalid_raw_gps_clears_fix_and_records_failure_provenance():
    provider = PoseSourceProvider()
    message = SimpleNamespace(
        timestamp_sample=1, timestamp=1, latitude_deg=float("nan"),
        longitude_deg=127.0, altitude_msl_m=10.0, eph=1.0, epv=2.0,
    )
    assert provider.store_raw_gps(
        "right", message, epoch=4, simulation_timestamp_ns=10, received_ns=20
    ) is None
    assert provider.raw_gps["right"] is None
    assert provider.raw_gps_failures["right"][1:] == (4, 20)


def test_pair_snapshot_is_a_shallow_immutable_view_of_capture_time_sources():
    provider = PoseSourceProvider()
    provider.attitudes["left"] = "left-attitude"
    snapshot = provider.snapshot(simulation_timestamp_ns=11, monotonic_ns=22, epoch=3)
    provider.attitudes["left"] = "new-attitude"
    assert snapshot["px4_attitudes"]["left"] == "left-attitude"
    assert snapshot["captured_simulation_timestamp_ns"] == 11
    assert snapshot["simulation_epoch"] == 3
