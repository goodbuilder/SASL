from pathlib import Path
import sys
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.ground_truth_provider import GroundTruthProvider


def _record(stamp, epoch=0):
    return SimpleNamespace(stamp_ns=stamp, epoch=epoch)


def test_small_future_lead_is_buffered_then_drained_in_timestamp_order():
    provider = GroundTruthProvider(capacity=4, reorder_lead_limit_ns=20)
    accepted = []
    provider.route(_record(110), epoch=0, latest_clock_ns=100, accept=accepted.append)
    provider.route(_record(105), epoch=0, latest_clock_ns=100, accept=accepted.append)
    assert accepted == []
    provider.drain(110, epoch=0, accept=accepted.append)
    assert [record.stamp_ns for record in accepted] == [105, 110]


def test_old_epoch_and_excessive_future_records_are_rejected():
    provider = GroundTruthProvider(reorder_lead_limit_ns=20)
    accepted = []
    provider.route(_record(90, epoch=1), epoch=2, latest_clock_ns=100, accept=accepted.append)
    provider.route(_record(121, epoch=2), epoch=2, latest_clock_ns=100, accept=accepted.append)
    assert accepted == []
    assert provider.stale_count == 2
    assert provider.quarantine_rejected_count == 1


def test_reset_quarantine_requires_contiguous_post_floor_samples():
    provider = GroundTruthProvider(reorder_lead_limit_ns=20)
    provider.reset(100, 10)
    accepted = []
    provider.route(_record(111), epoch=0, latest_clock_ns=111, accept=accepted.append)
    assert provider.quarantine_active is True
    provider.route(_record(112), epoch=0, latest_clock_ns=112, accept=accepted.append)
    assert provider.quarantine_active is False
    assert provider.quarantine_release_count == 1
    assert [record.stamp_ns for record in accepted] == [111, 112]
