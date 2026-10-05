from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.pair_evaluation_registry import PairEvaluationRegistry


def test_join_is_ready_only_when_all_three_inputs_exist():
    registry = PairEvaluationRegistry(4)
    registry.pairs[7] = "pair"
    assert registry.store_measurement(7, "measurement") is True
    assert registry.ready(7) is None
    assert registry.store_alignment(7, "world_enu_alignment") is True
    assert registry.ready(7) == ("pair", "measurement", "world_enu_alignment")


def test_finalize_atomically_removes_join_and_creates_tombstone():
    registry = PairEvaluationRegistry(4)
    registry.pairs[3] = "pair"
    registry.store_measurement(3, "measurement")
    registry.store_alignment(3, "alignment")

    assert registry.finalize(3) == ("pair", "measurement", "alignment")
    assert registry.is_terminal(3) is True
    assert registry.ready(3) is None
    assert registry.store_measurement(3, "late") is False


def test_failed_tombstone_prevents_late_alignment_resurrection():
    registry = PairEvaluationRegistry(4)
    registry.pairs[9] = "pair"
    registry.finalize(9, failed=True)
    registry.pairs[9] = "late-recreated-pair"

    assert registry.store_alignment(9, "late-alignment") is False
    assert 9 in registry.failed_ids
