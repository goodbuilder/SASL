from pathlib import Path
import sys
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.pair_work_scheduler import PairWorkScheduler


def _pair(pair_id):
    return SimpleNamespace(pair_id=pair_id)


def test_scheduler_alternates_measurement_and_alignment_when_both_ready():
    scheduler = PairWorkScheduler()
    scheduler.measurements.extend((_pair(1), _pair(2)))
    scheduler.alignments.extend(((_pair(3), "a3"), (_pair(4), "a4")))

    assert scheduler.pop_next()[0] == "measurement"
    assert scheduler.pop_next()[0] == "alignment"
    assert scheduler.pop_next()[0] == "measurement"
    assert scheduler.pop_next()[0] == "alignment"
    assert scheduler.empty is True


def test_remove_pair_removes_both_work_classes_without_reordering_others():
    scheduler = PairWorkScheduler()
    scheduler.measurements.extend((_pair(1), _pair(2), _pair(3)))
    scheduler.alignments.extend(((_pair(2), "a2"), (_pair(3), "a3")))

    scheduler.remove_pair(2)

    assert [pair.pair_id for pair in scheduler.measurements] == [1, 3]
    assert [item[0].pair_id for item in scheduler.alignments] == [3]


def test_clear_reports_all_dropped_work_and_resets_fairness():
    scheduler = PairWorkScheduler()
    scheduler.measurements.append(_pair(1))
    scheduler.alignments.append((_pair(2), "a2"))
    scheduler.prefer_alignment_next = True

    assert scheduler.clear() == 2
    assert scheduler.empty is True
    assert scheduler.prefer_alignment_next is False
