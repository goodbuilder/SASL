from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.research_terminal_coordinator import ResearchTerminalCoordinator


class Recorder:
    def __init__(self, results):
        self.results = iter(results)
        self.records = []
        self.overrun_invalid = True
        self.first_error_type = "queue_full"
        self.first_error_message = "capacity reached"

    def submit(self, record):
        self.records.append(record)
        return next(self.results)


def test_claim_allows_exactly_one_terminal_record_per_epoch_pair():
    coordinator = ResearchTerminalCoordinator(Recorder([True]))
    assert coordinator.claim(2, 7) is True
    assert coordinator.claim(2, 7) is False
    assert coordinator.claim(3, 7) is True


def test_submit_tracks_success_and_contains_first_recorder_failure():
    recorder = Recorder([True, False, False])
    coordinator = ResearchTerminalCoordinator(recorder)

    assert coordinator.submit_claimed({"pair_id": 1}) is True
    assert coordinator.submit_claimed({"pair_id": 2}) is False
    recorder.first_error_type = "later_error"
    recorder.first_error_message = "later"
    assert coordinator.submit_claimed({"pair_id": 3}) is False

    assert coordinator.terminal_record_count == 1
    assert coordinator.recording_error_count == 2
    assert coordinator.recording_overrun_invalid is True
    assert coordinator.first_error_type == "queue_full"
    assert coordinator.first_error_message == "capacity reached"
