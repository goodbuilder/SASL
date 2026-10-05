from dataclasses import asdict
import json

from Workstation.workstation_core.measurement_data_store import MeasurementDataStore
from Workstation.workstation_core.measurement_journal import load_measurement_record
from Workstation.workstation_core.measurement_record import MeasurementRegistry
from Workstation.tests.measurement_fixtures import (
    MEASUREMENT_ID, checkerboard_attempt, detection_observation, frozen_snapshot, outcome_for,
)


def test_journal_rebuild_preserves_inputs_results_and_recorded_rt(tmp_path, monkeypatch):
    store = MeasurementDataStore(tmp_path, timestamped_filenames=True)
    registry = MeasurementRegistry(
        data_root_provider=lambda: tmp_path, per_run_storage=True,
        context_started=store.register_measurement, context_stopped=store.finish_measurement,
        journal_sink=lambda payload: store.enqueue("measurement.journal", payload, priority=True),
    )
    store.add_enqueue_observer(registry.observe_store_record)
    record = registry.begin(MEASUREMENT_ID, started_utc_ns=1_000_000_000, started_monotonic_ns=10)
    registry.observe_detection("left", detection_observation("left", 340, 1.0))
    registry.observe_detection("right", detection_observation("right", 300, 1.01))
    record.add_alignment_row({"method_id": "checkerboard_stereo", "source": "profile-test"})
    attempt = checkerboard_attempt(1)
    pair = frozen_snapshot(attempt)
    record.register_pair(pair.canonical_payload(), submitted_method_ids=("checkerboard_stereo",), not_attempted={"terrain_pnp": "no input"})
    registry.route_outcome(outcome_for(attempt, pair))
    registry.stop(MEASUREMENT_ID, stopped_utc_ns=9_000_000_000, stopped_monotonic_ns=9_000_000_000)
    assert store.close(timeout_s=3)
    before = record.snapshot()
    # Rebuilding must use the originally recorded geometry, not run current solvers.
    import Workstation.workstation_core.measurement_record as module
    monkeypatch.setattr(module, "derive_method_result", lambda *_: (_ for _ in ()).throw(AssertionError("recalculated")))
    context = registry.context_for(MEASUREMENT_ID)
    rebuilt = load_measurement_record(context.stream_path("measurement_journal")).snapshot()
    assert json.dumps(asdict(before)["pairs"], sort_keys=True) == json.dumps(asdict(rebuilt)["pairs"], sort_keys=True)
    assert json.dumps(asdict(before)["method_results"], sort_keys=True) == json.dumps(asdict(rebuilt)["method_results"], sort_keys=True)
    assert asdict(before)["detections"] == asdict(rebuilt)["detections"]
    assert before.alignment_rows == rebuilt.alignment_rows
    assert before.started_utc_ns == rebuilt.started_utc_ns
    assert rebuilt.storage["raw_read_errors"] == 0


def test_interrupted_journal_is_read_only_and_marks_incomplete(tmp_path):
    store = MeasurementDataStore(tmp_path, timestamped_filenames=True)
    registry = MeasurementRegistry(data_root_provider=lambda: tmp_path, per_run_storage=True,
        context_started=store.register_measurement,
        journal_sink=lambda payload: store.enqueue("measurement.journal", payload, priority=True))
    registry.begin("crash", started_utc_ns=1, started_monotonic_ns=1)
    assert store.close(timeout_s=3)
    path = registry.context_for("crash").stream_path("measurement_journal")
    with path.open("ab") as handle:
        handle.write(b'{"incomplete":')
    original = path.read_bytes()
    record = load_measurement_record(path)
    assert record.state == "INTERRUPTED"
    assert record.storage_snapshot()["raw_read_errors"] >= 1
    assert path.read_bytes() == original
