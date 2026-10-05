import csv
import json

from Workstation.workstation_core.measurement_artifacts import reserve_measurement_context
from Workstation.workstation_core.measurement_data_store import LiveDataRows, MeasurementDataStore


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_one_writer_routes_late_rows_to_original_run(tmp_path):
    store = MeasurementDataStore(tmp_path, timestamped_filenames=True)
    outside_path = store.path_for("events")
    assert store.enqueue("events", {"event_type": "outside"})
    a = reserve_measurement_context(tmp_path, "a", 1, 1)
    b = reserve_measurement_context(tmp_path, "b", 2, 2)
    store.register_measurement(a)
    first = LiveDataRows(store, "measurements")
    assert first.append({"n": 1})
    store.finish_measurement("a")
    store.register_measurement(b)
    second = LiveDataRows(store, "measurements")
    assert second.append({"n": 2})
    assert first.replace(0, {"n": 3})
    assert store.enqueue("events", {"measurement_id": "a", "event_type": "late-video"})
    assert store.close(timeout_s=3)
    assert records(outside_path)[0]["event_type"] == "outside"
    assert [r["n"] for r in records(a.stream_path("measurements"))] == [1, 3]
    assert [r["n"] for r in records(b.stream_path("measurements"))] == [2]
    assert records(a.stream_path("events"))[0]["event_type"] == "late-video"
    for context in (a, b):
        path = context.stream_path("measurements")
        with path.with_suffix(".csv").open(newline="") as handle:
            mirrored = [json.loads(r["record_json"]) for r in csv.DictReader(handle)]
        assert mirrored == records(path)
        assert all(r["measurement_started_utc_ns"] == context.started_utc_ns for r in mirrored)


def test_csv_retry_restores_rows_that_outlived_retry_reserve(tmp_path):
    store = MeasurementDataStore(tmp_path, retry_reserve_capacity=1)
    target = store.csv_path_for("events")
    target.write_text("foreign-header\n")
    for index in range(5):
        assert store.enqueue("events", {"n": index, "nested": {"ns": 1788000000000000001}})
    store.export_checkpoint(timeout_s=3)
    assert store.snapshot()["streams"]["events"]["csv"]["state"] == "FAILED"
    target.unlink()  # operator removes the conflicting file
    store.retry_failed()
    import time
    deadline = time.monotonic() + 3
    while store.snapshot()["streams"]["events"]["csv"]["written_records"] != 5 and time.monotonic() < deadline:
        time.sleep(0.02)
    assert store.close(timeout_s=3)
    with target.open(newline="") as handle:
        mirrored = [json.loads(row["record_json"]) for row in csv.DictReader(handle)]
    assert mirrored == records(store.path_for("events"))
    assert len(mirrored) == 5


def test_late_payload_allocates_index_in_its_original_file(tmp_path):
    store = MeasurementDataStore(tmp_path, timestamped_filenames=True)
    a = reserve_measurement_context(tmp_path, "a", 1, 1)
    b = reserve_measurement_context(tmp_path, "b", 2, 2)
    store.register_measurement(a)
    first = LiveDataRows(store, "method_results")
    for n in range(3):
        assert first.append({"n": n})
    store.register_measurement(b)
    current = LiveDataRows(store, "method_results")
    assert current.append({"n": 100})
    assert current.append({"n": 3, "runtime_generation": "a"})
    assert current.append({"n": 101})
    assert store.close(timeout_s=3)
    assert [r["_record_index"] for r in records(a.stream_path("method_results"))] == [0, 1, 2, 3]
    assert [r["n"] for r in records(a.stream_path("method_results"))] == [0, 1, 2, 3]
    assert [r["_record_index"] for r in records(b.stream_path("method_results"))] == [0, 1]


def test_measurement_original_preserves_nested_source_identity_and_long_values(tmp_path):
    store = MeasurementDataStore(tmp_path, timestamped_filenames=True)
    context = reserve_measurement_context(tmp_path, "a", 1, 1)
    store.register_measurement(context)
    original = {"session_id": "device-provenance", "ns": 1788000000000000001, "text": "원본\n\"value\"" * 5000}
    assert store.enqueue("events", {"event_type": "raw", "payload": original, "session_id": "legacy-owner"})
    assert store.close(timeout_s=3)
    path = context.stream_path("events")
    saved = records(path)[0]
    assert saved["payload"] == original
    assert saved["source_session_membership"]["session_id"] == "legacy-owner"
    csv.field_size_limit(1024 * 1024)
    with path.with_suffix(".csv").open(newline="") as handle:
        assert json.loads(next(csv.DictReader(handle))["record_json"]) == saved
