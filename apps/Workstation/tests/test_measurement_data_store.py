from __future__ import annotations

import csv
import json
import queue
import re
import threading
import time
from pathlib import Path

from Workstation.workstation_core.measurement_data_store import (
    CSV_FIELDNAMES,
    CSV_RECORD_SCHEMA,
    CSV_STREAM_PATHS,
    LiveDataRows,
    MeasurementDataStore,
    allocate_timestamped_path,
    creation_timestamp_text,
    timestamped_csv_path,
    timestamped_stream_path,
)
from Workstation.data_storage_controller import DataStorageController


def _wait_for(predicate, timeout_s: float = 2.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition did not become true")


def test_creation_timestamp_is_local_offset_and_binary_paths_never_overwrite(tmp_path: Path) -> None:
    stamp = creation_timestamp_text(1_787_634_123_456_789_012)
    assert re.fullmatch(r"\d{8}_\d{6}_\d{9}[+-]\d{4}", stamp)
    assert stamp.endswith("+0900")

    first = allocate_timestamped_path(
        tmp_path,
        "videos",
        label="left",
        suffix=".mp4",
        utc_ns=1_787_634_123_456_789_012,
    )
    first.write_bytes(b"old")
    second = allocate_timestamped_path(
        tmp_path,
        "videos",
        label="left",
        suffix=".mp4",
        utc_ns=1_787_634_123_456_789_012,
    )
    assert second != first
    assert second.name.endswith("-01.mp4")
    assert first.read_bytes() == b"old"


def test_fixed_stream_appends_across_restarts_and_removes_session_membership(tmp_path: Path) -> None:
    first = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert first.enqueue(
        "measurements",
        {
            "value": 1,
            "session_id": "old-session",
            "parent_session_id": "old-parent",
            "measurement_package_id": "m000001",
            "payload": {
                "session_id": "nested-old-session",
                "kept": "nested-value",
            },
            "coordinate_contract": "baseline_center_frd",
        },
    )
    assert first.close(timeout_s=2.0)

    second = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert second.enqueue("measurements", {"value": 2})
    assert second.close(timeout_s=2.0)

    rows = [json.loads(line) for line in (tmp_path / "measurements.jsonl").read_text().splitlines()]
    assert [row["value"] for row in rows] == [1, 2]
    assert rows[0]["coordinate_contract"] == "baseline_center_frd"
    assert "session_id" not in rows[0]
    assert "parent_session_id" not in rows[0]
    assert "measurement_package_id" not in rows[0]
    assert rows[0]["payload"] == {"kept": "nested-value"}
    assert all(isinstance(row["recorded_utc_ns"], int) for row in rows)
    assert all(isinstance(row["recorded_monotonic_ns"], int) for row in rows)


def test_csv_mirror_preserves_canonical_record_and_envelope(tmp_path: Path) -> None:
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert store.enqueue(
        "measurements",
        {
            "runtime_generation": "run-1",
            "measurement_id": "measurement-1",
            "event_type": "measurement_result",
            "sample_type": "pixhawk_rtk_live",
            "status": "VALID",
            "coordinate_contract": "baseline_center_frd",
            "baseline_center_forward_m": 12.5,
            "nested": {"한국어": "값", "values": [1, 2, 3]},
        },
    )
    assert store.close(timeout_s=2.0)

    jsonl_rows = [
        json.loads(line)
        for line in (tmp_path / "measurements.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    with (tmp_path / "measurements.csv").open(
        "r", newline="", encoding="utf-8"
    ) as handle:
        reader = csv.DictReader(handle)
        csv_rows = list(reader)

    assert reader.fieldnames == list(CSV_FIELDNAMES)
    assert len(jsonl_rows) == len(csv_rows) == 1
    assert json.loads(csv_rows[0]["record_json"]) == jsonl_rows[0]
    assert csv_rows[0]["csv_schema"] == CSV_RECORD_SCHEMA
    assert csv_rows[0]["recorded_utc_ns"] == str(jsonl_rows[0]["recorded_utc_ns"])
    assert csv_rows[0]["recorded_monotonic_ns"] == str(
        jsonl_rows[0]["recorded_monotonic_ns"]
    )
    assert csv_rows[0]["runtime_generation"] == "run-1"
    assert csv_rows[0]["measurement_id"] == "measurement-1"
    assert csv_rows[0]["event_type"] == "measurement_result"
    assert csv_rows[0]["sample_type"] == "pixhawk_rtk_live"
    assert csv_rows[0]["status"] == "VALID"
    mirror = store.snapshot()["streams"]["measurements"]["csv"]
    assert mirror["path"] == str(tmp_path / "measurements.csv")
    assert mirror["state"] == "ACTIVE"
    assert mirror["written_records"] == 1


def test_csv_stream_paths_are_same_stem_for_every_fixed_stream() -> None:
    assert set(CSV_STREAM_PATHS) == set(
        {
            "events",
            "measurements",
            "method_evaluations",
            "camera.left",
            "camera.right",
            "px4.left",
            "px4.right",
            "px4.drone",
            "gnss.base",
            "gnss.baseline.center",
            "geometry.comparisons",
            "capture.events",
            "alignment",
            "artifacts",
            "application.log",
            "native.frames",
        }
    )
    assert all(path.suffix == ".csv" for path in CSV_STREAM_PATHS.values())
    assert timestamped_csv_path(
        "measurements", "20260904_120000_000000001+0900"
    ).name == "measurements_20260904_120000_000000001+0900.csv"


def test_csv_mirror_appends_across_store_restarts(tmp_path: Path) -> None:
    first = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert first.enqueue("events", {"event_type": "first", "value": 1})
    assert first.close(timeout_s=2.0)

    second = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert second.enqueue("events", {"event_type": "second", "value": 2})
    assert second.close(timeout_s=2.0)

    with (tmp_path / "events.csv").open(
        "r", newline="", encoding="utf-8"
    ) as handle:
        rows = list(csv.DictReader(handle))
    assert [row["event_type"] for row in rows] == ["first", "second"]
    assert (tmp_path / "events.csv").read_text(encoding="utf-8").count(
        "csv_schema"
    ) == 1


def test_csv_failure_does_not_block_jsonl_and_retry_recovers(tmp_path: Path) -> None:
    bad_csv_path = tmp_path / "events.csv"
    bad_csv_path.mkdir()
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert store.enqueue("events", {"event_type": "retry-me"})
    assert store.enqueue("measurements", {"result": "VALID"})
    _wait_for(
        lambda: store.snapshot()["streams"]["events"]["csv"]["state"] == "FAILED"
        and store.snapshot()["streams"]["measurements"]["csv"]["written_records"] == 1
    )

    failed = store.snapshot()
    assert failed["streams"]["events"]["written_records"] == 1
    assert failed["streams"]["events"]["csv"]["retry_pending"] == 1
    assert failed["streams"]["events"]["csv"]["dropped_records"] == 0
    assert failed["state"] == "DEGRADED"
    assert (tmp_path / "events.jsonl").is_file()

    bad_csv_path.rmdir()
    store.retry_failed()
    _wait_for(
        lambda: store.snapshot()["streams"]["events"]["csv"]["written_records"] == 1
    )
    assert store.close(timeout_s=2.0)

    with (tmp_path / "events.csv").open(
        "r", newline="", encoding="utf-8"
    ) as handle:
        [row] = list(csv.DictReader(handle))
    assert row["event_type"] == "retry-me"
    assert json.loads(row["record_json"])["event_type"] == "retry-me"


def test_invalid_csv_header_is_not_replaced_and_jsonl_remains_authoritative(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "events.csv"
    csv_path.write_text("foreign,header\nkeep,this\n", encoding="utf-8")
    original = csv_path.read_bytes()
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert store.enqueue("events", {"event_type": "header-check"})
    _wait_for(
        lambda: store.snapshot()["streams"]["events"]["csv"]["state"] == "FAILED"
    )
    assert store.snapshot()["streams"]["events"]["written_records"] == 1
    assert csv_path.read_bytes() == original
    assert store.close(timeout_s=2.0)


def test_csv_retry_reserve_is_bounded_without_rejecting_jsonl_enqueue(
    tmp_path: Path,
) -> None:
    bad_csv_path = tmp_path / "events.csv"
    bad_csv_path.mkdir()
    store = MeasurementDataStore(
        tmp_path,
        fsync_interval_s=10.0,
        retry_reserve_capacity=2,
    )
    assert store.enqueue("events", {"sequence": 1})
    _wait_for(
        lambda: store.snapshot()["streams"]["events"]["csv"]["state"] == "FAILED"
    )
    assert store.enqueue("events", {"sequence": 2})
    assert store.enqueue("events", {"sequence": 3})
    _wait_for(
        lambda: store.snapshot()["streams"]["events"]["csv"]["retry_pending"] == 2
    )

    snapshot = store.snapshot()
    assert snapshot["streams"]["events"]["written_records"] == 3
    assert snapshot["streams"]["events"]["csv"]["dropped_records"] == 1
    assert snapshot["csv_dropped_records"] == 1
    assert store.close(timeout_s=2.0)


def test_timestamped_stream_files_use_one_stamp_and_do_not_replace_prior_set(
    tmp_path: Path,
) -> None:
    first_stamp = "20260903_120000_000000001+0900"
    second_stamp = first_stamp
    first = MeasurementDataStore(
        tmp_path,
        fsync_interval_s=0.05,
        timestamped_filenames=True,
        file_stamp=first_stamp,
    )
    assert first.enqueue("measurements", {"value": 1})
    first_path = first.path_for("measurements")
    assert first_path == tmp_path / timestamped_stream_path("measurements", first_stamp)
    assert first.close(timeout_s=2.0)

    second = MeasurementDataStore(
        tmp_path,
        fsync_interval_s=0.05,
        timestamped_filenames=True,
        file_stamp=second_stamp,
    )
    assert second.enqueue("measurements", {"value": 2})
    second_path = second.path_for("measurements")
    assert second_path != first_path
    assert second_path.name.endswith("-01.jsonl")
    assert second.close(timeout_s=2.0)

    assert first_path.with_suffix(".csv").is_file()
    assert second_path.with_suffix(".csv").is_file()
    assert first_path.read_text(encoding="utf-8").count("\n") == 1
    assert second_path.read_text(encoding="utf-8").count("\n") == 1
    assert not (tmp_path / "measurements.jsonl").exists()


def test_gnss_baseline_center_is_a_fixed_best_effort_bulk_stream(tmp_path: Path) -> None:
    first = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert first.enqueue(
        "gnss.baseline.center",
        {
            "geometry_state": "CURRENT",
            "coordinate_frame": "gnss_baseline_center_local_level_frd",
            "gnss_baseline_center_forward_m": 10.0,
        },
    )
    assert first.close(timeout_s=2.0)

    second = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert second.enqueue(
        "gnss.baseline.center",
        {
            "geometry_state": "STALE",
            "coordinate_frame": "gnss_baseline_center_local_level_frd",
            "gnss_baseline_center_forward_m": 11.0,
        },
    )
    assert second.close(timeout_s=2.0)

    path = tmp_path / "gnss_baseline_center.jsonl"
    rows = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
    ]
    assert [row["geometry_state"] for row in rows] == ["CURRENT", "STALE"]
    snapshot = second.snapshot()["streams"]["gnss.baseline.center"]
    assert snapshot["priority"] is False
    assert snapshot["written_records"] == 1


def test_failed_gnss_baseline_center_stream_does_not_block_measurements(tmp_path: Path) -> None:
    bad_path = tmp_path / "gnss_baseline_center.jsonl"
    bad_path.mkdir()
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert store.enqueue("gnss.baseline.center", {"geometry_state": "CURRENT"})
    assert store.enqueue("measurements", {"result": "VALID"})
    _wait_for(
        lambda: store.snapshot()["streams"]["gnss.baseline.center"]["state"]
        == "FAILED"
    )
    _wait_for(
        lambda: store.snapshot()["streams"]["measurements"]["written_records"] == 1
    )
    assert json.loads((tmp_path / "measurements.jsonl").read_text())["result"] == "VALID"
    assert store.close(timeout_s=2.0)


def test_live_rows_append_after_store_close_is_rejected_without_pending_queue(
    tmp_path: Path,
) -> None:
    store = MeasurementDataStore(tmp_path)
    rows = LiveDataRows(store, "measurements", priority=True)
    assert store.close(timeout_s=2.0)

    assert rows.append({"result": "LATE"}) is False

    snapshot = store.snapshot()
    assert snapshot["lifecycle_state"] == "CLOSED"
    assert snapshot["lifecycle_rejected_records"] == 1
    assert snapshot["priority_pending"] == 0
    assert snapshot["bulk_pending"] == 0
    assert snapshot["streams"]["measurements"]["dropped_records"] == 1
    assert snapshot["streams"]["measurements"]["last_error"].endswith("CLOSED")


def test_store_snapshot_tracks_open_closing_closed_and_rejects_during_close(
    tmp_path: Path,
) -> None:
    store = MeasurementDataStore(tmp_path)
    entered = threading.Event()
    release = threading.Event()
    original_write = store._write

    def blocked_write(item):
        entered.set()
        release.wait(timeout=2.0)
        original_write(item)

    store._write = blocked_write
    assert store.snapshot()["lifecycle_state"] == "OPEN"
    assert store.enqueue("events", {"event_type": "before-close"})
    assert entered.wait(timeout=1.0)

    assert store.close(timeout_s=0.01) is False
    closing = store.snapshot()
    assert closing["lifecycle_state"] == "CLOSING"
    assert closing["writer_alive"] is True
    assert store.enqueue("events", {"event_type": "during-close"}) is False
    assert store.snapshot()["lifecycle_rejected_records"] == 1

    release.set()
    assert store.close(timeout_s=2.0)
    closed = store.snapshot()
    assert closed["lifecycle_state"] == "CLOSED"
    assert closed["writer_alive"] is False
    assert closed["priority_pending"] == 0
    assert closed["bulk_pending"] == 0
    assert closed["streams"]["events"]["written_records"] == 1
    assert closed["streams"]["events"]["dropped_records"] == 1


def test_enqueue_and_close_are_serialized_without_stranding_admitted_record(
    tmp_path: Path,
) -> None:
    store = MeasurementDataStore(tmp_path, priority_capacity=2)
    enqueue_entered = threading.Event()
    enqueue_release = threading.Event()
    close_done = threading.Event()

    class PausingQueue(queue.Queue):
        def put_nowait(self, item):
            enqueue_entered.set()
            enqueue_release.wait(timeout=2.0)
            return super().put_nowait(item)

    store._priority = PausingQueue(maxsize=2)
    results: dict[str, bool] = {}
    producer = threading.Thread(
        target=lambda: results.setdefault(
            "enqueue",
            store.enqueue("events", {"event_type": "racing-close"}),
        )
    )
    producer.start()
    assert enqueue_entered.wait(timeout=1.0)

    def close_store() -> None:
        results["close"] = store.close(timeout_s=2.0)
        close_done.set()

    closer = threading.Thread(target=close_store)
    closer.start()
    time.sleep(0.05)
    assert close_done.is_set() is False

    enqueue_release.set()
    producer.join(timeout=2.0)
    closer.join(timeout=2.0)
    assert producer.is_alive() is False
    assert closer.is_alive() is False
    assert results == {"enqueue": True, "close": True}

    snapshot = store.snapshot()
    assert snapshot["lifecycle_state"] == "CLOSED"
    assert snapshot["lifecycle_rejected_records"] == 0
    assert snapshot["priority_pending"] == 0
    assert snapshot["bulk_pending"] == 0
    assert snapshot["streams"]["events"]["written_records"] == 1
    [record] = [
        json.loads(line)
        for line in (tmp_path / "events.jsonl").read_text().splitlines()
    ]
    assert record["event_type"] == "racing-close"


def test_torn_tail_is_preserved_as_one_bad_line_and_new_records_continue(tmp_path: Path) -> None:
    target = tmp_path / "events.jsonl"
    target.write_bytes(b'{"old":1}\n{"torn":')
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert store.enqueue("events", {"event_type": "measurement_started"})
    assert store.close(timeout_s=2.0)

    lines = target.read_bytes().splitlines()
    assert lines[0] == b'{"old":1}'
    assert lines[1] == b'{"torn":'
    assert json.loads(lines[2])["event_type"] == "measurement_started"


def test_one_failed_stream_does_not_block_priority_results_and_can_retry(tmp_path: Path) -> None:
    bad_path = tmp_path / "camera_left.jsonl"
    bad_path.mkdir()
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert store.enqueue("camera.left", {"frame": 1})
    assert store.enqueue("measurements", {"result": "VALID"})
    _wait_for(
        lambda: store.snapshot()["streams"]["camera.left"]["state"] == "FAILED"
    )
    _wait_for(
        lambda: store.snapshot()["streams"]["measurements"]["written_records"] == 1
    )
    assert store.snapshot()["state"] == "DEGRADED"
    assert json.loads((tmp_path / "measurements.jsonl").read_text())["result"] == "VALID"

    bad_path.rmdir()
    store.retry_failed()
    assert store.enqueue("camera.left", {"frame": 2})
    _wait_for(
        lambda: store.snapshot()["streams"]["camera.left"]["written_records"] == 1
    )
    assert store.close(timeout_s=2.0)


def test_essential_failed_stream_buffers_and_replays_in_fifo_order(tmp_path: Path) -> None:
    bad_path = tmp_path / "measurements.jsonl"
    bad_path.mkdir()
    store = MeasurementDataStore(
        tmp_path,
        fsync_interval_s=10.0,
        retry_reserve_capacity=4,
    )
    entered = threading.Event()
    release = threading.Event()
    original_write = store._write

    def pause_first_write(item):
        if not entered.is_set():
            entered.set()
            release.wait(timeout=2.0)
        original_write(item)

    store._write = pause_first_write
    assert store.enqueue("measurements", {"sequence": 1})
    assert entered.wait(timeout=1.0)
    assert store.enqueue("measurements", {"sequence": 2})
    assert store.enqueue("measurements", {"sequence": 3})
    release.set()
    _wait_for(
        lambda: store.snapshot()["streams"]["measurements"]["state"] == "FAILED"
    )
    _wait_for(lambda: store.snapshot()["essential_retry_pending"] == 3)

    bad_path.rmdir()
    store.retry_failed()
    _wait_for(
        lambda: store.snapshot()["streams"]["measurements"]["written_records"] == 3
    )
    assert store.close(timeout_s=2.0)
    rows = [json.loads(line) for line in bad_path.read_text().splitlines()]
    assert [row["sequence"] for row in rows] == [1, 2, 3]


def test_retry_fifo_keeps_older_writer_queue_record_ahead_of_direct_reserve(
    tmp_path: Path,
) -> None:
    bad_path = tmp_path / "measurements.jsonl"
    bad_path.mkdir()
    store = MeasurementDataStore(
        tmp_path,
        fsync_interval_s=10.0,
        retry_reserve_capacity=4,
    )
    first_entered = threading.Event()
    release_first = threading.Event()
    first_failed = threading.Event()
    release_writer = threading.Event()
    original_write = store._write

    def control_first_write(item):
        if not first_entered.is_set():
            first_entered.set()
            assert release_first.wait(timeout=1.0)
            original_write(item)
            first_failed.set()
            assert release_writer.wait(timeout=1.0)
            return
        original_write(item)

    store._write = control_first_write
    assert store.enqueue("measurements", {"sequence": 1})
    assert first_entered.wait(timeout=1.0)
    assert store.enqueue("measurements", {"sequence": 2})
    release_first.set()
    assert first_failed.wait(timeout=1.0)
    assert store.enqueue("measurements", {"sequence": 3})
    release_writer.set()
    _wait_for(lambda: store.snapshot()["essential_retry_pending"] == 3)

    bad_path.rmdir()
    store.retry_failed()
    _wait_for(
        lambda: store.snapshot()["streams"]["measurements"]["written_records"] == 3
    )
    assert store.close(timeout_s=2.0)
    rows = [json.loads(line) for line in bad_path.read_text().splitlines()]
    assert [row["sequence"] for row in rows] == [1, 2, 3]


def test_retry_of_one_essential_stream_is_not_blocked_by_another_failed_stream(
    tmp_path: Path,
) -> None:
    events_path = tmp_path / "events.jsonl"
    measurements_path = tmp_path / "measurements.jsonl"
    events_path.mkdir()
    measurements_path.mkdir()
    store = MeasurementDataStore(tmp_path, fsync_interval_s=10.0)
    assert store.enqueue("events", {"sequence": 1})
    assert store.enqueue("measurements", {"sequence": 2})
    _wait_for(
        lambda: store.snapshot()["streams"]["events"]["state"] == "FAILED"
        and store.snapshot()["streams"]["measurements"]["state"] == "FAILED"
        and store.snapshot()["essential_retry_pending"] == 2
    )

    measurements_path.rmdir()
    with store._state_lock:
        store._states["measurements"].state = "IDLE"
        store._states["measurements"].last_error = ""
        store._next_retry_monotonic_s["measurements"] = None
        store._next_retry_monotonic_s["events"] = time.monotonic() + 60.0
    store._drain_retry_reserve()
    _wait_for(
        lambda: store.snapshot()["streams"]["measurements"]["written_records"] == 1
    )
    assert store.snapshot()["streams"]["events"]["state"] == "FAILED"
    assert store.snapshot()["essential_retry_pending"] == 1

    events_path.rmdir()
    store.retry_failed()
    _wait_for(lambda: store.snapshot()["streams"]["events"]["written_records"] == 1)
    assert store.close(timeout_s=2.0)


def test_retry_failed_racing_close_cannot_strand_record_in_closed_queue(
    tmp_path: Path,
) -> None:
    bad_path = tmp_path / "events.jsonl"
    bad_path.mkdir()
    store = MeasurementDataStore(tmp_path, fsync_interval_s=10.0)
    assert store.enqueue("events", {"sequence": 1})
    _wait_for(
        lambda: store.snapshot()["streams"]["events"]["state"] == "FAILED"
        and store.snapshot()["essential_retry_pending"] == 1
    )

    empty_observed = threading.Event()
    empty_release = threading.Event()

    class PausingEmptyQueue(queue.Queue):
        def empty(self):
            result = super().empty()
            if store._stop_requested.is_set() and not empty_observed.is_set():
                empty_observed.set()
                empty_release.wait(timeout=2.0)
            return result

    store._priority = PausingEmptyQueue(maxsize=store.priority_capacity)
    close_result: dict[str, bool] = {}
    closer = threading.Thread(
        target=lambda: close_result.setdefault(
            "closed",
            store.close(timeout_s=2.0),
        )
    )
    closer.start()
    assert empty_observed.wait(timeout=1.0)
    assert store.snapshot()["lifecycle_state"] == "CLOSING"

    # A retry that loses the lifecycle race must not move reserve data behind
    # the writer's already-observed empty priority queue.
    store.retry_failed()
    during_close = store.snapshot()
    assert during_close["priority_pending"] == 0
    assert during_close["essential_retry_pending"] == 1
    assert during_close["active_operations"] == 0

    empty_release.set()
    closer.join(timeout=2.0)
    assert closer.is_alive() is False
    assert close_result == {"closed": True}

    closed = store.snapshot()
    assert closed["lifecycle_state"] == "CLOSED"
    assert closed["writer_alive"] is False
    assert closed["priority_pending"] == 0
    assert closed["bulk_pending"] == 0
    assert closed["essential_retry_pending"] == 0
    assert closed["streams"]["events"]["written_records"] == 0
    assert closed["streams"]["events"]["dropped_records"] == 1


def test_essential_retry_reserve_is_bounded_and_rejects_newest(tmp_path: Path) -> None:
    bad_path = tmp_path / "events.jsonl"
    bad_path.mkdir()
    store = MeasurementDataStore(
        tmp_path,
        fsync_interval_s=10.0,
        retry_reserve_capacity=2,
    )
    assert store.enqueue("events", {"sequence": 1})
    _wait_for(lambda: store.snapshot()["streams"]["events"]["state"] == "FAILED")
    assert store.enqueue("events", {"sequence": 2})
    assert store.enqueue("events", {"sequence": 3}) is False
    snapshot = store.snapshot()
    assert snapshot["essential_retry_pending"] == 2
    assert snapshot["streams"]["events"]["dropped_records"] == 1

    bad_path.rmdir()
    store.retry_failed()
    _wait_for(lambda: store.snapshot()["streams"]["events"]["written_records"] == 2)
    assert store.close(timeout_s=2.0)
    rows = [json.loads(line) for line in bad_path.read_text().splitlines()]
    assert [row["sequence"] for row in rows] == [1, 2]


def test_bulk_overload_drops_bulk_without_consuming_reserved_result_queue(tmp_path: Path) -> None:
    store = MeasurementDataStore(
        tmp_path,
        priority_capacity=2,
        bulk_capacity=2,
        fsync_interval_s=1.0,
    )
    entered = threading.Event()
    release = threading.Event()
    original_write = store._write

    def blocked_write(item):
        entered.set()
        release.wait(timeout=2.0)
        original_write(item)

    store._write = blocked_write
    assert store.enqueue("camera.left", {"frame": 0})
    assert entered.wait(timeout=1.0)
    for value in range(1, 8):
        assert store.enqueue("camera.left", {"frame": value})
    assert store.enqueue("measurements", {"result": "VALID"})
    snapshot = store.snapshot()
    assert snapshot["streams"]["camera.left"]["dropped_records"] > 0
    assert snapshot["streams"]["measurements"]["dropped_records"] == 0
    release.set()
    assert store.close(timeout_s=2.0)
    rows = [json.loads(line) for line in (tmp_path / "measurements.jsonl").read_text().splitlines()]
    assert rows[-1]["result"] == "VALID"


def test_concurrent_bulk_refill_drops_losing_producer_without_raising(tmp_path: Path) -> None:
    store = MeasurementDataStore(
        tmp_path,
        priority_capacity=1,
        bulk_capacity=1,
        fsync_interval_s=1.0,
    )
    writer_entered = threading.Event()
    writer_release = threading.Event()
    original_write = store._write

    def blocked_write(item):
        writer_entered.set()
        writer_release.wait(timeout=2.0)
        original_write(item)

    store._write = blocked_write
    assert store.enqueue("events", {"hold_writer": True}, priority=True)
    assert writer_entered.wait(timeout=1.0)

    evicted = threading.Event()
    admit_loser = threading.Event()

    class PausingQueue(queue.Queue):
        def get_nowait(self):
            item = super().get_nowait()
            if threading.current_thread().name == "bulk-losing-producer":
                evicted.set()
                admit_loser.wait(timeout=2.0)
            return item

    store._bulk = PausingQueue(maxsize=1)
    assert store.enqueue("camera.left", {"frame": "oldest"}, priority=False)

    results: dict[str, bool] = {}
    errors: dict[str, BaseException] = {}

    def enqueue(name: str, stream: str) -> None:
        try:
            results[name] = store.enqueue(stream, {"producer": name}, priority=False)
        except BaseException as exc:  # pragma: no cover - regression oracle
            errors[name] = exc

    losing = threading.Thread(
        target=enqueue,
        name="bulk-losing-producer",
        args=("losing", "camera.right"),
    )
    losing.start()
    assert evicted.wait(timeout=1.0)

    winner = threading.Thread(
        target=enqueue,
        name="bulk-winning-producer",
        args=("winning", "gnss.base"),
    )
    winner.start()
    winner.join(timeout=1.0)
    assert not winner.is_alive()

    admit_loser.set()
    losing.join(timeout=1.0)
    assert not losing.is_alive()
    writer_release.set()
    assert store.close(timeout_s=2.0)

    assert errors == {}
    assert results == {"winning": True, "losing": False}
    streams = store.snapshot()["streams"]
    assert streams["camera.left"]["dropped_records"] == 1
    assert streams["camera.right"]["dropped_records"] == 1
    assert streams["gnss.base"]["dropped_records"] == 0
    assert streams["gnss.base"]["written_records"] == 1


def test_live_rows_keep_current_preview_and_persist_replacements_as_new_events(tmp_path: Path) -> None:
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    rows = LiveDataRows(store, "measurements", preview_limit=1)
    assert rows.append({"result": "BLOCKED"})
    assert rows.replace(0, {"result": "VALID"})
    assert rows[0]["result"] == "VALID"
    assert rows.preview_rows == [{"result": "VALID"}]
    assert store.close(timeout_s=2.0)
    persisted = [
        json.loads(line)
        for line in (tmp_path / "measurements.jsonl").read_text().splitlines()
    ]
    assert [row["_record_operation"] for row in persisted] == ["append", "replace"]
    assert [row["result"] for row in persisted] == ["BLOCKED", "VALID"]


def test_live_rows_failed_initial_append_retries_as_append_then_replaces() -> None:
    calls: list[dict[str, object]] = []
    outcomes = iter((False, True, True))

    class Store:
        def enqueue(self, stream, payload, *, priority=None):
            calls.append(
                {
                    "stream": stream,
                    "payload": dict(payload),
                    "priority": priority,
                }
            )
            return next(outcomes)

    rows = LiveDataRows(Store(), "measurements", priority=True)

    assert rows.append({"result": "BLOCKED"}) is False
    assert rows.replace(0, {"result": "VALID"}) is True
    assert rows.replace(0, {"result": "ESTIMATED"}) is True

    assert [call["payload"]["_record_operation"] for call in calls] == [
        "append",
        "append",
        "replace",
    ]
    assert [call["payload"]["_record_index"] for call in calls] == [0, 0, 0]
    assert rows[0]["result"] == "ESTIMATED"


def test_live_rows_clear_forgets_admitted_indices() -> None:
    calls: list[dict[str, object]] = []
    outcomes = iter((True, False, True))

    class Store:
        def enqueue(self, _stream, payload, *, priority=None):
            calls.append(dict(payload))
            return next(outcomes)

    rows = LiveDataRows(Store(), "measurements")
    assert rows.append({"result": "OLD"}) is True

    rows.clear()

    assert rows.append({"result": "BLOCKED"}) is False
    assert rows.replace(0, {"result": "VALID"}) is True
    assert [call["_record_operation"] for call in calls] == [
        "append",
        "append",
        "append",
    ]


def test_bad_record_drops_only_itself_and_later_results_continue(tmp_path: Path) -> None:
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    assert store.enqueue("measurements", {"bad": object()}) is False
    assert store.enqueue("measurements", {"result": "VALID"}) is True
    assert store.close(timeout_s=2.0)

    [persisted] = [
        json.loads(line)
        for line in (tmp_path / "measurements.jsonl").read_text().splitlines()
    ]
    assert persisted["result"] == "VALID"
    snapshot = store.snapshot()
    assert snapshot["streams"]["measurements"]["state"] == "ACTIVE"
    assert snapshot["streams"]["measurements"]["dropped_records"] == 1


def test_default_and_legacy_default_use_repository_recordings_root(tmp_path: Path) -> None:
    expected = tmp_path / "repository-recordings"

    for configured in ("", "measurement_data"):
        class Panel:
            app_root = tmp_path / "data-root"
            default_measurement_data_root = expected
            settings = type("Settings", (), {"measurement_data_root": configured})()

            @staticmethod
            def _workstation_setting(_name, default):
                return configured if _name == "measurement_data_root" else default

        controller = DataStorageController(Panel())
        snapshot = controller.data_storage_snapshot()
        assert snapshot["root"] == str(expected.resolve())
        assert snapshot["streams"]["measurements"]["filename_mode"] == "timestamped"
        assert Path(snapshot["streams"]["measurements"]["path"]).name.startswith(
            "measurements_"
        )
        assert controller.close_workstation_data(timeout_s=2.0)


def test_data_storage_controller_publishes_timestamped_csv_mirror(tmp_path: Path) -> None:
    class Panel:
        app_root = tmp_path / "data-root"
        default_measurement_data_root = tmp_path / "repository-recordings"
        settings = type("Settings", (), {"measurement_data_root": ""})()

        @staticmethod
        def _workstation_setting(_name, default):
            return default

    controller = DataStorageController(Panel())
    assert controller.append_workstation_data(
        "measurements",
        {
            "measurement_id": "m-1",
            "status": "VALID",
            "coordinate_contract": "baseline_center_frd",
        },
    )
    _wait_for(
        lambda: controller.data_storage_snapshot()["streams"]["measurements"]["csv"][
            "written_records"
        ]
        == 1
    )
    snapshot = controller.data_storage_snapshot()
    csv_path = Path(snapshot["streams"]["measurements"]["csv"]["path"])
    assert csv_path.suffix == ".csv"
    assert csv_path.name.startswith("measurements_")
    assert csv_path.is_file()
    assert controller.close_workstation_data(timeout_s=2.0)


def test_custom_data_root_remains_relative_to_workstation_data_root(tmp_path: Path) -> None:
    class Panel:
        app_root = tmp_path / "data-root"
        default_measurement_data_root = tmp_path / "repository-recordings"
        settings = type("Settings", (), {"measurement_data_root": "field/run-a"})()

        @staticmethod
        def _workstation_setting(_name, default):
            return "field/run-a" if _name == "measurement_data_root" else default

    controller = DataStorageController(Panel())
    assert controller.data_storage_snapshot()["root"] == str(
        (Panel.app_root / "field" / "run-a").resolve()
    )
    assert controller.close_workstation_data(timeout_s=2.0)


def test_invalid_absolute_setting_falls_back_to_default_recordings_root(
    tmp_path: Path,
) -> None:
    expected = tmp_path / "repository-recordings"

    class Panel:
        app_root = tmp_path
        default_measurement_data_root = expected
        settings = type("Settings", (), {"measurement_data_root": "/outside/data"})()

        @staticmethod
        def _workstation_setting(_name, default):
            return "/outside/data" if _name == "measurement_data_root" else default

    controller = DataStorageController(Panel())
    snapshot = controller.data_storage_snapshot()
    assert snapshot["root"] == str(expected.resolve())
    assert snapshot["state"] == "DEGRADED"
    assert "portable relative path" in snapshot["configuration_error"]
    assert controller.append_workstation_data("events", {"event_type": "still_available"})
    assert controller.close_workstation_data(timeout_s=2.0)
