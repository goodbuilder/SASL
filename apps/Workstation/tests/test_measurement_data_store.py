from __future__ import annotations

import json
import queue
import re
import threading
import time
from pathlib import Path

from Workstation.workstation_core.measurement_data_store import (
    LiveDataRows,
    MeasurementDataStore,
    allocate_timestamped_path,
    creation_timestamp_text,
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
        assert controller.data_storage_snapshot()["root"] == str(expected.resolve())
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
