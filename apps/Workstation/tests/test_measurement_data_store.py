from __future__ import annotations

import json
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


def test_data_root_is_relative_and_invalid_absolute_setting_falls_back(tmp_path: Path) -> None:
    class Panel:
        app_root = tmp_path
        settings = type("Settings", (), {"measurement_data_root": "/outside/data"})()

        @staticmethod
        def _workstation_setting(_name, default):
            return "/outside/data" if _name == "measurement_data_root" else default

    controller = DataStorageController(Panel())
    snapshot = controller.data_storage_snapshot()
    assert snapshot["root"] == str((tmp_path / "measurement_data").resolve())
    assert snapshot["state"] == "DEGRADED"
    assert "portable relative path" in snapshot["configuration_error"]
    assert controller.append_workstation_data("events", {"event_type": "still_available"})
    assert controller.close_workstation_data(timeout_s=2.0)
