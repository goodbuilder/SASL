from __future__ import annotations

import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from zipfile import ZipFile

import pytest

from Workstation.workstation_core.latest_measurement_xlsx import (
    LATEST_MEASUREMENT_WORKBOOK_NAME,
    LATEST_MEASUREMENT_WORKBOOK_SCHEMA,
    LatestMeasurementExportCancelled,
    discover_latest_measurement,
    timestamped_workbook_name,
    workbook_identity,
    write_latest_measurement_xlsx,
)
from Workstation.workstation_core.measurement_data_store import MeasurementDataStore


def _append(path: Path, *rows: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        for row in rows:
            handle.write(
                json.dumps(
                    row,
                    ensure_ascii=False,
                    allow_nan=False,
                    sort_keys=True,
                    separators=(",", ":"),
                )
                + "\n"
            )


def _worksheet_bytes(path: Path, sheet_prefix: str) -> bytes:
    with ZipFile(path) as archive:
        workbook = archive.read("xl/workbook.xml")
        # Fixed base sheets are emitted in BASE_SHEET_NAMES order. Find the
        # matching sheet's relationship id without relying on shared strings.
        from xml.etree import ElementTree

        root = ElementTree.fromstring(workbook)
        main = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
        rel = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
        relationship_id = ""
        for sheet in root.findall(f"{{{main}}}sheets/{{{main}}}sheet"):
            if str(sheet.attrib.get("name") or "").startswith(sheet_prefix):
                relationship_id = str(sheet.attrib[f"{{{rel}}}id"])
                break
        assert relationship_id
        relationships = ElementTree.fromstring(
            archive.read("xl/_rels/workbook.xml.rels")
        )
        package_rel = "http://schemas.openxmlformats.org/package/2006/relationships"
        target = ""
        for item in relationships.findall(f"{{{package_rel}}}Relationship"):
            if item.attrib.get("Id") == relationship_id:
                target = str(item.attrib["Target"])
                break
        assert target
        member = target.lstrip("/")
        if not member.startswith("xl/"):
            member = f"xl/{member}"
        return archive.read(member)


def _fixture(root: Path) -> tuple[str, str]:
    old = "run-old"
    latest = "run-latest"
    _append(
        root / "events.jsonl",
        {
            "event_type": "measurement_start_requested",
            "runtime_generation": old,
            "started_utc_ns": 1_700_000_000_000_000_000,
            "recorded_utc_ns": 1_700_000_000_000_000_001,
        },
        {
            "event_type": "measurement_stopped",
            "runtime_generation": old,
            "stopped_utc_ns": 1_700_000_001_000_000_000,
            "recorded_utc_ns": 1_700_000_001_000_000_001,
        },
        {
            "event_type": "measurement_start_requested",
            "runtime_generation": latest,
            "started_utc_ns": 1_700_000_002_123_456_789,
            "recorded_utc_ns": 1_700_000_002_123_456_790,
        },
        {
            "event_type": "measurement_stopped",
            "runtime_generation": latest,
            "stopped_utc_ns": 1_700_000_012_987_654_321,
            "recorded_utc_ns": 1_700_000_012_987_654_322,
        },
    )
    _append(
        root / "measurements.jsonl",
        {
            "measurement_id": old,
            "_record_index": 0,
            "_record_operation": "append",
            "pair_id": "old-pair",
            "distance_m": 999.0,
            "recorded_utc_ns": 1_700_000_000_100_000_000,
        },
        {
            "measurement_id": latest,
            "_record_index": 0,
            "_record_operation": "append",
            "pair_id": "latest-pair",
            "distance_m": 10.0,
            "comment": "=HYPERLINK(\"bad\")",
            "recorded_utc_ns": 1_700_000_003_000_000_123,
        },
        {
            "measurement_id": latest,
            "_record_index": 0,
            "_record_operation": "replace",
            "pair_id": "latest-pair",
            "distance_m": 11.5,
            "recorded_utc_ns": 1_700_000_004_000_000_456,
        },
    )
    _append(
        root / "method_evaluations.jsonl",
        {
            "measurement_id": latest,
            "pair_id": "latest-pair",
            "pair_sequence": 1,
            "snapshot_sha256": "a" * 64,
            "recorded_utc_ns": 1_700_000_005_000_000_000,
            "snapshot_payload": {
                "measurement_id": "nested-wrong-generation",
                "schema": "fixture-input-v1",
                "nested": {"value": 7},
            },
            "outcomes": [
                {
                    "measurement_id": "nested-wrong-generation",
                    "method_id": "pixhawk_rtk_live",
                    "status": "VALID",
                    "distance_m": 11.5,
                },
                {"method_id": "terrain_pnp", "status": "BLOCKED", "distance_m": None},
            ],
        },
    )
    _append(
        root / "camera_left.jsonl",
        {
            "measurement_id": latest,
            "sequence": 1,
            "left_only": 1,
            "recorded_utc_ns": 1_700_000_006_000_000_000,
        },
        {
            "measurement_id": latest,
            "sequence": 2,
            "right_only": 2,
            "recorded_utc_ns": 1_700_000_007_000_000_000,
        },
    )
    _append(
        root / "px4_left.jsonl",
        {
            "measurement_id": latest,
            "sequence": 1,
            "yaw_deg": 12.5,
            "recorded_utc_ns": 1_700_000_006_500_000_000,
        },
    )
    _append(
        root / "gnss_baseline_center.jsonl",
        {
            "measurement_id": old,
            "sequence": 1,
            "geometry_state": "CURRENT",
            "gnss_baseline_center_forward_m": 987654.321,
            "recorded_utc_ns": 1_700_000_005_500_000_000,
        },
        {
            "measurement_id": latest,
            "sequence": 2,
            "geometry_state": "CURRENT",
            "gnss_baseline_center_forward_m": 10.125,
            "gnss_baseline_center_right_m": -2.5,
            "gnss_baseline_center_down_m": 4.75,
            "gnss_baseline_center_range_m": 11.45,
            "three_source_timestamp_spread_ms": 999.9,
            "recorded_utc_ns": 1_700_000_007_500_000_000,
        },
    )
    _append(
        root / "native_frames.jsonl",
        {
            "runtime_generation": latest,
            "request_id": "native-1",
            "event_type": "native_frame_completed",
            "recorded_utc_ns": 1_700_000_008_000_000_000,
        },
    )
    _append(
        root / "artifacts.jsonl",
        {
            "runtime_generation": latest,
            "event_type": "video_closed",
            "path": "/data/video.mp4",
            "recorded_utc_ns": 1_700_000_009_000_000_000,
        },
    )
    return old, latest


def test_latest_workbook_filters_generation_materializes_results_and_expands_methods(
    tmp_path: Path,
) -> None:
    old, latest = _fixture(tmp_path)

    result = write_latest_measurement_xlsx(tmp_path)

    assert result.path == tmp_path / LATEST_MEASUREMENT_WORKBOOK_NAME
    assert result.generation == latest
    assert result.row_counts["Results"] == 1
    assert result.row_counts["ResultHistory"] == 2
    assert result.row_counts["MethodBundles"] == 1
    assert result.row_counts["MethodOutcomes"] == 2
    assert result.row_counts["MethodInputs"] == 1
    assert result.row_counts["CameraLeft"] == 2
    assert result.row_counts["PixhawkP2"] == 0
    assert result.row_counts["GnssBaselineCenter"] == 1
    assert workbook_identity(result.path) == {
        "workstation_schema": LATEST_MEASUREMENT_WORKBOOK_SCHEMA,
        "runtime_generation": latest,
    }

    results_xml = _worksheet_bytes(result.path, "Results")
    history_xml = _worksheet_bytes(result.path, "ResultHistory")
    methods_xml = _worksheet_bytes(result.path, "MethodOutcomes")
    inputs_xml = _worksheet_bytes(result.path, "MethodInputs")
    camera_xml = _worksheet_bytes(result.path, "CameraLeft")
    gnss_center_xml = _worksheet_bytes(result.path, "GnssBaselineCenter")
    assert b"11.5" in results_xml
    assert b"999" not in results_xml
    assert old.encode() not in history_xml
    assert b"pixhawk_rtk_live" in methods_xml
    assert b"terrain_pnp" in methods_xml
    assert b"nested-wrong-generation" not in methods_xml
    assert b"nested-wrong-generation" not in inputs_xml
    assert b"10.125" in gnss_center_xml
    assert b"987654.321" not in gnss_center_xml
    assert b"NA()" in camera_xml
    assert b"#N/A" in camera_xml
    assert b"2023-11-15T" in history_xml
    assert b"1700000004000000456" in results_xml


def test_latest_workbook_reads_timestamped_stream_files(tmp_path: Path) -> None:
    generation = "run-timestamped"
    store = MeasurementDataStore(
        tmp_path,
        timestamped_filenames=True,
        file_stamp="20260903_120000_000000001+0900",
    )
    try:
        assert store.enqueue(
            "events",
            {
                "event_type": "measurement_stopped",
                "runtime_generation": generation,
                "stopped_utc_ns": 1_700_000_002,
            },
            priority=True,
        )
        assert store.enqueue(
            "measurements",
            {
                "measurement_id": generation,
                "_record_index": 0,
                "_record_operation": "append",
                "pair_id": "timestamped-pair",
                "distance_m": 12.5,
            },
            priority=True,
        )
        assert store.close(timeout_s=2.0)
    finally:
        if store.alive:
            assert store.close(timeout_s=2.0)

    result = write_latest_measurement_xlsx(tmp_path, generation=generation)

    assert result.row_counts["Results"] == 1
    assert result.row_counts["ResultHistory"] == 1
    assert result.issue_count == 0


def test_formula_like_source_text_is_never_written_as_a_formula(tmp_path: Path) -> None:
    _old, latest = _fixture(tmp_path)

    output = write_latest_measurement_xlsx(tmp_path, generation=latest).path

    history_xml = _worksheet_bytes(output, "ResultHistory")
    assert b"HYPERLINK" in history_xml
    assert b"<f>HYPERLINK" not in history_xml


def test_discovery_falls_back_to_latest_saved_identity_without_stop_event(tmp_path: Path) -> None:
    _append(
        tmp_path / "measurements.jsonl",
        {"measurement_id": "run-a", "recorded_utc_ns": 10},
        {"measurement_id": "run-b", "recorded_utc_ns": 20},
    )

    selection = discover_latest_measurement(tmp_path)

    assert selection.generation == "run-b"
    assert selection.completion_evidence == "inferred_saved_identity"


def test_malformed_and_torn_rows_are_reported_without_aborting(tmp_path: Path) -> None:
    generation = "run-issues"
    path = tmp_path / "measurements.jsonl"
    _append(path, {"measurement_id": generation, "recorded_utc_ns": 1})
    with path.open("ab") as handle:
        handle.write(b'{"measurement_id":"run-issues",bad}\n')
        handle.write(b'{"measurement_id":"run-issues"')

    result = write_latest_measurement_xlsx(tmp_path, generation=generation)

    assert result.issue_count == 2
    issues_xml = _worksheet_bytes(result.path, "Issues")
    assert b"incomplete final JSONL record excluded" in issues_xml
    assert b"Expecting property name" in issues_xml


def test_row_and_column_limits_create_deterministic_shards(tmp_path: Path) -> None:
    generation = "run-shards"
    for index in range(5):
        _append(
            tmp_path / "camera_left.jsonl",
            {
                "measurement_id": generation,
                "sequence": index,
                "a": index,
                "b": index,
                "c": index,
                "d": index,
                "recorded_utc_ns": index + 1,
            },
        )

    result = write_latest_measurement_xlsx(
        tmp_path,
        generation=generation,
        max_rows_per_sheet=3,
        max_columns_per_sheet=4,
    )

    with ZipFile(result.path) as archive:
        workbook_xml = archive.read("xl/workbook.xml")
    assert b"CameraLeft_r001c01" in workbook_xml
    assert b"CameraLeft_r003c" in workbook_xml


def test_foreign_output_is_preserved_and_temporary_file_is_removed(tmp_path: Path) -> None:
    output = tmp_path / LATEST_MEASUREMENT_WORKBOOK_NAME
    output.write_bytes(b"user-owned")

    with pytest.raises(FileExistsError, match="unowned"):
        write_latest_measurement_xlsx(tmp_path, generation="run-foreign")

    assert output.read_bytes() == b"user-owned"
    assert not list(tmp_path.glob(".latest_measurement.xlsx.*.tmp.xlsx"))


def test_cancelled_export_never_publishes(tmp_path: Path) -> None:
    cancelled = threading.Event()
    cancelled.set()

    with pytest.raises(LatestMeasurementExportCancelled):
        write_latest_measurement_xlsx(
            tmp_path,
            generation="run-cancelled",
            cancel_event=cancelled,
        )

    assert not (tmp_path / LATEST_MEASUREMENT_WORKBOOK_NAME).exists()
    assert not list(tmp_path.glob(".latest_measurement.xlsx.*.tmp.xlsx"))


def test_data_store_checkpoint_drains_admitted_rows_and_returns_exact_offsets(
    tmp_path: Path,
) -> None:
    store = MeasurementDataStore(tmp_path, fsync_interval_s=10.0)
    try:
        assert store.enqueue(
            "measurements",
            {"measurement_id": "run-checkpoint", "sentinel": "written"},
        )

        checkpoint = store.export_checkpoint(timeout_s=2.0)

        path = tmp_path / "measurements.jsonl"
        assert checkpoint.settled is True
        assert checkpoint.pending_before_cutoff == 0
        assert checkpoint.stream_byte_limits["measurements"] == path.stat().st_size
        assert json.loads(path.read_text(encoding="utf-8"))["sentinel"] == "written"
        assert checkpoint.storage_snapshot["checkpoint_pending_records"] == 0
    finally:
        assert store.close(timeout_s=2.0)


def test_data_store_checkpoint_reports_retryable_pending_then_recovers(
    tmp_path: Path,
) -> None:
    blocked = tmp_path / "events.jsonl"
    blocked.mkdir()
    store = MeasurementDataStore(tmp_path, fsync_interval_s=10.0)
    try:
        assert store.enqueue(
            "events",
            {"event_type": "checkpoint-sentinel", "runtime_generation": "run-pending"},
        )
        deadline = time.monotonic() + 2.0
        while store.snapshot()["essential_retry_pending"] == 0:
            assert time.monotonic() < deadline
            time.sleep(0.01)

        partial = store.export_checkpoint(timeout_s=0.05)

        assert partial.settled is False
        assert partial.pending_before_cutoff == 1
        assert partial.stream_byte_limits["events"] == 0

        blocked.rmdir()
        store.retry_failed()
        recovered = store.export_checkpoint(timeout_s=2.0)

        assert recovered.settled is True
        assert recovered.pending_before_cutoff == 0
        assert (tmp_path / "events.jsonl").is_file()
    finally:
        assert store.close(timeout_s=2.0)


def _wait_until(predicate, *, timeout_s: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(0.01)


def _storage_panel(root: Path) -> SimpleNamespace:
    return SimpleNamespace(
        app_root=root,
        default_measurement_data_root=root,
        _measurement_run_state="IDLE",
        _measurement_run_last_generation="",
        _manual_recording_active=False,
        distance_recording_enabled=False,
        video_recording_enabled=False,
    )


def test_data_storage_controller_generates_and_reports_one_latest_workbook(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import Workstation.data_storage_controller as controller_module

    monkeypatch.setattr(controller_module, "LATEST_MEASUREMENT_XLSX_QUIET_S", 0.0)
    panel = _storage_panel(tmp_path)
    controller = controller_module.DataStorageController(panel)
    generation = "run-controller"
    try:
        assert controller.append_workstation_data(
            "measurements",
            {"measurement_id": generation, "pair_id": "pair-controller"},
            priority=True,
        )
        assert controller.request_latest_measurement_xlsx(
            generation,
            started_utc_ns=1_700_000_000_000_000_000,
            stopped_utc_ns=1_700_000_001_000_000_000,
            automatic=True,
        )
        _wait_until(
            lambda: controller.latest_measurement_xlsx_snapshot()["state"]
            in {"READY", "FAILED"}
        )

        snapshot = controller.latest_measurement_xlsx_snapshot()
        assert snapshot["state"] == "READY", snapshot
        assert snapshot["target_generation"] == generation
        assert snapshot["published_generation"] == generation
        assert snapshot["row_counts"]["Results"] == 1
        output = Path(str(snapshot["output_path"]))
        assert output.is_file()
        assert output.name == timestamped_workbook_name(
            int(snapshot["requested_utc_ns"])
        )
    finally:
        assert controller.close_workstation_data(timeout_s=3.0)


def test_data_storage_controller_keeps_prior_timestamped_workbook_on_next_export(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import Workstation.data_storage_controller as controller_module

    monkeypatch.setattr(controller_module, "LATEST_MEASUREMENT_XLSX_QUIET_S", 0.0)
    panel = _storage_panel(tmp_path)
    controller = controller_module.DataStorageController(panel)
    try:
        for generation, distance in (("run-first", 1.0), ("run-second", 2.0)):
            assert controller.append_workstation_data(
                "measurements",
                {
                    "measurement_id": generation,
                    "_record_index": 0,
                    "_record_operation": "append",
                    "pair_id": generation,
                    "distance_m": distance,
                },
                priority=True,
            )
            assert controller.request_latest_measurement_xlsx(
                generation,
                automatic=False,
            )
            _wait_until(
                lambda: controller.latest_measurement_xlsx_snapshot()["state"]
                in {"READY", "FAILED"}
            )
            snapshot = controller.latest_measurement_xlsx_snapshot()
            assert snapshot["state"] == "READY", snapshot
            assert snapshot["published_generation"] == generation

        outputs = sorted(tmp_path.glob("latest_measurement_*.xlsx"))
        assert len(outputs) == 2
        assert outputs[0] != outputs[1]
    finally:
        assert controller.close_workstation_data(timeout_s=3.0)


def test_data_storage_controller_recovers_existing_owned_workbook_status(
    tmp_path: Path,
) -> None:
    import Workstation.data_storage_controller as controller_module

    result = write_latest_measurement_xlsx(
        tmp_path,
        generation="run-restart-ready",
    )
    controller = controller_module.DataStorageController(_storage_panel(tmp_path))

    snapshot = controller.latest_measurement_xlsx_snapshot()

    assert snapshot["state"] == "READY"
    assert snapshot["published_generation"] == "run-restart-ready"
    assert snapshot["output_path"] == str(result.path)
    assert controller.close_workstation_data(timeout_s=1.0)


def test_new_recording_cancels_active_export_without_blocking_enqueue(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import Workstation.data_storage_controller as controller_module

    monkeypatch.setattr(controller_module, "LATEST_MEASUREMENT_XLSX_QUIET_S", 0.0)
    started = threading.Event()
    cancelled = threading.Event()

    def blocking_export(*_args, cancel_event=None, **_kwargs):
        started.set()
        assert cancel_event is not None
        while not cancel_event.wait(0.01):
            pass
        cancelled.set()
        raise LatestMeasurementExportCancelled("fixture cancellation")

    monkeypatch.setattr(controller_module, "write_latest_measurement_xlsx", blocking_export)
    panel = _storage_panel(tmp_path)
    controller = controller_module.DataStorageController(panel)
    try:
        assert controller.request_latest_measurement_xlsx(
            "run-before-new",
            automatic=True,
        )
        assert started.wait(2.0)

        panel._measurement_run_state = "RUNNING"
        before = time.monotonic()
        assert controller.append_workstation_data(
            "events",
            {
                "event_type": "measurement_start_requested",
                "runtime_generation": "run-new",
            },
            priority=True,
        )
        elapsed = time.monotonic() - before

        assert elapsed < 0.1
        assert cancelled.wait(2.0)
        assert not (tmp_path / LATEST_MEASUREMENT_WORKBOOK_NAME).exists()
    finally:
        assert controller.close_workstation_data(timeout_s=3.0)


def test_real_panel_stop_publishes_latest_xlsx_through_delegate_chain(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from PyQt5 import QtWidgets

    from Workstation import data_storage_controller as controller_module
    from Workstation import standalone_gui

    monkeypatch.setattr(controller_module, "LATEST_MEASUREMENT_XLSX_QUIET_S", 0.0)
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_initial_status",
        lambda self: None,
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda self: None,
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    try:
        assert panel.begin_measurement_run()
        generation = panel._measurement_run_generation
        assert panel.append_workstation_data(
            "measurements",
            {
                "measurement_id": generation,
                "pair_id": "delegate-pair",
                "distance_m": 4.5,
            },
            priority=True,
        )

        assert panel.request_measurement_stop()
        _wait_until(
            lambda: panel.latest_measurement_xlsx_snapshot()["state"]
            in {"READY", "FAILED"},
            timeout_s=8.0,
        )

        snapshot = panel.latest_measurement_xlsx_snapshot()
        assert snapshot["state"] == "READY", snapshot
        assert snapshot["published_generation"] == generation
        outputs = list((tmp_path / "recordings").glob("latest_measurement_*.xlsx"))
        assert len(outputs) == 1
        assert outputs[0].is_file()
    finally:
        assert panel.close_workstation_data(timeout_s=3.0)
        panel._shutdown_coordinator = SimpleNamespace(run=lambda: True)
        panel.close()
        panel.deleteLater()
        app.processEvents()
