"""Readability projections added to the recorded evidence.

- ``_record_index`` is monotonic for the whole stream file, not per run owner.
- ``application_log`` rows carry an inferred ``level``.
- Accepting an optical-pose suggestion produces one flat ``alignment`` row.
- Shutdown can archive a closed session's files under ``<root>/<token>/``.
- The workbook gains ``05_Method_Compare`` and ``06_Optical_Pose`` sheets.
"""

from __future__ import annotations

import json
from pathlib import Path

from Workstation.data_storage_controller import DataStorageController
from Workstation.optical_pose_controller import accepted_suggestion_alignment_row
from Workstation.workstation_core.log_level import classify_log_level
from Workstation.workstation_core.measurement_data_store import (
    LiveDataRows,
    MeasurementDataStore,
    STREAM_PATHS,
)
from Workstation.workstation_core.measurement_record import (
    EventRow,
    MeasurementSnapshot,
    MethodResultRow,
    PairRow,
)
from Workstation.workstation_core.measurement_workbook import (
    _method_compare_table,
    _optical_pose_rows,
)


def test_record_index_is_monotonic_across_row_owners(tmp_path: Path) -> None:
    store = MeasurementDataStore(tmp_path, fsync_interval_s=0.05)
    first = LiveDataRows(store, "measurements")
    assert first.append({"run": 1, "n": 0})
    assert first.append({"run": 1, "n": 1})
    assert first.replace(1, {"run": 1, "n": 1, "fixed": True})
    second = LiveDataRows(store, "measurements")  # next Measurement run
    assert second.record_index_base == 2
    assert second.append({"run": 2, "n": 0})
    assert second.replace(0, {"run": 2, "n": 0, "fixed": True})
    assert store.close(timeout_s=2.0)
    persisted = [
        json.loads(line)
        for line in (tmp_path / "measurements.jsonl").read_text().splitlines()
    ]
    assert [(row["_record_index"], row["_record_operation"]) for row in persisted] == [
        (0, "append"),
        (1, "append"),
        (1, "replace"),
        (2, "append"),
        (2, "replace"),
    ]


def test_method_results_is_a_registered_priority_stream() -> None:
    assert STREAM_PATHS["method_results"].name == "method_results.jsonl"


def test_log_level_classification() -> None:
    assert classify_log_level("[저장] 영상 녹화 시작 상태: FAILED") == "ERROR"
    assert classify_log_level("[픽스호크] 오른쪽 MAVLink liveness: ATTITUDE STALE") == "WARNING"
    assert classify_log_level("[카메라] 왼쪽 스트림 상태: YOLO 시작 중") == "INFO"


def test_accepted_suggestion_alignment_row_flattens_relative_rotation() -> None:
    row = accepted_suggestion_alignment_row(
        {
            "mode": "cloud",
            "suggestion_id": "auto-cloud-1",
            "accepted_utc_ns": 1788761400000000000,
            "state": "GOOD",
            "inlier_count": 226,
            "match_count": 231,
            "residual_px": 0.65,
            "confidence": 0.86,
            "eligible": True,
            "relative_rotation": {
                "available": True,
                "yaw_right_minus_left_deg": -0.034,
                "pitch_right_minus_left_deg": -0.047,
                "roll_right_minus_left_deg": -0.26,
                "fit_inlier_count": 206,
                "fit_residual_px": 15.7,
                "far_field_du_median_px": -2.0,
                "far_field_dv_median_px": -6.0,
            },
            "source_frames": {"left": {"image_sha256": "aa"}, "right": {"image_sha256": "bb"}},
            "input_hashes": {"gnss_baseline_sha256": "cc"},
            "gnss_baseline": {"baseline_m": 24.77, "fix_quality": "RTK_FIXED"},
        }
    )
    assert row["event_type"] == "optical_pose_auto_suggestion_accepted"
    assert row["method_id"] == "cloud_gnss"
    assert row["source"] == "auto-cloud-1"
    assert row["relative_yaw_right_minus_left_deg"] == -0.034
    assert row["relative_fit_inlier_count"] == 206
    assert row["left_image_sha256"] == "aa" and row["right_image_sha256"] == "bb"
    assert row["gnss_baseline_m"] == 24.77


class _Panel:
    def __init__(self) -> None:
        self.logs: list[str] = []

    def _log(self, text: str) -> None:
        self.logs.append(text)


def test_archive_session_recordings_moves_closed_session_files(tmp_path: Path) -> None:
    root = tmp_path / "recordings"
    root.mkdir()
    store = MeasurementDataStore(root, timestamped_filenames=True, fsync_interval_s=0.05)
    stamp = store.file_stamp
    assert store.enqueue("events", {"event_type": "x"}, priority=True)
    assert store.close(timeout_s=2.0)
    session_prefix = stamp[:15]
    run_prefix = session_prefix[:9] + f"{int(session_prefix[9:]) + 1:06d}"
    (root / f"{run_prefix}_measurement.xlsx").write_bytes(b"x")
    (root / f"{run_prefix}_left.mp4").write_bytes(b"")
    (root / "frames" / "optical-pose-burst").mkdir(parents=True)
    (root / "frames" / "optical-pose-burst" / "a.png").write_bytes(b"png")
    (root / "20200101_000000_measurement.xlsx").write_bytes(b"old")  # earlier session: untouched
    (root / "latest_measurement_x.xlsx").write_bytes(b"derived")  # not a run artifact

    controller = DataStorageController.__new__(DataStorageController)
    controller.panel = _Panel()
    controller._store = store
    controller._root = root

    assert controller.archive_session_recordings() is True
    session_dir = root / session_prefix
    assert (session_dir / f"events_{stamp}.jsonl").exists()
    assert (session_dir / f"events_{stamp}.csv").exists()
    assert (session_dir / "frames" / "optical-pose-burst" / "a.png").exists()
    assert (session_dir / "runs" / run_prefix / f"{run_prefix}_measurement.xlsx").exists()
    assert (session_dir / "runs" / run_prefix / f"{run_prefix}_left.mp4").exists()
    assert (root / "20200101_000000_measurement.xlsx").exists()
    assert (root / "latest_measurement_x.xlsx").exists()
    assert not any(f"_{stamp}." in path.name for path in root.iterdir())
    # idempotent
    assert controller.archive_session_recordings() is True


def test_archive_waits_for_an_open_store(tmp_path: Path) -> None:
    store = MeasurementDataStore(tmp_path, timestamped_filenames=True, fsync_interval_s=0.05)
    controller = DataStorageController.__new__(DataStorageController)
    controller.panel = _Panel()
    controller._store = store
    controller._root = tmp_path
    try:
        assert controller.archive_session_recordings() is False
    finally:
        assert store.close(timeout_s=2.0)


def _snapshot(**overrides: object) -> MeasurementSnapshot:
    pair = PairRow(
        pair_id="p1",
        pair_sequence=1,
        created_unix_ns=1788761400000000000,
        created_monotonic_ns=1,
        timestamp_basis="app_receive_monotonic",
        time_delta_s=0.01,
        left_detection_id="l",
        right_detection_id="r",
        left_detection={},
        right_detection={},
        capture_sync={},
    )
    rtk = MethodResultRow(
        pair_id="p1", pair_sequence=1, method_id="pixhawk_rtk_live", attempted=True, status="ESTIMATED", reason="",
        derived={"forward_m": 260.0, "right_m": -37.0, "down_m": -5.0, "range_m": 263.0, "ray_angle_deg": 5.3},
        outcome={"result": {"match": {"disparity_px": 462.3}}},
    )
    yaw90 = MethodResultRow(
        pair_id="p1", pair_sequence=1, method_id="pixhawk_rp_gps_baseline_yaw", attempted=True, status="ESTIMATED", reason="",
        derived={"forward_m": 437.9, "right_m": -19.2, "down_m": -8.2, "range_m": 438.4, "ray_angle_deg": 3.2},
    )
    fields = dict(
        measurement_id="run-1", file_prefix="20260907_151917", workbook_path=None, state="STOPPED",
        revision=1, started_local_iso="", started_utc_ns=0, started_monotonic_ns=0,
        stopped_local_iso="", stopped_utc_ns=None, stopped_monotonic_ns=None, duration_s=None,
        metadata={}, calibration={}, alignment_rows=(), detections={}, pairs=(pair,),
        method_results=(rtk, yaw90), telemetry={}, events=(), artifacts=(), storage={},
        running=False, pending_method_jobs=0,
    )
    fields.update(overrides)
    known = set(MeasurementSnapshot.__dataclass_fields__)
    return MeasurementSnapshot(**{key: value for key, value in fields.items() if key in known})


def test_method_compare_table_puts_methods_side_by_side() -> None:
    columns, rows = _method_compare_table(_snapshot())
    assert "Pixhawk_RTK_range_m" in columns and "Pixhawk_RP_GPS_Yaw_range_m" in columns
    assert columns[-1] == "yaw90_over_rtk_range_ratio"
    assert len(rows) == 1
    row = rows[0]
    assert row["Pixhawk_RTK_range_m"] == 263.0
    assert row["Pixhawk_RP_GPS_Yaw_range_m"] == 438.4
    assert row["disparity_px"] == 462.3
    assert abs(row["yaw90_over_rtk_range_ratio"] - 438.4 / 263.0) < 1e-9


def test_optical_pose_rows_from_alignment_and_events() -> None:
    alignment = {
        "event_type": "optical_pose_auto_suggestion_at_start",
        "recorded_utc_ns": 1788761400000000000,
        "mode": "cloud",
        "source": "auto-cloud-1",
        "inlier_count": 226,
        "relative_yaw_right_minus_left_deg": -0.03,
    }
    event = EventRow(
        event_type="optical_pose_auto_suggestion_created",
        utc_ns=1788761500000000000,
        monotonic_ns=2,
        payload={
            "mode": "mountain",
            "suggestion_id": "auto-mountain-2",
            "suggestion": {
                "suggestion_id": "auto-mountain-2",
                "mode": "mountain",
                "state": "GOOD",
                "inlier_count": 30,
                "relative_rotation": {"available": True, "yaw_right_minus_left_deg": -0.18},
                "gnss_baseline": {"baseline_m": 25.5, "fix_quality": "DEGRADED"},
            },
        },
    )
    rows = _optical_pose_rows(_snapshot(alignment_rows=(alignment,), events=(event,)))
    assert [row["record_source"] for row in rows] == ["alignment", "event"]
    assert rows[0]["suggestion_id"] == "auto-cloud-1"
    assert rows[0]["relative_yaw_right_minus_left_deg"] == -0.03
    assert rows[1]["suggestion_id"] == "auto-mountain-2"
    assert rows[1]["relative_yaw_right_minus_left_deg"] == -0.18
    assert rows[1]["gnss_baseline_m"] == 25.5
