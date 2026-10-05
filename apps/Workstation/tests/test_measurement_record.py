"""Measurement lifecycle: Start->Stop ownership, late outcomes, isolation."""

from __future__ import annotations

from datetime import datetime, timedelta
from pathlib import Path

from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.measurement_record import (
    RECORD_STATE_STOPPED,
    RECORD_STATE_STOPPED_DRAINING,
    STATUS_NOT_ATTEMPTED,
    STATUS_PENDING,
    MeasurementRegistry,
    detection_id_for,
    telemetry_sheet_for,
)
from Workstation.tests.measurement_fixtures import (
    MEASUREMENT_ID,
    STARTED_LOCAL,
    checkerboard_attempt,
    detection_observation,
    frozen_snapshot,
    outcome_for,
    pixhawk_attempt,
    px4_row,
)


class _FakeWriter:
    def __init__(self) -> None:
        self.requests: list[tuple[str, str]] = []
        self.now_calls: list[str] = []

    def request(self, record, *, reason: str = "") -> None:
        self.requests.append((record.measurement_id, reason))

    def write_now(self, record, *, reason: str = "", timeout_s: float = 5.0) -> bool:
        self.now_calls.append(record.measurement_id)
        return True


def _registry(tmp_path: Path) -> tuple[MeasurementRegistry, _FakeWriter]:
    writer = _FakeWriter()
    clock = iter(STARTED_LOCAL + timedelta(seconds=index) for index in range(100))
    registry = MeasurementRegistry(
        data_root_provider=lambda: tmp_path,
        writer=writer,
        clock_local=lambda: next(clock),
    )
    return registry, writer


def _begin(registry: MeasurementRegistry, measurement_id: str, *, utc_ns: int = 1_757_000_000_000_000_000):
    return registry.begin(measurement_id, started_utc_ns=utc_ns, started_monotonic_ns=1_000_000)


def test_start_allocates_prefixed_workbook_and_same_second_gets_suffix(tmp_path: Path) -> None:
    writer = _FakeWriter()
    registry = MeasurementRegistry(
        data_root_provider=lambda: tmp_path,
        writer=writer,
        clock_local=lambda: STARTED_LOCAL,
    )

    first = _begin(registry, "run-1")
    registry.stop("run-1", stopped_utc_ns=2, stopped_monotonic_ns=2_000_000)
    second = _begin(registry, "run-2")

    assert first.workbook_path == tmp_path / "20260906_143005_measurement.xlsx"
    assert second.workbook_path == tmp_path / "20260906_143005-01_measurement.xlsx"
    assert first.file_prefix == "20260906_143005"
    assert second.file_prefix == "20260906_143005-01"
    assert [reason for _id, reason in writer.requests] == ["start", "stop", "start"]


def test_workbook_allocation_failure_never_blocks_start(tmp_path: Path) -> None:
    blocked = tmp_path / "file-not-dir"
    blocked.write_text("x")
    registry = MeasurementRegistry(
        data_root_provider=lambda: blocked / "child",
        writer=_FakeWriter(),
        clock_local=lambda: STARTED_LOCAL,
    )

    record = _begin(registry, "run-1")

    assert record.running
    assert record.workbook_path is None
    assert record.storage_snapshot()["state"] == "FAILED"
    assert any(event.event_type == "STORAGE_FAILURE" for event in record.snapshot().events)


def test_pairs_and_all_methods_are_tracked_with_not_attempted_reasons(tmp_path: Path) -> None:
    registry, _writer = _registry(tmp_path)
    record = _begin(registry, MEASUREMENT_ID)
    left = detection_observation("left", 340.0, 1.0, candidates=((100.0, 0.4, 1), (340.0, 0.95, 1), (500.0, 0.7, 2)))
    right = detection_observation("right", 300.0, 1.01)
    registry.observe_detection("left", left)
    registry.observe_detection("right", right)
    attempt = checkerboard_attempt(1)
    snapshot = frozen_snapshot(attempt)

    pair = record.register_pair(
        snapshot.canonical_payload(),
        submitted_method_ids=("checkerboard_stereo",),
        not_attempted={method_id: f"{method_id} not ready" for method_id in MEASUREMENT_METHOD_IDS if method_id != "checkerboard_stereo"},
    )

    view = record.snapshot()
    assert len(view.detections["left"]) == 3
    assert [row.selected for row in view.detections["left"]] == [False, True, False]
    assert view.detections["left"][1].detection_id == detection_id_for("left", left.timestamp_metadata, 1)
    assert pair.left_detection_id == view.detections["left"][0].detection_id.rsplit(":", 1)[0] + ":0"
    statuses = {row.method_id: row.status for row in view.method_results}
    assert statuses["checkerboard_stereo"] == STATUS_PENDING
    assert all(statuses[method_id] == STATUS_NOT_ATTEMPTED for method_id in MEASUREMENT_METHOD_IDS if method_id != "checkerboard_stereo")
    assert record.counters["terrain_pnp"][STATUS_NOT_ATTEMPTED] == 1
    assert record.pending_method_count() == 1


def test_late_outcome_updates_stopped_record_not_the_new_one(tmp_path: Path) -> None:
    registry, writer = _registry(tmp_path)
    first = _begin(registry, MEASUREMENT_ID)
    attempt = checkerboard_attempt(1)
    snapshot = frozen_snapshot(attempt)
    first.register_pair(snapshot.canonical_payload(), submitted_method_ids=("checkerboard_stereo",), not_attempted={})
    registry.stop(MEASUREMENT_ID, stopped_utc_ns=5, stopped_monotonic_ns=5_000_000)
    assert first.state == RECORD_STATE_STOPPED_DRAINING
    assert registry.is_current(MEASUREMENT_ID) is False

    second = _begin(registry, "run-second")
    assert registry.current() is second
    assert registry.is_current("run-second") is True
    assert registry.is_current("never-seen") is None

    routed = registry.route_outcome(outcome_for(attempt, snapshot))

    assert routed is first
    assert first.state == RECORD_STATE_STOPPED
    assert first.pending_method_count() == 0
    row = next(r for r in first.snapshot().method_results if r.method_id == "checkerboard_stereo")
    assert row.status in {"VALID", "ESTIMATED"}
    assert row.derived is not None and row.derived["range_m"] > 0.0
    assert second.snapshot().method_results == ()
    assert second.counters["checkerboard_stereo"]["VALID"] == 0
    assert writer.requests[-1] == (MEASUREMENT_ID, "outcome")
    registry.tick()
    assert registry.record_for(MEASUREMENT_ID) is first  # not yet written
    first.note_written_revision(first.revision)
    registry.tick()
    assert registry.record_for(MEASUREMENT_ID) is None
    assert registry.record_for("run-second") is second


def test_stop_without_pending_jobs_is_final_and_records_times(tmp_path: Path) -> None:
    registry, _writer = _registry(tmp_path)
    record = _begin(registry, MEASUREMENT_ID)

    registry.stop(MEASUREMENT_ID, stopped_utc_ns=7_000, stopped_monotonic_ns=9_000_000)

    view = record.snapshot()
    assert record.state == RECORD_STATE_STOPPED
    assert view.stopped_utc_ns == 7_000
    assert view.duration_s == (9_000_000 - 1_000_000) * 1e-9
    assert [event.event_type for event in view.events] == ["MEASUREMENT_STARTED", "MEASUREMENT_STOPPED"]
    assert datetime.fromisoformat(view.stopped_local_iso) > datetime.fromisoformat(view.started_local_iso)


def test_telemetry_rows_route_to_side_specific_sheets_without_downsampling(tmp_path: Path) -> None:
    registry, _writer = _registry(tmp_path)
    record = _begin(registry, MEASUREMENT_ID)
    for sequence in range(250):
        registry.observe_store_record("px4.left", px4_row("left", "pixhawk_attitude", sequence, roll_deg=0.1 * sequence))
    registry.observe_store_record("px4.right", px4_row("right", "gps_rtk", 0, latitude_deg=37.0))
    registry.observe_store_record("px4.right", px4_row("right", "magnetometer", 1, mag_x=1.0))
    registry.observe_store_record("px4.drone", px4_row("drone", "mavlink_sys_status", 0, voltage_battery=12.6))
    registry.observe_store_record("px4.drone", px4_row("drone", "mavlink_local_position_ned", 1, x=1.0))
    registry.observe_store_record("gnss.base", {"sample_type": "gnss_base_position", "latitude_deg": 37.1})
    registry.observe_store_record("gnss.baseline.center", {"sample_type": "gnss_baseline_center_frd", "forward_m": 12.0})
    registry.observe_store_record("measurements", {"measurement_id": MEASUREMENT_ID, "distance_m": 1.0})
    registry.observe_store_record("events", {"event_type": "storage_recovered", "runtime_generation": MEASUREMENT_ID})
    registry.stop(MEASUREMENT_ID, stopped_utc_ns=5, stopped_monotonic_ns=5_000_000)
    registry.observe_store_record("px4.left", px4_row("left", "pixhawk_attitude", 999))

    view = record.snapshot()
    assert len(view.telemetry["20_PX4L_Attitude"]) == 250
    assert [row["sequence"] for row in view.telemetry["20_PX4L_Attitude"]][:3] == [0, 1, 2]
    assert len(view.telemetry["31_PX4R_GNSS_RTK"]) == 1
    assert len(view.telemetry["32_PX4R_IMU_Mag"]) == 1
    assert len(view.telemetry["45_Drone_System"]) == 1
    assert len(view.telemetry["42_Drone_Position"]) == 1
    assert len(view.telemetry["43_Base_GNSS"]) == 1
    assert len(view.telemetry["46_GNSS_Baseline_Center"]) == 1
    assert "measurements" not in view.telemetry
    assert any(event.event_type == "storage_recovered" for event in view.events)
    assert telemetry_sheet_for("px4.left", {"sample_type": "unknown_thing"}) == "25_PX4L_Other"
    assert registry.observer_errors == 0


def test_shutdown_marks_pending_jobs_failed_and_writes_every_record(tmp_path: Path) -> None:
    registry, writer = _registry(tmp_path)
    first = _begin(registry, MEASUREMENT_ID)
    attempt = pixhawk_attempt(1)
    snapshot = frozen_snapshot(attempt)
    first.register_pair(snapshot.canonical_payload(), submitted_method_ids=("pixhawk_rtk_live",), not_attempted={})
    registry.stop(MEASUREMENT_ID, stopped_utc_ns=5, stopped_monotonic_ns=5_000_000)
    second = _begin(registry, "run-second")

    registry.finalize_all("application shutdown before method completion")

    row = next(r for r in first.snapshot().method_results if r.method_id == "pixhawk_rtk_live")
    assert row.status == "FAILED"
    assert "application shutdown" in row.reason
    assert first.state == RECORD_STATE_STOPPED
    assert any(event.event_type == "METHOD_FAILED" for event in first.snapshot().events)
    assert any(event.event_type == "APPLICATION_SHUTDOWN" for event in second.snapshot().events)
    assert sorted(writer.now_calls) == sorted([MEASUREMENT_ID, "run-second"])


def test_observer_exceptions_are_counted_not_raised(tmp_path: Path) -> None:
    registry, _writer = _registry(tmp_path)
    _begin(registry, MEASUREMENT_ID)

    registry.observe_detection("left", object())
    registry.observe_store_record("px4.left", None)  # type: ignore[arg-type]

    assert registry.observer_errors == 2
    assert registry.current() is not None
