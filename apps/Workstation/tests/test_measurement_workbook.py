"""Workbook layout, N/A handling, exact integers, atomic publish, writer thread."""

from __future__ import annotations

import time
import zipfile
from datetime import timedelta
from pathlib import Path

import pytest

from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.measurement_record import (
    ALWAYS_PRESENT_TELEMETRY_SHEETS,
    METHOD_SHEET_NAMES,
    MeasurementRegistry,
)
from Workstation.workstation_core.measurement_workbook import (
    FLAT_COLUMNS,
    SECTION_A,
    SECTION_A_COLUMNS,
    SECTION_B,
    SECTION_C,
    SECTION_D,
    SECTION_E,
    MeasurementWorkbookWriter,
    write_measurement_workbook,
)
from Workstation.workstation_core.measurement_xlsx_read import (
    find_marker_row,
    read_workbook,
    table_rows,
)
from Workstation.tests.measurement_fixtures import (
    MEASUREMENT_ID,
    STARTED_LOCAL,
    checkerboard_attempt,
    detection_observation,
    frozen_snapshot,
    intrinsics,
    outcome_for,
    pixhawk_attempt,
    px4_row,
)

EXPECTED_SHEETS = (
    "00_Summary",
    "01_Left_Detections",
    "02_Right_Detections",
    "03_Calibration",
    "04_Alignment",
    *(METHOD_SHEET_NAMES[method_id] for method_id in MEASUREMENT_METHOD_IDS),
    *ALWAYS_PRESENT_TELEMETRY_SHEETS,
    "50_Events",
    "90_Flat",
    "91_Exact",
    "98_Provenance",
    "99_Definitions",
)


def _populated_record(tmp_path: Path, *, stop: bool = True):
    clock = iter(STARTED_LOCAL + timedelta(seconds=index) for index in range(100))
    registry = MeasurementRegistry(data_root_provider=lambda: tmp_path, clock_local=lambda: next(clock))
    record = registry.begin(
        MEASUREMENT_ID,
        started_utc_ns=1_757_000_000_000_000_000,
        started_monotonic_ns=1_000_000,
        metadata={"pair_window_s": 0.1},
        calibration={
            "profile_sha256": "c" * 64,
            "source_path": "calibration/zoom08.json",
            "left": {
                "width_px": 640,
                "height_px": 480,
                "camera_matrix": ((900.0, 0.0, 320.0), (0.0, 901.5, 240.25), (0.0, 0.0, 1.0)),
                "distortion_coefficients": (0.01, -0.002, 0.0001, -0.0002, 0.0),
                "reprojection_error_px": 0.2,
                "image_count": 20,
            },
            "right": {
                "width_px": 640,
                "height_px": 480,
                "camera_matrix": ((900.0, 0.0, 320.0), (0.0, 901.5, 240.25), (0.0, 0.0, 1.0)),
                "distortion_coefficients": (0.01, -0.002, 0.0001, -0.0002, 0.0),
                "reprojection_error_px": 0.2,
                "image_count": 20,
            },
        },
    )
    record.add_alignment_row(
        {
            "timestamp_utc_ns": 1_757_000_000_000_000_001,
            "event_type": "applied_at_start",
            "method_id": "checkerboard_stereo",
            "source": "profile:zoom08",
            "R11": 1.0, "R12": 0.0, "R13": 0.0, "R21": 0.0, "R22": 1.0, "R23": 0.0, "R31": 0.0, "R32": 0.0, "R33": 1.0,
            "T_R": -0.4, "T_D": 0.0, "T_F": 0.0,
            "source_sha256": "d" * 64,
        }
    )
    registry.observe_detection("left", detection_observation("left", 340.0, 1.0, candidates=((340.0, 0.95, 1), (100.0, 0.5, 1))))
    registry.observe_detection("right", detection_observation("right", 300.0, 1.01))
    for sequence in range(3):
        registry.observe_store_record("px4.left", px4_row("left", "pixhawk_attitude", sequence, roll_deg=0.5 * sequence, q_w=1.0))
    registry.observe_store_record("px4.left", px4_row("left", "gps_rtk", 0, gps_utc_us=1_757_000_000_123_456, latitude_deg=37.0))
    registry.observe_store_record("gnss.base", {"measurement_id": MEASUREMENT_ID, "sample_type": "gnss_base_position", "latitude_deg": 37.1, "ellipsoid_height_m": 55.5})
    registry.observe_store_record("artifacts", {"event_type": "video_segment_allocated", "runtime_generation": MEASUREMENT_ID, "path": "20260906_143005_left.mp4"})

    good = checkerboard_attempt(1)
    good_snapshot = frozen_snapshot(good)
    record.register_pair(
        good_snapshot.canonical_payload(),
        submitted_method_ids=("checkerboard_stereo",),
        not_attempted={m: "no candidate" for m in MEASUREMENT_METHOD_IDS if m != "checkerboard_stereo"},
        method_evidence={"checkerboard_stereo": {"instance_id": "checkerboard:profile-a", "profile_id": "zoom08", "component_sources": {"left.roll_deg": "checkerboard"}}},
    )
    registry.route_outcome(outcome_for(good, good_snapshot))

    live = pixhawk_attempt(2, timestamp_s=2.0)
    live_snapshot = frozen_snapshot(live)
    record.register_pair(
        live_snapshot.canonical_payload(),
        submitted_method_ids=("pixhawk_rtk_live", "checkerboard_stereo"),
        not_attempted={m: "no candidate" for m in MEASUREMENT_METHOD_IDS if m not in {"pixhawk_rtk_live", "checkerboard_stereo"}},
    )
    registry.route_outcome(outcome_for(live, live_snapshot))

    degenerate = checkerboard_attempt(3, timestamp_s=3.0, intrinsics_snapshot=intrinsics(translation_left_to_right_m=(0.0, 0.0, 0.0)))
    degenerate_snapshot = frozen_snapshot(degenerate)
    record.register_pair(degenerate_snapshot.canonical_payload(), submitted_method_ids=("checkerboard_stereo",), not_attempted={})
    registry.route_outcome(outcome_for(degenerate, degenerate_snapshot))
    if stop:
        registry.stop(MEASUREMENT_ID, stopped_utc_ns=1_757_000_012_000_000_000, stopped_monotonic_ns=13_000_000)
    return registry, record


def test_workbook_has_every_sheet_and_sections_in_the_same_order(tmp_path: Path) -> None:
    _registry, record = _populated_record(tmp_path)
    path = write_measurement_workbook(record.snapshot(), record.workbook_path)

    sheets = read_workbook(path)
    assert path.name == "20260906_143005_measurement.xlsx"
    assert [name for name in sheets if name in EXPECTED_SHEETS] == list(EXPECTED_SHEETS)
    assert "46_GNSS_Baseline_Center" not in sheets
    for method_id in MEASUREMENT_METHOD_IDS:
        grid = sheets[METHOD_SHEET_NAMES[method_id]]
        markers = [find_marker_row(grid, marker) for marker in (SECTION_A, SECTION_B, SECTION_C, SECTION_D, SECTION_E)]
        assert markers == sorted(markers)
        header = grid[markers[0] + 1]
        assert tuple(header[: len(SECTION_A_COLUMNS)]) == SECTION_A_COLUMNS


def test_method_sheets_keep_not_attempted_pending_and_blocked_rows(tmp_path: Path) -> None:
    _registry, record = _populated_record(tmp_path)
    path = write_measurement_workbook(record.snapshot(), record.workbook_path)
    sheets = read_workbook(path)

    checker = sheets[METHOD_SHEET_NAMES["checkerboard_stereo"]]
    rows = table_rows(checker, find_marker_row(checker, SECTION_A) + 1)
    assert [row["Attempted"] for row in rows] == ["YES", "YES", "YES"]
    assert rows[0]["Status"] in {"VALID", "ESTIMATED"}
    assert rows[1]["Status"] == "PENDING"
    assert rows[1]["Forward"] == "N/A"
    assert rows[2]["Status"] == "BLOCKED"
    assert rows[2]["Reason"] == "zero_baseline"
    assert rows[2]["Range"] == "N/A"
    assert rows[2]["R11"] == pytest.approx(0.9998476951563913)
    assert isinstance(rows[0]["time_utc_ns"], str)
    assert rows[0]["time_utc_ns"] == str(1_757_000_001_000_000_000)

    terrain = sheets[METHOD_SHEET_NAMES["terrain_pnp"]]
    terrain_rows = table_rows(terrain, find_marker_row(terrain, SECTION_A) + 1)
    assert [row["Attempted"] for row in terrain_rows] == ["NO", "NO", "NO"]
    assert {row["Status"] for row in terrain_rows} == {"N/A"}
    assert terrain_rows[0]["Reason"] == "no candidate"

    pixhawk = sheets[METHOD_SHEET_NAMES["pixhawk_rtk_live"]]
    a_rows = table_rows(pixhawk, find_marker_row(pixhawk, SECTION_A) + 1)
    live_row = next(row for row in a_rows if row["Attempted"] == "YES")
    assert live_row["Status"] in {"VALID", "ESTIMATED"}
    assert live_row["Range"] > 0
    c_rows = table_rows(pixhawk, find_marker_row(pixhawk, SECTION_C) + 1)
    c_live = next(row for row in c_rows if row["L_u"] != "N/A")
    assert c_live["L_u"] == 340.0 and abs(c_live["L_ray_F"]) < 1.0
    d_rows = table_rows(pixhawk, find_marker_row(pixhawk, SECTION_D) + 1)
    fields = {row["field"] for row in d_rows}
    assert {"left_attitude.quaternion_wxyz.w", "right_gps.latitude_deg", "baseline.ned_right_from_left_m.E", "component_source.left.roll_deg"} <= fields
    gps_time = next(row for row in d_rows if row["field"] == "left_gps.gps_utc_us")
    assert gps_time["value"] == str(1_757_000_000_123_456)
    e_rows = table_rows(pixhawk, find_marker_row(pixhawk, SECTION_E) + 1)
    e_live = next(row for row in e_rows if row["triangulation_reason"] == "ok")
    assert e_live["left_depth_m"] > 0 and e_live["ray_angle_deg"] > 0


def test_detection_calibration_alignment_summary_and_flat_sheets(tmp_path: Path) -> None:
    _registry, record = _populated_record(tmp_path)
    path = write_measurement_workbook(record.snapshot(), record.workbook_path)
    sheets = read_workbook(path)

    left = table_rows(sheets["01_Left_Detections"])
    assert [row["selected"] for row in left] == ["YES", "NO"]
    assert left[0]["app_receive_monotonic_ns"] == "1000000000"
    assert left[0]["x1"] == 335.0 and left[0]["class_name"] == "target"

    calibration = table_rows(sheets["03_Calibration"])
    assert [row["side"] for row in calibration] == ["left", "right"]
    assert calibration[0]["K22"] == 901.5 and calibration[0]["D_count"] == 5 and calibration[0]["D6"] == "N/A"
    assert calibration[0]["profile_sha256"] == "c" * 64

    alignment = table_rows(sheets["04_Alignment"])
    assert alignment[0]["method_id"] == "checkerboard_stereo" and alignment[0]["T_R"] == -0.4
    assert alignment[0]["timestamp_utc_ns"] == "1757000000000000001"

    summary = sheets["00_Summary"]
    keys = {row[0]: row[1] for row in summary if len(row) > 1 and isinstance(row[0], str)}
    assert keys["Measurement ID"] == MEASUREMENT_ID
    assert keys["Stereo pair count"] == 3
    assert keys["Stop local time"] != "N/A"
    header_row = next(index for index, row in enumerate(summary) if row and row[0] == "Method")
    method_rows = table_rows(summary, header_row)
    checker = next(row for row in method_rows if row["Method"] == "checkerboard_stereo")
    assert checker["Attempted count"] == 3 and checker["PENDING count"] == 1 and checker["BLOCKED count"] == 1
    assert checker["Range min"] == checker["Range max"] and checker["Range min"] > 0
    terrain = next(row for row in method_rows if row["Method"] == "terrain_pnp")
    assert terrain["N/A count"] == 3 and terrain["Range min"] == "N/A"
    with zipfile.ZipFile(path) as archive:
        summary_xml = archive.read("xl/worksheets/sheet1.xml").decode()
    assert "location=\"&apos;10_Pixhawk_RTK&apos;!A1\"" in summary_xml or "10_Pixhawk_RTK" in summary_xml

    flat = table_rows(sheets["90_Flat"])
    assert tuple(sheets["90_Flat"][0][: len(FLAT_COLUMNS)]) == FLAT_COLUMNS
    assert len(flat) == 3 * len(MEASUREMENT_METHOD_IDS)
    assert {row["method_id"] for row in flat} == set(MEASUREMENT_METHOD_IDS)

    telemetry = table_rows(sheets["20_PX4L_Attitude"])
    assert len(telemetry) == 3
    assert telemetry[0]["sample_monotonic_ns"] == "1000000000"
    columns = sheets["20_PX4L_Attitude"][0]
    assert columns[:4] == ["measurement_id", "side", "sample_type", "sequence"]
    assert columns.index("roll_deg") < columns.index("namespace")
    gnss = table_rows(sheets["21_PX4L_GNSS_RTK"])
    assert gnss[0]["gps_utc_us"] == "1757000000123456"
    assert table_rows(sheets["43_Base_GNSS"])[0]["ellipsoid_height_m"] == 55.5
    assert table_rows(sheets["22_PX4L_IMU_Mag"]) == []

    events = table_rows(sheets["50_Events"])
    assert [row["event_type"] for row in events][:1] == ["MEASUREMENT_STARTED"]
    assert "MEASUREMENT_STOPPED" in {row["event_type"] for row in events}
    provenance = {row["key"]: row["value"] for row in table_rows(sheets["98_Provenance"])}
    assert provenance["measurement_id"] == MEASUREMENT_ID
    assert "artifact.video_segment_allocated" in provenance
    assert "library.xlsxwriter" in provenance
    definitions = {row["term"] for row in table_rows(sheets["99_Definitions"])}
    assert {"Measurement", "RDF", "FRD", "NED", "Attempted", "N/A", "ray_gap_m", "time_delta_s"} <= definitions


def test_exact_sheet_round_trips_bits(tmp_path: Path) -> None:
    _registry, record = _populated_record(tmp_path)
    path = write_measurement_workbook(record.snapshot(), record.workbook_path)
    exact = table_rows(read_workbook(path)["91_Exact"])

    row = next(item for item in exact if item["method_id"] == "checkerboard_stereo" and item["field"] == "Range")
    assert float.fromhex(row["value_hex"]) == row["value"]
    assert float(row["value_repr"]) == row["value"]
    fields = {item["field"] for item in exact if item["method_id"] == "pixhawk_rtk_live"}
    assert {"R11", "T_R", "left.u", "left.ray_F", "left.q_w", "left.K11", "left.D5", "Forward", "ray_gap_m"} <= fields
    calibration = {item["field"] for item in exact if item["method_id"] == "calibration"}
    assert {"left.K11", "right.D5"} <= calibration


def test_failed_write_keeps_previous_workbook_and_leaves_no_temp(tmp_path: Path) -> None:
    _registry, record = _populated_record(tmp_path)
    path = write_measurement_workbook(record.snapshot(), record.workbook_path)
    before = path.read_bytes()

    broken = record.snapshot()
    object.__setattr__(broken, "method_results", ("not a row",))
    with pytest.raises(Exception):
        write_measurement_workbook(broken, path)

    assert path.read_bytes() == before
    assert [item.name for item in tmp_path.iterdir()] == [path.name]


def test_writer_thread_writes_on_stop_and_late_outcome_and_reports_failures(tmp_path: Path) -> None:
    writer = MeasurementWorkbookWriter(checkpoint_interval_s=0.0)
    try:
        clock = iter(STARTED_LOCAL + timedelta(seconds=index) for index in range(100))
        registry = MeasurementRegistry(data_root_provider=lambda: tmp_path, writer=writer, clock_local=lambda: next(clock))
        record = registry.begin(MEASUREMENT_ID, started_utc_ns=1, started_monotonic_ns=1_000_000)
        attempt = checkerboard_attempt(1)
        snapshot = frozen_snapshot(attempt)
        record.register_pair(snapshot.canonical_payload(), submitted_method_ids=("checkerboard_stereo",), not_attempted={})
        registry.stop(MEASUREMENT_ID, stopped_utc_ns=5, stopped_monotonic_ns=5_000_000)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and record.storage_snapshot()["write_count"] < 1:
            time.sleep(0.02)
        assert record.workbook_path.exists()
        first_rows = table_rows(read_workbook(record.workbook_path)["90_Flat"])
        assert next(r for r in first_rows if r["method_id"] == "checkerboard_stereo")["Status"] == "PENDING"

        registry.route_outcome(outcome_for(attempt, snapshot))
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and record.written_revision < record.revision:
            time.sleep(0.02)
        rows = table_rows(read_workbook(record.workbook_path)["90_Flat"])
        assert next(r for r in rows if r["method_id"] == "checkerboard_stereo")["Status"] in {"VALID", "ESTIMATED"}
        assert record.storage_snapshot()["state"] == "READY"

        record.workbook_path.unlink()
        record.workbook_path.mkdir()  # a directory in the way -> os.replace fails
        record.add_event("PROBE")
        writer.request(record, reason="stop")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and record.storage_snapshot()["failure_count"] < 1:
            time.sleep(0.02)
        status = record.storage_snapshot()
        assert status["state"] == "FAILED" and status["last_error"]
        assert any(event.event_type == "STORAGE_FAILURE" for event in record.snapshot().events)
        assert writer.snapshot()["failure_count"] >= 1
        assert not [item for item in tmp_path.iterdir() if item.name.startswith(".")]
    finally:
        assert writer.close(timeout_s=5.0)
