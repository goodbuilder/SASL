import csv
import json
import math
from pathlib import Path
from xml.etree import ElementTree
from zipfile import ZipFile

from wiris_stereo_distance.recording_session import (
    CAMERA_TIMELINE_COMPACT_COLUMNS,
    compact_px4_timeline_rows,
    discard_retryable_session,
    DISTANCE_MEASUREMENT_COLUMNS,
    GNSS_BASE_COMPACT_COLUMNS,
    PX4_TIMELINE_COMPACT_COLUMNS,
    SpoolRows,
    discover_retryable_sessions,
    format_video_paths,
    load_retryable_session,
    raw_video_artifact_errors,
    raw_video_paths,
    save_session_files,
    session_dir,
    session_id_now,
    session_sheet_previews,
    write_recovery_metadata,
)


def test_recording_session_paths_use_stable_session_layout() -> None:
    root = Path("/tmp/recordings")

    assert session_dir(root, "20260605_120000") == (
        root / "sessions" / "20260605_120000"
    )
    assert raw_video_paths(root, "20260605_120000") == {
        "left": root / "sessions" / "20260605_120000" / "video" / "raw_left.mp4",
        "right": root / "sessions" / "20260605_120000" / "video" / "raw_right.mp4",
    }


def test_format_video_paths_handles_empty_and_populated_paths() -> None:
    assert format_video_paths({}) == "저장 경로 없음"
    assert (
        format_video_paths({"left": Path("/tmp/session/video/raw_left.mp4")})
        == "left=/tmp/session/video/raw_left.mp4"
    )


def test_raw_video_artifact_validation_requires_mp4_for_received_frames(tmp_path) -> None:
    left = tmp_path / "raw_left.mp4"
    right = tmp_path / "raw_right.mp4"
    left.write_bytes(b"\x00\x00\x00\x20ftypisom")
    right.write_bytes(b"")

    errors = raw_video_artifact_errors(
        {"left": left, "right": right},
        {"left": 12, "right": 8},
    )

    assert not any("left" in error for error in errors)
    assert any("right MP4가 0바이트" in error for error in errors)


def test_raw_video_artifact_validation_ignores_camera_without_frames(tmp_path) -> None:
    assert raw_video_artifact_errors(
        {"left": tmp_path / "missing.mp4"},
        {"left": 0, "right": 0},
    ) == ()


def test_compact_px4_timeline_preserves_mag_samples_and_slot_status() -> None:
    rows = compact_px4_timeline_rows(
        [
            {
                "sample_type": "magnetometer",
                "topic": "/uav_0/fmu/out/sensor_mag_1",
                "sensor_slot": "CAN1",
                "mag_value_units": "gauss",
                "mag_device_type": "RM3100",
                "mag_x": 1.25,
                "mag_y": -2.5,
                "mag_z": 3.75,
                "mag_vector_norm": 4.677,
                "mag_temperature_c": 24.5,
                "mag_error_count": 2,
                "mag_px4_timestamp_s": 12.3,
                "device_id": 8944899,
            },
            {
                "sample_type": "mag_slot_status",
                "topic": "px4_mag_manager",
                "sensor_slot": "CAL_MAG1",
                "mag_slot": 1,
                "mag_cal_slot": "CAL_MAG1",
                "mag_device_id": 8944899,
                "mag_device_type": "RM3100",
                "mag_interface": "DroneCAN",
                "mag_is_rm3100": True,
                "mag_priority": 100,
                "mag_rotation": 0,
                "mag_enabled": True,
                "mag_expected_selected": True,
                "mag_actual_selected": True,
                "mag_expected_selected_device_id": 8944899,
                "mag_actual_selected_device_id": 8944899,
                "mag_is_actual_rm3100": True,
                "mag_armed": False,
                "mag_status": "Actual selected",
                "mag_warning_text": "ok",
                "mag_rm3100_primary_id": 8944899,
                "mag_rm3100_backup_id": 8944387,
                "mag_param_uavcan_enable": 2,
                "mag_param_uavcan_sub_mag": 1,
                "mag_param_ekf2_mag_type": 0,
                "mag_param_snapshot_json": "{\"CAL_MAG1_PRIO\":100}",
            },
        ]
    )

    sample, status = rows
    assert sample["sample_type"] == "magnetometer"
    assert sample["mag_sensor_slot"] == "CAN1"
    assert sample["mag_x"] == 1.25
    assert sample["mag_y"] == -2.5
    assert sample["mag_z"] == 3.75
    assert sample["mag_vector_norm"] == 4.677
    assert status["sample_type"] == "mag_slot_status"
    assert status["mag_cal_slot"] == "CAL_MAG1"
    assert status["mag_priority"] == 100
    assert status["mag_actual_selected"] is True
    assert status["mag_status"] == "Actual selected"
    assert status["mag_param_uavcan_sub_mag"] == 1
    assert "mag_device_id" not in sample
    assert "mag_device_type" not in sample
    assert "mag_rm3100_primary_id" not in status


def test_compact_px4_timeline_widens_mavlink_mag_streams() -> None:
    rows = compact_px4_timeline_rows(
        [
            {
                "sequence": 10,
                "recording_time_ns": 1_000_000_000,
                "sample_type": "magnetometer",
                "topic": "mavlink/SCALED_IMU",
                "mag_value_units": "milligauss",
                "mag_x": 1.0,
                "mag_y": 2.0,
                "mag_z": 3.0,
            },
            {
                "sequence": 11,
                "recording_time_ns": 1_008_000_000,
                "sample_type": "magnetometer",
                "topic": "mavlink/SCALED_IMU2",
                "mag_value_units": "milligauss",
                "mag_x": 4.0,
                "mag_y": 5.0,
                "mag_z": 6.0,
            },
            {
                "sequence": 12,
                "recording_time_ns": 1_016_000_000,
                "sample_type": "magnetometer",
                "topic": "mavlink/SCALED_IMU3",
                "mag_value_units": "milligauss",
                "mag_x": 7.0,
                "mag_y": 8.0,
                "mag_z": 9.0,
            },
        ]
    )

    assert len(rows) == 1
    wide = rows[0]
    assert wide["topic"] == "mavlink/SCALED_IMU"
    assert wide["mag_scaled_imu_x"] == 1.0
    assert wide["mag_scaled_imu_y"] == 2.0
    assert wide["mag_scaled_imu_z"] == 3.0
    assert wide["mag_scaled_imu_unit"] == "milligauss"
    assert wide["mag_scaled_imu2_x"] == 4.0
    assert wide["mag_scaled_imu2_y"] == 5.0
    assert wide["mag_scaled_imu2_z"] == 6.0
    assert wide["mag_scaled_imu3_x"] == 7.0
    assert wide["mag_scaled_imu3_y"] == 8.0
    assert wide["mag_scaled_imu3_z"] == 9.0


def test_compact_px4_timeline_keeps_unmatched_mavlink_mag_rows() -> None:
    rows = compact_px4_timeline_rows(
        [
            {
                "recording_time_ns": 1_000_000_000,
                "sample_type": "magnetometer",
                "topic": "mavlink/SCALED_IMU",
                "mag_value_units": "milligauss",
                "mag_x": 1.0,
                "mag_y": 2.0,
                "mag_z": 3.0,
            },
            {
                "recording_time_ns": 1_100_000_000,
                "sample_type": "magnetometer",
                "topic": "mavlink/SCALED_IMU2",
                "mag_value_units": "milligauss",
                "mag_x": 4.0,
                "mag_y": 5.0,
                "mag_z": 6.0,
            },
        ]
    )

    assert len(rows) == 2
    assert rows[0]["mag_scaled_imu_x"] == 1.0
    assert rows[0]["mag_scaled_imu2_x"] is None
    assert rows[1]["mag_scaled_imu_x"] is None
    assert rows[1]["mag_scaled_imu2_x"] == 4.0


def test_compact_px4_timeline_widens_gps1_gps2_and_preserves_generated_utc() -> None:
    rows = compact_px4_timeline_rows(
        [
            {
                "sample_type": "gps",
                "topic": "mavlink/GPS_RAW_INT",
                "latitude_deg": 37.123456,
                "longitude_deg": 127.123456,
                "altitude_m": 42.5,
                "fix_type": 6,
                "satellites_used": 18,
                "gps_utc_us": 1_800_000_000_123_456,
            },
            {
                "sample_type": "gps",
                "topic": "mavlink/GPS2_RAW",
                "latitude_deg": 37.123457,
                "longitude_deg": 127.123457,
                "altitude_m": 42.7,
                "fix_type": 5,
                "satellites_used": 16,
                "gps_utc_us": 1_800_000_000_223_456,
            },
        ]
    )

    assert len(rows) == 2
    gps1, gps2 = rows
    assert gps1["gps1_latitude_deg"] == 37.123456
    assert gps1["gps1_longitude_deg"] == 127.123456
    assert gps1["gps1_altitude_m"] == 42.5
    assert gps1["gps1_fix_type"] == 6
    assert gps1["gps1_satellites_used"] == 18
    assert gps1["gps_utc_time_ns"] == 1_800_000_000_123_456_000
    assert gps1["gps_utc_source"] == "GPS1"
    assert gps1["gps2_latitude_deg"] is None
    assert gps2["gps1_latitude_deg"] is None
    assert gps2["gps2_latitude_deg"] == 37.123457
    assert gps2["gps2_altitude_m"] == 42.7
    assert gps2["gps2_fix_type"] == 5
    assert gps2["gps2_satellites_used"] == 16
    assert gps2["gps_utc_time_ns"] == 1_800_000_000_223_456_000
    assert gps2["gps_utc_source"] == "GPS2"


def test_save_session_files_publishes_mag_rows_to_px4_artifacts(tmp_path) -> None:
    saved = save_session_files(
        tmp_path / "mag_session",
        metadata={"session_id": "mag_session"},
        pair_rows=[],
        px4_rows={
            "left": [
                {
                    "sequence": 0,
                    "sample_monotonic_ns": 2_500_000_000,
                    "sample_type": "magnetometer",
                    "topic": "/uav_0/fmu/out/sensor_mag_1",
                    "sensor_slot": "CAN1",
                    "mag_value_units": "gauss",
                    "mag_device_type": "RM3100",
                    "mag_x": 0.1,
                    "mag_y": 0.2,
                    "mag_z": 0.3,
                    "mag_vector_norm": 0.3741657387,
                },
                {
                    "sequence": 1,
                    "sample_monotonic_ns": 2_600_000_000,
                    "sample_type": "mag_slot_status",
                    "topic": "px4_mag_manager",
                    "sensor_slot": "CAL_MAG1",
                    "mag_slot": 1,
                    "mag_cal_slot": "CAL_MAG1",
                    "mag_device_id": 8944899,
                    "mag_priority": 100,
                    "mag_enabled": True,
                    "mag_actual_selected": True,
                    "mag_status": "Actual selected",
                },
            ],
            "right": [
                {
                    "sequence": 0,
                    "sample_monotonic_ns": 2_500_000_000,
                    "sample_type": "magnetometer",
                    "topic": "/uav_1/fmu/out/sensor_mag_3",
                    "sensor_slot": "GPS1_MAG",
                    "mag_value_units": "gauss",
                    "mag_device_type": "GPS1 integrated IST8310 compass",
                    "mag_x": 1.1,
                    "mag_y": 1.2,
                    "mag_z": 1.3,
                    "mag_vector_norm": 2.0,
                }
            ],
        },
    )

    with saved.px4_left_csv.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["sample_type"] == "magnetometer"
    assert rows[0]["mag_sensor_slot"] == "CAN1"
    assert rows[0]["mag_x"] == "0.1"
    assert rows[0]["mag_y"] == "0.2"
    assert rows[0]["mag_z"] == "0.3"
    assert rows[0]["mag_vector_norm"] == "0.3741657387"
    assert rows[1]["sample_type"] == "mag_slot_status"
    assert rows[1]["mag_priority"] == "100"
    assert rows[1]["mag_actual_selected"] == "True"

    with saved.px4_right_csv.open("r", newline="", encoding="utf-8") as handle:
        right_rows = list(csv.DictReader(handle))
    assert right_rows[0]["mag_sensor_slot"] == "GPS1_MAG"
    assert right_rows[0]["mag_x"] == "1.1"
    assert right_rows[0]["mag_y"] == "1.2"
    assert right_rows[0]["mag_z"] == "1.3"
    assert right_rows[0]["mag_vector_norm"] == "2.0"

    xlsx_rows = _xlsx_table_rows(saved.xlsx, 5)
    assert xlsx_rows[0]["sample_type"] == "magnetometer"
    assert xlsx_rows[1]["mag_cal_slot"] == "CAL_MAG1"
    assert xlsx_rows[1]["mag_priority"] == 100


def _xlsx_sheet_names_and_row_counts(path: Path) -> tuple[list[str], list[int]]:
    spreadsheet_namespace = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
    relationship_namespace = (
        "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
    )
    package_relationship_namespace = (
        "http://schemas.openxmlformats.org/package/2006/relationships"
    )
    with ZipFile(path) as archive:
        workbook = ElementTree.fromstring(archive.read("xl/workbook.xml"))
        relationships = ElementTree.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
        targets = {
            relationship.attrib["Id"]: relationship.attrib["Target"]
            for relationship in relationships.findall(
                f"{{{package_relationship_namespace}}}Relationship"
            )
        }
        sheets = workbook.find(f"{{{spreadsheet_namespace}}}sheets")
        assert sheets is not None
        names: list[str] = []
        row_counts: list[int] = []
        for sheet in sheets:
            names.append(sheet.attrib["name"])
            relationship_id = sheet.attrib[f"{{{relationship_namespace}}}id"]
            target = targets[relationship_id].lstrip("/")
            worksheet_path = target if target.startswith("xl/") else f"xl/{target}"
            worksheet = ElementTree.fromstring(archive.read(worksheet_path))
            sheet_data = worksheet.find(f"{{{spreadsheet_namespace}}}sheetData")
            assert sheet_data is not None
            row_counts.append(len(sheet_data.findall(f"{{{spreadsheet_namespace}}}row")))
    return names, row_counts


def _xlsx_dynamic_axis_values(path: Path) -> list[list[tuple[str, str]]]:
    spreadsheet_namespace = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
    axes: list[list[tuple[str, str]]] = []
    with ZipFile(path) as archive:
        for sheet_index in range(2, 7):
            worksheet = ElementTree.fromstring(
                archive.read(f"xl/worksheets/sheet{sheet_index}.xml")
            )
            sheet_data = worksheet.find(f"{{{spreadsheet_namespace}}}sheetData")
            assert sheet_data is not None
            values: list[tuple[str, str]] = []
            for row in sheet_data.findall(f"{{{spreadsheet_namespace}}}row")[1:]:
                cells = row.findall(f"{{{spreadsheet_namespace}}}c")[:2]
                numeric = [
                    cell.find(f"{{{spreadsheet_namespace}}}v")
                    for cell in cells
                ]
                assert all(value is not None and value.text is not None for value in numeric)
                values.append((numeric[0].text, numeric[1].text))  # type: ignore[union-attr]
            axes.append(values)
    return axes


def _xlsx_sheet_headers(path: Path, sheet_index: int) -> list[str]:
    spreadsheet_namespace = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
    with ZipFile(path) as archive:
        shared_strings = ElementTree.fromstring(archive.read("xl/sharedStrings.xml"))
        strings = [
            "".join(
                node.text or ""
                for node in item.iter(f"{{{spreadsheet_namespace}}}t")
            )
            for item in shared_strings.findall(f"{{{spreadsheet_namespace}}}si")
        ]
        worksheet = ElementTree.fromstring(
            archive.read(f"xl/worksheets/sheet{sheet_index}.xml")
        )
        sheet_data = worksheet.find(f"{{{spreadsheet_namespace}}}sheetData")
        assert sheet_data is not None
        header_row = sheet_data.find(f"{{{spreadsheet_namespace}}}row")
        assert header_row is not None
        headers: list[str] = []
        for cell in header_row.findall(f"{{{spreadsheet_namespace}}}c"):
            value = cell.find(f"{{{spreadsheet_namespace}}}v")
            assert value is not None and value.text is not None
            headers.append(strings[int(value.text)] if cell.attrib.get("t") == "s" else value.text)
    return headers


def _xlsx_table_rows(path: Path, sheet_index: int) -> list[dict[str, object]]:
    spreadsheet_namespace = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"

    def column_index(cell_ref: str) -> int:
        letters = "".join(ch for ch in cell_ref if ch.isalpha())
        index = 0
        for ch in letters:
            index = index * 26 + (ord(ch.upper()) - ord("A") + 1)
        return index - 1

    with ZipFile(path) as archive:
        shared_strings = ElementTree.fromstring(archive.read("xl/sharedStrings.xml"))
        strings = [
            "".join(
                node.text or ""
                for node in item.iter(f"{{{spreadsheet_namespace}}}t")
            )
            for item in shared_strings.findall(f"{{{spreadsheet_namespace}}}si")
        ]
        worksheet = ElementTree.fromstring(
            archive.read(f"xl/worksheets/sheet{sheet_index}.xml")
        )
        sheet_data = worksheet.find(f"{{{spreadsheet_namespace}}}sheetData")
        assert sheet_data is not None
        xml_rows = sheet_data.findall(f"{{{spreadsheet_namespace}}}row")
        assert xml_rows

        def cell_value(cell):
            value = cell.find(f"{{{spreadsheet_namespace}}}v")
            if value is None or value.text is None:
                return ""
            if cell.attrib.get("t") == "s":
                return strings[int(value.text)]
            text = value.text
            try:
                number = float(text)
            except ValueError:
                return text
            return int(number) if number.is_integer() else number

        header_cells = xml_rows[0].findall(f"{{{spreadsheet_namespace}}}c")
        header_by_index = {
            column_index(cell.attrib["r"]): str(cell_value(cell))
            for cell in header_cells
        }
        headers = [
            header_by_index.get(index, "")
            for index in range(max(header_by_index) + 1)
        ]
        rows: list[dict[str, object]] = []
        for row in xml_rows[1:]:
            values = {
                column_index(cell.attrib["r"]): cell_value(cell)
                for cell in row.findall(f"{{{spreadsheet_namespace}}}c")
            }
            rows.append(
                {
                    header: values.get(index, "")
                    for index, header in enumerate(headers)
                    if header
                }
            )
        return rows


def test_save_session_files_writes_compact_resource_csvs_parquet_and_integrated_xlsx(tmp_path) -> None:
    output_dir = tmp_path / "sessions" / "20260605_120000"
    pair_rows = [
        {
            "sample_index": 7,
            "session_elapsed_s": 1.25,
            "measurement_frame": "baseline_center_frd",
            "distance_m": 5.4,
            "range_m": 5.4,
            "baseline_center_forward_m": 5.4,
            "baseline_center_right_m": 0.0,
            "baseline_center_down_m": 0.0,
            "source_left_camera_rdf_right_m": 0.15,
            "source_left_camera_rdf_down_m": 0.0,
            "source_left_camera_rdf_forward_m": 5.4,
            "left_gps_latitude_deg": 37.1,
            "left_roll_deg": 0.1,
            "baseline_m": 1.8,
            "pair_quality_status": "matched",
        },
        {
            "sample_index": 8,
            "session_elapsed_s": 1.30,
            "measurement_frame": "baseline_center_frd",
            "distance_m": 5.3,
            "range_m": 5.3,
            "baseline_center_forward_m": 5.3,
            "baseline_center_right_m": 0.0,
            "baseline_center_down_m": 0.0,
            "source_left_camera_rdf_right_m": 0.15,
            "source_left_camera_rdf_down_m": 0.0,
            "source_left_camera_rdf_forward_m": 5.3,
            "left_gps_latitude_deg": 37.2,
            "left_roll_deg": 0.2,
            "baseline_m": 1.8,
            "pair_quality_status": "matched",
        },
    ]
    saved = save_session_files(
        output_dir,
        metadata={
            "session_id": "20260605_120000",
            "pair_count": 2,
            "calibration": {"profile": "night"},
        },
        pair_rows=pair_rows,
        camera_rows={
            "left": [
                {
                    "sequence": 0,
                    "session_id": "20260605_120000",
                    "side": "left",
                    "sample_type": "gui_camera_frame",
                    "sample_monotonic_ns": 1_000_000_000,
                    "source_utc_ns": 1_800_000_000_000_000_000,
                    "rtp_timestamp": 90_000,
                    "rtp_ssrc": 11,
                    "rtp_marker_seq": 42,
                    "rtcp_sr_received_count": 3,
                    "rtcp_sr_rtp_timestamp": 45_000,
                    "rtcp_clock_rate_hz": 90_000,
                    "rtcp_sr_age_s": 0.25,
                    "rtcp_sr_pc_offset_s": -0.010,
                    "rtcp_frame_receive_delay_s": 0.055,
                    "rtcp_frame_transport_delay_s": 0.012,
                    "rtcp_camera_ntp_rate": 0.999,
                    "rtcp_camera_media_rate": 0.979,
                }
            ]
        },
        px4_rows={
            "right": [
                {
                    "sequence": 0,
                    "session_id": "20260605_120000",
                    "side": "right",
                    "sample_type": "gps_rtk",
                    "sample_monotonic_ns": 1_100_000_000,
                    "px4_timestamp_sample_us": 900_000,
                    "topic": "/uav_1/fmu/out/vehicle_gps_position",
                    "lat_deg": 37.123456789,
                    "lon_deg": 127.987654321,
                    "alt_m": 42.25,
                    "altitude_ellipsoid_m": 47.5,
                    "fix_type": 6,
                    "satellites": 18,
                    "eph_m": 0.012,
                    "epv_m": 0.034,
                    "rtk_label": "RTK Fixed",
                    "gps_utc_us": 1_800_000_000_123_456,
                    "heading_deg": 91.5,
                    "heading_accuracy_deg": 0.2,
                    "velocity_n_m_s": 1.25,
                    "velocity_e_m_s": -0.5,
                    "velocity_d_m_s": 0.125,
                    "velocity_ned_valid": "true",
                }
            ]
        },
        gnss_base_rows=[
            {
                "sequence": 0,
                "session_id": "20260605_120000",
                "side": "base",
                "sample_type": "gnss_base_position",
                "sample_monotonic_ns": 1_150_000_000,
                "session_utc_ns": 1_800_000_000_150_000_000,
                "host_receive_utc_ns": 1_800_000_000_200_000_000,
                "latitude_deg": 37.401234,
                "longitude_deg": 127.123456,
                "ellipsoid_height_m": 52.3,
                "msl_altitude_m": 48.1,
                "height_type": "ELLIPSOID",
                "fixed": True,
                "device_id": "base-f9p",
                "gnss_operation_state": "BASE_AS_ROVER_FIXED",
            }
        ],
    )

    assert saved.outputs == [
        saved.session_static_json,
        saved.distance_measurements_csv,
        saved.distance_measurements_parquet,
        saved.xlsx,
        saved.camera_left_csv,
        saved.camera_left_parquet,
        saved.camera_right_csv,
        saved.camera_right_parquet,
        saved.px4_left_csv,
        saved.px4_left_parquet,
        saved.px4_right_csv,
        saved.px4_right_parquet,
        saved.px4_left_xlsx,
        saved.px4_right_xlsx,
        saved.session_rtk_xlsx,
        saved.gnss_base_csv,
        saved.gnss_base_parquet,
    ]
    assert saved.csv == output_dir / "20260605_120000_distance_measurements.csv"
    assert saved.xlsx == output_dir / "20260605_120000.xlsx"
    assert _xlsx_sheet_names_and_row_counts(saved.px4_left_xlsx)[0] == [
        "ATTITUDE",
        "GPS1",
        "GPS2",
        "MAG",
        "GLOBAL_POSITION",
        "LOCAL_POSITION_NED",
        "RAW_IMU",
        "MAG_CONFIG",
    ]
    assert _xlsx_sheet_names_and_row_counts(saved.px4_right_xlsx)[0] == [
        "ATTITUDE",
        "GPS1",
        "GPS2",
        "MAG",
        "GLOBAL_POSITION",
        "LOCAL_POSITION_NED",
        "RAW_IMU",
        "MAG_CONFIG",
    ]
    assert _xlsx_sheet_names_and_row_counts(saved.session_rtk_xlsx)[0] == ["GPS_RTK"]
    assert sorted(path.name for path in output_dir.iterdir()) == [
        ".generations",
        "20260605_120000.xlsx",
        "20260605_120000_camera_left.csv",
        "20260605_120000_camera_left.parquet",
        "20260605_120000_camera_right.csv",
        "20260605_120000_camera_right.parquet",
        "20260605_120000_distance_measurements.csv",
        "20260605_120000_distance_measurements.parquet",
        "20260605_120000_gnss_base.csv",
        "20260605_120000_gnss_base.parquet",
        "20260605_120000_px4_left.csv",
        "20260605_120000_px4_left.parquet",
        "20260605_120000_px4_left.xlsx",
        "20260605_120000_px4_right.csv",
        "20260605_120000_px4_right.parquet",
        "20260605_120000_px4_right.xlsx",
        "20260605_120000_session_rtk.xlsx",
        "20260605_120000_session_static.json",
        "published",
        "session_manifest.json",
    ]
    with saved.csv.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2
    assert rows[0]["sample_index"] == "7"
    assert rows[0]["recording_time_ns"] == "1250000000"
    assert rows[1]["recording_time_ns"] == "1300000000"
    assert rows[0]["distance_m"] == "5.4"
    assert set(rows[0]) == set(DISTANCE_MEASUREMENT_COLUMNS)

    import pyarrow.parquet as pq

    parquet_table = pq.read_table(saved.distance_measurements_parquet)
    assert parquet_table.column_names == list(DISTANCE_MEASUREMENT_COLUMNS)
    assert parquet_table["recording_time_ns"].to_pylist() == [1_250_000_000, 1_300_000_000]
    static_payload = json.loads(saved.session_static_json.read_text(encoding="utf-8"))
    assert static_payload["time_axis"]["primary"] == "recording_time_ns"
    assert static_payload["metadata"]["calibration"]["profile"] == "night"

    sheet_names, sheet_row_counts = _xlsx_sheet_names_and_row_counts(saved.xlsx)
    assert sheet_names == [
        "Summary",
        "DistanceMeasurements",
        "CameraLeft",
        "CameraRight",
        "Px4Left",
        "Px4Right",
        "CalibrationStatic",
    ]
    dynamic_counts = dict(zip(sheet_names, sheet_row_counts, strict=True))
    assert dynamic_counts["DistanceMeasurements"] == len(pair_rows) + 1
    assert dynamic_counts["CameraLeft"] == 2
    assert dynamic_counts["CameraRight"] == 1
    assert dynamic_counts["Px4Left"] == 1
    assert dynamic_counts["Px4Right"] == 2
    measurement_headers = set(_xlsx_sheet_headers(saved.xlsx, 2))
    assert set(DISTANCE_MEASUREMENT_COLUMNS) == measurement_headers
    assert {
        "recording_time_ns",
        "distance_m",
        "baseline_center_forward_m",
        "baseline_center_right_m",
        "baseline_center_down_m",
    } <= measurement_headers
    with saved.camera_left_csv.open("r", newline="", encoding="utf-8") as handle:
        camera_rows = list(csv.DictReader(handle))
    assert set(camera_rows[0]) == set(CAMERA_TIMELINE_COMPACT_COLUMNS)
    assert camera_rows[0]["recording_time_ns"] == ""
    assert camera_rows[0]["device_utc_time_ns"] == "1800000000000000000"
    assert camera_rows[0]["rtp_ssrc"] == "11"
    assert camera_rows[0]["rtcp_clock_rate_hz"] == "90000.0"
    assert camera_rows[0]["rtcp_receive_delay_ns"] == "55000000"
    assert camera_rows[0]["rtcp_transport_delay_ns"] == "12000000"
    assert camera_rows[0]["rtcp_camera_media_rate"] == "0.979"
    with saved.px4_right_csv.open("r", newline="", encoding="utf-8") as handle:
        px4_rows = list(csv.DictReader(handle))
    assert set(px4_rows[0]) == set(PX4_TIMELINE_COMPACT_COLUMNS)
    assert px4_rows[0]["sample_type"] == "gps_rtk"
    assert px4_rows[0]["topic"] == "/uav_1/fmu/out/vehicle_gps_position"
    assert px4_rows[0]["gps_utc_time_ns"] == "1800000000123456000"
    assert px4_rows[0]["gps_utc_source"] == "GPS1"
    assert px4_rows[0]["latitude_deg"] == "37.123456789"
    assert px4_rows[0]["longitude_deg"] == "127.987654321"
    assert px4_rows[0]["altitude_m"] == "42.25"
    assert px4_rows[0]["fix_type"] == "6"
    assert px4_rows[0]["satellites_used"] == "18"
    assert "altitude_ellipsoid_m" not in px4_rows[0]
    assert "rtk_label" not in px4_rows[0]
    assert "velocity_ned_valid" not in px4_rows[0]
    px4_xlsx_rows = _xlsx_table_rows(saved.xlsx, 6)
    assert px4_xlsx_rows[0]["sample_type"] == "gps_rtk"
    assert px4_xlsx_rows[0]["fix_type"] == 6
    assert px4_xlsx_rows[0]["satellites_used"] == 18
    assert px4_xlsx_rows[0]["gps_utc_time_ns"] == 1800000000123456000
    assert px4_xlsx_rows[0]["latitude_deg"] == 37.123456789
    assert px4_xlsx_rows[0]["longitude_deg"] == 127.987654321
    assert "rtk_label" not in px4_xlsx_rows[0]
    assert "velocity_n_m_s" not in px4_xlsx_rows[0]
    with saved.gnss_base_csv.open("r", newline="", encoding="utf-8") as handle:
        gnss_base_rows = list(csv.DictReader(handle))
    assert set(gnss_base_rows[0]) == set(GNSS_BASE_COMPACT_COLUMNS)
    assert gnss_base_rows[0]["latitude_deg"] == "37.401234"
    assert gnss_base_rows[0]["longitude_deg"] == "127.123456"
    assert gnss_base_rows[0]["ellipsoid_height_m"] == "52.3"
    assert gnss_base_rows[0]["height_type"] == "ELLIPSOID"
    assert gnss_base_rows[0]["fixed"] == "True"
    assert gnss_base_rows[0]["device_id"] == "base-f9p"
    assert gnss_base_rows[0]["gnss_operation_state"] == "BASE_AS_ROVER_FIXED"
    assert gnss_base_rows[0]["session_utc_ns"] == "1800000000150000000"
    assert gnss_base_rows[0]["host_receive_utc_ns"] == "1800000000200000000"

    gnss_base_parquet_table = pq.read_table(saved.gnss_base_parquet)
    assert gnss_base_parquet_table.column_names == list(GNSS_BASE_COMPACT_COLUMNS)
    assert gnss_base_parquet_table["latitude_deg"].to_pylist() == [37.401234]

    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    assert manifest["status"] == "finalized"
    assert manifest["artifact_roles"][saved.distance_measurements_parquet.name] == "distance_measurements_parquet"
    assert manifest["artifact_roles"][saved.gnss_base_csv.name] == "gnss_base_timeline_csv"


def test_save_session_files_creates_headers_for_empty_pair_rows(tmp_path) -> None:
    output_dir = tmp_path / "empty_session"

    saved = save_session_files(
        output_dir,
        metadata={"session_id": "empty_session"},
        pair_rows=[],
    )

    with saved.csv.open("r", newline="", encoding="utf-8") as handle:
        assert next(csv.reader(handle)) == list(DISTANCE_MEASUREMENT_COLUMNS)
    for path in (
        saved.camera_left_csv,
        saved.camera_right_csv,
        saved.camera_left_parquet,
        saved.camera_right_parquet,
        saved.px4_left_csv,
        saved.px4_left_parquet,
        saved.px4_right_csv,
        saved.px4_right_parquet,
    ):
        assert path.is_file()
    for path in (saved.camera_left_csv, saved.camera_right_csv):
        with path.open("r", newline="", encoding="utf-8") as handle:
            assert next(csv.reader(handle)) == list(CAMERA_TIMELINE_COMPACT_COLUMNS)
    for path in (saved.px4_left_csv, saved.px4_right_csv):
        with path.open("r", newline="", encoding="utf-8") as handle:
            assert next(csv.reader(handle)) == list(PX4_TIMELINE_COMPACT_COLUMNS)
    sheet_names, sheet_row_counts = _xlsx_sheet_names_and_row_counts(saved.xlsx)
    dynamic_counts = dict(zip(sheet_names, sheet_row_counts, strict=True))
    assert {dynamic_counts[name] for name in sheet_names[1:6]} == {1}


def test_xlsx_failure_keeps_all_authoritative_csvs_and_retry_manifest(tmp_path, monkeypatch) -> None:
    import wiris_stereo_distance.recording_session as recording_session

    def fail_xlsx(*_args, **_kwargs):
        raise OSError("workbook locked")

    monkeypatch.setattr(recording_session, "_write_xlsx", fail_xlsx)
    saved = save_session_files(
        tmp_path / "session",
        metadata={"session_id": "session"},
        pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0}],
        camera_rows={"left": [{"sequence": 0, "side": "left"}]},
        px4_rows={"right": [{"sequence": 0, "side": "right"}]},
    )

    assert saved.finalized is False
    assert "workbook locked" in saved.error
    assert all(path.is_file() for path in (
        saved.session_static_json,
        saved.csv,
        saved.distance_measurements_parquet,
        saved.camera_left_csv,
        saved.camera_left_parquet,
        saved.camera_right_csv,
        saved.camera_right_parquet,
        saved.px4_left_csv,
        saved.px4_left_parquet,
        saved.px4_right_csv,
        saved.px4_right_parquet,
    ))
    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["retryable"] is True
    assert manifest["artifacts"]["session.xlsx"] == "failed"


def test_xlsx_writes_nonfinite_values_as_blank_cells(tmp_path) -> None:
    saved = save_session_files(
        tmp_path / "nonfinite",
        metadata={"session_id": "nonfinite", "calibration": {"invalid": math.nan}},
        pair_rows=[
            {
                "sample_index": 0,
                "session_elapsed_s": 0.0,
                "distance_m": math.inf,
            }
        ],
        px4_rows={
            "left": [
                {
                    "sample_type": "mavlink_vfr_hud",
                    "mavlink_vfr_airspeed_m_s": math.nan,
                }
            ]
        },
    )

    assert saved.finalized is True
    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    assert manifest["status"] == "finalized"


def test_csv_prepare_failure_does_not_replace_any_authoritative_csv(tmp_path, monkeypatch) -> None:
    import wiris_stereo_distance.recording_session as recording_session

    output_dir = tmp_path / "session"
    output_dir.mkdir()
    names = [
        "session_session_static.json",
        "session_distance_measurements.csv",
        "session_distance_measurements.parquet",
        "session_camera_left.csv",
        "session_camera_left.parquet",
        "session_camera_right.csv",
        "session_camera_right.parquet",
        "session_px4_left.csv",
        "session_px4_left.parquet",
        "session_px4_right.csv",
        "session_px4_right.parquet",
    ]
    for name in names:
        (output_dir / name).write_text(f"old-{name}\n", encoding="utf-8")
    original_write = recording_session.write_dynamic_csv

    def fail_third(output, rows, **kwargs):
        if output.name == "session_camera_right.csv":
            raise OSError("disk full")
        return original_write(output, rows, **kwargs)

    monkeypatch.setattr(recording_session, "write_dynamic_csv", fail_third)
    try:
        save_session_files(
            output_dir,
            metadata={"session_id": "session"},
            pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0}],
        )
    except OSError as exc:
        assert "disk full" in str(exc)
    else:
        raise AssertionError("expected CSV preparation failure")

    for name in names:
        assert (output_dir / name).read_text(encoding="utf-8") == f"old-{name}\n"
    manifest = json.loads((output_dir / "session_manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["phase"] == "prepare_resource_files"
    assert all(state == "prepared" for state in manifest["artifacts"].values())


def test_spool_rows_preserve_all_rows_with_bounded_preview(tmp_path) -> None:
    rows = SpoolRows(preview_limit=4)
    rows.bind(tmp_path / "rows.jsonl")
    for sequence in range(25):
        rows.append({"sequence": sequence})

    assert len(rows) == 25
    assert rows.preview_count == 4
    assert [row["sequence"] for row in rows] == list(range(25))
    assert rows[-1]["sequence"] == 24


def test_session_ids_do_not_collide_within_the_same_clock_tick(monkeypatch) -> None:
    import wiris_stereo_distance.recording_session as recording_session

    monkeypatch.setattr(recording_session.time, "time_ns", lambda: 1_800_000_000_123_456_789)
    first = session_id_now()
    second = session_id_now()

    assert first != second
    assert first[:15] == second[:15]


def test_publish_failure_keeps_previous_generation_visible(tmp_path, monkeypatch) -> None:
    import wiris_stereo_distance.recording_session as recording_session

    output_dir = tmp_path / "session"
    first = save_session_files(
        output_dir,
        metadata={"session_id": "session"},
        pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0, "distance_m": 1.0}],
    )
    previous_pointer = (output_dir / "published").readlink()
    previous_csv = first.csv.read_text(encoding="utf-8")
    original_publish = recording_session._publish_generation

    monkeypatch.setattr(
        recording_session,
        "_publish_generation",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("pointer fault")),
    )
    try:
        save_session_files(
            output_dir,
            metadata={"session_id": "session"},
            pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0, "distance_m": 9.0}],
        )
    except OSError as exc:
        assert "pointer fault" in str(exc)
    else:
        raise AssertionError("expected publish failure")

    assert (output_dir / "published").readlink() == previous_pointer
    assert first.csv.read_text(encoding="utf-8") == previous_csv
    manifest = json.loads(first.manifest.read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["phase"] == "publish"
    assert len(list((output_dir / ".generations").iterdir())) == 2

    monkeypatch.setattr(recording_session, "_publish_generation", original_publish)
    save_session_files(
        output_dir,
        metadata={"session_id": "session"},
        pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0, "distance_m": 2.0}],
    )
    generations = list((output_dir / ".generations").iterdir())
    assert len(generations) == 1
    assert generations[0].name == (output_dir / "published").readlink().name


def test_spool_iteration_does_not_stop_later_durable_appends(tmp_path) -> None:
    path = tmp_path / "rows.jsonl"
    rows = SpoolRows(preview_limit=3)
    rows.bind(path)
    assert list(rows) == []
    for sequence in range(12):
        rows.append({"sequence": sequence})
    rows.close()

    recovered = SpoolRows(preview_limit=3)
    recovered.open_existing(path)
    assert [row["sequence"] for row in recovered] == list(range(12))
    assert recovered.preview_count == 3


def test_retryable_session_discovery_and_pair_row_recovery(tmp_path) -> None:
    recordings_root = tmp_path / "recordings"
    output_dir = recordings_root / "sessions" / "recover-me"
    pair_rows = SpoolRows(preview_limit=2, supports_updates=True)
    pair_rows.bind(output_dir / ".spool" / "pair_rows.jsonl")
    for sequence in range(5):
        pair_rows.append({"sample_index": sequence, "distance_m": float(sequence)})
    pair_rows.close()
    write_recovery_metadata(
        output_dir,
        {"session_id": "recover-me", "session_started_unix_ns": 123},
    )
    (output_dir / "session_manifest.json").write_text(
        json.dumps({"status": "failed", "retryable": True}), encoding="utf-8"
    )

    assert discover_retryable_sessions(recordings_root) == [output_dir]
    recovered = load_retryable_session(output_dir)
    assert len(recovered.pair_rows) == 5
    assert [row["sample_index"] for row in recovered.pair_rows] == list(range(5))
    assert recovered.metadata["session_id"] == "recover-me"


def test_discard_retryable_session_removes_spool_and_stops_recovery(tmp_path) -> None:
    recordings_root = tmp_path / "recordings"
    output_dir = recordings_root / "sessions" / "discard-me"
    rows = SpoolRows()
    rows.bind(output_dir / ".spool" / "pair_rows.jsonl")
    rows.append({"sample_index": 0})
    rows.close()
    write_recovery_metadata(output_dir, {"session_id": "discard-me"})
    (output_dir / "session_manifest.json").write_text(
        json.dumps({"status": "failed", "retryable": True}), encoding="utf-8"
    )

    assert discover_retryable_sessions(recordings_root) == [output_dir]
    assert discard_retryable_session(output_dir, reason="test_discard") is True
    assert not (output_dir / ".spool").exists()
    manifest = json.loads((output_dir / "session_manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "discarded"
    assert manifest["retryable"] is False
    assert manifest["discard_reason"] == "test_discard"
    assert discover_retryable_sessions(recordings_root) == []


def test_retry_cleans_finalize_and_preparing_orphans(tmp_path) -> None:
    output_dir = tmp_path / "session"
    (output_dir / ".finalize-old").mkdir(parents=True)
    (output_dir / ".generations" / "old.preparing").mkdir(parents=True)

    save_session_files(
        output_dir,
        metadata={"session_id": "session"},
        pair_rows=[],
    )

    assert not (output_dir / ".finalize-old").exists()
    assert not (output_dir / ".generations" / "old.preparing").exists()


def test_session_sheet_previews_match_the_written_workbook(tmp_path) -> None:
    metadata = {
        "session_id": "preview-session",
        "calibration": {"left_fx": 1234.5},
    }
    pair_rows = [
        {
            "sample_index": 0,
            "session_elapsed_s": 0.0,
            "distance_m": 5.0,
            "gps_latitude_deg": 37.1,
            "attitude_roll_deg": 1.0,
            "baseline_length_m": 0.3,
            "quality": "GOOD",
        },
        {
            "sample_index": 1,
            "session_elapsed_s": 0.5,
            "distance_m": 5.2,
            "gps_latitude_deg": 37.2,
            "attitude_roll_deg": 1.1,
            "baseline_length_m": 0.3,
            "quality": "BAD",
        },
    ]
    output_dir = tmp_path / "session"

    saved = save_session_files(output_dir, metadata=metadata, pair_rows=pair_rows)
    previews = session_sheet_previews(metadata, pair_rows)
    names, row_counts = _xlsx_sheet_names_and_row_counts(saved.xlsx)

    assert names == [preview.name for preview in previews]
    for preview, row_count in zip(previews, row_counts):
        expected = (
            len(preview.entries)
            if preview.kind == "key_value"
            else (len(pair_rows) if preview.name == "DistanceMeasurements" else 0)
        )
        # Every sheet carries one header row on top of its body rows.
        assert row_count == expected + 1, preview.name


def test_legacy_regular_outputs_are_migrated_to_stable_generation_links(tmp_path) -> None:
    output_dir = tmp_path / "session"
    output_dir.mkdir()
    (output_dir / "session.csv").write_text("legacy\n", encoding="utf-8")

    saved = save_session_files(
        output_dir,
        metadata={"session_id": "session"},
        pair_rows=[{"sample_index": 0, "session_elapsed_s": 0.0}],
    )

    assert saved.csv.is_symlink()
    assert saved.csv.is_file()
    assert (output_dir / "published").is_symlink()
