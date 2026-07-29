from __future__ import annotations

import csv
import hashlib
import importlib.util
import io
import json
import sys
import tempfile
import time
import unittest
import zipfile
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "operator_recorder.py"
AUDIT_SCRIPT_PATH = ROOT_DIR / "scripts" / "audit_operator_telemetry_residual_preflight_schema.py"


def load_module():
    spec = importlib.util.spec_from_file_location("operator_recorder_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_audit_module():
    spec = importlib.util.spec_from_file_location("operator_telemetry_schema_audit_under_test", AUDIT_SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {AUDIT_SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeClockNow:
    @property
    def nanoseconds(self) -> int:
        return 123_456_789


class FakeClock:
    def now(self) -> FakeClockNow:
        return FakeClockNow()


class FakeMsg:
    def __init__(self, data):
        self.data = data


class FakePointStamped:
    def __init__(self, forward, right, down, *, sec=123, nanosec=456_000_000):
        self.point = type("Point", (), {"x": forward, "y": right, "z": down})()
        self.header = type("Header", (), {"stamp": type("Stamp", (), {"sec": sec, "nanosec": nanosec})()})()


class FakeSensorGps:
    timestamp = 100_000
    timestamp_sample = 99_000
    latitude_deg = 37.1
    longitude_deg = 127.2
    altitude_msl_m = 123.4
    eph = 0.5
    epv = 0.7
    vel_ned_valid = True
    vel_n_m_s = 1.0
    vel_e_m_s = 2.0
    vel_d_m_s = 3.0


class FakeVehicleGlobalPosition:
    timestamp = 200_000
    timestamp_sample = 199_000
    lat = 37.11
    lon = 127.21
    alt = 124.4
    eph = 0.4
    epv = 0.6


class FakeVehicleOdometry:
    POSE_FRAME_NED = 1
    VELOCITY_FRAME_NED = 1
    pose_frame = POSE_FRAME_NED
    velocity_frame = VELOCITY_FRAME_NED
    timestamp = 300_000
    timestamp_sample = 299_000
    position = (0.0, 0.0, 0.0)
    q = (1.0, 0.0, 0.0, 0.0)
    velocity = (1.0, 2.0, 3.0)
    reset_counter = 2


class OperatorRecorderSchemaTests(unittest.TestCase):
    def test_recorder_declares_current_public_frd_schema(self) -> None:
        module = load_module()

        self.assertEqual(module.AXIS_CONTRACT, "stereodistance_public_frd:forward,right,down,range")
        for field in (
            "raw_forward_m",
            "raw_right_m",
            "raw_down_m",
            "estimated_forward_m",
            "estimated_right_m",
            "estimated_down_m",
            "corrected_forward_m",
            "corrected_right_m",
            "corrected_down_m",
        ):
            self.assertIn(field, module.ESTIMATED_POSITION_FIELDS + module.CORRECTED_POSITION_FIELDS)

    def test_callbacks_and_csv_row_emit_preflight_required_fields(self) -> None:
        module = load_module()
        recorder = object.__new__(module.OperatorRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.start_monotonic = time.monotonic()
        recorder.get_clock = lambda: FakeClock()
        output = io.StringIO()
        recorder.csv_file = output
        recorder.csv_writer = csv.DictWriter(output, fieldnames=module.TELEMETRY_FIELDS)
        recorder.csv_writer.writeheader()

        recorder._estimated_position_cb(FakeMsg([1000.0, 7.0, 4.0, 1000.0325]))
        recorder._corrected_position_cb(FakeMsg([999.0, 6.5, 3.5, 999.0274]))
        recorder._optical_position_cb(FakeMsg([1001.0, 7.5, 4.5, 1001.038]))
        recorder._ideal_position_cb(FakeMsg([1000.5, 7.1, 4.1, 1000.533]))
        recorder._float_cb(FakeMsg(200.0), "disparity_px", "/stereo/disparity")
        recorder._ground_truth_position_cb(FakeMsg([1000.0, 7.0, 4.0, 1000.0325]))
        recorder._gps_baseline_position_cb(
            "raw_gps_baseline", FakePointStamped(1002.0, 6.0, 5.0)
        )
        recorder._gps_baseline_position_cb(
            "global_position_baseline", FakePointStamped(999.0, 8.0, 4.5)
        )
        recorder._detections_cb(FakeMsg([1, 690, 380, 710, 400, 0.9, 700, 390, 1, 490, 380, 510, 400, 0.8, 500, 390]))
        recorder._profile_status_cb(
            FakeMsg(json.dumps({"profile_id": "unit", "profile_mode": "sim3", "fallback_level": 2}))
        )
        recorder._ray_metrics_cb(
            FakeMsg(
                json.dumps(
                    {
                        "ideal_left_u_px": 701.0,
                        "ideal_left_v_px": 391.0,
                        "ideal_right_u_px": 501.0,
                        "ideal_right_v_px": 391.0,
                        "ideal_disparity_px": 200.0,
                        "ideal_pixel_source": "unit_test_projection",
                        "nominal": {
                            "computation_status": "computed",
                            "forward_m": 1000.0,
                            "right_m": 7.0,
                            "down_m": 4.0,
                            "range_m": 1000.0325,
                            "gap_m": 0.1,
                            "intersection_angle_deg": 1.2,
                            "left_depth_m": 1000.0,
                            "right_depth_m": 1000.1,
                            "left_point_frd_m": [1000.0, 7.0, 4.0],
                            "right_point_frd_m": [1000.0, 7.1, 4.0],
                            "midpoint_frd_m": [1000.0, 7.05, 4.0],
                        },
                    }
                )
            )
        )
        recorder._write_csv_row()
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))

        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row["axis_contract"], module.AXIS_CONTRACT)
        self.assertEqual(row["raw_forward_m"], "1000")
        self.assertEqual(row["raw_right_m"], "7")
        self.assertEqual(row["raw_down_m"], "4")
        self.assertEqual(row["estimated_forward_m"], "1000")
        self.assertEqual(row["corrected_forward_m"], "999")
        self.assertEqual(row["corrected_right_m"], "6.5")
        self.assertEqual(row["corrected_down_m"], "3.5")
        self.assertEqual(row["optical_forward_m"], "1001")
        self.assertEqual(row["ideal_forward_m"], "1000.5")
        self.assertEqual(row["raw_disparity_px"], "200")
        self.assertEqual(row["ideal_left_u_px"], "701")
        self.assertEqual(row["ideal_right_u_px"], "501")
        self.assertEqual(row["ideal_pixel_source"], "unit_test_projection")
        self.assertEqual(row["nominal_ray_computation_status"], "computed")
        self.assertEqual(row["nominal_ray_gap_m"], "0.1")
        self.assertEqual(row["nominal_ray_midpoint_right_m"], "7.05")
        self.assertEqual(row["left_x1"], "690")
        self.assertEqual(row["left_y1"], "380")
        self.assertEqual(row["left_x2"], "710")
        self.assertEqual(row["left_y2"], "400")
        self.assertEqual(row["right_valid"], "1")
        self.assertEqual(row["right_x1"], "490")
        self.assertEqual(row["right_y1"], "380")
        self.assertEqual(row["right_x2"], "510")
        self.assertEqual(row["right_y2"], "400")
        self.assertEqual(row["right_cx"], "500")
        self.assertEqual(row["right_width_px"], "20")
        self.assertEqual(row["profile_id"], "unit")
        self.assertEqual(row["profile_mode"], "sim3")
        self.assertEqual(row["profile_fallback_level"], "2")
        self.assertEqual(row["raw_gps_baseline_forward_m"], "1002")
        self.assertEqual(row["raw_gps_baseline_position_error_m"], "2.44948974")
        self.assertEqual(row["global_position_baseline_forward_m"], "999")
        self.assertEqual(row["global_position_baseline_position_error_m"], "1.5")
        self.assertEqual(row["raw_minus_global_baseline_forward_m"], "3")
        self.assertEqual(row["raw_minus_global_baseline_right_m"], "-2")
        self.assertEqual(row["raw_minus_global_baseline_down_m"], "0.5")
        self.assertEqual(row["raw_minus_global_baseline_delta_m"], "3.64005494")
        self.assertEqual(row["truth_range_bin_100m"], "1000")

    def test_detection_schema_keeps_bbox_corners_as_the_source_values(self) -> None:
        module = load_module()

        self.assertEqual(
            tuple(field for field in module.DETECTION_FIELDS if field.startswith("left_") and field.endswith(("x1", "y1", "x2", "y2"))),
            ("left_x1", "left_y1", "left_x2", "left_y2"),
        )
        self.assertEqual(
            tuple(field for field in module.DETECTION_FIELDS if field.startswith("right_") and field.endswith(("x1", "y1", "x2", "y2"))),
            ("right_x1", "right_y1", "right_x2", "right_y2"),
        )

    def test_progress_sidecar_reports_facts_atomically(self) -> None:
        module = load_module()
        recorder = object.__new__(module.OperatorRecorder)
        recorder.mode = "full"
        recorder.row_count = 25
        recorder.last_elapsed_sec = 1.25
        recorder.valid_row_counts = {field: 20 for field in module.PROGRESS_VALIDITY_FIELDS}
        with tempfile.TemporaryDirectory() as tmpdir:
            recorder.session_dir = Path(tmpdir)

            recorder._write_progress(state="finalized")

            progress_path = Path(tmpdir) / "progress.json"
            self.assertTrue(progress_path.is_file())
            self.assertFalse((Path(tmpdir) / "progress.json.tmp").exists())
            payload = json.loads(progress_path.read_text(encoding="utf-8"))
        self.assertEqual(payload["state"], "finalized")
        self.assertEqual(payload["mode"], "full")
        self.assertEqual(payload["row_count"], 25)
        self.assertEqual(payload["valid_row_counts"]["ground_truth_m"], 20)

    def test_progress_sidecar_identifies_timestamped_primary_artifacts(self) -> None:
        module = load_module()
        recorder = object.__new__(module.OperatorRecorder)
        recorder.mode = "lite"
        recorder.row_count = 0
        recorder.last_elapsed_sec = 0.0
        recorder.valid_row_counts = {field: 0 for field in module.PROGRESS_VALIDITY_FIELDS}
        recorder.recording_timestamp = "20260729_143015"
        with tempfile.TemporaryDirectory() as tmpdir:
            recorder.session_dir = Path(tmpdir)
            recorder.csv_path = recorder.session_dir / "telemetry_20260729_143015.csv"
            recorder.xlsx_path = recorder.session_dir / "session_20260729_143015.xlsx"

            recorder._write_progress()

            payload = json.loads((recorder.session_dir / "progress.json").read_text(encoding="utf-8"))
        self.assertEqual(payload["recording_timestamp"], "20260729_143015")
        self.assertEqual(payload["csv"], "telemetry_20260729_143015.csv")
        self.assertEqual(payload["xlsx"], "session_20260729_143015.xlsx")

    def test_final_xlsx_mirrors_one_authoritative_csv_into_purpose_sheets(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            source = Path(tmpdir) / "telemetry.csv"
            workbook = Path(tmpdir) / "session.xlsx"
            with source.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=module.TELEMETRY_FIELDS)
                writer.writeheader()
                writer.writerow(
                    {
                        "time_sec": "1.0",
                        "axis_contract": module.AXIS_CONTRACT,
                        "raw_range_m": "2000.0",
                        "yolo_inference_ms": "45.0",
                        "yolo_update_hz": "20.5",
                        "raw_gps_baseline_forward_m": "1999.0",
                        "global_position_baseline_forward_m": "1998.0",
                        "left_x1": "10",
                        "left_y1": "20",
                        "left_x2": "30",
                        "left_y2": "40",
                        "right_x1": "50",
                        "right_y1": "60",
                        "right_x2": "70",
                        "right_y2": "80",
                        "nominal_ray_computation_status": "computed",
                        "nominal_ray_forward_m": "2000",
                        "nominal_ray_right_m": "1",
                        "nominal_ray_down_m": "2",
                        "nominal_ray_range_m": "2000.00125",
                        "nominal_ray_gap_m": "0.05",
                        "nominal_ray_intersection_angle_deg": "1.5",
                    }
                )

            result = module.export_session_xlsx(
                source,
                workbook,
                mode="full",
                manifest_rows=(("yolo_target_hz", "20.0"),),
            )

            self.assertEqual(result["row_count"], 1)
            self.assertEqual(result["csv_sha256"], hashlib.sha256(source.read_bytes()).hexdigest())
            self.assertTrue(workbook.is_file())
            self.assertFalse((Path(tmpdir) / "session.xlsx.tmp").exists())
            with zipfile.ZipFile(workbook) as archive:
                workbook_xml = archive.read("xl/workbook.xml").decode("utf-8")
                worksheets_xml = "\n".join(
                    archive.read(name).decode("utf-8")
                    for name in archive.namelist()
                    if name.startswith("xl/worksheets/")
                )
            for sheet_name in (
                "Summary",
                "UAV PX4",
                "Left PX4",
                "Right PX4",
                "Camera Observations",
                "Stereo Estimates",
                "Quality Events",
                "Rays",
                "Performance",
                "Config",
            ):
                self.assertIn(f'name="{sheet_name}"', workbook_xml)
            self.assertIn(result["csv_sha256"], worksheets_xml)
            self.assertIn("nominal", worksheets_xml)

    def test_compact_session_export_separates_static_json_and_minimal_time_csvs(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            source = Path(tmpdir) / "telemetry.csv"
            with source.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=module.TELEMETRY_FIELDS)
                writer.writeheader()
                writer.writerow(
                    {
                        "time_sec": "1.0",
                        "ros_time_sec": "2.0",
                        "axis_contract": module.AXIS_CONTRACT,
                        "ground_truth_m": "1000.1",
                        "drone_forward_m": "1000",
                        "drone_right_m": "7",
                        "drone_down_m": "4",
                        "raw_forward_m": "999",
                        "raw_right_m": "8",
                        "raw_down_m": "5",
                        "raw_range_m": "999.04",
                        "left_x1": "10",
                        "left_y1": "20",
                        "left_x2": "30",
                        "left_y2": "40",
                        "left_confidence": "0.9",
                        "right_x1": "50",
                        "right_y1": "60",
                        "right_x2": "70",
                        "right_y2": "80",
                        "right_confidence": "0.8",
                        "nominal_ray_midpoint_forward_m": "998",
                        "nominal_ray_midpoint_right_m": "6",
                        "nominal_ray_midpoint_down_m": "3",
                        "nominal_ray_range_m": "998.02",
                        "nominal_ray_gap_m": "0.05",
                        "nominal_ray_intersection_angle_deg": "1.5",
                        "uam_px4_position_n_m": "1",
                        "uam_px4_position_e_m": "2",
                        "uam_px4_position_d_m": "-3",
                    }
                )

            result = module.export_compact_session_files(
                source,
                Path(tmpdir),
                mode="lite",
                csv_hz=30.0,
                video_fps=20.0,
                manifest_rows=(("camera_baseline_m", "25.0"),),
            )

            static_payload = json.loads((Path(tmpdir) / "session_static.json").read_text(encoding="utf-8"))
            manifest = json.loads((Path(tmpdir) / "session_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(static_payload["static_values"]["camera_baseline_m"], "25.0")
            self.assertEqual(static_payload["axis_contract"], module.AXIS_CONTRACT)
            self.assertEqual(manifest["artifact_roles"]["telemetry.csv"], "legacy_wide_telemetry_csv")
            self.assertEqual(result["row_counts"]["distance_measurements"], 1)
            self.assertEqual(result["row_counts"]["ray_summary"], 1)
            self.assertEqual(result["row_counts"]["detections"], 1)
            self.assertEqual(result["row_counts"]["px4_uam"], 1)
            with (Path(tmpdir) / "ray_summary.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["truth_Rg_m"], "1000.1")
            self.assertEqual(rows[0]["nominal_forward_m"], "998")
            self.assertEqual(tuple(rows[0].keys()), module.COMPACT_RAY_SUMMARY_FIELDS)
            with (Path(tmpdir) / "distance_measurements.csv").open(newline="", encoding="utf-8") as handle:
                distance_rows = list(csv.DictReader(handle))
            self.assertEqual(tuple(distance_rows[0].keys()), module.COMPACT_DISTANCE_FIELDS)
            self.assertNotIn("left_x1", distance_rows[0])

    def test_camera_px4_samples_and_estimated_optical_extrinsic_are_recorded(self) -> None:
        module = load_module()
        recorder = object.__new__(module.OperatorRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.camera_runtime = {
            label: {
                "gazebo_base_position_world_enu_m": None,
                "gazebo_base_rotation_world_from_body": None,
                "mount_translation_body_m": None,
                "mount_rotation_body_to_optical": None,
                "mount_rpy_deg": None,
                "px4_position_ned_m": None,
                "px4_rotation_ned_from_body": None,
                "bootstrap_position_ned_m": None,
                "bootstrap_position_world_enu_m": None,
            }
            for label in ("left", "right")
        }
        diagnostic = {
            "schema": "camera_pose_comparison/v1",
            "observed_base_link_position_world_enu_m": [-12.5, 0.0, 10.0],
            "observed_base_link_orientation_world_from_entity_xyzw": [0.0, 0.0, 0.0, 1.0],
            "observed_optical_position_world_enu_m": [-12.5, 0.0, 10.0],
            "observed_optical_orientation_world_from_entity_xyzw": [0.0, 0.0, 0.0, 1.0],
        }

        recorder._camera_raw_gps_cb("left", FakeSensorGps())
        recorder._camera_global_position_cb("left", FakeVehicleGlobalPosition())
        recorder._camera_pose_diagnostic_cb("left", FakeMsg(json.dumps(diagnostic)))
        recorder._camera_odometry_cb("left", FakeVehicleOdometry())
        recorder._camera_raw_gps_cb("uam", FakeSensorGps())
        recorder._camera_global_position_cb("uam", FakeVehicleGlobalPosition())
        recorder._camera_odometry_cb("uam", FakeVehicleOdometry())

        self.assertEqual(recorder.latest["left_raw_gps_lat_deg"], 37.1)
        self.assertEqual(recorder.latest["left_global_position_lon_deg"], 127.21)
        self.assertEqual(recorder.latest["left_px4_velocity_e_m_s"], 2.0)
        self.assertEqual(recorder.latest["uam_raw_gps_lat_deg"], 37.1)
        self.assertEqual(recorder.latest["uam_global_position_lon_deg"], 127.21)
        self.assertEqual(recorder.latest["uam_px4_velocity_e_m_s"], 2.0)
        self.assertEqual(recorder.latest["left_px4_estimated_optical_world_enu_x_m"], -12.5)
        self.assertEqual(recorder.latest["left_px4_estimated_optical_world_enu_y_m"], 0.0)
        self.assertEqual(recorder.latest["left_px4_estimated_optical_world_enu_z_m"], 10.0)
        self.assertEqual(recorder.latest["left_px4_estimated_optical_world_enu_roll_deg"], 180.0)
        for field in (
            "left_raw_gps_lat_deg",
            "left_global_position_lat_deg",
            "left_px4_velocity_n_m_s",
            "left_px4_estimated_optical_world_enu_x_m",
            "right_px4_estimated_optical_world_enu_yaw_deg",
            "uam_raw_gps_lat_deg",
            "uam_global_position_lat_deg",
            "uam_px4_velocity_n_m_s",
        ):
            self.assertIn(field, module.TELEMETRY_FIELDS)
        manifest_rows = dict(recorder._camera_xlsx_manifest_rows())
        self.assertEqual(
            manifest_rows["left_camera_mount_base_to_optical_translation_m"],
            "0,0,0",
        )
        self.assertEqual(
            manifest_rows["left_camera_mount_source"], "gazebo_camera_pose_diagnostics"
        )

    def test_recorder_csv_row_passes_residual_preflight_schema_audit(self) -> None:
        module = load_module()
        audit_module = load_audit_module()
        recorder = object.__new__(module.OperatorRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.start_monotonic = time.monotonic()
        recorder.get_clock = lambda: FakeClock()
        with tempfile.TemporaryDirectory() as tmpdir:
            telemetry = Path(tmpdir) / "telemetry.csv"
            with telemetry.open("w", newline="", encoding="utf-8") as handle:
                recorder.csv_file = handle
                recorder.csv_writer = csv.DictWriter(handle, fieldnames=module.TELEMETRY_FIELDS)
                recorder.csv_writer.writeheader()
                recorder._estimated_position_cb(FakeMsg([1000.0, 7.0, 4.0, 1000.0325]))
                recorder._corrected_position_cb(FakeMsg([999.0, 6.5, 3.5, 999.0274]))
                recorder._float_cb(FakeMsg(200.0), "disparity_px", "/stereo/disparity")
                recorder._detections_cb(
                    FakeMsg([1, 690, 380, 710, 400, 0.9, 700, 390, 1, 490, 380, 510, 400, 0.8, 500, 390])
                )
                recorder._write_csv_row()

            payload = audit_module.audit_schema(input_csv=telemetry)

        self.assertEqual(payload["status"], "PASS")
        self.assertEqual(payload["valid_row_count"], 1)
        self.assertFalse(payload["runtime_truth_used"])


if __name__ == "__main__":
    unittest.main()
