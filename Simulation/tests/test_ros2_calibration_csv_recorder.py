from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from argparse import Namespace


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_recorder_module():
    package_dir = ROOT_DIR / "stereo_ground_sim"
    if str(package_dir) not in sys.path:
        sys.path.insert(0, str(package_dir))
    spec = importlib.util.spec_from_file_location(
        "ros2_calibration_csv_recorder_under_test",
        ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "calibration_csv_recorder.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load ros2_calibration_csv_recorder.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Ros2CalibrationCsvRecorderTests(unittest.TestCase):
    def test_px4_local_position_xyz_is_written_as_ned_only(self) -> None:
        module = load_recorder_module()
        msg = type("VehicleLocalPosition", (), {"x": 1000.0, "y": 8.0, "z": 4.0})()

        fields = module.fmu_local_position_to_fields(msg)

        self.assertEqual(fields["truth_north_m"], 1000.0)
        self.assertEqual(fields["truth_east_m"], 8.0)
        self.assertEqual(fields["truth_down_ned_m"], 4.0)
        self.assertNotIn("truth_right_m", fields)
        self.assertNotIn("truth_down_m", fields)
        self.assertNotIn("truth_forward_m", fields)

    def test_global_position_is_converted_to_ned_with_origin_metadata(self) -> None:
        module = load_recorder_module()
        msg = type("VehicleGlobalPosition", (), {"lat": 37.00001, "lon": 127.00002, "alt": 103.0})()

        fields, reference = module.global_position_to_fields(
            msg,
            reference=(37.0, 127.0, 100.0),
            origin_kind="manual_test_origin",
            source_type="global_position",
        )

        self.assertEqual(reference, (37.0, 127.0, 100.0))
        self.assertEqual(fields["truth_source_type"], "global_position")
        self.assertEqual(fields["truth_source_frame"], "global_lla")
        self.assertEqual(fields["truth_source_axis_contract"], "lat_deg,lon_deg,alt_m")
        self.assertGreater(float(fields["truth_north_m"]), 1.0)
        self.assertGreater(float(fields["truth_east_m"]), 1.0)
        self.assertEqual(fields["truth_down_ned_m"], -3.0)
        self.assertEqual(fields["truth_origin_kind"], "manual_test_origin")

    def test_prefixed_fields_preserve_all_truth_source_snapshots(self) -> None:
        module = load_recorder_module()
        fields = {
            "truth_source_type": "global_position",
            "truth_source_frame": "global_lla",
            "truth_source_axis_contract": "lat_deg,lon_deg,alt_m",
            "truth_raw_lat_deg": 37.0,
            "truth_north_m": 1.0,
            "truth_east_m": 2.0,
            "truth_down_ned_m": -3.0,
        }

        prefixed = module.prefixed_fields(fields, "global")

        self.assertEqual(prefixed["global_truth_source_type"], "global_position")
        self.assertEqual(prefixed["global_truth_source_frame"], "global_lla")
        self.assertEqual(prefixed["global_truth_source_axis_contract"], "lat_deg,lon_deg,alt_m")
        self.assertEqual(prefixed["global_truth_raw_lat_deg"], 37.0)
        self.assertEqual(prefixed["global_truth_north_m"], 1.0)
        self.assertNotIn("truth_north_m", prefixed)

    def test_csv_value_preserves_string_contract_metadata(self) -> None:
        module = load_recorder_module()

        self.assertEqual(module.csv_value("global_position"), "global_position")
        self.assertEqual(module.csv_value(1.25), "1.250000000")
        self.assertEqual(module.csv_value(None), "")

    def test_stereodistance_frd_topic_is_exported_with_named_raw_aliases(self) -> None:
        module = load_recorder_module()

        fields = module.frd_array_to_fields(
            [1000.0, 8.0, 4.0, 1000.04],
            prefix="estimated",
            include_raw_alias=True,
        )

        self.assertEqual(fields["estimated_forward_m"], 1000.0)
        self.assertEqual(fields["estimated_right_m"], 8.0)
        self.assertEqual(fields["estimated_down_m"], 4.0)
        self.assertEqual(fields["raw_forward_m"], 1000.0)
        self.assertEqual(fields["raw_right_m"], 8.0)
        self.assertEqual(fields["raw_down_m"], 4.0)
        self.assertEqual(fields["raw_range_m"], 1000.04)

    def test_detections_array_exports_bbox_centers_and_disparity_inputs(self) -> None:
        module = load_recorder_module()
        values = [
            1.0,
            10.0,
            20.0,
            30.0,
            40.0,
            0.9,
            20.0,
            30.0,
            1.0,
            5.0,
            22.0,
            25.0,
            42.0,
            0.8,
            15.0,
            32.0,
        ]

        fields = module.detections_array_to_fields(values)

        self.assertEqual(fields["left_valid"], 1.0)
        self.assertEqual(fields["left_cx_px"], 20.0)
        self.assertEqual(fields["left_cy_px"], 30.0)
        self.assertEqual(fields["right_valid"], 1.0)
        self.assertEqual(fields["right_cx_px"], 15.0)
        self.assertEqual(fields["right_cy_px"], 32.0)

    def test_manifest_declares_source_truth_to_stereo_rdf_contract(self) -> None:
        module = load_recorder_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            output_csv = Path(tmpdir) / "calibration.csv"
            module.write_manifest(
                output_csv,
                Namespace(
                    truth_topic="/fmu/out/vehicle_local_position",
                    local_truth_topic="/fmu/out/vehicle_local_position",
                    global_truth_topic="/fmu/out/vehicle_global_position",
                    gps_truth_topic="/fmu/out/vehicle_gps_position",
                    truth_source="local_ned",
                    sim_truth_topic="/ground_truth_position",
                    sim_truth_role="oracle",
                    estimated_topic="/stereo/estimated_position",
                    detections_topic="/stereo/detections",
                    disparity_topic="/stereo/disparity",
                ),
            )
            manifest = (output_csv.parent / "manifest.txt").read_text(encoding="utf-8")

        self.assertIn("truth_rdf_transform_contract=", manifest)
        self.assertIn("T_source_to_stereo_rdf", manifest)
        self.assertIn("axis aliasing is forbidden", manifest)
        self.assertIn("sim_truth_role=oracle", manifest)
        self.assertNotIn("T_ned_to_stereo_rdf", manifest)

    def test_sim_truth_is_oracle_by_default_not_training_truth(self) -> None:
        module = load_recorder_module()
        recorder = module.CalibrationCsvRecorder.__new__(module.CalibrationCsvRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.args = Namespace(sim_truth_topic="/ground_truth_position", sim_truth_role="oracle")
        recorder._now_sec = lambda: 123.0
        msg = type("Float32MultiArray", (), {"data": [100.0, 8.0, 4.0, 100.4]})()

        module.CalibrationCsvRecorder._sim_truth_cb(recorder, msg)

        self.assertEqual(recorder.latest["sim_truth_role"], "oracle")
        self.assertEqual(recorder.latest["sim_truth_forward_m"], 100.0)
        self.assertEqual(recorder.latest["sim_truth_right_m"], 8.0)
        self.assertNotIn("truth_forward_m", recorder.latest)
        self.assertNotIn("truth_right_m", recorder.latest)

    def test_sim_truth_training_role_explicitly_fills_training_truth(self) -> None:
        module = load_recorder_module()
        recorder = module.CalibrationCsvRecorder.__new__(module.CalibrationCsvRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.args = Namespace(sim_truth_topic="/ground_truth_position", sim_truth_role="training")
        recorder._now_sec = lambda: 123.0
        msg = type("Float32MultiArray", (), {"data": [100.0, 8.0, 4.0, 100.4]})()

        module.CalibrationCsvRecorder._sim_truth_cb(recorder, msg)

        self.assertEqual(recorder.latest["sim_truth_role"], "training")
        self.assertEqual(recorder.latest["truth_forward_m"], 100.0)
        self.assertEqual(recorder.latest["truth_right_m"], 8.0)

    def test_selected_truth_source_only_updates_canonical_truth_fields(self) -> None:
        module = load_recorder_module()
        recorder = module.CalibrationCsvRecorder.__new__(module.CalibrationCsvRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.args = Namespace(
            truth_source="global_position",
            truth_topic="/fmu/out/vehicle_local_position",
            local_truth_topic="/fmu/out/vehicle_local_position",
            global_truth_topic="/fmu/out/vehicle_global_position",
            gps_truth_topic="/fmu/out/vehicle_gps_position",
        )
        recorder.global_truth_lla_reference = None
        recorder.gps_truth_lla_reference = None
        recorder._now_sec = lambda: 123.0
        local_msg = type("VehicleLocalPosition", (), {"x": 100.0, "y": 8.0, "z": 4.0})()
        global_msg = type("VehicleGlobalPosition", (), {"lat": 37.00001, "lon": 127.00002, "alt": 103.0})()

        module.CalibrationCsvRecorder._local_truth_cb(recorder, local_msg)
        self.assertEqual(recorder.latest["local_truth_north_m"], 100.0)
        self.assertNotIn("truth_north_m", recorder.latest)

        module.CalibrationCsvRecorder._global_truth_cb(recorder, global_msg)
        self.assertEqual(recorder.latest["global_truth_source_type"], "global_position")
        self.assertEqual(recorder.latest["truth_source_type"], "global_position")
        self.assertEqual(recorder.latest["truth_north_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
