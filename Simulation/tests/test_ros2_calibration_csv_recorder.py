from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


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
    def test_px4_local_position_xyz_is_written_as_ned_and_rdf(self) -> None:
        module = load_recorder_module()
        msg = type("VehicleLocalPosition", (), {"x": 1000.0, "y": 8.0, "z": 4.0})()

        fields = module.fmu_local_position_to_fields(msg)

        self.assertEqual(fields["truth_north_m"], 1000.0)
        self.assertEqual(fields["truth_east_m"], 8.0)
        self.assertEqual(fields["truth_down_ned_m"], 4.0)
        self.assertEqual(fields["truth_right_m"], 8.0)
        self.assertEqual(fields["truth_down_m"], 4.0)
        self.assertEqual(fields["truth_forward_m"], 1000.0)

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


if __name__ == "__main__":
    unittest.main()
