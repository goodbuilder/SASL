from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_readiness_module():
    spec = importlib.util.spec_from_file_location(
        "calibration_topic_readiness_check_under_test",
        ROOT_DIR / "scripts" / "calibration_topic_readiness_check.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load calibration_topic_readiness_check.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CalibrationTopicReadinessCheckTests(unittest.TestCase):
    def test_parse_topic_list_and_evaluate_px4_truth_topics(self) -> None:
        module = load_readiness_module()
        available = module.parse_topic_list_t(
            """
/fmu/out/vehicle_local_position [px4_msgs/msg/VehicleLocalPosition]
/stereo/estimated_position [std_msgs/msg/Float32MultiArray]
/stereo/detections [std_msgs/msg/Float32MultiArray]
/stereo/disparity [std_msgs/msg/Float32]
"""
        )
        expected = {
            "/fmu/out/vehicle_local_position": "px4_msgs/msg/VehicleLocalPosition",
            "/stereo/nominal_pose_position": "std_msgs/msg/Float32MultiArray",
            "/stereo/detections": "std_msgs/msg/Float32MultiArray",
            "/stereo/disparity": "std_msgs/msg/Float32",
        }

        summary = module.evaluate_topics(available, expected)

        self.assertTrue(summary["pass_readiness"])
        self.assertEqual(summary["px4_truth_axis_contract"], "px4_local_position_ned:north,east,down")
        self.assertEqual(
            summary["calibration_rdf_axis_contract"],
            "stereo_center_opencv_optical:RDF(right,down,forward)",
        )

    def test_evaluate_reports_missing_or_wrong_type(self) -> None:
        module = load_readiness_module()
        available = module.parse_topic_list_t(
            """
/stereo/estimated_position [std_msgs/msg/Float32MultiArray]
/stereo/detections [std_msgs/msg/String]
"""
        )
        expected = {
            "/ground_truth_position": "std_msgs/msg/Float32MultiArray",
            "/stereo/nominal_pose_position": "std_msgs/msg/Float32MultiArray",
            "/stereo/detections": "std_msgs/msg/Float32MultiArray",
        }

        summary = module.evaluate_topics(available, expected)

        self.assertFalse(summary["pass_readiness"])
        self.assertFalse(summary["topic_checks"]["/ground_truth_position"]["present"])
        self.assertFalse(summary["topic_checks"]["/stereo/detections"]["type_ok"])


if __name__ == "__main__":
    unittest.main()
