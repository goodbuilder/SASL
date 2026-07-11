from __future__ import annotations

import json
import math
import sys
import types
import unittest
from collections import deque
from importlib.util import find_spec
from pathlib import Path

import numpy as np


def install_ros_import_stubs() -> None:
    if "rclpy" in sys.modules or find_spec("rclpy") is not None:
        return

    class Message:
        def __init__(self, **kwargs):
            for key, value in kwargs.items():
                setattr(self, key, value)

    class Node:
        pass

    class MultiThreadedExecutor:
        def __init__(self, *args, **kwargs):
            pass

    class QoSProfile:
        def __init__(self, *args, **kwargs):
            pass

    class Marker(Message):
        TEXT_VIEW_FACING = 9
        ADD = 0

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.MultiThreadedExecutor = MultiThreadedExecutor
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = Node
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=object())
    rclpy_qos.QoSProfile = QoSProfile
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=object())

    rcl_interfaces = types.ModuleType("rcl_interfaces")
    rcl_interfaces_msg = types.ModuleType("rcl_interfaces.msg")
    rcl_interfaces_msg.ParameterDescriptor = Message
    rcl_interfaces_msg.SetParametersResult = Message

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = Message

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float32 = Message
    std_msgs_msg.Float32MultiArray = Message
    std_msgs_msg.String = Message

    visualization_msgs = types.ModuleType("visualization_msgs")
    visualization_msgs_msg = types.ModuleType("visualization_msgs.msg")
    visualization_msgs_msg.Marker = Marker

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.executors": rclpy_executors,
            "rclpy.node": rclpy_node,
            "rclpy.qos": rclpy_qos,
            "rcl_interfaces": rcl_interfaces,
            "rcl_interfaces.msg": rcl_interfaces_msg,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
            "visualization_msgs": visualization_msgs,
            "visualization_msgs.msg": visualization_msgs_msg,
        }
    )


install_ros_import_stubs()

PACKAGE_DIR = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.runtime_guard_policy import EstimatedToTruthSim3Profile  # noqa: E402
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode  # noqa: E402
from stereo_ground_sim.stereo_triangulation_adapter import COMPARISON_SOURCE  # noqa: E402


WIDTH_PX = 1280
HEIGHT_PX = 720
HORIZONTAL_FOV_RAD = 0.154997710698
BASELINE_M = 25.0


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_node() -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.horizontal_fov_rad = HORIZONTAL_FOV_RAD
    node.baseline_m = BASELINE_M
    node.synthetic_detection_bbox_size_px = 32.0
    node.yolo_load_error = ""
    node.yolo_class_name = "drone"
    node.bbox_center_adjustment = None
    node.observation_fusion_adapter = None
    node.observation_fusion_buffer = deque(maxlen=1)
    node.disparity_residual_profile = None
    node.disparity_residual_feature_selector = None
    node.disparity_residual_feature_selector_window = deque(maxlen=1)
    return node


def image():
    return np.zeros((HEIGHT_PX, WIDTH_PX, 3), dtype=np.uint8)


def synthetic_pair(node, *, right_m=7.0, down_m=4.0, forward_m=1000.0):
    left, right = StereoDistanceNode._synthetic_detection_pair_from_rdf(
        node,
        right_m=right_m,
        down_m=down_m,
        forward_m=forward_m,
        width=WIDTH_PX,
        height=HEIGHT_PX,
    )
    assert left is not None and right is not None
    return left, right


def assert_json_compatible_primitives(testcase: unittest.TestCase, value) -> None:
    json.dumps(value, sort_keys=True)
    if value is None or isinstance(value, (str, bool, int, float)):
        return
    if isinstance(value, list):
        for item in value:
            assert_json_compatible_primitives(testcase, item)
        return
    if isinstance(value, dict):
        for key, item in value.items():
            testcase.assertIsInstance(key, str)
            assert_json_compatible_primitives(testcase, item)
        return
    testcase.fail(f"non-primitive value in shadow summary: {type(value)!r}")


class StereoDistanceGeneralRayShadowTests(unittest.TestCase):
    def assert_shadow_compatible(self, metrics, places: int = 6) -> None:
        shadow = metrics["general_ray_shadow"]
        self.assertTrue(shadow["valid"])
        self.assertEqual(shadow["status"], "ok")
        self.assertEqual(shadow["output_frame"], "baseline_center_frd")
        self.assertLess(abs(shadow["legacy_forward_delta_m"]), 1.0e-6)
        self.assertLess(abs(shadow["legacy_right_delta_m"]), 1.0e-6)
        self.assertLess(abs(shadow["legacy_down_delta_m"]), 1.0e-6)
        self.assertLess(abs(shadow["legacy_range_delta_m"]), 1.0e-6)
        self.assertAlmostEqual(metrics["estimated_forward_m"], shadow["forward_m"], places=places)
        self.assertAlmostEqual(metrics["estimated_right_m"], shadow["right_m"], places=places)
        self.assertAlmostEqual(metrics["estimated_down_m"], shadow["down_m"], places=places)
        self.assertAlmostEqual(metrics["estimated_range_m"], shadow["range_m"], places=places)

    def test_nominal_bbox_pair_adds_valid_shadow_without_changing_estimate(self) -> None:
        node = make_node()
        left, right = synthetic_pair(node, right_m=7.0, down_m=4.0, forward_m=1000.0)
        legacy = StereoDistanceNode._position_from_centers(
            node,
            left["cx"],
            left["cy"],
            right["cx"],
            right["cy"],
            WIDTH_PX,
            HEIGHT_PX,
        )

        metrics = StereoDistanceNode._calculate_metrics(node, image(), left, right, 1000.0, "unit")

        self.assert_shadow_compatible(metrics)
        self.assertAlmostEqual(metrics["estimated_forward_m"], legacy["forward_m"], places=9)
        self.assertAlmostEqual(metrics["estimated_right_m"], legacy["right_m"], places=9)
        self.assertAlmostEqual(metrics["estimated_down_m"], legacy["down_m"], places=9)
        self.assertAlmostEqual(metrics["estimated_range_m"], legacy["range_m"], places=9)

    def test_corrected_profile_behavior_is_unchanged(self) -> None:
        node = make_node()
        node.bbox_center_adjustment = EstimatedToTruthSim3Profile.from_json_dict(
            {
                "mode": "estimated_to_truth_sim3",
                "profile_id": "shadow-correction-unit",
                "scale": 1.0,
                "rotation_matrix_rdf": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                "translation_rdf_m": [1.0, 2.0, 3.0],
            }
        )
        left, right = synthetic_pair(node)

        metrics = StereoDistanceNode._calculate_metrics(node, image(), left, right, 1000.0, "unit")

        self.assert_shadow_compatible(metrics)
        self.assertEqual(metrics["profile_status"]["profile_mode"], "estimated_to_truth_sim3")
        self.assertAlmostEqual(metrics["corrected_right_m"], metrics["estimated_right_m"] + 1.0)
        self.assertAlmostEqual(metrics["corrected_down_m"], metrics["estimated_down_m"] + 2.0)
        self.assertAlmostEqual(metrics["corrected_forward_m"], metrics["estimated_forward_m"] + 3.0)

    def test_missing_left_detection_marks_shadow_not_computed(self) -> None:
        node = make_node()
        _, right = synthetic_pair(node)

        metrics = StereoDistanceNode._calculate_metrics(node, image(), None, right, 1000.0, "unit")

        self.assertFalse(metrics["general_ray_shadow"]["valid"])
        self.assertEqual(metrics["general_ray_shadow"]["status"], "not_computed_incomplete_bbox_pair")
        self.assertEqual(metrics["general_ray_shadow"]["pixel_bounds_status"], "not_evaluated")

    def test_missing_right_detection_marks_shadow_not_computed(self) -> None:
        node = make_node()
        left, _ = synthetic_pair(node)

        metrics = StereoDistanceNode._calculate_metrics(node, image(), left, None, 1000.0, "unit")

        self.assertFalse(metrics["general_ray_shadow"]["valid"])
        self.assertEqual(metrics["general_ray_shadow"]["status"], "not_computed_incomplete_bbox_pair")
        self.assertEqual(metrics["general_ray_shadow"]["pixel_bounds_status"], "not_evaluated")

    def test_swapped_centers_do_not_abort_node(self) -> None:
        node = make_node()
        left, right = synthetic_pair(node)
        swapped_left = {**left, "cx": right["cx"], "cy": right["cy"]}
        swapped_right = {**right, "cx": left["cx"], "cy": left["cy"]}

        metrics = StereoDistanceNode._calculate_metrics(node, image(), swapped_left, swapped_right, 1000.0, "unit")

        self.assertIsNotNone(metrics["estimated_forward_m"])
        self.assertFalse(metrics["general_ray_shadow"]["valid"])
        self.assertEqual(metrics["general_ray_shadow"]["status"], "non_positive_depth")

    def test_non_finite_observation_does_not_abort_node(self) -> None:
        node = make_node()
        left, right = synthetic_pair(node)
        left = {**left, "cx": math.nan}

        metrics = StereoDistanceNode._calculate_metrics(node, image(), left, right, 1000.0, "unit")

        self.assertFalse(metrics["general_ray_shadow"]["valid"])
        self.assertEqual(metrics["general_ray_shadow"]["status"], "non_finite_pixel")
        self.assertEqual(metrics["general_ray_shadow"]["pixel_bounds_status"], "not_evaluated")

    def test_publish_estimated_position_keeps_legacy_array_order(self) -> None:
        node = make_node()
        node.estimated_position_pub = FakePublisher()
        node.corrected_position_pub = FakePublisher()
        node.bbox_adjustment_pub = FakePublisher()
        node.profile_status_pub = FakePublisher()
        metrics = {
            "estimated_forward_m": 100.0,
            "estimated_right_m": 7.0,
            "estimated_down_m": 4.0,
            "estimated_range_m": 100.325,
            "corrected_forward_m": 103.0,
            "corrected_right_m": 8.0,
            "corrected_down_m": 6.0,
            "corrected_range_m": 103.348,
            "bbox_adjustment": None,
            "profile_status": StereoDistanceNode._profile_status(node),
            "observation_fusion_status": "disabled",
        }

        StereoDistanceNode._publish_estimated_position(node, metrics)

        self.assertEqual(node.estimated_position_pub.messages[-1].data, [100.0, 7.0, 4.0, 100.325])
        self.assertEqual(node.corrected_position_pub.messages[-1].data, [103.0, 8.0, 6.0, 103.348])

    def test_profile_status_json_does_not_include_shadow(self) -> None:
        node = make_node()
        metrics = {
            "profile_status": StereoDistanceNode._profile_status(node, "profile", "mode", 0, "unit"),
            "observation_fusion_status": "disabled",
            "general_ray_shadow": {"valid": True, "status": "ok"},
        }

        msg = StereoDistanceNode._profile_status_to_msg(node, metrics)
        payload = json.loads(msg.data)

        self.assertNotIn("general_ray_shadow", payload)
        self.assertEqual(payload["profile_id"], "profile")

    def test_shadow_summary_contains_json_compatible_primitives(self) -> None:
        node = make_node()
        left, right = synthetic_pair(node)
        metrics = StereoDistanceNode._calculate_metrics(node, image(), left, right, 1000.0, "unit")

        assert_json_compatible_primitives(self, metrics["general_ray_shadow"])

    def test_synthetic_detection_round_trip_preserves_existing_estimate(self) -> None:
        node = make_node()
        left, right = synthetic_pair(node, right_m=7.0, down_m=4.0, forward_m=1000.0)

        metrics = StereoDistanceNode._calculate_metrics(node, image(), left, right, 1000.0, "unit")

        self.assertAlmostEqual(metrics["estimated_right_m"], 7.0, places=6)
        self.assertAlmostEqual(metrics["estimated_down_m"], 4.0, places=6)
        self.assertAlmostEqual(metrics["estimated_forward_m"], 1000.0, places=6)

    def test_shadow_deltas_compare_prefusion_legacy_result(self) -> None:
        node = make_node()
        left, right = synthetic_pair(node, right_m=7.0, down_m=4.0, forward_m=1000.0)
        legacy = StereoDistanceNode._position_from_centers(
            node,
            left["cx"],
            left["cy"],
            right["cx"],
            right["cy"],
            WIDTH_PX,
            HEIGHT_PX,
        )
        fused = {
            "right_m": 900.0,
            "down_m": -800.0,
            "forward_m": 700.0,
            "range_m": math.sqrt(900.0 * 900.0 + 800.0 * 800.0 + 700.0 * 700.0),
            "distance_m": 700.0,
            "disparity_px": 42.0,
        }
        node._apply_observation_fusion = lambda left_obs, right_obs, width, height: (dict(fused), "fused:unit")

        metrics = StereoDistanceNode._calculate_metrics(node, image(), left, right, 1000.0, "unit")
        shadow = metrics["general_ray_shadow"]

        self.assertEqual(metrics["observation_fusion_status"], "fused:unit")
        self.assertEqual(metrics["estimated_forward_m"], fused["forward_m"])
        self.assertEqual(metrics["estimated_right_m"], fused["right_m"])
        self.assertEqual(metrics["estimated_down_m"], fused["down_m"])
        self.assertEqual(shadow["comparison_source"], COMPARISON_SOURCE)
        self.assertLess(abs(shadow["forward_m"] - legacy["forward_m"]), 1.0e-6)
        self.assertLess(abs(shadow["right_m"] - legacy["right_m"]), 1.0e-6)
        self.assertLess(abs(shadow["down_m"] - legacy["down_m"]), 1.0e-6)
        self.assertLess(abs(shadow["legacy_forward_delta_m"]), 1.0e-6)
        self.assertGreater(abs(shadow["forward_m"] - fused["forward_m"]), 100.0)


if __name__ == "__main__":
    unittest.main()
