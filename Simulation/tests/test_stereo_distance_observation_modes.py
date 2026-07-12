from __future__ import annotations

import json
import math
import sys
import threading
import types
import unittest
from collections import deque
from importlib import import_module
from importlib.util import find_spec
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))


ROS_IMPORT_MODULES = (
    "rclpy",
    "rclpy.executors",
    "rclpy.node",
    "rclpy.qos",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "visualization_msgs",
    "visualization_msgs.msg",
    "stereo_ground_sim.stereo_distance_node",
)
_MISSING = object()


def _module_snapshot(names=ROS_IMPORT_MODULES) -> dict[str, object]:
    return {name: sys.modules.get(name, _MISSING) for name in names}


def _restore_module_snapshot(snapshot: dict[str, object]) -> None:
    for name, module in snapshot.items():
        if module is _MISSING:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


def _make_ros_import_stubs() -> dict[str, types.ModuleType]:
    class Message:
        def __init__(self, *args, **kwargs):
            if args:
                self.data = args[0]
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
    std_msgs_msg.Float64 = Message
    std_msgs_msg.Float64MultiArray = Message
    std_msgs_msg.String = Message

    visualization_msgs = types.ModuleType("visualization_msgs")
    visualization_msgs_msg = types.ModuleType("visualization_msgs.msg")
    visualization_msgs_msg.Marker = Marker

    rcl_interfaces.msg = rcl_interfaces_msg
    sensor_msgs.msg = sensor_msgs_msg
    std_msgs.msg = std_msgs_msg
    visualization_msgs.msg = visualization_msgs_msg
    return {
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


def import_node_module():
    module_name = "stereo_ground_sim.stereo_distance_node"
    if find_spec("rclpy") is not None:
        return import_module(module_name), False

    snapshot = _module_snapshot()
    package = sys.modules.get("stereo_ground_sim")
    had_package_attr = package is not None and hasattr(package, "stereo_distance_node")
    package_attr = getattr(package, "stereo_distance_node", None) if package is not None else None
    try:
        sys.modules.pop(module_name, None)
        sys.modules.update(_make_ros_import_stubs())
        module = import_module(module_name)
        return module, True
    finally:
        _restore_module_snapshot(snapshot)
        if package is not None:
            if had_package_attr:
                setattr(package, "stereo_distance_node", package_attr)
            elif hasattr(package, "stereo_distance_node"):
                delattr(package, "stereo_distance_node")


node_module, USING_ROS_IMPORT_STUBS = import_node_module()
StereoDistanceNode = node_module.StereoDistanceNode

from stereo_ground_sim.coordinate_frames import FrameId, FramePoint  # noqa: E402
from stereo_ground_sim.observation_comparison import GroundTruthSnapshot  # noqa: E402


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeParam:
    def __init__(self, name, value):
        self.name = name
        self.value = value


def valid_snapshot(sequence: int = 1) -> GroundTruthSnapshot:
    point = FramePoint((1000.0, 7.0, 4.0), FrameId.BASELINE_CENTER_FRD)
    return GroundTruthSnapshot(
        sequence=sequence,
        received_monotonic_s=10.0 + sequence,
        point_baseline_center_frd=point,
        range_m=1000.0324994718922,
        oracle_valid=True,
        oracle_status="ok",
        oracle_source_role="target_reference_link",
        oracle_matched_entity_name="x500_15m::target_reference_link",
        oracle_fallback_used=False,
    )


def invalid_snapshot(sequence: int = 1, status: str = "malformed_pose_payload") -> GroundTruthSnapshot:
    return GroundTruthSnapshot(
        sequence=sequence,
        received_monotonic_s=10.0 + sequence,
        point_baseline_center_frd=None,
        range_m=None,
        oracle_valid=False,
        oracle_status=status,
        oracle_source_role=None,
        oracle_matched_entity_name=None,
        oracle_fallback_used=False,
    )


def make_node(mode: str = "both") -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.observation_mode = mode
    node.baseline_m = 25.0
    node.horizontal_fov_rad = 0.154997710698
    node.camera_image_width_px = 1280
    node.camera_image_height_px = 720
    node.camera_clip_near_m = 0.1
    node.camera_clip_far_m = 2500.0
    node.lock = threading.Lock()
    node.oracle_sample_seq = 0
    node.current_ground_truth_snapshot = invalid_snapshot(0, "not_computed_init")
    node.last_valid_ground_truth_snapshot = None
    node.latest_ideal_measurement = None
    node.latest_bbox_general_ray_measurement = None
    node.latest_observation_comparison = None
    node.yolo_model_path = ""
    node.yolo_target_hz = 0.0
    node.enable_external_bbox_detections = False
    node.enable_synthetic_detections = False
    node.bbox_configured_source = node_module.BBOX_SOURCE_UNAVAILABLE
    node.ideal_estimated_position_pub = FakePublisher()
    node.ideal_position_error_pub = FakePublisher()
    node.ideal_disparity_pub = FakePublisher()
    node.bbox_general_ray_estimated_position_pub = FakePublisher()
    node.bbox_general_ray_position_error_pub = FakePublisher()
    node.bbox_vs_ideal_pixel_delta_pub = FakePublisher()
    node.bbox_vs_ideal_position_delta_pub = FakePublisher()
    node.observation_status_pub = FakePublisher()
    node.detections_pub = FakePublisher()
    node.distance_pub = FakePublisher()
    node.disparity_pub = FakePublisher()
    node.error_pub = FakePublisher()
    node.comparison_pub = FakePublisher()
    node.estimated_position_pub = FakePublisher()
    node.corrected_position_pub = FakePublisher()
    node.bbox_adjustment_pub = FakePublisher()
    node.actual_distance_pub = FakePublisher()
    node.ground_truth_position_pub = FakePublisher()
    node.synthetic_detection_bbox_size_px = 32.0
    node.left_msg = None
    node.right_msg = None
    node.bbox_center_adjustment = None
    node.observation_fusion_adapter = None
    node.observation_fusion_buffer = deque(maxlen=1)
    node.disparity_residual_profile = None
    node.disparity_residual_feature_selector = None
    node.disparity_residual_feature_selector_window = deque(maxlen=1)
    return node


def ideal_pixel_centers(node: StereoDistanceNode, snapshot: GroundTruthSnapshot):
    measurement = StereoDistanceNode._compute_ideal_measurement(node, snapshot)
    return (
        {"cx": measurement.left_u_px, "cy": measurement.left_v_px},
        {"cx": measurement.right_u_px, "cy": measurement.right_v_px},
    )


class StereoDistanceObservationModeTests(unittest.TestCase):
    def test_observation_mode_validation(self) -> None:
        self.assertEqual(StereoDistanceNode._observation_mode_value(" both "), "both")
        self.assertEqual(StereoDistanceNode._observation_mode_value("bbox"), "bbox")
        self.assertEqual(
            StereoDistanceNode._observation_mode_value("ideal_projection"),
            "ideal_projection",
        )
        for value in ("Both", "BOTH", "unknown", 3):
            with self.assertRaises(ValueError):
                StereoDistanceNode._observation_mode_value(value)

    def test_camera_contract_validation(self) -> None:
        self.assertEqual(StereoDistanceNode._positive_int_config(1280, field_name="width"), 1280)
        for value in (True, 0, -1, 1.2, "1280"):
            with self.assertRaises(ValueError):
                StereoDistanceNode._positive_int_config(value, field_name="width")
        StereoDistanceNode._validate_camera_clip_config(0.0, 1.0)
        for near, far in ((True, 1.0), (0.2, 0.2), (-0.1, 1.0), (0.1, math.inf), (0.1, math.nan)):
            with self.assertRaises(ValueError):
                StereoDistanceNode._validate_camera_clip_config(near, far)

    def test_restart_required_parameters_reject_runtime_update(self) -> None:
        node = make_node()
        for name in (
            "observation_mode",
            "camera_image_width_px",
            "camera_image_height_px",
            "camera_clip_near_m",
            "camera_clip_far_m",
            "enable_synthetic_detections",
            "enable_external_bbox_detections",
            "external_bbox_detections_topic",
        ):
            result = StereoDistanceNode._on_param_update(node, [FakeParam(name, "x")])
            self.assertFalse(result.successful)

    def test_bbox_source_arbitration(self) -> None:
        node = make_node()
        self.assertEqual(StereoDistanceNode._configured_bbox_source_label(node), node_module.BBOX_SOURCE_UNAVAILABLE)
        node.yolo_model_path = "/tmp/model.pt"
        node.yolo_target_hz = 20.0
        self.assertEqual(StereoDistanceNode._configured_bbox_source_label(node), node_module.BBOX_SOURCE_YOLO)
        node.yolo_model_path = ""
        node.enable_external_bbox_detections = True
        self.assertEqual(StereoDistanceNode._configured_bbox_source_label(node), node_module.BBOX_SOURCE_EXTERNAL)
        node.enable_external_bbox_detections = False
        node.enable_synthetic_detections = True
        self.assertEqual(StereoDistanceNode._configured_bbox_source_label(node), node_module.BBOX_SOURCE_SYNTHETIC)
        node.enable_external_bbox_detections = True
        with self.assertRaises(ValueError):
            StereoDistanceNode._configured_bbox_source_label(node)

    def test_valid_and_invalid_oracle_sequences_update_current_snapshot(self) -> None:
        node = make_node()
        with node.lock:
            seq1 = StereoDistanceNode._next_oracle_sample_seq_locked(node)
            node.current_ground_truth_snapshot = valid_snapshot(seq1)
            node.last_valid_ground_truth_snapshot = node.current_ground_truth_snapshot
            seq2 = StereoDistanceNode._next_oracle_sample_seq_locked(node)
            node.current_ground_truth_snapshot = invalid_snapshot(seq2)

        self.assertEqual(seq1, 1)
        self.assertEqual(seq2, 2)
        self.assertFalse(node.current_ground_truth_snapshot.oracle_valid)
        self.assertIsNone(node.current_ground_truth_snapshot.point_baseline_center_frd)
        self.assertIsNotNone(node.last_valid_ground_truth_snapshot.point_baseline_center_frd)

    def test_valid_oracle_publishes_ideal_and_unavailable_bbox_status(self) -> None:
        node = make_node("both")
        snapshot = valid_snapshot(3)
        measurement = StereoDistanceNode._process_ideal_snapshot(node, snapshot)

        self.assertTrue(measurement.valid)
        self.assertEqual(len(node.ideal_estimated_position_pub.messages), 1)
        self.assertEqual(len(node.bbox_general_ray_estimated_position_pub.messages), 1)
        status = json.loads(node.observation_status_pub.messages[-1].data)
        self.assertEqual(status["observation_mode"], "both")
        self.assertTrue(status["ideal_valid"])
        self.assertFalse(status["bbox_valid"])
        self.assertEqual(status["bbox_configured_source"], "unavailable")

    def test_invalid_oracle_publishes_ideal_nan_without_stale_numeric(self) -> None:
        node = make_node("ideal_projection")
        measurement = StereoDistanceNode._process_ideal_snapshot(node, invalid_snapshot(4))

        self.assertFalse(measurement.valid)
        self.assertTrue(all(math.isnan(value) for value in node.ideal_estimated_position_pub.messages[-1].data))
        self.assertTrue(math.isnan(node.ideal_disparity_pub.messages[-1].data))
        payload = node.observation_status_pub.messages[-1].data
        self.assertNotIn("NaN", payload)
        self.assertNotIn("Infinity", payload)
        status = json.loads(payload)
        self.assertIsNone(status["ideal_left_u_px"])

    def test_bbox_mode_does_not_publish_ideal_numeric(self) -> None:
        node = make_node("bbox")
        StereoDistanceNode._process_ideal_snapshot(node, valid_snapshot(5))
        self.assertEqual(node.ideal_estimated_position_pub.messages, [])

    def test_ideal_projection_mode_does_not_publish_bbox_research(self) -> None:
        node = make_node("ideal_projection")
        snapshot = valid_snapshot(6)
        left, right = ideal_pixel_centers(node, snapshot)
        node.current_ground_truth_snapshot = snapshot

        bbox, comparison = StereoDistanceNode._process_bbox_research_observation(
            node,
            left,
            right,
            source=node_module.BBOX_SOURCE_EXTERNAL,
            observation_received_monotonic_s=20.0,
        )

        self.assertIsNone(bbox)
        self.assertIsNone(comparison)
        self.assertEqual(node.bbox_general_ray_estimated_position_pub.messages, [])

    def test_both_mode_bbox_companion_uses_same_sequence(self) -> None:
        node = make_node("both")
        node.bbox_configured_source = node_module.BBOX_SOURCE_EXTERNAL
        snapshot = valid_snapshot(7)
        node.current_ground_truth_snapshot = snapshot
        left, right = ideal_pixel_centers(node, snapshot)

        bbox, comparison = StereoDistanceNode._process_bbox_research_observation(
            node,
            left,
            right,
            source=node_module.BBOX_SOURCE_EXTERNAL,
            observation_received_monotonic_s=22.0,
        )

        self.assertTrue(bbox.valid)
        self.assertTrue(comparison.valid)
        self.assertTrue(comparison.same_oracle_sequence)
        self.assertEqual(bbox.oracle_sequence, snapshot.sequence)
        self.assertLess(comparison.bbox_minus_ideal_norm_m, 1e-6)

    def test_incomplete_bbox_pair_publishes_nan_and_status(self) -> None:
        node = make_node("both")
        node.bbox_configured_source = node_module.BBOX_SOURCE_EXTERNAL
        node.current_ground_truth_snapshot = valid_snapshot(8)

        bbox, comparison = StereoDistanceNode._process_bbox_research_observation(
            node,
            None,
            None,
            source=node_module.BBOX_SOURCE_EXTERNAL,
            observation_received_monotonic_s=24.0,
        )

        self.assertFalse(bbox.valid)
        self.assertIsNone(comparison)
        self.assertTrue(all(math.isnan(value) for value in node.bbox_general_ray_estimated_position_pub.messages[-1].data))
        payload = node.observation_status_pub.messages[-1].data
        self.assertNotIn("NaN", payload)
        status = json.loads(payload)
        self.assertEqual(status["bbox_status"], "incomplete_bbox_pair")
        self.assertIsNone(status["comparison_valid"])

    def test_synthetic_detection_research_source_label(self) -> None:
        node = make_node("both")
        node.bbox_configured_source = node_module.BBOX_SOURCE_SYNTHETIC
        snapshot = valid_snapshot(9)
        node.current_ground_truth_snapshot = snapshot
        left, right = ideal_pixel_centers(node, snapshot)

        bbox, _comparison = StereoDistanceNode._process_bbox_research_observation(
            node,
            left,
            right,
            source=node_module.BBOX_SOURCE_SYNTHETIC,
            observation_received_monotonic_s=25.0,
        )

        self.assertEqual(bbox.observation_source, "synthetic_bbox_compatibility")
        status = json.loads(node.observation_status_pub.messages[-1].data)
        self.assertEqual(status["bbox_observation_source"], "synthetic_bbox_compatibility")


if __name__ == "__main__":
    unittest.main()
