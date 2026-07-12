from __future__ import annotations

import importlib
import json
import math
import sys
import threading
import types
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPO_ROOT / "Simulation" / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


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


snapshot = _module_snapshot()
sys.modules.update(_make_ros_import_stubs())
try:
    node_module = importlib.import_module("stereo_ground_sim.stereo_distance_node")
finally:
    _restore_module_snapshot(snapshot)

from stereo_ground_sim.attitude_error_model import (  # noqa: E402
    CameraAttitudeErrorDeg,
    StereoAttitudeErrorConfig,
    evaluate_raw_attitude_measurement,
)
from stereo_ground_sim.coordinate_frames import FrameId, FramePoint  # noqa: E402
from stereo_ground_sim.observation_comparison import GroundTruthSnapshot  # noqa: E402
from stereo_ground_sim.stereo_triangulation_adapter import pinhole_intrinsics_from_horizontal_fov  # noqa: E402

StereoDistanceNode = node_module.StereoDistanceNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeParam:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class ParamValue:
    def __init__(self, value):
        self.value = value


def valid_snapshot(sequence: int = 101, point=(1852.0, 20.0, -6.0)) -> GroundTruthSnapshot:
    frame_point = FramePoint(point, FrameId.BASELINE_CENTER_FRD)
    return GroundTruthSnapshot(
        sequence=sequence,
        received_monotonic_s=100.0 + sequence,
        point_baseline_center_frd=frame_point,
        range_m=math.sqrt(sum(float(value) * float(value) for value in point)),
        oracle_valid=True,
        oracle_status="ok",
        oracle_source_role="target_reference_link",
        oracle_matched_entity_name="x500_15m::target_reference_link",
        oracle_fallback_used=False,
    )


def invalid_snapshot(sequence: int = 103) -> GroundTruthSnapshot:
    return GroundTruthSnapshot(
        sequence=sequence,
        received_monotonic_s=100.0 + sequence,
        point_baseline_center_frd=None,
        range_m=None,
        oracle_valid=False,
        oracle_status="malformed_pose_payload",
        oracle_source_role=None,
        oracle_matched_entity_name=None,
        oracle_fallback_used=False,
    )


def zero_config() -> StereoAttitudeErrorConfig:
    return StereoAttitudeErrorConfig(
        left=CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
        right=CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
    )


def left_yaw_config() -> StereoAttitudeErrorConfig:
    return StereoAttitudeErrorConfig(
        left=CameraAttitudeErrorDeg(0.0, 0.0, 0.01),
        right=CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
    )


def make_node(mode: str = "both", config: StereoAttitudeErrorConfig | None = None) -> StereoDistanceNode:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.observation_mode = mode
    node.baseline_m = 25.0
    node.horizontal_fov_rad = 0.154997710698
    node.camera_image_width_px = 1280
    node.camera_image_height_px = 720
    node.camera_clip_near_m = 0.1
    node.camera_clip_far_m = 2500.0
    node.attitude_error_config = zero_config() if config is None else config
    node.lock = threading.Lock()
    node.current_ground_truth_snapshot = invalid_snapshot(0)
    node.latest_ideal_measurement = None
    node.latest_bbox_general_ray_measurement = None
    node.latest_observation_comparison = None
    node.bbox_configured_source = node_module.BBOX_SOURCE_UNAVAILABLE
    node.attitude_actual_pixels_pub = FakePublisher()
    node.attitude_actual_disparity_pub = FakePublisher()
    node.attitude_raw_estimated_position_pub = FakePublisher()
    node.attitude_raw_position_error_pub = FakePublisher()
    node.attitude_raw_ray_quality_pub = FakePublisher()
    node.attitude_model_status_pub = FakePublisher()
    node.ideal_estimated_position_pub = FakePublisher()
    node.ideal_position_error_pub = FakePublisher()
    node.ideal_disparity_pub = FakePublisher()
    node.bbox_general_ray_estimated_position_pub = FakePublisher()
    node.bbox_general_ray_position_error_pub = FakePublisher()
    node.bbox_vs_ideal_pixel_delta_pub = FakePublisher()
    node.bbox_vs_ideal_position_delta_pub = FakePublisher()
    node.observation_status_pub = FakePublisher()
    return node


def assert_all_nan(testcase: unittest.TestCase, message) -> None:
    testcase.assertTrue(all(math.isnan(float(value)) for value in message.data))


def status_payload(node: StereoDistanceNode) -> dict[str, object]:
    return json.loads(node.attitude_model_status_pub.messages[-1].data)


class Phase3BAttitudeNodeWiringTests(unittest.TestCase):
    def test_attitude_parameter_config_validation(self) -> None:
        node = StereoDistanceNode.__new__(StereoDistanceNode)
        values = {
            "left_roll_error_deg": 0.0,
            "left_pitch_error_deg": -0.2,
            "left_yaw_error_deg": 0.3,
            "right_roll_error_deg": -0.4,
            "right_pitch_error_deg": 0.5,
            "right_yaw_error_deg": -0.6,
        }
        node.get_parameter = lambda name: ParamValue(values[name])
        config = StereoDistanceNode._attitude_error_config(node)
        self.assertAlmostEqual(config.left.pitch_deg, -0.2)
        self.assertAlmostEqual(config.right.yaw_deg, -0.6)

        for bad in (True, "0.0", math.nan, math.inf, -math.inf):
            values["left_roll_error_deg"] = bad
            with self.assertRaises((TypeError, ValueError)):
                StereoDistanceNode._attitude_error_config(node)

    def test_zero_attitude_publishes_raw_baseline(self) -> None:
        node = make_node()
        snapshot = valid_snapshot(101)
        measurement = StereoDistanceNode._process_attitude_model_snapshot(node, snapshot)

        self.assertTrue(measurement.valid)
        self.assertEqual(measurement.status, "ok")
        pixels = node.attitude_actual_pixels_pub.messages[-1].data
        self.assertAlmostEqual(pixels[0], 784.6293305394881, delta=1e-9)
        self.assertAlmostEqual(pixels[1], 333.2992005157868, delta=1e-9)
        self.assertAlmostEqual(pixels[2], 673.3759993552665, delta=1e-9)
        self.assertAlmostEqual(pixels[3], 333.2992005157868, delta=1e-9)
        self.assertAlmostEqual(pixels[4], 111.25333118422168, delta=1e-9)
        self.assertAlmostEqual(pixels[5], 0.0, delta=1e-9)
        self.assertLess(measurement.raw_error_norm_m, 1e-6)
        self.assertLess(measurement.raw_triangulation.ray_gap_m, 1e-9)

        status = status_payload(node)
        self.assertEqual(status["oracle_sequence"], 101)
        self.assertTrue(status["attitude_model_valid"])
        self.assertFalse(status["gazebo_sdf_equivalence_verified"])
        self.assertEqual(status["output_frame"], "baseline_center_frd")

    def test_left_yaw_matches_phase3a_and_hardcoded_reference(self) -> None:
        node = make_node(config=left_yaw_config())
        snapshot = valid_snapshot(102)
        baseline = evaluate_raw_attitude_measurement(
            snapshot.point_baseline_center_frd,
            pinhole_intrinsics_from_horizontal_fov(
                width_px=1280,
                height_px=720,
                horizontal_fov_rad=0.154997710698,
            ),
            25.0,
            zero_config(),
            clip_near_m=0.1,
            clip_far_m=2500.0,
        )
        expected = evaluate_raw_attitude_measurement(
            snapshot.point_baseline_center_frd,
            pinhole_intrinsics_from_horizontal_fov(
                width_px=1280,
                height_px=720,
                horizontal_fov_rad=0.154997710698,
            ),
            25.0,
            left_yaw_config(),
            clip_near_m=0.1,
            clip_far_m=2500.0,
        )

        measurement = StereoDistanceNode._process_attitude_model_snapshot(node, snapshot)
        pixels = node.attitude_actual_pixels_pub.messages[-1].data
        du_left = pixels[0] - baseline.actual_projection.left.u_px
        disparity_delta = pixels[4] - baseline.actual_projection.signed_disparity_px

        self.assertAlmostEqual(du_left, -1.4388772994, delta=1e-6)
        self.assertAlmostEqual(disparity_delta, -1.4388772994, delta=1e-6)
        self.assertAlmostEqual(measurement.raw_error_baseline_center_frd_m[0], 24.2663935314, delta=1e-5)
        self.assertAlmostEqual(measurement.raw_error_baseline_center_frd_m[0], expected.raw_error_baseline_center_frd_m[0], delta=1e-12)

    def test_invalid_oracle_publishes_nan_and_json_nulls(self) -> None:
        node = make_node()
        StereoDistanceNode._process_attitude_model_snapshot(node, valid_snapshot(101))
        StereoDistanceNode._process_attitude_model_snapshot(node, invalid_snapshot(103))

        assert_all_nan(self, node.attitude_actual_pixels_pub.messages[-1])
        self.assertTrue(math.isnan(node.attitude_actual_disparity_pub.messages[-1].data))
        assert_all_nan(self, node.attitude_raw_estimated_position_pub.messages[-1])
        assert_all_nan(self, node.attitude_raw_position_error_pub.messages[-1])
        assert_all_nan(self, node.attitude_raw_ray_quality_pub.messages[-1])

        raw_json = node.attitude_model_status_pub.messages[-1].data
        self.assertNotIn("NaN", raw_json)
        self.assertNotIn("Infinity", raw_json)
        status = json.loads(raw_json)
        self.assertEqual(status["oracle_sequence"], 103)
        self.assertFalse(status["attitude_model_valid"])
        self.assertEqual(status["attitude_model_status"], "invalid_oracle_snapshot")
        self.assertIsNone(status["actual_left_u_px"])
        self.assertIsNone(status["raw_forward_m"])

    def test_projection_invalid_cases_publish_nan_but_preserve_status(self) -> None:
        cases = [
            (valid_snapshot(104, point=(-10.0, 0.0, 0.0)), "non_positive_depth"),
            (valid_snapshot(105, point=(1000.0, 10000.0, 0.0)), "out_of_bounds"),
        ]
        clip_node = make_node()
        clip_node.camera_clip_far_m = 900.0
        cases.append((valid_snapshot(106, point=(1000.0, 0.0, 0.0)), "outside_clip_range"))

        for snapshot, expected_status in cases:
            node = clip_node if snapshot.sequence == 106 else make_node()
            StereoDistanceNode._process_attitude_model_snapshot(node, snapshot)
            assert_all_nan(self, node.attitude_actual_pixels_pub.messages[-1])
            assert_all_nan(self, node.attitude_raw_estimated_position_pub.messages[-1])
            status = status_payload(node)
            self.assertIn(expected_status, {status["actual_left_projection_status"], status["actual_right_projection_status"]})
            self.assertIsNotNone(status["actual_left_projection_status"])

    def test_zero_baseline_preserves_actual_pixels_and_invalidates_raw(self) -> None:
        node = make_node()
        node.baseline_m = 0.0
        measurement = StereoDistanceNode._process_attitude_model_snapshot(node, valid_snapshot(107))

        self.assertFalse(measurement.valid)
        self.assertEqual(measurement.status, "raw_triangulation_invalid:zero_baseline")
        self.assertFalse(any(math.isnan(float(value)) for value in node.attitude_actual_pixels_pub.messages[-1].data))
        assert_all_nan(self, node.attitude_raw_estimated_position_pub.messages[-1])
        status = status_payload(node)
        self.assertEqual(status["raw_triangulation_status"], "zero_baseline")
        self.assertEqual(status["ray_gap_m"], 0.0)
        self.assertIsNone(status["left_ray_parameter_m"])

    def test_attitude_branch_is_observation_mode_independent(self) -> None:
        for mode in ("bbox", "ideal_projection", "both"):
            node = make_node(mode)
            measurement = StereoDistanceNode._process_attitude_model_snapshot(node, valid_snapshot(108))
            self.assertTrue(measurement.valid)
            self.assertEqual(len(node.attitude_model_status_pub.messages), 1)
            self.assertEqual(status_payload(node)["oracle_sequence"], 108)

    def test_oracle_wrapper_calls_attitude_then_existing_ideal_with_same_snapshot(self) -> None:
        node = make_node("both")
        snapshot = valid_snapshot(109)
        calls = []

        def attitude(seen):
            calls.append(("attitude", seen))
            return "attitude-result"

        def ideal(seen):
            calls.append(("ideal", seen))
            return "ideal-result"

        node._process_attitude_model_snapshot = attitude
        node._process_ideal_snapshot = ideal
        result = StereoDistanceNode._process_oracle_snapshot_measurements(node, snapshot)

        self.assertEqual(result, ("attitude-result", "ideal-result"))
        self.assertEqual(calls, [("attitude", snapshot), ("ideal", snapshot)])

    def test_oracle_stream_source_uses_wrapper_for_all_snapshot_paths(self) -> None:
        source = (PACKAGE_ROOT / "stereo_ground_sim/stereo_distance_node.py").read_text(encoding="utf-8")
        self.assertEqual(source.count("_process_oracle_snapshot_measurements(snapshot_to_publish)"), 4)
        self.assertNotIn("_process_ideal_snapshot(snapshot_to_publish)", source)

    def test_status_uses_local_snapshot_not_current_cache(self) -> None:
        node = make_node()
        node.current_ground_truth_snapshot = valid_snapshot(201)
        StereoDistanceNode._process_attitude_model_snapshot(node, valid_snapshot(200))
        status = status_payload(node)
        self.assertEqual(status["oracle_sequence"], 200)

    def test_runtime_attitude_parameter_updates_are_restart_required(self) -> None:
        node = make_node()
        for name in node_module.ATTITUDE_STATIC_PARAMETERS:
            result = StereoDistanceNode._on_param_update(node, [FakeParam(name, 0.1)])
            self.assertFalse(result.successful)
            self.assertIn("full simulation restart", result.reason)

    def test_existing_topic_contracts_are_preserved_and_new_topics_exist(self) -> None:
        source = (PACKAGE_ROOT / "stereo_ground_sim/stereo_distance_node.py").read_text(encoding="utf-8")
        for topic in (
            '"/stereo/ideal/estimated_position"',
            '"/stereo/ideal/position_error"',
            '"/stereo/ideal/disparity"',
            '"/stereo/bbox_general_ray/estimated_position"',
            '"/stereo/bbox_general_ray/position_error"',
            '"/stereo/bbox_vs_ideal/pixel_delta"',
            '"/stereo/bbox_vs_ideal/position_delta"',
            '"/stereo/observation_status"',
            '"/stereo/attitude_model/actual/pixels"',
            '"/stereo/attitude_model/actual/disparity"',
            '"/stereo/attitude_model/raw/estimated_position"',
            '"/stereo/attitude_model/raw/position_error"',
            '"/stereo/attitude_model/raw/ray_quality"',
            '"/stereo/attitude_model/status"',
        ):
            self.assertIn(topic, source)


if __name__ == "__main__":
    unittest.main()
