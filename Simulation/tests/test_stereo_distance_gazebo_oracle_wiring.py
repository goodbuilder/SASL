from __future__ import annotations

import json
import math
import sys
import threading
import types
import unittest
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
    """Create temporary ROS stubs for direct-method Node import only.

    These tests validate direct-method Node wiring. They are not a ROS
    executor, transport, or Gazebo runtime smoke test.
    """

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


def import_stereo_distance_node_class():
    module_name = "stereo_ground_sim.stereo_distance_node"
    if find_spec("rclpy") is not None:
        return import_module(module_name).StereoDistanceNode, False

    snapshot = _module_snapshot()
    package = sys.modules.get("stereo_ground_sim")
    had_package_attr = package is not None and hasattr(package, "stereo_distance_node")
    package_attr = getattr(package, "stereo_distance_node", None) if package is not None else None
    try:
        sys.modules.pop(module_name, None)
        sys.modules.update(_make_ros_import_stubs())
        module = import_module(module_name)
        return module.StereoDistanceNode, True
    finally:
        _restore_module_snapshot(snapshot)
        if package is not None:
            if had_package_attr:
                setattr(package, "stereo_distance_node", package_attr)
            elif hasattr(package, "stereo_distance_node"):
                delattr(package, "stereo_distance_node")


from stereo_ground_sim.coordinate_frames import FrameId  # noqa: E402
from stereo_ground_sim.gazebo_oracle import OracleSourceRole, OracleStatus  # noqa: E402


StereoDistanceNode, USING_ROS_IMPORT_STUBS = import_stereo_distance_node_class()


R_B_W = (
    0.0,
    1.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -1.0,
)


class FakePublisher:
    def __init__(self, lock=None):
        self.messages = []
        self.lock = lock

    def publish(self, msg):
        if self.lock is not None:
            assert self.lock.depth == 0
        self.messages.append(msg)


class FakeParam:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class FakeProcess:
    def __init__(self, lines):
        self.stdout = list(lines)


class FakeCondition:
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False


class TrackingLock:
    def __init__(self):
        self.depth = 0

    def __enter__(self):
        self.depth += 1
        return self

    def __exit__(self, *_args):
        self.depth -= 1
        return False


def payload(name="x500_15m::target_reference_link", *, x=0.0, y=2000.0, z=0.75):
    return {"pose": [{"name": name, "position": {"x": x, "y": y, "z": z}}]}


def line_for(value):
    return json.dumps(value) + "\n"


def assert_tuple_close(testcase, actual, expected, delta=1.0e-9):
    testcase.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        testcase.assertAlmostEqual(float(actual_value), float(expected_value), delta=delta)


def make_node(*, tracking_lock=False):
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.target_model_name = "x500_15m"
    node.target_reference_link_name = "target_reference_link"
    node.oracle_base_link_name = "base_link"
    (
        node.gazebo_oracle_policy,
        node.gazebo_world_to_baseline_transform,
    ) = StereoDistanceNode._build_gazebo_oracle_configuration(
        target_model_name=node.target_model_name,
        target_reference_link_name=node.target_reference_link_name,
        oracle_base_link_name=node.oracle_base_link_name,
        baseline_origin_world_enu_m=(0.0, 0.0, 5.0),
        rotation_baseline_world_row_major=R_B_W,
    )
    node.actual_distance_source = "gazebo"
    node.actual_distance_read_hz = 1000.0
    node.last_gazebo_pose_sample_attempt = 0.0
    node.last_gazebo_pose_update = 0.0
    node.actual_distance_sample_seq = 0
    node.last_published_actual_distance_seq = 0
    node.cached_actual_distance = 2000.0
    node.cached_ground_truth_position = {
        "right_m": 0.0,
        "down_m": 0.0,
        "forward_m": 2000.0,
        "ground_truth_m": 2000.0,
    }
    node.has_valid_gazebo_ground_truth = False
    node.cached_ground_truth_metadata = StereoDistanceNode._initial_gazebo_oracle_metadata()
    node.shutdown_event = threading.Event()
    node.reference_distance_m = 2000.0
    node.fly_target_in_gazebo = False
    node.lock = TrackingLock() if tracking_lock else threading.RLock()
    node.metrics = {
        "distance_m": 123.0,
        "disparity_px": 4.0,
        "estimated_right_m": None,
        "estimated_down_m": None,
        "estimated_forward_m": None,
        "estimated_range_m": None,
        "corrected_right_m": None,
        "corrected_down_m": None,
        "corrected_forward_m": None,
        "corrected_range_m": None,
        "bbox_adjustment": None,
        "profile_status": {"profile_id": "", "profile_mode": "", "fallback_level": 3, "fallback_reason": "init"},
        "observation_fusion_status": "disabled",
        "general_ray_shadow": {"valid": False, "status": "not_computed"},
        "actual_distance_m": 2000.0,
        "distance_error_m": 77.0,
        "mission_phase": "init",
        "status": "unit",
    }
    publish_lock = node.lock if hasattr(node.lock, "depth") else None
    node.actual_distance_pub = FakePublisher(publish_lock)
    node.ground_truth_position_pub = FakePublisher(publish_lock)
    node.distance_pub = FakePublisher()
    node.error_pub = FakePublisher()
    node.disparity_pub = FakePublisher()
    node.estimated_position_pub = FakePublisher()
    node.corrected_position_pub = FakePublisher()
    node.bbox_adjustment_pub = FakePublisher()
    node.profile_status_pub = FakePublisher()
    node.comparison_pub = FakePublisher()
    node.yolo_inference_ms_pub = FakePublisher()
    node.yolo_update_hz_pub = FakePublisher()
    node.detections_pub = FakePublisher()
    node.yolo_condition = FakeCondition()
    node.yolo_inference_in_progress = False
    node.last_yolo_completion = None
    node.resolved_yolo_device = "cpu"
    node.gazebo_pose_stream_started = True
    node._mission_state = lambda: {"distance_m": 999.0, "phase": "unit"}
    return node


def consume_one(node, record):
    node.last_gazebo_pose_sample_attempt = 0.0
    StereoDistanceNode._consume_gazebo_pose_stream(node, FakeProcess([line_for(record)]))


def direct_yolo_metrics(actual, *, distance=123.0, disparity=4.0):
    error = None if distance is None else distance - actual
    return {
        "distance_m": distance,
        "disparity_px": disparity,
        "estimated_right_m": 0.0,
        "estimated_down_m": 0.0,
        "estimated_forward_m": distance,
        "estimated_range_m": distance,
        "corrected_right_m": None,
        "corrected_down_m": None,
        "corrected_forward_m": None,
        "corrected_range_m": None,
        "bbox_adjustment": None,
        "profile_status": {"profile_id": "", "profile_mode": "", "fallback_level": 3, "fallback_reason": "unit"},
        "observation_fusion_status": "disabled",
        "general_ray_shadow": {"valid": False, "status": "not_computed"},
        "actual_distance_m": actual,
        "distance_error_m": error,
        "mission_phase": "unit",
        "status": "unit yolo",
    }


def install_direct_yolo_stubs(node, *, metrics_factory=None):
    factory = metrics_factory or (lambda actual: direct_yolo_metrics(actual))
    node._msg_to_rgb = lambda msg: msg
    node._resolve_yolo_device = lambda: "cpu"
    node._detect_drone_pair = lambda left, right, device=None: (
        {"bbox": (0.0, 0.0, 1.0, 1.0), "confidence": 1.0, "cx": 0.5, "cy": 0.5},
        {"bbox": (0.0, 0.0, 1.0, 1.0), "confidence": 1.0, "cx": 0.4, "cy": 0.5},
    )
    node._calculate_metrics = lambda left_image, left, right, actual, phase: factory(actual)
    node._detections_to_msg = lambda left, right: types.SimpleNamespace(data=[1.0])
    node._publish_estimated_position = lambda metrics: None


def run_direct_yolo_update(node):
    StereoDistanceNode._update_metrics_from_messages(
        node,
        object(),
        object(),
        node.cached_actual_distance,
        "unit",
        device="cpu",
    )


class StereoDistanceGazeboOracleWiringTests(unittest.TestCase):
    def test_default_policy_order_and_names(self):
        node = make_node()

        self.assertEqual(
            tuple(source.role for source in node.gazebo_oracle_policy.sources),
            (
                OracleSourceRole.TARGET_REFERENCE_LINK,
                OracleSourceRole.BASE_LINK,
                OracleSourceRole.MODEL_POSE,
            ),
        )
        self.assertEqual(node.gazebo_oracle_policy.sources[0].exact_names[0], "x500_15m::target_reference_link")
        self.assertEqual(node.gazebo_oracle_policy.sources[1].exact_names[0], "x500_15m::base_link")
        self.assertEqual(node.gazebo_oracle_policy.sources[2].exact_names[0], "x500_15m")

    def test_default_transform_contract(self):
        node = make_node()
        transform = node.gazebo_world_to_baseline_transform

        self.assertEqual(transform.source_frame, FrameId.GAZEBO_WORLD_ENU)
        self.assertEqual(transform.target_frame, FrameId.BASELINE_CENTER_FRD)
        assert_tuple_close(self, transform.origin_target_in_source_m, (0.0, 0.0, 5.0))
        self.assertEqual(
            transform.rotation_target_source,
            ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
        )

    def test_configuration_rejects_invalid_origin_and_rotation_shapes(self):
        with self.assertRaisesRegex(ValueError, "baseline_origin_world_enu_m"):
            StereoDistanceNode._build_gazebo_oracle_configuration(
                target_model_name="x500_15m",
                target_reference_link_name="target_reference_link",
                oracle_base_link_name="base_link",
                baseline_origin_world_enu_m=(0.0, 0.0),
                rotation_baseline_world_row_major=R_B_W,
            )
        with self.assertRaisesRegex(ValueError, "rotation_baseline_world_row_major"):
            StereoDistanceNode._build_gazebo_oracle_configuration(
                target_model_name="x500_15m",
                target_reference_link_name="target_reference_link",
                oracle_base_link_name="base_link",
                baseline_origin_world_enu_m=(0.0, 0.0, 5.0),
                rotation_baseline_world_row_major=R_B_W[:8],
            )

    def test_configuration_rejects_non_finite_and_bool_numeric_elements(self):
        for origin in ((math.nan, 0.0, 5.0), (True, 0.0, 5.0)):
            with self.subTest(origin=origin):
                with self.assertRaises(ValueError):
                    StereoDistanceNode._build_gazebo_oracle_configuration(
                        target_model_name="x500_15m",
                        target_reference_link_name="target_reference_link",
                        oracle_base_link_name="base_link",
                        baseline_origin_world_enu_m=origin,
                        rotation_baseline_world_row_major=R_B_W,
                    )
        bad_rotation = list(R_B_W)
        bad_rotation[0] = math.inf
        with self.assertRaises(ValueError):
            StereoDistanceNode._build_gazebo_oracle_configuration(
                target_model_name="x500_15m",
                target_reference_link_name="target_reference_link",
                oracle_base_link_name="base_link",
                baseline_origin_world_enu_m=(0.0, 0.0, 5.0),
                rotation_baseline_world_row_major=bad_rotation,
            )

    def test_configuration_rejects_bad_rotation_matrices(self):
        non_orthonormal = (1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        reflection = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0)
        for rotation in (non_orthonormal, reflection):
            with self.subTest(rotation=rotation):
                with self.assertRaises(ValueError):
                    StereoDistanceNode._build_gazebo_oracle_configuration(
                        target_model_name="x500_15m",
                        target_reference_link_name="target_reference_link",
                        oracle_base_link_name="base_link",
                        baseline_origin_world_enu_m=(0.0, 0.0, 5.0),
                        rotation_baseline_world_row_major=rotation,
                    )

    def test_configuration_rejects_empty_entity_name(self):
        with self.assertRaisesRegex(ValueError, "target_model_name"):
            StereoDistanceNode._build_gazebo_oracle_configuration(
                target_model_name="",
                target_reference_link_name="target_reference_link",
                oracle_base_link_name="base_link",
                baseline_origin_world_enu_m=(0.0, 0.0, 5.0),
                rotation_baseline_world_row_major=R_B_W,
            )

    def test_static_oracle_parameters_are_rejected_without_partial_mutation(self):
        node = make_node()
        node.baseline_m = 25.0
        policy_id = id(node.gazebo_oracle_policy)
        transform_id = id(node.gazebo_world_to_baseline_transform)

        for name in (
            "target_model_name",
            "target_reference_link_name",
            "oracle_base_link_name",
            "baseline_origin_world_enu_m",
            "rotation_baseline_world_row_major",
        ):
            with self.subTest(name=name):
                result = StereoDistanceNode._on_param_update(
                    node,
                    [FakeParam(name, "changed"), FakeParam("baseline_m", 99.0)],
                )
                self.assertFalse(result.successful)
                self.assertIn("require node restart", result.reason)
                self.assertEqual(node.baseline_m, 25.0)
                self.assertEqual(id(node.gazebo_oracle_policy), policy_id)
                self.assertEqual(id(node.gazebo_world_to_baseline_transform), transform_id)

    def test_target_reference_exact_valid_wiring(self):
        node = make_node()

        truth = StereoDistanceNode._ground_truth_from_gazebo_pose_payload(
            node,
            payload("x500_15m::target_reference_link"),
        )

        self.assertEqual(set(truth), {"right_m", "down_m", "forward_m", "ground_truth_m"})
        self.assertAlmostEqual(truth["forward_m"], 2000.0)
        self.assertAlmostEqual(truth["right_m"], 0.0)
        self.assertAlmostEqual(truth["down_m"], 4.25)
        self.assertAlmostEqual(truth["ground_truth_m"], math.sqrt(2000.0**2 + 4.25**2))

    def test_base_and_model_fallback_wiring(self):
        for name, status, fallback_reason in (
            ("x500_15m::base_link", "ok_base_link_fallback", "target_reference_link_not_found"),
            ("x500_15m", "ok_model_pose_fallback", "target_reference_link_and_base_link_not_found"),
        ):
            with self.subTest(name=name):
                node = make_node()
                consume_one(node, payload(name))
                self.assertTrue(node.cached_ground_truth_metadata["oracle_valid"])
                self.assertEqual(node.cached_ground_truth_metadata["oracle_status"], status)
                self.assertEqual(node.cached_ground_truth_metadata["oracle_fallback_reason"], fallback_reason)

    def test_known_sdf_fixture_and_signed_coordinates(self):
        cases = (
            ((0.0, 2000.0, 0.75), (2000.0, 0.0, 4.25)),
            ((0.0, -10.0, 5.0), (-10.0, 0.0, 0.0)),
            ((-3.0, 0.0, 5.0), (0.0, -3.0, 0.0)),
            ((0.0, 0.0, 8.0), (0.0, 0.0, -3.0)),
            ((-2.0, -4.0, 7.0), (-4.0, -2.0, -2.0)),
        )
        for world, expected in cases:
            with self.subTest(world=world):
                node = make_node()
                truth = StereoDistanceNode._ground_truth_from_gazebo_pose_payload(
                    node,
                    payload(x=world[0], y=world[1], z=world[2]),
                )
                assert_tuple_close(
                    self,
                    (truth["forward_m"], truth["right_m"], truth["down_m"]),
                    expected,
                )

    def test_distance_wrapper_returns_signed_forward(self):
        node = make_node()

        forward = StereoDistanceNode._distance_from_gazebo_pose_payload(
            node,
            payload(x=0.0, y=-12.0, z=5.0),
        )

        self.assertEqual(forward, -12.0)

    def test_valid_stream_sample_updates_cache_metadata_sequence_and_publish(self):
        node = make_node()

        consume_one(node, payload(x=0.0, y=2000.0, z=0.75))

        self.assertTrue(node.has_valid_gazebo_ground_truth)
        self.assertEqual(node.actual_distance_sample_seq, 1)
        self.assertEqual(node.last_published_actual_distance_seq, 1)
        self.assertGreater(node.last_gazebo_pose_update, 0.0)
        self.assertAlmostEqual(node.cached_actual_distance, 2000.0)
        self.assertAlmostEqual(node.cached_ground_truth_position["ground_truth_m"], math.sqrt(2000.0**2 + 4.25**2))
        self.assertEqual(node.cached_ground_truth_metadata["oracle_status"], "ok_target_reference_link")
        self.assertEqual(node.cached_ground_truth_metadata["oracle_source_role"], "target_reference_link")
        self.assertIsNone(node.cached_ground_truth_metadata["oracle_fallback_reason"] or None)
        self.assertEqual(len(node.actual_distance_pub.messages), 1)
        self.assertEqual(node.actual_distance_pub.messages[-1].data, 2000.0)
        assert_tuple_close(
            self,
            [
                node.cached_ground_truth_position["forward_m"],
                node.cached_ground_truth_position["right_m"],
                node.cached_ground_truth_position["down_m"],
                node.cached_ground_truth_position["ground_truth_m"],
            ],
            [2000.0, 0.0, 4.25, math.sqrt(2000.0**2 + 4.25**2)],
        )
        assert_tuple_close(
            self,
            node.ground_truth_position_pub.messages[-1].data,
            [2000.0, 0.0, 4.25, math.sqrt(2000.0**2 + 4.25**2)],
            delta=1.0e-5,
        )
        json.dumps(node.cached_ground_truth_metadata, sort_keys=True)

    def test_invalid_samples_preserve_last_valid_cache_and_update_metadata(self):
        invalid_records = (
            payload("missing_model"),
            {
                "pose": [
                    payload("world_a::x500_15m::target_reference_link")["pose"][0],
                    payload("world_b::x500_15m::target_reference_link")["pose"][0],
                ]
            },
            {"name": "x500_15m::target_reference_link"},
            {"name": "x500_15m::target_reference_link", "position": [1.0, 2.0, 3.0]},
            payload(z=math.nan),
            "not a mapping",
        )
        expected_statuses = (
            "oracle_source_not_found",
            "ambiguous_entity_name",
            "missing_position",
            "unsupported_pose_format",
            "non_finite_position",
            "malformed_pose_payload",
        )
        for record, expected_status in zip(invalid_records, expected_statuses):
            with self.subTest(status=expected_status):
                node = make_node()
                consume_one(node, payload(x=0.0, y=2000.0, z=0.75))
                cached = dict(node.cached_ground_truth_position)
                seq = node.actual_distance_sample_seq
                last_valid = node.cached_ground_truth_metadata["oracle_last_valid_monotonic_s"]
                valid_publish_count = len(node.actual_distance_pub.messages)
                node.last_gazebo_pose_sample_attempt = 0.0

                consume_one(node, record)

                self.assertEqual(node.cached_ground_truth_position, cached)
                self.assertEqual(node.actual_distance_sample_seq, seq)
                self.assertEqual(len(node.actual_distance_pub.messages), valid_publish_count)
                self.assertEqual(node.cached_ground_truth_metadata["oracle_status"], expected_status)
                self.assertFalse(node.cached_ground_truth_metadata["oracle_valid"])
                self.assertTrue(node.cached_ground_truth_metadata["oracle_has_last_valid"])
                self.assertEqual(node.cached_ground_truth_metadata["oracle_last_valid_monotonic_s"], last_valid)
                self.assertGreaterEqual(node.cached_ground_truth_metadata["oracle_last_sample_monotonic_s"], last_valid)

    def test_json_decode_failure_updates_metadata_without_publish(self):
        node = make_node()
        node.last_gazebo_pose_sample_attempt = 0.0

        StereoDistanceNode._consume_gazebo_pose_stream(node, FakeProcess(["{bad json\n"]))

        self.assertEqual(node.cached_ground_truth_metadata["oracle_status"], "malformed_pose_payload")
        self.assertFalse(node.cached_ground_truth_metadata["oracle_valid"])
        self.assertFalse(node.cached_ground_truth_metadata["oracle_has_last_valid"])
        self.assertEqual(node.actual_distance_sample_seq, 0)
        self.assertEqual(len(node.actual_distance_pub.messages), 0)
        self.assertEqual(node.last_gazebo_pose_update, 0.0)
        self.assertGreater(node.last_gazebo_pose_sample_attempt, 0.0)

    def test_valid_recovery_after_invalid_sample(self):
        node = make_node()
        consume_one(node, payload("missing_model"))
        self.assertFalse(node.cached_ground_truth_metadata["oracle_valid"])
        node.last_gazebo_pose_sample_attempt = 0.0

        consume_one(node, payload(x=0.0, y=100.0, z=5.0))

        self.assertTrue(node.cached_ground_truth_metadata["oracle_valid"])
        self.assertTrue(node.cached_ground_truth_metadata["oracle_has_last_valid"])
        self.assertEqual(node.actual_distance_sample_seq, 1)
        self.assertEqual(node.cached_actual_distance, 100.0)

    def test_initial_fake_ground_truth_is_not_published(self):
        node = make_node()

        StereoDistanceNode._publish_actual_distance(node)

        self.assertEqual(node.actual_distance_sample_seq, 0)
        self.assertEqual(node.last_published_actual_distance_seq, 0)
        self.assertEqual(len(node.actual_distance_pub.messages), 0)
        self.assertEqual(len(node.ground_truth_position_pub.messages), 0)
        comparison = node.comparison_pub.messages[-1].data
        self.assertTrue(math.isnan(comparison[0]))
        self.assertTrue(math.isnan(comparison[2]))

    def test_stream_claim_prevents_plot_duplicate_publish(self):
        node = make_node()
        consume_one(node, payload(x=0.0, y=100.0, z=5.0))
        self.assertEqual(len(node.actual_distance_pub.messages), 1)

        StereoDistanceNode._publish_actual_distance(node)

        self.assertEqual(len(node.actual_distance_pub.messages), 1)
        self.assertEqual(len(node.ground_truth_position_pub.messages), 1)

    def test_plot_thread_claims_unpublished_valid_sequence(self):
        node = make_node()
        result = StereoDistanceNode._gazebo_oracle_result_from_payload(node, payload(x=0.0, y=100.0, z=5.0))
        position = StereoDistanceNode._ground_truth_position_from_oracle_result(node, result)
        node.cached_ground_truth_position = position
        node.cached_actual_distance = position["forward_m"]
        node.has_valid_gazebo_ground_truth = True
        node.cached_ground_truth_metadata = StereoDistanceNode._gazebo_oracle_metadata_from_result(
            node,
            result,
            now=1.0,
        )
        node.actual_distance_sample_seq = 7
        node.last_published_actual_distance_seq = 6

        StereoDistanceNode._publish_actual_distance(node)

        self.assertEqual(node.last_published_actual_distance_seq, 7)
        self.assertEqual(node.actual_distance_pub.messages[-1].data, 100.0)

    def test_invalid_latest_sample_masks_comparison_actual_and_error(self):
        node = make_node()
        consume_one(node, payload(x=0.0, y=100.0, z=5.0))
        node.metrics["actual_distance_m"] = 100.0
        node.metrics["distance_error_m"] = 12.0
        node.last_gazebo_pose_sample_attempt = 0.0
        consume_one(node, payload("missing_model"))

        StereoDistanceNode._publish_plot_samples(node)

        comparison = node.comparison_pub.messages[-1].data
        self.assertTrue(math.isnan(comparison[0]))
        self.assertEqual(comparison[1], 123.0)
        self.assertTrue(math.isnan(comparison[2]))
        self.assertEqual(comparison[3], 4.0)

    def test_valid_recovery_restores_comparison_actual_and_error(self):
        node = make_node()
        consume_one(node, payload("missing_model"))
        node.last_gazebo_pose_sample_attempt = 0.0
        consume_one(node, payload(x=0.0, y=-50.0, z=5.0))
        node.metrics["distance_error_m"] = -173.0

        StereoDistanceNode._publish_actual_distance(node)

        comparison = node.comparison_pub.messages[-1].data
        self.assertEqual(comparison[0], -50.0)
        self.assertEqual(comparison[2], -173.0)

    def test_direct_yolo_path_masks_invalid_latest_oracle_truth(self):
        node = make_node()
        consume_one(node, payload(x=0.0, y=100.0, z=5.0))
        cached_actual = node.cached_actual_distance
        cached_position = dict(node.cached_ground_truth_position)
        sequence = node.actual_distance_sample_seq
        last_valid_update = node.last_gazebo_pose_update
        node.last_gazebo_pose_sample_attempt = 0.0
        consume_one(node, payload("missing_model"))
        install_direct_yolo_stubs(node)

        run_direct_yolo_update(node)

        comparison = node.comparison_pub.messages[-1].data
        self.assertTrue(math.isnan(comparison[0]))
        self.assertEqual(comparison[1], 123.0)
        self.assertTrue(math.isnan(comparison[2]))
        self.assertEqual(comparison[3], 4.0)
        self.assertTrue(math.isnan(node.error_pub.messages[-1].data))
        self.assertEqual(node.cached_actual_distance, cached_actual)
        self.assertEqual(node.cached_ground_truth_position, cached_position)
        self.assertEqual(node.actual_distance_sample_seq, sequence)
        self.assertEqual(node.last_gazebo_pose_update, last_valid_update)
        self.assertTrue(math.isnan(node.metrics["actual_distance_m"]))
        self.assertTrue(math.isnan(node.metrics["distance_error_m"]))

    def test_direct_yolo_path_valid_recovery_uses_signed_oracle_truth(self):
        node = make_node()
        consume_one(node, payload("missing_model"))
        node.last_gazebo_pose_sample_attempt = 0.0
        consume_one(node, payload(x=0.0, y=-50.0, z=5.0))
        install_direct_yolo_stubs(node)

        run_direct_yolo_update(node)

        comparison = node.comparison_pub.messages[-1].data
        self.assertEqual(comparison[0], -50.0)
        self.assertEqual(comparison[1], 123.0)
        self.assertEqual(comparison[2], 173.0)
        self.assertEqual(node.error_pub.messages[-1].data, 173.0)

    def test_plot_and_direct_yolo_paths_share_invalid_comparison_policy(self):
        metrics = direct_yolo_metrics(100.0)
        node = make_node()
        consume_one(node, payload(x=0.0, y=100.0, z=5.0))
        node.last_gazebo_pose_sample_attempt = 0.0
        consume_one(node, payload("missing_model"))
        metadata = dict(node.cached_ground_truth_metadata)
        helper_actual, helper_error = StereoDistanceNode._comparison_actual_and_error_for_publish(
            node,
            metrics=metrics,
            ground_truth_metadata=metadata,
        )
        self.assertTrue(math.isnan(helper_actual))
        self.assertTrue(math.isnan(helper_error))

        node.metrics = dict(metrics)
        StereoDistanceNode._publish_plot_samples(node)
        plot_comparison = node.comparison_pub.messages[-1].data

        install_direct_yolo_stubs(node, metrics_factory=lambda actual: dict(metrics))
        run_direct_yolo_update(node)
        direct_comparison = node.comparison_pub.messages[-1].data

        self.assertTrue(math.isnan(plot_comparison[0]))
        self.assertEqual(plot_comparison[1], direct_comparison[1])
        self.assertTrue(math.isnan(plot_comparison[2]))
        self.assertTrue(math.isnan(direct_comparison[0]))
        self.assertTrue(math.isnan(direct_comparison[2]))

    def test_direct_yolo_publish_occurs_outside_lock(self):
        node = make_node(tracking_lock=True)
        consume_one(node, payload(x=0.0, y=100.0, z=5.0))
        node.last_gazebo_pose_sample_attempt = 0.0
        consume_one(node, payload("missing_model"))
        node.distance_pub = FakePublisher(node.lock)
        node.error_pub = FakePublisher(node.lock)
        node.disparity_pub = FakePublisher(node.lock)
        node.comparison_pub = FakePublisher(node.lock)
        node.detections_pub = FakePublisher(node.lock)
        node.yolo_inference_ms_pub = FakePublisher(node.lock)
        node.yolo_update_hz_pub = FakePublisher(node.lock)
        install_direct_yolo_stubs(node)

        run_direct_yolo_update(node)

        self.assertEqual(len(node.comparison_pub.messages), 1)
        self.assertEqual(len(node.error_pub.messages), 1)
        self.assertEqual(node.lock.depth, 0)

    def test_publish_ground_truth_position_order(self):
        node = make_node()
        position = {"forward_m": 1.0, "right_m": 2.0, "down_m": 3.0, "ground_truth_m": 4.0}

        StereoDistanceNode._publish_ground_truth_position(node, position)

        self.assertEqual(list(node.ground_truth_position_pub.messages[-1].data), [1.0, 2.0, 3.0, 4.0])

    def test_publish_occurs_outside_lock_for_stream_sample(self):
        node = make_node(tracking_lock=True)
        node.actual_distance_pub = FakePublisher(node.lock)
        node.ground_truth_position_pub = FakePublisher(node.lock)

        consume_one(node, payload(x=0.0, y=100.0, z=5.0))

        self.assertEqual(len(node.actual_distance_pub.messages), 1)
        self.assertEqual(len(node.ground_truth_position_pub.messages), 1)
        self.assertEqual(node.lock.depth, 0)

    def test_compatibility_wrappers_and_removed_legacy_finder(self):
        node = make_node()

        self.assertIsNone(StereoDistanceNode._ground_truth_from_gazebo_pose_payload(node, payload("missing_model")))
        self.assertIsNone(StereoDistanceNode._distance_from_gazebo_pose_payload(node, payload("missing_model")))
        self.assertFalse(hasattr(StereoDistanceNode, "_find_pose_by_name"))

    def test_fallback_ros_stubs_do_not_leak_sys_modules(self):
        if not USING_ROS_IMPORT_STUBS:
            self.skipTest("real rclpy import path is available")
        before = _module_snapshot()

        node_class, used_stubs = import_stereo_distance_node_class()
        after = _module_snapshot()

        self.assertTrue(used_stubs)
        self.assertEqual(node_class.__name__, "StereoDistanceNode")
        for name in ROS_IMPORT_MODULES:
            self.assertIs(after[name], before[name], msg=name)


if __name__ == "__main__":
    unittest.main()
