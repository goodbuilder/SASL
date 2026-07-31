"""ROS 2 GT actual-vs-ideal attitude measurement node for PoseSimulation M3.

The node subscribes only to the external Simulation pose interfaces defined by
M2.  It publishes typed diagnostics under ``/pose_simulation`` and has no
command, Gazebo, PX4, RTK, UI, or recording path.
"""

from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
from typing import Any, Mapping

try:  # Keep pure imports usable when ROS 2 or generated interfaces are absent.
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from pose_simulation_interfaces.msg import PoseError, PoseSimulationStatus, StereoGeometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - exercised on non-ROS test machines
    rclpy = None  # type: ignore[assignment]
    get_package_share_directory = None  # type: ignore[assignment]
    PoseStamped = None  # type: ignore[assignment,misc]
    PoseError = None  # type: ignore[assignment,misc]
    PoseSimulationStatus = None  # type: ignore[assignment,misc]
    StereoGeometry = None  # type: ignore[assignment,misc]
    Node = object  # type: ignore[assignment,misc]
    String = None  # type: ignore[assignment,misc]
    DurabilityPolicy = None  # type: ignore[assignment,misc]
    QoSProfile = None  # type: ignore[assignment,misc]
    ReliabilityPolicy = None  # type: ignore[assignment,misc]

try:
    import yaml
except ImportError:  # pragma: no cover - ROS distributions normally provide PyYAML
    yaml = None  # type: ignore[assignment]

from .config import ConfigError, resolve_config
from .input_adapters import (
    GroundTruthStreamValidator,
    StaticCameraSnapshotStore,
    adapt_camera_pose_diagnostic,
    adapt_ground_truth_pose,
)
from .input_types import (
    AlignedPoseInputs,
    CAMERA_OPTICAL_FRAME,
    GAZEBO_SIM_CLOCK,
    STATIC_SNAPSHOT_MODE,
    WAITING_FOR_GT,
    WORLD_FRAME,
)
from .measurement import GtMeasurementEngine, GtMeasurementResult
from .pose_math import (
    OPTICAL_RPY_CONVENTION,
    Pose3D,
    PoseContract,
    RIG_FRAME,
    matrix_to_quaternion_xyzw,
    validate_pose_contract,
)


DEFAULT_EVALUATION_FRAME = WORLD_FRAME
INVALID_METRIC = float("nan")


def _application_root() -> Path:
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory("pose_simulation"))
        except Exception:
            pass
    return Path(__file__).resolve().parents[1]


def _resolve_contract_path(raw_path: object) -> Path:
    path = Path(str(raw_path)).expanduser()
    if not path.is_absolute():
        path = _application_root() / path
    return path.resolve()


def load_pose_contract(path: Path) -> tuple[PoseContract, str]:
    """Load and validate the application-owned M1 contract with its hash."""

    if yaml is None:
        raise RuntimeError("PyYAML is required to load PoseSimulation pose_contract.yaml")
    if not path.is_file():
        raise ConfigError(f"PoseSimulation pose contract does not exist: {path}")
    raw_bytes = path.read_bytes()
    try:
        parsed = yaml.safe_load(raw_bytes.decode("utf-8"))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ConfigError(f"invalid PoseSimulation pose contract: {path}: {exc}") from exc
    try:
        contract = validate_pose_contract(parsed)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"invalid PoseSimulation pose contract: {path}: {exc}") from exc
    return contract, hashlib.sha256(raw_bytes).hexdigest()


def _stamp_to_ros(message: Any, stamp_ns: int | None) -> None:
    stamp = 0 if stamp_ns is None or stamp_ns < 0 else int(stamp_ns)
    message.header.stamp.sec = stamp // 1_000_000_000
    message.header.stamp.nanosec = stamp % 1_000_000_000


def _set_header(message: Any, stamp_ns: int | None, frame_id: str = WORLD_FRAME) -> None:
    _stamp_to_ros(message, stamp_ns)
    message.header.frame_id = frame_id


def _set_nan(value_owner: Any, field_name: str) -> None:
    setattr(value_owner, field_name, INVALID_METRIC)


def _metric_or_nan(valid: bool, value: float | None) -> float:
    if not valid or value is None or not math.isfinite(float(value)):
        return INVALID_METRIC
    return float(value)


def _pose_stamped(pose: Pose3D, stamp_ns: int | None, frame_id: str) -> Any:
    message = PoseStamped()
    _set_header(message, stamp_ns, frame_id)
    message.pose.position.x, message.pose.position.y, message.pose.position.z = (
        pose.position_world_enu_m
    )
    quaternion = matrix_to_quaternion_xyzw(pose.rotation_world_from_frame)
    (
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ) = quaternion
    return message


def pose_error_message(
    result: GtMeasurementResult,
    *,
    contract: PoseContract,
) -> Any:
    """Convert one pure result to the typed ``PoseError`` message."""

    message = PoseError()
    _set_header(message, result.evaluation_stamp_ns, contract.world_frame)
    message.evaluation_id = int(result.evaluation_id)
    message.valid = bool(result.valid)
    message.status = result.status
    message.reference_type = "ground_truth_ideal"
    message.world_frame = contract.world_frame
    message.error_frame = CAMERA_OPTICAL_FRAME
    message.euler_convention = OPTICAL_RPY_CONVENTION

    gt = result.ground_truth.ground_truth
    left = result.ground_truth.left_actual_camera
    right = result.ground_truth.right_actual_camera
    message.gt_stamp_ns = int(gt.stamp_ns or 0) if gt is not None else 0
    message.left_camera_source_stamp_ns = (
        int(left.gazebo_stamp_ns or 0) if left is not None else 0
    )
    message.right_camera_source_stamp_ns = (
        int(right.gazebo_stamp_ns or 0) if right is not None else 0
    )

    left_error = result.left_error
    right_error = result.right_error
    left_valid = result.valid and left_error is not None and left_error.valid
    right_valid = result.valid and right_error is not None and right_error.valid
    for prefix, error, valid in (
        ("left", left_error, left_valid),
        ("right", right_error, right_valid),
    ):
        if error is None:
            for suffix in (
                "roll_error_deg",
                "pitch_error_deg",
                "yaw_error_deg",
                "rotation_error_deg",
                "optical_axis_error_deg",
            ):
                _set_nan(message, f"{prefix}_{suffix}")
            setattr(message, f"{prefix}_gimbal_lock", False)
            continue
        rotation = error.rotation
        setattr(message, f"{prefix}_roll_error_deg", _metric_or_nan(valid, rotation.roll_error_deg))
        setattr(message, f"{prefix}_pitch_error_deg", _metric_or_nan(valid, rotation.pitch_error_deg))
        setattr(message, f"{prefix}_yaw_error_deg", _metric_or_nan(valid, rotation.yaw_error_deg))
        setattr(message, f"{prefix}_rotation_error_deg", _metric_or_nan(valid, rotation.rotation_error_deg))
        setattr(
            message,
            f"{prefix}_optical_axis_error_deg",
            _metric_or_nan(valid, error.optical_axis_to_target_error_deg),
        )
        setattr(message, f"{prefix}_gimbal_lock", bool(rotation.gimbal_lock))
    return message


def stereo_geometry_message(
    result: GtMeasurementResult,
    *,
    contract: PoseContract,
) -> Any:
    """Convert one pure result to the typed ``StereoGeometry`` message."""

    message = StereoGeometry()
    _set_header(message, result.evaluation_stamp_ns, contract.world_frame)
    message.evaluation_id = int(result.evaluation_id)
    message.valid = bool(result.valid)
    message.status = result.status
    message.world_frame = contract.world_frame
    message.rig_frame = contract.rig_frame
    relative = result.relative_error
    relative_valid = result.valid and relative is not None and relative.valid
    if relative is None:
        for field in (
            "relative_roll_error_deg",
            "relative_pitch_error_deg",
            "relative_yaw_error_deg",
            "relative_rotation_error_deg",
            "optical_axis_angle_deg",
        ):
            _set_nan(message, field)
    else:
        message.relative_roll_error_deg = _metric_or_nan(relative_valid, relative.roll_error_deg)
        message.relative_pitch_error_deg = _metric_or_nan(relative_valid, relative.pitch_error_deg)
        message.relative_yaw_error_deg = _metric_or_nan(relative_valid, relative.yaw_error_deg)
        message.relative_rotation_error_deg = _metric_or_nan(relative_valid, relative.rotation_error_deg)
        message.optical_axis_angle_deg = _metric_or_nan(relative_valid, relative.optical_axis_angle_deg)

    baseline = result.baseline
    baseline_valid = result.valid and baseline is not None and baseline.valid
    if baseline is None:
        vector = (INVALID_METRIC, INVALID_METRIC, INVALID_METRIC)
        for field in (
            "actual_baseline_length_m",
            "nominal_baseline_length_m",
            "baseline_length_error_m",
            "baseline_direction_error_deg",
        ):
            _set_nan(message, field)
        midpoint = vector
    else:
        vector = tuple(
            _metric_or_nan(baseline_valid, value)
            for value in baseline.vector_world_enu_m
        )
        message.actual_baseline_length_m = _metric_or_nan(baseline_valid, baseline.actual_length_m)
        message.nominal_baseline_length_m = _metric_or_nan(baseline_valid, baseline.nominal_length_m)
        message.baseline_length_error_m = _metric_or_nan(baseline_valid, baseline.length_error_m)
        message.baseline_direction_error_deg = _metric_or_nan(
            baseline_valid, baseline.direction_error_deg
        )
        midpoint = tuple(
            _metric_or_nan(baseline_valid, value)
            for value in baseline.midpoint_world_enu_m
        )
    message.actual_baseline_vector_world_enu_m.x = vector[0]
    message.actual_baseline_vector_world_enu_m.y = vector[1]
    message.actual_baseline_vector_world_enu_m.z = vector[2]
    message.actual_midpoint_world_enu_m.x = midpoint[0]
    message.actual_midpoint_world_enu_m.y = midpoint[1]
    message.actual_midpoint_world_enu_m.z = midpoint[2]
    return message


class PoseSimulationNode(Node):
    """Read-only GT-triggered PoseSimulation M3 node."""

    def __init__(self) -> None:  # pragma: no cover - ROS wiring is smoke-tested
        if rclpy is None or PoseError is None:
            raise RuntimeError(
                "PoseSimulation M3 requires ROS 2 and the built pose_simulation_interfaces package"
            )
        super().__init__("pose_simulation")
        self._config = self._load_config()
        self._values = self._config.values
        if self._values["POSESIM_CAMERA_POSE_MODE"] != STATIC_SNAPSHOT_MODE:
            raise RuntimeError("PoseSimulation M3 currently requires static_snapshot camera input")
        if not self._values["POSESIM_USE_SIM_TIME"] or self._values["POSESIM_EVALUATION_TIME_SOURCE"] != GAZEBO_SIM_CLOCK:
            raise RuntimeError("PoseSimulation M3 requires the Gazebo simulation clock contract")
        if self._values["POSESIM_RIG_CENTER_SOURCE"] != "actual_optical_midpoint":
            raise RuntimeError("PoseSimulation M3 requires actual optical midpoint as rig center")
        if self._values["POSESIM_STEREO_POLICY"] != "parallel_rigid":
            raise RuntimeError("PoseSimulation M3 requires the rigid parallel stereo policy")
        contract_path = _resolve_contract_path(self._values["POSESIM_POSE_CONTRACT_PATH"])
        self._contract, self._contract_sha256 = load_pose_contract(contract_path)
        self._latest_gt = None
        self._latest_left = None
        self._latest_right = None
        self._gt_validator = GroundTruthStreamValidator(
            reject_duplicate_stamps=bool(self._values["POSESIM_REJECT_DUPLICATE_STAMPS"]),
            reject_out_of_order=bool(self._values["POSESIM_REJECT_OUT_OF_ORDER"]),
        )
        self._left_store = StaticCameraSnapshotStore("left")
        self._right_store = StaticCameraSnapshotStore("right")
        self._engine = GtMeasurementEngine(self._contract)
        self._publishers = self._create_publishers()
        self._create_subscriptions()
        self._publish_waiting_status()
        self.get_logger().info(
            "PoseSimulation M3 started in read-only GT measurement mode; "
            "pose commands and RTK comparison are disabled"
        )

    @staticmethod
    def _load_config() -> Any:
        try:
            configured_path = os.environ.get("POSESIM_CONFIG")
            base_path = configured_path or str(_application_root() / "config" / "posesimulation.env")
            return resolve_config(config_path=base_path)
        except ConfigError:
            raise

    def _create_publishers(self) -> Mapping[str, Any]:
        return {
            "pose_error": self.create_publisher(
                PoseError, str(self._values["POSESIM_POSE_ERROR_TOPIC"]), 10
            ),
            "stereo_geometry": self.create_publisher(
                StereoGeometry, str(self._values["POSESIM_STEREO_GEOMETRY_TOPIC"]), 10
            ),
            "status": self.create_publisher(
                PoseSimulationStatus, str(self._values["POSESIM_STATUS_TOPIC"]), 10
            ),
            "ideal_rig": self.create_publisher(
                PoseStamped, str(self._values["POSESIM_GT_IDEAL_RIG_POSE_TOPIC"]), 10
            ),
            "ideal_left": self.create_publisher(
                PoseStamped, str(self._values["POSESIM_GT_IDEAL_LEFT_POSE_TOPIC"]), 10
            ),
            "ideal_right": self.create_publisher(
                PoseStamped, str(self._values["POSESIM_GT_IDEAL_RIGHT_POSE_TOPIC"]), 10
            ),
            # Read-only evaluated pose exposure for the RViz adapter.  These
            # topics carry the already validated M3 sample at the evaluation
            # stamp; they do not introduce a new source or alter measurement
            # semantics.
            "gt_target": self.create_publisher(
                PoseStamped, str(self._values["POSESIM_GT_TARGET_POSE_TOPIC"]), 10
            ),
            "actual_left": self.create_publisher(
                PoseStamped, str(self._values["POSESIM_ACTUAL_LEFT_POSE_TOPIC"]), 10
            ),
            "actual_right": self.create_publisher(
                PoseStamped, str(self._values["POSESIM_ACTUAL_RIGHT_POSE_TOPIC"]), 10
            ),
        }

    def _create_subscriptions(self) -> None:
        gt_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseStamped,
            str(self._values["POSESIM_GT_TOPIC"]),
            self._ground_truth_callback,
            gt_qos,
        )
        self.create_subscription(
            String,
            str(self._values["POSESIM_LEFT_CAMERA_POSE_TOPIC"]),
            self._left_camera_callback,
            camera_qos,
        )
        self.create_subscription(
            String,
            str(self._values["POSESIM_RIGHT_CAMERA_POSE_TOPIC"]),
            self._right_camera_callback,
            camera_qos,
        )

    def _ground_truth_callback(self, message: Any) -> None:
        sample = adapt_ground_truth_pose(
            message,
            source_topic=str(self._values["POSESIM_GT_TOPIC"]),
            reference_name=str(self._values["POSESIM_GT_REFERENCE_NAME"]),
            receive_timestamp_ns=self.get_clock().now().nanoseconds,
            clock_domain=GAZEBO_SIM_CLOCK,
        )
        self._latest_gt = self._gt_validator.accept(sample)
        self._publish_evaluation()

    def _camera_sample(self, message: Any, side: str) -> Any:
        key = (
            "POSESIM_LEFT_CAMERA_POSE_TOPIC"
            if side == "left"
            else "POSESIM_RIGHT_CAMERA_POSE_TOPIC"
        )
        entity_key = (
            "POSESIM_LEFT_CAMERA_ENTITY_NAME"
            if side == "left"
            else "POSESIM_RIGHT_CAMERA_ENTITY_NAME"
        )
        return adapt_camera_pose_diagnostic(
            getattr(message, "data", None),
            expected_side=side,
            source_topic=str(self._values[key]),
            expected_schema=str(self._values["POSESIM_CAMERA_POSE_SCHEMA"]),
            source_mode=str(self._values["POSESIM_CAMERA_POSE_MODE"]),
            expected_entity_name=str(self._values[entity_key]),
            receive_timestamp_ns=self.get_clock().now().nanoseconds,
            clock_domain=GAZEBO_SIM_CLOCK,
        )

    def _left_camera_callback(self, message: Any) -> None:
        sample = self._left_store.accept(self._camera_sample(message, "left"))
        self._latest_left = sample
        if not sample.valid and self._latest_gt is not None:
            self._publish_evaluation()
        elif not sample.valid:
            self._publish_waiting_status(sample.status, sample.invalid_reason)

    def _right_camera_callback(self, message: Any) -> None:
        sample = self._right_store.accept(self._camera_sample(message, "right"))
        self._latest_right = sample
        if not sample.valid and self._latest_gt is not None:
            self._publish_evaluation()
        elif not sample.valid:
            self._publish_waiting_status(sample.status, sample.invalid_reason)

    def _publish_evaluation(self) -> None:
        from .time_alignment import align_static_snapshot

        aligned = align_static_snapshot(
            self._latest_gt,
            self._latest_left,
            self._latest_right,
            expected_clock_domain=GAZEBO_SIM_CLOCK,
        )
        result = self._engine.evaluate(aligned)
        self._publish_status(result)
        if result.evaluation_stamp_ns is None:
            return
        self._publishers["pose_error"].publish(
            pose_error_message(result, contract=self._contract)
        )
        self._publishers["stereo_geometry"].publish(
            stereo_geometry_message(result, contract=self._contract)
        )
        if result.ideal_ground_truth is None:
            return
        ideal = result.ideal_ground_truth
        if ideal.rig_pose is not None:
            self._publishers["ideal_rig"].publish(
                _pose_stamped(ideal.rig_pose, result.evaluation_stamp_ns, self._contract.world_frame)
            )
        if ideal.left_pose is not None:
            self._publishers["ideal_left"].publish(
                _pose_stamped(ideal.left_pose, result.evaluation_stamp_ns, self._contract.world_frame)
            )
        if ideal.right_pose is not None:
            self._publishers["ideal_right"].publish(
                _pose_stamped(ideal.right_pose, result.evaluation_stamp_ns, self._contract.world_frame)
            )
        aligned_gt = result.ground_truth.ground_truth
        aligned_left = result.ground_truth.left_actual_camera
        aligned_right = result.ground_truth.right_actual_camera
        if aligned_gt is not None:
            self._publishers["gt_target"].publish(
                _pose_stamped(aligned_gt.pose, result.evaluation_stamp_ns, self._contract.world_frame)
            )
        if aligned_left is not None:
            self._publishers["actual_left"].publish(
                _pose_stamped(aligned_left.pose, result.evaluation_stamp_ns, self._contract.world_frame)
            )
        if aligned_right is not None:
            self._publishers["actual_right"].publish(
                _pose_stamped(aligned_right.pose, result.evaluation_stamp_ns, self._contract.world_frame)
            )

    def _publish_waiting_status(
        self,
        status: str = WAITING_FOR_GT,
        reason: str = "waiting for an external Ground Truth sample",
    ) -> None:
        self._publish_status(
            GtMeasurementResult(
                evaluation_id=0,
                evaluation_stamp_ns=None,
                valid=False,
                status=status,
                reason=reason,
                alignment_method="unresolved",
                ideal_ground_truth=None,
                left_error=None,
                right_error=None,
                relative_error=None,
                baseline=None,
                ground_truth=AlignedPoseInputs(
                    evaluation_id=None,
                    evaluation_stamp_ns=None,
                    ground_truth=None,
                    left_actual_camera=None,
                    right_actual_camera=None,
                    alignment_method="unresolved",
                    gt_time_residual_ns=None,
                    left_time_residual_ns=None,
                    right_time_residual_ns=None,
                    clock_domain=None,
                    valid=False,
                    status=status,
                    invalid_reason=reason,
                ),
            )
        )

    def _publish_status(self, result: GtMeasurementResult) -> None:
        message = PoseSimulationStatus()
        _set_header(message, result.evaluation_stamp_ns, self._contract.world_frame)
        message.evaluation_id = int(result.evaluation_id)
        message.valid = bool(result.valid)
        message.state = result.status
        message.reason = result.reason
        message.gt_topic = str(self._values["POSESIM_GT_TOPIC"])
        message.gt_reference_name = str(self._values["POSESIM_GT_REFERENCE_NAME"])
        message.left_camera_topic = str(self._values["POSESIM_LEFT_CAMERA_POSE_TOPIC"])
        message.right_camera_topic = str(self._values["POSESIM_RIGHT_CAMERA_POSE_TOPIC"])
        for prefix, sample in (("left", self._latest_left), ("right", self._latest_right)):
            if sample is None:
                setattr(message, f"{prefix}_entity_name", "")
                setattr(message, f"{prefix}_entity_id", -1)
                setattr(message, f"{prefix}_sdf_sha256", "")
            else:
                setattr(message, f"{prefix}_entity_name", sample.entity_name)
                setattr(message, f"{prefix}_entity_id", int(sample.entity_id or -1))
                setattr(message, f"{prefix}_sdf_sha256", sample.generated_sdf_sha256)
        message.pose_contract_sha256 = self._contract_sha256
        message.camera_source_mode = str(self._values["POSESIM_CAMERA_POSE_MODE"])
        message.read_only = True
        message.pose_command_enabled = False
        message.rtk_comparison_implemented = False
        self._publishers["status"].publish(message)

def main(args: list[str] | None = None) -> int:
    if rclpy is None:
        raise RuntimeError("PoseSimulation M3 requires an installed ROS 2 runtime")
    rclpy.init(args=args)
    node = None
    try:
        node = PoseSimulationNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


__all__ = [
    "PoseSimulationNode",
    "load_pose_contract",
    "main",
    "pose_error_message",
    "stereo_geometry_message",
]
