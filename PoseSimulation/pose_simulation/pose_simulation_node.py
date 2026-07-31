"""ROS 2 adapter for the diagnostic-only PoseSimulation application."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Any, Deque, Sequence

try:  # Keep pure-math imports usable on machines without ROS 2 installed.
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node as _RosNode
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - exercised only outside a ROS install
    rclpy = None  # type: ignore[assignment]
    PoseStamped = None  # type: ignore[assignment,misc]
    String = None  # type: ignore[assignment,misc]
    _RosNode = object  # type: ignore[assignment,misc]

from .coordinate_adapter import Wgs84ReferenceOrigin
from .evaluation import (
    NominalStereoExtrinsics,
    PoseEvaluationResult,
    evaluate_pose_snapshot,
)
from .gazebo_pose_adapter import GazeboPoseRecord, GazeboPoseStream
from .pose_math import (
    Matrix3,
    Pose3D,
    Vector3,
    as_matrix3,
    as_vector3,
    interpolate_pose,
    matrix_to_quaternion_xyzw,
)
from .sources import (
    RTK_STATUS_COORDINATE_TRANSFORM_FAILED,
    RTK_STATUS_JUMP_REJECTED,
    RTK_STATUS_NONFINITE,
    RTK_STATUS_STALE,
    RTK_STATUS_VALID_FIXED,
    RTK_STATUS_VALID_NON_FIXED,
    PoseSourceSample,
    RtkSample,
    SampleParseError,
    pose_source_from_camera_diagnostic_json,
    pose_source_from_pose_stamped,
    rtk_sample_from_message,
    rtk_world_position,
)


def _matrix_from_flat(values: Sequence[float], name: str) -> Matrix3:
    if len(values) != 9:
        raise ValueError(f"{name} must contain nine row-major values")
    return as_matrix3(
        [values[0:3], values[3:6], values[6:9]],
        name,
    )


def _now_receive_ns() -> int:
    return time.monotonic_ns()


class PoseSimulationNode(_RosNode):
    """Observe and compare poses; it never sends a pose or attitude command."""

    def __init__(self) -> None:  # pragma: no cover - ROS wiring is integration-tested
        if rclpy is None:
            raise RuntimeError("PoseSimulationNode requires an installed ROS 2 runtime")
        super().__init__("pose_simulation")
        self._declare_parameters()
        self._load_parameters()
        self._setup_publishers()
        self._setup_subscriptions()
        self._setup_gazebo_raw_pose_streams()
        self._timer = self.create_timer(1.0 / self._evaluation_rate_hz, self._evaluate_timer)
        self.get_logger().info(
            "PoseSimulation diagnostic-only node started: "
            f"ground_truth_primary={self._ground_truth_primary_topic}, "
            f"ground_truth_fallback={self._ground_truth_fallback_topic}, "
            f"rtk={self._rtk_topic}"
        )

    def _declare_parameters(self) -> None:
        declarations = {
            "ground_truth_primary_topic": "/simulation/ground_truth/target_reference_link",
            "ground_truth_fallback_topic": "/simulation/ground_truth/vehicle_reference_link",
            "ground_truth_model_fallback_topic": "",
            "target_entity_name": "target_reference_link",
            "gazebo_link_rotation_from_optical_flat": [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0],
            "enable_gazebo_raw_pose_streams": False,
            "gazebo_executable": "gz",
            "gazebo_target_pose_topic": "/model/target/pose",
            "gazebo_target_entity_name": "target_reference_link",
            "gazebo_target_parent_model_name": "target",
            "gazebo_left_pose_topic": "/model/left_stereo_camera/pose_static",
            "gazebo_left_entity_name": "left_stereo_camera::optical_link",
            "gazebo_left_parent_model_name": "left_stereo_camera",
            "gazebo_right_pose_topic": "/model/right_stereo_camera/pose_static",
            "gazebo_right_entity_name": "right_stereo_camera::optical_link",
            "gazebo_right_parent_model_name": "right_stereo_camera",
            "actual_left_camera_topic": "/simulation/camera_pose_diagnostics/left",
            "actual_right_camera_topic": "/simulation/camera_pose_diagnostics/right",
            "expected_left_optical_entity_name": "left_stereo_camera::optical_link",
            "expected_right_optical_entity_name": "right_stereo_camera::optical_link",
            "rtk_topic": "/fmu/out/vehicle_gps_position",
            "evaluation_rate_hz": 10.0,
            "history_size": 100,
            "max_source_age_s": 1.0,
            "max_interpolation_gap_s": 0.25,
            "max_gazebo_pair_skew_s": 0.25,
            "rtk_max_age_s": 1.0,
            "rtk_altitude_field": "altitude_ellipsoid_m",
            "minimum_rtk_fix_type": 5,
            "require_rtk_fixed": False,
            "max_rtk_horizontal_accuracy_m": 5.0,
            "max_rtk_vertical_accuracy_m": 5.0,
            "max_rtk_jump_m": 50.0,
            "reference_origin_latitude_deg": 0.0,
            "reference_origin_longitude_deg": 0.0,
            "reference_origin_altitude_m": 0.0,
            "reference_origin_altitude_datum": "ellipsoid",
            "gazebo_origin_east_m": 0.0,
            "gazebo_origin_north_m": 0.0,
            "gazebo_origin_up_m": 0.0,
            "left_translation_rig_m": [0.0, -12.5, 0.0],
            "right_translation_rig_m": [0.0, 12.5, 0.0],
            "left_rotation_rig_from_optical_flat": [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            "right_rotation_rig_from_optical_flat": [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            "world_up_world_enu": [0.0, 0.0, 1.0],
            "fallback_horizontal_axis_world_enu": [1.0, 0.0, 0.0],
            "near_vertical_threshold": 1.0e-9,
            "ground_truth_source_frame": "gazebo_world_enu",
            "target_point_alignment_mode": "zero_lever_arm_assumption",
            "lever_arm_applied": False,
        }
        for name, default in declarations.items():
            self.declare_parameter(name, default)

    def _parameter(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _load_parameters(self) -> None:
        self._ground_truth_primary_topic = str(self._parameter("ground_truth_primary_topic"))
        self._ground_truth_fallback_topic = str(self._parameter("ground_truth_fallback_topic"))
        self._ground_truth_model_fallback_topic = str(
            self._parameter("ground_truth_model_fallback_topic")
        ).strip()
        self._target_entity_name = str(self._parameter("target_entity_name"))
        self._camera_link_rotation_from_optical = _matrix_from_flat(
            self._parameter("gazebo_link_rotation_from_optical_flat"),
            "gazebo_link_rotation_from_optical_flat",
        )
        self._enable_gazebo_raw_pose_streams = bool(
            self._parameter("enable_gazebo_raw_pose_streams")
        )
        self._gazebo_executable = str(self._parameter("gazebo_executable"))
        self._gazebo_target_pose_topic = str(self._parameter("gazebo_target_pose_topic"))
        self._gazebo_target_entity_name = str(self._parameter("gazebo_target_entity_name"))
        self._gazebo_target_parent_model_name = str(
            self._parameter("gazebo_target_parent_model_name")
        )
        self._gazebo_left_pose_topic = str(self._parameter("gazebo_left_pose_topic"))
        self._gazebo_left_entity_name = str(self._parameter("gazebo_left_entity_name"))
        self._gazebo_left_parent_model_name = str(
            self._parameter("gazebo_left_parent_model_name")
        )
        self._gazebo_right_pose_topic = str(self._parameter("gazebo_right_pose_topic"))
        self._gazebo_right_entity_name = str(self._parameter("gazebo_right_entity_name"))
        self._gazebo_right_parent_model_name = str(
            self._parameter("gazebo_right_parent_model_name")
        )
        self._left_camera_topic = str(self._parameter("actual_left_camera_topic"))
        self._right_camera_topic = str(self._parameter("actual_right_camera_topic"))
        self._expected_left_optical_entity_name = str(
            self._parameter("expected_left_optical_entity_name")
        )
        self._expected_right_optical_entity_name = str(
            self._parameter("expected_right_optical_entity_name")
        )
        self._rtk_topic = str(self._parameter("rtk_topic"))
        self._evaluation_rate_hz = float(self._parameter("evaluation_rate_hz"))
        self._history_size = int(self._parameter("history_size"))
        self._max_source_age_ns = int(float(self._parameter("max_source_age_s")) * 1.0e9)
        self._max_interpolation_gap_ns = int(float(self._parameter("max_interpolation_gap_s")) * 1.0e9)
        self._max_gazebo_pair_skew_ns = int(float(self._parameter("max_gazebo_pair_skew_s")) * 1.0e9)
        self._rtk_max_age_ns = int(float(self._parameter("rtk_max_age_s")) * 1.0e9)
        if self._evaluation_rate_hz <= 0.0 or self._history_size < 2:
            raise ValueError("evaluation_rate_hz must be positive and history_size must be >= 2")
        self._minimum_rtk_fix_type = int(self._parameter("minimum_rtk_fix_type"))
        self._require_rtk_fixed = bool(self._parameter("require_rtk_fixed"))
        self._max_rtk_horizontal_accuracy_m = float(
            self._parameter("max_rtk_horizontal_accuracy_m")
        )
        self._max_rtk_vertical_accuracy_m = float(
            self._parameter("max_rtk_vertical_accuracy_m")
        )
        self._max_rtk_jump_m = float(self._parameter("max_rtk_jump_m"))
        self._rtk_altitude_field = str(self._parameter("rtk_altitude_field"))
        self._reference_origin = Wgs84ReferenceOrigin(
            float(self._parameter("reference_origin_latitude_deg")),
            float(self._parameter("reference_origin_longitude_deg")),
            float(self._parameter("reference_origin_altitude_m")),
            str(self._parameter("reference_origin_altitude_datum")),
        ).validate()
        self._gazebo_origin_offset = (
            float(self._parameter("gazebo_origin_east_m")),
            float(self._parameter("gazebo_origin_north_m")),
            float(self._parameter("gazebo_origin_up_m")),
        )
        left_translation = as_vector3(self._parameter("left_translation_rig_m"), "left_translation_rig_m")
        right_translation = as_vector3(self._parameter("right_translation_rig_m"), "right_translation_rig_m")
        left_rotation = _matrix_from_flat(
            self._parameter("left_rotation_rig_from_optical_flat"),
            "left_rotation_rig_from_optical_flat",
        )
        right_rotation = _matrix_from_flat(
            self._parameter("right_rotation_rig_from_optical_flat"),
            "right_rotation_rig_from_optical_flat",
        )
        self._extrinsics = NominalStereoExtrinsics(
            left_translation,
            right_translation,
            left_rotation,
            right_rotation,
        ).validate()
        self._world_up = as_vector3(self._parameter("world_up_world_enu"), "world_up_world_enu")
        self._fallback_axis = as_vector3(
            self._parameter("fallback_horizontal_axis_world_enu"),
            "fallback_horizontal_axis_world_enu",
        )
        self._near_vertical_threshold = float(self._parameter("near_vertical_threshold"))
        self._target_alignment_mode = str(self._parameter("target_point_alignment_mode"))
        self._lever_arm_applied = bool(self._parameter("lever_arm_applied"))

        self._gt_primary_history: Deque[PoseSourceSample] = deque(maxlen=self._history_size)
        self._gt_fallback_history: Deque[PoseSourceSample] = deque(maxlen=self._history_size)
        self._gt_model_history: Deque[PoseSourceSample] = deque(maxlen=self._history_size)
        self._left_history: Deque[PoseSourceSample] = deque(maxlen=self._history_size)
        self._right_history: Deque[PoseSourceSample] = deque(maxlen=self._history_size)
        self._latest_rtk: RtkSample | None = None
        self._rtk_world_position: Vector3 | None = None
        self._rtk_status = "INVALID_FIX"
        self._previous_accepted_rtk_world: Vector3 | None = None
        self._last_evaluation: PoseEvaluationResult | None = None
        self._gazebo_streams: list[GazeboPoseStream] = []

    def _setup_publishers(self) -> None:
        self._pose_error_publisher = self.create_publisher(String, "pose_error", 10)
        self._stereo_geometry_publisher = self.create_publisher(String, "stereo_geometry", 10)
        self._status_publisher = self.create_publisher(String, "status", 10)
        self._ideal_gt_left_publisher = self.create_publisher(
            PoseStamped, "ideal_pose/ground_truth/left", 10
        )
        self._ideal_gt_right_publisher = self.create_publisher(
            PoseStamped, "ideal_pose/ground_truth/right", 10
        )
        self._ideal_gt_rig_publisher = self.create_publisher(
            PoseStamped, "ideal_pose/ground_truth/rig", 10
        )
        self._ideal_rtk_left_publisher = self.create_publisher(
            PoseStamped, "ideal_pose/rtk/left", 10
        )
        self._ideal_rtk_right_publisher = self.create_publisher(
            PoseStamped, "ideal_pose/rtk/right", 10
        )
        self._ideal_rtk_rig_publisher = self.create_publisher(
            PoseStamped, "ideal_pose/rtk/rig", 10
        )
        self._actual_left_publisher = self.create_publisher(
            PoseStamped, "actual_pose/left", 10
        )
        self._actual_right_publisher = self.create_publisher(
            PoseStamped, "actual_pose/right", 10
        )

    def _setup_subscriptions(self) -> None:
        self.create_subscription(
            PoseStamped,
            self._ground_truth_primary_topic,
            self._ground_truth_primary_callback,
            10,
        )
        if self._ground_truth_fallback_topic:
            self.create_subscription(
                PoseStamped,
                self._ground_truth_fallback_topic,
                self._ground_truth_fallback_callback,
                10,
            )
        if self._ground_truth_model_fallback_topic:
            self.create_subscription(
                PoseStamped,
                self._ground_truth_model_fallback_topic,
                self._ground_truth_model_callback,
                10,
            )
        if self._left_camera_topic:
            self.create_subscription(String, self._left_camera_topic, self._left_camera_callback, 10)
        if self._right_camera_topic:
            self.create_subscription(String, self._right_camera_topic, self._right_camera_callback, 10)
        try:
            from px4_msgs.msg import VehicleGpsPosition as gps_message_type

            self._rtk_message_type_name = "px4_msgs/msg/VehicleGpsPosition"
        except ImportError:
            try:
                from px4_msgs.msg import SensorGps as gps_message_type

                self._rtk_message_type_name = "px4_msgs/msg/SensorGps (vehicle_gps_position)"
            except ImportError as exc:
                raise RuntimeError(
                    "px4_msgs must provide VehicleGpsPosition or SensorGps"
                ) from exc
        self.create_subscription(gps_message_type, self._rtk_topic, self._rtk_callback, 10)

    def _append_pose_sample(
        self,
        history: Deque[PoseSourceSample],
        sample: PoseSourceSample,
        label: str,
    ) -> None:
        if history and sample.source_timestamp_ns < history[-1].source_timestamp_ns:
            # A backwards Gazebo stamp normally means a simulation restart.
            # Do not mix epochs; start a fresh local history.
            history.clear()
            self.get_logger().warning(f"{label} timestamp moved backwards; starting a new sample epoch")
        history.append(sample)

    def _setup_gazebo_raw_pose_streams(self) -> None:
        if not self._enable_gazebo_raw_pose_streams:
            return
        target_source_is_authoritative = self._gazebo_target_entity_name == self._target_entity_name
        target_is_model_fallback = (
            self._gazebo_target_entity_name == self._gazebo_target_parent_model_name
        )
        if target_source_is_authoritative:
            target_history = self._gt_primary_history
            target_source = "TARGET_REFERENCE_LINK"
        elif target_is_model_fallback:
            target_history = self._gt_model_history
            target_source = "MODEL_POSE_FALLBACK"
        else:
            target_history = self._gt_fallback_history
            target_source = "CONFIGURED_FALLBACK_LINK"
        target_stream = GazeboPoseStream(
            topic=self._gazebo_target_pose_topic,
            entity_name=self._gazebo_target_entity_name,
            parent_model_name=self._gazebo_target_parent_model_name,
            on_record=lambda record: self._raw_target_record_callback(
                record, target_history, target_source
            ),
            on_error=lambda message: self.get_logger().warning(message),
            executable=self._gazebo_executable,
        )
        left_stream = GazeboPoseStream(
            topic=self._gazebo_left_pose_topic,
            entity_name=self._gazebo_left_entity_name,
            parent_model_name=self._gazebo_left_parent_model_name,
            on_record=lambda record: self._raw_camera_record_callback(
                record, self._left_history, "left"
            ),
            on_error=lambda message: self.get_logger().warning(message),
            executable=self._gazebo_executable,
        )
        right_stream = GazeboPoseStream(
            topic=self._gazebo_right_pose_topic,
            entity_name=self._gazebo_right_entity_name,
            parent_model_name=self._gazebo_right_parent_model_name,
            on_record=lambda record: self._raw_camera_record_callback(
                record, self._right_history, "right"
            ),
            on_error=lambda message: self.get_logger().warning(message),
            executable=self._gazebo_executable,
        )
        self._gazebo_streams = [target_stream, left_stream, right_stream]
        for stream in self._gazebo_streams:
            stream.start()
        self.get_logger().info("Raw Gazebo Pose_V observation streams enabled")

    def _raw_target_record_callback(
        self,
        record: GazeboPoseRecord,
        history: Deque[PoseSourceSample],
        source: str,
    ) -> None:
        sample = PoseSourceSample(
            source=source,
            frame_id="gazebo_world_enu",
            pose=record.as_pose(),
            source_timestamp_ns=record.stamp_ns,
            receive_timestamp_ns=_now_receive_ns(),
            entity_name=record.entity_name,
        )
        self._append_pose_sample(history, sample, source)

    def _raw_camera_record_callback(
        self,
        record: GazeboPoseRecord,
        history: Deque[PoseSourceSample],
        side: str,
    ) -> None:
        sample = PoseSourceSample(
            source=f"gazebo_actual_{side}_optical",
            frame_id="camera_optical_rdf",
            pose=record.as_optical_pose(self._camera_link_rotation_from_optical),
            source_timestamp_ns=record.stamp_ns,
            receive_timestamp_ns=_now_receive_ns(),
            entity_name=record.entity_name,
        )
        self._append_pose_sample(history, sample, f"raw actual {side} optical")

    def _ground_truth_primary_callback(self, message: Any) -> None:
        self._pose_callback(message, self._gt_primary_history, "TARGET_REFERENCE_LINK", "target_reference_link")

    def _ground_truth_fallback_callback(self, message: Any) -> None:
        self._pose_callback(message, self._gt_fallback_history, "CONFIGURED_FALLBACK_LINK", "vehicle_reference_link")

    def _ground_truth_model_callback(self, message: Any) -> None:
        self._pose_callback(message, self._gt_model_history, "MODEL_POSE_FALLBACK", "model_pose")

    def _pose_callback(
        self,
        message: Any,
        history: Deque[PoseSourceSample],
        source: str,
        entity_name: str,
    ) -> None:
        try:
            sample = pose_source_from_pose_stamped(
                message,
                source=source,
                receive_timestamp_ns=_now_receive_ns(),
                frame_id=str(self._parameter("ground_truth_source_frame")),
                entity_name=entity_name,
            )
            self._append_pose_sample(history, sample, source)
        except SampleParseError as exc:
            self.get_logger().warning(f"{source} sample rejected: {exc}")

    def _left_camera_callback(self, message: Any) -> None:
        self._camera_callback(message, self._left_history, "left")

    def _right_camera_callback(self, message: Any) -> None:
        self._camera_callback(message, self._right_history, "right")

    def _camera_callback(
        self,
        message: Any,
        history: Deque[PoseSourceSample],
        side: str,
    ) -> None:
        try:
            sample = pose_source_from_camera_diagnostic_json(
                message.data,
                source=f"gazebo_actual_{side}_optical",
                receive_timestamp_ns=_now_receive_ns(),
                camera_link_rotation_from_optical=self._camera_link_rotation_from_optical,
                expected_entity_name=(
                    self._expected_left_optical_entity_name
                    if side == "left"
                    else self._expected_right_optical_entity_name
                ),
            )
            self._append_pose_sample(history, sample, f"actual {side} optical")
        except (AttributeError, SampleParseError) as exc:
            self.get_logger().warning(f"actual {side} optical sample rejected: {exc}")

    def _rtk_callback(self, message: Any) -> None:
        receive_timestamp_ns = _now_receive_ns()
        try:
            sample = rtk_sample_from_message(
                message,
                source=self._rtk_topic,
                receive_timestamp_ns=receive_timestamp_ns,
            )
        except SampleParseError as exc:
            self._latest_rtk = None
            self._rtk_world_position = None
            self._rtk_status = RTK_STATUS_NONFINITE
            self.get_logger().warning(f"RTK sample rejected: {exc}")
            return
        self._latest_rtk = sample
        status = sample.status(
            minimum_fix_type=self._minimum_rtk_fix_type,
            require_rtk_fixed=False,
            max_horizontal_accuracy_m=self._max_rtk_horizontal_accuracy_m,
            max_vertical_accuracy_m=self._max_rtk_vertical_accuracy_m,
        )
        if status not in {RTK_STATUS_VALID_FIXED, RTK_STATUS_VALID_NON_FIXED}:
            self._rtk_world_position = None
            self._rtk_status = status
            return
        try:
            world_position = rtk_world_position(
                sample,
                altitude_field=self._rtk_altitude_field,
                reference_origin=self._reference_origin,
                gazebo_origin_in_reference_enu_m=self._gazebo_origin_offset,
            )
        except (SampleParseError, ValueError) as exc:
            self._rtk_world_position = None
            self._rtk_status = RTK_STATUS_COORDINATE_TRANSFORM_FAILED
            self.get_logger().warning(f"RTK coordinate conversion failed: {exc}")
            return
        if (
            self._previous_accepted_rtk_world is not None
            and math.isfinite(self._max_rtk_jump_m)
            and self._max_rtk_jump_m >= 0.0
        ):
            jump = math.sqrt(
                sum(
                    (world_position[index] - self._previous_accepted_rtk_world[index]) ** 2
                    for index in range(3)
                )
            )
            if jump > self._max_rtk_jump_m:
                self._rtk_world_position = None
                self._rtk_status = RTK_STATUS_JUMP_REJECTED
                return
        if self._require_rtk_fixed and status != RTK_STATUS_VALID_FIXED:
            self._rtk_world_position = None
            self._rtk_status = RTK_STATUS_VALID_NON_FIXED
            return
        self._rtk_world_position = world_position
        self._previous_accepted_rtk_world = world_position
        self._rtk_status = status

    @staticmethod
    def _fresh(sample: PoseSourceSample | None, now_ns: int, max_age_ns: int) -> bool:
        if sample is None:
            return False
        age_ns = now_ns - sample.receive_timestamp_ns
        return 0 <= age_ns <= max_age_ns

    def _select_ground_truth(self, now_ns: int) -> tuple[Deque[PoseSourceSample] | None, str, bool, str]:
        candidates = (
            (self._gt_primary_history, "TARGET_REFERENCE_LINK", False, "target_reference_link"),
            (self._gt_fallback_history, "CONFIGURED_FALLBACK_LINK", True, "vehicle_reference_link"),
            (self._gt_model_history, "MODEL_POSE_FALLBACK", True, "model_pose"),
        )
        for history, source, fallback_used, entity_name in candidates:
            if history and self._fresh(history[-1], now_ns, self._max_source_age_ns):
                return history, source, fallback_used, entity_name
        return None, "INVALID", False, self._target_entity_name

    def _sample_at_stamp(
        self,
        history: Deque[PoseSourceSample] | None,
        target_stamp_ns: int,
    ) -> PoseSourceSample | None:
        if not history:
            return None
        samples = list(history)
        if target_stamp_ns < samples[0].source_timestamp_ns:
            return None
        for sample in samples:
            if sample.source_timestamp_ns == target_stamp_ns:
                return sample
        for left, right in zip(samples, samples[1:]):
            if left.source_timestamp_ns <= target_stamp_ns <= right.source_timestamp_ns:
                gap = right.source_timestamp_ns - left.source_timestamp_ns
                if gap < 0 or gap > self._max_interpolation_gap_ns:
                    return None
                if gap == 0:
                    return left
                alpha = (target_stamp_ns - left.source_timestamp_ns) / gap
                pose = interpolate_pose(left.pose, right.pose, alpha)
                return PoseSourceSample(
                    source=left.source,
                    frame_id=left.frame_id,
                    pose=pose,
                    source_timestamp_ns=target_stamp_ns,
                    receive_timestamp_ns=max(left.receive_timestamp_ns, right.receive_timestamp_ns),
                    entity_name=left.entity_name,
                )
        # This branch is only an at-or-before sample.  It never extrapolates
        # forward; the caller chooses a target no newer than all latest data.
        previous = [sample for sample in samples if sample.source_timestamp_ns <= target_stamp_ns]
        if not previous:
            return None
        candidate = previous[-1]
        if target_stamp_ns - candidate.source_timestamp_ns > self._max_interpolation_gap_ns:
            return None
        return candidate

    def _synchronized_pose_samples(
        self,
        gt_history: Deque[PoseSourceSample] | None,
        now_receive_ns: int,
    ) -> tuple[PoseSourceSample | None, PoseSourceSample | None, PoseSourceSample | None, dict[str, Any]]:
        histories = (gt_history, self._left_history, self._right_history)
        latest = [history[-1] for history in histories if history]
        if len(latest) != 3:
            return None, None, None, {"alignment_status": "TIME_ALIGNMENT_FAILED"}
        if any(
            not self._fresh(sample, now_receive_ns, self._max_source_age_ns)
            for sample in latest
        ):
            return None, None, None, {
                "alignment_status": "TIME_ALIGNMENT_FAILED",
                "reason": "source_sample_stale",
            }
        target_stamp = min(sample.source_timestamp_ns for sample in latest)
        samples = tuple(self._sample_at_stamp(history, target_stamp) for history in histories)
        if any(sample is None for sample in samples):
            return None, None, None, {"alignment_status": "TIME_ALIGNMENT_FAILED"}
        gt, left, right = samples
        assert gt is not None and left is not None and right is not None
        stamps = (gt.source_timestamp_ns, left.source_timestamp_ns, right.source_timestamp_ns)
        skew = max(stamps) - min(stamps)
        if skew > self._max_gazebo_pair_skew_ns:
            return None, None, None, {
                "alignment_status": "TIME_ALIGNMENT_FAILED",
                "gazebo_pair_skew_ms": skew / 1.0e6,
            }
        return gt, left, right, {
            "alignment_status": "VALID",
            "gazebo_common_source_timestamp_ns": target_stamp,
            "gt_time_residual_ms": 0.0,
            "left_pose_time_residual_ms": 0.0,
            "right_pose_time_residual_ms": 0.0,
            "gazebo_pair_skew_ms": skew / 1.0e6,
        }

    def _evaluation_timestamp_ns(self) -> int:
        try:
            value = int(self.get_clock().now().nanoseconds)
        except Exception:
            value = 0
        return value if value > 0 else time.time_ns()

    @staticmethod
    def _pose_stamped_from_pose(pose: Pose3D, stamp_ns: int) -> Any:
        message = PoseStamped()
        message.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        message.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        message.header.frame_id = "gazebo_world_enu"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.position_world_enu_m
        quaternion = matrix_to_quaternion_xyzw(pose.rotation_world_from_frame)
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = quaternion
        return message

    def _publish_ideal_pose(self, publisher: Any, pose: Pose3D | None, stamp_ns: int) -> None:
        if pose is not None:
            publisher.publish(self._pose_stamped_from_pose(pose, pose.source_timestamp_ns or stamp_ns))

    def _publish_actual_pose(self, publisher: Any, sample: PoseSourceSample | None) -> None:
        if sample is not None:
            publisher.publish(self._pose_stamped_from_pose(sample.pose, sample.source_timestamp_ns))

    def destroy_node(self) -> bool:
        for stream in getattr(self, "_gazebo_streams", []):
            stream.shutdown()
        self._gazebo_streams = []
        return super().destroy_node()

    @staticmethod
    def _publish_json(publisher: Any, payload: dict[str, Any]) -> None:
        publisher.publish(String(data=json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)))

    def _evaluate_timer(self) -> None:
        now_receive_ns = _now_receive_ns()
        gt_history, gt_source, fallback_used, target_entity_name = self._select_ground_truth(now_receive_ns)
        gt, left, right, time_residuals = self._synchronized_pose_samples(
            gt_history, now_receive_ns
        )
        if self._latest_rtk is None:
            rtk_status = "INVALID_FIX"
            rtk_valid = False
            rtk_target = None
            time_residuals["rtk_time_residual_ms"] = None
        else:
            rtk_age_ns = now_receive_ns - self._latest_rtk.receive_timestamp_ns
            if rtk_age_ns > self._rtk_max_age_ns:
                rtk_status = RTK_STATUS_STALE
                rtk_valid = False
                rtk_target = None
            elif self._rtk_status in {RTK_STATUS_VALID_FIXED, RTK_STATUS_VALID_NON_FIXED}:
                rtk_status = self._rtk_status
                rtk_valid = self._rtk_world_position is not None
                rtk_target = self._rtk_world_position if rtk_valid else None
            else:
                rtk_status = self._rtk_status
                rtk_valid = False
                rtk_target = None
            # PX4 timestamp and Gazebo timestamp are distinct clock domains;
            # no numeric cross-domain subtraction is performed.
            time_residuals["rtk_time_residual_ms"] = None
            time_residuals["rtk_receive_age_ms"] = rtk_age_ns / 1.0e6
        result = evaluate_pose_snapshot(
            ground_truth_target=gt,
            left_actual=left,
            right_actual=right,
            rtk_target_world_enu_m=rtk_target,
            rtk_valid=rtk_valid,
            rtk_status=rtk_status,
            extrinsics=self._extrinsics,
            evaluation_timestamp_ns=self._evaluation_timestamp_ns(),
            ground_truth_source=gt_source,
            target_entity_name=target_entity_name,
            rtk_source_topic=self._rtk_topic,
            reference_origin=self._reference_origin,
            gazebo_origin_in_reference_enu_m=self._gazebo_origin_offset,
            altitude_datum=self._reference_origin.altitude_datum,
            target_point_alignment_mode=self._target_alignment_mode,
            lever_arm_applied=self._lever_arm_applied,
            rig_center_source="actual_optical_midpoint",
            fallback_used=fallback_used,
            time_alignment_method="latest_common_gazebo_stamp_with_linear_position_and_quaternion_slerp_no_future_extrapolation",
            time_residuals=time_residuals,
            near_vertical_threshold=self._near_vertical_threshold,
            world_up_world_enu=self._world_up,
            fallback_horizontal_axis_world_enu=self._fallback_axis,
        )
        self._last_evaluation = result
        self._publish_json(self._pose_error_publisher, result.pose_error)
        self._publish_json(self._stereo_geometry_publisher, result.stereo_geometry)
        result.status["rtk_message_type"] = self._rtk_message_type_name
        result.status["rtk_source_timestamp_us"] = (
            None if self._latest_rtk is None else self._latest_rtk.source_timestamp_us
        )
        result.status["px4_timestamp_us"] = result.status["rtk_source_timestamp_us"]
        result.status["rtk_receive_timestamp_ns"] = (
            None if self._latest_rtk is None else self._latest_rtk.receive_timestamp_ns
        )
        result.status["gazebo_link_rotation_from_optical_rdf"] = [
            value
            for row in self._camera_link_rotation_from_optical
            for value in row
        ]
        self._publish_json(self._status_publisher, result.status)
        self._publish_ideal_pose(self._ideal_gt_left_publisher, result.ideal_ground_truth.left_pose, result.status["evaluation_timestamp_ns"])
        self._publish_ideal_pose(self._ideal_gt_right_publisher, result.ideal_ground_truth.right_pose, result.status["evaluation_timestamp_ns"])
        self._publish_ideal_pose(self._ideal_gt_rig_publisher, result.ideal_ground_truth.rig_pose, result.status["evaluation_timestamp_ns"])
        self._publish_ideal_pose(self._ideal_rtk_left_publisher, result.ideal_rtk.left_pose, result.status["evaluation_timestamp_ns"])
        self._publish_ideal_pose(self._ideal_rtk_right_publisher, result.ideal_rtk.right_pose, result.status["evaluation_timestamp_ns"])
        self._publish_ideal_pose(self._ideal_rtk_rig_publisher, result.ideal_rtk.rig_pose, result.status["evaluation_timestamp_ns"])
        self._publish_actual_pose(self._actual_left_publisher, left)
        self._publish_actual_pose(self._actual_right_publisher, right)


def main() -> None:  # pragma: no cover - requires a ROS/Gazebo runtime
    if rclpy is None:
        raise RuntimeError("PoseSimulation requires ROS 2")
    rclpy.init()
    node = PoseSimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["PoseSimulationNode", "main"]
