"""Compare each camera PX4 local-NED pose with its Gazebo ``base_link`` pose.

PX4 local odometry is intentionally relative to an estimator origin selected at
startup.  It is therefore not an absolute Gazebo world pose.  This diagnostic
captures the first valid PX4 NED pose and the contemporaneous Gazebo base-link
ENU pose as an explicit bootstrap transform, then reports subsequent residuals
in Gazebo world ENU coordinates.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from typing import Any


SCHEMA = "camera_px4_pose_comparison/v1"


def _vector3(values: Any, *, field: str) -> tuple[float, float, float]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must contain exactly three numeric values") from exc
    if len(result) != 3:
        raise ValueError(f"{field} must contain exactly three values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field} must contain only finite values")
    return result  # type: ignore[return-value]


def ned_delta_to_enu(delta_ned_m: tuple[float, float, float]) -> tuple[float, float, float]:
    """Convert a local NED displacement to an ENU displacement."""

    north_m, east_m, down_m = _vector3(delta_ned_m, field="delta_ned_m")
    return east_m, north_m, -down_m


def _subtract(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(left[index] - right[index] for index in range(3))  # type: ignore[return-value]


def _add(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(left[index] + right[index] for index in range(3))  # type: ignore[return-value]


def _norm(values: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in values))


@dataclass(frozen=True)
class CameraPx4PoseComparison:
    schema: str
    camera_label: str
    px4_odometry_topic: str
    px4_timestamp_us: int
    px4_pose_frame: str
    px4_position_ned_m: tuple[float, float, float]
    px4_bootstrap_position_ned_m: tuple[float, float, float]
    gazebo_stamp_ns: int
    gazebo_bootstrap_base_link_position_world_enu_m: tuple[float, float, float]
    observed_gazebo_base_link_position_world_enu_m: tuple[float, float, float]
    px4_relative_position_world_enu_m: tuple[float, float, float]
    px4_estimated_base_link_position_world_enu_m: tuple[float, float, float]
    px4_to_gazebo_base_link_delta_world_enu_m: tuple[float, float, float]
    px4_to_gazebo_base_link_delta_norm_m: float
    px4_reset_counter: int


def compare_px4_ned_to_gazebo_base_link(
    *,
    camera_label: str,
    px4_odometry_topic: str,
    px4_timestamp_us: int,
    px4_position_ned_m: tuple[float, float, float],
    px4_bootstrap_position_ned_m: tuple[float, float, float],
    gazebo_stamp_ns: int,
    gazebo_bootstrap_base_link_position_world_enu_m: tuple[float, float, float],
    observed_gazebo_base_link_position_world_enu_m: tuple[float, float, float],
    px4_reset_counter: int,
) -> CameraPx4PoseComparison:
    """Return a provenance-preserving PX4-local-NED to Gazebo-world-ENU comparison."""

    current_ned = _vector3(px4_position_ned_m, field="px4_position_ned_m")
    bootstrap_ned = _vector3(px4_bootstrap_position_ned_m, field="px4_bootstrap_position_ned_m")
    gazebo_bootstrap = _vector3(
        gazebo_bootstrap_base_link_position_world_enu_m,
        field="gazebo_bootstrap_base_link_position_world_enu_m",
    )
    gazebo_observed = _vector3(
        observed_gazebo_base_link_position_world_enu_m,
        field="observed_gazebo_base_link_position_world_enu_m",
    )
    relative_enu = ned_delta_to_enu(_subtract(current_ned, bootstrap_ned))
    px4_estimated = _add(gazebo_bootstrap, relative_enu)
    delta = _subtract(px4_estimated, gazebo_observed)
    if px4_timestamp_us <= 0 or gazebo_stamp_ns <= 0:
        raise ValueError("PX4 and Gazebo timestamps must be positive")
    return CameraPx4PoseComparison(
        schema=SCHEMA,
        camera_label=str(camera_label),
        px4_odometry_topic=str(px4_odometry_topic),
        px4_timestamp_us=int(px4_timestamp_us),
        px4_pose_frame="local_ned",
        px4_position_ned_m=current_ned,
        px4_bootstrap_position_ned_m=bootstrap_ned,
        gazebo_stamp_ns=int(gazebo_stamp_ns),
        gazebo_bootstrap_base_link_position_world_enu_m=gazebo_bootstrap,
        observed_gazebo_base_link_position_world_enu_m=gazebo_observed,
        px4_relative_position_world_enu_m=relative_enu,
        px4_estimated_base_link_position_world_enu_m=px4_estimated,
        px4_to_gazebo_base_link_delta_world_enu_m=delta,
        px4_to_gazebo_base_link_delta_norm_m=_norm(delta),
        px4_reset_counter=int(px4_reset_counter),
    )


def _gazebo_base_pose_from_message(data: str) -> tuple[int, tuple[float, float, float]]:
    payload = json.loads(data)
    if not isinstance(payload, dict) or payload.get("schema") != "camera_pose_comparison/v1":
        raise ValueError("Gazebo camera diagnostic schema is not camera_pose_comparison/v1")
    stamp_ns = payload.get("gazebo_stamp_ns")
    if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
        raise ValueError("Gazebo camera diagnostic has no positive gazebo_stamp_ns")
    return stamp_ns, _vector3(
        payload.get("observed_base_link_position_world_enu_m"),
        field="observed_base_link_position_world_enu_m",
    )


class CameraPx4PoseDiagnostics:
    """Publish a camera PX4 local-NED versus Gazebo base-link comparison stream."""

    def __init__(self) -> None:  # pragma: no cover - requires live ROS/PX4
        try:
            import rclpy
            from px4_msgs.msg import VehicleOdometry
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
        except ImportError as exc:
            raise RuntimeError("rclpy, std_msgs, and px4_msgs are required for PX4 pose diagnostics") from exc

        class _Node(Node):
            pass

        self._node = _Node("camera_px4_pose_diagnostics")
        for name, default in (
            ("camera_label", "left"),
            ("px4_odometry_topic", "/camera_left/fmu/out/vehicle_odometry"),
            ("gazebo_camera_diagnostic_topic", "/simulation/camera_pose_diagnostics/left"),
            ("output_topic", "/simulation/camera_px4_pose_diagnostics/left"),
        ):
            self._node.declare_parameter(name, default)
        self._camera_label = str(self._node.get_parameter("camera_label").value)
        self._px4_topic = str(self._node.get_parameter("px4_odometry_topic").value)
        self._string_type = String
        gazebo_topic = str(self._node.get_parameter("gazebo_camera_diagnostic_topic").value)
        output_topic = str(self._node.get_parameter("output_topic").value)
        self._latest_gazebo: tuple[int, tuple[float, float, float]] | None = None
        self._bootstrap_ned: tuple[float, float, float] | None = None
        self._bootstrap_gazebo: tuple[int, tuple[float, float, float]] | None = None
        self._publisher = self._node.create_publisher(
            String,
            output_topic,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._node.create_subscription(
            String,
            gazebo_topic,
            self._on_gazebo_diagnostic,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._node.create_subscription(
            VehicleOdometry,
            self._px4_topic,
            self._on_px4_odometry,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    @property
    def node(self) -> Any:
        return self._node

    def _on_gazebo_diagnostic(self, message: Any) -> None:
        try:
            self._latest_gazebo = _gazebo_base_pose_from_message(message.data)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._node.get_logger().warning(f"ignored invalid Gazebo camera diagnostic: {exc}")

    def _on_px4_odometry(self, message: Any) -> None:
        if self._latest_gazebo is None:
            return
        if int(message.pose_frame) != int(message.POSE_FRAME_NED):
            self._node.get_logger().warning("ignored PX4 odometry whose pose_frame is not local NED")
            return
        try:
            current_ned = _vector3(message.position, field="PX4 vehicle_odometry.position")
            timestamp_us = int(message.timestamp_sample or message.timestamp)
            if timestamp_us <= 0:
                raise ValueError("PX4 vehicle_odometry timestamp is not positive")
        except (TypeError, ValueError) as exc:
            self._node.get_logger().warning(f"ignored invalid PX4 odometry: {exc}")
            return
        if self._bootstrap_ned is None:
            self._bootstrap_ned = current_ned
            self._bootstrap_gazebo = self._latest_gazebo
            self._node.get_logger().info(
                "captured PX4 local-NED to Gazebo-world-ENU bootstrap "
                f"for {self._camera_label}: px4_ned={current_ned}, gazebo_enu={self._latest_gazebo[1]}"
            )
        assert self._bootstrap_gazebo is not None
        gazebo_stamp_ns, gazebo_position = self._latest_gazebo
        _, bootstrap_position = self._bootstrap_gazebo
        comparison = compare_px4_ned_to_gazebo_base_link(
            camera_label=self._camera_label,
            px4_odometry_topic=self._px4_topic,
            px4_timestamp_us=timestamp_us,
            px4_position_ned_m=current_ned,
            px4_bootstrap_position_ned_m=self._bootstrap_ned,
            gazebo_stamp_ns=gazebo_stamp_ns,
            gazebo_bootstrap_base_link_position_world_enu_m=bootstrap_position,
            observed_gazebo_base_link_position_world_enu_m=gazebo_position,
            px4_reset_counter=int(message.reset_counter),
        )
        self._publisher.publish(
            self._string_type(data=json.dumps(asdict(comparison), sort_keys=True, separators=(",", ":")))
        )

    def shutdown(self) -> None:
        self._node.destroy_node()


def main() -> None:  # pragma: no cover - requires live ROS/PX4
    import rclpy

    rclpy.init()
    diagnostics = CameraPx4PoseDiagnostics()
    try:
        rclpy.spin(diagnostics.node)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostics.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
