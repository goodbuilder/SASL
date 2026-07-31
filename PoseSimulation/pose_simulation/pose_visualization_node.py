"""ROS 2 adapter that publishes read-only PoseSimulation RViz diagnostics."""

from __future__ import annotations

import json
import math
import time
from typing import Any, Mapping

try:  # Keep the pure marker and snapshot modules importable without ROS 2.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Point, PoseStamped
    from rclpy.node import Node
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:  # pragma: no cover - exercised outside a ROS installation
    rclpy = None
    DiagnosticArray = None  # type: ignore[assignment,misc]
    DiagnosticStatus = None  # type: ignore[assignment,misc]
    KeyValue = None  # type: ignore[assignment,misc]
    Point = None  # type: ignore[assignment,misc]
    PoseStamped = None  # type: ignore[assignment,misc]
    Node = object  # type: ignore[assignment,misc]
    String = None  # type: ignore[assignment,misc]
    Marker = None  # type: ignore[assignment,misc]
    MarkerArray = None  # type: ignore[assignment,misc]

from .diagnostic_adapter import build_diagnostic_records
from .marker_factory import MarkerSpec, build_marker_specs
from .ui_snapshot import PoseRecord, PoseSimulationSnapshot, WORLD_FRAME, build_ui_snapshot


def _stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _pose_record_from_ros(message: Any, *, source: str) -> PoseRecord:
    frame_id = str(getattr(message.header, "frame_id", "") or "")
    position = (
        float(message.pose.position.x),
        float(message.pose.position.y),
        float(message.pose.position.z),
    )
    quaternion = (
        float(message.pose.orientation.x),
        float(message.pose.orientation.y),
        float(message.pose.orientation.z),
        float(message.pose.orientation.w),
    )
    if frame_id != WORLD_FRAME:
        raise ValueError(f"{source} frame mismatch: {frame_id}")
    if not all(math.isfinite(value) for value in (*position, *quaternion)):
        raise ValueError(f"{source} contains NaN or Inf")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError(f"{source} quaternion is zero")
    normalized = tuple(value / norm for value in quaternion)
    return PoseRecord(
        frame_id=frame_id,
        position_world_enu_m=position,
        orientation_xyzw=normalized,  # type: ignore[arg-type]
        source_timestamp_ns=_stamp_ns(message),
        source=source,
    )


class PoseVisualizationNode(Node):
    """Pair existing PoseSimulation outputs and publish MarkerArray/diagnostics."""

    def __init__(self) -> None:  # pragma: no cover - ROS wiring is smoke-tested
        if rclpy is None:
            raise RuntimeError("PoseVisualizationNode requires ROS 2")
        super().__init__("pose_visualization")
        self._declare_parameters()
        self._load_parameters()
        self._payloads: dict[str, Mapping[str, Any]] = {}
        self._payload_history: dict[str, dict[str, Mapping[str, Any]]] = {
            "pose_error": {},
            "stereo_geometry": {},
            "status": {},
        }
        self._actual_poses: dict[str, dict[int, PoseRecord]] = {
            "actual_left": {},
            "actual_right": {},
        }
        self._last_snapshot: PoseSimulationSnapshot | None = None
        self._marker_publisher = self.create_publisher(MarkerArray, self._marker_topic, 10)
        self._diagnostic_publisher = self.create_publisher(DiagnosticArray, self._diagnostics_topic, 10)
        self._subscriptions = [
            self.create_subscription(String, self._pose_error_topic, lambda msg: self._json_callback("pose_error", msg), 10),
            self.create_subscription(String, self._stereo_geometry_topic, lambda msg: self._json_callback("stereo_geometry", msg), 10),
            self.create_subscription(String, self._status_topic, lambda msg: self._json_callback("status", msg), 10),
            self.create_subscription(PoseStamped, self._actual_left_topic, lambda msg: self._actual_pose_callback("actual_left", msg), 10),
            self.create_subscription(PoseStamped, self._actual_right_topic, lambda msg: self._actual_pose_callback("actual_right", msg), 10),
        ]
        self.get_logger().info(
            "PoseSimulation read-only visualization started: "
            f"markers={self._marker_topic}, diagnostics={self._diagnostics_topic}"
        )

    def _declare_parameters(self) -> None:
        declarations = {
            "pose_error_topic": "pose_error",
            "stereo_geometry_topic": "stereo_geometry",
            "status_topic": "status",
            "actual_left_topic": "actual_pose/left",
            "actual_right_topic": "actual_pose/right",
            "marker_topic": "visualization/markers",
            "diagnostics_topic": "diagnostics",
            "ray_mode": "normalized",
            "normalized_ray_length_m": 50.0,
            "show_detail_axes": False,
            "axis_length_m": 12.0,
        }
        for name, default in declarations.items():
            self.declare_parameter(name, default)

    def _parameter(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _load_parameters(self) -> None:
        self._pose_error_topic = str(self._parameter("pose_error_topic"))
        self._stereo_geometry_topic = str(self._parameter("stereo_geometry_topic"))
        self._status_topic = str(self._parameter("status_topic"))
        self._actual_left_topic = str(self._parameter("actual_left_topic"))
        self._actual_right_topic = str(self._parameter("actual_right_topic"))
        self._marker_topic = str(self._parameter("marker_topic"))
        self._diagnostics_topic = str(self._parameter("diagnostics_topic"))
        self._ray_mode = str(self._parameter("ray_mode")).strip().lower()
        if self._ray_mode not in {"normalized", "full_distance"}:
            raise ValueError("ray_mode must be normalized or full_distance")
        self._normalized_ray_length_m = float(self._parameter("normalized_ray_length_m"))
        self._show_detail_axes = bool(self._parameter("show_detail_axes"))
        self._axis_length_m = float(self._parameter("axis_length_m"))
        if self._normalized_ray_length_m <= 0.0 or self._axis_length_m <= 0.0:
            raise ValueError("visualization lengths must be positive")

    def _json_callback(self, name: str, message: Any) -> None:
        try:
            payload = json.loads(str(message.data))
            if not isinstance(payload, Mapping):
                raise ValueError("payload is not a JSON object")
            stored = dict(payload)
            self._payloads[name] = stored
            evaluation_timestamp = stored.get("evaluation_timestamp_ns")
            evaluation_id = None
            try:
                evaluation_id = f"evaluation:{int(evaluation_timestamp)}"
            except (TypeError, ValueError):
                pass
            if evaluation_id is not None:
                history = self._payload_history[name]
                history[evaluation_id] = stored
                while len(history) > 32:
                    del history[next(iter(history))]
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"PoseSimulation {name} payload rejected: {exc}")
            self._payloads.pop(name, None)
        self._try_publish_snapshot()

    def _actual_pose_callback(self, name: str, message: Any) -> None:
        try:
            record = _pose_record_from_ros(message, source=name)
            history = self._actual_poses[name]
            history[record.source_timestamp_ns] = record
            while len(history) > 64:
                del history[next(iter(history))]
        except (AttributeError, TypeError, ValueError) as exc:
            self._actual_poses[name].clear()
            self.get_logger().warning(f"PoseSimulation {name} pose rejected: {exc}")
        self._try_publish_snapshot()

    def _try_publish_snapshot(self) -> None:
        common_ids = set(self._payload_history["pose_error"])
        common_ids &= set(self._payload_history["stereo_geometry"])
        common_ids &= set(self._payload_history["status"])
        if common_ids:
            evaluation_id = sorted(common_ids, key=lambda value: int(value.split(":", 1)[1]))[-1]
            pose_error = self._payload_history["pose_error"][evaluation_id]
            stereo_geometry = self._payload_history["stereo_geometry"][evaluation_id]
            status = self._payload_history["status"][evaluation_id]
            actual_poses: dict[str, PoseRecord] = {}
            for name, timestamp_key in (
                ("actual_left", "left_camera_pose_timestamp_ns"),
                ("actual_right", "right_camera_pose_timestamp_ns"),
            ):
                try:
                    timestamp = int(status.get(timestamp_key))
                except (TypeError, ValueError):
                    continue
                record = self._actual_poses[name].get(timestamp)
                if record is not None:
                    actual_poses[name] = record
        else:
            pose_error = self._payloads.get("pose_error")
            stereo_geometry = self._payloads.get("stereo_geometry")
            status = self._payloads.get("status")
            actual_poses = {}
        snapshot = build_ui_snapshot(
            pose_error,
            stereo_geometry,
            status,
            actual_poses=actual_poses,
            now_receive_ns=time.monotonic_ns(),
        )
        self._last_snapshot = snapshot
        specs = build_marker_specs(
            snapshot,
            ray_mode=self._ray_mode,
            normalized_ray_length_m=self._normalized_ray_length_m,
            show_detail_axes=self._show_detail_axes,
            axis_length_m=self._axis_length_m,
        )
        self._marker_publisher.publish(self._marker_array_from_specs(specs))
        self._diagnostic_publisher.publish(self._diagnostic_array_from_snapshot(snapshot))

    def _marker_array_from_specs(self, specs: tuple[MarkerSpec, ...]) -> Any:
        message = MarkerArray()
        message.markers = [self._marker_from_spec(spec) for spec in specs]
        return message

    def _marker_from_spec(self, spec: MarkerSpec) -> Any:
        marker = Marker()
        marker.header.frame_id = spec.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = spec.namespace
        marker.id = int(spec.marker_id)
        marker.action = Marker.DELETEALL if spec.action == "DELETEALL" else Marker.ADD
        marker.type = {
            "arrow": Marker.ARROW,
            "line_strip": Marker.LINE_STRIP,
            "line_list": Marker.LINE_LIST,
            "sphere": Marker.SPHERE,
            "text": Marker.TEXT_VIEW_FACING,
            "none": Marker.CUBE,
        }.get(spec.marker_type, Marker.CUBE)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec.position
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = spec.orientation_xyzw
        if Point is not None:
            marker.points = [Point(x=point[0], y=point[1], z=point[2]) for point in spec.points]
        marker.scale.x, marker.scale.y, marker.scale.z = spec.scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec.color
        marker.text = spec.text
        marker.lifetime.sec = int(max(0.0, spec.lifetime_s))
        marker.lifetime.nanosec = int(max(0.0, spec.lifetime_s) % 1.0 * 1.0e9)
        return marker

    def _diagnostic_array_from_snapshot(self, snapshot: PoseSimulationSnapshot) -> Any:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = WORLD_FRAME
        for record in build_diagnostic_records(snapshot):
            status = DiagnosticStatus()
            status.name = record.name
            # Humble's generated DiagnosticStatus setter represents uint8 as
            # a one-byte Python bytes value.
            status.level = bytes([max(0, min(255, int(record.level)))])
            status.message = record.message
            status.values = [KeyValue(key=key, value=value) for key, value in record.values]
            array.status.append(status)
        return array


def main() -> None:  # pragma: no cover - requires ROS 2/Gazebo runtime
    if rclpy is None:
        raise RuntimeError("PoseVisualizationNode requires ROS 2")
    rclpy.init()
    node = PoseVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["PoseVisualizationNode", "main"]
