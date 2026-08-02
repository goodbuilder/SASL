"""Read-only typed PoseSimulation adapter for RViz MarkerArray output."""

from __future__ import annotations

from typing import Any

try:  # Keep the pure visualization modules importable without ROS 2.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Point, PoseStamped
    from pose_simulation_interfaces.msg import PoseError, PoseSimulationStatus, StereoGeometry
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:  # pragma: no cover - exercised outside a ROS installation
    rclpy = None
    DiagnosticArray = None  # type: ignore[assignment,misc]
    DiagnosticStatus = None  # type: ignore[assignment,misc]
    KeyValue = None  # type: ignore[assignment,misc]
    Point = None  # type: ignore[assignment,misc]
    PoseStamped = None  # type: ignore[assignment,misc]
    PoseError = None  # type: ignore[assignment,misc]
    PoseSimulationStatus = None  # type: ignore[assignment,misc]
    StereoGeometry = None  # type: ignore[assignment,misc]
    Node = object  # type: ignore[assignment,misc]
    Marker = None  # type: ignore[assignment,misc]
    MarkerArray = None  # type: ignore[assignment,misc]

from .marker_factory import MarkerSpec
from .visualization_marker_factory import (
    VisualizationConfig,
    build_visualization_marker_specs,
)
from .visualization_snapshot import (
    build_visualization_snapshot,
    stamp_ns,
)


class PoseVisualizationNode(Node):
    """Pair typed M3 outputs and publish one evaluation-atomic MarkerArray."""

    def __init__(self) -> None:  # pragma: no cover - ROS wiring is smoke-tested
        if rclpy is None or PoseError is None or MarkerArray is None:
            raise RuntimeError(
                "PoseVisualizationNode requires ROS 2 and pose_simulation_interfaces"
            )
        super().__init__("pose_visualization")
        self._declare_parameters()
        self._load_parameters()
        self._typed_history: dict[str, dict[tuple[int, int], Any]] = {
            "pose_error": {},
            "stereo_geometry": {},
            "status": {},
        }
        self._pose_history: dict[str, dict[int, Any]] = {
            "gt_target": {},
            "actual_left": {},
            "actual_right": {},
            "ideal_gt_rig": {},
            "ideal_gt_left": {},
            "ideal_gt_right": {},
            "rtk_target": {},
            "ideal_rtk_rig": {},
            "ideal_rtk_left": {},
            "ideal_rtk_right": {},
        }
        self._last_signature: tuple[Any, ...] | None = None
        self._marker_publisher = self.create_publisher(MarkerArray, self._marker_topic, 10)
        self._diagnostic_publisher = self.create_publisher(DiagnosticArray, self._diagnostics_topic, 10)
        self._subscriptions = []
        self._create_subscriptions()
        self._publish_snapshot(build_visualization_snapshot(pose_error=None, stereo_geometry=None, status=None))
        self.get_logger().info(
            "PoseSimulation typed RViz visualization started read-only: "
            f"markers={self._marker_topic}"
        )

    def _declare_parameters(self) -> None:
        values = {
            "pose_error_topic": "pose_error",
            "stereo_geometry_topic": "stereo_geometry",
            "status_topic": "status",
            "gt_target_topic": "target/ground_truth",
            "actual_left_topic": "actual_pose/left",
            "actual_right_topic": "actual_pose/right",
            "ideal_gt_rig_topic": "ideal_pose/ground_truth/rig",
            "ideal_gt_left_topic": "ideal_pose/ground_truth/left",
            "ideal_gt_right_topic": "ideal_pose/ground_truth/right",
            "rtk_target_topic": "",
            "ideal_rtk_rig_topic": "ideal_pose/rtk/rig",
            "ideal_rtk_left_topic": "ideal_pose/rtk/left",
            "ideal_rtk_right_topic": "ideal_pose/rtk/right",
            "marker_topic": "visualization/markers",
            "diagnostics_topic": "diagnostics",
            "ray_mode": "normalized",
            "normalized_ray_length_m": 50.0,
            "camera_axis_length_m": 5.0,
            "rig_axis_length_m": 10.0,
            "target_marker_scale_m": 12.0,
            "optical_center_scale_m": 3.0,
            "frustum_length_m": 3.0,
            "frustum_width_m": 1.0,
            "frustum_height_m": 0.7,
            "marker_visual_scale": 1.0,
            "ray_shaft_width_m": 2.0,
            "ray_head_width_m": 2.0,
            "ray_line_width_m": 2.0,
            "show_meshes": True,
            "camera_mesh_resource": "package://pose_simulation/models/wiris_camera/meshes/WIRIS.stl",
            "camera_mesh_scale_m": 0.001,
            "drone_mesh_resource": "package://pose_simulation/models/x500_15m/meshes/x500_15m.stl",
            "drone_mesh_scale_m": 22.0,
            "show_actual": True,
            "show_gt_ideal": True,
            "show_rtk_ideal": False,
            "show_actual_forward": True,
            "show_gt_los": True,
            "show_rtk_los": False,
            "show_error_arcs": True,
            "show_baseline": True,
            "show_text": True,
            "show_detailed_optical_axes": False,
            "marker_lifetime_s": 0.0,
            "angular_visual_scale": 1.0,
            "position_error_visual_scale": 1.0,
        }
        for name, value in values.items():
            self.declare_parameter(name, value)

    def _parameter(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _topic(self, name: str) -> str:
        return str(self._parameter(name)).strip()

    def _load_parameters(self) -> None:
        self._topics = {name: self._topic(name) for name in (
            "pose_error_topic",
            "stereo_geometry_topic",
            "status_topic",
            "gt_target_topic",
            "actual_left_topic",
            "actual_right_topic",
            "ideal_gt_rig_topic",
            "ideal_gt_left_topic",
            "ideal_gt_right_topic",
            "rtk_target_topic",
            "ideal_rtk_rig_topic",
            "ideal_rtk_left_topic",
            "ideal_rtk_right_topic",
        )}
        self._marker_topic = self._topic("marker_topic")
        self._diagnostics_topic = self._topic("diagnostics_topic")
        self._config = VisualizationConfig(
            ray_mode=str(self._parameter("ray_mode")).strip().lower(),
            normalized_ray_length_m=float(self._parameter("normalized_ray_length_m")),
            camera_axis_length_m=float(self._parameter("camera_axis_length_m")),
            rig_axis_length_m=float(self._parameter("rig_axis_length_m")),
            target_marker_scale_m=float(self._parameter("target_marker_scale_m")),
            optical_center_scale_m=float(self._parameter("optical_center_scale_m")),
            frustum_length_m=float(self._parameter("frustum_length_m")),
            frustum_width_m=float(self._parameter("frustum_width_m")),
            frustum_height_m=float(self._parameter("frustum_height_m")),
            marker_visual_scale=float(self._parameter("marker_visual_scale")),
            ray_shaft_width_m=float(self._parameter("ray_shaft_width_m")),
            ray_head_width_m=float(self._parameter("ray_head_width_m")),
            ray_line_width_m=float(self._parameter("ray_line_width_m")),
            show_meshes=bool(self._parameter("show_meshes")),
            camera_mesh_resource=str(self._parameter("camera_mesh_resource")).strip(),
            camera_mesh_scale_m=float(self._parameter("camera_mesh_scale_m")),
            drone_mesh_resource=str(self._parameter("drone_mesh_resource")).strip(),
            drone_mesh_scale_m=float(self._parameter("drone_mesh_scale_m")),
            show_actual=bool(self._parameter("show_actual")),
            show_gt_ideal=bool(self._parameter("show_gt_ideal")),
            show_rtk_ideal=bool(self._parameter("show_rtk_ideal")),
            show_actual_forward=bool(self._parameter("show_actual_forward")),
            show_gt_los=bool(self._parameter("show_gt_los")),
            show_rtk_los=bool(self._parameter("show_rtk_los")),
            show_error_arcs=bool(self._parameter("show_error_arcs")),
            show_baseline=bool(self._parameter("show_baseline")),
            show_text=bool(self._parameter("show_text")),
            show_detailed_optical_axes=bool(self._parameter("show_detailed_optical_axes")),
            marker_lifetime_s=float(self._parameter("marker_lifetime_s")),
            angular_visual_scale=float(self._parameter("angular_visual_scale")),
            position_error_visual_scale=float(self._parameter("position_error_visual_scale")),
        ).validate()

    def _create_subscriptions(self) -> None:
        typed = (
            ("pose_error", PoseError, "pose_error_topic"),
            ("stereo_geometry", StereoGeometry, "stereo_geometry_topic"),
            ("status", PoseSimulationStatus, "status_topic"),
        )
        for name, message_type, parameter_name in typed:
            self._subscriptions.append(
                self.create_subscription(
                    message_type,
                    self._topics[parameter_name],
                    lambda message, key=name: self._typed_callback(key, message),
                    10,
                )
            )
        pose_topics = (
            ("gt_target", "gt_target_topic"),
            ("actual_left", "actual_left_topic"),
            ("actual_right", "actual_right_topic"),
            ("ideal_gt_rig", "ideal_gt_rig_topic"),
            ("ideal_gt_left", "ideal_gt_left_topic"),
            ("ideal_gt_right", "ideal_gt_right_topic"),
            ("rtk_target", "rtk_target_topic"),
            ("ideal_rtk_rig", "ideal_rtk_rig_topic"),
            ("ideal_rtk_left", "ideal_rtk_left_topic"),
            ("ideal_rtk_right", "ideal_rtk_right_topic"),
        )
        for name, parameter_name in pose_topics:
            topic = self._topics[parameter_name]
            if not topic:
                continue
            self._subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda message, key=name: self._pose_callback(key, message),
                    10,
                )
            )

    def _typed_callback(self, name: str, message: Any) -> None:
        message_stamp = stamp_ns(message)
        try:
            message_id = int(message.evaluation_id)
        except (AttributeError, TypeError, ValueError):
            return
        if message_stamp is None:
            return
        history = self._typed_history[name]
        history[(message_id, message_stamp)] = message
        while len(history) > 64:
            del history[next(iter(history))]
        self._try_publish_latest_status()

    def _pose_callback(self, name: str, message: Any) -> None:
        message_stamp = stamp_ns(message)
        if message_stamp is None:
            return
        history = self._pose_history[name]
        history[message_stamp] = message
        while len(history) > 64:
            del history[next(iter(history))]
        self._try_publish_latest_status()

    def _latest_status(self) -> Any | None:
        history = self._typed_history["status"]
        if not history:
            return None
        return max(history.values(), key=lambda message: (stamp_ns(message) or 0, int(message.evaluation_id)))

    def _try_publish_latest_status(self) -> None:
        status = self._latest_status()
        if status is None:
            return
        evaluation_id = int(status.evaluation_id)
        evaluation_stamp_ns = stamp_ns(status) or 0
        key = (evaluation_id, evaluation_stamp_ns)
        pose_error = self._typed_history["pose_error"].get(key)
        stereo_geometry = self._typed_history["stereo_geometry"].get(key)
        poses = {
            name: history.get(evaluation_stamp_ns)
            for name, history in self._pose_history.items()
        }
        source_topics = {
            "pose_error": self._topics["pose_error_topic"],
            "stereo_geometry": self._topics["stereo_geometry_topic"],
            "status": self._topics["status_topic"],
            **{name: self._topics[f"{name}_topic"] for name in (
                "gt_target", "actual_left", "actual_right", "ideal_gt_rig", "ideal_gt_left", "ideal_gt_right",
                "rtk_target", "ideal_rtk_rig", "ideal_rtk_left", "ideal_rtk_right",
            )},
        }
        snapshot = build_visualization_snapshot(
            pose_error=pose_error,
            stereo_geometry=stereo_geometry,
            status=status,
            gt_target=poses["gt_target"],
            actual_left=poses["actual_left"],
            actual_right=poses["actual_right"],
            ideal_gt_rig=poses["ideal_gt_rig"],
            ideal_gt_left=poses["ideal_gt_left"],
            ideal_gt_right=poses["ideal_gt_right"],
            rtk_target=poses["rtk_target"],
            ideal_rtk_rig=poses["ideal_rtk_rig"],
            ideal_rtk_left=poses["ideal_rtk_left"],
            ideal_rtk_right=poses["ideal_rtk_right"],
            source_topics=source_topics,
        )
        self._publish_snapshot(snapshot)

    def _publish_snapshot(self, snapshot: Any) -> None:
        signature = (
            snapshot.evaluation_id,
            snapshot.evaluation_stamp_ns,
            snapshot.valid,
            snapshot.state,
            snapshot.invalid_reason,
            snapshot.rtk_available,
            tuple(sorted(snapshot.source_identity.items())),
        )
        if signature == self._last_signature:
            return
        self._last_signature = signature
        specs = build_visualization_marker_specs(snapshot, self._config)
        message = MarkerArray()
        message.markers = [self._marker_from_spec(spec) for spec in specs]
        self._marker_publisher.publish(message)
        self._diagnostic_publisher.publish(self._diagnostic_array(snapshot))

    def _marker_from_spec(self, spec: MarkerSpec) -> Any:
        marker = Marker()
        marker.header.frame_id = spec.frame_id
        stamp = spec.stamp_ns
        if stamp <= 0:
            stamp = int(self.get_clock().now().nanoseconds)
        marker.header.stamp.sec = int(stamp // 1_000_000_000)
        marker.header.stamp.nanosec = int(stamp % 1_000_000_000)
        marker.ns = spec.namespace
        marker.id = int(spec.marker_id)
        marker.action = Marker.DELETE if spec.action == "DELETE" else Marker.ADD
        marker.type = {
            "arrow": Marker.ARROW,
            "line_strip": Marker.LINE_STRIP,
            "line_list": Marker.LINE_LIST,
            "sphere": Marker.SPHERE,
            "text": Marker.TEXT_VIEW_FACING,
            "mesh": Marker.MESH_RESOURCE,
            "none": Marker.CUBE,
        }.get(spec.marker_type, Marker.CUBE)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec.position
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = spec.orientation_xyzw
        if Point is not None:
            marker.points = [Point(x=point[0], y=point[1], z=point[2]) for point in spec.points]
        marker.scale.x, marker.scale.y, marker.scale.z = spec.scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec.color
        marker.text = spec.text
        marker.mesh_resource = spec.mesh_resource
        marker.mesh_use_embedded_materials = spec.mesh_use_embedded_materials
        marker.lifetime.sec = int(max(0.0, spec.lifetime_s))
        marker.lifetime.nanosec = int((max(0.0, spec.lifetime_s) % 1.0) * 1.0e9)
        return marker

    def _diagnostic_array(self, snapshot: Any) -> Any:
        array = DiagnosticArray()
        array.header.frame_id = snapshot.world_frame
        array.header.stamp.sec = int(max(0, snapshot.evaluation_stamp_ns) // 1_000_000_000)
        array.header.stamp.nanosec = int(max(0, snapshot.evaluation_stamp_ns) % 1_000_000_000)
        status = DiagnosticStatus()
        status.name = "PoseSimulation/Visualization"
        status.level = bytes([0 if snapshot.valid else 1])
        status.message = "VALID" if snapshot.valid else snapshot.invalid_reason or snapshot.state
        status.values = [
            KeyValue(key="evaluation_id", value=str(snapshot.evaluation_id)),
            KeyValue(key="evaluation_stamp_ns", value=str(snapshot.evaluation_stamp_ns)),
            KeyValue(key="fixed_frame", value=snapshot.world_frame),
            KeyValue(key="read_only", value="true"),
            KeyValue(key="pose_command_enabled", value="false"),
        ]
        array.status.append(status)
        return array


def main() -> None:  # pragma: no cover - requires ROS 2 runtime
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
