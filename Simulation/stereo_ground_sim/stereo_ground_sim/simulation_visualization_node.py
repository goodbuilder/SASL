"""Research-only RViz adapter for the Simulation stereo pipeline.

The adapter consumes the values already published by ``StereoDistanceNode``.
It does not replace production projection or triangulation.  All visualization
geometry is expressed in the existing ``baseline_center_frd`` contract.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Any, Iterable

from stereo_ground_sim.coordinate_frames import FrameId
from stereo_ground_sim.stereo_triangulation_adapter import (
    nominal_stereo_rays_from_centers,
    pinhole_intrinsics_from_horizontal_fov,
)

BASELINE_FRAME = FrameId.BASELINE_CENTER_FRD.value
MARKER_TOPIC = "/simulation/visualization/markers"
GROUND_TRUTH_PATH_TOPIC = "/simulation/visualization/ground_truth_path"
RAW_PATH_TOPIC = "/simulation/visualization/raw_path"
CORRECTED_PATH_TOPIC = "/simulation/visualization/corrected_path"
_MARKER_IDENTITIES = (
    ("camera_optical_centers", 1), ("camera_optical_centers", 2),
    ("rays/analytic/left", 10), ("rays/analytic/right", 11),
    ("ground_truth", 20), ("raw_estimate", 21), ("corrected_estimate", 22),
    ("closest_points/analytic", 30), ("closest_points/analytic", 31),
    ("ray_gap/analytic", 32), ("baseline_axes", 40), ("baseline_axes", 41),
    ("diagnostics", 50),
)


def _finite(value: object, *, name: str) -> float:
    if isinstance(value, (bool, str)):
        raise TypeError(f"{name} must be finite numeric")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def finite_vector(values: Iterable[object] | None, *, length: int = 3) -> tuple[float, ...] | None:
    if values is None:
        return None
    values = tuple(values)
    if len(values) != length:
        return None
    try:
        return tuple(_finite(value, name="vector") for value in values)
    except (TypeError, ValueError):
        return None


def camera_centers_baseline_frd(baseline_m: object) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    baseline = _finite(baseline_m, name="baseline_m")
    if baseline < 0.0:
        raise ValueError("baseline_m must be non-negative")
    half = baseline * 0.5
    return (0.0, -half, 0.0), (0.0, half, 0.0)


def stable_marker_identity(namespace: str, marker_id: int) -> tuple[str, int]:
    if not isinstance(namespace, str) or not namespace:
        raise ValueError("namespace must be non-empty")
    if isinstance(marker_id, bool) or not isinstance(marker_id, int) or marker_id < 0:
        raise ValueError("marker_id must be a non-negative integer")
    return namespace, marker_id


def safe_marker_vector(values: Iterable[object] | None) -> tuple[float, float, float] | None:
    vector = finite_vector(values)
    return None if vector is None else vector  # type: ignore[return-value]


def _point_on_ray(origin: tuple[float, float, float], direction: tuple[float, float, float], parameter: object) -> tuple[float, float, float] | None:
    try:
        distance = _finite(parameter, name="ray_parameter_m")
    except (TypeError, ValueError):
        return None
    return tuple(origin[index] + distance * direction[index] for index in range(3))


try:  # Keep pure contract helpers importable in non-ROS unit tests.
    import rclpy
    from nav_msgs.msg import Path
    from geometry_msgs.msg import Point, PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import Image  # noqa: F401  # imported for ROS environment diagnostics
    from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:  # pragma: no cover - exercised only on non-ROS hosts
    rclpy = None
    Node = object  # type: ignore[assignment,misc]
    ExternalShutdownException = Exception
    Path = Point = PoseStamped = Float32MultiArray = Float64MultiArray = String = Marker = MarkerArray = None


class SimulationVisualizationNode(Node):
    """Publish stable MarkerArray and Path views of existing numeric topics."""

    def __init__(self) -> None:
        super().__init__("simulation_visualization_node")
        self.declare_parameter("baseline_m", 25.0)
        self.declare_parameter("camera_image_width_px", 1280)
        self.declare_parameter("camera_image_height_px", 720)
        self.declare_parameter("horizontal_fov_rad", 0.154997710698)
        self.declare_parameter("stale_timeout_s", 2.0)
        self.baseline_m = float(self.get_parameter("baseline_m").value)
        self.width_px = int(self.get_parameter("camera_image_width_px").value)
        self.height_px = int(self.get_parameter("camera_image_height_px").value)
        self.horizontal_fov_rad = float(self.get_parameter("horizontal_fov_rad").value)
        self.stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self.latest: dict[str, Any] = {"ground_truth": None, "raw": None, "corrected": None, "status": {}}
        self.last_update_monotonic_s = 0.0
        self.paths = {key: deque(maxlen=500) for key in ("ground_truth", "raw", "corrected")}
        self.marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)
        self.path_pubs = {
            "ground_truth": self.create_publisher(Path, GROUND_TRUTH_PATH_TOPIC, 10),
            "raw": self.create_publisher(Path, RAW_PATH_TOPIC, 10),
            "corrected": self.create_publisher(Path, CORRECTED_PATH_TOPIC, 10),
        }
        self.create_subscription(Float32MultiArray, "/ground_truth_position", self._ground_truth_cb, 10)
        self.create_subscription(Float64MultiArray, "/stereo/attitude_model/raw/estimated_position", self._raw_cb, 10)
        self.create_subscription(Float32MultiArray, "/stereo/corrected_position", self._corrected_cb, 10)
        self.create_subscription(String, "/stereo/attitude_model/status", self._status_cb, 10)
        self.create_timer(0.2, self._publish)

    def _position_callback(self, key: str, msg: Any) -> None:
        values = finite_vector(getattr(msg, "data", None), length=4)
        self.latest[key] = None if values is None else values[:3]
        if self.latest[key] is not None:
            self.paths[key].append(self.latest[key])
        self.last_update_monotonic_s = time.monotonic()

    def _ground_truth_cb(self, msg: Any) -> None:
        self._position_callback("ground_truth", msg)

    def _raw_cb(self, msg: Any) -> None:
        self._position_callback("raw", msg)

    def _corrected_cb(self, msg: Any) -> None:
        self._position_callback("corrected", msg)

    def _status_cb(self, msg: Any) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = {}
        self.latest["status"] = payload if isinstance(payload, dict) else {}
        self.last_update_monotonic_s = time.monotonic()

    @staticmethod
    def _stamp(node: "SimulationVisualizationNode") -> Any:
        return node.get_clock().now().to_msg()

    def _make_marker(self, marker_id: int, namespace: str, marker_type: int, *, action: int = 0) -> Any:
        marker = Marker()
        marker.header.frame_id = BASELINE_FRAME
        marker.header.stamp = self._stamp(self)
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = action
        marker.pose.orientation.w = 1.0
        return marker

    def _delete_markers(self) -> list[Any]:
        return [self._make_marker(marker_id, namespace, Marker.SPHERE, action=Marker.DELETE) for namespace, marker_id in _MARKER_IDENTITIES]

    def _point_marker(self, marker_id: int, namespace: str, point: tuple[float, float, float] | None, color: tuple[float, float, float]) -> Any | None:
        if point is None:
            return None
        marker = self._make_marker(marker_id, namespace, Marker.SPHERE)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point
        marker.scale.x = marker.scale.y = marker.scale.z = 1.8
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 1.0)
        return marker

    def _line_marker(self, marker_id: int, namespace: str, start: tuple[float, float, float], end: tuple[float, float, float], color: tuple[float, float, float]) -> Any:
        marker = self._make_marker(marker_id, namespace, Marker.LINE_LIST)
        marker.scale.x = 0.25
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 1.0)
        for values in (start, end):
            point = Point(x=values[0], y=values[1], z=values[2])
            marker.points.append(point)
        return marker

    def _publish(self) -> None:
        if not rclpy:
            return
        status = self.latest["status"]
        markers = self._delete_markers()
        try:
            left_center, right_center = camera_centers_baseline_frd(self.baseline_m)
            markers.extend([
                self._point_marker(1, "camera_optical_centers", left_center, (0.1, 0.5, 1.0)),
                self._point_marker(2, "camera_optical_centers", right_center, (1.0, 0.4, 0.1)),
                self._line_marker(40, "baseline_axes", (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                self._line_marker(41, "baseline_axes", (0.0, 0.0, 0.0), (0.0, 10.0, 0.0), (0.0, 1.0, 0.0)),
            ])
            intrinsics = pinhole_intrinsics_from_horizontal_fov(
                width_px=self.width_px, height_px=self.height_px, horizontal_fov_rad=self.horizontal_fov_rad
            )
            pixels = (status.get("actual_left_u_px"), status.get("actual_left_v_px"), status.get("actual_right_u_px"), status.get("actual_right_v_px"))
            if all(value is not None and math.isfinite(float(value)) for value in pixels):
                left_ray, right_ray = nominal_stereo_rays_from_centers(
                    left_cx_px=float(pixels[0]), left_cy_px=float(pixels[1]),
                    right_cx_px=float(pixels[2]), right_cy_px=float(pixels[3]),
                    intrinsics=intrinsics, baseline_m=self.baseline_m,
                )
                ray_length = max(20.0, float(status.get("left_ray_parameter_m") or 0.0), float(status.get("right_ray_parameter_m") or 0.0))
                left_end = tuple(left_ray.origin.xyz_m[i] + ray_length * left_ray.direction.xyz_unit[i] for i in range(3))
                right_end = tuple(right_ray.origin.xyz_m[i] + ray_length * right_ray.direction.xyz_unit[i] for i in range(3))
                markers.extend([
                    self._line_marker(10, "rays/analytic/left", left_ray.origin.xyz_m, left_end, (0.1, 0.7, 1.0)),
                    self._line_marker(11, "rays/analytic/right", right_ray.origin.xyz_m, right_end, (1.0, 0.5, 0.1)),
                ])
                left_closest = _point_on_ray(left_ray.origin.xyz_m, left_ray.direction.xyz_unit, status.get("left_ray_parameter_m"))
                right_closest = _point_on_ray(right_ray.origin.xyz_m, right_ray.direction.xyz_unit, status.get("right_ray_parameter_m"))
                for marker_id, point, color in ((30, left_closest, (0.1, 0.7, 1.0)), (31, right_closest, (1.0, 0.5, 0.1))):
                    marker = self._point_marker(marker_id, "closest_points/analytic", point, color)
                    if marker is not None:
                        markers.append(marker)
                if left_closest is not None and right_closest is not None:
                    markers.append(self._line_marker(32, "ray_gap/analytic", left_closest, right_closest, (1.0, 1.0, 0.1)))
            for marker_id, namespace, key, color in (
                (20, "ground_truth", "ground_truth", (0.2, 1.0, 0.2)),
                (21, "raw_estimate", "raw", (1.0, 0.2, 0.2)),
                (22, "corrected_estimate", "corrected", (0.8, 0.2, 1.0)),
            ):
                marker = self._point_marker(marker_id, namespace, self.latest[key], color)
                if marker is not None:
                    markers.append(marker)
        except (TypeError, ValueError, OverflowError):
            pass
        text = self._make_marker(50, "diagnostics", Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = (10.0, 0.0, -4.0)
        text.scale.z = 1.2
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "\n".join([
            f"source: {status.get('measurement_source', 'waiting')}",
            f"status: {status.get('attitude_model_status', 'waiting')}",
            f"ray gap: {status.get('ray_gap_m', '--')} m",
            f"angle: {status.get('intersection_angle_rad', '--')} rad",
            "ray source: analytic attitude-model pixels",
            "rays/observed: unavailable — no trusted observed ray topic",
            f"stale: {self.last_update_monotonic_s <= 0.0 or time.monotonic() - self.last_update_monotonic_s > self.stale_timeout_s}",
        ])
        markers.append(text)
        array = MarkerArray()
        array.markers = [marker for marker in markers if marker is not None]
        self.marker_pub.publish(array)
        self._publish_paths()

    def _publish_paths(self) -> None:
        stamp = self._stamp(self)
        for key, publisher in self.path_pubs.items():
            path = Path()
            path.header.frame_id = BASELINE_FRAME
            path.header.stamp = stamp
            for values in self.paths[key]:
                pose = PoseStamped()
                pose.header.frame_id = BASELINE_FRAME
                pose.header.stamp = stamp
                pose.pose.orientation.w = 1.0
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = values
                path.poses.append(pose)
            publisher.publish(path)


def main() -> None:
    if rclpy is None:  # pragma: no cover
        raise RuntimeError("ROS 2 is required to run simulation_visualization_node")
    rclpy.init()
    node = SimulationVisualizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = [
    "BASELINE_FRAME",
    "MARKER_TOPIC",
    "GROUND_TRUTH_PATH_TOPIC",
    "RAW_PATH_TOPIC",
    "CORRECTED_PATH_TOPIC",
    "camera_centers_baseline_frd",
    "finite_vector",
    "safe_marker_vector",
    "stable_marker_identity",
    "SimulationVisualizationNode",
    "main",
]
