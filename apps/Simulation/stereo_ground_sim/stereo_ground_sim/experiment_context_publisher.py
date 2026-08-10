"""Publish the immutable Simulation experiment context over ROS 2."""

from __future__ import annotations

import math
from typing import Any

try:  # pragma: no cover - pure payload construction is tested without ROS
    import rclpy
    from geometry_msgs.msg import Pose, Vector3
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from simulation_interfaces.msg import SimulationExperimentContext
    from std_msgs.msg import Header
except ImportError:  # pragma: no cover
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]
    Pose = Vector3 = Header = SimulationExperimentContext = None  # type: ignore[assignment,misc]
    DurabilityPolicy = HistoryPolicy = QoSProfile = ReliabilityPolicy = None  # type: ignore[assignment,misc]

from .experiment_context import (
    CAMERA_ERROR_AXES,
    CAMERA_ERROR_FRAME,
    CONTEXT_SCHEMA,
    QUATERNION_ORDER,
    ROTATION_COMPOSITION,
    SOURCE_APPLICATION,
    WORLD_FRAME,
    build_context_payload,
    camera_pose_quaternion_xyzw,
    context_id_sha256,
)


def _parameter(node: Any, name: str) -> Any:
    return node.get_parameter(name).value


def _vec(values: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(f"{name} must be a three-element parameter array")
    parsed = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{name} must be finite")
    return parsed  # type: ignore[return-value]


def _set_vector(message: Any, values: Any) -> None:
    message.x, message.y, message.z = values


def _set_pose(message: Any, position: Any, orientation: Any) -> None:
    message.position.x, message.position.y, message.position.z = position
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = orientation


def build_context_message(node: Any) -> tuple[Any, dict[str, Any]]:
    """Resolve publisher parameters, recompute effective RPY, and build a message."""

    left_position = _vec(_parameter(node, "nominal_left_position_world_enu_m"), "nominal_left_position_world_enu_m")
    right_position = _vec(_parameter(node, "nominal_right_position_world_enu_m"), "nominal_right_position_world_enu_m")
    left_orientation = tuple(float(value) for value in _parameter(node, "nominal_left_orientation_xyzw"))
    right_orientation = tuple(float(value) for value in _parameter(node, "nominal_right_orientation_xyzw"))
    left_direct = _vec(_parameter(node, "left_direct_rpy_deg"), "left_direct_rpy_deg")
    right_direct = _vec(_parameter(node, "right_direct_rpy_deg"), "right_direct_rpy_deg")
    common = _vec(_parameter(node, "common_rpy_deg"), "common_rpy_deg")
    relative = _vec(_parameter(node, "relative_rpy_deg"), "relative_rpy_deg")
    zoom_valid = bool(_parameter(node, "zoom_index_valid"))
    zoom_index = float(_parameter(node, "zoom_index")) if zoom_valid else 0.0

    payload = build_context_payload(
        source_application=str(_parameter(node, "source_application")),
        run_id=str(_parameter(node, "run_id")),
        case_id=str(_parameter(node, "case_id")),
        scenario_name=str(_parameter(node, "scenario_name")),
        simulation_epoch=int(_parameter(node, "simulation_epoch")),
        simulation_epoch_valid=bool(_parameter(node, "simulation_epoch_valid")),
        simulation_epoch_source=str(_parameter(node, "simulation_epoch_source")),
        repository=str(_parameter(node, "repository")),
        branch=str(_parameter(node, "branch")),
        commit_sha=str(_parameter(node, "commit_sha")),
        git_status=str(_parameter(node, "git_status")),
        generated_sdf_sha256=str(_parameter(node, "generated_sdf_sha256")),
        nominal_baseline_m=float(_parameter(node, "nominal_baseline_m")),
        zoom_index=zoom_index,
        zoom_index_valid=zoom_valid,
        zoom_index_source=str(_parameter(node, "zoom_index_source")),
        camera_image_width_px=int(_parameter(node, "camera_image_width_px")),
        camera_image_height_px=int(_parameter(node, "camera_image_height_px")),
        camera_update_rate_hz=float(_parameter(node, "camera_update_rate_hz")),
        camera_horizontal_fov_rad=float(_parameter(node, "camera_horizontal_fov_rad")),
        observation_mode=str(_parameter(node, "observation_mode")),
        nominal_left_position_world_enu_m=left_position,
        nominal_left_orientation_xyzw=left_orientation,
        nominal_right_position_world_enu_m=right_position,
        nominal_right_orientation_xyzw=right_orientation,
        left_direct_rpy_deg=left_direct,
        right_direct_rpy_deg=right_direct,
        common_rpy_deg=common,
        relative_rpy_deg=relative,
    )
    if payload["source_application"] != SOURCE_APPLICATION:
        raise ValueError("source_application must be Simulation")
    if payload["world_frame"] != WORLD_FRAME:
        raise ValueError("world frame contract is fixed to gazebo_world_enu")
    if not payload["generated_sdf_sha256"]:
        raise ValueError("generated_sdf_sha256 is required for a valid context")
    context_hash = context_id_sha256(payload)

    message = SimulationExperimentContext()
    message.schema = CONTEXT_SCHEMA
    message.valid = True
    message.status = "VALID"
    message.source_application = payload["source_application"]
    message.context_id_sha256 = context_hash
    message.run_id = payload["run_id"]
    message.case_id = payload["case_id"]
    message.scenario_name = payload["scenario_name"]
    message.simulation_epoch = payload["simulation_epoch"]
    message.simulation_epoch_valid = payload["simulation_epoch_valid"]
    message.simulation_epoch_source = payload["simulation_epoch_source"]
    message.repository = payload["repository"]
    message.branch = payload["branch"]
    message.commit_sha = payload["commit_sha"]
    message.git_status = payload["git_status"]
    message.generated_sdf_sha256 = payload["generated_sdf_sha256"]
    message.world_frame = payload["world_frame"]
    message.camera_error_frame = payload["camera_error_frame"]
    message.camera_error_axes = payload["camera_error_axes"]
    message.rotation_composition = payload["rotation_composition"]
    message.quaternion_order = payload["quaternion_order"]
    message.nominal_baseline_m = payload["nominal_baseline_m"]
    message.zoom_index = payload["zoom_index"]
    message.zoom_index_valid = payload["zoom_index_valid"]
    message.zoom_index_source = payload["zoom_index_source"]
    message.camera_image_width_px = payload["camera_image_width_px"]
    message.camera_image_height_px = payload["camera_image_height_px"]
    message.camera_update_rate_hz = payload["camera_update_rate_hz"]
    message.camera_horizontal_fov_rad = payload["camera_horizontal_fov_rad"]
    message.observation_mode = payload["observation_mode"]
    _set_pose(
        message.nominal_left_optical_pose_world_enu,
        payload["nominal_left_optical_pose_world_enu"]["position_world_enu_m"],
        payload["nominal_left_optical_pose_world_enu"]["orientation_xyzw"],
    )
    _set_pose(
        message.nominal_right_optical_pose_world_enu,
        payload["nominal_right_optical_pose_world_enu"]["position_world_enu_m"],
        payload["nominal_right_optical_pose_world_enu"]["orientation_xyzw"],
    )
    for field in ("left_direct_rpy_deg", "right_direct_rpy_deg", "common_rpy_deg", "relative_rpy_deg", "left_effective_rpy_deg", "right_effective_rpy_deg"):
        _set_vector(getattr(message, field), payload[field])
    message.header.frame_id = WORLD_FRAME
    return message, payload


class ExperimentContextPublisher(Node):
    """One-shot transient-local publisher for an immutable Simulation run."""

    def __init__(self) -> None:  # pragma: no cover - ROS wiring is smoke-tested
        if rclpy is None or SimulationExperimentContext is None:
            raise RuntimeError("simulation_interfaces and ROS 2 are required")
        super().__init__("simulation_experiment_context")
        self._declare_parameters()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            SimulationExperimentContext,
            str(_parameter(self, "context_topic")),
            qos,
        )
        message, payload = build_context_message(self)
        message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)
        self._context_hash = context_id_sha256(payload)
        self.get_logger().info(
            "published immutable SimulationExperimentContext "
            f"schema={message.schema} hash={self._context_hash}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "context_topic": "/simulation/experiment_context",
            "source_application": SOURCE_APPLICATION,
            "run_id": "",
            "case_id": "",
            "scenario_name": "unspecified",
            "simulation_epoch": 0,
            "simulation_epoch_valid": False,
            "simulation_epoch_source": "not_published_by_simulation_launch",
            "repository": "",
            "branch": "",
            "commit_sha": "",
            "git_status": "",
            "generated_sdf_sha256": "",
            "nominal_baseline_m": 25.0,
            "zoom_index": 0.0,
            "zoom_index_valid": False,
            "zoom_index_source": "unresolved_launch_default",
            "camera_image_width_px": 1280,
            "camera_image_height_px": 720,
            "camera_update_rate_hz": 20.0,
            "camera_horizontal_fov_rad": 0.154997710698,
            "observation_mode": "unspecified",
            "nominal_left_position_world_enu_m": [-12.5, 0.0, 5.0],
            "nominal_left_orientation_xyzw": [-0.7071067811865475, 0.0, 0.0, 0.7071067811865476],
            "nominal_right_position_world_enu_m": [12.5, 0.0, 5.0],
            "nominal_right_orientation_xyzw": [-0.7071067811865475, 0.0, 0.0, 0.7071067811865476],
            "left_direct_rpy_deg": [0.0, 0.0, 0.0],
            "right_direct_rpy_deg": [0.0, 0.0, 0.0],
            "common_rpy_deg": [0.0, 0.0, 0.0],
            "relative_rpy_deg": [0.0, 0.0, 0.0],
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)


def main(args: list[str] | None = None) -> int:
    if rclpy is None:
        raise RuntimeError("ROS 2 is required to run the context publisher")
    rclpy.init(args=args)
    node = None
    try:
        node = ExperimentContextPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


__all__ = ["ExperimentContextPublisher", "build_context_message", "main"]
