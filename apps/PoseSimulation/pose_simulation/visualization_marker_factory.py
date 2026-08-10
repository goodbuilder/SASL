"""Pure MarkerSpec factory for the typed PoseSimulation RViz view.

The factory receives one validated :class:`VisualizationSnapshot`.  It only
turns world-frame poses and already-computed M3 metrics into display geometry;
it never recomputes attitude, LOS, baseline, or time alignment values.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .marker_factory import COLORS, MarkerSpec
from .visualization_snapshot import RIG_FRAME, VisualizationSnapshot, WorldPose


Vector3 = tuple[float, float, float]
WORLD_FRAME = "gazebo_world_enu"
CAMERA_MESH_RESOURCE = "package://pose_simulation/models/wiris_camera/meshes/WIRIS.stl"
DRONE_MESH_RESOURCE = "package://pose_simulation/models/x500_15m/meshes/x500_15m.stl"
DRONE_MESH_ORIENTATION_XYZW = (0.0, 0.0, 1.0, 0.0)

AXIS_X = (0.92, 0.20, 0.20, 0.95)
AXIS_Y = (0.20, 0.90, 0.25, 0.95)
AXIS_Z = (0.20, 0.45, 0.98, 0.95)


@dataclass(frozen=True)
class VisualizationConfig:
    """Finite visual parameters; all lengths are metres and angles are deg."""

    ray_mode: str = "normalized"
    normalized_ray_length_m: float = 50.0
    camera_axis_length_m: float = 5.0
    rig_axis_length_m: float = 10.0
    target_marker_scale_m: float = 12.0
    optical_center_scale_m: float = 3.0
    frustum_length_m: float = 3.0
    frustum_width_m: float = 1.0
    frustum_height_m: float = 0.7
    marker_visual_scale: float = 1.0
    ray_shaft_width_m: float = 2.0
    ray_head_width_m: float = 2.0
    ray_line_width_m: float = 2.0
    show_meshes: bool = True
    camera_mesh_resource: str = CAMERA_MESH_RESOURCE
    camera_mesh_scale_m: float = 0.001
    drone_mesh_resource: str = DRONE_MESH_RESOURCE
    drone_mesh_scale_m: float = 22.0
    show_actual: bool = True
    show_gt_ideal: bool = True
    show_rtk_ideal: bool = False
    show_actual_forward: bool = True
    show_gt_los: bool = True
    show_rtk_los: bool = False
    show_error_arcs: bool = True
    show_baseline: bool = True
    show_text: bool = True
    show_detailed_optical_axes: bool = False
    # Snapshots are deduplicated by the node, so markers must persist until the
    # next snapshot or an explicit DELETE is published.  Zero means forever in
    # visualization_msgs/Marker.
    marker_lifetime_s: float = 0.0
    angular_visual_scale: float = 1.0
    position_error_visual_scale: float = 1.0

    def validate(self) -> "VisualizationConfig":
        if self.ray_mode not in {"normalized", "full_distance"}:
            raise ValueError("ray_mode must be normalized or full_distance")
        positive = (
            "normalized_ray_length_m",
            "camera_axis_length_m",
            "rig_axis_length_m",
            "target_marker_scale_m",
            "optical_center_scale_m",
            "frustum_length_m",
            "frustum_width_m",
            "frustum_height_m",
            "marker_visual_scale",
            "ray_shaft_width_m",
            "ray_head_width_m",
            "ray_line_width_m",
            "camera_mesh_scale_m",
            "drone_mesh_scale_m",
            "angular_visual_scale",
            "position_error_visual_scale",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        lifetime = float(self.marker_lifetime_s)
        if not math.isfinite(lifetime) or lifetime < 0.0:
            raise ValueError("marker_lifetime_s must be finite and non-negative")
        return self


def _add(a: Vector3, b: Vector3) -> Vector3:
    return tuple(a[index] + b[index] for index in range(3))  # type: ignore[return-value]


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return tuple(a[index] - b[index] for index in range(3))  # type: ignore[return-value]


def _scale(a: Vector3, factor: float) -> Vector3:
    return tuple(value * factor for value in a)  # type: ignore[return-value]


def _norm(a: Vector3) -> float:
    return math.sqrt(sum(value * value for value in a))


def _unit(a: Vector3) -> Vector3 | None:
    length = _norm(a)
    if not math.isfinite(length) or length <= 1.0e-12:
        return None
    return _scale(a, 1.0 / length)


def _quat_rotate(quaternion: tuple[float, float, float, float], vector: Vector3) -> Vector3:
    x, y, z, w = quaternion
    qv = (x, y, z)
    uv = (
        qv[1] * vector[2] - qv[2] * vector[1],
        qv[2] * vector[0] - qv[0] * vector[2],
        qv[0] * vector[1] - qv[1] * vector[0],
    )
    uuv = (
        qv[1] * uv[2] - qv[2] * uv[1],
        qv[2] * uv[0] - qv[0] * uv[2],
        qv[0] * uv[1] - qv[1] * uv[0],
    )
    return _add(vector, _scale(_add(_scale(uv, w), uuv), 2.0))


def _quat_multiply(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _axis(pose: WorldPose, name: str) -> Vector3:
    if pose.local_frame == RIG_FRAME:
        local = {"forward": (1.0, 0.0, 0.0), "right": (0.0, 1.0, 0.0), "down": (0.0, 0.0, 1.0)}[name]
    else:
        # Camera Optical RDF: X Right, Y Down, Z Forward.
        local = {"forward": (0.0, 0.0, 1.0), "right": (1.0, 0.0, 0.0), "down": (0.0, 1.0, 0.0)}[name]
    return _quat_rotate(pose.orientation_xyzw, local)


def _spec(
    namespace: str,
    marker_id: int,
    marker_type: str,
    *,
    snapshot: VisualizationSnapshot,
    position: Vector3 = (0.0, 0.0, 0.0),
    points: Iterable[Vector3] = (),
    scale: Vector3 = (1.0, 1.0, 1.0),
    color: tuple[float, float, float, float] = COLORS["text"],
    text: str = "",
    action: str = "ADD",
    lifetime_s: float = 0.5,
) -> MarkerSpec:
    return MarkerSpec(
        namespace=namespace,
        marker_id=marker_id,
        marker_type=marker_type,
        action=action,
        frame_id=WORLD_FRAME,
        position=position,
        points=tuple(points),
        scale=scale,
        color=color,
        text=text,
        lifetime_s=lifetime_s,
        stamp_ns=snapshot.evaluation_stamp_ns,
    )


def _sphere(namespace: str, marker_id: int, position: Vector3, size: float, color, snapshot: VisualizationSnapshot, config: VisualizationConfig) -> MarkerSpec:
    size *= config.marker_visual_scale
    return _spec(namespace, marker_id, "sphere", snapshot=snapshot, position=position, scale=(size, size, size), color=color, lifetime_s=config.marker_lifetime_s)


def _arrow(namespace: str, marker_id: int, start: Vector3, end: Vector3, color, snapshot: VisualizationSnapshot, config: VisualizationConfig, *, shaft: float = 0.08, head: float = 0.24) -> MarkerSpec:
    return _spec(namespace, marker_id, "arrow", snapshot=snapshot, points=(start, end), scale=(config.ray_shaft_width_m, config.ray_head_width_m, config.ray_head_width_m), color=color, lifetime_s=config.marker_lifetime_s)


def _line(namespace: str, marker_id: int, points: Iterable[Vector3], color, snapshot: VisualizationSnapshot, config: VisualizationConfig, *, width: float = 0.08) -> MarkerSpec:
    return _spec(namespace, marker_id, "line_strip", snapshot=snapshot, points=points, scale=(config.ray_line_width_m, 0.0, 0.0), color=color, lifetime_s=config.marker_lifetime_s)


def _text(namespace: str, marker_id: int, position: Vector3, text: str, color, snapshot: VisualizationSnapshot, config: VisualizationConfig) -> MarkerSpec:
    return _spec(namespace, marker_id, "text", snapshot=snapshot, position=position, scale=(0.0, 0.0, 0.85), color=color, text=text, lifetime_s=config.marker_lifetime_s)


def _mesh(
    namespace: str,
    marker_id: int,
    pose: WorldPose,
    resource: str,
    scale_m: float,
    color,
    snapshot: VisualizationSnapshot,
    config: VisualizationConfig,
    *,
    orientation_correction_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> MarkerSpec:
    return MarkerSpec(
        namespace=namespace,
        marker_id=marker_id,
        marker_type="mesh",
        frame_id=WORLD_FRAME,
        position=pose.position_world_enu_m,
        orientation_xyzw=_quat_multiply(pose.orientation_xyzw, orientation_correction_xyzw),
        scale=(scale_m, scale_m, scale_m),
        color=color,
        mesh_resource=resource,
        lifetime_s=config.marker_lifetime_s,
        stamp_ns=snapshot.evaluation_stamp_ns,
    )


def _symbolic_frustum(pose: WorldPose, namespace: str, marker_id: int, color, snapshot: VisualizationSnapshot, config: VisualizationConfig) -> MarkerSpec:
    center = pose.position_world_enu_m
    forward = _axis(pose, "forward")
    right = _axis(pose, "right")
    down = _axis(pose, "down")
    far = _add(center, _scale(forward, config.frustum_length_m))
    half_right = _scale(right, config.frustum_width_m * 0.5)
    half_down = _scale(down, config.frustum_height_m * 0.5)
    corners = (
        _add(_sub(far, half_right), _scale(half_down, -1.0)),
        _add(_add(far, half_right), _scale(half_down, -1.0)),
        _add(_add(far, half_right), half_down),
        _add(_sub(far, half_right), half_down),
    )
    points: list[Vector3] = []
    for corner in corners:
        points.extend((center, corner))
    for first, second in zip(corners, corners[1:] + corners[:1]):
        points.extend((first, second))
    return _spec(namespace, marker_id, "line_list", snapshot=snapshot, points=points, scale=(config.ray_line_width_m, 0.0, 0.0), color=color, lifetime_s=config.marker_lifetime_s)


def _pose_markers(
    namespace: str,
    label: str,
    pose: WorldPose,
    color,
    snapshot: VisualizationSnapshot,
    config: VisualizationConfig,
    *,
    marker_id: int,
    axis_length_m: float,
    rig: bool = False,
    mesh_resource: str = "",
    mesh_scale_m: float = 1.0,
) -> list[MarkerSpec]:
    center = pose.position_world_enu_m
    result = [_sphere(namespace, marker_id, center, config.optical_center_scale_m if not rig else config.target_marker_scale_m * 0.45, color, snapshot, config)]
    if mesh_resource:
        result.append(_mesh(namespace, marker_id + 5, pose, mesh_resource, mesh_scale_m, color, snapshot, config))
    if config.show_actual_forward or rig:
        result.append(_arrow(namespace, marker_id + 1, center, _add(center, _scale(_axis(pose, "forward"), axis_length_m)), color, snapshot, config))
    detailed = rig or config.show_detailed_optical_axes
    if detailed:
        result.append(_arrow(namespace, marker_id + 2, center, _add(center, _scale(_axis(pose, "right"), axis_length_m * 0.55)), AXIS_Y, snapshot, config, shaft=0.05, head=0.16))
        result.append(_arrow(namespace, marker_id + 3, center, _add(center, _scale(_axis(pose, "down"), axis_length_m * 0.55)), AXIS_Z, snapshot, config, shaft=0.05, head=0.16))
    if not rig:
        result.append(_symbolic_frustum(pose, namespace, marker_id + 4, color, snapshot, config))
    if config.show_text:
        result.append(_text("pose_sim/text", marker_id + 1000, _add(center, (0.0, 0.0, 1.4)), label, color, snapshot, config))
    return result


def _ray_end(origin: Vector3, target: Vector3, config: VisualizationConfig) -> Vector3 | None:
    direction = _unit(_sub(target, origin))
    if direction is None:
        return None
    if config.ray_mode == "full_distance":
        return target
    return _add(origin, _scale(direction, config.normalized_ray_length_m))


def _delete_keys() -> tuple[tuple[str, int], ...]:
    namespaces = {
        "pose_sim/target/gt": (0, 1, 2),
        "pose_sim/target/rtk": (0, 1, 2),
        "pose_sim/target/rtk_gt_error": (0, 1),
        "pose_sim/actual/left": (0, 1, 2, 3, 4, 5),
        "pose_sim/actual/right": (10, 11, 12, 13, 14, 15),
        "pose_sim/actual/rig": (0, 1),
        "pose_sim/ideal_gt/left": (20, 21, 22, 23, 24),
        "pose_sim/ideal_gt/right": (30, 31, 32, 33, 34),
        "pose_sim/ideal_gt/rig": (35, 36, 37, 38),
        "pose_sim/ideal_rtk/left": (40, 41, 42, 43, 44),
        "pose_sim/ideal_rtk/right": (50, 51, 52, 53, 54),
        "pose_sim/ideal_rtk/rig": (55, 56, 57, 58),
        "pose_sim/rays/actual": (0, 1),
        "pose_sim/rays/gt_los": (0, 1),
        "pose_sim/rays/rtk_los": (0, 1),
        "pose_sim/error/left": (0, 1),
        "pose_sim/error/right": (0, 1),
        "pose_sim/error/relative": (0, 1),
        "pose_sim/baseline/actual": (0, 1, 2),
        "pose_sim/baseline/ideal_gt": (0, 1, 2),
        "pose_sim/baseline/ideal_rtk": (0, 1, 2),
        "pose_sim/status": (0,),
        "pose_sim/text": (1000, 1010, 1020, 1030, 1035, 1040, 1050, 1055),
    }
    return tuple((namespace, marker_id) for namespace, ids in namespaces.items() for marker_id in ids)


def build_delete_markers(snapshot: VisualizationSnapshot, reason: str = "") -> tuple[MarkerSpec, ...]:
    """Delete every deterministic object from the previous scene.

    Invalid-state details remain available through ``diagnostics_topic``.  Do
    not render them as a central red RViz text marker, which obscures the 3D
    scene while the input stream is settling.
    """

    specs = tuple(
        _spec(namespace, marker_id, "none", snapshot=snapshot, action="DELETE", lifetime_s=0.0)
        for namespace, marker_id in _delete_keys()
    )
    return specs


def _metric(value: float, unit: str) -> str:
    sign = "+" if value > 0.0 else ""
    return f"{sign}{value:.3f}{unit}"


def build_visualization_marker_specs(
    snapshot: VisualizationSnapshot,
    config: VisualizationConfig,
) -> tuple[MarkerSpec, ...]:
    """Build one atomic deterministic MarkerSpec set for one evaluation."""

    config = config.validate()
    if not snapshot.valid:
        return build_delete_markers(snapshot, snapshot.invalid_reason or snapshot.state)
    required = (snapshot.gt_target, snapshot.actual_left, snapshot.actual_right, snapshot.ideal_gt_rig, snapshot.ideal_gt_left, snapshot.ideal_gt_right, snapshot.left_error, snapshot.right_error, snapshot.stereo)
    if any(value is None for value in required):
        return build_delete_markers(snapshot, "Required visualization geometry is unavailable")

    assert snapshot.gt_target and snapshot.actual_left and snapshot.actual_right
    assert snapshot.ideal_gt_rig and snapshot.ideal_gt_left and snapshot.ideal_gt_right
    assert snapshot.left_error and snapshot.right_error and snapshot.stereo
    specs: list[MarkerSpec] = list(build_delete_markers(snapshot))

    if config.show_actual:
        specs += _pose_markers("pose_sim/actual/left", "Left Actual Optical", snapshot.actual_left, COLORS["actual_left"], snapshot, config, marker_id=0, axis_length_m=config.camera_axis_length_m, mesh_resource=config.camera_mesh_resource if config.show_meshes else "", mesh_scale_m=config.camera_mesh_scale_m)
        specs += _pose_markers("pose_sim/actual/right", "Right Actual Optical", snapshot.actual_right, COLORS["actual_right"], snapshot, config, marker_id=10, axis_length_m=config.camera_axis_length_m, mesh_resource=config.camera_mesh_resource if config.show_meshes else "", mesh_scale_m=config.camera_mesh_scale_m)
        specs.append(_sphere("pose_sim/actual/rig", 0, snapshot.stereo.actual_midpoint_world_enu_m, config.target_marker_scale_m * 0.35, COLORS["baseline_actual"], snapshot, config))
        if config.show_text:
            specs.append(_text("pose_sim/actual/rig", 1, _add(snapshot.stereo.actual_midpoint_world_enu_m, (0.0, 0.0, 1.0)), "Actual optical midpoint", COLORS["baseline_actual"], snapshot, config))

    if config.show_gt_ideal:
        specs.append(_sphere("pose_sim/target/gt", 0, snapshot.gt_target.position_world_enu_m, config.target_marker_scale_m, COLORS["gt"], snapshot, config))
        if config.show_meshes:
            specs.append(_mesh("pose_sim/target/gt", 2, snapshot.gt_target, config.drone_mesh_resource, config.drone_mesh_scale_m, COLORS["gt"], snapshot, config, orientation_correction_xyzw=DRONE_MESH_ORIENTATION_XYZW))
        if config.show_text:
            label = f"Gazebo GT — {snapshot.gt_reference_name or 'vehicle_reference_link'}\nEvaluation {snapshot.evaluation_id} · Sim time {snapshot.evaluation_stamp_ns / 1.0e9:.3f} s"
            specs.append(_text("pose_sim/target/gt", 1, _add(snapshot.gt_target.position_world_enu_m, (0.0, 0.0, config.target_marker_scale_m)), label, COLORS["gt"], snapshot, config))
        specs += _pose_markers("pose_sim/ideal_gt/left", "Left GT Ideal Optical", snapshot.ideal_gt_left, COLORS["ideal_gt"], snapshot, config, marker_id=20, axis_length_m=config.camera_axis_length_m)
        specs += _pose_markers("pose_sim/ideal_gt/right", "Right GT Ideal Optical", snapshot.ideal_gt_right, COLORS["ideal_gt"], snapshot, config, marker_id=30, axis_length_m=config.camera_axis_length_m)
        specs += _pose_markers("pose_sim/ideal_gt/rig", "GT Ideal Rig (FRD)", snapshot.ideal_gt_rig, COLORS["ideal_gt"], snapshot, config, marker_id=35, axis_length_m=config.rig_axis_length_m, rig=True)

    if config.show_rtk_ideal and snapshot.rtk_available and snapshot.rtk_target and snapshot.ideal_rtk_rig and snapshot.ideal_rtk_left and snapshot.ideal_rtk_right:
        specs.append(_sphere("pose_sim/target/rtk", 0, snapshot.rtk_target.position_world_enu_m, config.target_marker_scale_m * 0.85, COLORS["rtk"], snapshot, config))
        if config.show_text:
            specs.append(_text("pose_sim/target/rtk", 1, _add(snapshot.rtk_target.position_world_enu_m, (0.0, 0.0, config.target_marker_scale_m)), "RTK operational reference", COLORS["rtk"], snapshot, config))
        specs += _pose_markers("pose_sim/ideal_rtk/left", "Left RTK Ideal Optical", snapshot.ideal_rtk_left, COLORS["ideal_rtk"], snapshot, config, marker_id=40, axis_length_m=config.camera_axis_length_m)
        specs += _pose_markers("pose_sim/ideal_rtk/right", "Right RTK Ideal Optical", snapshot.ideal_rtk_right, COLORS["ideal_rtk"], snapshot, config, marker_id=50, axis_length_m=config.camera_axis_length_m)
        specs += _pose_markers("pose_sim/ideal_rtk/rig", "RTK Ideal Rig (FRD)", snapshot.ideal_rtk_rig, COLORS["ideal_rtk"], snapshot, config, marker_id=55, axis_length_m=config.rig_axis_length_m, rig=True)
        rtk_delta = _sub(snapshot.rtk_target.position_world_enu_m, snapshot.gt_target.position_world_enu_m)
        rtk_display_end = _add(
            snapshot.gt_target.position_world_enu_m,
            _scale(rtk_delta, config.position_error_visual_scale),
        )
        specs.append(_arrow("pose_sim/target/rtk_gt_error", 0, snapshot.gt_target.position_world_enu_m, rtk_display_end, COLORS["error"], snapshot, config, shaft=0.12, head=0.3))
        if config.show_text and config.position_error_visual_scale != 1.0:
            specs.append(_text("pose_sim/target/rtk_gt_error", 2, _add(rtk_display_end, (0.0, 0.0, 0.6)), f"RTK−GT position vector ×{config.position_error_visual_scale:g} visual scale", COLORS["error"], snapshot, config))

    if config.show_baseline:
        specs.append(_line("pose_sim/baseline/actual", 0, (snapshot.actual_left.position_world_enu_m, snapshot.actual_right.position_world_enu_m), COLORS["baseline_actual"], snapshot, config, width=0.16))
        specs.append(_line("pose_sim/baseline/ideal_gt", 0, (snapshot.ideal_gt_left.position_world_enu_m, snapshot.ideal_gt_right.position_world_enu_m), COLORS["baseline_gt"], snapshot, config, width=0.11))
        if config.show_text:
            midpoint = snapshot.stereo.actual_midpoint_world_enu_m
            baseline_text = (
                f"Actual baseline: {snapshot.stereo.actual_baseline_length_m:.3f} m\n"
                f"Nominal baseline: {snapshot.stereo.nominal_baseline_length_m:.3f} m\n"
                f"Direction error: {snapshot.stereo.baseline_direction_error_deg:.3f}°"
            )
            specs.append(_text("pose_sim/baseline/actual", 1, _add(midpoint, (0.0, 0.0, 2.0)), baseline_text, COLORS["baseline_actual"], snapshot, config))
        if config.show_rtk_ideal and snapshot.rtk_available and snapshot.ideal_rtk_left and snapshot.ideal_rtk_right:
            specs.append(_line("pose_sim/baseline/ideal_rtk", 0, (snapshot.ideal_rtk_left.position_world_enu_m, snapshot.ideal_rtk_right.position_world_enu_m), COLORS["baseline_rtk"], snapshot, config, width=0.09))

    if config.show_actual_forward:
        for marker_id, pose, color in ((0, snapshot.actual_left, COLORS["actual_left"]), (1, snapshot.actual_right, COLORS["actual_right"])):
            start = pose.position_world_enu_m
            specs.append(_arrow("pose_sim/rays/actual", marker_id, start, _add(start, _scale(_axis(pose, "forward"), config.camera_axis_length_m)), color, snapshot, config, shaft=0.05, head=0.16))

    if config.show_gt_los:
        for marker_id, pose in ((0, snapshot.actual_left), (1, snapshot.actual_right)):
            end = _ray_end(pose.position_world_enu_m, snapshot.gt_target.position_world_enu_m, config)
            if end is not None:
                specs.append(_arrow("pose_sim/rays/gt_los", marker_id, pose.position_world_enu_m, end, COLORS["gt"], snapshot, config, shaft=0.045, head=0.15))

    if config.show_rtk_los and snapshot.rtk_available and snapshot.rtk_target:
        for marker_id, pose in ((0, snapshot.actual_left), (1, snapshot.actual_right)):
            end = _ray_end(pose.position_world_enu_m, snapshot.rtk_target.position_world_enu_m, config)
            if end is not None:
                specs.append(_arrow("pose_sim/rays/rtk_los", marker_id, pose.position_world_enu_m, end, COLORS["rtk"], snapshot, config, shaft=0.045, head=0.15))

    if config.show_error_arcs:
        for marker_id, actual, ideal, error in ((0, snapshot.actual_left, snapshot.ideal_gt_left, snapshot.left_error), (1, snapshot.actual_right, snapshot.ideal_gt_right, snapshot.right_error)):
            error_display_length = config.camera_axis_length_m * config.angular_visual_scale
            actual_tip = _add(actual.position_world_enu_m, _scale(_axis(actual, "forward"), error_display_length))
            ideal_tip = _add(actual.position_world_enu_m, _scale(_axis(ideal, "forward"), error_display_length))
            specs.append(_line("pose_sim/error/left" if marker_id == 0 else "pose_sim/error/right", 0, (actual_tip, ideal_tip), COLORS["error"], snapshot, config, width=0.12))
            if config.show_text:
                scale_note = "" if config.angular_visual_scale == 1.0 else f"\nAngular arc ×{config.angular_visual_scale:g} visual scale"
                specs.append(_text("pose_sim/error/left" if marker_id == 0 else "pose_sim/error/right", 1, _add(actual.position_world_enu_m, (0.0, 0.0, 3.0)), f"{'L' if marker_id == 0 else 'R'} LOS error: {error.optical_axis_error_deg:.3f}°{scale_note}", COLORS["error"], snapshot, config))
        if config.show_text:
            anchor = snapshot.stereo.actual_midpoint_world_enu_m
            specs.append(_text("pose_sim/error/relative", 0, _add(anchor, (0.0, 0.0, 4.0)), f"Relative yaw: {_metric(snapshot.stereo.relative_yaw_error_deg, '°')}\nOptical-axis separation: {snapshot.stereo.optical_axis_angle_deg:.3f}°", COLORS["error"], snapshot, config))

    return tuple(specs)


__all__ = ["VisualizationConfig", "build_delete_markers", "build_visualization_marker_specs"]
