"""Pure Marker specification factory for PoseSimulation RViz diagnostics.

The factory consumes an already validated UI snapshot.  It creates display
geometry only; it does not calculate or modify any PoseSimulation result.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Mapping

from .ui_snapshot import PoseRecord, PoseSimulationSnapshot, WORLD_FRAME


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Color = tuple[float, float, float, float]


@dataclass(frozen=True)
class MarkerSpec:
    namespace: str
    marker_id: int
    marker_type: str
    action: str = "ADD"
    frame_id: str = WORLD_FRAME
    position: Vector3 = (0.0, 0.0, 0.0)
    orientation_xyzw: Quaternion = (0.0, 0.0, 0.0, 1.0)
    points: tuple[Vector3, ...] = ()
    scale: Vector3 = (1.0, 1.0, 1.0)
    color: Color = (1.0, 1.0, 1.0, 1.0)
    text: str = ""
    mesh_resource: str = ""
    mesh_use_embedded_materials: bool = False
    lifetime_s: float = 0.0
    stamp_ns: int = 0


COLORS: Mapping[str, Color] = {
    "gt": (0.15, 0.75, 1.0, 0.95),
    "rtk": (1.0, 0.65, 0.12, 0.95),
    "actual_left": (0.95, 0.18, 0.18, 0.95),
    "actual_right": (0.2, 0.95, 0.35, 0.95),
    "ideal_gt": (0.2, 0.55, 1.0, 0.85),
    "ideal_rtk": (1.0, 0.55, 0.1, 0.85),
    "baseline_actual": (1.0, 0.2, 0.75, 0.95),
    "baseline_gt": (0.15, 0.8, 1.0, 0.9),
    "baseline_rtk": (1.0, 0.65, 0.1, 0.9),
    "error": (1.0, 0.1, 0.1, 0.95),
    "text": (1.0, 1.0, 1.0, 1.0),
}


def _finite_vector(value: Any) -> Vector3 | None:
    try:
        parsed = tuple(float(component) for component in value)
    except (TypeError, ValueError):
        return None
    if len(parsed) != 3 or not all(math.isfinite(component) for component in parsed):
        return None
    return parsed  # type: ignore[return-value]


def _add(a: Vector3, b: Vector3) -> Vector3:
    return tuple(a[index] + b[index] for index in range(3))  # type: ignore[return-value]


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return tuple(a[index] - b[index] for index in range(3))  # type: ignore[return-value]


def _scale(value: Vector3, factor: float) -> Vector3:
    return tuple(component * factor for component in value)  # type: ignore[return-value]


def _norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))


def _unit(value: Vector3) -> Vector3 | None:
    length = _norm(value)
    if length <= 1.0e-12 or not math.isfinite(length):
        return None
    return _scale(value, 1.0 / length)


def _quat_rotate(quaternion: Quaternion, vector: Vector3) -> Vector3:
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


def _local_axis(frame_kind: str, axis: str) -> Vector3:
    if frame_kind == "rig":
        return {
            "forward": (1.0, 0.0, 0.0),
            "right": (0.0, 1.0, 0.0),
            "down": (0.0, 0.0, 1.0),
        }[axis]
    return {
        "forward": (0.0, 0.0, 1.0),
        "right": (1.0, 0.0, 0.0),
        "down": (0.0, 1.0, 0.0),
    }[axis]


def _forward(pose: PoseRecord, frame_kind: str = "optical") -> Vector3:
    return _quat_rotate(pose.orientation_xyzw, _local_axis(frame_kind, "forward"))


def _right(pose: PoseRecord, frame_kind: str = "optical") -> Vector3:
    return _quat_rotate(pose.orientation_xyzw, _local_axis(frame_kind, "right"))


def _down(pose: PoseRecord, frame_kind: str = "optical") -> Vector3:
    return _quat_rotate(pose.orientation_xyzw, _local_axis(frame_kind, "down"))


def _arrow(
    namespace: str,
    marker_id: int,
    start: Vector3,
    end: Vector3,
    color: Color,
    *,
    shaft_m: float = 0.08,
    head_m: float = 0.22,
) -> MarkerSpec:
    return MarkerSpec(
        namespace=namespace,
        marker_id=marker_id,
        marker_type="arrow",
        points=(start, end),
        scale=(shaft_m, head_m, head_m),
        color=color,
    )


def _line(namespace: str, marker_id: int, points: Iterable[Vector3], color: Color, *, width_m: float = 0.08) -> MarkerSpec:
    return MarkerSpec(
        namespace=namespace,
        marker_id=marker_id,
        marker_type="line_strip",
        points=tuple(points),
        scale=(width_m, 0.0, 0.0),
        color=color,
    )


def _sphere(namespace: str, marker_id: int, position: Vector3, color: Color, *, size_m: float = 0.7) -> MarkerSpec:
    return MarkerSpec(
        namespace=namespace,
        marker_id=marker_id,
        marker_type="sphere",
        position=position,
        scale=(size_m, size_m, size_m),
        color=color,
    )


def _text(namespace: str, marker_id: int, position: Vector3, value: str, color: Color = COLORS["text"]) -> MarkerSpec:
    return MarkerSpec(
        namespace=namespace,
        marker_id=marker_id,
        marker_type="text",
        position=position,
        scale=(0.0, 0.0, 0.8),
        color=color,
        text=value,
    )


def _frustum(
    namespace: str,
    marker_id: int,
    pose: PoseRecord,
    color: Color,
    *,
    length_m: float,
    width_m: float,
    height_m: float,
) -> MarkerSpec:
    """Return a camera-center-to-far-plane wireframe in optical RDF axes."""

    center = pose.position_world_enu_m
    forward = _forward(pose)
    right = _right(pose)
    down = _down(pose)
    far_center = _add(center, _scale(forward, length_m))
    half_right = _scale(right, width_m * 0.5)
    half_down = _scale(down, height_m * 0.5)
    corners = (
        _add(_sub(far_center, half_right), _scale(half_down, -1.0)),
        _add(_add(far_center, half_right), _scale(half_down, -1.0)),
        _add(_add(far_center, half_right), half_down),
        _add(_sub(far_center, half_right), half_down),
    )
    points: list[Vector3] = []
    for corner in corners:
        points.extend((center, corner))
    for first, second in zip(corners, corners[1:] + corners[:1]):
        points.extend((first, second))
    return MarkerSpec(
        namespace=namespace,
        marker_id=marker_id,
        marker_type="line_list",
        points=tuple(points),
        scale=(0.045, 0.0, 0.0),
        color=color,
    )


def _ray_end(origin: Vector3, target: Vector3 | None, *, mode: str, normalized_length_m: float) -> Vector3 | None:
    if target is None:
        return None
    direction = _unit(_sub(target, origin))
    if direction is None:
        return None
    if mode == "full_distance":
        return target
    return _add(origin, _scale(direction, normalized_length_m))


def _pose_markers(
    namespace: str,
    label: str,
    pose: PoseRecord,
    color: Color,
    *,
    show_detail_axes: bool,
    marker_id: int,
    axis_length_m: float,
    frame_kind: str = "optical",
) -> list[MarkerSpec]:
    center = pose.position_world_enu_m
    result = [_sphere(namespace, marker_id, center, color, size_m=0.45)]
    result.append(_arrow(namespace, marker_id + 1, center, _add(center, _scale(_forward(pose, frame_kind), axis_length_m)), color))
    result.append(_text("pose_sim/text", marker_id + 1000, _add(center, (0.0, 0.0, 1.0)), label, color))
    if frame_kind == "optical":
        result.append(
            _frustum(
                namespace,
                marker_id + 4,
                pose,
                color,
                length_m=max(1.0, axis_length_m * 0.55),
                width_m=max(0.8, axis_length_m * 0.18),
                height_m=max(0.6, axis_length_m * 0.14),
            )
        )
    if show_detail_axes:
        result.append(_arrow(namespace, marker_id + 2, center, _add(center, _scale(_right(pose, frame_kind), axis_length_m * 0.55)), (0.95, 0.8, 0.1, 0.9), shaft_m=0.05))
        result.append(_arrow(namespace, marker_id + 3, center, _add(center, _scale(_down(pose, frame_kind), axis_length_m * 0.55)), (0.7, 0.7, 0.95, 0.9), shaft_m=0.05))
    return result


def _error_text(snapshot: PoseSimulationSnapshot, position: Vector3) -> str:
    left = snapshot.pose_error.get("left_actual_vs_gt_ideal", {})
    right = snapshot.pose_error.get("right_actual_vs_gt_ideal", {})
    relative = snapshot.stereo_geometry
    def fmt(mapping: Any, key: str) -> str:
        if not isinstance(mapping, Mapping):
            return "—"
        value = mapping.get(key)
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return "—"
        return "—" if not math.isfinite(parsed) else f"{parsed:.3f}"
    return (
        f"eval {snapshot.evaluation_id or '—'}\n"
        f"L GT rot {fmt(left, 'rotation_error_deg')}° / LOS {fmt(left, 'optical_axis_to_gt_target_error_deg')}°\n"
        f"R GT rot {fmt(right, 'rotation_error_deg')}° / LOS {fmt(right, 'optical_axis_to_gt_target_error_deg')}°\n"
        f"Rel yaw {fmt(relative, 'relative_yaw_error_deg')}°"
    )


def build_marker_specs(
    snapshot: PoseSimulationSnapshot,
    *,
    ray_mode: str = "normalized",
    normalized_ray_length_m: float = 50.0,
    show_detail_axes: bool = False,
    axis_length_m: float = 12.0,
) -> tuple[MarkerSpec, ...]:
    """Create one atomic MarkerArray specification for ``snapshot``."""

    specs: list[MarkerSpec] = [MarkerSpec("pose_sim", 0, "none", action="DELETEALL")]
    if snapshot.pair_state != "ALIGNED" or snapshot.state.marker_alignment != "ALIGNED":
        specs.append(_text("pose_sim/text", 9999, (0.0, 0.0, 1.0), snapshot.reason or snapshot.state.marker_alignment, COLORS["error"]))
        return tuple(specs)

    gt_target = snapshot.target_gt_world_enu_m
    rtk_target = snapshot.target_rtk_world_enu_m
    if gt_target is not None:
        specs.append(_sphere("pose_sim/target/gt", 0, gt_target, COLORS["gt"], size_m=1.0))
        gt_source = str(snapshot.status.get("ground_truth_source", "") or "")
        target_name = str(snapshot.status.get("target_entity_name", "") or "")
        if gt_source == "TARGET_REFERENCE_LINK":
            gt_label = "GT target_reference_link"
        else:
            gt_label = f"GT fallback: {target_name or gt_source or 'unknown'}"
        specs.append(_text("pose_sim/target/gt", 1, _add(gt_target, (0.0, 0.0, 1.5)), gt_label, COLORS["gt"]))
        if bool(snapshot.status.get("fallback_used", False)):
            specs.append(_text("pose_sim/target/gt", 2, _add(gt_target, (0.0, 0.0, 2.4)), "WARNING — FALLBACK GROUND-TRUTH SOURCE", COLORS["error"]))
    if rtk_target is not None:
        specs.append(_sphere("pose_sim/target/rtk", 0, rtk_target, COLORS["rtk"], size_m=0.85))
        specs.append(_text("pose_sim/target/rtk", 1, _add(rtk_target, (0.0, 0.0, 1.5)), "RTK operational reference", COLORS["rtk"]))
    if gt_target is not None and rtk_target is not None:
        specs.append(_arrow("pose_sim/target/rtk_gt_error", 0, gt_target, rtk_target, COLORS["error"], shaft_m=0.12, head_m=0.3))
        error_m = _norm(_sub(rtk_target, gt_target))
        specs.append(_text("pose_sim/target/rtk_gt_error", 1, _add(gt_target, (0.0, 0.0, 2.2)), f"RTK−GT 3D error: {error_m:.3f} m", COLORS["error"]))

    actual_left = snapshot.poses.get("actual_left")
    actual_right = snapshot.poses.get("actual_right")
    gt_left = snapshot.poses.get("ideal_gt_left")
    gt_right = snapshot.poses.get("ideal_gt_right")
    rtk_left = snapshot.poses.get("ideal_rtk_left")
    rtk_right = snapshot.poses.get("ideal_rtk_right")

    for key, namespace, label, color, marker_id in (
        ("actual_left", "pose_sim/actual/left", "Left Actual Optical", COLORS["actual_left"], 0),
        ("actual_right", "pose_sim/actual/right", "Right Actual Optical", COLORS["actual_right"], 10),
        ("ideal_gt_left", "pose_sim/ideal_gt/left", "Left GT Ideal Optical", COLORS["ideal_gt"], 20),
        ("ideal_gt_right", "pose_sim/ideal_gt/right", "Right GT Ideal Optical", COLORS["ideal_gt"], 30),
        ("ideal_gt_rig", "pose_sim/ideal_gt/rig", "GT Ideal Rig", COLORS["ideal_gt"], 35),
    ):
        pose = snapshot.poses.get(key)
        if pose is not None:
            specs.extend(_pose_markers(namespace, label, pose, color, show_detail_axes=show_detail_axes, marker_id=marker_id, axis_length_m=axis_length_m, frame_kind="rig" if key.endswith("_rig") else "optical"))
    for actual, ideal, namespace, marker_id in (
        (actual_left, gt_left, "pose_sim/error/left", 0),
        (actual_right, gt_right, "pose_sim/error/right", 1),
    ):
        if actual is not None and ideal is not None:
            actual_tip = _add(actual.position_world_enu_m, _scale(_forward(actual), axis_length_m))
            ideal_tip = _add(actual.position_world_enu_m, _scale(_forward(ideal), axis_length_m))
            specs.append(_line(namespace, marker_id, (actual_tip, ideal_tip), COLORS["error"], width_m=0.12))
    if rtk_target is not None:
        for key, namespace, label, marker_id in (
            ("ideal_rtk_left", "pose_sim/ideal_rtk/left", "Left RTK Ideal Optical", 40),
            ("ideal_rtk_right", "pose_sim/ideal_rtk/right", "Right RTK Ideal Optical", 50),
            ("ideal_rtk_rig", "pose_sim/ideal_rtk/rig", "RTK Ideal Rig", 55),
        ):
            pose = snapshot.poses.get(key)
            if pose is not None:
                specs.extend(_pose_markers(namespace, label, pose, COLORS["ideal_rtk"], show_detail_axes=show_detail_axes, marker_id=marker_id, axis_length_m=axis_length_m, frame_kind="rig" if key.endswith("_rig") else "optical"))

    if actual_left is not None and actual_right is not None:
        specs.append(_line("pose_sim/baseline/actual", 0, (actual_left.position_world_enu_m, actual_right.position_world_enu_m), COLORS["baseline_actual"], width_m=0.16))
        midpoint = _scale(_add(actual_left.position_world_enu_m, actual_right.position_world_enu_m), 0.5)
        specs.append(_sphere("pose_sim/actual/rig", 0, midpoint, COLORS["baseline_actual"], size_m=0.6))
        specs.append(_text("pose_sim/baseline/actual", 1, _add(midpoint, (0.0, 0.0, 1.0)), "Actual baseline", COLORS["baseline_actual"]))
    if gt_left is not None and gt_right is not None:
        specs.append(_line("pose_sim/baseline/ideal_gt", 0, (gt_left.position_world_enu_m, gt_right.position_world_enu_m), COLORS["baseline_gt"], width_m=0.12))
        midpoint = _scale(_add(gt_left.position_world_enu_m, gt_right.position_world_enu_m), 0.5)
        specs.append(_text("pose_sim/baseline/ideal_gt", 1, _add(midpoint, (0.0, 0.0, 1.3)), "GT ideal baseline", COLORS["baseline_gt"]))
    if rtk_left is not None and rtk_right is not None and rtk_target is not None:
        specs.append(_line("pose_sim/baseline/ideal_rtk", 0, (rtk_left.position_world_enu_m, rtk_right.position_world_enu_m), COLORS["baseline_rtk"], width_m=0.1))
        midpoint = _scale(_add(rtk_left.position_world_enu_m, rtk_right.position_world_enu_m), 0.5)
        specs.append(_text("pose_sim/baseline/ideal_rtk", 1, _add(midpoint, (0.0, 0.0, 1.6)), "RTK ideal baseline", COLORS["baseline_rtk"]))

    for pose, marker_id in ((actual_left, 0), (actual_right, 1)):
        if pose is not None:
            start = pose.position_world_enu_m
            end = _add(start, _scale(_forward(pose), axis_length_m))
            specs.append(_arrow("pose_sim/rays/actual", marker_id, start, end, COLORS["actual_left" if marker_id == 0 else "actual_right"], shaft_m=0.045, head_m=0.15))

    for pose, target, namespace, marker_id in (
        (actual_left, gt_target, "pose_sim/rays/gt_los", 0),
        (actual_right, gt_target, "pose_sim/rays/gt_los", 1),
    ):
        if pose is not None:
            end = _ray_end(pose.position_world_enu_m, target, mode=ray_mode, normalized_length_m=normalized_ray_length_m)
            if end is not None:
                specs.append(_arrow(namespace, marker_id, pose.position_world_enu_m, end, COLORS["gt"], shaft_m=0.04, head_m=0.14))
    if rtk_target is not None:
        for pose, marker_id in ((actual_left, 0), (actual_right, 1)):
            if pose is not None:
                end = _ray_end(pose.position_world_enu_m, rtk_target, mode=ray_mode, normalized_length_m=normalized_ray_length_m)
                if end is not None:
                    specs.append(_arrow("pose_sim/rays/rtk_los", marker_id, pose.position_world_enu_m, end, COLORS["rtk"], shaft_m=0.04, head_m=0.14))
    anchor = actual_left.position_world_enu_m if actual_left is not None else (0.0, 0.0, 0.0)
    relative_yaw = snapshot.stereo_geometry.get("relative_yaw_error_deg")
    try:
        relative_text = f"Relative yaw error: {float(relative_yaw):.3f}°"
    except (TypeError, ValueError):
        relative_text = "Relative yaw error: UNAVAILABLE"
    specs.append(_text("pose_sim/error/relative", 0, _add(anchor, (0.0, 0.0, 3.8)), relative_text, COLORS["error"]))
    specs.append(_text("pose_sim/text", 0, _add(anchor, (0.0, 0.0, 2.8)), _error_text(snapshot, anchor), COLORS["text"]))
    return tuple(specs)


__all__ = ["COLORS", "MarkerSpec", "build_marker_specs"]
