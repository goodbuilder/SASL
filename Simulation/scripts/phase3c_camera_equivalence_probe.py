#!/usr/bin/env python3
"""Phase 3C-A rendered camera equivalence probe.

This script measures whether the production Gazebo camera rendering semantics
match the Phase 3A analytic Actual camera model. It intentionally does not
change production launch, SDF, or attitude math contracts. A verdict of
``not_equivalent`` is a valid evidence result; ``inconclusive`` means the probe
did not collect enough trustworthy runtime evidence.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import signal
import subprocess
import sys
import time
import types
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # Pure unit tests may run without ROS sourced.
    get_package_share_directory = None

from stereo_ground_sim.attitude_error_model import (
    CameraAttitudeErrorDeg,
    StereoAttitudeErrorConfig,
    project_stereo_with_actual_transforms,
    stereo_actual_camera_transforms,
)
from stereo_ground_sim.coordinate_frames import (
    FrameId,
    FramePoint,
    inverse_rigid_transform,
    rotation_matrix_from_quaternion_wxyz,
    transform_point,
)
from stereo_ground_sim.gazebo_oracle import make_gazebo_world_to_baseline_transform
from stereo_ground_sim.stereo_triangulation_adapter import pinhole_intrinsics_from_horizontal_fov


BASE_COMMIT = "981ecba84d8f20493f9dc48d87adcb9f5a8e5ef9"
PRE_ADAPTER_EVIDENCE_REFERENCE = "/tmp/sasl-phase3c-a-standard-review-fix"
ROLL_OBSERVABILITY_MARKER = "right_positive"
ATTITUDE_SEMANTICS_CRITERIA = {
    "single_axis_required_total": 12,
    "single_axis_differential_median_px": 0.25,
    "single_axis_differential_max_px": 0.75,
    "composite_differential_median_px": 0.5,
    "composite_differential_max_px": 1.0,
    "centroid_stddev_max_px": 0.1,
}
WORLD_NAME = "stereo_ground_drone"
MARKER_MODEL_NAME = "phase3c_equivalence_marker"
WIDTH_PX = 1280
HEIGHT_PX = 720
BASELINE_M = 25.0
CAMERA_HEIGHT_M = 5.0
HORIZONTAL_FOV_RAD = 0.154997710698
CAMERA_CLIP_NEAR_M = 0.1
CAMERA_CLIP_FAR_M = 2500.0
R_B_W = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)
O_W_B = (0.0, 0.0, 5.0)
R_M_C_ZERO_CANDIDATE = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
)
REQUIRED_POSE_CHECKS_PER_MARKER_SAMPLE = 11

MARKER_POINTS_BASELINE_FRD_M = {
    "center": (1852.0, 20.0, -6.0),
    "right_positive": (1852.0, 60.0, -6.0),
    "right_negative": (1852.0, -20.0, -6.0),
    "down_positive": (1852.0, 20.0, 2.0),
    "down_negative": (1852.0, 20.0, -14.0),
    "nearer": (1200.0, 20.0, -6.0),
    "farther": (2300.0, 20.0, -6.0),
}

EXPECTED_MARKER_POINTS_WORLD_ENU_M = {
    "center": (20.0, 1852.0, 11.0),
    "right_positive": (60.0, 1852.0, 11.0),
    "right_negative": (-20.0, 1852.0, 11.0),
    "down_positive": (20.0, 1852.0, 3.0),
    "down_negative": (20.0, 1852.0, 19.0),
    "nearer": (20.0, 1200.0, 11.0),
    "farther": (20.0, 2300.0, 11.0),
}


@dataclass(frozen=True)
class AttitudeCase:
    name: str
    left: CameraAttitudeErrorDeg
    right: CameraAttitudeErrorDeg
    marker_names: tuple[str, ...]


@dataclass(frozen=True)
class MarkerDetection:
    valid: bool
    status: str
    u_px: float | None
    v_px: float | None
    pixel_count: int
    component_count: int
    bbox: tuple[int, int, int, int] | None
    centroid_method: str


@dataclass(frozen=True)
class ProcessHandle:
    name: str
    process: subprocess.Popen
    log_handle: Any


def world_to_baseline_transform():
    return make_gazebo_world_to_baseline_transform(
        origin_world_enu_m=O_W_B,
        rotation_baseline_world=R_B_W,
    )


def baseline_to_world_transform():
    return inverse_rigid_transform(world_to_baseline_transform())


def marker_world_point(marker_name: str) -> tuple[float, float, float]:
    point = FramePoint(MARKER_POINTS_BASELINE_FRD_M[marker_name], FrameId.BASELINE_CENTER_FRD)
    world = transform_point(point, baseline_to_world_transform())
    return tuple(float(value) for value in world.xyz_m)


def marker_baseline_point_from_world(world_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    point = FramePoint(world_xyz, FrameId.GAZEBO_WORLD_ENU)
    baseline = transform_point(point, world_to_baseline_transform())
    return tuple(float(value) for value in baseline.xyz_m)


def attitude_config(case: AttitudeCase) -> StereoAttitudeErrorConfig:
    return StereoAttitudeErrorConfig(left=case.left, right=case.right)


def intrinsics():
    return pinhole_intrinsics_from_horizontal_fov(
        width_px=WIDTH_PX,
        height_px=HEIGHT_PX,
        horizontal_fov_rad=HORIZONTAL_FOV_RAD,
    )


def analytic_projection(marker_name: str, case: AttitudeCase) -> dict[str, Any]:
    point = FramePoint(MARKER_POINTS_BASELINE_FRD_M[marker_name], FrameId.BASELINE_CENTER_FRD)
    projection = project_stereo_with_actual_transforms(
        point,
        stereo_actual_camera_transforms(BASELINE_M, attitude_config(case)),
        intrinsics(),
        clip_near_m=CAMERA_CLIP_NEAR_M,
        clip_far_m=CAMERA_CLIP_FAR_M,
    )
    return {
        "valid": projection.valid,
        "status": projection.status.value,
        "left": {
            "status": projection.left.status.value,
            "u_px": projection.left.u_px,
            "v_px": projection.left.v_px,
        },
        "right": {
            "status": projection.right.status.value,
            "u_px": projection.right.u_px,
            "v_px": projection.right.v_px,
        },
        "signed_disparity_px": projection.signed_disparity_px,
        "vertical_disparity_px": projection.vertical_disparity_px,
    }


def standard_cases() -> list[AttitudeCase]:
    zero = CameraAttitudeErrorDeg(0.0, 0.0, 0.0)
    center = ("center",)
    cases = [AttitudeCase("zero", zero, zero, tuple(MARKER_POINTS_BASELINE_FRD_M))]
    for side in ("left", "right"):
        for axis in ("roll", "pitch", "yaw"):
            for sign, label in ((+1.0, "positive"), (-1.0, "negative")):
                values = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
                values[axis] = sign * 0.1
                error = CameraAttitudeErrorDeg(values["roll"], values["pitch"], values["yaw"])
                left = error if side == "left" else zero
                right = error if side == "right" else zero
                marker_names = (ROLL_OBSERVABILITY_MARKER,) if axis == "roll" else center
                cases.append(AttitudeCase(f"{side}_{axis}_{label}", left, right, marker_names))
    cases.append(AttitudeCase("common_yaw_positive", CameraAttitudeErrorDeg(0.0, 0.0, 0.1), CameraAttitudeErrorDeg(0.0, 0.0, 0.1), center))
    cases.append(AttitudeCase("relative_yaw_symmetric", CameraAttitudeErrorDeg(0.0, 0.0, -0.1), CameraAttitudeErrorDeg(0.0, 0.0, 0.1), center))
    cases.append(
        AttitudeCase(
            "composite",
            CameraAttitudeErrorDeg(+0.10, -0.05, +0.08),
            CameraAttitudeErrorDeg(-0.07, +0.04, -0.06),
            center,
        )
    )
    cases.append(AttitudeCase("left_yaw_sensitivity_0p01", CameraAttitudeErrorDeg(0.0, 0.0, 0.01), zero, center))
    return cases


def smoke_cases() -> list[AttitudeCase]:
    zero = CameraAttitudeErrorDeg(0.0, 0.0, 0.0)
    return [
        AttitudeCase("zero", zero, zero, ("center",)),
        AttitudeCase("left_yaw_positive", CameraAttitudeErrorDeg(0.0, 0.0, 0.1), zero, ("center",)),
    ]


def case_matrix(name: str) -> list[AttitudeCase]:
    if name == "smoke":
        return smoke_cases()
    if name == "standard":
        return standard_cases()
    raise ValueError(f"unsupported case matrix: {name}")


def rotation_from_sdf_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[tuple[float, float, float], ...]:
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    rx = np.asarray(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)), dtype=float)
    ry = np.asarray(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)), dtype=float)
    rz = np.asarray(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)), dtype=float)
    return tuple(tuple(float(value) for value in row) for row in (rz @ ry @ rx))


def quaternion_xyzw_to_wxyz(orientation: dict[str, Any]) -> tuple[float, float, float, float]:
    return (
        float(orientation.get("w", 1.0)),
        float(orientation.get("x", 0.0)),
        float(orientation.get("y", 0.0)),
        float(orientation.get("z", 0.0)),
    )


def candidate_sdf_error_from_camera_error(error: CameraAttitudeErrorDeg) -> tuple[float, float, float]:
    return (error.roll_deg, -error.pitch_deg, -error.yaw_deg)


def add_marker_to_world_xml(world_xml: str, marker_world_enu_m: tuple[float, float, float], *, radius_m: float = 1.0) -> str:
    root = ET.fromstring(world_xml)
    world = root.find("world")
    if world is None:
        raise ValueError("world element not found")
    for existing in world.findall("model"):
        if existing.attrib.get("name") == MARKER_MODEL_NAME:
            world.remove(existing)
    model = ET.Element("model", {"name": MARKER_MODEL_NAME})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = (
        f"{marker_world_enu_m[0]:.9f} {marker_world_enu_m[1]:.9f} {marker_world_enu_m[2]:.9f} 0 0 0"
    )
    link = ET.SubElement(model, "link", {"name": "link"})
    visual = ET.SubElement(link, "visual", {"name": "marker_visual"})
    ET.SubElement(visual, "cast_shadows").text = "false"
    geometry = ET.SubElement(visual, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    ET.SubElement(sphere, "radius").text = f"{radius_m:.9f}"
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "0 0 0 1"
    ET.SubElement(material, "diffuse").text = "0 0 0 1"
    ET.SubElement(material, "emissive").text = "1 0 1 1"
    include_index = next((idx for idx, child in enumerate(list(world)) if child.tag == "include"), len(list(world)))
    world.insert(include_index, model)
    return ET.tostring(root, encoding="unicode")


def _pose_text_to_values(text: str | None) -> tuple[float, float, float, float, float, float]:
    if text is None or not text.strip():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    values = [float(item) for item in text.split()]
    if len(values) != 6:
        raise ValueError(f"expected six pose values, got {len(values)}")
    return tuple(values)  # type: ignore[return-value]


def parse_generated_model_poses(world_path: Path) -> dict[str, dict[str, Any]]:
    root = ET.fromstring(world_path.read_text(encoding="utf-8"))
    world = root.find("world")
    if world is None:
        raise ValueError("world element not found")
    poses: dict[str, dict[str, Any]] = {}
    for model in world.findall("model"):
        name = model.attrib.get("name")
        if not name:
            continue
        x, y, z, roll, pitch, yaw = _pose_text_to_values(model.findtext("pose"))
        poses[name] = {
            "position_world_enu_m": (x, y, z),
            "roll_rad": roll,
            "pitch_rad": pitch,
            "yaw_rad": yaw,
            "rotation_world_model": rotation_from_sdf_rpy(roll, pitch, yaw),
            "has_identity_link": model.find("link[@name='link']") is not None and _pose_text_to_values(model.find("link[@name='link']").findtext("pose") if model.find("link[@name='link']") is not None else None) == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        }
    return poses


def write_probe_world(base_world_path: Path, output_world_path: Path, marker_world_enu_m: tuple[float, float, float]) -> Path:
    output_world_path.parent.mkdir(parents=True, exist_ok=True)
    output_world_path.write_text(
        add_marker_to_world_xml(base_world_path.read_text(encoding="utf-8"), marker_world_enu_m),
        encoding="utf-8",
    )
    return output_world_path


def _fake_launch_configuration_module(values: dict[str, Any]):
    class FakeLaunchConfiguration:
        def __init__(self, name: str):
            self.name = name

        def perform(self, context):
            return str(context.launch_configurations[self.name])

    return FakeLaunchConfiguration


def load_production_launch_module(package_share: Path):
    launch_path = package_share / "launch" / "stereo_ground_sim.launch.py"
    spec = importlib.util.spec_from_file_location("_phase3c_stereo_ground_launch", launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load launch file: {launch_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def default_launch_config(case: AttitudeCase) -> dict[str, str]:
    return {
        "baseline_m": str(BASELINE_M),
        "camera_height_m": str(CAMERA_HEIGHT_M),
        "horizontal_fov_rad": str(HORIZONTAL_FOV_RAD),
        "camera_image_width_px": str(WIDTH_PX),
        "camera_image_height_px": str(HEIGHT_PX),
        "camera_update_rate_hz": "20.0",
        "camera_clip_near_m": str(CAMERA_CLIP_NEAR_M),
        "camera_clip_far_m": str(CAMERA_CLIP_FAR_M),
        "approach_start_distance_m": "2000.0",
        "target_spawn_x_m": "0.0",
        "target_spawn_y_m": "2000.0",
        "target_spawn_z_m": "0.75",
        "target_spawn_yaw_deg": "-90.0",
        "left_roll_error_deg": str(case.left.roll_deg),
        "left_pitch_error_deg": str(case.left.pitch_deg),
        "left_yaw_error_deg": str(case.left.yaw_deg),
        "right_roll_error_deg": str(case.right.roll_deg),
        "right_pitch_error_deg": str(case.right.pitch_deg),
        "right_yaw_error_deg": str(case.right.yaw_deg),
    }


def generate_production_world(case: AttitudeCase, generated_world_path: Path, package_share: Path | None = None) -> Path:
    if package_share is None:
        if get_package_share_directory is None:
            raise RuntimeError("ament_index_python is required to find the installed package share")
        package_share = Path(get_package_share_directory("stereo_ground_sim"))
    module = load_production_launch_module(package_share)
    from launch import LaunchContext

    context = LaunchContext()
    context.launch_configurations.update(default_launch_config(case))
    generated_world_path.parent.mkdir(parents=True, exist_ok=True)
    return Path(module._generate_world(context, generated_world_path=str(generated_world_path)))


def decode_image_to_rgb_array(msg: Any) -> tuple[np.ndarray | None, str]:
    encoding = str(getattr(msg, "encoding", "")).lower()
    width = int(getattr(msg, "width"))
    height = int(getattr(msg, "height"))
    step = int(getattr(msg, "step"))
    data = bytes(getattr(msg, "data"))
    if width != WIDTH_PX or height != HEIGHT_PX:
        return None, "dimension_mismatch"
    if encoding not in {"rgb8", "bgr8", "8uc3"}:
        return None, "encoding_mismatch"
    expected_min = height * step
    if len(data) < expected_min or step < width * 3:
        return None, "dimension_mismatch"
    raw = np.frombuffer(data[:expected_min], dtype=np.uint8).reshape((height, step))
    arr = raw[:, : width * 3].reshape((height, width, 3)).copy()
    if encoding == "bgr8":
        arr = arr[:, :, ::-1]
    return arr, "ok"


def marker_mask(rgb: np.ndarray, *, threshold: float = 80.0, min_brightness: int = 80) -> np.ndarray:
    red = rgb[:, :, 0].astype(np.int16)
    green = rgb[:, :, 1].astype(np.int16)
    blue = rgb[:, :, 2].astype(np.int16)
    score = np.minimum(red, blue) - green
    return (red >= min_brightness) & (blue >= min_brightness) & (score >= threshold)


def connected_components(mask: np.ndarray) -> list[np.ndarray]:
    visited = np.zeros(mask.shape, dtype=bool)
    components: list[np.ndarray] = []
    height, width = mask.shape
    for start_y, start_x in np.argwhere(mask):
        if visited[start_y, start_x]:
            continue
        queue = deque([(int(start_y), int(start_x))])
        visited[start_y, start_x] = True
        pixels = []
        while queue:
            y, x = queue.popleft()
            pixels.append((y, x))
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < height and 0 <= nx < width and mask[ny, nx] and not visited[ny, nx]:
                    visited[ny, nx] = True
                    queue.append((ny, nx))
        components.append(np.asarray(pixels, dtype=int))
    return components


def detect_marker(rgb: np.ndarray | None, *, threshold: float = 80.0, min_area_px: int = 5, max_area_px: int = 200000) -> MarkerDetection:
    if rgb is None:
        return MarkerDetection(False, "encoding_mismatch", None, None, 0, 0, None, "weighted_rgb_dominance")
    mask = marker_mask(rgb, threshold=threshold)
    components = connected_components(mask)
    if not components:
        return MarkerDetection(False, "marker_not_found", None, None, 0, 0, None, "weighted_rgb_dominance")
    component_infos = []
    for component in components:
        ys = component[:, 0]
        xs = component[:, 1]
        area = int(len(component))
        component_infos.append((area, int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()), component))
    valid = [info for info in component_infos if min_area_px <= info[0] <= max_area_px]
    if not valid:
        largest = max(component_infos, key=lambda item: item[0])
        return MarkerDetection(False, "area_out_of_range", None, None, int(largest[0]), len(components), largest[1:5], "weighted_rgb_dominance")
    valid.sort(key=lambda item: item[0], reverse=True)
    if len(valid) > 1 and valid[1][0] >= 0.25 * valid[0][0]:
        return MarkerDetection(False, "multiple_components", None, None, int(valid[0][0]), len(components), valid[0][1:5], "weighted_rgb_dominance")
    area, min_x, min_y, max_x, max_y, component = valid[0]
    if min_x <= 0 or min_y <= 0 or max_x >= rgb.shape[1] - 1 or max_y >= rgb.shape[0] - 1:
        return MarkerDetection(False, "marker_clipped", None, None, int(area), len(components), (min_x, min_y, max_x, max_y), "weighted_rgb_dominance")
    ys = component[:, 0]
    xs = component[:, 1]
    red = rgb[ys, xs, 0].astype(float)
    green = rgb[ys, xs, 1].astype(float)
    blue = rgb[ys, xs, 2].astype(float)
    weights = np.maximum(1.0, np.minimum(red, blue) - green)
    u = float(np.average(xs, weights=weights))
    v = float(np.average(ys, weights=weights))
    return MarkerDetection(True, "ok", u, v, int(area), len(components), (min_x, min_y, max_x, max_y), "weighted_rgb_dominance")


def intrinsic_delta_from_camera_info(camera_info: dict[str, Any], analytic_intrinsics: Any) -> dict[str, float | None]:
    k = camera_info.get("k") or camera_info.get("K") or camera_info.get("intrinsics", {}).get("k")
    if not k or len(k) < 6:
        return {"fx_delta_px": None, "fy_delta_px": None, "cx_delta_px": None, "cy_delta_px": None}
    return {
        "fx_delta_px": float(k[0]) - float(analytic_intrinsics.fx_px),
        "fy_delta_px": float(k[4]) - float(analytic_intrinsics.fy_px),
        "cx_delta_px": float(k[2]) - float(analytic_intrinsics.cx_px),
        "cy_delta_px": float(k[5]) - float(analytic_intrinsics.cy_px),
    }


def validate_camera_info(camera_info_by_topic: dict[str, dict[str, Any]]) -> dict[str, Any]:
    analytic = intrinsics()
    topic_to_side = {
        "/left_camera/camera_info": "left",
        "/right_camera/camera_info": "right",
    }
    result: dict[str, Any] = {"valid": True, "sides": {}, "camera_info_intrinsic_delta": {}}
    for topic, side in topic_to_side.items():
        info = camera_info_by_topic.get(topic)
        side_result = {"valid": False, "status": "missing"}
        if info is not None:
            k = info.get("k")
            p = info.get("p")
            finite_k = isinstance(k, list) and len(k) == 9 and all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in k)
            finite_p = isinstance(p, list) and len(p) == 12 and all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in p)
            dims_ok = info.get("width") == WIDTH_PX and info.get("height") == HEIGHT_PX
            valid = bool(dims_ok and finite_k and finite_p)
            if not dims_ok:
                status = "dimension_mismatch"
            elif not finite_k or not finite_p:
                status = "invalid_k_or_p"
            else:
                status = "ok"
            side_result = {
                "valid": valid,
                "status": status,
                "width": info.get("width"),
                "height": info.get("height"),
                "k_length": len(k) if isinstance(k, list) else None,
                "p_length": len(p) if isinstance(p, list) else None,
            }
        result["sides"][side] = side_result
        result["camera_info_intrinsic_delta"][side] = intrinsic_delta_from_camera_info(info or {}, analytic)
        result["valid"] = result["valid"] and bool(side_result["valid"])
    return result


def residual(rendered: dict[str, float], analytic: dict[str, float]) -> dict[str, float]:
    du = float(rendered["u_px"]) - float(analytic["u_px"])
    dv = float(rendered["v_px"]) - float(analytic["v_px"])
    return {"du_px": du, "dv_px": dv, "pixel_error_px": math.hypot(du, dv)}


def differential_residual(error_result: dict[str, Any], zero_result: dict[str, Any]) -> dict[str, float]:
    rendered_du = error_result["rendered"]["u_px"] - zero_result["rendered"]["u_px"]
    rendered_dv = error_result["rendered"]["v_px"] - zero_result["rendered"]["v_px"]
    analytic_du = error_result["analytic"]["u_px"] - zero_result["analytic"]["u_px"]
    analytic_dv = error_result["analytic"]["v_px"] - zero_result["analytic"]["v_px"]
    return {
        "delta_u_rendered_px": rendered_du,
        "delta_v_rendered_px": rendered_dv,
        "delta_u_analytic_px": analytic_du,
        "delta_v_analytic_px": analytic_dv,
        "delta_u_residual_px": rendered_du - analytic_du,
        "delta_v_residual_px": rendered_dv - analytic_dv,
        "delta_pixel_error_px": math.hypot(rendered_du - analytic_du, rendered_dv - analytic_dv),
    }


def summarize(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"count": 0, "median": None, "max": None, "mean": None}
    arr = np.asarray(values, dtype=float)
    return {
        "count": int(arr.size),
        "median": float(np.median(arr)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
    }


def determine_verdict(summary: dict[str, Any]) -> str:
    if not summary.get("infrastructure_valid", False):
        return "inconclusive"
    if summary.get("pose_coverage_rate", 0.0) < 1.0:
        return "inconclusive"
    if summary.get("marker_detection_success_rate", 0.0) < 0.95:
        return "inconclusive"
    if summary.get("required_case_coverage_rate", 0.0) < 1.0:
        return "inconclusive"
    absolute = summary.get("absolute_residual_summary", {})
    differential = summary.get("differential_residual_summary", {})
    composite = summary.get("composite_differential_residual_summary", {})
    centroid = summary.get("centroid_stddev_summary", {})
    sign = summary.get("single_axis_sign_agreement", {})
    equivalent = (
        (absolute.get("median") is not None and absolute["median"] <= 0.5)
        and (absolute.get("max") is not None and absolute["max"] <= 1.0)
        and (differential.get("median") is not None and differential["median"] <= 0.25)
        and (differential.get("max") is not None and differential["max"] <= 0.75)
        and (composite.get("median") is not None and composite["median"] <= 0.5)
        and (composite.get("max") is not None and composite["max"] <= 1.0)
        and (centroid.get("max") is not None and centroid["max"] <= 0.1)
        and sign.get("matched") == sign.get("total") == 12
    )
    return "equivalent" if equivalent else "not_equivalent"


def determine_attitude_semantics_verdict(summary: dict[str, Any]) -> str:
    """Judge attitude movement semantics independently of absolute alignment."""

    if not summary.get("infrastructure_valid", False):
        return "inconclusive"

    sign = summary.get("single_axis_sign_agreement", {})
    required_total = ATTITUDE_SEMANTICS_CRITERIA["single_axis_required_total"]
    if sign.get("total") != required_total or (sign.get("evaluable") or 0) < required_total:
        return "inconclusive"

    differential = summary.get("differential_residual_summary", {})
    composite = summary.get("composite_differential_residual_summary", {})
    centroid = summary.get("centroid_stddev_summary", {})
    required_summary_values = (
        differential.get("median"),
        differential.get("max"),
        composite.get("median"),
        composite.get("max"),
        centroid.get("max"),
        sign.get("matched"),
    )
    if any(value is None for value in required_summary_values):
        return "inconclusive"

    semantics_aligned = (
        sign.get("matched") == required_total
        and differential["median"] <= ATTITUDE_SEMANTICS_CRITERIA["single_axis_differential_median_px"]
        and differential["max"] <= ATTITUDE_SEMANTICS_CRITERIA["single_axis_differential_max_px"]
        and composite["median"] <= ATTITUDE_SEMANTICS_CRITERIA["composite_differential_median_px"]
        and composite["max"] <= ATTITUDE_SEMANTICS_CRITERIA["composite_differential_max_px"]
        and centroid["max"] <= ATTITUDE_SEMANTICS_CRITERIA["centroid_stddev_max_px"]
    )
    return "aligned" if semantics_aligned else "not_aligned"


def classify_mismatches(summary: dict[str, Any]) -> list[str]:
    classes: list[str] = []
    pose = summary.get("pose_residual_summary", {})
    camera_center_error = pose.get("camera_center_error_max_m")
    if camera_center_error is not None and camera_center_error > 1.0e-6:
        classes.append("center_mismatch")
    absolute = summary.get("absolute_residual_summary", {})
    if absolute.get("max") is not None and absolute["max"] > 1.0:
        classes.append("nominal_optical_mapping_mismatch")
    sign = summary.get("single_axis_sign_agreement", {})
    if sign.get("matched") is not None and sign.get("total") is not None and sign["matched"] < sign["total"]:
        classes.append("single_axis_sign_mismatch")
    composite = summary.get("composite_differential_residual_summary", {})
    if composite.get("max") is not None and composite["max"] > 1.0:
        classes.append("composite_order_mismatch")
    if absolute.get("median") is not None and absolute["median"] > 0.5:
        diff = summary.get("differential_residual_summary", {})
        if diff.get("median") is not None and diff["median"] <= 0.25:
            classes.append("constant_subpixel_offset")
    return sorted(set(classes))


def json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (str, bool, int)) or value is None:
        return value
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return str(value)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_safe(payload), ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(json_safe(payload), ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")


def environment_metadata() -> dict[str, Any]:
    def run(args: list[str]) -> str | None:
        try:
            return subprocess.run(args, check=False, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5).stdout.strip()
        except Exception:
            return None

    return {
        "gz_path": run(["which", "gz"]),
        "gazebo_sim_version": run(["gz", "sim", "--version"]),
        "gazebo_sim_versions": run(["gz", "sim", "--versions"]),
        "sdformat_versions": run(["gz", "sdf", "--versions"]),
        "gz_msgs_versions": run(["gz", "msg", "--versions"]),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "gz_partition": os.environ.get("GZ_PARTITION"),
        "headless_rendering_supported": "--headless-rendering" in (run(["gz", "sim", "--help"]) or ""),
        "quaternion_payload_order": "x,y,z,w",
    }


def configure_runtime_isolation(output_dir: Path) -> dict[str, str]:
    partition = os.environ.get("GZ_PARTITION") or f"phase3c_{os.getpid()}_{abs(hash(str(output_dir))) % 100000}"
    domain_id = os.environ.get("ROS_DOMAIN_ID") or str(100 + (os.getpid() % 100))
    os.environ["GZ_PARTITION"] = partition
    os.environ["ROS_DOMAIN_ID"] = domain_id
    return {"GZ_PARTITION": partition, "ROS_DOMAIN_ID": domain_id}


def start_process(name: str, command: list[str], log_path: Path, *, env: dict[str, str] | None = None) -> ProcessHandle:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )
    return ProcessHandle(name=name, process=process, log_handle=log)


def stop_process(handle: ProcessHandle, *, timeout_s: float = 10.0) -> None:
    process = handle.process
    try:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=timeout_s)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=timeout_s)
        except ProcessLookupError:
            pass
    finally:
        handle.log_handle.close()


def set_marker_pose(world_xyz: tuple[float, float, float], *, timeout_ms: int = 5000, env: dict[str, str] | None = None) -> bool:
    request = (
        f'name: "{MARKER_MODEL_NAME}" '
        f'position {{ x: {world_xyz[0]:.9f} y: {world_xyz[1]:.9f} z: {world_xyz[2]:.9f} }} '
        'orientation { w: 1 x: 0 y: 0 z: 0 }'
    )
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{WORLD_NAME}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(timeout_ms),
            "--req",
            request,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
        env=env,
    )
    return result.returncode == 0 and "true" in result.stdout.lower()


def echo_gz_topic_json(topic: str, *, num: int = 1, timeout_s: float = 5.0, env: dict[str, str] | None = None) -> list[dict[str, Any]]:
    result = subprocess.run(
        ["gz", "topic", "-e", "-t", topic, "--json-output", "-n", str(num)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
        timeout=timeout_s,
        env=env,
    )
    messages = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line or not line.startswith(("{", "[")):
            continue
        try:
            messages.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return messages


def _pose_records(payload: Any, parent_name: str | None = None) -> list[dict[str, Any]]:
    if isinstance(payload, dict):
        records = []
        name = payload.get("name")
        scoped_name = f"{parent_name}::{name}" if parent_name and name and ("position" in payload or "pose" in payload or "pos" in payload) else name
        if scoped_name and ("position" in payload or "pose" in payload or "pos" in payload):
            if scoped_name != name:
                payload = dict(payload)
                payload["name"] = scoped_name
            records.append(payload)
        child_parent = name if name and parent_name is None else parent_name
        for key, value in payload.items():
            if key in {"link", "links"} and name:
                records.extend(_pose_records(value, name))
            else:
                records.extend(_pose_records(value, child_parent))
        return records
    if isinstance(payload, list):
        records = []
        for item in payload:
            records.extend(_pose_records(item, parent_name))
        return records
    return []


def pose_by_name(payloads: list[dict[str, Any]], name: str) -> dict[str, Any] | None:
    for payload in payloads:
        for record in _pose_records(payload):
            if record.get("name") == name:
                return record
    return None


def pose_rotation_matrix(record: dict[str, Any]) -> tuple[tuple[float, float, float], ...] | None:
    source = record.get("pose") if isinstance(record.get("pose"), dict) else record
    orientation = source.get("orientation") or source.get("rot")
    if not isinstance(orientation, dict):
        return None
    return rotation_matrix_from_quaternion_wxyz(quaternion_xyzw_to_wxyz(orientation))


def pose_position_xyz(record: dict[str, Any]) -> tuple[float, float, float] | None:
    source = record.get("pose") if isinstance(record.get("pose"), dict) else record
    position = source.get("position") or source.get("pos")
    if not isinstance(position, dict):
        return None
    return (
        float(position.get("x", 0.0)),
        float(position.get("y", 0.0)),
        float(position.get("z", 0.0)),
    )


def vector_norm(values: tuple[float, float, float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def vector_delta(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def matrix_max_abs_delta(a: Any, b: Any) -> float:
    return float(np.max(np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))))


def _pose_check(name: str, value: float | None, tolerance: float) -> dict[str, Any]:
    return {
        "name": name,
        "value": value,
        "tolerance": tolerance,
        "passed": value is not None and math.isfinite(float(value)) and value <= tolerance,
    }


def pose_evidence_from_records(
    *,
    generated_poses: dict[str, dict[str, Any]],
    runtime_records: dict[str, dict[str, Any] | None],
    commanded_marker_world_enu_m: tuple[float, float, float],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    details: dict[str, Any] = {"checks": checks}

    def _runtime_position(name: str) -> tuple[float, float, float] | None:
        record = runtime_records.get(name)
        return pose_position_xyz(record) if record is not None else None

    def _runtime_rotation(name: str) -> tuple[tuple[float, float, float], ...] | None:
        record = runtime_records.get(name)
        return pose_rotation_matrix(record) if record is not None else None

    for side in ("left", "right"):
        model_name = f"{side}_stereo_camera"
        link_name = f"{model_name}::link"
        generated = generated_poses.get(model_name)
        model_pos = _runtime_position(model_name)
        link_pos = _runtime_position(link_name)
        model_rot = _runtime_rotation(model_name)
        link_rot = _runtime_rotation(link_name)
        generated_pos = generated.get("position_world_enu_m") if generated else None
        generated_rot = generated.get("rotation_world_model") if generated else None
        center_error = vector_norm(vector_delta(model_pos, generated_pos)) if model_pos and generated_pos else None
        rotation_error = matrix_max_abs_delta(model_rot, generated_rot) if model_rot is not None and generated_rot is not None else None
        model_link_center_error = vector_norm(vector_delta(model_pos, link_pos)) if model_pos and link_pos else None
        model_link_rotation_error = matrix_max_abs_delta(model_rot, link_rot) if model_rot is not None and link_rot is not None else None
        details[f"{side}_generated_runtime_center_error_m"] = center_error
        details[f"{side}_generated_runtime_rotation_max_error"] = rotation_error
        details[f"{side}_model_link_center_error_m"] = model_link_center_error
        details[f"{side}_model_link_rotation_max_error"] = model_link_rotation_error
        checks.append(_pose_check(f"{side}_generated_runtime_center", center_error, 1.0e-6))
        checks.append(_pose_check(f"{side}_generated_runtime_rotation", rotation_error, 1.0e-8))
        checks.append(_pose_check(f"{side}_model_link_center", model_link_center_error, 1.0e-6))
        checks.append(_pose_check(f"{side}_model_link_rotation", model_link_rotation_error, 1.0e-8))

    left_pos = _runtime_position("left_stereo_camera")
    right_pos = _runtime_position("right_stereo_camera")
    if left_pos and right_pos:
        baseline_vector = vector_delta(right_pos, left_pos)
        baseline_length_m = vector_norm(baseline_vector)
        midpoint = tuple((left_pos[idx] + right_pos[idx]) / 2.0 for idx in range(3))
        baseline_length_error = abs(baseline_length_m - BASELINE_M)
        baseline_midpoint_error = vector_norm(vector_delta(midpoint, O_W_B))
    else:
        baseline_length_m = None
        midpoint = None
        baseline_length_error = None
        baseline_midpoint_error = None
    marker_pos = _runtime_position(MARKER_MODEL_NAME)
    marker_pose_error = vector_norm(vector_delta(marker_pos, commanded_marker_world_enu_m)) if marker_pos else None
    details.update(
        {
            "runtime_baseline_length_m": baseline_length_m,
            "baseline_length_error_m": baseline_length_error,
            "runtime_baseline_midpoint_world_enu_m": midpoint,
            "baseline_midpoint_error_m": baseline_midpoint_error,
            "marker_commanded_runtime_position_error_m": marker_pose_error,
        }
    )
    checks.append(_pose_check("baseline_length", baseline_length_error, 1.0e-6))
    checks.append(_pose_check("baseline_midpoint", baseline_midpoint_error, 1.0e-6))
    checks.append(_pose_check("marker_commanded_runtime_position", marker_pose_error, 1.0e-6))
    details["valid"] = all(check["passed"] for check in checks)
    details["confirmed_check_count"] = sum(1 for check in checks if check["passed"])
    details["required_check_count"] = len(checks)
    return details


def complete_static_identity_link_records(
    generated_poses: dict[str, dict[str, Any]],
    runtime_records: dict[str, dict[str, Any] | None],
) -> dict[str, dict[str, Any] | None]:
    completed = dict(runtime_records)
    for model_name in ("left_stereo_camera", "right_stereo_camera"):
        link_name = f"{model_name}::link"
        if completed.get(link_name) is None and completed.get(model_name) is not None and generated_poses.get(model_name, {}).get("has_identity_link"):
            link_record = dict(completed[model_name])
            link_record["name"] = link_name
            link_record["pose_source"] = "implicit_static_identity_link_from_model_pose"
            completed[link_name] = link_record
    return completed


class RosImageCollector:
    def __init__(self, topics: tuple[str, ...], camera_info_topics: tuple[str, ...]):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image

        self.rclpy = rclpy
        self.node = Node("phase3c_camera_equivalence_probe")
        self.images = {topic: deque(maxlen=200) for topic in topics}
        self.camera_info = {}
        for topic in topics:
            self.node.create_subscription(Image, topic, lambda msg, topic=topic: self.images[topic].append((time.monotonic(), msg)), 10)
        for topic in camera_info_topics:
            self.node.create_subscription(CameraInfo, topic, lambda msg, topic=topic: self.camera_info.setdefault(topic, msg), 10)

    def spin_until(self, predicate, timeout_s: float) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def close(self) -> None:
        self.node.destroy_node()


def camera_info_to_dict(msg: Any) -> dict[str, Any]:
    return {
        "width": int(msg.width),
        "height": int(msg.height),
        "distortion_model": getattr(msg, "distortion_model", None),
        "d": list(getattr(msg, "d", [])),
        "k": list(getattr(msg, "k", [])),
        "r": list(getattr(msg, "r", [])),
        "p": list(getattr(msg, "p", [])),
        "binning_x": int(getattr(msg, "binning_x", 0)),
        "binning_y": int(getattr(msg, "binning_y", 0)),
        "roi": str(getattr(msg, "roi", "")),
    }


def sample_marker_from_images(
    collector: RosImageCollector,
    *,
    discard_frames: int,
    sample_frames: int,
    marker_threshold: float,
    output_dir: Path,
    case_name: str,
    marker_name: str,
) -> dict[str, Any]:
    topics = ("/left_camera/image", "/right_camera/image")
    target_counts = {topic: len(collector.images[topic]) + discard_frames + sample_frames for topic in topics}
    if not collector.spin_until(lambda: all(len(collector.images[t]) >= target_counts[t] for t in topics), timeout_s=20.0):
        return {"valid": False, "status": "image_topic_timeout"}
    result: dict[str, Any] = {"valid": True, "status": "ok", "cameras": {}}
    for topic in topics:
        frames = list(collector.images[topic])[-sample_frames:]
        detections = []
        decoded_count = 0
        for received_monotonic_s, msg in frames:
            rgb, decode_status = decode_image_to_rgb_array(msg)
            if decode_status == "ok":
                decoded_count += 1
            detection = detect_marker(rgb, threshold=marker_threshold)
            if decode_status != "ok":
                detection = MarkerDetection(False, decode_status, None, None, 0, 0, None, "weighted_rgb_dominance")
            detections.append(detection)
            append_jsonl(
                output_dir / "image_detection_samples.jsonl",
                {
                    "case": case_name,
                    "marker": marker_name,
                    "topic": topic,
                    "received_monotonic_s": received_monotonic_s,
                    "encoding": getattr(msg, "encoding", None),
                    "width": getattr(msg, "width", None),
                    "height": getattr(msg, "height", None),
                    "step": getattr(msg, "step", None),
                    "detection": detection.__dict__,
                },
            )
        valid_detections = [item for item in detections if item.valid]
        requested_count = sample_frames
        valid_count = len(valid_detections)
        invalid_count = requested_count - valid_count
        frame_success_rate = 0.0 if requested_count == 0 else valid_count / requested_count
        if not valid_detections or frame_success_rate < 0.95:
            result["valid"] = False
            result["status"] = "marker_detection_rate_below_threshold" if valid_detections else "marker_not_found"
            result["cameras"][topic] = {
                "valid": False,
                "requested_frame_count": requested_count,
                "decoded_frame_count": decoded_count,
                "valid_detection_frame_count": valid_count,
                "invalid_detection_frame_count": invalid_count,
                "frame_detection_success_rate": frame_success_rate,
                "status_counts": _status_counts(detections),
            }
            continue
        us = [item.u_px for item in valid_detections if item.u_px is not None]
        vs = [item.v_px for item in valid_detections if item.v_px is not None]
        result["cameras"][topic] = {
            "valid": True,
            "u_px": float(np.median(us)),
            "v_px": float(np.median(vs)),
            "stddev_px": float(max(np.std(us), np.std(vs))),
            "accepted_frame_count": len(valid_detections),
            "requested_frame_count": requested_count,
            "decoded_frame_count": decoded_count,
            "valid_detection_frame_count": valid_count,
            "invalid_detection_frame_count": invalid_count,
            "frame_detection_success_rate": frame_success_rate,
            "status_counts": _status_counts(detections),
        }
    return result


def _status_counts(items: list[MarkerDetection]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for item in items:
        counts[item.status] = counts.get(item.status, 0) + 1
    return counts


def run_case(case: AttitudeCase, args: argparse.Namespace, output_dir: Path, package_share: Path) -> list[dict[str, Any]]:
    generated_dir = output_dir / "generated_worlds"
    base_world = generated_dir / f"{case.name}_production.sdf"
    probe_world = generated_dir / f"{case.name}_marker.sdf"
    first_marker = case.marker_names[0]
    production_world = generate_production_world(case, base_world, package_share=package_share)
    write_probe_world(production_world, probe_world, marker_world_point(first_marker))
    generated_poses = parse_generated_model_poses(probe_world)

    env = os.environ.copy()
    models_dir = str(package_share / "models")
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([models_dir, env.get("GZ_SIM_RESOURCE_PATH", "")]).rstrip(os.pathsep)
    case_log_dir = output_dir / "logs" / case.name
    gz_command = ["gz", "sim", "-r", "-s"]
    if args.headless and environment_metadata().get("headless_rendering_supported"):
        gz_command.append("--headless-rendering")
    if args.gui:
        gz_command = ["gz", "sim", "-r"]
    gz_command.append(str(probe_world))
    gz_log_path = case_log_dir / "gazebo.log"
    bridge_log_path = case_log_dir / "bridge.log"
    gz = start_process("gz", gz_command, gz_log_path, env=env)
    bridge = start_process(
        "bridge",
        [
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "/left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        bridge_log_path,
        env=env,
    )
    handles = [gz, bridge]
    results: list[dict[str, Any]] = []
    try:
        import rclpy

        rclpy.init(args=None)
        collector = RosImageCollector(
            ("/left_camera/image", "/right_camera/image"),
            ("/left_camera/camera_info", "/right_camera/camera_info"),
        )
        try:
            if not collector.spin_until(lambda: all(collector.camera_info.values()) and all(collector.images[t] for t in collector.images), args.startup_timeout_s):
                return [{"case": case.name, "valid": False, "status": "startup_timeout"}]
            write_json(
                output_dir / "camera_info.json",
                {topic: camera_info_to_dict(msg) for topic, msg in collector.camera_info.items()},
            )
            topics = subprocess.run(["gz", "topic", "-l"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, env=env)
            (output_dir / "gz_topics.txt").write_text(topics.stdout, encoding="utf-8")
            for marker_name in case.marker_names:
                world_xyz = marker_world_point(marker_name)
                pose_ok = set_marker_pose(world_xyz, timeout_ms=int(args.pose_timeout_s * 1000), env=env)
                time.sleep(0.5)
                pose_payloads = echo_gz_topic_json(f"/world/{WORLD_NAME}/pose/info", num=1, timeout_s=args.pose_timeout_s, env=env)
                pose_payloads.extend(echo_gz_topic_json(f"/world/{WORLD_NAME}/state", num=1, timeout_s=args.pose_timeout_s, env=env))
                required_pose_names = (
                    "left_stereo_camera",
                    "left_stereo_camera::link",
                    "right_stereo_camera",
                    "right_stereo_camera::link",
                    MARKER_MODEL_NAME,
                )
                runtime_records = {name: pose_by_name(pose_payloads, name) for name in required_pose_names}
                raw_runtime_pose_names_present = {name: runtime_records[name] is not None for name in required_pose_names}
                runtime_records = complete_static_identity_link_records(generated_poses, runtime_records)
                pose_evidence = pose_evidence_from_records(
                    generated_poses=generated_poses,
                    runtime_records=runtime_records,
                    commanded_marker_world_enu_m=world_xyz,
                )
                pose_evidence["pose_command_success"] = pose_ok
                pose_evidence["runtime_pose_names_present"] = {name: runtime_records[name] is not None for name in required_pose_names}
                pose_evidence["raw_runtime_pose_names_present"] = raw_runtime_pose_names_present
                append_jsonl(
                    output_dir / "pose_samples.jsonl",
                    {
                        "case": case.name,
                        "marker": marker_name,
                        "pose_ok": pose_ok,
                        "pose_evidence": pose_evidence,
                        "runtime_records": runtime_records,
                    },
                )
                if not pose_ok or not pose_evidence["valid"]:
                    result = {
                        "case": case.name,
                        "marker": marker_name,
                        "valid": False,
                        "status": "pose_infrastructure_invalid",
                        "pose_evidence": pose_evidence,
                        "log_paths": {"gazebo": str(gz_log_path), "bridge": str(bridge_log_path)},
                    }
                    append_jsonl(output_dir / "case_results.jsonl", result)
                    results.append(result)
                    continue
                sample = sample_marker_from_images(
                    collector,
                    discard_frames=args.discard_frames,
                    sample_frames=args.sample_frames,
                    marker_threshold=args.marker_threshold,
                    output_dir=output_dir,
                    case_name=case.name,
                    marker_name=marker_name,
                )
                analytic = analytic_projection(marker_name, case)
                result = {
                    "case": case.name,
                    "marker": marker_name,
                    "attitude_deg": {
                        "left": case.left.__dict__,
                        "right": case.right.__dict__,
                    },
                    "marker_world_enu_m": world_xyz,
                    "marker_baseline_center_frd_m": MARKER_POINTS_BASELINE_FRD_M[marker_name],
                    "analytic_projection": analytic,
                    "sample": sample,
                    "pose_evidence": pose_evidence,
                    "log_paths": {"gazebo": str(gz_log_path), "bridge": str(bridge_log_path)},
                    "valid": bool(sample.get("valid") and analytic["valid"] and pose_evidence["valid"]),
                    "status": "ok" if sample.get("valid") and analytic["valid"] else sample.get("status", analytic["status"]),
                }
                append_jsonl(output_dir / "case_results.jsonl", result)
                results.append(result)
        finally:
            collector.close()
            rclpy.shutdown()
    finally:
        for handle in reversed(handles):
            stop_process(handle)
    return results


def summarize_runtime_results(results: list[dict[str, Any]], output_dir: Path, matrix_name: str, thresholds: dict[str, Any]) -> dict[str, Any]:
    absolute_errors = []
    single_axis_differential_errors = []
    composite_differential_errors = []
    centroid_stddevs = []
    requested_detection_frames = 0
    valid_detection_frames = 0
    observations: dict[tuple[str, str, str], dict[str, float]] = {}
    result_by_key = {(result.get("case"), result.get("marker")): result for result in results}
    required_case_defs = case_matrix(matrix_name)
    required_case_names = {case.name for case in required_case_defs}
    pose_confirmed = 0
    pose_required = 0
    pose_values = {
        "camera_center_error_max_m": [],
        "model_link_center_error_max_m": [],
        "baseline_length_error_max_m": [],
        "baseline_midpoint_error_max_m": [],
        "marker_pose_error_max_m": [],
        "generated_runtime_rotation_max_error": [],
        "model_link_rotation_max_error": [],
    }
    case_log_paths: dict[str, Any] = {}
    for result in results:
        pose = result.get("pose_evidence") or {}
        pose_confirmed += int(pose.get("confirmed_check_count") or 0)
        pose_required += int(pose.get("required_check_count") or 0)
        for side in ("left", "right"):
            for key, target in (
                (f"{side}_generated_runtime_center_error_m", "camera_center_error_max_m"),
                (f"{side}_generated_runtime_rotation_max_error", "generated_runtime_rotation_max_error"),
                (f"{side}_model_link_center_error_m", "model_link_center_error_max_m"),
                (f"{side}_model_link_rotation_max_error", "model_link_rotation_max_error"),
            ):
                value = pose.get(key)
                if value is not None:
                    pose_values[target].append(float(value))
        for key, target in (
            ("baseline_length_error_m", "baseline_length_error_max_m"),
            ("baseline_midpoint_error_m", "baseline_midpoint_error_max_m"),
            ("marker_commanded_runtime_position_error_m", "marker_pose_error_max_m"),
        ):
            value = pose.get(key)
            if value is not None:
                pose_values[target].append(float(value))
        if result.get("log_paths"):
            case_log_paths.setdefault(str(result.get("case")), result.get("log_paths"))
        if not result.get("sample"):
            continue
        sample = result["sample"]["cameras"]
        analytic = result["analytic_projection"]
        for side, topic in (("left", "/left_camera/image"), ("right", "/right_camera/image")):
            cam = sample.get(topic, {})
            requested_detection_frames += int(cam.get("requested_frame_count") or 0)
            valid_detection_frames += int(cam.get("valid_detection_frame_count") or 0)
            if cam.get("valid") and analytic.get(side, {}).get("u_px") is not None:
                rendered = {"u_px": cam["u_px"], "v_px": cam["v_px"]}
                expected = {"u_px": analytic[side]["u_px"], "v_px": analytic[side]["v_px"]}
                observations[(result["case"], result["marker"], side)] = {
                    "rendered_u": float(cam["u_px"]),
                    "rendered_v": float(cam["v_px"]),
                    "analytic_u": float(analytic[side]["u_px"]),
                    "analytic_v": float(analytic[side]["v_px"]),
                }
                if result.get("case") == "zero":
                    absolute_errors.append(residual(rendered, expected)["pixel_error_px"])
                centroid_stddevs.append(cam.get("stddev_px", math.nan))

    def _sign(value: float) -> int:
        if value > 0.0:
            return 1
        if value < 0.0:
            return -1
        return 0

    sign_matched = 0
    sign_total = 0
    sign_evaluable = 0
    sign_details = []
    for (case_name, marker_name, side), obs in observations.items():
        zero = observations.get(("zero", marker_name, side))
        if zero is None or case_name == "zero":
            continue
        rendered_du = obs["rendered_u"] - zero["rendered_u"]
        rendered_dv = obs["rendered_v"] - zero["rendered_v"]
        analytic_du = obs["analytic_u"] - zero["analytic_u"]
        analytic_dv = obs["analytic_v"] - zero["analytic_v"]
        error_px = math.hypot(rendered_du - analytic_du, rendered_dv - analytic_dv)
        is_single_axis = (
            case_name.startswith(("left_roll_", "left_pitch_", "left_yaw_", "right_roll_", "right_pitch_", "right_yaw_"))
            and case_name.endswith(("_positive", "_negative"))
        )
        if is_single_axis:
            single_axis_differential_errors.append(error_px)
            moved_side = "left" if case_name.startswith("left_") else "right"
            if side == moved_side:
                sign_total += 1
                analytic_shift = np.asarray([analytic_du, analytic_dv], dtype=float)
                rendered_shift = np.asarray([rendered_du, rendered_dv], dtype=float)
                analytic_norm = float(np.linalg.norm(analytic_shift))
                rendered_norm = float(np.linalg.norm(rendered_shift))
                evaluable = analytic_norm >= 0.25 and rendered_norm >= 0.25
                dominant_index = int(np.argmax(np.abs(analytic_shift)))
                dot = float(np.dot(analytic_shift, rendered_shift))
                matched = bool(evaluable and dot > 0.0 and _sign(float(analytic_shift[dominant_index])) == _sign(float(rendered_shift[dominant_index])))
                if evaluable:
                    sign_evaluable += 1
                if matched:
                    sign_matched += 1
                sign_details.append(
                    {
                        "case": case_name,
                        "camera": side,
                        "analytic_shift_px": [analytic_du, analytic_dv],
                        "rendered_shift_px": [rendered_du, rendered_dv],
                        "dot_product": dot,
                        "dominant_component": "u" if dominant_index == 0 else "v",
                        "evaluable": evaluable,
                        "matched": matched,
                    }
                )
        elif case_name == "composite":
            composite_differential_errors.append(error_px)

    covered_cases = 0
    for case in required_case_defs:
        case_covered = True
        for marker_name in case.marker_names:
            result = result_by_key.get((case.name, marker_name))
            if result is None:
                case_covered = False
                break
            sample = result.get("sample", {})
            cameras = sample.get("cameras", {})
            analytic = result.get("analytic_projection", {})
            if not (
                (result.get("pose_evidence") or {}).get("valid")
                and analytic.get("valid")
                and cameras.get("/left_camera/image", {}).get("valid")
                and cameras.get("/right_camera/image", {}).get("valid")
            ):
                case_covered = False
                break
        if case_covered:
            covered_cases += 1

    camera_info_path = output_dir / "camera_info.json"
    camera_info_payload = json.loads(camera_info_path.read_text(encoding="utf-8")) if camera_info_path.exists() else {}
    camera_info_validation = validate_camera_info(camera_info_payload)
    expected_pose_required = sum(len(case.marker_names) for case in required_case_defs) * REQUIRED_POSE_CHECKS_PER_MARKER_SAMPLE
    pose_coverage_rate = 0.0 if expected_pose_required == 0 else pose_confirmed / expected_pose_required
    marker_detection_success_rate = 0.0 if requested_detection_frames == 0 else valid_detection_frames / requested_detection_frames
    required_case_coverage_rate = covered_cases / max(1, len(required_case_names))
    required_single_axis = sum(
        1
        for case in required_case_defs
        if case.name.startswith(("left_roll_", "left_pitch_", "left_yaw_", "right_roll_", "right_pitch_", "right_yaw_"))
        and case.name.endswith(("_positive", "_negative"))
    )
    all_single_axis_evaluable = sign_evaluable == required_single_axis
    pose_residual_summary = {
        key: (max(values) if values else None)
        for key, values in pose_values.items()
    }
    pose_tolerances_ok = (
        pose_residual_summary["camera_center_error_max_m"] is not None
        and pose_residual_summary["camera_center_error_max_m"] <= 1.0e-6
        and pose_residual_summary["generated_runtime_rotation_max_error"] is not None
        and pose_residual_summary["generated_runtime_rotation_max_error"] <= 1.0e-8
        and pose_residual_summary["model_link_center_error_max_m"] is not None
        and pose_residual_summary["model_link_center_error_max_m"] <= 1.0e-6
        and pose_residual_summary["model_link_rotation_max_error"] is not None
        and pose_residual_summary["model_link_rotation_max_error"] <= 1.0e-8
        and pose_residual_summary["baseline_length_error_max_m"] is not None
        and pose_residual_summary["baseline_length_error_max_m"] <= 1.0e-6
        and pose_residual_summary["baseline_midpoint_error_max_m"] is not None
        and pose_residual_summary["baseline_midpoint_error_max_m"] <= 1.0e-6
        and pose_residual_summary["marker_pose_error_max_m"] is not None
        and pose_residual_summary["marker_pose_error_max_m"] <= 1.0e-6
    )
    infrastructure_valid = (
        pose_coverage_rate == 1.0
        and required_case_coverage_rate == 1.0
        and marker_detection_success_rate >= 0.95
        and bool(camera_info_validation["valid"])
        and pose_tolerances_ok
    )
    summary = {
        "schema_version": 1,
        "base_commit": BASE_COMMIT,
        "probe_commit_or_worktree_head": _git_head(),
        "environment": environment_metadata(),
        "case_matrix": matrix_name,
        "thresholds": thresholds,
        "infrastructure_valid": infrastructure_valid,
        "marker_detection_success_rate": marker_detection_success_rate,
        "marker_detection_frame_counts": {
            "requested": requested_detection_frames,
            "valid": valid_detection_frames,
            "invalid": requested_detection_frames - valid_detection_frames,
        },
        "pose_coverage_rate": pose_coverage_rate,
        "pose_check_counts": {"confirmed": pose_confirmed, "required": expected_pose_required, "observed_required": pose_required},
        "required_case_coverage_rate": required_case_coverage_rate,
        "single_axis_sign_agreement": {"matched": sign_matched, "total": sign_total, "evaluable": sign_evaluable, "required_evaluable": required_single_axis},
        "single_axis_sign_details": sign_details,
        "absolute_residual_summary": summarize(absolute_errors),
        "differential_residual_summary": summarize(single_axis_differential_errors),
        "composite_differential_residual_summary": summarize(composite_differential_errors),
        "centroid_stddev_summary": summarize([value for value in centroid_stddevs if math.isfinite(float(value))]),
        "pose_residual_summary": pose_residual_summary,
        "camera_info_valid": bool(camera_info_validation["valid"]),
        "camera_info_validation": camera_info_validation["sides"],
        "camera_info_intrinsic_delta": camera_info_validation["camera_info_intrinsic_delta"],
        "case_log_paths": case_log_paths,
        "roll_observability_marker": ROLL_OBSERVABILITY_MARKER,
        "pre_adapter_evidence_reference": PRE_ADAPTER_EVIDENCE_REFERENCE,
        "pre_adapter_evidence_available": Path(PRE_ADAPTER_EVIDENCE_REFERENCE).exists(),
        "attitude_semantics_criteria": dict(ATTITUDE_SEMANTICS_CRITERIA),
        "current_mapping_hypothesis": {
            "R_M_C_ZERO_CANDIDATE": R_M_C_ZERO_CANDIDATE,
            "roll_sdf_error": "+roll_camera_error",
            "pitch_sdf_error": "-pitch_camera_error",
            "yaw_sdf_error": "-yaw_camera_error",
            "verified_contract": False,
        },
        "gazebo_sdf_equivalence_verified": False,
    }
    summary["equivalence_verdict"] = determine_verdict(summary)
    summary["attitude_semantics_verdict"] = determine_attitude_semantics_verdict(summary)
    summary["mismatch_classifications"] = classify_mismatches(summary)
    write_json(output_dir / "summary.json", summary)
    return summary


def _git_head() -> str | None:
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, check=False).stdout.strip()
    except Exception:
        return None


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--case-matrix", choices=("smoke", "standard"), default="smoke")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--headless", action="store_true")
    mode.add_argument("--gui", action="store_true")
    parser.add_argument("--startup-timeout-s", type=float, default=30.0)
    parser.add_argument("--pose-timeout-s", type=float, default=5.0)
    parser.add_argument("--discard-frames", type=int, default=10)
    parser.add_argument("--sample-frames", type=int, default=10)
    parser.add_argument("--marker-threshold", type=float, default=80.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_LOG_DIR", str(output_dir / "ros_logs"))
    configure_runtime_isolation(output_dir)
    write_json(output_dir / "environment.json", environment_metadata())
    if get_package_share_directory is None:
        raise RuntimeError("ament_index_python is required for runtime probing")
    package_share = Path(get_package_share_directory("stereo_ground_sim"))
    thresholds = {
        "zero_absolute_median_px": 0.5,
        "zero_absolute_max_px": 1.0,
        "single_axis_differential_median_px": 0.25,
        "single_axis_differential_max_px": 0.75,
        "composite_differential_median_px": 0.5,
        "composite_differential_max_px": 1.0,
        "centroid_stddev_max_px": 0.1,
        "marker_detection_success_min": 0.95,
    }
    all_results = []
    for case in case_matrix(args.case_matrix):
        all_results.extend(run_case(case, args, output_dir, package_share))
    summary = summarize_runtime_results(all_results, output_dir, args.case_matrix, thresholds)
    print(json.dumps(json_safe(summary), ensure_ascii=False, sort_keys=True, allow_nan=False))
    return 2 if (
        summary["equivalence_verdict"] == "inconclusive"
        or summary["attitude_semantics_verdict"] == "inconclusive"
    ) else 0


if __name__ == "__main__":
    raise SystemExit(main())
