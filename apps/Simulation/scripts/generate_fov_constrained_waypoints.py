#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.coordinate_frames import (  # noqa: E402
    BASELINE_CENTER_FRD_FRAME,
    WORLD_FRAME,
    apply_optical_rpy_error,
    build_baseline_center_frd,
    nominal_camera_poses,
    project_world_point_to_pixel,
)


PX4_WAYPOINT_FRAME = "px4_local_ned_setpoint:north,east,altitude_agl_m"
WORLD_RFU_FRAME = "baseline_center_world_rfu:right,front,up"
WORLD_TO_PX4_WAYPOINT_CONTRACT = (
    "simulation_px4_mission_contract: Gazebo world ENU minus the PX4 home "
    "world ENU origin maps to local NED north/east/down; mission altitude_agl_m "
    "is local up and the runner sends down=-altitude_agl_m"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate disjoint calibration and validation PX4 waypoint sets "
            "inside the stereo camera 3D FOV, at any configured optical "
            "attitude (--cam-l/r-optical-roll/pitch/yaw-deg; default 0 "
            "reproduces the old level-camera behavior)."
        )
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260729)
    parser.add_argument("--calibration-count", type=int, default=12)
    parser.add_argument("--validation-count", type=int, default=12)
    parser.add_argument("--min-range-m", type=float, default=500.0)
    parser.add_argument("--max-range-m", type=float, default=2000.0)
    parser.add_argument(
        "--range-step-m", type=float, default=None,
        help=(
            "If set, generate points on the FOV boresight (right=down=0, i.e. "
            "dead centre of the FOV) stepped from --min-range-m to "
            "--max-range-m in this increment, instead of scattering them "
            "randomly through the FOV cone. The point count for each enabled "
            "phase (calibration/validation) is then derived from the range "
            "interval, overriding --calibration-count/--validation-count as "
            "a count (their nonzero-ness still selects whether that phase "
            "runs)."
        ),
    )
    parser.add_argument(
        "--range-count", type=int, default=None,
        help=(
            "If set with --range-step-m omitted, generate this many evenly "
            "spaced boresight/zigzag range points including both endpoints. "
            "The count must be at least 2."
        ),
    )
    parser.add_argument(
        "--lateral-pattern", choices=("center", "zigzag"), default="center",
        help="Use a centered boresight path or an alternating lateral path.",
    )
    parser.add_argument(
        "--zigzag-amplitude-m", type=float, default=2.0,
        help="Absolute lateral offset for the zigzag pattern, in metres.",
    )
    parser.add_argument(
        "--waypoints-rfu-m",
        default="",
        help=(
            "Semicolon-separated baseline-center world-aligned Right,Front,Up "
            "waypoints. Explicit points preserve input order and are retained "
            "even when outside one or both camera images."
        ),
    )
    parser.add_argument(
        "--zigzag-overlap-ratio",
        type=float,
        default=0.4,
        help=(
            "For explicit RFU zigzag paths, use this fraction of the nominal "
            "symmetric horizontal stereo-overlap half-width."
        ),
    )
    parser.add_argument("--camera-baseline-m", type=float, default=25.0)
    parser.add_argument("--camera-height-m", type=float, default=0.5)
    parser.add_argument(
        "--px4-origin-world-east-m", type=float, default=0.0,
        help="PX4 local origin in Gazebo world ENU east (world X), metres.",
    )
    parser.add_argument(
        "--px4-origin-world-north-m", type=float, default=0.0,
        help="PX4 local origin in Gazebo world ENU north (world Y), metres.",
    )
    parser.add_argument(
        "--px4-origin-world-up-m", type=float, default=0.0,
        help="PX4 local origin in Gazebo world ENU up (world Z), metres.",
    )
    parser.add_argument(
        "--cam-l-optical-roll-deg", type=float, default=0.0,
        help="Left camera optical RDF roll, deg. Matches CAM_L_OPTICAL_ROLL_DEG.",
    )
    parser.add_argument(
        "--cam-l-optical-pitch-deg", type=float, default=0.0,
        help="Left camera optical RDF pitch, deg. Matches CAM_L_OPTICAL_PITCH_DEG.",
    )
    parser.add_argument(
        "--cam-l-optical-yaw-deg", type=float, default=0.0,
        help="Left camera optical RDF yaw, deg. Matches CAM_L_OPTICAL_YAW_DEG.",
    )
    parser.add_argument(
        "--cam-r-optical-roll-deg", type=float, default=0.0,
        help="Right camera optical RDF roll, deg. Matches CAM_R_OPTICAL_ROLL_DEG.",
    )
    parser.add_argument(
        "--cam-r-optical-pitch-deg", type=float, default=0.0,
        help="Right camera optical RDF pitch, deg. Matches CAM_R_OPTICAL_PITCH_DEG.",
    )
    parser.add_argument(
        "--cam-r-optical-yaw-deg", type=float, default=0.0,
        help="Right camera optical RDF yaw, deg. Matches CAM_R_OPTICAL_YAW_DEG.",
    )
    parser.add_argument("--horizontal-fov-rad", type=float, default=0.154997710698)
    parser.add_argument("--image-width-px", type=int, default=1280)
    parser.add_argument("--image-height-px", type=int, default=720)
    parser.add_argument("--fov-margin-rad", type=float, default=0.01)
    parser.add_argument("--pixel-margin-px", type=float, default=32.0)
    parser.add_argument("--min-set-separation-m", type=float, default=20.0)
    parser.add_argument("--max-attempts", type=int, default=100000)
    return parser.parse_args()


def vertical_fov_rad(
    horizontal_fov_rad: float,
    image_width_px: int,
    image_height_px: int,
) -> float:
    aspect = float(image_height_px) / float(image_width_px)
    return 2.0 * math.atan(math.tan(horizontal_fov_rad * 0.5) * aspect)


def focal_lengths_px(
    horizontal_fov_rad: float,
    image_width_px: int,
    image_height_px: int,
) -> tuple[float, float]:
    fx = float(image_width_px) / (2.0 * math.tan(horizontal_fov_rad * 0.5))
    fy = fx
    return fx, fy


def baseline_distance(a: Mapping[str, float], b: Mapping[str, float]) -> float:
    return math.sqrt(
        (float(a["baseline_forward_m"]) - float(b["baseline_forward_m"])) ** 2
        + (float(a["baseline_right_m"]) - float(b["baseline_right_m"])) ** 2
        + (float(a["baseline_down_m"]) - float(b["baseline_down_m"])) ** 2
    )


def inside_pixel_margin(
    pixel: tuple[float, float] | None,
    *,
    width_px: int,
    height_px: int,
    margin_px: float,
) -> bool:
    if pixel is None:
        return False
    u, v = pixel
    return (
        margin_px <= u <= float(width_px) - margin_px
        and margin_px <= v <= float(height_px) - margin_px
    )


def parse_rfu_waypoints(value: str) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index, entry in enumerate(str(value).split(";"), start=1):
        entry = entry.strip()
        if not entry:
            continue
        parts = [part.strip() for part in entry.split(",")]
        if len(parts) != 3:
            raise ValueError(
                f"RFU waypoint {index} must be right,front,up; got: {entry}"
            )
        point = tuple(float(part) for part in parts)
        if not all(math.isfinite(component) for component in point):
            raise ValueError(f"RFU waypoint {index} must be finite")
        if point[1] <= 0.0:
            raise ValueError(f"RFU waypoint {index} Front must be positive")
        points.append(point)  # type: ignore[arg-type]
    return points


def _project_rfu_point(
    point_rfu_m: tuple[float, float, float],
    *,
    baseline_origin_world_enu_m: tuple[float, float, float],
    camera,
    fx_px: float,
    fy_px: float,
    image_width_px: int,
    image_height_px: int,
) -> tuple[float, float] | None:
    right_m, front_m, up_m = point_rfu_m
    world = (
        baseline_origin_world_enu_m[0] + right_m,
        baseline_origin_world_enu_m[1] + front_m,
        baseline_origin_world_enu_m[2] + up_m,
    )
    return project_world_point_to_pixel(
        world,
        camera,
        fx_px=fx_px,
        fy_px=fy_px,
        cx_px=float(image_width_px) * 0.5,
        cy_px=float(image_height_px) * 0.5,
    )


def horizontal_stereo_overlap_half_width_m(
    base_point_rfu_m: tuple[float, float, float],
    *,
    baseline_origin_world_enu_m: tuple[float, float, float],
    left_camera,
    right_camera,
    fx_px: float,
    fy_px: float,
    image_width_px: int,
    image_height_px: int,
    pixel_margin_px: float,
) -> float:
    """Return the symmetric nominal horizontal overlap around the input R.

    Vertical visibility is deliberately excluded: explicit mission points are
    retained after they leave the vertical FOV, while the lateral zigzag still
    tapers according to the remaining horizontal stereo overlap.
    """

    base_right_m, front_m, up_m = base_point_rfu_m

    def visible(offset_m: float) -> bool:
        point = (base_right_m + offset_m, front_m, up_m)
        for camera in (left_camera, right_camera):
            pixel = _project_rfu_point(
                point,
                baseline_origin_world_enu_m=baseline_origin_world_enu_m,
                camera=camera,
                fx_px=fx_px,
                fy_px=fy_px,
                image_width_px=image_width_px,
                image_height_px=image_height_px,
            )
            if pixel is None:
                return False
            if not pixel_margin_px <= pixel[0] <= float(image_width_px) - pixel_margin_px:
                return False
        return True

    if not visible(0.0):
        return 0.0

    def limit(sign: float) -> float:
        low = 0.0
        high = 1.0
        maximum_search_m = max(1000.0, abs(front_m) * 4.0)
        while high < maximum_search_m and visible(sign * high):
            low = high
            high *= 2.0
        high = min(high, maximum_search_m)
        if visible(sign * high):
            return high
        for _ in range(64):
            midpoint = (low + high) * 0.5
            if visible(sign * midpoint):
                low = midpoint
            else:
                high = midpoint
        return low

    return min(limit(-1.0), limit(1.0))


def sample_one(
    rng: random.Random,
    *,
    phase: str,
    index: int,
    existing_other_set: Sequence[Mapping[str, float]],
    min_set_separation_m: float,
    min_range_m: float,
    max_range_m: float,
    horizontal_fov_rad: float,
    vertical_fov: float,
    fov_margin_rad: float,
    frame,
    left_camera,
    right_camera,
    fx_px: float,
    fy_px: float,
    image_width_px: int,
    image_height_px: int,
    pixel_margin_px: float,
    max_attempts: int,
    px4_origin_world_enu_m: tuple[float, float, float],
    forced_forward_m: float | None = None,
    forced_right_m: float | None = None,
    forced_down_m: float | None = None,
) -> dict[str, float | int | str]:
    usable_h = horizontal_fov_rad * 0.5 - fov_margin_rad
    usable_v = vertical_fov * 0.5 - fov_margin_rad
    if usable_h <= 0.0 or usable_v <= 0.0:
        raise ValueError("FOV margin leaves no usable angular envelope")

    for _ in range(max_attempts):
        forward = forced_forward_m if forced_forward_m is not None else rng.uniform(min_range_m, max_range_m)
        right = forced_right_m if forced_right_m is not None else rng.uniform(-math.tan(usable_h) * forward, math.tan(usable_h) * forward)
        down = forced_down_m if forced_down_m is not None else rng.uniform(-math.tan(usable_v) * forward, math.tan(usable_v) * forward)
        world = frame.point_to_world((forward, right, down))
        left_pixel = project_world_point_to_pixel(
            world,
            left_camera,
            fx_px=fx_px,
            fy_px=fy_px,
            cx_px=float(image_width_px) * 0.5,
            cy_px=float(image_height_px) * 0.5,
        )
        right_pixel = project_world_point_to_pixel(
            world,
            right_camera,
            fx_px=fx_px,
            fy_px=fy_px,
            cx_px=float(image_width_px) * 0.5,
            cy_px=float(image_height_px) * 0.5,
        )
        if not inside_pixel_margin(
            left_pixel,
            width_px=image_width_px,
            height_px=image_height_px,
            margin_px=pixel_margin_px,
        ):
            continue
        if not inside_pixel_margin(
            right_pixel,
            width_px=image_width_px,
            height_px=image_height_px,
            margin_px=pixel_margin_px,
        ):
            continue
        candidate: dict[str, float | int | str] = {
            "phase": phase,
            "index": index,
            "px4_waypoint_frame": PX4_WAYPOINT_FRAME,
            "world_frame": WORLD_FRAME,
            "baseline_frame": BASELINE_CENTER_FRD_FRAME,
            "world_to_px4_waypoint_contract": WORLD_TO_PX4_WAYPOINT_CONTRACT,
            # Position input is absolute Gazebo world ENU [East, North, Up].
            # PX4 mission setpoints are local NED relative to the vehicle's
            # home/spawn origin. The CSV stores local Up as altitude_agl_m;
            # px4_auto_mission converts it explicitly to local Down by negating it.
            "north_m": world[1] - px4_origin_world_enu_m[1],
            "east_m": world[0] - px4_origin_world_enu_m[0],
            "altitude_agl_m": world[2] - px4_origin_world_enu_m[2],
            "world_x_m": world[0],
            "world_y_m": world[1],
            "world_z_m": world[2],
            "baseline_forward_m": forward,
            "baseline_right_m": right,
            "baseline_down_m": down,
            "baseline_range_m": math.sqrt(forward * forward + right * right + down * down),
            "horizontal_angle_rad": math.atan2(right, forward),
            "vertical_angle_rad": math.atan2(down, forward),
            "left_u_px": left_pixel[0],
            "left_v_px": left_pixel[1],
            "right_u_px": right_pixel[0],
            "right_v_px": right_pixel[1],
        }
        if any(baseline_distance(candidate, other) < min_set_separation_m for other in existing_other_set):
            continue
        return candidate
    raise RuntimeError(f"could not sample {phase} waypoint {index} inside FOV after {max_attempts} attempts")


def build_explicit_rfu_row(
    *,
    index: int,
    requested_rfu_m: tuple[float, float, float],
    applied_right_m: float,
    overlap_half_width_m: float,
    overlap_ratio: float,
    frame,
    left_camera,
    right_camera,
    fx_px: float,
    fy_px: float,
    image_width_px: int,
    image_height_px: int,
    pixel_margin_px: float,
    px4_origin_world_enu_m: tuple[float, float, float],
) -> dict[str, float | int | str | bool]:
    _, front_m, up_m = requested_rfu_m
    applied_rfu_m = (applied_right_m, front_m, up_m)
    origin = frame.origin_world_enu_m
    world = (
        origin[0] + applied_right_m,
        origin[1] + front_m,
        origin[2] + up_m,
    )
    baseline_forward_m, baseline_right_m, baseline_down_m = frame.point_from_world(world)
    left_pixel = _project_rfu_point(
        applied_rfu_m,
        baseline_origin_world_enu_m=origin,
        camera=left_camera,
        fx_px=fx_px,
        fy_px=fy_px,
        image_width_px=image_width_px,
        image_height_px=image_height_px,
    )
    right_pixel = _project_rfu_point(
        applied_rfu_m,
        baseline_origin_world_enu_m=origin,
        camera=right_camera,
        fx_px=fx_px,
        fy_px=fy_px,
        image_width_px=image_width_px,
        image_height_px=image_height_px,
    )
    left_visible = inside_pixel_margin(
        left_pixel,
        width_px=image_width_px,
        height_px=image_height_px,
        margin_px=pixel_margin_px,
    )
    right_visible = inside_pixel_margin(
        right_pixel,
        width_px=image_width_px,
        height_px=image_height_px,
        margin_px=pixel_margin_px,
    )
    altitude_agl_m = world[2] - px4_origin_world_enu_m[2]
    if altitude_agl_m < 0.0:
        raise ValueError(
            f"RFU waypoint {index + 1} maps below the PX4 local origin: "
            f"altitude_agl_m={altitude_agl_m}"
        )
    baseline_range_m = math.sqrt(
        baseline_forward_m * baseline_forward_m
        + baseline_right_m * baseline_right_m
        + baseline_down_m * baseline_down_m
    )
    return {
        "phase": "validation",
        "index": index,
        "source_order": index,
        "source_frame": WORLD_RFU_FRAME,
        "px4_waypoint_frame": PX4_WAYPOINT_FRAME,
        "world_frame": WORLD_FRAME,
        "baseline_frame": BASELINE_CENTER_FRD_FRAME,
        "world_to_px4_waypoint_contract": WORLD_TO_PX4_WAYPOINT_CONTRACT,
        "requested_rfu_right_m": requested_rfu_m[0],
        "requested_rfu_front_m": front_m,
        "requested_rfu_up_m": up_m,
        "world_rfu_right_m": applied_right_m,
        "world_rfu_front_m": front_m,
        "world_rfu_up_m": up_m,
        "zigzag_overlap_half_width_m": overlap_half_width_m,
        "zigzag_overlap_ratio": overlap_ratio,
        "north_m": world[1] - px4_origin_world_enu_m[1],
        "east_m": world[0] - px4_origin_world_enu_m[0],
        "altitude_agl_m": altitude_agl_m,
        "world_x_m": world[0],
        "world_y_m": world[1],
        "world_z_m": world[2],
        "baseline_forward_m": baseline_forward_m,
        "baseline_right_m": baseline_right_m,
        "baseline_down_m": baseline_down_m,
        "baseline_range_m": baseline_range_m,
        "horizontal_angle_rad": math.atan2(baseline_right_m, baseline_forward_m),
        "vertical_angle_rad": math.atan2(baseline_down_m, baseline_forward_m),
        "left_u_px": math.nan if left_pixel is None else left_pixel[0],
        "left_v_px": math.nan if left_pixel is None else left_pixel[1],
        "right_u_px": math.nan if right_pixel is None else right_pixel[0],
        "right_v_px": math.nan if right_pixel is None else right_pixel[1],
        "nominal_left_visible": left_visible,
        "nominal_right_visible": right_visible,
        "nominal_stereo_visible": left_visible and right_visible,
    }


def build_stepped_range_values(min_range_m: float, max_range_m: float, step_m: float) -> list[float]:
    if step_m <= 0.0:
        raise ValueError("range step must be positive")
    values: list[float] = []
    n = 0
    value = min_range_m
    while value <= max_range_m + 1e-6:
        values.append(value)
        n += 1
        value = min_range_m + n * step_m
    return values


def build_range_count_values(min_range_m: float, max_range_m: float, count: int) -> list[float]:
    if count < 2:
        raise ValueError("range count must be at least 2")
    step_m = (max_range_m - min_range_m) / float(count - 1)
    return [min_range_m + index * step_m for index in range(count)]


def lateral_offset_m(args: argparse.Namespace, index: int) -> float:
    pattern = getattr(args, "lateral_pattern", "center")
    if pattern == "center":
        return 0.0
    amplitude = float(getattr(args, "zigzag_amplitude_m", 2.0))
    if amplitude < 0.0 or not math.isfinite(amplitude):
        raise ValueError("zigzag amplitude must be finite and non-negative")
    return amplitude if index % 2 == 0 else -amplitude


def generate_waypoints(
    args: argparse.Namespace,
) -> tuple[
    list[dict[str, float | int | str]],
    list[dict[str, float | int | str]],
    dict[str, object],
]:
    if args.calibration_count < 0 or args.validation_count < 0:
        raise ValueError("waypoint counts must be non-negative")
    if args.min_range_m <= 0.0 or args.max_range_m <= args.min_range_m:
        raise ValueError("range bounds must be positive and increasing")
    range_step_m = getattr(args, "range_step_m", None)
    range_count = getattr(args, "range_count", None)
    explicit_rfu_points = parse_rfu_waypoints(getattr(args, "waypoints_rfu_m", ""))
    selected_modes = sum(
        value is not None and value != []
        for value in (range_step_m, range_count, explicit_rfu_points)
    )
    if selected_modes > 1:
        raise ValueError(
            "range-step-m, range-count, and waypoints-rfu-m are mutually exclusive"
        )
    if range_count is not None and range_count < 2:
        raise ValueError("range count must be at least 2")
    zigzag_overlap_ratio = float(getattr(args, "zigzag_overlap_ratio", 0.4))
    if not math.isfinite(zigzag_overlap_ratio) or not 0.0 <= zigzag_overlap_ratio <= 1.0:
        raise ValueError("zigzag overlap ratio must be between 0 and 1")
    if explicit_rfu_points:
        if args.calibration_count != 0:
            raise ValueError("explicit RFU waypoints require calibration-count=0")
        if args.validation_count not in (0, len(explicit_rfu_points)):
            raise ValueError(
                "validation-count must be 0 or match the explicit RFU waypoint count"
            )
    rng = random.Random(args.seed)
    nominal_left, nominal_right = nominal_camera_poses(args.camera_baseline_m, args.camera_height_m)
    # nominal_camera_poses() is always level (roll=pitch=0); rotate it to the
    # actual configured optical attitude so the sampling cone -- and the
    # left/right pixel-margin check in sample_one() -- match where the
    # cameras really point, not just the level case. Zero angles are a no-op,
    # so this generalizes the old level-only behavior rather than replacing it.
    left_rpy_deg = tuple(
        float(getattr(args, name, 0.0))
        for name in ("cam_l_optical_roll_deg", "cam_l_optical_pitch_deg", "cam_l_optical_yaw_deg")
    )
    right_rpy_deg = tuple(
        float(getattr(args, name, 0.0))
        for name in ("cam_r_optical_roll_deg", "cam_r_optical_pitch_deg", "cam_r_optical_yaw_deg")
    )
    left_camera = apply_optical_rpy_error(
        nominal_left,
        *(math.radians(value) for value in left_rpy_deg),
    )
    right_camera = apply_optical_rpy_error(
        nominal_right,
        *(math.radians(value) for value in right_rpy_deg),
    )
    frame = build_baseline_center_frd(
        left_camera.center_world_enu_m,
        right_camera.center_world_enu_m,
        left_camera.forward_world_enu,
        right_camera.forward_world_enu,
    )
    v_fov = vertical_fov_rad(args.horizontal_fov_rad, args.image_width_px, args.image_height_px)
    fx_px, fy_px = focal_lengths_px(args.horizontal_fov_rad, args.image_width_px, args.image_height_px)
    px4_origin_world_enu_m = (
        float(args.px4_origin_world_east_m),
        float(args.px4_origin_world_north_m),
        float(args.px4_origin_world_up_m),
    )

    calibration: list[dict[str, object]] = []
    validation: list[dict[str, object]] = []
    if explicit_rfu_points:
        for index, requested_rfu_m in enumerate(explicit_rfu_points):
            overlap_half_width_m = horizontal_stereo_overlap_half_width_m(
                requested_rfu_m,
                baseline_origin_world_enu_m=frame.origin_world_enu_m,
                left_camera=left_camera,
                right_camera=right_camera,
                fx_px=fx_px,
                fy_px=fy_px,
                image_width_px=args.image_width_px,
                image_height_px=args.image_height_px,
                pixel_margin_px=args.pixel_margin_px,
            )
            applied_right_m = requested_rfu_m[0]
            if getattr(args, "lateral_pattern", "center") == "zigzag":
                sign = 1.0 if index % 2 == 0 else -1.0
                applied_right_m += sign * zigzag_overlap_ratio * overlap_half_width_m
            validation.append(
                build_explicit_rfu_row(
                    index=index,
                    requested_rfu_m=requested_rfu_m,
                    applied_right_m=applied_right_m,
                    overlap_half_width_m=overlap_half_width_m,
                    overlap_ratio=(
                        zigzag_overlap_ratio
                        if getattr(args, "lateral_pattern", "center") == "zigzag"
                        else 0.0
                    ),
                    frame=frame,
                    left_camera=left_camera,
                    right_camera=right_camera,
                    fx_px=fx_px,
                    fy_px=fy_px,
                    image_width_px=args.image_width_px,
                    image_height_px=args.image_height_px,
                    pixel_margin_px=args.pixel_margin_px,
                    px4_origin_world_enu_m=px4_origin_world_enu_m,
                )
            )
    else:
        calibration_range_values = (
            build_stepped_range_values(args.min_range_m, args.max_range_m, range_step_m)
            if range_step_m is not None and args.calibration_count > 0
            else build_range_count_values(args.min_range_m, args.max_range_m, range_count)
            if range_count is not None and args.calibration_count > 0
            else None
        )
        calibration_total = len(calibration_range_values) if calibration_range_values is not None else args.calibration_count
        for index in range(calibration_total):
            calibration.append(
                sample_one(
                    rng,
                    phase="calibration",
                    index=index,
                    existing_other_set=(),
                    min_set_separation_m=0.0,
                    min_range_m=args.min_range_m,
                    max_range_m=args.max_range_m,
                    horizontal_fov_rad=args.horizontal_fov_rad,
                    vertical_fov=v_fov,
                    fov_margin_rad=args.fov_margin_rad,
                    frame=frame,
                    left_camera=left_camera,
                    right_camera=right_camera,
                    fx_px=fx_px,
                    fy_px=fy_px,
                    image_width_px=args.image_width_px,
                    image_height_px=args.image_height_px,
                    pixel_margin_px=args.pixel_margin_px,
                    max_attempts=args.max_attempts,
                    px4_origin_world_enu_m=px4_origin_world_enu_m,
                    forced_forward_m=calibration_range_values[index] if calibration_range_values is not None else None,
                    forced_right_m=lateral_offset_m(args, index) if calibration_range_values is not None else None,
                    forced_down_m=0.0 if calibration_range_values is not None else None,
                )
            )

        validation_range_values = (
            build_stepped_range_values(args.min_range_m, args.max_range_m, range_step_m)
            if range_step_m is not None and args.validation_count > 0
            else build_range_count_values(args.min_range_m, args.max_range_m, range_count)
            if range_count is not None and args.validation_count > 0
            else None
        )
        validation_total = len(validation_range_values) if validation_range_values is not None else args.validation_count
        for index in range(validation_total):
            validation.append(
                sample_one(
                    rng,
                    phase="validation",
                    index=index,
                    existing_other_set=calibration,
                    min_set_separation_m=args.min_set_separation_m,
                    min_range_m=args.min_range_m,
                    max_range_m=args.max_range_m,
                    horizontal_fov_rad=args.horizontal_fov_rad,
                    vertical_fov=v_fov,
                    fov_margin_rad=args.fov_margin_rad,
                    frame=frame,
                    left_camera=left_camera,
                    right_camera=right_camera,
                    fx_px=fx_px,
                    fy_px=fy_px,
                    image_width_px=args.image_width_px,
                    image_height_px=args.image_height_px,
                    pixel_margin_px=args.pixel_margin_px,
                    max_attempts=args.max_attempts,
                    px4_origin_world_enu_m=px4_origin_world_enu_m,
                    forced_forward_m=validation_range_values[index] if validation_range_values is not None else None,
                    forced_right_m=lateral_offset_m(args, index) if validation_range_values is not None else None,
                    forced_down_m=0.0 if validation_range_values is not None else None,
                )
            )

    manifest = {
        "contract": "Calibration.md waypoint generation contract",
        "calibration_count": len(calibration),
        "validation_count": len(validation),
        "sets_disjoint": all(
            baseline_distance(left, right) >= args.min_set_separation_m
            for left in calibration
            for right in validation
        ),
        "min_set_separation_m": args.min_set_separation_m,
        "cam_l_optical_roll_deg": left_rpy_deg[0],
        "cam_l_optical_pitch_deg": left_rpy_deg[1],
        "cam_l_optical_yaw_deg": left_rpy_deg[2],
        "cam_r_optical_roll_deg": right_rpy_deg[0],
        "cam_r_optical_pitch_deg": right_rpy_deg[1],
        "cam_r_optical_yaw_deg": right_rpy_deg[2],
        "world_frame": WORLD_FRAME,
        "baseline_frame": BASELINE_CENTER_FRD_FRAME,
        "px4_waypoint_frame": PX4_WAYPOINT_FRAME,
        "world_to_px4_waypoint_contract": WORLD_TO_PX4_WAYPOINT_CONTRACT,
        "px4_origin_world_enu_m": {
            "east": px4_origin_world_enu_m[0],
            "north": px4_origin_world_enu_m[1],
            "up": px4_origin_world_enu_m[2],
        },
        "horizontal_fov_rad": args.horizontal_fov_rad,
        "vertical_fov_rad": v_fov,
        "fov_margin_rad": args.fov_margin_rad,
        "pixel_margin_px": args.pixel_margin_px,
        "range_min_m": args.min_range_m,
        "range_max_m": args.max_range_m,
        "range_step_m": range_step_m,
        "range_count": range_count,
        "waypoint_mode": (
            "explicit_baseline_center_world_rfu"
            if explicit_rfu_points
            else "sampled_or_boresight"
        ),
        "waypoints_rfu_m": [list(point) for point in explicit_rfu_points],
        "world_rfu_frame": WORLD_RFU_FRAME,
        "lateral_pattern": getattr(args, "lateral_pattern", "center"),
        "zigzag_amplitude_m": float(getattr(args, "zigzag_amplitude_m", 2.0)),
        "zigzag_overlap_ratio": zigzag_overlap_ratio,
        "nominal_stereo_visible_count": sum(
            bool(row.get("nominal_stereo_visible")) for row in validation
        ) if explicit_rfu_points else len(validation),
    }
    return calibration, validation, manifest


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_mission_path(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    """Write north,east,altitude_agl_m ordered far-to-near for flight use.

    ``rows`` (calibration/validation) are i.i.d. random samples with no
    inherent order; the *_waypoints.csv files preserve that raw order for
    calibration analysis. This file is the one meant to be flown (e.g. via
    px4_auto_mission.py --path-file), so it's sorted by descending range --
    each row is still an individually FOV/margin-verified point, just
    sequenced into an approach instead of a scatter.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    if rows and all("source_order" in row for row in rows):
        ordered = sorted(rows, key=lambda row: int(row["source_order"]))
    else:
        ordered = sorted(rows, key=lambda row: float(row["baseline_forward_m"]), reverse=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("north,east,altitude_agl_m\n")
        for row in ordered:
            handle.write(
                f"{float(row['north_m']):.6f},"
                f"{float(row['east_m']):.6f},"
                f"{float(row['altitude_agl_m']):.6f}\n"
            )


def main() -> int:
    args = parse_args()
    calibration, validation, manifest = generate_waypoints(args)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "calibration_waypoints.csv", calibration)
    write_csv(args.output_dir / "validation_waypoints.csv", validation)
    write_mission_path(args.output_dir / "calibration_mission.csv", calibration)
    write_mission_path(args.output_dir / "validation_mission.csv", validation)
    (args.output_dir / "waypoint_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"wrote FOV-constrained waypoint sets to {args.output_dir}")
    print(
        f"calibration={len(calibration)} "
        f"validation={len(validation)} "
        f"disjoint={manifest['sets_disjoint']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
