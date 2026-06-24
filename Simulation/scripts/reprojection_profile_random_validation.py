#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import rclpy

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
PROJECT_DIR = ROOT_DIR.parent


def _find_stereo_distance_app() -> Path:
    if os.environ.get("STEREO_DISTANCE_APP"):
        return Path(os.environ["STEREO_DISTANCE_APP"]).expanduser()
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "StereoDistance"
        if (candidate / "wiris_stereo_distance").is_dir():
            return candidate
    return PROJECT_DIR / "StereoDistance"


APP_DIR = _find_stereo_distance_app()
sys.path.insert(0, str(PROJECT_DIR))
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(APP_DIR))

import bbox_profile_random_validation as validation  # noqa: E402
import origin_bbox_output_profile_train as train  # noqa: E402
from wiris_stereo_distance.simulation_origin import (  # noqa: E402
    build_origin_bbox_reprojection_profile_from_telemetry,
    evaluate_origin_bbox_reprojection_profile_rows,
    load_telemetry_rows,
    save_reprojection_profile,
    simulation_comparison_rows,
    write_dynamic_csv,
    write_json,
)
from wiris_stereo_distance.reprojection_profile import (  # noqa: E402
    DEFAULT_REPROJECTION_CORRECTION_MODE,
    DEFAULT_REPROJECTION_DISPARITY_GAIN,
    DEFAULT_REPROJECTION_OUTPUT_POLICY,
    DEFAULT_REPROJECTION_SCREEN_U_GAIN,
    DEFAULT_REPROJECTION_SCREEN_V_GAIN,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Collect one coverage Origin-BBOX Reprojection Profile trajectory and five "
            "random validation trajectories, then gate RDF axis RMSE improvement."
        )
    )
    parser.add_argument("--seed", type=int, default=20260622)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--warmup-s", type=float, default=12.0)
    parser.add_argument("--settle-s", type=float, default=3.0)
    parser.add_argument("--trial-duration-s", type=float, default=18.0)
    parser.add_argument("--profile-trial-duration-s", type=float)
    parser.add_argument("--analysis-skip-s", type=float, default=3.0)
    parser.add_argument("--command-hz", type=float, default=2.0)
    parser.add_argument("--sample-hz", type=float, default=12.0)
    parser.add_argument("--min-samples-per-trial", type=int, default=40)
    parser.add_argument("--baseline-m", type=float, default=25.0)
    parser.add_argument("--camera-height-m", type=float, default=10.0)
    parser.add_argument("--horizontal-fov-rad", type=float, default=0.154997710698)
    parser.add_argument("--image-width-px", type=int, default=1280)
    parser.add_argument("--image-height-px", type=int, default=720)
    parser.add_argument("--camera-update-rate-hz", type=float, default=30.0)
    parser.add_argument("--camera-clip-near-m", type=float, default=0.1)
    parser.add_argument("--camera-clip-far-m", type=float, default=4000.0)
    parser.add_argument("--plot-publish-hz", type=float, default=30.0)
    parser.add_argument("--actual-distance-read-hz", type=float, default=15.0)
    parser.add_argument("--yolo-model", default=validation.DEFAULT_YOLO_MODEL)
    parser.add_argument("--yolo-class", default="UAV")
    parser.add_argument("--yolo-confidence", type=float, default=0.10)
    parser.add_argument("--yolo-device", default="cuda:0")
    parser.add_argument("--yolo-target-hz", type=float, default=40.0)
    parser.add_argument("--yolo-imgsz", type=int, default=1280)
    parser.add_argument("--yolo-python", default=sys.executable)
    parser.add_argument(
        "--yolo-pythonpath",
        default="",
    )
    parser.add_argument("--target-model-name", default="x500_15m")
    parser.add_argument("--target-yaw-deg", type=float, default=-90.0)
    parser.add_argument("--left-roll-error-deg", type=float, default=0.0)
    parser.add_argument("--left-pitch-error-deg", type=float, default=0.0)
    parser.add_argument("--left-yaw-error-deg", type=float, default=0.0)
    parser.add_argument("--right-roll-error-deg", type=float, default=0.0)
    parser.add_argument("--right-pitch-error-deg", type=float, default=0.0)
    parser.add_argument("--right-yaw-error-deg", type=float, default=0.0)
    parser.add_argument("--forward-min-m", type=float, default=260.0)
    parser.add_argument("--forward-max-m", type=float, default=1500.0)
    parser.add_argument("--altitude-min-m", type=float, default=5.8)
    parser.add_argument("--altitude-max-m", type=float, default=13.5)
    parser.add_argument("--overlap-margin", type=float, default=0.55)
    parser.add_argument(
        "--validation-trajectory-mode",
        choices=("random", "screen_coverage"),
        default="random",
        help="random preserves historical validation; screen_coverage intentionally covers 3x3 screen bins.",
    )
    parser.add_argument("--validation-screen-u-fracs", default="0.31,0.50,0.69")
    parser.add_argument("--validation-screen-v-fracs", default="0.31,0.50,0.69")
    parser.add_argument(
        "--validation-ranges-m",
        default="",
        help="Optional range list for screen_coverage validation. Defaults to --profile-ranges-m.",
    )
    parser.add_argument("--validation-altitude-min-m", type=float)
    parser.add_argument("--validation-altitude-max-m", type=float)
    parser.add_argument("--validation-waypoint-repeat", type=int, default=1)
    parser.add_argument("--validation-min-waypoints-per-trial", type=int, default=4)
    parser.add_argument(
        "--trajectory-diagnostics-only",
        action="store_true",
        help="Write trajectory/projection diagnostics and exit before launching ROS/Gazebo.",
    )
    parser.add_argument("--profile-u-bins", type=int, default=3)
    parser.add_argument("--profile-v-bins", type=int, default=3)
    parser.add_argument("--profile-depth-bins", type=int, default=4)
    parser.add_argument("--profile-min-cell-samples", type=int, default=3)
    parser.add_argument("--profile-lookup-policy", choices=("exact", "nearest"), default="exact")
    parser.add_argument(
        "--profile-correction-mode",
        choices=("full", "disparity_only", "component_gains"),
        default=DEFAULT_REPROJECTION_CORRECTION_MODE,
    )
    parser.add_argument("--profile-disparity-gain", type=float, default=DEFAULT_REPROJECTION_DISPARITY_GAIN)
    parser.add_argument("--profile-screen-u-gain", type=float, default=DEFAULT_REPROJECTION_SCREEN_U_GAIN)
    parser.add_argument("--profile-screen-v-gain", type=float, default=DEFAULT_REPROJECTION_SCREEN_V_GAIN)
    parser.add_argument(
        "--profile-output-policy",
        choices=("triangulated", "range_only_forward_fusion"),
        default=DEFAULT_REPROJECTION_OUTPUT_POLICY,
    )
    parser.add_argument("--profile-screen-u-fracs", default="0.31,0.50,0.69")
    parser.add_argument("--profile-screen-v-fracs", default="0.31,0.50,0.69")
    parser.add_argument("--profile-ranges-m", default="300,450,650,950,1300")
    parser.add_argument("--profile-altitude-min-m", type=float, default=1.0)
    parser.add_argument("--profile-altitude-max-m", type=float, default=25.0)
    parser.add_argument("--profile-waypoint-repeat", type=int, default=1)
    parser.add_argument(
        "--save-roi-images",
        action="store_true",
        help="Save left/right bbox ROI crops alongside telemetry rows for image-based method testing.",
    )
    parser.add_argument(
        "--roi-output-dir",
        default="",
        help="Optional ROI crop directory. Defaults to <output-dir>/roi_images when --save-roi-images is set.",
    )
    parser.add_argument("--roi-save-stride", type=int, default=12)
    parser.add_argument("--roi-crop-margin", type=float, default=0.25)
    parser.add_argument("--roi-jpeg-quality", type=int, default=90)
    parser.add_argument(
        "--roi-image-format",
        choices=("ppm", "jpg"),
        default="ppm",
        help="PPM avoids OpenCV/NumPy ABI issues; JPG uses OpenCV when available.",
    )
    return parser.parse_args()


def launch_args(args: argparse.Namespace, first_point: tuple[float, float, float]) -> list[str]:
    command = train.launch_args(args, first_point)
    replacements = {
        "left_roll_error_deg:=0.0": f"left_roll_error_deg:={args.left_roll_error_deg}",
        "left_pitch_error_deg:=0.0": f"left_pitch_error_deg:={args.left_pitch_error_deg}",
        "left_yaw_error_deg:=0.0": f"left_yaw_error_deg:={args.left_yaw_error_deg}",
        "right_roll_error_deg:=0.0": f"right_roll_error_deg:={args.right_roll_error_deg}",
        "right_pitch_error_deg:=0.0": f"right_pitch_error_deg:={args.right_pitch_error_deg}",
        "right_yaw_error_deg:=0.0": f"right_yaw_error_deg:={args.right_yaw_error_deg}",
    }
    return [replacements.get(item, item) for item in command]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_float_list(text: str) -> list[float]:
    return [float(part.strip()) for part in str(text).split(",") if part.strip()]


def focal_length_px(args: argparse.Namespace) -> float:
    return (float(args.image_width_px) * 0.5) / math.tan(float(args.horizontal_fov_rad) * 0.5)


def rotation_matrix_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[tuple[float, float, float], ...]:
    cr = math.cos(roll_rad)
    sr = math.sin(roll_rad)
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def transpose_apply(matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
        matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
        matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2],
    )


def camera_error_rpy(args: argparse.Namespace, side: str) -> tuple[float, float, float]:
    if side == "left":
        return (
            math.radians(float(args.left_roll_error_deg)),
            math.radians(float(args.left_pitch_error_deg)),
            math.radians(float(args.left_yaw_error_deg)),
        )
    return (
        math.radians(float(args.right_roll_error_deg)),
        math.radians(float(args.right_pitch_error_deg)),
        math.radians(float(args.right_yaw_error_deg)),
    )


def project_point_with_attitude(
    point: tuple[float, float, float],
    args: argparse.Namespace,
    *,
    side: str,
    enforce_clip: bool = True,
) -> dict[str, float] | None:
    world_x, world_y, world_z = point
    camera_x = -float(args.baseline_m) * 0.5 if side == "left" else float(args.baseline_m) * 0.5
    camera_y = 0.0
    camera_z = float(args.camera_height_m)
    roll, pitch, yaw_error = camera_error_rpy(args, side)
    world_from_link = rotation_matrix_rpy(roll, pitch, math.pi * 0.5 + yaw_error)
    link_x, link_y, link_z = transpose_apply(
        world_from_link,
        (float(world_x) - camera_x, float(world_y) - camera_y, float(world_z) - camera_z),
    )
    # Gazebo camera link +X points forward at nominal yaw=pi/2; image right is -link Y,
    # image down is -link Z. This maps zero-error projection to the stereo FRD contract.
    optical_right = -link_y
    optical_down = -link_z
    optical_forward = link_x
    if not math.isfinite(optical_forward) or optical_forward <= float(args.camera_clip_near_m):
        return None
    if enforce_clip and optical_forward > float(args.camera_clip_far_m):
        return None
    fx = focal_length_px(args)
    u = float(args.image_width_px) * 0.5 + optical_right * fx / optical_forward
    v = float(args.image_height_px) * 0.5 + optical_down * fx / optical_forward
    return {
        "u": u,
        "v": v,
        "right": optical_right,
        "down": optical_down,
        "forward": optical_forward,
    }


def attitude_projection(
    point: tuple[float, float, float],
    args: argparse.Namespace,
    *,
    enforce_clip: bool = True,
) -> dict[str, float] | None:
    left = project_point_with_attitude(point, args, side="left", enforce_clip=enforce_clip)
    right = project_point_with_attitude(point, args, side="right", enforce_clip=enforce_clip)
    if left is None or right is None:
        return None
    return {
        "left_u": left["u"],
        "left_v": left["v"],
        "right_u": right["u"],
        "right_v": right["v"],
        "screen_u": (left["u"] + right["u"]) * 0.5,
        "screen_v": (left["v"] + right["v"]) * 0.5,
        "disparity_px": abs(left["u"] - right["u"]),
        "signed_disparity_px": left["u"] - right["u"],
        "left_forward_m": left["forward"],
        "right_forward_m": right["forward"],
    }


def projected_center_inside_image(projection: dict[str, float], args: argparse.Namespace) -> bool:
    return (
        0.0 <= projection["left_u"] <= float(args.image_width_px)
        and 0.0 <= projection["left_v"] <= float(args.image_height_px)
        and 0.0 <= projection["right_u"] <= float(args.image_width_px)
        and 0.0 <= projection["right_v"] <= float(args.image_height_px)
    )


def point_in_attitude_common_fov(
    point: tuple[float, float, float],
    args: argparse.Namespace,
    *,
    altitude_min_m: float,
    altitude_max_m: float,
) -> bool:
    _right, _forward, altitude = point
    if not (float(altitude_min_m) <= altitude <= float(altitude_max_m)):
        return False
    projection = attitude_projection(point, args)
    return projection is not None and projected_center_inside_image(projection, args)


def profile_point_in_common_fov(point: tuple[float, float, float], args: argparse.Namespace) -> bool:
    return point_in_attitude_common_fov(
        point,
        args,
        altitude_min_m=float(args.profile_altitude_min_m),
        altitude_max_m=float(args.profile_altitude_max_m),
    )


def waypoint_for_screen_fraction(
    *,
    u_frac: float,
    v_frac: float,
    forward_m: float,
    args: argparse.Namespace,
    altitude_min_m: float | None = None,
    altitude_max_m: float | None = None,
) -> tuple[float, float, float] | None:
    fx = focal_length_px(args)
    image_cx = float(args.image_width_px) * 0.5
    image_cy = float(args.image_height_px) * 0.5
    target_u = float(args.image_width_px) * float(u_frac)
    target_v = float(args.image_height_px) * float(v_frac)
    right = (target_u - image_cx) * float(forward_m) / fx
    down = (target_v - image_cy) * float(forward_m) / fx
    altitude = float(args.camera_height_m) - down
    point = (right, float(forward_m), altitude)
    for _ in range(8):
        projection = attitude_projection(point, args)
        if projection is None:
            break
        du = target_u - projection["screen_u"]
        dv = target_v - projection["screen_v"]
        if abs(du) < 0.25 and abs(dv) < 0.25:
            break
        right += du * float(forward_m) / fx
        altitude -= dv * float(forward_m) / fx
        point = (right, float(forward_m), altitude)
    altitude_low = float(args.profile_altitude_min_m) if altitude_min_m is None else float(altitude_min_m)
    altitude_high = float(args.profile_altitude_max_m) if altitude_max_m is None else float(altitude_max_m)
    if not point_in_attitude_common_fov(point, args, altitude_min_m=altitude_low, altitude_max_m=altitude_high):
        return None
    return point


def coverage_profile_trajectory(args: argparse.Namespace) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    u_fracs = parse_float_list(args.profile_screen_u_fracs)
    v_fracs = parse_float_list(args.profile_screen_v_fracs)
    ranges = parse_float_list(args.profile_ranges_m)
    if not u_fracs or not v_fracs or not ranges:
        raise RuntimeError("profile screen fractions and ranges must not be empty")
    for range_index, forward in enumerate(ranges):
        v_order = v_fracs if range_index % 2 == 0 else list(reversed(v_fracs))
        for v_index, v_frac in enumerate(v_order):
            u_order = u_fracs if v_index % 2 == 0 else list(reversed(u_fracs))
            for u_frac in u_order:
                point = waypoint_for_screen_fraction(
                    u_frac=u_frac,
                    v_frac=v_frac,
                    forward_m=forward,
                    args=args,
                )
                if point is not None:
                    points.append(point)
    if len(points) < 9:
        raise RuntimeError(f"profile trajectory produced too few feasible image-space waypoints: {len(points)}")
    repeat = max(1, int(args.profile_waypoint_repeat))
    repeated: list[tuple[float, float, float]] = []
    for point in points:
        repeated.extend([point] * repeat)
    return repeated


def random_waypoint_attitude_aware(rng, args: argparse.Namespace) -> tuple[float, float, float]:
    for _ in range(500):
        forward = rng.uniform(float(args.forward_min_m), float(args.forward_max_m))
        u_frac = rng.uniform(0.25, 0.75)
        v_frac = rng.uniform(0.25, 0.75)
        point = waypoint_for_screen_fraction(
            u_frac=u_frac,
            v_frac=v_frac,
            forward_m=forward,
            args=args,
            altitude_min_m=float(args.altitude_min_m),
            altitude_max_m=float(args.altitude_max_m),
        )
        if point is not None:
            return point
    raise RuntimeError("failed to sample an attitude-aware validation waypoint inside the common FOV")


def build_random_trajectory_attitude_aware(rng, args: argparse.Namespace) -> list[tuple[float, float, float]]:
    waypoint_count = rng.randint(4, 6)
    points = [random_waypoint_attitude_aware(rng, args) for _ in range(waypoint_count)]
    points.sort(key=lambda item: item[1], reverse=True)
    return points


def validation_altitude_limits(args: argparse.Namespace) -> tuple[float, float]:
    low = float(args.altitude_min_m) if args.validation_altitude_min_m is None else float(args.validation_altitude_min_m)
    high = float(args.altitude_max_m) if args.validation_altitude_max_m is None else float(args.validation_altitude_max_m)
    return low, high


def screen_coverage_validation_trajectories(args: argparse.Namespace) -> list[list[tuple[float, float, float]]]:
    u_fracs = parse_float_list(args.validation_screen_u_fracs)
    v_fracs = parse_float_list(args.validation_screen_v_fracs)
    ranges = parse_float_list(args.validation_ranges_m) if str(args.validation_ranges_m).strip() else parse_float_list(
        args.profile_ranges_m
    )
    if not u_fracs or not v_fracs or not ranges:
        raise RuntimeError("validation screen fractions and ranges must not be empty")
    altitude_low, altitude_high = validation_altitude_limits(args)
    repeat = max(1, int(args.validation_waypoint_repeat))
    trajectories: list[list[tuple[float, float, float]]] = []
    for trial_index in range(5):
        forward = ranges[trial_index % len(ranges)]
        v_order = v_fracs if trial_index % 2 == 0 else list(reversed(v_fracs))
        points: list[tuple[float, float, float]] = []
        for v_index, v_frac in enumerate(v_order):
            u_order = u_fracs if v_index % 2 == 0 else list(reversed(u_fracs))
            for u_frac in u_order:
                point = waypoint_for_screen_fraction(
                    u_frac=u_frac,
                    v_frac=v_frac,
                    forward_m=forward,
                    args=args,
                    altitude_min_m=altitude_low,
                    altitude_max_m=altitude_high,
                )
                if point is not None:
                    points.extend([point] * repeat)
        if len(points) < int(args.validation_min_waypoints_per_trial):
            raise RuntimeError(
                "screen_coverage validation trajectory produced too few feasible waypoints "
                f"for trial {trial_index + 2}: {len(points)}"
            )
        trajectories.append(points)
    return trajectories


def build_validation_trajectories(rng, args: argparse.Namespace) -> list[list[tuple[float, float, float]]]:
    if args.validation_trajectory_mode == "screen_coverage":
        return screen_coverage_validation_trajectories(args)
    return [build_random_trajectory_attitude_aware(rng, args) for _ in range(5)]


def projection_bin(value: float, *, bins: int, size: float) -> int | None:
    if not math.isfinite(value) or value < 0.0 or value > size:
        return None
    index = int(value / size * bins)
    return max(0, min(bins - 1, index))


def trajectory_projection_diagnostics(
    trajectories: list[list[tuple[float, float, float]]],
    args: argparse.Namespace,
) -> dict[str, object]:
    trial_rows: list[dict[str, object]] = []
    all_screen_bins: set[str] = set()
    all_depth_bins: set[int] = set()
    validation_screen_bins: set[str] = set()
    validation_depth_bins: set[int] = set()
    for trial_id, trajectory in enumerate(trajectories, start=1):
        trial_screen_bins: set[str] = set()
        trial_depth_bins: set[int] = set()
        projected_count = 0
        for point in trajectory:
            projection = attitude_projection(point, args)
            if projection is None:
                continue
            projected_count += 1
            u_bin = projection_bin(
                projection["screen_u"],
                bins=int(args.profile_u_bins),
                size=float(args.image_width_px),
            )
            v_bin = projection_bin(
                projection["screen_v"],
                bins=int(args.profile_v_bins),
                size=float(args.image_height_px),
            )
            if u_bin is not None and v_bin is not None:
                key = f"{u_bin}:{v_bin}"
                trial_screen_bins.add(key)
                all_screen_bins.add(key)
                if trial_id >= 2:
                    validation_screen_bins.add(key)
            disparity = float(projection["disparity_px"])
            nominal = nominal_projection(point, args)
            # Use nominal disparity order as a coarse coverage diagnostic independent of profile fitting.
            nominal_disparity = float(nominal["disparity_px"])
            if math.isfinite(disparity) and math.isfinite(nominal_disparity):
                depth_bin = min(3, max(0, int(disparity / max(1.0, nominal_disparity) * 2.0)))
                trial_depth_bins.add(depth_bin)
                all_depth_bins.add(depth_bin)
                if trial_id >= 2:
                    validation_depth_bins.add(depth_bin)
        trial_rows.append(
            {
                "trial": trial_id,
                "waypoint_count": len(trajectory),
                "projected_count": projected_count,
                "screen_bins": sorted(trial_screen_bins),
                "screen_bin_count": len(trial_screen_bins),
                "depth_bins": sorted(trial_depth_bins),
            }
        )
    total_screen_bins = int(args.profile_u_bins) * int(args.profile_v_bins)
    return {
        "trajectory_mode": args.validation_trajectory_mode,
        "trial_rows": trial_rows,
        "all_screen_bins": sorted(all_screen_bins),
        "all_screen_bin_count": len(all_screen_bins),
        "validation_screen_bins": sorted(validation_screen_bins),
        "validation_screen_bin_count": len(validation_screen_bins),
        "screen_bin_total": total_screen_bins,
        "screen_bin_coverage_ratio": len(all_screen_bins) / total_screen_bins if total_screen_bins else 0.0,
        "validation_screen_bin_coverage_ratio": (
            len(validation_screen_bins) / total_screen_bins if total_screen_bins else 0.0
        ),
        "all_depth_bins": sorted(all_depth_bins),
        "validation_depth_bins": sorted(validation_depth_bins),
    }


def nominal_projection(point: tuple[float, float, float], args: argparse.Namespace) -> dict[str, float]:
    right, forward, altitude = point
    fx = focal_length_px(args)
    down = float(args.camera_height_m) - float(altitude)
    left_u = float(args.image_width_px) * 0.5 + (float(right) + float(args.baseline_m) * 0.5) * fx / float(forward)
    right_u = float(args.image_width_px) * 0.5 + (float(right) - float(args.baseline_m) * 0.5) * fx / float(forward)
    v = float(args.image_height_px) * 0.5 + down * fx / float(forward)
    return {
        "left_u": left_u,
        "left_v": v,
        "right_u": right_u,
        "right_v": v,
        "disparity_px": abs(left_u - right_u),
    }


def disparity_zero_diagnostic(args: argparse.Namespace) -> dict[str, object]:
    fx = focal_length_px(args)
    max_forward = max(float(args.camera_clip_far_m), float(args.forward_max_m), 20000.0)
    best: dict[str, float] | None = None
    best_unclipped: dict[str, float] | None = None
    previous: tuple[float, float] | None = None
    previous_unclipped: tuple[float, float] | None = None
    sign_crossing: float | None = None
    sign_crossing_unclipped: float | None = None
    for index in range(1, 4000):
        forward = float(args.camera_clip_near_m) + (max_forward - float(args.camera_clip_near_m)) * index / 3999.0
        point = (0.0, forward, float(args.camera_height_m))
        projection = attitude_projection(point, args)
        if projection is None:
            projection_unclipped = attitude_projection(point, args, enforce_clip=False)
        else:
            signed = projection["signed_disparity_px"]
            abs_disp = abs(signed)
            if best is None or abs_disp < best["abs_disparity_px"]:
                best = {
                    "forward_m": forward,
                    "signed_disparity_px": signed,
                    "abs_disparity_px": abs_disp,
                    "left_u": projection["left_u"],
                    "right_u": projection["right_u"],
                }
            if previous is not None and previous[1] * signed <= 0.0 and sign_crossing is None:
                prev_forward, prev_signed = previous
                denom = signed - prev_signed
                sign_crossing = forward if abs(denom) < 1.0e-12 else prev_forward - prev_signed * (forward - prev_forward) / denom
            previous = (forward, signed)
            projection_unclipped = projection
        if projection_unclipped is None:
            continue
        signed_unclipped = projection_unclipped["signed_disparity_px"]
        abs_unclipped = abs(signed_unclipped)
        if best_unclipped is None or abs_unclipped < best_unclipped["abs_disparity_px"]:
            best_unclipped = {
                "forward_m": forward,
                "signed_disparity_px": signed_unclipped,
                "abs_disparity_px": abs_unclipped,
                "left_u": projection_unclipped["left_u"],
                "right_u": projection_unclipped["right_u"],
            }
        if previous_unclipped is not None and previous_unclipped[1] * signed_unclipped <= 0.0 and sign_crossing_unclipped is None:
            prev_forward, prev_signed = previous_unclipped
            denom = signed_unclipped - prev_signed
            sign_crossing_unclipped = (
                forward if abs(denom) < 1.0e-12 else prev_forward - prev_signed * (forward - prev_forward) / denom
            )
        previous_unclipped = (forward, signed_unclipped)
    toe_in_deg = max(abs(float(args.left_yaw_error_deg)), abs(float(args.right_yaw_error_deg)))
    symmetric_zero_forward = None
    symmetric_required_nominal_disparity = None
    if toe_in_deg > 0.0:
        alpha = math.radians(toe_in_deg)
        symmetric_zero_forward = float(args.baseline_m) / (2.0 * math.tan(alpha))
        symmetric_required_nominal_disparity = 2.0 * fx * math.tan(alpha)
    current_right_only_zero_forward = None
    if abs(float(args.right_yaw_error_deg)) > 0.0 and abs(float(args.left_yaw_error_deg)) < 1.0e-12:
        alpha = math.radians(abs(float(args.right_yaw_error_deg)))
        current_right_only_zero_forward = float(args.baseline_m) / math.tan(alpha)
    point_1000 = (0.0, 1000.0, float(args.camera_height_m))
    actual_1000 = attitude_projection(point_1000, args)
    nominal_1000 = nominal_projection(point_1000, args)
    du_dv_1000 = None
    if actual_1000 is not None:
        du_dv_1000 = {
            "du_left_px": nominal_1000["left_u"] - actual_1000["left_u"],
            "dv_left_px": nominal_1000["left_v"] - actual_1000["left_v"],
            "du_right_px": nominal_1000["right_u"] - actual_1000["right_u"],
            "dv_right_px": nominal_1000["right_v"] - actual_1000["right_v"],
            "actual_signed_disparity_px": actual_1000["signed_disparity_px"],
            "nominal_disparity_px": nominal_1000["disparity_px"],
            "required_disparity_opening_px": nominal_1000["disparity_px"] - abs(actual_1000["signed_disparity_px"]),
        }
    right_only_zero_du_dv = None
    if current_right_only_zero_forward is not None:
        point_zero = (0.0, current_right_only_zero_forward, float(args.camera_height_m))
        actual_zero = attitude_projection(point_zero, args, enforce_clip=False)
        nominal_zero = nominal_projection(point_zero, args)
        if actual_zero is not None:
            right_only_zero_du_dv = {
                "forward_m": current_right_only_zero_forward,
                "du_left_px": nominal_zero["left_u"] - actual_zero["left_u"],
                "dv_left_px": nominal_zero["left_v"] - actual_zero["left_v"],
                "du_right_px": nominal_zero["right_u"] - actual_zero["right_u"],
                "dv_right_px": nominal_zero["right_v"] - actual_zero["right_v"],
                "actual_signed_disparity_px": actual_zero["signed_disparity_px"],
                "nominal_disparity_px": nominal_zero["disparity_px"],
                "required_disparity_opening_px": nominal_zero["disparity_px"] - abs(actual_zero["signed_disparity_px"]),
            }
    symmetric_yaw_only_zero_du_dv = None
    if symmetric_zero_forward is not None:
        alpha = math.radians(toe_in_deg)
        symmetric_yaw_only_zero_du_dv = {
            "forward_m": symmetric_zero_forward,
            "du_left_px": fx * math.tan(alpha),
            "dv_left_px": 0.0,
            "du_right_px": -fx * math.tan(alpha),
            "dv_right_px": 0.0,
            "actual_signed_disparity_px": 0.0,
            "nominal_disparity_px": symmetric_required_nominal_disparity,
            "required_disparity_opening_px": symmetric_required_nominal_disparity,
        }
    return {
        "model": "SDF pose Rz(pi/2+yaw)*Ry(pitch)*Rx(roll), optical=(right=-link_y, down=-link_z, forward=link_x)",
        "best_centerline_disparity_search": best,
        "centerline_signed_disparity_zero_crossing_forward_m": sign_crossing,
        "best_centerline_disparity_unclipped_search": best_unclipped,
        "centerline_signed_disparity_zero_crossing_unclipped_forward_m": sign_crossing_unclipped,
        "symmetric_toe_in_zero_forward_m": symmetric_zero_forward,
        "symmetric_toe_in_required_nominal_disparity_px_at_zero": symmetric_required_nominal_disparity,
        "symmetric_yaw_only_zero_du_dv_actual_to_nominal": symmetric_yaw_only_zero_du_dv,
        "right_only_toe_in_zero_forward_m": current_right_only_zero_forward,
        "right_only_zero_du_dv_actual_to_nominal": right_only_zero_du_dv,
        "du_dv_centerline_1000m_actual_to_nominal": du_dv_1000,
    }


def collect_rows(
    args: argparse.Namespace,
    output_dir: Path,
    trajectories: list[list[tuple[float, float, float]]],
) -> dict[int, list[dict[str, object]]]:
    first_point = trajectories[0][0]
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = env.get("ROS_LOG_DIR", "/tmp/ros2-origin-bbox-reprojection-validation-log")
    env["GZ_CONFIG_HOME"] = env.get("GZ_CONFIG_HOME", "/tmp/gz-origin-bbox-reprojection-validation-config")
    model_path = str(ROOT_DIR / "install/stereo_ground_sim/share/stereo_ground_sim/models")
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        item for item in [model_path, env.get("GZ_SIM_RESOURCE_PATH", "")] if item
    )
    if args.yolo_pythonpath and Path(args.yolo_pythonpath).is_dir():
        env["PYTHONPATH"] = os.pathsep.join(
            item for item in [args.yolo_pythonpath, env.get("PYTHONPATH", "")] if item
        )

    validation.kill_matching_processes("stereo_ground_drone_attitude_error.sdf")
    validation.kill_matching_processes("ros2 launch stereo_ground_sim stereo_ground_sim.launch.py")
    sim, sim_log = validation.run_process(launch_args(args, first_point), output_dir / "simulation.log", env)

    rclpy.init(args=None)
    roi_output_dir = None
    if args.save_roi_images:
        roi_output_dir = Path(args.roi_output_dir) if args.roi_output_dir else output_dir / "roi_images"
    recorder = train.CalibrationRecorder(
        args.sample_hz,
        roi_output_dir=roi_output_dir,
        roi_save_stride=int(args.roi_save_stride) if args.save_roi_images else 0,
        roi_crop_margin=float(args.roi_crop_margin),
        roi_jpeg_quality=int(args.roi_jpeg_quality),
        roi_image_format=str(args.roi_image_format),
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()
    try:
        time.sleep(args.warmup_s)
        if sim.poll() is not None:
            raise RuntimeError(f"simulation exited during warmup with code {sim.returncode}")
        period = 1.0 / max(0.1, args.command_hz)
        for trial_id, trajectory in enumerate(trajectories, start=1):
            trial_duration_s = (
                float(args.profile_trial_duration_s)
                if trial_id == 1 and args.profile_trial_duration_s is not None
                else float(args.trial_duration_s)
            )
            print(f"trial {trial_id}/{len(trajectories)}: waypoints={len(trajectory)}", flush=True)
            validation.command_target_pose(args.target_model_name, trajectory[0], args.target_yaw_deg)
            time.sleep(args.settle_s)
            recorder.begin_trial(trial_id)
            started = time.monotonic()
            while True:
                elapsed = time.monotonic() - started
                if elapsed >= trial_duration_s:
                    break
                if sim.poll() is not None:
                    raise RuntimeError(f"simulation exited early with code {sim.returncode}")
                point = validation.interpolate(trajectory, elapsed / trial_duration_s)
                validation.command_target_pose(args.target_model_name, point, args.target_yaw_deg)
                time.sleep(period)
            validation.command_target_pose(args.target_model_name, trajectory[-1], args.target_yaw_deg)
            recorder.end_trial()
    finally:
        recorder.end_trial()
        snapshot = recorder.snapshot()
        recorder.destroy_node()
        rclpy.shutdown()
        validation.stop_process(sim)
        validation.kill_matching_processes("stereo_ground_drone_attitude_error.sdf")
        sim_log.close()

    filtered: dict[int, list[dict[str, object]]] = {}
    for trial_id, rows in snapshot.items():
        usable = [row for row in rows if float(row["trial_t_s"]) >= args.analysis_skip_s]
        if len(usable) >= args.min_samples_per_trial:
            filtered[int(trial_id)] = usable
    missing = [trial for trial in range(1, len(trajectories) + 1) if trial not in filtered]
    if missing:
        raise RuntimeError(f"insufficient usable samples for trial(s): {missing}")
    return filtered


def normalize_row(row: dict[str, object]) -> dict[str, object]:
    normalized = dict(row)
    left_cx = float(row["left_cx_px"])
    left_cy = float(row["left_cy_px"])
    right_cx = float(row["right_cx_px"])
    right_cy = float(row["right_cy_px"])
    est_right = float(row["estimated_right_m"])
    est_down = float(row["estimated_down_m"])
    est_forward = float(row["estimated_forward_m"])
    raw_range = math.sqrt(est_right * est_right + est_down * est_down + est_forward * est_forward)
    normalized.update(
        {
            "axis_contract": "stereo_center_frd:forward_right_down",
            "truth_right_m": row["gt_right_m"],
            "truth_down_m": row["gt_down_m"],
            "truth_forward_m": row["gt_forward_m"],
            "left_cx": row["left_cx_px"],
            "left_cy": row["left_cy_px"],
            "right_cx": row["right_cx_px"],
            "right_cy": row["right_cy_px"],
            "left_width_px": f"{max(0.0, float(row['left_x2_px']) - float(row['left_x1_px'])):.9g}",
            "left_height_px": f"{max(0.0, float(row['left_y2_px']) - float(row['left_y1_px'])):.9g}",
            "right_width_px": f"{max(0.0, float(row['right_x2_px']) - float(row['right_x1_px'])):.9g}",
            "right_height_px": f"{max(0.0, float(row['right_y2_px']) - float(row['right_y1_px'])):.9g}",
            "raw_disparity_px": f"{abs(left_cx - right_cx):.9g}",
            "raw_range_m": f"{raw_range:.9g}",
            "estimated_range_m": f"{raw_range:.9g}",
        }
    )
    return normalized


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def rmse(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else math.nan


def axis_gate(evaluated: list[dict[str, object]], trial: int) -> dict[str, object]:
    result: dict[str, object] = {"trial": trial, "samples": len(evaluated), "pass": "yes"}
    for axis in ("right", "down", "forward"):
        raw = [float(row[f"raw_error_{axis}_m"]) for row in evaluated if str(row.get(f"raw_error_{axis}_m", "")).strip()]
        corrected = [
            float(row[f"origin_bbox_reprojection_error_{axis}_m"])
            for row in evaluated
            if str(row.get(f"origin_bbox_reprojection_error_{axis}_m", "")).strip()
        ]
        raw_rmse = rmse(raw)
        corrected_rmse = rmse(corrected)
        result[f"{axis}_estimated_rmse_m"] = raw_rmse
        result[f"{axis}_corrected_rmse_m"] = corrected_rmse
        result[f"{axis}_rmse_improvement_m"] = raw_rmse - corrected_rmse
        if not corrected or not corrected_rmse < raw_rmse:
            result["pass"] = "no"
    fallback_counts: dict[str, int] = {}
    for row in evaluated:
        reason = str(row.get("origin_bbox_reprojection_fallback_reason", ""))
        fallback_counts[reason] = fallback_counts.get(reason, 0) + 1
    result["fallback_counts"] = json.dumps(fallback_counts, sort_keys=True)
    return result


def bin_index(value: float, edges: list[float]) -> int | None:
    if value < edges[0] or value > edges[-1]:
        return None
    for index in range(len(edges) - 1):
        if edges[index] <= value < edges[index + 1] or (index == len(edges) - 2 and value <= edges[index + 1]):
            return index
    return None


def profile_coverage_summary(profile, rows: list[dict[str, object]]) -> dict[str, object]:
    observed_screen_bins: set[str] = set()
    observed_depth_bins: set[int] = set()
    observed_cells: set[str] = set()
    row_cell_counts: dict[str, int] = {}
    for row in rows:
        left_u = float(row["left_cx"])
        left_v = float(row["left_cy"])
        right_u = float(row["right_cx"])
        right_v = float(row["right_cy"])
        disparity = float(row["raw_disparity_px"])
        screen_u = (left_u + right_u) * 0.5
        screen_v = (left_v + right_v) * 0.5
        u_bin = bin_index(screen_u, profile.u_edges)
        v_bin = bin_index(screen_v, profile.v_edges)
        depth_bin = bin_index(disparity, profile.depth_edges)
        if u_bin is not None and v_bin is not None:
            observed_screen_bins.add(f"{u_bin}:{v_bin}")
        if depth_bin is not None:
            observed_depth_bins.add(depth_bin)
        if u_bin is not None and v_bin is not None and depth_bin is not None:
            key = f"{u_bin}:{v_bin}:{depth_bin}"
            observed_cells.add(key)
            row_cell_counts[key] = row_cell_counts.get(key, 0) + 1
    profile_screen_bins = {":".join(key.split(":")[:2]) for key in profile.cells}
    total_screen_bins = (len(profile.u_edges) - 1) * (len(profile.v_edges) - 1)
    return {
        "row_count": len(rows),
        "profile_cell_count": len(profile.cells),
        "observed_screen_bins": sorted(observed_screen_bins),
        "profile_screen_bins": sorted(profile_screen_bins),
        "screen_bin_coverage_count": len(profile_screen_bins),
        "screen_bin_total": total_screen_bins,
        "screen_bin_coverage_ratio": len(profile_screen_bins) / total_screen_bins if total_screen_bins else 0.0,
        "observed_depth_bins": sorted(observed_depth_bins),
        "profile_depth_bins": sorted({int(cell.get("depth_bin", -1)) for cell in profile.cells.values()}),
        "observed_cell_count": len(observed_cells),
        "row_cell_counts": row_cell_counts,
    }


def main() -> int:
    args = parse_args()
    output_dir = (
        Path(args.output_dir)
        if args.output_dir
        else ROOT_DIR / "reports" / f"origin_bbox_reprojection_validation_{dt.datetime.now():%Y%m%d_%H%M%S}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    rng = validation.random.Random(args.seed)
    trajectories = [coverage_profile_trajectory(args)] + build_validation_trajectories(rng, args)
    disparity_diagnostic = disparity_zero_diagnostic(args)
    trajectory_diagnostic = trajectory_projection_diagnostics(trajectories, args)
    metadata = {
        "created_at": dt.datetime.now().isoformat(timespec="seconds"),
        "seed": args.seed,
        "pass_rule": "validation trials 2-6 must each reduce Right, Down, and Forward RMSE versus estimated/raw",
        "trajectory_generation": (
            "attitude-aware: waypoints are sampled from left/right reprojection with configured "
            "camera roll/pitch/yaw errors, and accepted only when both projected centers are inside the image"
        ),
        "profile_trial": 1,
        "validation_trials": [2, 3, 4, 5, 6],
        "profile_bins": {
            "u": args.profile_u_bins,
            "v": args.profile_v_bins,
            "depth": args.profile_depth_bins,
            "min_cell_samples": args.profile_min_cell_samples,
            "lookup_policy": args.profile_lookup_policy,
            "correction_mode": args.profile_correction_mode,
            "disparity_gain": args.profile_disparity_gain,
            "screen_u_gain": args.profile_screen_u_gain,
            "screen_v_gain": args.profile_screen_v_gain,
            "output_policy": args.profile_output_policy,
            "screen_u_fracs": parse_float_list(args.profile_screen_u_fracs),
            "screen_v_fracs": parse_float_list(args.profile_screen_v_fracs),
            "ranges_m": parse_float_list(args.profile_ranges_m),
            "altitude_min_m": args.profile_altitude_min_m,
            "altitude_max_m": args.profile_altitude_max_m,
            "waypoint_repeat": args.profile_waypoint_repeat,
        },
        "validation_trajectory": {
            "mode": args.validation_trajectory_mode,
            "screen_u_fracs": parse_float_list(args.validation_screen_u_fracs),
            "screen_v_fracs": parse_float_list(args.validation_screen_v_fracs),
            "ranges_m": (
                parse_float_list(args.validation_ranges_m)
                if str(args.validation_ranges_m).strip()
                else parse_float_list(args.profile_ranges_m)
            ),
            "altitude_min_m": validation_altitude_limits(args)[0],
            "altitude_max_m": validation_altitude_limits(args)[1],
            "waypoint_repeat": args.validation_waypoint_repeat,
            "min_waypoints_per_trial": args.validation_min_waypoints_per_trial,
        },
        "attitude_error_deg": {
            "left_roll": args.left_roll_error_deg,
            "left_pitch": args.left_pitch_error_deg,
            "left_yaw": args.left_yaw_error_deg,
            "right_roll": args.right_roll_error_deg,
            "right_pitch": args.right_pitch_error_deg,
            "right_yaw": args.right_yaw_error_deg,
        },
        "disparity_zero_diagnostic": disparity_diagnostic,
        "roi_capture": {
            "enabled": bool(args.save_roi_images),
            "output_dir": str(Path(args.roi_output_dir) if args.roi_output_dir else output_dir / "roi_images"),
            "save_stride": int(args.roi_save_stride),
            "crop_margin": float(args.roi_crop_margin),
            "jpeg_quality": int(args.roi_jpeg_quality),
            "image_format": str(args.roi_image_format),
            "row_fields": ["left_roi_path", "right_roi_path"],
        },
        "trajectories": trajectories,
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "disparity_zero_diagnostic.json").write_text(
        json.dumps(disparity_diagnostic, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output_dir / "trajectory_projection_diagnostic.json").write_text(
        json.dumps(trajectory_diagnostic, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if args.trajectory_diagnostics_only:
        print(f"Trajectory diagnostics: {output_dir / 'trajectory_projection_diagnostic.json'}")
        print(
            "Validation screen coverage: "
            f"{trajectory_diagnostic['validation_screen_bin_count']}/{trajectory_diagnostic['screen_bin_total']} "
            f"({trajectory_diagnostic['validation_screen_bin_coverage_ratio']:.3f})"
        )
        return 0

    rows_by_trial = collect_rows(args, output_dir, trajectories)
    all_raw = [row for trial in sorted(rows_by_trial) for row in rows_by_trial[trial]]
    write_rows(output_dir / "raw_samples.csv", all_raw)

    normalized_dir = output_dir / "normalized"
    normalized_paths: dict[int, Path] = {}
    for trial in sorted(rows_by_trial):
        normalized = [normalize_row(row) for row in rows_by_trial[trial]]
        path = normalized_dir / f"trial_{trial:02d}.csv"
        write_rows(path, normalized)
        normalized_paths[trial] = path

    profile_path = output_dir / "origin_bbox_reprojection_profile.json"
    build_comparison = output_dir / "profile_build_comparison.csv"
    build_summary_path = output_dir / "profile_build_summary.json"
    profile, build_rows = build_origin_bbox_reprojection_profile_from_telemetry(
        normalized_paths[1],
        profile_id=f"coverage_trial01_seed{args.seed}",
        min_samples=args.min_samples_per_trial,
        image_width_px=args.image_width_px,
        image_height_px=args.image_height_px,
        baseline_m=args.baseline_m,
        u_bins=args.profile_u_bins,
        v_bins=args.profile_v_bins,
        depth_bins=args.profile_depth_bins,
        min_cell_samples=args.profile_min_cell_samples,
        bin_axis="disparity",
        lookup_policy=args.profile_lookup_policy,
        correction_mode=args.profile_correction_mode,
        disparity_gain=args.profile_disparity_gain,
        screen_u_gain=args.profile_screen_u_gain,
        screen_v_gain=args.profile_screen_v_gain,
        output_policy=args.profile_output_policy,
    )
    build_evaluated, build_summary = evaluate_origin_bbox_reprojection_profile_rows(
        build_rows,
        profile,
        image_width_px=args.image_width_px,
        image_height_px=args.image_height_px,
        baseline_m=args.baseline_m,
    )
    coverage_summary = profile_coverage_summary(
        profile,
        [normalize_row(row) for row in rows_by_trial[1]],
    )
    build_summary.update({f"profile_coverage_{key}": value for key, value in coverage_summary.items()})
    save_reprojection_profile(profile_path, profile)
    write_dynamic_csv(build_comparison, build_evaluated)
    write_json(build_summary_path, build_summary)
    write_json(output_dir / "profile_coverage_summary.json", coverage_summary)

    gate_rows = []
    final_pass = True
    for trial in range(2, 7):
        rows = simulation_comparison_rows(load_telemetry_rows(normalized_paths[trial]))
        evaluated, summary = evaluate_origin_bbox_reprojection_profile_rows(
            rows,
            profile,
            image_width_px=args.image_width_px,
            image_height_px=args.image_height_px,
            baseline_m=args.baseline_m,
        )
        write_dynamic_csv(output_dir / f"validation_trial_{trial:02d}_comparison.csv", evaluated)
        write_json(output_dir / f"validation_trial_{trial:02d}_summary.json", summary)
        gate = axis_gate(evaluated, trial)
        if gate["pass"] != "yes":
            final_pass = False
        gate_rows.append(gate)
    write_rows(output_dir / "validation_gate_summary.csv", gate_rows)
    (output_dir / "result.txt").write_text(
        f"final_pass={'yes' if final_pass else 'no'}\n"
        f"profile={profile_path}\n"
        f"summary={output_dir / 'validation_gate_summary.csv'}\n",
        encoding="utf-8",
    )
    print(f"Profile: {profile_path}")
    print(f"Summary: {output_dir / 'validation_gate_summary.csv'}")
    print(f"Final pass: {'yes' if final_pass else 'no'}")
    return 0 if final_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
