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
WORLD_TO_PX4_WAYPOINT_CONTRACT = (
    "simulation_px4_mission_contract: waypoint north/east/altitude_agl_m maps "
    "to Gazebo world x/y/z for the current PX4 SITL mission runner"
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
    parser.add_argument("--camera-baseline-m", type=float, default=25.0)
    parser.add_argument("--camera-height-m", type=float, default=0.5)
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
) -> dict[str, float | int | str]:
    usable_h = horizontal_fov_rad * 0.5 - fov_margin_rad
    usable_v = vertical_fov * 0.5 - fov_margin_rad
    if usable_h <= 0.0 or usable_v <= 0.0:
        raise ValueError("FOV margin leaves no usable angular envelope")

    for _ in range(max_attempts):
        forward = rng.uniform(min_range_m, max_range_m)
        right = rng.uniform(-math.tan(usable_h) * forward, math.tan(usable_h) * forward)
        down = rng.uniform(-math.tan(usable_v) * forward, math.tan(usable_v) * forward)
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
            # world is Gazebo ENU: world[0]=East, world[1]=North, world[2]=Up
            # (coordinate_frames.R_ENU_FROM_NED). PX4's north/east are NED, so
            # north_m must read world[1] and east_m must read world[0] -- not
            # the same index, despite the visual symmetry of the old code.
            "north_m": world[1],
            "east_m": world[0],
            "altitude_agl_m": world[2],
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

    calibration: list[dict[str, float | int | str]] = []
    for index in range(args.calibration_count):
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
            )
        )

    validation: list[dict[str, float | int | str]] = []
    for index in range(args.validation_count):
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
        "horizontal_fov_rad": args.horizontal_fov_rad,
        "vertical_fov_rad": v_fov,
        "fov_margin_rad": args.fov_margin_rad,
        "pixel_margin_px": args.pixel_margin_px,
        "range_min_m": args.min_range_m,
        "range_max_m": args.max_range_m,
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
