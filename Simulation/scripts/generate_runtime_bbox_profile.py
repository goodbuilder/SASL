#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import json
import math
import random
from pathlib import Path


DEFAULT_SOURCE_MATRIX = (
    "/home/moontaehyung/영상감시시스템/ref/Stereo_Camera_3D_Position_DKL/"
    "Cent_Bbox_Adj_Mtrx.csv"
)
DEFAULT_PROFILE_SOURCE = (
    "/home/moontaehyung/영상감시시스템/ref/Stereo_Camera_3D_Position_DKL/"
    "Gazebo_Sim/time (csv)/8x/pitch0_yaw0.csv"
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Generate a runtime-resolution bbox-center profile and verify that profile "
            "source samples plus random validation waypoints stay in the stereo overlap region."
        )
    )
    parser.add_argument("--source-matrix", default=DEFAULT_SOURCE_MATRIX)
    parser.add_argument("--profile-source-csv", default=DEFAULT_PROFILE_SOURCE)
    parser.add_argument("--source-width-px", type=float, default=1920.0)
    parser.add_argument("--source-height-px", type=float, default=1080.0)
    parser.add_argument("--runtime-width-px", type=float, default=1280.0)
    parser.add_argument("--runtime-height-px", type=float, default=720.0)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--output-name", default="Cent_Bbox_Adj_Mtrx_1280x720_runtime.csv")
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260609)
    parser.add_argument("--baseline-m", type=float, default=25.0)
    parser.add_argument("--camera-height-m", type=float, default=10.0)
    parser.add_argument("--horizontal-fov-rad", type=float, default=0.154997710698)
    parser.add_argument("--forward-min-m", type=float, default=260.0)
    parser.add_argument("--forward-max-m", type=float, default=1500.0)
    parser.add_argument("--altitude-min-m", type=float, default=5.8)
    parser.add_argument("--altitude-max-m", type=float, default=13.5)
    parser.add_argument(
        "--overlap-margin",
        type=float,
        default=0.55,
        help="Fraction of computed common FOV width/height used for validation waypoints.",
    )
    return parser.parse_args()


def read_matrix(path):
    rows = []
    with Path(path).open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row:
                continue
            rows.append([float(value) for value in row])
    if len(rows) != 6 or any(len(row) != 4 for row in rows):
        raise ValueError(f"expected a 6x4 matrix, got {len(rows)} rows from {path}")
    return rows


def write_matrix(path, matrix):
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        for row in matrix:
            writer.writerow([f"{value:.15g}" for value in row])


def map_runtime_point(block, x_rt, y_rt, args):
    sx = args.runtime_width_px / args.source_width_px
    sy = args.runtime_height_px / args.source_height_px
    x_src = x_rt / sx
    y_src = y_rt / sy
    matlab_x = args.source_width_px - x_src
    matlab_y = args.source_height_px - y_src
    vec = [matlab_x, matlab_y, 1.0]
    adjusted_matlab = [
        sum(block[row][col] * vec[col] for col in range(3)) + block[row][3]
        for row in range(2)
    ]
    adjusted_src_x = args.source_width_px - adjusted_matlab[0]
    adjusted_src_y = args.source_height_px - adjusted_matlab[1]
    return adjusted_src_x * sx, adjusted_src_y * sy


def transform_block(block, args):
    p00 = map_runtime_point(block, 0.0, 0.0, args)
    p10 = map_runtime_point(block, 1.0, 0.0, args)
    p01 = map_runtime_point(block, 0.0, 1.0, args)
    x_a = p10[0] - p00[0]
    x_b = p01[0] - p00[0]
    y_a = p10[1] - p00[1]
    y_b = p01[1] - p00[1]
    return [
        [x_a, x_b, 0.0, p00[0]],
        [y_a, y_b, 0.0, p00[1]],
        [0.0, 0.0, 1.0, 0.0],
    ]


def transform_matrix(source_matrix, args):
    left = transform_block(source_matrix[0:3], args)
    right = transform_block(source_matrix[3:6], args)
    return left + right


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number)


def range_summary(values):
    values = [float(value) for value in values if finite(value)]
    if not values:
        return {"min": None, "max": None}
    return {"min": min(values), "max": max(values)}


def read_profile_rows(path):
    rows = []
    with Path(path).open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                rows.append(
                    {
                        "gt_x": float(row["gt_x"]),
                        "gt_y": float(row["gt_y"]),
                        "gt_z": float(row["gt_z"]),
                        "left_u": float(row["cam1_cx"]),
                        "left_v": float(row["cam1_cy"]),
                        "right_u": float(row["cam2_cx"]),
                        "right_v": float(row["cam2_cy"]),
                    }
                )
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def profile_overlap_summary(path, args):
    path = Path(path)
    if not path.exists():
        return {"exists": False, "path": str(path)}
    rows = read_profile_rows(path)
    if not rows:
        return {"exists": True, "path": str(path), "samples": 0}

    sx = args.runtime_width_px / args.source_width_px
    sy = args.runtime_height_px / args.source_height_px
    scaled = []
    source_inside = 0
    runtime_inside = 0
    for row in rows:
        left_u_rt = row["left_u"] * sx
        left_v_rt = row["left_v"] * sy
        right_u_rt = row["right_u"] * sx
        right_v_rt = row["right_v"] * sy
        scaled.append((left_u_rt, left_v_rt, right_u_rt, right_v_rt))
        if (
            0.0 <= row["left_u"] <= args.source_width_px
            and 0.0 <= row["left_v"] <= args.source_height_px
            and 0.0 <= row["right_u"] <= args.source_width_px
            and 0.0 <= row["right_v"] <= args.source_height_px
        ):
            source_inside += 1
        if (
            0.0 <= left_u_rt <= args.runtime_width_px
            and 0.0 <= left_v_rt <= args.runtime_height_px
            and 0.0 <= right_u_rt <= args.runtime_width_px
            and 0.0 <= right_v_rt <= args.runtime_height_px
        ):
            runtime_inside += 1

    all_source_inside = source_inside == len(rows)
    all_runtime_inside = runtime_inside == len(rows)
    return {
        "exists": True,
        "path": str(path),
        "samples": len(rows),
        "source_image_px": [args.source_width_px, args.source_height_px],
        "runtime_image_px": [args.runtime_width_px, args.runtime_height_px],
        "source_to_runtime_scale": [sx, sy],
        "source_dual_centers_inside_image_samples": source_inside,
        "runtime_scaled_dual_centers_inside_image_samples": runtime_inside,
        "implicit_common_fov_by_dual_detection": all_source_inside and all_runtime_inside,
        "profile_generation_overlap_verdict": "pass" if all_source_inside and all_runtime_inside else "fail",
        "source_gt_ranges": {
            "gt_x": range_summary(row["gt_x"] for row in rows),
            "gt_y": range_summary(row["gt_y"] for row in rows),
            "gt_z": range_summary(row["gt_z"] for row in rows),
        },
        "runtime_scaled_bbox_ranges_px": {
            "left_u": range_summary(item[0] for item in scaled),
            "left_v": range_summary(item[1] for item in scaled),
            "right_u": range_summary(item[2] for item in scaled),
            "right_v": range_summary(item[3] for item in scaled),
        },
        "overlap_basis": (
            "Each profile sample has a detected bbox center in both camera images; after "
            "scaling to runtime resolution, every center remains inside the runtime image."
        ),
    }


def vertical_fov_rad(horizontal_fov_rad, width, height):
    return 2.0 * math.atan(math.tan(horizontal_fov_rad * 0.5) * (height / width))


def overlap_bounds(forward_m, args):
    half_h = math.tan(args.horizontal_fov_rad * 0.5) * forward_m
    left_min = -half_h - args.baseline_m * 0.5
    left_max = half_h - args.baseline_m * 0.5
    right_min = -half_h + args.baseline_m * 0.5
    right_max = half_h + args.baseline_m * 0.5
    common_min = max(left_min, right_min)
    common_max = min(left_max, right_max)
    half_v = math.tan(
        vertical_fov_rad(args.horizontal_fov_rad, args.runtime_width_px, args.runtime_height_px) * 0.5
    ) * forward_m
    z_min = args.camera_height_m - half_v
    z_max = args.camera_height_m + half_v
    return common_min, common_max, z_min, z_max


def point_in_common_fov(right_m, forward_m, altitude_m, args):
    right_min, right_max, altitude_min, altitude_max = overlap_bounds(forward_m, args)
    return right_min <= right_m <= right_max and altitude_min <= altitude_m <= altitude_max


def random_waypoint(rng, args):
    for _ in range(100):
        forward = rng.uniform(args.forward_min_m, args.forward_max_m)
        right_min, right_max, altitude_min, altitude_max = overlap_bounds(forward, args)
        if right_min >= right_max or altitude_min >= altitude_max:
            continue
        center_right = (right_min + right_max) * 0.5
        half_right = (right_max - right_min) * 0.5 * args.overlap_margin
        right = rng.uniform(center_right - half_right, center_right + half_right)
        z_low = max(args.altitude_min_m, args.camera_height_m - (altitude_max - altitude_min) * 0.25)
        z_high = min(args.altitude_max_m, args.camera_height_m + (altitude_max - altitude_min) * 0.25)
        if z_low > z_high:
            z_low = max(args.altitude_min_m, altitude_min)
            z_high = min(args.altitude_max_m, altitude_max)
        if z_low > z_high:
            continue
        altitude = rng.uniform(z_low, z_high)
        if point_in_common_fov(right, forward, altitude, args):
            return right, forward, altitude
    raise RuntimeError("failed to sample a waypoint inside the common FOV")


def build_random_trajectory(rng, args):
    waypoint_count = rng.randint(4, 6)
    points = [random_waypoint(rng, args) for _ in range(waypoint_count)]
    points.sort(key=lambda item: item[1], reverse=True)
    return points


def validation_overlap_summary(args):
    rng = random.Random(args.seed)
    trajectories = [build_random_trajectory(rng, args) for _ in range(args.trials)]
    rows = []
    for trial_id, trajectory in enumerate(trajectories, start=1):
        for waypoint_id, point in enumerate(trajectory, start=1):
            right, forward, altitude = point
            right_min, right_max, altitude_min, altitude_max = overlap_bounds(forward, args)
            rows.append(
                {
                    "trial": trial_id,
                    "waypoint": waypoint_id,
                    "right_m": right,
                    "forward_m": forward,
                    "altitude_m": altitude,
                    "right_min_m": right_min,
                    "right_max_m": right_max,
                    "altitude_min_m": altitude_min,
                    "altitude_max_m": altitude_max,
                    "inside_common_fov": point_in_common_fov(right, forward, altitude, args),
                }
            )
    all_inside = all(row["inside_common_fov"] for row in rows)
    return {
        "random_seed": args.seed,
        "trials": args.trials,
        "waypoints": len(rows),
        "all_waypoints_inside_common_fov": all_inside,
        "validation_overlap_verdict": "pass" if all_inside else "fail",
        "runtime_common_fov_model": {
            "baseline_m": args.baseline_m,
            "camera_height_m": args.camera_height_m,
            "horizontal_fov_rad": args.horizontal_fov_rad,
            "horizontal_fov_deg": math.degrees(args.horizontal_fov_rad),
            "vertical_fov_rad": vertical_fov_rad(
                args.horizontal_fov_rad, args.runtime_width_px, args.runtime_height_px
            ),
            "runtime_image_px": [args.runtime_width_px, args.runtime_height_px],
            "overlap_margin": args.overlap_margin,
        },
        "waypoint_ranges": {
            "right_m": range_summary(row["right_m"] for row in rows),
            "forward_m": range_summary(row["forward_m"] for row in rows),
            "altitude_m": range_summary(row["altitude_m"] for row in rows),
        },
        "trajectories": trajectories,
        "waypoint_rows": rows,
    }


def write_waypoint_csv(path, rows):
    fieldnames = [
        "trial",
        "waypoint",
        "right_m",
        "forward_m",
        "altitude_m",
        "right_min_m",
        "right_max_m",
        "altitude_min_m",
        "altitude_max_m",
        "inside_common_fov",
    ]
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: (f"{value:.9g}" if isinstance(value, float) else value)
                    for key, value in row.items()
                }
            )


def main():
    args = parse_args()
    if args.trials <= 0:
        raise SystemExit("--trials must be greater than zero")
    if args.runtime_width_px <= 0 or args.runtime_height_px <= 0:
        raise SystemExit("runtime image dimensions must be positive")

    root = Path(__file__).resolve().parents[1]
    output_dir = (
        Path(args.output_dir)
        if args.output_dir
        else root / "reports" / f"bbox_profile_runtime_{dt.datetime.now():%Y%m%d_%H%M%S}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    source_matrix = read_matrix(args.source_matrix)
    runtime_matrix = transform_matrix(source_matrix, args)
    profile_path = output_dir / args.output_name
    write_matrix(profile_path, runtime_matrix)

    profile_overlap = profile_overlap_summary(args.profile_source_csv, args)
    validation_overlap = validation_overlap_summary(args)
    waypoint_rows = validation_overlap.pop("waypoint_rows")
    write_waypoint_csv(output_dir / "validation_waypoints.csv", waypoint_rows)

    final_pass = (
        profile_overlap.get("profile_generation_overlap_verdict") == "pass"
        and validation_overlap.get("validation_overlap_verdict") == "pass"
    )
    metadata = {
        "created_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_matrix_path": str(Path(args.source_matrix)),
        "generated_profile_path": str(profile_path),
        "source_coordinate_contract": "MATLAB-flipped 1920x1080 bbox-center affine matrix",
        "runtime_coordinate_contract": "top-left image coordinates used by stereo_distance_node._adjust_bbox_centers",
        "source_image_px": [args.source_width_px, args.source_height_px],
        "runtime_image_px": [args.runtime_width_px, args.runtime_height_px],
        "profile_source_overlap": profile_overlap,
        "validation_overlap": validation_overlap,
        "final_overlap_verdict": "pass" if final_pass else "fail",
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output_dir / "result.txt").write_text(
        "\n".join(
            [
                f"generated_profile={profile_path}",
                f"profile_generation_overlap={profile_overlap.get('profile_generation_overlap_verdict')}",
                f"validation_overlap={validation_overlap.get('validation_overlap_verdict')}",
                f"final_overlap_verdict={'pass' if final_pass else 'fail'}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"Generated profile: {profile_path}")
    print(f"Metadata: {output_dir / 'metadata.json'}")
    print(f"Validation waypoints: {output_dir / 'validation_waypoints.csv'}")
    print(f"Final overlap verdict: {'pass' if final_pass else 'fail'}")
    return 0 if final_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
