#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


DEFAULT_IMAGE_WIDTH_PX = 1280.0
DEFAULT_IMAGE_HEIGHT_PX = 720.0
DEFAULT_BASELINE_M = 25.0
DEFAULT_HORIZONTAL_FOV_RAD = 0.154997710698


@dataclass(frozen=True)
class CameraModel:
    width_px: float
    height_px: float
    fx_px: float
    baseline_m: float

    @property
    def fy_px(self) -> float:
        return self.fx_px

    @property
    def cx_px(self) -> float:
        return self.width_px * 0.5

    @property
    def cy_px(self) -> float:
        return self.height_px * 0.5


@dataclass(frozen=True)
class Sample:
    truth_right_m: float
    truth_down_m: float
    truth_forward_m: float
    left_u_px: float
    left_v_px: float
    right_u_px: float
    right_v_px: float
    trial_t_s: float | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit a global right-camera extrinsic rotation from one normalized trial, "
            "then offline-retriangulate validation trials."
        )
    )
    parser.add_argument("report_dir", type=Path, help="Validation report containing normalized/trial_XX.csv files.")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--train-trial", type=int, default=1)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--image-width-px", type=float, default=DEFAULT_IMAGE_WIDTH_PX)
    parser.add_argument("--image-height-px", type=float, default=DEFAULT_IMAGE_HEIGHT_PX)
    parser.add_argument("--baseline-m", type=float, default=DEFAULT_BASELINE_M)
    parser.add_argument("--horizontal-fov-rad", type=float, default=DEFAULT_HORIZONTAL_FOV_RAD)
    parser.add_argument("--max-rotation-deg", type=float, default=0.8)
    parser.add_argument(
        "--fixed-rotation-deg",
        default="",
        help="Optional roll,pitch,yaw optical rotation in degrees. When set, skip fitting and only evaluate.",
    )
    parser.add_argument(
        "--residual-policy",
        choices=("none", "global_bias"),
        default="none",
        help="Optional residual observation model applied after the physical rotation fit.",
    )
    return parser.parse_args()


def validation_trials(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def parse_rotation_deg(text: str) -> tuple[float, float, float] | None:
    if not text.strip():
        return None
    parts = [float(part.strip()) for part in text.split(",") if part.strip()]
    if len(parts) != 3:
        raise ValueError("--fixed-rotation-deg must contain roll,pitch,yaw")
    return (math.radians(parts[0]), math.radians(parts[1]), math.radians(parts[2]))


def focal_px(width_px: float, horizontal_fov_rad: float) -> float:
    return (float(width_px) * 0.5) / math.tan(float(horizontal_fov_rad) * 0.5)


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def first_float(row: dict[str, str], names: Sequence[str]) -> float | None:
    for name in names:
        value = finite_float(row.get(name))
        if value is not None:
            return value
    return None


def load_samples(path: Path) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            truth_right = first_float(row, ("truth_right_m", "gt_right_m"))
            truth_down = first_float(row, ("truth_down_m", "gt_down_m"))
            truth_forward = first_float(row, ("truth_forward_m", "gt_forward_m"))
            left_u = first_float(row, ("left_cx", "left_cx_px"))
            left_v = first_float(row, ("left_cy", "left_cy_px"))
            right_u = first_float(row, ("right_cx", "right_cx_px"))
            right_v = first_float(row, ("right_cy", "right_cy_px"))
            if None in (truth_right, truth_down, truth_forward, left_u, left_v, right_u, right_v):
                continue
            if truth_forward is None or truth_forward <= 0.0:
                continue
            samples.append(
                Sample(
                    truth_right_m=float(truth_right),
                    truth_down_m=float(truth_down),
                    truth_forward_m=float(truth_forward),
                    left_u_px=float(left_u),
                    left_v_px=float(left_v),
                    right_u_px=float(right_u),
                    right_v_px=float(right_v),
                    trial_t_s=first_float(row, ("trial_t_s",)),
                )
            )
    return samples


def matrix_from_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[tuple[float, float, float], ...]:
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


def apply_matrix(matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def apply_transpose(
    matrix: tuple[tuple[float, float, float], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
        matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
        matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2],
    )


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def scale(a: tuple[float, float, float], value: float) -> tuple[float, float, float]:
    return (a[0] * value, a[1] * value, a[2] * value)


def add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def norm(a: tuple[float, float, float]) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: tuple[float, float, float]) -> tuple[float, float, float]:
    length = norm(a)
    if length <= 0.0:
        raise ValueError("zero-length vector")
    return (a[0] / length, a[1] / length, a[2] / length)


def project_right(sample: Sample, camera: CameraModel, angles_rad: Sequence[float]) -> tuple[float, float] | None:
    rotation = matrix_from_rpy(float(angles_rad[0]), float(angles_rad[1]), float(angles_rad[2]))
    point_right_nominal = (
        sample.truth_right_m - camera.baseline_m * 0.5,
        sample.truth_down_m,
        sample.truth_forward_m,
    )
    point_actual = apply_transpose(rotation, point_right_nominal)
    if point_actual[2] <= 0.0:
        return None
    return (
        camera.cx_px + point_actual[0] * camera.fx_px / point_actual[2],
        camera.cy_px + point_actual[1] * camera.fy_px / point_actual[2],
    )


def project_left(sample: Sample, camera: CameraModel) -> tuple[float, float]:
    return (
        camera.cx_px + (sample.truth_right_m + camera.baseline_m * 0.5) * camera.fx_px / sample.truth_forward_m,
        camera.cy_px + sample.truth_down_m * camera.fy_px / sample.truth_forward_m,
    )


def huber_loss(values: Iterable[float], delta: float = 8.0) -> float:
    values_list = list(values)
    if not values_list:
        return float("inf")
    total = 0.0
    for value in values_list:
        abs_value = abs(value)
        if abs_value <= delta:
            total += 0.5 * value * value
        else:
            total += delta * (abs_value - 0.5 * delta)
    return total / len(values_list)


def reprojection_loss(samples: Sequence[Sample], camera: CameraModel, angles_rad: Sequence[float]) -> float:
    residuals: list[float] = []
    for sample in samples:
        projected = project_right(sample, camera, angles_rad)
        if projected is None:
            continue
        residuals.append(projected[0] - sample.right_u_px)
        residuals.append(projected[1] - sample.right_v_px)
    return huber_loss(residuals)


def fit_right_rotation(
    samples: Sequence[Sample],
    camera: CameraModel,
    *,
    max_rotation_deg: float,
) -> tuple[float, float, float]:
    limit = math.radians(max_rotation_deg)
    current = [0.0, 0.0, 0.0]
    best_loss = reprojection_loss(samples, camera, current)
    for roll_deg in (-0.3, 0.0, 0.3):
        for pitch_deg in (-0.3, 0.0, 0.3):
            for yaw_deg in (-0.3, 0.0, 0.3):
                candidate = [math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)]
                loss = reprojection_loss(samples, camera, candidate)
                if loss < best_loss:
                    best_loss = loss
                    current = candidate

    for step_deg in (0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001):
        step = math.radians(step_deg)
        improved = True
        while improved:
            improved = False
            best_candidate = list(current)
            best_loss = reprojection_loss(samples, camera, current)
            for index in range(3):
                for sign in (-1.0, 1.0):
                    candidate = list(current)
                    candidate[index] += sign * step
                    if abs(candidate[index]) > limit:
                        continue
                    loss = reprojection_loss(samples, camera, candidate)
                    if loss < best_loss - 1.0e-10:
                        best_loss = loss
                        best_candidate = candidate
                        improved = True
            current = best_candidate
    return (current[0], current[1], current[2])


def fit_global_bias(samples: Sequence[Sample], camera: CameraModel, angles_rad: Sequence[float]) -> tuple[float, float, float, float]:
    left_du: list[float] = []
    left_dv: list[float] = []
    right_du: list[float] = []
    right_dv: list[float] = []
    for sample in samples:
        left_projected = project_left(sample, camera)
        right_projected = project_right(sample, camera, angles_rad)
        if right_projected is None:
            continue
        left_du.append(left_projected[0] - sample.left_u_px)
        left_dv.append(left_projected[1] - sample.left_v_px)
        right_du.append(right_projected[0] - sample.right_u_px)
        right_dv.append(right_projected[1] - sample.right_v_px)
    if not right_du:
        return (0.0, 0.0, 0.0, 0.0)
    return (
        float(statistics.median(left_du)),
        float(statistics.median(left_dv)),
        float(statistics.median(right_du)),
        float(statistics.median(right_dv)),
    )


def triangulate_rays(
    left_ray: tuple[float, float, float],
    right_ray_in_left: tuple[float, float, float],
    camera: CameraModel,
) -> tuple[float, float, float] | None:
    d1 = normalize(left_ray)
    d2 = normalize(right_ray_in_left)
    translation = (camera.baseline_m, 0.0, 0.0)
    a = dot(d1, d1)
    b = dot(d1, d2)
    c = dot(d2, d2)
    w0 = scale(translation, -1.0)
    d = dot(d1, w0)
    e = dot(d2, w0)
    denom = a * c - b * b
    if abs(denom) <= 1.0e-12:
        return None
    s = (b * e - c * d) / denom
    u = (a * e - b * d) / denom
    if s <= 0.0 or u <= 0.0:
        return None
    left_point = scale(d1, s)
    right_point = add(translation, scale(d2, u))
    midpoint_left = scale(add(left_point, right_point), 0.5)
    return (
        midpoint_left[0] - camera.baseline_m * 0.5,
        midpoint_left[1],
        midpoint_left[2],
    )


def raw_triangulate(sample: Sample, camera: CameraModel) -> tuple[float, float, float] | None:
    disparity = abs(sample.left_u_px - sample.right_u_px)
    if disparity < 0.5:
        return None
    forward = camera.fx_px * camera.baseline_m / disparity
    right = ((sample.left_u_px + sample.right_u_px) * 0.5 - camera.cx_px) * forward / camera.fx_px
    down = ((sample.left_v_px + sample.right_v_px) * 0.5 - camera.cy_px) * forward / camera.fy_px
    return (right, down, forward)


def corrected_triangulate(
    sample: Sample,
    camera: CameraModel,
    angles_rad: Sequence[float],
    pixel_bias: Sequence[float],
) -> tuple[float, float, float] | None:
    left_u = sample.left_u_px + float(pixel_bias[0])
    left_v = sample.left_v_px + float(pixel_bias[1])
    right_u = sample.right_u_px + float(pixel_bias[2])
    right_v = sample.right_v_px + float(pixel_bias[3])
    left_ray = normalize(((left_u - camera.cx_px) / camera.fx_px, (left_v - camera.cy_px) / camera.fy_px, 1.0))
    right_ray = normalize(((right_u - camera.cx_px) / camera.fx_px, (right_v - camera.cy_px) / camera.fy_px, 1.0))
    rotation = matrix_from_rpy(float(angles_rad[0]), float(angles_rad[1]), float(angles_rad[2]))
    return triangulate_rays(left_ray, apply_matrix(rotation, right_ray), camera)


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def evaluate(
    samples: Sequence[Sample],
    camera: CameraModel,
    angles_rad: Sequence[float],
    pixel_bias: Sequence[float],
) -> tuple[list[dict[str, str]], dict[str, float | str | int]]:
    rows: list[dict[str, str]] = []
    errors: dict[str, list[float]] = {
        "raw_right": [],
        "raw_down": [],
        "raw_forward": [],
        "corrected_right": [],
        "corrected_down": [],
        "corrected_forward": [],
    }
    skipped = 0
    for sample in samples:
        raw = raw_triangulate(sample, camera)
        corrected = corrected_triangulate(sample, camera, angles_rad, pixel_bias)
        if raw is None or corrected is None:
            skipped += 1
            continue
        raw_error = (
            raw[0] - sample.truth_right_m,
            raw[1] - sample.truth_down_m,
            raw[2] - sample.truth_forward_m,
        )
        corrected_error = (
            corrected[0] - sample.truth_right_m,
            corrected[1] - sample.truth_down_m,
            corrected[2] - sample.truth_forward_m,
        )
        errors["raw_right"].append(raw_error[0])
        errors["raw_down"].append(raw_error[1])
        errors["raw_forward"].append(raw_error[2])
        errors["corrected_right"].append(corrected_error[0])
        errors["corrected_down"].append(corrected_error[1])
        errors["corrected_forward"].append(corrected_error[2])
        rows.append(
            {
                "trial_t_s": "" if sample.trial_t_s is None else f"{sample.trial_t_s:.6f}",
                "truth_right_m": f"{sample.truth_right_m:.9f}",
                "truth_down_m": f"{sample.truth_down_m:.9f}",
                "truth_forward_m": f"{sample.truth_forward_m:.9f}",
                "raw_right_m": f"{raw[0]:.9f}",
                "raw_down_m": f"{raw[1]:.9f}",
                "raw_forward_m": f"{raw[2]:.9f}",
                "corrected_right_m": f"{corrected[0]:.9f}",
                "corrected_down_m": f"{corrected[1]:.9f}",
                "corrected_forward_m": f"{corrected[2]:.9f}",
                "raw_error_right_m": f"{raw_error[0]:.9f}",
                "raw_error_down_m": f"{raw_error[1]:.9f}",
                "raw_error_forward_m": f"{raw_error[2]:.9f}",
                "corrected_error_right_m": f"{corrected_error[0]:.9f}",
                "corrected_error_down_m": f"{corrected_error[1]:.9f}",
                "corrected_error_forward_m": f"{corrected_error[2]:.9f}",
            }
        )

    summary: dict[str, float | str | int] = {"sample_count": len(samples), "evaluated_count": len(rows), "skipped_count": skipped}
    pass_all = True
    for axis in ("right", "down", "forward"):
        raw_value = rmse(errors[f"raw_{axis}"])
        corrected_value = rmse(errors[f"corrected_{axis}"])
        improvement = raw_value - corrected_value
        summary[f"raw_{axis}_rmse_m"] = raw_value
        summary[f"corrected_{axis}_rmse_m"] = corrected_value
        summary[f"{axis}_rmse_improvement_m"] = improvement
        if not (math.isfinite(improvement) and improvement >= 0.0):
            pass_all = False
    raw_position = [
        math.sqrt(r * r + d * d + f * f)
        for r, d, f in zip(errors["raw_right"], errors["raw_down"], errors["raw_forward"])
    ]
    corrected_position = [
        math.sqrt(r * r + d * d + f * f)
        for r, d, f in zip(errors["corrected_right"], errors["corrected_down"], errors["corrected_forward"])
    ]
    summary["raw_position_rmse_m"] = rmse(raw_position)
    summary["corrected_position_rmse_m"] = rmse(corrected_position)
    summary["position_rmse_improvement_m"] = summary["raw_position_rmse_m"] - summary["corrected_position_rmse_m"]
    summary["pass"] = "yes" if pass_all else "no"
    return rows, summary


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    camera = CameraModel(
        width_px=float(args.image_width_px),
        height_px=float(args.image_height_px),
        fx_px=focal_px(args.image_width_px, args.horizontal_fov_rad),
        baseline_m=float(args.baseline_m),
    )
    normalized_dir = args.report_dir / "normalized"
    train_path = normalized_dir / f"trial_{args.train_trial:02d}.csv"
    train_samples = load_samples(train_path)
    if not train_samples:
        raise RuntimeError(f"no train samples loaded from {train_path}")

    fixed_rotation = parse_rotation_deg(args.fixed_rotation_deg)
    if fixed_rotation is None:
        angles_rad = fit_right_rotation(train_samples, camera, max_rotation_deg=args.max_rotation_deg)
        rotation_source = "fit_right_reprojection_huber"
    else:
        angles_rad = fixed_rotation
        rotation_source = "fixed_cli_optical_rpy"
    if args.residual_policy == "global_bias":
        pixel_bias = fit_global_bias(train_samples, camera, angles_rad)
    else:
        pixel_bias = (0.0, 0.0, 0.0, 0.0)

    output_dir = args.output_dir
    if output_dir is None:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = args.report_dir / f"extrinsic_refinement_{args.residual_policy}_{stamp}"

    train_loss = reprojection_loss(train_samples, camera, angles_rad)
    profile = {
        "method": "right_camera_reprojection_extrinsic_refinement_v1",
        "source_report_dir": str(args.report_dir),
        "train_trial": args.train_trial,
        "sample_count": len(train_samples),
        "camera": {
            "image_width_px": camera.width_px,
            "image_height_px": camera.height_px,
            "fx_px": camera.fx_px,
            "baseline_m": camera.baseline_m,
        },
        "rotation_right_actual_to_nominal": {
            "source": rotation_source,
            "roll_deg": math.degrees(angles_rad[0]),
            "pitch_deg": math.degrees(angles_rad[1]),
            "yaw_deg": math.degrees(angles_rad[2]),
            "matrix": matrix_from_rpy(*angles_rad),
        },
        "residual_policy": args.residual_policy,
        "pixel_bias_du_dv": {
            "left_du_px": pixel_bias[0],
            "left_dv_px": pixel_bias[1],
            "right_du_px": pixel_bias[2],
            "right_dv_px": pixel_bias[3],
        },
        "train_reprojection_huber_loss_px": train_loss,
        "zero_rotation_reprojection_huber_loss_px": reprojection_loss(train_samples, camera, (0.0, 0.0, 0.0)),
    }
    write_json(output_dir / "extrinsic_refinement_profile.json", profile)

    gate_rows: list[dict[str, str]] = []
    final_pass = True
    for trial in validation_trials(args.validation_trials):
        samples = load_samples(normalized_dir / f"trial_{trial:02d}.csv")
        rows, summary = evaluate(samples, camera, angles_rad, pixel_bias)
        write_csv(output_dir / f"validation_trial_{trial:02d}_extrinsic_comparison.csv", rows)
        write_json(output_dir / f"validation_trial_{trial:02d}_summary.json", summary)
        if summary["pass"] != "yes":
            final_pass = False
        gate_rows.append(
            {
                "trial": str(trial),
                "sample_count": str(summary["sample_count"]),
                "evaluated_count": str(summary["evaluated_count"]),
                "right_rmse_improvement_m": f"{float(summary['right_rmse_improvement_m']):.9f}",
                "down_rmse_improvement_m": f"{float(summary['down_rmse_improvement_m']):.9f}",
                "forward_rmse_improvement_m": f"{float(summary['forward_rmse_improvement_m']):.9f}",
                "position_rmse_improvement_m": f"{float(summary['position_rmse_improvement_m']):.9f}",
                "pass": str(summary["pass"]),
            }
        )

    write_csv(output_dir / "validation_gate_summary.csv", gate_rows)
    (output_dir / "result.txt").write_text(
        f"final_pass={'yes' if final_pass else 'no'}\n"
        f"profile={output_dir / 'extrinsic_refinement_profile.json'}\n"
        f"summary={output_dir / 'validation_gate_summary.csv'}\n",
        encoding="utf-8",
    )
    print(f"Profile: {output_dir / 'extrinsic_refinement_profile.json'}")
    print(f"Summary: {output_dir / 'validation_gate_summary.csv'}")
    print(f"Final pass: {'yes' if final_pass else 'no'}")
    return 0 if final_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
