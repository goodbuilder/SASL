#!/usr/bin/env python3
"""Validate one left/right PX4 SITL magnetometer-frame qualification pair.

The validator reads real PX4 ULogs.  It compares the delivered ``sensor_mag``
body-FRD vector with an independently constructed WMM NED vector rotated by
the configured body attitude, and separately checks the EKF attitude,
inclination, strength, fusion flags, reset stability, and left/right yaw.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterable

try:
    import numpy as np
    from pyulog import ULog
except ImportError as exc:  # pragma: no cover - exercised by deployment environment
    print(
        "error: validation requires numpy and pyulog; install with "
        "python3 -m pip install --user numpy pyulog",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


SCHEMA = "simulation-px4-magnetometer-frame-qualification-v1"
TOPICS = (
    "sensor_mag",
    "vehicle_attitude",
    "estimator_status",
    "estimator_status_flags",
)


class ValidationError(RuntimeError):
    """Raised when an input artifact cannot support the qualification."""


def wrap_deg(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def _rpy_rotation_ned_from_frd(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> tuple[tuple[float, float, float], ...]:
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Active Hamilton-compatible Rz(yaw) Ry(pitch) Rx(roll), body FRD -> NED.
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def expected_body_field_gauss(
    *,
    declination_deg: float,
    inclination_deg: float,
    strength_gauss: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> tuple[float, float, float]:
    declination = math.radians(float(declination_deg))
    inclination = math.radians(float(inclination_deg))
    strength = float(strength_gauss)
    horizontal = strength * math.cos(inclination)
    field_ned = (
        horizontal * math.cos(declination),
        horizontal * math.sin(declination),
        strength * math.sin(inclination),
    )
    rotation = _rpy_rotation_ned_from_frd(roll_deg, pitch_deg, yaw_deg)
    return tuple(
        sum(rotation[row][column] * field_ned[row] for row in range(3))
        for column in range(3)
    )


def _dataset(ulog: ULog, name: str):
    matches = [item for item in ulog.data_list if item.name == name and item.multi_id == 0]
    if len(matches) != 1:
        raise ValidationError(f"ULog requires exactly one {name}[0] dataset, found {len(matches)}")
    return matches[0]


def _window_mask(
    timestamps: np.ndarray,
    *,
    first_attitude_us: int,
    end_attitude_us: int,
    warmup_s: float,
    window_s: float,
) -> np.ndarray:
    start_us = max(
        first_attitude_us + int(warmup_s * 1_000_000),
        end_attitude_us - int(window_s * 1_000_000),
    )
    mask = (timestamps >= start_us) & (timestamps <= end_attitude_us)
    if not bool(np.any(mask)):
        raise ValidationError("ULog has no samples in the requested stable window")
    return mask


def _attitude_rpy_deg(data: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    w = np.asarray(data["q[0]"], dtype=float)
    x = np.asarray(data["q[1]"], dtype=float)
    y = np.asarray(data["q[2]"], dtype=float)
    z = np.asarray(data["q[3]"], dtype=float)
    roll = np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    pitch_argument = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(pitch_argument))
    yaw = np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return roll, pitch, yaw


def _circular_stats_deg(values: Iterable[float]) -> tuple[float, float]:
    radians = [math.radians(float(value)) for value in values]
    if not radians:
        raise ValidationError("cannot compute circular statistics for an empty sample")
    mean = math.degrees(
        math.atan2(
            statistics.fmean(math.sin(value) for value in radians),
            statistics.fmean(math.cos(value) for value in radians),
        )
    )
    residuals = [wrap_deg(math.degrees(value) - mean) for value in radians]
    return wrap_deg(mean), statistics.pstdev(residuals)


def _number_stats(values: np.ndarray) -> dict[str, float]:
    parsed = np.asarray(values, dtype=float)
    if parsed.size == 0 or not bool(np.all(np.isfinite(parsed))):
        raise ValidationError("qualification samples must be finite and non-empty")
    return {
        "mean": float(np.mean(parsed)),
        "stddev": float(np.std(parsed)),
        "min": float(np.min(parsed)),
        "max": float(np.max(parsed)),
    }


def analyze_side(
    path: Path,
    *,
    expected_rpy_deg: tuple[float, float, float],
    expected_declination_deg: float,
    warmup_s: float,
    window_s: float,
    max_declination_error_deg: float,
    max_mag_axis_error_gauss: float,
    max_inclination_error_deg: float,
    max_strength_relative_error: float,
    max_yaw_error_deg: float,
    max_yaw_stddev_deg: float,
) -> dict[str, Any]:
    source = path.resolve()
    if not source.is_file():
        raise ValidationError(f"ULog does not exist: {source}")
    ulog = ULog(str(source), message_name_filter_list=list(TOPICS))
    attitude = _dataset(ulog, "vehicle_attitude")
    sensor_mag = _dataset(ulog, "sensor_mag")
    estimator_status = _dataset(ulog, "estimator_status")
    estimator_flags = _dataset(ulog, "estimator_status_flags")

    attitude_timestamps = np.asarray(attitude.data["timestamp"], dtype=np.int64)
    if attitude_timestamps.size < 2:
        raise ValidationError("vehicle_attitude has fewer than two samples")
    first_us, end_us = int(attitude_timestamps[0]), int(attitude_timestamps[-1])
    if end_us - first_us < int((warmup_s + 1.0) * 1_000_000):
        raise ValidationError(
            f"ULog duration {(end_us - first_us) / 1e6:.3f}s is shorter than warmup {warmup_s:.3f}s"
        )

    attitude_mask = _window_mask(
        attitude_timestamps,
        first_attitude_us=first_us,
        end_attitude_us=end_us,
        warmup_s=warmup_s,
        window_s=window_s,
    )
    mag_mask = _window_mask(
        np.asarray(sensor_mag.data["timestamp"], dtype=np.int64),
        first_attitude_us=first_us,
        end_attitude_us=end_us,
        warmup_s=warmup_s,
        window_s=window_s,
    )
    status_mask = _window_mask(
        np.asarray(estimator_status.data["timestamp"], dtype=np.int64),
        first_attitude_us=first_us,
        end_attitude_us=end_us,
        warmup_s=warmup_s,
        window_s=window_s,
    )

    roll, pitch, yaw = _attitude_rpy_deg(attitude.data)
    yaw_mean, yaw_stddev = _circular_stats_deg(yaw[attitude_mask])
    roll_stats = _number_stats(roll[attitude_mask])
    pitch_stats = _number_stats(pitch[attitude_mask])
    yaw_error_deg = wrap_deg(yaw_mean - expected_rpy_deg[2])

    declination = float(ulog.initial_parameters.get("EKF2_MAG_DECL", math.nan))
    mag_type = int(ulog.initial_parameters.get("EKF2_MAG_TYPE", -999))
    declination_type = int(ulog.initial_parameters.get("EKF2_DECL_TYPE", -999))
    inclination = np.asarray(estimator_status.data["mag_inclination_deg"], dtype=float)[status_mask]
    inclination_ref = np.asarray(estimator_status.data["mag_inclination_ref_deg"], dtype=float)[status_mask]
    strength = np.asarray(estimator_status.data["mag_strength_gs"], dtype=float)[status_mask]
    strength_ref = np.asarray(estimator_status.data["mag_strength_ref_gs"], dtype=float)[status_mask]
    inclination_error = float(np.mean(np.abs(inclination - inclination_ref)))
    strength_reference_mean = float(np.mean(strength_ref))
    strength_relative_error = float(
        np.mean(np.abs(strength - strength_ref)) / max(abs(strength_reference_mean), 1.0e-12)
    )

    mag_mean = tuple(
        float(np.mean(np.asarray(sensor_mag.data[axis], dtype=float)[mag_mask]))
        for axis in ("x", "y", "z")
    )
    expected_mag = expected_body_field_gauss(
        declination_deg=declination,
        inclination_deg=float(np.mean(inclination_ref)),
        strength_gauss=strength_reference_mean,
        roll_deg=expected_rpy_deg[0],
        pitch_deg=expected_rpy_deg[1],
        yaw_deg=expected_rpy_deg[2],
    )
    mag_axis_errors = tuple(actual - expected for actual, expected in zip(mag_mean, expected_mag))
    max_mag_axis_error = max(abs(value) for value in mag_axis_errors)

    reset_values = sorted(
        {int(value) for value in np.asarray(attitude.data["quat_reset_counter"])[attitude_mask]}
    )
    flags = {
        name: int(estimator_flags.data[name][-1])
        for name in (
            "cs_mag_hdg",
            "cs_mag_dec",
            "cs_mag_3d",
            "cs_mag_field_disturbed",
            "cs_yaw_align",
            "fs_bad_hdg",
            "fs_bad_mag_decl",
            "fs_bad_mag_x",
            "fs_bad_mag_y",
            "fs_bad_mag_z",
            "reject_yaw",
        )
    }
    checks = {
        "declination_matches_expected": abs(wrap_deg(declination - expected_declination_deg))
        <= max_declination_error_deg,
        "magnetic_heading_mode": mag_type == 1,
        "geographic_declination_enabled": bool(declination_type & 1),
        "body_mag_axes_match": max_mag_axis_error <= max_mag_axis_error_gauss,
        "inclination_matches": inclination_error <= max_inclination_error_deg,
        "strength_matches": strength_relative_error <= max_strength_relative_error,
        "yaw_matches": abs(yaw_error_deg) <= max_yaw_error_deg,
        "yaw_is_stable": yaw_stddev <= max_yaw_stddev_deg,
        "reset_counter_stable": len(reset_values) == 1,
        "mag_heading_fused": flags["cs_mag_hdg"] == 1,
        "mag_declination_fused": flags["cs_mag_dec"] == 1,
        "mag_3d_not_used": flags["cs_mag_3d"] == 0,
        "mag_not_disturbed": flags["cs_mag_field_disturbed"] == 0,
        "yaw_aligned": flags["cs_yaw_align"] == 1,
        "yaw_not_rejected": flags["reject_yaw"] == 0,
        "mag_fault_flags_clear": all(
            flags[name] == 0
            for name in (
                "fs_bad_hdg",
                "fs_bad_mag_decl",
                "fs_bad_mag_x",
                "fs_bad_mag_y",
                "fs_bad_mag_z",
            )
        ),
    }
    return {
        "source": source.name,
        "duration_s": (end_us - first_us) / 1_000_000.0,
        "window": {
            "warmup_s": warmup_s,
            "window_s": window_s,
            "attitude_samples": int(np.count_nonzero(attitude_mask)),
            "mag_samples": int(np.count_nonzero(mag_mask)),
            "status_samples": int(np.count_nonzero(status_mask)),
        },
        "parameters": {
            "EKF2_MAG_TYPE": mag_type,
            "EKF2_DECL_TYPE": declination_type,
            "EKF2_MAG_DECL": declination,
        },
        "expected_rpy_deg": list(expected_rpy_deg),
        "attitude": {
            "roll": roll_stats,
            "pitch": pitch_stats,
            "yaw_mean_deg": yaw_mean,
            "yaw_stddev_deg": yaw_stddev,
            "yaw_error_deg": yaw_error_deg,
            "quat_reset_counter_values": reset_values,
        },
        "magnetometer": {
            "actual_body_frd_mean_gauss": list(mag_mean),
            "expected_body_frd_gauss": list(expected_mag),
            "axis_error_gauss": list(mag_axis_errors),
            "max_abs_axis_error_gauss": max_mag_axis_error,
            "inclination_mean_deg": float(np.mean(inclination)),
            "inclination_reference_mean_deg": float(np.mean(inclination_ref)),
            "inclination_mean_abs_error_deg": inclination_error,
            "strength_mean_gauss": float(np.mean(strength)),
            "strength_reference_mean_gauss": strength_reference_mean,
            "strength_relative_error": strength_relative_error,
        },
        "estimator_flags": flags,
        "checks": checks,
        "status": "PASS" if all(checks.values()) else "FAIL",
    }


def validate_pair(args: argparse.Namespace) -> dict[str, Any]:
    expected_rpy = (
        float(args.expected_roll_deg),
        float(args.expected_pitch_deg),
        float(args.expected_yaw_deg),
    )
    common = {
        "expected_rpy_deg": expected_rpy,
        "expected_declination_deg": float(args.expected_declination_deg),
        "warmup_s": float(args.warmup_s),
        "window_s": float(args.window_s),
        "max_declination_error_deg": float(args.max_declination_error_deg),
        "max_mag_axis_error_gauss": float(args.max_mag_axis_error_gauss),
        "max_inclination_error_deg": float(args.max_inclination_error_deg),
        "max_strength_relative_error": float(args.max_strength_relative_error),
        "max_yaw_error_deg": float(args.max_yaw_error_deg),
        "max_yaw_stddev_deg": float(args.max_yaw_stddev_deg),
    }
    left = analyze_side(args.left_ulog, **common)
    right = analyze_side(args.right_ulog, **common)
    yaw_difference = wrap_deg(
        left["attitude"]["yaw_mean_deg"] - right["attitude"]["yaw_mean_deg"]
    )
    pair_checks = {
        "left_pass": left["status"] == "PASS",
        "right_pass": right["status"] == "PASS",
        "left_right_yaw_matches": abs(yaw_difference) <= args.max_left_right_yaw_deg,
    }
    return {
        "schema": SCHEMA,
        "expected_rpy_deg": list(expected_rpy),
        "expected_declination_deg": float(args.expected_declination_deg),
        "thresholds": {
            "max_declination_error_deg": args.max_declination_error_deg,
            "max_mag_axis_error_gauss": args.max_mag_axis_error_gauss,
            "max_inclination_error_deg": args.max_inclination_error_deg,
            "max_strength_relative_error": args.max_strength_relative_error,
            "max_yaw_error_deg": args.max_yaw_error_deg,
            "max_yaw_stddev_deg": args.max_yaw_stddev_deg,
            "max_left_right_yaw_deg": args.max_left_right_yaw_deg,
        },
        "left": left,
        "right": right,
        "left_minus_right_yaw_deg": yaw_difference,
        "checks": pair_checks,
        "status": "PASS" if all(pair_checks.values()) else "FAIL",
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-ulog", required=True, type=Path)
    parser.add_argument("--right-ulog", required=True, type=Path)
    parser.add_argument("--expected-roll-deg", required=True, type=float)
    parser.add_argument("--expected-pitch-deg", required=True, type=float)
    parser.add_argument("--expected-yaw-deg", required=True, type=float)
    parser.add_argument("--expected-declination-deg", required=True, type=float)
    parser.add_argument("--warmup-s", type=float, default=30.0)
    parser.add_argument("--window-s", type=float, default=15.0)
    parser.add_argument("--max-declination-error-deg", type=float, default=0.05)
    parser.add_argument("--max-mag-axis-error-gauss", type=float, default=0.002)
    parser.add_argument("--max-inclination-error-deg", type=float, default=0.5)
    parser.add_argument("--max-strength-relative-error", type=float, default=0.005)
    parser.add_argument("--max-yaw-error-deg", type=float, default=1.0)
    parser.add_argument("--max-yaw-stddev-deg", type=float, default=0.1)
    parser.add_argument("--max-left-right-yaw-deg", type=float, default=0.25)
    parser.add_argument("--output-json", type=Path)
    return parser


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    target = path.resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=target.parent,
        prefix=f".{target.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")
        temporary = Path(handle.name)
    os.replace(temporary, target)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        payload = validate_pair(args)
    except (ValidationError, KeyError, TypeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    if args.output_json is not None:
        _write_json(args.output_json, payload)
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
