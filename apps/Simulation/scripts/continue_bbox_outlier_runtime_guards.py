#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping, Sequence

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(ROOT_DIR / "stereo_ground_sim") not in sys.path:
    sys.path.insert(0, str(ROOT_DIR / "stereo_ground_sim"))

import rtk_stereo_calibration_pipeline as pipeline  # noqa: E402
from stereo_ground_sim.runtime_guard_policy import EstimatedToTruthSim3Profile  # noqa: E402


TRUTH_RDF_TRANSFORM_CONTRACT = (
    "explicit T_source_to_stereo_rdf required; direct axis alias "
    "rdf_right=east/rdf_down=down/rdf_forward=north is forbidden"
)
NED_TO_RDF_CONTRACT = TRUTH_RDF_TRANSFORM_CONTRACT
TARGET_RANGE_M = 1852.0
TARGET_WINDOW_M = 30.0
PROFILE_JSON = (
    ROOT_DIR
    / "reports/experiments/EXP-20260615-186-runtime-aligned-d6d7-profile-export-replay-audit/output/recommended_runtime_profile.json"
)
REPORT_ROOT = ROOT_DIR / "reports/experiments"


@dataclass(frozen=True)
class TrialPrediction:
    seed: int
    truth: np.ndarray
    raw: np.ndarray
    corrected_by_method: dict[str, np.ndarray]
    invalid_by_method: dict[str, int]
    fallback_by_method: dict[str, int]
    changed_by_method: dict[str, int]


@dataclass(frozen=True)
class MethodSpec:
    name: str
    purpose: str
    apply: Callable[[pipeline.CalibrationSample, np.ndarray], tuple[np.ndarray, bool]]


def profile_apply(
    profile: EstimatedToTruthSim3Profile,
    sample: pipeline.CalibrationSample,
) -> tuple[np.ndarray | None, str]:
    result = profile.apply(
        {
            "right_m": float(sample.raw_rdf_m[0]),
            "down_m": float(sample.raw_rdf_m[1]),
            "forward_m": float(sample.raw_rdf_m[2]),
            "disparity_px": float(sample.measured_disparity_px),
            "left_u_px": float(sample.left_u_px),
            "left_v_px": float(sample.left_v_px),
        }
    )
    if result.corrected is None:
        return None, str(result.status.get("reason", "invalid"))
    point = np.array(
        [
            float(result.corrected["right_m"]),
            float(result.corrected["down_m"]),
            float(result.corrected["forward_m"]),
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(point)):
        return None, "nonfinite_corrected"
    return point, str(result.status.get("reason", "ok"))


def raw_range(sample: pipeline.CalibrationSample) -> float:
    return float(np.linalg.norm(sample.raw_rdf_m))


def point_with_range_delta_cap(raw: np.ndarray, corrected: np.ndarray, cap_m: float) -> np.ndarray:
    raw_norm = float(np.linalg.norm(raw))
    corrected_norm = float(np.linalg.norm(corrected))
    delta = corrected_norm - raw_norm
    if abs(delta) <= cap_m or corrected_norm <= 1.0e-9:
        return corrected
    capped_norm = raw_norm + math.copysign(cap_m, delta)
    return corrected * (capped_norm / corrected_norm)


def blend(raw: np.ndarray, corrected: np.ndarray, alpha: float) -> np.ndarray:
    return raw + float(alpha) * (corrected - raw)


def make_exp199_methods() -> list[MethodSpec]:
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for window in (40.0, 60.0, 90.0, 120.0, 180.0):
        for alpha in (0.55, 0.70, 0.85, 0.95):
            name = f"raw_target_window_w{window:g}_alpha{alpha:g}".replace(".", "p")

            def apply(sample: pipeline.CalibrationSample, corr: np.ndarray, *, window=window, alpha=alpha) -> tuple[np.ndarray, bool]:
                hit = abs(raw_range(sample) - TARGET_RANGE_M) <= window
                return (blend(sample.raw_rdf_m, corr, alpha) if hit else corr, hit)

            methods.append(
                MethodSpec(
                    name,
                    "raw-range target-neighborhood blend guard; runtime uses raw range only",
                    apply,
                )
            )
    return methods


def make_exp200_methods() -> list[MethodSpec]:
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for window in (60.0, 100.0, 150.0, 220.0):
        for trigger in (1.5, 2.0, 2.5, 3.0):
            for cap in (2.0, 2.5, 3.0, 4.0):
                name = f"raw_target_w{window:g}_delta{trigger:g}_cap{cap:g}".replace(".", "p")

                def apply(
                    sample: pipeline.CalibrationSample,
                    corr: np.ndarray,
                    *,
                    window=window,
                    trigger=trigger,
                    cap=cap,
                ) -> tuple[np.ndarray, bool]:
                    raw_norm = raw_range(sample)
                    corr_norm = float(np.linalg.norm(corr))
                    hit = abs(raw_norm - TARGET_RANGE_M) <= window and abs(corr_norm - raw_norm) >= trigger
                    return (point_with_range_delta_cap(sample.raw_rdf_m, corr, cap) if hit else corr, hit)

                methods.append(
                    MethodSpec(
                        name,
                        "raw-target-window range-delta cap; runtime uses raw range and corrected/raw delta only",
                        apply,
                    )
                )
    return methods


def make_exp201_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    target_disparity = camera.fx_meas_px * camera.baseline_meas_m / TARGET_RANGE_M
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for half_width in (0.12, 0.18, 0.24, 0.32, 0.45):
        for alpha in (0.60, 0.75, 0.90):
            name = f"target_disparity_hw{half_width:g}_alpha{alpha:g}".replace(".", "p")

            def apply(
                sample: pipeline.CalibrationSample,
                corr: np.ndarray,
                *,
                half_width=half_width,
                alpha=alpha,
                target_disparity=target_disparity,
            ) -> tuple[np.ndarray, bool]:
                hit = abs(float(sample.measured_disparity_px) - target_disparity) <= half_width
                return (blend(sample.raw_rdf_m, corr, alpha) if hit else corr, hit)

            methods.append(
                MethodSpec(
                    name,
                    "target-disparity neighborhood blend guard; runtime uses measured disparity only",
                    apply,
                )
            )
    return methods


def make_exp202_methods() -> list[MethodSpec]:
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for lateral_threshold in (35.0, 45.0, 55.0, 70.0):
        for vertical_threshold in (18.0, 24.0, 32.0):
            for alpha in (0.65, 0.80, 0.90):
                name = f"lateral{lateral_threshold:g}_vertical{vertical_threshold:g}_alpha{alpha:g}".replace(".", "p")

                def apply(
                    sample: pipeline.CalibrationSample,
                    corr: np.ndarray,
                    *,
                    lateral_threshold=lateral_threshold,
                    vertical_threshold=vertical_threshold,
                    alpha=alpha,
                ) -> tuple[np.ndarray, bool]:
                    raw = sample.raw_rdf_m
                    hit = abs(float(raw[0])) >= lateral_threshold or abs(float(raw[1])) >= vertical_threshold
                    return (blend(raw, corr, alpha) if hit else corr, hit)

                methods.append(
                    MethodSpec(
                        name,
                        "raw RDF lateral/down outlier blend guard; runtime uses raw RDF only",
                        apply,
                    )
                )
    return methods


def make_exp203_methods() -> list[MethodSpec]:
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for window in (60.0, 100.0, 150.0):
        for lateral_threshold in (35.0, 45.0, 55.0):
            for alpha in (0.70, 0.85, 0.95):
                name = f"raw_target_w{window:g}_or_lateral{lateral_threshold:g}_alpha{alpha:g}".replace(".", "p")

                def apply(
                    sample: pipeline.CalibrationSample,
                    corr: np.ndarray,
                    *,
                    window=window,
                    lateral_threshold=lateral_threshold,
                    alpha=alpha,
                ) -> tuple[np.ndarray, bool]:
                    raw = sample.raw_rdf_m
                    hit = abs(raw_range(sample) - TARGET_RANGE_M) <= window or abs(float(raw[0])) >= lateral_threshold
                    return (blend(raw, corr, alpha) if hit else corr, hit)

                methods.append(
                    MethodSpec(
                        name,
                        "combined raw-target-window/lateral blend guard; runtime uses raw RDF/range only",
                        apply,
                    )
                )
    return methods


def make_target_disparity_blend_method(
    camera: pipeline.CameraModel,
    *,
    half_width: float,
    alpha: float,
    name: str | None = None,
) -> MethodSpec:
    target_disparity = camera.fx_meas_px * camera.baseline_meas_m / TARGET_RANGE_M
    method_name = name or f"target_disparity_hw{half_width:g}_alpha{alpha:g}".replace(".", "p")

    def apply(
        sample: pipeline.CalibrationSample,
        corr: np.ndarray,
        *,
        half_width=half_width,
        alpha=alpha,
        target_disparity=target_disparity,
    ) -> tuple[np.ndarray, bool]:
        hit = abs(float(sample.measured_disparity_px) - target_disparity) <= half_width
        return (blend(sample.raw_rdf_m, corr, alpha) if hit else corr, hit)

    return MethodSpec(
        method_name,
        "fixed measured-disparity target-neighborhood blend guard; runtime uses measured disparity only",
        apply,
    )


def target_disparity_guard_payload(
    camera: pipeline.CameraModel,
    *,
    half_width: float,
    alpha: float,
) -> dict[str, float]:
    return {
        "target_disparity_px": camera.fx_meas_px * camera.baseline_meas_m / TARGET_RANGE_M,
        "half_width_px": float(half_width),
        "alpha_inside": float(alpha),
        "alpha_outside": 1.0,
    }


def load_target_disparity_guarded_profile(
    camera: pipeline.CameraModel,
    *,
    half_width: float,
    alpha: float,
) -> EstimatedToTruthSim3Profile:
    data = json.loads(PROFILE_JSON.read_text(encoding="utf-8"))
    data["profile_id"] = f"{data.get('profile_id', 'd6d7')}_target_disparity_hw{half_width:g}_alpha{alpha:g}"
    data["target_disparity_blend_guard"] = target_disparity_guard_payload(
        camera,
        half_width=half_width,
        alpha=alpha,
    )
    return EstimatedToTruthSim3Profile.from_json_dict(data)


def target_range_away_guard_payload(
    *,
    window_m: float,
    alpha: float,
) -> dict[str, float]:
    return {
        "target_range_m": TARGET_RANGE_M,
        "window_m": float(window_m),
        "alpha_away": float(alpha),
    }


def load_target_range_away_guarded_profile(
    *,
    window_m: float,
    alpha: float,
) -> EstimatedToTruthSim3Profile:
    data = json.loads(PROFILE_JSON.read_text(encoding="utf-8"))
    data["profile_id"] = f"{data.get('profile_id', 'd6d7')}_away_target_w{window_m:g}_alpha{alpha:g}"
    data["target_range_away_blend_guard"] = target_range_away_guard_payload(
        window_m=window_m,
        alpha=alpha,
    )
    return EstimatedToTruthSim3Profile.from_json_dict(data)


def make_runtime_profile_method(
    camera: pipeline.CameraModel,
    *,
    half_width: float,
    alpha: float,
    name: str,
) -> MethodSpec:
    guarded_profile = load_target_disparity_guarded_profile(camera, half_width=half_width, alpha=alpha)

    def apply(sample: pipeline.CalibrationSample, corr: np.ndarray) -> tuple[np.ndarray, bool]:
        point, reason = profile_apply(guarded_profile, sample)
        if point is None:
            return sample.raw_rdf_m, False
        changed = reason != "ok"
        return point, changed

    return MethodSpec(
        name,
        "actual EstimatedToTruthSim3Profile target_disparity_blend_guard replay; StereoDistance runtime path",
        apply,
    )


def make_away_runtime_profile_method(
    *,
    window_m: float,
    alpha: float,
    name: str,
) -> MethodSpec:
    guarded_profile = load_target_range_away_guarded_profile(window_m=window_m, alpha=alpha)

    def apply(sample: pipeline.CalibrationSample, corr: np.ndarray) -> tuple[np.ndarray, bool]:
        point, reason = profile_apply(guarded_profile, sample)
        if point is None:
            return sample.raw_rdf_m, False
        return point, reason != "ok"

    return MethodSpec(
        name,
        "actual EstimatedToTruthSim3Profile target_range_away_blend_guard replay; StereoDistance runtime path",
        apply,
    )


def make_combined_runtime_profile_method(
    camera: pipeline.CameraModel,
    *,
    half_width: float,
    target_alpha: float,
    away_window_m: float,
    away_alpha: float,
    name: str,
) -> MethodSpec:
    data = json.loads(PROFILE_JSON.read_text(encoding="utf-8"))
    data["profile_id"] = (
        f"{data.get('profile_id', 'd6d7')}_td_hw{half_width:g}_a{target_alpha:g}"
        f"_away_w{away_window_m:g}_a{away_alpha:g}"
    )
    data["target_disparity_blend_guard"] = target_disparity_guard_payload(
        camera,
        half_width=half_width,
        alpha=target_alpha,
    )
    data["target_range_away_blend_guard"] = target_range_away_guard_payload(
        window_m=away_window_m,
        alpha=away_alpha,
    )
    guarded_profile = EstimatedToTruthSim3Profile.from_json_dict(data)

    def apply(sample: pipeline.CalibrationSample, corr: np.ndarray) -> tuple[np.ndarray, bool]:
        point, reason = profile_apply(guarded_profile, sample)
        if point is None:
            return sample.raw_rdf_m, False
        return point, reason != "ok"

    return MethodSpec(
        name,
        "actual EstimatedToTruthSim3Profile combined target_disparity and target_range_away guards",
        apply,
    )


def target_disparity_cap_guard_payload(
    camera: pipeline.CameraModel,
    *,
    half_width: float,
    cap_m: float,
) -> dict[str, float]:
    return {
        "target_disparity_px": camera.fx_meas_px * camera.baseline_meas_m / TARGET_RANGE_M,
        "half_width_px": float(half_width),
        "range_delta_cap_m": float(cap_m),
    }


def make_cap_runtime_profile_method(
    camera: pipeline.CameraModel,
    *,
    half_width: float,
    cap_m: float,
    name: str,
) -> MethodSpec:
    data = json.loads(PROFILE_JSON.read_text(encoding="utf-8"))
    data["profile_id"] = f"{data.get('profile_id', 'd6d7')}_tdcap_hw{half_width:g}_cap{cap_m:g}"
    data["target_disparity_range_delta_cap_guard"] = target_disparity_cap_guard_payload(
        camera,
        half_width=half_width,
        cap_m=cap_m,
    )
    guarded_profile = EstimatedToTruthSim3Profile.from_json_dict(data)

    def apply(sample: pipeline.CalibrationSample, corr: np.ndarray) -> tuple[np.ndarray, bool]:
        point, reason = profile_apply(guarded_profile, sample)
        if point is None:
            return sample.raw_rdf_m, False
        return point, reason != "ok"

    return MethodSpec(
        name,
        "actual EstimatedToTruthSim3Profile target_disparity_range_delta_cap_guard replay",
        apply,
    )


def make_exp204_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_target_disparity_blend_method(
            camera,
            half_width=0.18,
            alpha=0.75,
            name="target_disparity_hw0p18_alpha0p75_fixed",
        ),
    ]


def make_exp205_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for half_width in (0.14, 0.16, 0.18, 0.20, 0.22):
        for alpha in (0.70, 0.75, 0.80, 0.85):
            methods.append(make_target_disparity_blend_method(camera, half_width=half_width, alpha=alpha))
    return methods


def make_exp206_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_target_disparity_blend_method(
            camera,
            half_width=0.18,
            alpha=0.75,
            name="target_disparity_hw0p18_alpha0p75_fixed_repeat",
        ),
        make_target_disparity_blend_method(
            camera,
            half_width=0.20,
            alpha=0.75,
            name="target_disparity_hw0p20_alpha0p75_fixed",
        ),
        make_target_disparity_blend_method(
            camera,
            half_width=0.18,
            alpha=0.80,
            name="target_disparity_hw0p18_alpha0p80_fixed",
        ),
    ]


def make_exp207_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_runtime_profile_method(
            camera,
            half_width=0.18,
            alpha=0.75,
            name="runtime_profile_target_disparity_hw0p18_alpha0p75",
        ),
    ]


def make_exp208_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_runtime_profile_method(
            camera,
            half_width=0.18,
            alpha=0.75,
            name="runtime_profile_target_disparity_hw0p18_alpha0p75",
        ),
        make_runtime_profile_method(
            camera,
            half_width=0.18,
            alpha=0.80,
            name="runtime_profile_target_disparity_hw0p18_alpha0p80",
        ),
        make_runtime_profile_method(
            camera,
            half_width=0.18,
            alpha=0.85,
            name="runtime_profile_target_disparity_hw0p18_alpha0p85",
        ),
    ]


def make_exp209_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    target_disparity = camera.fx_meas_px * camera.baseline_meas_m / TARGET_RANGE_M
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for half_width in (0.18, 0.30, 0.45, 0.70):
        for cap in (2.0, 3.0, 4.0, 5.0, 7.0):
            name = f"target_disparity_hw{half_width:g}_range_delta_cap{cap:g}".replace(".", "p")

            def apply(
                sample: pipeline.CalibrationSample,
                corr: np.ndarray,
                *,
                half_width=half_width,
                cap=cap,
                target_disparity=target_disparity,
            ) -> tuple[np.ndarray, bool]:
                hit = abs(float(sample.measured_disparity_px) - target_disparity) <= half_width
                return (point_with_range_delta_cap(sample.raw_rdf_m, corr, cap) if hit else corr, hit)

            methods.append(
                MethodSpec(
                    name,
                    "target-disparity neighborhood range-delta cap; runtime uses measured disparity and raw/corrected range delta",
                    apply,
                )
            )
    return methods


def make_exp210_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    target_disparity = camera.fx_meas_px * camera.baseline_meas_m / TARGET_RANGE_M
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for half_width in (0.18, 0.30, 0.45):
        for cap in (3.0, 4.0, 5.0):
            for alpha in (0.70, 0.80, 0.90):
                name = f"target_disparity_hw{half_width:g}_alpha{alpha:g}_cap{cap:g}".replace(".", "p")

                def apply(
                    sample: pipeline.CalibrationSample,
                    corr: np.ndarray,
                    *,
                    half_width=half_width,
                    cap=cap,
                    alpha=alpha,
                    target_disparity=target_disparity,
                ) -> tuple[np.ndarray, bool]:
                    hit = abs(float(sample.measured_disparity_px) - target_disparity) <= half_width
                    if not hit:
                        return corr, False
                    blended = blend(sample.raw_rdf_m, corr, alpha)
                    return point_with_range_delta_cap(sample.raw_rdf_m, blended, cap), True

                methods.append(
                    MethodSpec(
                        name,
                        "target-disparity neighborhood blend plus range-delta cap; runtime uses measured disparity and raw/corrected range delta",
                        apply,
                    )
                )
    return methods


def make_exp211_methods() -> list[MethodSpec]:
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for window in (40.0, 70.0, 110.0, 160.0, 240.0):
        for alpha in (0.0, 0.50, 0.70, 0.85):
            name = f"away_from_target_w{window:g}_alpha{alpha:g}".replace(".", "p")

            def apply(
                sample: pipeline.CalibrationSample,
                corr: np.ndarray,
                *,
                window=window,
                alpha=alpha,
            ) -> tuple[np.ndarray, bool]:
                raw_norm = raw_range(sample)
                corr_norm = float(np.linalg.norm(corr))
                delta = corr_norm - raw_norm
                raw_offset = raw_norm - TARGET_RANGE_M
                hit = abs(raw_offset) <= window and raw_offset * delta > 0.0
                return (blend(sample.raw_rdf_m, corr, alpha) if hit else corr, hit)

            methods.append(
                MethodSpec(
                    name,
                    "suppress corrections that move range farther from the mission target; runtime uses raw range, corrected range, and target range",
                    apply,
                )
            )
    return methods


def make_exp212_methods() -> list[MethodSpec]:
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for window in (70.0, 110.0, 160.0):
        for cap in (1.5, 2.0, 3.0, 4.0):
            name = f"away_from_target_w{window:g}_cap{cap:g}".replace(".", "p")

            def apply(
                sample: pipeline.CalibrationSample,
                corr: np.ndarray,
                *,
                window=window,
                cap=cap,
            ) -> tuple[np.ndarray, bool]:
                raw_norm = raw_range(sample)
                corr_norm = float(np.linalg.norm(corr))
                delta = corr_norm - raw_norm
                raw_offset = raw_norm - TARGET_RANGE_M
                hit = abs(raw_offset) <= window and raw_offset * delta > 0.0
                return (point_with_range_delta_cap(sample.raw_rdf_m, corr, cap) if hit else corr, hit)

            methods.append(
                MethodSpec(
                    name,
                    "cap corrections that move range farther from the mission target; runtime uses raw range, corrected range, and target range",
                    apply,
                )
            )
    return methods


def make_exp213_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    target_disparity = camera.fx_meas_px * camera.baseline_meas_m / TARGET_RANGE_M
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_target_disparity_blend_method(
            camera,
            half_width=0.18,
            alpha=0.80,
            name="target_disparity_hw0p18_alpha0p80_fixed",
        ),
    ]
    for half_width in (0.18, 0.30):
        for td_alpha in (0.80, 0.85):
            for away_window in (50.0, 70.0, 100.0):
                for away_alpha in (0.70, 0.85):
                    name = (
                        f"td_hw{half_width:g}_a{td_alpha:g}_away_w{away_window:g}_a{away_alpha:g}"
                    ).replace(".", "p")

                    def apply(
                        sample: pipeline.CalibrationSample,
                        corr: np.ndarray,
                        *,
                        half_width=half_width,
                        td_alpha=td_alpha,
                        away_window=away_window,
                        away_alpha=away_alpha,
                        target_disparity=target_disparity,
                    ) -> tuple[np.ndarray, bool]:
                        point = corr
                        changed = False
                        if abs(float(sample.measured_disparity_px) - target_disparity) <= half_width:
                            point = blend(sample.raw_rdf_m, point, td_alpha)
                            changed = True
                        raw_norm = raw_range(sample)
                        point_norm = float(np.linalg.norm(point))
                        raw_offset = raw_norm - TARGET_RANGE_M
                        delta = point_norm - raw_norm
                        if abs(raw_offset) <= away_window and raw_offset * delta > 0.0:
                            point = blend(sample.raw_rdf_m, point, away_alpha)
                            changed = True
                        return point, changed

                    methods.append(
                        MethodSpec(
                            name,
                            "combined target-disparity blend and away-from-target suppression",
                            apply,
                        )
                    )
    return methods


def make_exp214_methods() -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        MethodSpec(
            "away_from_target_w70_alpha0p85_fixed",
            "fixed away-from-target blend stability audit",
            lambda sample, corr: (
                blend(sample.raw_rdf_m, corr, 0.85)
                if abs(raw_range(sample) - TARGET_RANGE_M) <= 70.0
                and (raw_range(sample) - TARGET_RANGE_M) * (float(np.linalg.norm(corr)) - raw_range(sample)) > 0.0
                else corr,
                abs(raw_range(sample) - TARGET_RANGE_M) <= 70.0
                and (raw_range(sample) - TARGET_RANGE_M) * (float(np.linalg.norm(corr)) - raw_range(sample)) > 0.0,
            ),
        ),
    ]


def make_exp215_methods() -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        MethodSpec(
            "away_from_target_w70_cap4_fixed",
            "fixed away-from-target range-delta cap stability audit",
            lambda sample, corr: (
                point_with_range_delta_cap(sample.raw_rdf_m, corr, 4.0)
                if abs(raw_range(sample) - TARGET_RANGE_M) <= 70.0
                and (raw_range(sample) - TARGET_RANGE_M) * (float(np.linalg.norm(corr)) - raw_range(sample)) > 0.0
                else corr,
                abs(raw_range(sample) - TARGET_RANGE_M) <= 70.0
                and (raw_range(sample) - TARGET_RANGE_M) * (float(np.linalg.norm(corr)) - raw_range(sample)) > 0.0,
            ),
        ),
    ]


def make_exp216_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    target_disparity = camera.fx_meas_px * camera.baseline_meas_m / TARGET_RANGE_M

    def combo(sample: pipeline.CalibrationSample, corr: np.ndarray) -> tuple[np.ndarray, bool]:
        point = corr
        changed = False
        if abs(float(sample.measured_disparity_px) - target_disparity) <= 0.18:
            point = blend(sample.raw_rdf_m, point, 0.80)
            changed = True
        raw_norm = raw_range(sample)
        delta = float(np.linalg.norm(point)) - raw_norm
        raw_offset = raw_norm - TARGET_RANGE_M
        if abs(raw_offset) <= 50.0 and raw_offset * delta > 0.0:
            point = blend(sample.raw_rdf_m, point, 0.85)
            changed = True
        return point, changed

    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        MethodSpec("td_hw0p18_a0p8_away_w50_a0p85_fixed", "fixed combined guard stability audit", combo),
    ]


def make_exp217_methods() -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_away_runtime_profile_method(
            window_m=70.0,
            alpha=0.85,
            name="runtime_profile_away_from_target_w70_alpha0p85",
        ),
    ]


def make_exp226_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.24,
            target_alpha=0.90,
            away_window_m=100.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p24_a0p9_away_w100_a0p9",
        ),
    ]


def make_exp235_methods() -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_away_runtime_profile_method(
            window_m=70.0,
            alpha=0.50,
            name="runtime_profile_away_from_target_w70_alpha0p5",
        ),
        make_away_runtime_profile_method(
            window_m=140.0,
            alpha=0.50,
            name="runtime_profile_away_from_target_w140_alpha0p5",
        ),
        make_away_runtime_profile_method(
            window_m=220.0,
            alpha=0.50,
            name="runtime_profile_away_from_target_w220_alpha0p5",
        ),
    ]


def make_exp246_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.24,
            target_alpha=0.90,
            away_window_m=70.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p24_a0p9_away_w70_a0p9",
        ),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.30,
            target_alpha=0.85,
            away_window_m=100.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p3_a0p85_away_w100_a0p9",
        ),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.18,
            target_alpha=0.75,
            away_window_m=70.0,
            away_alpha=0.85,
            name="runtime_profile_td_hw0p18_a0p75_away_w70_a0p85",
        ),
    ]


def make_exp261_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.18,
            target_alpha=0.80,
            away_window_m=50.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p18_a0p8_away_w50_a0p9",
        ),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.30,
            target_alpha=0.85,
            away_window_m=50.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p3_a0p85_away_w50_a0p9",
        ),
    ]


def make_exp321_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.24,
            target_alpha=0.85,
            away_window_m=70.0,
            away_alpha=0.85,
            name="runtime_profile_td_hw0p24_a0p85_away_w70_a0p85",
        ),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.24,
            target_alpha=0.75,
            away_window_m=50.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p24_a0p75_away_w50_a0p9",
        ),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.24,
            target_alpha=0.90,
            away_window_m=100.0,
            away_alpha=0.85,
            name="runtime_profile_td_hw0p24_a0p9_away_w100_a0p85",
        ),
    ]


def make_exp381_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_cap_runtime_profile_method(
            camera,
            half_width=0.18,
            cap_m=4.0,
            name="runtime_profile_tdcap_hw0p18_cap4",
        ),
        make_cap_runtime_profile_method(
            camera,
            half_width=0.30,
            cap_m=4.0,
            name="runtime_profile_tdcap_hw0p3_cap4",
        ),
        make_cap_runtime_profile_method(
            camera,
            half_width=0.18,
            cap_m=3.0,
            name="runtime_profile_tdcap_hw0p18_cap3",
        ),
    ]


def make_exp441_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.18,
            target_alpha=0.85,
            away_window_m=70.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p18_a0p85_away_w70_a0p9",
        ),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.24,
            target_alpha=0.85,
            away_window_m=100.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p24_a0p85_away_w100_a0p9",
        ),
    ]


def make_exp481_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.30,
            target_alpha=0.80,
            away_window_m=50.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p3_a0p8_away_w50_a0p9",
        ),
        make_runtime_profile_method(
            camera,
            half_width=0.18,
            alpha=0.85,
            name="runtime_profile_target_disparity_hw0p18_alpha0p85",
        ),
    ]


def make_exp541_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.24,
            target_alpha=0.80,
            away_window_m=50.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p24_a0p8_away_w50_a0p9",
        ),
    ]


def make_exp581_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.18,
            target_alpha=0.80,
            away_window_m=50.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p18_a0p8_away_w50_a0p9_repeat",
        ),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.24,
            target_alpha=0.85,
            away_window_m=100.0,
            away_alpha=0.85,
            name="runtime_profile_td_hw0p24_a0p85_away_w100_a0p85_repeat",
        ),
    ]


def make_exp621_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.24,
            target_alpha=0.75,
            away_window_m=70.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p24_a0p75_away_w70_a0p9",
        ),
    ]


def make_exp641_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    return [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.18,
            target_alpha=0.90,
            away_window_m=70.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p18_a0p9_away_w70_a0p9",
        ),
        make_combined_runtime_profile_method(
            camera,
            half_width=0.18,
            target_alpha=0.85,
            away_window_m=70.0,
            away_alpha=0.90,
            name="runtime_profile_td_hw0p18_a0p85_away_w70_a0p9_repeat",
        ),
    ]


EXP921_PROFILE_CANDIDATES = {
    "runtime_profile_exp899_td_hw0p18_a0p75_away_w100_a0p9": (0.18, 0.75, 100.0, 0.90),
    "runtime_profile_exp891_td_hw0p24_a0p9_away_w100_a0p85": (0.24, 0.90, 100.0, 0.85),
    "runtime_profile_exp873_td_hw0p3_a0p9_away_w50_a0p9": (0.30, 0.90, 50.0, 0.90),
}


def make_exp921_methods(camera: pipeline.CameraModel) -> list[MethodSpec]:
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for name, (half_width, target_alpha, away_window_m, away_alpha) in EXP921_PROFILE_CANDIDATES.items():
        methods.append(
            make_combined_runtime_profile_method(
                camera,
                half_width=half_width,
                target_alpha=target_alpha,
                away_window_m=away_window_m,
                away_alpha=away_alpha,
                name=name,
            )
        )
    return methods


def combined_profile_data(
    camera: pipeline.CameraModel,
    *,
    profile_id: str,
    half_width: float,
    target_alpha: float,
    away_window_m: float,
    away_alpha: float,
) -> dict[str, object]:
    profile_data = json.loads(PROFILE_JSON.read_text(encoding="utf-8"))
    profile_data["profile_id"] = profile_id
    profile_data["target_disparity_blend_guard"] = target_disparity_guard_payload(
        camera,
        half_width=half_width,
        alpha=target_alpha,
    )
    profile_data["target_range_away_blend_guard"] = target_range_away_guard_payload(
        window_m=away_window_m,
        alpha=away_alpha,
    )
    return profile_data


def recipes(camera: pipeline.CameraModel) -> dict[int, tuple[str, str, Callable[[], list[MethodSpec]]]]:
    return {
        199: (
            "bbox-outlier-conditional-raw-target-window-blend-search",
            "Search raw-range target-neighborhood blend guards for outlier-stress target max without runtime truth.",
            make_exp199_methods,
        ),
        200: (
            "bbox-outlier-conditional-raw-target-range-delta-cap-search",
            "Search target-neighborhood range-delta caps using raw range and corrected/raw delta only.",
            make_exp200_methods,
        ),
        201: (
            "bbox-outlier-target-disparity-neighborhood-blend-search",
            "Search measured-disparity target-neighborhood blend guards without truth.",
            lambda: make_exp201_methods(camera),
        ),
        202: (
            "bbox-outlier-raw-rdf-lateral-down-outlier-blend-search",
            "Search raw RDF lateral/down outlier guards for bbox stress.",
            make_exp202_methods,
        ),
        203: (
            "bbox-outlier-combined-target-lateral-blend-search",
            "Search combined raw target-window and lateral outlier guards.",
            make_exp203_methods,
        ),
        204: (
            "bbox-outlier-target-disparity-blend-800seed-stability-audit",
            "Audit fixed EXP-201 measured-disparity blend guard on 800 fresh outlier-stress seeds.",
            lambda: make_exp204_methods(camera),
        ),
        205: (
            "bbox-outlier-target-disparity-blend-fine-search",
            "Fine search around EXP-201 measured-disparity half-width and blend alpha.",
            lambda: make_exp205_methods(camera),
        ),
        206: (
            "bbox-outlier-target-disparity-blend-neighbor-stability-audit",
            "Audit nearby measured-disparity blend guards for stability after EXP-205 fine search.",
            lambda: make_exp206_methods(camera),
        ),
        207: (
            "bbox-outlier-target-disparity-blend-runtime-profile-replay-audit",
            "Replay actual runtime profile JSON with target_disparity_blend_guard on fresh outlier-stress seeds.",
            lambda: make_exp207_methods(camera),
        ),
        208: (
            "bbox-outlier-target-disparity-blend-runtime-profile-alpha-audit",
            "Replay actual runtime profile JSON variants for target-disparity blend alpha on fresh outlier-stress seeds.",
            lambda: make_exp208_methods(camera),
        ),
        209: (
            "bbox-outlier-target-disparity-range-delta-cap-search",
            "Search target-disparity range-delta caps to reduce target-window worst-case error.",
            lambda: make_exp209_methods(camera),
        ),
        210: (
            "bbox-outlier-target-disparity-blend-cap-search",
            "Search target-disparity blend plus range-delta cap combinations.",
            lambda: make_exp210_methods(camera),
        ),
        211: (
            "bbox-outlier-away-from-target-blend-search",
            "Search guards that suppress corrections moving range farther from the mission target.",
            make_exp211_methods,
        ),
        212: (
            "bbox-outlier-away-from-target-cap-search",
            "Search range-delta caps for corrections moving range farther from the mission target.",
            make_exp212_methods,
        ),
        213: (
            "bbox-outlier-target-disparity-away-combo-search",
            "Search combined measured-disparity target blend and away-from-target suppression.",
            lambda: make_exp213_methods(camera),
        ),
        214: (
            "bbox-outlier-away-from-target-blend-800seed-stability-audit",
            "Audit fixed away-from-target blend guard on 800 fresh outlier-stress seeds.",
            make_exp214_methods,
        ),
        215: (
            "bbox-outlier-away-from-target-cap-800seed-stability-audit",
            "Audit fixed away-from-target range-delta cap guard on 800 fresh outlier-stress seeds.",
            make_exp215_methods,
        ),
        216: (
            "bbox-outlier-target-disparity-away-combo-800seed-stability-audit",
            "Audit fixed target-disparity plus away-from-target combo guard on 800 fresh outlier-stress seeds.",
            lambda: make_exp216_methods(camera),
        ),
        217: (
            "bbox-outlier-away-from-target-runtime-profile-replay-audit",
            "Replay actual runtime profile JSON with target_range_away_blend_guard on fresh outlier-stress seeds.",
            make_exp217_methods,
        ),
        226: (
            "bbox-outlier-target-disparity-away-combo-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for EXP-224 best combined guard on fresh outlier-stress seeds.",
            lambda: make_exp226_methods(camera),
        ),
        235: (
            "bbox-outlier-away-from-target-max-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON variants for target-max-oriented away-from-target alpha0.5 guards.",
            make_exp235_methods,
        ),
        246: (
            "bbox-outlier-generic-combo-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for repeated generic td-away combo candidates.",
            lambda: make_exp246_methods(camera),
        ),
        261: (
            "bbox-outlier-exp248-combo-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for EXP-248 aggregate candidate and neighbor.",
            lambda: make_exp261_methods(camera),
        ),
        321: (
            "bbox-outlier-exp308-exp313-combo-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for EXP-308 and EXP-313 combo candidates.",
            lambda: make_exp321_methods(camera),
        ),
        381: (
            "bbox-outlier-target-disparity-cap-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for repeated target-disparity range-delta cap candidates.",
            lambda: make_exp381_methods(camera),
        ),
        441: (
            "bbox-outlier-exp437-exp439-combo-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for EXP-437 and EXP-439 combo candidates.",
            lambda: make_exp441_methods(camera),
        ),
        481: (
            "bbox-outlier-exp472-exp477-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for EXP-472 combo and EXP-477 target-disparity candidates.",
            lambda: make_exp481_methods(camera),
        ),
        541: (
            "bbox-outlier-exp538-combo-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for EXP-538 combo candidate.",
            lambda: make_exp541_methods(camera),
        ),
        581: (
            "bbox-outlier-exp575-repeat-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for repeated EXP-575 and EXP-578 candidates.",
            lambda: make_exp581_methods(camera),
        ),
        621: (
            "bbox-outlier-exp614-combo-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for EXP-614 combo candidate.",
            lambda: make_exp621_methods(camera),
        ),
        641: (
            "bbox-outlier-exp635-combo-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for EXP-635 and neighbor combo candidates.",
            lambda: make_exp641_methods(camera),
        ),
        921: (
            "bbox-outlier-exp899-exp891-exp873-combo-runtime-profile-800seed-audit",
            "Replay actual runtime profile JSON for strongest EXP-873/891/899 generic combo candidates.",
            lambda: make_exp921_methods(camera),
        ),
    }


def make_generic_runtime_guard_methods(exp_num: int, camera: pipeline.CameraModel) -> list[MethodSpec]:
    rng = np.random.default_rng(20260616 + int(exp_num) * 7919)
    target_disparity = camera.fx_meas_px * camera.baseline_meas_m / TARGET_RANGE_M
    methods = [
        MethodSpec("nominal_uncapped", "baseline fixed d6/d7 profile", lambda sample, corr: (corr, False)),
    ]
    for index in range(24):
        family = int(rng.integers(0, 6))
        if family == 0:
            half_width = float(rng.choice([0.12, 0.18, 0.24, 0.30, 0.45, 0.70]))
            alpha = float(rng.choice([0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95]))
            name = f"generic{index:02d}_td_hw{half_width:g}_alpha{alpha:g}".replace(".", "p")

            def apply(sample: pipeline.CalibrationSample, corr: np.ndarray, *, half_width=half_width, alpha=alpha) -> tuple[np.ndarray, bool]:
                hit = abs(float(sample.measured_disparity_px) - target_disparity) <= half_width
                return (blend(sample.raw_rdf_m, corr, alpha) if hit else corr, hit)

        elif family == 1:
            window = float(rng.choice([40.0, 70.0, 100.0, 140.0, 220.0]))
            alpha = float(rng.choice([0.50, 0.70, 0.80, 0.85, 0.90]))
            name = f"generic{index:02d}_away_w{window:g}_alpha{alpha:g}".replace(".", "p")

            def apply(sample: pipeline.CalibrationSample, corr: np.ndarray, *, window=window, alpha=alpha) -> tuple[np.ndarray, bool]:
                raw_norm = raw_range(sample)
                delta = float(np.linalg.norm(corr)) - raw_norm
                raw_offset = raw_norm - TARGET_RANGE_M
                hit = abs(raw_offset) <= window and raw_offset * delta > 0.0
                return (blend(sample.raw_rdf_m, corr, alpha) if hit else corr, hit)

        elif family == 2:
            window = float(rng.choice([40.0, 70.0, 100.0, 150.0]))
            alpha = float(rng.choice([0.65, 0.75, 0.85, 0.95]))
            name = f"generic{index:02d}_rawtarget_w{window:g}_alpha{alpha:g}".replace(".", "p")

            def apply(sample: pipeline.CalibrationSample, corr: np.ndarray, *, window=window, alpha=alpha) -> tuple[np.ndarray, bool]:
                hit = abs(raw_range(sample) - TARGET_RANGE_M) <= window
                return (blend(sample.raw_rdf_m, corr, alpha) if hit else corr, hit)

        elif family == 3:
            lateral = float(rng.choice([30.0, 40.0, 50.0, 65.0]))
            vertical = float(rng.choice([16.0, 22.0, 30.0]))
            alpha = float(rng.choice([0.70, 0.80, 0.90]))
            name = f"generic{index:02d}_lat{lateral:g}_vert{vertical:g}_alpha{alpha:g}".replace(".", "p")

            def apply(sample: pipeline.CalibrationSample, corr: np.ndarray, *, lateral=lateral, vertical=vertical, alpha=alpha) -> tuple[np.ndarray, bool]:
                raw = sample.raw_rdf_m
                hit = abs(float(raw[0])) >= lateral or abs(float(raw[1])) >= vertical
                return (blend(raw, corr, alpha) if hit else corr, hit)

        elif family == 4:
            half_width = float(rng.choice([0.18, 0.24, 0.30, 0.45]))
            td_alpha = float(rng.choice([0.75, 0.80, 0.85, 0.90]))
            away_window = float(rng.choice([50.0, 70.0, 100.0]))
            away_alpha = float(rng.choice([0.75, 0.85, 0.90]))
            name = f"generic{index:02d}_tdaway_hw{half_width:g}_ta{td_alpha:g}_w{away_window:g}_aa{away_alpha:g}".replace(".", "p")

            def apply(
                sample: pipeline.CalibrationSample,
                corr: np.ndarray,
                *,
                half_width=half_width,
                td_alpha=td_alpha,
                away_window=away_window,
                away_alpha=away_alpha,
            ) -> tuple[np.ndarray, bool]:
                point = corr
                changed = False
                if abs(float(sample.measured_disparity_px) - target_disparity) <= half_width:
                    point = blend(sample.raw_rdf_m, point, td_alpha)
                    changed = True
                raw_norm = raw_range(sample)
                delta = float(np.linalg.norm(point)) - raw_norm
                raw_offset = raw_norm - TARGET_RANGE_M
                if abs(raw_offset) <= away_window and raw_offset * delta > 0.0:
                    point = blend(sample.raw_rdf_m, point, away_alpha)
                    changed = True
                return point, changed

        else:
            half_width = float(rng.choice([0.18, 0.30, 0.45, 0.70]))
            cap = float(rng.choice([2.0, 3.0, 4.0, 5.0, 7.0]))
            name = f"generic{index:02d}_tdcap_hw{half_width:g}_cap{cap:g}".replace(".", "p")

            def apply(sample: pipeline.CalibrationSample, corr: np.ndarray, *, half_width=half_width, cap=cap) -> tuple[np.ndarray, bool]:
                hit = abs(float(sample.measured_disparity_px) - target_disparity) <= half_width
                return (point_with_range_delta_cap(sample.raw_rdf_m, corr, cap) if hit else corr, hit)

        methods.append(MethodSpec(name, "deterministic generic runtime-safe guard search", apply))
    return methods


def load_samples(seed: int, camera: pipeline.CameraModel) -> list[pipeline.CalibrationSample]:
    source = pipeline.SyntheticCalibrationDataSource(
        camera=camera,
        injection=pipeline.ErrorInjection(
            disparity_outlier_rate=0.06,
            disparity_outlier_px=0.45,
            bbox_outlier_rate=0.08,
            bbox_outlier_px=2.5,
        ),
        sample_count=420,
        seed=seed,
        target_range_m=TARGET_RANGE_M,
        target_window_m=TARGET_WINDOW_M,
        target_oversample_count=72,
    )
    return source.load_samples()


def rmse(values: np.ndarray) -> float:
    if values.size == 0:
        return math.nan
    return float(math.sqrt(float(np.mean(values * values))))


def summarize_method(
    *,
    name: str,
    predictions: Sequence[np.ndarray],
    truth_rows: Sequence[np.ndarray],
    raw_rows: Sequence[np.ndarray],
    invalid_count: int,
    fallback_count: int,
    changed_count: int,
    per_seed_passes: int,
    trial_count: int,
) -> dict[str, object]:
    truth = np.vstack(truth_rows)
    raw = np.vstack(raw_rows)
    corrected = np.vstack(predictions)
    truth_range = np.linalg.norm(truth, axis=1)
    raw_range_values = np.linalg.norm(raw, axis=1)
    corrected_range = np.linalg.norm(corrected, axis=1)
    raw_range_error = raw_range_values - truth_range
    corrected_range_error = corrected_range - truth_range
    position_error = np.linalg.norm(corrected - truth, axis=1)
    raw_position_error = np.linalg.norm(raw - truth, axis=1)
    target_mask = np.abs(truth_range - TARGET_RANGE_M) <= TARGET_WINDOW_M
    target_errors = corrected_range_error[target_mask]
    raw_target_errors = raw_range_error[target_mask]
    status = (
        "STRICT_PASS"
        if invalid_count == 0
        and rmse(corrected_range_error) < rmse(raw_range_error)
        and rmse(position_error) < rmse(raw_position_error)
        and int(target_mask.sum()) > 0
        else "FAIL"
    )
    return {
        "method": name,
        "validation_sample_count": int(truth.shape[0]),
        "invalid_count": int(invalid_count),
        "fallback_count": int(fallback_count),
        "pass_count": int(per_seed_passes),
        "trial_count": int(trial_count),
        "changed_count": int(changed_count),
        "status": status,
        "validation_range_rmse_raw_m": rmse(raw_range_error),
        "validation_range_rmse_corrected_m": rmse(corrected_range_error),
        "validation_position_rmse_raw_m": rmse(raw_position_error),
        "validation_position_rmse_corrected_m": rmse(position_error),
        "validation_right_rmse_corrected_m": rmse(corrected[:, 0] - truth[:, 0]),
        "validation_down_rmse_corrected_m": rmse(corrected[:, 1] - truth[:, 1]),
        "validation_forward_rmse_corrected_m": rmse(corrected[:, 2] - truth[:, 2]),
        "target_1852m_sample_count": int(target_mask.sum()),
        "target_1852m_rmse_raw_m": rmse(raw_target_errors),
        "target_1852m_rmse_m": rmse(target_errors),
        "target_1852m_max_abs_error_m": float(np.max(np.abs(target_errors))) if target_errors.size else math.nan,
        "runtime_truth_used": False,
    }


def run_experiment(
    *,
    exp_num: int,
    exp_id: str,
    purpose: str,
    method_specs: Sequence[MethodSpec],
    seeds: Sequence[int],
    camera: pipeline.CameraModel,
    profile: EstimatedToTruthSim3Profile,
) -> dict[str, object]:
    truth_by_method: dict[str, list[np.ndarray]] = {method.name: [] for method in method_specs}
    raw_by_method: dict[str, list[np.ndarray]] = {method.name: [] for method in method_specs}
    pred_by_method: dict[str, list[np.ndarray]] = {method.name: [] for method in method_specs}
    invalid_by_method = {method.name: 0 for method in method_specs}
    fallback_by_method = {method.name: 0 for method in method_specs}
    changed_by_method = {method.name: 0 for method in method_specs}
    pass_by_method = {method.name: 0 for method in method_specs}

    for seed in seeds:
        samples = load_samples(seed, camera)
        seed_truth = np.vstack([sample.truth_rdf_m for sample in samples])
        seed_raw = np.vstack([sample.raw_rdf_m for sample in samples])
        seed_raw_range_error = np.linalg.norm(seed_raw, axis=1) - np.linalg.norm(seed_truth, axis=1)
        seed_raw_position_error = np.linalg.norm(seed_raw - seed_truth, axis=1)
        seed_preds: dict[str, list[np.ndarray]] = {method.name: [] for method in method_specs}
        for sample in samples:
            nominal, reason = profile_apply(profile, sample)
            for method in method_specs:
                truth_by_method[method.name].append(sample.truth_rdf_m)
                raw_by_method[method.name].append(sample.raw_rdf_m)
                if nominal is None:
                    invalid_by_method[method.name] += 1
                    pred = sample.raw_rdf_m
                else:
                    pred, changed = method.apply(sample, nominal)
                    changed_by_method[method.name] += int(changed)
                    if reason != "ok":
                        fallback_by_method[method.name] += 1
                pred = np.asarray(pred, dtype=float)
                pred_by_method[method.name].append(pred)
                seed_preds[method.name].append(pred)
        for method in method_specs:
            seed_corrected = np.vstack(seed_preds[method.name])
            seed_range_error = np.linalg.norm(seed_corrected, axis=1) - np.linalg.norm(seed_truth, axis=1)
            seed_position_error = np.linalg.norm(seed_corrected - seed_truth, axis=1)
            if rmse(seed_range_error) < rmse(seed_raw_range_error) and rmse(seed_position_error) < rmse(seed_raw_position_error):
                pass_by_method[method.name] += 1

    rows = [
        summarize_method(
            name=method.name,
            predictions=pred_by_method[method.name],
            truth_rows=truth_by_method[method.name],
            raw_rows=raw_by_method[method.name],
            invalid_count=invalid_by_method[method.name],
            fallback_count=fallback_by_method[method.name],
            changed_count=changed_by_method[method.name],
            per_seed_passes=pass_by_method[method.name],
            trial_count=len(seeds),
        )
        for method in method_specs
    ]
    best_range = min(rows, key=lambda row: float(row["validation_range_rmse_corrected_m"]))
    best_position = min(rows, key=lambda row: float(row["validation_position_rmse_corrected_m"]))
    best_target = min(rows, key=lambda row: float(row["target_1852m_rmse_m"]))
    best_max = min(rows, key=lambda row: float(row["target_1852m_max_abs_error_m"]))
    status = "STRICT_PASS" if any(row["status"] == "STRICT_PASS" for row in rows) else "FAIL"
    return {
        "experiment_id": exp_id,
        "experiment_number": exp_num,
        "status": status,
        "stage": exp_id.split("-", 2)[-1],
        "purpose": purpose,
        "profile_json": str(PROFILE_JSON),
        "seed_start": int(seeds[0]),
        "seed_end": int(seeds[-1]),
        "trial_count": len(seeds),
        "coordinate_contracts_explicit": True,
        "px4_truth_frame": "px4_local_position_ned",
        "px4_truth_axis_order": "north,east,down",
        "internal_correction_frame": "stereo_center_opencv_optical",
        "internal_correction_axis_order": "right,down,forward",
        "public_stereodistance_topic_order": "forward,right,down,range",
        "truth_rdf_transform_contract": TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": NED_TO_RDF_CONTRACT,
        "runtime_truth_used": False,
        "methods": rows,
        "best_method": best_range,
        "best_position_rmse_method": best_position,
        "best_target_rmse_method": best_target,
        "best_target_max_abs_method": best_max,
    }


def write_outputs(exp_dir: Path, metrics: Mapping[str, object], method_specs: Sequence[MethodSpec]) -> None:
    exp_dir.mkdir(parents=True, exist_ok=True)
    (exp_dir / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    methods = list(metrics["methods"])  # type: ignore[index]
    with (exp_dir / "method_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(methods[0].keys())
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(methods)
    purpose_by_name = {method.name: method.purpose for method in method_specs}
    best = metrics["best_method"]  # type: ignore[index]
    best_target = metrics["best_target_rmse_method"]  # type: ignore[index]
    best_max = metrics["best_target_max_abs_method"]  # type: ignore[index]
    readme = [
        f"# {metrics['experiment_id']}",
        "",
        f"Purpose: {metrics['purpose']}",
        "",
        "Method: replay fixed d6/d7 runtime profile under bbox/disparity outlier stress, then apply runtime-safe guard variants. Runtime inputs are raw RDF/range, measured disparity, and fixed profile output; truth is used only for offline scoring.",
        "",
        "Coordinate contract: validation truth must be stereo-center OpenCV optical RDF from an explicit T_source_to_stereo_rdf; direct NED-to-RDF axis aliasing is forbidden. Internal correction is RDF `right/down/forward`; public StereoDistance topics remain FRD+range `forward/right/down/range`.",
        "",
        "Pass/fail criteria: STRICT_PASS if any method improves aggregate range and position RMSE versus raw, has invalid count 0, nonzero target-window coverage, and runtime truth is unused.",
        "",
        f"Result: {metrics['status']}. Best range `{best['method']}` range RMSE {best['validation_range_rmse_corrected_m']:.9f} m. Best target `{best_target['method']}` target RMSE {best_target['target_1852m_rmse_m']:.9f} m. Best target max `{best_max['method']}` max abs {best_max['target_1852m_max_abs_error_m']:.9f} m.",
        "",
        "Method purposes:",
    ]
    for name, purpose in sorted(purpose_by_name.items()):
        readme.append(f"- `{name}`: {purpose}")
    (exp_dir / "README.md").write_text("\n".join(readme) + "\n", encoding="utf-8")
    (exp_dir / "command.md").write_text(
        "# Command\n\nGenerated by `apps/Simulation/scripts/continue_bbox_outlier_runtime_guards.py`.\n",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", type=int, default=199)
    parser.add_argument("--end", type=int, default=203)
    parser.add_argument("--seed-start", type=int, default=20298159)
    parser.add_argument("--trials", type=int, default=200)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    camera = pipeline.CameraModel()
    profile = EstimatedToTruthSim3Profile.from_json_dict(json.loads(PROFILE_JSON.read_text(encoding="utf-8")))
    recipe_map = recipes(camera)
    seed_cursor = int(args.seed_start)
    for exp_num in range(args.start, args.end + 1):
        if exp_num in recipe_map:
            slug, purpose, factory = recipe_map[exp_num]
        elif exp_num >= 218:
            slug = f"bbox-outlier-generic-runtime-guard-search-{exp_num:03d}"
            purpose = "Deterministic generic runtime-safe guard search after hand-authored bbox-outlier recipes."
            factory = lambda exp_num=exp_num: make_generic_runtime_guard_methods(exp_num, camera)
        else:
            continue
        exp_id = f"EXP-20260616-{exp_num:03d}-{slug}"
        exp_dir = REPORT_ROOT / exp_id
        seeds = list(range(seed_cursor, seed_cursor + int(args.trials)))
        seed_cursor += int(args.trials)
        methods = factory()
        metrics = run_experiment(
            exp_num=exp_num,
            exp_id=exp_id,
            purpose=purpose,
            method_specs=methods,
            seeds=seeds,
            camera=camera,
            profile=profile,
        )
        write_outputs(exp_dir, metrics, methods)
        if exp_num == 207:
            profile_data = json.loads(PROFILE_JSON.read_text(encoding="utf-8"))
            profile_data["profile_id"] = "d6d7_target_disparity_hw0p18_alpha0p75_runtime_profile"
            profile_data["target_disparity_blend_guard"] = target_disparity_guard_payload(
                camera,
                half_width=0.18,
                alpha=0.75,
            )
            output_dir = exp_dir / "output"
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / "recommended_runtime_profile.json").write_text(
                json.dumps(profile_data, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        if exp_num == 217:
            profile_data = json.loads(PROFILE_JSON.read_text(encoding="utf-8"))
            profile_data["profile_id"] = "d6d7_away_from_target_w70_alpha0p85_runtime_profile"
            profile_data["target_range_away_blend_guard"] = target_range_away_guard_payload(
                window_m=70.0,
                alpha=0.85,
            )
            output_dir = exp_dir / "output"
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / "recommended_runtime_profile.json").write_text(
                json.dumps(profile_data, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        if exp_num == 226:
            profile_data = json.loads(PROFILE_JSON.read_text(encoding="utf-8"))
            profile_data["profile_id"] = "d6d7_td_hw0p24_a0p9_away_w100_a0p9_runtime_profile"
            profile_data["target_disparity_blend_guard"] = target_disparity_guard_payload(
                camera,
                half_width=0.24,
                alpha=0.90,
            )
            profile_data["target_range_away_blend_guard"] = target_range_away_guard_payload(
                window_m=100.0,
                alpha=0.90,
            )
            output_dir = exp_dir / "output"
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / "recommended_runtime_profile.json").write_text(
                json.dumps(profile_data, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        if exp_num == 921:
            best_method_name = str(metrics["best_method"]["method"])  # type: ignore[index]
            if best_method_name in EXP921_PROFILE_CANDIDATES:
                half_width, target_alpha, away_window_m, away_alpha = EXP921_PROFILE_CANDIDATES[best_method_name]
                profile_data = combined_profile_data(
                    camera,
                    profile_id=f"d6d7_{best_method_name}",
                    half_width=half_width,
                    target_alpha=target_alpha,
                    away_window_m=away_window_m,
                    away_alpha=away_alpha,
                )
                output_dir = exp_dir / "output"
                output_dir.mkdir(parents=True, exist_ok=True)
                (output_dir / "recommended_runtime_profile.json").write_text(
                    json.dumps(profile_data, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
        best = metrics["best_method"]
        best_target = metrics["best_target_rmse_method"]
        best_max = metrics["best_target_max_abs_method"]
        print(
            exp_id,
            metrics["status"],
            "best_range",
            best["method"],
            f"{best['validation_range_rmse_corrected_m']:.6f}",
            "best_target",
            best_target["method"],
            f"{best_target['target_1852m_rmse_m']:.6f}",
            "best_max",
            best_max["method"],
            f"{best_max['target_1852m_max_abs_error_m']:.6f}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
