from __future__ import annotations

import json
import math
import re
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Iterable, Mapping, Sequence

import numpy as np


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
DEFAULT_CAMERA_WIDTH_PX = 1280.0
DEFAULT_CAMERA_HORIZONTAL_FOV_RAD = 0.154997710698
DEFAULT_CAMERA_BASELINE_M = 25.0
DEFAULT_NOMINAL_DISPARITY_FORWARD_PRODUCT_PX_M = (
    (DEFAULT_CAMERA_WIDTH_PX * 0.5)
    / math.tan(DEFAULT_CAMERA_HORIZONTAL_FOV_RAD * 0.5)
    * DEFAULT_CAMERA_BASELINE_M
)
RUNTIME_FEATURE_FIELDS = (
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
    "raw_range_m",
    "measured_disparity_px",
    "left_u_px",
    "left_v_px",
)
FORBIDDEN_RUNTIME_FIELDS = (
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "truth_north_m",
    "truth_east_m",
    "truth_down_ned_m",
    "validation_range_rmse_corrected_m",
    "target_max_abs_error_corrected_m",
)
RULE_RE = re.compile(
    r"unless\s+(?P<feature>feature_[A-Za-z0-9_]+)\s+(?P<direction>ge|le|gt|lt)\s+(?P<threshold>[-+0-9.eE]+)",
)


def finite_float(value: object, default: float = float("nan")) -> float:
    if value in (None, ""):
        return default
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def finite_array(values: Sequence[float]) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    return arr[np.isfinite(arr)]


def safe_mean(values: np.ndarray) -> float:
    return float(np.mean(values)) if values.size else float("nan")


def safe_std(values: np.ndarray) -> float:
    return float(np.std(values)) if values.size else float("nan")


def safe_percentile(values: np.ndarray, percentile: float) -> float:
    return float(np.percentile(values, percentile)) if values.size else float("nan")


def percentile_iqr(values: np.ndarray) -> float:
    if values.size == 0:
        return float("nan")
    q75, q25 = np.percentile(values, [75.0, 25.0])
    return float(q75 - q25)


@dataclass(frozen=True)
class RuntimeMeasurementSample:
    raw_right_m: float
    raw_down_m: float
    raw_forward_m: float
    measured_disparity_px: float
    raw_range_m: float | None = None
    left_u_px: float | None = None
    left_v_px: float | None = None

    @classmethod
    def from_public_frd_range(
        cls,
        estimated_frd_range: Sequence[object],
        measured_disparity_px: object,
        left_u_px: object | None = None,
        left_v_px: object | None = None,
    ) -> "RuntimeMeasurementSample | None":
        """Build a runtime sample from the public StereoDistance FRD topic.

        The public topic contract is ``[forward, right, down, range]`` in meters.
        Selector features are computed in stereo-center optical RDF
        ``[right, down, forward]``. This adapter is the only axis reorder needed
        when feeding ``/stereo/estimated_position`` into the packaged selector.
        """
        if len(estimated_frd_range) < 3:
            return None
        raw_forward = finite_float(estimated_frd_range[0])
        raw_right = finite_float(estimated_frd_range[1])
        raw_down = finite_float(estimated_frd_range[2])
        raw_range = finite_float(estimated_frd_range[3]) if len(estimated_frd_range) >= 4 else float("nan")
        disparity = finite_float(measured_disparity_px)
        if not all(math.isfinite(value) for value in (raw_right, raw_down, raw_forward, disparity)):
            return None
        if not math.isfinite(raw_range):
            raw_range = math.sqrt(raw_right * raw_right + raw_down * raw_down + raw_forward * raw_forward)
        left_u = finite_float(left_u_px)
        left_v = finite_float(left_v_px)
        return cls(
            raw_right_m=raw_right,
            raw_down_m=raw_down,
            raw_forward_m=raw_forward,
            measured_disparity_px=disparity,
            raw_range_m=raw_range,
            left_u_px=left_u if math.isfinite(left_u) else None,
            left_v_px=left_v if math.isfinite(left_v) else None,
        )

    @classmethod
    def from_mapping(cls, row: Mapping[str, object]) -> "RuntimeMeasurementSample | None":
        if "estimated_frd_range" in row:
            estimated_frd_range = row["estimated_frd_range"]
            if isinstance(estimated_frd_range, Sequence) and not isinstance(estimated_frd_range, (str, bytes)):
                return cls.from_public_frd_range(
                    estimated_frd_range,
                    row.get("measured_disparity_px", row.get("raw_disparity_px")),
                    row.get("left_u_px", row.get("left_cx_px")),
                    row.get("left_v_px", row.get("left_cy_px")),
                )
        raw_right = finite_float(row.get("raw_right_m"))
        raw_down = finite_float(row.get("raw_down_m"))
        raw_forward = finite_float(row.get("raw_forward_m"))
        disparity = finite_float(row.get("measured_disparity_px", row.get("raw_disparity_px")))
        if not all(math.isfinite(value) for value in (raw_right, raw_down, raw_forward, disparity)):
            return None
        raw_range = finite_float(row.get("raw_range_m"))
        if not math.isfinite(raw_range):
            raw_range = math.sqrt(raw_right * raw_right + raw_down * raw_down + raw_forward * raw_forward)
        left_u = finite_float(row.get("left_u_px", row.get("left_cx_px")))
        left_v = finite_float(row.get("left_v_px", row.get("left_cy_px")))
        return cls(
            raw_right_m=raw_right,
            raw_down_m=raw_down,
            raw_forward_m=raw_forward,
            measured_disparity_px=disparity,
            raw_range_m=raw_range,
            left_u_px=left_u if math.isfinite(left_u) else None,
            left_v_px=left_v if math.isfinite(left_v) else None,
        )


class RuntimeFeatureWindow:
    def __init__(self, max_samples: int = 520) -> None:
        self.max_samples = max(1, int(max_samples))
        self._samples: Deque[RuntimeMeasurementSample] = deque(maxlen=self.max_samples)

    def add(self, sample: RuntimeMeasurementSample) -> None:
        self._samples.append(sample)

    def extend_from_rows(self, rows: Iterable[Mapping[str, object]]) -> None:
        for row in rows:
            sample = RuntimeMeasurementSample.from_mapping(row)
            if sample is not None:
                self.add(sample)

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def features(self) -> dict[str, float]:
        if not self._samples:
            raise ValueError("runtime feature window has no valid raw RDF/disparity samples")
        matrix = np.asarray(
            [
                [
                    sample.raw_right_m,
                    sample.raw_down_m,
                    sample.raw_forward_m,
                    sample.raw_range_m
                    if sample.raw_range_m is not None
                    else math.sqrt(
                        sample.raw_right_m * sample.raw_right_m
                        + sample.raw_down_m * sample.raw_down_m
                        + sample.raw_forward_m * sample.raw_forward_m
                    ),
                    sample.measured_disparity_px,
                    float("nan") if sample.left_u_px is None else sample.left_u_px,
                    float("nan") if sample.left_v_px is None else sample.left_v_px,
                ]
                for sample in self._samples
            ],
            dtype=float,
        )
        raw_right = matrix[:, 0]
        raw_down = matrix[:, 1]
        raw_forward = matrix[:, 2]
        raw_range = matrix[:, 3]
        disparity = matrix[:, 4]
        left_u = finite_array(matrix[:, 5])
        left_v = finite_array(matrix[:, 6])
        forward_abs = np.maximum(np.abs(raw_forward), 1e-9)
        product = disparity * raw_forward
        product_values = finite_array(product)
        product_mean = safe_mean(product_values)
        product_std = safe_std(product_values)
        product_ratio = product_mean / max(DEFAULT_NOMINAL_DISPARITY_FORWARD_PRODUCT_PX_M, 1e-9)
        product_ratio_values = product_values / max(DEFAULT_NOMINAL_DISPARITY_FORWARD_PRODUCT_PX_M, 1e-9)
        product_ratio_median = safe_percentile(product_ratio_values, 50.0)
        product_ratio_p10 = safe_percentile(product_ratio_values, 10.0)
        product_ratio_iqr = percentile_iqr(product_ratio_values)
        product_abs_fractional_errors = finite_array(np.abs(product_ratio_values - 1.0))
        return {
            "count_log1p": math.log1p(float(matrix.shape[0])),
            "raw_forward_mean_m": safe_mean(finite_array(raw_forward)),
            "raw_forward_std_m": safe_std(finite_array(raw_forward)),
            "raw_forward_iqr_m": percentile_iqr(finite_array(raw_forward)),
            "raw_range_mean_m": safe_mean(finite_array(raw_range)),
            "raw_range_std_m": safe_std(finite_array(raw_range)),
            "measured_disparity_mean_px": safe_mean(finite_array(disparity)),
            "measured_disparity_std_px": safe_std(finite_array(disparity)),
            "measured_disparity_iqr_px": percentile_iqr(finite_array(disparity)),
            "disparity_forward_product_mean_px_m": product_mean,
            "disparity_forward_product_cv": product_std / max(abs(product_mean), 1e-9),
            "disparity_forward_product_ratio_to_nominal": product_ratio,
            "disparity_forward_product_abs_fractional_error": abs(product_ratio - 1.0),
            "disparity_forward_product_ratio_median_to_nominal": product_ratio_median,
            "disparity_forward_product_ratio_p10_to_nominal": product_ratio_p10,
            "disparity_forward_product_ratio_iqr": product_ratio_iqr,
            "disparity_forward_product_abs_fractional_error_median": safe_percentile(
                product_abs_fractional_errors,
                50.0,
            ),
            "right_over_forward_mean": safe_mean(finite_array(raw_right / forward_abs)),
            "right_over_forward_std": safe_std(finite_array(raw_right / forward_abs)),
            "down_over_forward_mean": safe_mean(finite_array(raw_down / forward_abs)),
            "down_over_forward_std": safe_std(finite_array(raw_down / forward_abs)),
            "left_u_mean_px": safe_mean(left_u),
            "left_u_std_px": safe_std(left_u),
            "left_v_mean_px": safe_mean(left_v),
            "left_v_std_px": safe_std(left_v),
        }


def parse_override_rule(rule: str) -> dict[str, object]:
    match = RULE_RE.search(str(rule or ""))
    if not match:
        return {"enabled": False, "feature_name": "", "direction": "", "threshold": float("nan")}
    return {
        "enabled": True,
        "feature_name": match.group("feature"),
        "direction": match.group("direction"),
        "threshold": float(match.group("threshold")),
    }


def override_applies(features: Mapping[str, float], rule: Mapping[str, object]) -> bool:
    if not bool(rule.get("enabled")):
        return False
    bare_name = str(rule.get("feature_name", "")).removeprefix("feature_")
    value = finite_float(features.get(bare_name))
    threshold = finite_float(rule.get("threshold"))
    if not math.isfinite(value) or not math.isfinite(threshold):
        return False
    direction = str(rule.get("direction", ""))
    if direction == "ge":
        return value >= threshold
    if direction == "gt":
        return value > threshold
    if direction == "le":
        return value <= threshold
    if direction == "lt":
        return value < threshold
    return False


class PackagedRuntimeDeploymentSelector:
    def __init__(self, manifest: Mapping[str, object]) -> None:
        self.manifest = dict(manifest)
        self.override_rule = parse_override_rule(str(self.manifest.get("runtime_rule", "")))
        self.override_profile_label = str(self.manifest.get("override_profile_label", "single_pooled"))
        self.baseline_selector = str(self.manifest.get("baseline_selector", "catalog_guard"))

    @classmethod
    def from_json_file(cls, path: str | Path) -> "PackagedRuntimeDeploymentSelector":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("runtime deployment selector manifest must be a JSON object")
        return cls(payload)

    def select(self, features: Mapping[str, float]) -> dict[str, object]:
        use_override = override_applies(features, self.override_rule)
        selected_label = self.override_profile_label if use_override else self.baseline_selector
        return {
            "selected_label": selected_label,
            "override_applied": use_override,
            "override_rule": dict(self.override_rule),
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "selection_contract": self.manifest.get("selection_contract", ""),
        }
