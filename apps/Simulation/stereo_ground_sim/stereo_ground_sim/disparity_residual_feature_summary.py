from __future__ import annotations

import math
from typing import Mapping, Sequence

import numpy as np


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def finite_stats(prefix: str, values: Sequence[float] | np.ndarray) -> dict[str, float]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {
            f"{prefix}_mean": 0.0,
            f"{prefix}_std": 0.0,
            f"{prefix}_p05": 0.0,
            f"{prefix}_p50": 0.0,
            f"{prefix}_p95": 0.0,
        }
    return {
        f"{prefix}_mean": float(np.mean(finite)),
        f"{prefix}_std": float(np.std(finite)),
        f"{prefix}_p05": float(np.percentile(finite, 5.0)),
        f"{prefix}_p50": float(np.percentile(finite, 50.0)),
        f"{prefix}_p95": float(np.percentile(finite, 95.0)),
    }


def feature_summary_from_arrays(
    *,
    left_u_px: Sequence[float] | np.ndarray,
    left_v_px: Sequence[float] | np.ndarray,
    measured_disparity_px: Sequence[float] | np.ndarray,
    raw_rdf_m: Sequence[Sequence[float]] | np.ndarray,
    fused_disparity_px: Sequence[float] | np.ndarray | None = None,
    observation_rdf_m: Sequence[Sequence[float]] | np.ndarray | None = None,
    feature_label: str = "runtime_observation_window",
    observation_rdf_source: str = "",
) -> dict[str, object]:
    measured = np.asarray(measured_disparity_px, dtype=float)
    raw_points = np.asarray(raw_rdf_m, dtype=float)
    if raw_points.ndim != 2 or raw_points.shape[1] != 3:
        raise ValueError("raw_rdf_m must be an Nx3 RDF(right,down,forward) array")
    fused = measured if fused_disparity_px is None else np.asarray(fused_disparity_px, dtype=float)
    observation_points = raw_points if observation_rdf_m is None else np.asarray(observation_rdf_m, dtype=float)
    if observation_points.ndim != 2 or observation_points.shape != raw_points.shape:
        raise ValueError("observation_rdf_m must match raw_rdf_m shape")

    raw_range = np.linalg.norm(raw_points, axis=1)
    observation_range = np.linalg.norm(observation_points, axis=1)
    observation_delta = observation_points - raw_points
    features: dict[str, object] = {
        "feature_label": feature_label,
        "row_count": int(raw_points.shape[0]),
        "observation_rdf_source": observation_rdf_source or ("raw_rdf_fallback" if observation_rdf_m is None else "observation_rdf_array"),
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }
    features.update(finite_stats("left_u_px", left_u_px))
    features.update(finite_stats("left_v_px", left_v_px))
    features.update(finite_stats("measured_disparity_px", measured))
    features.update(finite_stats("log_measured_disparity", np.log(np.maximum(measured, 1.0e-9))))
    features.update(finite_stats("fused_disparity_px", fused))
    features.update(finite_stats("raw_range_m", raw_range))
    features.update(finite_stats("observation_range_m", observation_range))
    features.update(finite_stats("observation_delta_norm_m", np.linalg.norm(observation_delta, axis=1)))
    for axis_index, axis_name in enumerate(("right", "down", "forward")):
        features.update(finite_stats(f"raw_{axis_name}_m", raw_points[:, axis_index]))
        features.update(finite_stats(f"observation_{axis_name}_m", observation_points[:, axis_index]))
        features.update(finite_stats(f"observation_delta_{axis_name}_m", observation_delta[:, axis_index]))
    return features


def feature_summary_from_runtime_rows(
    rows: Sequence[Mapping[str, object]],
    *,
    feature_label: str = "runtime_observation_window",
    observation_rdf_source: str = "",
) -> dict[str, object]:
    def values(name: str) -> np.ndarray:
        parsed: list[float] = []
        for row in rows:
            try:
                value = float(str(row.get(name, "")).strip())
            except (TypeError, ValueError):
                value = float("nan")
            parsed.append(value if math.isfinite(value) else float("nan"))
        return np.asarray(parsed, dtype=float)

    raw = np.column_stack([values("raw_right_m"), values("raw_down_m"), values("raw_forward_m")])
    has_observation = all(
        any(name in row for row in rows)
        for name in ("observation_right_m", "observation_down_m", "observation_forward_m")
    )
    observation = (
        np.column_stack([values("observation_right_m"), values("observation_down_m"), values("observation_forward_m")])
        if has_observation
        else None
    )
    fused = values("fused_disparity_px") if any("fused_disparity_px" in row for row in rows) else None
    return feature_summary_from_arrays(
        left_u_px=values("left_u_px"),
        left_v_px=values("left_v_px"),
        measured_disparity_px=values("measured_disparity_px"),
        raw_rdf_m=raw,
        fused_disparity_px=fused,
        observation_rdf_m=observation,
        feature_label=feature_label,
        observation_rdf_source=observation_rdf_source,
    )
