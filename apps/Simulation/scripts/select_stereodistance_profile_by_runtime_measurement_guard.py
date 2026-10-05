#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_by_calibration_replay import (  # noqa: E402
    candidate_source_scenarios,
    finite_float,
    read_csv,
    replay_candidate,
    write_csv,
    write_json,
)
from select_stereodistance_profile_from_catalog import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    catalog_root,
    find_profile_row,
    load_catalog,
)


FORBIDDEN_SELECTION_FIELDS = {
    "truth_north_m",
    "truth_east_m",
    "truth_down_ned_m",
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "gt_right_m",
    "gt_down_m",
    "gt_forward_m",
    "true_disparity_px",
    "raw_disparity_residual_px",
    "raw_range_error_m",
    "corrected_range_error_m",
}

RAW_FEATURE_FIELDS = (
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
    "raw_range_m",
    "measured_disparity_px",
    "left_u_px",
    "left_v_px",
)

DEFAULT_CAMERA_WIDTH_PX = 1280.0
DEFAULT_CAMERA_HORIZONTAL_FOV_RAD = 0.154997710698
DEFAULT_CAMERA_BASELINE_M = 25.0
DEFAULT_NOMINAL_DISPARITY_FORWARD_PRODUCT_PX_M = (
    (DEFAULT_CAMERA_WIDTH_PX * 0.5)
    / math.tan(DEFAULT_CAMERA_HORIZONTAL_FOV_RAD * 0.5)
    * DEFAULT_CAMERA_BASELINE_M
)
DEFAULT_INTRINSIC_SCALE_GAP_SOURCE_SCENARIO = "intrinsic_scale_stress"
DEFAULT_INTRINSIC_SCALE_GAP_PRODUCT_RATIO_P10_THRESHOLD = 1.0037

FEATURE_NAMES = (
    "count_log1p",
    "raw_forward_mean_m",
    "raw_forward_std_m",
    "raw_forward_iqr_m",
    "raw_range_mean_m",
    "raw_range_std_m",
    "measured_disparity_mean_px",
    "measured_disparity_std_px",
    "measured_disparity_iqr_px",
    "disparity_forward_product_mean_px_m",
    "disparity_forward_product_cv",
    "disparity_forward_product_ratio_to_nominal",
    "disparity_forward_product_abs_fractional_error",
    "disparity_forward_product_ratio_median_to_nominal",
    "disparity_forward_product_ratio_p10_to_nominal",
    "disparity_forward_product_ratio_iqr",
    "disparity_forward_product_abs_fractional_error_median",
    "right_over_forward_mean",
    "right_over_forward_std",
    "down_over_forward_mean",
    "down_over_forward_std",
    "left_u_mean_px",
    "left_u_std_px",
    "left_v_mean_px",
    "left_v_std_px",
)

FEATURE_SUBSETS = {
    "range_geometry": (
        "count_log1p",
        "raw_forward_mean_m",
        "raw_forward_std_m",
        "raw_forward_iqr_m",
        "raw_range_mean_m",
        "raw_range_std_m",
    ),
    "disparity_depth": (
        "measured_disparity_mean_px",
        "measured_disparity_std_px",
        "measured_disparity_iqr_px",
        "disparity_forward_product_mean_px_m",
        "disparity_forward_product_cv",
        "disparity_forward_product_ratio_to_nominal",
        "disparity_forward_product_abs_fractional_error",
        "disparity_forward_product_ratio_median_to_nominal",
        "disparity_forward_product_ratio_p10_to_nominal",
        "disparity_forward_product_ratio_iqr",
        "disparity_forward_product_abs_fractional_error_median",
    ),
    "bearing_image": (
        "right_over_forward_mean",
        "right_over_forward_std",
        "down_over_forward_mean",
        "down_over_forward_std",
        "left_u_mean_px",
        "left_u_std_px",
        "left_v_mean_px",
        "left_v_std_px",
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Select/audit a StereoDistance catalog profile with a runtime-measurement guard. "
            "Profile selection uses only raw stereo-center RDF, measured disparity, and image-center "
            "measurements. Calibration truth is used only after selection for offline audit metrics."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--input-csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--candidate-source-mode", choices=("minimal_set", "all_catalog"), default="minimal_set")
    parser.add_argument(
        "--selection-policy",
        choices=(
            "nearest_centroid",
            "robust_family_fallback",
            "transfer_risk_fallback",
            "feature_vote_transfer_risk",
            "intrinsic_scale_gap_transfer_risk",
            "knn_transfer_risk",
            "centroid_knn_union_transfer_risk",
            "centroid_knn_veto_transfer_risk",
        ),
        default="nearest_centroid",
        help=(
            "nearest_centroid chooses the closest raw-measurement scenario. robust_family_fallback first "
            "classifies with the same raw features, then maps nominal/yaw/bbox-like low-range geometry to "
            "the validated bbox_outlier_stress profile when available. transfer_risk_fallback discovers "
            "low-risk confusion clusters from the offline transfer matrix. intrinsic_scale_gap_transfer_risk "
            "first checks a runtime disparity-forward product-ratio guard, then falls back to transfer risk. "
            "feature_vote_transfer_risk "
            "adds a raw feature-subset vote before the same transfer-risk selection. knn_transfer_risk "
            "uses nearest raw measurement runs instead of scenario centroids before transfer-risk selection. "
            "centroid_knn_union_transfer_risk scores the union of centroid low-risk and kNN-neighbor candidates."
        ),
    )
    parser.add_argument("--intrinsic-scale-gap-source-scenario", default=DEFAULT_INTRINSIC_SCALE_GAP_SOURCE_SCENARIO)
    parser.add_argument(
        "--intrinsic-scale-gap-product-ratio-p10-threshold",
        type=float,
        default=DEFAULT_INTRINSIC_SCALE_GAP_PRODUCT_RATIO_P10_THRESHOLD,
        help=(
            "Audit-derived runtime guard threshold for disparity*raw_forward/(nominal f*baseline). "
            "If p10 exceeds this value and the source scenario exists, select the intrinsic-scale profile."
        ),
    )
    parser.add_argument("--knn-neighbor-count", type=int, default=5)
    parser.add_argument("--transfer-matrix-csv", type=Path)
    parser.add_argument("--transfer-low-risk-rmse-limit-m", type=float, default=1.3)
    parser.add_argument("--transfer-low-risk-target-limit-m", type=float, default=2.5)
    parser.add_argument("--transfer-risk-target-weight", type=float, default=0.10)
    parser.add_argument("--audit-all-sweep-seeds", action="store_true")
    parser.add_argument("--include-target-in-centroids", action="store_true")
    parser.add_argument(
        "--exclude-target-scenario-from-library",
        action="store_true",
        help=(
            "When auditing a sweep, remove the target scenario from both raw-feature library and "
            "candidate source scenarios. This is stricter than excluding only the target CSV."
        ),
    )
    parser.add_argument("--allow-unvalidated", action="store_true")
    return parser.parse_args()


def finite_array(values: Sequence[float]) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    return arr[np.isfinite(arr)]


def row_float(row: Mapping[str, str], name: str) -> float:
    return finite_float(row.get(name), float("nan"))


def percentile_iqr(values: np.ndarray) -> float:
    if values.size == 0:
        return float("nan")
    q75, q25 = np.percentile(values, [75.0, 25.0])
    return float(q75 - q25)


def safe_percentile(values: np.ndarray, percentile: float) -> float:
    return float(np.percentile(values, percentile)) if values.size else float("nan")


def safe_mean(values: np.ndarray) -> float:
    return float(np.mean(values)) if values.size else float("nan")


def safe_std(values: np.ndarray) -> float:
    return float(np.std(values)) if values.size else float("nan")


def load_raw_feature_matrix(rows: Sequence[Mapping[str, str]]) -> np.ndarray:
    values: list[list[float]] = []
    for row in rows:
        raw_right = row_float(row, "raw_right_m")
        raw_down = row_float(row, "raw_down_m")
        raw_forward = row_float(row, "raw_forward_m")
        raw_range = row_float(row, "raw_range_m")
        if not math.isfinite(raw_range) and all(math.isfinite(value) for value in (raw_right, raw_down, raw_forward)):
            raw_range = math.sqrt(raw_right * raw_right + raw_down * raw_down + raw_forward * raw_forward)
        disparity = row_float(row, "measured_disparity_px")
        left_u = row_float(row, "left_u_px")
        left_v = row_float(row, "left_v_px")
        if all(math.isfinite(value) for value in (raw_right, raw_down, raw_forward, raw_range, disparity)):
            values.append([raw_right, raw_down, raw_forward, raw_range, disparity, left_u, left_v])
    return np.asarray(values, dtype=float)


def runtime_measurement_features(rows: Sequence[Mapping[str, str]]) -> dict[str, float]:
    matrix = load_raw_feature_matrix(rows)
    if matrix.size == 0:
        raise ValueError("no raw RDF/disparity rows available for runtime measurement guard")
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


def feature_vector(features: Mapping[str, float]) -> np.ndarray:
    return np.asarray([finite_float(features.get(name), 0.0) for name in FEATURE_NAMES], dtype=float)


def feature_subset_indices(names: Sequence[str]) -> np.ndarray:
    index_by_name = {name: index for index, name in enumerate(FEATURE_NAMES)}
    return np.asarray([index_by_name[name] for name in names], dtype=int)


def sweep_calibration_csvs(sweep_dir: Path) -> list[tuple[str, str, Path]]:
    rows: list[tuple[str, str, Path]] = []
    for path in sorted(sweep_dir.glob("*/seed_*/calibration_samples.csv")):
        scenario = path.parent.parent.name
        seed = path.parent.name
        rows.append((scenario, seed, path))
    return rows


def source_scenarios_for_mode(
    *,
    catalog: Mapping[str, object],
    catalog_json: Path,
    minimal_set_json: Path | None,
    candidate_source_mode: str,
) -> list[str]:
    if candidate_source_mode == "minimal_set":
        return candidate_source_scenarios(catalog, minimal_set_json)
    if candidate_source_mode == "all_catalog":
        return candidate_source_scenarios(catalog, None)
    raise ValueError("candidate_source_mode must be 'minimal_set' or 'all_catalog'")


def default_transfer_matrix_csv(sweep_dir: Path) -> Path:
    return sweep_dir / "scenario_profile_catalog_transfer_matrix.csv"


def load_transfer_matrix(path: Path | None) -> dict[tuple[str, str], dict[str, str]]:
    if path is None or not path.is_file():
        return {}
    rows = read_csv(path)
    matrix: dict[tuple[str, str], dict[str, str]] = {}
    for row in rows:
        source = str(row.get("source_scenario", "")).strip()
        target = str(row.get("target_scenario", "")).strip()
        if source and target:
            matrix[(source, target)] = dict(row)
    return matrix


def transfer_row_is_low_risk(
    row: Mapping[str, str] | None,
    *,
    rmse_limit_m: float,
    target_limit_m: float,
) -> bool:
    if row is None:
        return False
    if str(row.get("pass_all", "")).strip() != "True":
        return False
    rmse = finite_float(row.get("mean_validation_range_rmse_corrected_m"), float("inf"))
    target = finite_float(row.get("max_target_abs_mean_error_corrected_m"), float("inf"))
    return rmse <= rmse_limit_m and target <= target_limit_m


def transfer_pair_is_low_risk(
    matrix: Mapping[tuple[str, str], Mapping[str, str]],
    left: str,
    right: str,
    *,
    rmse_limit_m: float,
    target_limit_m: float,
) -> bool:
    if left == right:
        return True
    return transfer_row_is_low_risk(
        matrix.get((left, right)),
        rmse_limit_m=rmse_limit_m,
        target_limit_m=target_limit_m,
    ) and transfer_row_is_low_risk(
        matrix.get((right, left)),
        rmse_limit_m=rmse_limit_m,
        target_limit_m=target_limit_m,
    )


def transfer_confusion_cluster(
    *,
    preliminary: str,
    candidate_scenarios: Sequence[str],
    matrix: Mapping[tuple[str, str], Mapping[str, str]],
    rmse_limit_m: float,
    target_limit_m: float,
) -> list[str]:
    cluster: list[str] = []
    for scenario in candidate_scenarios:
        if transfer_pair_is_low_risk(
            matrix,
            preliminary,
            scenario,
            rmse_limit_m=rmse_limit_m,
            target_limit_m=target_limit_m,
        ):
            cluster.append(scenario)
    if preliminary not in cluster:
        cluster.insert(0, preliminary)
    return list(dict.fromkeys(cluster))


def transfer_source_risk_score(
    *,
    source: str,
    targets: Sequence[str],
    matrix: Mapping[tuple[str, str], Mapping[str, str]],
    target_weight: float = 0.10,
) -> tuple[float, dict[str, object]]:
    rows: list[Mapping[str, str]] = []
    fail_count = 0
    rmse_values: list[float] = []
    target_values: list[float] = []
    improvement_values: list[float] = []
    for target in targets:
        row = matrix.get((source, target))
        if row is None:
            fail_count += 1
            continue
        rows.append(row)
        if str(row.get("pass_all", "")).strip() != "True":
            fail_count += 1
        rmse_values.append(finite_float(row.get("mean_validation_range_rmse_corrected_m"), float("inf")))
        target_values.append(finite_float(row.get("max_target_abs_mean_error_corrected_m"), float("inf")))
        improvement_values.append(finite_float(row.get("min_validation_range_rmse_improvement_m"), float("-inf")))
    mean_rmse = float(np.mean(rmse_values)) if rmse_values else float("inf")
    max_target = float(np.max(target_values)) if target_values else float("inf")
    min_improvement = float(np.min(improvement_values)) if improvement_values else float("-inf")
    score = mean_rmse + target_weight * max_target + 2.0 * max(0.0, -min_improvement) + 100.0 * fail_count
    return score, {
        "source_scenario": source,
        "target_cluster": ",".join(targets),
        "transfer_fail_count": fail_count,
        "transfer_mean_rmse_m": mean_rmse,
        "transfer_max_target_abs_mean_error_m": max_target,
        "transfer_min_range_improvement_m": min_improvement,
        "transfer_risk_target_weight": target_weight,
        "transfer_risk_score": score,
    }


def build_feature_library(
    *,
    sweep_dir: Path,
    source_scenarios: Sequence[str],
    exclude_csv: Path | None = None,
) -> list[dict[str, object]]:
    source_set = set(source_scenarios)
    exclude_resolved = exclude_csv.resolve() if exclude_csv else None
    library: list[dict[str, object]] = []
    for scenario, seed, path in sweep_calibration_csvs(sweep_dir):
        if scenario not in source_set:
            continue
        if exclude_resolved is not None and path.resolve() == exclude_resolved:
            continue
        rows = read_csv(path)
        features = runtime_measurement_features(rows)
        library.append(
            {
                "scenario": scenario,
                "seed": seed,
                "csv_path": str(path),
                "features": features,
                "feature_vector": feature_vector(features),
            }
        )
    if not library:
        raise ValueError("no source scenario calibration CSVs found for runtime measurement guard")
    return library


def fit_centroids(library: Sequence[Mapping[str, object]]) -> dict[str, object]:
    vectors = np.vstack([np.asarray(item["feature_vector"], dtype=float) for item in library])
    center = np.nanmean(vectors, axis=0)
    scale = np.nanstd(vectors, axis=0)
    scale = np.where(np.isfinite(scale) & (scale > 1e-9), scale, 1.0)
    normalized = (np.nan_to_num(vectors, nan=0.0) - center) / scale
    centroids: dict[str, np.ndarray] = {}
    counts: dict[str, int] = {}
    for item, vector in zip(library, normalized):
        scenario = str(item["scenario"])
        centroids.setdefault(scenario, np.zeros(vector.shape, dtype=float))
        centroids[scenario] += vector
        counts[scenario] = counts.get(scenario, 0) + 1
    for scenario, total in list(centroids.items()):
        centroids[scenario] = total / float(counts[scenario])
    return {"center": center, "scale": scale, "centroids": centroids, "counts": counts}


def ranked_centroid_distances(
    vector: np.ndarray,
    centroids: Mapping[str, np.ndarray],
    counts: Mapping[str, int],
    subset_indices: np.ndarray | None = None,
) -> list[dict[str, object]]:
    distances = []
    for scenario, centroid in sorted(centroids.items()):
        if subset_indices is None:
            left = vector
            right = centroid
        else:
            left = vector[subset_indices]
            right = centroid[subset_indices]
        distance = float(np.linalg.norm(left - right))
        distances.append(
            {
                "scenario": scenario,
                "distance": distance,
                "training_sample_count": int(counts[scenario]),
            }
        )
    return sorted(distances, key=lambda row: (row["distance"], row["scenario"]))


def ranked_library_neighbors(
    vector: np.ndarray,
    model: Mapping[str, object],
    library: Sequence[Mapping[str, object]],
) -> list[dict[str, object]]:
    center = np.asarray(model["center"], dtype=float)
    scale = np.asarray(model["scale"], dtype=float)
    rows: list[dict[str, object]] = []
    for item in library:
        item_vector = (np.nan_to_num(np.asarray(item["feature_vector"], dtype=float), nan=0.0) - center) / scale
        rows.append(
            {
                "scenario": str(item.get("scenario", "")),
                "seed": str(item.get("seed", "")),
                "csv_path": str(item.get("csv_path", "")),
                "distance": float(np.linalg.norm(vector - item_vector)),
            }
        )
    return sorted(rows, key=lambda row: (row["distance"], row["scenario"], row["seed"], row["csv_path"]))


def knn_preliminary(
    *,
    vector: np.ndarray,
    model: Mapping[str, object],
    library: Sequence[Mapping[str, object]],
    neighbor_count: int,
) -> tuple[str, list[dict[str, object]], list[str]]:
    neighbors = ranked_library_neighbors(vector, model, library)
    if not neighbors:
        raise ValueError("knn selector has no library neighbors")
    k = max(1, min(int(neighbor_count), len(neighbors)))
    top_neighbors = neighbors[:k]
    aggregate: dict[str, dict[str, float]] = {}
    for rank, row in enumerate(top_neighbors, start=1):
        scenario = str(row["scenario"])
        stats = aggregate.setdefault(scenario, {"vote_count": 0.0, "rank_sum": 0.0, "distance_sum": 0.0})
        stats["vote_count"] += 1.0
        stats["rank_sum"] += float(rank)
        stats["distance_sum"] += finite_float(row.get("distance"), float("inf"))
    preliminary = sorted(
        aggregate.items(),
        key=lambda item: (-item[1]["vote_count"], item[1]["rank_sum"], item[1]["distance_sum"], item[0]),
    )[0][0]
    candidate_scenarios = list(dict.fromkeys(str(row["scenario"]) for row in top_neighbors))
    return preliminary, top_neighbors, candidate_scenarios


def feature_vote_preliminary(
    *,
    vector: np.ndarray,
    centroids: Mapping[str, np.ndarray],
    counts: Mapping[str, int],
) -> tuple[str, list[dict[str, object]]]:
    vote_rows: list[dict[str, object]] = []
    aggregate: dict[str, dict[str, float]] = {}
    for subset_name, feature_names in FEATURE_SUBSETS.items():
        subset_indices = feature_subset_indices(feature_names)
        ranked = ranked_centroid_distances(vector, centroids, counts, subset_indices)
        for rank, row in enumerate(ranked, start=1):
            scenario = str(row["scenario"])
            stats = aggregate.setdefault(scenario, {"vote_count": 0.0, "rank_sum": 0.0, "distance_sum": 0.0})
            if rank == 1:
                stats["vote_count"] += 1.0
            stats["rank_sum"] += float(rank)
            stats["distance_sum"] += finite_float(row.get("distance"), float("inf"))
        if ranked:
            vote_rows.append(
                {
                    "feature_subset": subset_name,
                    "winner_scenario": ranked[0]["scenario"],
                    "winner_distance": ranked[0]["distance"],
                    "ranked_scenarios_json": json.dumps(ranked, ensure_ascii=False, allow_nan=False),
                    "feature_names": ",".join(feature_names),
                }
            )
    if not aggregate:
        raise ValueError("feature-vote selector has no centroid votes")
    best = sorted(
        aggregate.items(),
        key=lambda item: (-item[1]["vote_count"], item[1]["rank_sum"], item[1]["distance_sum"], item[0]),
    )[0][0]
    return best, vote_rows


def select_source_by_runtime_measurement(
    *,
    rows: Sequence[Mapping[str, str]],
    library: Sequence[Mapping[str, object]],
    available_source_scenarios: Sequence[str] = (),
    selection_policy: str = "nearest_centroid",
    transfer_matrix: Mapping[tuple[str, str], Mapping[str, str]] | None = None,
    transfer_low_risk_rmse_limit_m: float = 1.3,
    transfer_low_risk_target_limit_m: float = 2.5,
    transfer_risk_target_weight: float = 0.10,
    knn_neighbor_count: int = 5,
    intrinsic_scale_gap_source_scenario: str = DEFAULT_INTRINSIC_SCALE_GAP_SOURCE_SCENARIO,
    intrinsic_scale_gap_product_ratio_p10_threshold: float = DEFAULT_INTRINSIC_SCALE_GAP_PRODUCT_RATIO_P10_THRESHOLD,
) -> dict[str, object]:
    model = fit_centroids(library)
    features = runtime_measurement_features(rows)
    vector = (np.nan_to_num(feature_vector(features), nan=0.0) - model["center"]) / model["scale"]
    distances = ranked_centroid_distances(vector, model["centroids"], model["counts"])
    if not distances:
        raise ValueError("runtime measurement guard has no centroids")
    preliminary = str(distances[0]["scenario"])
    selected = preliminary
    fallback_reason = ""
    transfer_cluster: list[str] = []
    transfer_score_rows: list[dict[str, object]] = []
    feature_vote_rows: list[dict[str, object]] = []
    knn_neighbor_rows: list[dict[str, object]] = []
    available = set(available_source_scenarios)
    if selection_policy == "robust_family_fallback":
        if preliminary in {"nominal", "yaw_pitch_stress", "bbox_outlier_stress"}:
            if "bbox_outlier_stress" in available:
                selected = "bbox_outlier_stress"
                fallback_reason = "low_range_nominal_yaw_bbox_family_uses_bbox_outlier_robust_profile"
            elif "disparity_noise_stress" in available:
                selected = "disparity_noise_stress"
                fallback_reason = "bbox_profile_unavailable_low_range_family_uses_disparity_noise_profile"
    elif selection_policy == "transfer_risk_fallback":
        matrix = transfer_matrix or {}
        if not matrix:
            raise ValueError("transfer_risk_fallback requires --transfer-matrix-csv or sweep transfer matrix")
        distance_scenarios = [str(row["scenario"]) for row in distances]
        transfer_cluster = transfer_confusion_cluster(
            preliminary=preliminary,
            candidate_scenarios=distance_scenarios,
            matrix=matrix,
            rmse_limit_m=transfer_low_risk_rmse_limit_m,
            target_limit_m=transfer_low_risk_target_limit_m,
        )
        source_candidates = [scenario for scenario in available_source_scenarios if scenario in transfer_cluster]
        if not source_candidates:
            source_candidates = [preliminary]
        scored: list[tuple[float, str, dict[str, object]]] = []
        for source in source_candidates:
            score, score_row = transfer_source_risk_score(
                source=source,
                targets=transfer_cluster,
                matrix=matrix,
                target_weight=transfer_risk_target_weight,
            )
            scored.append((score, source, score_row))
        scored = sorted(scored, key=lambda item: (item[0], item[1]))
        selected = scored[0][1]
        transfer_score_rows = [item[2] for item in scored]
        if selected != preliminary:
            fallback_reason = "transfer_matrix_low_risk_cluster_prefers_lower_risk_source"
    elif selection_policy == "intrinsic_scale_gap_transfer_risk":
        matrix = transfer_matrix or {}
        if not matrix:
            raise ValueError("intrinsic_scale_gap_transfer_risk requires --transfer-matrix-csv or sweep transfer matrix")
        scale_gap_source = str(intrinsic_scale_gap_source_scenario)
        product_ratio_p10 = finite_float(
            features.get("disparity_forward_product_ratio_p10_to_nominal"),
            float("nan"),
        )
        scale_gap_detected = (
            math.isfinite(product_ratio_p10)
            and product_ratio_p10 >= float(intrinsic_scale_gap_product_ratio_p10_threshold)
        )
        if scale_gap_detected and scale_gap_source in available:
            selected = scale_gap_source
            transfer_cluster = [scale_gap_source]
            fallback_reason = "intrinsic_scale_product_ratio_guard_selects_scale_profile"
            transfer_score_rows = [
                {
                    "source_scenario": scale_gap_source,
                    "target_cluster": scale_gap_source,
                    "intrinsic_scale_gap_product_ratio_p10": product_ratio_p10,
                    "intrinsic_scale_gap_product_ratio_p10_threshold": intrinsic_scale_gap_product_ratio_p10_threshold,
                    "intrinsic_scale_gap_source_scenario": scale_gap_source,
                    "transfer_risk_score": 0.0,
                }
            ]
        else:
            distance_scenarios = [str(row["scenario"]) for row in distances]
            transfer_cluster = transfer_confusion_cluster(
                preliminary=preliminary,
                candidate_scenarios=distance_scenarios,
                matrix=matrix,
                rmse_limit_m=transfer_low_risk_rmse_limit_m,
                target_limit_m=transfer_low_risk_target_limit_m,
            )
            source_candidates = [scenario for scenario in available_source_scenarios if scenario in transfer_cluster]
            if not source_candidates:
                source_candidates = [preliminary]
            scored = []
            for source in source_candidates:
                score, score_row = transfer_source_risk_score(
                    source=source,
                    targets=transfer_cluster,
                    matrix=matrix,
                    target_weight=transfer_risk_target_weight,
                )
                score_row["intrinsic_scale_gap_detected"] = scale_gap_detected
                score_row["intrinsic_scale_gap_product_ratio_p10"] = product_ratio_p10
                score_row[
                    "intrinsic_scale_gap_product_ratio_p10_threshold"
                ] = intrinsic_scale_gap_product_ratio_p10_threshold
                score_row["intrinsic_scale_gap_source_scenario"] = scale_gap_source
                scored.append((score, source, score_row))
            scored = sorted(scored, key=lambda item: (item[0], item[1]))
            selected = scored[0][1]
            transfer_score_rows = [item[2] for item in scored]
            if scale_gap_detected and scale_gap_source not in available:
                fallback_reason = "intrinsic_scale_gap_detected_but_source_unavailable"
            elif selected != preliminary:
                fallback_reason = "transfer_matrix_low_risk_cluster_prefers_lower_risk_source"
    elif selection_policy == "feature_vote_transfer_risk":
        matrix = transfer_matrix or {}
        if not matrix:
            raise ValueError("feature_vote_transfer_risk requires --transfer-matrix-csv or sweep transfer matrix")
        preliminary, feature_vote_rows = feature_vote_preliminary(
            vector=vector,
            centroids=model["centroids"],
            counts=model["counts"],
        )
        distance_scenarios = [str(row["scenario"]) for row in distances]
        voted_scenarios = [str(row["winner_scenario"]) for row in feature_vote_rows]
        cluster_seed_order = list(dict.fromkeys([preliminary, *voted_scenarios]))
        cluster_members: list[str] = []
        for seed_scenario in cluster_seed_order:
            for scenario in transfer_confusion_cluster(
                preliminary=seed_scenario,
                candidate_scenarios=distance_scenarios,
                matrix=matrix,
                rmse_limit_m=transfer_low_risk_rmse_limit_m,
                target_limit_m=transfer_low_risk_target_limit_m,
            ):
                if scenario not in cluster_members:
                    cluster_members.append(scenario)
        transfer_cluster = cluster_members or [preliminary]
        source_candidates = [scenario for scenario in available_source_scenarios if scenario in transfer_cluster]
        if not source_candidates:
            source_candidates = [preliminary]
        scored = []
        for source in source_candidates:
            score, score_row = transfer_source_risk_score(
                source=source,
                targets=transfer_cluster,
                matrix=matrix,
                target_weight=transfer_risk_target_weight,
            )
            score_row["feature_vote_preliminary_source_scenario"] = preliminary
            score_row["feature_vote_winners"] = ",".join(voted_scenarios)
            scored.append((score, source, score_row))
        scored = sorted(scored, key=lambda item: (item[0], item[1]))
        selected = scored[0][1]
        transfer_score_rows = [item[2] for item in scored]
        if selected != preliminary:
            fallback_reason = "feature_vote_transfer_cluster_prefers_lower_risk_source"
    elif selection_policy == "knn_transfer_risk":
        matrix = transfer_matrix or {}
        if not matrix:
            raise ValueError("knn_transfer_risk requires --transfer-matrix-csv or sweep transfer matrix")
        preliminary, knn_neighbor_rows, knn_scenarios = knn_preliminary(
            vector=vector,
            model=model,
            library=library,
            neighbor_count=knn_neighbor_count,
        )
        transfer_cluster = transfer_confusion_cluster(
            preliminary=preliminary,
            candidate_scenarios=knn_scenarios,
            matrix=matrix,
            rmse_limit_m=transfer_low_risk_rmse_limit_m,
            target_limit_m=transfer_low_risk_target_limit_m,
        )
        source_candidates = [scenario for scenario in available_source_scenarios if scenario in transfer_cluster]
        if not source_candidates:
            source_candidates = [preliminary]
        scored = []
        for source in source_candidates:
            score, score_row = transfer_source_risk_score(
                source=source,
                targets=transfer_cluster,
                matrix=matrix,
                target_weight=transfer_risk_target_weight,
            )
            score_row["knn_preliminary_source_scenario"] = preliminary
            score_row["knn_neighbor_count"] = knn_neighbor_count
            score_row["knn_candidate_scenarios"] = ",".join(knn_scenarios)
            scored.append((score, source, score_row))
        scored = sorted(scored, key=lambda item: (item[0], item[1]))
        selected = scored[0][1]
        transfer_score_rows = [item[2] for item in scored]
        if selected != preliminary:
            fallback_reason = "knn_transfer_cluster_prefers_lower_risk_source"
    elif selection_policy == "centroid_knn_union_transfer_risk":
        matrix = transfer_matrix or {}
        if not matrix:
            raise ValueError("centroid_knn_union_transfer_risk requires --transfer-matrix-csv or sweep transfer matrix")
        centroid_preliminary = preliminary
        distance_scenarios = [str(row["scenario"]) for row in distances]
        centroid_cluster = transfer_confusion_cluster(
            preliminary=centroid_preliminary,
            candidate_scenarios=distance_scenarios,
            matrix=matrix,
            rmse_limit_m=transfer_low_risk_rmse_limit_m,
            target_limit_m=transfer_low_risk_target_limit_m,
        )
        knn_prelim, knn_neighbor_rows, knn_scenarios = knn_preliminary(
            vector=vector,
            model=model,
            library=library,
            neighbor_count=knn_neighbor_count,
        )
        transfer_cluster = list(dict.fromkeys([*centroid_cluster, *knn_scenarios]))
        source_candidates = [scenario for scenario in available_source_scenarios if scenario in transfer_cluster]
        if not source_candidates:
            source_candidates = [centroid_preliminary]
        scored = []
        for source in source_candidates:
            score, score_row = transfer_source_risk_score(
                source=source,
                targets=transfer_cluster,
                matrix=matrix,
                target_weight=transfer_risk_target_weight,
            )
            score_row["centroid_preliminary_source_scenario"] = centroid_preliminary
            score_row["centroid_transfer_cluster"] = ",".join(centroid_cluster)
            score_row["knn_preliminary_source_scenario"] = knn_prelim
            score_row["knn_neighbor_count"] = knn_neighbor_count
            score_row["knn_candidate_scenarios"] = ",".join(knn_scenarios)
            scored.append((score, source, score_row))
        scored = sorted(scored, key=lambda item: (item[0], item[1]))
        selected = scored[0][1]
        transfer_score_rows = [item[2] for item in scored]
        if selected != centroid_preliminary:
            fallback_reason = "centroid_knn_union_prefers_lower_transfer_risk_source"
    elif selection_policy == "centroid_knn_veto_transfer_risk":
        matrix = transfer_matrix or {}
        if not matrix:
            raise ValueError("centroid_knn_veto_transfer_risk requires --transfer-matrix-csv or sweep transfer matrix")
        centroid_preliminary = preliminary
        distance_scenarios = [str(row["scenario"]) for row in distances]
        centroid_cluster = transfer_confusion_cluster(
            preliminary=centroid_preliminary,
            candidate_scenarios=distance_scenarios,
            matrix=matrix,
            rmse_limit_m=transfer_low_risk_rmse_limit_m,
            target_limit_m=transfer_low_risk_target_limit_m,
        )
        knn_prelim, knn_neighbor_rows, knn_scenarios = knn_preliminary(
            vector=vector,
            model=model,
            library=library,
            neighbor_count=knn_neighbor_count,
        )
        safe_knn_scenarios = [
            scenario
            for scenario in knn_scenarios
            if transfer_pair_is_low_risk(
                matrix,
                centroid_preliminary,
                scenario,
                rmse_limit_m=transfer_low_risk_rmse_limit_m,
                target_limit_m=transfer_low_risk_target_limit_m,
            )
        ]
        transfer_cluster = list(dict.fromkeys([*centroid_cluster, *safe_knn_scenarios]))
        source_candidates = [scenario for scenario in available_source_scenarios if scenario in transfer_cluster]
        if not source_candidates:
            source_candidates = [centroid_preliminary]
        scored = []
        for source in source_candidates:
            score, score_row = transfer_source_risk_score(
                source=source,
                targets=transfer_cluster,
                matrix=matrix,
                target_weight=transfer_risk_target_weight,
            )
            score_row["centroid_preliminary_source_scenario"] = centroid_preliminary
            score_row["centroid_transfer_cluster"] = ",".join(centroid_cluster)
            score_row["knn_preliminary_source_scenario"] = knn_prelim
            score_row["knn_neighbor_count"] = knn_neighbor_count
            score_row["knn_candidate_scenarios"] = ",".join(knn_scenarios)
            score_row["safe_knn_candidate_scenarios"] = ",".join(safe_knn_scenarios)
            scored.append((score, source, score_row))
        scored = sorted(scored, key=lambda item: (item[0], item[1]))
        selected = scored[0][1]
        transfer_score_rows = [item[2] for item in scored]
        if selected != centroid_preliminary:
            fallback_reason = "centroid_knn_veto_prefers_lower_transfer_risk_source"
    elif selection_policy != "nearest_centroid":
        raise ValueError(
            "selection_policy must be 'nearest_centroid', 'robust_family_fallback', "
            "'transfer_risk_fallback', 'feature_vote_transfer_risk', 'knn_transfer_risk', "
            "'intrinsic_scale_gap_transfer_risk', 'centroid_knn_union_transfer_risk', or "
            "'centroid_knn_veto_transfer_risk'"
        )
    return {
        "preliminary_source_scenario": preliminary,
        "selected_source_scenario": selected,
        "selection_policy": selection_policy,
        "fallback_reason": fallback_reason,
        "selected_distance": distances[0]["distance"],
        "distance_rows": distances,
        "transfer_confusion_cluster": transfer_cluster,
        "transfer_score_rows": transfer_score_rows,
        "feature_vote_rows": feature_vote_rows,
        "knn_neighbor_rows": knn_neighbor_rows,
        "features": features,
    }


def catalog_profile_path(catalog_json: Path, catalog: Mapping[str, object], scenario: str) -> Path:
    profile_row = find_profile_row(catalog, scenario)
    relative_profile = str(profile_row.get("catalog_profile_path", ""))
    if not relative_profile:
        raise ValueError(f"catalog profile for scenario {scenario!r} has no path")
    return catalog_root(catalog_json) / relative_profile


def audit_one_input(
    *,
    sweep_dir: Path,
    catalog_json: Path,
    catalog: Mapping[str, object],
    source_scenarios: Sequence[str],
    input_csv: Path,
    output_dir: Path,
    split: str,
    target_range_m: float,
    target_window_m: float,
    target_error_limit_m: float,
    allow_unvalidated: bool,
    include_target_in_centroids: bool,
    selection_policy: str,
    transfer_matrix: Mapping[tuple[str, str], Mapping[str, str]],
    transfer_low_risk_rmse_limit_m: float,
    transfer_low_risk_target_limit_m: float,
    transfer_risk_target_weight: float,
    knn_neighbor_count: int,
    exclude_target_scenario_from_library: bool,
    intrinsic_scale_gap_source_scenario: str,
    intrinsic_scale_gap_product_ratio_p10_threshold: float,
    target_scenario: str = "",
    target_seed: str = "",
) -> dict[str, object]:
    rows = read_csv(input_csv)
    eligible_source_scenarios = list(source_scenarios)
    if exclude_target_scenario_from_library and target_scenario:
        eligible_source_scenarios = [
            scenario for scenario in eligible_source_scenarios if scenario != target_scenario
        ]
        if not eligible_source_scenarios:
            raise ValueError("target-scenario exclusion removed every candidate source scenario")
    library = build_feature_library(
        sweep_dir=sweep_dir,
        source_scenarios=eligible_source_scenarios,
        exclude_csv=None if include_target_in_centroids else input_csv,
    )
    selection = select_source_by_runtime_measurement(
        rows=rows,
        library=library,
        available_source_scenarios=eligible_source_scenarios,
        selection_policy=selection_policy,
        transfer_matrix=transfer_matrix,
        transfer_low_risk_rmse_limit_m=transfer_low_risk_rmse_limit_m,
        transfer_low_risk_target_limit_m=transfer_low_risk_target_limit_m,
        transfer_risk_target_weight=transfer_risk_target_weight,
        knn_neighbor_count=knn_neighbor_count,
        intrinsic_scale_gap_source_scenario=intrinsic_scale_gap_source_scenario,
        intrinsic_scale_gap_product_ratio_p10_threshold=intrinsic_scale_gap_product_ratio_p10_threshold,
    )
    selected = str(selection["selected_source_scenario"])
    replay = replay_candidate(
        catalog_json=catalog_json,
        catalog=catalog,
        scenario=selected,
        rows=rows,
        split=split,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
        target_error_limit_m=target_error_limit_m,
        allow_unvalidated=allow_unvalidated,
        stability_splits=("train", "validation"),
    )
    validation_only_fail_reasons: list[str] = []
    validation_improvement = finite_float(replay.get("validation_range_rmse_improvement_m"), float("nan"))
    target_count = int(float(replay.get("target_window_count", 0) or 0))
    target_abs = finite_float(replay.get("target_abs_mean_error_corrected_m"), float("nan"))
    target_max_abs = finite_float(replay.get("target_max_abs_error_corrected_m"), float("nan"))
    if not math.isfinite(validation_improvement) or validation_improvement <= 0.0:
        validation_only_fail_reasons.append("validation:no_positive_range_rmse_improvement")
    if target_count <= 0:
        validation_only_fail_reasons.append("validation:target_window_missing")
    if not math.isfinite(target_abs):
        validation_only_fail_reasons.append("validation:target_error_not_finite")
    elif target_abs > target_error_limit_m:
        validation_only_fail_reasons.append("validation:target_error_over_limit")
    if not math.isfinite(target_max_abs):
        validation_only_fail_reasons.append("validation:target_max_error_not_finite")
    elif target_max_abs > target_error_limit_m:
        validation_only_fail_reasons.append("validation:target_max_error_over_limit")
    validation_only_pass = not validation_only_fail_reasons
    feature_summary = selection["features"]
    row = {
        "target_scenario": target_scenario,
        "target_seed": target_seed,
        "input_csv": str(input_csv),
        "eligible_source_scenarios": ",".join(eligible_source_scenarios),
        "exclude_target_scenario_from_library": bool(exclude_target_scenario_from_library),
        "preliminary_source_scenario": selection.get("preliminary_source_scenario", ""),
        "selected_source_scenario": selected,
        "selection_policy": selection.get("selection_policy", ""),
        "fallback_reason": selection.get("fallback_reason", ""),
        "selected_distance": selection["selected_distance"],
        "distance_rows_json": json.dumps(selection["distance_rows"], ensure_ascii=False, allow_nan=False),
        "transfer_confusion_cluster": ",".join(str(item) for item in selection.get("transfer_confusion_cluster", [])),
        "transfer_risk_target_weight": transfer_risk_target_weight,
        "intrinsic_scale_gap_source_scenario": intrinsic_scale_gap_source_scenario,
        "intrinsic_scale_gap_product_ratio_p10_threshold": intrinsic_scale_gap_product_ratio_p10_threshold,
        "transfer_score_rows_json": json.dumps(
            selection.get("transfer_score_rows", []), ensure_ascii=False, allow_nan=False
        ),
        "feature_vote_rows_json": json.dumps(selection.get("feature_vote_rows", []), ensure_ascii=False, allow_nan=False),
        "knn_neighbor_rows_json": json.dumps(selection.get("knn_neighbor_rows", []), ensure_ascii=False, allow_nan=False),
        "pass_runtime_measurement_guard_replay": bool(replay.get("pass_replay")),
        "pass_reason": replay.get("pass_reason", ""),
        "validation_only_pass_runtime_measurement_guard_replay": validation_only_pass,
        "validation_only_pass_reason": "pass" if validation_only_pass else ";".join(validation_only_fail_reasons),
        "selected_method": replay.get("method", ""),
        "validation_range_rmse_raw_m": replay.get("validation_range_rmse_raw_m"),
        "validation_range_rmse_corrected_m": replay.get("validation_range_rmse_corrected_m"),
        "validation_range_rmse_improvement_m": replay.get("validation_range_rmse_improvement_m"),
        "target_window_count": replay.get("target_window_count"),
        "target_abs_mean_error_corrected_m": replay.get("target_abs_mean_error_corrected_m"),
        "target_max_abs_error_corrected_m": replay.get("target_max_abs_error_corrected_m"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_input_fields": ",".join(RAW_FEATURE_FIELDS),
        "forbidden_selection_fields": ",".join(sorted(FORBIDDEN_SELECTION_FIELDS)),
        "selection_contract": (
            "runtime measurement guard selection uses raw stereo-center RDF(right,down,forward), "
            "raw range, measured disparity, and image-center statistics only; truth is audit-only"
        ),
    }
    for name in FEATURE_NAMES:
        row[f"feature_{name}"] = feature_summary.get(name)
    return row


def summarize_audit(rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    pass_rows = [row for row in rows if bool(row.get("pass_runtime_measurement_guard_replay"))]
    validation_only_pass_rows = [
        row for row in rows if bool(row.get("validation_only_pass_runtime_measurement_guard_replay"))
    ]
    corrected = [
        finite_float(row.get("validation_range_rmse_corrected_m"), float("nan"))
        for row in rows
        if math.isfinite(finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")))
    ]
    target_abs = [
        finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan"))
        for row in rows
        if math.isfinite(finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")))
    ]
    target_max = [
        finite_float(row.get("target_max_abs_error_corrected_m"), float("nan"))
        for row in rows
        if math.isfinite(finite_float(row.get("target_max_abs_error_corrected_m"), float("nan")))
    ]
    return {
        "mode": "runtime_measurement_guard_audit",
        "audit_count": len(rows),
        "pass_count": len(pass_rows),
        "pass_all": len(rows) > 0 and len(pass_rows) == len(rows),
        "validation_only_pass_count": len(validation_only_pass_rows),
        "validation_only_pass_all": len(rows) > 0 and len(validation_only_pass_rows) == len(rows),
        "validation_range_rmse_corrected_mean_m": float(np.mean(corrected)) if corrected else float("nan"),
        "validation_range_rmse_corrected_max_m": float(np.max(corrected)) if corrected else float("nan"),
        "target_abs_mean_error_mean_m": float(np.mean(target_abs)) if target_abs else float("nan"),
        "target_abs_mean_error_max_m": float(np.max(target_abs)) if target_abs else float("nan"),
        "target_max_abs_error_max_m": float(np.max(target_max)) if target_max else float("nan"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_input_fields": ",".join(RAW_FEATURE_FIELDS),
        "forbidden_selection_fields": ",".join(sorted(FORBIDDEN_SELECTION_FIELDS)),
        "selection_contract": "truth-free raw-measurement profile selection; truth used only for offline replay audit",
    }


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "StereoDistance runtime measurement guard audit",
        "",
        "Purpose:",
        (
            "Choose a validated StereoDistance correction profile from raw runtime-like measurements "
            "without using calibration truth during selection."
        ),
        "",
        "Contracts:",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_input_fields: {summary.get('selection_input_fields', '')}",
        f"forbidden_selection_fields: {summary.get('forbidden_selection_fields', '')}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Summary:",
    ]
    for key in (
        "candidate_source_mode",
        "selection_policy",
        "candidate_source_scenarios",
        "transfer_matrix_csv",
        "transfer_matrix_row_count",
        "transfer_low_risk_rmse_limit_m",
        "transfer_low_risk_target_limit_m",
        "transfer_risk_target_weight",
        "knn_neighbor_count",
        "intrinsic_scale_gap_source_scenario",
        "intrinsic_scale_gap_product_ratio_p10_threshold",
        "audit_count",
        "pass_count",
        "pass_all",
        "validation_only_pass_count",
        "validation_only_pass_all",
        "validation_range_rmse_corrected_mean_m",
        "validation_range_rmse_corrected_max_m",
        "target_abs_mean_error_mean_m",
        "target_abs_mean_error_max_m",
        "target_max_abs_error_max_m",
    ):
        value = summary.get(key, "")
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Rows:"])
    for row in rows:
        lines.append(
            "{target_scenario}/{target_seed}: preliminary={preliminary} selected={selected} "
            "pass={passed} rmse_corrected={rmse} target_max_abs={target_max} reason={reason}".format(
                target_scenario=row.get("target_scenario", ""),
                target_seed=row.get("target_seed", ""),
                preliminary=row.get("preliminary_source_scenario", ""),
                selected=row.get("selected_source_scenario", ""),
                passed=row.get("pass_runtime_measurement_guard_replay", ""),
                rmse=row.get("validation_range_rmse_corrected_m", ""),
                target_max=row.get("target_max_abs_error_corrected_m", ""),
                reason=row.get("pass_reason", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_selection_counts(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    pairs: dict[tuple[str, str], int] = {}
    target_names: list[str] = []
    selected_names: list[str] = []
    for row in rows:
        target = str(row.get("target_scenario", "") or "input")
        selected = str(row.get("selected_source_scenario", "") or "none")
        pairs[(target, selected)] = pairs.get((target, selected), 0) + 1
        if target not in target_names:
            target_names.append(target)
        if selected not in selected_names:
            selected_names.append(selected)
    if not pairs:
        return
    matrix = np.zeros((len(target_names), len(selected_names)), dtype=float)
    for (target, selected), count in pairs.items():
        matrix[target_names.index(target), selected_names.index(selected)] = count
    width = max(7.0, 1.3 * len(selected_names))
    height = max(4.5, 0.65 * len(target_names))
    plt.figure(figsize=(width, height))
    plt.imshow(matrix, cmap="Blues", aspect="auto")
    plt.xticks(np.arange(len(selected_names)), selected_names, rotation=35, ha="right")
    plt.yticks(np.arange(len(target_names)), target_names)
    plt.xlabel("selected source profile")
    plt.ylabel("target audit scenario")
    plt.title("Runtime measurement guard selection counts")
    for y in range(matrix.shape[0]):
        for x in range(matrix.shape[1]):
            value = int(matrix[y, x])
            if value:
                plt.text(x, y, str(value), ha="center", va="center", color="black")
    plt.colorbar(label="count")
    plt.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(path, dpi=150)
    plt.close()


def run_guard(args: argparse.Namespace) -> dict[str, object]:
    catalog = load_catalog(args.catalog_json)
    exclude_target_scenario_from_library = bool(
        getattr(args, "exclude_target_scenario_from_library", False)
    )
    intrinsic_scale_gap_source_scenario = str(
        getattr(args, "intrinsic_scale_gap_source_scenario", DEFAULT_INTRINSIC_SCALE_GAP_SOURCE_SCENARIO)
    )
    intrinsic_scale_gap_product_ratio_p10_threshold = float(
        getattr(
            args,
            "intrinsic_scale_gap_product_ratio_p10_threshold",
            DEFAULT_INTRINSIC_SCALE_GAP_PRODUCT_RATIO_P10_THRESHOLD,
        )
    )
    source_scenarios = source_scenarios_for_mode(
        catalog=catalog,
        catalog_json=args.catalog_json,
        minimal_set_json=args.minimal_set_json,
        candidate_source_mode=args.candidate_source_mode,
    )
    transfer_matrix_csv = args.transfer_matrix_csv or default_transfer_matrix_csv(args.sweep_dir)
    transfer_matrix = load_transfer_matrix(transfer_matrix_csv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    audit_rows: list[dict[str, object]] = []
    if args.audit_all_sweep_seeds:
        for scenario, seed, input_csv in sweep_calibration_csvs(args.sweep_dir):
            audit_rows.append(
                audit_one_input(
                    sweep_dir=args.sweep_dir,
                    catalog_json=args.catalog_json,
                    catalog=catalog,
                    source_scenarios=source_scenarios,
                    input_csv=input_csv,
                    output_dir=args.output_dir,
                    split=args.split,
                    target_range_m=args.target_range_m,
                    target_window_m=args.target_window_m,
                    target_error_limit_m=args.target_error_limit_m,
                    allow_unvalidated=args.allow_unvalidated,
                    include_target_in_centroids=args.include_target_in_centroids,
                    selection_policy=args.selection_policy,
                    transfer_matrix=transfer_matrix,
                    transfer_low_risk_rmse_limit_m=args.transfer_low_risk_rmse_limit_m,
                    transfer_low_risk_target_limit_m=args.transfer_low_risk_target_limit_m,
                    transfer_risk_target_weight=args.transfer_risk_target_weight,
                    knn_neighbor_count=args.knn_neighbor_count,
                    exclude_target_scenario_from_library=exclude_target_scenario_from_library,
                    intrinsic_scale_gap_source_scenario=intrinsic_scale_gap_source_scenario,
                    intrinsic_scale_gap_product_ratio_p10_threshold=intrinsic_scale_gap_product_ratio_p10_threshold,
                    target_scenario=scenario,
                    target_seed=seed,
                )
            )
    else:
        if args.input_csv is None:
            raise ValueError("--input-csv is required unless --audit-all-sweep-seeds is set")
        audit_rows.append(
            audit_one_input(
                sweep_dir=args.sweep_dir,
                catalog_json=args.catalog_json,
                catalog=catalog,
                source_scenarios=source_scenarios,
                input_csv=args.input_csv,
                output_dir=args.output_dir,
                split=args.split,
                target_range_m=args.target_range_m,
                target_window_m=args.target_window_m,
                target_error_limit_m=args.target_error_limit_m,
                allow_unvalidated=args.allow_unvalidated,
                include_target_in_centroids=args.include_target_in_centroids,
                selection_policy=args.selection_policy,
                transfer_matrix=transfer_matrix,
                transfer_low_risk_rmse_limit_m=args.transfer_low_risk_rmse_limit_m,
                transfer_low_risk_target_limit_m=args.transfer_low_risk_target_limit_m,
                transfer_risk_target_weight=args.transfer_risk_target_weight,
                knn_neighbor_count=args.knn_neighbor_count,
                exclude_target_scenario_from_library=exclude_target_scenario_from_library,
                intrinsic_scale_gap_source_scenario=intrinsic_scale_gap_source_scenario,
                intrinsic_scale_gap_product_ratio_p10_threshold=intrinsic_scale_gap_product_ratio_p10_threshold,
            )
        )
        if args.output_profile is not None:
            selected = str(audit_rows[0]["selected_source_scenario"])
            shutil.copyfile(catalog_profile_path(args.catalog_json, catalog, selected), args.output_profile)
    write_csv(args.output_dir / "runtime_measurement_guard_audit.csv", audit_rows)
    summary = summarize_audit(audit_rows)
    summary["candidate_source_mode"] = args.candidate_source_mode
    summary["candidate_source_scenarios"] = ",".join(source_scenarios)
    summary["selection_policy"] = args.selection_policy
    summary["transfer_matrix_csv"] = str(transfer_matrix_csv) if transfer_matrix_csv else ""
    summary["transfer_matrix_row_count"] = len(transfer_matrix)
    summary["transfer_low_risk_rmse_limit_m"] = args.transfer_low_risk_rmse_limit_m
    summary["transfer_low_risk_target_limit_m"] = args.transfer_low_risk_target_limit_m
    summary["transfer_risk_target_weight"] = args.transfer_risk_target_weight
    summary["knn_neighbor_count"] = args.knn_neighbor_count
    summary["intrinsic_scale_gap_source_scenario"] = intrinsic_scale_gap_source_scenario
    summary["intrinsic_scale_gap_product_ratio_p10_threshold"] = intrinsic_scale_gap_product_ratio_p10_threshold
    summary["include_target_in_centroids"] = bool(args.include_target_in_centroids)
    summary["exclude_target_scenario_from_library"] = exclude_target_scenario_from_library
    write_json(args.output_dir / "runtime_measurement_guard_summary.json", summary)
    write_summary_txt(args.output_dir / "runtime_measurement_guard_summary.txt", summary, audit_rows)
    plot_selection_counts(args.output_dir / "runtime_measurement_guard_selection_counts.png", audit_rows)
    return summary


def main() -> int:
    summary = run_guard(parse_args())
    print(f"wrote runtime measurement guard audit to {summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
