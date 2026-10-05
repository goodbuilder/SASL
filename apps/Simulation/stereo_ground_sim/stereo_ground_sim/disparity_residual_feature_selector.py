from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Mapping

import numpy as np


FORBIDDEN_RUNTIME_TOKENS = ("truth", "true_", "gt_", "px4", "rtk", "ned", "validation", "replay", "target_window")


def load_json(path: str | Path) -> dict[str, object]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def has_forbidden_name(values: object) -> bool:
    if not isinstance(values, list):
        return True
    names = " ".join(str(value).lower() for value in values)
    return any(token in names for token in FORBIDDEN_RUNTIME_TOKENS)


def resolve_catalog_path(catalog_json: str | Path, value: object) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else (Path(catalog_json).resolve().parent / path).resolve()


def numeric_feature_value(row: Mapping[str, object], name: str) -> float:
    try:
        parsed = float(row.get(name, 0.0))
    except (TypeError, ValueError):
        return 0.0
    return parsed if math.isfinite(parsed) else 0.0


class DisparityResidualFeatureSelector:
    """Truth-free nearest-centroid selector for residual disparity profiles."""

    def __init__(self, *, selector_model_json: str | Path, catalog_json: str | Path):
        self.selector_model_json = Path(selector_model_json).resolve()
        self.catalog_json = Path(catalog_json).resolve()
        self.model = load_json(self.selector_model_json)
        self.catalog = load_json(self.catalog_json)
        self.feature_names = [str(name) for name in self.model.get("feature_names", [])]
        if not self.feature_names:
            raise ValueError("selector model feature_names is empty")
        if has_forbidden_name(self.feature_names):
            raise ValueError("selector model feature_names contain forbidden truth/runtime tokens")
        self.mean = np.asarray(self.model.get("mean", []), dtype=float)
        self.scale = np.asarray(self.model.get("scale", []), dtype=float)
        if self.mean.shape != (len(self.feature_names),) or self.scale.shape != (len(self.feature_names),):
            raise ValueError("selector model mean/scale length mismatch")
        self.scale = np.where(np.isfinite(self.scale) & (np.abs(self.scale) > 1.0e-12), self.scale, 1.0)
        centroids = self.model.get("centroids", {})
        if not isinstance(centroids, Mapping) or not centroids:
            raise ValueError("selector model centroids must be a non-empty object")
        self.centroids = {str(label): np.asarray(values, dtype=float) for label, values in centroids.items()}
        for label, centroid in self.centroids.items():
            if centroid.shape != self.mean.shape:
                raise ValueError(f"centroid length mismatch for {label}")
        paths = self.catalog.get("profile_paths", {})
        if not isinstance(paths, Mapping):
            raise ValueError("catalog profile_paths must be an object")
        self.profile_paths = {str(label): resolve_catalog_path(self.catalog_json, path) for label, path in paths.items()}

    def select(self, feature_row: Mapping[str, object]) -> dict[str, object]:
        vector = np.asarray([numeric_feature_value(feature_row, name) for name in self.feature_names], dtype=float)
        normalized = (vector - self.mean) / self.scale
        distances = sorted(
            (label, float(np.linalg.norm(normalized - centroid)))
            for label, centroid in self.centroids.items()
        )
        distances.sort(key=lambda item: item[1])
        selected_label, best_distance = distances[0]
        second_distance = distances[1][1] if len(distances) > 1 else best_distance + 1.0e9
        selected_profile = self.profile_paths.get(selected_label, Path(""))
        return {
            "selected_label": selected_label,
            "selected_profile": str(selected_profile),
            "selected_profile_exists": selected_profile.is_file(),
            "best_distance": best_distance,
            "second_distance": second_distance,
            "distance_margin": float(second_distance - best_distance),
            "runtime_truth_used": False,
        }
