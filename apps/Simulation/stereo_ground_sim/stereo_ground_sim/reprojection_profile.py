import hashlib
import json
import math
import statistics
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


FALLBACK_EXACT = 0
FALLBACK_NEAREST = 1
FALLBACK_NO_CELL = 2
FALLBACK_INVALID = 3


@dataclass(frozen=True)
class StereoCameraModel:
    width_px: float
    height_px: float
    horizontal_fov_rad: float
    baseline_m: float

    @property
    def fx(self) -> float:
        return (float(self.width_px) * 0.5) / math.tan(float(self.horizontal_fov_rad) * 0.5)

    @property
    def fy(self) -> float:
        return self.fx

    @property
    def cx(self) -> float:
        return float(self.width_px) * 0.5

    @property
    def cy(self) -> float:
        return float(self.height_px) * 0.5

    def project_frd(self, forward_m: float, right_m: float, down_m: float) -> Dict[str, float]:
        forward = float(forward_m)
        if not math.isfinite(forward) or forward <= 0.0:
            raise ValueError("forward_m must be finite and positive")
        right = float(right_m)
        down = float(down_m)
        half_baseline = float(self.baseline_m) * 0.5
        return {
            "left_u": self.cx + (right + half_baseline) * self.fx / forward,
            "left_v": self.cy + down * self.fy / forward,
            "right_u": self.cx + (right - half_baseline) * self.fx / forward,
            "right_v": self.cy + down * self.fy / forward,
        }

    def triangulate(self, left_u: float, left_v: float, right_u: float, right_v: float) -> Dict[str, float]:
        disparity = abs(float(left_u) - float(right_u))
        if not math.isfinite(disparity) or disparity < 0.5:
            raise ValueError("disparity_px is invalid")
        forward = self.fx * float(self.baseline_m) / disparity
        target_u = (float(left_u) + float(right_u)) * 0.5
        target_v = (float(left_v) + float(right_v)) * 0.5
        right = (target_u - self.cx) * forward / self.fx
        down = (target_v - self.cy) * forward / self.fy
        return {
            "forward_m": forward,
            "right_m": right,
            "down_m": down,
            "range_m": math.sqrt(forward * forward + right * right + down * down),
            "disparity_px": disparity,
        }


@dataclass(frozen=True)
class ReprojectionResidualSample:
    left_u: float
    left_v: float
    right_u: float
    right_v: float
    raw_disparity_px: float
    raw_range_m: Optional[float]
    du_left: float
    dv_left: float
    du_right: float
    dv_right: float


@dataclass(frozen=True)
class ReprojectionLookupResult:
    corrected_left_u: float
    corrected_left_v: float
    corrected_right_u: float
    corrected_right_v: float
    du_left: float
    dv_left: float
    du_right: float
    dv_right: float
    fallback_level: int
    fallback_reason: str
    profile_id: str
    sample_count: int


def _finite(value) -> bool:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number)


def _as_float(row: Mapping[str, object], *names: str) -> Optional[float]:
    for name in names:
        if name not in row:
            continue
        value = row[name]
        if value in ("", None):
            continue
        if _finite(value):
            return float(value)
    return None


def residual_sample_from_row(row: Mapping[str, object], camera: StereoCameraModel) -> ReprojectionResidualSample:
    left_u = _as_float(row, "left_cx", "left_cx_px", "left_bbox_cx_px")
    left_v = _as_float(row, "left_cy", "left_cy_px", "left_bbox_cy_px")
    right_u = _as_float(row, "right_cx", "right_cx_px", "right_bbox_cx_px")
    right_v = _as_float(row, "right_cy", "right_cy_px", "right_bbox_cy_px")
    forward = _as_float(row, "gt_forward_m", "drone_forward_m", "truth_forward_m")
    right = _as_float(row, "gt_right_m", "drone_right_m", "truth_right_m")
    down = _as_float(row, "gt_down_m", "drone_down_m", "truth_down_m")
    raw_disparity = _as_float(row, "raw_disparity_px", "disparity_px")
    raw_range = _as_float(row, "raw_range_m", "estimated_range_m")
    required = (left_u, left_v, right_u, right_v, forward, right, down)
    if any(value is None for value in required):
        raise ValueError("row does not contain complete bbox center and truth FRD fields")
    if raw_disparity is None:
        raw_disparity = abs(float(left_u) - float(right_u))
    projected = camera.project_frd(forward, right, down)
    return ReprojectionResidualSample(
        left_u=float(left_u),
        left_v=float(left_v),
        right_u=float(right_u),
        right_v=float(right_v),
        raw_disparity_px=float(raw_disparity),
        raw_range_m=None if raw_range is None else float(raw_range),
        du_left=projected["left_u"] - float(left_u),
        dv_left=projected["left_v"] - float(left_v),
        du_right=projected["right_u"] - float(right_u),
        dv_right=projected["right_v"] - float(right_v),
    )


def residual_samples_from_rows(
    rows: Iterable[Mapping[str, object]], camera: StereoCameraModel
) -> List[ReprojectionResidualSample]:
    samples = []
    for row in rows:
        try:
            samples.append(residual_sample_from_row(row, camera))
        except ValueError:
            continue
    return samples


def _bin_index(value: float, edges: Sequence[float]) -> Optional[int]:
    if not _finite(value) or len(edges) < 2:
        return None
    number = float(value)
    if number < float(edges[0]) or number > float(edges[-1]):
        return None
    for index in range(len(edges) - 1):
        lo = float(edges[index])
        hi = float(edges[index + 1])
        if lo <= number < hi or (index == len(edges) - 2 and number <= hi):
            return index
    return None


def _median(values: Iterable[float]) -> float:
    return float(statistics.median(float(value) for value in values))


class OriginBBoxReprojectionProfile:
    mode = "origin_bbox_reprojection_grid"

    def __init__(
        self,
        profile_id: str,
        image_width_px: float,
        image_height_px: float,
        u_edges: Sequence[float],
        v_edges: Sequence[float],
        disparity_edges: Sequence[float],
        cells: Mapping[str, Mapping[str, float]],
        min_disparity_px: float = 0.5,
        lookup_policy: str = "exact",
        max_nearest_cell_distance: float = 1.75,
    ):
        self.profile_id = str(profile_id)
        self.profile_id_hash = int(hashlib.sha1(self.profile_id.encode("utf-8")).hexdigest()[:7], 16)
        self.image_width_px = float(image_width_px)
        self.image_height_px = float(image_height_px)
        self.u_edges = [float(value) for value in u_edges]
        self.v_edges = [float(value) for value in v_edges]
        self.disparity_edges = [float(value) for value in disparity_edges]
        self.cells = {str(key): dict(value) for key, value in cells.items()}
        self.min_disparity_px = float(min_disparity_px)
        self.lookup_policy = str(lookup_policy or "exact")
        self.max_nearest_cell_distance = float(max_nearest_cell_distance)

    @classmethod
    def from_json_dict(cls, data: Mapping[str, object]):
        grid = data.get("grid") or {}
        return cls(
            profile_id=str(data.get("profile_id") or data.get("id") or "origin_bbox_reprojection"),
            image_width_px=float(data.get("image_width_px", 1280.0)),
            image_height_px=float(data.get("image_height_px", 720.0)),
            u_edges=grid.get("u_edges", data.get("u_edges", [])),
            v_edges=grid.get("v_edges", data.get("v_edges", [])),
            disparity_edges=grid.get("disparity_edges", data.get("disparity_edges", [])),
            cells=data.get("cells", {}),
            min_disparity_px=float(data.get("min_disparity_px", 0.5)),
            lookup_policy=str(data.get("lookup_policy", "exact")),
            max_nearest_cell_distance=float(data.get("max_nearest_cell_distance", 1.75)),
        )

    @classmethod
    def build(
        cls,
        samples: Sequence[ReprojectionResidualSample],
        image_width_px: float,
        image_height_px: float,
        u_bins: int = 6,
        v_bins: int = 4,
        disparity_bins: int = 6,
        min_samples_per_cell: int = 3,
        profile_id: str = "origin_bbox_reprojection",
        lookup_policy: str = "exact",
        min_disparity_px: float = 0.5,
    ):
        if u_bins <= 0 or v_bins <= 0 or disparity_bins <= 0:
            raise ValueError("bin counts must be positive")
        valid = [
            sample
            for sample in samples
            if _finite(sample.raw_disparity_px) and float(sample.raw_disparity_px) >= min_disparity_px
        ]
        if not valid:
            raise ValueError("no valid samples for reprojection profile")
        u_edges = [float(image_width_px) * index / u_bins for index in range(u_bins + 1)]
        v_edges = [float(image_height_px) * index / v_bins for index in range(v_bins + 1)]
        disparities = sorted(float(sample.raw_disparity_px) for sample in valid)
        if len(set(round(value, 9) for value in disparities)) == 1:
            center = disparities[0]
            disparity_edges = [max(min_disparity_px, center - 0.5), center + 0.5]
        else:
            disparity_edges = []
            for index in range(disparity_bins + 1):
                position = (len(disparities) - 1) * index / disparity_bins
                lo = math.floor(position)
                hi = math.ceil(position)
                if lo == hi:
                    edge = disparities[int(position)]
                else:
                    edge = disparities[lo] * (hi - position) + disparities[hi] * (position - lo)
                disparity_edges.append(float(edge))
            disparity_edges[0] = max(min_disparity_px, disparity_edges[0] - 1.0e-6)
            disparity_edges[-1] = disparity_edges[-1] + 1.0e-6
        grouped: Dict[str, List[ReprojectionResidualSample]] = {}
        for sample in valid:
            key = cls._cell_key_for_values(
                sample.left_u,
                sample.left_v,
                sample.right_u,
                sample.right_v,
                sample.raw_disparity_px,
                u_edges,
                v_edges,
                disparity_edges,
            )
            if key is None:
                continue
            grouped.setdefault(key, []).append(sample)
        cells = {}
        for key, group in grouped.items():
            if len(group) < min_samples_per_cell:
                continue
            cells[key] = {
                "du_left": _median(sample.du_left for sample in group),
                "dv_left": _median(sample.dv_left for sample in group),
                "du_right": _median(sample.du_right for sample in group),
                "dv_right": _median(sample.dv_right for sample in group),
                "sample_count": float(len(group)),
            }
        return cls(
            profile_id=profile_id,
            image_width_px=image_width_px,
            image_height_px=image_height_px,
            u_edges=u_edges,
            v_edges=v_edges,
            disparity_edges=disparity_edges,
            cells=cells,
            min_disparity_px=min_disparity_px,
            lookup_policy=lookup_policy,
        )

    @staticmethod
    def _cell_key_for_values(
        left_u: float,
        left_v: float,
        right_u: float,
        right_v: float,
        raw_disparity_px: float,
        u_edges: Sequence[float],
        v_edges: Sequence[float],
        disparity_edges: Sequence[float],
    ) -> Optional[str]:
        screen_u = (float(left_u) + float(right_u)) * 0.5
        screen_v = (float(left_v) + float(right_v)) * 0.5
        u_index = _bin_index(screen_u, u_edges)
        v_index = _bin_index(screen_v, v_edges)
        disparity_index = _bin_index(raw_disparity_px, disparity_edges)
        if u_index is None or v_index is None or disparity_index is None:
            return None
        return f"{u_index}:{v_index}:{disparity_index}"

    def _cell_key(self, left_u: float, left_v: float, right_u: float, right_v: float, disparity: float):
        return self._cell_key_for_values(
            left_u,
            left_v,
            right_u,
            right_v,
            disparity,
            self.u_edges,
            self.v_edges,
            self.disparity_edges,
        )

    def _nearest_cell(self, key: str):
        if self.lookup_policy != "nearest" or not self.cells:
            return None, None
        target = tuple(int(part) for part in key.split(":"))
        best_key = None
        best_distance = None
        for candidate in self.cells:
            point = tuple(int(part) for part in candidate.split(":"))
            distance = math.sqrt(sum((a - b) * (a - b) for a, b in zip(target, point)))
            if best_distance is None or distance < best_distance:
                best_key = candidate
                best_distance = distance
        if best_distance is None or best_distance > self.max_nearest_cell_distance:
            return None, None
        return best_key, best_distance

    def apply(
        self,
        left_u: float,
        left_v: float,
        right_u: float,
        right_v: float,
        raw_disparity_px: Optional[float] = None,
        raw_range_m: Optional[float] = None,
    ) -> ReprojectionLookupResult:
        values = (left_u, left_v, right_u, right_v)
        if any(not _finite(value) for value in values):
            return self._noop(left_u, left_v, right_u, right_v, FALLBACK_INVALID, "incomplete_bbox_pair")
        disparity = abs(float(left_u) - float(right_u)) if raw_disparity_px is None else float(raw_disparity_px)
        if not _finite(disparity) or disparity < self.min_disparity_px:
            return self._noop(left_u, left_v, right_u, right_v, FALLBACK_INVALID, "bad_disparity")
        key = self._cell_key(left_u, left_v, right_u, right_v, disparity)
        if key is None:
            return self._noop(left_u, left_v, right_u, right_v, FALLBACK_NO_CELL, "outside_profile_range")
        cell = self.cells.get(key)
        fallback_level = FALLBACK_EXACT
        fallback_reason = "exact"
        if cell is None:
            nearest_key, _distance = self._nearest_cell(key)
            if nearest_key is None:
                return self._noop(left_u, left_v, right_u, right_v, FALLBACK_NO_CELL, "missing_cell")
            cell = self.cells[nearest_key]
            fallback_level = FALLBACK_NEAREST
            fallback_reason = "nearest_cell"
        du_left = float(cell.get("du_left", 0.0))
        dv_left = float(cell.get("dv_left", 0.0))
        du_right = float(cell.get("du_right", 0.0))
        dv_right = float(cell.get("dv_right", 0.0))
        return ReprojectionLookupResult(
            corrected_left_u=float(left_u) + du_left,
            corrected_left_v=float(left_v) + dv_left,
            corrected_right_u=float(right_u) + du_right,
            corrected_right_v=float(right_v) + dv_right,
            du_left=du_left,
            dv_left=dv_left,
            du_right=du_right,
            dv_right=dv_right,
            fallback_level=fallback_level,
            fallback_reason=fallback_reason,
            profile_id=self.profile_id,
            sample_count=int(float(cell.get("sample_count", 0.0))),
        )

    def _noop(self, left_u, left_v, right_u, right_v, level: int, reason: str) -> ReprojectionLookupResult:
        return ReprojectionLookupResult(
            corrected_left_u=float(left_u) if _finite(left_u) else math.nan,
            corrected_left_v=float(left_v) if _finite(left_v) else math.nan,
            corrected_right_u=float(right_u) if _finite(right_u) else math.nan,
            corrected_right_v=float(right_v) if _finite(right_v) else math.nan,
            du_left=0.0,
            dv_left=0.0,
            du_right=0.0,
            dv_right=0.0,
            fallback_level=level,
            fallback_reason=reason,
            profile_id=self.profile_id,
            sample_count=0,
        )

    def to_json_dict(self) -> Dict[str, object]:
        return {
            "mode": self.mode,
            "profile_id": self.profile_id,
            "feature_contract": "avg-screen-grid(left/right bbox center u/v) + raw_disparity_px",
            "label_contract": "duL,dvL,duR,dvR = GT reprojection pixel - YOLO bbox center pixel",
            "image_width_px": self.image_width_px,
            "image_height_px": self.image_height_px,
            "min_disparity_px": self.min_disparity_px,
            "lookup_policy": self.lookup_policy,
            "max_nearest_cell_distance": self.max_nearest_cell_distance,
            "grid": {
                "u_edges": self.u_edges,
                "v_edges": self.v_edges,
                "disparity_edges": self.disparity_edges,
            },
            "cells": self.cells,
        }


def dump_profile_json(profile: OriginBBoxReprojectionProfile, path: str) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(profile.to_json_dict(), handle, ensure_ascii=False, indent=2)
        handle.write("\n")
