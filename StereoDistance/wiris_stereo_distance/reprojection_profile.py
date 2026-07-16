from __future__ import annotations

import hashlib
import json
import math
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from wiris_stereo_distance.calibration import CameraIntrinsics, StereoIntrinsicsResult
from wiris_stereo_distance.triangulation import (
    stereo_center_point_from_left_camera,
    triangulate_pixels_with_relative_pose,
)
from wiris_stereo_distance.uam_telemetry import UamAlignmentProfile


ORIGIN_BBOX_REPROJECTION_METHOD = "origin-bbox-reprojection-grid-v1"
ORIGIN_BBOX_REPROJECTION_MODE = "origin_bbox_reprojection_grid"
DEFAULT_REPROJECTION_CORRECTION_MODE = "component_gains"
DEFAULT_REPROJECTION_DISPARITY_GAIN = 0.125
DEFAULT_REPROJECTION_SCREEN_U_GAIN = 0.0
DEFAULT_REPROJECTION_SCREEN_V_GAIN = 0.025
DEFAULT_REPROJECTION_OUTPUT_POLICY = "range_only_forward_fusion"
FALLBACK_EXACT = 0
FALLBACK_NEAREST = 1
FALLBACK_NO_CELL = 2
FALLBACK_INVALID = 3


@dataclass(frozen=True)
class RectifiedStereoCameraModel:
    image_width_px: float
    image_height_px: float
    focal_length_px: float
    baseline_m: float
    left_cx_px: float | None = None
    left_cy_px: float | None = None
    right_cx_px: float | None = None
    right_cy_px: float | None = None
    fy_px: float | None = None

    @property
    def fx(self) -> float:
        return float(self.focal_length_px)

    @property
    def fy(self) -> float:
        return float(self.fy_px if self.fy_px is not None else self.focal_length_px)

    @property
    def left_cx(self) -> float:
        return float(self.left_cx_px if self.left_cx_px is not None else self.image_width_px * 0.5)

    @property
    def left_cy(self) -> float:
        return float(self.left_cy_px if self.left_cy_px is not None else self.image_height_px * 0.5)

    @property
    def right_cx(self) -> float:
        return float(self.right_cx_px if self.right_cx_px is not None else self.image_width_px * 0.5)

    @property
    def right_cy(self) -> float:
        return float(self.right_cy_px if self.right_cy_px is not None else self.image_height_px * 0.5)

    def _intrinsics(self, *, cx: float, cy: float) -> CameraIntrinsics:
        return CameraIntrinsics(
            width=max(1, int(round(self.image_width_px))),
            height=max(1, int(round(self.image_height_px))),
            camera_matrix=[
                [self.fx, 0.0, float(cx)],
                [0.0, self.fy, float(cy)],
                [0.0, 0.0, 1.0],
            ],
            distortion_coefficients=[],
            reprojection_error_px=0.0,
            image_count=0,
        )

    def project_truth_center(
        self,
        *,
        right_m: float,
        down_m: float,
        forward_m: float,
    ) -> dict[str, float]:
        forward = float(forward_m)
        if not math.isfinite(forward) or forward <= 0.0:
            raise ValueError("truth forward_m must be finite and positive")
        right = float(right_m)
        down = float(down_m)
        half_baseline = float(self.baseline_m) * 0.5
        return {
            "left_u": self.left_cx + (right + half_baseline) * self.fx / forward,
            "left_v": self.left_cy + down * self.fy / forward,
            "right_u": self.right_cx + (right - half_baseline) * self.fx / forward,
            "right_v": self.right_cy + down * self.fy / forward,
        }

    def triangulate_centers(
        self,
        *,
        left_u: float,
        left_v: float,
        right_u: float,
        right_v: float,
    ) -> dict[str, float]:
        disparity = abs(float(left_u) - float(right_u))
        if not _finite(disparity) or disparity < 0.5:
            raise ValueError("disparity_px is invalid")
        translation = (float(self.baseline_m), 0.0, 0.0)
        triangulated = triangulate_pixels_with_relative_pose(
            left_pixel=(float(left_u), float(left_v)),
            right_pixel=(float(right_u), float(right_v)),
            left_intrinsics=self._intrinsics(cx=self.left_cx, cy=self.left_cy),
            right_intrinsics=self._intrinsics(cx=self.right_cx, cy=self.right_cy),
            rotation_right_to_left=[
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            translation_right_in_left_m=translation,
        )
        if triangulated is None:
            raise ValueError("ray triangulation failed")
        right, down, forward = stereo_center_point_from_left_camera(
            triangulated.point_left_m,
            translation,
        )
        return {
            "right_m": right,
            "down_m": down,
            "forward_m": forward,
            "range_m": math.sqrt(right * right + down * down + forward * forward),
            "disparity_px": disparity,
        }


@dataclass(frozen=True)
class ReprojectionResidualSample:
    left_u: float
    left_v: float
    right_u: float
    right_v: float
    raw_disparity_px: float
    raw_range_m: float | None
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
    cell_key: str
    sample_count: int

    @property
    def applied(self) -> bool:
        return self.fallback_level in (FALLBACK_EXACT, FALLBACK_NEAREST)


def _finite(value: object) -> bool:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number)


def _as_float(row: Mapping[str, object], *names: str) -> float | None:
    for name in names:
        value = row.get(name)
        if value in (None, ""):
            continue
        if _finite(value):
            return float(value)
    return None


def _median(values: Iterable[float]) -> float:
    return float(statistics.median(float(value) for value in values))


def _bin_index(value: float, edges: Sequence[float]) -> int | None:
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


def _quantile_edges(values: Sequence[float], bins: int, *, min_edge: float | None = None) -> list[float]:
    if bins <= 0:
        raise ValueError("bin count must be positive")
    ordered = sorted(float(value) for value in values if _finite(value))
    if not ordered:
        raise ValueError("cannot build edges from empty values")
    unique = {round(value, 9) for value in ordered}
    if len(unique) == 1:
        center = ordered[0]
        lo = center - 0.5
        if min_edge is not None:
            lo = max(float(min_edge), lo)
        return [lo, center + 0.5]
    edges: list[float] = []
    for index in range(bins + 1):
        position = (len(ordered) - 1) * index / bins
        lo_index = math.floor(position)
        hi_index = math.ceil(position)
        if lo_index == hi_index:
            edge = ordered[int(position)]
        else:
            edge = ordered[lo_index] * (hi_index - position) + ordered[hi_index] * (position - lo_index)
        edges.append(float(edge))
    if min_edge is not None:
        edges[0] = max(float(min_edge), edges[0] - 1.0e-6)
    else:
        edges[0] -= 1.0e-6
    edges[-1] += 1.0e-6
    return edges


def truth_stereo_center_from_row(
    row: Mapping[str, object],
    *,
    alignment_profile: UamAlignmentProfile | None = None,
) -> tuple[float, float, float] | None:
    right = _as_float(row, "truth_right_m", "truth_rdf_right_m", "drone_right_m", "gt_right_m")
    down = _as_float(row, "truth_down_m", "truth_rdf_down_m", "drone_down_m", "gt_down_m")
    forward = _as_float(row, "truth_forward_m", "truth_rdf_forward_m", "drone_forward_m", "gt_forward_m")
    if right is not None and down is not None and forward is not None:
        return right, down, forward

    frd_forward = _as_float(row, "truth_frd_forward_m", "gt_frd_forward_m")
    frd_right = _as_float(row, "truth_frd_right_m", "gt_frd_right_m")
    frd_down = _as_float(row, "truth_frd_down_m", "gt_frd_down_m")
    if frd_forward is not None and frd_right is not None and frd_down is not None:
        return frd_right, frd_down, frd_forward

    north = _as_float(row, "truth_north_m", "truth_n_m", "drone_north_m", "drone_x_m")
    east = _as_float(row, "truth_east_m", "truth_e_m", "drone_east_m", "drone_y_m")
    ned_down = _as_float(row, "truth_down_ned_m", "truth_d_m", "drone_down_ned_m", "drone_z_m")
    if alignment_profile is not None and north is not None and east is not None and ned_down is not None:
        return alignment_profile.inverse_apply((north, east, ned_down))
    return None


def residual_sample_from_row(
    row: Mapping[str, object],
    camera: RectifiedStereoCameraModel,
    *,
    alignment_profile: UamAlignmentProfile | None = None,
) -> ReprojectionResidualSample:
    left_u = _as_float(row, "left_cx", "left_cx_px", "left_bbox_cx_px", "left_bbox_center_x_px")
    left_v = _as_float(row, "left_cy", "left_cy_px", "left_bbox_cy_px", "left_bbox_center_y_px")
    right_u = _as_float(row, "right_cx", "right_cx_px", "right_bbox_cx_px", "right_bbox_center_x_px")
    right_v = _as_float(row, "right_cy", "right_cy_px", "right_bbox_cy_px", "right_bbox_center_y_px")
    truth = truth_stereo_center_from_row(row, alignment_profile=alignment_profile)
    raw_disparity = _as_float(row, "raw_disparity_px", "disparity_px")
    raw_range = _as_float(row, "raw_range_m", "estimated_range_m", "measurement_distance_m", "distance_m")
    if None in (left_u, left_v, right_u, right_v) or truth is None:
        raise ValueError("row does not contain complete bbox center and truth fields")
    if raw_disparity is None:
        raw_disparity = abs(float(left_u) - float(right_u))
    projected = camera.project_truth_center(right_m=truth[0], down_m=truth[1], forward_m=truth[2])
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
    rows: Iterable[Mapping[str, object]],
    camera: RectifiedStereoCameraModel,
    *,
    alignment_profile: UamAlignmentProfile | None = None,
) -> list[ReprojectionResidualSample]:
    samples: list[ReprojectionResidualSample] = []
    for row in rows:
        try:
            samples.append(residual_sample_from_row(row, camera, alignment_profile=alignment_profile))
        except ValueError:
            continue
    return samples


class OriginBBoxReprojectionProfile:
    method = ORIGIN_BBOX_REPROJECTION_METHOD
    mode = ORIGIN_BBOX_REPROJECTION_MODE

    def __init__(
        self,
        *,
        profile_id: str,
        image_width_px: float,
        image_height_px: float,
        u_edges: Sequence[float],
        v_edges: Sequence[float],
        depth_edges: Sequence[float],
        cells: Mapping[str, Mapping[str, object]],
        bin_axis: str = "disparity",
        min_disparity_px: float = 0.5,
        lookup_policy: str = "exact",
        max_nearest_cell_distance: float = 1.75,
        correction_mode: str = "full",
        disparity_gain: float = 1.0,
        screen_u_gain: float = 1.0,
        screen_v_gain: float = 1.0,
        output_policy: str = DEFAULT_REPROJECTION_OUTPUT_POLICY,
        created_unix_s: float | None = None,
        calibration_envelope: Mapping[str, object] | None = None,
    ) -> None:
        if bin_axis not in ("disparity", "range"):
            raise ValueError("bin_axis must be 'disparity' or 'range'")
        if correction_mode not in ("full", "disparity_only", "component_gains"):
            raise ValueError("correction_mode must be 'full', 'disparity_only', or 'component_gains'")
        if output_policy not in ("triangulated", "range_only_forward_fusion"):
            raise ValueError("output_policy must be 'triangulated' or 'range_only_forward_fusion'")
        self.profile_id = str(profile_id)
        self.profile_id_hash = int(hashlib.sha1(self.profile_id.encode("utf-8")).hexdigest()[:7], 16)
        self.image_width_px = float(image_width_px)
        self.image_height_px = float(image_height_px)
        self.u_edges = [float(value) for value in u_edges]
        self.v_edges = [float(value) for value in v_edges]
        self.depth_edges = [float(value) for value in depth_edges]
        self.cells = {str(key): dict(value) for key, value in cells.items()}
        self.bin_axis = str(bin_axis)
        self.min_disparity_px = float(min_disparity_px)
        self.lookup_policy = str(lookup_policy or "exact")
        self.max_nearest_cell_distance = float(max_nearest_cell_distance)
        self.correction_mode = str(correction_mode)
        self.disparity_gain = float(disparity_gain)
        self.screen_u_gain = float(screen_u_gain)
        self.screen_v_gain = float(screen_v_gain)
        self.output_policy = str(output_policy)
        self.created_unix_s = time.time() if created_unix_s is None else float(created_unix_s)
        self.calibration_envelope = dict(calibration_envelope or {})

    @classmethod
    def from_dict(cls, data: Mapping[str, object]) -> "OriginBBoxReprojectionProfile":
        grid = data.get("grid") or {}
        if not isinstance(grid, Mapping):
            grid = {}
        depth_edges = (
            grid.get("depth_edges")
            or grid.get("disparity_edges")
            or grid.get("range_edges_m")
            or data.get("depth_edges")
            or data.get("disparity_edges")
            or data.get("range_edges_m")
            or []
        )
        return cls(
            profile_id=str(data.get("profile_id") or data.get("id") or "origin_bbox_reprojection"),
            image_width_px=float(data.get("image_width_px", 1280.0)),
            image_height_px=float(data.get("image_height_px", 720.0)),
            u_edges=grid.get("u_edges", data.get("u_edges", [])),
            v_edges=grid.get("v_edges", data.get("v_edges", [])),
            depth_edges=depth_edges,
            cells=data.get("cells", {}),
            bin_axis=str(data.get("bin_axis", "disparity")),
            min_disparity_px=float(data.get("min_disparity_px", 0.5)),
            lookup_policy=str(data.get("lookup_policy", "exact")),
            max_nearest_cell_distance=float(data.get("max_nearest_cell_distance", 1.75)),
            correction_mode=str(data.get("correction_mode", DEFAULT_REPROJECTION_CORRECTION_MODE)),
            disparity_gain=float(data.get("disparity_gain", DEFAULT_REPROJECTION_DISPARITY_GAIN)),
            screen_u_gain=float(data.get("screen_u_gain", DEFAULT_REPROJECTION_SCREEN_U_GAIN)),
            screen_v_gain=float(data.get("screen_v_gain", DEFAULT_REPROJECTION_SCREEN_V_GAIN)),
            output_policy=str(data.get("output_policy", DEFAULT_REPROJECTION_OUTPUT_POLICY)),
            created_unix_s=float(data.get("created_unix_s", time.time())),
            calibration_envelope=data.get("calibration_envelope") if isinstance(data.get("calibration_envelope"), Mapping) else {},
        )

    @classmethod
    def build(
        cls,
        samples: Sequence[ReprojectionResidualSample],
        *,
        image_width_px: float,
        image_height_px: float,
        u_bins: int = 6,
        v_bins: int = 4,
        depth_bins: int = 6,
        min_samples_per_cell: int = 3,
        profile_id: str = "origin_bbox_reprojection",
        bin_axis: str = "disparity",
        lookup_policy: str = "exact",
        min_disparity_px: float = 0.5,
        correction_mode: str = DEFAULT_REPROJECTION_CORRECTION_MODE,
        disparity_gain: float = DEFAULT_REPROJECTION_DISPARITY_GAIN,
        screen_u_gain: float = DEFAULT_REPROJECTION_SCREEN_U_GAIN,
        screen_v_gain: float = DEFAULT_REPROJECTION_SCREEN_V_GAIN,
        output_policy: str = DEFAULT_REPROJECTION_OUTPUT_POLICY,
        calibration_envelope: Mapping[str, object] | None = None,
    ) -> "OriginBBoxReprojectionProfile":
        if u_bins <= 0 or v_bins <= 0 or depth_bins <= 0:
            raise ValueError("bin counts must be positive")
        if bin_axis not in ("disparity", "range"):
            raise ValueError("bin_axis must be 'disparity' or 'range'")
        valid = [
            sample
            for sample in samples
            if _finite(sample.raw_disparity_px) and float(sample.raw_disparity_px) >= min_disparity_px
        ]
        if bin_axis == "range":
            valid = [sample for sample in valid if sample.raw_range_m is not None and _finite(sample.raw_range_m)]
        if not valid:
            raise ValueError("no valid samples for reprojection profile")
        u_edges = [float(image_width_px) * index / u_bins for index in range(u_bins + 1)]
        v_edges = [float(image_height_px) * index / v_bins for index in range(v_bins + 1)]
        depth_values = [
            float(sample.raw_disparity_px if bin_axis == "disparity" else sample.raw_range_m)
            for sample in valid
        ]
        depth_edges = _quantile_edges(
            depth_values,
            depth_bins,
            min_edge=min_disparity_px if bin_axis == "disparity" else 0.0,
        )
        grouped: dict[str, list[ReprojectionResidualSample]] = {}
        for sample in valid:
            key = cls._cell_key_for_values(
                sample.left_u,
                sample.left_v,
                sample.right_u,
                sample.right_v,
                sample.raw_disparity_px,
                sample.raw_range_m,
                u_edges,
                v_edges,
                depth_edges,
                bin_axis,
            )
            if key is not None:
                grouped.setdefault(key, []).append(sample)
        cells: dict[str, dict[str, object]] = {}
        for key, group in grouped.items():
            if len(group) < min_samples_per_cell:
                continue
            screen_u_index, screen_v_index, depth_index = (int(part) for part in key.split(":"))
            cells[key] = {
                "u_bin": screen_u_index,
                "v_bin": screen_v_index,
                "depth_bin": depth_index,
                "du_left": _median(sample.du_left for sample in group),
                "dv_left": _median(sample.dv_left for sample in group),
                "du_right": _median(sample.du_right for sample in group),
                "dv_right": _median(sample.dv_right for sample in group),
                "sample_count": len(group),
            }
        return cls(
            profile_id=profile_id,
            image_width_px=image_width_px,
            image_height_px=image_height_px,
            u_edges=u_edges,
            v_edges=v_edges,
            depth_edges=depth_edges,
            cells=cells,
            bin_axis=bin_axis,
            min_disparity_px=min_disparity_px,
            lookup_policy=lookup_policy,
            correction_mode=correction_mode,
            disparity_gain=disparity_gain,
            screen_u_gain=screen_u_gain,
            screen_v_gain=screen_v_gain,
            output_policy=output_policy,
            calibration_envelope=calibration_envelope,
        )

    def _effective_residuals(
        self,
        *,
        du_left: float,
        dv_left: float,
        du_right: float,
        dv_right: float,
    ) -> tuple[float, float, float, float]:
        if self.correction_mode == "full":
            return du_left, dv_left, du_right, dv_right
        horizontal_shift = (du_left + du_right) * 0.5
        horizontal_disparity = (du_left - du_right) * 0.5
        vertical_shift = (dv_left + dv_right) * 0.5
        if self.correction_mode == "disparity_only":
            return horizontal_disparity, 0.0, -horizontal_disparity, 0.0
        corrected_horizontal_shift = horizontal_shift * self.screen_u_gain
        corrected_horizontal_disparity = horizontal_disparity * self.disparity_gain
        corrected_vertical_shift = vertical_shift * self.screen_v_gain
        return (
            corrected_horizontal_shift + corrected_horizontal_disparity,
            corrected_vertical_shift,
            corrected_horizontal_shift - corrected_horizontal_disparity,
            corrected_vertical_shift,
        )

    @staticmethod
    def _cell_key_for_values(
        left_u: float,
        left_v: float,
        right_u: float,
        right_v: float,
        raw_disparity_px: float | None,
        raw_range_m: float | None,
        u_edges: Sequence[float],
        v_edges: Sequence[float],
        depth_edges: Sequence[float],
        bin_axis: str,
    ) -> str | None:
        screen_u = (float(left_u) + float(right_u)) * 0.5
        screen_v = (float(left_v) + float(right_v)) * 0.5
        u_index = _bin_index(screen_u, u_edges)
        v_index = _bin_index(screen_v, v_edges)
        depth_value = raw_disparity_px if bin_axis == "disparity" else raw_range_m
        depth_index = _bin_index(float("nan") if depth_value is None else float(depth_value), depth_edges)
        if u_index is None or v_index is None or depth_index is None:
            return None
        return f"{u_index}:{v_index}:{depth_index}"

    def _cell_key(
        self,
        left_u: float,
        left_v: float,
        right_u: float,
        right_v: float,
        raw_disparity_px: float | None,
        raw_range_m: float | None,
    ) -> str | None:
        return self._cell_key_for_values(
            left_u,
            left_v,
            right_u,
            right_v,
            raw_disparity_px,
            raw_range_m,
            self.u_edges,
            self.v_edges,
            self.depth_edges,
            self.bin_axis,
        )

    def _nearest_cell(self, key: str) -> tuple[str | None, float | None]:
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
        *,
        raw_disparity_px: float | None = None,
        raw_range_m: float | None = None,
    ) -> ReprojectionLookupResult:
        if any(not _finite(value) for value in (left_u, left_v, right_u, right_v)):
            return self._noop(left_u, left_v, right_u, right_v, FALLBACK_INVALID, "incomplete_bbox_pair")
        disparity = abs(float(left_u) - float(right_u)) if raw_disparity_px is None else float(raw_disparity_px)
        if not _finite(disparity) or disparity < self.min_disparity_px:
            return self._noop(left_u, left_v, right_u, right_v, FALLBACK_INVALID, "bad_disparity")
        range_value = None if raw_range_m is None else float(raw_range_m)
        if self.bin_axis == "range" and (range_value is None or not _finite(range_value)):
            return self._noop(left_u, left_v, right_u, right_v, FALLBACK_INVALID, "missing_raw_range")
        key = self._cell_key(left_u, left_v, right_u, right_v, disparity, range_value)
        if key is None:
            return self._noop(left_u, left_v, right_u, right_v, FALLBACK_NO_CELL, "outside_profile_range")
        cell = self.cells.get(key)
        fallback_level = FALLBACK_EXACT
        fallback_reason = "exact"
        cell_key = key
        if cell is None:
            nearest_key, _distance = self._nearest_cell(key)
            if nearest_key is None:
                return self._noop(left_u, left_v, right_u, right_v, FALLBACK_NO_CELL, "missing_cell", cell_key=key)
            cell = self.cells[nearest_key]
            cell_key = nearest_key
            fallback_level = FALLBACK_NEAREST
            fallback_reason = "nearest_cell"
        du_left, dv_left, du_right, dv_right = self._effective_residuals(
            du_left=float(cell.get("du_left", 0.0)),
            dv_left=float(cell.get("dv_left", 0.0)),
            du_right=float(cell.get("du_right", 0.0)),
            dv_right=float(cell.get("dv_right", 0.0)),
        )
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
            cell_key=cell_key,
            sample_count=int(float(cell.get("sample_count", 0))),
        )

    def _noop(
        self,
        left_u: float,
        left_v: float,
        right_u: float,
        right_v: float,
        level: int,
        reason: str,
        *,
        cell_key: str = "",
    ) -> ReprojectionLookupResult:
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
            cell_key=cell_key,
            sample_count=0,
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "profile_id": self.profile_id,
            "method": self.method,
            "mode": self.mode,
            "feature_contract": "avg-screen-grid(left/right bbox center u/v) + raw_disparity_px or raw_range_m",
            "label_contract": "duL,dvL,duR,dvR = GT reprojection pixel - YOLO bbox center pixel",
            "image_width_px": self.image_width_px,
            "image_height_px": self.image_height_px,
            "bin_axis": self.bin_axis,
            "min_disparity_px": self.min_disparity_px,
            "lookup_policy": self.lookup_policy,
            "max_nearest_cell_distance": self.max_nearest_cell_distance,
            "correction_mode": self.correction_mode,
            "disparity_gain": self.disparity_gain,
            "screen_u_gain": self.screen_u_gain,
            "screen_v_gain": self.screen_v_gain,
            "output_policy": self.output_policy,
            "created_unix_s": self.created_unix_s,
            "calibration_envelope": self.calibration_envelope,
            "grid": {
                "u_edges": self.u_edges,
                "v_edges": self.v_edges,
                "depth_edges": self.depth_edges,
                "disparity_edges": self.depth_edges if self.bin_axis == "disparity" else [],
                "range_edges_m": self.depth_edges if self.bin_axis == "range" else [],
            },
            "cells": self.cells,
        }


def calibration_envelope_from_intrinsics(
    intrinsics: StereoIntrinsicsResult | None,
    *,
    intrinsics_path: str | Path | None = None,
    profile_source: str = "runtime-active-intrinsics",
) -> dict[str, Any]:
    if intrinsics is None:
        return {
            "schema": "reprojection-profile-calibration-envelope-v1",
            "profile_source": profile_source,
            "intrinsics_available": False,
            "warnings": ["no active intrinsics attached to reprojection profile"],
        }
    left_coverage = dict(intrinsics.left.coverage_summary or {})
    right_coverage = dict(intrinsics.right.coverage_summary or {})
    stereo = dict(intrinsics.stereo_calibration or {})
    warnings: list[str] = []
    for side, coverage in (("left", left_coverage), ("right", right_coverage)):
        for warning in coverage.get("warnings", []) or []:
            warnings.append(f"{side}: {warning}")
    if not stereo:
        warnings.append("stereo_calibration missing")
    elif stereo.get("epipolar_mean_error_px") is None:
        warnings.append("stereo_calibration epipolar residual missing")
    return {
        "schema": "reprojection-profile-calibration-envelope-v1",
        "profile_source": profile_source,
        "intrinsics_available": True,
        "intrinsics_path": "" if intrinsics_path is None else str(intrinsics_path),
        "left": {
            "image_size": {"width": int(intrinsics.left.width), "height": int(intrinsics.left.height)},
            "image_count": int(intrinsics.left.image_count),
            "reprojection_error_px": float(intrinsics.left.reprojection_error_px),
            "coverage_summary": left_coverage,
        },
        "right": {
            "image_size": {"width": int(intrinsics.right.width), "height": int(intrinsics.right.height)},
            "image_count": int(intrinsics.right.image_count),
            "reprojection_error_px": float(intrinsics.right.reprojection_error_px),
            "coverage_summary": right_coverage,
        },
        "stereo_calibration": {
            "available": bool(stereo),
            "method": stereo.get("method"),
            "pair_count": stereo.get("pair_count"),
            "rms_error_px": stereo.get("rms_error_px"),
            "epipolar_mean_error_px": stereo.get("epipolar_mean_error_px"),
            "epipolar_max_error_px": stereo.get("epipolar_max_error_px"),
        },
        "warnings": warnings,
    }


def load_reprojection_profile(path: Path) -> OriginBBoxReprojectionProfile:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("profile JSON root must be an object")
    return OriginBBoxReprojectionProfile.from_dict(payload)


def save_reprojection_profile(path: Path, profile: OriginBBoxReprojectionProfile) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(profile.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
