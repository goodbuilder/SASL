from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence


DEFAULT_RANGE_EDGES_M = (0.0, 500.0, 800.0, 1100.0, 1400.0, 2000.0)
DEFAULT_RAW_DOWN_EDGES_M = (-10.0, -2.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 10.0)
DEFAULT_CENTER_DELTA_DOWN_EDGES_M = (-5.0, 0.0, 0.25, 0.4, 0.6, 0.8, 1.2, 5.0)

RDF_AXES = ("right", "down", "forward")
STEREODISTANCE_TOPIC_AXES = ("forward", "right", "down", "range")
AXIS_CONTRACT = (
    "internal_policy_vector=RDF(right,down,forward); "
    "legacy_stereodistance_topics=FRD_range(forward,right,down,range)"
)


def _finite(value: object) -> bool:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number)


def _bin_index(value: float, edges: Sequence[float]) -> Optional[int]:
    if len(edges) < 2 or not _finite(value):
        return None
    number = float(value)
    for index in range(len(edges) - 1):
        lo = float(edges[index])
        hi = float(edges[index + 1])
        if lo <= number < hi or (index == len(edges) - 2 and number <= hi):
            return index
    return None


def _identity_matrix() -> tuple[tuple[float, float, float], ...]:
    return (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )


def _matrix_from_json(value: object) -> tuple[tuple[float, float, float], ...]:
    if not isinstance(value, list) or len(value) != 3:
        return _identity_matrix()
    rows: list[tuple[float, float, float]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 3:
            return _identity_matrix()
        rows.append((float(row[0]), float(row[1]), float(row[2])))
    return tuple(rows)


def _apply_matrix(
    matrix: tuple[tuple[float, float, float], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def _interp_piecewise_linear(value: float, x_values: Sequence[float], y_values: Sequence[float]) -> float:
    if len(x_values) == 0 or len(x_values) != len(y_values):
        return 0.0
    pairs = sorted((float(x), float(y)) for x, y in zip(x_values, y_values))
    if len(pairs) == 1:
        return pairs[0][1]
    number = float(value)
    if number <= pairs[0][0]:
        return pairs[0][1]
    if number >= pairs[-1][0]:
        return pairs[-1][1]
    for left, right in zip(pairs[:-1], pairs[1:]):
        x0, y0 = left
        x1, y1 = right
        if x0 <= number <= x1:
            width = x1 - x0
            if abs(width) <= 1.0e-12:
                return y0
            alpha = (number - x0) / width
            return y0 + alpha * (y1 - y0)
    return pairs[-1][1]


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: tuple[float, float, float], value: float) -> tuple[float, float, float]:
    return (a[0] * value, a[1] * value, a[2] * value)


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(_dot(vector, vector))
    if length <= 0.0:
        raise ValueError("zero-length vector")
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _candidate_source_and_gain(candidate: str) -> tuple[str, float]:
    if candidate == "raw" or not candidate:
        return "raw", 0.0
    for prefix, source in (("anchor_gain_", "anchor"), ("center_gain_", "center")):
        if candidate.startswith(prefix):
            return source, float(candidate.removeprefix(prefix))
    raise ValueError(f"unsupported candidate name: {candidate}")


@dataclass(frozen=True)
class RuntimeCameraModel:
    width_px: float
    height_px: float
    horizontal_fov_rad: float
    baseline_m: float

    @property
    def fx_px(self) -> float:
        return (float(self.width_px) * 0.5) / math.tan(float(self.horizontal_fov_rad) * 0.5)

    @property
    def cx_px(self) -> float:
        return float(self.width_px) * 0.5

    @property
    def cy_px(self) -> float:
        return float(self.height_px) * 0.5


@dataclass(frozen=True)
class GuardPolicyResult:
    corrected: Optional[dict[str, float]]
    status: dict[str, object]


class EstimatedToTruthSim3Profile:
    mode = "estimated_to_truth_sim3"

    def __init__(
        self,
        *,
        profile_id: str,
        scale: float,
        rotation_matrix_rdf: Sequence[Sequence[float]],
        translation_rdf_m: Sequence[float],
        forward_residual_disparity_coefficients: Sequence[float] = (),
        raw_rdf_ray_disparity_correction_coefficients: Sequence[float] = (),
        raw_rdf_ray_inverse_depth_correction_coefficients: Sequence[float] = (),
        raw_rdf_ray_disparity_forward_product_px_m: float | None = None,
        forward_residual_disparity_suppression_band: Mapping[str, object] | None = None,
        forward_residual_clamp_max_abs_m: float | None = None,
        forward_residual_disparity_lut: Mapping[str, object] | None = None,
        forward_residual_raw_forward_lut: Mapping[str, object] | None = None,
        forward_residual_raw_forward_query: str = "raw_forward",
        forward_residual_agreement_gate: Mapping[str, object] | None = None,
        rdf_residual_disparity_coefficients: Mapping[str, Sequence[float]] | None = None,
        rdf_residual_disparity_lut: Mapping[str, Mapping[str, object]] | None = None,
        rdf_residual_raw_rdf_polynomial: Mapping[str, object] | None = None,
        piecewise_sim3_raw_forward: Mapping[str, object] | None = None,
        piecewise_sim3_blend_alpha: float = 1.0,
        left_disparity_triangulation_model: Mapping[str, object] | None = None,
        target_disparity_blend_guard: Mapping[str, object] | None = None,
        target_range_away_blend_guard: Mapping[str, object] | None = None,
        target_disparity_range_delta_cap_guard: Mapping[str, object] | None = None,
    ) -> None:
        self.profile_id = str(profile_id or self.mode)
        self.profile_id_hash = int(hashlib.sha1(self.profile_id.encode("utf-8")).hexdigest()[:7], 16)
        self.scale = float(scale)
        self.rotation_matrix_rdf = _matrix_from_json([list(row) for row in rotation_matrix_rdf])
        if len(translation_rdf_m) != 3:
            raise ValueError("translation_rdf_m must contain three RDF values")
        self.translation_rdf_m = tuple(float(value) for value in translation_rdf_m)
        self.forward_residual_disparity_coefficients = tuple(
            float(value) for value in forward_residual_disparity_coefficients
        )
        self.raw_rdf_ray_disparity_correction_coefficients = tuple(
            float(value) for value in raw_rdf_ray_disparity_correction_coefficients
        )
        self.raw_rdf_ray_inverse_depth_correction_coefficients = tuple(
            float(value) for value in raw_rdf_ray_inverse_depth_correction_coefficients
        )
        self.raw_rdf_ray_disparity_forward_product_px_m = (
            None
            if raw_rdf_ray_disparity_forward_product_px_m is None
            else float(raw_rdf_ray_disparity_forward_product_px_m)
        )
        self.forward_residual_disparity_suppression_band = self._parse_suppression_band(
            forward_residual_disparity_suppression_band
        )
        self.forward_residual_clamp_max_abs_m = self._parse_forward_residual_clamp(
            forward_residual_clamp_max_abs_m
        )
        self.forward_residual_disparity_lut = self._parse_forward_lut(forward_residual_disparity_lut)
        self.forward_residual_raw_forward_lut = self._parse_forward_lut(
            forward_residual_raw_forward_lut,
            x_key="x_values_m",
        )
        if forward_residual_raw_forward_query not in {"raw_forward", "base_forward"}:
            raise ValueError("forward_residual_raw_forward_query must be raw_forward or base_forward")
        self.forward_residual_raw_forward_query = str(forward_residual_raw_forward_query)
        self.forward_residual_agreement_gate = self._parse_agreement_gate(forward_residual_agreement_gate)
        residual_by_axis = rdf_residual_disparity_coefficients or {}
        self.rdf_residual_disparity_coefficients = {
            axis: tuple(float(value) for value in residual_by_axis.get(axis, ()))
            for axis in RDF_AXES
        }
        self.rdf_residual_disparity_lut = self._parse_rdf_lut(rdf_residual_disparity_lut)
        self.rdf_residual_raw_rdf_polynomial = self._parse_raw_rdf_polynomial(
            rdf_residual_raw_rdf_polynomial
        )
        self.piecewise_sim3_raw_forward = self._parse_piecewise_sim3_raw_forward(piecewise_sim3_raw_forward)
        self.piecewise_sim3_blend_alpha = min(1.0, max(0.0, float(piecewise_sim3_blend_alpha)))
        self.left_disparity_triangulation_model = self._parse_left_disparity_triangulation_model(
            left_disparity_triangulation_model
        )
        self.target_disparity_blend_guard = self._parse_target_disparity_blend_guard(
            target_disparity_blend_guard
        )
        self.target_range_away_blend_guard = self._parse_target_range_away_blend_guard(
            target_range_away_blend_guard
        )
        self.target_disparity_range_delta_cap_guard = self._parse_target_disparity_range_delta_cap_guard(
            target_disparity_range_delta_cap_guard
        )

    @classmethod
    def from_json_dict(cls, data: Mapping[str, object]):
        if data.get("mode") != cls.mode:
            raise ValueError(f"unsupported mode: {data.get('mode')!r}")
        rotation = data.get("rotation_matrix_rdf")
        if rotation is None:
            raise ValueError("estimated_to_truth_sim3 requires rotation_matrix_rdf")
        translation = data.get("translation_rdf_m")
        if not isinstance(translation, list):
            raise ValueError("estimated_to_truth_sim3 requires translation_rdf_m")
        residual = data.get("forward_residual_disparity_coefficients_high_to_low", [])
        if residual is None:
            residual = []
        if not isinstance(residual, list):
            raise ValueError("forward residual coefficients must be a list")
        ray_disparity_correction = data.get("raw_rdf_ray_disparity_correction_coefficients_high_to_low", [])
        if ray_disparity_correction is None:
            ray_disparity_correction = []
        if not isinstance(ray_disparity_correction, list):
            raise ValueError("raw RDF ray disparity correction coefficients must be a list")
        ray_inverse_depth_correction = data.get("raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low", [])
        if ray_inverse_depth_correction is None:
            ray_inverse_depth_correction = []
        if not isinstance(ray_inverse_depth_correction, list):
            raise ValueError("raw RDF ray inverse-depth correction coefficients must be a list")
        ray_disparity_forward_product = data.get("raw_rdf_ray_disparity_forward_product_px_m")
        if ray_disparity_forward_product is not None:
            ray_disparity_forward_product = float(ray_disparity_forward_product)
            if not _finite(ray_disparity_forward_product) or ray_disparity_forward_product <= 0.0:
                raise ValueError("raw_rdf_ray_disparity_forward_product_px_m must be positive and finite")
        suppression_band = data.get("forward_residual_disparity_suppression_band")
        if suppression_band is not None and not isinstance(suppression_band, Mapping):
            raise ValueError("forward residual suppression band must be an object")
        forward_residual_clamp = data.get("forward_residual_clamp_max_abs_m")
        forward_lut = data.get("forward_residual_disparity_lut")
        if forward_lut is not None and not isinstance(forward_lut, Mapping):
            raise ValueError("forward residual LUT must be an object")
        raw_forward_lut = data.get("forward_residual_raw_forward_lut")
        if raw_forward_lut is not None and not isinstance(raw_forward_lut, Mapping):
            raise ValueError("forward residual raw-forward LUT must be an object")
        raw_forward_query = str(data.get("forward_residual_raw_forward_query", "raw_forward") or "raw_forward")
        if raw_forward_query not in {"raw_forward", "base_forward"}:
            raise ValueError("forward_residual_raw_forward_query must be raw_forward or base_forward")
        agreement_gate = data.get("forward_residual_agreement_gate")
        if agreement_gate is not None and not isinstance(agreement_gate, Mapping):
            raise ValueError("forward residual agreement gate must be an object")
        rdf_residual = data.get("rdf_residual_disparity_coefficients_high_to_low", {})
        if rdf_residual is None:
            rdf_residual = {}
        if not isinstance(rdf_residual, Mapping):
            raise ValueError("rdf residual coefficients must be an object keyed by right/down/forward")
        for axis, coefficients in rdf_residual.items():
            if axis not in RDF_AXES:
                raise ValueError(f"unsupported RDF residual axis: {axis!r}")
            if not isinstance(coefficients, list):
                raise ValueError(f"RDF residual coefficients for {axis!r} must be a list")
        rdf_lut = data.get("rdf_residual_disparity_lut", {})
        if rdf_lut is None:
            rdf_lut = {}
        if not isinstance(rdf_lut, Mapping):
            raise ValueError("RDF residual LUT must be an object keyed by right/down/forward")
        for axis, lut in rdf_lut.items():
            if axis not in RDF_AXES:
                raise ValueError(f"unsupported RDF residual LUT axis: {axis!r}")
            if not isinstance(lut, Mapping):
                raise ValueError(f"RDF residual LUT for {axis!r} must be an object")
        raw_rdf_poly = data.get("rdf_residual_raw_rdf_polynomial")
        if raw_rdf_poly is not None and not isinstance(raw_rdf_poly, Mapping):
            raise ValueError("raw RDF residual polynomial must be an object")
        piecewise_sim3 = data.get("piecewise_sim3_raw_forward")
        if piecewise_sim3 is not None and not isinstance(piecewise_sim3, Mapping):
            raise ValueError("piecewise Sim(3) raw-forward model must be an object")
        piecewise_alpha = float(data.get("piecewise_sim3_blend_alpha", 1.0) or 0.0)
        left_disparity_model = data.get("left_disparity_triangulation_model")
        if left_disparity_model is not None and not isinstance(left_disparity_model, Mapping):
            raise ValueError("left disparity triangulation model must be an object")
        target_disparity_blend_guard = data.get("target_disparity_blend_guard")
        if target_disparity_blend_guard is not None and not isinstance(target_disparity_blend_guard, Mapping):
            raise ValueError("target disparity blend guard must be an object")
        target_range_away_blend_guard = data.get("target_range_away_blend_guard")
        if target_range_away_blend_guard is not None and not isinstance(target_range_away_blend_guard, Mapping):
            raise ValueError("target range away blend guard must be an object")
        target_disparity_range_delta_cap_guard = data.get("target_disparity_range_delta_cap_guard")
        if (
            target_disparity_range_delta_cap_guard is not None
            and not isinstance(target_disparity_range_delta_cap_guard, Mapping)
        ):
            raise ValueError("target disparity range-delta cap guard must be an object")
        return cls(
            profile_id=str(data.get("profile_id") or cls.mode),
            scale=float(data.get("scale", 1.0)),
            rotation_matrix_rdf=rotation,
            translation_rdf_m=translation,
            forward_residual_disparity_coefficients=residual,
            raw_rdf_ray_disparity_correction_coefficients=ray_disparity_correction,
            raw_rdf_ray_inverse_depth_correction_coefficients=ray_inverse_depth_correction,
            raw_rdf_ray_disparity_forward_product_px_m=ray_disparity_forward_product,
            forward_residual_disparity_suppression_band=suppression_band,
            forward_residual_clamp_max_abs_m=(
                None if forward_residual_clamp is None else float(forward_residual_clamp)
            ),
            forward_residual_disparity_lut=forward_lut,
            forward_residual_raw_forward_lut=raw_forward_lut,
            forward_residual_raw_forward_query=raw_forward_query,
            forward_residual_agreement_gate=agreement_gate,
            rdf_residual_disparity_coefficients=rdf_residual,
            rdf_residual_disparity_lut=rdf_lut,
            rdf_residual_raw_rdf_polynomial=raw_rdf_poly,
            piecewise_sim3_raw_forward=piecewise_sim3,
            piecewise_sim3_blend_alpha=piecewise_alpha,
            left_disparity_triangulation_model=left_disparity_model,
            target_disparity_blend_guard=target_disparity_blend_guard,
            target_range_away_blend_guard=target_range_away_blend_guard,
            target_disparity_range_delta_cap_guard=target_disparity_range_delta_cap_guard,
        )

    @staticmethod
    def _parse_suppression_band(band: Mapping[str, object] | None) -> dict[str, float] | None:
        if band is None:
            return None
        center = float(band.get("center_disparity_px", band.get("center_px", 0.0)))
        half_width = float(band.get("half_width_px", 0.0))
        alpha_inside = float(band.get("alpha_inside", 1.0))
        alpha_outside = float(band.get("alpha_outside", 1.0))
        if not all(_finite(value) for value in (center, half_width, alpha_inside, alpha_outside)):
            raise ValueError("forward residual suppression band values must be finite")
        if half_width < 0.0:
            raise ValueError("forward residual suppression band half_width_px must be non-negative")
        return {
            "center_disparity_px": center,
            "half_width_px": half_width,
            "alpha_inside": alpha_inside,
            "alpha_outside": alpha_outside,
        }

    @staticmethod
    def _parse_forward_residual_clamp(value: float | None) -> float | None:
        if value is None:
            return None
        parsed = float(value)
        if not _finite(parsed) or parsed < 0.0:
            raise ValueError("forward residual clamp max_abs_m must be finite and non-negative")
        return parsed

    @staticmethod
    def _parse_target_disparity_blend_guard(
        guard: Mapping[str, object] | None,
    ) -> dict[str, float] | None:
        if guard is None:
            return None
        target = float(guard.get("target_disparity_px", guard.get("center_disparity_px", 0.0)))
        half_width = float(guard.get("half_width_px", 0.0))
        alpha_inside = float(guard.get("alpha_inside", guard.get("blend_alpha", 1.0)))
        alpha_outside = float(guard.get("alpha_outside", 1.0))
        if not all(_finite(value) for value in (target, half_width, alpha_inside, alpha_outside)):
            raise ValueError("target disparity blend guard values must be finite")
        if target <= 0.0:
            raise ValueError("target disparity blend guard target_disparity_px must be positive")
        if half_width < 0.0:
            raise ValueError("target disparity blend guard half_width_px must be non-negative")
        if not 0.0 <= alpha_inside <= 1.0 or not 0.0 <= alpha_outside <= 1.0:
            raise ValueError("target disparity blend guard alpha values must be in [0, 1]")
        return {
            "target_disparity_px": target,
            "half_width_px": half_width,
            "alpha_inside": alpha_inside,
            "alpha_outside": alpha_outside,
        }

    @staticmethod
    def _parse_target_range_away_blend_guard(
        guard: Mapping[str, object] | None,
    ) -> dict[str, float] | None:
        if guard is None:
            return None
        target_range = float(guard.get("target_range_m", 0.0))
        window = float(guard.get("window_m", guard.get("half_width_m", 0.0)))
        alpha_away = float(guard.get("alpha_away", guard.get("blend_alpha", 1.0)))
        if not all(_finite(value) for value in (target_range, window, alpha_away)):
            raise ValueError("target range away blend guard values must be finite")
        if target_range <= 0.0:
            raise ValueError("target range away blend guard target_range_m must be positive")
        if window < 0.0:
            raise ValueError("target range away blend guard window_m must be non-negative")
        if not 0.0 <= alpha_away <= 1.0:
            raise ValueError("target range away blend guard alpha_away must be in [0, 1]")
        return {
            "target_range_m": target_range,
            "window_m": window,
            "alpha_away": alpha_away,
        }

    @staticmethod
    def _parse_target_disparity_range_delta_cap_guard(
        guard: Mapping[str, object] | None,
    ) -> dict[str, float] | None:
        if guard is None:
            return None
        target = float(guard.get("target_disparity_px", guard.get("center_disparity_px", 0.0)))
        half_width = float(guard.get("half_width_px", 0.0))
        cap = float(guard.get("range_delta_cap_m", guard.get("cap_m", 0.0)))
        if not all(_finite(value) for value in (target, half_width, cap)):
            raise ValueError("target disparity range-delta cap guard values must be finite")
        if target <= 0.0:
            raise ValueError("target disparity range-delta cap guard target_disparity_px must be positive")
        if half_width < 0.0:
            raise ValueError("target disparity range-delta cap guard half_width_px must be non-negative")
        if cap < 0.0:
            raise ValueError("target disparity range-delta cap guard cap must be non-negative")
        return {
            "target_disparity_px": target,
            "half_width_px": half_width,
            "range_delta_cap_m": cap,
        }

    @staticmethod
    def _parse_forward_lut(
        lut: Mapping[str, object] | None,
        *,
        x_key: str = "x_values_px",
    ) -> dict[str, tuple[float, ...]]:
        if lut is None:
            return {"x_values_px": (), "residual_values_m": ()}
        x_values = lut.get(x_key, lut.get("x_values_px", []))
        y_values = lut.get("residual_values_m", [])
        if not isinstance(x_values, list) or not isinstance(y_values, list):
            raise ValueError(f"forward residual LUT requires {x_key} and residual_values_m lists")
        if len(x_values) != len(y_values):
            raise ValueError("forward residual LUT x/y lengths must match")
        x_tuple = tuple(float(value) for value in x_values)
        y_tuple = tuple(float(value) for value in y_values)
        if not all(_finite(value) for value in (*x_tuple, *y_tuple)):
            raise ValueError("forward residual LUT values must be finite")
        return {"x_values_px": x_tuple, "residual_values_m": y_tuple}

    @staticmethod
    def _parse_agreement_gate(gate: Mapping[str, object] | None) -> dict[str, float] | None:
        if gate is None:
            return None
        max_difference = float(gate.get("max_abs_difference_m", gate.get("max_difference_m", 0.0)))
        alpha_disagree = float(gate.get("alpha_disagree", 1.0))
        if not all(_finite(value) for value in (max_difference, alpha_disagree)):
            raise ValueError("forward residual agreement gate values must be finite")
        if max_difference < 0.0:
            raise ValueError("forward residual agreement gate max_abs_difference_m must be non-negative")
        alpha_disagree = min(1.0, max(0.0, alpha_disagree))
        return {
            "max_abs_difference_m": max_difference,
            "alpha_disagree": alpha_disagree,
        }

    @classmethod
    def _parse_rdf_lut(
        cls,
        lut_by_axis: Mapping[str, Mapping[str, object]] | None,
    ) -> dict[str, dict[str, tuple[float, ...]]]:
        lut_by_axis = lut_by_axis or {}
        return {
            axis: cls._parse_forward_lut(lut_by_axis.get(axis))
            for axis in RDF_AXES
        }

    @staticmethod
    def _parse_raw_rdf_polynomial(model: Mapping[str, object] | None) -> dict[str, object] | None:
        if model is None:
            return None
        degree = int(model.get("degree", 1))
        if degree not in (1, 2):
            raise ValueError("raw RDF residual polynomial degree must be 1 or 2")
        feature_names = model.get("feature_names", [])
        coefficients = model.get("coefficient_matrix_m", [])
        if not isinstance(feature_names, list) or not isinstance(coefficients, list):
            raise ValueError("raw RDF residual polynomial requires feature_names and coefficient_matrix_m lists")
        coefficient_rows: list[tuple[float, float, float]] = []
        for row in coefficients:
            if not isinstance(row, list) or len(row) != 3:
                raise ValueError("raw RDF residual coefficient rows must contain right/down/forward values")
            values = tuple(float(value) for value in row)
            if not all(_finite(value) for value in values):
                raise ValueError("raw RDF residual coefficients must be finite")
            coefficient_rows.append(values)
        if len(feature_names) != len(coefficient_rows):
            raise ValueError("raw RDF residual feature/coefficient lengths must match")
        forward_center = float(model.get("forward_center_m", 0.0))
        forward_scale = float(model.get("forward_scale_m", 1.0))
        if not _finite(forward_center) or not _finite(forward_scale) or abs(forward_scale) <= 1.0e-12:
            raise ValueError("raw RDF residual forward normalization values must be finite and nonzero")
        cap = model.get("residual_norm_cap_m")
        parsed_cap = None if cap is None else max(0.0, float(cap))
        if parsed_cap is not None and not _finite(parsed_cap):
            raise ValueError("raw RDF residual cap must be finite")
        return {
            "degree": degree,
            "feature_names": tuple(str(name) for name in feature_names),
            "forward_center_m": forward_center,
            "forward_scale_m": forward_scale,
            "coefficient_matrix_m": tuple(coefficient_rows),
            "residual_norm_cap_m": parsed_cap,
        }

    @staticmethod
    def _parse_piecewise_sim3_raw_forward(model: Mapping[str, object] | None) -> dict[str, object] | None:
        if model is None:
            return None
        x_values = model.get("x_values_m", [])
        transforms = model.get("transforms", [])
        if not isinstance(x_values, list) or not isinstance(transforms, list):
            raise ValueError("piecewise Sim(3) raw-forward model requires x_values_m and transforms lists")
        if len(x_values) != len(transforms):
            raise ValueError("piecewise Sim(3) raw-forward x/transforms lengths must match")
        if len(x_values) == 0:
            return {"x_values_m": (), "transforms": ()}
        parsed_x = tuple(float(value) for value in x_values)
        if not all(_finite(value) for value in parsed_x):
            raise ValueError("piecewise Sim(3) raw-forward x_values_m must be finite")
        parsed_transforms: list[dict[str, object]] = []
        for transform in transforms:
            if not isinstance(transform, Mapping):
                raise ValueError("piecewise Sim(3) transform entries must be objects")
            rotation = transform.get("rotation_matrix_rdf")
            translation = transform.get("translation_rdf_m")
            if rotation is None or not isinstance(translation, list) or len(translation) != 3:
                raise ValueError("piecewise Sim(3) transform requires rotation_matrix_rdf and translation_rdf_m")
            scale = float(transform.get("scale", 1.0))
            translation_tuple = tuple(float(value) for value in translation)
            if not _finite(scale) or not all(_finite(value) for value in translation_tuple):
                raise ValueError("piecewise Sim(3) transform scale/translation values must be finite")
            parsed_transforms.append(
                {
                    "scale": scale,
                    "rotation_matrix_rdf": _matrix_from_json(rotation),
                    "translation_rdf_m": translation_tuple,
                    "sample_count": int(transform.get("sample_count", 0) or 0),
                }
            )
        order = sorted(range(len(parsed_x)), key=lambda index: parsed_x[index])
        return {
            "x_values_m": tuple(parsed_x[index] for index in order),
            "transforms": tuple(parsed_transforms[index] for index in order),
        }

    @staticmethod
    def _parse_left_disparity_triangulation_model(model: Mapping[str, object] | None) -> dict[str, object] | None:
        if model is None:
            return None
        required = ("fx_px", "fy_px", "cx_px", "cy_px", "baseline_m")
        parsed = {name: float(model.get(name, math.nan)) for name in required}
        if not all(_finite(value) for value in parsed.values()):
            raise ValueError("left disparity triangulation model requires finite fx/fy/cx/cy/baseline")
        if parsed["fx_px"] <= 0.0 or parsed["fy_px"] <= 0.0 or parsed["baseline_m"] <= 0.0:
            raise ValueError("left disparity triangulation model fx/fy/baseline must be positive")
        coefficients = model.get("disparity_correction_coefficients_high_to_low", [1.0, 0.0])
        if coefficients is None:
            coefficients = [1.0, 0.0]
        if not isinstance(coefficients, list) or len(coefficients) == 0:
            raise ValueError("left disparity triangulation model requires disparity coefficients list")
        coefficient_tuple = tuple(float(value) for value in coefficients)
        if not all(_finite(value) for value in coefficient_tuple):
            raise ValueError("left disparity triangulation disparity coefficients must be finite")
        parsed["disparity_correction_coefficients_high_to_low"] = coefficient_tuple
        parsed["left_u_offset_px"] = float(model.get("left_u_offset_px", 0.0) or 0.0)
        parsed["left_v_offset_px"] = float(model.get("left_v_offset_px", 0.0) or 0.0)
        if not _finite(parsed["left_u_offset_px"]) or not _finite(parsed["left_v_offset_px"]):
            raise ValueError("left disparity triangulation offsets must be finite")
        return parsed

    def _triangulate_left_disparity_raw_point(
        self,
        raw: Mapping[str, object],
    ) -> tuple[tuple[float, float, float], float] | None:
        model = self.left_disparity_triangulation_model
        if model is None:
            return None
        required = ("left_u_px", "left_v_px", "disparity_px")
        if any(raw.get(name) is None for name in required):
            return None
        left_u = float(raw["left_u_px"]) + float(model["left_u_offset_px"])
        left_v = float(raw["left_v_px"]) + float(model["left_v_offset_px"])
        measured_disparity = float(raw["disparity_px"])
        if not all(_finite(value) for value in (left_u, left_v, measured_disparity)):
            return None
        corrected_disparity = 0.0
        coefficients = model["disparity_correction_coefficients_high_to_low"]
        assert isinstance(coefficients, tuple)
        for coefficient in coefficients:
            corrected_disparity = corrected_disparity * measured_disparity + coefficient
        if not _finite(corrected_disparity) or corrected_disparity <= 1.0e-9:
            return None
        fx = float(model["fx_px"])
        fy = float(model["fy_px"])
        cx = float(model["cx_px"])
        cy = float(model["cy_px"])
        baseline = float(model["baseline_m"])
        forward = fx * baseline / corrected_disparity
        right = (left_u - cx) * forward / fx - baseline * 0.5
        down = (left_v - cy) * forward / fy
        raw_point = (right, down, forward)
        if not all(_finite(value) for value in raw_point):
            return None
        return raw_point, corrected_disparity

    @staticmethod
    def _apply_sim3_transform(transform: Mapping[str, object], raw_point: tuple[float, float, float]) -> tuple[float, float, float]:
        matrix = transform["rotation_matrix_rdf"]
        scale = float(transform["scale"])
        translation = transform["translation_rdf_m"]
        assert isinstance(matrix, tuple)
        assert isinstance(translation, tuple)
        rotated = _apply_matrix(matrix, raw_point)
        return _add(_scale(rotated, scale), translation)

    def _apply_global_sim3(self, raw_point: tuple[float, float, float]) -> tuple[float, float, float]:
        rotated = _apply_matrix(self.rotation_matrix_rdf, raw_point)
        return _add(_scale(rotated, self.scale), self.translation_rdf_m)

    def _apply_piecewise_sim3_raw_forward(self, raw_point: tuple[float, float, float]) -> tuple[float, float, float] | None:
        model = self.piecewise_sim3_raw_forward
        if model is None:
            return None
        x_values = model["x_values_m"]
        transforms = model["transforms"]
        assert isinstance(x_values, tuple)
        assert isinstance(transforms, tuple)
        if not x_values:
            return None
        raw_forward = raw_point[2]
        if len(x_values) == 1 or raw_forward <= x_values[0]:
            transform = transforms[0]
            assert isinstance(transform, Mapping)
            return self._apply_sim3_transform(transform, raw_point)
        if raw_forward >= x_values[-1]:
            transform = transforms[-1]
            assert isinstance(transform, Mapping)
            return self._apply_sim3_transform(transform, raw_point)
        for index in range(len(x_values) - 1):
            left_x = float(x_values[index])
            right_x = float(x_values[index + 1])
            if left_x <= raw_forward <= right_x:
                left_transform = transforms[index]
                right_transform = transforms[index + 1]
                assert isinstance(left_transform, Mapping)
                assert isinstance(right_transform, Mapping)
                width = max(1.0e-12, right_x - left_x)
                alpha = (raw_forward - left_x) / width
                left_point = self._apply_sim3_transform(left_transform, raw_point)
                right_point = self._apply_sim3_transform(right_transform, raw_point)
                return (
                    (1.0 - alpha) * left_point[0] + alpha * right_point[0],
                    (1.0 - alpha) * left_point[1] + alpha * right_point[1],
                    (1.0 - alpha) * left_point[2] + alpha * right_point[2],
                )
        transform = transforms[-1]
        assert isinstance(transform, Mapping)
        return self._apply_sim3_transform(transform, raw_point)

    @staticmethod
    def _raw_rdf_polynomial_features(
        raw_point: tuple[float, float, float],
        model: Mapping[str, object],
    ) -> tuple[float, ...]:
        right, down, forward = raw_point
        safe_forward = forward if abs(forward) > 1.0e-9 else 1.0e-9
        right_over_forward = right / safe_forward
        down_over_forward = down / safe_forward
        forward_norm = (forward - float(model["forward_center_m"])) / float(model["forward_scale_m"])
        values = [
            1.0,
            right_over_forward,
            down_over_forward,
            forward_norm,
        ]
        if int(model["degree"]) >= 2:
            values.extend(
                [
                    right_over_forward * right_over_forward,
                    right_over_forward * down_over_forward,
                    down_over_forward * down_over_forward,
                    right_over_forward * forward_norm,
                    down_over_forward * forward_norm,
                    forward_norm * forward_norm,
                ]
            )
        return tuple(values)

    def _forward_residual_alpha(self, disparity_px: float) -> float:
        band = self.forward_residual_disparity_suppression_band
        if not band:
            return 1.0
        if abs(float(disparity_px) - band["center_disparity_px"]) <= band["half_width_px"]:
            return band["alpha_inside"]
        return band["alpha_outside"]

    def apply(self, raw: Mapping[str, object]) -> GuardPolicyResult:
        required = ("right_m", "down_m", "forward_m", "disparity_px")
        if any(raw.get(name) is None for name in required):
            return GuardPolicyResult(None, self._status(3, "invalid_raw"))
        raw_point = (float(raw["right_m"]), float(raw["down_m"]), float(raw["forward_m"]))
        if not all(_finite(value) for value in raw_point):
            return GuardPolicyResult(None, self._status(3, "invalid_raw"))
        output_disparity = float(raw["disparity_px"])
        left_disparity_model_applied = False
        triangulated = self._triangulate_left_disparity_raw_point(raw)
        if self.left_disparity_triangulation_model is not None:
            if triangulated is None:
                return GuardPolicyResult(None, self._status(3, "invalid_left_disparity_triangulation"))
            raw_point, output_disparity = triangulated
            left_disparity_model_applied = True
        ray_disparity_forward_product = self.raw_rdf_ray_disparity_forward_product_px_m
        if ray_disparity_forward_product is not None:
            disparity = float(raw["disparity_px"])
            raw_forward = raw_point[2]
            if disparity <= 1.0e-9 or abs(raw_forward) <= 1.0e-9:
                return GuardPolicyResult(None, self._status(3, "invalid_disparity_forward_product_correction"))
            corrected_forward = ray_disparity_forward_product / disparity
            if not _finite(corrected_forward):
                return GuardPolicyResult(None, self._status(3, "invalid_disparity_forward_product_correction"))
            raw_point = _scale(raw_point, corrected_forward / raw_forward)
        ray_disparity_coefficients = self.raw_rdf_ray_disparity_correction_coefficients
        if ray_disparity_coefficients:
            disparity = float(raw["disparity_px"])
            corrected_disparity = 0.0
            for coefficient in ray_disparity_coefficients:
                corrected_disparity = corrected_disparity * disparity + coefficient
            if not _finite(corrected_disparity) or corrected_disparity <= 1.0e-9:
                return GuardPolicyResult(None, self._status(3, "invalid_disparity_correction"))
            ray_scale = disparity / corrected_disparity
            raw_point = _scale(raw_point, ray_scale)
        ray_inverse_depth_coefficients = self.raw_rdf_ray_inverse_depth_correction_coefficients
        if ray_inverse_depth_coefficients:
            raw_forward = raw_point[2]
            if raw_forward <= 1.0e-9:
                return GuardPolicyResult(None, self._status(3, "invalid_inverse_depth_correction"))
            inverse_depth = 1.0 / raw_forward
            corrected_inverse_depth = 0.0
            for coefficient in ray_inverse_depth_coefficients:
                corrected_inverse_depth = corrected_inverse_depth * inverse_depth + coefficient
            if not _finite(corrected_inverse_depth) or corrected_inverse_depth <= 1.0e-12:
                return GuardPolicyResult(None, self._status(3, "invalid_inverse_depth_correction"))
            corrected_forward = 1.0 / corrected_inverse_depth
            raw_point = _scale(raw_point, corrected_forward / raw_forward)
        target_disparity_blend_raw_point = raw_point
        piecewise_point = self._apply_piecewise_sim3_raw_forward(raw_point)
        if piecewise_point is None:
            corrected_point = self._apply_global_sim3(raw_point)
        else:
            alpha = self.piecewise_sim3_blend_alpha
            if alpha >= 1.0:
                corrected_point = piecewise_point
            elif alpha <= 0.0:
                corrected_point = self._apply_global_sim3(raw_point)
            else:
                global_point = self._apply_global_sim3(raw_point)
                corrected_point = (
                    (1.0 - alpha) * global_point[0] + alpha * piecewise_point[0],
                    (1.0 - alpha) * global_point[1] + alpha * piecewise_point[1],
                    (1.0 - alpha) * global_point[2] + alpha * piecewise_point[2],
                )
        residual_coefficients = self.forward_residual_disparity_coefficients
        residual_alpha = 1.0
        forward_residual_unclamped = 0.0
        forward_residual_applied = 0.0
        rdf_forward_residual_unclamped = 0.0
        rdf_forward_residual_applied = 0.0
        if residual_coefficients:
            disparity = float(raw["disparity_px"])
            residual = 0.0
            for coefficient in residual_coefficients:
                residual = residual * disparity + coefficient
            forward_residual_unclamped = residual
            if self.forward_residual_clamp_max_abs_m is not None:
                limit = self.forward_residual_clamp_max_abs_m
                residual = min(limit, max(-limit, residual))
            residual_alpha = self._forward_residual_alpha(disparity)
            forward_residual_applied = residual_alpha * residual
            corrected_point = (
                corrected_point[0],
                corrected_point[1],
                corrected_point[2] + forward_residual_applied,
            )
        lut_residuals: list[float] = []
        if self.forward_residual_disparity_lut["x_values_px"]:
            disparity = float(raw["disparity_px"])
            lut_residual = _interp_piecewise_linear(
                disparity,
                self.forward_residual_disparity_lut["x_values_px"],
                self.forward_residual_disparity_lut["residual_values_m"],
            )
            lut_residuals.append(lut_residual)
        if self.forward_residual_raw_forward_lut["x_values_px"]:
            raw_forward = (
                float(raw["forward_m"])
                if self.forward_residual_raw_forward_query == "raw_forward"
                else float(corrected_point[2])
            )
            lut_residual = _interp_piecewise_linear(
                raw_forward,
                self.forward_residual_raw_forward_lut["x_values_px"],
                self.forward_residual_raw_forward_lut["residual_values_m"],
            )
            lut_residuals.append(lut_residual)
        if lut_residuals:
            lut_alpha = 1.0
            gate = self.forward_residual_agreement_gate
            if gate is not None and len(lut_residuals) >= 2:
                if abs(lut_residuals[0] - lut_residuals[1]) > gate["max_abs_difference_m"]:
                    lut_alpha = gate["alpha_disagree"]
            corrected_point = (
                corrected_point[0],
                corrected_point[1],
                corrected_point[2] + lut_alpha * sum(lut_residuals),
            )
        if any(self.rdf_residual_disparity_coefficients.values()):
            disparity = float(raw["disparity_px"])
            residuals = []
            for axis in RDF_AXES:
                residual = 0.0
                for coefficient in self.rdf_residual_disparity_coefficients[axis]:
                    residual = residual * disparity + coefficient
                residuals.append(residual)
            rdf_forward_residual_unclamped = residuals[2]
            if self.forward_residual_clamp_max_abs_m is not None:
                limit = self.forward_residual_clamp_max_abs_m
                residuals[2] = min(limit, max(-limit, residuals[2]))
            rdf_forward_residual_applied = residuals[2]
            corrected_point = (
                corrected_point[0] + residuals[0],
                corrected_point[1] + residuals[1],
                corrected_point[2] + residuals[2],
            )
        if any(lut["x_values_px"] for lut in self.rdf_residual_disparity_lut.values()):
            disparity = float(raw["disparity_px"])
            residuals = [
                _interp_piecewise_linear(
                    disparity,
                    self.rdf_residual_disparity_lut[axis]["x_values_px"],
                    self.rdf_residual_disparity_lut[axis]["residual_values_m"],
                )
                for axis in RDF_AXES
            ]
            corrected_point = (
                corrected_point[0] + residuals[0],
                corrected_point[1] + residuals[1],
                corrected_point[2] + residuals[2],
            )
        if self.rdf_residual_raw_rdf_polynomial is not None:
            model = self.rdf_residual_raw_rdf_polynomial
            features = self._raw_rdf_polynomial_features(raw_point, model)
            coefficient_rows = model["coefficient_matrix_m"]
            if len(features) != len(coefficient_rows):
                return GuardPolicyResult(None, self._status(3, "invalid_raw_rdf_residual_polynomial"))
            residuals = [0.0, 0.0, 0.0]
            for feature, row in zip(features, coefficient_rows):
                residuals[0] += feature * row[0]
                residuals[1] += feature * row[1]
                residuals[2] += feature * row[2]
            cap = model.get("residual_norm_cap_m")
            if cap is not None:
                norm = math.sqrt(_dot(tuple(residuals), tuple(residuals)))
                if norm > 1.0e-12:
                    residual_scale = min(1.0, float(cap) / norm)
                    residuals = [value * residual_scale for value in residuals]
            corrected_point = (
                corrected_point[0] + residuals[0],
                corrected_point[1] + residuals[1],
                corrected_point[2] + residuals[2],
            )
        target_disparity_blend_guard_applied = False
        target_disparity_blend_alpha = 1.0
        if self.target_disparity_blend_guard is not None:
            guard = self.target_disparity_blend_guard
            measured_disparity = float(raw["disparity_px"])
            if not _finite(measured_disparity):
                return GuardPolicyResult(None, self._status(3, "invalid_target_disparity_blend_guard"))
            in_window = abs(measured_disparity - guard["target_disparity_px"]) <= guard["half_width_px"]
            target_disparity_blend_guard_applied = bool(in_window)
            target_disparity_blend_alpha = guard["alpha_inside"] if in_window else guard["alpha_outside"]
            if target_disparity_blend_alpha < 1.0:
                alpha = target_disparity_blend_alpha
                corrected_point = (
                    target_disparity_blend_raw_point[0] + alpha * (corrected_point[0] - target_disparity_blend_raw_point[0]),
                    target_disparity_blend_raw_point[1] + alpha * (corrected_point[1] - target_disparity_blend_raw_point[1]),
                    target_disparity_blend_raw_point[2] + alpha * (corrected_point[2] - target_disparity_blend_raw_point[2]),
                )
        target_range_away_blend_guard_applied = False
        target_range_away_blend_alpha = 1.0
        if self.target_range_away_blend_guard is not None:
            guard = self.target_range_away_blend_guard
            raw_range = math.sqrt(_dot(target_disparity_blend_raw_point, target_disparity_blend_raw_point))
            corrected_range_for_guard = math.sqrt(_dot(corrected_point, corrected_point))
            raw_offset = raw_range - guard["target_range_m"]
            correction_delta = corrected_range_for_guard - raw_range
            in_window = abs(raw_offset) <= guard["window_m"]
            moving_away = raw_offset * correction_delta > 0.0
            target_range_away_blend_guard_applied = bool(in_window and moving_away)
            if target_range_away_blend_guard_applied:
                target_range_away_blend_alpha = guard["alpha_away"]
                alpha = target_range_away_blend_alpha
                corrected_point = (
                    target_disparity_blend_raw_point[0] + alpha * (corrected_point[0] - target_disparity_blend_raw_point[0]),
                    target_disparity_blend_raw_point[1] + alpha * (corrected_point[1] - target_disparity_blend_raw_point[1]),
                    target_disparity_blend_raw_point[2] + alpha * (corrected_point[2] - target_disparity_blend_raw_point[2]),
                )
        target_disparity_range_delta_cap_guard_applied = False
        target_disparity_range_delta_cap_m = 0.0
        if self.target_disparity_range_delta_cap_guard is not None:
            guard = self.target_disparity_range_delta_cap_guard
            measured_disparity = float(raw["disparity_px"])
            if not _finite(measured_disparity):
                return GuardPolicyResult(None, self._status(3, "invalid_target_disparity_range_delta_cap_guard"))
            in_window = abs(measured_disparity - guard["target_disparity_px"]) <= guard["half_width_px"]
            raw_range = math.sqrt(_dot(target_disparity_blend_raw_point, target_disparity_blend_raw_point))
            corrected_range_for_guard = math.sqrt(_dot(corrected_point, corrected_point))
            delta = corrected_range_for_guard - raw_range
            cap = guard["range_delta_cap_m"]
            target_disparity_range_delta_cap_m = cap
            if in_window and abs(delta) > cap and corrected_range_for_guard > 1.0e-12:
                target_disparity_range_delta_cap_guard_applied = True
                capped_range = raw_range + math.copysign(cap, delta)
                scale = capped_range / corrected_range_for_guard
                corrected_point = (
                    corrected_point[0] * scale,
                    corrected_point[1] * scale,
                    corrected_point[2] * scale,
                )
        corrected_range = math.sqrt(_dot(corrected_point, corrected_point))
        status = self._status(0, "estimated_to_truth_sim3")
        status.update(
            {
                "profile_id_hash": float(self.profile_id_hash),
                "axis_contract": AXIS_CONTRACT,
                "forward_residual_alpha": float(residual_alpha),
                "forward_residual_unclamped_m": float(forward_residual_unclamped),
                "forward_residual_applied_m": float(forward_residual_applied),
                "rdf_forward_residual_unclamped_m": float(rdf_forward_residual_unclamped),
                "rdf_forward_residual_applied_m": float(rdf_forward_residual_applied),
                "piecewise_sim3_raw_forward_active": self.piecewise_sim3_raw_forward is not None,
                "piecewise_sim3_blend_alpha": self.piecewise_sim3_blend_alpha,
                "left_disparity_triangulation_model_applied": left_disparity_model_applied,
                "target_disparity_blend_guard_applied": target_disparity_blend_guard_applied,
                "target_disparity_blend_alpha": target_disparity_blend_alpha,
                "target_range_away_blend_guard_applied": target_range_away_blend_guard_applied,
                "target_range_away_blend_alpha": target_range_away_blend_alpha,
                "target_disparity_range_delta_cap_guard_applied": target_disparity_range_delta_cap_guard_applied,
                "target_disparity_range_delta_cap_m": target_disparity_range_delta_cap_m,
            }
        )
        if self.forward_residual_agreement_gate is not None:
            status["forward_residual_agreement_gate"] = dict(self.forward_residual_agreement_gate)
        return GuardPolicyResult(
            {
                "right_m": float(corrected_point[0]),
                "down_m": float(corrected_point[1]),
                "forward_m": float(corrected_point[2]),
                "range_m": float(corrected_range),
                "disparity_px": float(output_disparity),
            },
            status,
        )

    def _status(self, fallback_level: int, fallback_reason: str) -> dict[str, object]:
        return {
            "profile_id": self.profile_id,
            "profile_mode": self.mode,
            "fallback_level": int(fallback_level),
            "fallback_reason": str(fallback_reason),
        }


class EstimatedToTruthDisparityProfile:
    mode = "estimated_to_truth_disparity_poly"

    def __init__(
        self,
        *,
        profile_id: str,
        disparity_coefficients_high_to_low: Sequence[float] = (),
        inverse_depth_coefficients_high_to_low: Sequence[float] = (),
        correction_space: str = "raw_rdf_ray_scale",
    ) -> None:
        self.profile_id = str(profile_id or self.mode)
        self.profile_id_hash = int(hashlib.sha1(self.profile_id.encode("utf-8")).hexdigest()[:7], 16)
        if not disparity_coefficients_high_to_low and not inverse_depth_coefficients_high_to_low:
            raise ValueError("disparity profile requires polynomial or inverse-depth coefficients")
        self.disparity_coefficients_high_to_low = tuple(
            float(value) for value in disparity_coefficients_high_to_low
        )
        self.inverse_depth_coefficients_high_to_low = tuple(
            float(value) for value in inverse_depth_coefficients_high_to_low
        )
        self.correction_space = str(correction_space or "raw_rdf_ray_scale")
        if self.correction_space not in {"raw_rdf_ray_scale", "raw_rdf_inverse_depth_ray_scale"}:
            raise ValueError(f"unsupported disparity correction_space: {self.correction_space!r}")

    @classmethod
    def from_json_dict(cls, data: Mapping[str, object]):
        if data.get("mode") != cls.mode:
            raise ValueError(f"unsupported mode: {data.get('mode')!r}")
        coefficients = data.get("d_corr_coefficients_high_to_low")
        if coefficients is None:
            coefficients = data.get("disparity_coefficients_high_to_low")
        inverse_depth_coefficients = data.get("inverse_depth_coefficients_high_to_low")
        if coefficients is None:
            coefficients = []
        if inverse_depth_coefficients is None:
            inverse_depth_coefficients = []
        if not isinstance(coefficients, list):
            raise ValueError("d_corr_coefficients_high_to_low must be a list when provided")
        if not isinstance(inverse_depth_coefficients, list):
            raise ValueError("inverse_depth_coefficients_high_to_low must be a list when provided")
        return cls(
            profile_id=str(data.get("profile_id") or cls.mode),
            disparity_coefficients_high_to_low=coefficients,
            inverse_depth_coefficients_high_to_low=inverse_depth_coefficients,
            correction_space=str(data.get("correction_space") or "raw_rdf_ray_scale"),
        )

    def apply(self, raw: Mapping[str, object]) -> GuardPolicyResult:
        required = ("right_m", "down_m", "forward_m", "disparity_px")
        if any(raw.get(name) is None for name in required):
            return GuardPolicyResult(None, self._status(3, "invalid_raw"))
        raw_point = (float(raw["right_m"]), float(raw["down_m"]), float(raw["forward_m"]))
        disparity = float(raw["disparity_px"])
        if not all(_finite(value) for value in (*raw_point, disparity)):
            return GuardPolicyResult(None, self._status(3, "invalid_raw"))
        if raw_point[2] <= 1.0e-9 or disparity <= 1.0e-9:
            return GuardPolicyResult(None, self._status(3, "invalid_raw"))
        if self.correction_space == "raw_rdf_inverse_depth_ray_scale":
            inverse_depth = 1.0 / raw_point[2]
            corrected_inverse_depth = 0.0
            for coefficient in self.inverse_depth_coefficients_high_to_low:
                corrected_inverse_depth = corrected_inverse_depth * inverse_depth + coefficient
            if not _finite(corrected_inverse_depth) or corrected_inverse_depth <= 1.0e-12:
                return GuardPolicyResult(None, self._status(3, "invalid_corrected_inverse_depth"))
            corrected_forward = 1.0 / corrected_inverse_depth
            scale = corrected_forward / raw_point[2]
            corrected_disparity = disparity / scale
        else:
            corrected_disparity = 0.0
            for coefficient in self.disparity_coefficients_high_to_low:
                corrected_disparity = corrected_disparity * disparity + coefficient
            if not _finite(corrected_disparity) or corrected_disparity <= 1.0e-9:
                return GuardPolicyResult(None, self._status(3, "invalid_corrected_disparity"))
            scale = disparity / corrected_disparity
        corrected_point = _scale(raw_point, scale)
        corrected_range = math.sqrt(_dot(corrected_point, corrected_point))
        status = self._status(0, "estimated_to_truth_disparity_poly")
        status.update(
            {
                "profile_id_hash": float(self.profile_id_hash),
                "axis_contract": AXIS_CONTRACT,
                "corrected_disparity_px": float(corrected_disparity),
                "disparity_scale": float(scale),
            }
        )
        return GuardPolicyResult(
            {
                "right_m": float(corrected_point[0]),
                "down_m": float(corrected_point[1]),
                "forward_m": float(corrected_point[2]),
                "range_m": float(corrected_range),
                "disparity_px": float(corrected_disparity),
            },
            status,
        )

    def _status(self, fallback_level: int, fallback_reason: str) -> dict[str, object]:
        return {
            "profile_id": self.profile_id,
            "profile_mode": self.mode,
            "fallback_level": int(fallback_level),
            "fallback_reason": str(fallback_reason),
        }


class EstimatedToTruthAffineProfile:
    mode = "estimated_to_truth_affine_rdf"

    def __init__(
        self,
        *,
        profile_id: str,
        affine_matrix_rdf: Sequence[Sequence[float]],
        translation_rdf_m: Sequence[float],
        forward_residual_disparity_lut: Mapping[str, object] | None = None,
        forward_residual_raw_forward_lut: Mapping[str, object] | None = None,
        rdf_residual_disparity_lut: Mapping[str, Mapping[str, object]] | None = None,
    ) -> None:
        self.profile_id = str(profile_id or self.mode)
        self.profile_id_hash = int(hashlib.sha1(self.profile_id.encode("utf-8")).hexdigest()[:7], 16)
        self.affine_matrix_rdf = _matrix_from_json([list(row) for row in affine_matrix_rdf])
        if len(translation_rdf_m) != 3:
            raise ValueError("translation_rdf_m must contain three RDF values")
        self.translation_rdf_m = tuple(float(value) for value in translation_rdf_m)
        self.forward_residual_disparity_lut = EstimatedToTruthSim3Profile._parse_forward_lut(
            forward_residual_disparity_lut
        )
        self.forward_residual_raw_forward_lut = EstimatedToTruthSim3Profile._parse_forward_lut(
            forward_residual_raw_forward_lut,
            x_key="x_values_m",
        )
        self.rdf_residual_disparity_lut = EstimatedToTruthSim3Profile._parse_rdf_lut(
            rdf_residual_disparity_lut
        )

    @classmethod
    def from_json_dict(cls, data: Mapping[str, object]):
        if data.get("mode") != cls.mode:
            raise ValueError(f"unsupported mode: {data.get('mode')!r}")
        matrix = data.get("affine_matrix_rdf")
        if matrix is None:
            raise ValueError("estimated_to_truth_affine_rdf requires affine_matrix_rdf")
        translation = data.get("translation_rdf_m")
        if not isinstance(translation, list):
            raise ValueError("estimated_to_truth_affine_rdf requires translation_rdf_m")
        forward_lut = data.get("forward_residual_disparity_lut")
        if forward_lut is not None and not isinstance(forward_lut, Mapping):
            raise ValueError("forward residual LUT must be an object")
        raw_forward_lut = data.get("forward_residual_raw_forward_lut")
        if raw_forward_lut is not None and not isinstance(raw_forward_lut, Mapping):
            raise ValueError("forward residual raw-forward LUT must be an object")
        rdf_lut = data.get("rdf_residual_disparity_lut", {})
        if rdf_lut is None:
            rdf_lut = {}
        if not isinstance(rdf_lut, Mapping):
            raise ValueError("RDF residual LUT must be an object keyed by right/down/forward")
        for axis, lut in rdf_lut.items():
            if axis not in RDF_AXES:
                raise ValueError(f"unsupported RDF residual LUT axis: {axis!r}")
            if not isinstance(lut, Mapping):
                raise ValueError(f"RDF residual LUT for {axis!r} must be an object")
        return cls(
            profile_id=str(data.get("profile_id") or cls.mode),
            affine_matrix_rdf=matrix,
            translation_rdf_m=translation,
            forward_residual_disparity_lut=forward_lut,
            forward_residual_raw_forward_lut=raw_forward_lut,
            rdf_residual_disparity_lut=rdf_lut,
        )

    def apply(self, raw: Mapping[str, object]) -> GuardPolicyResult:
        required = ("right_m", "down_m", "forward_m", "disparity_px")
        if any(raw.get(name) is None for name in required):
            return GuardPolicyResult(None, self._status(3, "invalid_raw"))
        raw_point = (float(raw["right_m"]), float(raw["down_m"]), float(raw["forward_m"]))
        if not all(_finite(value) for value in raw_point):
            return GuardPolicyResult(None, self._status(3, "invalid_raw"))
        corrected_point = _add(_apply_matrix(self.affine_matrix_rdf, raw_point), self.translation_rdf_m)
        disparity = float(raw["disparity_px"])
        if not _finite(disparity):
            return GuardPolicyResult(None, self._status(3, "invalid_raw"))
        if self.forward_residual_disparity_lut["x_values_px"]:
            lut_residual = _interp_piecewise_linear(
                disparity,
                self.forward_residual_disparity_lut["x_values_px"],
                self.forward_residual_disparity_lut["residual_values_m"],
            )
            corrected_point = (
                corrected_point[0],
                corrected_point[1],
                corrected_point[2] + lut_residual,
            )
        if self.forward_residual_raw_forward_lut["x_values_px"]:
            lut_residual = _interp_piecewise_linear(
                raw_point[2],
                self.forward_residual_raw_forward_lut["x_values_px"],
                self.forward_residual_raw_forward_lut["residual_values_m"],
            )
            corrected_point = (
                corrected_point[0],
                corrected_point[1],
                corrected_point[2] + lut_residual,
            )
        if any(lut["x_values_px"] for lut in self.rdf_residual_disparity_lut.values()):
            residuals = [
                _interp_piecewise_linear(
                    disparity,
                    self.rdf_residual_disparity_lut[axis]["x_values_px"],
                    self.rdf_residual_disparity_lut[axis]["residual_values_m"],
                )
                for axis in RDF_AXES
            ]
            corrected_point = (
                corrected_point[0] + residuals[0],
                corrected_point[1] + residuals[1],
                corrected_point[2] + residuals[2],
            )
        corrected_range = math.sqrt(_dot(corrected_point, corrected_point))
        status = self._status(0, "estimated_to_truth_affine_rdf")
        status.update(
            {
                "profile_id_hash": float(self.profile_id_hash),
                "axis_contract": AXIS_CONTRACT,
            }
        )
        return GuardPolicyResult(
            {
                "right_m": float(corrected_point[0]),
                "down_m": float(corrected_point[1]),
                "forward_m": float(corrected_point[2]),
                "range_m": float(corrected_range),
                "disparity_px": float(raw["disparity_px"]),
            },
            status,
        )

    def _status(self, fallback_level: int, fallback_reason: str) -> dict[str, object]:
        return {
            "profile_id": self.profile_id,
            "profile_mode": self.mode,
            "fallback_level": int(fallback_level),
            "fallback_reason": str(fallback_reason),
        }


class BoundedGainForwardCellGuardPolicy:
    mode = "bounded_gain_forward_cell_guard"

    def __init__(
        self,
        *,
        profile_id: str,
        profile_family: str,
        anchor_profile: Mapping[str, object],
        right_actual_to_nominal_rotation: tuple[tuple[float, float, float], ...],
        range_edges_m: Sequence[float],
        raw_down_edges_m: Sequence[float],
        center_delta_down_edges_m: Sequence[float],
        right_candidate: str,
        right_min_range_m: float,
        forward_candidate: str,
        forward_min_range_m: float,
        forward_raw_anchor_cells: Sequence[str],
        down_cells: Mapping[str, Mapping[str, object]],
    ) -> None:
        self.profile_id = str(profile_id or "bounded_gain_forward_cell_guard")
        self.profile_id_hash = int(hashlib.sha1(self.profile_id.encode("utf-8")).hexdigest()[:7], 16)
        self.profile_family = str(profile_family or "unknown")
        self.anchor_profile = dict(anchor_profile)
        self.right_actual_to_nominal_rotation = right_actual_to_nominal_rotation
        self.range_edges_m = tuple(float(value) for value in range_edges_m)
        self.raw_down_edges_m = tuple(float(value) for value in raw_down_edges_m)
        self.center_delta_down_edges_m = tuple(float(value) for value in center_delta_down_edges_m)
        self.right_candidate = str(right_candidate or "anchor_gain_0.1")
        self.right_min_range_m = float(right_min_range_m)
        self.forward_candidate = str(forward_candidate or "anchor_gain_1")
        self.forward_min_range_m = float(forward_min_range_m)
        self.forward_raw_anchor_cells = {str(value) for value in forward_raw_anchor_cells}
        self.down_cells = {str(key): dict(value) for key, value in down_cells.items()}

    @classmethod
    def from_json_dict(cls, data: Mapping[str, object]):
        if data.get("mode") != cls.mode:
            raise ValueError(f"unsupported mode: {data.get('mode')!r}")
        anchor_profile = data.get("anchor_profile")
        if isinstance(anchor_profile, Mapping) and isinstance(anchor_profile.get("anchor_profile"), Mapping):
            anchor_profile = anchor_profile["anchor_profile"]
        if not isinstance(anchor_profile, Mapping):
            raise ValueError("bounded gain policy requires anchor_profile")

        right_policy = data.get("right_policy") if isinstance(data.get("right_policy"), Mapping) else {}
        forward_policy = data.get("forward_policy") if isinstance(data.get("forward_policy"), Mapping) else {}
        down_policy = data.get("down_policy") if isinstance(data.get("down_policy"), Mapping) else {}
        down_cells = down_policy.get("cells", {}) if isinstance(down_policy, Mapping) else {}
        if not isinstance(down_cells, Mapping):
            raise ValueError("down_policy.cells must be an object")

        return cls(
            profile_id=str(data.get("profile_id") or "bounded_gain_forward_cell_guard"),
            profile_family=str(data.get("profile_family") or data.get("runtime_profile_family") or "unknown"),
            anchor_profile=anchor_profile,
            right_actual_to_nominal_rotation=_matrix_from_json(
                data.get("right_actual_to_nominal_rotation")
            ),
            range_edges_m=data.get("range_edges_m", DEFAULT_RANGE_EDGES_M),
            raw_down_edges_m=data.get("raw_down_edges_m", DEFAULT_RAW_DOWN_EDGES_M),
            center_delta_down_edges_m=data.get(
                "center_delta_down_edges_m",
                DEFAULT_CENTER_DELTA_DOWN_EDGES_M,
            ),
            right_candidate=str(right_policy.get("candidate", "anchor_gain_0.1")),
            right_min_range_m=float(right_policy.get("min_range_m", 1100.0)),
            forward_candidate=str(forward_policy.get("candidate", "anchor_gain_1")),
            forward_min_range_m=float(forward_policy.get("min_range_m", 500.0)),
            forward_raw_anchor_cells=forward_policy.get("raw_fallback_anchor_cells", []),
            down_cells=down_cells,
        )

    def apply(
        self,
        left_detection: Mapping[str, object],
        right_detection: Mapping[str, object],
        raw: Mapping[str, object],
        *,
        width_px: float,
        height_px: float,
        horizontal_fov_rad: float,
        baseline_m: float,
    ) -> GuardPolicyResult:
        status = self._status(fallback_level=3, fallback_reason="init")
        if any(raw.get(name) is None for name in ("right_m", "down_m", "forward_m", "range_m", "disparity_px")):
            status["fallback_reason"] = "invalid_raw"
            return GuardPolicyResult(None, status)
        raw_point = (float(raw["right_m"]), float(raw["down_m"]), float(raw["forward_m"]))
        if not all(_finite(value) for value in raw_point):
            status["fallback_reason"] = "invalid_raw"
            return GuardPolicyResult(None, status)

        camera = RuntimeCameraModel(
            width_px=float(width_px),
            height_px=float(height_px),
            horizontal_fov_rad=float(horizontal_fov_rad),
            baseline_m=float(baseline_m),
        )
        try:
            anchor_point, anchor_cell = self._anchor_point(left_detection, right_detection, camera)
            center_point = self._triangulate_with_rotation(
                (float(left_detection["cx"]), float(left_detection["cy"])),
                (float(right_detection["cx"]), float(right_detection["cy"])),
                camera,
            )
        except (KeyError, TypeError, ValueError):
            status["fallback_reason"] = "invalid_bbox"
            return GuardPolicyResult(None, status)
        if anchor_point is None or center_point is None:
            status["fallback_reason"] = "candidate_triangulation_failed"
            return GuardPolicyResult(None, status)

        right_value, right_name = self._axis_value(
            self.right_candidate if raw_point[2] >= self.right_min_range_m else "raw",
            0,
            raw_point,
            anchor_point,
            center_point,
        )

        forward_candidate = self.forward_candidate if raw_point[2] >= self.forward_min_range_m else "raw"
        if anchor_cell in self.forward_raw_anchor_cells:
            forward_candidate = "raw"
        forward_value, forward_name = self._axis_value(
            forward_candidate,
            2,
            raw_point,
            anchor_point,
            center_point,
        )

        center_delta_down = center_point[1] - raw_point[1]
        down_key = self._down_feature_key(raw_point, center_delta_down)
        if down_key is None:
            status["fallback_reason"] = "outside_policy_range"
            status["profile_family"] = self.profile_family
            return GuardPolicyResult(None, status)
        down_candidate = "raw"
        down_cell = self.down_cells.get(down_key)
        if isinstance(down_cell, Mapping):
            down_candidate = str(down_cell.get("candidate", "raw"))
        down_value, down_name = self._axis_value(
            down_candidate,
            1,
            raw_point,
            anchor_point,
            center_point,
        )

        corrected_range = math.sqrt(
            right_value * right_value + down_value * down_value + forward_value * forward_value
        )
        status = self._status(fallback_level=0, fallback_reason="bounded_gain_forward_cell_guard")
        status.update(
            {
                "profile_family": self.profile_family,
                "anchor_cell": anchor_cell,
                "down_feature_key": down_key,
                "candidate_right": right_name,
                "candidate_down": down_name,
                "candidate_forward": forward_name,
                "profile_id_hash": float(self.profile_id_hash),
            }
        )
        return GuardPolicyResult(
            {
                "right_m": float(right_value),
                "down_m": float(down_value),
                "forward_m": float(forward_value),
                "range_m": float(corrected_range),
                "disparity_px": float(raw["disparity_px"]),
            },
            status,
        )

    def _status(self, *, fallback_level: int, fallback_reason: str) -> dict[str, object]:
        return {
            "profile_id": self.profile_id,
            "profile_mode": self.mode,
            "fallback_level": int(fallback_level),
            "fallback_reason": str(fallback_reason),
        }

    def _axis_value(
        self,
        candidate: str,
        axis_index: int,
        raw_point: tuple[float, float, float],
        anchor_point: tuple[float, float, float],
        center_point: tuple[float, float, float],
    ) -> tuple[float, str]:
        source_name, gain = _candidate_source_and_gain(candidate)
        if source_name == "raw":
            return raw_point[axis_index], "raw"
        source_point = anchor_point if source_name == "anchor" else center_point
        value = raw_point[axis_index] + gain * (source_point[axis_index] - raw_point[axis_index])
        return float(value), candidate

    def _down_feature_key(
        self,
        raw_point: tuple[float, float, float],
        center_delta_down: float,
    ) -> Optional[str]:
        # Guard policy raw points use the internal RDF axis contract:
        # right/down/forward.
        range_bin = _bin_index(raw_point[2], self.range_edges_m)
        raw_down_bin = _bin_index(raw_point[1], self.raw_down_edges_m)
        center_delta_down_bin = _bin_index(center_delta_down, self.center_delta_down_edges_m)
        if range_bin is None or raw_down_bin is None or center_delta_down_bin is None:
            return None
        return f"{self.profile_family}|{range_bin}|{raw_down_bin}|{center_delta_down_bin}"

    def _anchor_point(
        self,
        left_detection: Mapping[str, object],
        right_detection: Mapping[str, object],
        camera: RuntimeCameraModel,
    ) -> tuple[Optional[tuple[float, float, float]], str]:
        anchor, anchor_cell = self._anchor_for_detection(left_detection, right_detection, camera)
        left_x1, left_y1, left_x2, left_y2 = self._bbox(left_detection)
        right_x1, right_y1, right_x2, right_y2 = self._bbox(right_detection)
        left_pixel = (
            left_x1 + float(anchor["left_u"]) * (left_x2 - left_x1),
            left_y1 + float(anchor["left_v"]) * (left_y2 - left_y1),
        )
        right_pixel = (
            right_x1 + float(anchor["right_u"]) * (right_x2 - right_x1),
            right_y1 + float(anchor["right_v"]) * (right_y2 - right_y1),
        )
        return self._triangulate_with_rotation(left_pixel, right_pixel, camera), anchor_cell

    def _anchor_for_detection(
        self,
        left_detection: Mapping[str, object],
        right_detection: Mapping[str, object],
        camera: RuntimeCameraModel,
    ) -> tuple[Mapping[str, object], str]:
        global_anchor = self.anchor_profile.get("global_anchor")
        if not isinstance(global_anchor, Mapping):
            raise ValueError("anchor profile has no global_anchor")
        mode = str(self.anchor_profile.get("mode", "global"))
        if mode == "global":
            return global_anchor, "global"
        if mode == "disparity_bin":
            disparity = abs(float(left_detection["cx"]) - float(right_detection["cx"]))
            index = self._profile_bin_index(disparity, self.anchor_profile.get("disparity_edges_px"))
            cell = self._profile_cell(str(index)) if index is not None else None
            if cell is not None and isinstance(cell.get("anchor"), Mapping):
                return cell["anchor"], f"disparity_bin_{index}"
            return global_anchor, "global_fallback"
        if mode == "screen_grid":
            key = self._screen_cell_key(left_detection, camera)
            cell = self._profile_cell(key) if key is not None else None
            if cell is not None and isinstance(cell.get("anchor"), Mapping):
                return cell["anchor"], f"screen_grid_{key}"
            nearest_key = self._nearest_screen_cell_key(key)
            nearest_cell = self._profile_cell(nearest_key) if nearest_key is not None else None
            if nearest_cell is not None and isinstance(nearest_cell.get("anchor"), Mapping):
                return nearest_cell["anchor"], f"screen_nearest_{nearest_key}"
            return global_anchor, "global_fallback"
        return global_anchor, "global_fallback"

    def _profile_cell(self, key: object) -> Optional[Mapping[str, object]]:
        cells = self.anchor_profile.get("cells")
        if not isinstance(cells, Mapping) or key is None:
            return None
        cell = cells.get(str(key))
        return cell if isinstance(cell, Mapping) else None

    def _profile_bin_index(self, value: float, edges: object) -> Optional[int]:
        if not isinstance(edges, list) or len(edges) < 2:
            return None
        for index in range(len(edges) - 1):
            lo = float(edges[index])
            hi = float(edges[index + 1])
            if lo <= value < hi or (index == len(edges) - 2 and value <= hi):
                return index
        return None

    def _screen_cell_key(self, left_detection: Mapping[str, object], camera: RuntimeCameraModel) -> Optional[str]:
        cols = int(self.anchor_profile.get("screen_cols", 0))
        rows = int(self.anchor_profile.get("screen_rows", 0))
        if cols <= 0 or rows <= 0:
            return None
        u = float(left_detection["cx"])
        v = float(left_detection["cy"])
        if not (0.0 <= u <= camera.width_px and 0.0 <= v <= camera.height_px):
            return None
        col = min(cols - 1, max(0, int(u / camera.width_px * cols)))
        row = min(rows - 1, max(0, int(v / camera.height_px * rows)))
        return f"{row}_{col}"

    def _nearest_screen_cell_key(self, key: Optional[str]) -> Optional[str]:
        if key is None or not self.anchor_profile.get("nearest_screen_cell"):
            return None
        cells = self.anchor_profile.get("cells")
        if not isinstance(cells, Mapping):
            return None
        try:
            row_text, col_text = key.split("_", 1)
            row = int(row_text)
            col = int(col_text)
        except ValueError:
            return None
        best_key = None
        best_distance = None
        for candidate in cells:
            try:
                candidate_row_text, candidate_col_text = str(candidate).split("_", 1)
                candidate_row = int(candidate_row_text)
                candidate_col = int(candidate_col_text)
            except ValueError:
                continue
            distance = (candidate_row - row) ** 2 + (candidate_col - col) ** 2
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_key = str(candidate)
        return best_key

    def _triangulate_with_rotation(
        self,
        left_pixel: tuple[float, float],
        right_pixel: tuple[float, float],
        camera: RuntimeCameraModel,
    ) -> Optional[tuple[float, float, float]]:
        left_ray = _normalize(
            (
                (left_pixel[0] - camera.cx_px) / camera.fx_px,
                (left_pixel[1] - camera.cy_px) / camera.fx_px,
                1.0,
            )
        )
        right_ray = _normalize(
            (
                (right_pixel[0] - camera.cx_px) / camera.fx_px,
                (right_pixel[1] - camera.cy_px) / camera.fx_px,
                1.0,
            )
        )
        return self._triangulate_rays(
            left_ray,
            _apply_matrix(self.right_actual_to_nominal_rotation, right_ray),
            camera,
        )

    def _triangulate_rays(
        self,
        left_ray: tuple[float, float, float],
        right_ray_in_left: tuple[float, float, float],
        camera: RuntimeCameraModel,
    ) -> Optional[tuple[float, float, float]]:
        d1 = _normalize(left_ray)
        d2 = _normalize(right_ray_in_left)
        translation = (camera.baseline_m, 0.0, 0.0)
        a = _dot(d1, d1)
        b = _dot(d1, d2)
        c = _dot(d2, d2)
        w0 = _scale(translation, -1.0)
        d = _dot(d1, w0)
        e = _dot(d2, w0)
        denom = a * c - b * b
        if abs(denom) <= 1.0e-12:
            return None
        s = (b * e - c * d) / denom
        u = (a * e - b * d) / denom
        if s <= 0.0 or u <= 0.0:
            return None
        midpoint_left = _scale(_add(_scale(d1, s), _add(translation, _scale(d2, u))), 0.5)
        return (midpoint_left[0] - camera.baseline_m * 0.5, midpoint_left[1], midpoint_left[2])

    def _bbox(self, detection: Mapping[str, object]) -> tuple[float, float, float, float]:
        bbox = detection.get("bbox")
        if not isinstance(bbox, (tuple, list)) or len(bbox) != 4:
            raise ValueError("detection has no bbox")
        return (float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))
