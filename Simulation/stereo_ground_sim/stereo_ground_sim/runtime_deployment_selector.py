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
DEFAULT_CAMERA_HEIGHT_PX = 720.0
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


@dataclass(frozen=True)
class RuntimeImageObservation:
    """One runtime left-image stereo observation.

    Input contract: ``left_u_px``/``left_v_px`` are left-image pixel coordinates,
    and ``measured_disparity_px`` is left minus right image disparity in pixels.
    No PX4/RTK truth, validation metric, or replay label is accepted here.
    """

    left_u_px: float
    left_v_px: float
    measured_disparity_px: float

    @classmethod
    def from_mapping(cls, row: Mapping[str, object]) -> "RuntimeImageObservation | None":
        left_u = finite_float(row.get("left_u_px", row.get("left_cx_px")))
        left_v = finite_float(row.get("left_v_px", row.get("left_cy_px")))
        disparity = finite_float(row.get("measured_disparity_px", row.get("raw_disparity_px")))
        if not all(math.isfinite(value) for value in (left_u, left_v, disparity)):
            return None
        if disparity <= 0.0:
            return None
        return cls(
            left_u_px=left_u,
            left_v_px=left_v,
            measured_disparity_px=disparity,
        )


@dataclass(frozen=True)
class RuntimeStereoCameraContract:
    """Fixed stereo geometry used to convert image observations to RDF.

    Output frame is stereo-center OpenCV optical RDF:
    ``right_m = +image right``, ``down_m = +image down``,
    ``forward_m = +optical forward``. Baseline is along RDF right, and the
    public StereoDistance FRD+range topic is not used in this conversion.
    """

    width_px: float = DEFAULT_CAMERA_WIDTH_PX
    height_px: float = DEFAULT_CAMERA_HEIGHT_PX
    horizontal_fov_rad: float = DEFAULT_CAMERA_HORIZONTAL_FOV_RAD
    baseline_m: float = DEFAULT_CAMERA_BASELINE_M

    @property
    def fx_px(self) -> float:
        return (self.width_px * 0.5) / math.tan(self.horizontal_fov_rad * 0.5)

    @property
    def fy_px(self) -> float:
        return self.fx_px

    @property
    def cx_px(self) -> float:
        return self.width_px * 0.5

    @property
    def cy_px(self) -> float:
        return self.height_px * 0.5

    def triangulate_left_observation(self, observation: RuntimeImageObservation) -> RuntimeMeasurementSample | None:
        disparity = finite_float(observation.measured_disparity_px)
        if not math.isfinite(disparity) or disparity <= 0.0:
            return None
        forward_m = self.fx_px * self.baseline_m / disparity
        right_m = (observation.left_u_px - self.cx_px) * forward_m / self.fx_px - self.baseline_m * 0.5
        down_m = (observation.left_v_px - self.cy_px) * forward_m / self.fy_px
        if not all(math.isfinite(value) for value in (right_m, down_m, forward_m)):
            return None
        raw_range_m = math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)
        return RuntimeMeasurementSample(
            raw_right_m=right_m,
            raw_down_m=down_m,
            raw_forward_m=forward_m,
            measured_disparity_px=disparity,
            raw_range_m=raw_range_m,
            left_u_px=observation.left_u_px,
            left_v_px=observation.left_v_px,
        )


@dataclass(frozen=True)
class RuntimeScenarioMetadata:
    """Runtime geometry used to guard scenario-specific profiles.

    All fields are metadata only. They do not introduce PX4/GPS truth into
    runtime selection; they describe the current camera/target setup so a
    profile can no-op outside its validated envelope.
    """

    baseline_m: float | None = None
    camera_height_m: float | None = None
    target_size_m: float | None = None
    zoom_index: float | None = None
    horizontal_fov_rad: float | None = None
    raw_range_m: float | None = None

    @classmethod
    def from_features(cls, features: Mapping[str, object]) -> "RuntimeScenarioMetadata":
        return cls(
            baseline_m=_optional_finite(features.get("baseline_m")),
            camera_height_m=_optional_finite(features.get("camera_height_m")),
            target_size_m=_optional_finite(features.get("target_size_m")),
            zoom_index=_optional_finite(features.get("zoom_index")),
            horizontal_fov_rad=_optional_finite(features.get("horizontal_fov_rad")),
            raw_range_m=_optional_finite(features.get("raw_range_m", features.get("raw_range_mean_m"))),
        )


def _optional_finite(value: object) -> float | None:
    parsed = finite_float(value)
    return parsed if math.isfinite(parsed) else None


def _range_contains(bounds: object, value: float | None) -> bool:
    if value is None or not isinstance(bounds, Sequence) or isinstance(bounds, (str, bytes)) or len(bounds) != 2:
        return True
    lower = _optional_finite(bounds[0])
    upper = _optional_finite(bounds[1])
    if lower is not None and value < lower:
        return False
    if upper is not None and value > upper:
        return False
    return True


def _scalar_matches(expected: object, value: float | None, *, tolerance: float = 1.0e-6) -> bool:
    if value is None or expected in (None, ""):
        return True
    parsed = _optional_finite(expected)
    if parsed is None:
        return True
    return abs(value - parsed) <= tolerance


def scenario_envelope_match(
    envelope: Mapping[str, object] | None,
    scenario: RuntimeScenarioMetadata,
) -> tuple[bool, list[str]]:
    if not envelope:
        return True, []
    failures: list[str] = []
    if not _range_contains(envelope.get("baseline_m"), scenario.baseline_m):
        failures.append("baseline_m_outside_envelope")
    if not _range_contains(envelope.get("camera_height_m"), scenario.camera_height_m):
        failures.append("camera_height_m_outside_envelope")
    if not _range_contains(envelope.get("target_size_m"), scenario.target_size_m):
        failures.append("target_size_m_outside_envelope")
    if not _range_contains(envelope.get("range_m"), scenario.raw_range_m):
        failures.append("range_m_outside_envelope")
    if not _scalar_matches(envelope.get("zoom_index"), scenario.zoom_index):
        failures.append("zoom_index_outside_envelope")
    if not _scalar_matches(envelope.get("horizontal_fov_rad"), scenario.horizontal_fov_rad, tolerance=1.0e-9):
        failures.append("horizontal_fov_rad_outside_envelope")
    return not failures, failures


def fused_runtime_observation_value(
    values: Sequence[float],
    *,
    mode: str = "median",
    trim_fraction: float = 0.2,
) -> float:
    arr = finite_array(values)
    if arr.size == 0:
        return float("nan")
    arr = np.sort(arr)
    if mode == "median":
        return float(np.median(arr))
    if mode == "mean":
        return float(np.mean(arr))
    if mode == "trimmed_mean":
        trim = int(math.floor(max(0.0, float(trim_fraction)) * arr.size))
        if trim * 2 >= arr.size:
            return float(np.median(arr))
        return float(np.mean(arr[trim : arr.size - trim]))
    raise ValueError(f"unsupported runtime observation fusion mode: {mode}")


def fuse_runtime_image_observations(
    observations: Iterable[RuntimeImageObservation],
    *,
    min_observations: int = 1,
    mode: str = "median",
    trim_fraction: float = 0.2,
) -> RuntimeImageObservation | None:
    valid_observations = [
        observation
        for observation in observations
        if all(
            math.isfinite(value)
            for value in (
                observation.left_u_px,
                observation.left_v_px,
                observation.measured_disparity_px,
            )
        )
        and observation.measured_disparity_px > 0.0
    ]
    if len(valid_observations) < max(1, int(min_observations)):
        return None
    fused = RuntimeImageObservation(
        left_u_px=fused_runtime_observation_value(
            [observation.left_u_px for observation in valid_observations],
            mode=mode,
            trim_fraction=trim_fraction,
        ),
        left_v_px=fused_runtime_observation_value(
            [observation.left_v_px for observation in valid_observations],
            mode=mode,
            trim_fraction=trim_fraction,
        ),
        measured_disparity_px=fused_runtime_observation_value(
            [observation.measured_disparity_px for observation in valid_observations],
            mode=mode,
            trim_fraction=trim_fraction,
        ),
    )
    if not all(
        math.isfinite(value)
        for value in (
            fused.left_u_px,
            fused.left_v_px,
            fused.measured_disparity_px,
        )
    ):
        return None
    if fused.measured_disparity_px <= 0.0:
        return None
    return fused


class RepeatedObservationFusion:
    """Fuse multiple independent image observations into one raw RDF sample.

    Runtime input assumptions: observations are independent detections for the
    same timestamp or short burst, expressed only as left image center pixels and
    measured disparity. Output is a ``RuntimeMeasurementSample`` in stereo-center
    RDF, ready for profile application or runtime selector features.
    """

    def __init__(
        self,
        *,
        camera: RuntimeStereoCameraContract | None = None,
        min_observations: int = 3,
        mode: str = "median",
        trim_fraction: float = 0.2,
    ) -> None:
        self.camera = camera if camera is not None else RuntimeStereoCameraContract()
        self.min_observations = max(1, int(min_observations))
        self.mode = mode
        self.trim_fraction = float(trim_fraction)

    def fuse(self, observations: Iterable[RuntimeImageObservation]) -> RuntimeMeasurementSample | None:
        fused = fuse_runtime_image_observations(
            observations,
            min_observations=self.min_observations,
            mode=self.mode,
            trim_fraction=self.trim_fraction,
        )
        if fused is None:
            return None
        return self.camera.triangulate_left_observation(fused)

    def fuse_rows(self, rows: Iterable[Mapping[str, object]]) -> RuntimeMeasurementSample | None:
        observations = [
            observation
            for row in rows
            if (observation := RuntimeImageObservation.from_mapping(row)) is not None
        ]
        return self.fuse(observations)


class MultiBurstObservationFusion:
    """Fuse several short image-observation bursts into one raw RDF sample.

    Runtime input assumptions: each burst contains left-image center pixels and
    measured disparity for the same target over a short time span. Common-mode
    image association errors may exist within one burst, so each burst is first
    fused in pixel/disparity space and the burst representatives are fused
    again. Output is a stereo-center RDF ``RuntimeMeasurementSample``.
    """

    def __init__(
        self,
        *,
        camera: RuntimeStereoCameraContract | None = None,
        min_bursts: int = 3,
        min_observations_per_burst: int = 5,
        inner_mode: str = "trimmed_mean",
        outer_mode: str = "median",
        trim_fraction: float = 0.2,
    ) -> None:
        self.camera = camera if camera is not None else RuntimeStereoCameraContract()
        self.min_bursts = max(1, int(min_bursts))
        self.min_observations_per_burst = max(1, int(min_observations_per_burst))
        self.inner_mode = inner_mode
        self.outer_mode = outer_mode
        self.trim_fraction = float(trim_fraction)

    def fuse(self, bursts: Iterable[Iterable[RuntimeImageObservation]]) -> RuntimeMeasurementSample | None:
        fused_bursts: list[RuntimeImageObservation] = []
        for burst in bursts:
            fused_burst = fuse_runtime_image_observations(
                burst,
                min_observations=self.min_observations_per_burst,
                mode=self.inner_mode,
                trim_fraction=self.trim_fraction,
            )
            if fused_burst is not None:
                fused_bursts.append(fused_burst)
        if len(fused_bursts) < self.min_bursts:
            return None
        fused = fuse_runtime_image_observations(
            fused_bursts,
            min_observations=self.min_bursts,
            mode=self.outer_mode,
            trim_fraction=self.trim_fraction,
        )
        if fused is None:
            return None
        return self.camera.triangulate_left_observation(fused)

    def fuse_row_groups(self, row_groups: Iterable[Iterable[Mapping[str, object]]]) -> RuntimeMeasurementSample | None:
        bursts: list[list[RuntimeImageObservation]] = []
        for rows in row_groups:
            burst = [
                observation
                for row in rows
                if (observation := RuntimeImageObservation.from_mapping(row)) is not None
            ]
            bursts.append(burst)
        return self.fuse(bursts)


@dataclass(frozen=True)
class DisparityResidualRuntimeProfile:
    """Runtime residual correction driven by runtime measurement features.

    The coefficients are fitted offline with calibration truth, but applying the
    profile consumes only a runtime ``RuntimeMeasurementSample`` in
    stereo-center RDF plus its measured disparity. Array/field order is
    explicit: RDF is right/down/forward, so ``raw_forward_m`` is the forward
    axis of the stereo-center RDF point. Public FRD topics are not inputs here.
    """

    feature_space: str
    correction_space: str
    degree: int
    feature_mean: float
    feature_scale: float
    coefficients_low_to_high: tuple[float, ...]
    correction_clip: float
    profile_id: str = ""
    method: str = ""

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> "DisparityResidualRuntimeProfile":
        profile_type = str(payload.get("profile_type", payload.get("class", "")))
        if profile_type not in {"DisparityResidualRuntimeProfile", "disparity_residual_runtime_profile"}:
            raise ValueError(f"unsupported disparity residual profile type: {profile_type}")
        coefficients = payload.get("coefficients_low_to_high", [])
        if not isinstance(coefficients, Sequence) or isinstance(coefficients, (str, bytes)):
            raise ValueError("coefficients_low_to_high must be a list")
        return cls(
            feature_space=str(payload.get("feature_space", "log_disparity")),
            correction_space=str(payload.get("correction_space", "ray_scale")),
            degree=int(finite_float(payload.get("degree"), 0.0)),
            feature_mean=finite_float(payload.get("feature_mean"), 0.0),
            feature_scale=max(1.0e-12, finite_float(payload.get("feature_scale"), 1.0)),
            coefficients_low_to_high=tuple(float(value) for value in coefficients),
            correction_clip=max(0.0, finite_float(payload.get("correction_clip"), 0.0)),
            profile_id=str(payload.get("profile_id", "")),
            method=str(payload.get("method", "")),
        )

    @classmethod
    def from_json_file(cls, path: str | Path) -> "DisparityResidualRuntimeProfile":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("disparity residual profile must be a JSON object")
        return cls.from_mapping(payload)

    def _feature(self, sample: RuntimeMeasurementSample) -> float:
        disparity = finite_float(sample.measured_disparity_px)
        if not math.isfinite(disparity) or disparity <= 0.0:
            return float("nan")
        if self.feature_space in {"disparity", "inverse_depth"}:
            return disparity
        if self.feature_space == "log_disparity":
            return math.log(max(disparity, 1.0e-9))
        raw_forward = finite_float(sample.raw_forward_m)
        if self.feature_space == "raw_forward":
            return raw_forward
        if self.feature_space == "log_raw_forward":
            if not math.isfinite(raw_forward) or raw_forward <= 0.0:
                return float("nan")
            return math.log(max(raw_forward, 1.0e-9))
        return float("nan")

    def correction_value(self, sample: RuntimeMeasurementSample) -> float:
        feature = self._feature(sample)
        if not math.isfinite(feature):
            return float("nan")
        z = (feature - self.feature_mean) / max(self.feature_scale, 1.0e-12)
        value = 0.0
        power = 1.0
        for coefficient in self.coefficients_low_to_high:
            value += float(coefficient) * power
            power *= z
        if self.correction_clip > 0.0:
            value = max(-self.correction_clip, min(self.correction_clip, value))
        return float(value)

    def apply(self, sample: RuntimeMeasurementSample) -> RuntimeMeasurementSample | None:
        correction = self.correction_value(sample)
        if not math.isfinite(correction):
            return None
        right = float(sample.raw_right_m)
        down = float(sample.raw_down_m)
        forward = float(sample.raw_forward_m)
        if self.correction_space == "forward_add":
            forward = forward + correction
        elif self.correction_space == "ray_scale":
            scale = 1.0 + correction
            right *= scale
            down *= scale
            forward *= scale
        else:
            return None
        if not all(math.isfinite(value) for value in (right, down, forward)):
            return None
        return RuntimeMeasurementSample(
            raw_right_m=right,
            raw_down_m=down,
            raw_forward_m=forward,
            measured_disparity_px=sample.measured_disparity_px,
            raw_range_m=math.sqrt(right * right + down * down + forward * forward),
            left_u_px=sample.left_u_px,
            left_v_px=sample.left_v_px,
        )


def build_observation_fusion_adapter(
    adapter_config: Mapping[str, object],
    *,
    camera: RuntimeStereoCameraContract | None = None,
) -> RepeatedObservationFusion | MultiBurstObservationFusion:
    """Build a runtime observation-fusion adapter from manifest config.

    The config is expected to come from
    ``observation_fusion_runtime_manifest.json``. It selects only the fusion
    strategy and numeric parameters; it does not add truth, validation metrics,
    public FRD positions, or replay labels to the runtime input contract.
    """
    adapter_class = str(adapter_config.get("class", ""))
    if adapter_class == "RepeatedObservationFusion":
        return RepeatedObservationFusion(
            camera=camera,
            min_observations=int(finite_float(adapter_config.get("min_observations"), 3.0)),
            mode=str(adapter_config.get("mode", "median")),
            trim_fraction=finite_float(adapter_config.get("trim_fraction"), 0.2),
        )
    if adapter_class == "MultiBurstObservationFusion":
        return MultiBurstObservationFusion(
            camera=camera,
            min_bursts=int(finite_float(adapter_config.get("min_bursts"), 3.0)),
            min_observations_per_burst=int(
                finite_float(
                    adapter_config.get("min_observations_per_burst"),
                    5.0,
                )
            ),
            inner_mode=str(adapter_config.get("inner_mode", "trimmed_mean")),
            outer_mode=str(adapter_config.get("outer_mode", "median")),
            trim_fraction=finite_float(adapter_config.get("trim_fraction"), 0.2),
        )
    raise ValueError(f"unsupported observation fusion adapter class: {adapter_class}")


def observation_fusion_tier(
    manifest: Mapping[str, object],
    *,
    tier: str | None = None,
) -> Mapping[str, object]:
    tiers = manifest.get("tiers")
    if not isinstance(tiers, Sequence) or isinstance(tiers, (str, bytes)):
        raise ValueError("observation fusion manifest must contain a tiers list")
    selected_tier = tier or str(manifest.get("default_recommendation", ""))
    for item in tiers:
        if isinstance(item, Mapping) and str(item.get("tier", "")) == selected_tier:
            return item
    raise ValueError(f"observation fusion tier not found: {selected_tier}")


def load_observation_fusion_adapter_from_manifest(
    path: str | Path,
    *,
    tier: str | None = None,
    camera: RuntimeStereoCameraContract | None = None,
) -> tuple[RepeatedObservationFusion | MultiBurstObservationFusion, Mapping[str, object]]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("observation fusion manifest must be a JSON object")
    tier_payload = observation_fusion_tier(payload, tier=tier)
    adapter_config = tier_payload.get("adapter")
    if not isinstance(adapter_config, Mapping):
        raise ValueError("observation fusion tier must contain an adapter object")
    return build_observation_fusion_adapter(adapter_config, camera=camera), tier_payload


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
        label_map = self.manifest.get("label_profile_map", {})
        self.label_profile_map = {
            str(key): str(value)
            for key, value in label_map.items()
            if str(key) and str(value)
        } if isinstance(label_map, Mapping) else {}
        envelope = self.manifest.get("validated_envelope", {})
        self.validated_envelope = dict(envelope) if isinstance(envelope, Mapping) else {}

    @classmethod
    def from_json_file(cls, path: str | Path) -> "PackagedRuntimeDeploymentSelector":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("runtime deployment selector manifest must be a JSON object")
        return cls(payload)

    def select(self, features: Mapping[str, object]) -> dict[str, object]:
        scenario = RuntimeScenarioMetadata.from_features(features)
        envelope_ok, envelope_failures = scenario_envelope_match(self.validated_envelope, scenario)
        use_override = override_applies(features, self.override_rule)
        deployment_label = str(
            features.get(
                "deployment_label",
                features.get("scenario_label", features.get("scenario_name", "")),
            )
            or ""
        )
        label_profile = self.label_profile_map.get(deployment_label, "")
        label_selection_applied = bool(label_profile) and envelope_ok
        if label_selection_applied:
            selected_label = label_profile
        elif use_override and envelope_ok:
            selected_label = self.override_profile_label
        else:
            selected_label = self.baseline_selector
        return {
            "selected_label": selected_label,
            "deployment_label": deployment_label,
            "label_selection_applied": label_selection_applied,
            "override_applied": (not label_selection_applied) and use_override and envelope_ok,
            "scenario_envelope_ok": envelope_ok,
            "scenario_envelope_failures": envelope_failures,
            "override_rule": dict(self.override_rule),
            "label_profile_map": dict(self.label_profile_map),
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "selection_contract": self.manifest.get("selection_contract", ""),
        }
