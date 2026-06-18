from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class TrackingFilterConfig:
    process_accel_std_mps2: float = 4.0
    initial_position_std_m: float = 25.0
    initial_velocity_std_mps: float = 20.0
    reset_after_s: float = 3.0
    max_normalized_innovation: float = 12.0
    min_lateral_std_m: float = 0.25
    min_forward_std_m: float = 2.0
    max_lateral_std_m: float = 50.0
    max_forward_std_m: float = 250.0


@dataclass(frozen=True)
class TrackingUpdateResult:
    point_m: tuple[float, float, float]
    velocity_mps: tuple[float, float, float]
    covariance_diag_m2: tuple[float, float, float]
    measurement_std_m: tuple[float, float, float]
    accepted: bool
    initialized: bool
    reset: bool
    normalized_innovation: float
    timestamp_s: float
    filter_id: str = "constant_velocity_kalman_v1"


def _clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def _finite_tuple(values: Sequence[float], length: int) -> bool:
    if len(values) != length:
        return False
    return all(math.isfinite(float(value)) for value in values)


def point_range_m(point_m: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in point_m))


def measurement_std_from_stereo(
    *,
    point_m: Sequence[float],
    focal_length_px: float,
    baseline_m: float,
    disparity_px: float,
    left_confidence: float | None = None,
    right_confidence: float | None = None,
    config: TrackingFilterConfig | None = None,
) -> tuple[float, float, float]:
    cfg = config or TrackingFilterConfig()
    if not _finite_tuple(point_m, 3):
        raise ValueError("point_m must contain three finite values")
    fx = float(focal_length_px)
    baseline = float(baseline_m)
    disparity = abs(float(disparity_px))
    if fx <= 0.0 or baseline <= 0.0 or disparity <= 0.0:
        raise ValueError("invalid stereo geometry for tracking covariance")

    confidences = [value for value in (left_confidence, right_confidence) if value is not None and math.isfinite(value)]
    confidence = sum(confidences) / len(confidences) if confidences else 0.5
    confidence = _clamp(confidence, 0.05, 1.0)
    sigma_px = 0.35 + 1.20 * (1.0 - confidence)
    sigma_disparity_px = math.sqrt(2.0) * sigma_px

    range_m = point_range_m(point_m)
    forward_m = max(1.0, abs(float(point_m[2])))
    lateral_std = _clamp(range_m / fx * sigma_px, cfg.min_lateral_std_m, cfg.max_lateral_std_m)
    forward_std = _clamp(
        forward_m * forward_m / (fx * baseline) * sigma_disparity_px,
        cfg.min_forward_std_m,
        cfg.max_forward_std_m,
    )
    return (lateral_std, lateral_std, forward_std)


def measurement_variance_from_stereo(**kwargs) -> tuple[float, float, float]:
    std = measurement_std_from_stereo(**kwargs)
    return (std[0] * std[0], std[1] * std[1], std[2] * std[2])


class ConstantVelocityKalmanFilter:
    def __init__(self, config: TrackingFilterConfig | None = None) -> None:
        self.config = config or TrackingFilterConfig()
        self._position = [0.0, 0.0, 0.0]
        self._velocity = [0.0, 0.0, 0.0]
        self._covariances = [
            [
                [self.config.initial_position_std_m**2, 0.0],
                [0.0, self.config.initial_velocity_std_mps**2],
            ]
            for _ in range(3)
        ]
        self._timestamp_s: float | None = None
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    def reset(self) -> None:
        self._position = [0.0, 0.0, 0.0]
        self._velocity = [0.0, 0.0, 0.0]
        self._covariances = [
            [
                [self.config.initial_position_std_m**2, 0.0],
                [0.0, self.config.initial_velocity_std_mps**2],
            ]
            for _ in range(3)
        ]
        self._timestamp_s = None
        self._initialized = False

    def _initialize(
        self,
        *,
        timestamp_s: float,
        point_m: Sequence[float],
        measurement_variance_m2: Sequence[float],
    ) -> TrackingUpdateResult:
        self._position = [float(point_m[0]), float(point_m[1]), float(point_m[2])]
        self._velocity = [0.0, 0.0, 0.0]
        self._covariances = [
            [
                [max(float(measurement_variance_m2[index]), 1.0e-9), 0.0],
                [0.0, self.config.initial_velocity_std_mps**2],
            ]
            for index in range(3)
        ]
        self._timestamp_s = float(timestamp_s)
        self._initialized = True
        return self._result(
            timestamp_s=timestamp_s,
            accepted=True,
            reset=True,
            normalized_innovation=0.0,
            measurement_variance_m2=measurement_variance_m2,
        )

    def update(
        self,
        *,
        timestamp_s: float,
        point_m: Sequence[float],
        measurement_variance_m2: Sequence[float],
    ) -> TrackingUpdateResult:
        if not _finite_tuple(point_m, 3):
            raise ValueError("point_m must contain three finite values")
        if not _finite_tuple(measurement_variance_m2, 3):
            raise ValueError("measurement_variance_m2 must contain three finite values")
        timestamp = float(timestamp_s)
        variances = tuple(max(float(value), 1.0e-9) for value in measurement_variance_m2)
        if not self._initialized or self._timestamp_s is None:
            return self._initialize(timestamp_s=timestamp, point_m=point_m, measurement_variance_m2=variances)

        dt = timestamp - self._timestamp_s
        if dt <= 0.0:
            dt = 1.0e-3
        if dt > self.config.reset_after_s:
            return self._initialize(timestamp_s=timestamp, point_m=point_m, measurement_variance_m2=variances)

        self._predict(dt)
        innovations = [float(point_m[index]) - self._position[index] for index in range(3)]
        innovation_variances = [self._covariances[index][0][0] + variances[index] for index in range(3)]
        normalized_innovation = math.sqrt(
            sum(
                innovations[index] * innovations[index] / max(innovation_variances[index], 1.0e-9)
                for index in range(3)
            )
        )
        accepted = normalized_innovation <= self.config.max_normalized_innovation
        if accepted:
            for index in range(3):
                self._update_axis(index, float(point_m[index]), variances[index])
        self._timestamp_s = timestamp
        return self._result(
            timestamp_s=timestamp,
            accepted=accepted,
            reset=False,
            normalized_innovation=normalized_innovation,
            measurement_variance_m2=variances,
        )

    def _predict(self, dt: float) -> None:
        accel_var = self.config.process_accel_std_mps2**2
        q00 = 0.25 * dt**4 * accel_var
        q01 = 0.5 * dt**3 * accel_var
        q11 = dt**2 * accel_var
        for index in range(3):
            self._position[index] += self._velocity[index] * dt
            p00, p01 = self._covariances[index][0]
            p10, p11 = self._covariances[index][1]
            self._covariances[index] = [
                [p00 + dt * (p10 + p01) + dt * dt * p11 + q00, p01 + dt * p11 + q01],
                [p10 + dt * p11 + q01, p11 + q11],
            ]

    def _update_axis(self, index: int, measurement: float, variance: float) -> None:
        p00, p01 = self._covariances[index][0]
        p10, p11 = self._covariances[index][1]
        innovation = measurement - self._position[index]
        s = max(p00 + variance, 1.0e-9)
        k0 = p00 / s
        k1 = p10 / s
        self._position[index] += k0 * innovation
        self._velocity[index] += k1 * innovation
        self._covariances[index] = [
            [(1.0 - k0) * p00, (1.0 - k0) * p01],
            [p10 - k1 * p00, p11 - k1 * p01],
        ]

    def _result(
        self,
        *,
        timestamp_s: float,
        accepted: bool,
        reset: bool,
        normalized_innovation: float,
        measurement_variance_m2: Sequence[float],
    ) -> TrackingUpdateResult:
        return TrackingUpdateResult(
            point_m=(self._position[0], self._position[1], self._position[2]),
            velocity_mps=(self._velocity[0], self._velocity[1], self._velocity[2]),
            covariance_diag_m2=(
                self._covariances[0][0][0],
                self._covariances[1][0][0],
                self._covariances[2][0][0],
            ),
            measurement_std_m=(
                math.sqrt(float(measurement_variance_m2[0])),
                math.sqrt(float(measurement_variance_m2[1])),
                math.sqrt(float(measurement_variance_m2[2])),
            ),
            accepted=accepted,
            initialized=self._initialized,
            reset=reset,
            normalized_innovation=float(normalized_innovation),
            timestamp_s=float(timestamp_s),
        )
