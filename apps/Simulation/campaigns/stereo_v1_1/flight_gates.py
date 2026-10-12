"""Pre-science frame-origin and five-second settle gates for SITL flights."""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Sequence

from .trajectory_contract import LocalNed, WorldEnu, observed_origin_world_enu


@dataclass(frozen=True)
class OriginObservation:
    gt_world_enu_m: WorldEnu
    px4_local_ned_m: LocalNed
    gt_receive_monotonic_ns: int
    px4_receive_monotonic_ns: int
    simulation_epoch: int
    xy_reset_counter: int
    z_reset_counter: int


def fit_observed_origin(
    samples: Sequence[OriginObservation],
    *,
    min_samples: int = 20,
    max_receive_skew_ns: int = 100_000_000,
    max_residual_m: float = 0.5,
) -> WorldEnu:
    """Freeze an observed ENU origin from one stable PX4/Gazebo epoch.

    GT and local positions are paired by callback receive monotonic time,
    not by their incompatible source clock domains. The caller must separately
    ensure both messages are current and the vehicle is stationary. All inputs
    use metres, and no configured spawn/home coordinate is substituted.
    """
    if min_samples < 2 or max_receive_skew_ns < 0 or not math.isfinite(max_residual_m) or max_residual_m <= 0:
        raise ValueError("invalid observed-origin gate settings")
    accepted = [
        sample for sample in samples
        if abs(sample.gt_receive_monotonic_ns - sample.px4_receive_monotonic_ns)
        <= max_receive_skew_ns
    ]
    if len(accepted) < min_samples:
        raise ValueError("insufficient synchronized origin observations")
    if len({sample.simulation_epoch for sample in accepted}) != 1:
        raise ValueError("origin observations cross simulation epochs")
    if len({(sample.xy_reset_counter, sample.z_reset_counter) for sample in accepted}) != 1:
        raise ValueError("PX4 local position reset during origin observations")
    origins = [
        observed_origin_world_enu(sample.gt_world_enu_m, sample.px4_local_ned_m)
        for sample in accepted
    ]
    center = WorldEnu(
        statistics.median(origin.east_m for origin in origins),
        statistics.median(origin.north_m for origin in origins),
        statistics.median(origin.up_m for origin in origins),
    )
    for origin in origins:
        if math.dist(center.as_xyz(), origin.as_xyz()) > max_residual_m:
            raise ValueError("observed origin is inconsistent across samples")
    return center


@dataclass(frozen=True)
class SettleSample:
    simulation_time_ns: int
    speed_mps: float
    position_error_m: float
    heading_error_deg: float
    simulation_epoch: int
    xy_reset_counter: int
    z_reset_counter: int
    world_position_error_m: float | None = None


def science_start_ready(
    samples: Sequence[SettleSample],
    *,
    duration_ns: int = 5_000_000_000,
    max_sample_gap_ns: int = 200_000_000,
    max_speed_mps: float = 0.3,
    max_position_error_m: float = 0.5,
    max_heading_error_deg: float = 3.0,
) -> bool:
    """Check continuous five-second speed/position/heading stability.

    Speed <0.3 m/s, reference-point position error <0.5 m and commanded
    180-degree heading error <3 deg must hold throughout the interval.
    Missing time, epoch changes or PX4 estimator resets prevent readiness.
    """
    if (duration_ns <= 0 or max_sample_gap_ns <= 0 or not samples
            or not all(math.isfinite(value) and value > 0 for value in
                       (max_speed_mps, max_position_error_m, max_heading_error_deg))):
        return False
    times = [sample.simulation_time_ns for sample in samples]
    if any(right <= left for left, right in zip(times, times[1:])):
        return False
    cutoff = times[-1] - duration_ns
    previous = [sample for sample in samples if sample.simulation_time_ns <= cutoff]
    if not previous:
        return False
    window = [previous[-1]] + [
        sample for sample in samples if sample.simulation_time_ns > cutoff
    ]
    if times[-1] - window[0].simulation_time_ns < duration_ns:
        return False
    if len(window) < 2 or window[1].simulation_time_ns - cutoff > max_sample_gap_ns:
        return False
    if any(right.simulation_time_ns - left.simulation_time_ns > max_sample_gap_ns
           for left, right in zip(window, window[1:])):
        return False
    if len({(sample.simulation_epoch, sample.xy_reset_counter, sample.z_reset_counter)
            for sample in window}) != 1:
        return False
    return all(
        math.isfinite(sample.speed_mps)
        and math.isfinite(sample.position_error_m)
        and math.isfinite(sample.heading_error_deg)
        and sample.speed_mps < max_speed_mps
        and sample.position_error_m < max_position_error_m
        and (sample.world_position_error_m is None
             or (math.isfinite(sample.world_position_error_m)
                 and sample.world_position_error_m < max_position_error_m))
        and abs(sample.heading_error_deg) < max_heading_error_deg
        for sample in window
    )
