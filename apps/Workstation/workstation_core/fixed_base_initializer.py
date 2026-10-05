"""Network-assisted fixed-base coordinate acquisition and validation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from Workstation.workstation_core.gnss_operation import FixedBaseConfig, HeightType
from Workstation.workstation_core.gnss_profile import GnssCoordinateProfile, GnssCoordinateSample, build_coordinate_profile


@dataclass(frozen=True)
class FixedBaseValidation:
    valid: bool
    reasons: tuple[str, ...]
    profile: GnssCoordinateProfile | None


class FixedBaseInitializer:
    """Collect multiple fixed samples; never promotes a single sample."""

    def __init__(self, config: FixedBaseConfig) -> None:
        self.config = config

    def validate(
        self,
        samples: Iterable[GnssCoordinateSample],
        *,
        acquisition_start_utc: str = "",
        acquisition_end_utc: str = "",
        source_caster: str = "",
        source_mountpoint: str = "",
        coordinate_source_type: str = "NETWORK_ASSISTED_FIXED_BASE",
        antenna_height_m: float | None = None,
        antenna_reference_point: str = "",
        datum: str = "WGS84",
        reference_epoch: str | None = None,
    ) -> FixedBaseValidation:
        rows = tuple(samples)
        reasons: list[str] = []
        if not rows:
            return FixedBaseValidation(False, ("no coordinate samples",), None)
        latest = max(rows, key=lambda row: row.timestamp_s)
        if not latest.fixed:
            reasons.append("latest coordinate sample is not RTK Fixed")
        try:
            profile = build_coordinate_profile(
                rows,
                acquisition_start_utc=acquisition_start_utc,
                acquisition_end_utc=acquisition_end_utc,
                source_caster=source_caster,
                source_mountpoint=source_mountpoint,
                coordinate_source_type=coordinate_source_type,
                antenna_height_m=antenna_height_m,
                antenna_reference_point=antenna_reference_point,
                datum=datum,
                reference_epoch=reference_epoch,
            )
        except ValueError as exc:
            return FixedBaseValidation(False, (str(exc),), None)
        if profile.collection_duration_s < self.config.acquisition_duration_s:
            reasons.append(
                f"collection duration {profile.collection_duration_s:.3f}s < {self.config.acquisition_duration_s:.3f}s"
            )
        if profile.fixed_duration_s < self.config.minimum_fixed_duration_s:
            reasons.append(
                f"fixed duration {profile.fixed_duration_s:.3f}s < {self.config.minimum_fixed_duration_s:.3f}s"
            )
        if profile.fixed_ratio < self.config.minimum_fixed_ratio:
            reasons.append(
                f"fixed ratio {profile.fixed_ratio:.4f} < {self.config.minimum_fixed_ratio:.4f}"
            )
        if profile.horizontal_scatter_m > self.config.maximum_horizontal_scatter_m:
            reasons.append(
                f"horizontal scatter {profile.horizontal_scatter_m:.4f}m > {self.config.maximum_horizontal_scatter_m:.4f}m"
            )
        if profile.vertical_scatter_m > self.config.maximum_vertical_scatter_m:
            reasons.append(
                f"vertical scatter {profile.vertical_scatter_m:.4f}m > {self.config.maximum_vertical_scatter_m:.4f}m"
            )
        if profile.height_type == HeightType.UNKNOWN:
            reasons.append("height type is UNKNOWN")
        return FixedBaseValidation(not reasons, tuple(reasons), profile if not reasons else profile)
