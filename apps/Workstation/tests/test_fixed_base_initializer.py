from __future__ import annotations

from Workstation.workstation_core.fixed_base_initializer import FixedBaseInitializer
from Workstation.workstation_core.gnss_operation import FixedBaseConfig, HeightType
from Workstation.workstation_core.gnss_profile import GnssCoordinateSample


def _sample(timestamp_s: float, *, fixed: bool) -> GnssCoordinateSample:
    return GnssCoordinateSample(
        latitude_deg=37.0,
        longitude_deg=127.0,
        ellipsoid_height_m=52.0,
        msl_altitude_m=48.0,
        height_type=HeightType.ELLIPSOID,
        timestamp_s=timestamp_s,
        fixed=fixed,
    )


def test_latest_non_fixed_sample_invalidates_fixed_base_candidate() -> None:
    samples = tuple(_sample(index * 10.0, fixed=True) for index in range(19)) + (
        _sample(190.0, fixed=False),
    )
    config = FixedBaseConfig(
        acquisition_duration_s=180.0,
        minimum_fixed_duration_s=60.0,
        minimum_fixed_ratio=0.95,
        maximum_horizontal_scatter_m=0.03,
        maximum_vertical_scatter_m=0.05,
        height_type=HeightType.ELLIPSOID,
    )

    validation = FixedBaseInitializer(config).validate(samples)

    assert validation.valid is False
    assert any("latest" in reason.lower() or "current" in reason.lower() for reason in validation.reasons)
    assert validation.profile is not None
    assert validation.profile.fixed_duration_s == 0.0


def test_fixed_duration_uses_current_contiguous_fixed_segment() -> None:
    samples = (
        _sample(0.0, fixed=True),
        _sample(100.0, fixed=True),
        _sample(110.0, fixed=False),
        _sample(170.0, fixed=True),
        _sample(190.0, fixed=True),
    )
    config = FixedBaseConfig(
        acquisition_duration_s=180.0,
        minimum_fixed_duration_s=60.0,
        minimum_fixed_ratio=0.5,
        maximum_horizontal_scatter_m=0.03,
        maximum_vertical_scatter_m=0.05,
        height_type=HeightType.ELLIPSOID,
    )

    validation = FixedBaseInitializer(config).validate(samples)

    assert validation.valid is False
    assert validation.profile is not None
    assert validation.profile.fixed_duration_s == 20.0
    assert any("fixed duration" in reason for reason in validation.reasons)
