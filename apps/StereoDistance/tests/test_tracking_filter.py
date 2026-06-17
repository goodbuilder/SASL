from __future__ import annotations

import math

from wiris_stereo_distance.tracking_filter import (
    ConstantVelocityKalmanFilter,
    TrackingFilterConfig,
    measurement_std_from_stereo,
)


def test_tracking_filter_initializes_from_first_measurement() -> None:
    tracker = ConstantVelocityKalmanFilter()

    result = tracker.update(
        timestamp_s=10.0,
        point_m=(1.0, 2.0, 100.0),
        measurement_variance_m2=(0.25, 0.25, 16.0),
    )

    assert result.reset
    assert result.accepted
    assert result.point_m == (1.0, 2.0, 100.0)
    assert result.velocity_mps == (0.0, 0.0, 0.0)


def test_tracking_filter_smooths_noisy_measurements() -> None:
    tracker = ConstantVelocityKalmanFilter(
        TrackingFilterConfig(
            process_accel_std_mps2=0.1,
            initial_velocity_std_mps=0.1,
            max_normalized_innovation=100.0,
        )
    )
    tracker.update(timestamp_s=0.0, point_m=(0.0, 0.0, 100.0), measurement_variance_m2=(1.0, 1.0, 1.0))

    result = tracker.update(
        timestamp_s=1.0,
        point_m=(0.0, 0.0, 130.0),
        measurement_variance_m2=(10000.0, 10000.0, 10000.0),
    )

    assert result.accepted
    assert 100.0 < result.point_m[2] < 130.0


def test_tracking_filter_rejects_large_outlier() -> None:
    tracker = ConstantVelocityKalmanFilter(TrackingFilterConfig(max_normalized_innovation=2.0))
    tracker.update(timestamp_s=0.0, point_m=(0.0, 0.0, 100.0), measurement_variance_m2=(0.25, 0.25, 1.0))

    result = tracker.update(
        timestamp_s=0.1,
        point_m=(1000.0, 1000.0, 5000.0),
        measurement_variance_m2=(0.25, 0.25, 1.0),
    )

    assert not result.accepted
    assert result.point_m[0] != 1000.0
    assert result.point_m[2] != 5000.0


def test_measurement_std_models_forward_as_disparity_sensitive() -> None:
    right_std, down_std, forward_std = measurement_std_from_stereo(
        point_m=(0.0, 0.0, 1852.0),
        focal_length_px=8241.0,
        baseline_m=25.0,
        disparity_px=111.0,
        left_confidence=0.7,
        right_confidence=0.7,
    )

    assert right_std == down_std
    assert forward_std > right_std
    assert math.isclose(right_std, 0.25)
    assert forward_std > 10.0
