"""All-sky frames still yield a Mountain suggestion, but flagged.

The percentile-based sky/ground split always labels part of a frame as
"ground", so the ridge bias used to filter cloud texture as if it were terrain
without saying so. ``detect_mountain`` now measures the absolute separability of
the two classes; when neither side has a separable ground class it skips the
ridge bias, proceeds with unfiltered far-field correspondences, and records
``filter_tier="unfiltered_all_sky"`` plus a warning so the operator knows the
points are sky/cloud texture rather than a ridge.
"""

from __future__ import annotations

import cv2
import numpy as np
import pytest

from Workstation.workstation_core.optical_pose_auto_select import (
    _estimate_ground_mask_with_metrics,
    detect_mountain,
)


def _sky_gradient(rng: np.random.Generator, noise: float = 4.0) -> np.ndarray:
    column = np.linspace(200.0, 120.0, 480)[:, None]
    image = np.repeat(column, 640, axis=1) + rng.normal(0.0, noise, (480, 640))
    return np.clip(image, 0, 255).astype(np.float32)


def _all_sky_clouds(rng: np.random.Generator, strength: float = 60.0) -> np.ndarray:
    base = _sky_gradient(rng, noise=6.0)
    blobs = np.zeros((480, 640), np.float32)
    for _ in range(40):
        center = (int(rng.integers(0, 640)), int(rng.integers(0, 480)))
        radius = int(rng.integers(20, 90))
        cv2.circle(blobs, center, radius, float(rng.uniform(-strength, strength)), -1)
    blobs = cv2.GaussianBlur(blobs, (0, 0), 15)
    return np.clip(base + blobs, 0, 255).astype(np.uint8)


def _sky_over_terrain(rng: np.random.Generator) -> np.ndarray:
    image = _sky_gradient(rng, noise=3.0)
    terrain = rng.normal(70.0, 25.0, (200, 640))
    image[280:] = terrain
    return np.clip(image, 0, 255).astype(np.uint8)


def _shift_right(image: np.ndarray, pixels: int = 6) -> np.ndarray:
    return np.roll(image, pixels, axis=1)


def test_all_sky_metrics_report_no_separable_ground() -> None:
    rng = np.random.default_rng(7)
    mask, metrics = _estimate_ground_mask_with_metrics(_all_sky_clouds(rng), cv2, np)
    assert mask is None
    assert metrics["ground_detectable"] is False
    assert metrics["reason"] == "no_separable_ground_class"
    assert metrics["texture_ratio"] < 2.0


def test_sky_over_terrain_metrics_report_ground() -> None:
    rng = np.random.default_rng(7)
    mask, metrics = _estimate_ground_mask_with_metrics(_sky_over_terrain(rng), cv2, np)
    assert mask is not None
    assert metrics["ground_detectable"] is True
    assert metrics["texture_ratio"] >= 2.0
    assert metrics["brightness_gap"] >= 0.12
    # Ground rows dominate the bottom of the frame, not the sky band.
    assert mask[400:, :].mean() > 0.8
    assert mask[:200, :].mean() < 0.5


def _all_sky_textured(rng: np.random.Generator) -> np.ndarray:
    """All-sky frame with enough cloud detail for SIFT to find correspondences."""

    base = _all_sky_clouds(rng, strength=70.0).astype(np.float32)
    detail = cv2.GaussianBlur(rng.normal(0.0, 30.0, (480, 640)).astype(np.float32), (0, 0), 3)
    return np.clip(base + detail, 0, 255).astype(np.uint8)


def test_all_sky_pair_still_yields_flagged_far_field_suggestion() -> None:
    rng = np.random.default_rng(11)
    left = _all_sky_textured(rng)
    right = _shift_right(left)
    result = detect_mountain(left, right, max_points=12)

    assert result.state in {"GOOD", "LOW_CONFIDENCE"}
    assert result.failure_reason != "no_ground_detected_all_sky"
    assert 4 <= len(result.left.points) <= 12
    assert result.metadata["filter_tier"] == "unfiltered_all_sky"
    separability = result.metadata["sky_ground_separability"]
    assert separability["left"]["ground_detectable"] is False
    assert separability["right"]["ground_detectable"] is False
    assert any("far-field sky/cloud" in warning for warning in result.warnings)
    assert any("far-field sky/cloud" in warning for warning in result.left.warnings)


def test_terrain_pair_records_filter_tier_and_is_not_flagged_all_sky() -> None:
    rng = np.random.default_rng(11)
    left = _sky_over_terrain(rng)
    right = _shift_right(left)
    result = detect_mountain(left, right, max_points=12)

    tier = result.metadata.get("filter_tier")
    assert tier in {"ridge_boundary", "ground_only", "unfiltered"} or result.state == "NO_DATA"
    assert tier != "unfiltered_all_sky"
    assert not any("far-field sky/cloud" in warning for warning in result.warnings)
    if result.state != "NO_DATA":
        assert result.metadata["sky_ground_separability"]["left"]["ground_detectable"] is True


def test_one_sided_all_sky_is_not_flagged_all_sky() -> None:
    rng = np.random.default_rng(11)
    left = _sky_over_terrain(rng)
    right = _all_sky_clouds(rng)
    result = detect_mountain(left, right, max_points=12)

    assert result.metadata.get("filter_tier") != "unfiltered_all_sky"


def test_far_field_only_skips_ridge_bias_even_with_terrain() -> None:
    rng = np.random.default_rng(11)
    left = _sky_over_terrain(rng)
    right = _shift_right(left)
    result = detect_mountain(left, right, max_points=12, far_field_only=True)

    assert result.metadata["filter_tier"] == "cloud_far_field"
    assert any("Cloud far-field mode" in warning for warning in result.warnings)
    # Separability is still measured and recorded for audit.
    assert result.metadata["sky_ground_separability"]["left"]["ground_detectable"] is True


@pytest.mark.parametrize("strength", [20.0, 60.0, 90.0])
def test_cloud_strength_sweep_stays_below_ground_texture_gate(strength: float) -> None:
    rng = np.random.default_rng(3)
    _mask, metrics = _estimate_ground_mask_with_metrics(_all_sky_clouds(rng, strength), cv2, np)
    assert metrics["ground_detectable"] is False
