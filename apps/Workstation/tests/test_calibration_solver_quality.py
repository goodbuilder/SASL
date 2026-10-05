from __future__ import annotations

import json
import math
from dataclasses import asdict, replace
from types import SimpleNamespace

import numpy as np
import pytest

from Workstation.workstation_core import calibration as calibration_module
from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    CheckerboardObservationCache,
    calibrate_camera_from_images,
    checkerboard_observation_from_points,
    solve_camera_from_observations,
)
from Workstation.workstation_core.calibration_quality import (
    evaluate_camera_calibration_quality,
)


cv2 = pytest.importorskip("cv2")

IMAGE_SIZE = (1280, 720)
PATTERN_SIZE = (9, 6)
SQUARE_SIZE_M = 0.03
KNOWN_CAMERA_MATRIX = np.asarray(
    (
        (900.0, 0.0, 640.0),
        (0.0, 910.0, 360.0),
        (0.0, 0.0, 1.0),
    ),
    dtype=np.float64,
)
OBJECT_POINTS_M = np.asarray(
    tuple(
        (column * SQUARE_SIZE_M, row * SQUARE_SIZE_M, 0.0)
        for row in range(PATTERN_SIZE[1])
        for column in range(PATTERN_SIZE[0])
    ),
    dtype=np.float64,
)


def _rotation_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """Independent Euler rotation oracle; the solver itself uses OpenCV."""

    rx, ry, rz = (math.radians(value) for value in (rx_deg, ry_deg, rz_deg))
    rotate_x = np.asarray(
        ((1.0, 0.0, 0.0), (0.0, math.cos(rx), -math.sin(rx)), (0.0, math.sin(rx), math.cos(rx)))
    )
    rotate_y = np.asarray(
        ((math.cos(ry), 0.0, math.sin(ry)), (0.0, 1.0, 0.0), (-math.sin(ry), 0.0, math.cos(ry)))
    )
    rotate_z = np.asarray(
        ((math.cos(rz), -math.sin(rz), 0.0), (math.sin(rz), math.cos(rz), 0.0), (0.0, 0.0, 1.0))
    )
    return rotate_z @ rotate_y @ rotate_x


def _project_without_distortion(
    rotation: np.ndarray,
    translation_m: tuple[float, float, float],
) -> np.ndarray:
    """Project RDF camera points with a hand-written zero-distortion pinhole model."""

    camera_points = (rotation @ OBJECT_POINTS_M.T).T + np.asarray(translation_m)
    assert np.all(camera_points[:, 2] > 0.0)
    return np.column_stack(
        (
            KNOWN_CAMERA_MATRIX[0, 0] * camera_points[:, 0] / camera_points[:, 2]
            + KNOWN_CAMERA_MATRIX[0, 2],
            KNOWN_CAMERA_MATRIX[1, 1] * camera_points[:, 1] / camera_points[:, 2]
            + KNOWN_CAMERA_MATRIX[1, 2],
        )
    )


def _synthetic_cache(*, diverse: bool, noise_seed: int = 42) -> CheckerboardObservationCache:
    generator = np.random.default_rng(noise_seed)
    observations = []
    if diverse:
        rotations = (
            (-24.0, -25.0, -12.0),
            (-20.0, 0.0, 0.0),
            (-24.0, 25.0, 12.0),
            (0.0, -25.0, 10.0),
            (0.0, 0.0, -10.0),
            (0.0, 25.0, 0.0),
            (24.0, -25.0, -12.0),
            (20.0, 0.0, 12.0),
            (24.0, 25.0, 0.0),
        )
        normalized_centers = (
            (0.18, 0.18),
            (0.50, 0.18),
            (0.82, 0.18),
            (0.18, 0.50),
            (0.50, 0.50),
            (0.82, 0.50),
            (0.18, 0.82),
            (0.50, 0.82),
            (0.82, 0.82),
        )
        board_center_m = np.asarray((0.12, 0.075, 0.0))
        for index in range(27):
            rotation = _rotation_matrix(*rotations[index % len(rotations)])
            center_x, center_y = normalized_centers[index % len(normalized_centers)]
            distance_m = (0.90, 1.15, 1.35)[index // len(normalized_centers)]
            desired_center_m = np.asarray(
                (
                    (center_x * IMAGE_SIZE[0] - KNOWN_CAMERA_MATRIX[0, 2])
                    * distance_m
                    / KNOWN_CAMERA_MATRIX[0, 0],
                    (center_y * IMAGE_SIZE[1] - KNOWN_CAMERA_MATRIX[1, 2])
                    * distance_m
                    / KNOWN_CAMERA_MATRIX[1, 1],
                    distance_m,
                )
            )
            translation = desired_center_m - rotation @ board_center_m
            image_points = _project_without_distortion(rotation, translation)
            image_points += generator.normal(0.0, 0.08, image_points.shape)
            assert image_points[:, 0].min() > 0.0
            assert image_points[:, 0].max() < IMAGE_SIZE[0]
            assert image_points[:, 1].min() > 0.0
            assert image_points[:, 1].max() < IMAGE_SIZE[1]
            observations.append(
                checkerboard_observation_from_points(
                    view_id=f"diverse-{index:03d}",
                    image_size=IMAGE_SIZE,
                    pattern_size=PATTERN_SIZE,
                    object_points_m=OBJECT_POINTS_M,
                    image_points_px=image_points,
                )
            )
    else:
        # All board normals remain aligned with the optical +Z axis. Screen
        # position, scale, and roll change, but yaw/pitch information is absent.
        for index in range(16):
            rotation = _rotation_matrix(0.0, 0.0, -3.0 + index % 7)
            translation = (
                -0.12 + (index % 4) * 0.07,
                -0.08 + (index // 4) * 0.04,
                1.20 + (index % 3) * 0.10,
            )
            image_points = _project_without_distortion(rotation, translation)
            image_points += generator.normal(0.0, 0.08, image_points.shape)
            observations.append(
                checkerboard_observation_from_points(
                    view_id=f"fronto-parallel-{index:03d}",
                    image_size=IMAGE_SIZE,
                    pattern_size=PATTERN_SIZE,
                    object_points_m=OBJECT_POINTS_M,
                    image_points_px=image_points,
                )
            )
    return CheckerboardObservationCache(
        image_size=IMAGE_SIZE,
        pattern_size=PATTERN_SIZE,
        observations=tuple(observations),
        input_count=len(observations),
    )


def test_diverse_synthetic_views_recover_known_intrinsics_and_emit_portable_diagnostics() -> None:
    result = solve_camera_from_observations(
        _synthetic_cache(diverse=True),
        subset_runs=8,
        subset_fraction=0.70,
        subset_seed=19,
    )

    matrix = np.asarray(result.intrinsics.camera_matrix)
    assert result.intrinsics.reprojection_error_px < 0.2
    assert matrix[0, 0] == pytest.approx(KNOWN_CAMERA_MATRIX[0, 0], abs=3.0)
    assert matrix[1, 1] == pytest.approx(KNOWN_CAMERA_MATRIX[1, 1], abs=3.0)
    assert matrix[0, 2] == pytest.approx(KNOWN_CAMERA_MATRIX[0, 2], abs=4.0)
    assert matrix[1, 2] == pytest.approx(KNOWN_CAMERA_MATRIX[1, 2], abs=4.0)
    assert result.calibration_flags == 0
    assert result.intrinsic_standard_deviations["cx_px"] is not None
    assert len(result.per_view_errors_px) == 27
    assert max(result.per_view_errors_px) < 0.2
    assert len(result.view_geometry) == 27
    assert set(result.view_geometry[0]) == {
        "view_id",
        "image_size",
        "corner_count",
        "center_normalized",
        "bbox_normalized",
        "bbox_extent_normalized",
        "coverage_fraction",
        "roll_deg",
        "top_bottom_ratio",
        "left_right_ratio",
        "scale",
    }
    assert len(result.subset_estimates) == 8
    assert result.subset_summary["cx_span_fraction_of_width"] < 0.01
    assert result.subset_summary["cy_span_fraction_of_height"] < 0.01
    assert result.subset_summary["fx_relative_span"] < 0.01
    assert result.subset_summary["fy_relative_span"] < 0.01
    assert result.quality_inputs()["subset_estimates"] == list(result.subset_estimates)
    quality = evaluate_camera_calibration_quality(**result.quality_inputs())
    assert quality.state == "PASS"
    assert quality.reason_codes == ()

    # The diagnostics cross worker/persistence boundaries as JSON-safe plain
    # data. In particular, no QImage or numpy array is retained in the cache.
    json.dumps(asdict(result), allow_nan=False)


def test_fronto_parallel_views_have_low_rms_but_unstable_subset_principal_point() -> None:
    cache = _synthetic_cache(diverse=False)
    first = solve_camera_from_observations(
        cache,
        subset_runs=10,
        subset_fraction=0.70,
        subset_seed=23,
    )
    repeated = solve_camera_from_observations(
        cache,
        subset_runs=10,
        subset_fraction=0.70,
        subset_seed=23,
    )

    assert first.intrinsics.reprojection_error_px < 0.2
    assert len(first.subset_estimates) >= 2
    assert (
        first.subset_summary["cx_span_fraction_of_width"] > 0.02
        or first.subset_summary["cy_span_fraction_of_height"] > 0.02
    )
    assert max(
        first.subset_summary["fx_relative_span"],
        first.subset_summary["fy_relative_span"],
    ) > 0.10
    assert first.subset_membership == repeated.subset_membership
    assert first.subset_estimates == repeated.subset_estimates
    assert not (first.calibration_flags & cv2.CALIB_FIX_PRINCIPAL_POINT)
    quality = evaluate_camera_calibration_quality(**first.quality_inputs())
    assert quality.state == "FAIL"
    assert (
        "subset_principal_x_span_exceeds_limit" in quality.reason_codes
        or "subset_principal_y_span_exceeds_limit" in quality.reason_codes
    )


def test_cached_solver_rejects_mixed_image_sizes_before_optimization() -> None:
    cache = _synthetic_cache(diverse=False)
    mixed_observations = list(cache.observations[:3])
    mixed_observations[1] = replace(mixed_observations[1], image_size=(640, 360))
    mixed_cache = replace(cache, observations=tuple(mixed_observations), input_count=3)

    with pytest.raises(ValueError, match="image size mismatch"):
        solve_camera_from_observations(mixed_cache, subset_runs=0)


def test_legacy_calibrate_camera_api_delegates_without_subset_cost(monkeypatch) -> None:
    expected = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 910.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0] * 5,
        reprojection_error_px=0.1,
        image_count=8,
    )
    calls = []

    def fake_rich_solver(paths, config, **kwargs):
        calls.append((paths, config, kwargs))
        return SimpleNamespace(intrinsics=expected)

    monkeypatch.setattr(calibration_module, "calibrate_camera_with_diagnostics", fake_rich_solver)
    config = CheckerboardConfig(9, 6, 0.03)

    result = calibrate_camera_from_images(["one.png", "two.png"], config)

    assert result is expected
    assert calls == [(["one.png", "two.png"], config, {"progress_callback": None, "subset_runs": 0})]
