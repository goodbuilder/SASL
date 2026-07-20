from __future__ import annotations

import math

import pytest

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_FRAME,
    STEREO_CENTER_OPTICAL_FRAME,
)
from stereo_ground_sim.ray_measurement import (
    camera_model_from_camera_info,
    empty_triangulation_status,
    gazebo_world_xyz_to_baseline_center_frd,
    intrinsics_from_camera_info_k,
    nominal_camera_origins_center_rdf,
    pixel_grid_contract_reason,
    project_nominal_stereo_point,
    triangulation_status,
    triangulate_nominal_stereo_pixels,
)
from stereo_ground_sim.calibration_csv_recorder import CSV_FIELDS, frd_array_to_fields
from stereo_ground_sim.external_bbox_publisher import detection_values_from_rdf
from stereo_ground_sim.operator_recorder import CSV_FIELDS as OPERATOR_CSV_FIELDS


def test_camera_info_k_and_nominal_pose_round_trip_to_public_frd() -> None:
    left = intrinsics_from_camera_info_k((920.0, 0.0, 631.0, 0.0, 900.0, 358.0, 0.0, 0.0, 1.0))
    right = intrinsics_from_camera_info_k((880.0, 0.0, 646.0, 0.0, 910.0, 363.0, 0.0, 0.0, 1.0))
    expected_rdf = (1.2, -0.4, 12.0)

    left_pixel, right_pixel = project_nominal_stereo_point(
        expected_rdf,
        left_intrinsics=left,
        right_intrinsics=right,
        baseline_m=0.3,
    )
    assert left_pixel is not None and right_pixel is not None

    attempt = triangulate_nominal_stereo_pixels(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        left_intrinsics=left,
        right_intrinsics=right,
        baseline_m=0.3,
    )

    assert attempt.reason == "ok"
    assert attempt.result is not None
    assert attempt.result.point_center_rdf_m == pytest.approx(expected_rdf, abs=1.0e-10)
    assert attempt.result.point_center_frd_m == pytest.approx((12.0, 1.2, -0.4), abs=1.0e-10)
    assert attempt.result.point_frame == STEREO_CENTER_OPTICAL_FRAME
    assert attempt.result.output_frame == BASELINE_CENTER_FRD_FRAME
    assert attempt.result.closest_gap_m < 1.0e-10
    status = triangulation_status(attempt)
    assert status["valid"] is True
    assert status["point_frame"] == STEREO_CENTER_OPTICAL_FRAME
    assert status["frame"] == BASELINE_CENTER_FRD_FRAME
    assert status["method"] == "two_ray_closest_midpoint"


def test_nominal_raw_estimator_contract_accepts_only_positive_baseline() -> None:
    assert nominal_camera_origins_center_rdf(0.3) == ((-0.15, 0.0, 0.0), (0.15, 0.0, 0.0))
    with pytest.raises(ValueError, match="positive"):
        nominal_camera_origins_center_rdf(0.0)


def test_camera_info_model_binds_zero_distortion_k_to_exact_pixel_grid() -> None:
    k = (900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0)
    left = camera_model_from_camera_info(
        k=k,
        width_px=1280,
        height_px=720,
        distortion=(0.0, -0.0, 0.0, 0.0, 0.0),
    )
    right = camera_model_from_camera_info(
        k=k,
        width_px=640,
        height_px=480,
        distortion=(),
    )

    assert left.pixel_grid == (1280, 720)
    assert right.pixel_grid == (640, 480)
    assert left.distortion_coefficients == (0.0, -0.0, 0.0, 0.0, 0.0)
    assert right.distortion_coefficients == ()
    assert pixel_grid_contract_reason(
        left_input_grid=(1280, 720),
        right_input_grid=(640, 480),
        left_camera=left,
        right_camera=right,
    ) is None
    assert pixel_grid_contract_reason(
        left_input_grid=(1280, 720),
        right_input_grid=(1280, 720),
        left_camera=left,
        right_camera=right,
    ) == "pixel_grid_mismatch"


def test_camera_info_model_rejects_nonzero_distortion_without_using_k() -> None:
    k = (900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0)

    with pytest.raises(ValueError, match="^unsupported_camera_distortion$"):
        camera_model_from_camera_info(
            k=k,
            width_px=1280,
            height_px=720,
            distortion=(1.0e-12, 0.0, 0.0, 0.0, 0.0),
        )


@pytest.mark.parametrize("world_y_m", [73.5, -73.5])
def test_gazebo_world_y_preserves_signed_baseline_center_forward(world_y_m: float) -> None:
    forward_m, right_m, down_m = gazebo_world_xyz_to_baseline_center_frd(
        world_x_m=-4.0,
        world_y_m=world_y_m,
        world_z_m=2.0,
        camera_height_m=10.0,
        target_origin_z_offset_m=0.5,
    )

    assert forward_m == world_y_m
    assert right_m == -4.0
    assert down_m == 7.5
    assert math.sqrt(forward_m**2 + right_m**2 + down_m**2) == pytest.approx(
        math.sqrt(world_y_m**2 + 4.0**2 + 7.5**2)
    )


def test_raw_csv_schema_uses_named_frd_and_separate_ray_frames() -> None:
    values = frd_array_to_fields((12.0, 1.2, -0.4, 12.066482), prefix="raw")

    assert values == {
        "raw_forward_m": 12.0,
        "raw_right_m": 1.2,
        "raw_down_m": -0.4,
        "raw_range_m": 12.066482,
    }
    assert "triangulation_point_frame" in CSV_FIELDS
    assert "triangulation_frame" in CSV_FIELDS
    assert not any(field.startswith("estimated_") for field in CSV_FIELDS)
    assert not any("correct" in field.lower() for field in CSV_FIELDS)
    assert len(OPERATOR_CSV_FIELDS) == len(set(OPERATOR_CSV_FIELDS))
    assert "triangulation_point_frame" in OPERATOR_CSV_FIELDS
    assert not any("correct" in field.lower() for field in OPERATOR_CSV_FIELDS)


def test_invalid_status_is_strict_json_safe() -> None:
    status = empty_triangulation_status("near_parallel_rays")

    assert status["valid"] is False
    assert status["reason"] == "near_parallel_rays"
    assert status["closest_gap_m"] is None
    assert status["point_frame"] == STEREO_CENTER_OPTICAL_FRAME
    assert status["frame"] == BASELINE_CENTER_FRD_FRAME


def test_external_bbox_generator_uses_full_projection_compatible_with_ray_core() -> None:
    width_px = 1280
    height_px = 720
    horizontal_fov_rad = 0.4
    fx = (width_px * 0.5) / math.tan(horizontal_fov_rad * 0.5)
    intrinsics = intrinsics_from_camera_info_k(
        (fx, 0.0, width_px * 0.5, 0.0, fx, height_px * 0.5, 0.0, 0.0, 1.0)
    )
    values = detection_values_from_rdf(
        right_m=1.1,
        down_m=-0.2,
        forward_m=30.0,
        width_px=width_px,
        height_px=height_px,
        horizontal_fov_rad=horizontal_fov_rad,
        baseline_m=0.8,
        bbox_size_px=20.0,
    )

    attempt = triangulate_nominal_stereo_pixels(
        left_pixel=(values[6], values[7]),
        right_pixel=(values[14], values[15]),
        left_intrinsics=intrinsics,
        right_intrinsics=intrinsics,
        baseline_m=0.8,
    )

    assert attempt.result is not None
    assert attempt.result.point_center_rdf_m == pytest.approx((1.1, -0.2, 30.0), abs=1.0e-10)
