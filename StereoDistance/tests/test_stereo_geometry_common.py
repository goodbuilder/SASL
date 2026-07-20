from __future__ import annotations

from math import cos, radians, sin

import pytest

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_FRAME,
    PinholeIntrinsics,
    STEREO_CENTER_OPTICAL_FRAME,
    project_center_point_to_pixel,
    triangulate_pixels,
    triangulate_rays,
)


def _rotation_about_down(degrees: float):
    angle = radians(degrees)
    return (
        (cos(angle), 0.0, sin(angle)),
        (0.0, 1.0, 0.0),
        (-sin(angle), 0.0, cos(angle)),
    )


def test_full_pose_projection_round_trips_through_two_ray_midpoint() -> None:
    left_intrinsics = PinholeIntrinsics(920.0, 900.0, 631.0, 358.0)
    right_intrinsics = PinholeIntrinsics(880.0, 910.0, 646.0, 363.0)
    left_origin = (-0.15, 0.0, 0.0)
    right_origin = (0.15, 0.0, 0.0)
    left_rotation = _rotation_about_down(1.2)
    right_rotation = _rotation_about_down(-0.8)
    expected_center_rdf = (0.42, -0.18, 12.0)

    left_pixel = project_center_point_to_pixel(
        expected_center_rdf,
        camera_origin_center_rdf_m=left_origin,
        rotation_center_from_camera_rdf=left_rotation,
        intrinsics=left_intrinsics,
    )
    right_pixel = project_center_point_to_pixel(
        expected_center_rdf,
        camera_origin_center_rdf_m=right_origin,
        rotation_center_from_camera_rdf=right_rotation,
        intrinsics=right_intrinsics,
    )

    assert left_pixel is not None
    assert right_pixel is not None
    attempt = triangulate_pixels(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        left_intrinsics=left_intrinsics,
        right_intrinsics=right_intrinsics,
        left_origin_center_rdf_m=left_origin,
        right_origin_center_rdf_m=right_origin,
        rotation_center_from_left_camera_rdf=left_rotation,
        rotation_center_from_right_camera_rdf=right_rotation,
    )

    assert attempt.reason == "ok"
    assert attempt.result is not None
    assert attempt.result.point_center_rdf_m == pytest.approx(expected_center_rdf, abs=1.0e-9)
    assert attempt.result.point_center_frd_m == pytest.approx((12.0, 0.42, -0.18), abs=1.0e-9)
    assert attempt.result.closest_gap_m == pytest.approx(0.0, abs=1.0e-10)
    assert attempt.result.point_frame == STEREO_CENTER_OPTICAL_FRAME
    assert attempt.result.output_frame == BASELINE_CENTER_FRD_FRAME


def test_equal_horizontal_pixels_are_not_rejected_when_camera_rays_are_rotated() -> None:
    attempt = triangulate_pixels(
        left_pixel=(640.0, 360.0),
        right_pixel=(640.0, 360.0),
        left_intrinsics=PinholeIntrinsics(900.0, 900.0, 640.0, 360.0),
        right_intrinsics=PinholeIntrinsics(900.0, 900.0, 640.0, 360.0),
        left_origin_center_rdf_m=(-0.15, 0.0, 0.0),
        right_origin_center_rdf_m=(0.15, 0.0, 0.0),
        rotation_center_from_left_camera_rdf=_rotation_about_down(1.0),
        rotation_center_from_right_camera_rdf=_rotation_about_down(-1.0),
    )

    assert attempt.result is not None
    assert attempt.result.point_center_frd_m[0] == pytest.approx(8.593, abs=0.002)


def test_parallel_and_behind_camera_geometry_report_explicit_reasons() -> None:
    parallel = triangulate_rays(
        left_origin_center_rdf_m=(-0.15, 0.0, 0.0),
        right_origin_center_rdf_m=(0.15, 0.0, 0.0),
        left_ray_center_rdf=(0.0, 0.0, 1.0),
        right_ray_center_rdf=(0.0, 0.0, 1.0),
    )
    behind = triangulate_rays(
        left_origin_center_rdf_m=(-0.15, 0.0, 0.0),
        right_origin_center_rdf_m=(0.15, 0.0, 0.0),
        left_ray_center_rdf=(-0.1, 0.0, 1.0),
        right_ray_center_rdf=(0.1, 0.0, 1.0),
    )

    assert parallel.reason == "near_parallel_rays"
    assert behind.reason == "non_positive_depth"
