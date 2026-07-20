import math

import pytest

from sasl_common.stereo_geometry import (
    LEFT_CAMERA_RDF_AXIS_CONTRACT,
    LEFT_CAMERA_RDF_FRAME,
    LEFT_CAMERA_RDF_VECTOR_CONTRACT,
    PinholeIntrinsics,
    TRIANGULATION_METHOD,
    normalize_vector,
    ray_from_pixel,
    triangulate_rays,
)


def test_pinhole_center_pixel_is_forward_optical_ray() -> None:
    intrinsics = PinholeIntrinsics(fx_px=900.0, fy_px=900.0, cx_px=640.0, cy_px=360.0)

    assert ray_from_pixel(640.0, 360.0, intrinsics) == (0.0, 0.0, 1.0)


def test_two_ray_midpoint_is_expressed_from_left_optical_center() -> None:
    target_left_rdf_m = (0.42, -0.18, 12.0)
    right_origin_left_rdf_m = (0.30, 0.0, 0.0)
    left_ray = normalize_vector(target_left_rdf_m)
    right_ray = normalize_vector(
        (
            target_left_rdf_m[0] - right_origin_left_rdf_m[0],
            target_left_rdf_m[1] - right_origin_left_rdf_m[1],
            target_left_rdf_m[2] - right_origin_left_rdf_m[2],
        )
    )

    attempt = triangulate_rays(
        right_origin_left_rdf_m=right_origin_left_rdf_m,
        left_ray_left_rdf=left_ray,
        right_ray_left_rdf=right_ray,
    )

    assert attempt.reason == "ok"
    assert attempt.result is not None
    assert attempt.result.point_left_rdf_m == pytest.approx(target_left_rdf_m, abs=1.0e-9)
    assert attempt.result.range_m == pytest.approx(math.sqrt(sum(value * value for value in target_left_rdf_m)))
    assert attempt.result.method == TRIANGULATION_METHOD
    assert attempt.result.point_frame == LEFT_CAMERA_RDF_FRAME
    assert attempt.result.output_frame == LEFT_CAMERA_RDF_FRAME
    assert attempt.result.axis_contract == LEFT_CAMERA_RDF_AXIS_CONTRACT
    assert attempt.result.vector_contract == LEFT_CAMERA_RDF_VECTOR_CONTRACT


def test_two_ray_midpoint_keeps_left_origin_lateral_offset() -> None:
    target_left_rdf_m = (0.15, 0.0, 5.4)
    right_origin_left_rdf_m = (0.30, 0.0, 0.0)

    attempt = triangulate_rays(
        right_origin_left_rdf_m=right_origin_left_rdf_m,
        left_ray_left_rdf=normalize_vector(target_left_rdf_m),
        right_ray_left_rdf=normalize_vector((-0.15, 0.0, 5.4)),
    )

    assert attempt.result is not None
    assert attempt.result.point_left_rdf_m == pytest.approx(target_left_rdf_m, abs=1.0e-9)
    assert attempt.result.point_left_rdf_m[0] == pytest.approx(0.15)


@pytest.mark.parametrize(
    ("right_origin", "left_ray", "right_ray", "reason"),
    (
        ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.1, 0.0, 1.0), "zero_baseline"),
        ((0.3, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), "near_parallel_rays"),
        ((0.3, 0.0, 0.0), (-0.1, 0.0, 1.0), (0.1, 0.0, 1.0), "non_positive_depth"),
    ),
)
def test_two_ray_invalid_geometry_fails_closed(right_origin, left_ray, right_ray, reason) -> None:
    attempt = triangulate_rays(
        right_origin_left_rdf_m=right_origin,
        left_ray_left_rdf=left_ray,
        right_ray_left_rdf=right_ray,
    )

    assert attempt.result is None
    assert attempt.reason == reason
