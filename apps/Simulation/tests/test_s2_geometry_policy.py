from __future__ import annotations

import math

import pytest

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_FRAME,
    GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    GEOMETRY_POLICY_LEGACY_FORWARD_ONLY,
    LEFT_CAMERA_RDF_FRAME,
    triangulate_rays,
)


def test_common_frame_does_not_select_signed_depth_policy() -> None:
    arguments = dict(
        right_origin_left_rdf_m=(0.3, 0.0, 0.0),
        left_ray_left_rdf=(-0.1, 0.0, 1.0),
        right_ray_left_rdf=(0.1, 0.0, 1.0),
    )
    default = triangulate_rays(**arguments, common_frame=BASELINE_CENTER_FRD_FRAME)
    diagnostic = triangulate_rays(
        **arguments,
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert default.reason == "non_positive_depth"
    assert diagnostic.result is not None
    assert diagnostic.result.left_depth_m < 0.0
    assert diagnostic.result.output_frame == BASELINE_CENTER_FRD_FRAME


def test_default_policy_and_frame_preserve_stereodistance_contract() -> None:
    attempt = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(0.0, 0.0, 1.0),
    )
    assert attempt.reason == "near_parallel_rays"
    assert GEOMETRY_POLICY_LEGACY_FORWARD_ONLY == "legacy_forward_only"
    assert LEFT_CAMERA_RDF_FRAME == "left_camera_rdf"


@pytest.mark.parametrize("angle_rad", (1e-7, 1e-8, 1e-9, 1e-12))
def test_diagnostic_near_parallel_angles_are_computed(angle_rad: float) -> None:
    attempt = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(math.sin(angle_rad), 0.0, math.cos(angle_rad)),
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert attempt.result is not None
    assert attempt.result.computation_status == "computed"
    assert math.isfinite(attempt.result.condition_number)
    assert attempt.result.ray_angle_deg > 0.0
    assert math.radians(attempt.result.ray_angle_deg) == pytest.approx(angle_rad, rel=1e-12, abs=1e-18)


def test_exact_parallel_is_rank_deficient_only_in_diagnostic_policy() -> None:
    attempt = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(0.0, 0.0, 1.0),
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert attempt.result is None
    assert attempt.reason == "rank_deficient"
