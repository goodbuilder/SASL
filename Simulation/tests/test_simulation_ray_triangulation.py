from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from sasl_common.stereo_geometry import (
    BASELINE_CENTER_FRD_AXIS_CONTRACT,
    BASELINE_CENTER_FRD_FRAME,
    BASELINE_CENTER_FRD_VECTOR_CONTRACT,
    GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    triangulate_rays,
)
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


def test_known_point_and_skew_ray_midpoint() -> None:
    target = (8.0, -2.0, 30.0)
    left_origin = (0.0, 0.0, 0.0)
    right_origin = (2.0, 0.0, 0.0)

    def unit(origin):
        delta = tuple(target[i] - origin[i] for i in range(3))
        norm = math.sqrt(sum(value * value for value in delta))
        return tuple(value / norm for value in delta)

    result = triangulate_rays(
        left_origin_left_rdf_m=left_origin,
        right_origin_left_rdf_m=right_origin,
        left_ray_left_rdf=unit(left_origin),
        right_ray_left_rdf=unit(right_origin),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    ).result
    assert result is not None
    assert max(abs(a - b) for a, b in zip(result.midpoint_rdf_m, target)) < 1.0e-9
    assert result.closest_gap_m < 1.0e-9
    assert result.geometry_rank == 2
    assert result.computation_status == "computed"


def test_camera_swap_preserves_midpoint() -> None:
    first = triangulate_rays(
        right_origin_left_rdf_m=(2.0, 0.0, 0.0),
        left_ray_left_rdf=(0.2, 0.0, 1.0),
        right_ray_left_rdf=(-0.2, 0.0, 1.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    ).result
    second = triangulate_rays(
        left_origin_left_rdf_m=(2.0, 0.0, 0.0),
        right_origin_left_rdf_m=(0.0, 0.0, 0.0),
        left_ray_left_rdf=(-0.2, 0.0, 1.0),
        right_ray_left_rdf=(0.2, 0.0, 1.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    ).result
    assert first is not None and second is not None
    assert max(abs(a - b) for a, b in zip(first.midpoint_rdf_m, second.midpoint_rdf_m)) < 1.0e-9


def test_real_skew_rays_return_known_closest_points_and_swap_invariant() -> None:
    first = triangulate_rays(
        left_origin_left_rdf_m=(0.0, 0.0, 0.0),
        right_origin_left_rdf_m=(2.0, -3.0, 1.0),
        left_ray_left_rdf=(1.0, 0.0, 0.0),
        right_ray_left_rdf=(0.0, 1.0, 0.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        axis_contract=BASELINE_CENTER_FRD_AXIS_CONTRACT,
        vector_contract=BASELINE_CENTER_FRD_VECTOR_CONTRACT,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    second = triangulate_rays(
        left_origin_left_rdf_m=(2.0, -3.0, 1.0),
        right_origin_left_rdf_m=(0.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 1.0, 0.0),
        right_ray_left_rdf=(1.0, 0.0, 0.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        axis_contract=BASELINE_CENTER_FRD_AXIS_CONTRACT,
        vector_contract=BASELINE_CENTER_FRD_VECTOR_CONTRACT,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert first.result is not None and second.result is not None
    assert first.result.point_on_left_ray_m == pytest.approx((2.0, 0.0, 0.0), abs=1.0e-9)
    assert first.result.point_on_right_ray_m == pytest.approx((2.0, 0.0, 1.0), abs=1.0e-9)
    assert first.result.midpoint_m == pytest.approx((2.0, 0.0, 0.5), abs=1.0e-9)
    assert first.result.closest_gap_m == pytest.approx(1.0, abs=1.0e-9)
    assert first.result.left_depth_m == pytest.approx(2.0, abs=1.0e-9)
    assert first.result.right_depth_m == pytest.approx(3.0, abs=1.0e-9)
    assert second.result.midpoint_m == pytest.approx(first.result.midpoint_m, abs=1.0e-9)
    assert first.result.point_frame == BASELINE_CENTER_FRD_FRAME
    assert first.result.output_frame == BASELINE_CENTER_FRD_FRAME


@pytest.mark.parametrize("angle_rad", (1.0e-7, 1.0e-8, 1.0e-9, 1.0e-12))
def test_near_parallel_rays_are_stable_and_report_conditioning(angle_rad: float) -> None:
    attempt = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(math.sin(angle_rad), 0.0, math.cos(angle_rad)),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert attempt.reason in ("ok", "invalid_solution", "rank_deficient")
    assert "condition_number" in attempt.diagnostics
    if attempt.result is not None:
        assert attempt.result.computation_status == "computed"
        assert math.isfinite(attempt.result.condition_number)


def test_nonfinite_and_zero_ray_fail_without_exceptions() -> None:
    nonfinite = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(float("nan"), 0.0, 1.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    zero = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 0.0),
        right_ray_left_rdf=(0.0, 0.0, 1.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert nonfinite.result is None and nonfinite.reason == "invalid_input"
    assert zero.result is None and zero.reason == "invalid_input"


def test_status_policy_for_invalid_parallel_near_parallel_and_negative_depth() -> None:
    zero = triangulate_rays(
        right_origin_left_rdf_m=(0.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(0.0, 0.0, 1.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert zero.result is None and zero.reason == "zero_baseline"
    parallel = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(0.0, 0.0, 1.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert parallel.result is None and parallel.reason == "rank_deficient"
    near = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(1.0e-7, 0.0, 1.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert near.result is not None
    assert near.result.computation_status == "computed"
    assert near.result.condition_number > 1.0e6
    negative = triangulate_rays(
        right_origin_left_rdf_m=(0.3, 0.0, 0.0),
        left_ray_left_rdf=(-0.1, 0.0, 1.0),
        right_ray_left_rdf=(0.1, 0.0, 1.0),
        common_frame=BASELINE_CENTER_FRD_FRAME,
        geometry_policy=GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    )
    assert negative.result is not None
    assert negative.result.left_depth_m < 0.0
    assert negative.result.computation_status == "computed"


def test_simulation_pixel_path_reconstructs_known_baseline_point() -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    node.synthetic_detection_bbox_size_px = 32.0
    left, right = node._synthetic_detection_pair_from_rdf(
        right_m=7.0,
        down_m=4.0,
        forward_m=1000.0,
        width=1280,
        height=720,
    )
    raw = node._triangulate_center_pixels(left["cx"], left["cy"], right["cx"], right["cy"], 1280, 720)
    assert raw["computation_status"] == "computed"
    assert max(
        abs(raw[key] - expected) for key, expected in (("forward_m", 1000.0), ("right_m", 7.0), ("down_m", 4.0))
    ) < 1.0e-6


def test_ideal_reference_projection_raw_matches_same_baseline_ground_truth() -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.baseline_m = 25.0
    node.camera_height_m = 5.0
    node.horizontal_fov_rad = 0.154997710698
    node.synthetic_detection_bbox_size_px = 32.0
    node._ensure_camera_frames()
    world_point = node.baseline_center_frd.point_to_world((500.0, -3.0, 2.0))
    left, right = node._ideal_detection_pair_from_reference_world(world_point, 1280, 720)
    raw = node._triangulate_center_pixels(left["cx"], left["cy"], right["cx"], right["cy"], 1280, 720)
    assert raw["computation_status"] == "computed"
    assert max(
        abs(raw[key] - expected) for key, expected in (("forward_m", 500.0), ("right_m", -3.0), ("down_m", 2.0))
    ) < 1.0e-6
