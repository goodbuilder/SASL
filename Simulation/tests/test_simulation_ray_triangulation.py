from __future__ import annotations

import math
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from sasl_common.stereo_geometry import triangulate_rays
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
    ).result
    second = triangulate_rays(
        left_origin_left_rdf_m=(2.0, 0.0, 0.0),
        right_origin_left_rdf_m=(0.0, 0.0, 0.0),
        left_ray_left_rdf=(-0.2, 0.0, 1.0),
        right_ray_left_rdf=(0.2, 0.0, 1.0),
    ).result
    assert first is not None and second is not None
    assert max(abs(a - b) for a, b in zip(first.midpoint_rdf_m, second.midpoint_rdf_m)) < 1.0e-9


def test_status_policy_for_invalid_parallel_near_parallel_and_negative_depth() -> None:
    zero = triangulate_rays(
        right_origin_left_rdf_m=(0.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(0.0, 0.0, 1.0),
    )
    assert zero.result is None and zero.reason == "zero_baseline"
    parallel = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(0.0, 0.0, 1.0),
    )
    assert parallel.result is None and parallel.reason == "rank_deficient"
    near = triangulate_rays(
        right_origin_left_rdf_m=(1.0, 0.0, 0.0),
        left_ray_left_rdf=(0.0, 0.0, 1.0),
        right_ray_left_rdf=(1.0e-7, 0.0, 1.0),
    )
    assert near.result is not None
    assert near.result.computation_status == "computed"
    assert near.result.condition_number > 1.0e6
    negative = triangulate_rays(
        right_origin_left_rdf_m=(0.3, 0.0, 0.0),
        left_ray_left_rdf=(-0.1, 0.0, 1.0),
        right_ray_left_rdf=(0.1, 0.0, 1.0),
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
    world_point = node.baseline_center_frd.point_to_world((120.0, -3.0, 2.0))
    left, right = node._ideal_detection_pair_from_reference_world(world_point, 1280, 720)
    raw = node._triangulate_center_pixels(left["cx"], left["cy"], right["cx"], right["cy"], 1280, 720)
    assert raw["computation_status"] == "computed"
    assert max(
        abs(raw[key] - expected) for key, expected in (("forward_m", 120.0), ("right_m", -3.0), ("down_m", 2.0))
    ) < 1.0e-6
