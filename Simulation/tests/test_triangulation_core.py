from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.coordinate_frames import (  # noqa: E402
    FrameDirection,
    FrameId,
    FramePoint,
    Ray3D,
    RigidTransform3D,
    rotation_matrix_from_quaternion_wxyz,
    transform_ray,
)
from stereo_ground_sim.triangulation_core import (  # noqa: E402
    PinholeIntrinsics,
    TriangulationStatus,
    camera_ray_from_pixel,
    closest_points_between_rays,
    triangulate_rays_baseline_center_frd,
)


def ray(origin, direction, frame: FrameId = FrameId.BASELINE_CENTER_FRD) -> Ray3D:
    return Ray3D(FramePoint(tuple(origin), frame), FrameDirection.from_vector(tuple(direction), frame))


def norm(values) -> float:
    return float(np.linalg.norm(np.asarray(values, dtype=float)))


class TriangulationCoreTests(unittest.TestCase):
    def intrinsics(self) -> PinholeIntrinsics:
        return PinholeIntrinsics(
            width_px=1280,
            height_px=720,
            fx_px=900.0,
            fy_px=900.0,
            cx_px=640.0,
            cy_px=360.0,
        )

    def project_baseline_point_to_pixel(self, point_frd, camera_origin_frd, intrinsics):
        relative = np.asarray(point_frd, dtype=float) - np.asarray(camera_origin_frd, dtype=float)
        forward, right, down = relative
        return (
            intrinsics.cx_px + right * intrinsics.fx_px / forward,
            intrinsics.cy_px + down * intrinsics.fy_px / forward,
        )

    def test_camera_ray_from_pixel_center_is_forward_rdf(self) -> None:
        ray3d = camera_ray_from_pixel(640.0, 360.0, self.intrinsics())

        self.assertEqual(ray3d.frame, FrameId.CAMERA_OPTICAL_RDF)
        self.assertEqual(ray3d.origin.xyz_m, (0.0, 0.0, 0.0))
        self.assertEqual(ray3d.direction.xyz_unit, (0.0, 0.0, 1.0))

    def test_ideal_parallel_stereo_geometry(self) -> None:
        baseline_m = 25.0
        point = np.array((1000.0, 7.0, 4.0), dtype=float)
        left_origin = np.array((0.0, -baseline_m / 2.0, 0.0), dtype=float)
        right_origin = np.array((0.0, baseline_m / 2.0, 0.0), dtype=float)

        result = triangulate_rays_baseline_center_frd(
            ray(left_origin, point - left_origin),
            ray(right_origin, point - right_origin),
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.status, TriangulationStatus.OK)
        self.assertLess(norm(np.asarray(result.point_m) - point), 1.0e-9)
        self.assertLess(result.ray_gap_m, 1.0e-9)

    def test_exact_ray_intersection(self) -> None:
        target = np.array((10.0, 2.0, -1.0), dtype=float)
        result = triangulate_rays_baseline_center_frd(
            ray((0.0, 0.0, 0.0), target),
            ray((0.0, 4.0, 0.0), target - np.array((0.0, 4.0, 0.0))),
        )

        self.assertTrue(result.valid)
        self.assertLess(norm(np.asarray(result.point_m) - target), 1.0e-9)
        self.assertLess(result.ray_gap_m, 1.0e-9)

    def test_skew_ray_midpoint_and_gap(self) -> None:
        result = triangulate_rays_baseline_center_frd(
            ray((0.0, -1.0, 0.0), (10.0, 1.0, 0.2)),
            ray((0.0, 1.0, 0.5), (10.0, -1.0, -0.1)),
        )

        self.assertTrue(result.valid)
        midpoint = 0.5 * (np.asarray(result.closest_point_left_m) + np.asarray(result.closest_point_right_m))
        self.assertLess(norm(midpoint - np.asarray(result.point_m)), 1.0e-12)
        self.assertGreater(result.ray_gap_m, 0.0)

    def test_arbitrary_rotated_and_translated_camera_poses(self) -> None:
        target = np.array((120.0, 6.0, -3.0), dtype=float)
        left_origin = np.array((0.0, -12.0, 1.0), dtype=float)
        right_origin = np.array((2.0, 13.0, -0.5), dtype=float)
        left_yaw_rad = math.radians(12.0)
        right_yaw_rad = math.radians(-8.0)
        rotation_left_b_a = rotation_matrix_from_quaternion_wxyz(
            (math.cos(left_yaw_rad / 2.0), 0.0, 0.0, math.sin(left_yaw_rad / 2.0))
        )
        rotation_right_b_a = rotation_matrix_from_quaternion_wxyz(
            (math.cos(right_yaw_rad / 2.0), 0.0, 0.0, math.sin(right_yaw_rad / 2.0))
        )

        def camera_ray_in_baseline(camera_origin_b, rotation_b_a):
            r_b_a = np.asarray(rotation_b_a, dtype=float)
            r_a_b = r_b_a.T
            origin_b_in_a = r_a_b @ (-camera_origin_b)
            direction_a = r_a_b @ (target - camera_origin_b)
            transform = RigidTransform3D(
                source_frame=FrameId.CAMERA_OPTICAL_RDF,
                target_frame=FrameId.BASELINE_CENTER_FRD,
                rotation_target_source=rotation_b_a,
                origin_target_in_source_m=tuple(origin_b_in_a),
            )
            local_ray = ray((0.0, 0.0, 0.0), direction_a, FrameId.CAMERA_OPTICAL_RDF)
            return transform_ray(local_ray, transform)

        result = triangulate_rays_baseline_center_frd(
            camera_ray_in_baseline(left_origin, rotation_left_b_a),
            camera_ray_in_baseline(right_origin, rotation_right_b_a),
        )

        self.assertTrue(result.valid)
        self.assertLess(norm(np.asarray(result.point_m) - target), 1.0e-9)

    def test_numerical_parallel_rays(self) -> None:
        result = triangulate_rays_baseline_center_frd(
            ray((0.0, -1.0, 0.0), (1.0, 0.0, 0.0)),
            ray((0.0, 1.0, 0.0), (1.0, 1.0e-14, 0.0)),
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, TriangulationStatus.PARALLEL_RAYS)
        self.assertAlmostEqual(result.ray_gap_m, 2.0)

    def test_explicit_near_parallel_rejection(self) -> None:
        result = triangulate_rays_baseline_center_frd(
            ray((0.0, -1.0, 0.0), (1.0, 0.0, 0.0)),
            ray((0.0, 1.0, 0.0), (1.0, -1.0e-5, 0.0)),
            parallel_denominator_tolerance=1.0e-16,
            min_intersection_angle_rad=1.0e-3,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, TriangulationStatus.NEAR_PARALLEL_RAYS)
        self.assertIsNotNone(result.ray_gap_m)

    def test_anti_parallel_rays(self) -> None:
        result = triangulate_rays_baseline_center_frd(
            ray((0.0, -1.0, 0.0), (1.0, 0.0, 0.0)),
            ray((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)),
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, TriangulationStatus.PARALLEL_RAYS)
        self.assertAlmostEqual(result.intersection_angle_rad, math.pi)

    def test_zero_baseline(self) -> None:
        result = triangulate_rays_baseline_center_frd(
            ray((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            ray((0.0, 0.0, 0.0), (1.0, 0.1, 0.0)),
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, TriangulationStatus.ZERO_BASELINE)
        self.assertEqual(result.ray_gap_m, 0.0)

    def test_negative_left_parameter(self) -> None:
        result = triangulate_rays_baseline_center_frd(
            ray((2.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            ray((0.0, -1.0, 0.0), (1.0, 1.0, 0.0)),
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, TriangulationStatus.NON_POSITIVE_DEPTH)
        self.assertLessEqual(result.left_ray_parameter_m, 0.0)

    def test_negative_right_parameter(self) -> None:
        result = triangulate_rays_baseline_center_frd(
            ray((0.0, -1.0, 0.0), (1.0, 1.0, 0.0)),
            ray((2.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, TriangulationStatus.NON_POSITIVE_DEPTH)
        self.assertLessEqual(result.right_ray_parameter_m, 0.0)

    def test_both_parameters_negative(self) -> None:
        result = triangulate_rays_baseline_center_frd(
            ray((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            ray((0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, TriangulationStatus.NON_POSITIVE_DEPTH)
        self.assertLessEqual(result.left_ray_parameter_m, 0.0)
        self.assertLessEqual(result.right_ray_parameter_m, 0.0)

    def test_zero_direction_construction_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "zero vector"):
            ray((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def test_non_finite_input_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            camera_ray_from_pixel(math.nan, 360.0, self.intrinsics())
        with self.assertRaisesRegex(ValueError, "finite"):
            PinholeIntrinsics(1280, 720, math.inf, 900.0, 640.0, 360.0)

    def test_frame_mismatch_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "baseline_center_frd"):
            triangulate_rays_baseline_center_frd(
                ray((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF),
                ray((0.0, 1.0, 0.0), (1.0, -1.0, 0.0)),
            )
        with self.assertRaisesRegex(ValueError, "same frame"):
            closest_points_between_rays(
                ray((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF),
                ray((0.0, 1.0, 0.0), (1.0, -1.0, 0.0), FrameId.BASELINE_CENTER_FRD),
            )

    def test_left_right_swap_symmetry(self) -> None:
        left = ray((0.0, -1.0, 0.0), (10.0, 1.0, 0.5))
        right = ray((0.0, 1.0, 0.3), (10.0, -1.0, -0.2))

        result = triangulate_rays_baseline_center_frd(left, right)
        swapped = triangulate_rays_baseline_center_frd(right, left)

        self.assertTrue(result.valid)
        self.assertTrue(swapped.valid)
        self.assertLess(norm(np.asarray(result.point_m) - np.asarray(swapped.point_m)), 1.0e-12)
        self.assertAlmostEqual(result.range_m, swapped.range_m, places=12)
        self.assertAlmostEqual(result.ray_gap_m, swapped.ray_gap_m, places=12)
        self.assertAlmostEqual(result.intersection_angle_rad, swapped.intersection_angle_rad, places=12)

    def test_known_point_projection_and_reconstruction(self) -> None:
        intrinsics = self.intrinsics()
        baseline_m = 25.0
        point = np.array((1852.0, 20.0, -6.0), dtype=float)
        left_origin = np.array((0.0, -baseline_m / 2.0, 0.0), dtype=float)
        right_origin = np.array((0.0, baseline_m / 2.0, 0.0), dtype=float)
        left_u, left_v = self.project_baseline_point_to_pixel(point, left_origin, intrinsics)
        right_u, right_v = self.project_baseline_point_to_pixel(point, right_origin, intrinsics)
        rdf_to_frd_left = RigidTransform3D(
            source_frame=FrameId.CAMERA_OPTICAL_RDF,
            target_frame=FrameId.BASELINE_CENTER_FRD,
            rotation_target_source=((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            origin_target_in_source_m=(baseline_m / 2.0, 0.0, 0.0),
        )
        rdf_to_frd_right = RigidTransform3D(
            source_frame=FrameId.CAMERA_OPTICAL_RDF,
            target_frame=FrameId.BASELINE_CENTER_FRD,
            rotation_target_source=((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            origin_target_in_source_m=(-baseline_m / 2.0, 0.0, 0.0),
        )

        result = triangulate_rays_baseline_center_frd(
            transform_ray(camera_ray_from_pixel(left_u, left_v, intrinsics), rdf_to_frd_left),
            transform_ray(camera_ray_from_pixel(right_u, right_v, intrinsics), rdf_to_frd_right),
        )

        self.assertTrue(result.valid)
        self.assertLess(norm(np.asarray(result.point_m) - point), 1.0e-6)
        self.assertLess(result.ray_gap_m, 1.0e-9)

    def test_disparity_formula_comparison(self) -> None:
        intrinsics = self.intrinsics()
        baseline_m = 25.0
        for forward_m in (300.0, 1000.0, 1852.0):
            disparity_px = intrinsics.fx_px * baseline_m / forward_m
            left_u = intrinsics.cx_px + disparity_px * 0.5
            right_u = intrinsics.cx_px - disparity_px * 0.5
            left_origin = (0.0, -baseline_m / 2.0, 0.0)
            right_origin = (0.0, baseline_m / 2.0, 0.0)
            rdf_to_frd_left = RigidTransform3D(
                source_frame=FrameId.CAMERA_OPTICAL_RDF,
                target_frame=FrameId.BASELINE_CENTER_FRD,
                rotation_target_source=((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                origin_target_in_source_m=(baseline_m / 2.0, 0.0, 0.0),
            )
            rdf_to_frd_right = RigidTransform3D(
                source_frame=FrameId.CAMERA_OPTICAL_RDF,
                target_frame=FrameId.BASELINE_CENTER_FRD,
                rotation_target_source=((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                origin_target_in_source_m=(-baseline_m / 2.0, 0.0, 0.0),
            )

            result = triangulate_rays_baseline_center_frd(
                transform_ray(camera_ray_from_pixel(left_u, intrinsics.cy_px, intrinsics), rdf_to_frd_left),
                transform_ray(camera_ray_from_pixel(right_u, intrinsics.cy_px, intrinsics), rdf_to_frd_right),
            )

            self.assertTrue(result.valid)
            self.assertLess(abs(result.point_m[0] - forward_m), 1.0e-6)
            self.assertLess(abs(result.range_m - forward_m), 1.0e-6)


if __name__ == "__main__":
    unittest.main()
