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
    FrameId,
    FramePoint,
    RigidTransform3D,
    rotation_matrix_from_quaternion_wxyz,
    transform_point,
)
from stereo_ground_sim.ideal_projection import (  # noqa: E402
    CameraProjectionResult,
    IdealStereoProjection,
    ProjectionStatus,
    StereoProjectionStatus,
    project_nominal_stereo_point,
    project_point_to_camera_pixel,
)
from stereo_ground_sim.stereo_triangulation_adapter import (  # noqa: E402
    nominal_camera_transforms,
    nominal_stereo_rays_from_centers,
    pinhole_intrinsics_from_horizontal_fov,
    triangulate_nominal_stereo_centers,
)
from stereo_ground_sim.triangulation_core import PinholeIntrinsics, TriangulationStatus  # noqa: E402


WIDTH_PX = 1280
HEIGHT_PX = 720
HORIZONTAL_FOV_RAD = 0.154997710698
BASELINE_M = 25.0
PIXEL_TOL = 1.0e-9


def norm(values) -> float:
    return float(np.linalg.norm(np.asarray(values, dtype=float)))


def point_frd(values) -> FramePoint:
    return FramePoint(tuple(float(value) for value in values), FrameId.BASELINE_CENTER_FRD)


def intrinsics() -> PinholeIntrinsics:
    return pinhole_intrinsics_from_horizontal_fov(
        width_px=WIDTH_PX,
        height_px=HEIGHT_PX,
        horizontal_fov_rad=HORIZONTAL_FOV_RAD,
    )


def direct_nominal_pixels(point_baseline_frd_m, *, baseline_m=BASELINE_M):
    forward_m, right_m, down_m = (float(value) for value in point_baseline_frd_m)
    camera = intrinsics()
    left_u = camera.cx_px + camera.fx_px * (right_m + baseline_m * 0.5) / forward_m
    right_u = camera.cx_px + camera.fx_px * (right_m - baseline_m * 0.5) / forward_m
    v = camera.cy_px + camera.fy_px * down_m / forward_m
    return left_u, v, right_u, v


def assert_almost(testcase: unittest.TestCase, actual: float, expected: float, delta: float = PIXEL_TOL) -> None:
    testcase.assertAlmostEqual(float(actual), float(expected), delta=delta)


class IdealProjectionTests(unittest.TestCase):
    def test_known_numeric_fixture(self) -> None:
        camera = intrinsics()
        projection = project_nominal_stereo_point(
            point_frd((1852.0, 20.0, -6.0)),
            camera,
            BASELINE_M,
        )

        self.assertIsInstance(projection, IdealStereoProjection)
        self.assertIsInstance(projection.left, CameraProjectionResult)
        self.assertTrue(projection.valid)
        self.assertEqual(projection.status, StereoProjectionStatus.OK)
        assert_almost(self, camera.fx_px, 8241.646774127139)
        assert_almost(self, camera.fy_px, 8241.646774127139)
        assert_almost(self, projection.left.u_px, 784.6293305394881)
        assert_almost(self, projection.left.v_px, 333.2992005157868)
        assert_almost(self, projection.right.u_px, 673.3759993552665)
        assert_almost(self, projection.right.v_px, 333.2992005157868)
        assert_almost(self, projection.signed_disparity_px, 111.25333118422168)
        assert_almost(self, projection.vertical_disparity_px, 0.0)

    def test_centerline_sign_fixture(self) -> None:
        camera = intrinsics()
        projection = project_nominal_stereo_point(point_frd((1852.0, 0.0, 0.0)), camera, BASELINE_M)

        assert_almost(self, projection.left.u_px, 695.6266655921108)
        assert_almost(self, projection.right.u_px, 584.3733344078892)
        self.assertGreater(projection.left.u_px, camera.cx_px)
        self.assertLess(projection.right.u_px, camera.cx_px)
        self.assertGreater(projection.signed_disparity_px, 0.0)
        assert_almost(self, projection.vertical_disparity_px, 0.0)

    def test_round_trip_nominal_points(self) -> None:
        camera = intrinsics()
        points = (
            (300.0, 0.0, 0.0),
            (1000.0, 0.0, 0.0),
            (1852.0, 0.0, 0.0),
            (2000.0, 0.0, 0.0),
            (1000.0, 7.0, 0.0),
            (1000.0, 0.0, 4.0),
            (1000.0, 7.0, 4.0),
            (1852.0, 20.0, -6.0),
        )
        max_reconstruction_error = 0.0
        max_ray_gap = 0.0
        max_pixel_reprojection_error = 0.0
        max_unit_vector_norm_error = 0.0

        for values in points:
            with self.subTest(point=values):
                projection = project_nominal_stereo_point(point_frd(values), camera, BASELINE_M)
                self.assertTrue(projection.valid)

                triangulation = triangulate_nominal_stereo_centers(
                    left_cx_px=projection.left.u_px,
                    left_cy_px=projection.left.v_px,
                    right_cx_px=projection.right.u_px,
                    right_cy_px=projection.right.v_px,
                    width_px=WIDTH_PX,
                    height_px=HEIGHT_PX,
                    horizontal_fov_rad=HORIZONTAL_FOV_RAD,
                    baseline_m=BASELINE_M,
                )
                result = triangulation.triangulation_result
                self.assertTrue(result.valid)
                self.assertEqual(result.output_frame, FrameId.BASELINE_CENTER_FRD)

                reconstruction_error = norm(np.asarray(result.point_m) - np.asarray(values, dtype=float))
                max_reconstruction_error = max(max_reconstruction_error, reconstruction_error)
                max_ray_gap = max(max_ray_gap, float(result.ray_gap_m))
                self.assertLess(reconstruction_error, 1.0e-6)
                self.assertLess(result.ray_gap_m, 1.0e-9)

                reprojection = project_nominal_stereo_point(point_frd(result.point_m), camera, BASELINE_M)
                pixel_reprojection_error = max(
                    abs(reprojection.left.u_px - projection.left.u_px),
                    abs(reprojection.left.v_px - projection.left.v_px),
                    abs(reprojection.right.u_px - projection.right.u_px),
                    abs(reprojection.right.v_px - projection.right.v_px),
                )
                max_pixel_reprojection_error = max(max_pixel_reprojection_error, pixel_reprojection_error)
                self.assertLess(pixel_reprojection_error, 1.0e-9)

                left_ray, right_ray = nominal_stereo_rays_from_centers(
                    left_cx_px=projection.left.u_px,
                    left_cy_px=projection.left.v_px,
                    right_cx_px=projection.right.u_px,
                    right_cy_px=projection.right.v_px,
                    intrinsics=camera,
                    baseline_m=BASELINE_M,
                )
                left_norm_error = abs(norm(left_ray.direction.xyz_unit) - 1.0)
                right_norm_error = abs(norm(right_ray.direction.xyz_unit) - 1.0)
                max_unit_vector_norm_error = max(max_unit_vector_norm_error, left_norm_error, right_norm_error)
                self.assertLess(left_norm_error, 1.0e-12)
                self.assertLess(right_norm_error, 1.0e-12)

        self.assertLess(max_reconstruction_error, 1.0e-6)
        self.assertLess(max_ray_gap, 1.0e-9)
        self.assertLess(max_pixel_reprojection_error, 1.0e-9)
        self.assertLess(max_unit_vector_norm_error, 1.0e-12)

    def test_arbitrary_transform_projection_uses_transform_inverse(self) -> None:
        camera = PinholeIntrinsics(1280, 720, 900.0, 900.0, 640.0, 360.0)
        rotation = rotation_matrix_from_quaternion_wxyz(
            (math.cos(math.pi / 8.0), math.sin(math.pi / 8.0), 0.0, 0.0)
        )
        camera_to_baseline = RigidTransform3D(
            FrameId.CAMERA_OPTICAL_RDF,
            FrameId.BASELINE_CENTER_FRD,
            rotation,
            (1.0, -2.0, 3.0),
        )
        camera_point = FramePoint((2.0, 1.0, 10.0), FrameId.CAMERA_OPTICAL_RDF)
        baseline_point = transform_point(camera_point, camera_to_baseline)

        projection = project_point_to_camera_pixel(baseline_point, camera_to_baseline, camera)

        self.assertTrue(projection.valid)
        assert_almost(self, projection.u_px, 640.0 + 900.0 * 2.0 / 10.0)
        assert_almost(self, projection.v_px, 360.0 + 900.0 * 1.0 / 10.0)
        for actual, expected in zip(projection.point_camera_rdf_m, camera_point.xyz_m):
            self.assertAlmostEqual(actual, expected, delta=1.0e-12)

    def test_wrong_point_frame_rejected(self) -> None:
        left_transform, _ = nominal_camera_transforms(baseline_m=BASELINE_M)
        with self.assertRaises(ValueError):
            project_point_to_camera_pixel(
                FramePoint((1.0, 2.0, 3.0), FrameId.GAZEBO_WORLD_ENU),
                left_transform,
                intrinsics(),
            )

    def test_wrong_transform_direction_rejected(self) -> None:
        camera = intrinsics()
        point = point_frd((1000.0, 0.0, 0.0))
        wrong_source = RigidTransform3D(
            FrameId.BASELINE_CENTER_FRD,
            FrameId.BASELINE_CENTER_FRD,
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (0.0, 0.0, 0.0),
        )
        wrong_target = RigidTransform3D(
            FrameId.CAMERA_OPTICAL_RDF,
            FrameId.DRONE_BODY_FRD,
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (0.0, 0.0, 0.0),
        )

        with self.assertRaises(ValueError):
            project_point_to_camera_pixel(point, wrong_source, camera)
        with self.assertRaises(ValueError):
            project_point_to_camera_pixel(point, wrong_target, camera)

    def test_wrong_types_rejected(self) -> None:
        left_transform, _ = nominal_camera_transforms(baseline_m=BASELINE_M)
        camera = intrinsics()
        with self.assertRaises(TypeError):
            project_point_to_camera_pixel((1000.0, 0.0, 0.0), left_transform, camera)
        with self.assertRaises(TypeError):
            project_point_to_camera_pixel(point_frd((1000.0, 0.0, 0.0)), object(), camera)
        with self.assertRaises(TypeError):
            project_point_to_camera_pixel(point_frd((1000.0, 0.0, 0.0)), left_transform, object())

    def test_non_positive_depth_returns_structured_invalid(self) -> None:
        camera = intrinsics()
        for values in ((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0)):
            with self.subTest(point=values):
                projection = project_nominal_stereo_point(point_frd(values), camera, BASELINE_M)
                self.assertFalse(projection.valid)
                self.assertEqual(projection.status, StereoProjectionStatus.BOTH_INVALID)
                self.assertEqual(projection.left.status, ProjectionStatus.NON_POSITIVE_DEPTH)
                self.assertIsNone(projection.left.u_px)
                self.assertIsNone(projection.left.v_px)
                self.assertIsNone(projection.left.in_bounds)
                self.assertFalse(projection.left.clip_valid)
                self.assertIsNone(projection.signed_disparity_px)
                self.assertIsNone(projection.vertical_disparity_px)

    def test_out_of_bounds_keeps_diagnostic_pixels(self) -> None:
        projection = project_nominal_stereo_point(point_frd((1000.0, 500.0, 0.0)), intrinsics(), BASELINE_M)

        self.assertFalse(projection.valid)
        self.assertEqual(projection.left.status, ProjectionStatus.OUT_OF_BOUNDS)
        self.assertFalse(projection.left.in_bounds)
        self.assertTrue(math.isfinite(projection.left.u_px))
        self.assertTrue(math.isfinite(projection.left.v_px))

    def test_clip_policy_on_optical_axis(self) -> None:
        camera = intrinsics()
        left_transform, _ = nominal_camera_transforms(baseline_m=BASELINE_M)
        cases = (
            (4.0, ProjectionStatus.OUTSIDE_CLIP_RANGE, False),
            (5.0, ProjectionStatus.OK, True),
            (15.0, ProjectionStatus.OK, True),
            (16.0, ProjectionStatus.OUTSIDE_CLIP_RANGE, False),
        )
        for depth_m, expected_status, expected_valid in cases:
            with self.subTest(depth_m=depth_m):
                baseline_point = transform_point(
                    FramePoint((0.0, 0.0, depth_m), FrameId.CAMERA_OPTICAL_RDF),
                    left_transform,
                )
                projection = project_point_to_camera_pixel(
                    baseline_point,
                    left_transform,
                    camera,
                    clip_near_m=5.0,
                    clip_far_m=15.0,
                )

                self.assertEqual(projection.status, expected_status)
                self.assertEqual(projection.valid, expected_valid)
                self.assertTrue(projection.in_bounds)

    def test_bounds_and_clip_can_fail_together(self) -> None:
        projection = project_nominal_stereo_point(
            point_frd((1000.0, 500.0, 0.0)),
            intrinsics(),
            BASELINE_M,
            clip_near_m=1.0,
            clip_far_m=500.0,
        )

        self.assertFalse(projection.valid)
        self.assertEqual(projection.left.status, ProjectionStatus.OUTSIDE_CLIP_AND_BOUNDS)
        self.assertFalse(projection.left.in_bounds)
        self.assertFalse(projection.left.clip_valid)

    def test_invalid_clip_configuration_rejected(self) -> None:
        left_transform, _ = nominal_camera_transforms(baseline_m=BASELINE_M)
        point = point_frd((1000.0, 0.0, 0.0))
        camera = intrinsics()
        invalid_kwargs = (
            {"clip_near_m": 1.0},
            {"clip_far_m": 10.0},
            {"clip_near_m": -1.0, "clip_far_m": 10.0},
            {"clip_near_m": 10.0, "clip_far_m": 10.0},
            {"clip_near_m": 10.0, "clip_far_m": 9.0},
            {"clip_near_m": True, "clip_far_m": 10.0},
            {"clip_near_m": 1.0, "clip_far_m": False},
            {"clip_near_m": "1.0", "clip_far_m": 10.0},
            {"clip_near_m": 1.0, "clip_far_m": "10.0"},
            {"clip_near_m": math.nan, "clip_far_m": 10.0},
            {"clip_near_m": 1.0, "clip_far_m": math.inf},
        )

        for kwargs in invalid_kwargs:
            with self.subTest(kwargs=kwargs):
                with self.assertRaises((TypeError, ValueError)):
                    project_point_to_camera_pixel(point, left_transform, camera, **kwargs)

    def test_negative_baseline_rejected(self) -> None:
        for value in (-1.0, True, "25.0", math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises((TypeError, ValueError)):
                    project_nominal_stereo_point(point_frd((1000.0, 0.0, 0.0)), intrinsics(), value)

    def test_zero_baseline_projects_but_triangulation_is_zero_baseline_invalid(self) -> None:
        camera = intrinsics()
        projection = project_nominal_stereo_point(point_frd((1000.0, 7.0, 4.0)), camera, 0.0)

        self.assertTrue(projection.valid)
        assert_almost(self, projection.left.u_px, projection.right.u_px)
        assert_almost(self, projection.left.v_px, projection.right.v_px)
        assert_almost(self, projection.signed_disparity_px, 0.0)

        triangulation = triangulate_nominal_stereo_centers(
            left_cx_px=projection.left.u_px,
            left_cy_px=projection.left.v_px,
            right_cx_px=projection.right.u_px,
            right_cy_px=projection.right.v_px,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=0.0,
        )
        self.assertFalse(triangulation.triangulation_result.valid)
        self.assertEqual(triangulation.triangulation_result.status, TriangulationStatus.ZERO_BASELINE)

    def test_diagnostic_disparity_for_positive_depth_invalid_results(self) -> None:
        bounds_invalid = project_nominal_stereo_point(point_frd((1000.0, 500.0, 0.0)), intrinsics(), BASELINE_M)
        clip_invalid = project_nominal_stereo_point(
            point_frd((1000.0, 0.0, 0.0)),
            intrinsics(),
            BASELINE_M,
            clip_near_m=1.0,
            clip_far_m=500.0,
        )

        self.assertFalse(bounds_invalid.valid)
        self.assertTrue(math.isfinite(bounds_invalid.signed_disparity_px))
        self.assertTrue(math.isfinite(bounds_invalid.vertical_disparity_px))
        self.assertFalse(clip_invalid.valid)
        self.assertTrue(math.isfinite(clip_invalid.signed_disparity_px))
        self.assertTrue(math.isfinite(clip_invalid.vertical_disparity_px))

        non_positive = project_nominal_stereo_point(point_frd((0.0, 0.0, 0.0)), intrinsics(), BASELINE_M)
        self.assertIsNone(non_positive.signed_disparity_px)
        self.assertIsNone(non_positive.vertical_disparity_px)

    def test_direct_formula_oracle_matches_projection(self) -> None:
        expected = direct_nominal_pixels((1852.0, 20.0, -6.0))
        projection = project_nominal_stereo_point(point_frd((1852.0, 20.0, -6.0)), intrinsics(), BASELINE_M)

        actual = (projection.left.u_px, projection.left.v_px, projection.right.u_px, projection.right.v_px)
        for actual_value, expected_value in zip(actual, expected):
            assert_almost(self, actual_value, expected_value)


if __name__ == "__main__":
    unittest.main()
