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
    transform_direction,
    transform_point,
)
from stereo_ground_sim.stereo_triangulation_adapter import (  # noqa: E402
    R_FRD_RDF,
    general_ray_shadow_summary,
    nominal_camera_transforms,
    nominal_stereo_rays_from_centers,
    pinhole_intrinsics_from_horizontal_fov,
    triangulate_nominal_stereo_centers,
)
from stereo_ground_sim.triangulation_core import TriangulationStatus, camera_ray_from_pixel  # noqa: E402


WIDTH_PX = 1280
HEIGHT_PX = 720
HORIZONTAL_FOV_RAD = 0.154997710698
BASELINE_M = 25.0


def norm(values) -> float:
    return float(np.linalg.norm(np.asarray(values, dtype=float)))


def project_point_to_pixels(point_frd_m):
    intrinsics = pinhole_intrinsics_from_horizontal_fov(
        width_px=WIDTH_PX,
        height_px=HEIGHT_PX,
        horizontal_fov_rad=HORIZONTAL_FOV_RAD,
    )
    point = np.asarray(point_frd_m, dtype=float)

    def project(origin_frd_m):
        relative = point - np.asarray(origin_frd_m, dtype=float)
        forward, right, down = relative
        return (
            intrinsics.cx_px + right * intrinsics.fx_px / forward,
            intrinsics.cy_px + down * intrinsics.fy_px / forward,
        )

    left = project((0.0, -BASELINE_M * 0.5, 0.0))
    right = project((0.0, BASELINE_M * 0.5, 0.0))
    return left[0], left[1], right[0], right[1]


def legacy_position_from_centers(left_cx, left_cy, right_cx, right_cy):
    intrinsics = pinhole_intrinsics_from_horizontal_fov(
        width_px=WIDTH_PX,
        height_px=HEIGHT_PX,
        horizontal_fov_rad=HORIZONTAL_FOV_RAD,
    )
    disparity = abs(float(left_cx) - float(right_cx))
    if disparity < 0.5:
        return {
            "forward_m": None,
            "right_m": None,
            "down_m": None,
            "range_m": None,
        }
    forward = intrinsics.fx_px * BASELINE_M / disparity
    right = (((float(left_cx) + float(right_cx)) * 0.5) - intrinsics.cx_px) * forward / intrinsics.fx_px
    down = (((float(left_cy) + float(right_cy)) * 0.5) - intrinsics.cy_px) * forward / intrinsics.fy_px
    return {
        "forward_m": forward,
        "right_m": right,
        "down_m": down,
        "range_m": math.sqrt(forward * forward + right * right + down * down),
    }


class StereoTriangulationAdapterTests(unittest.TestCase):
    def assert_tuple_close(self, actual, expected, places: int = 12) -> None:
        self.assertEqual(len(actual), len(expected))
        for value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(value, expected_value, places=places)

    def triangulate_point(self, point_frd_m):
        left_cx, left_cy, right_cx, right_cy = project_point_to_pixels(point_frd_m)
        return triangulate_nominal_stereo_centers(
            left_cx_px=left_cx,
            left_cy_px=left_cy,
            right_cx_px=right_cx,
            right_cy_px=right_cy,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

    def triangulate_pixels(self, left_cx, left_cy, right_cx, right_cy):
        return triangulate_nominal_stereo_centers(
            left_cx_px=left_cx,
            left_cy_px=left_cy,
            right_cx_px=right_cx,
            right_cy_px=right_cy,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

    def test_image_center_camera_ray_is_forward_rdf(self) -> None:
        intrinsics = pinhole_intrinsics_from_horizontal_fov(
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
        )

        ray = camera_ray_from_pixel(intrinsics.cx_px, intrinsics.cy_px, intrinsics)

        self.assertEqual(ray.frame, FrameId.CAMERA_OPTICAL_RDF)
        self.assertEqual(ray.origin.xyz_m, (0.0, 0.0, 0.0))
        self.assertEqual(ray.direction.xyz_unit, (0.0, 0.0, 1.0))

    def test_rdf_to_frd_axis_contract(self) -> None:
        left_transform, _ = nominal_camera_transforms(baseline_m=0.0)

        right = transform_direction(FrameDirection((1.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF), left_transform)
        down = transform_direction(FrameDirection((0.0, 1.0, 0.0), FrameId.CAMERA_OPTICAL_RDF), left_transform)
        forward = transform_direction(FrameDirection((0.0, 0.0, 1.0), FrameId.CAMERA_OPTICAL_RDF), left_transform)

        self.assertEqual(R_FRD_RDF, ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
        self.assert_tuple_close(right.xyz_unit, (0.0, 1.0, 0.0))
        self.assert_tuple_close(down.xyz_unit, (0.0, 0.0, 1.0))
        self.assert_tuple_close(forward.xyz_unit, (1.0, 0.0, 0.0))

    def test_nominal_optical_centers_and_transform_signs(self) -> None:
        left_transform, right_transform = nominal_camera_transforms(baseline_m=BASELINE_M)

        left_center = transform_point(FramePoint((0.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF), left_transform)
        right_center = transform_point(FramePoint((0.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF), right_transform)

        self.assert_tuple_close(left_center.xyz_m, (0.0, -BASELINE_M * 0.5, 0.0))
        self.assert_tuple_close(right_center.xyz_m, (0.0, BASELINE_M * 0.5, 0.0))
        self.assert_tuple_close(left_transform.origin_target_in_source_m, (BASELINE_M * 0.5, 0.0, 0.0))
        self.assert_tuple_close(right_transform.origin_target_in_source_m, (-BASELINE_M * 0.5, 0.0, 0.0))

    def assert_legacy_compatible(self, point_frd_m) -> None:
        left_cx, left_cy, right_cx, right_cy = project_point_to_pixels(point_frd_m)
        result = triangulate_nominal_stereo_centers(
            left_cx_px=left_cx,
            left_cy_px=left_cy,
            right_cx_px=right_cx,
            right_cy_px=right_cy,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )
        legacy = legacy_position_from_centers(left_cx, left_cy, right_cx, right_cy)
        summary = general_ray_shadow_summary(
            result,
            legacy_forward_m=legacy["forward_m"],
            legacy_right_m=legacy["right_m"],
            legacy_down_m=legacy["down_m"],
            legacy_range_m=legacy["range_m"],
        )

        self.assertTrue(summary["valid"])
        self.assertEqual(summary["status"], TriangulationStatus.OK.value)
        self.assertLess(abs(summary["legacy_forward_delta_m"]), 1.0e-6)
        self.assertLess(abs(summary["legacy_right_delta_m"]), 1.0e-6)
        self.assertLess(abs(summary["legacy_down_delta_m"]), 1.0e-6)
        self.assertLess(abs(summary["legacy_range_delta_m"]), 1.0e-6)

    def test_legacy_compatibility_300_m(self) -> None:
        self.assert_legacy_compatible((300.0, 0.0, 0.0))

    def test_legacy_compatibility_1000_m(self) -> None:
        self.assert_legacy_compatible((1000.0, 0.0, 0.0))

    def test_legacy_compatibility_1852_m(self) -> None:
        self.assert_legacy_compatible((1852.0, 0.0, 0.0))

    def test_horizontal_offset_target(self) -> None:
        self.assert_legacy_compatible((1000.0, 7.0, 0.0))

    def test_vertical_offset_target(self) -> None:
        self.assert_legacy_compatible((1000.0, 0.0, 4.0))

    def test_horizontal_and_vertical_offset_target(self) -> None:
        self.assert_legacy_compatible((1000.0, 7.0, 4.0))

    def test_known_point_reconstruction(self) -> None:
        point = np.asarray((1852.0, 20.0, -6.0), dtype=float)

        result = self.triangulate_point(point)

        self.assertTrue(result.triangulation_result.valid)
        self.assertLess(norm(np.asarray(result.triangulation_result.point_m) - point), 1.0e-6)
        self.assertLess(result.triangulation_result.ray_gap_m, 1.0e-9)

    def test_left_right_center_swap_is_invalid(self) -> None:
        left_cx, left_cy, right_cx, right_cy = project_point_to_pixels((1000.0, 0.0, 0.0))

        result = triangulate_nominal_stereo_centers(
            left_cx_px=right_cx,
            left_cy_px=right_cy,
            right_cx_px=left_cx,
            right_cy_px=left_cy,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        self.assertTrue(result.observation_valid)
        self.assertFalse(result.triangulation_result.valid)
        self.assertEqual(result.triangulation_result.status, TriangulationStatus.NON_POSITIVE_DEPTH)

    def test_zero_disparity_is_invalid(self) -> None:
        result = triangulate_nominal_stereo_centers(
            left_cx_px=640.0,
            left_cy_px=360.0,
            right_cx_px=640.0,
            right_cy_px=360.0,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        self.assertFalse(result.triangulation_result.valid)
        self.assertEqual(result.triangulation_result.status, TriangulationStatus.PARALLEL_RAYS)

    def test_negative_signed_disparity_is_invalid(self) -> None:
        result = triangulate_nominal_stereo_centers(
            left_cx_px=630.0,
            left_cy_px=360.0,
            right_cx_px=650.0,
            right_cy_px=360.0,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        self.assertFalse(result.triangulation_result.valid)
        self.assertEqual(result.triangulation_result.status, TriangulationStatus.NON_POSITIVE_DEPTH)

    def test_non_finite_left_pixel_returns_invalid_observation(self) -> None:
        result = triangulate_nominal_stereo_centers(
            left_cx_px=math.nan,
            left_cy_px=360.0,
            right_cx_px=620.0,
            right_cy_px=360.0,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        self.assertFalse(result.observation_valid)
        self.assertEqual(result.observation_status, "non_finite_pixel")
        self.assertIsNone(result.triangulation_result)

    def test_non_finite_right_pixel_returns_invalid_observation(self) -> None:
        result = triangulate_nominal_stereo_centers(
            left_cx_px=660.0,
            left_cy_px=360.0,
            right_cx_px=math.inf,
            right_cy_px=360.0,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        self.assertFalse(result.observation_valid)
        self.assertEqual(result.observation_status, "non_finite_pixel")
        self.assertIsNone(result.triangulation_result)

    def test_finite_out_of_bounds_pixels_are_calculated(self) -> None:
        cases = [
            ("left_out_of_bounds", -1.0, 360.0, 600.0, 360.0),
            ("right_out_of_bounds", 680.0, 360.0, -1.0, 360.0),
            ("both_out_of_bounds", -1.0, 360.0, 1280.0, 360.0),
        ]
        for status, left_cx, left_cy, right_cx, right_cy in cases:
            with self.subTest(status=status):
                result = triangulate_nominal_stereo_centers(
                    left_cx_px=left_cx,
                    left_cy_px=left_cy,
                    right_cx_px=right_cx,
                    right_cy_px=right_cy,
                    width_px=WIDTH_PX,
                    height_px=HEIGHT_PX,
                    horizontal_fov_rad=HORIZONTAL_FOV_RAD,
                    baseline_m=BASELINE_M,
                )

                self.assertEqual(result.pixel_bounds_status, status)
                self.assertIsNotNone(result.triangulation_result)

    def test_all_in_bounds_pixel_status(self) -> None:
        result = self.triangulate_pixels(680.0, 360.0, 600.0, 360.0)

        self.assertEqual(result.pixel_bounds_status, "all_in_bounds")

    def test_left_out_of_bounds_pixel_status(self) -> None:
        result = self.triangulate_pixels(-1.0, 360.0, 600.0, 360.0)

        self.assertEqual(result.pixel_bounds_status, "left_out_of_bounds")

    def test_right_out_of_bounds_pixel_status(self) -> None:
        result = self.triangulate_pixels(680.0, 360.0, -1.0, 360.0)

        self.assertEqual(result.pixel_bounds_status, "right_out_of_bounds")

    def test_both_out_of_bounds_pixel_status(self) -> None:
        result = self.triangulate_pixels(-1.0, 360.0, 1280.0, 360.0)

        self.assertEqual(result.pixel_bounds_status, "both_out_of_bounds")

    def test_output_frame_is_baseline_center_frd(self) -> None:
        result = self.triangulate_point((1000.0, 0.0, 0.0))

        self.assertEqual(result.triangulation_result.output_frame, FrameId.BASELINE_CENTER_FRD)

    def test_nominal_ray_directions_are_unit_vectors(self) -> None:
        intrinsics = pinhole_intrinsics_from_horizontal_fov(
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
        )
        left_cx, left_cy, right_cx, right_cy = project_point_to_pixels((1000.0, 7.0, 4.0))

        left_ray, right_ray = nominal_stereo_rays_from_centers(
            left_cx_px=left_cx,
            left_cy_px=left_cy,
            right_cx_px=right_cx,
            right_cy_px=right_cy,
            intrinsics=intrinsics,
            baseline_m=BASELINE_M,
        )

        self.assertLess(abs(norm(left_ray.direction.xyz_unit) - 1.0), 1.0e-12)
        self.assertLess(abs(norm(right_ray.direction.xyz_unit) - 1.0), 1.0e-12)

    def test_invalid_horizontal_fov_rejected(self) -> None:
        for fov in (0.0, math.pi, math.nan, math.inf):
            with self.subTest(fov=fov):
                with self.assertRaises(ValueError):
                    pinhole_intrinsics_from_horizontal_fov(
                        width_px=WIDTH_PX,
                        height_px=HEIGHT_PX,
                        horizontal_fov_rad=fov,
                    )

    def test_invalid_width_height_rejected(self) -> None:
        invalid_cases = [
            {"width_px": 0},
            {"height_px": -1},
            {"width_px": 1280.5},
            {"height_px": True},
        ]
        for override in invalid_cases:
            with self.subTest(override=override):
                kwargs = {
                    "width_px": WIDTH_PX,
                    "height_px": HEIGHT_PX,
                    "horizontal_fov_rad": HORIZONTAL_FOV_RAD,
                }
                kwargs.update(override)
                with self.assertRaises(ValueError):
                    pinhole_intrinsics_from_horizontal_fov(**kwargs)

    def test_negative_baseline_rejected(self) -> None:
        with self.assertRaises(ValueError):
            nominal_camera_transforms(baseline_m=-1.0)
        with self.assertRaises(ValueError):
            triangulate_nominal_stereo_centers(
                left_cx_px=660.0,
                left_cy_px=360.0,
                right_cx_px=620.0,
                right_cy_px=360.0,
                width_px=WIDTH_PX,
                height_px=HEIGHT_PX,
                horizontal_fov_rad=HORIZONTAL_FOV_RAD,
                baseline_m=-1.0,
            )


if __name__ == "__main__":
    unittest.main()
