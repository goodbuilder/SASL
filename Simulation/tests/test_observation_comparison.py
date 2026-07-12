from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.coordinate_frames import FrameId, FramePoint  # noqa: E402
from stereo_ground_sim.observation_comparison import (  # noqa: E402
    GroundTruthSnapshot,
    PixelObservation,
    bbox_measurement_from_pixels,
    compare_observation_measurements,
    ideal_measurement_from_snapshot,
)
from stereo_ground_sim.stereo_triangulation_adapter import pinhole_intrinsics_from_horizontal_fov  # noqa: E402


WIDTH_PX = 1280
HEIGHT_PX = 720
HORIZONTAL_FOV_RAD = 0.154997710698
BASELINE_M = 25.0


def norm(values) -> float:
    return float(np.linalg.norm(np.asarray(values, dtype=float)))


def intrinsics():
    return pinhole_intrinsics_from_horizontal_fov(
        width_px=WIDTH_PX,
        height_px=HEIGHT_PX,
        horizontal_fov_rad=HORIZONTAL_FOV_RAD,
    )


def snapshot(point=(1852.0, 20.0, -6.0), *, sequence=7, valid=True):
    if not valid:
        return GroundTruthSnapshot(
            sequence=sequence,
            received_monotonic_s=12.5,
            point_baseline_center_frd=None,
            range_m=None,
            oracle_valid=False,
            oracle_status="oracle_source_not_found",
            oracle_source_role=None,
            oracle_matched_entity_name=None,
            oracle_fallback_used=False,
        )
    point_obj = FramePoint(tuple(float(value) for value in point), FrameId.BASELINE_CENTER_FRD)
    return GroundTruthSnapshot(
        sequence=sequence,
        received_monotonic_s=12.5,
        point_baseline_center_frd=point_obj,
        range_m=norm(point),
        oracle_valid=True,
        oracle_status="ok_target_reference_link",
        oracle_source_role="target_reference_link",
        oracle_matched_entity_name="target_reference_link",
        oracle_fallback_used=False,
    )


def direct_pixels(point):
    forward_m, right_m, down_m = (float(value) for value in point)
    camera = intrinsics()
    left_u = camera.cx_px + camera.fx_px * (right_m + BASELINE_M * 0.5) / forward_m
    right_u = camera.cx_px + camera.fx_px * (right_m - BASELINE_M * 0.5) / forward_m
    v = camera.cy_px + camera.fy_px * down_m / forward_m
    return left_u, v, right_u, v


def pixels_from_ideal(ideal, *, source="external_bbox", received=13.0):
    return PixelObservation(
        ideal.left_u_px,
        ideal.left_v_px,
        ideal.right_u_px,
        ideal.right_v_px,
        source,
        received,
    )


class ObservationComparisonTests(unittest.TestCase):
    def test_valid_ground_truth_snapshot_validation(self) -> None:
        snap = snapshot()

        self.assertTrue(snap.oracle_valid)
        self.assertEqual(snap.sequence, 7)
        self.assertEqual(snap.point_baseline_center_frd.frame, FrameId.BASELINE_CENTER_FRD)
        self.assertAlmostEqual(snap.range_m, norm((1852.0, 20.0, -6.0)))

    def test_invalid_snapshot_requires_point_and_range_none(self) -> None:
        self.assertFalse(snapshot(valid=False).oracle_valid)
        with self.assertRaises(ValueError):
            GroundTruthSnapshot(
                0,
                0.0,
                FramePoint((1.0, 2.0, 3.0), FrameId.BASELINE_CENTER_FRD),
                None,
                False,
                "invalid",
                None,
                None,
                False,
            )

    def test_wrong_point_frame_rejected(self) -> None:
        with self.assertRaises(ValueError):
            GroundTruthSnapshot(
                0,
                0.0,
                FramePoint((1.0, 2.0, 3.0), FrameId.GAZEBO_WORLD_ENU),
                1.0,
                True,
                "ok",
                "target_reference_link",
                "target_reference_link",
                False,
            )

    def test_known_gt_ideal_round_trip(self) -> None:
        snap = snapshot()
        ideal = ideal_measurement_from_snapshot(
            snap,
            intrinsics=intrinsics(),
            baseline_m=BASELINE_M,
            clip_near_m=0.1,
            clip_far_m=2500.0,
        )
        expected_pixels = direct_pixels(snap.point_baseline_center_frd.xyz_m)

        self.assertTrue(ideal.valid)
        self.assertEqual(ideal.status, "ok")
        self.assertEqual(ideal.pixel_bounds_status, "all_in_bounds")
        self.assertEqual(ideal.oracle_sequence, snap.sequence)
        self.assertLess(norm(np.asarray(ideal.estimated_point_baseline_center_frd_m) - np.asarray(snap.point_baseline_center_frd.xyz_m)), 1.0e-6)
        self.assertLess(ideal.ray_gap_m, 1.0e-9)
        self.assertEqual(ideal.estimated_point_baseline_center_frd_m.__class__, tuple)
        for actual, expected in zip(
            (ideal.left_u_px, ideal.left_v_px, ideal.right_u_px, ideal.right_v_px),
            expected_pixels,
        ):
            self.assertAlmostEqual(actual, expected, delta=1.0e-9)

    def test_bbox_pixels_equal_ideal_pixels(self) -> None:
        snap = snapshot()
        ideal = ideal_measurement_from_snapshot(
            snap,
            intrinsics=intrinsics(),
            baseline_m=BASELINE_M,
            clip_near_m=0.1,
            clip_far_m=2500.0,
        )
        bbox = bbox_measurement_from_pixels(
            snap,
            pixels_from_ideal(ideal),
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )
        comparison = compare_observation_measurements(ideal, bbox)

        self.assertTrue(bbox.valid)
        self.assertEqual(bbox.observation_source, "external_bbox")
        self.assertTrue(comparison.valid)
        self.assertTrue(comparison.same_oracle_sequence)
        self.assertLess(comparison.bbox_minus_ideal_norm_m, 1.0e-6)
        self.assertLess(comparison.error_identity_residual_m, 1.0e-9)

    def test_deterministic_bbox_pixel_offset(self) -> None:
        snap = snapshot()
        ideal = ideal_measurement_from_snapshot(
            snap,
            intrinsics=intrinsics(),
            baseline_m=BASELINE_M,
            clip_near_m=0.1,
            clip_far_m=2500.0,
        )
        pixels = PixelObservation(
            ideal.left_u_px + 0.25,
            ideal.left_v_px + 0.10,
            ideal.right_u_px - 0.25,
            ideal.right_v_px - 0.10,
            "external_bbox",
            13.0,
        )

        bbox = bbox_measurement_from_pixels(
            snap,
            pixels,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )
        comparison = compare_observation_measurements(ideal, bbox)

        self.assertTrue(comparison.valid)
        self.assertAlmostEqual(comparison.pixel_delta_left_px[0], 0.25, delta=1.0e-12)
        self.assertAlmostEqual(comparison.pixel_delta_left_px[1], 0.10, delta=1.0e-12)
        self.assertAlmostEqual(comparison.pixel_delta_right_px[0], -0.25, delta=1.0e-12)
        self.assertAlmostEqual(comparison.pixel_delta_right_px[1], -0.10, delta=1.0e-12)
        self.assertAlmostEqual(comparison.disparity_delta_px, 0.5)
        self.assertTrue(math.isfinite(comparison.bbox_minus_ideal_norm_m))
        self.assertGreater(comparison.bbox_minus_ideal_norm_m, 0.0)
        self.assertLess(comparison.error_identity_residual_m, 1.0e-9)

    def test_different_sequence_comparison_invalid(self) -> None:
        ideal = ideal_measurement_from_snapshot(
            snapshot(sequence=1),
            intrinsics=intrinsics(),
            baseline_m=BASELINE_M,
            clip_near_m=0.1,
            clip_far_m=2500.0,
        )
        bbox = bbox_measurement_from_pixels(
            snapshot(sequence=2),
            pixels_from_ideal(ideal),
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        comparison = compare_observation_measurements(ideal, bbox)

        self.assertFalse(comparison.valid)
        self.assertEqual(comparison.status, "different_oracle_sequence")
        self.assertFalse(comparison.same_oracle_sequence)

    def test_invalid_snapshot_ideal_invalid(self) -> None:
        ideal = ideal_measurement_from_snapshot(
            snapshot(valid=False),
            intrinsics=intrinsics(),
            baseline_m=BASELINE_M,
            clip_near_m=0.1,
            clip_far_m=2500.0,
        )

        self.assertFalse(ideal.valid)
        self.assertEqual(ideal.status, "invalid_oracle_snapshot")
        self.assertIsNone(ideal.estimated_point_baseline_center_frd_m)

    def test_invalid_snapshot_allows_bbox_standalone_geometry(self) -> None:
        pixels = PixelObservation(*direct_pixels((1852.0, 20.0, -6.0)), "external_bbox", 13.0)

        bbox = bbox_measurement_from_pixels(
            snapshot(valid=False),
            pixels,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        self.assertTrue(bbox.valid)
        self.assertEqual(bbox.status, "ok_no_ground_truth")
        self.assertIsNotNone(bbox.estimated_point_baseline_center_frd_m)
        self.assertIsNone(bbox.error_vector_baseline_center_frd_m)

    def test_out_of_bounds_bbox_measurement_invalid(self) -> None:
        bbox = bbox_measurement_from_pixels(
            snapshot(),
            PixelObservation(-1.0, 360.0, 640.0, 360.0, "external_bbox", 13.0),
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        self.assertFalse(bbox.valid)
        self.assertEqual(bbox.status, "out_of_bounds")
        self.assertEqual(bbox.pixel_bounds_status, "left_out_of_bounds")

    def test_parallel_zero_disparity_bbox_invalid(self) -> None:
        bbox = bbox_measurement_from_pixels(
            snapshot(),
            PixelObservation(640.0, 360.0, 640.0, 360.0, "external_bbox", 13.0),
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        self.assertFalse(bbox.valid)
        self.assertEqual(bbox.status, "triangulation_invalid:parallel_rays")

    def test_non_finite_pixel_rejected(self) -> None:
        for value in (math.nan, math.inf, True, "1.0"):
            with self.subTest(value=value):
                with self.assertRaises((TypeError, ValueError)):
                    PixelObservation(value, 360.0, 640.0, 360.0, "external_bbox", 13.0)

    def test_source_metadata_preserved(self) -> None:
        pixels = PixelObservation(*direct_pixels((1852.0, 20.0, -6.0)), "synthetic_bbox_compatibility", 13.0)

        bbox = bbox_measurement_from_pixels(
            snapshot(),
            pixels,
            width_px=WIDTH_PX,
            height_px=HEIGHT_PX,
            horizontal_fov_rad=HORIZONTAL_FOV_RAD,
            baseline_m=BASELINE_M,
        )

        self.assertEqual(bbox.observation_source, "synthetic_bbox_compatibility")
        self.assertEqual(bbox.observation_received_monotonic_s, 13.0)


if __name__ == "__main__":
    unittest.main()
