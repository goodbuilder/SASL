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
    inverse_rigid_transform,
    rotation_matrix_from_quaternion_wxyz,
    transform_point,
)
from stereo_ground_sim.stereo_triangulation_adapter import nominal_camera_transforms  # noqa: E402


BASELINE_M = 25.0
ABS_TOL = 1.0e-12


def norm(values) -> float:
    return float(np.linalg.norm(np.asarray(values, dtype=float)))


def assert_tuple_close(testcase: unittest.TestCase, actual, expected, delta: float = ABS_TOL) -> None:
    testcase.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        testcase.assertAlmostEqual(float(actual_value), float(expected_value), delta=delta)


def assert_transform_close(testcase: unittest.TestCase, actual, expected) -> None:
    testcase.assertEqual(actual.source_frame, expected.source_frame)
    testcase.assertEqual(actual.target_frame, expected.target_frame)
    for actual_row, expected_row in zip(actual.rotation_target_source, expected.rotation_target_source):
        assert_tuple_close(testcase, actual_row, expected_row)
    assert_tuple_close(testcase, actual.origin_target_in_source_m, expected.origin_target_in_source_m)


class InverseRigidTransformTests(unittest.TestCase):
    def test_identity_inverse(self) -> None:
        transform = RigidTransform3D(
            FrameId.BASELINE_CENTER_FRD,
            FrameId.BASELINE_CENTER_FRD,
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (0.0, 0.0, 0.0),
        )

        inverse = inverse_rigid_transform(transform)

        assert_transform_close(self, inverse, transform)

    def test_pure_translation_inverse(self) -> None:
        transform = RigidTransform3D(
            FrameId.GAZEBO_WORLD_ENU,
            FrameId.BASELINE_CENTER_FRD,
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (10.0, -2.0, 5.0),
        )

        inverse = inverse_rigid_transform(transform)

        self.assertEqual(inverse.source_frame, FrameId.BASELINE_CENTER_FRD)
        self.assertEqual(inverse.target_frame, FrameId.GAZEBO_WORLD_ENU)
        assert_tuple_close(self, inverse.origin_target_in_source_m, (-10.0, 2.0, -5.0))

    def test_pure_rotation_inverse(self) -> None:
        rotation = rotation_matrix_from_quaternion_wxyz(
            (math.cos(math.pi / 6.0), 0.0, 0.0, math.sin(math.pi / 6.0))
        )
        transform = RigidTransform3D(
            FrameId.DRONE_BODY_FRD,
            FrameId.BASELINE_CENTER_FRD,
            rotation,
            (0.0, 0.0, 0.0),
        )

        inverse = inverse_rigid_transform(transform)

        expected_rotation = tuple(tuple(float(value) for value in row) for row in np.asarray(rotation).T)
        for actual_row, expected_row in zip(inverse.rotation_target_source, expected_rotation):
            assert_tuple_close(self, actual_row, expected_row)
        assert_tuple_close(self, inverse.origin_target_in_source_m, (0.0, 0.0, 0.0))

    def test_rotation_translation_origin_formula_and_round_trips(self) -> None:
        rotation = rotation_matrix_from_quaternion_wxyz(
            (math.cos(math.pi / 8.0), math.sin(math.pi / 8.0), 0.0, 0.0)
        )
        origin = (3.0, -4.0, 5.0)
        transform = RigidTransform3D(
            FrameId.GAZEBO_WORLD_ENU,
            FrameId.BASELINE_CENTER_FRD,
            rotation,
            origin,
        )

        inverse = inverse_rigid_transform(transform)
        expected_origin = -(np.asarray(rotation, dtype=float) @ np.asarray(origin, dtype=float))

        self.assertEqual(inverse.source_frame, FrameId.BASELINE_CENTER_FRD)
        self.assertEqual(inverse.target_frame, FrameId.GAZEBO_WORLD_ENU)
        assert_tuple_close(self, inverse.origin_target_in_source_m, expected_origin)
        for actual_row, expected_row in zip(inverse.rotation_target_source, np.asarray(rotation).T):
            assert_tuple_close(self, actual_row, expected_row)

        point_a = FramePoint((11.0, 17.0, -2.0), FrameId.GAZEBO_WORLD_ENU)
        point_b = FramePoint((8.0, -6.0, 4.0), FrameId.BASELINE_CENTER_FRD)

        round_trip_a = transform_point(transform_point(point_a, transform), inverse)
        round_trip_b = transform_point(transform_point(point_b, inverse), transform)

        self.assertLess(norm(np.asarray(round_trip_a.xyz_m) - np.asarray(point_a.xyz_m)), ABS_TOL)
        self.assertLess(norm(np.asarray(round_trip_b.xyz_m) - np.asarray(point_b.xyz_m)), ABS_TOL)

    def test_inverse_twice_restores_transform(self) -> None:
        rotation = rotation_matrix_from_quaternion_wxyz(
            (math.cos(math.pi / 10.0), 0.0, math.sin(math.pi / 10.0), 0.0)
        )
        transform = RigidTransform3D(
            FrameId.CAMERA_OPTICAL_RDF,
            FrameId.BASELINE_CENTER_FRD,
            rotation,
            (2.0, -3.0, 4.0),
        )

        restored = inverse_rigid_transform(inverse_rigid_transform(transform))

        assert_transform_close(self, restored, transform)

    def test_nominal_camera_inverse_centers(self) -> None:
        left_transform, right_transform = nominal_camera_transforms(baseline_m=BASELINE_M)

        left_inverse = inverse_rigid_transform(left_transform)
        right_inverse = inverse_rigid_transform(right_transform)

        self.assertEqual(left_inverse.source_frame, FrameId.BASELINE_CENTER_FRD)
        self.assertEqual(left_inverse.target_frame, FrameId.CAMERA_OPTICAL_RDF)
        self.assertEqual(right_inverse.source_frame, FrameId.BASELINE_CENTER_FRD)
        self.assertEqual(right_inverse.target_frame, FrameId.CAMERA_OPTICAL_RDF)
        assert_tuple_close(self, left_inverse.origin_target_in_source_m, (0.0, -BASELINE_M * 0.5, 0.0))
        assert_tuple_close(self, right_inverse.origin_target_in_source_m, (0.0, BASELINE_M * 0.5, 0.0))

        left_center = transform_point(FramePoint((0.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF), left_transform)
        right_center = transform_point(FramePoint((0.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF), right_transform)
        assert_tuple_close(self, left_center.xyz_m, (0.0, -BASELINE_M * 0.5, 0.0))
        assert_tuple_close(self, right_center.xyz_m, (0.0, BASELINE_M * 0.5, 0.0))

    def test_invalid_input_type_rejected(self) -> None:
        with self.assertRaises(TypeError):
            inverse_rigid_transform(object())


if __name__ == "__main__":
    unittest.main()
