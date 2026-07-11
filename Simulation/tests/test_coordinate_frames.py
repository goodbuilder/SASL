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
    quaternion_wxyz_to_xyzw,
    quaternion_xyzw_to_wxyz,
    rotation_matrix_from_quaternion_wxyz,
    transform_direction,
    transform_point,
    transform_ray,
    validate_rotation_matrix,
)


def assert_tuple_close(testcase: unittest.TestCase, actual, expected, places: int = 12) -> None:
    testcase.assertEqual(len(actual), len(expected))
    for left, right in zip(actual, expected):
        testcase.assertAlmostEqual(left, right, places=places)


class CoordinateFrameTests(unittest.TestCase):
    def test_identity_point_transform(self) -> None:
        transform = RigidTransform3D(
            source_frame=FrameId.BASELINE_CENTER_FRD,
            target_frame=FrameId.BASELINE_CENTER_FRD,
            rotation_target_source=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            origin_target_in_source_m=(0.0, 0.0, 0.0),
        )
        point = FramePoint((1.0, 2.0, 3.0), FrameId.BASELINE_CENTER_FRD)

        result = transform_point(point, transform)

        self.assertEqual(result.frame, FrameId.BASELINE_CENTER_FRD)
        assert_tuple_close(self, result.xyz_m, (1.0, 2.0, 3.0))

    def test_translation_only_point_transform(self) -> None:
        transform = RigidTransform3D(
            source_frame=FrameId.GAZEBO_WORLD_ENU,
            target_frame=FrameId.BASELINE_CENTER_FRD,
            rotation_target_source=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            origin_target_in_source_m=(10.0, -2.0, 1.0),
        )

        result = transform_point(FramePoint((13.0, 3.0, 9.0), FrameId.GAZEBO_WORLD_ENU), transform)

        assert_tuple_close(self, result.xyz_m, (3.0, 5.0, 8.0))

    def test_direction_transform_does_not_apply_translation(self) -> None:
        transform = RigidTransform3D(
            source_frame=FrameId.GAZEBO_WORLD_ENU,
            target_frame=FrameId.BASELINE_CENTER_FRD,
            rotation_target_source=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            origin_target_in_source_m=(100.0, 200.0, 300.0),
        )

        result = transform_direction(FrameDirection((0.0, 1.0, 0.0), FrameId.GAZEBO_WORLD_ENU), transform)

        assert_tuple_close(self, result.xyz_unit, (0.0, 1.0, 0.0))

    def test_90_degree_rotation(self) -> None:
        rotation = rotation_matrix_from_quaternion_wxyz(
            (math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0))
        )
        transform = RigidTransform3D(
            source_frame=FrameId.DRONE_BODY_FRD,
            target_frame=FrameId.DRONE_BODY_FRD,
            rotation_target_source=rotation,
            origin_target_in_source_m=(0.0, 0.0, 0.0),
        )

        result = transform_direction(FrameDirection((1.0, 0.0, 0.0), FrameId.DRONE_BODY_FRD), transform)

        assert_tuple_close(self, result.xyz_unit, (0.0, 1.0, 0.0))

    def test_enu_to_ned_to_enu_round_trip(self) -> None:
        r_ned_enu = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
        enu_to_ned = RigidTransform3D(
            source_frame=FrameId.GAZEBO_WORLD_ENU,
            target_frame=FrameId.PX4_LOCAL_NED,
            rotation_target_source=r_ned_enu,
            origin_target_in_source_m=(0.0, 0.0, 0.0),
        )
        ned_to_enu = RigidTransform3D(
            source_frame=FrameId.PX4_LOCAL_NED,
            target_frame=FrameId.GAZEBO_WORLD_ENU,
            rotation_target_source=r_ned_enu,
            origin_target_in_source_m=(0.0, 0.0, 0.0),
        )
        point = FramePoint((3.25, -7.5, 12.0), FrameId.GAZEBO_WORLD_ENU)

        round_trip = transform_point(transform_point(point, enu_to_ned), ned_to_enu)

        max_error = max(abs(a - b) for a, b in zip(point.xyz_m, round_trip.xyz_m))
        self.assertLess(max_error, 1.0e-9)

    def test_rdf_to_frd_axis_transform(self) -> None:
        rdf_to_frd = RigidTransform3D(
            source_frame=FrameId.CAMERA_OPTICAL_RDF,
            target_frame=FrameId.BASELINE_CENTER_FRD,
            rotation_target_source=((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            origin_target_in_source_m=(0.0, 0.0, 0.0),
        )

        result = transform_direction(FrameDirection((0.0, 0.0, 1.0), FrameId.CAMERA_OPTICAL_RDF), rdf_to_frd)

        assert_tuple_close(self, result.xyz_unit, (1.0, 0.0, 0.0))

    def test_quaternion_order_round_trip(self) -> None:
        px4 = (0.5, -0.5, 0.5, -0.5)

        scipy = quaternion_wxyz_to_xyzw(px4)
        round_trip = quaternion_xyzw_to_wxyz(scipy)

        assert_tuple_close(self, round_trip, px4)

    def test_quaternion_rotation_matrix_known_case(self) -> None:
        rotation = rotation_matrix_from_quaternion_wxyz(
            (math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0))
        )
        expected = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))

        max_error = max(abs(a - b) for row_a, row_b in zip(rotation, expected) for a, b in zip(row_a, row_b))
        self.assertLess(max_error, 1.0e-12)

    def test_zero_quaternion_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "zero"):
            quaternion_wxyz_to_xyzw((0.0, 0.0, 0.0, 0.0))

    def test_non_finite_quaternion_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            quaternion_wxyz_to_xyzw((1.0, math.nan, 0.0, 0.0))

    def test_invalid_rotation_shape_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "3x3"):
            validate_rotation_matrix(((1.0, 0.0), (0.0, 1.0)))

    def test_non_orthonormal_matrix_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            validate_rotation_matrix(((1.0, 0.1, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))

    def test_reflection_matrix_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "determinant"):
            validate_rotation_matrix(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)))

    def test_non_finite_matrix_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            validate_rotation_matrix(((1.0, 0.0, 0.0), (0.0, math.inf, 0.0), (0.0, 0.0, 1.0)))

    def test_zero_direction_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "zero vector"):
            FrameDirection.from_vector((0.0, 0.0, 0.0), FrameId.BASELINE_CENTER_FRD)

    def test_frame_mismatch_rejected(self) -> None:
        transform = RigidTransform3D(
            source_frame=FrameId.GAZEBO_WORLD_ENU,
            target_frame=FrameId.BASELINE_CENTER_FRD,
            rotation_target_source=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            origin_target_in_source_m=(0.0, 0.0, 0.0),
        )
        with self.assertRaisesRegex(ValueError, "source frame"):
            transform_point(FramePoint((1.0, 2.0, 3.0), FrameId.PX4_LOCAL_NED), transform)
        with self.assertRaisesRegex(ValueError, "frames must match"):
            Ray3D(
                FramePoint((0.0, 0.0, 0.0), FrameId.BASELINE_CENTER_FRD),
                FrameDirection((1.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF),
            )

    def test_unit_direction_norm_preserved(self) -> None:
        rotation = rotation_matrix_from_quaternion_wxyz(
            (math.cos(math.pi / 8.0), math.sin(math.pi / 8.0), 0.0, 0.0)
        )
        transform = RigidTransform3D(
            source_frame=FrameId.BASELINE_CENTER_FRD,
            target_frame=FrameId.BASELINE_CENTER_FRD,
            rotation_target_source=rotation,
            origin_target_in_source_m=(0.0, 0.0, 0.0),
        )
        ray = Ray3D(
            FramePoint((1.0, 2.0, 3.0), FrameId.BASELINE_CENTER_FRD),
            FrameDirection.from_vector((1.0, 2.0, 3.0), FrameId.BASELINE_CENTER_FRD),
        )

        transformed = transform_ray(ray, transform)

        norm_error = abs(np.linalg.norm(transformed.direction.xyz_unit) - 1.0)
        self.assertLess(norm_error, 1.0e-12)


if __name__ == "__main__":
    unittest.main()
