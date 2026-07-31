import math
import unittest

from pose_simulation.evaluation import DEFAULT_RIG_FROM_OPTICAL_RDF, NominalStereoExtrinsics
from pose_simulation.pose_math import (
    IDENTITY_ROTATION,
    Pose3D,
    angle_between,
    compute_baseline_geometry,
    compute_camera_pose_error,
    compute_look_at_rig_rotation,
    compute_relative_camera_pose_error,
    compute_rotation_error,
    derive_camera_optical_pose,
    determinant,
    matrix_multiply,
    matrix_vector,
    rotation_to_euler_zyx_rad,
    validate_rotation_matrix,
)


def rotation_z(angle_rad):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def rotation_x(angle_rad):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


class PoseMathTests(unittest.TestCase):
    def test_look_at_is_orthonormal_and_front_follows_los(self):
        result = compute_look_at_rig_rotation((0.0, 0.0, 0.0), (0.0, 1000.0, 100.0))
        self.assertTrue(result.valid)
        metrics = validate_rotation_matrix(result.rotation_world_from_rig)
        self.assertLess(metrics["orthogonality_error"], 1.0e-12)
        self.assertLess(metrics["determinant_error"], 1.0e-12)
        expected_front = (0.0, 1000.0 / math.sqrt(1_010_000.0), 100.0 / math.sqrt(1_010_000.0))
        self.assertLess(angle_between(result.front_world_enu, expected_front), 1.0e-12)
        self.assertAlmostEqual(determinant(result.rotation_world_from_rig), 1.0, places=12)

    def test_optical_rdf_to_rig_frd_axes_are_explicit(self):
        pose = derive_camera_optical_pose(
            (0.0, 0.0, 0.0),
            IDENTITY_ROTATION,
            (0.0, 0.0, 0.0),
            DEFAULT_RIG_FROM_OPTICAL_RDF,
        )
        self.assertEqual(matrix_vector(pose.rotation_world_from_frame, (0.0, 0.0, 1.0)), (1.0, 0.0, 0.0))
        self.assertEqual(matrix_vector(pose.rotation_world_from_frame, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))
        self.assertEqual(matrix_vector(pose.rotation_world_from_frame, (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0))

    def test_known_positive_yaw_error_uses_declared_euler_sign(self):
        error = compute_rotation_error(IDENTITY_ROTATION, rotation_z(math.radians(0.1)))
        self.assertAlmostEqual(error.roll_error_deg, 0.0, delta=1.0e-12)
        self.assertAlmostEqual(error.pitch_error_deg, 0.0, delta=1.0e-12)
        self.assertAlmostEqual(error.yaw_error_deg, 0.1, delta=1.0e-12)
        self.assertAlmostEqual(error.rotation_error_deg, 0.1, delta=1.0e-12)

    def test_relative_yaw_error_is_right_minus_left(self):
        extrinsics = NominalStereoExtrinsics(
            left_rotation_rig_from_optical=DEFAULT_RIG_FROM_OPTICAL_RDF,
            right_rotation_rig_from_optical=DEFAULT_RIG_FROM_OPTICAL_RDF,
        )
        result = compute_relative_camera_pose_error(
            rotation_z(math.radians(0.10)),
            rotation_z(math.radians(0.14)),
            extrinsics.left_rotation_rig_from_optical,
            extrinsics.right_rotation_rig_from_optical,
        )
        self.assertAlmostEqual(result.yaw_error_deg, 0.04, delta=1.0e-12)

    def test_optical_axis_error_is_zero_and_then_point_two_degrees(self):
        ideal = Pose3D((0.0, 0.0, 0.0), IDENTITY_ROTATION, "camera_optical_rdf")
        exact = compute_camera_pose_error(ideal, ideal, (0.0, 0.0, 10.0))
        self.assertAlmostEqual(exact.optical_axis_to_target_error_deg, 0.0, delta=1.0e-12)
        actual = Pose3D((0.0, 0.0, 0.0), rotation_x(math.radians(0.2)), "camera_optical_rdf")
        offset = compute_camera_pose_error(ideal, actual, (0.0, 0.0, 10.0))
        self.assertAlmostEqual(offset.optical_axis_to_target_error_deg, 0.2, delta=1.0e-12)

    def test_baseline_geometry(self):
        result = compute_baseline_geometry(
            (0.0, -12.5, 0.0),
            (0.0, 12.5, 0.0),
            (0.0, 25.0, 0.0),
            IDENTITY_ROTATION,
        )
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.actual_length_m, 25.0, delta=1.0e-12)
        self.assertAlmostEqual(result.nominal_length_m, 25.0, delta=1.0e-12)
        self.assertAlmostEqual(result.length_error_m, 0.0, delta=1.0e-12)
        self.assertAlmostEqual(result.midpoint_world_enu_m[0], 0.0, delta=1.0e-12)
        self.assertAlmostEqual(result.midpoint_world_enu_m[1], 0.0, delta=1.0e-12)
        self.assertIsNone(result.midpoint_error_m)

    def test_zero_los_and_near_vertical_have_explicit_status(self):
        zero = compute_look_at_rig_rotation((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertFalse(zero.valid)
        self.assertEqual(zero.status, "INVALID_ZERO_LOS")
        vertical = compute_look_at_rig_rotation((0.0, 0.0, 0.0), (0.0, 0.0, 100.0))
        self.assertTrue(vertical.valid)
        self.assertTrue(vertical.status.startswith("NEAR_VERTICAL_"))

    def test_reflection_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_rotation_matrix(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)))

    def test_non_normalized_quaternion_is_rejected_at_boundary(self):
        from pose_simulation.pose_math import quaternion_xyzw_to_matrix

        with self.assertRaises(ValueError):
            quaternion_xyzw_to_matrix((0.0, 0.0, 0.0, 2.0))


if __name__ == "__main__":
    unittest.main()
