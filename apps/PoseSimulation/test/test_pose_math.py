import math
import unittest

from pose_simulation.evaluation import DEFAULT_RIG_FROM_OPTICAL_RDF, NominalStereoExtrinsics
from pose_simulation.pose_math import (
    CAMERA_OPTICAL_FRAME,
    IDENTITY_ROTATION,
    Pose3D,
    angle_between,
    compute_baseline_geometry,
    compute_baseline_geometry_from_translations,
    compute_camera_pose_error,
    compute_look_at_rig_rotation,
    compute_relative_camera_pose_error,
    compute_rotation_error,
    decompose_optical_rdf_error,
    derive_camera_optical_pose,
    determinant,
    matrix_multiply,
    matrix_vector,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
    interpolate_pose,
    validate_rotation_matrix,
)


def rotation_forward(angle_rad):
    """Active rotation about optical RDF +Forward (+Z)."""

    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def rotation_right(angle_rad):
    """Active rotation about optical RDF +Right (+X)."""

    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rotation_down(angle_rad):
    """Active rotation about optical RDF +Down (+Y)."""

    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def optical_rpy(roll, pitch, yaw):
    return matrix_multiply(
        rotation_down(yaw), matrix_multiply(rotation_right(pitch), rotation_forward(roll))
    )


def frobenius_error(left, right):
    return math.sqrt(
        sum(
            (left[row][column] - right[row][column]) ** 2
            for row in range(3)
            for column in range(3)
        )
    )


class PoseMathTests(unittest.TestCase):
    def test_north_look_at_uses_front_cross_world_up(self):
        result = compute_look_at_rig_rotation((0.0, 0.0, 0.0), (0.0, 1000.0, 0.0))
        self.assertTrue(result.valid)
        self.assertEqual(result.status, "VALID")
        self.assertEqual(result.front_world_enu, (0.0, 1.0, 0.0))
        self.assertEqual(result.right_world_enu, (1.0, 0.0, 0.0))
        self.assertEqual(result.down_world_enu, (0.0, 0.0, -1.0))
        metrics = validate_rotation_matrix(result.rotation_world_from_rig)
        self.assertLess(metrics["orthogonality_error"], 1.0e-12)
        self.assertLess(metrics["determinant_error"], 1.0e-12)
        self.assertAlmostEqual(determinant(result.rotation_world_from_rig), 1.0, places=12)

    def test_east_look_at_has_negative_world_y_right(self):
        result = compute_look_at_rig_rotation((0.0, 0.0, 0.0), (1000.0, 0.0, 0.0))
        self.assertTrue(result.valid)
        self.assertEqual(result.front_world_enu, (1.0, 0.0, 0.0))
        self.assertEqual(result.right_world_enu, (0.0, -1.0, 0.0))
        self.assertEqual(result.down_world_enu, (0.0, 0.0, -1.0))
        self.assertLess(angle_between(result.front_world_enu, (1.0, 0.0, 0.0)), 1.0e-12)

    def test_optical_rdf_to_rig_frd_axes_are_explicit(self):
        pose = derive_camera_optical_pose(
            (0.0, 0.0, 0.0),
            IDENTITY_ROTATION,
            (0.0, 0.0, 0.0),
            DEFAULT_RIG_FROM_OPTICAL_RDF,
        )
        self.assertEqual(pose.frame_id, CAMERA_OPTICAL_FRAME)
        self.assertEqual(
            matrix_vector(pose.rotation_world_from_frame, (0.0, 0.0, 1.0)),
            (1.0, 0.0, 0.0),
        )
        self.assertEqual(
            matrix_vector(pose.rotation_world_from_frame, (1.0, 0.0, 0.0)),
            (0.0, 1.0, 0.0),
        )
        self.assertEqual(
            matrix_vector(pose.rotation_world_from_frame, (0.0, 1.0, 0.0)),
            (0.0, 0.0, 1.0),
        )

    def test_known_positive_optical_yaw_error_uses_declared_sign(self):
        error = compute_rotation_error(IDENTITY_ROTATION, rotation_down(math.radians(0.1)))
        self.assertAlmostEqual(error.roll_error_deg, 0.0, delta=1.0e-12)
        self.assertAlmostEqual(error.pitch_error_deg, 0.0, delta=1.0e-12)
        self.assertAlmostEqual(error.yaw_error_deg, 0.1, delta=1.0e-12)
        self.assertAlmostEqual(error.rotation_error_deg, 0.1, delta=1.0e-12)
        self.assertEqual(error.euler_convention, "optical_rdf_RdownYaw_RrightPitch_RforwardRoll")

    def test_composite_optical_rpy_round_trip(self):
        expected = (math.radians(0.30), math.radians(-0.20), math.radians(0.40))
        rotation = optical_rpy(*expected)
        result = decompose_optical_rdf_error(rotation)
        self.assertFalse(result.gimbal_lock)
        self.assertEqual(result.status, "VALID")
        self.assertAlmostEqual(result.roll_error_deg, 0.30, delta=1.0e-10)
        self.assertAlmostEqual(result.pitch_error_deg, -0.20, delta=1.0e-10)
        self.assertAlmostEqual(result.yaw_error_deg, 0.40, delta=1.0e-10)
        reconstructed = optical_rpy(
            result.roll_error_rad, result.pitch_error_rad, result.yaw_error_rad
        )
        self.assertLess(frobenius_error(rotation, reconstructed), 1.0e-12)

    def test_gimbal_lock_is_reported_but_total_angle_remains_valid(self):
        rotation = optical_rpy(0.0, math.pi / 2.0, 0.0)
        result = decompose_optical_rdf_error(rotation)
        self.assertTrue(result.valid)
        self.assertTrue(result.gimbal_lock)
        self.assertEqual(result.status, "GIMBAL_LOCK")
        error = compute_rotation_error(IDENTITY_ROTATION, rotation)
        self.assertTrue(error.valid)
        self.assertTrue(error.gimbal_lock)
        self.assertAlmostEqual(error.rotation_error_deg, 90.0, delta=1.0e-10)

    def test_near_gimbal_lock_uses_configured_status_threshold(self):
        rotation = optical_rpy(0.0, math.pi / 2.0 - 1.0e-12, 0.0)
        result = decompose_optical_rdf_error(rotation, gimbal_lock_threshold=1.0e-10)
        self.assertTrue(result.gimbal_lock)
        self.assertEqual(result.status, "GIMBAL_LOCK")

    def test_relative_yaw_error_is_right_minus_left(self):
        extrinsics = NominalStereoExtrinsics(
            left_rotation_rig_from_optical=DEFAULT_RIG_FROM_OPTICAL_RDF,
            right_rotation_rig_from_optical=DEFAULT_RIG_FROM_OPTICAL_RDF,
        )
        result = compute_relative_camera_pose_error(
            rotation_down(math.radians(0.10)),
            rotation_down(math.radians(0.14)),
            extrinsics.left_rotation_rig_from_optical,
            extrinsics.right_rotation_rig_from_optical,
        )
        self.assertAlmostEqual(result.yaw_error_deg, 0.04, delta=1.0e-12)
        self.assertAlmostEqual(result.rotation_error_deg, 0.04, delta=1.0e-12)

    def test_optical_axis_error_is_zero_then_point_two_degrees(self):
        ideal = Pose3D((0.0, 0.0, 0.0), IDENTITY_ROTATION, CAMERA_OPTICAL_FRAME)
        exact = compute_camera_pose_error(ideal, ideal, (0.0, 0.0, 10.0))
        self.assertAlmostEqual(exact.optical_axis_to_target_error_deg, 0.0, delta=1.0e-12)
        actual = Pose3D(
            (0.0, 0.0, 0.0), rotation_right(math.radians(0.2)), CAMERA_OPTICAL_FRAME
        )
        offset = compute_camera_pose_error(ideal, actual, (0.0, 0.0, 10.0))
        self.assertAlmostEqual(offset.optical_axis_to_target_error_deg, 0.2, delta=1.0e-12)

    def test_target_at_camera_center_is_invalid_only_for_optical_axis_metric(self):
        pose = Pose3D((0.0, 0.0, 0.0), IDENTITY_ROTATION, CAMERA_OPTICAL_FRAME)
        result = compute_camera_pose_error(pose, pose, (0.0, 0.0, 0.0))
        self.assertIsNone(result.optical_axis_to_target_error_deg)
        self.assertFalse(result.valid)
        self.assertTrue(result.rotation.valid)

    def test_camera_frame_mismatch_is_rejected(self):
        ideal = Pose3D((0.0, 0.0, 0.0), IDENTITY_ROTATION, CAMERA_OPTICAL_FRAME)
        actual = Pose3D((0.0, 0.0, 0.0), IDENTITY_ROTATION, "other_frame")
        with self.assertRaisesRegex(ValueError, "frame mismatch"):
            compute_camera_pose_error(ideal, actual, (0.0, 0.0, 10.0))
        rig_pose = Pose3D((0.0, 0.0, 0.0), IDENTITY_ROTATION, "baseline_center_frd")
        with self.assertRaisesRegex(ValueError, "camera pose error requires"):
            compute_camera_pose_error(rig_pose, rig_pose, (0.0, 0.0, 10.0))

    def test_interpolation_frame_mismatch_is_rejected(self):
        left = Pose3D((0.0, 0.0, 0.0), IDENTITY_ROTATION, CAMERA_OPTICAL_FRAME)
        right = Pose3D((0.0, 0.0, 0.0), IDENTITY_ROTATION, "other_frame")
        with self.assertRaisesRegex(ValueError, "frame mismatch"):
            interpolate_pose(left, right, 0.5)

    def test_baseline_geometry_and_translation_wrapper(self):
        result = compute_baseline_geometry_from_translations(
            (0.0, -12.5, 0.0),
            (0.0, 12.5, 0.0),
            (0.0, -12.5, 0.0),
            (0.0, 12.5, 0.0),
            IDENTITY_ROTATION,
        )
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.actual_length_m, 25.0, delta=1.0e-12)
        self.assertAlmostEqual(result.nominal_length_m, 25.0, delta=1.0e-12)
        self.assertAlmostEqual(result.length_error_m, 0.0, delta=1.0e-12)
        self.assertEqual(result.midpoint_world_enu_m, (0.0, 0.0, 0.0))
        self.assertIsNone(result.midpoint_error_m)

    def test_zero_and_opposite_baseline_are_not_valid(self):
        zero = compute_baseline_geometry(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 25.0, 0.0),
            IDENTITY_ROTATION,
        )
        self.assertFalse(zero.valid)
        self.assertIsNone(zero.direction_error_deg)
        opposite = compute_baseline_geometry(
            (0.0, 12.5, 0.0),
            (0.0, -12.5, 0.0),
            (0.0, 25.0, 0.0),
            IDENTITY_ROTATION,
        )
        self.assertTrue(opposite.valid)
        self.assertAlmostEqual(opposite.direction_error_deg, 180.0, delta=1.0e-12)

    def test_invalid_rotation_and_nonfinite_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            validate_rotation_matrix(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)))
        with self.assertRaises(ValueError):
            compute_baseline_geometry(
                (0.0, 0.0, 0.0),
                (0.0, 25.0, 0.0),
                (0.0, 25.0, 0.0),
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, float("nan"))),
            )
        bad_look_at = compute_look_at_rig_rotation(
            (0.0, 0.0, 0.0), (float("nan"), 1.0, 0.0)
        )
        self.assertFalse(bad_look_at.valid)
        self.assertEqual(bad_look_at.status, "NONFINITE_INPUT")

    def test_zero_los_and_vertical_los_statuses_are_explicit(self):
        zero = compute_look_at_rig_rotation((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertFalse(zero.valid)
        self.assertEqual(zero.status, "ZERO_LOS")
        unresolved = compute_look_at_rig_rotation((0.0, 0.0, 0.0), (0.0, 0.0, 100.0))
        self.assertFalse(unresolved.valid)
        self.assertEqual(unresolved.status, "VERTICAL_LOS_UNRESOLVED")
        resolved = compute_look_at_rig_rotation(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 100.0),
            previous_right_axis_world_enu=(1.0, 0.0, 0.0),
        )
        self.assertTrue(resolved.valid)
        self.assertEqual(resolved.status, "VERTICAL_LOS_WITH_PREVIOUS_RIGHT")
        self.assertLess(validate_rotation_matrix(resolved.rotation_world_from_rig)["determinant_error"], 1.0e-12)
        degenerate_previous = compute_look_at_rig_rotation(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 100.0),
            previous_right_axis_world_enu=(0.0, 0.0, 1.0),
        )
        self.assertFalse(degenerate_previous.valid)
        self.assertEqual(degenerate_previous.status, "VERTICAL_LOS_UNRESOLVED")

    def test_quaternion_sign_equivalence_and_boundary_failures(self):
        rotation = optical_rpy(math.radians(0.3), math.radians(-0.2), math.radians(0.4))
        quaternion = matrix_to_quaternion_xyzw(rotation)
        self.assertLess(
            frobenius_error(
                quaternion_xyzw_to_matrix(quaternion),
                quaternion_xyzw_to_matrix(tuple(-value for value in quaternion)),
            ),
            1.0e-12,
        )
        with self.assertRaises(ValueError):
            quaternion_xyzw_to_matrix((0.0, 0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            quaternion_xyzw_to_matrix((0.0, 0.0, 0.0, 2.0))

    def test_wrong_quaternion_order_fixture_is_not_silently_equal(self):
        angle = math.radians(30.0)
        correct_xyzw = (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))
        wrong_wxyz_fixture = (correct_xyzw[3], correct_xyzw[0], correct_xyzw[1], correct_xyzw[2])
        self.assertGreater(
            frobenius_error(
                quaternion_xyzw_to_matrix(correct_xyzw),
                quaternion_xyzw_to_matrix(wrong_wxyz_fixture),
            ),
            1.0e-3,
        )

    def test_parallel_and_antiparallel_axis_angles_are_clamped(self):
        self.assertAlmostEqual(angle_between((0.0, 0.0, 1.0), (0.0, 0.0, 1.0)), 0.0, delta=1.0e-12)
        self.assertAlmostEqual(angle_between((0.0, 0.0, 1.0), (0.0, 0.0, -1.0)), math.pi, delta=1.0e-12)


if __name__ == "__main__":
    unittest.main()
