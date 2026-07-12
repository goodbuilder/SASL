from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

import stereo_ground_sim.gazebo_camera_attitude_adapter as adapter  # noqa: E402
from stereo_ground_sim.attitude_error_model import (  # noqa: E402
    CameraAttitudeErrorDeg,
    StereoAttitudeErrorConfig,
    rotation_from_attitude_error_deg,
)
from stereo_ground_sim.stereo_triangulation_adapter import R_FRD_RDF  # noqa: E402


EXPECTED_R_M_C_FIXED = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
)


def matrix_close(testcase: unittest.TestCase, actual, expected, delta: float = 1.0e-12) -> None:
    actual_arr = np.asarray(actual, dtype=float)
    expected_arr = np.asarray(expected, dtype=float)
    testcase.assertEqual(actual_arr.shape, (3, 3))
    testcase.assertLessEqual(float(np.max(np.abs(actual_arr - expected_arr))), delta)


def tuple_close(testcase: unittest.TestCase, actual, expected, delta: float = 1.0e-12) -> None:
    testcase.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        testcase.assertAlmostEqual(float(actual_value), float(expected_value), delta=delta)


def assert_rotation(testcase: unittest.TestCase, rotation) -> None:
    arr = np.asarray(rotation, dtype=float)
    testcase.assertLessEqual(float(np.max(np.abs(arr.T @ arr - np.eye(3)))), 1.0e-12)
    testcase.assertLessEqual(abs(float(np.linalg.det(arr)) - 1.0), 1.0e-12)


def rz(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.asarray(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)), dtype=float)


def nominal_model_rotation() -> np.ndarray:
    return rz(math.pi / 2.0)


def r_w_b() -> np.ndarray:
    return np.asarray(adapter.R_B_W, dtype=float).T


def desired_optical_rotation(error: CameraAttitudeErrorDeg) -> np.ndarray:
    return r_w_b() @ (np.asarray(R_FRD_RDF, dtype=float) @ np.asarray(rotation_from_attitude_error_deg(error), dtype=float))


def delta_path_model_rotation(error: CameraAttitudeErrorDeg) -> np.ndarray:
    r_m_c = np.asarray(adapter.derive_fixed_model_to_optical_rotation(), dtype=float)
    r_delta_c = np.asarray(rotation_from_attitude_error_deg(error), dtype=float)
    return nominal_model_rotation() @ (r_m_c @ r_delta_c @ r_m_c.T)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class GazeboCameraAttitudeAdapterTests(unittest.TestCase):
    def test_fixed_mapping_is_derived_from_nominal_frames(self) -> None:
        mapping = adapter.derive_fixed_model_to_optical_rotation()

        matrix_close(self, mapping, EXPECTED_R_M_C_FIXED)
        assert_rotation(self, mapping)

    def test_zero_attitude_matches_nominal_model_rotation(self) -> None:
        result = adapter.gazebo_model_rotation_from_camera_attitude_error(CameraAttitudeErrorDeg(0.0, 0.0, 0.0))

        matrix_close(self, result.rotation_world_model, nominal_model_rotation())
        tuple_close(self, result.quaternion_xyzw, (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))
        self.assertTrue(result.sdf_rpy.valid)
        self.assertEqual(result.sdf_rpy.status, adapter.SDF_RPY_STATUS_OK)
        assert result.sdf_rpy.rpy_rad is not None
        tuple_close(self, result.sdf_rpy.rpy_rad, (0.0, 0.0, math.pi / 2.0))

    def test_single_axis_rotations_recover_desired_optical_chain(self) -> None:
        cases = [
            CameraAttitudeErrorDeg(+0.1, 0.0, 0.0),
            CameraAttitudeErrorDeg(-0.1, 0.0, 0.0),
            CameraAttitudeErrorDeg(0.0, +0.1, 0.0),
            CameraAttitudeErrorDeg(0.0, -0.1, 0.0),
            CameraAttitudeErrorDeg(0.0, 0.0, +0.1),
            CameraAttitudeErrorDeg(0.0, 0.0, -0.1),
        ]
        r_m_c = np.asarray(adapter.derive_fixed_model_to_optical_rotation(), dtype=float)
        for error in cases:
            with self.subTest(error=error):
                result = adapter.gazebo_model_rotation_from_camera_attitude_error(error)
                recovered = np.asarray(result.rotation_world_model, dtype=float) @ r_m_c
                matrix_close(self, recovered, desired_optical_rotation(error))
                assert_rotation(self, result.rotation_world_model)

    def test_exact_sdf_rpy_diagnostic_for_single_axis_and_composite(self) -> None:
        cases = [
            CameraAttitudeErrorDeg(+0.1, 0.0, 0.0),
            CameraAttitudeErrorDeg(-0.1, 0.0, 0.0),
            CameraAttitudeErrorDeg(0.0, +0.1, 0.0),
            CameraAttitudeErrorDeg(0.0, -0.1, 0.0),
            CameraAttitudeErrorDeg(0.0, 0.0, +0.1),
            CameraAttitudeErrorDeg(0.0, 0.0, -0.1),
            CameraAttitudeErrorDeg(+0.10, -0.05, +0.08),
        ]
        for error in cases:
            with self.subTest(error=error):
                result = adapter.gazebo_model_rotation_from_camera_attitude_error(error)
                self.assertTrue(result.sdf_rpy.valid)
                assert result.sdf_rpy.rpy_rad is not None
                expected = (
                    math.radians(error.roll_deg),
                    math.radians(-error.pitch_deg),
                    wrap_pi(math.pi / 2.0 - math.radians(error.yaw_deg)),
                )
                tuple_close(self, result.sdf_rpy.rpy_rad, expected)

    def test_composite_absolute_path_matches_delta_path_and_optical_chain(self) -> None:
        error = CameraAttitudeErrorDeg(+0.10, -0.05, +0.08)
        result = adapter.gazebo_model_rotation_from_camera_attitude_error(error)
        r_m_c = np.asarray(adapter.derive_fixed_model_to_optical_rotation(), dtype=float)

        matrix_close(self, result.rotation_world_model, delta_path_model_rotation(error))
        matrix_close(self, np.asarray(result.rotation_world_model) @ r_m_c, desired_optical_rotation(error))
        self.assertEqual(len([field for field in result.__dataclass_fields__ if "translation" in field or "center" in field]), 0)

    def test_stereo_rotations_apply_single_camera_adapter_independently(self) -> None:
        left_error = CameraAttitudeErrorDeg(0.1, 0.0, 0.0)
        right_error = CameraAttitudeErrorDeg(0.0, -0.1, 0.0)
        zero = CameraAttitudeErrorDeg(0.0, 0.0, 0.0)

        left_only = adapter.gazebo_stereo_model_rotations_from_attitude_errors(
            StereoAttitudeErrorConfig(left=left_error, right=zero)
        )
        right_only = adapter.gazebo_stereo_model_rotations_from_attitude_errors(
            StereoAttitudeErrorConfig(left=zero, right=right_error)
        )
        common = adapter.gazebo_stereo_model_rotations_from_attitude_errors(
            StereoAttitudeErrorConfig(left=left_error, right=left_error)
        )
        relative = adapter.gazebo_stereo_model_rotations_from_attitude_errors(
            StereoAttitudeErrorConfig(left=CameraAttitudeErrorDeg(0.0, 0.0, -0.1), right=CameraAttitudeErrorDeg(0.0, 0.0, +0.1))
        )

        matrix_close(self, left_only.left.rotation_world_model, adapter.gazebo_model_rotation_from_camera_attitude_error(left_error).rotation_world_model)
        matrix_close(self, left_only.right.rotation_world_model, nominal_model_rotation())
        matrix_close(self, right_only.left.rotation_world_model, nominal_model_rotation())
        matrix_close(self, right_only.right.rotation_world_model, adapter.gazebo_model_rotation_from_camera_attitude_error(right_error).rotation_world_model)
        matrix_close(self, common.left.rotation_world_model, common.right.rotation_world_model)
        self.assertGreater(
            float(np.max(np.abs(np.asarray(relative.left.rotation_world_model) - np.asarray(relative.right.rotation_world_model)))),
            1.0e-6,
        )

    def test_quaternion_round_trip_order_and_canonical_sign(self) -> None:
        result = adapter.gazebo_model_rotation_from_camera_attitude_error(CameraAttitudeErrorDeg(+0.10, -0.05, +0.08))
        quat = result.quaternion_xyzw

        self.assertAlmostEqual(math.sqrt(sum(value * value for value in quat)), 1.0, delta=1.0e-12)
        self.assertGreater(quat[3], 0.0)
        matrix_close(self, adapter.rotation_matrix_from_quaternion_xyzw(quat), result.rotation_world_model)
        matrix_close(self, adapter.rotation_matrix_from_quaternion_xyzw(tuple(-value for value in quat)), result.rotation_world_model)

    def test_quaternion_invalid_inputs_are_rejected(self) -> None:
        invalid_values = [
            (0.0, 0.0, 0.0, 0.0),
            (True, 0.0, 0.0, 1.0),
            ("0", 0.0, 0.0, 1.0),
            (math.nan, 0.0, 0.0, 1.0),
            (math.inf, 0.0, 0.0, 1.0),
            (0.0, 0.0, 1.0),
        ]
        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises((TypeError, ValueError)):
                    adapter.rotation_matrix_from_quaternion_xyzw(value)

    def test_sdf_rpy_round_trip_and_angle_ranges(self) -> None:
        result = adapter.gazebo_model_rotation_from_camera_attitude_error(CameraAttitudeErrorDeg(+0.10, -0.05, +0.08))

        self.assertTrue(result.sdf_rpy.valid)
        assert result.sdf_rpy.rpy_rad is not None
        self.assertLessEqual(abs(result.sdf_rpy.rpy_rad[0]), math.pi)
        self.assertLessEqual(abs(result.sdf_rpy.rpy_rad[1]), math.pi / 2.0)
        self.assertLessEqual(abs(result.sdf_rpy.rpy_rad[2]), math.pi)
        self.assertLessEqual(result.sdf_rpy.round_trip_max_error, 1.0e-10)
        matrix_close(self, adapter.rotation_matrix_from_sdf_rpy_rad(*result.sdf_rpy.rpy_rad), result.rotation_world_model, 1.0e-10)

    def test_sdf_rpy_gimbal_lock_is_structured_invalid(self) -> None:
        rotation = adapter.rotation_matrix_from_sdf_rpy_rad(0.0, math.pi / 2.0, 0.0)

        result = adapter.rotation_matrix_to_sdf_rpy(rotation)

        self.assertFalse(result.valid)
        self.assertEqual(result.status, adapter.SDF_RPY_STATUS_GIMBAL_LOCK)
        self.assertIsNone(result.rpy_rad)
        self.assertIsNone(result.round_trip_max_error)

    def test_matrix_invalid_inputs_are_rejected(self) -> None:
        invalid_matrices = [
            ((1.0, 0.0), (0.0, 1.0)),
            ((True, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (("1", 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            ((math.nan, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            ((math.inf, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ((1.0, 0.1, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            ((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        ]
        for value in invalid_matrices:
            with self.subTest(value=value):
                with self.assertRaises((TypeError, ValueError)):
                    adapter.rotation_matrix_to_quaternion_xyzw(value)

    def test_scalar_invalid_inputs_are_rejected(self) -> None:
        invalid_scalars = [True, np.bool_(False), "0", math.nan, math.inf]
        for value in invalid_scalars:
            with self.subTest(value=value):
                with self.assertRaises((TypeError, ValueError)):
                    adapter.rotation_matrix_from_sdf_rpy_rad(value, 0.0, 0.0)

    def test_public_dataclasses_validate_contents(self) -> None:
        with self.assertRaises(ValueError):
            adapter.SdfRpyRotation(valid=True, status=adapter.SDF_RPY_STATUS_OK, rpy_rad=None, round_trip_max_error=0.0)
        with self.assertRaises(ValueError):
            adapter.SdfRpyRotation(valid=False, status=adapter.SDF_RPY_STATUS_GIMBAL_LOCK, rpy_rad=(0.0, 0.0, 0.0), round_trip_max_error=None)
        with self.assertRaises(TypeError):
            adapter.GazeboStereoModelRotations(left=object(), right=adapter.gazebo_model_rotation_from_camera_attitude_error(CameraAttitudeErrorDeg(0.0, 0.0, 0.0)))

    def test_module_is_pure_source_contract(self) -> None:
        source = Path(adapter.__file__).read_text(encoding="utf-8")
        self.assertNotIn("rclpy", source)
        self.assertNotIn("subprocess", source)
        self.assertNotIn("open(", source)
        self.assertNotIn("os.environ", source)
        self.assertNotIn("gz ", source)
        self.assertNotIn("launch", source)


if __name__ == "__main__":
    unittest.main()
