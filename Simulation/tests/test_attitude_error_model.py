from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

import stereo_ground_sim.attitude_error_model as attitude_error_module  # noqa: E402
from stereo_ground_sim.attitude_error_model import (  # noqa: E402
    AttitudeErrorDecomposition,
    CameraAttitudeErrorDeg,
    RawAttitudeMeasurement,
    StereoAttitudeErrorConfig,
    StereoCameraTransforms,
    camera_transform_with_fixed_center,
    decompose_common_relative_attitude,
    evaluate_raw_attitude_measurement,
    project_stereo_with_actual_transforms,
    reconstruct_raw_with_nominal_transforms,
    rotation_from_attitude_error_deg,
    stereo_actual_camera_transforms,
)
from stereo_ground_sim.coordinate_frames import (  # noqa: E402
    FrameId,
    FramePoint,
    RigidTransform3D,
    transform_point,
)
from stereo_ground_sim.ideal_projection import ProjectionStatus  # noqa: E402
from stereo_ground_sim.stereo_triangulation_adapter import (  # noqa: E402
    nominal_camera_transforms,
    pinhole_intrinsics_from_horizontal_fov,
)
from stereo_ground_sim.triangulation_core import TriangulationStatus  # noqa: E402


WIDTH_PX = 1280
HEIGHT_PX = 720
HORIZONTAL_FOV_RAD = 0.154997710698
BASELINE_M = 25.0
PIXEL_TOL = 1.0e-9


def norm(values) -> float:
    return float(np.linalg.norm(np.asarray(values, dtype=float)))


def point_frd(values) -> FramePoint:
    return FramePoint(tuple(float(value) for value in values), FrameId.BASELINE_CENTER_FRD)


def intrinsics():
    return pinhole_intrinsics_from_horizontal_fov(
        width_px=WIDTH_PX,
        height_px=HEIGHT_PX,
        horizontal_fov_rad=HORIZONTAL_FOV_RAD,
    )


def zero_config() -> StereoAttitudeErrorConfig:
    zero = CameraAttitudeErrorDeg(0.0, 0.0, 0.0)
    return StereoAttitudeErrorConfig(left=zero, right=zero)


def tuple_close(testcase: unittest.TestCase, actual, expected, delta: float = 1.0e-12) -> None:
    testcase.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        testcase.assertAlmostEqual(float(actual_value), float(expected_value), delta=delta)


def matrix_close(testcase: unittest.TestCase, actual, expected, delta: float = 1.0e-12) -> None:
    actual_arr = np.asarray(actual, dtype=float)
    expected_arr = np.asarray(expected, dtype=float)
    testcase.assertEqual(actual_arr.shape, (3, 3))
    testcase.assertLess(float(np.max(np.abs(actual_arr - expected_arr))), delta)


def assert_rotation_valid(testcase: unittest.TestCase, rotation) -> None:
    arr = np.asarray(rotation, dtype=float)
    testcase.assertLess(float(np.max(np.abs(arr.T @ arr - np.eye(3)))), 1.0e-12)
    testcase.assertLess(abs(float(np.linalg.det(arr)) - 1.0), 1.0e-12)


def center_of(transform: RigidTransform3D):
    return transform_point(FramePoint((0.0, 0.0, 0.0), FrameId.CAMERA_OPTICAL_RDF), transform).xyz_m


class AttitudeErrorModelTests(unittest.TestCase):
    def test_module_docstring_records_rotation_contract(self) -> None:
        doc = attitude_error_module.__doc__

        self.assertIsNotNone(doc)
        assert doc is not None
        self.assertIn("fixed-axis", doc)
        self.assertIn("nominal Camera Optical RDF", doc)
        self.assertIn("R_delta_C = R_Y(yaw) @ R_X(pitch) @ R_Z(roll)", doc)
        self.assertIn("R_B_C_actual = R_B_C_nominal @ R_delta_C", doc)
        self.assertIn("does not claim equivalence with Gazebo SDF Euler semantics", doc)

    def test_rotation_contract_single_axis_and_composite_order(self) -> None:
        angle = math.radians(0.5)
        c = math.cos(angle)
        s = math.sin(angle)

        expected_roll = ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
        expected_pitch = ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))
        expected_yaw = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))

        matrix_close(self, rotation_from_attitude_error_deg(CameraAttitudeErrorDeg(0.5, 0.0, 0.0)), expected_roll)
        matrix_close(self, rotation_from_attitude_error_deg(CameraAttitudeErrorDeg(0.0, 0.5, 0.0)), expected_pitch)
        matrix_close(self, rotation_from_attitude_error_deg(CameraAttitudeErrorDeg(0.0, 0.0, 0.5)), expected_yaw)

        roll = math.radians(0.3)
        pitch = math.radians(0.2)
        yaw = math.radians(0.1)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rz = np.asarray(((cr, -sr, 0.0), (sr, cr, 0.0), (0.0, 0.0, 1.0)))
        rx = np.asarray(((1.0, 0.0, 0.0), (0.0, cp, -sp), (0.0, sp, cp)))
        ry = np.asarray(((cy, 0.0, sy), (0.0, 1.0, 0.0), (-sy, 0.0, cy)))
        expected = ry @ rx @ rz
        selected = rotation_from_attitude_error_deg(CameraAttitudeErrorDeg(0.3, 0.2, 0.1))

        matrix_close(self, selected, expected)
        assert_rotation_valid(self, selected)

    def test_zero_error_nominal_equivalence_and_raw_round_trip(self) -> None:
        camera = intrinsics()
        gt = point_frd((1852.0, 20.0, -6.0))
        measurement = evaluate_raw_attitude_measurement(gt, camera, BASELINE_M, zero_config())

        self.assertIsInstance(measurement, RawAttitudeMeasurement)
        self.assertTrue(measurement.valid)
        self.assertEqual(measurement.status, "ok")
        self.assertAlmostEqual(measurement.actual_projection.left.u_px, 784.6293305394881, delta=PIXEL_TOL)
        self.assertAlmostEqual(measurement.actual_projection.left.v_px, 333.2992005157868, delta=PIXEL_TOL)
        self.assertAlmostEqual(measurement.actual_projection.right.u_px, 673.3759993552665, delta=PIXEL_TOL)
        self.assertAlmostEqual(measurement.actual_projection.right.v_px, 333.2992005157868, delta=PIXEL_TOL)
        self.assertAlmostEqual(measurement.actual_projection.signed_disparity_px, 111.25333118422168, delta=PIXEL_TOL)
        self.assertLess(abs(measurement.actual_projection.vertical_disparity_px), 1.0e-9)
        self.assertLess(measurement.raw_error_norm_m, 1.0e-6)
        self.assertLess(measurement.raw_triangulation.ray_gap_m, 1.0e-9)

        nominal_left, nominal_right = nominal_camera_transforms(baseline_m=BASELINE_M)
        for actual, nominal in (
            (measurement.actual_transforms.left, nominal_left),
            (measurement.actual_transforms.right, nominal_right),
        ):
            rotation_delta = np.asarray(actual.rotation_target_source) - np.asarray(nominal.rotation_target_source)
            self.assertLess(float(np.max(np.abs(rotation_delta))), 1.0e-12)
            self.assertLess(norm(np.asarray(center_of(actual)) - np.asarray(center_of(nominal))), 1.0e-12)

    def test_camera_center_invariant_for_arbitrary_errors(self) -> None:
        config = StereoAttitudeErrorConfig(
            left=CameraAttitudeErrorDeg(+0.7, -0.4, +0.2),
            right=CameraAttitudeErrorDeg(-0.3, +0.6, -0.5),
        )

        transforms = stereo_actual_camera_transforms(BASELINE_M, config)

        tuple_close(self, center_of(transforms.left), (0.0, -12.5, 0.0))
        tuple_close(self, center_of(transforms.right), (0.0, +12.5, 0.0))

    def test_origin_recalculation_prevents_camera_center_shift(self) -> None:
        left_nominal, _ = nominal_camera_transforms(baseline_m=BASELINE_M)
        error = CameraAttitudeErrorDeg(0.0, 0.0, 1.0)
        good = camera_transform_with_fixed_center(left_nominal, error)

        bad = RigidTransform3D(
            FrameId.CAMERA_OPTICAL_RDF,
            FrameId.BASELINE_CENTER_FRD,
            good.rotation_target_source,
            left_nominal.origin_target_in_source_m,
        )
        good_delta = np.asarray(center_of(good)) - np.asarray((0.0, -12.5, 0.0))
        bad_delta = np.asarray(center_of(bad)) - np.asarray((0.0, -12.5, 0.0))

        self.assertLess(norm(good_delta), 1.0e-12)
        self.assertGreater(abs(float(bad_delta[0])), 0.21)
        self.assertGreater(norm(bad_delta), 0.21)

    def test_common_relative_metadata(self) -> None:
        common = decompose_common_relative_attitude(
            StereoAttitudeErrorConfig(
                left=CameraAttitudeErrorDeg(0.0, 0.0, +0.1),
                right=CameraAttitudeErrorDeg(0.0, 0.0, +0.1),
            )
        )
        self.assertIsInstance(common, AttitudeErrorDecomposition)
        self.assertAlmostEqual(common.common_yaw_deg, +0.1, delta=1.0e-12)
        self.assertAlmostEqual(common.relative_yaw_deg, 0.0, delta=1.0e-12)

        relative = decompose_common_relative_attitude(
            StereoAttitudeErrorConfig(
                left=CameraAttitudeErrorDeg(0.0, 0.0, -0.1),
                right=CameraAttitudeErrorDeg(0.0, 0.0, +0.1),
            )
        )
        self.assertAlmostEqual(relative.common_yaw_deg, 0.0, delta=1.0e-12)
        self.assertAlmostEqual(relative.relative_yaw_deg, +0.2, delta=1.0e-12)

    def test_single_axis_pixel_disparity_and_dominant_raw_error_signs(self) -> None:
        camera = intrinsics()
        gt = point_frd((1852.0, 20.0, -6.0))
        baseline = evaluate_raw_attitude_measurement(gt, camera, BASELINE_M, zero_config())

        cases = (
            ("left", "roll", +1, -1, -1, -1),
            ("left", "roll", -1, +1, +1, +1),
            ("left", "pitch", +1, -1, +1, -1),
            ("left", "pitch", -1, +1, -1, +1),
            ("left", "yaw", +1, -1, +1, -1),
            ("left", "yaw", -1, +1, -1, +1),
            ("right", "roll", +1, -1, -1, +1),
            ("right", "roll", -1, +1, +1, -1),
            ("right", "pitch", +1, -1, +1, +1),
            ("right", "pitch", -1, +1, -1, -1),
            ("right", "yaw", +1, -1, +1, +1),
            ("right", "yaw", -1, +1, -1, -1),
        )
        for side, axis, sign, expected_du_sign, expected_dv_sign, expected_disparity_sign in cases:
            with self.subTest(side=side, axis=axis, sign=sign):
                error = CameraAttitudeErrorDeg(
                    roll_deg=0.01 * sign if axis == "roll" else 0.0,
                    pitch_deg=0.01 * sign if axis == "pitch" else 0.0,
                    yaw_deg=0.01 * sign if axis == "yaw" else 0.0,
                )
                config = StereoAttitudeErrorConfig(
                    left=error if side == "left" else CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
                    right=error if side == "right" else CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
                )
                actual = evaluate_raw_attitude_measurement(gt, camera, BASELINE_M, config)
                if side == "left":
                    du = actual.actual_projection.left.u_px - baseline.actual_projection.left.u_px
                    dv = actual.actual_projection.left.v_px - baseline.actual_projection.left.v_px
                else:
                    du = actual.actual_projection.right.u_px - baseline.actual_projection.right.u_px
                    dv = actual.actual_projection.right.v_px - baseline.actual_projection.right.v_px
                disparity_delta = (
                    actual.actual_projection.signed_disparity_px - baseline.actual_projection.signed_disparity_px
                )
                self.assertGreater(abs(du), 1.0e-6)
                self.assertGreater(abs(dv), 1.0e-6)
                self.assertGreater(abs(disparity_delta), 1.0e-6)
                self.assertGreater(du * expected_du_sign, 0.0)
                self.assertGreater(dv * expected_dv_sign, 0.0)
                self.assertGreater(disparity_delta * expected_disparity_sign, 0.0)

                raw_forward, _, raw_down = actual.raw_error_baseline_center_frd_m
                if side == "left" and axis == "roll" and sign > 0:
                    self.assertGreater(raw_forward, +0.07)
                if side == "right" and axis == "roll" and sign > 0:
                    self.assertLess(raw_forward, -0.07)
                if side == "left" and axis == "yaw" and sign > 0:
                    self.assertGreater(raw_forward, +24.0)
                    self.assertAlmostEqual(du, -1.4388772994, delta=1.0e-6)
                    self.assertAlmostEqual(disparity_delta, -1.4388772994, delta=1.0e-6)
                    self.assertAlmostEqual(raw_forward, +24.2663935314, delta=1.0e-5)
                if side == "right" and axis == "yaw" and sign > 0:
                    self.assertLess(raw_forward, -23.0)
                    self.assertAlmostEqual(du, -1.4384613081, delta=1.0e-6)
                    self.assertAlmostEqual(disparity_delta, +1.4384613081, delta=1.0e-6)
                    self.assertAlmostEqual(raw_forward, -23.6399677599, delta=1.0e-5)
                if side == "left" and axis == "pitch" and sign > 0:
                    self.assertLess(raw_forward, -0.30)
                    self.assertGreater(raw_down, +0.16)
                if side == "right" and axis == "pitch" and sign > 0:
                    self.assertLess(raw_forward, -0.30)
                    self.assertGreater(raw_down, +0.16)

    def test_non_commutativity_selected_order(self) -> None:
        error = CameraAttitudeErrorDeg(0.3, 0.2, 0.1)
        selected = np.asarray(rotation_from_attitude_error_deg(error), dtype=float)

        roll = math.radians(0.3)
        pitch = math.radians(0.2)
        yaw = math.radians(0.1)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rz = np.asarray(((cr, -sr, 0.0), (sr, cr, 0.0), (0.0, 0.0, 1.0)))
        rx = np.asarray(((1.0, 0.0, 0.0), (0.0, cp, -sp), (0.0, sp, cp)))
        ry = np.asarray(((cy, 0.0, sy), (0.0, 1.0, 0.0), (-sy, 0.0, cy)))
        alternate = rz @ rx @ ry
        hardcoded_reference = np.asarray(
            (
                (0.99998480108082743, -0.00522986360717438, 0.00174531773276064),
                (0.00523593193213612, 0.99998019998872945, -0.00349065141522373),
                (-0.00172702754464805, 0.00349973672594370, 0.99999238458035722),
            )
        )

        self.assertGreater(float(np.max(np.abs(selected - alternate))), 1.0e-8)
        self.assertLess(float(np.max(np.abs(selected - hardcoded_reference))), 1.0e-12)

        camera = intrinsics()
        gt = point_frd((1852.0, 20.0, -6.0))
        selected_config = StereoAttitudeErrorConfig(error, CameraAttitudeErrorDeg(0.0, 0.0, 0.0))
        selected_projection = evaluate_raw_attitude_measurement(gt, camera, BASELINE_M, selected_config)
        left_nominal, right_nominal = nominal_camera_transforms(baseline_m=BASELINE_M)
        alt_transform = RigidTransform3D(
            FrameId.CAMERA_OPTICAL_RDF,
            FrameId.BASELINE_CENTER_FRD,
            left_nominal.rotation_target_source,
            left_nominal.origin_target_in_source_m,
        )
        alt_rotation_transform = RigidTransform3D(
            FrameId.CAMERA_OPTICAL_RDF,
            FrameId.BASELINE_CENTER_FRD,
            np.asarray(left_nominal.rotation_target_source) @ alternate,
            (0.0, 0.0, 0.0),
        )
        # Recompute the alternate origin with the same fixed center contract.
        center_b = np.asarray(center_of(left_nominal))
        alt_actual = RigidTransform3D(
            FrameId.CAMERA_OPTICAL_RDF,
            FrameId.BASELINE_CENTER_FRD,
            alt_rotation_transform.rotation_target_source,
            tuple(float(v) for v in (-(np.asarray(alt_rotation_transform.rotation_target_source).T @ center_b))),
        )
        alt_projection = project_stereo_with_actual_transforms(
            gt,
            StereoCameraTransforms(alt_actual, right_nominal, BASELINE_M),
            camera,
        )
        self.assertGreater(abs(selected_projection.actual_projection.left.u_px - alt_projection.left.u_px), 1.0e-8)
        self.assertGreater(abs(selected_projection.actual_projection.left.v_px - alt_projection.left.v_px), 1.0e-8)
        self.assertIsInstance(alt_transform, RigidTransform3D)

    def test_structured_invalid_and_input_validation(self) -> None:
        for value in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                CameraAttitudeErrorDeg(value, 0.0, 0.0)
        for value in (True, np.bool_(False), "0.0"):
            with self.assertRaises(TypeError):
                CameraAttitudeErrorDeg(value, 0.0, 0.0)
        with self.assertRaises(TypeError):
            StereoAttitudeErrorConfig(left=(0.0, 0.0, 0.0), right=CameraAttitudeErrorDeg(0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            stereo_actual_camera_transforms(-1.0, zero_config())
        with self.assertRaises(ValueError):
            evaluate_raw_attitude_measurement(
                FramePoint((1.0, 2.0, 3.0), FrameId.GAZEBO_WORLD_ENU),
                intrinsics(),
                BASELINE_M,
                zero_config(),
            )

        wrong_transform = RigidTransform3D(
            FrameId.BASELINE_CENTER_FRD,
            FrameId.BASELINE_CENTER_FRD,
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (0.0, 0.0, 0.0),
        )
        with self.assertRaises(ValueError):
            camera_transform_with_fixed_center(wrong_transform, CameraAttitudeErrorDeg(0.0, 0.0, 0.0))

    def test_zero_baseline_returns_structured_raw_invalid(self) -> None:
        measurement = evaluate_raw_attitude_measurement(
            point_frd((1000.0, 0.0, 0.0)),
            intrinsics(),
            0.0,
            zero_config(),
        )

        self.assertFalse(measurement.valid)
        self.assertEqual(measurement.status, "raw_triangulation_invalid:zero_baseline")
        self.assertEqual(measurement.raw_triangulation.status, TriangulationStatus.ZERO_BASELINE)

    def test_projection_structured_invalid_cases(self) -> None:
        camera = intrinsics()

        behind = evaluate_raw_attitude_measurement(point_frd((-10.0, 0.0, 0.0)), camera, BASELINE_M, zero_config())
        self.assertFalse(behind.valid)
        self.assertEqual(behind.status, "projection_invalid:both")
        self.assertEqual(behind.actual_projection.left.status, ProjectionStatus.NON_POSITIVE_DEPTH)

        out_of_bounds = evaluate_raw_attitude_measurement(
            point_frd((1000.0, 70.0, 0.0)),
            camera,
            BASELINE_M,
            zero_config(),
        )
        self.assertFalse(out_of_bounds.valid)
        self.assertEqual(out_of_bounds.status, "projection_invalid:left")
        self.assertEqual(out_of_bounds.actual_projection.left.status, ProjectionStatus.OUT_OF_BOUNDS)

        outside_clip = evaluate_raw_attitude_measurement(
            point_frd((1000.0, 0.0, 0.0)),
            camera,
            BASELINE_M,
            zero_config(),
            clip_near_m=1500.0,
            clip_far_m=2000.0,
        )
        self.assertFalse(outside_clip.valid)
        self.assertEqual(outside_clip.status, "projection_invalid:both")
        self.assertEqual(outside_clip.actual_projection.left.status, ProjectionStatus.OUTSIDE_CLIP_RANGE)

    def test_reconstruct_rejects_invalid_projection(self) -> None:
        invalid = evaluate_raw_attitude_measurement(point_frd((-10.0, 0.0, 0.0)), intrinsics(), BASELINE_M, zero_config())
        with self.assertRaises(ValueError):
            reconstruct_raw_with_nominal_transforms(invalid.actual_projection, intrinsics(), BASELINE_M)


if __name__ == "__main__":
    unittest.main()
