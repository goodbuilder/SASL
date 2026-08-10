from __future__ import annotations

import math
import unittest

from pose_simulation.attitude_provenance import (
    INJECTION_VALID,
    optical_rpy_error_rotation,
    validate_attitude_provenance,
)
from pose_simulation.experiment_context import ValidatedSimulationExperimentContext
from pose_simulation.pose_math import IDENTITY_ROTATION, Pose3D, matrix_multiply


def _context(left=(0.0, 0.0, 0.2), right=(0.0, 0.0, 0.4)):
    return ValidatedSimulationExperimentContext(
        context_id_sha256="fixture",
        run_id="run",
        case_id="case",
        scenario_name="fixture",
        simulation_epoch=0,
        simulation_epoch_valid=False,
        simulation_epoch_source="unavailable",
        generated_sdf_sha256="sdf",
        nominal_left_pose_world=Pose3D((-12.5, 0.0, 5.0), IDENTITY_ROTATION, "camera_optical_rdf"),
        nominal_right_pose_world=Pose3D((12.5, 0.0, 5.0), IDENTITY_ROTATION, "camera_optical_rdf"),
        left_direct_rpy_deg=(0.0, 0.0, 0.0),
        right_direct_rpy_deg=(0.0, 0.0, 0.0),
        common_rpy_deg=(0.0, 0.0, 0.3),
        relative_rpy_deg=(0.0, 0.0, 0.2),
        left_effective_rpy_deg=left,
        right_effective_rpy_deg=right,
        world_frame="gazebo_world_enu",
        camera_error_frame="camera_optical_rdf",
        camera_error_axes="right_down_forward",
        rotation_composition="R_down_yaw@R_right_pitch@R_forward_roll",
        quaternion_order="xyzw",
    )


def _pose(position, rotation):
    return Pose3D(position, rotation, "camera_optical_rdf")


class AttitudeProvenanceTests(unittest.TestCase):
    def test_commanded_equals_observed_has_zero_so3_residual(self):
        context = _context()
        left_rotation = optical_rpy_error_rotation(*context.left_effective_rpy_deg)
        right_rotation = optical_rpy_error_rotation(*context.right_effective_rpy_deg)
        result = validate_attitude_provenance(
            context,
            ideal_left_pose_world=_pose((-12.5, 0.0, 5.0), IDENTITY_ROTATION),
            ideal_right_pose_world=_pose((12.5, 0.0, 5.0), IDENTITY_ROTATION),
            actual_left_pose_world=_pose((-12.5, 0.0, 5.0), left_rotation),
            actual_right_pose_world=_pose((12.5, 0.0, 5.0), right_rotation),
        )
        self.assertTrue(result.valid)
        self.assertEqual(result.status, INJECTION_VALID)
        self.assertLess(result.left_injection_residual_rotation_deg, 1.0e-10)
        self.assertLess(result.right_injection_residual_rotation_deg, 1.0e-10)
        self.assertLess(result.relative_residual_rotation_deg, 1.0e-10)

    def test_composite_rpy_round_trip(self):
        rotation = optical_rpy_error_rotation(0.30, -0.20, 0.40)
        context = _context(left=(0.30, -0.20, 0.40), right=(0.30, -0.20, 0.40))
        result = validate_attitude_provenance(
            context,
            ideal_left_pose_world=_pose((-12.5, 0.0, 5.0), IDENTITY_ROTATION),
            ideal_right_pose_world=_pose((12.5, 0.0, 5.0), IDENTITY_ROTATION),
            actual_left_pose_world=_pose((-12.5, 0.0, 5.0), rotation),
            actual_right_pose_world=_pose((12.5, 0.0, 5.0), rotation),
        )
        self.assertAlmostEqual(result.left_commanded_effective_rpy_deg[0], 0.30, delta=1.0e-10)
        self.assertAlmostEqual(result.left_commanded_effective_rpy_deg[1], -0.20, delta=1.0e-10)
        self.assertAlmostEqual(result.left_commanded_effective_rpy_deg[2], 0.40, delta=1.0e-10)
        self.assertLess(result.left_injection_residual_rotation_deg, 1.0e-10)

    def test_off_axis_target_separates_geometry_from_injection(self):
        context = _context()
        geometry = optical_rpy_error_rotation(0.0, 0.0, 1.0)
        injection = optical_rpy_error_rotation(*context.left_effective_rpy_deg)
        result = validate_attitude_provenance(
            context,
            ideal_left_pose_world=_pose((-12.5, 0.0, 5.0), geometry),
            ideal_right_pose_world=_pose((12.5, 0.0, 5.0), geometry),
            actual_left_pose_world=_pose((-12.5, 0.0, 5.0), injection),
            actual_right_pose_world=_pose((12.5, 0.0, 5.0), optical_rpy_error_rotation(*context.right_effective_rpy_deg)),
        )
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.left_gt_ideal_to_nominal_rpy_deg[2], -1.0, delta=1.0e-10)
        self.assertLess(result.left_composition_closure_rotation_deg, 1.0e-10)
        self.assertNotAlmostEqual(result.left_gt_ideal_to_actual_rpy_deg[2], context.left_effective_rpy_deg[2], delta=1.0e-3)

    def test_threshold_failure_is_not_reported_as_valid(self):
        context = _context()
        actual = optical_rpy_error_rotation(0.0, 0.0, 0.21)
        result = validate_attitude_provenance(
            context,
            ideal_left_pose_world=_pose((-12.5, 0.0, 5.0), IDENTITY_ROTATION),
            ideal_right_pose_world=_pose((12.5, 0.0, 5.0), IDENTITY_ROTATION),
            actual_left_pose_world=_pose((-12.5, 0.0, 5.0), actual),
            actual_right_pose_world=_pose((12.5, 0.0, 5.0), actual),
            max_injection_residual_deg=1.0e-6,
        )
        self.assertFalse(result.valid)


if __name__ == "__main__":
    unittest.main()
