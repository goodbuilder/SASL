from __future__ import annotations

import copy
import math
import unittest
from types import SimpleNamespace

from pose_simulation.experiment_context import (
    CONTEXT_FRAME_MISMATCH,
    CONTEXT_EFFECTIVE_RPY_MISMATCH,
    CONTEXT_HASH_MISMATCH,
    CONTEXT_MUTATED_IN_RUN,
    CONTEXT_QUATERNION_ORDER_MISMATCH,
    CONTEXT_ROTATION_CONVENTION_MISMATCH,
    CONTEXT_SCHEMA_MISMATCH,
    CONTEXT_SOURCE_MISMATCH,
    SimulationContextStore,
    context_id_sha256,
    context_payload_from_message,
    validate_simulation_context,
)
from pose_simulation.pose_math import CAMERA_OPTICAL_FRAME


def _vec(values):
    return SimpleNamespace(x=values[0], y=values[1], z=values[2])


def _pose(position, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        position=_vec(position),
        orientation=SimpleNamespace(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
        ),
    )


def _context():
    message = SimpleNamespace(
        schema="simulation_experiment_context/v1",
        valid=True,
        status="VALID",
        source_application="Simulation",
        context_id_sha256="",
        run_id="run-a",
        case_id="case-a",
        scenario_name="fixture",
        simulation_epoch=0,
        simulation_epoch_valid=False,
        simulation_epoch_source="unavailable",
        repository="repo",
        branch="branch",
        commit_sha="commit",
        git_status="",
        generated_sdf_sha256="sdf-a",
        world_frame="gazebo_world_enu",
        camera_error_frame="camera_optical_rdf",
        camera_error_axes="right_down_forward",
        rotation_composition="R_down_yaw@R_right_pitch@R_forward_roll",
        quaternion_order="xyzw",
        nominal_baseline_m=25.0,
        zoom_index=0.0,
        zoom_index_valid=False,
        zoom_index_source="unavailable",
        camera_image_width_px=1280,
        camera_image_height_px=720,
        camera_update_rate_hz=20.0,
        camera_horizontal_fov_rad=0.154997710698,
        observation_mode="fixture",
        nominal_left_optical_pose_world_enu=_pose((-12.5, 0.0, 5.0)),
        nominal_right_optical_pose_world_enu=_pose((12.5, 0.0, 5.0)),
        left_direct_rpy_deg=_vec((0.0, 0.0, 0.0)),
        right_direct_rpy_deg=_vec((0.0, 0.0, 0.0)),
        common_rpy_deg=_vec((0.0, 0.0, 0.3)),
        relative_rpy_deg=_vec((0.0, 0.0, 0.2)),
        left_effective_rpy_deg=_vec((0.0, 0.0, 0.2)),
        right_effective_rpy_deg=_vec((0.0, 0.0, 0.4)),
    )
    message.context_id_sha256 = context_id_sha256(context_payload_from_message(message))
    return message


class ExperimentContextTests(unittest.TestCase):
    def test_valid_context_recomputes_effective_rpy_and_hash(self):
        result = validate_simulation_context(_context())
        self.assertTrue(result.valid)
        self.assertEqual(result.status, "CONTEXT_VALID")
        self.assertEqual(result.context.left_effective_rpy_deg, (0.0, 0.0, 0.2))
        self.assertEqual(result.context.right_effective_rpy_deg, (0.0, 0.0, 0.4))
        self.assertEqual(result.context.nominal_left_pose_world.frame_id, CAMERA_OPTICAL_FRAME)
        self.assertEqual(result.context.nominal_right_pose_world.frame_id, CAMERA_OPTICAL_FRAME)

    def test_quaternion_sign_is_same_context_identity(self):
        positive = _context()
        negative = copy.deepcopy(positive)
        negative.nominal_left_optical_pose_world_enu.orientation.x *= -1.0
        negative.nominal_left_optical_pose_world_enu.orientation.y *= -1.0
        negative.nominal_left_optical_pose_world_enu.orientation.z *= -1.0
        negative.nominal_left_optical_pose_world_enu.orientation.w *= -1.0
        self.assertEqual(
            context_id_sha256(context_payload_from_message(positive)),
            context_id_sha256(context_payload_from_message(negative)),
        )
        self.assertTrue(validate_simulation_context(negative).valid)

    def test_bad_schema_hash_and_effective_values_fail_closed(self):
        bad_schema = _context()
        bad_schema.schema = "other/v1"
        self.assertEqual(validate_simulation_context(bad_schema).status, CONTEXT_SCHEMA_MISMATCH)

        bad_hash = _context()
        bad_hash.context_id_sha256 = "0" * 64
        self.assertEqual(validate_simulation_context(bad_hash).status, CONTEXT_HASH_MISMATCH)

        bad_effective = _context()
        bad_effective.left_effective_rpy_deg.z = 0.21
        self.assertEqual(validate_simulation_context(bad_effective).status, CONTEXT_EFFECTIVE_RPY_MISMATCH)

    def test_source_frame_and_rotation_contracts_fail_closed(self):
        bad_source = _context()
        bad_source.source_application = "PoseSimulation"
        self.assertEqual(validate_simulation_context(bad_source).status, CONTEXT_SOURCE_MISMATCH)

        bad_frame = _context()
        bad_frame.world_frame = "px4_local_ned"
        self.assertEqual(validate_simulation_context(bad_frame).status, CONTEXT_FRAME_MISMATCH)

        bad_axes = _context()
        bad_axes.camera_error_axes = "forward_right_down"
        self.assertEqual(validate_simulation_context(bad_axes).status, CONTEXT_FRAME_MISMATCH)

        bad_rotation = _context()
        bad_rotation.rotation_composition = "R_wrong"
        self.assertEqual(
            validate_simulation_context(bad_rotation).status,
            CONTEXT_ROTATION_CONVENTION_MISMATCH,
        )

        bad_quaternion_order = _context()
        bad_quaternion_order.quaternion_order = "wxyz"
        self.assertEqual(
            validate_simulation_context(bad_quaternion_order).status,
            CONTEXT_QUATERNION_ORDER_MISMATCH,
        )

    def test_missing_sdf_identity_is_rejected(self):
        missing = _context()
        missing.generated_sdf_sha256 = ""
        missing.context_id_sha256 = context_id_sha256(context_payload_from_message(missing))
        result = validate_simulation_context(missing)
        self.assertFalse(result.valid)
        self.assertEqual(result.status, CONTEXT_HASH_MISMATCH)

    def test_context_mutation_in_same_run_is_rejected(self):
        store = SimulationContextStore()
        first = _context()
        self.assertTrue(store.accept(first).valid)
        second = _context()
        second.case_id = "case-mutated"
        second.context_id_sha256 = context_id_sha256(context_payload_from_message(second))
        result = store.accept(second)
        self.assertFalse(result.valid)
        self.assertEqual(result.status, CONTEXT_MUTATED_IN_RUN)
        self.assertIsNone(store.context)

    def test_nonfinite_payload_is_rejected(self):
        bad = _context()
        bad.camera_update_rate_hz = math.nan
        result = validate_simulation_context(bad)
        self.assertFalse(result.valid)


if __name__ == "__main__":
    unittest.main()
