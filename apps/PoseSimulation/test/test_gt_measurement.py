from __future__ import annotations

import json
import math
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import yaml

from pose_simulation.input_adapters import adapt_camera_pose_diagnostic, adapt_ground_truth_pose
from pose_simulation.input_types import AlignedPoseInputs
from pose_simulation.measurement import (
    GtMeasurementEngine,
    evaluate_gt_aligned_inputs,
    nominal_extrinsics_from_contract,
)
from pose_simulation.pose_math import (
    GAZEBO_LINK_FLU_FROM_OPTICAL_RDF,
    compute_look_at_rig_rotation,
    derive_camera_optical_pose,
    matrix_multiply,
    matrix_to_quaternion_xyzw,
    transpose,
    validate_pose_contract,
)
from pose_simulation.time_alignment import align_static_snapshot


CONTRACT_PATH = Path(__file__).parents[1] / "config" / "pose_contract.yaml"


def _gt_message(stamp_ns: int, target: tuple[float, float, float]):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
            frame_id="gazebo_world_enu",
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=target[0], y=target[1], z=target[2]),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _rotation_down(angle_rad: float):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _camera_payload(
    *,
    side: str,
    position: tuple[float, float, float],
    optical_rotation,
    stamp_ns: int = 1_000_000_000,
) -> dict[str, object]:
    entity_rotation = matrix_multiply(
        optical_rotation,
        transpose(GAZEBO_LINK_FLU_FROM_OPTICAL_RDF),
    )
    return {
        "schema": "camera_pose_comparison/v1",
        "camera_label": side,
        "gazebo_stamp_ns": stamp_ns,
        "observed_optical_position_world_enu_m": list(position),
        "observed_optical_orientation_world_from_entity_xyzw": list(
            matrix_to_quaternion_xyzw(entity_rotation)
        ),
        "observed_optical_entity_name": f"{side}_stereo_camera::optical_link",
        "observed_optical_entity_id": 100 if side == "left" else 101,
        "generated_sdf_sha256": "fixture",
    }


def _contract():
    with CONTRACT_PATH.open("r", encoding="utf-8") as stream:
        return validate_pose_contract(yaml.safe_load(stream))


def _aligned(
    target: tuple[float, float, float],
    *,
    left_yaw: float,
    right_yaw: float,
    stamp_ns: int = 10_250_000_000,
):
    contract = _contract()
    nominal = nominal_extrinsics_from_contract(contract)
    look = compute_look_at_rig_rotation((0.0, 0.0, 0.0), target)
    assert look.valid and look.rotation_world_from_rig is not None
    left_ideal = derive_camera_optical_pose(
        (0.0, 0.0, 0.0),
        look.rotation_world_from_rig,
        nominal.left_translation_rig_m,
        nominal.left_rotation_rig_from_optical,
    )
    right_ideal = derive_camera_optical_pose(
        (0.0, 0.0, 0.0),
        look.rotation_world_from_rig,
        nominal.right_translation_rig_m,
        nominal.right_rotation_rig_from_optical,
    )
    left_actual_rotation = matrix_multiply(
        left_ideal.rotation_world_from_frame, _rotation_down(math.radians(left_yaw))
    )
    right_actual_rotation = matrix_multiply(
        right_ideal.rotation_world_from_frame, _rotation_down(math.radians(right_yaw))
    )
    gt = adapt_ground_truth_pose(
        _gt_message(stamp_ns, target),
        source_topic="/simulation/ground_truth/vehicle_reference_link",
        reference_name="vehicle_reference_link",
        clock_domain="gazebo_sim_time",
    )
    left = adapt_camera_pose_diagnostic(
        json.dumps(
            _camera_payload(
                side="left",
                position=left_ideal.position_world_enu_m,
                optical_rotation=left_actual_rotation,
            )
        ),
        expected_side="left",
        source_topic="/simulation/camera_pose_diagnostics/left",
        expected_entity_name="left_stereo_camera::optical_link",
        clock_domain="gazebo_sim_time",
    )
    right = adapt_camera_pose_diagnostic(
        json.dumps(
            _camera_payload(
                side="right",
                position=right_ideal.position_world_enu_m,
                optical_rotation=right_actual_rotation,
            )
        ),
        expected_side="right",
        source_topic="/simulation/camera_pose_diagnostics/right",
        expected_entity_name="right_stereo_camera::optical_link",
        clock_domain="gazebo_sim_time",
    )
    return align_static_snapshot(gt, left, right)


class GtMeasurementTests(unittest.TestCase):
    def test_known_left_right_attitude_and_baseline_values(self):
        aligned = _aligned((0.0, 1000.0, 0.0), left_yaw=0.10, right_yaw=0.14)
        result = evaluate_gt_aligned_inputs(aligned, _contract(), evaluation_id=1)
        self.assertTrue(result.valid)
        assert result.left_error and result.right_error and result.relative_error and result.baseline
        self.assertAlmostEqual(result.left_error.rotation.yaw_error_deg, 0.10, delta=1.0e-9)
        self.assertAlmostEqual(result.right_error.rotation.yaw_error_deg, 0.14, delta=1.0e-9)
        self.assertAlmostEqual(result.relative_error.yaw_error_deg, 0.04, delta=1.0e-9)
        self.assertAlmostEqual(result.baseline.actual_length_m, 25.0, delta=1.0e-10)
        self.assertAlmostEqual(result.baseline.length_error_m, 0.0, delta=1.0e-10)
        self.assertAlmostEqual(result.baseline.direction_error_deg, 0.0, delta=1.0e-10)
        self.assertEqual(result.evaluation_id, 1)
        self.assertEqual(result.evaluation_stamp_ns, 10_250_000_000)

    def test_target_motion_changes_ideal_orientation_but_not_actual_inputs(self):
        contract = _contract()
        first_aligned = _aligned((0.0, 1000.0, 0.0), left_yaw=0.0, right_yaw=0.0)
        moved_gt = adapt_ground_truth_pose(
            _gt_message(10_350_000_000, (100.0, 1000.0, 50.0)),
            source_topic="/simulation/ground_truth/vehicle_reference_link",
            reference_name="vehicle_reference_link",
            clock_domain="gazebo_sim_time",
        )
        second_aligned = replace(
            first_aligned,
            evaluation_id="gt:10350000000",
            evaluation_stamp_ns=10_350_000_000,
            ground_truth=moved_gt,
        )
        first = evaluate_gt_aligned_inputs(first_aligned, contract, evaluation_id=10)
        second = evaluate_gt_aligned_inputs(second_aligned, contract, evaluation_id=11)
        self.assertTrue(first.valid)
        self.assertTrue(second.valid)
        assert first.ideal_ground_truth and second.ideal_ground_truth
        assert first.ideal_ground_truth.rig_pose and second.ideal_ground_truth.rig_pose
        self.assertNotEqual(
            first.ideal_ground_truth.rig_pose.rotation_world_from_frame,
            second.ideal_ground_truth.rig_pose.rotation_world_from_frame,
        )
        self.assertEqual(
            first_aligned.left_actual_camera.entity_name,
            second_aligned.left_actual_camera.entity_name,
        )
        self.assertEqual(
            first_aligned.left_actual_camera.generated_sdf_sha256,
            second_aligned.left_actual_camera.generated_sdf_sha256,
        )

    def test_invalid_alignment_never_produces_a_valid_measurement(self):
        aligned = align_static_snapshot(None, None, None)
        result = evaluate_gt_aligned_inputs(aligned, _contract(), evaluation_id=2)
        self.assertFalse(result.valid)
        self.assertEqual(result.status, "WAITING_FOR_GT")
        self.assertIsNone(result.left_error)
        self.assertIsNone(result.baseline)

    def test_zero_los_is_explicitly_invalid(self):
        # The cameras are centred at the origin for the fixture's actual
        # midpoint, so a target at the midpoint must fail M1 look-at.
        north_aligned = _aligned((0.0, 1000.0, 0.0), left_yaw=0.0, right_yaw=0.0)
        zero_gt = adapt_ground_truth_pose(
            _gt_message(10_250_000_000, (0.0, 0.0, 0.0)),
            source_topic="/simulation/ground_truth/vehicle_reference_link",
            reference_name="vehicle_reference_link",
            clock_domain="gazebo_sim_time",
        )
        aligned = replace(
            north_aligned,
            ground_truth=zero_gt,
            evaluation_stamp_ns=10_250_000_000,
        )
        result = evaluate_gt_aligned_inputs(aligned, _contract(), evaluation_id=3)
        self.assertFalse(result.valid)
        self.assertEqual(result.status, "ZERO_LOS")

    def test_engine_assigns_monotonic_ids_and_common_gt_stamps(self):
        engine = GtMeasurementEngine(_contract())
        results = [
            engine.evaluate(
                _aligned(
                    (0.0, 1000.0 + index, 0.0),
                    left_yaw=0.0,
                    right_yaw=0.0,
                    stamp_ns=10_250_000_000 + index,
                )
            )
            for index in range(100)
        ]
        self.assertEqual([result.evaluation_id for result in results], list(range(1, 101)))
        self.assertTrue(all(result.valid for result in results))
        self.assertEqual(
            [result.evaluation_stamp_ns for result in results],
            [10_250_000_000 + index for index in range(100)],
        )


if __name__ == "__main__":
    unittest.main()
