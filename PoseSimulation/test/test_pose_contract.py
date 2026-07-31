import copy
import math
import unittest
from pathlib import Path

import yaml

from pose_simulation.pose_math import (
    LEFT_CAMERA_FRAME,
    RIG_FRAME,
    RIGHT_CAMERA_FRAME,
    WORLD_FRAME,
    validate_pose_contract,
    validate_rotation_matrix,
)


CONTRACT_PATH = Path(__file__).parents[1] / "config" / "pose_contract.yaml"


def load_contract():
    with CONTRACT_PATH.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class PoseContractTests(unittest.TestCase):
    def test_checked_in_contract_is_valid_and_matches_geometry(self):
        result = validate_pose_contract(load_contract())
        self.assertEqual(result.schema, "pose_simulation_contract/v1")
        self.assertEqual(result.world_frame, WORLD_FRAME)
        self.assertEqual(result.rig_frame, RIG_FRAME)
        self.assertEqual(result.left_camera_frame, LEFT_CAMERA_FRAME)
        self.assertEqual(result.right_camera_frame, RIGHT_CAMERA_FRAME)
        self.assertEqual(result.position_unit, "m")
        self.assertEqual(result.angle_internal_unit, "rad")
        self.assertEqual(result.angle_output_unit, "deg")
        self.assertAlmostEqual(result.baseline_m, 25.0, delta=1.0e-12)
        for rotation in (
            result.rotation_rig_from_left_optical,
            result.rotation_rig_from_right_optical,
        ):
            metrics = validate_rotation_matrix(rotation)
            self.assertLess(metrics["orthogonality_error"], 1.0e-12)
            self.assertLess(metrics["determinant_error"], 1.0e-12)

    def test_baseline_translation_mismatch_is_rejected(self):
        contract = load_contract()
        contract["nominal_geometry"]["right_center_rig_frd_m"] = [0.0, 13.0, 0.0]
        with self.assertRaisesRegex(ValueError, "baseline_m"):
            validate_pose_contract(contract)

    def test_reflection_is_rejected(self):
        contract = load_contract()
        contract["nominal_geometry"]["rotation_rig_from_left_optical"] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0],
        ]
        with self.assertRaisesRegex(ValueError, "determinant"):
            validate_pose_contract(contract)

    def test_schema_frame_unit_and_convention_guards(self):
        cases = (
            ("schema", "wrong", "schema"),
            ("frames", {"world": "px4_local_ned"}, "frames.world"),
            ("units", {"position": "cm"}, "units.position"),
            (
                "optical_error_convention",
                {"yaw_axis": "positive_up"},
                "optical_error_convention.yaw_axis",
            ),
        )
        for section, change, expected in cases:
            with self.subTest(section=section):
                contract = load_contract()
                if section in {"schema"}:
                    contract[section] = change
                else:
                    contract[section].update(change)
                with self.assertRaisesRegex(ValueError, expected):
                    validate_pose_contract(contract)

    def test_missing_and_nonfinite_values_are_rejected(self):
        missing = load_contract()
        del missing["nominal_geometry"]["rotation_rig_from_right_optical"]
        with self.assertRaises(ValueError):
            validate_pose_contract(missing)

        nonfinite = load_contract()
        nonfinite["world_up_world_enu"][2] = math.nan
        with self.assertRaisesRegex(ValueError, "world_up_world_enu"):
            validate_pose_contract(nonfinite)

    def test_contract_validation_does_not_mutate_input(self):
        contract = load_contract()
        original = copy.deepcopy(contract)
        validate_pose_contract(contract)
        self.assertEqual(contract, original)


if __name__ == "__main__":
    unittest.main()
