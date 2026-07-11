from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.gazebo_oracle import (  # noqa: E402
    OracleSourceRole,
    OracleStatus,
    ground_truth_from_gazebo_pose_payload,
    make_gazebo_world_to_baseline_transform,
    select_oracle_pose,
    x500_oracle_source_policy,
)


R_B_W = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)


def runtime_record(name: str, *, entity_id: int, position=None):
    record = {"name": name, "id": entity_id, "orientation": {"w": 1.0}}
    if position is not None:
        record["position"] = position
    return record


def runtime_payload(*records):
    return {"pose": list(records)}


def model_record(*, y=2000.0, z=0.75, entity_id=101):
    return runtime_record("x500_15m", entity_id=entity_id, position={"y": y, "z": z})


def assert_tuple_close(testcase: unittest.TestCase, actual, expected, delta=1.0e-9):
    testcase.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        testcase.assertAlmostEqual(float(actual_value), float(expected_value), delta=delta)


class GazeboRuntimePoseVOracleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = x500_oracle_source_policy()
        self.transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W,
        )

    def oracle(self, payload):
        return ground_truth_from_gazebo_pose_payload(payload, self.policy, self.transform)

    def test_runtime_zero_offset_target_uses_model_world_pose(self) -> None:
        result = self.oracle(
            runtime_payload(
                model_record(),
                runtime_record(
                    "target_reference_link",
                    entity_id=113,
                    position={"y": 1.1368683772161603e-13},
                ),
            )
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.status, OracleStatus.OK_TARGET_REFERENCE_LINK)
        self.assertEqual(result.source_role, OracleSourceRole.TARGET_REFERENCE_LINK)
        self.assertEqual(result.matched_entity_name, "target_reference_link")
        self.assertFalse(result.fallback_used)
        assert_tuple_close(self, result.point_world_enu_m.xyz_m, (0.0, 2000.0, 0.75))
        assert_tuple_close(self, result.point_baseline_center_frd_m.xyz_m, (2000.0, 0.0, 4.25))

    def test_runtime_empty_position_is_zero_local_translation(self) -> None:
        result = self.oracle(
            runtime_payload(
                model_record(),
                runtime_record("target_reference_link", entity_id=113, position={}),
            )
        )

        self.assertTrue(result.valid)
        assert_tuple_close(self, result.point_world_enu_m.xyz_m, (0.0, 2000.0, 0.75))

    def test_runtime_base_link_fallback_uses_model_world_pose(self) -> None:
        result = self.oracle(
            runtime_payload(
                model_record(),
                runtime_record("base_link", entity_id=102, position={}),
            )
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.status, OracleStatus.OK_BASE_LINK_FALLBACK)
        self.assertEqual(result.source_role, OracleSourceRole.BASE_LINK)
        self.assertTrue(result.fallback_used)
        assert_tuple_close(self, result.point_world_enu_m.xyz_m, (0.0, 2000.0, 0.75))

    def test_runtime_model_fallback_allows_omitted_zero_axis(self) -> None:
        result = self.oracle(runtime_payload(model_record(y=123.0, z=4.0)))

        self.assertTrue(result.valid)
        self.assertEqual(result.status, OracleStatus.OK_MODEL_POSE_FALLBACK)
        self.assertEqual(result.source_role, OracleSourceRole.MODEL_POSE)
        assert_tuple_close(self, result.point_world_enu_m.xyz_m, (0.0, 123.0, 4.0))

    def test_nonzero_local_link_offset_is_invalid_without_model_fallback(self) -> None:
        result = self.oracle(
            runtime_payload(
                model_record(),
                runtime_record("target_reference_link", entity_id=113, position={"x": 2.0e-9}),
            )
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.UNSUPPORTED_NONZERO_LINK_OFFSET)
        self.assertEqual(result.source_role, OracleSourceRole.TARGET_REFERENCE_LINK)
        self.assertEqual(result.matched_entity_name, "target_reference_link")
        self.assertFalse(result.fallback_used)
        self.assertIsNone(result.point_world_enu_m)

    def test_malformed_runtime_local_position_is_invalid(self) -> None:
        cases = (
            (runtime_record("target_reference_link", entity_id=113), OracleStatus.MISSING_POSITION),
            (runtime_record("target_reference_link", entity_id=113, position=[0, 0, 0]), OracleStatus.UNSUPPORTED_POSE_FORMAT),
            (runtime_record("target_reference_link", entity_id=113, position={"x": True}), OracleStatus.UNSUPPORTED_POSE_FORMAT),
            (runtime_record("target_reference_link", entity_id=113, position={"x": math.nan}), OracleStatus.NON_FINITE_POSITION),
            (runtime_record("target_reference_link", entity_id=113, position={"x": math.inf}), OracleStatus.NON_FINITE_POSITION),
        )
        for target_record, expected_status in cases:
            with self.subTest(expected_status=expected_status):
                result = self.oracle(runtime_payload(model_record(), target_record))
                self.assertFalse(result.valid)
                self.assertEqual(result.status, expected_status)
                self.assertEqual(result.source_role, OracleSourceRole.TARGET_REFERENCE_LINK)

    def test_ambiguous_runtime_model_record_is_invalid(self) -> None:
        result = self.oracle(
            runtime_payload(
                model_record(y=1.0, entity_id=101),
                model_record(y=2.0, entity_id=201),
                runtime_record("target_reference_link", entity_id=113, position={}),
            )
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.AMBIGUOUS_ENTITY_NAME)
        self.assertEqual(result.source_role, OracleSourceRole.TARGET_REFERENCE_LINK)
        self.assertEqual(result.matched_entity_name, "target_reference_link")

    def test_legacy_flat_target_payload_stays_world_pose(self) -> None:
        result = self.oracle(
            {
                "pose": [
                    {
                        "name": "x500_15m::target_reference_link",
                        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                    }
                ]
            }
        )

        self.assertTrue(result.valid)
        assert_tuple_close(self, result.point_world_enu_m.xyz_m, (1.0, 2.0, 3.0))

    def test_legacy_flat_missing_axis_stays_missing_position(self) -> None:
        selection = select_oracle_pose(
            {
                "pose": [
                    {
                        "name": "x500_15m::target_reference_link",
                        "position": {"x": 1.0, "y": 2.0},
                    }
                ]
            },
            self.policy,
        )

        self.assertFalse(selection.valid)
        self.assertEqual(selection.status, OracleStatus.MISSING_POSITION)

    def test_runtime_unscoped_names_are_selected(self) -> None:
        result = self.oracle(
            runtime_payload(
                model_record(),
                runtime_record("base_link", entity_id=102, position={}),
                runtime_record("target_reference_link", entity_id=113, position={}),
            )
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.source_role, OracleSourceRole.TARGET_REFERENCE_LINK)
        self.assertEqual(result.matched_entity_name, "target_reference_link")
        self.assertEqual(result.status, OracleStatus.OK_TARGET_REFERENCE_LINK)


if __name__ == "__main__":
    unittest.main()
