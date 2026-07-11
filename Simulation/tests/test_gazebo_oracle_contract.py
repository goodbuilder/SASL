from __future__ import annotations

import math
import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.coordinate_frames import FrameId, RigidTransform3D  # noqa: E402
from stereo_ground_sim.gazebo_oracle import (  # noqa: E402
    GazeboOracleResult,
    OracleMatchKind,
    OraclePoseSelection,
    OracleSourceCandidate,
    OracleSourcePolicy,
    OracleSourceRole,
    OracleStatus,
    ground_truth_from_gazebo_pose_payload,
    make_gazebo_world_to_baseline_transform,
    select_oracle_pose,
    x500_oracle_source_policy,
)


R_B_W_SDF = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)


def assert_tuple_close(testcase: unittest.TestCase, actual, expected, places: int = 12) -> None:
    testcase.assertEqual(len(actual), len(expected))
    for left, right in zip(actual, expected):
        testcase.assertAlmostEqual(left, right, places=places)


def pose_record(name: object, x: object = 1.0, y: object = 2.0, z: object = 3.0) -> dict[str, object]:
    return {"name": name, "position": {"x": x, "y": y, "z": z}}


def nested_payload(*records: dict[str, object]) -> dict[str, object]:
    return {"pose": [{"child": [{"pose": list(records)}]}]}


class GazeboOraclePolicyTests(unittest.TestCase):
    def test_normal_target_base_model_policy(self) -> None:
        policy = x500_oracle_source_policy()

        self.assertEqual(
            tuple(source.role for source in policy.sources),
            (
                OracleSourceRole.TARGET_REFERENCE_LINK,
                OracleSourceRole.BASE_LINK,
                OracleSourceRole.MODEL_POSE,
            ),
        )
        self.assertEqual(policy.sources[0].exact_names[0], "x500_15m::target_reference_link")
        self.assertEqual(policy.sources[1].exact_names[0], "x500_15m::base_link")
        self.assertEqual(policy.sources[2].exact_names[0], "x500_15m")

    def test_role_order_error_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "source order"):
            OracleSourcePolicy(
                sources=(
                    OracleSourceCandidate(OracleSourceRole.BASE_LINK, ("base_link",), ("::base_link",)),
                    OracleSourceCandidate(
                        OracleSourceRole.TARGET_REFERENCE_LINK,
                        ("target_reference_link",),
                        ("::target_reference_link",),
                    ),
                    OracleSourceCandidate(OracleSourceRole.MODEL_POSE, ("x500_15m",), ("::x500_15m",)),
                )
            )

    def test_duplicate_role_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "source order"):
            OracleSourcePolicy(
                sources=(
                    OracleSourceCandidate(
                        OracleSourceRole.TARGET_REFERENCE_LINK,
                        ("target_reference_link",),
                        ("::target_reference_link",),
                    ),
                    OracleSourceCandidate(OracleSourceRole.BASE_LINK, ("base_link",), ("::base_link",)),
                    OracleSourceCandidate(OracleSourceRole.BASE_LINK, ("other_base_link",), ("::other_base_link",)),
                )
            )

    def test_empty_exact_name_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "non-empty"):
            OracleSourceCandidate(OracleSourceRole.BASE_LINK, ("",), ("::base_link",))

    def test_invalid_suffix_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "scoped suffix"):
            OracleSourceCandidate(OracleSourceRole.BASE_LINK, ("base_link",), ("base_link",))

    def test_duplicate_exact_and_suffix_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicates"):
            OracleSourceCandidate(OracleSourceRole.BASE_LINK, ("base_link", "base_link"), ("::base_link",))
        with self.assertRaisesRegex(ValueError, "duplicates"):
            OracleSourceCandidate(OracleSourceRole.BASE_LINK, ("base_link",), ("::base_link", "::base_link"))


class GazeboOracleSelectionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = x500_oracle_source_policy()

    def test_target_reference_link_exact_selected(self) -> None:
        result = select_oracle_pose(
            nested_payload(pose_record("x500_15m::target_reference_link")),
            self.policy,
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.status, OracleStatus.OK_TARGET_REFERENCE_LINK)
        self.assertEqual(result.source_role, OracleSourceRole.TARGET_REFERENCE_LINK)
        self.assertEqual(result.matched_entity_name, "x500_15m::target_reference_link")
        self.assertEqual(result.match_kind, OracleMatchKind.EXACT)
        self.assertEqual(result.source_priority, 0)
        self.assertFalse(result.fallback_used)
        self.assertEqual(result.fallback_reason, "")

    def test_scoped_target_exact_precedes_unscoped_exact(self) -> None:
        result = select_oracle_pose(
            nested_payload(
                pose_record("target_reference_link", x=99.0),
                pose_record("x500_15m::target_reference_link", x=7.0),
            ),
            self.policy,
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.matched_entity_name, "x500_15m::target_reference_link")
        self.assertEqual(result.point_world_enu_m.xyz_m[0], 7.0)

    def test_target_absent_base_fallback(self) -> None:
        result = select_oracle_pose(nested_payload(pose_record("x500_15m::base_link")), self.policy)

        self.assertTrue(result.valid)
        self.assertEqual(result.status, OracleStatus.OK_BASE_LINK_FALLBACK)
        self.assertEqual(result.source_role, OracleSourceRole.BASE_LINK)
        self.assertTrue(result.fallback_used)
        self.assertEqual(result.fallback_reason, "target_reference_link_not_found")

    def test_target_and_base_absent_model_fallback(self) -> None:
        result = select_oracle_pose(nested_payload(pose_record("x500_15m")), self.policy)

        self.assertTrue(result.valid)
        self.assertEqual(result.status, OracleStatus.OK_MODEL_POSE_FALLBACK)
        self.assertEqual(result.source_role, OracleSourceRole.MODEL_POSE)
        self.assertTrue(result.fallback_used)
        self.assertEqual(result.fallback_reason, "target_reference_link_and_base_link_not_found")

    def test_all_sources_absent(self) -> None:
        result = select_oracle_pose(nested_payload(pose_record("other_model")), self.policy)

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.ORACLE_SOURCE_NOT_FOUND)
        self.assertIsNone(result.source_role)
        self.assertIsNone(result.matched_entity_name)
        self.assertIsNone(result.match_kind)
        self.assertIsNone(result.point_world_enu_m)

    def test_exact_duplicate_is_ambiguous(self) -> None:
        result = select_oracle_pose(
            nested_payload(
                pose_record("x500_15m::target_reference_link", x=1.0),
                pose_record("x500_15m::target_reference_link", x=2.0),
            ),
            self.policy,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.AMBIGUOUS_ENTITY_NAME)
        self.assertIsNone(result.source_role)

    def test_suffix_unique_selected(self) -> None:
        result = select_oracle_pose(
            nested_payload(pose_record("world::x500_15m::target_reference_link")),
            self.policy,
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.match_kind, OracleMatchKind.UNIQUE_SCOPED_SUFFIX)
        self.assertEqual(result.matched_entity_name, "world::x500_15m::target_reference_link")

    def test_suffix_duplicate_is_ambiguous(self) -> None:
        result = select_oracle_pose(
            nested_payload(
                pose_record("world_a::x500_15m::target_reference_link"),
                pose_record("world_b::x500_15m::target_reference_link"),
            ),
            self.policy,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.AMBIGUOUS_ENTITY_NAME)

    def test_malformed_top_level_payload(self) -> None:
        result = select_oracle_pose("not a payload", self.policy)

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.MALFORMED_POSE_PAYLOAD)

    def test_nested_dict_list_traversal(self) -> None:
        payload = {"a": [{"b": {"c": [pose_record("x500_15m::base_link", x=4.0)]}}]}

        result = select_oracle_pose(payload, self.policy)

        self.assertTrue(result.valid)
        self.assertEqual(result.matched_entity_name, "x500_15m::base_link")
        self.assertEqual(result.point_world_enu_m.xyz_m, (4.0, 2.0, 3.0))

    def test_empty_or_non_string_name_ignored(self) -> None:
        result = select_oracle_pose(
            nested_payload(
                pose_record(""),
                pose_record(None),
                pose_record("x500_15m", x=8.0),
            ),
            self.policy,
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.matched_entity_name, "x500_15m")
        self.assertEqual(result.point_world_enu_m.xyz_m[0], 8.0)

    def test_target_missing_position_blocks_base_fallback(self) -> None:
        result = select_oracle_pose(
            nested_payload(
                {"name": "x500_15m::target_reference_link"},
                pose_record("x500_15m::base_link"),
            ),
            self.policy,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.MISSING_POSITION)
        self.assertEqual(result.source_role, OracleSourceRole.TARGET_REFERENCE_LINK)
        self.assertFalse(result.fallback_used)

    def test_target_bad_position_format_blocks_base_fallback(self) -> None:
        result = select_oracle_pose(
            nested_payload(
                {"name": "x500_15m::target_reference_link", "position": [1.0, 2.0, 3.0]},
                pose_record("x500_15m::base_link"),
            ),
            self.policy,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.UNSUPPORTED_POSE_FORMAT)
        self.assertEqual(result.source_role, OracleSourceRole.TARGET_REFERENCE_LINK)

    def test_target_nan_position_blocks_base_fallback(self) -> None:
        result = select_oracle_pose(
            nested_payload(
                pose_record("x500_15m::target_reference_link", x=math.nan),
                pose_record("x500_15m::base_link"),
            ),
            self.policy,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.NON_FINITE_POSITION)
        self.assertEqual(result.source_role, OracleSourceRole.TARGET_REFERENCE_LINK)

    def test_target_ambiguous_blocks_base_fallback(self) -> None:
        result = select_oracle_pose(
            nested_payload(
                pose_record("x500_15m::target_reference_link", x=1.0),
                pose_record("x500_15m::target_reference_link", x=2.0),
                pose_record("x500_15m::base_link", x=3.0),
            ),
            self.policy,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.AMBIGUOUS_ENTITY_NAME)
        self.assertIsNone(result.source_role)

    def test_invalid_base_blocks_model_fallback(self) -> None:
        result = select_oracle_pose(
            nested_payload(
                {"name": "x500_15m::base_link"},
                pose_record("x500_15m", x=10.0),
            ),
            self.policy,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.MISSING_POSITION)
        self.assertEqual(result.source_role, OracleSourceRole.BASE_LINK)
        self.assertTrue(result.fallback_used)
        self.assertEqual(result.fallback_reason, "target_reference_link_not_found")


class GazeboOraclePositionParsingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = x500_oracle_source_policy()

    def test_position_field_supported(self) -> None:
        result = select_oracle_pose(
            {"name": "x500_15m::target_reference_link", "position": {"x": 1, "y": 2.5, "z": -3}},
            self.policy,
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.point_world_enu_m.frame, FrameId.GAZEBO_WORLD_ENU)
        assert_tuple_close(self, result.point_world_enu_m.xyz_m, (1.0, 2.5, -3.0))

    def test_pos_field_supported(self) -> None:
        result = select_oracle_pose(
            {"name": "x500_15m::target_reference_link", "pos": {"x": -1.0, "y": 2.0, "z": 3.0}},
            self.policy,
        )

        self.assertTrue(result.valid)
        assert_tuple_close(self, result.point_world_enu_m.xyz_m, (-1.0, 2.0, 3.0))

    def test_missing_axis(self) -> None:
        result = select_oracle_pose(
            {"name": "x500_15m::target_reference_link", "position": {"x": 1.0, "y": 2.0}},
            self.policy,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.MISSING_POSITION)

    def test_nonnumeric_axis(self) -> None:
        result = select_oracle_pose(pose_record("x500_15m::target_reference_link", x="east"), self.policy)

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.UNSUPPORTED_POSE_FORMAT)

    def test_nan_and_inf_rejected(self) -> None:
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                result = select_oracle_pose(pose_record("x500_15m::target_reference_link", z=value), self.policy)
                self.assertFalse(result.valid)
                self.assertEqual(result.status, OracleStatus.NON_FINITE_POSITION)

    def test_bad_orientation_does_not_affect_valid_position(self) -> None:
        result = select_oracle_pose(
            {
                "name": "x500_15m::target_reference_link",
                "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                "orientation": {"x": "bad", "y": math.nan},
            },
            self.policy,
        )

        self.assertTrue(result.valid)
        assert_tuple_close(self, result.point_world_enu_m.xyz_m, (1.0, 2.0, 3.0))


class GazeboOracleTransformTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = x500_oracle_source_policy()

    def result_for(self, point_world, transform) -> GazeboOracleResult:
        return ground_truth_from_gazebo_pose_payload(
            pose_record("x500_15m::target_reference_link", *point_world),
            self.policy,
            transform,
        )

    def test_identity_rotation_with_translated_origin(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(10.0, 20.0, 30.0),
            rotation_baseline_world=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        )

        result = self.result_for((13.0, 24.0, 35.0), transform)

        self.assertTrue(result.valid)
        assert_tuple_close(self, result.point_baseline_center_frd_m.xyz_m, (3.0, 4.0, 5.0))

    def test_current_sdf_known_case(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )

        result = self.result_for((0.0, 2000.0, 0.75), transform)

        self.assertTrue(result.valid)
        assert_tuple_close(self, result.point_baseline_center_frd_m.xyz_m, (2000.0, 0.0, 4.25))

    def test_yaw_90_degree_proper_rotation(self) -> None:
        r_b_w = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 0.0),
            rotation_baseline_world=r_b_w,
        )

        result = self.result_for((1.0, 0.0, 0.0), transform)

        assert_tuple_close(self, result.point_baseline_center_frd_m.xyz_m, (0.0, 1.0, 0.0))

    def test_non_zero_proper_three_axis_rotation_matrix(self) -> None:
        r_b_w = (
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0),
        )
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(1.0, 2.0, 3.0),
            rotation_baseline_world=r_b_w,
        )

        result = self.result_for((5.0, 7.0, 11.0), transform)

        assert_tuple_close(self, result.point_baseline_center_frd_m.xyz_m, (5.0, 8.0, 4.0))

    def test_signed_negative_front_for_target_behind(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )

        result = self.result_for((0.0, -10.0, 5.0), transform)

        self.assertEqual(result.point_baseline_center_frd_m.xyz_m[0], -10.0)

    def test_signed_negative_right_for_target_left(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )

        result = self.result_for((-3.0, 0.0, 5.0), transform)

        self.assertEqual(result.point_baseline_center_frd_m.xyz_m[1], -3.0)

    def test_signed_negative_down_for_target_above(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )

        result = self.result_for((0.0, 0.0, 8.0), transform)

        self.assertEqual(result.point_baseline_center_frd_m.xyz_m[2], -3.0)

    def test_abs_is_not_applied(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )

        result = self.result_for((-2.0, -4.0, 7.0), transform)

        assert_tuple_close(self, result.point_baseline_center_frd_m.xyz_m, (-4.0, -2.0, -2.0))

    def test_formula_round_trip(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(2.0, -3.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )
        world_point = (11.0, 17.0, -2.0)

        result = self.result_for(world_point, transform)
        r_w_b = tuple(zip(*R_B_W_SDF))
        recovered = []
        for row in r_w_b:
            recovered.append(sum(row[i] * result.point_baseline_center_frd_m.xyz_m[i] for i in range(3)))
        recovered = tuple(recovered[i] + transform.origin_target_in_source_m[i] for i in range(3))

        max_error = max(abs(a - b) for a, b in zip(recovered, world_point))
        self.assertLess(max_error, 1.0e-9)

    def test_invalid_rotation_matrix_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            make_gazebo_world_to_baseline_transform(
                origin_world_enu_m=(0.0, 0.0, 0.0),
                rotation_baseline_world=((1.0, 0.1, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            )

    def test_reflection_matrix_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "determinant"):
            make_gazebo_world_to_baseline_transform(
                origin_world_enu_m=(0.0, 0.0, 0.0),
                rotation_baseline_world=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)),
            )

    def test_non_finite_baseline_origin_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            make_gazebo_world_to_baseline_transform(
                origin_world_enu_m=(0.0, math.inf, 0.0),
                rotation_baseline_world=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            )

    def test_wrong_transform_source_frame_rejected(self) -> None:
        transform = RigidTransform3D(
            source_frame=FrameId.PX4_LOCAL_NED,
            target_frame=FrameId.BASELINE_CENTER_FRD,
            rotation_target_source=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            origin_target_in_source_m=(0.0, 0.0, 0.0),
        )

        with self.assertRaisesRegex(ValueError, "source frame"):
            self.result_for((1.0, 2.0, 3.0), transform)

    def test_wrong_transform_target_frame_rejected(self) -> None:
        transform = RigidTransform3D(
            source_frame=FrameId.GAZEBO_WORLD_ENU,
            target_frame=FrameId.PX4_LOCAL_NED,
            rotation_target_source=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            origin_target_in_source_m=(0.0, 0.0, 0.0),
        )

        with self.assertRaisesRegex(ValueError, "target frame"):
            self.result_for((1.0, 2.0, 3.0), transform)

    def test_world_and_output_point_frames(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )

        result = self.result_for((0.0, 2000.0, 0.75), transform)

        self.assertEqual(result.point_world_enu_m.frame, FrameId.GAZEBO_WORLD_ENU)
        self.assertEqual(result.point_baseline_center_frd_m.frame, FrameId.BASELINE_CENTER_FRD)

    def test_fallback_metadata_preserved_in_ground_truth_result(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )

        result = ground_truth_from_gazebo_pose_payload(
            nested_payload(pose_record("x500_15m::base_link")),
            self.policy,
            transform,
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.status, OracleStatus.OK_BASE_LINK_FALLBACK)
        self.assertTrue(result.fallback_used)
        self.assertEqual(result.fallback_reason, "target_reference_link_not_found")

    def test_dataclasses_are_immutable(self) -> None:
        selection = select_oracle_pose(
            nested_payload(pose_record("x500_15m::target_reference_link")),
            self.policy,
        )
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )
        result = ground_truth_from_gazebo_pose_payload(
            nested_payload(pose_record("x500_15m::target_reference_link")),
            self.policy,
            transform,
        )

        with self.assertRaises(FrozenInstanceError):
            selection.valid = False
        with self.assertRaises(FrozenInstanceError):
            result.valid = False

    def test_invalid_selection_maps_to_invalid_ground_truth_result(self) -> None:
        transform = make_gazebo_world_to_baseline_transform(
            origin_world_enu_m=(0.0, 0.0, 5.0),
            rotation_baseline_world=R_B_W_SDF,
        )

        result = ground_truth_from_gazebo_pose_payload(
            nested_payload(pose_record("other_model")),
            self.policy,
            transform,
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.status, OracleStatus.ORACLE_SOURCE_NOT_FOUND)
        self.assertIsNone(result.point_world_enu_m)
        self.assertIsNone(result.point_baseline_center_frd_m)


if __name__ == "__main__":
    unittest.main()
