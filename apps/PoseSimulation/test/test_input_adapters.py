from __future__ import annotations

import copy
import json
import math
import unittest
from types import SimpleNamespace

from pose_simulation.input_adapters import (
    GroundTruthStreamValidator,
    StaticCameraSnapshotStore,
    adapt_camera_pose_diagnostic,
    adapt_ground_truth_pose,
)
from pose_simulation.input_types import (
    CAMERA_ENTITY_CHANGED,
    CAMERA_FIELD_MISSING,
    CAMERA_MALFORMED_JSON,
    CAMERA_NONFINITE,
    CAMERA_QUATERNION_INVALID,
    CAMERA_SCHEMA_MISMATCH,
    CAMERA_SDF_HASH_CHANGED,
    CAMERA_SIDE_MISMATCH,
    CAMERA_STATIC_SNAPSHOT_CHANGED,
    CAMERA_TIMESTAMP_INVALID,
    CLOCK_DOMAIN_UNRESOLVED,
    GT_DUPLICATE_STAMP,
    GT_FRAME_MISMATCH,
    GT_NONFINITE,
    GT_OUT_OF_ORDER,
    GT_TIMESTAMP_INVALID,
    VALID,
)


SOURCE_TOPIC = "/simulation/camera_pose_diagnostics/left"


def _gt_message(stamp_ns: int = 10_250_000_000, *, frame_id: str = "gazebo_world_enu", position=(0.0, 1000.0, 100.0), quaternion=(0.0, 0.0, 0.0, 2.0)):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000),
            frame_id=frame_id,
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
            orientation=SimpleNamespace(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
            ),
        ),
    )


def _camera_payload(*, side: str = "left", stamp_ns: int = 1_000_000_000) -> dict[str, object]:
    return {
        "schema": "camera_pose_comparison/v1",
        "camera_label": side,
        "gazebo_stamp_ns": stamp_ns,
        "observed_optical_position_world_enu_m": [-12.5, 0.0, 10.0],
        "observed_optical_orientation_world_from_entity_xyzw": [
            0.0,
            0.0,
            0.7071067811865475,
            0.7071067811865476,
        ],
        "observed_optical_entity_name": f"{side}_stereo_camera::optical_link",
        "observed_optical_entity_id": 100,
        "generated_sdf_sha256": "fixture",
    }


def _adapt_camera(payload: dict[str, object], *, side: str = "left", mode: str = "static_snapshot"):
    return adapt_camera_pose_diagnostic(
        json.dumps(payload),
        expected_side=side,
        source_topic=SOURCE_TOPIC,
        source_mode=mode,
        expected_entity_name=f"{side}_stereo_camera::optical_link",
        clock_domain="gazebo_sim_time",
    )


class InputAdapterTests(unittest.TestCase):
    def test_ground_truth_pose_stamp_and_quaternion_are_normalized_exactly(self):
        sample = adapt_ground_truth_pose(
            _gt_message(),
            source_topic="/simulation/ground_truth/vehicle_reference_link",
            receive_timestamp_ns=999,
            clock_domain="gazebo_sim_time",
        )
        self.assertTrue(sample.valid)
        self.assertEqual(sample.status, VALID)
        self.assertEqual(sample.stamp_ns, 10_250_000_000)
        self.assertEqual(sample.position_world_enu_m, (0.0, 1000.0, 100.0))
        self.assertEqual(sample.orientation_world_from_reference_xyzw, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(sample.receive_timestamp_ns, 999)

    def test_ground_truth_rejects_frame_nonfinite_timestamp_and_unresolved_clock(self):
        cases = (
            (_gt_message(frame_id="px4_local_ned"), GT_FRAME_MISMATCH),
            (_gt_message(position=(math.nan, 0.0, 0.0)), GT_NONFINITE),
            (_gt_message(stamp_ns=-1), GT_TIMESTAMP_INVALID),
        )
        for message, status in cases:
            with self.subTest(status=status):
                sample = adapt_ground_truth_pose(
                    message,
                    source_topic="/simulation/ground_truth/vehicle_reference_link",
                    clock_domain="gazebo_sim_time",
                )
                self.assertFalse(sample.valid)
                self.assertEqual(sample.status, status)
        fractional_stamp = _gt_message()
        fractional_stamp.header.stamp.sec = 10.5
        self.assertEqual(
            adapt_ground_truth_pose(
                fractional_stamp,
                source_topic="/simulation/ground_truth/vehicle_reference_link",
                clock_domain="gazebo_sim_time",
            ).status,
            GT_TIMESTAMP_INVALID,
        )
        unresolved = adapt_ground_truth_pose(
            _gt_message(),
            source_topic="/simulation/ground_truth/vehicle_reference_link",
            clock_domain=None,
        )
        self.assertEqual(unresolved.status, CLOCK_DOMAIN_UNRESOLVED)

    def test_camera_fixture_consumes_observed_optical_fields_and_preserves_identity(self):
        sample = _adapt_camera(_camera_payload())
        self.assertTrue(sample.valid)
        self.assertEqual(sample.camera_side, "left")
        self.assertEqual(sample.gazebo_stamp_ns, 1_000_000_000)
        self.assertEqual(sample.position_world_enu_m, (-12.5, 0.0, 10.0))
        self.assertEqual(sample.source_schema, "camera_pose_comparison/v1")
        self.assertEqual(sample.entity_name, "left_stereo_camera::optical_link")
        self.assertEqual(sample.entity_id, 100)
        self.assertEqual(sample.generated_sdf_sha256, "fixture")
        self.assertAlmostEqual(
            math.sqrt(sum(value * value for value in sample.orientation_world_from_optical_xyzw)),
            1.0,
            places=15,
        )

    def test_camera_unknown_fields_are_ignored_but_desired_pose_is_not_used(self):
        payload = _camera_payload()
        payload.update(
            {
                "unexpected": {"nested": True},
                "desired_optical_position_world_enu_m": [999.0, 999.0, 999.0],
            }
        )
        sample = _adapt_camera(payload)
        self.assertTrue(sample.valid)
        self.assertEqual(sample.position_world_enu_m, (-12.5, 0.0, 10.0))

    def test_wrong_quaternion_order_fixture_is_not_silently_treated_as_same_pose(self):
        expected_payload = _camera_payload()
        expected_payload["observed_optical_orientation_world_from_entity_xyzw"] = [
            0.1,
            0.2,
            0.3,
            0.9,
        ]
        wrong_order_payload = copy.deepcopy(expected_payload)
        wrong_order_payload["observed_optical_orientation_world_from_entity_xyzw"] = [
            0.9,
            0.1,
            0.2,
            0.3,
        ]
        expected = _adapt_camera(expected_payload)
        wrong_order = _adapt_camera(wrong_order_payload)
        self.assertTrue(expected.valid)
        self.assertTrue(wrong_order.valid)
        self.assertNotEqual(
            expected.rotation_world_from_optical,
            wrong_order.rotation_world_from_optical,
        )

    def test_camera_schema_side_json_and_required_field_failures_are_explicit(self):
        cases = []
        missing_schema = _camera_payload()
        del missing_schema["schema"]
        cases.append((missing_schema, CAMERA_SCHEMA_MISMATCH))
        cases.append((_camera_payload(side="right"), CAMERA_SIDE_MISMATCH))
        missing_position = _camera_payload()
        del missing_position["observed_optical_position_world_enu_m"]
        cases.append((missing_position, CAMERA_FIELD_MISSING))
        wrong_quaternion = _camera_payload()
        wrong_quaternion["observed_optical_orientation_world_from_entity_xyzw"] = [0.0, 0.0, 1.0]
        cases.append((wrong_quaternion, CAMERA_QUATERNION_INVALID))
        zero_quaternion = _camera_payload()
        zero_quaternion["observed_optical_orientation_world_from_entity_xyzw"] = [0.0, 0.0, 0.0, 0.0]
        cases.append((zero_quaternion, CAMERA_QUATERNION_INVALID))
        nonfinite_position = _camera_payload()
        nonfinite_position["observed_optical_position_world_enu_m"] = [float("inf"), 0.0, 0.0]
        cases.append((nonfinite_position, CAMERA_NONFINITE))
        for payload, status in cases:
            with self.subTest(status=status):
                sample = _adapt_camera(payload)
                self.assertFalse(sample.valid)
                self.assertEqual(sample.status, status)
        malformed = adapt_camera_pose_diagnostic(
            "{not-json",
            expected_side="left",
            source_topic=SOURCE_TOPIC,
            clock_domain="gazebo_sim_time",
        )
        self.assertEqual(malformed.status, CAMERA_MALFORMED_JSON)

    def test_camera_timestamp_identity_hash_and_clock_failures(self):
        negative_stamp = _camera_payload(stamp_ns=-1)
        self.assertEqual(_adapt_camera(negative_stamp).status, CAMERA_TIMESTAMP_INVALID)
        changed_entity = _camera_payload()
        changed_entity["observed_optical_entity_id"] = 101
        sample = adapt_camera_pose_diagnostic(
            changed_entity,
            expected_side="left",
            source_topic=SOURCE_TOPIC,
            expected_entity_name="left_stereo_camera::optical_link",
            expected_entity_id=100,
            clock_domain="gazebo_sim_time",
        )
        self.assertEqual(sample.status, CAMERA_ENTITY_CHANGED)
        changed_hash = _camera_payload()
        changed_hash["generated_sdf_sha256"] = "other"
        sample = adapt_camera_pose_diagnostic(
            changed_hash,
            expected_side="left",
            source_topic=SOURCE_TOPIC,
            expected_entity_name="left_stereo_camera::optical_link",
            expected_sdf_sha256="fixture",
            clock_domain="gazebo_sim_time",
        )
        self.assertEqual(sample.status, CAMERA_SDF_HASH_CHANGED)
        unresolved = _adapt_camera(_camera_payload(), side="left")
        self.assertTrue(unresolved.valid)
        unresolved = adapt_camera_pose_diagnostic(
            _camera_payload(),
            expected_side="left",
            source_topic=SOURCE_TOPIC,
            expected_entity_name="left_stereo_camera::optical_link",
            clock_domain=None,
        )
        self.assertEqual(unresolved.status, CLOCK_DOMAIN_UNRESOLVED)

    def test_static_snapshot_is_immutable_and_detects_identity_or_pose_changes(self):
        store = StaticCameraSnapshotStore("left")
        first = _adapt_camera(_camera_payload())
        self.assertIs(store.accept(first), first)
        sign_flipped = _camera_payload()
        sign_flipped["observed_optical_orientation_world_from_entity_xyzw"] = [
            0.0,
            0.0,
            -0.7071067811865475,
            -0.7071067811865476,
        ]
        self.assertIs(store.accept(_adapt_camera(sign_flipped)), first)

        changed_pose = _camera_payload()
        changed_pose["observed_optical_position_world_enu_m"] = [-12.4, 0.0, 10.0]
        changed = store.accept(_adapt_camera(changed_pose))
        self.assertFalse(changed.valid)
        self.assertEqual(changed.status, CAMERA_STATIC_SNAPSHOT_CHANGED)
        after_close = store.accept(first)
        self.assertFalse(after_close.valid)
        self.assertEqual(after_close.status, CAMERA_STATIC_SNAPSHOT_CHANGED)

        entity_store = StaticCameraSnapshotStore("left")
        entity_store.accept(first)
        changed_id = _camera_payload()
        changed_id["observed_optical_entity_id"] = 101
        changed_entity_sample = entity_store.accept(_adapt_camera(changed_id))
        self.assertEqual(changed_entity_sample.status, CAMERA_ENTITY_CHANGED)

        hash_store = StaticCameraSnapshotStore("left")
        hash_store.accept(first)
        changed_sdf = _camera_payload()
        changed_sdf["generated_sdf_sha256"] = "other"
        changed_hash_sample = hash_store.accept(_adapt_camera(changed_sdf))
        self.assertEqual(changed_hash_sample.status, CAMERA_SDF_HASH_CHANGED)

    def test_ground_truth_stream_rejects_out_of_order_and_duplicate_stamps(self):
        validator = GroundTruthStreamValidator()
        first = adapt_ground_truth_pose(
            _gt_message(stamp_ns=10),
            source_topic="/simulation/ground_truth/vehicle_reference_link",
            clock_domain="gazebo_sim_time",
        )
        self.assertTrue(validator.accept(first).valid)
        duplicate = validator.accept(
            adapt_ground_truth_pose(
                _gt_message(stamp_ns=10),
                source_topic="/simulation/ground_truth/vehicle_reference_link",
                clock_domain="gazebo_sim_time",
            )
        )
        self.assertEqual(duplicate.status, GT_DUPLICATE_STAMP)
        backwards = validator.accept(
            adapt_ground_truth_pose(
                _gt_message(stamp_ns=9),
                source_topic="/simulation/ground_truth/vehicle_reference_link",
                clock_domain="gazebo_sim_time",
            )
        )
        self.assertEqual(backwards.status, GT_OUT_OF_ORDER)


if __name__ == "__main__":
    unittest.main()
