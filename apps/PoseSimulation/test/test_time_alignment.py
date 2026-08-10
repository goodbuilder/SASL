from __future__ import annotations

import unittest
from dataclasses import replace

from pose_simulation.input_adapters import adapt_camera_pose_diagnostic, adapt_ground_truth_pose
from pose_simulation.input_types import (
    CAMERA_STATIC_SNAPSHOT_CHANGED,
    CLOCK_DOMAIN_UNRESOLVED,
    SOURCE_MODE_MISMATCH,
    TIME_ALIGNMENT_FAILED,
    VALID,
    WAITING_FOR_GT,
    WAITING_FOR_LEFT_CAMERA,
    WAITING_FOR_RIGHT_CAMERA,
)
from pose_simulation.time_alignment import align_dynamic_streams, align_static_snapshot, sample_pose

from .test_input_adapters import _camera_payload, _gt_message


def _gt(stamp_ns: int):
    return adapt_ground_truth_pose(
        _gt_message(stamp_ns=stamp_ns),
        source_topic="/simulation/ground_truth/vehicle_reference_link",
        clock_domain="gazebo_sim_time",
    )


def _camera(side: str, stamp_ns: int, *, mode: str = "static_snapshot", x: float = -12.5):
    payload = _camera_payload(side=side, stamp_ns=stamp_ns)
    payload["observed_optical_position_world_enu_m"] = [x, 0.0, 10.0]
    return adapt_camera_pose_diagnostic(
        payload,
        expected_side=side,
        source_topic=f"/simulation/camera_pose_diagnostics/{side}",
        source_mode=mode,
        expected_entity_name=f"{side}_stereo_camera::optical_link",
        clock_domain="gazebo_sim_time",
    )


class TimeAlignmentTests(unittest.TestCase):
    def test_static_snapshot_uses_gt_stamp_and_not_camera_stamp(self):
        aligned = align_static_snapshot(_gt(10_250_000_000), _camera("left", 1), _camera("right", 1, x=12.5))
        self.assertTrue(aligned.valid)
        self.assertEqual(aligned.status, VALID)
        self.assertEqual(aligned.evaluation_id, "gt:10250000000")
        self.assertEqual(aligned.evaluation_stamp_ns, 10_250_000_000)
        self.assertEqual(aligned.alignment_method, "static_snapshot")
        self.assertEqual(aligned.gt_time_residual_ns, 0)
        self.assertIsNone(aligned.left_time_residual_ns)
        self.assertIsNone(aligned.right_time_residual_ns)

    def test_flattened_samples_materialize_as_publishable_poses(self):
        aligned = align_static_snapshot(
            _gt(10_250_000_000),
            _camera("left", 1),
            _camera("right", 1, x=12.5),
        )
        self.assertTrue(aligned.valid)
        assert aligned.ground_truth is not None
        assert aligned.left_actual_camera is not None
        self.assertEqual(sample_pose(aligned.ground_truth).frame_id, "gazebo_world_enu")
        self.assertEqual(sample_pose(aligned.left_actual_camera).frame_id, "camera_optical_rdf")
        self.assertEqual(
            sample_pose(aligned.ground_truth).position_world_enu_m,
            aligned.ground_truth.position_world_enu_m,
        )

    def test_static_alignment_reports_missing_and_invalid_inputs_without_reuse(self):
        self.assertEqual(align_static_snapshot(None, None, None).status, WAITING_FOR_GT)
        gt = _gt(10)
        self.assertEqual(align_static_snapshot(gt, None, None).status, WAITING_FOR_LEFT_CAMERA)
        left = _camera("left", 1)
        self.assertEqual(align_static_snapshot(gt, left, None).status, WAITING_FOR_RIGHT_CAMERA)
        changed_left = _camera("left", 2)
        changed_left = replace(
            changed_left,
            valid=False,
            status=CAMERA_STATIC_SNAPSHOT_CHANGED,
            invalid_reason="changed",
        )
        invalid = align_static_snapshot(gt, changed_left, _camera("right", 1, x=12.5))
        self.assertFalse(invalid.valid)
        self.assertEqual(invalid.status, CAMERA_STATIC_SNAPSHOT_CHANGED)

    def test_static_alignment_propagates_unresolved_clock_and_source_mode(self):
        unresolved = align_static_snapshot(
            _gt(10),
            _camera("left", 1),
            _camera("right", 1, x=12.5),
            expected_clock_domain="other_clock",
        )
        self.assertEqual(unresolved.status, CLOCK_DOMAIN_UNRESOLVED)
        dynamic_left = _camera("left", 1, mode="dynamic_stream")
        mismatch = align_static_snapshot(_gt(10), dynamic_left, _camera("right", 1, x=12.5))
        self.assertEqual(mismatch.status, SOURCE_MODE_MISMATCH)

    def test_dynamic_stream_interpolates_camera_positions_without_future_extrapolation(self):
        aligned = align_dynamic_streams(
            [_gt(1_000_000_000)],
            [_camera("left", 0, mode="dynamic_stream", x=-10.0), _camera("left", 2_000_000_000, mode="dynamic_stream", x=-12.0)],
            [_camera("right", 0, mode="dynamic_stream", x=10.0), _camera("right", 2_000_000_000, mode="dynamic_stream", x=12.0)],
            max_interpolation_gap_ns=2_000_000_000,
        )
        self.assertTrue(aligned.valid)
        self.assertEqual(aligned.status, VALID)
        self.assertEqual(aligned.alignment_method, "interpolate")
        self.assertEqual(aligned.evaluation_stamp_ns, 1_000_000_000)
        self.assertEqual(aligned.left_actual_camera.position_world_enu_m[0], -11.0)
        self.assertEqual(aligned.right_actual_camera.position_world_enu_m[0], 11.0)

        future = align_dynamic_streams(
            [_gt(0)],
            [_camera("left", 1, mode="dynamic_stream")],
            [_camera("right", 1, mode="dynamic_stream", x=12.5)],
            max_interpolation_gap_ns=2_000_000_000,
        )
        self.assertFalse(future.valid)
        self.assertEqual(future.status, TIME_ALIGNMENT_FAILED)

    def test_dynamic_stream_rejects_static_source_and_future_policy(self):
        static = align_dynamic_streams(
            [_gt(1)],
            [_camera("left", 1)],
            [_camera("right", 1, x=12.5)],
        )
        self.assertEqual(static.status, SOURCE_MODE_MISMATCH)
        forbidden = align_dynamic_streams(
            [_gt(1)],
            [_camera("left", 1, mode="dynamic_stream")],
            [_camera("right", 1, mode="dynamic_stream", x=12.5)],
            allow_future_extrapolation=True,
        )
        self.assertEqual(forbidden.status, TIME_ALIGNMENT_FAILED)


if __name__ == "__main__":
    unittest.main()
