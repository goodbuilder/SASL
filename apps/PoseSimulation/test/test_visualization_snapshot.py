import math
import unittest
from types import SimpleNamespace

from pose_simulation.visualization_snapshot import (
    WORLD_FRAME,
    build_visualization_snapshot,
    world_pose_from_pose_stamped,
)


def _stamp(value_ns):
    return SimpleNamespace(sec=value_ns // 1_000_000_000, nanosec=value_ns % 1_000_000_000)


def _pose(stamp_ns, position=(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 0.0, 1.0), frame=WORLD_FRAME):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=_stamp(stamp_ns)),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
            orientation=SimpleNamespace(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]),
        ),
    )


def _metric_message(stamp_ns, evaluation_id=7, valid=True):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=WORLD_FRAME, stamp=_stamp(stamp_ns)),
        evaluation_id=evaluation_id,
        valid=valid,
        status="VALID" if valid else "INVALID",
        left_roll_error_deg=0.1,
        left_pitch_error_deg=0.2,
        left_yaw_error_deg=0.3,
        left_rotation_error_deg=0.4,
        left_optical_axis_error_deg=0.5,
        left_gimbal_lock=False,
        right_roll_error_deg=0.6,
        right_pitch_error_deg=0.7,
        right_yaw_error_deg=0.8,
        right_rotation_error_deg=0.9,
        right_optical_axis_error_deg=1.0,
        right_gimbal_lock=False,
        relative_roll_error_deg=0.01,
        relative_pitch_error_deg=0.02,
        relative_yaw_error_deg=0.03,
        relative_rotation_error_deg=0.04,
        optical_axis_angle_deg=0.05,
        actual_baseline_vector_world_enu_m=SimpleNamespace(x=0.0, y=25.0, z=0.0),
        actual_baseline_length_m=25.0,
        nominal_baseline_length_m=25.0,
        baseline_length_error_m=0.0,
        baseline_direction_error_deg=0.0,
        actual_midpoint_world_enu_m=SimpleNamespace(x=0.0, y=0.0, z=0.0),
    )


def _status(stamp_ns, evaluation_id=7, valid=True):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=WORLD_FRAME, stamp=_stamp(stamp_ns)),
        evaluation_id=evaluation_id,
        valid=valid,
        state="VALID" if valid else "WAITING_FOR_GT",
        reason="" if valid else "Waiting for Ground Truth",
        gt_topic="/simulation/ground_truth/vehicle_reference_link",
        gt_reference_name="vehicle_reference_link",
        left_camera_topic="/simulation/camera_pose_diagnostics/left",
        right_camera_topic="/simulation/camera_pose_diagnostics/right",
        left_entity_name="left_stereo_camera::optical_link",
        left_entity_id=11,
        right_entity_name="right_stereo_camera::optical_link",
        right_entity_id=12,
        left_sdf_sha256="left-hash",
        right_sdf_sha256="right-hash",
        pose_contract_sha256="contract-hash",
        camera_source_mode="static_snapshot",
        rtk_comparison_implemented=False,
    )


def _valid_snapshot(stamp_ns=1_000_000_000, evaluation_id=7):
    pose_error = _metric_message(stamp_ns, evaluation_id)
    stereo = _metric_message(stamp_ns, evaluation_id)
    status = _status(stamp_ns, evaluation_id)
    return build_visualization_snapshot(
        pose_error=pose_error,
        stereo_geometry=stereo,
        status=status,
        gt_target=_pose(stamp_ns, (0.0, 1000.0, 0.0)),
        actual_left=_pose(stamp_ns, (-12.5, 0.0, 0.0)),
        actual_right=_pose(stamp_ns, (12.5, 0.0, 0.0)),
        ideal_gt_rig=_pose(stamp_ns, (0.0, 0.0, 0.0)),
        ideal_gt_left=_pose(stamp_ns, (-12.5, 0.0, 0.0)),
        ideal_gt_right=_pose(stamp_ns, (12.5, 0.0, 0.0)),
    )


class VisualizationSnapshotTests(unittest.TestCase):
    def test_valid_typed_messages_are_paired_by_id_and_stamp(self):
        snapshot = _valid_snapshot()
        self.assertTrue(snapshot.valid)
        self.assertEqual(snapshot.evaluation_id, 7)
        self.assertEqual(snapshot.evaluation_stamp_ns, 1_000_000_000)
        self.assertEqual(snapshot.actual_left.position_world_enu_m, (-12.5, 0.0, 0.0))
        self.assertEqual(snapshot.stereo.actual_baseline_length_m, 25.0)
        self.assertEqual(snapshot.source_identity["camera_source_mode"], "static_snapshot")

    def test_ros_message_vector_objects_are_supported(self):
        pose = world_pose_from_pose_stamped(
            _pose(10, quaternion=(0.0, 0.0, 0.0, 2.0)),
            local_frame="left_camera_optical_rdf",
            source_topic="actual_pose/left",
            expected_stamp_ns=10,
        )
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in pose.orientation_xyzw)), 1.0)

    def test_evaluation_mismatch_is_invalid(self):
        snapshot = _valid_snapshot()
        stereo = _metric_message(1_000_000_001, 7)
        invalid = build_visualization_snapshot(
            pose_error=_metric_message(1_000_000_000, 7),
            stereo_geometry=stereo,
            status=_status(1_000_000_000, 7),
        )
        self.assertFalse(invalid.valid)
        self.assertEqual(invalid.state, "EVALUATION_MISMATCH")
        self.assertIsNone(snapshot.rtk_target)

    def test_invalid_status_does_not_reuse_previous_geometry(self):
        invalid = build_visualization_snapshot(
            pose_error=None,
            stereo_geometry=None,
            status=_status(2_000_000_000, 8, valid=False),
        )
        self.assertFalse(invalid.valid)
        self.assertEqual(invalid.state, "WAITING_FOR_GT")
        self.assertIsNone(invalid.actual_left)

    def test_frame_mismatch_and_nonfinite_quaternion_fail_closed(self):
        self.assertIsNone(
            world_pose_from_pose_stamped(
                _pose(10, frame="px4_local_ned"),
                local_frame="left_camera_optical_rdf",
                source_topic="actual_pose/left",
            )
        )
        bad = _pose(10, quaternion=(0.0, 0.0, float("nan"), 1.0))
        self.assertIsNone(
            world_pose_from_pose_stamped(
                bad,
                local_frame="left_camera_optical_rdf",
                source_topic="actual_pose/left",
            )
        )


if __name__ == "__main__":
    unittest.main()
