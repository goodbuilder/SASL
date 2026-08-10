import math
import unittest

from pose_simulation_rqt.ui_model import (
    build_ui_snapshot,
    format_metric,
    source_state_label,
    status_reason,
)


WORLD = "gazebo_world_enu"


def _typed_fixture(*, valid=True, evaluation_id=1542, stamp_ns=124_963_000_000, status_state="VALID"):
    pose_error = {
        "evaluation_id": evaluation_id,
        "evaluation_timestamp_ns": stamp_ns,
        "valid": valid,
        "status": status_state,
        "world_frame": WORLD,
        "error_frame": "camera_optical_rdf",
        "euler_convention": "optical_rdf_roll_pitch_yaw",
        "gt_stamp_ns": stamp_ns,
        "left_camera_source_stamp_ns": stamp_ns,
        "right_camera_source_stamp_ns": stamp_ns,
        "left_roll_error_deg": 0.01,
        "left_pitch_error_deg": -0.02,
        "left_yaw_error_deg": 0.10,
        "left_rotation_error_deg": 0.103,
        "left_optical_axis_error_deg": 0.382,
        "left_gimbal_lock": False,
        "right_roll_error_deg": 0.012,
        "right_pitch_error_deg": -0.03,
        "right_yaw_error_deg": 0.14,
        "right_rotation_error_deg": 0.144,
        "right_optical_axis_error_deg": 0.385,
        "right_gimbal_lock": False,
    }
    stereo = {
        "evaluation_id": evaluation_id,
        "evaluation_timestamp_ns": stamp_ns,
        "valid": valid,
        "status": status_state,
        "world_frame": WORLD,
        "rig_frame": "baseline_center_frd",
        "relative_roll_error_deg": 0.0,
        "relative_pitch_error_deg": 0.0,
        "relative_yaw_error_deg": 0.04,
        "relative_rotation_error_deg": 0.041,
        "optical_axis_angle_deg": 0.041,
        "actual_baseline_vector_world_enu_m": {"x": 25.0, "y": 0.0, "z": 0.0},
        "actual_baseline_length_m": 25.0,
        "nominal_baseline_length_m": 25.0,
        "baseline_length_error_m": 0.0,
        "baseline_direction_error_deg": 0.021,
        "actual_midpoint_world_enu_m": {"x": 0.0, "y": 0.0, "z": 0.0},
    }
    status = {
        "evaluation_id": evaluation_id,
        "evaluation_timestamp_ns": stamp_ns,
        "valid": valid,
        "state": status_state,
        "reason": "",
        "gt_topic": "/simulation/ground_truth/vehicle_reference_link",
        "gt_reference_name": "vehicle_reference_link",
        "left_camera_topic": "/simulation/camera_pose_diagnostics/left",
        "right_camera_topic": "/simulation/camera_pose_diagnostics/right",
        "camera_source_mode": "static_snapshot",
        "read_only": True,
        "pose_command_enabled": False,
        "rtk_comparison_implemented": False,
    }
    return pose_error, stereo, status


class RqtUiModelTests(unittest.TestCase):
    def test_normal_typed_sample_is_paired_and_preserves_metrics(self):
        snapshot = build_ui_snapshot(*_typed_fixture())
        self.assertTrue(snapshot.valid)
        self.assertTrue(snapshot.aligned)
        self.assertEqual(snapshot.evaluation_id, 1542)
        self.assertAlmostEqual(snapshot.sim_time_s, 124.963)
        self.assertAlmostEqual(snapshot.pose_error.yaw_deg, 0.10)
        self.assertAlmostEqual(snapshot.right_error.los_deg, 0.385)
        self.assertAlmostEqual(snapshot.stereo.relative_yaw_deg, 0.04)
        self.assertEqual(snapshot.camera_source_mode, "static_snapshot")
        self.assertEqual(snapshot.clock_domain, "Gazebo simulation time")

    def test_invalid_nan_metrics_are_unavailable_not_zero(self):
        pose, stereo, status = _typed_fixture(valid=False, status_state="CAMERA_SCHEMA_MISMATCH")
        pose["left_yaw_error_deg"] = math.nan
        stereo["relative_yaw_error_deg"] = math.nan
        snapshot = build_ui_snapshot(pose, stereo, status)
        self.assertFalse(snapshot.valid)
        self.assertIsNone(snapshot.pose_error.yaw_deg)
        self.assertIsNone(snapshot.stereo.relative_yaw_deg)
        self.assertEqual(format_metric(snapshot.stereo.relative_yaw_deg, unit="°"), "—")
        self.assertIn("Camera message format", snapshot.reason)

    def test_evaluation_id_mismatch_is_alignment_pending(self):
        pose, stereo, status = _typed_fixture()
        stereo["evaluation_id"] += 1
        snapshot = build_ui_snapshot(pose, stereo, status)
        self.assertFalse(snapshot.valid)
        self.assertFalse(snapshot.aligned)
        self.assertEqual(snapshot.overall_label, "Alignment pending")
        self.assertEqual(snapshot.reason, "Result messages do not share one evaluation sample")

    def test_waiting_status_is_natural_language(self):
        snapshot = build_ui_snapshot(
            None,
            None,
            {"evaluation_id": 0, "valid": False, "state": "WAITING_FOR_LEFT_CAMERA", "reason": "left snapshot missing"},
        )
        self.assertEqual(snapshot.overall_label, "Waiting for sources")
        self.assertEqual(snapshot.reason, "Waiting for left camera pose: left snapshot missing")
        self.assertEqual(snapshot.gt_state, "WAITING")

    def test_frame_mismatch_is_invalid(self):
        pose, stereo, status = _typed_fixture()
        pose["world_frame"] = "px4_local_ned"
        snapshot = build_ui_snapshot(pose, stereo, status)
        self.assertFalse(snapshot.valid)
        self.assertEqual(snapshot.frame_state, "INVALID")
        self.assertEqual(snapshot.overall_label, "Input invalid")

    def test_status_reason_mapping_and_badge_text_are_centralized(self):
        self.assertEqual(status_reason("TIME_ALIGNMENT_FAILED"), "Source timestamps cannot be aligned")
        self.assertEqual(status_reason("FRAME_MISMATCH"), "Incoming frame does not match gazebo_world_enu")
        self.assertEqual(source_state_label("UNALIGNED"), "Unaligned")
        self.assertEqual(format_metric(0.0, unit=" m"), "0.000 m")


if __name__ == "__main__":
    unittest.main()
