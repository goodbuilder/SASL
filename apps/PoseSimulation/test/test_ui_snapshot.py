import unittest

from pose_simulation.diagnostic_adapter import build_diagnostic_records
from pose_simulation.marker_factory import build_marker_specs
from pose_simulation.ui_snapshot import PoseRecord, build_ui_snapshot


WORLD = "gazebo_world_enu"


def _pose(position, stamp=10, valid=True, frame="gazebo_world_enu"):
    return {
        "frame_id": frame,
        "position_world_enu_m": list(position),
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "source_timestamp_ns": stamp,
        "valid": valid,
    }


def _actual(position, source, stamp=10):
    return PoseRecord(WORLD, tuple(position), (0.0, 0.0, 0.0, 1.0), stamp, source)


def payloads(evaluation=100, *, rtk_valid=True, stereo_evaluation=None, frame=WORLD):
    pose_error = {
        "schema": "pose_simulation/pose_error/v1",
        "evaluation_timestamp_ns": evaluation,
        "frame_id": frame,
        "ideal_pose": {
            "ground_truth": {
                "source_type": "ground_truth",
                "target_world_enu_m": [0.0, 100.0, 0.0],
                "rig": _pose([0.0, 0.0, 0.0], frame="baseline_center_frd"),
                "left_optical": _pose([0.0, -12.5, 0.0], frame="camera_optical_rdf"),
                "right_optical": _pose([0.0, 12.5, 0.0], frame="camera_optical_rdf"),
                "valid": True,
            },
            "rtk": {
                "source_type": "rtk",
                "target_world_enu_m": [1.0, 100.0, 0.0] if rtk_valid else None,
                "rig": _pose([0.0, 0.0, 0.0], stamp=None, valid=rtk_valid, frame="baseline_center_frd"),
                "left_optical": _pose([0.0, -12.5, 0.0], stamp=None, valid=rtk_valid, frame="camera_optical_rdf"),
                "right_optical": _pose([0.0, 12.5, 0.0], stamp=None, valid=rtk_valid, frame="camera_optical_rdf"),
                "valid": rtk_valid,
            },
        },
        "left_actual_vs_gt_ideal": {
            "roll_error_deg": 0.1,
            "pitch_error_deg": 0.2,
            "yaw_error_deg": 0.3,
            "rotation_error_deg": 0.4,
            "optical_axis_to_gt_target_error_deg": 0.5,
            "valid": True,
        },
        "right_actual_vs_gt_ideal": {
            "roll_error_deg": 0.6,
            "pitch_error_deg": 0.7,
            "yaw_error_deg": 0.8,
            "rotation_error_deg": 0.9,
            "optical_axis_to_gt_target_error_deg": 1.0,
            "valid": True,
        },
        "left_actual_vs_rtk_ideal": {
            "rotation_error_deg": 1.1,
            "optical_axis_to_rtk_target_error_deg": 1.2,
            "valid": rtk_valid,
        },
        "right_actual_vs_rtk_ideal": {
            "rotation_error_deg": 1.3,
            "optical_axis_to_rtk_target_error_deg": 1.4,
            "valid": rtk_valid,
        },
        "rtk_ideal_vs_gt_ideal": {
            "roll_error_deg": 0.0,
            "pitch_error_deg": 0.0,
            "yaw_error_deg": 0.57,
            "rotation_error_deg": 0.57,
            "los_direction_error_deg": 0.57,
            "valid": rtk_valid,
        },
    }
    stereo = {
        "schema": "pose_simulation/stereo_geometry/v1",
        "evaluation_timestamp_ns": evaluation if stereo_evaluation is None else stereo_evaluation,
        "frame_id": frame,
        "relative_roll_error_deg": 0.01,
        "relative_pitch_error_deg": 0.02,
        "relative_yaw_error_deg": 0.03,
        "relative_rotation_error_deg": 0.04,
        "left_right_optical_axis_angle_deg": 0.05,
        "baseline": {
            "vector_world_enu_m": [0.0, 25.0, 0.0],
            "actual_length_m": 25.0,
            "nominal_length_m": 25.0,
            "length_error_m": 0.0,
            "direction_error_deg": 0.0,
            "midpoint_world_enu_m": [0.0, 0.0, 0.0],
            "midpoint_error_m": None,
            "valid": True,
        },
    }
    status = {
        "schema": "pose_simulation/status/v1",
        "evaluation_timestamp_ns": evaluation,
        "ground_truth_source": "TARGET_REFERENCE_LINK",
        "target_entity_name": "target_reference_link",
        "left_camera_pose_timestamp_ns": 10,
        "right_camera_pose_timestamp_ns": 10,
        "fallback_used": False,
        "rig_center_source": "actual_optical_midpoint",
        "rtk_source_topic": "/fmu/out/vehicle_gps_position",
        "rtk_status": "VALID_RTK_FIXED" if rtk_valid else "INVALID_FIX",
        "rtk_reference_valid_for_operational_evaluation": rtk_valid,
        "altitude_datum": "ellipsoid",
        "target_point_alignment_mode": "zero_lever_arm_assumption",
        "lever_arm_applied": False,
        "coordinate_frames": {
            "world": WORLD,
            "px4_local": "px4_local_ned",
            "rig": "baseline_center_frd",
            "camera_optical": "camera_optical_rdf",
        },
        "time_residuals": {
            "alignment_status": "VALID",
            "gt_time_residual_ms": 0.0,
            "left_pose_time_residual_ms": 0.0,
            "right_pose_time_residual_ms": 0.0,
            "rtk_time_residual_ms": None,
        },
        "validity": {
            "required_pose_inputs": True,
            "ground_truth_ideal": True,
            "rtk_ideal": rtk_valid,
            "actual_vs_gt": True,
            "actual_vs_rtk": rtk_valid,
            "stereo_relative": True,
            "baseline": True,
        },
    }
    return pose_error, stereo, status


class UiSnapshotTests(unittest.TestCase):
    def _snapshot(self, *, rtk_valid=True, stereo_evaluation=None, frame=WORLD, actual=True):
        error, stereo, status = payloads(
            rtk_valid=rtk_valid,
            stereo_evaluation=stereo_evaluation,
            frame=frame,
        )
        actual_poses = {
            "actual_left": _actual((0.0, -12.5, 0.0), "actual_left"),
            "actual_right": _actual((0.0, 12.5, 0.0), "actual_right"),
        } if actual else {}
        return build_ui_snapshot(error, stereo, status, actual_poses=actual_poses)

    def test_normal_sample_is_aligned_and_immutable(self):
        snapshot = self._snapshot()
        self.assertEqual(snapshot.pair_state, "ALIGNED")
        self.assertEqual(snapshot.state.marker_alignment, "ALIGNED")
        self.assertEqual(snapshot.state.ground_truth, "VALID")
        self.assertEqual(snapshot.state.rtk, "VALID")
        self.assertIn("ideal_rtk_left", snapshot.poses)
        with self.assertRaises(TypeError):
            snapshot.status["new"] = True

    def test_invalid_rtk_does_not_invalidate_gt(self):
        snapshot = self._snapshot(rtk_valid=False)
        self.assertEqual(snapshot.state.ground_truth, "VALID")
        self.assertEqual(snapshot.state.rtk, "INVALID")
        self.assertIsNone(snapshot.target_rtk_world_enu_m)
        self.assertTrue(snapshot.validity["actual_vs_gt"])

    def test_pair_id_mismatch_is_not_combined(self):
        snapshot = self._snapshot(stereo_evaluation=101)
        self.assertEqual(snapshot.pair_state, "PAIR ID MISMATCH")
        self.assertEqual(snapshot.state.marker_alignment, "PAIR ID MISMATCH")
        specs = build_marker_specs(snapshot)
        self.assertEqual(specs[0].action, "DELETEALL")

    def test_frame_mismatch_is_invalid(self):
        snapshot = self._snapshot(frame="px4_local_ned")
        self.assertEqual(snapshot.state.frame_contract, "INVALID")
        self.assertNotEqual(snapshot.pair_state, "ALIGNED")

    def test_missing_actual_pose_is_unaligned_for_rviz(self):
        snapshot = self._snapshot(actual=False)
        self.assertEqual(snapshot.state.marker_alignment, "UNALIGNED SAMPLE")
        self.assertEqual(snapshot.pair_state, "UNALIGNED SAMPLE")

    def test_diagnostics_include_write_guard(self):
        records = build_diagnostic_records(self._snapshot(rtk_valid=False))
        write_guard = next(record for record in records if record.name.endswith("WriteGuard"))
        self.assertEqual(write_guard.level, 0)
        self.assertIn("POSE_COMMAND_DISABLED", write_guard.message)

    def test_invalid_rtk_marker_is_deleted_not_retained(self):
        snapshot = self._snapshot(rtk_valid=False)
        specs = build_marker_specs(snapshot)
        namespaces = {spec.namespace for spec in specs}
        self.assertNotIn("pose_sim/target/rtk", namespaces)
        self.assertNotIn("pose_sim/ideal_rtk/left", namespaces)
        self.assertEqual(specs[0].action, "DELETEALL")

    def test_normal_marker_set_is_world_framed_and_has_optical_forward(self):
        specs = build_marker_specs(self._snapshot())
        self.assertTrue(specs)
        self.assertTrue(all(spec.frame_id == WORLD for spec in specs))
        left_forward = next(
            spec for spec in specs
            if spec.namespace == "pose_sim/actual/left" and spec.marker_id == 1
        )
        self.assertEqual(left_forward.points[0], (0.0, -12.5, 0.0))
        self.assertEqual(left_forward.points[1][2], 12.0)
        left_frustum = next(
            spec for spec in specs
            if spec.namespace == "pose_sim/actual/left" and spec.marker_id == 4
        )
        self.assertEqual(left_frustum.marker_type, "line_list")
        rig_forward = next(
            spec for spec in specs
            if spec.namespace == "pose_sim/ideal_gt/rig" and spec.marker_id == 36
        )
        self.assertEqual(rig_forward.points[1][0], 12.0)
        self.assertIn("pose_sim/baseline/actual", {spec.namespace for spec in specs})
        self.assertIn("pose_sim/error/left", {spec.namespace for spec in specs})
        self.assertIn("pose_sim/error/right", {spec.namespace for spec in specs})
        self.assertIn("pose_sim/error/relative", {spec.namespace for spec in specs})
        self.assertIn("pose_sim/rays/actual", {spec.namespace for spec in specs})
        self.assertIn("pose_sim/ideal_gt/left", {spec.namespace for spec in specs})
        self.assertIn("pose_sim/ideal_gt/rig", {spec.namespace for spec in specs})


if __name__ == "__main__":
    unittest.main()
