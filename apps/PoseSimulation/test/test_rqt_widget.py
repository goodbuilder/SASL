import os
import unittest

try:
    from python_qt_binding.QtWidgets import QApplication
    from pose_simulation_rqt.pose_simulation_widget import CameraPanel, ImageFrame, PoseSimulationWidget
    from pose_simulation_rqt.ui_model import build_ui_snapshot
except Exception:  # pragma: no cover - optional GUI test dependency
    QApplication = None
    PoseSimulationWidget = None
    CameraPanel = None
    ImageFrame = None
    build_ui_snapshot = None


def _sample(valid=True):
    pose = {
        "evaluation_id": 7,
        "evaluation_timestamp_ns": 124_963_000_000,
        "valid": valid,
        "world_frame": "gazebo_world_enu",
        "error_frame": "camera_optical_rdf",
        "left_roll_error_deg": 0.01,
        "left_pitch_error_deg": -0.02,
        "left_yaw_error_deg": 0.10,
        "left_rotation_error_deg": 0.103,
        "left_optical_axis_error_deg": 0.382,
        "right_roll_error_deg": 0.012,
        "right_pitch_error_deg": -0.03,
        "right_yaw_error_deg": 0.14,
        "right_rotation_error_deg": 0.144,
        "right_optical_axis_error_deg": 0.385,
        "gt_stamp_ns": 124_963_000_000,
        "left_camera_source_stamp_ns": 124_963_000_000,
        "right_camera_source_stamp_ns": 124_963_000_000,
    }
    stereo = {
        "evaluation_id": 7,
        "evaluation_timestamp_ns": 124_963_000_000,
        "valid": valid,
        "world_frame": "gazebo_world_enu",
        "rig_frame": "baseline_center_frd",
        "relative_roll_error_deg": 0.0,
        "relative_pitch_error_deg": 0.0,
        "relative_yaw_error_deg": 0.04,
        "relative_rotation_error_deg": 0.041,
        "optical_axis_angle_deg": 0.041,
        "actual_baseline_length_m": 25.0,
        "nominal_baseline_length_m": 25.0,
        "baseline_length_error_m": 0.0,
        "baseline_direction_error_deg": 0.021,
        "actual_baseline_vector_world_enu_m": {"x": 25.0, "y": 0.0, "z": 0.0},
        "actual_midpoint_world_enu_m": {"x": 0.0, "y": 0.0, "z": 0.0},
    }
    status = {
        "evaluation_id": 7,
        "evaluation_timestamp_ns": 124_963_000_000,
        "valid": valid,
        "state": "VALID" if valid else "CAMERA_SCHEMA_MISMATCH",
        "reason": "" if valid else "fixture invalid",
        "gt_topic": "/simulation/ground_truth/vehicle_reference_link",
        "gt_reference_name": "vehicle_reference_link",
        "left_camera_topic": "/simulation/camera_pose_diagnostics/left",
        "right_camera_topic": "/simulation/camera_pose_diagnostics/right",
        "camera_source_mode": "static_snapshot",
    }
    return build_ui_snapshot(pose, stereo, status)


@unittest.skipIf(QApplication is None, "Qt is not installed")
class RqtWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        cls.app = QApplication.instance() or QApplication([])

    def test_final_tab_structure_and_header(self):
        widget = PoseSimulationWidget(start_bridge=False)
        try:
            self.assertEqual(
                [widget._tabs.tabText(i) for i in range(widget._tabs.count())],
                ["Overview", "Camera", "Pose Error", "Stereo Geometry", "Reference && Time", "Trends", "Diagnostics"],
            )
            self.assertEqual(widget._status_labels["overall"].text(), "Waiting for sources")
            self.assertEqual(widget._status_labels["readonly"].text(), "Read-only")
            self.assertNotIn("evaluation:", widget._status_labels["evaluation"].text())
            self.assertNotIn("timestamp:", widget._status_labels["sim_time"].text())
        finally:
            widget.shutdown()
            widget.deleteLater()
            self.app.processEvents()

    def test_valid_metrics_are_displayed_and_invalid_metrics_are_dashes(self):
        widget = PoseSimulationWidget(start_bridge=False)
        try:
            widget.update_snapshot(_sample(valid=True))
            self.assertEqual(widget._metric_labels["ov_rel_yaw"].text(), "+0.040°")
            self.assertEqual(widget._metric_labels["ov_l_los"].text(), "+0.382°")
            self.assertEqual(widget._metric_labels["ov_base_dir"].text(), "+0.021°")
            self.assertEqual(widget._status_labels["overall"].text(), "Measuring")
            widget.update_snapshot(_sample(valid=False))
            self.assertEqual(widget._metric_labels["ov_rel_yaw"].text(), "—")
            self.assertEqual(widget._metric_labels["ov_l_los"].text(), "—")
            self.assertNotIn("0.000", widget._metric_labels["ov_rel_yaw"].text())
        finally:
            widget.shutdown()
            widget.deleteLater()
            self.app.processEvents()

    def test_plugin_reopen_does_not_share_widget_state(self):
        first = PoseSimulationWidget(start_bridge=False)
        first.update_snapshot(_sample(valid=True))
        first.shutdown()
        first.deleteLater()
        second = PoseSimulationWidget(start_bridge=False)
        try:
            self.assertEqual(second._status_labels["overall"].text(), "Waiting for sources")
            self.assertEqual(len(second._history), 0)
        finally:
            second.shutdown()
            second.deleteLater()
            self.app.processEvents()

    def test_pause_keeps_latest_and_displayed_samples_distinct(self):
        widget = PoseSimulationWidget(start_bridge=False)
        try:
            widget.update_snapshot(_sample(valid=True))
            widget._pause_button.setChecked(True)
            invalid = _sample(valid=False)
            widget.update_snapshot(invalid)
            self.assertTrue(widget._latest_snapshot is invalid)
            self.assertEqual(widget._snapshot.evaluation_id, 7)
            self.assertEqual(widget._last_valid_snapshot.evaluation_id, 7)
            self.assertEqual(widget._metric_labels["ov_rel_yaw"].text(), "+0.040°")
            widget._pause_button.setChecked(False)
            self.assertFalse(widget._snapshot.valid)
            self.assertEqual(widget._metric_labels["ov_rel_yaw"].text(), "—")
        finally:
            widget.shutdown()
            widget.deleteLater()
            self.app.processEvents()

    def test_camera_panel_keeps_source_metadata_and_empty_state(self):
        panel = CameraPanel("Left camera", "/left_camera/detection_image")
        self.assertIn("Camera stream unavailable", panel._image_label.text())
        frame = ImageFrame("left", bytes([255, 0, 0] * 4), 2, 2, "rgb8", 1_000_000_000, 1)
        panel.set_frame(frame)
        self.assertEqual(panel._image.size().width(), 2)
        self.assertIn("2×2", panel._status.text())
        panel.resize(640, 360)
        panel.show()
        self.app.processEvents()
        panel.close()


if __name__ == "__main__":
    unittest.main()
