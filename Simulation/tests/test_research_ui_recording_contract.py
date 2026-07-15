from __future__ import annotations

import os
import json
import signal
import tempfile
import time
import unittest
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "stereo_ground_sim"
import sys

sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.recording_control import (  # noqa: E402
    MODE_OPTIONS,
    RecordingControlWidget,
    RecordingSessionController,
    build_operator_recorder_command,
    mode_value,
    sanitize_run_name,
    unique_output_dir,
)
from stereo_ground_sim import recording_control  # noqa: E402
from stereo_ground_sim.recorder_panel import RecorderPanel  # noqa: E402
from stereo_ground_sim.simulation_ui_contract import (  # noqa: E402
    RecordingReadiness,
)


def write_success_manifest(controller):
    output = controller.output_path
    (output / "manifest.json").write_text(
        json.dumps(
            {
                "recording_state": "complete",
                "artifact_integrity": True,
                "formal_experiment_eligible": True,
            }
        ),
        encoding="utf-8",
    )


class ResearchUiRecordingContractTests(unittest.TestCase):
    def test_mode_mapping_and_default_research(self):
        self.assertEqual(tuple(value for _label, value in MODE_OPTIONS), ("lite", "full", "debug"))
        self.assertEqual(mode_value("Data Only"), "lite")
        self.assertEqual(mode_value("Research"), "full")
        self.assertEqual(mode_value("Debug/Reprocess"), "debug")
        self.assertEqual(MODE_OPTIONS[1], ("Research", "full"))

    def test_run_name_sanitization_and_collision_suffix(self):
        self.assertEqual(sanitize_run_name("yaw relative/005deg"), "yaw_relative_005deg")
        self.assertEqual(sanitize_run_name(".."), "run")
        self.assertEqual(sanitize_run_name(""), "run")
        with tempfile.TemporaryDirectory() as directory:
            fixed = datetime(2026, 7, 14, 12, 34, 56)
            first = unique_output_dir(directory, "yaw relative/005deg", "full", now=fixed)
            Path(first).mkdir(parents=True)
            second = unique_output_dir(directory, "yaw relative/005deg", "full", now=fixed)
            self.assertNotEqual(first, second)
            self.assertTrue(second.name.endswith("_2"))

    def test_command_builder_preserves_recorder_entrypoint(self):
        command = build_operator_recorder_command(
            python_executable="/usr/bin/python3",
            mode="full",
            output_dir="/tmp/example",
        )
        self.assertEqual(
            command,
            [
                "/usr/bin/python3",
                "-m",
                "stereo_ground_sim.operator_recorder",
                "--mode",
                "full",
                "--output-dir",
                "/tmp/example",
            ],
        )

    def test_manifest_and_debug_topic_contract_are_additive(self):
        source = (PACKAGE_ROOT / "stereo_ground_sim" / "operator_recorder.py").read_text(encoding="utf-8")
        for field in (
            "analytic_forward_m",
            "left_roll_deg",
            "relative_yaw_deg",
            "disparity_difference_px",
            "intersection_angle_deg",
            "oracle_matched_entity_name",
            "px4_mission_phase_source",
            "px4_boot_timestamp_us",
            "ground_truth_receive_ros_time_sec",
        ):
            self.assertIn(field, source)
        for field in (
            '"recording_state": "in_progress"',
            '"synchronized_measurements": False',
            '"formal_cross_source_error_valid": False',
            '"ground_truth_policy": "gazebo_target_reference_link"',
            '"px4_role": "auxiliary_not_ground_truth"',
        ):
            self.assertIn(field, source)
        for topic in (
            "/left_camera/detection_image",
            "/right_camera/detection_image",
            "/stereo/attitude_model/status",
            "/stereo/observation_status",
            "/stereo/profile_status",
            "/simulation/px4/status",
        ):
            self.assertIn(topic, source)

    def test_rviz_default_and_cli_external_recording_contract(self):
        source = (ROOT / "run_operator_view.sh").read_text(encoding="utf-8")
        self.assertIn("launch_rviz=false", source)
        self.assertIn('launch_args+=("launch_rviz:=false")', source)
        self.assertIn('launch_args+=("launch_rviz:=$launch_rviz")', source)
        self.assertIn("OPERATOR_RECORDING_MANAGED_EXTERNALLY", source)

    def test_start_before_recording_has_no_output_directory(self):
        try:
            from python_qt_binding.QtWidgets import QApplication
        except Exception as exc:  # pragma: no cover - optional rqt host
            self.skipTest(f"Qt unavailable: {exc}")
        app = QApplication.instance() or QApplication([])

        class FakeProcess:
            pid = 987654

            def __init__(self):
                self.returncode = None
                self.signals = []

            def poll(self):
                return self.returncode

            def send_signal(self, sig):
                self.signals.append(sig)

            def wait(self, timeout=None):
                if self.returncode is None:
                    raise TimeoutError(timeout)
                return self.returncode

        processes = []

        def factory(*_args, **_kwargs):
            process = FakeProcess()
            processes.append(process)
            return process

        controller = RecordingSessionController(process_factory=factory, poll_interval_ms=1000)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "operator_runs"
            self.assertFalse(root.exists())
            self.assertTrue(controller.start(mode="full", run_name="research", save_root=root))
            self.assertEqual(len(processes), 1)
            self.assertTrue(controller.output_path.exists())
            controller.poll()
            self.assertEqual(controller.state, "RECORDING")
            self.assertTrue(controller.stop())
            write_success_manifest(controller)
            processes[0].returncode = 0
            controller.poll()
            self.assertEqual(controller.state, "COMPLETE")
        controller.shutdown()
        app.processEvents()

    def test_controller_persists_readiness_at_start_snapshot(self):
        try:
            from python_qt_binding.QtWidgets import QApplication
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"Qt unavailable: {exc}")
        app = QApplication.instance() or QApplication([])

        class FakeProcess:
            pid = 9876541

            def __init__(self):
                self.returncode = None

            def poll(self):
                return self.returncode

            def send_signal(self, _sig):
                self.returncode = 0

            def wait(self, timeout=None):
                if self.returncode is None:
                    raise TimeoutError(timeout)
                return self.returncode

        controller = RecordingSessionController(
            process_factory=lambda *_args, **_kwargs: FakeProcess(),
            poll_interval_ms=1000,
            stop_timeout_s=0.01,
            terminate_timeout_s=0.01,
        )
        controller.set_readiness(RecordingReadiness(True, True, ()))
        with tempfile.TemporaryDirectory() as directory:
            self.assertTrue(controller.start(mode="lite", run_name="ready", save_root=directory))
            payload = json.loads(
                (controller.output_path / "readiness_at_start.json").read_text(encoding="utf-8")
            )
            self.assertEqual(payload, {"start_allowed": True, "formal_eligible": True, "reasons": []})
        controller.shutdown()
        if controller._shutdown_thread is not None:
            controller._shutdown_thread.join(timeout=1.0)
        app.processEvents()

    def test_widget_blocks_unprovided_research_but_allows_debug(self):
        try:
            from python_qt_binding.QtWidgets import QApplication
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"Qt unavailable: {exc}")
        app = QApplication.instance() or QApplication([])
        widget = RecordingControlWidget()
        self.assertFalse(widget.start_button.isEnabled())
        widget.set_readiness(RecordingReadiness(True, True, ()))
        self.assertTrue(widget.start_button.isEnabled())
        widget.mode_combo.setCurrentIndex(2)
        self.assertTrue(widget.start_button.isEnabled())
        widget.mode_combo.setCurrentIndex(1)
        self.assertFalse(widget.start_button.isEnabled())
        widget.deleteLater()
        app.processEvents()

    def test_standalone_recorder_panel_uses_shared_readiness_model(self):
        try:
            from python_qt_binding.QtWidgets import QApplication
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"Qt unavailable: {exc}")
        app = QApplication.instance() or QApplication([])

        from python_qt_binding.QtCore import QObject

        class Context(QObject):
            def __init__(self):
                super().__init__()
                self.widget = None

            def add_widget(self, widget):
                self.widget = widget

        context = Context()
        # Exercise the standalone panel without requiring a live ROS graph;
        # its subscriber still owns the same pure model used by OperatorPanel.
        with patch.object(recording_control, "rclpy", None):
            panel = RecorderPanel(context)
        self.assertFalse(panel._recording_control.start_button.isEnabled())

        model = panel._readiness_subscriber.model
        now = 10_000_000_000
        safe_px4 = {
            "px4_connected": True,
            "flight_contract": {
                "verified": True,
                "source": "stereo_distance_node_parameters",
                "flight_authority": "px4_sitl",
                "fly_target_in_gazebo": False,
                "mission_profile": "manual",
                "actual_distance_source": "gazebo",
            },
        }
        safe_observation = {
            "observation_mode": "both",
            "oracle_source_role": "target_reference_link",
            "oracle_valid": True,
            "oracle_fallback_used": False,
            "bbox_left_valid": True,
            "bbox_right_valid": True,
        }
        fixture = {
            "ground_truth": [10.0, 1.0, 2.0, 10.3],
            "observed_raw": [10.1, 1.1, 2.1, 10.5],
            "analytic_raw": [10.2, 1.2, 2.2, 10.6],
            "corrected": [10.0, 1.0, 2.0, 10.3],
            "attitude_status": safe_observation,
            "observation_status": safe_observation,
            "profile_status": {"profile_mode": "no_profile"},
            "px4_status": safe_px4,
            "left_image": {"received": True},
            "right_image": {"received": True},
        }
        for key, value in fixture.items():
            model.update(key, value, receive_ros_time_ns=now, receive_monotonic_ns=now)
        panel._recording_control.set_readiness(model.readiness(now_monotonic_ns=now))
        self.assertTrue(panel._recording_control.start_button.isEnabled())
        self.assertIn("PARAMETER SNAPSHOT PENDING", panel._recording_control.readiness_label.text())
        self.assertNotIn("FORMAL READY", panel._recording_control.readiness_label.text())

        panel._recording_control.mode_combo.setCurrentIndex(2)
        self.assertTrue(panel._recording_control.start_button.isEnabled())
        self.assertIn("DEBUG", panel._recording_control.readiness_label.text())
        panel.shutdown_plugin()
        panel._widget.deleteLater()
        app.processEvents()

    def test_read_only_save_root_becomes_ui_error(self):
        try:
            from python_qt_binding.QtWidgets import QApplication
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"Qt unavailable: {exc}")
        app = QApplication.instance() or QApplication([])
        widget = RecordingControlWidget()
        widget.set_readiness(RecordingReadiness(True, True, ()))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "save-root-file"
            root.write_text("not a directory", encoding="utf-8")
            widget.save_root_edit.setText(str(root))
            self.assertFalse(widget._start())
            self.assertEqual(widget.state_label.text(), "ERROR")
            self.assertIn("ERROR", widget.state_label.text())
        widget.deleteLater()
        app.processEvents()

    def test_controller_rejects_duplicate_start_and_uses_nonblocking_stop(self):
        try:
            from python_qt_binding.QtWidgets import QApplication
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"Qt unavailable: {exc}")
        app = QApplication.instance() or QApplication([])

        class FakeProcess:
            pid = 987655

            def __init__(self):
                self.returncode = None
                self.signals = []

            def poll(self):
                return self.returncode

            def send_signal(self, sig):
                self.signals.append(sig)

            def wait(self, timeout=None):
                if self.returncode is None:
                    raise TimeoutError(timeout)
                return self.returncode

        process = FakeProcess()
        controller = RecordingSessionController(
            process_factory=lambda *_args, **_kwargs: process,
            poll_interval_ms=1000,
        )
        with tempfile.TemporaryDirectory() as directory:
            self.assertTrue(controller.start(mode="lite", run_name="duplicate", save_root=directory))
            self.assertFalse(controller.start(mode="lite", run_name="duplicate", save_root=directory))
            self.assertTrue(controller.stop())
            self.assertEqual(controller.state, "STOPPING")
            write_success_manifest(controller)
            process.returncode = 0
            controller.poll()
            self.assertEqual(controller.state, "COMPLETE")
        controller.shutdown()
        app.processEvents()

    def test_stop_escalates_sigint_sigterm_sigkill_without_wait(self):
        try:
            from python_qt_binding.QtWidgets import QApplication
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"Qt unavailable: {exc}")
        app = QApplication.instance() or QApplication([])

        class FakeProcess:
            pid = 987656

            def __init__(self):
                self.returncode = None

            def poll(self):
                return self.returncode

            def wait(self, timeout=None):
                if self.returncode is None:
                    raise TimeoutError(timeout)
                return self.returncode

        process = FakeProcess()
        controller = RecordingSessionController(
            process_factory=lambda *_args, **_kwargs: process,
            poll_interval_ms=1000,
            stop_timeout_s=1.0,
            terminate_timeout_s=1.0,
        )
        with tempfile.TemporaryDirectory() as directory, patch(
            "stereo_ground_sim.recording_control._send_group_signal"
        ) as send_signal:
            self.assertTrue(controller.start(mode="full", run_name="escalation", save_root=directory))
            controller.poll()
            self.assertTrue(controller.stop())
            controller._stop_deadline = time.monotonic() - 1.0
            controller.poll()
            controller._stop_deadline = time.monotonic() - 1.0
            controller.poll()
            self.assertEqual(
                [call.args[1] for call in send_signal.call_args_list],
                [signal.SIGINT, signal.SIGTERM, signal.SIGKILL],
            )
            write_success_manifest(controller)
            process.returncode = 0
            controller.poll()
            self.assertEqual(controller.state, "COMPLETE")
        controller.shutdown()
        app.processEvents()

    def test_video_output_alias_keeps_detection_topic_timestamps(self):
        from stereo_ground_sim import operator_recorder

        class FakeWriter:
            def __init__(self):
                self.frames = 0

            def isOpened(self):
                return True

            def write(self, _image):
                self.frames += 1

        class FakeCv2:
            @staticmethod
            def VideoWriter_fourcc(*_chars):
                return 0

            @staticmethod
            def VideoWriter(*_args):
                return FakeWriter()

        class FakeClock:
            @staticmethod
            def now():
                return SimpleNamespace(nanoseconds=12_345_000_000)

        class FakeRecorder:
            session_dir = None
            video_fps = 20.0
            latest = {}
            last_stamp = {}
            video_writers = {}
            _msg_to_bgr = operator_recorder.OperatorRecorder._msg_to_bgr

            @staticmethod
            def get_clock():
                return FakeClock()

            @staticmethod
            def get_logger():
                return SimpleNamespace(error=lambda *_args, **_kwargs: None)

        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=7, nanosec=500_000_000)),
            encoding="bgr8",
            width=1,
            height=1,
            step=3,
            data=bytes((1, 2, 3)),
        )
        previous_cv2 = operator_recorder.cv2
        operator_recorder.cv2 = FakeCv2
        try:
            with tempfile.TemporaryDirectory() as directory:
                recorder = FakeRecorder()
                recorder.session_dir = Path(directory)
                operator_recorder.OperatorRecorder._image_cb(recorder, "left", message, "left_overlay")
        finally:
            operator_recorder.cv2 = previous_cv2
        self.assertIn("/left_camera/detection_image", FakeRecorder.last_stamp)
        self.assertEqual(FakeRecorder.latest["left_image_header_time_sec"], 7.5)
        self.assertIn("left_overlay", FakeRecorder.video_writers)


if __name__ == "__main__":
    unittest.main()
