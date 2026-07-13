from __future__ import annotations

import json
import sys
import signal
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stereo_ground_sim"))

from stereo_ground_sim.operator_recorder import evaluate_artifact_integrity  # noqa: E402
from stereo_ground_sim import operator_recorder  # noqa: E402
from stereo_ground_sim.recording_control import RecordingSessionController  # noqa: E402
from stereo_ground_sim.simulation_ui_contract import evaluate_recording_outcome  # noqa: E402


def write_lite_artifacts(root: Path, rows=1):
    (root / "telemetry.csv").write_text("row\n" * rows, encoding="utf-8")
    (root / "manifest.txt").write_text("manifest\n", encoding="utf-8")
    (root / "manifest.json").write_text("{}\n", encoding="utf-8")


class RecordingIntegrityContractTests(unittest.TestCase):
    def test_lite_requires_csv_and_at_least_one_row(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ok, errors = evaluate_artifact_integrity(root, "lite", csv_row_count=0)
            self.assertFalse(ok)
            self.assertIn("telemetry.csv has zero rows", errors)
            write_lite_artifacts(root)
            ok, errors = evaluate_artifact_integrity(root, "lite", csv_row_count=1)
            self.assertTrue(ok, errors)

    def test_full_requires_nonempty_video_and_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_lite_artifacts(root)
            (root / "left_overlay.mp4").write_bytes(b"left")
            (root / "right_overlay.mp4").write_bytes(b"right")
            ok, errors = evaluate_artifact_integrity(
                root,
                "full",
                csv_row_count=1,
                left_video_frame_count=1,
                right_video_frame_count=1,
            )
            self.assertTrue(ok, errors)
            (root / "right_overlay.mp4").write_bytes(b"")
            ok, errors = evaluate_artifact_integrity(
                root,
                "full",
                csv_row_count=1,
                left_video_frame_count=1,
                right_video_frame_count=1,
            )
            self.assertFalse(ok)
            self.assertIn("empty artifact: right_overlay.mp4", errors)

    def test_debug_requires_metadata_and_zero_rosbag_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_lite_artifacts(root)
            for name in ("left_overlay.mp4", "right_overlay.mp4"):
                (root / name).write_bytes(b"video")
            (root / "rosbag").mkdir()
            (root / "rosbag" / "metadata.yaml").write_text("bagfile_information: {}\n", encoding="utf-8")
            ok, errors = evaluate_artifact_integrity(
                root,
                "debug",
                csv_row_count=1,
                left_video_frame_count=1,
                right_video_frame_count=1,
                bag_return_code=0,
                bag_reaped=True,
            )
            self.assertTrue(ok, errors)
            ok, errors = evaluate_artifact_integrity(
                root,
                "debug",
                csv_row_count=1,
                left_video_frame_count=1,
                right_video_frame_count=1,
                bag_return_code=1,
                bag_reaped=True,
            )
            self.assertFalse(ok)
            self.assertTrue(any("return code" in error for error in errors))

    def test_artifact_and_formal_outcomes_are_independent(self):
        self.assertEqual(
            evaluate_recording_outcome(True, True),
            {
                "recording_state": "complete",
                "artifact_integrity": True,
                "formal_experiment_eligible": True,
            },
        )
        self.assertFalse(evaluate_recording_outcome(True, False)["formal_experiment_eligible"])
        self.assertEqual(evaluate_recording_outcome(False, True)["recording_state"], "error")
        self.assertFalse(evaluate_recording_outcome(True, True, [{"type": "violation"}])["formal_experiment_eligible"])

    def test_mp4_writer_failure_is_artifact_error(self):
        class ClosedWriter:
            def isOpened(self):
                return False

        class FakeCv2:
            @staticmethod
            def VideoWriter_fourcc(*_args):
                return 0

            @staticmethod
            def VideoWriter(*_args):
                return ClosedWriter()

        recorder = object.__new__(operator_recorder.OperatorRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.counts = {field: 0 for field in operator_recorder.COUNTER_FIELDS}
        recorder._lock = threading.RLock()
        recorder.video_writers = {}
        recorder.video_fps = 20.0
        recorder._video_writer_errors = {}
        recorder.session_dir = Path(tempfile.mkdtemp())
        recorder.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_700_000_000_000_000_000)
        )
        recorder.get_logger = lambda: SimpleNamespace(error=lambda *_a, **_k: None)
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1_700_000_000, nanosec=1)),
            encoding="bgr8",
            width=1,
            height=1,
            step=3,
            data=bytes((1, 2, 3)),
        )
        previous_cv2 = operator_recorder.cv2
        operator_recorder.cv2 = FakeCv2
        try:
            recorder._image_cb("left", message, "left_overlay")
        finally:
            operator_recorder.cv2 = previous_cv2

        self.assertEqual(recorder.counts["left_image_received_count"], 1)
        self.assertEqual(recorder.counts["left_video_frame_count"], 0)
        self.assertIn("left_overlay", recorder._video_writer_errors)

    def test_shutdown_escalates_reaps_and_releases_root_lock(self):
        try:
            from python_qt_binding.QtWidgets import QApplication
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"Qt unavailable: {exc}")
        app = QApplication.instance() or QApplication([])

        class FakeProcess:
            pid = 876543

            def __init__(self):
                self.returncode = None
                self.signals = []

            def poll(self):
                return self.returncode

            def wait(self, timeout=None):
                if self.returncode is None:
                    raise TimeoutError(timeout)
                return self.returncode

            def send_signal(self, sig):
                self.signals.append(sig)
                if sig == signal.SIGKILL:
                    self.returncode = -signal.SIGKILL

        process = FakeProcess()
        controller = RecordingSessionController(
            process_factory=lambda *_args, **_kwargs: process,
            poll_interval_ms=1000,
            stop_timeout_s=0.05,
            terminate_timeout_s=0.05,
        )
        with tempfile.TemporaryDirectory() as directory, patch(
            "stereo_ground_sim.recording_control._send_group_signal",
            side_effect=lambda proc, sig: proc.send_signal(sig),
        ):
            root = Path(directory)
            self.assertTrue(controller.start(mode="lite", run_name="shutdown", save_root=root))
            controller.shutdown()
            self.assertIsNotNone(controller._shutdown_thread)
            controller._shutdown_thread.join(timeout=2.0)
            self.assertFalse(controller._shutdown_thread.is_alive())
            self.assertEqual(process.signals, [signal.SIGINT, signal.SIGTERM, signal.SIGKILL])
            self.assertIsNotNone(process.poll())
            self.assertIsNone(controller.process)
            self.assertIsNone(controller._lock_key)
            self.assertTrue(controller.shutdown_complete)
            self.assertEqual(controller.state, "ERROR")
        app.processEvents()

    def test_concurrent_callbacks_and_snapshot_have_no_shared_state_exception(self):
        recorder = object.__new__(operator_recorder.OperatorRecorder)
        recorder.latest = {}
        recorder.last_stamp = {}
        recorder.counts = {field: 0 for field in operator_recorder.COUNTER_FIELDS}
        recorder._lock = threading.RLock()
        recorder.start_monotonic = 0.0
        recorder.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_700_000_000_000_000_000)
        )
        output = tempfile.TemporaryFile(mode="w+", newline="")
        recorder.csv_file = output
        import csv

        fields = list(
            dict.fromkeys(
                ["time_sec", "ros_time_sec", "axis_contract"]
                + list(operator_recorder.NUMERIC_TOPICS.values())
                + operator_recorder.COMPARISON_FIELDS
                + operator_recorder.GROUND_TRUTH_POSITION_FIELDS
                + operator_recorder.ESTIMATED_POSITION_FIELDS
                + operator_recorder.CORRECTED_POSITION_FIELDS
                + operator_recorder.BBOX_ADJUSTMENT_FIELDS
                + operator_recorder.DETECTION_FIELDS
                + operator_recorder.PROFILE_STATUS_FIELDS
                + operator_recorder.RESEARCH_FIELDS
                + operator_recorder.RECEIVE_TIME_FIELDS
                + operator_recorder.RECEIVE_TIME_NS_FIELDS
                + operator_recorder.SOURCE_STATE_FIELDS
                + operator_recorder.SOURCE_RECEIVE_AGE_FIELDS
                + operator_recorder.FLIGHT_CONTRACT_FIELDS
            )
        )
        recorder.csv_writer = csv.DictWriter(output, fieldnames=fields)
        recorder.csv_writer.writeheader()
        errors = []

        def callback_worker():
            try:
                for index in range(80):
                    recorder._ground_truth_position_cb(
                        SimpleNamespace(data=[float(index), 1.0, 2.0, 3.0])
                    )
            except Exception as exc:  # pragma: no cover - assertion captures regressions
                errors.append(exc)

        def snapshot_worker():
            try:
                for _index in range(80):
                    recorder._write_csv_row()
            except Exception as exc:  # pragma: no cover - assertion captures regressions
                errors.append(exc)

        threads = [threading.Thread(target=callback_worker) for _ in range(2)]
        threads.append(threading.Thread(target=snapshot_worker))
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=2.0)
            self.assertFalse(thread.is_alive())
        output.close()
        self.assertEqual(errors, [])

    def test_finalize_is_idempotent(self):
        recorder = object.__new__(operator_recorder.OperatorRecorder)
        recorder._finalize_lock = threading.Lock()
        recorder._finalized = False
        recorder._finalize_result = False
        recorder._lock = threading.RLock()
        recorder.mode = "lite"
        recorder.video_writers = {}
        recorder.csv_file = None
        recorder._parameter_snapshot_status = "available"
        recorder._stop_parameter_timer = lambda: None
        recorder._complete_parameter_snapshot = lambda: None
        calls = []
        recorder._finalize_manifest = lambda: calls.append("finalize") or True

        self.assertTrue(recorder.finalize())
        self.assertTrue(recorder.finalize())
        self.assertEqual(calls, ["finalize"])

    def test_valid_research_fixture_finalizes_formal_with_ns_and_video_counts(self):
        now_ns = 1_700_000_000_000_000_000
        recorder = object.__new__(operator_recorder.OperatorRecorder)
        recorder.mode = "full"
        recorder.session_dir = Path(tempfile.mkdtemp())
        recorder.csv_hz = 30.0
        recorder.video_fps = 20.0
        recorder.bag_storage = "sqlite3"
        recorder._lock = threading.RLock()
        recorder.latest = {
            "ground_truth": [10.0, 1.0, 2.0, 10.3],
            "observed_raw": [10.1, 1.1, 2.1, 10.5],
            "analytic_raw": [10.2, 1.2, 2.2, 10.6],
            "corrected": [10.0, 1.0, 2.0, 10.3],
            "attitude_status": {
                "oracle_source_role": "target_reference_link",
                "oracle_valid": True,
                "oracle_fallback_used": False,
            },
            "observation_status": {
                "observation_mode": "both",
                "oracle_source_role": "target_reference_link",
                "oracle_valid": True,
                "oracle_fallback_used": False,
                "bbox_left_valid": True,
                "bbox_right_valid": True,
                "bbox_left_u_px": 100.0,
                "bbox_left_v_px": 50.0,
                "bbox_right_u_px": 90.0,
                "bbox_right_v_px": 50.0,
            },
            "profile_status": {"profile_mode": "no_profile"},
            "px4_status": {
                "px4_connected": True,
                "flight_contract": {
                    "verified": True,
                    "source": "stereo_distance_node_parameters",
                    "flight_authority": "px4_sitl",
                    "fly_target_in_gazebo": False,
                    "mission_profile": "manual",
                    "actual_distance_source": "gazebo",
                },
            },
            "left_image": {"received": True},
            "right_image": {"received": True},
        }
        recorder.last_stamp = {
            "/ground_truth_position": now_ns,
            "/stereo/estimated_position": now_ns,
            "/stereo/attitude_model/raw/estimated_position": now_ns,
            "/stereo/corrected_position": now_ns,
            "/stereo/attitude_model/status": now_ns,
            "/stereo/observation_status": now_ns,
            "/stereo/profile_status": now_ns,
            "/simulation/px4/status": now_ns,
            "/left_camera/detection_image": now_ns,
            "/right_camera/detection_image": now_ns,
        }
        recorder.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns)
        )
        recorder.counts = {field: 0 for field in operator_recorder.COUNTER_FIELDS}
        recorder.counts.update(
            csv_row_count=1,
            ground_truth_message_count=1,
            ground_truth_valid_sample_count=1,
            observed_message_count=1,
            observed_valid_sample_count=1,
            analytic_message_count=1,
            analytic_valid_sample_count=1,
            corrected_message_count=1,
            corrected_valid_sample_count=1,
            attitude_status_message_count=1,
            observation_status_message_count=1,
            profile_status_message_count=1,
            px4_status_message_count=1,
            left_image_received_count=1,
            right_image_received_count=1,
            left_video_frame_count=1,
            right_video_frame_count=1,
        )
        recorder._video_writer_errors = {}
        recorder._bag_error = None
        recorder._bag_return_code = None
        recorder._bag_reaped = True
        recorder._parameter_snapshot_status = "available"
        recorder._parameter_snapshot_error = ""
        recorder._parameter_snapshot_values = {"observation_mode": "both"}
        recorder._readiness_at_start = {
            "start_allowed": True,
            "formal_eligible": True,
            "reasons": [],
        }
        recorder._manifest = recorder._initial_manifest()
        (recorder.session_dir / "telemetry.csv").write_text("header\nrow\n", encoding="utf-8")
        (recorder.session_dir / "left_overlay.mp4").write_bytes(b"left")
        (recorder.session_dir / "right_overlay.mp4").write_bytes(b"right")
        (recorder.session_dir / "parameter_snapshot.json").write_text(
            json.dumps({"status": "available"}), encoding="utf-8"
        )
        recorder._write_manifest()

        self.assertTrue(recorder._finalize_manifest())
        manifest = json.loads((recorder.session_dir / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["recording_state"], "complete")
        self.assertTrue(manifest["artifact_integrity"])
        self.assertTrue(manifest["formal_experiment_eligible"])
        self.assertEqual(manifest["counts"]["left_video_frame_count"], 1)
        self.assertEqual(manifest["readiness_at_end"]["formal_eligible"], True)


if __name__ == "__main__":
    unittest.main()
