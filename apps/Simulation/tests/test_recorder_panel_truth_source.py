from __future__ import annotations

import importlib.util
import signal
import sys
import tempfile
import types
import unittest
from unittest import mock
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def install_qt_stubs() -> None:
    qt_core = types.ModuleType("python_qt_binding.QtCore")
    qt_widgets = types.ModuleType("python_qt_binding.QtWidgets")
    qt_gui = types.ModuleType("qt_gui.plugin")

    class _Dummy:
        def __init__(self, *args, **kwargs):
            pass

        def __getattr__(self, _name):
            return _Dummy()

        def __call__(self, *args, **kwargs):
            return _Dummy()

    class _Plugin:
        def __init__(self, *args, **kwargs):
            pass

    qt_core.QTimer = _Dummy
    for name in (
        "QApplication",
        "QCheckBox",
        "QComboBox",
        "QDockWidget",
        "QFileDialog",
        "QGridLayout",
        "QHBoxLayout",
        "QLabel",
        "QLineEdit",
        "QMainWindow",
        "QPushButton",
        "QVBoxLayout",
        "QWidget",
    ):
        setattr(qt_widgets, name, _Dummy)
    qt_gui.Plugin = _Plugin

    sys.modules["python_qt_binding"] = types.ModuleType("python_qt_binding")
    sys.modules["python_qt_binding.QtCore"] = qt_core
    sys.modules["python_qt_binding.QtWidgets"] = qt_widgets
    sys.modules["qt_gui"] = types.ModuleType("qt_gui")
    sys.modules["qt_gui.plugin"] = qt_gui


def load_panel_module():
    install_qt_stubs()
    spec = importlib.util.spec_from_file_location(
        "recorder_panel_under_test",
        ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "recorder_panel.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load recorder_panel.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RecorderPanelTruthSourceTests(unittest.TestCase):
    def test_calibration_command_includes_selected_truth_source_and_oracle_topic(self) -> None:
        module = load_panel_module()

        cmd = module.build_calibration_recorder_command(
            python_executable="python3",
            output_csv="/tmp/calibration_global.csv",
            truth_source="global_position",
            sim_truth_topic="/ground_truth_position",
            sim_truth_role="oracle",
        )

        self.assertEqual(cmd[:3], ["python3", "-m", "stereo_ground_sim.calibration_csv_recorder"])
        self.assertIn("--truth-source", cmd)
        self.assertEqual(cmd[cmd.index("--truth-source") + 1], "global_position")
        self.assertIn("--sim-truth-topic", cmd)
        self.assertEqual(cmd[cmd.index("--sim-truth-topic") + 1], "/ground_truth_position")
        self.assertEqual(cmd[cmd.index("--sim-truth-role") + 1], "oracle")

    def test_calibration_command_omits_sim_truth_topic_when_oracle_unchecked(self) -> None:
        module = load_panel_module()

        cmd = module.build_calibration_recorder_command(
            python_executable="python3",
            output_csv="/tmp/calibration_gps.csv",
            truth_source="gps_position",
            sim_truth_topic="",
        )

        self.assertNotIn("--sim-truth-topic", cmd)
        self.assertEqual(cmd[cmd.index("--truth-source") + 1], "gps_position")

    def test_recording_buttons_keep_csv_mp4_and_debug_presets_mapped_to_recorder_modes(self) -> None:
        module = load_panel_module()

        self.assertEqual(module.PRIMARY_RECORD_BUTTON_TEXT, "Save CSV")
        self.assertEqual(
            module.RECORDING_MODE_BUTTONS,
            (
                ("lite", "Save CSV"),
                ("full", "Save MP4"),
                ("debug", "Save Debug"),
            ),
        )

        for mode, _button_text in module.RECORDING_MODE_BUTTONS:
            with self.subTest(mode=mode):
                self.assertEqual(
                    module.build_operator_recorder_command(
                        python_executable="python3",
                        mode=mode,
                        output_dir=f"/tmp/operator_{mode}",
                    ),
                    [
                        "python3",
                        "-m",
                        "stereo_ground_sim.operator_recorder",
                        "--mode",
                        mode,
                        "--output-dir",
                        f"/tmp/operator_{mode}",
                    ],
                )

    def test_auto_mission_command_and_stop_helper_use_owned_process_group(self) -> None:
        module = load_panel_module()
        self.assertEqual(
            module.build_mission_command("auto_fov", root_dir=Path("/sim")),
            ["/sim/run_auto.sh"],
        )

        class FakeProcess:
            pid = 321

            def __init__(self):
                self.return_code = None

            def poll(self):
                return self.return_code

            def wait(self, timeout=None):
                self.return_code = 0
                return self.return_code

        process = FakeProcess()
        with mock.patch.object(module.os, "killpg") as killpg:
            module.stop_process_group(process)

        killpg.assert_called_once_with(process.pid, signal.SIGINT)

    def test_stop_button_is_shared_by_recording_and_mission_lifecycle(self) -> None:
        source = (ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "recorder_panel.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('self._stop_button.clicked.connect(self._stop_all)', source)
        self.assertIn('"Stopping Auto and finalizing recording…"', source)
        self.assertIn("os.killpg(process.pid, signal.SIGINT)", source)

    def test_auto_does_not_create_a_mission_recorder_and_save_buttons_remain_independent(self) -> None:
        source = (ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "recorder_panel.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn('cmd.extend(["--record-dir", str(mission_record_path)])', source)
        self.assertNotIn("Stop the active recorder before starting Auto", source)
        self.assertIn("recording starts at the button click", source)

    def test_csv_tail_preview_keeps_header_and_latest_persisted_rows(self) -> None:
        module = load_panel_module()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.csv"
            path.write_text("time_sec,value\n0.0,first\n0.1,second\n0.2,third\n", encoding="utf-8")

            preview = module.read_recent_csv_lines(path, data_row_count=2)

        self.assertEqual(preview.splitlines(), ["time_sec,value", "0.1,second", "0.2,third"])

    def test_recording_artifact_paths_follow_timestamped_progress_metadata(self) -> None:
        module = load_panel_module()
        controls = object.__new__(module.RecordingControls)
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            csv_path = session / "telemetry_20260729_143015.csv"
            xlsx_path = session / "session_20260729_143015.xlsx"
            csv_path.write_text("time_sec\n", encoding="utf-8")
            xlsx_path.write_bytes(b"xlsx")
            (session / "progress.json").write_text(
                '{"csv":"telemetry_20260729_143015.csv","xlsx":"session_20260729_143015.xlsx"}',
                encoding="utf-8",
            )
            controls._last_record_path = session

            self.assertEqual(controls.recording_csv_path(), csv_path)
            self.assertEqual(controls.recording_xlsx_path(), xlsx_path)

    def test_csv_tail_preview_discards_an_incomplete_trailing_row(self) -> None:
        module = load_panel_module()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.csv"
            path.write_text(
                "time_sec,value\n0.0,first\n0.1,second\n0.2,partial",
                encoding="utf-8",
            )

            preview = module.read_recent_csv_lines(path, data_row_count=3)

        self.assertEqual(preview.splitlines(), ["time_sec,value", "0.0,first", "0.1,second"])

    def test_analysis_export_preserves_empty_values_and_keeps_the_raw_csv_unchanged(self) -> None:
        module = load_panel_module()
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "telemetry.csv"
            output = Path(directory) / "analysis.csv"
            source.write_text(
                "time_sec,nominal_pose_range_m,left_x1,right_x1,gps_baseline_forward_m,"
                "global_baseline_forward_m,gps_baseline_minus_global_baseline_delta_m,extra\n"
                "1.0,2000.0,10,20,1999.0,1998.0,1.0,retain-only-in-raw\n"
                "2.0,,,,,,,\n",
                encoding="utf-8",
            )
            raw_before = source.read_text(encoding="utf-8")

            module.export_analysis_csv(source, output)

            self.assertEqual(source.read_text(encoding="utf-8"), raw_before)
            lines = output.read_text(encoding="utf-8").splitlines()
            self.assertEqual(lines[0], ",".join(module.ANALYSIS_CSV_FIELDS))
            self.assertEqual(lines[1], "1.0,2000.0,10,20,1999.0,1998.0,1.0")
            self.assertEqual(lines[2], "2.0,,,,,,")


if __name__ == "__main__":
    unittest.main()
