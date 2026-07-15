from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
_MISSING = object()
_QT_MODULE_NAMES = (
    "python_qt_binding",
    "python_qt_binding.QtCore",
    "python_qt_binding.QtWidgets",
    "qt_gui",
    "qt_gui.plugin",
    "stereo_ground_sim.recording_control",
)


def install_qt_stubs():
    previous = {name: sys.modules.get(name, _MISSING) for name in _QT_MODULE_NAMES}
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
    return previous


def restore_qt_modules(previous) -> None:
    for name, module in previous.items():
        if module is _MISSING:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


def load_panel_module():
    previous = install_qt_stubs()
    try:
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
    finally:
        restore_qt_modules(previous)


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

    def test_operator_command_is_unchanged_for_lite_recording(self) -> None:
        module = load_panel_module()

        cmd = module.build_operator_recorder_command(
            python_executable="python3",
            mode="lite",
            output_dir="/tmp/operator_lite",
        )

        self.assertEqual(
            cmd,
            [
                "python3",
                "-m",
                "stereo_ground_sim.operator_recorder",
                "--mode",
                "lite",
                "--output-dir",
                "/tmp/operator_lite",
            ],
        )


if __name__ == "__main__":
    unittest.main()
