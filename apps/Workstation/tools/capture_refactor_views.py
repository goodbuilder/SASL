#!/usr/bin/env python3
"""Render the real panel/shell with hardware startup disabled in an isolated netns.

Example from the repository root:
  unshare --user --map-root-user --net python3 -B apps/Workstation/tools/capture_refactor_views.py --output <artifact-dir>
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import traceback
from unittest.mock import patch


def _skip_hardware_startup(_panel) -> None:
    """Keep the callable alive until Qt has dispatched queued startup slots."""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-root", type=Path,
                        help="Optional preserved checkout for before/after characterization")
    args = parser.parse_args()
    links = json.loads(subprocess.check_output(["ip", "-j", "link", "show"], text=True))
    if [link["ifname"] for link in links] != ["lo"]:
        raise SystemExit("Refusing capture outside a loopback-only network namespace")
    subprocess.run(["ip", "link", "set", "lo", "up"], check=True)
    root = args.source_root.resolve() if args.source_root else Path(__file__).resolve().parents[3]
    sys.path[:0] = [str(root / "apps"), str(root / "common/python")]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    os.environ["QT_OPENGL"] = "software"
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
    with tempfile.TemporaryDirectory(prefix="workstation-refactor-visual-") as temporary:
        data = Path(temporary) / "workstation_data"
        os.environ["SASL_WORKSTATION_DATA_ROOT"] = str(data)
        os.environ["XDG_CONFIG_HOME"] = str(Path(temporary) / "config")
        os.environ["XDG_CACHE_HOME"] = str(Path(temporary) / "cache")
        os.environ["QT_SHADER_CACHE_PATH"] = str(Path(temporary) / "shader-cache")
        from PyQt5 import QtCore, QtTest, QtWidgets
        from Workstation.application.standalone_gui import WirisStereoDistancePanel
        from Workstation.application.standalone_backend import StandalonePanelBackend
        from Workstation.application.main_window import WorkstationMainWindow

        callback_errors, rendering_limitations = [], []

        def exception_hook(kind, value, tb):
            trace = traceback.extract_tb(tb)
            known_offscreen_gl = (
                isinstance(value, RuntimeError)
                and str(value).startswith("pyqtgraph.opengl: Requires >= OpenGL 2.1;")
                and any(frame.filename.endswith("pyqtgraph/opengl/GLViewWidget.py") for frame in trace)
            )
            target = rendering_limitations if known_offscreen_gl else callback_errors
            target.append(f"{kind.__name__}: {value}")
            traceback.print_exception(kind, value, tb)

        sys.excepthook = exception_hook
        QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseSoftwareOpenGL, True)
        app = QtWidgets.QApplication([])
        with patch.object(WirisStereoDistancePanel, "_initial_status", _skip_hardware_startup), \
                patch.object(WirisStereoDistancePanel, "_auto_load_intrinsics", _skip_hardware_startup):
            panel = WirisStereoDistancePanel(app_root=data, auto_start=False, auto_start_pixhawk=False)
            app.processEvents()
        window = WorkstationMainWindow(backend=StandalonePanelBackend(panel))
        window.resize(1366, 900)
        window.show()
        views = (
            ("overview", "overview"),
            ("devices.cameras", "camera"),
            ("processing.measurement", "measurement"),
            ("tasks.data", "data"),
            ("tasks.calibration", "calibration"),
            ("tasks.advanced", "advanced"),
        )
        observed = []
        for context, name in views:
            window.navigation.select_context(context)
            app.processEvents()
            QtTest.QTest.qWait(40)
            path = output / f"{name}.png"
            assert window.grab().save(str(path))
            observed.append({"context": context, "image": path.name,
                             "width": window.width(), "height": window.height()})
        for _ in range(200):
            window.close()
            app.processEvents()
            if not window.isVisible():
                break
            QtTest.QTest.qWait(10)
        shutdown = panel._app_lifecycle_controller()._shutdown_resource_snapshot()
        if window.isVisible() or not shutdown["complete"]:
            raise RuntimeError(f"visual fixture did not finish clean shutdown: {shutdown}")
        summary = {
            "environment": "offscreen real panel and shell; hardware startup disabled; isolated network",
            "views": observed,
            "feature_count": len(getattr(getattr(panel, "application", None), "features", {})),
            "shutdown": shutdown, "callback_errors": callback_errors,
            "rendering_limitations": rendering_limitations,
        }
        (output / "visual-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        window.deleteLater()
        app.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        app.processEvents()
        if callback_errors:
            raise RuntimeError(f"Unhandled Qt callback errors: {callback_errors}")
        print(json.dumps(summary))


if __name__ == "__main__":
    main()
