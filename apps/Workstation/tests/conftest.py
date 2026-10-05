"""Pytest path setup for Workstation tests moved to the repository root."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest


# These values must be fixed before any test module imports PyQt5.  Keeping
# rendering in software also prevents OpenGL/CUDA teardown order from making
# otherwise deterministic widget tests crash in a headless process.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_OPENGL", "software")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
APPS_ROOT = REPOSITORY_ROOT / "apps"
STEREO_WIRIS_ROOT = APPS_ROOT / "StereoWIRIS"
COMMON_PYTHON_ROOT = REPOSITORY_ROOT / "common" / "python"

collect_ignore = []
if os.environ.get("WORKSTATION_RUN_LOCAL_PHOTO_REGRESSION") != "1":
    collect_ignore.append("test_optical_pose_mountain_real_photo_regression.py")

_LOCAL_TEST_FILES = {
    "test_optical_pose_mountain_real_photo_regression.py",
}
_INTEGRATION_NAME_PARTS = (
    "_integration",
    "_end_to_end",
    "_ui",
    "_workspace",
    "_lifecycle",
    "_review",
    "_session_controller",
    "_session_persistence",
    "_owned_workflow",
)
_INTEGRATION_TEST_FILES = {
    "test_panel_delegate_bindings.py",
    "test_rtcm_pipeline_workers.py",
    "test_runtime_closure.py",
}

for path in (REPOSITORY_ROOT, APPS_ROOT, COMMON_PYTHON_ROOT, STEREO_WIRIS_ROOT):
    path_text = str(path)
    if path_text not in sys.path:
        sys.path.insert(0, path_text)


_QAPPLICATION_REFERENCES: list[object] = []


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line("markers", "unit: isolated logic or component contract test")
    config.addinivalue_line("markers", "integration: cross-layer, UI, persistence, or worker handoff test")
    config.addinivalue_line("markers", "local: opt-in test requiring machine-local assets or capabilities")


def _test_layer(path: Path) -> str:
    name = path.name
    if name in _LOCAL_TEST_FILES:
        return "local"
    if (
        name.startswith("test_workstation_")
        or name in _INTEGRATION_TEST_FILES
        or any(part in name for part in _INTEGRATION_NAME_PARTS)
    ):
        return "integration"
    return "unit"


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    for item in items:
        item.add_marker(getattr(pytest.mark, _test_layer(Path(str(item.fspath)))))


@pytest.fixture(scope="session", autouse=True)
def workstation_qapplication() -> object:
    """Keep one QApplication alive for the complete Workstation test run.

    Most historical tests construct a QApplication inline.  Without a
    session-owned strong reference, PyQt can destroy and recreate the C++
    singleton while widgets or queued deletes from a previous test still
    exist, which caused a cumulative segmentation fault in test_workstation.
    """

    from PyQt5 import QtCore, QtWidgets

    application = QtWidgets.QApplication.instance()
    if application is None:
        QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseSoftwareOpenGL, True)
        application = QtWidgets.QApplication(["workstation-tests", "-platform", "offscreen"])
    application.setQuitOnLastWindowClosed(False)
    _QAPPLICATION_REFERENCES.append(application)
    yield application

    for widget in tuple(application.topLevelWidgets()):
        try:
            widget.close()
            widget.deleteLater()
        except RuntimeError:
            pass
    application.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    application.processEvents()
