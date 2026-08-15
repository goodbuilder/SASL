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

for path in (REPOSITORY_ROOT, APPS_ROOT, COMMON_PYTHON_ROOT, STEREO_WIRIS_ROOT):
    path_text = str(path)
    if path_text not in sys.path:
        sys.path.insert(0, path_text)


_QAPPLICATION_REFERENCES: list[object] = []


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
