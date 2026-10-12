"""Real panel/shell event-loop regression for the isolated visual QA fixture."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


def test_visual_capture_drains_startup_callbacks_and_closes_all_resources(tmp_path):
    namespace_tool = shutil.which("unshare")
    if namespace_tool is None:
        pytest.skip("Linux user/network namespaces are required for device-isolated capture")
    repository = Path(__file__).resolve().parents[3]
    output = tmp_path / "views"
    environment = os.environ.copy()
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    result = subprocess.run(
        [namespace_tool, "--user", "--map-root-user", "--net", sys.executable,
         "-B", str(repository / "apps/Workstation/tools/capture_refactor_views.py"),
         "--output", str(output)],
        cwd=repository, env=environment, text=True, capture_output=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    # Qt reports an invalid deferred Python callable only through excepthook;
    # a zero subprocess exit status and a successful grab do not prove dispatch.
    assert "TypeError:" not in result.stderr, result.stderr
    assert "object is not callable" not in result.stderr, result.stderr
    summary = json.loads((output / "visual-summary.json").read_text())
    assert summary["callback_errors"] == []
    assert isinstance(summary["rendering_limitations"], list)
    for limitation in summary["rendering_limitations"]:
        assert limitation.startswith("RuntimeError: pyqtgraph.opengl: Requires >= OpenGL 2.1;")
    assert [view["context"] for view in summary["views"]] == [
        "overview", "devices.cameras", "processing.measurement", "tasks.data",
        "tasks.calibration", "tasks.advanced",
    ]
    for view in summary["views"]:
        image = output / view["image"]
        assert image.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        assert (view["width"], view["height"]) == (1366, 900)
    assert summary["shutdown"]["complete"] is True
    assert summary["shutdown"]["running_threads"] == []
    assert summary["shutdown"]["live_processes"] == []
    assert summary["shutdown"]["open_sockets"] == []
    assert summary["shutdown"]["shared_memory"] == []
