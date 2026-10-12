"""Observation lifecycle decisions must run without a Qt widget or storage."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

import pytest

from Workstation.workstation_core.measurement_run import MeasurementRunService


def test_run_reserves_start_before_io_and_ignores_stale_finish():
    run = MeasurementRunService()
    assert run.queue("camera busy")
    assert run.snapshot.state == "QUEUED"
    assert run.begin("run-1", 100, 200, {"time_basis": "app_receive_monotonic"})
    assert run.snapshot.state == "STARTING"
    assert not run.begin("run-duplicate", 101, 201, {})
    run.set_state("RUNNING", "observing")
    assert run.mark_stopping()
    assert run.finish("run-1") is not None
    assert run.snapshot.state == "IDLE"
    assert run.begin("run-2", 300, 400, {})
    run.set_state("RUNNING", "observing")
    assert run.finish("run-1") is None
    assert run.snapshot.generation == "run-2"
    assert run.snapshot.state == "RUNNING"


def test_sources_and_exported_metadata_cannot_change_run_ownership():
    metadata = {"config": {"limits": [1, 2]}}
    run = MeasurementRunService()
    run.begin("run-1", 100, 200, metadata)
    run.set_state("RUNNING", "observing")
    metadata["config"]["limits"].clear()
    first = run.source_transition("camera.left", "LIVE", "frame", 110, 210)
    assert first.accepted and first.event["runtime_generation"] == "run-1"
    assert run.source_transition("camera.right", "LIVE", "frame", 111, 211).accepted
    assert run.snapshot.active_sides == ("left", "right")
    assert run.source_transition("camera.left", "LIVE", "frame", 112, 212).event is None
    assert run.source_transition("camera.right", "UNAVAILABLE", "lost", 113, 213).accepted
    assert run.snapshot.active_sides == ("left",)
    assert run.snapshot.state == "RUNNING"
    public = run.snapshot.to_mapping()
    public["sources"]["camera.left"]["state"] = "LEFT"
    assert run.snapshot.sources["camera.left"]["state"] == "LIVE"
    assert list(run.snapshot.metadata["config"]["limits"]) == [1, 2]
    with pytest.raises(TypeError):
        run.snapshot.sources["camera.left"]["state"] = "LEFT"


def test_source_validation_cancel_and_idle_are_explicit():
    run = MeasurementRunService()
    assert not run.source_transition("camera.left", "LIVE", "frame", 1, 2).accepted
    assert not run.cancel("idle")
    assert run.queue("owner busy") and run.cancel("operator cancelled")
    assert run.snapshot.state == "IDLE"
    run.begin("run-1", 1, 2, {})
    run.set_state("RUNNING", "observing")
    with pytest.raises(ValueError, match="source_id"):
        run.source_transition("foreign", "LIVE", "bad", 1, 2)
    with pytest.raises(ValueError, match="source state"):
        run.source_transition("camera.left", "imaginary", "bad", 1, 2)
    assert not run.cancel("must use stop")


def test_headless_run_module_has_no_qt_or_gui_dependency():
    root = Path(__file__).resolve().parents[3]
    env = dict(os.environ, PYTHONPATH=os.pathsep.join((str(root / "apps"), str(root / "common/python"))))
    code = """
import sys
from Workstation.workstation_core.measurement_run import MeasurementRunService
run = MeasurementRunService()
assert run.begin('headless', 1, 2, {})
assert not any(name.startswith(('PyQt5', 'Workstation.standalone_gui', 'Workstation.recording_controller')) for name in sys.modules)
"""
    subprocess.run([sys.executable, "-B", "-c", code], env=env, check=True, capture_output=True, text=True)
