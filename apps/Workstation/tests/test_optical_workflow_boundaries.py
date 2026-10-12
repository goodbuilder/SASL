"""Optical workflow ownership, request identity, and apply failure boundaries.

Synthetic native u/v files and known NED/RDF poses are used; no device command,
network operation, runtime truth or changed solver formula is introduced.
"""

from __future__ import annotations

import ast
import copy
from pathlib import Path
import threading
import time

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.optical_pose.application import (
    OpticalApplicationEffects,
    OpticalApplicationInputs,
    OpticalProfileApplication,
)
from Workstation.optical_pose.capture import FrozenCaptureInputs, FrozenNativeFrame
from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.tests.test_optical_pose_controller_integration import _active_geometry_state, _annotations, _panel
from Workstation.workstation_core.active_geometry import ActiveGeometryStore


class HeldOpticalWorker(QtCore.QObject):
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, result, *, duplicate=False):
        super().__init__()
        self.result = result
        self.duplicate = duplicate
        self.entered = threading.Event()
        self.release_result = threading.Event()
        self.release_finish = threading.Event()

    @QtCore.pyqtSlot()
    def run(self):
        self.entered.set()
        if self.release_result.wait(3.0):
            self.completed.emit(self.result)
            if self.duplicate:
                self.completed.emit(self.result)
                self.failed.emit("duplicate failure")
        else:
            self.failed.emit("test result timeout")
        self.release_finish.wait(3.0)
        self.finished.emit()


def _until(predicate):
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert predicate()


def _close(owner, jobs):
    for job in jobs:
        job.worker.release_result.set()
        job.worker.release_finish.set()
    owner.request_shutdown()
    for job in jobs:
        try:
            job.thread.quit()
            assert job.thread.wait(3000)
        except RuntimeError:
            pass
    _until(lambda: owner.current is None)
    owner.deleteLater()
    QtWidgets.QApplication.instance().processEvents()


def _capture_fixture(tmp_path):
    panel = _panel(tmp_path)
    panel._log = lambda _message: None
    controller = OpticalPoseController(panel)
    inputs = controller._capture_inputs({})
    cameras = {}
    for side in ("left", "right"):
        path = tmp_path / f"{side}.png"
        assert panel.left_pane.latest_native_measurement_frame()[1].save(str(path))
        cameras[side] = {"state": "completed", "ftp_downloaded": True, "ftp_local_path": str(path)}
    return panel, controller, inputs, cameras


def _start(owner, inputs, cameras, *, identity="capture-1", mode="hr", duplicate=False):
    payload = {
        "_ftp_request_zoom_snapshot": inputs.evidence("zoom", {}),
        "_ftp_request_monotonic_s": inputs.captured_monotonic_s,
    }
    job = owner.start_ftp(
        mode=mode,
        capture_id=identity,
        inputs=inputs,
        payload=payload,
        worker_factory=lambda: HeldOpticalWorker(
            {"capture_id": identity, "cameras": copy.deepcopy(cameras)}, duplicate=duplicate
        ),
    )
    assert job.worker.entered.wait(1.0)
    return job


def _solved(tmp_path):
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    solved = controller.solve_workstation_optical_pose_horizon({"annotations": _annotations(frozen)})
    return panel, controller, solved["candidates"][-1]["candidate_id"]


def test_boundary_modules_have_explicit_inputs_and_no_controller_or_panel_dependency():
    root = Path(__file__).parents[1] / "optical_pose"
    modules = ("capture", "selection", "observations", "application", "presentation", "artifacts")
    for name in modules:
        tree = ast.parse((root / f"{name}.py").read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                assert not any(
                    part.startswith("panel_") or part in {"optical_pose_controller", "main_window", "workspace_views"}
                    for part in (node.module or "").split(".")
                )
            if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
                assert node.attr not in {"panel", "controller", "window", "host"}
            if isinstance(node, ast.FunctionDef):
                assert node.name != "__getattr__"
                assert not {"panel", "controller", "window", "host"} & {
                    arg.arg for arg in (*node.args.args, *node.args.kwonlyargs)
                }


def test_capture_inputs_detach_gnss_zoom_metadata_and_native_pixels(tmp_path):
    evidence = {"gnss": {"left": {"latitude_deg": 37.0}}, "zoom": {"left": {"index": 8}}}
    inputs = FrozenCaptureInputs("s", tmp_path, 12.0, None, None, evidence)
    image = QtGui.QImage(2, 2, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("red"))
    metadata = {"native_width": 2, "producer": {"frame": 1}}
    frame = FrozenNativeFrame(12.0, image, metadata)
    evidence["zoom"]["left"]["index"] = 1
    inputs.evidence("gnss")["left"]["latitude_deg"] = 0.0
    image.fill(QtGui.QColor("green"))
    metadata["producer"]["frame"] = 2
    materialized = frame.materialize()
    assert inputs.evidence("zoom")["left"]["index"] == 8
    assert inputs.evidence("gnss")["left"]["latitude_deg"] == 37.0
    assert materialized[1].pixelColor(0, 0) == QtGui.QColor("red")
    assert materialized[2]["producer"]["frame"] == 1
    materialized[1].fill(QtGui.QColor("blue"))
    assert frame.materialize()[1].pixelColor(0, 0) == QtGui.QColor("red")


def test_frozen_applicability_and_measurement_exports_do_not_follow_later_live_edits(tmp_path):
    panel, controller, candidate_id = _solved(tmp_path)
    before = controller._intrinsics_snapshot(use_frozen=True)
    panel.operating_intrinsics_profile = None
    panel.active_intrinsics = None
    panel.actual_visible_zoom["left"]["index"] = 2
    panel.camera_identity_evidence = {"left": {"serial": "changed"}}
    panel.selected_extrinsics_profile = None
    assert controller._intrinsics_snapshot(use_frozen=True) == before
    evidence = controller.workstation_optical_pose_measurement_evidence()
    evidence["candidates"].clear()
    evidence["candidate_priors"].clear()
    assert candidate_id in controller._candidates
    assert candidate_id in controller._candidate_priors
    assert controller._frozen_frames is controller.capture_owner._frozen_frames
    assert controller._drone_rtk_observations is controller.capture_owner.observations._drone_rtk_observations


@pytest.mark.parametrize("mode", ("hr", "oz"))
def test_ftp_result_keeps_admitted_inputs_and_resources_until_actual_finish(tmp_path, mode):
    panel, controller, inputs, cameras = _capture_fixture(tmp_path)
    owner = controller.capture_owner
    seen_threads = []
    original_emit = owner._emit
    owner._emit = lambda name, value: (seen_threads.append(QtCore.QThread.currentThread()), original_emit(name, value))
    jobs = []
    try:
        job = _start(owner, inputs, cameras, mode=mode, duplicate=True)
        jobs.append(job)
        panel.operating_intrinsics_profile = None
        panel.actual_visible_zoom["left"]["index"] = 1
        panel.camera_frame_metadata["left"]["camera_serial"] = "new-live-camera"
        owner.request_shutdown()
        job.worker.release_result.set()
        _until(lambda: bool(owner._frozen_frames))
        assert owner._frozen_intrinsics_profile is inputs.profile
        assert owner._frozen_frames["left"]["zoom"]["index"] == 8
        assert owner._frozen_frames["left"]["metadata"]["camera_serial"] == "SERIAL-LEFT"
        assert owner._frozen_frames["left"]["artifact"].timestamp_reference == "import-receive"
        assert owner._frozen_frames["left"]["metadata"]["import_received_monotonic_s"] >= inputs.captured_monotonic_s
        assert len(owner.observations._drone_rtk_observations) == 1
        assert all(thread is owner.thread() for thread in seen_threads)
        assert owner.current is job
        owner.retire(job)
        assert owner.current is job
        assert job.thread in owner.resource_inventory().values()
        with pytest.raises(RuntimeError, match="shutting down"):
            _start(owner, inputs, cameras, identity="closed")
        job.worker.release_finish.set()
        _until(lambda: owner.current is None)
        assert all(value is None for value in owner.resource_inventory().values())
    finally:
        _close(owner, jobs)


def test_old_session_result_and_old_finished_do_not_replace_new_burst_or_job(tmp_path):
    _panel_value, controller, inputs, cameras = _capture_fixture(tmp_path)
    owner = controller.capture_owner
    jobs = []
    try:
        old = _start(owner, inputs, cameras)
        jobs.append(old)
        controller.reset_workstation_optical_pose_session()
        newer = controller.capture_workstation_optical_pose_burst({})
        expected = newer["frozen_frame_artifacts"]
        old.worker.release_result.set()
        old.worker.release_finish.set()
        _until(lambda: owner.current is None)
        assert controller.workstation_optical_pose_snapshot()["frozen_frame_artifacts"] == expected
        assert len(owner.observations._drone_rtk_observations) == 1
        current = _start(owner, inputs, cameras, identity="capture-2", mode="oz")
        jobs.append(current)
        owner.deliver(old, value={"capture_id": "capture-1", "cameras": cameras})
        owner.retire(old)
        controller._handle_optical_pose_wiris_ftp_oz_capture_completed({"capture_id": "capture-2", "cameras": cameras})
        controller._clear_optical_pose_wiris_ftp_oz_capture_worker()
        assert owner.current is current
        assert controller.workstation_optical_pose_snapshot()["frozen_frame_artifacts"] == expected
    finally:
        _close(owner, jobs)


def test_ftp_result_superseded_by_manual_burst_retains_the_new_identity(tmp_path):
    _panel_value, controller, inputs, cameras = _capture_fixture(tmp_path)
    owner = controller.capture_owner
    jobs = []
    try:
        job = _start(owner, inputs, cameras)
        jobs.append(job)
        expected = controller.capture_workstation_optical_pose_burst({})["frozen_frame_artifacts"]
        job.worker.release_result.set()
        job.worker.release_finish.set()
        _until(lambda: owner.current is None)
        assert controller.workstation_optical_pose_snapshot()["frozen_frame_artifacts"] == expected
        assert any(name == "optical_pose_capture_superseded" for name, _value in controller.panel.events)
    finally:
        _close(owner, jobs)


def test_close_reentered_from_launch_log_never_starts_a_worker(tmp_path):
    _panel_value, controller, inputs, cameras = _capture_fixture(tmp_path)
    owner = controller.capture_owner
    worker = HeldOpticalWorker({"capture_id": "cancel", "cameras": cameras})
    owner._log = lambda _message: owner.request_shutdown()
    with pytest.raises(RuntimeError, match="cancelled before launch"):
        owner.start_ftp(mode="hr", capture_id="cancel", inputs=inputs, payload={}, worker_factory=lambda: worker)
    assert not worker.entered.is_set()
    assert owner.current is None
    assert not owner.busy
    owner.deleteLater()
    QtWidgets.QApplication.instance().processEvents()


def test_start_reporting_failure_after_launch_retains_result_and_thread(tmp_path, monkeypatch):
    _panel_value, controller, inputs, cameras = _capture_fixture(tmp_path)
    owner = controller.capture_owner
    original_start = QtCore.QThread.start

    def starts_then_raises(thread, *args, **kwargs):
        original_start(thread, *args, **kwargs)
        raise RuntimeError("reported after launch")

    monkeypatch.setattr(QtCore.QThread, "start", starts_then_raises)
    jobs = []
    try:
        job = _start(owner, inputs, cameras)
        jobs.append(job)
        assert "after launch" in owner.callback_error
        assert owner.current is job
        job.worker.release_result.set()
        job.worker.release_finish.set()
        _until(lambda: owner.current is None)
        assert len(owner.observations._drone_rtk_observations) == 1
        assert len(owner._frozen_frames) == 2
    finally:
        _close(owner, jobs)


def test_annotation_event_reentrancy_cannot_bind_old_pixels_to_a_new_burst(tmp_path):
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    snapshot = controller.capture_workstation_optical_pose_burst({})
    annotations = _annotations(snapshot, horizon=False)
    for side in ("left", "right"):
        annotations[side]["target_point"] = [{"u_px": 30.0, "v_px": 40.0}]
    original = panel.append_workstation_run_event

    def event(name, value, **kwargs):
        if name == "optical_pose_annotations_updated":
            controller.capture_workstation_optical_pose_burst({})
        return original(name, value, **kwargs)

    panel.append_workstation_run_event = event
    controller.record_workstation_optical_pose_annotation({"annotation_mode": "target", "annotations": annotations})
    assert controller._drone_rtk_multiview_samples == {}
    assert controller._annotation_review is None


def test_nested_apply_is_rejected_and_persistence_failure_restores_full_identity(tmp_path, monkeypatch):
    panel, controller, candidate_id = _solved(tmp_path)
    before = _active_geometry_state(panel, controller)
    nested = []

    def persist():
        with pytest.raises(RuntimeError, match="already in progress"):
            controller.apply_workstation_optical_pose_candidate({"candidate_id": candidate_id})
        nested.append(True)
        raise OSError("save failed")

    monkeypatch.setattr(controller, "_persist_session_alignment_snapshot", persist)
    with pytest.raises(OSError, match="save failed"):
        controller.apply_workstation_optical_pose_candidate({"candidate_id": candidate_id})
    assert nested == [True]
    assert _active_geometry_state(panel, controller) == before
    assert controller._profile_restore_stack == []


def test_old_application_failure_cannot_restore_over_new_shared_geometry_revision(tmp_path):
    panel, controller, candidate_id = _solved(tmp_path)
    registry = controller._candidate_registry
    owner = OpticalProfileApplication(registry)
    inputs = OpticalApplicationInputs(
        controller._intrinsics_snapshot(use_frozen=False),
        panel.active_intrinsics,
        panel._active_intrinsics_profile_path,
        panel._active_intrinsics_profile_hash,
        panel.selected_extrinsics_profile,
        2,
    )
    store = ActiveGeometryStore()
    restores = []

    def persist():
        store.update({"_active_optical_pose_selection_id": "later-owner-publication"})
        raise OSError("old save failed")

    effects = OpticalApplicationEffects(
        capture=lambda: {"old": True},
        commit_geometry=store.update,
        restore=restores.append,
        invalidate=lambda: None,
        persist=persist,
        revision=lambda: store.snapshot().revision,
    )
    with pytest.raises(OSError, match="old save failed"):
        owner.apply({"candidate_id": candidate_id}, inputs, effects)
    assert restores == []
    assert store.read_legacy("_active_optical_pose_selection_id") == "later-owner-publication"
    assert owner._profile_restore_stack == []


def test_revert_persistence_failure_keeps_active_state_and_retry_history(tmp_path, monkeypatch):
    panel, controller, candidate_id = _solved(tmp_path)
    controller.apply_workstation_optical_pose_candidate({"candidate_id": candidate_id})
    before = _active_geometry_state(panel, controller)
    history = list(controller._profile_restore_stack)
    monkeypatch.setattr(
        controller, "_persist_session_alignment_snapshot", lambda: (_ for _ in ()).throw(OSError("revert save failed"))
    )
    with pytest.raises(OSError, match="revert save failed"):
        controller.revert_workstation_optical_pose()
    assert _active_geometry_state(panel, controller) == before
    assert controller._profile_restore_stack == history
