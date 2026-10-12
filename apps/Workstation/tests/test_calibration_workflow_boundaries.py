"""Calibration ownership, immutable inputs and worker-retirement boundaries."""
from __future__ import annotations

import ast
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtGui

from Workstation.calibration_workflow.capture import CalibrationCapture
from Workstation.calibration_workflow.capture_request import CaptureRequestBuilder
from Workstation.calibration_workflow.ftp import CalibrationFtpTransactions, FtpQueueActions
from Workstation.calibration_workflow.inputs import CalibrationInputs
from Workstation.calibration_workflow.jobs import CalibrationJobs
from Workstation.calibration_workflow.oz_ledger import OzCaptureLedger
from Workstation.calibration_workflow.profiles import CalibrationProfiles
from Workstation.calibration_workflow.quality import CalibrationQuality
from Workstation.calibration_workflow.state import CalibrationRuntimeState
from Workstation.calibration_workflow.timing import CaptureTiming
from Workstation.calibration_workflow.validation import CalibrationValidation
from Workstation.workstation_core.active_geometry import ActiveGeometryStore
from Workstation.workstation_core.calibration import CameraIntrinsics, CheckerboardConfig, StereoExtrinsics, StereoIntrinsicsResult
from Workstation.workstation_core.calibration_timing import HOST_RECEIVE_PROXY


ROOT = Path(__file__).resolve().parents[1] / "calibration_workflow"


def _assert_owner_boundary(source):
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
            assert node.attr not in {"panel", "controller", "host", "workspace"}
        if isinstance(node, ast.ImportFrom):
            assert (node.module or "").split(".")[-1] not in {"calibration_controller", "panel_calibration", "standalone_gui"}
        if isinstance(node, ast.FunctionDef) and node.name == "__init__":
            assert not ({a.arg for a in node.args.args} & {"panel", "controller", "host", "workspace"})


def test_workflow_owners_have_no_back_reference_to_outer_adapter():
    for path in ROOT.glob("*.py"):
        _assert_owner_boundary(path.read_text())


@pytest.mark.parametrize("source", [
    "class Owner:\n    def __init__(self, panel): self.panel = panel",
    "from Workstation.calibration_controller import CalibrationWorkflowController",
    "class Owner:\n    def run(self): return self.controller.run()",
])
def test_workflow_boundary_rejects_forbidden_fixture(source):
    with pytest.raises(AssertionError):
        _assert_owner_boundary(source)


def _result(*, translation=None, quality="PASS"):
    camera = CameraIntrinsics(32, 24, [[90.0, 0.0, 16.0], [0.0, 91.0, 12.0], [0.0, 0.0, 1.0]], [0.0], 0.2, 8)
    extrinsics = None if translation is None else StereoExtrinsics(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], list(translation), 0.3, 8,
    )
    return StereoIntrinsicsResult(camera, camera, CheckerboardConfig(8, 6, 0.03), extrinsics=extrinsics, calibration_quality={"state": quality, "reason_codes": []})


def test_capture_snapshot_detaches_native_pixels_and_nested_metadata(tmp_path):
    left = QtGui.QImage(7, 5, QtGui.QImage.Format_RGB888)
    left.fill(QtGui.QColor(11, 22, 33))
    metadata = {
        "left": {"app_receive_monotonic_ns": 1000000000, "native_lossless_source": True, "h264_access_unit_nal_types": [1]},
        "right": {"app_receive_monotonic_ns": 1010000000, "native_lossless_source": True},
    }
    request = CaptureRequestBuilder.build(
        root=tmp_path, stem="001", frames={"left": left, "right": left},
        metadata_by_camera=metadata, preview_metadata={}, capture_origin="manual",
        pair_max_delta_s=0.1, zoom_snapshot={"capture_zoom_level": 1},
        config=None, opencv_error="fixture",
    )
    left.fill(QtGui.QColor("red"))
    metadata["left"]["h264_access_unit_nal_types"].append(5)
    arguments = request.worker_arguments()
    assert arguments["frames"]["left"].pixelColor(0, 0) == QtGui.QColor(11, 22, 33)
    assert arguments["metadata_payload"]["frame_grid_by_side"]["left"]["h264_access_unit_nal_types"] == [1]
    assert arguments["metadata_payload"]["frame_grid_by_side"]["left"]["saved_width"] == 7
    assert arguments["metadata_payload"]["frame_grid_by_side"]["left"]["saved_height"] == 5
    assert arguments["timing_evidence"]["evidence_class"] == HOST_RECEIVE_PROXY
    assert not arguments["timing_evidence"]["actual_exposure_verified"]


def test_timing_owner_keeps_host_receipt_distinct_from_exposure():
    evidence = CaptureTiming()._checkerboard_capture_pair_timing_evidence(
        {"left": {"app_receive_monotonic_ns": 1000000000}, "right": {"app_receive_monotonic_ns": 1100000000}},
        maximum_delta_s=0.1,
    )
    assert evidence.pair_persistence_allowed
    assert evidence.pair_delta_s == pytest.approx(0.1)
    assert evidence.evidence_class == HOST_RECEIVE_PROXY
    assert not evidence.precision_qualified
    assert not evidence.actual_exposure_verified


def test_input_owner_freezes_file_hash_and_separates_monocular_membership(tmp_path):
    files = {}
    for side in ("left", "right"):
        (tmp_path / side).mkdir()
        files[side] = []
        for key in ("001", "002", "003"):
            path = tmp_path / side / f"{side}_{key}.png"
            path.write_bytes((side + key).encode())
            files[side].append(path)
    selection = {"schema": "workstation-calibration-file-selection-v2", "root": ".", "manual_mode": False,
                 "excluded_left_paths": ["left/left_001.png"], "excluded_right_paths": [], "excluded_pair_keys": []}
    (tmp_path / "calibration_pair_selection.json").write_text(json.dumps(selection))
    owner = CalibrationInputs()
    snapshot, _pairs, explicit = owner.snapshot(tmp_path, mode="pixhawk", pair_limit=0)
    assert not explicit
    assert snapshot.left_paths == tuple(files["left"][1:])
    assert snapshot.right_paths == tuple(files["right"])
    assert [key for key, _left, _right in snapshot.pair_paths] == ["002", "003"]
    identity = snapshot.snapshot_sha256
    files["right"][0].write_bytes(b"changed after admission")
    assert snapshot.snapshot_sha256 == identity
    with pytest.raises(ValueError):
        snapshot.verify_files()


def test_quality_owner_uses_result_truth_and_requires_explicit_warning():
    owner = CalibrationQuality()
    state, reasons = owner._calibration_quality_state(_result(quality="WARNING"), ({"state": "PASS"},))
    assert state == "WARNING"
    assert not owner.decide("Apply", state, reasons).allowed
    assert owner.decide("Apply", state, reasons, warning_confirmed=True).allowed
    assert not owner.decide("Apply", "FAIL", (), warning_confirmed=True).allowed
    unassessed = replace(_result(), calibration_quality=None)
    assert owner._calibration_quality_state(unassessed)[0] == "UNASSESSED"


def test_validation_owner_keeps_original_optical_translation_and_offline_reference():
    owner = CalibrationValidation()
    reference = _result(translation=(-3.0, 0.0, 0.0))
    candidate = _result(translation=(-4.0, 0.0, 0.0))
    row = owner._checkerboard_validation_result_data("Random", candidate, reference, 8)
    assert row["baseline_m"] == 4.0
    assert row["baseline_delta_mm"] == 1000.0
    assert row["rotation_delta_deg"] == 0.0
    assert candidate.extrinsics.translation_left_to_right_m == [-4.0, 0.0, 0.0]
    _text, state = owner.complete_random_validation(
        [{"index": 1, "result": candidate}, {"index": 2, "error": "invalid image"}],
        reference=reference, mode="checkerboard", pair_count=8, runs=2, subset_count=3, seed=7, quality=("PASS", ()),
    )
    assert state == "WARNING"
    assert owner.checkerboard_validation_summary["random_validation"]["failed_runs"] == 1
    assert owner.checkerboard_validation_summary["random_validation"]["successful_runs"] == 1
    _text, state = owner.complete_random_validation(
        [{"index": 1, "error": "invalid image"}], reference=reference,
        mode="checkerboard", pair_count=8, runs=1, subset_count=3, seed=7, quality=("PASS", ()),
    )
    assert state == "FAIL"


def test_profile_publication_preserves_exact_content_and_application_store(tmp_path):
    store = ActiveGeometryStore()
    owner = CalibrationProfiles(store.update)
    source = tmp_path / "source.json"
    source.write_bytes(b'{"fixture":1}\n')
    published = owner._publish_immutable_calibration_file(source, root=tmp_path, family="intrinsics", prefix="fixture")
    source.write_bytes(b'{"fixture":2}\n')
    assert published.read_bytes() == b'{"fixture":1}\n'
    assert owner._publish_immutable_calibration_file(source, root=tmp_path, family="intrinsics", prefix="fixture") != published
    result = _result(translation=(-1.0, 0.0, 0.0))
    applied = owner.apply_bundle(
        root=tmp_path, result=result, zoom_level=2, candidate_path="", active_intrinsics=None,
        active_path="", selected_profile=None, selected_path="", selected_hash="",
        use_selected_extrinsics=True, extrinsics_profile_override=None,
        quality_state="PASS", quality_reasons=(), quality_warning_confirmed=False,
    )
    assert Path(applied.candidate_path).is_file()
    assert Path(applied.selected_path).is_file()
    assert store.read_legacy("active_intrinsics").extrinsics.translation_left_to_right_m == [-1.0, 0.0, 0.0]
    assert store.read_legacy("_active_intrinsics_source_zoom_level") == 2
    assert store.read_legacy("_selected_extrinsics_profile_path") == applied.selected_path
    before = store.snapshot().revision
    with pytest.raises(ValueError):
        owner.apply_bundle(
            root=tmp_path, result=result, zoom_level=2, candidate_path="", active_intrinsics=None,
            active_path="", selected_profile=None, selected_path="", selected_hash="",
            use_selected_extrinsics=True, extrinsics_profile_override=None,
            quality_state="WARNING", quality_reasons=("coverage",), quality_warning_confirmed=False,
        )
    assert store.snapshot().revision == before


def test_auto_stop_retains_inflight_worker_outcome_without_counting_it():
    owner = CalibrationCapture(read_live_inputs=lambda: None, submit_capture=lambda **_request: True, present_status=lambda _text: None)
    try:
        owner._auto_capture_active = True
        owner._auto_capture_saved_pairs = 2
        owner._capture_origin = "auto"
        owner._auto_capture_inflight_pair_key = (11, 12)
        owner.stop()
        owner.complete_auto(("left", "right"), True)
        assert owner._auto_capture_saved_pairs == 2
        assert owner._auto_capture_inflight_pair_key is None
        assert owner._capture_origin == "manual"
    finally:
        owner.stop()
        owner.deleteLater()


def test_ftp_queue_keeps_head_until_admission_and_rejects_old_resume(monkeypatch, tmp_path):
    callbacks, calls = [], []
    accepted = [False]
    owner = CalibrationFtpTransactions()
    owner.actions = FtpQueueActions(
        capture=lambda **request: calls.append(request) or accepted[0], jobs=lambda: None,
        legacy_thread=lambda: None, shutting_down=lambda: False, completed=lambda _saved: None,
    )
    first, second = ("first", {"left": 1}, {}), ("second", {"left": 2}, {})
    owner._pending_ftp_oz_sync_frames = [first, second]
    owner.queue_root = tmp_path / "admitted-root"
    monkeypatch.setattr(QtCore.QTimer, "singleShot", lambda _ms, callback: callbacks.append(callback))
    try:
        owner._process_next_ftp_oz_sync_pair()
        assert owner._pending_ftp_oz_sync_frames == [first, second]
        accepted[0] = True
        callbacks.pop(0)()
        assert owner._pending_ftp_oz_sync_frames == [second]
        assert calls[-1]["capture_root"] == tmp_path / "admitted-root"
        old_resume = callbacks.pop(0)
        replacement = [("new", {}, {})]
        owner._pending_ftp_oz_sync_frames = replacement
        old_resume()
        assert len(calls) == 2
        assert owner._pending_ftp_oz_sync_frames is replacement
        owner.request_shutdown()
        owner._process_next_ftp_oz_sync_pair()
        assert len(calls) == 2
    finally:
        owner.deleteLater()


def test_oz_batch_mismatch_does_not_mutate_ledger_or_schedule_deletion():
    ledger = {"rounds": [{"capture_id": "one", "downloaded": False}], "consumed_remote_names": {"left": [], "right": []}}
    original = json.loads(json.dumps(ledger))
    batch, messages, status = OzCaptureLedger().assemble_sync_batch(
        ledger, {"left": {"items": [{"name": "left.jpg", "valid": True}]}, "right": {"items": []}}, ["one"],
    )
    assert batch is None
    assert messages and "failed" in status
    assert ledger == original


def test_runtime_state_aliases_actual_capture_ftp_and_validation_owners():
    jobs = CalibrationJobs()
    state = CalibrationRuntimeState(ActiveGeometryStore(), jobs)
    try:
        state._auto_capture_active = True
        assert state.capture_runtime._auto_capture_active
        assert "_auto_capture_active" not in vars(state)
        marker = object()
        state.calibration_wiris_ftp_thread = marker
        assert state.ftp_transactions.resource_inventory()["calibration_wiris_ftp_thread"] is marker
        assert "calibration_wiris_ftp_thread" not in vars(state)
        state.checkerboard_validation_summary = {"source": "fixture"}
        assert state.validation.checkerboard_validation_summary == {"source": "fixture"}
        assert "checkerboard_validation_summary" not in vars(state)
        assert state.calibration_jobs is jobs
    finally:
        state.calibration_wiris_ftp_thread = None
        state.capture_runtime.stop()
        state.capture_runtime.deleteLater()
        state.ftp_transactions.deleteLater()
        jobs.deleteLater()


class _HeldFtpWorker(QtCore.QObject):
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        import threading
        self.entered = threading.Event()
        self.release = threading.Event()

    @QtCore.pyqtSlot()
    def run(self):
        self.entered.set()
        try:
            if not self.release.wait(2.0):
                raise RuntimeError("fixture release timeout")
            self.completed.emit({"downloaded": "original-request"})
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


def _until(predicate):
    import time
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        QtCore.QCoreApplication.processEvents()
        QtCore.QThread.msleep(1)
    assert predicate()


def test_ftp_no_start_and_shutdown_before_launch_release_the_slot(monkeypatch):
    owner = CalibrationFtpTransactions()
    failures = []
    worker = _HeldFtpWorker()
    thread, _ = owner.prepare("ftp", worker, completed=lambda _result: None, failed=failures.append, retired=lambda: None)
    monkeypatch.setattr(QtCore.QThread, "start", lambda _thread: None)
    try:
        assert not owner.start("ftp", expected_thread=thread)
        assert failures and "did not start" in failures[-1]
        assert not owner.busy
        assert all(value is None for value in owner.resource_inventory().values())
        pending = _HeldFtpWorker()
        pending_thread, _ = owner.prepare("ftp", pending, completed=lambda _result: None, failed=failures.append, retired=lambda: None)
        assert owner.request_shutdown()
        assert not owner.start("ftp", expected_thread=pending_thread)
        assert "before launch" in failures[-1]
        assert not owner.busy
    finally:
        owner.request_shutdown()
        owner.deleteLater()


def test_ftp_start_after_launch_error_keeps_download_completion(monkeypatch):
    owner = CalibrationFtpTransactions()
    outcomes, failures = [], []
    worker = _HeldFtpWorker()
    thread, _ = owner.prepare("ftp", worker, completed=outcomes.append, failed=failures.append, retired=lambda: None)
    original_start = QtCore.QThread.start
    def start_then_raise(value):
        original_start(value)
        raise RuntimeError("fixture exception after actual launch")
    try:
        with monkeypatch.context() as patch:
            patch.setattr(QtCore.QThread, "start", start_then_raise)
            assert owner.start("ftp", expected_thread=thread)
        assert worker.entered.wait(1.0)
        assert owner.calibration_wiris_ftp_thread is thread
        assert not owner.request_shutdown()
        assert owner.busy
        worker.release.set()
        _until(lambda: not owner.busy)
        assert outcomes == [{"downloaded": "original-request"}]
        assert failures == []
        assert owner.calibration_wiris_ftp_thread is None
    finally:
        worker.release.set()
        if owner.busy:
            _until(lambda: not owner.busy)
        owner.deleteLater()


def test_ftp_nested_finished_cannot_release_callback_root_or_replacement(tmp_path):
    owner = CalibrationFtpTransactions()
    observed = []
    first_worker = _HeldFtpWorker()
    first = None
    def completed(_value):
        assert not owner.retire(first)
        assert owner.root_for("ftp") == tmp_path / "old"
        blocked_worker = _HeldFtpWorker()
        with pytest.raises(RuntimeError, match="already admitted"):
            owner.prepare("ftp", blocked_worker, completed=lambda _r: None, failed=lambda _e: None, retired=lambda: None, root=tmp_path / "replacement")
        observed.append(owner.root_for("ftp"))
    owner.prepare("ftp", first_worker, completed=completed, failed=lambda _error: None, retired=lambda: None, root=tmp_path / "old")
    first = owner._transactions["ftp"]
    try:
        owner.deliver(first, "completed", {})
        assert observed == [tmp_path / "old"]
        assert not owner.busy
        second_worker = _HeldFtpWorker()
        second_thread, _ = owner.prepare("ftp", second_worker, completed=lambda _r: observed.append("second"), failed=lambda _e: None, retired=lambda: None, root=tmp_path / "new")
        second = owner._transactions["ftp"]
        assert not owner.retire(first)
        owner.deliver(first, "completed", {})
        assert not owner.clear("ftp")
        assert owner.calibration_wiris_ftp_thread is second_thread
        assert owner.root_for("ftp") == tmp_path / "new"
        assert observed == [tmp_path / "old"]
        assert not owner.start("ftp", expected_thread=first.thread)
        owner.fail_start(second, "fixture cleanup before launch")
        assert not owner.busy
    finally:
        first_worker.deleteLater()
        owner.request_shutdown()
        owner.deleteLater()


def test_ftp_prepare_signal_failure_leaves_no_owned_resource():
    class MissingSignals(QtCore.QObject):
        def run(self):
            pass
    owner = CalibrationFtpTransactions()
    worker = MissingSignals()
    try:
        with pytest.raises(AttributeError):
            owner.prepare("ftp", worker, completed=lambda _r: None, failed=lambda _e: None, retired=lambda: None)
        assert not owner.busy
        assert all(value is None for value in owner.resource_inventory().values())
    finally:
        owner.deleteLater()


def test_capture_completion_keeps_admitted_manifest_root_after_ui_changes(monkeypatch, tmp_path):
    import Workstation.calibration_controller as module
    written = []
    controller = module.CalibrationWorkflowController.__new__(module.CalibrationWorkflowController)
    controller.panel = SimpleNamespace(
        calibration_status_label=SimpleNamespace(setText=lambda _text: None),
        _log=lambda _message: None,
        _calibration_session_root=tmp_path / "new-ui-root",
    )
    monkeypatch.setattr(module.CaptureDirectories, "write_manifest", lambda root, **metadata: written.append((root, metadata)))
    try:
        controller._apply_capture_job_result(
            {"stem": "001", "saved_sides": ("left", "right"), "status_parts": (), "stereo_pair_saved": True},
            "manual", tmp_path / "admitted-root",
        )
        assert written == [(tmp_path / "admitted-root", {"source": "manual"})]
        assert controller.panel._calibration_session_root == tmp_path / "new-ui-root"
    finally:
        controller._capture_runtime_owner().stop()
        controller._capture_runtime_owner().deleteLater()


def test_minimum_effective_pose_policy_is_source_backed_and_idempotent():
    source = _result(translation=(-1.0, 0.0, 0.0))
    source = replace(source, left=replace(source.left, image_count=8), right=replace(source.right, image_count=7))
    assessed = CalibrationQuality.assess_minimum_effective_poses(source, 8)
    assert assessed.below_minimum
    assert source.calibration_quality == {"state": "PASS", "reason_codes": []}
    assert assessed.result.calibration_quality["state"] == "FAIL"
    code = "configured_minimum_effective_poses_not_met"
    assert assessed.result.calibration_quality["reason_codes"] == [code]
    assert assessed.result.calibration_quality["reasons"] == [{
        "severity": "FAIL", "code": code,
        "message": "effective poses left=8, right=7; configured minimum=8",
    }]
    assert assessed.result.left is source.left
    assert assessed.result.right is source.right
    assert assessed.result.extrinsics is source.extrinsics
    repeated = CalibrationQuality.assess_minimum_effective_poses(assessed.result, 8)
    assert repeated.result.calibration_quality == assessed.result.calibration_quality
    equal = replace(source, right=replace(source.right, image_count=8))
    passed = CalibrationQuality.assess_minimum_effective_poses(equal, 8)
    assert not passed.below_minimum
    assert passed.result is equal


def test_attempt_save_policy_keeps_warning_and_unknown_zoom_out_of_approved_aliases(tmp_path):
    owner = CalibrationProfiles()
    official = tmp_path / "intrinsics_zoom_02.json"
    official.write_bytes(b"previous approved profile")
    full = _result(translation=(-1.0, 0.0, 0.0))
    warning = owner.save_intrinsics_attempt(
        full, root=tmp_path, zoom_level=2, quality_state="WARNING", quality_reasons=("coverage",),
        metadata={"zoom_level": 2, "capture_zoom_level": 2}, approved_output=None,
    )
    assert not warning.approved
    assert warning.output.parent == tmp_path / "attempts"
    payload = json.loads(warning.output.read_text())
    assert payload["metadata"]["eligible_for_application"] is False
    assert payload["metadata"]["profile_status"] == "attempt_only"
    assert "extrinsics" in payload  # Full validation evidence retains its R,t.
    assert official.read_bytes() == b"previous approved profile"
    with pytest.raises(ValueError, match="attempt-only"):
        owner.publish_saved_intrinsics(warning, root=tmp_path, latest_output=official)
    unknown = owner.save_intrinsics_attempt(
        full, root=tmp_path, zoom_level=None, quality_state="PASS", quality_reasons=(),
        metadata={"zoom_level": None}, approved_output=None,
    )
    assert not unknown.approved
    assert "zoom_unknown_pass" in unknown.output.name
    assert official.read_bytes() == b"previous approved profile"


def test_intrinsics_only_save_and_partial_alias_failure_keep_primary_evidence(tmp_path, monkeypatch):
    import Workstation.calibration_workflow.profiles as module
    owner = CalibrationProfiles()
    full = _result(translation=(-1.0, 0.0, 0.0))
    saved = owner.save_intrinsics_attempt(
        full, root=tmp_path, zoom_level=2, quality_state="PASS", quality_reasons=(),
        metadata={"zoom_level": 2, "capture_zoom_level": 2},
        approved_output=tmp_path / "intrinsics_zoom_02.json", intrinsics_only=True,
    )
    assert saved.approved
    assert "extrinsics" not in json.loads(saved.output.read_text())
    assert full.extrinsics is not None
    before = saved.output.read_bytes()
    def reject_alias(*_args, **_kwargs):
        raise OSError("fixture alias publication failure")
    monkeypatch.setattr(module, "save_intrinsics_json", reject_alias)
    with pytest.raises(OSError, match="alias publication"):
        owner.publish_saved_intrinsics(saved, root=tmp_path, latest_output=tmp_path / "intrinsics.json")
    assert saved.output.read_bytes() == before
    assert saved.metadata["profile_status"] == "approved"


def test_capture_directory_policy_preserves_reuse_auto_layout_and_session_scope(tmp_path):
    from Workstation.calibration_workflow.capture_request import CaptureDirectories
    root = tmp_path / "default"
    cache = {}
    request = dict(configured_root=root, default_root=root, hierarchy_enabled=False,
                   session_state="IDLE", session_id="", session_directory=None,
                   capture_origin="manual", session_roots=cache, stamp="capture-stamp")
    first = CaptureDirectories.resolve_session_root(**request)
    assert first.root == root / "capture-stamp"
    assert first.created and first.activate
    CaptureDirectories.write_manifest(first.root, source=first.source)
    reused = CaptureDirectories.resolve_session_root(**request)
    assert reused.root == first.root
    assert not reused.created
    auto = CaptureDirectories.resolve_session_root(**{**request, "capture_origin": "auto"})
    assert auto.root == root / "capture-stamp_01" / "calib_capture" / "AutoCapture"
    assert cache["default"] == first.root
    assert cache["auto"] == auto.root
    pre_session_root = tmp_path / "reusable"
    pre = CaptureDirectories.resolve_session_root(**{**request, "configured_root": pre_session_root, "hierarchy_enabled": True})
    assert pre.root == pre_session_root
    assert pre.pre_session and not pre.activate
    assert "default" in cache  # Pre-session resolution leaves the session cache intact.
    selected = tmp_path / "session-b"
    opened = CaptureDirectories.resolve_session_root(**{
        **request, "configured_root": pre_session_root, "hierarchy_enabled": True,
        "session_state": "OPEN", "session_id": "session-b", "session_directory": selected,
    })
    assert opened.root == selected / "calibration" / "capture-stamp"
    assert cache["session-b:default"] == opened.root
    with pytest.raises(RuntimeError, match="owner is unavailable"):
        CaptureDirectories.resolve_session_root(**{**request, "hierarchy_enabled": True, "session_state": "OPEN"})


def test_capture_zoom_provenance_reader_does_not_substitute_live_zoom(tmp_path):
    owner = CalibrationInputs()
    metadata = tmp_path / "capture_metadata"
    metadata.mkdir()
    paths = [tmp_path / "left" / "left_001.png", tmp_path / "right" / "right_001.png"]
    (metadata / "capture_001.json").write_text(json.dumps({"capture_left_zoom_level": 2, "capture_right_zoom_level": 2}))
    assert owner._capture_zoom_level_for_paths(tmp_path, paths) == (2, "capture-time zoom=2")
    (metadata / "capture_001.json").write_text(json.dumps({"capture_left_zoom_level": 2, "capture_right_zoom_level": 3}))
    level, reason = owner._capture_zoom_level_for_paths(tmp_path, paths)
    assert level is None
    assert "missing" in reason
