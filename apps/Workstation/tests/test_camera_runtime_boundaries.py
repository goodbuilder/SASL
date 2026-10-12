"""Real camera-control lifetimes and native-generation/Stop handoffs."""

from dataclasses import replace
from pathlib import Path
import ast
import threading
import time
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets

from Workstation.camera_feature.control import CameraControlRuntime, CONTROL_JOB_FIELDS, CONTROL_LANES
from Workstation.camera_feature.connection_view import CONNECTION_VIEW_FIELDS
from Workstation.camera_feature.candidates import CANDIDATE_VIEW_FIELDS
from Workstation.camera_feature.command_view import COMMAND_VIEW_FIELDS
from Workstation.camera_runtime.finalization import VideoFinalization, VideoFinalizationPorts
from Workstation.camera_runtime.recovery import StreamRecovery, RecoveryPorts, SideStreamState
from Workstation.camera_runtime.stop import CameraStopRuntime, CameraStopPorts
from Workstation.camera_runtime.rejoin import ReactivationPorts, RejoinRecordingSnapshot, RejoinTarget
from Workstation.camera_stream_controller import CameraStreamLifecycleController
from Workstation.tests.test_panel_composition import panels


class HeldControlWorker(QtCore.QObject):
    detected = QtCore.pyqtSignal(object)
    checked = QtCore.pyqtSignal(object)
    enabled = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, kind):
        super().__init__()
        self.kind = kind
        self.entered = threading.Event()
        self.release = threading.Event()
        self.release_finish = threading.Event()
        self.result_sent = threading.Event()

    @QtCore.pyqtSlot()
    def run(self):
        self.entered.set()
        if self.release.wait(5.0):
            getattr(self, CONTROL_LANES[self.kind][1]).emit(
                {"kind": self.kind, "thread": QtCore.QThread.currentThread()}
            )
            self.failed.emit("duplicate terminal outcome")
            self.result_sent.set()
            self.release_finish.wait(5.0)
        self.finished.emit()


def _until(predicate):
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert predicate()


def _launch(owner, kind, events, workers):
    def factory():
        worker = HeldControlWorker(kind)
        workers.append(worker)
        return worker

    return owner.launch(
        kind,
        worker_factory=factory,
        completed=lambda value: events.append(("completed", value, QtCore.QThread.currentThread())),
        failed=lambda error: events.append(("failed", error, QtCore.QThread.currentThread())),
        retired=lambda: events.append(("retired", kind, QtCore.QThread.currentThread())),
    )


def _close(owner, workers):
    jobs = list(owner.jobs.values())
    for worker in workers:
        worker.release.set()
        worker.release_finish.set()
    owner.request_shutdown()
    for job in jobs:
        try:
            job.thread.quit()
            assert job.thread.wait(3000)
        except RuntimeError:
            pass
    _until(lambda: not owner.jobs)
    owner.deleteLater()
    QtWidgets.QApplication.instance().processEvents()


@pytest.mark.parametrize("kind", tuple(CONTROL_LANES))
def test_control_lane_delivers_on_gui_and_retains_resources_until_finished(kind):
    owner, events, workers = CameraControlRuntime(), [], []
    try:
        assert _launch(owner, kind, events, workers)
        first = owner.jobs[kind]
        assert first.worker.entered.wait(1.0)
        assert not _launch(owner, kind, events, workers)
        first.worker.release.set()
        assert first.worker.result_sent.wait(1.0)
        _until(lambda: bool(events))
        assert [row[0] for row in events] == ["completed"]
        assert events[0][1]["thread"] is first.thread
        assert events[0][2] is owner.thread()
        owner.retire(first)
        assert owner.jobs[kind] is first
        assert first.thread in owner.resource_inventory().values()
        assert first.worker in owner.resource_inventory().values()
        first.worker.release_finish.set()
        _until(lambda: not owner.jobs)
        assert [row[0] for row in events] == ["completed", "retired"]
        assert _launch(owner, kind, events, workers)
        newer = owner.jobs[kind]
        before = list(events)
        owner.deliver(first, "completed", {"old": True})
        owner.retire(first)
        assert events == before
        assert owner.jobs[kind] is newer
        assert not owner.accepts_legacy_outcome(kind)
    finally:
        _close(owner, workers)


def test_control_shutdown_discards_queued_results_but_keeps_true_retirement():
    owner, events, workers = CameraControlRuntime(), [], []
    try:
        for kind in ("detect", "control"):
            assert _launch(owner, kind, events, workers)
            assert owner.jobs[kind].worker.entered.wait(1.0)
        first = owner.jobs["detect"]
        first.worker.release.set()
        assert first.worker.result_sent.wait(1.0)
        assert owner.request_shutdown() is False
        assert first.thread in owner.resource_inventory().values()
        assert not _launch(owner, "check", events, workers)
        for worker in workers:
            worker.release.set()
            worker.release_finish.set()
        _until(lambda: not owner.jobs)
        assert [row[0] for row in events] == ["retired", "retired"]
        assert all(owner.resource_inventory()[field] is None for field in CONTROL_JOB_FIELDS)
    finally:
        _close(owner, workers)


@pytest.mark.parametrize("failure", ["raise", "no-start", "already-started"])
def test_control_start_failure_cleanup_and_partial_start_acceptance(monkeypatch, failure):
    owner, events, workers = CameraControlRuntime(), [], []
    original_start = QtCore.QThread.start

    def start(thread):
        if failure == "already-started":
            original_start(thread)
            assert workers[-1].entered.wait(1.0)
        if failure != "no-start":
            raise RuntimeError("injected start failure")

    try:
        with monkeypatch.context() as patch:
            patch.setattr(QtCore.QThread, "start", start)
            accepted = _launch(owner, "enable", events, workers)
        if failure == "already-started":
            assert accepted is True
            assert "enable" in owner.jobs
            workers[-1].release.set()
            workers[-1].release_finish.set()
            _until(lambda: not owner.jobs)
            assert [row[0] for row in events] == ["completed", "retired"]
        else:
            assert accepted is False
            assert not owner.jobs
            assert [row[0] for row in events] == ["failed", "retired"]
        assert _launch(owner, "enable", events, workers)
        assert owner.jobs["enable"].worker.entered.wait(1.0)
    finally:
        _close(owner, workers)


def test_sequence_stop_does_not_reactivate_camera_from_late_control_result():
    owner, events, workers = CameraControlRuntime(), [], []
    try:
        owner._rtsp_reactivation_side = "left"
        assert _launch(owner, "enable", events, workers)
        job = owner.jobs["enable"]
        assert job.worker.entered.wait(1.0)
        owner.cancel_sequence()
        assert owner._rtsp_reactivation_side is None
        assert owner.jobs["enable"] is job
        job.worker.release.set()
        job.worker.release_finish.set()
        _until(lambda: not owner.jobs)
        assert [row[0] for row in events] == ["retired"]
    finally:
        _close(owner, workers)


def _stop_all_without_device_effects(panel, monkeypatch):
    monkeypatch.setattr(panel, "_stop_rtcp_clock_observer", lambda: None)
    monkeypatch.setattr(panel, "stop_yolo_streams", lambda **kwargs: True)
    monkeypatch.setattr(panel, "_stop_pixhawk_operation_workers", lambda **kwargs: True)
    monkeypatch.setattr(panel, "_stop_calibration_preview_worker", lambda **kwargs: True)
    panel.mavlink_pixhawk = None
    return CameraStreamLifecycleController(panel)


@pytest.mark.parametrize("closing", [False, True], ids=["restartable-stop", "close"])
def test_stop_all_keeps_control_handles_until_queued_gui_finished(panels, monkeypatch, closing):
    panel = panels()
    controller = _stop_all_without_device_effects(panel, monkeypatch)
    monkeypatch.setattr(panel, "_shutdown_in_progress", closing)
    owner = panel.application.features["camera"].state.control_jobs
    events, workers = [], []
    try:
        assert _launch(owner, "check", events, workers)
        job = owner.jobs["check"]
        assert job.worker.entered.wait(1.0)
        job.worker.release.set()
        job.worker.release_finish.set()
        job.thread.quit()
        assert job.thread.wait(1000)
        assert events == []  # Native exit precedes GUI completion/retirement.
        assert controller.stop_all(wait=False) is False
        assert panel.rtsp_check_thread is job.thread
        assert panel.rtsp_check_worker is job.worker
        assert owner.jobs["check"] is job
        _until(lambda: not owner.jobs)
        assert controller.stop_all(wait=False) is True
        assert owner.closing is closing
        if not closing:
            assert _launch(owner, "check", events, workers)
            assert owner.jobs["check"].worker.entered.wait(1.0)
    finally:
        _close(owner, workers)


class HeldCaptureResult(QtCore.QObject):
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, fail=False):
        super().__init__()
        self.fail = fail
        self.entered = threading.Event()
        self.release = threading.Event()

    @QtCore.pyqtSlot()
    def run(self):
        self.entered.set()
        if self.release.wait(5.0):
            if self.fail:
                self.failed.emit("fixture capture failure")
            else:
                self.completed.emit({"saved": True})
        self.finished.emit()


@pytest.mark.parametrize("kind", ["calibration", "optical"])
def test_stop_all_close_waits_for_capture_owner_outcome_and_retirement(panels, monkeypatch, tmp_path, kind):
    panel = panels()
    controller = _stop_all_without_device_effects(panel, monkeypatch)
    monkeypatch.setattr(panel, "_shutdown_in_progress", True)
    results = []
    worker = HeldCaptureResult(fail=kind == "optical")
    if kind == "calibration":
        owner = panel.application.features["calibration"].state.ftp_transactions
        thread, _ = owner.prepare(
            "ftp",
            worker,
            root=tmp_path,
            completed=lambda result: results.append(result),
            failed=lambda error: results.append(error),
            retired=lambda: results.append("retired"),
        )
        owner.start("ftp", expected_thread=thread)
        field_name = "calibration_wiris_ftp_thread"
    else:
        from Workstation.optical_pose.capture import FrozenCaptureInputs

        owner = panel._optical_pose_controller().capture_owner
        monkeypatch.setattr(owner, "_log", results.append)
        inputs = FrozenCaptureInputs("capture-session", tmp_path, 10.0, None, None, {})
        job = owner.start_ftp(
            mode="hr",
            capture_id="stop-boundary",
            inputs=inputs,
            payload={},
            worker_factory=lambda: worker,
        )
        thread = job.thread
        field_name = "optical_pose_wiris_ftp_thread"
    try:
        assert worker.entered.wait(1.0)
        results.clear()
        worker.release.set()
        thread.quit()
        assert thread.wait(1000)
        assert controller.stop_all(wait=False) is False
        assert getattr(panel, field_name) is thread
        assert owner.busy
        assert results == []
        _until(lambda: not owner.busy)
        assert getattr(panel, field_name) is None
        assert controller.stop_all(wait=False) is True
        if kind == "calibration":
            assert results == [{"saved": True}, "retired"]
        else:
            assert any("fixture capture failure" in message for message in results)
    finally:
        worker.release.set()
        owner.request_shutdown()
        try:
            thread.quit()
            assert thread.wait(3000)
        except RuntimeError:
            pass
        _until(lambda: not owner.busy)


def test_real_panel_widgets_and_candidate_state_have_single_camera_owners(panels):
    panel = panels()
    camera = panel.application.features["camera"]
    for component, names in (
        (camera.views.connection, CONNECTION_VIEW_FIELDS),
        (camera.views.candidates, CANDIDATE_VIEW_FIELDS),
        (camera.views.commands, COMMAND_VIEW_FIELDS),
    ):
        for name in names:
            assert getattr(panel, name) is getattr(component, name), name
            assert name not in vars(camera.views), name
        assert not {"panel", "backend", "workspace"} & vars(component).keys()
    assert panel._camera_candidate_rows is camera.state.candidates.rows
    panel.left_url_edit.setText("rtsp://127.0.0.1/manual-left")
    panel.right_url_edit.setText("rtsp://127.0.0.1/manual-right")
    panel._apply_camera_detection({"mode": "rtsp", "rtsp": [], "control": []})
    assert panel.left_url_edit.text() == "rtsp://127.0.0.1/manual-left"
    assert panel.right_url_edit.text() == "rtsp://127.0.0.1/manual-right"
    assert camera.state.candidates.rows == []
    assert panel._rtsp_reactivation_stop_timer.parent() is panel


def test_frame_recovery_requires_current_newer_generation_and_preserves_source_metadata():
    recovery = StreamRecovery()
    recovery.note_failure("left", 5)
    recovery.note_failure("right", 20)
    recovery._measurement_camera_rejoin_expected_generations["left"] = 6
    assert recovery.admit_frame("left", {"stream_generation": 5}, current_generation=6) is None
    assert recovery.admit_frame("left", {"stream_generation": 5}, current_generation=5) is None
    source = {
        "stream_generation": 6,
        "frame_id": 17,
        "native_width": 1920,
        "timestamp_s": 1.25,
        "camera_rtcp_timestamp_ns": 42_000_000_123,
    }
    admitted = recovery.admit_frame("left", source, current_generation=6)
    assert admitted.integrity_recovered and admitted.manual_rejoin_ready
    assert admitted.metadata == source and admitted.metadata is not source
    assert recovery._camera_integrity_unavailable_sides == {"right"}
    assert recovery._camera_integrity_failed_generations == {"right": 20}
    assert "left" not in recovery._measurement_camera_rejoin_expected_generations


def test_one_side_integrity_failure_preserves_peer_and_observation_without_recording():
    recovery, calls = StreamRecovery(), []
    states = {side: SideStreamState(5, True, True) for side in ("left", "right")}

    def stop(side):
        states[side] = replace(states[side], generation=6, active=False)
        calls.append(("stop", side))

    ports = RecoveryPorts(
        read_side=states.__getitem__,
        stop_side=stop,
        clear_side=lambda side: calls.append(("clear", side)),
        update_availability=lambda active, detection: calls.append(("active", active, detection)),
        publish_source=lambda *args: calls.append(args),
        show_unavailable=lambda *args: None,
        show_recovered=lambda *args: None,
        recording_side_active=lambda side: False,
        refresh_view=lambda: None,
    )
    recovery.unavailable("left", "integrity failed", ports)
    recovery.unavailable("left", "duplicate", ports)
    assert ("active", ("right",), True) in calls
    assert calls.count(("stop", "left")) == 1
    assert states["right"] == SideStreamState(5, True, True)
    states["left"] = SideStreamState(7, True, True)
    assert recovery.admit_frame("left", {"stream_generation": 7}, current_generation=7).integrity_recovered
    recovery.recovered("left", ports)
    assert ("active", ("left", "right"), True) in calls
    assert any(row[:2] == ("camera.left", "LIVE") for row in calls)
    assert not any(row[:2] == ("recording.left", "LIVE") for row in calls)


def _finalization_ports(published, *, append=None):
    return VideoFinalizationPorts(
        read_finalization=lambda side, path: {},
        consume_finalization=lambda side, path: None,
        publish_evidence=lambda side, artifact: published.append((side, artifact)),
        append=append,
    )


def test_presence_and_persistence_failure_do_not_claim_integrity_or_block_stop(tmp_path):
    path = tmp_path / "segment.mp4"
    path.write_bytes(b"file presence is not proof of decodability")
    owner, published = VideoFinalization(), []
    owner.begin({"left": path}, backend="gstreamer", runtime_generation="run-7")

    def unavailable(*args, **kwargs):
        raise OSError("writer unavailable")

    assert owner.record_presence(_finalization_ports(published, append=unavailable)) is True
    assert owner.pending is None
    artifact = published[0][1]
    assert artifact["runtime_generation"] == "run-7"
    assert artifact["present"] is True
    assert artifact["integrity_validation_status"] == "NOT_RUN"
    assert artifact["recording_usable"] is False


def test_old_finalization_cannot_clear_or_publish_over_reentrant_new_pending(tmp_path):
    owner, published, appended = VideoFinalization(), [], []
    owner.begin({"left": tmp_path / "old.mp4"}, backend="gstreamer", runtime_generation="old")
    newer = {"paths": {"right": tmp_path / "new.mp4"}, "runtime_generation": "new"}

    def replace_pending(kind, artifact, **kwargs):
        appended.append(artifact)
        owner.pending = newer

    assert owner.record_presence(_finalization_ports(published, append=replace_pending)) is False
    assert owner.pending is newer
    assert published == []
    assert appended[0]["runtime_generation"] == "old"


@pytest.mark.parametrize("stopped,wait", [(True, False), (False, False), (False, True)])
def test_stop_order_waits_for_actual_media_and_keeps_recording_completion_separate(stopped, wait):
    order = []
    finalization = VideoFinalization()
    timers = SimpleNamespace(cancel_stop=lambda: order.append("cancel-watchdog"))
    owner = CameraStopRuntime(finalization, timers)
    ports = CameraStopPorts(
        stop_panes=lambda **kwargs: order.append(("panes", kwargs)) or stopped,
        schedule_watchdog=lambda: order.append("watchdog"),
        finish_recording=lambda clock: order.append(("recording", clock)),
        finish_run_if_stopping=lambda: order.append("run"),
        publish_stopped=lambda: order.append("display"),
        log=lambda text: order.append("log"),
        finalization=_finalization_ports([]),
    )
    assert owner.stop(wait=wait, finalize_session=True, clock_snapshot="captured", ports=ports) is stopped
    assert order[0] == ("panes", {"clear_previews": True, "wait": wait})
    if stopped:
        assert order[1:] == [("recording", "captured"), "cancel-watchdog", "run", "display"]
    elif wait:
        assert order[1:] == ["log"]
    else:
        assert order[1:] == ["watchdog", ("recording", "captured"), "display"]


def test_reentrant_start_invalidates_old_stop_cleanup():
    order = []
    owner = CameraStopRuntime(VideoFinalization(), SimpleNamespace(cancel_stop=lambda: None))

    def new_start(clock):
        owner.cancel()
        order.append("new-start")

    ports = CameraStopPorts(
        stop_panes=lambda **kwargs: True,
        schedule_watchdog=lambda: None,
        finish_recording=new_start,
        finish_run_if_stopping=lambda: order.append("old-run"),
        publish_stopped=lambda: order.append("old-display"),
        log=lambda text: None,
        finalization=_finalization_ports([]),
    )
    assert owner.stop(wait=False, finalize_session=True, clock_snapshot=None, ports=ports) is False
    assert order == ["new-start"]


@pytest.fixture
def rejoin_case(tmp_path):
    host = QtWidgets.QWidget()
    control = CameraControlRuntime(host, thread_parent=host)
    control._rtsp_reactivation_side = "right"
    control._rtsp_reactivation_enable_detection = True
    control._rtsp_reactivation_generation = 3
    recovery = StreamRecovery()
    recovery.note_failure("right", 6)
    case = {
        "control": control,
        "recovery": recovery,
        "calls": [],
        "recording": False,
        "run_state": "RUNNING",
        "paths": {},
        "decoder": "auto",
        "url": "rtsp://127.0.0.1/right",
        "start_ok": True,
        "sides": {"left": SideStreamState(4, True, True), "right": SideStreamState(7, False, False)},
    }

    def start_recording():
        case["calls"].append("start-recording")
        case["recording"] = True
        case["decoder"] = "software"
        case["paths"]["right"] = tmp_path / "initial.mp4"
        return True

    def allocate(side):
        case["calls"].append(("allocate", side))
        return tmp_path / "allocated.mp4"

    def install(side, path):
        case["calls"].append(("install", side, path))
        case["paths"][side] = path

    def start(url, **kwargs):
        case["calls"].append(("start", url, kwargs))
        if case["start_ok"]:
            case["sides"]["right"] = SideStreamState(8, True, True)
        return case["start_ok"]

    case["ports"] = ReactivationPorts(
        capture_target=lambda side: RejoinTarget(case["url"], start, lambda: case["sides"][side]),
        prepare_preview=lambda side, enabled: case["calls"].append(("prepare", side, enabled)),
        read_recording=lambda: RejoinRecordingSnapshot(case["recording"], case["run_state"]),
        start_recording=start_recording,
        current_segment=lambda side: case["paths"].get(side),
        allocate_segment=allocate,
        install_segment=install,
        stream_options=lambda side: {"decoder": case["decoder"]},
        recording_options=lambda side: {"record_video_path": str(case["paths"][side])} if side in case["paths"] else {},
        read_side=lambda side: case["sides"][side],
        publish_source=lambda *args: case["calls"].append(("source", *args)),
        set_availability=lambda *args: case["calls"].append(("availability", *args)),
        report=lambda *args: case["calls"].append(("report", *args)),
        refresh=lambda controls: case["calls"].append(("refresh", controls)),
        log=lambda message: case["calls"].append(("log", message)),
    )
    yield case
    control.request_shutdown()
    host.deleteLater()
    QtWidgets.QApplication.instance().sendPostedEvents(host, QtCore.QEvent.DeferredDelete)
    QtWidgets.QApplication.instance().processEvents()


@pytest.mark.parametrize("mode", ["initial", "existing", "unavailable"])
def test_reactivation_reuses_or_allocates_segment_and_preserves_observation_on_recording_failure(rejoin_case, mode):
    case = rejoin_case
    ports = case["ports"]
    if mode == "existing":
        case["recording"] = True
    elif mode == "unavailable":

        def fail_recording():
            raise OSError("storage unavailable")

        ports = replace(ports, start_recording=fail_recording)
    case["control"].finish_reactivation(True, "frame verified", recovery=case["recovery"], ports=ports)
    starts = [row for row in case["calls"] if isinstance(row, tuple) and row[0] == "start"]
    assert len(starts) == 1
    assert starts[0][1] == "rtsp://127.0.0.1/right"
    assert starts[0][2]["enable_detection"] is True
    if mode == "initial":
        assert "start-recording" in case["calls"]
        assert ("allocate", "right") not in case["calls"]
        assert starts[0][2]["record_video_path"].endswith("initial.mp4")
        assert starts[0][2]["decoder"] == "software"  # sampled after recording preparation
    elif mode == "existing":
        assert "start-recording" not in case["calls"]
        assert ("allocate", "right") in case["calls"]
        assert starts[0][2]["record_video_path"].endswith("allocated.mp4")
    else:
        assert "record_video_path" not in starts[0][2]
        assert ("source", "recording.right", "UNAVAILABLE", "recording could not join recovered camera") in case[
            "calls"
        ]
    assert case["recovery"]._measurement_camera_rejoin_expected_generations == {"right": 8}
    assert case["recovery"]._camera_integrity_unavailable_sides == {"right"}
    assert ("availability", ("left",), True) in case["calls"]
    assert ("report", "right", "pending", "", True) in case["calls"]
    assert case["control"]._rtsp_reactivation_side is None
    assert case["control"]._rtsp_reactivation_enable_detection is False


@pytest.mark.parametrize("step", ["start_recording", "allocate_segment", "start_stream"])
def test_stop_during_reactivation_effect_prevents_old_followup(rejoin_case, step):
    case = rejoin_case
    ports = case["ports"]
    if step == "start_stream":
        case["run_state"] = "IDLE"
        target = ports.capture_target("right")

        def start(url, **kwargs):
            result = target.start_stream(url, **kwargs)
            case["control"].invalidate_reactivation()
            return result

        ports = replace(ports, capture_target=lambda side: RejoinTarget(target.url, start, target.read_state))
    else:
        case["recording"] = step == "allocate_segment"
        original = getattr(ports, step)

        def stop_inside(*args):
            value = original(*args)
            case["control"].invalidate_reactivation()
            return value

        ports = replace(ports, **{step: stop_inside})
    case["control"].finish_reactivation(True, "", recovery=case["recovery"], ports=ports)
    assert case["recovery"]._measurement_camera_rejoin_expected_generations == {}
    assert not any(isinstance(row, tuple) and row[0] in {"install", "report", "availability"} for row in case["calls"])
    starts = [row for row in case["calls"] if isinstance(row, tuple) and row[0] == "start"]
    assert len(starts) == int(step == "start_stream")


def test_reactivation_failure_does_not_read_or_restart_a_media_target(rejoin_case):
    case = rejoin_case

    def forbidden(side):
        raise AssertionError("failed reactivation must not prepare a preview")

    ports = replace(case["ports"], capture_target=forbidden)
    case["control"].finish_reactivation(False, "no frame", recovery=case["recovery"], ports=ports)
    assert ("report", "right", "failed", "no frame", False) in case["calls"]
    assert ("source", "camera.right", "UNAVAILABLE", "no frame") in case["calls"]
    assert ("source", "recording.right", "UNAVAILABLE", "no frame") in case["calls"]
    assert case["recovery"]._measurement_camera_rejoin_expected_generations == {}


def test_new_camera_owners_have_no_panel_or_controller_back_reference():
    root = Path(__file__).resolve().parents[1]
    paths = [
        root / "camera_feature" / name
        for name in (
            "control.py",
            "connection_view.py",
            "candidates.py",
            "command_view.py",
        )
    ]
    paths += [root / "camera_runtime" / name for name in ("recovery.py", "finalization.py", "stop.py", "rejoin.py")]
    for path in paths:
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
                assert node.attr not in {"panel", "backend", "workspace", "controller"}, (path, node.lineno)
            if isinstance(node, ast.ImportFrom):
                assert not (node.module or "").endswith(("panel_camera", "camera_stream_controller")), path
