"""GNSS ownership, immutable evidence and real Qt receiver retirement contracts."""

import ast
import threading
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets
from Workstation.gnss_runtime.base import BASE_FIELDS, BaseReceiverEffects, BaseReceiverRuntime, BaseReceiverState
from Workstation.gnss_runtime.corrections import (
    CORRECTION_FIELDS,
    CorrectionEvents,
    CorrectionRuntime,
    CorrectionRuntimeState,
    _PipelineRelay,
)
from Workstation.gnss_runtime.evidence import EVIDENCE_FIELDS, EvidenceCapture, GnssEvidenceState, RtcmEvidenceRuntime
from Workstation.gnss_runtime.operation import (
    OPERATION_FIELDS,
    GnssOperationRuntime,
    GnssOperationRuntimeState,
    NetworkAdmissionInputs,
)
from Workstation.gnss_runtime_workers import GnssBaseSerialWorker
from Workstation.tests.test_panel_composition import panels  # noqa: F401
from Workstation.workstation_core.gnss_operation import (
    GgaPolicy,
    GnssOperationConfig,
    GnssOperationMode,
    GnssOperationState,
    HeightType,
)
from Workstation.workstation_core.gnss_profile import GnssCoordinateSample
from Workstation.workstation_core.rtcm_pipeline import RtcmRouteConfig


def _until(predicate):
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert predicate()


def _operation():
    events = []
    owner = GnssOperationRuntime(
        GnssOperationRuntimeState(), transitioned=lambda *args: events.append(args), health=lambda *args: None
    )
    return owner, events


class _ControlledSerial(GnssBaseSerialWorker):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.entered = threading.Event()
        self.release = threading.Event()

    @QtCore.pyqtSlot()
    def run(self):
        self.position_changed.emit("queued-position")
        self.entered.set()
        self.release.wait(3.0)
        self.finished.emit()

    def request_stop(self):
        self.release.set()


def _base(*, factory=_ControlledSerial):
    operation, _ = _operation()
    events = []
    owner = BaseReceiverRuntime(
        None,
        BaseReceiverState(),
        operation,
        config=lambda: operation.state.gnss_operation_manager.config,
        network_connected=lambda: False,
        effects=BaseReceiverEffects(
            lambda *args: None, lambda *args: None, lambda sample: events.append(("record", sample)), lambda: None
        ),
        serial_factory=factory,
        detect_factory=lambda **kwargs: None,
        serial_events={"position_changed": lambda sample: events.append((sample, QtCore.QThread.currentThread()))},
        detect_events={},
        retired=lambda kind: events.append(("retired", kind)),
    )
    return owner, events


def _close(owner):
    owner.stop()
    _until(lambda: owner.bindings["serial"] is None)
    owner.deleteLater()


def test_real_feature_aliases_refer_to_one_runtime_state_and_inventory(panels):  # noqa: F811
    panel = panels()
    feature = panel.application.features["gnss"]
    controller = panel._gnss_runtime_controller_owner
    owners = {
        "base": controller._base_runtime(),
        "operation": controller._operation_runtime(),
        "corrections": controller._correction_runtime(),
        "evidence": controller._evidence_runtime(),
    }
    for name, fields in (
        ("base", BASE_FIELDS),
        ("operation", OPERATION_FIELDS),
        ("corrections", CORRECTION_FIELDS),
        ("evidence", EVIDENCE_FIELDS),
    ):
        assert owners[name].state is getattr(feature.state, name)
        for field in fields:
            assert field not in vars(panel)
            assert field not in vars(feature.state)
            assert getattr(panel, field) is getattr(owners[name].state, field)
    assert "gnss_base_serial_thread" in feature.resource_inventory()
    assert "rtcm_pipeline_thread" in controller.resource_inventory()
    assert not any(
        value is not None for key, value in controller.resource_inventory().items() if key.endswith("_thread")
    )


def test_base_stop_rejects_queued_old_position_and_retains_binding_until_finished():
    owner, events = _base()
    try:
        assert owner.start_serial(
            port="fixture", baud=115200, height_type=HeightType.ELLIPSOID, wire=lambda worker: None
        )
        old = owner.bindings["serial"]
        assert old.worker.entered.wait(1.0)
        assert owner.stop() is False
        assert owner.resource_inventory()["gnss_base_owned_serial_thread"] is old.thread
        _until(lambda: owner.bindings["serial"] is None)
        assert events == [("retired", "serial")]
        assert owner.start_serial(port="new", baud=115200, height_type=HeightType.MSL, wire=lambda worker: None)
        current = owner.bindings["serial"]
        owner.deliver(old, "serial", "position_changed", "old")
        owner.retire(old, "serial")
        assert owner.state.gnss_base_serial_worker is current.worker
        _until(lambda: len(events) == 2)
        assert events[1] == ("queued-position", owner.thread())
    finally:
        _close(owner)


def test_base_wire_failure_releases_reservation_and_allows_retry():
    owner, events = _base()
    try:

        def fail_wire(worker):
            raise RuntimeError("wire unavailable")

        with pytest.raises(RuntimeError, match="wire unavailable"):
            owner.start_serial(port="fixture", baud=115200, height_type=HeightType.ELLIPSOID, wire=fail_wire)
        assert owner.bindings["serial"] is None
        assert owner.state.gnss_base_serial_thread is None
        assert owner.state.gnss_base_serial_worker is None
        assert events == [("retired", "serial")]
        assert owner.start_serial(
            port="fixture", baud=115200, height_type=HeightType.ELLIPSOID, wire=lambda worker: None
        )
    finally:
        _close(owner)


def test_survey_requires_current_ack_active_observation_and_requested_limits():
    owner, _ = _base()
    commands = []
    owner.state.gnss_base_serial_worker = SimpleNamespace(send_bytes=lambda *args: commands.append(args))
    config = GnssOperationConfig()
    try:
        assert owner.start_survey(config, duration_s=60, accuracy_m=0.5)
        generation = owner.state._gnss_survey_generation
        completed = {
            "valid": True,
            "active": False,
            "duration_s": 60,
            "mean_accuracy_m": 0.4,
            "mean_x_m": 123.0,
            "mean_y_m": -456.0,
            "mean_z_m": 789.0,
        }
        assert owner.observe_survey(completed) is False
        assert owner.operation.state.gnss_operation_manager.state == GnssOperationState.BASE_SURVEYING
        owner.state._gnss_survey_command_sent_generation = generation
        # Independent UBX ACK payload: CFG (0x06), TMODE3 (0x71).
        owner.observe_ubx(0x05, 0x01, b"\x06\x71")
        assert owner.state._gnss_survey_ack_generation == generation
        assert owner.observe_survey(completed) is False
        owner.observe_survey({**completed, "active": True, "valid": False})
        assert owner.observe_survey({**completed, "mean_accuracy_m": 0.6}) is False
        assert owner.observe_survey(completed) is True
        assert owner.operation.state.gnss_operation_manager.state == GnssOperationState.LOCAL_RTCM_ACTIVE
        assert owner.state.latest_gnss_base_survey_in["mean_y_m"] == -456.0
    finally:
        owner.state.gnss_base_serial_worker = None
        owner.deleteLater()


def test_operation_routes_capture_config_and_require_selected_live_drone_endpoint():
    owner, _ = _operation()
    config = GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE)
    config = replace(
        config, routing=replace(config.routing, left_enabled=True, right_enabled=False, drone_enabled=True)
    )
    inputs = {
        "selected_target": "right",
        "source": "ntrip",
        "include_rover_targets": True,
        "drone_source": "ntrip",
        "drone_forward": True,
        "drone_connected": True,
    }
    assert owner.target_sides(config, **inputs) == ("left", "drone")
    assert owner.target_sides(config, **{**inputs, "drone_connected": False}) == ("left",)
    assert owner.target_sides(config, **{**inputs, "source": "base"}) == ("left",)
    route = owner.route(config, ntrip_targets=("drone",), base_targets=("left",), base_serial_available=False)
    assert route.forward_ntrip_to_base is True
    assert route.base_serial_available is False


def test_evidence_context_is_immutable_and_closed_package_cannot_reactivate(tmp_path):
    capture = [
        EvidenceCapture(True, root=tmp_path, state="COLLECTING", session_id="original", measurement_id="m1", revision=3)
    ]
    owner = RtcmEvidenceRuntime(GnssEvidenceState(), capture=lambda: capture[0], activated=lambda metadata: None)
    frozen = owner.context()
    capture[0] = replace(capture[0], session_id="next", measurement_id="m2", revision=4)
    assert frozen.context_token == "original:m1:r0003"
    assert frozen.session_root == tmp_path
    capture[0] = replace(capture[0], drain_requested_token="next:m2:r0004")
    assert owner.context().state == "IDLE"
    received = []
    pipeline = SimpleNamespace(close_evidence_context_and_drain=lambda payload: received.append(payload) or True)
    assert owner.close_and_drain(pipeline, "original:m1:r0003")
    assert received == [{"context_token": "original:m1:r0003", "session_id": "next", "session_root": str(tmp_path)}]


def test_base_position_evidence_preserves_height_datum_and_receive_clocks():
    owner = RtcmEvidenceRuntime(
        GnssEvidenceState(), capture=lambda: EvidenceCapture(False), activated=lambda metadata: None
    )
    calls, external = [], []
    sample = GnssCoordinateSample(
        latitude_deg=-12.5,
        longitude_deg=135.25,
        ellipsoid_height_m=68.75,
        msl_altitude_m=42.25,
        height_type=HeightType.ELLIPSOID,
        timestamp_s=19.0,
        fixed=True,
        device_id="base",
    )

    def timeline(**payload):
        calls.append(payload)
        return dict(payload)

    owner.record_base_position(sample, operation_state="CONFIGURED", timeline=timeline, append_external=external.append)
    row = owner.state.gnss_base_recording_rows[0]
    assert (row["latitude_deg"], row["longitude_deg"], row["ellipsoid_height_m"], row["msl_altitude_m"]) == (
        -12.5,
        135.25,
        68.75,
        42.25,
    )
    assert row["height_type"] == "ELLIPSOID"
    assert calls[0]["sample_monotonic_s"] == 19.0
    assert calls[0]["source_utc_ns"] is None
    assert calls[0]["source_utc_method"] == "unavailable"
    assert calls[0]["host_receive_utc_ns"] > calls[0]["host_receive_monotonic_ns"]
    assert external == [row]
    owner.state.gnss_base_recording_rows.clear()


def test_old_pipeline_snapshot_cannot_replace_live_stats_but_evidence_drain_survives():
    observed = []
    state = CorrectionRuntimeState()
    evidence = RtcmEvidenceRuntime(
        GnssEvidenceState(), capture=lambda: EvidenceCapture(False), activated=lambda metadata: None
    )
    events = CorrectionEvents(
        lambda value: observed.append(("snapshot", value)),
        lambda value: None,
        lambda value: observed.append(("drained", value)),
        lambda value: None,
        lambda: None,
        lambda: None,
    )
    owner = CorrectionRuntime(
        None,
        state,
        route=RtcmRouteConfig,
        evidence=evidence,
        base_worker=lambda: None,
        subscriber=lambda: None,
        pipeline_factory=lambda **kwargs: None,
        injection_factory=lambda subscriber: None,
        events=events,
    )
    old_thread, old_worker = object(), object()
    relay = _PipelineRelay(owner, old_thread, old_worker)
    state.rtcm_pipeline_thread, state.rtcm_pipeline_worker = object(), object()
    relay.snapshot({"frames": 100})
    relay.evidence_drained({"context_token": "old-package"})
    assert observed == [("drained", {"context_token": "old-package"})]
    state.rtcm_pipeline_thread, state.rtcm_pipeline_worker = old_thread, old_worker
    relay.snapshot({"frames": 101})
    assert observed[-1] == ("snapshot", {"frames": 101})
    owner.deleteLater()


def _assert_leaf_boundary(tree):
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            assert node.attr != "panel", node.lineno
        if isinstance(node, ast.ImportFrom):
            assert node.module not in {
                "Workstation.gnss_runtime_controller",
                "Workstation.panel_gnss",
                "Workstation.panel_application",
            }


def test_gnss_leaf_owners_never_read_panel_or_import_outer_facade():
    folder = Path(__file__).resolve().parents[1] / "gnss_runtime"
    for filename in ("base.py", "operation.py", "corrections.py", "evidence.py"):
        _assert_leaf_boundary(ast.parse((folder / filename).read_text()))
    for forbidden in ("self.panel.latest_gps", "from Workstation.gnss_runtime_controller import GnssRuntimeController"):
        with pytest.raises(AssertionError):
            _assert_leaf_boundary(ast.parse(forbidden))


def test_missing_required_operation_adapter_fails_at_composition():
    with pytest.raises(TypeError, match="requires"):
        GnssOperationRuntime(GnssOperationRuntimeState(), transitioned=None, health=lambda *args: None)


def test_gnss_evidence_follows_recorder_row_replacement_and_preserves_primary_owner(panels):  # noqa: F811
    from Workstation.recording.binding import recording_state_for

    panel = panels()
    recording = recording_state_for(panel)
    rows = recording.state_for_field("gnss_base_recording_rows")
    primary = recording.ledger
    owner = panel._gnss_runtime_controller_owner._evidence_runtime()
    original = rows.gnss_base_recording_rows
    assert owner.state.gnss_base_recording_rows is original
    assert "_standalone_rows" not in vars(owner.state)

    # Session admission/recovery replaces this Recorder field. A GNSS owner
    # must follow that replacement rather than retain the old spool instance.
    replacement = []
    rows.gnss_base_recording_rows = replacement
    sample = GnssCoordinateSample(
        35.25, 129.5, 65.0, msl_altitude_m=40.0, height_type=HeightType.ELLIPSOID, timestamp_s=123.0, fixed=True
    )
    external = []
    owner.record_base_position(
        sample, operation_state="READY", timeline=lambda **row: row, append_external=external.append
    )
    assert panel.gnss_base_recording_rows is replacement
    assert owner.state.gnss_base_recording_rows is replacement
    assert len(replacement) == 1
    assert len(original) == 0
    assert replacement[0]["ellipsoid_height_m"] == 65.0
    assert replacement[0]["msl_altitude_m"] == 40.0
    assert external == replacement

    through_compatibility = []
    owner.state.gnss_base_recording_rows = through_compatibility
    assert rows.gnss_base_recording_rows is through_compatibility
    assert panel.gnss_base_recording_rows is through_compatibility
    assert recording.ledger is primary


def _assisted_admission(**changes):
    values = {
        "mode": GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE,
        "caster_host": "fixture.invalid",
        "mountpoint": "MOUNT",
        "gga_policy": GgaPolicy.BASE_GNSS,
        "fixed_height_type": HeightType.ELLIPSOID,
        "base_serial_connected": True,
        "base_position_present": True,
        "base_position_monotonic_s": 99.0,
        "now_monotonic_s": 100.0,
    }
    values.update(changes)
    return NetworkAdmissionInputs(**values)


@pytest.mark.parametrize(
    "changes,message,tone",
    [
        (
            {"mode": GnssOperationMode.LOCAL_SURVEY_IN, "caster_host": ""},
            "현장 모드에서는 NTRIP을 시작할 수 없습니다",
            "warn",
        ),
        ({"caster_host": "", "mountpoint": ""}, "caster host 필요", "bad"),
        ({"mountpoint": "", "base_serial_connected": False}, "mountpoint 필요", "bad"),
        (
            {"base_serial_connected": False, "gga_policy": GgaPolicy.SYSTEM_CENTER},
            "보조 Fixed Base 모드는 먼저 Base serial을 연결하세요",
            "bad",
        ),
        (
            {"gga_policy": GgaPolicy.SYSTEM_CENTER, "fixed_height_type": HeightType.MSL},
            "보조 Fixed Base는 Base GNSS GGA가 필요합니다",
            "bad",
        ),
        (
            {"fixed_height_type": HeightType.MSL, "base_position_present": False},
            "절대 WGS84 운용은 Fixed Base 높이를 ellipsoid로 설정하세요",
            "bad",
        ),
        (
            {"base_position_present": False, "base_position_monotonic_s": None},
            "Base WGS84 위치 샘플 대기 중입니다",
            "warn",
        ),
        ({"base_position_monotonic_s": 84.999}, "Base WGS84 위치 샘플 대기 중입니다", "warn"),
    ],
)
def test_network_admission_preserves_guard_order_messages_and_datum(changes, message, tone):
    decision = GnssOperationRuntime.assess_network_start(_assisted_admission(**changes))
    assert (decision.accepted, decision.message, decision.tone) == (False, message, tone)


@pytest.mark.parametrize("sample_time", [85.0, 100.0, 110.0])
def test_network_admission_keeps_exact_15s_boundary_and_future_timestamp_clamp(sample_time):
    request = _assisted_admission(base_position_monotonic_s=sample_time)
    assert GnssOperationRuntime.assess_network_start(request).accepted


def test_direct_network_admission_does_not_require_assisted_base_inputs():
    request = _assisted_admission(
        mode=GnssOperationMode.NETWORK_RTK_DIRECT,
        base_serial_connected=False,
        base_position_present=False,
        base_position_monotonic_s=None,
        gga_policy=GgaPolicy.SYSTEM_CENTER,
        fixed_height_type=HeightType.MSL,
    )
    assert GnssOperationRuntime.assess_network_start(request).accepted


def test_network_admission_inputs_do_not_requery_later_sample_or_clock(monkeypatch):
    from dataclasses import FrozenInstanceError

    from Workstation.gnss_runtime import operation

    sample = SimpleNamespace(timestamp_s=99.0)
    captured = _assisted_admission(base_position_monotonic_s=sample.timestamp_s)
    sample.timestamp_s = 1.0
    monkeypatch.setattr(
        operation, "time", SimpleNamespace(monotonic=lambda: pytest.fail("admission read a live clock"))
    )
    assert operation.GnssOperationRuntime.assess_network_start(captured).accepted
    with pytest.raises(FrozenInstanceError):
        captured.now_monotonic_s = 1000.0


def test_begin_network_owns_source_config_and_state_admission():
    owner, events = _operation()
    config = GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT)
    assert owner.begin_network(config)
    assert owner.state.gnss_active_operation_config is config
    assert owner.state.gnss_operation_gate_active is True
    assert owner.state.gnss_operation_manager.source_identity.source_kind == "ntrip-direct"
    assert [event[0] for event in events] == [GnssOperationState.INITIALIZING, GnssOperationState.NTRIP_CONNECTING]


def test_network_acquisition_reset_preserves_base_gga_hint_survey_and_resource():
    owner, _ = _base()
    hint = GnssCoordinateSample(
        35.25, 129.5, 65.0, msl_altitude_m=40.0, height_type=HeightType.ELLIPSOID, timestamp_s=123.0, fixed=True
    )
    worker = object()
    survey = {"mean_x_m": 101.0, "mean_y_m": -203.0, "mean_z_m": 307.0}
    state = owner.state
    state.gnss_base_serial_worker = worker
    state.latest_gnss_base_position = hint
    state.latest_gnss_base_survey_in = survey
    state.gnss_coordinate_samples.append(hint)
    state.gnss_base_survey_samples.append((100.0, 60, 0.1, False, True))
    state.gnss_fixed_base_profile = state.gnss_fixed_base_live_profile = state.gnss_fixed_base_pending_profile = (
        object()
    )
    state.gnss_fixed_base_command_sent = True
    state.gnss_fixed_base_reconfigure_expected = {"mode": 2}
    state.gnss_fixed_base_reconfigure_ack = state.gnss_fixed_base_reconfigure_readback = (
        state.gnss_fixed_base_reconfigure_rtcm
    ) = True
    state.gnss_fixed_base_reconfigure_deadline_s = 140.0
    try:
        owner.begin_network_acquisition()
        assert not state.gnss_coordinate_samples
        assert state.latest_gnss_base_position is hint
        assert state.latest_gnss_base_survey_in is survey
        assert len(state.gnss_base_survey_samples) == 1
        assert state.gnss_base_serial_worker is worker
        assert state.gnss_fixed_base_profile is None
        assert state.gnss_fixed_base_live_profile is None
        assert state.gnss_fixed_base_pending_profile is None
        assert state.gnss_fixed_base_command_sent is False
        assert state.gnss_fixed_base_reconfigure_expected is None
        assert state.gnss_fixed_base_reconfigure_ack is False
        assert state.gnss_fixed_base_reconfigure_readback is False
        assert state.gnss_fixed_base_reconfigure_rtcm is False
        assert state.gnss_fixed_base_reconfigure_deadline_s is None
    finally:
        state.gnss_base_serial_worker = None
        owner.deleteLater()
