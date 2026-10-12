"""Base serial lifetime, survey generation and fixed-coordinate acquisition.

NAV-PVT WGS84 latitude/longitude degrees and ellipsoid/MSL metres stay distinct.
NAV-SVIN mean_x/y/z are ECEF metres; CFG-TMODE3 readback compares ECEF centimetres
plus 0.1 mm high precision fields. Existing core encoders/validator own formulas.
"""

import time
from collections import deque
from dataclasses import dataclass, field

from PyQt5 import QtCore

from Workstation.ui.telemetry_plots import TelemetryHistoryPoint
from Workstation.workstation_core.fixed_base_initializer import FixedBaseInitializer
from Workstation.workstation_core.gnss_operation import GnssOperationMode, GnssOperationState, HeightType
from Workstation.workstation_core.gnss_profile import GnssCoordinateSample
from Workstation.workstation_core.gnss_protocol import (
    parse_ubx_ack,
    parse_ubx_cfg_tmode3,
    ubx_cfg_tmode3_disabled_frame,
    ubx_cfg_tmode3_fixed_frame,
    ubx_cfg_tmode3_poll_frame,
    ubx_cfg_tmode3_survey_in_frame,
    ubx_nav_svin_poll_frame,
)


@dataclass
class BaseReceiverState:
    gnss_base_serial_thread: object = None
    gnss_base_serial_worker: object = None
    gnss_base_serial_connected: bool = False
    gnss_base_detect_thread: object = None
    gnss_base_detect_worker: object = None
    gnss_coordinate_samples: object = field(default_factory=lambda: deque(maxlen=2400))
    latest_gnss_base_position: object = None
    gnss_fixed_base_profile: object = None
    gnss_fixed_base_live_profile: object = None
    gnss_fixed_base_pending_profile: object = None
    gnss_fixed_base_command_sent: bool = False
    gnss_fixed_base_reconfigure_expected: object = None
    gnss_fixed_base_reconfigure_ack: bool = False
    gnss_fixed_base_reconfigure_readback: bool = False
    gnss_fixed_base_reconfigure_rtcm: bool = False
    gnss_fixed_base_reconfigure_deadline_s: object = None
    latest_gnss_base_survey_in: object = None
    gnss_base_survey_samples: object = field(default_factory=lambda: deque(maxlen=240))
    gnss_base_satellite_count: object = None
    gnss_base_satellite_count_source: str = ""
    gnss_base_satellite_quality: object = None
    _gnss_survey_generation: int = 0
    _gnss_survey_command_sent_generation: int = 0
    _gnss_survey_ack_generation: int = 0
    _gnss_survey_active_seen_generation: int = 0
    _gnss_survey_requested_duration_s: int = 0
    _gnss_survey_requested_accuracy_m: float = 0.0

    def resource_inventory(self):
        return {
            name: getattr(self, name)
            for name in (
                "gnss_base_serial_thread",
                "gnss_base_serial_worker",
                "gnss_base_detect_thread",
                "gnss_base_detect_worker",
            )
        }


BASE_FIELDS = tuple(BaseReceiverState.__dataclass_fields__)


@dataclass
class BaseReceiverEffects:
    health: object
    plot: object
    record_position: object
    reset_corrections: object


@dataclass
class _ReceiverBinding:
    generation: int
    thread: object
    worker: object
    relay: object = None
    stopping: bool = False


class _ReceiverRelay(QtCore.QObject):
    def __init__(self, owner, binding, kind):
        super().__init__(owner)
        self.owner, self.binding, self.kind = owner, binding, kind

    @QtCore.pyqtSlot(str, int)
    def connected(self, port, baud):
        self.owner.deliver(self.binding, self.kind, "connected", port, baud)

    @QtCore.pyqtSlot(int, float, float)
    def stats_changed(self, total, rate, age):
        self.owner.deliver(self.binding, self.kind, "stats_changed", total, rate, age)

    @QtCore.pyqtSlot(object)
    def survey_in_changed(self, value):
        self.owner.deliver(self.binding, self.kind, "survey_in_changed", value)

    @QtCore.pyqtSlot(int, str)
    def satellite_count_changed(self, count, source):
        self.owner.deliver(self.binding, self.kind, "satellite_count_changed", count, source)

    @QtCore.pyqtSlot(object)
    def satellite_quality_changed(self, value):
        self.owner.deliver(self.binding, self.kind, "satellite_quality_changed", value)

    @QtCore.pyqtSlot(object)
    def position_changed(self, value):
        self.owner.deliver(self.binding, self.kind, "position_changed", value)

    @QtCore.pyqtSlot(int, int, object)
    def ubx_message_received(self, cls, msg, value):
        self.owner.deliver(self.binding, self.kind, "ubx_message_received", cls, msg, value)

    @QtCore.pyqtSlot(str, int)
    def command_sent(self, label, count):
        self.owner.deliver(self.binding, self.kind, "command_sent", label, count)

    @QtCore.pyqtSlot(str)
    def failed(self, value):
        self.owner.deliver(self.binding, self.kind, "failed", value)

    @QtCore.pyqtSlot(str)
    def disconnected(self, value):
        self.owner.deliver(self.binding, self.kind, "disconnected", value)

    @QtCore.pyqtSlot(object)
    def detected(self, value):
        self.owner.deliver(self.binding, self.kind, "detected", value)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner.retire(self.binding, self.kind)


class BaseReceiverRuntime(QtCore.QObject):
    def __init__(
        self,
        parent,
        state,
        operation,
        *,
        config,
        network_connected,
        effects,
        serial_factory,
        detect_factory,
        serial_events,
        detect_events,
        retired,
    ):
        super().__init__(parent)
        self.state, self.operation = state, operation
        self.config, self.network_connected = config, network_connected
        self.health, self.plot = effects.health, effects.plot
        self.record_position, self.reset_corrections = effects.record_position, effects.reset_corrections
        self.serial_factory, self.detect_factory = serial_factory, detect_factory
        self.events = {"serial": dict(serial_events), "detect": dict(detect_events)}
        self.retired = retired
        self.bindings = {"serial": None, "detect": None}
        self.generation = 0
        self.callback_error = ""
        required = (
            config,
            network_connected,
            effects.health,
            effects.plot,
            effects.record_position,
            effects.reset_corrections,
            serial_factory,
            detect_factory,
            retired,
        )
        if any(not callable(callback) for callback in required):
            raise TypeError("Base receiver requires configuration, resource and effect adapters")
        for callbacks in (self.events["serial"], self.events["detect"]):
            if any(not callable(callback) for callback in callbacks.values()):
                raise TypeError("Base receiver events require callable adapters")

    def start_serial(self, *, port, baud, height_type, wire):
        return self._start("serial", lambda: self.serial_factory(port=port, baud=baud, height_type=height_type), wire)

    def start_detection(self, *, baud):
        return self._start("detect", lambda: self.detect_factory(baud=baud), lambda worker: None)

    def _start(self, kind, factory, wire):
        if self.bindings[kind] is not None or getattr(self.state, f"gnss_base_{kind}_thread") is not None:
            return False
        worker = factory()
        thread = QtCore.QThread(self)
        self.generation += 1
        binding = _ReceiverBinding(self.generation, thread, worker)
        binding.relay = _ReceiverRelay(self, binding, kind)
        try:
            thread.started.connect(worker.run)
            for event in self.events[kind]:
                getattr(worker, event).connect(getattr(binding.relay, event), QtCore.Qt.QueuedConnection)
            worker.finished.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            thread.finished.connect(binding.relay.finished, QtCore.Qt.QueuedConnection)
            # Install handles before the direct data-plane wiring so its route
            # sees the exact reserved serial resource, as it did previously.
            self.bindings[kind] = binding
            setattr(self.state, f"gnss_base_{kind}_thread", thread)
            setattr(self.state, f"gnss_base_{kind}_worker", worker)
            wire(worker)
            worker.moveToThread(thread)
            thread.start()
            if not thread.isRunning():
                raise RuntimeError("Base receiver thread did not start")
        except Exception:
            if not thread.isRunning():
                self.bindings[kind] = None
                setattr(self.state, f"gnss_base_{kind}_thread", None)
                setattr(self.state, f"gnss_base_{kind}_worker", None)
                worker.deleteLater()
                binding.relay.deleteLater()
                thread.deleteLater()
                self.retired(kind)
            else:
                self.stop(kind)
            raise
        return True

    def deliver(self, binding, kind, event, *args):
        if self.bindings[kind] is not binding:
            return
        # Preserve disconnect/failure cleanup while stopping, but never admit
        # queued old positions, survey completion or commands to a new session.
        if binding.stopping and event not in {"disconnected", "failed"}:
            return
        try:
            self.events[kind][event](*args)
        except Exception as exc:  # noqa: BLE001 - retain handles after presentation failure
            self.callback_error = f"{kind}.{event}: {exc}"

    def busy(self, kind="serial"):
        return self.bindings[kind] is not None or getattr(self.state, f"gnss_base_{kind}_thread") is not None

    def stop(self, kind="serial"):
        binding = self.bindings[kind]
        worker = getattr(self.state, f"gnss_base_{kind}_worker") if binding is None else binding.worker
        if binding is not None:
            binding.stopping = True
        request_stop = getattr(worker, "request_stop", None)
        if callable(request_stop):
            request_stop()
        # Port discovery has no cancellation contract; retain its resources
        # and wait for finished rather than pretending QThread.quit stopped I/O.
        return binding is None and getattr(self.state, f"gnss_base_{kind}_thread") is None

    def retire(self, binding, kind):
        if self.bindings[kind] is not binding or binding.thread.isRunning():
            return
        self.bindings[kind] = None
        setattr(self.state, f"gnss_base_{kind}_thread", None)
        setattr(self.state, f"gnss_base_{kind}_worker", None)
        if kind == "serial":
            self.state.gnss_base_serial_connected = False
        try:
            self.retired(kind)
        finally:
            binding.relay.deleteLater()
            binding.thread.deleteLater()

    def resource_inventory(self):
        resources = self.state.resource_inventory()
        for kind, binding in self.bindings.items():
            if binding is not None:
                resources[f"gnss_base_owned_{kind}_thread"] = binding.thread
                resources[f"gnss_base_owned_{kind}_worker"] = binding.worker
        return resources

    def check_reconfiguration_timeout(self):
        if self.operation.state.gnss_operation_manager.state != GnssOperationState.BASE_RECONFIGURING:
            return False
        deadline = self.state.gnss_fixed_base_reconfigure_deadline_s
        if deadline is not None and time.monotonic() > deadline:
            self.fail_reconfiguration("CFG-TMODE3 ACK/readback/post-config RTCM timeout")
        return True

    def begin_network_acquisition(self):
        """Start a new fixed-coordinate collection without discarding its GGA hint.

        Latest NAV-PVT position, survey/satellite evidence and serial resources
        belong to their existing lifetimes. Only the old fixed transaction and
        collected coordinate window are reset, as in the original NTRIP Start.
        """

        self.state.gnss_coordinate_samples.clear()
        self.state.gnss_fixed_base_profile = None
        self.state.gnss_fixed_base_live_profile = None
        self.state.gnss_fixed_base_pending_profile = None
        self.state.gnss_fixed_base_command_sent = False
        self.state.gnss_fixed_base_reconfigure_expected = None
        self.state.gnss_fixed_base_reconfigure_ack = False
        self.state.gnss_fixed_base_reconfigure_readback = False
        self.state.gnss_fixed_base_reconfigure_rtcm = False
        self.state.gnss_fixed_base_reconfigure_deadline_s = None

    def start_survey(self, config, *, duration_s, accuracy_m):
        worker = self.state.gnss_base_serial_worker
        if worker is None:
            self.health("survey_in", "serial 미연결", "bad")
            return False
        if config.operation_mode != GnssOperationMode.LOCAL_SURVEY_IN:
            self.health("survey_in", "현재 모드에서는 현장 Survey-in을 시작할 수 없습니다", "warn")
            return False
        self.operation.configure(config)
        self.operation.state.gnss_operation_manager.set_source_identity(self.operation.source_identity(config))
        self.operation.state.gnss_active_operation_config = config
        self.operation.state.gnss_operation_gate_active = True
        self.state._gnss_survey_generation += 1
        self.state._gnss_survey_command_sent_generation = 0
        self.state._gnss_survey_ack_generation = 0
        self.state._gnss_survey_active_seen_generation = 0
        self.state._gnss_survey_requested_duration_s = int(duration_s)
        self.state._gnss_survey_requested_accuracy_m = float(accuracy_m)
        self.operation.transition(GnssOperationState.INITIALIZING, "현장 Base Survey-in 초기화")
        self.operation.transition(GnssOperationState.BASE_SURVEYING, "현장 Base Survey-in 진행")
        frame = ubx_cfg_tmode3_survey_in_frame(min_duration_s=duration_s, accuracy_m=accuracy_m)
        self.state.gnss_base_survey_samples.clear()
        worker.send_bytes("CFG-TMODE3 survey-in start", frame)
        worker.send_bytes("NAV-SVIN poll after survey-in start", ubx_nav_svin_poll_frame())
        self.health("survey_in", "survey-in 시작 명령 전송 대기", "warn")
        return True

    def stop_survey(self):
        worker = self.state.gnss_base_serial_worker
        if worker is None:
            self.health("survey_in", "serial 미연결", "bad")
            return False
        worker.send_bytes("CFG-TMODE3 disabled", ubx_cfg_tmode3_disabled_frame())
        worker.send_bytes("NAV-SVIN poll after survey-in stop", ubx_nav_svin_poll_frame())
        self.operation.state.gnss_operation_gate_active = False
        self.state._gnss_survey_command_sent_generation = 0
        self.state._gnss_survey_ack_generation = 0
        self.state._gnss_survey_active_seen_generation = 0
        self.operation.state.gnss_operation_manager.stop()
        self.operation.state.gnss_active_operation_config = None
        return True

    def observe_position(self, sample: object) -> None:
        if not isinstance(sample, GnssCoordinateSample):
            return
        self.state.gnss_coordinate_samples.append(sample)
        self.state.latest_gnss_base_position = sample
        height_m = sample.selected_height_m
        height_text = "--" if height_m is None else f"{height_m:.3f}m"
        self.health(
            "base_position",
            f"WGS84 {sample.latitude_deg:.7f}, {sample.longitude_deg:.7f} | h={height_text} ({sample.height_type.value}) | fix={sample.fixed}",
            "good" if sample.fixed else "warn",
        )
        config = self.config()
        self.record_position(sample)
        if config.operation_mode != GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE:
            return
        self.plot(
            "gnss_base_fixed_state_plot",
            TelemetryHistoryPoint(sample.timestamp_s, "fix", 1.0 if sample.fixed else 0.0),
        )
        if not self.network_connected() or self.state.gnss_fixed_base_command_sent:
            return
        state = self.operation.state.gnss_operation_manager.state
        if state == GnssOperationState.NTRIP_ACTIVE:
            self.operation.transition(GnssOperationState.BASE_AS_ROVER_FLOAT, "Base가 NTRIP 보정 rover로 수렴 중")
            state = self.operation.state.gnss_operation_manager.state
        if sample.fixed and state == GnssOperationState.BASE_AS_ROVER_FLOAT:
            self.operation.transition(GnssOperationState.BASE_AS_ROVER_FIXED, "Base rover가 RTK FIXED")
            state = self.operation.state.gnss_operation_manager.state
        if sample.fixed and state == GnssOperationState.BASE_AS_ROVER_FIXED:
            self.operation.transition(GnssOperationState.FIXED_COORDINATE_COLLECTING, "고정 좌표 반복 측정 수집")
        if self.operation.state.gnss_operation_manager.state != GnssOperationState.FIXED_COORDINATE_COLLECTING:
            self.health(
                "base_fixed",
                f"{self.operation.state.gnss_operation_manager.state.value} | Base fix={sample.fixed}",
                "warn",
            )
            return
        if sample.height_type != config.fixed_base.height_type:
            self.health("base_fixed", "Base 좌표 height datum이 활성 GNSS 운용 설정과 다릅니다", "bad")
            self.operation.transition(GnssOperationState.FAULT, "Base coordinate height datum changed during operation")
            return
        validation = FixedBaseInitializer(config.fixed_base).validate(
            tuple(self.state.gnss_coordinate_samples),
            acquisition_start_utc="",
            acquisition_end_utc="",
            source_caster=config.network.caster_host,
            source_mountpoint=config.network.mountpoint,
        )
        profile = validation.profile
        self.state.gnss_fixed_base_live_profile = profile
        if profile is None:
            self.health("base_fixed", "고정 좌표 프로파일 수집 중", "warn")
            return
        self.health(
            "base_fixed",
            f"FIXED 수집 {profile.collection_duration_s:.1f}s/{config.fixed_base.acquisition_duration_s:.1f}s | scatter H/V={profile.horizontal_scatter_m:.3f}/{profile.vertical_scatter_m:.3f}m",
            "warn" if not validation.valid else "good",
        )
        self.plot(
            "gnss_base_fixed_scatter_plot",
            TelemetryHistoryPoint(sample.timestamp_s, "horizontal", profile.horizontal_scatter_m),
        )
        self.plot(
            "gnss_base_fixed_scatter_plot",
            TelemetryHistoryPoint(sample.timestamp_s, "vertical", profile.vertical_scatter_m),
        )
        self.plot(
            "gnss_base_fixed_scatter_plot",
            TelemetryHistoryPoint(sample.timestamp_s, "target_h", config.fixed_base.maximum_horizontal_scatter_m),
        )
        self.plot(
            "gnss_base_fixed_scatter_plot",
            TelemetryHistoryPoint(sample.timestamp_s, "target_v", config.fixed_base.maximum_vertical_scatter_m),
        )
        if not validation.valid:
            return
        if profile.height_type != HeightType.ELLIPSOID:
            self.health("base_fixed", "고정 좌표 검증됨이지만 TMODE3에는 WGS84 ellipsoid height가 필요합니다", "bad")
            return
        worker = self.state.gnss_base_serial_worker
        if worker is None or not self.state.gnss_base_serial_connected:
            self.health("base_fixed", "고정 좌표 검증됨 | Base serial 재연결 필요", "bad")
            return
        if not self.operation.transition(GnssOperationState.FIXED_COORDINATE_VALIDATED, "반복 측정 좌표 검증 통과"):
            return
        if not self.operation.transition(GnssOperationState.BASE_RECONFIGURING, "CFG-TMODE3 fixed 좌표 적용"):
            return
        accuracy_m = max(0.01, profile.horizontal_scatter_m, profile.vertical_scatter_m)
        fixed_frame = ubx_cfg_tmode3_fixed_frame(
            latitude_deg=profile.latitude_deg,
            longitude_deg=profile.longitude_deg,
            ellipsoid_height_m=profile.height_m,
            fixed_position_accuracy_m=accuracy_m,
        )
        expected = parse_ubx_cfg_tmode3(fixed_frame[6:-2])
        if expected is None:
            self.operation.transition(GnssOperationState.FAULT, "CFG-TMODE3 fixed command could not be decoded")
            return
        self.reset_corrections()
        worker.send_bytes("CFG-TMODE3 fixed coordinate", fixed_frame)
        self.state.gnss_fixed_base_pending_profile = profile
        self.state.gnss_fixed_base_reconfigure_expected = expected
        self.state.gnss_fixed_base_reconfigure_ack = False
        self.state.gnss_fixed_base_reconfigure_readback = False
        self.state.gnss_fixed_base_reconfigure_rtcm = False
        self.state.gnss_fixed_base_reconfigure_deadline_s = time.monotonic() + 10.0
        self.state.gnss_fixed_base_command_sent = True
        self.health("base_fixed", "CFG-TMODE3 fixed 전송됨 | ACK/readback/post-config RTCM 대기", "warn")

    def fail_reconfiguration(self, reason: str) -> None:
        self.state.gnss_fixed_base_pending_profile = None
        self.state.gnss_fixed_base_reconfigure_expected = None
        self.state.gnss_fixed_base_reconfigure_deadline_s = None
        self.health("base_fixed", f"Fixed Base 재설정 실패: {reason}", "bad")
        self.operation.transition(GnssOperationState.FAULT, f"fixed base reconfiguration failed: {reason}")

    def observe_ubx(self, message_class: int, message_id: int, payload: object) -> None:
        """Advance fixed-base setup only from receiver evidence, not queued writes."""
        if not isinstance(payload, (bytes, bytearray)):
            return
        raw_payload = bytes(payload)
        ack = parse_ubx_ack(int(message_class), int(message_id), raw_payload)
        if self.operation.state.gnss_operation_manager.state == GnssOperationState.BASE_SURVEYING:
            if ack is None:
                return
            (accepted, target_class, target_id) = ack
            if (target_class, target_id) != (6, 113):
                return
            generation = int(self.state._gnss_survey_generation or 0)
            sent_generation = int(self.state._gnss_survey_command_sent_generation or 0)
            if generation <= 0 or sent_generation != generation:
                return
            if not accepted:
                self.operation.transition(GnssOperationState.FAULT, "survey-in CFG-TMODE3 ACK-NAK")
                return
            self.state._gnss_survey_ack_generation = generation
            worker = self.state.gnss_base_serial_worker
            if worker is not None:
                worker.send_bytes("NAV-SVIN poll after survey-in ACK", ubx_nav_svin_poll_frame())
            return
        if self.operation.state.gnss_operation_manager.state != GnssOperationState.BASE_RECONFIGURING:
            return
        if ack is not None:
            (accepted, target_class, target_id) = ack
            if (target_class, target_id) != (6, 113):
                return
            if not accepted:
                self.fail_reconfiguration("CFG-TMODE3 ACK-NAK")
                return
            self.state.gnss_fixed_base_reconfigure_ack = True
            worker = self.state.gnss_base_serial_worker
            if worker is None or not self.state.gnss_base_serial_connected:
                self.fail_reconfiguration("Base serial disconnected before CFG-TMODE3 readback")
                return
            worker.send_bytes("CFG-TMODE3 poll after fixed coordinate ACK", ubx_cfg_tmode3_poll_frame())
            self.health("base_fixed", "CFG-TMODE3 ACK 수신 | readback 대기", "warn")
            return
        if (int(message_class), int(message_id)) != (6, 113):
            return
        expected = self.state.gnss_fixed_base_reconfigure_expected
        received = parse_ubx_cfg_tmode3(raw_payload)
        if expected is None or received is None:
            self.fail_reconfiguration("invalid CFG-TMODE3 readback")
            return
        keys = ("mode", "ecef_cm", "ecef_hp_0_1mm", "fixed_accuracy_0_1mm")
        if any(received.get(key) != expected.get(key) for key in keys):
            self.fail_reconfiguration("CFG-TMODE3 readback mismatch")
            return
        self.state.gnss_fixed_base_reconfigure_readback = True
        self.health("base_fixed", "CFG-TMODE3 readback 확인 | post-config RTCM 대기", "warn")
        self.maybe_activate()

    def maybe_activate(self) -> None:
        if self.operation.state.gnss_operation_manager.state != GnssOperationState.BASE_RECONFIGURING:
            return
        if not (
            self.state.gnss_fixed_base_reconfigure_ack
            and self.state.gnss_fixed_base_reconfigure_readback
            and self.state.gnss_fixed_base_reconfigure_rtcm
        ):
            return
        profile = self.state.gnss_fixed_base_pending_profile
        if profile is None:
            self.fail_reconfiguration("validated profile was lost")
            return
        self.state.gnss_fixed_base_profile = profile
        self.state.gnss_fixed_base_pending_profile = None
        self.state.gnss_fixed_base_reconfigure_deadline_s = None
        self.operation.transition(
            GnssOperationState.LOCAL_RTCM_ACTIVE, "CFG-TMODE3 ACK/readback와 post-config Base RTCM 확인"
        )

    def reset_acquisition(self) -> None:
        """Clear all Base fixed-coordinate acquisition progress.

        A dropped Base serial link means the receiver's own survey-in/TMODE3
        state can no longer be trusted, so a subsequent connect must start
        the acquisition sequence from zero instead of resuming stale samples
        or a stale (possibly no-longer-valid) fixed-coordinate profile.
        """
        self.state.gnss_coordinate_samples.clear()
        self.state.latest_gnss_base_position = None
        self.state.gnss_fixed_base_profile = None
        self.state.gnss_fixed_base_live_profile = None
        self.state.gnss_fixed_base_pending_profile = None
        self.state.gnss_fixed_base_command_sent = False
        self.state.gnss_fixed_base_reconfigure_expected = None
        self.state.gnss_fixed_base_reconfigure_ack = False
        self.state.gnss_fixed_base_reconfigure_readback = False
        self.state.gnss_fixed_base_reconfigure_rtcm = False
        self.state.gnss_fixed_base_reconfigure_deadline_s = None
        self.state.latest_gnss_base_survey_in = None
        self.state.gnss_base_survey_samples.clear()
        self.state._gnss_survey_command_sent_generation = 0
        self.state._gnss_survey_ack_generation = 0
        self.state._gnss_survey_active_seen_generation = 0
        self.state.gnss_base_satellite_count = None
        self.state.gnss_base_satellite_count_source = ""
        self.state.gnss_base_satellite_quality = None
        self.reset_corrections()

    def observe_survey(self, state):
        self.state.latest_gnss_base_survey_in = dict(state)
        active, valid = bool(state.get("active")), bool(state.get("valid"))
        duration_s = int(state.get("duration_s") or 0)
        accuracy_m = float(state.get("mean_accuracy_m") or 0.0)
        generation = int(getattr(self.state, "_gnss_survey_generation", 0) or 0)
        generation_ready = bool(
            generation > 0
            and int(getattr(self.state, "_gnss_survey_command_sent_generation", 0) or 0) == generation
            and int(getattr(self.state, "_gnss_survey_ack_generation", 0) or 0) == generation
        )
        if active and not valid and generation_ready:
            self.state._gnss_survey_active_seen_generation = generation
        requested_duration_s = int(getattr(self.state, "_gnss_survey_requested_duration_s", 0) or 0)
        requested_accuracy_m = float(getattr(self.state, "_gnss_survey_requested_accuracy_m", 0.0) or 0.0)
        completion_correlated = bool(
            generation_ready
            and int(getattr(self.state, "_gnss_survey_active_seen_generation", 0) or 0) == generation
            and duration_s >= requested_duration_s
            and requested_accuracy_m > 0.0
            and accuracy_m <= requested_accuracy_m
        )
        if valid and completion_correlated and self.config().operation_mode == GnssOperationMode.LOCAL_SURVEY_IN:
            if self.operation.state.gnss_operation_manager.state == GnssOperationState.BASE_SURVEYING:
                self.operation.transition(
                    GnssOperationState.LOCAL_RTCM_ACTIVE, "현장 Survey-in valid; RTCM rover 라우팅"
                )
            return True
        elif valid and self.operation.state.gnss_operation_manager.state == GnssOperationState.BASE_SURVEYING:
            self.health(
                "survey_in",
                "valid NAV-SVIN ignored until current CFG ACK, active observation, and requested limits are confirmed",
                "warn",
            )

        return False
