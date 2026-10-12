"""RTCM data-plane/injection resource ownership and direct queue wiring.

Raw RTCM bytes, source timestamps, envelope sequences and immutable evidence
contexts remain in RtcmDataPlaneWorker. This owner never sends raw ingress via
GUI events. The existing subscriber remains the MAVLink connection/lock owner.
"""

import time
from dataclasses import dataclass, field

from PyQt5 import QtCore

from Workstation.workers.device_runtime_workers import PixhawkRtcmInjectionWorker
from Workstation.gnss_runtime.ntrip import NtripStatsForwarder
from Workstation.workers.gnss_runtime_workers import GnssBaseSerialWorker, NtripNetworkWorker, RtcmDataPlaneWorker
from Workstation.workstation_core.gnss_protocol import parse_rtcm3_message_type, parse_rtcm3_reference_station_arp


def _connect_gui_signal(signal, callback):
    signal.connect(callback, QtCore.Qt.QueuedConnection)


def new_rtcm_stats():
    return {
        "start_s": time.monotonic(),
        "last_frame_s": None,
        "frames": 0,
        "bytes": 0,
        "max_frame_bytes": 0,
        "message_types": {},
        "injection": {
            side: {"frames": 0, "bytes": 0, "packets": 0, "fail": 0, "blocked_stall": 0}
            for side in ("left", "right", "drone")
        },
    }


@dataclass
class CorrectionRuntimeState:
    rtcm_pipeline_thread: object = None
    rtcm_pipeline_worker: object = None
    _rtcm_pipeline_base_worker: object = None
    _rtcm_pipeline_injection_worker: object = None
    pixhawk_rtcm_thread: object = None
    pixhawk_rtcm_worker: object = None
    _pixhawk_rtcm_restart_pending: object = None
    gnss_base_rtcm_stats: dict = field(default_factory=new_rtcm_stats)
    latest_rtcm_reference_station: object = None

    def resource_inventory(self):
        return {
            name: getattr(self, name)
            for name in ("rtcm_pipeline_thread", "rtcm_pipeline_worker", "pixhawk_rtcm_thread", "pixhawk_rtcm_worker")
        }


CORRECTION_FIELDS = tuple(CorrectionRuntimeState.__dataclass_fields__)


@dataclass
class CorrectionEvents:
    snapshot: object
    evidence_started: object
    evidence_drained: object
    integrity_failed: object
    injection_retired: object
    reset_observed_counters: object


class _PipelineRelay(QtCore.QObject):
    def __init__(self, owner, thread, worker):
        super().__init__(owner)
        self.owner, self.resource_thread, self.worker = owner, thread, worker

    @QtCore.pyqtSlot(object)
    def snapshot(self, payload):
        state = self.owner.state
        if state.rtcm_pipeline_thread is self.resource_thread and state.rtcm_pipeline_worker is self.worker:
            self.owner.events.snapshot(payload)

    @QtCore.pyqtSlot(object)
    def evidence_started(self, payload):
        self.owner.events.evidence_started(payload)

    @QtCore.pyqtSlot(object)
    def evidence_drained(self, payload):
        # Evidence belongs to its captured package even after resource retirement.
        self.owner.events.evidence_drained(payload)

    @QtCore.pyqtSlot(object)
    def integrity_failed(self, payload):
        self.owner.events.integrity_failed(payload)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner._clear_rtcm_pipeline(self.resource_thread, self.worker)
        self.deleteLater()


class CorrectionRuntime(QtCore.QObject):
    RTCM_UI_UPDATE_INTERVAL_S = 0.5

    def __init__(
        self,
        parent,
        state,
        *,
        route,
        evidence,
        base_worker,
        subscriber,
        pipeline_factory,
        injection_factory,
        events,
        thread_factory=None,
    ):
        super().__init__(parent)
        self.state = state
        self._rtcm_route_config = route
        self.evidence = evidence
        self.base_worker, self.subscriber = base_worker, subscriber
        self.pipeline_factory, self.injection_factory = pipeline_factory, injection_factory
        self.events = events
        self.thread_factory = thread_factory or QtCore.QThread
        required = (
            route,
            evidence.context,
            base_worker,
            subscriber,
            pipeline_factory,
            injection_factory,
            self.thread_factory,
        )
        if any(not callable(callback) for callback in required):
            raise TypeError("RTCM lifecycle requires resource, route and evidence adapters")
        if any(not callable(getattr(events, name)) for name in events.__dataclass_fields__):
            raise TypeError("RTCM lifecycle requires explicit event adapters")

    def resource_inventory(self):
        return self.state.resource_inventory()

    def _ensure_rtcm_pipeline(self) -> RtcmDataPlaneWorker:
        existing = getattr(self.state, "rtcm_pipeline_worker", None)
        thread = getattr(self.state, "rtcm_pipeline_thread", None)
        if existing is not None and thread is not None and thread.isRunning():
            return existing
        if thread is not None and thread.isRunning():
            raise RuntimeError("RTCM pipeline thread exists without its worker")

        worker = self.pipeline_factory(snapshot_interval_s=self.RTCM_UI_UPDATE_INTERVAL_S)
        try:
            thread = self.thread_factory(self)
        except Exception:
            worker.deleteLater()
            raise
        relay = _PipelineRelay(self, thread, worker)
        worker.moveToThread(thread)
        _connect_gui_signal(worker.snapshot_ready, relay.snapshot)
        _connect_gui_signal(worker.evidence_started, relay.evidence_started)
        _connect_gui_signal(worker.evidence_drained, relay.evidence_drained)
        _connect_gui_signal(worker.integrity_failed, relay.integrity_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        _connect_gui_signal(thread.finished, relay.finished)
        thread.finished.connect(thread.deleteLater)
        self.state.rtcm_pipeline_thread = thread
        self.state.rtcm_pipeline_worker = worker
        self.state._rtcm_pipeline_base_worker = None
        self.state._rtcm_pipeline_injection_worker = None
        try:
            thread.start()
            if not thread.isRunning():
                raise RuntimeError("RTCM pipeline thread did not start")
            worker.configure_requested.emit(self._rtcm_route_config().as_dict())
            worker.evidence_context_requested.emit(self.evidence.context().as_dict())
            self._wire_rtcm_pipeline_base_worker(self.base_worker())
            self._wire_rtcm_pipeline_injection_worker(self.state.pixhawk_rtcm_worker)
        except Exception:
            if thread.isRunning():
                worker.stop_requested.emit()
            else:
                self._clear_rtcm_pipeline(thread, worker)
                worker.deleteLater()
                relay.deleteLater()
                thread.deleteLater()
            raise
        return worker

    def _wire_rtcm_pipeline_base_worker(self, base_worker: object | None) -> None:
        pipeline = getattr(self.state, "rtcm_pipeline_worker", None)
        if pipeline is None:
            return
        current = getattr(self.state, "_rtcm_pipeline_base_worker", None)
        if current is base_worker:
            return
        try:
            pipeline.base_write_requested.disconnect()
        except (TypeError, RuntimeError):
            pass
        self.state._rtcm_pipeline_base_worker = base_worker
        if base_worker is not None:
            # GnssBaseSerialWorker.run() owns a blocking serial loop, so its
            # QThread event loop cannot service a queued slot. send_bytes()
            # only performs a thread-safe Queue.put and is therefore the one
            # deliberate direct cross-thread adapter on this boundary.
            pipeline.base_write_requested.connect(
                base_worker.send_bytes,
                QtCore.Qt.DirectConnection,
            )

    def _wire_rtcm_pipeline_injection_worker(self, injection_worker: object | None) -> None:
        pipeline = getattr(self.state, "rtcm_pipeline_worker", None)
        if pipeline is None:
            return
        current = getattr(self.state, "_rtcm_pipeline_injection_worker", None)
        if current is injection_worker:
            return
        try:
            pipeline.injection_requested.disconnect()
        except (TypeError, RuntimeError):
            pass
        if current is not None:
            try:
                current.injected.disconnect(pipeline.injection_result_received)
            except (TypeError, RuntimeError):
                pass
        self.state._rtcm_pipeline_injection_worker = injection_worker
        if injection_worker is not None:
            # submit_envelope only touches the worker's locked bounded queue;
            # device I/O remains in _drain_one on the injection QThread.
            pipeline.injection_requested.connect(
                injection_worker.submit_envelope,
                QtCore.Qt.DirectConnection,
            )
            injection_worker.injected.connect(pipeline.injection_result_received)

    @staticmethod
    def _wire_rtcm_ntrip_source(
        source: NtripNetworkWorker,
        pipeline: RtcmDataPlaneWorker,
        *,
        session_token: int | None = None,
    ) -> None:
        if session_token is None:
            source.stats_snapshot_changed.connect(pipeline.ntrip_stats_received)
        else:
            source._session_stats_forwarder = NtripStatsForwarder(source, pipeline, session_token)
        source.chunk_envelope_received.connect(
            pipeline.submit_ntrip_envelope,
            QtCore.Qt.DirectConnection,
        )

    @staticmethod
    def _wire_rtcm_base_source(
        source: GnssBaseSerialWorker,
        pipeline: RtcmDataPlaneWorker,
    ) -> None:
        source.rtcm_frame_envelope_received.connect(
            pipeline.submit_base_envelope,
            QtCore.Qt.DirectConnection,
        )

    def _sync_rtcm_pipeline_config(self, *, reset: bool = False) -> None:
        pipeline = getattr(self.state, "rtcm_pipeline_worker", None)
        if pipeline is None:
            return
        self._wire_rtcm_pipeline_base_worker(self.base_worker())
        self._wire_rtcm_pipeline_injection_worker(getattr(self.state, "pixhawk_rtcm_worker", None))
        payload = self._rtcm_route_config().as_dict()
        if reset:
            self.events.reset_observed_counters()
            pipeline.reset_requested.emit(payload)
        else:
            pipeline.configure_requested.emit(payload)

    def _sync_rtcm_evidence_context(self) -> None:
        pipeline = getattr(self.state, "rtcm_pipeline_worker", None)
        if pipeline is not None:
            pipeline.evidence_context_requested.emit(self.evidence.context().as_dict())

    def _stop_rtcm_pipeline(self) -> bool:
        thread = getattr(self.state, "rtcm_pipeline_thread", None)
        worker = getattr(self.state, "rtcm_pipeline_worker", None)
        if thread is None or not thread.isRunning():
            return True
        if worker is not None:
            worker.stop_requested.emit()
        return False

    def _clear_rtcm_pipeline(
        self,
        thread: QtCore.QThread,
        worker: RtcmDataPlaneWorker,
    ) -> None:
        if getattr(self.state, "rtcm_pipeline_thread", None) is thread:
            self.state.rtcm_pipeline_thread = None
            self.state.rtcm_pipeline_worker = None
            self.state._rtcm_pipeline_base_worker = None
            self.state._rtcm_pipeline_injection_worker = None

    def _ensure_pixhawk_rtcm_worker(self) -> PixhawkRtcmInjectionWorker | None:
        subscriber = self.subscriber()
        if subscriber is None:
            return None
        existing_thread = self.state.pixhawk_rtcm_thread
        existing_worker = self.state.pixhawk_rtcm_worker
        if (
            existing_thread is not None
            and existing_worker is not None
            and existing_thread.isRunning()
            and existing_worker.subscriber is subscriber
        ):
            self._wire_rtcm_pipeline_injection_worker(existing_worker)
            return existing_worker
        if existing_thread is not None or existing_worker is not None:
            restart_token = (existing_thread, subscriber)
            if (
                existing_thread is not None
                and existing_thread.isRunning()
                and getattr(
                    self.state,
                    "_pixhawk_rtcm_restart_pending",
                    None,
                )
                != restart_token
            ):
                self.state._pixhawk_rtcm_restart_pending = restart_token
                _connect_gui_signal(
                    existing_thread.finished,
                    lambda old_thread=existing_thread, desired=subscriber: (
                        self._restart_pixhawk_rtcm_worker_after_retire(
                            old_thread,
                            desired,
                        )
                    ),
                )
            # QThread.wait() here used to freeze the GUI for up to one second
            # on every subscriber replacement.  Request retirement and let
            # the finished signal resume construction instead.
            if not self._retire_pixhawk_rtcm_worker(wait=False):
                return None
            if getattr(self.state, "_pixhawk_rtcm_restart_pending", None) == restart_token:
                self.state._pixhawk_rtcm_restart_pending = None
        self._ensure_rtcm_pipeline()
        worker = self.injection_factory(subscriber)
        try:
            thread = self.thread_factory(self)
        except Exception:
            worker.deleteLater()
            raise
        worker.moveToThread(thread)
        worker.request.connect(worker.enqueue, QtCore.Qt.QueuedConnection)
        worker.request_envelope.connect(worker.enqueue_envelope, QtCore.Qt.QueuedConnection)
        thread.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        _connect_gui_signal(
            thread.finished,
            lambda thread=thread, worker=worker: self._clear_pixhawk_rtcm_worker(
                thread,
                worker,
            ),
        )
        self.state.pixhawk_rtcm_thread = thread
        self.state.pixhawk_rtcm_worker = worker
        self._wire_rtcm_pipeline_injection_worker(worker)
        try:
            thread.start()
            if not thread.isRunning():
                raise RuntimeError("RTCM injection thread did not start")
        except Exception:
            self._retire_pixhawk_rtcm_worker(wait=False)
            if not thread.isRunning():
                worker.deleteLater()
                thread.deleteLater()
            raise
        return worker

    def _restart_pixhawk_rtcm_worker_after_retire(
        self,
        old_thread: object,
        desired_subscriber: object,
    ) -> None:
        token = (old_thread, desired_subscriber)
        if getattr(self.state, "_pixhawk_rtcm_restart_pending", None) != token:
            return
        self.state._pixhawk_rtcm_restart_pending = None
        if self.subscriber() is desired_subscriber:
            self._ensure_pixhawk_rtcm_worker()

    def _retire_pixhawk_rtcm_worker(
        self,
        *,
        expected_subscriber: object | None = None,
        wait: bool,
    ) -> bool:
        thread = getattr(self.state, "pixhawk_rtcm_thread", None)
        worker = getattr(self.state, "pixhawk_rtcm_worker", None)
        if worker is not None and (expected_subscriber is None or worker.subscriber is expected_subscriber):
            try:
                worker.clear_pending()
            except Exception:  # noqa: BLE001, S110 - preserve best-effort queue retirement
                pass
        elif expected_subscriber is not None:
            return True
        if getattr(self.state, "_rtcm_pipeline_injection_worker", None) is worker:
            self._wire_rtcm_pipeline_injection_worker(None)
        if thread is not None and thread.isRunning():
            thread.quit()
            if wait and not thread.wait(1000):
                return False
        if thread is None or not thread.isRunning():
            self._clear_pixhawk_rtcm_worker(thread, worker)
            return True
        return False

    def _clear_pixhawk_rtcm_worker(
        self,
        thread: QtCore.QThread | None = None,
        worker: PixhawkRtcmInjectionWorker | None = None,
    ) -> None:
        current_thread = getattr(self.state, "pixhawk_rtcm_thread", None)
        current_worker = getattr(self.state, "pixhawk_rtcm_worker", None)
        if thread is not None and current_thread is not thread:
            return
        if worker is not None and current_worker is not worker:
            return
        if getattr(self.state, "_rtcm_pipeline_injection_worker", None) is current_worker:
            self._wire_rtcm_pipeline_injection_worker(None)
        self.state.pixhawk_rtcm_thread = None
        self.state.pixhawk_rtcm_worker = None
        self._sync_rtcm_pipeline_config()
        self.events.injection_retired()

    def _record_gnss_base_rtcm_frame(self, payload: bytes) -> None:
        stats = self.state.gnss_base_rtcm_stats
        stats["frames"] = int(stats.get("frames") or 0) + 1
        stats["bytes"] = int(stats.get("bytes") or 0) + len(payload)
        stats["max_frame_bytes"] = max(int(stats.get("max_frame_bytes") or 0), len(payload))
        stats["last_frame_s"] = time.monotonic()
        message_type = parse_rtcm3_message_type(payload)
        if message_type is not None:
            message_types = stats.get("message_types")
            if not isinstance(message_types, dict):
                message_types = {}
                stats["message_types"] = message_types
            message_types[message_type] = int(message_types.get(message_type, 0)) + 1
        reference = parse_rtcm3_reference_station_arp(payload)
        if reference is not None:
            # A VRS caster fixes its virtual station from the GGA sent at
            # session start and holds it for that session, so this value
            # identifies the frame every rover fix from this stream resolves
            # against.  It is only replaced by a newly observed one, never
            # cleared on a stream gap: the retained fixes still belong to the
            # station they were measured under.
            station_id, ecef_01mm = reference
            self.state.latest_rtcm_reference_station = {
                "station_id": int(station_id),
                "ecef_01mm": tuple(int(value) for value in ecef_01mm),
                "observed_monotonic_s": time.monotonic(),
            }

    def _record_gnss_base_rtcm_injection(
        self,
        side: str,
        *,
        payload_bytes: int,
        packet_count: int = 0,
        failed: bool = False,
        blocked_stall: bool = False,
    ) -> None:
        injection = self.state.gnss_base_rtcm_stats.get("injection")
        if not isinstance(injection, dict):
            return
        side_stats = injection.get(side)
        if not isinstance(side_stats, dict):
            return
        if failed:
            side_stats["fail"] = int(side_stats.get("fail") or 0) + 1
            return
        if blocked_stall:
            side_stats["blocked_stall"] = int(side_stats.get("blocked_stall") or 0) + 1
            return
        side_stats["frames"] = int(side_stats.get("frames") or 0) + 1
        side_stats["bytes"] = int(side_stats.get("bytes") or 0) + int(payload_bytes)
        side_stats["packets"] = int(side_stats.get("packets") or 0) + int(packet_count)
        side_stats["last_delivery_s"] = time.monotonic()

    def reset_stats(self):
        self.state.gnss_base_rtcm_stats = new_rtcm_stats()
        if self.state.rtcm_pipeline_worker is not None:
            self.events.reset_observed_counters()
            self.state.rtcm_pipeline_worker.reset_requested.emit(self._rtcm_route_config().as_dict())

    def apply_snapshot_stats(self, snapshot):
        injection = snapshot.get("injection")
        message_types = snapshot.get("message_types")
        self.state.gnss_base_rtcm_stats = {
            "start_s": float(snapshot.get("start_s") or time.monotonic()),
            "last_frame_s": snapshot.get("last_frame_s"),
            "frames": int(snapshot.get("frames") or 0),
            "bytes": int(snapshot.get("bytes") or 0),
            "max_frame_bytes": int(snapshot.get("max_frame_bytes") or 0),
            "message_types": dict(message_types) if isinstance(message_types, dict) else {},
            "injection": (
                {side: dict(values) for side, values in injection.items() if isinstance(values, dict)}
                if isinstance(injection, dict)
                else {}
            ),
            "transport": dict(snapshot.get("transport") or {}),
            "integrity": dict(snapshot.get("integrity") or {}),
            "ingress": dict(snapshot.get("ingress") or {}),
        }
