from __future__ import annotations

from contextlib import contextmanager
from types import SimpleNamespace
import time

from PyQt5 import QtCore, QtWidgets

from Workstation.device_runtime_workers import PixhawkRtcmInjectionWorker
from Workstation.gnss_runtime_controller import GnssRuntimeController
from Workstation.gnss_runtime_workers import NtripNetworkWorker, RtcmDataPlaneWorker
from Workstation.workstation_core.gnss_operation import NetworkConfig


RTCM_1077 = bytes.fromhex("d300034350008d44be")


class _Subscriber:
    def __init__(self) -> None:
        self.sent: list[tuple[str, bytes]] = []

    @contextmanager
    def exclusive_command(self, _side: str):
        yield

    def send_rtcm_data(self, side: str, payload: bytes) -> int:
        self.sent.append((side, bytes(payload)))
        return 1


def _process_until(app, predicate, *, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while not predicate() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    return bool(predicate())


def test_ntrip_to_injection_evidence_and_ui_snapshot_end_to_end(monkeypatch, tmp_path) -> None:
    import Workstation.gnss_runtime_workers as runtime_workers

    class FakeClient:
        def __init__(self, _config, **_kwargs) -> None:
            self._stats = None

        @property
        def stats(self):
            return self._stats

        def connect(self, *, gga_sentence=None):
            self._stats = SimpleNamespace(
                connected_at_monotonic_s=time.monotonic(),
                bytes_received=0,
                chunks_received=0,
                last_chunk_monotonic_s=None,
                last_gga_monotonic_s=None,
            )
            return "ICY 200 OK", b""

        def iter_chunks(self, stop_event, **_kwargs):
            for sequence in range(20):
                self._stats = SimpleNamespace(
                    connected_at_monotonic_s=1.0,
                    bytes_received=(sequence + 1) * len(RTCM_1077),
                    chunks_received=sequence + 1,
                    last_chunk_monotonic_s=time.monotonic(),
                    last_gga_monotonic_s=None,
                )
                yield RTCM_1077
            stop_event.set()

        def close(self):
            return None

    monkeypatch.setattr(runtime_workers, "NtripClient", FakeClient)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    pipeline_thread = QtCore.QThread()
    pipeline = RtcmDataPlaneWorker(snapshot_interval_s=0.5)
    pipeline.moveToThread(pipeline_thread)
    pipeline.finished.connect(pipeline_thread.quit)
    snapshots: list[dict[str, object]] = []
    pipeline.snapshot_ready.connect(snapshots.append)

    subscriber = _Subscriber()
    injection_thread = QtCore.QThread()
    injection = PixhawkRtcmInjectionWorker(subscriber, max_queue=96)
    injection.moveToThread(injection_thread)
    pipeline.injection_requested.connect(
        injection.submit_envelope,
        QtCore.Qt.DirectConnection,
    )
    injection.injected.connect(pipeline.injection_result_received)
    pipeline.finished.connect(injection_thread.quit)

    ntrip_thread = QtCore.QThread()
    ntrip = NtripNetworkWorker(config=NetworkConfig())
    ntrip.moveToThread(ntrip_thread)
    ntrip_thread.started.connect(ntrip.run)
    ntrip.finished.connect(ntrip_thread.quit)
    GnssRuntimeController(object())._wire_rtcm_ntrip_source(ntrip, pipeline)

    pipeline_thread.start()
    injection_thread.start()
    pipeline.configure_requested.emit(
        {
            "ntrip_targets": ("left", "right"),
            "base_targets": (),
            "forward_ntrip_to_base": False,
            "base_serial_available": False,
        }
    )
    pipeline.evidence_context_requested.emit(
        {
            "state": "COLLECTING",
            "session_id": "e2e-session",
            "session_root": str(tmp_path / "e2e-session"),
        }
    )
    assert _process_until(
        app,
        lambda: bool(snapshots)
        and bool(snapshots[-1]["evidence_context"]["active"]),
    )

    ntrip_thread.start()
    assert _process_until(app, lambda: not ntrip_thread.isRunning())
    pipeline.stop_requested.emit()
    assert _process_until(app, lambda: not pipeline_thread.isRunning())
    assert _process_until(app, lambda: not injection_thread.isRunning())

    assert len(subscriber.sent) == 40
    assert subscriber.sent[0] == ("left", RTCM_1077)
    assert subscriber.sent[1] == ("right", RTCM_1077)
    assert subscriber.sent[-2:] == [("left", RTCM_1077), ("right", RTCM_1077)]
    root = tmp_path / "e2e-session" / "raw" / "gnss"
    assert (root / "ntrip_transport_chunk.rtcm").read_bytes() == RTCM_1077 * 20
    assert (root / "ntrip_rtcm_frame.rtcm").read_bytes() == RTCM_1077 * 20

    integrity = snapshots[-1]["integrity"]
    assert integrity["records_received"] == 40
    assert integrity["evidence_expected"] == 40
    assert integrity["evidence_persisted"] == 40
    assert integrity["evidence_failed"] == 0
    assert integrity["injection_expected"] == 20
    assert integrity["injection_outcomes"] == 20
    assert integrity["injection_pending"] == 0
    assert integrity["injection_unconfirmed"] == 0
    # Configure, evidence-context, first-data, and final drain snapshots are
    # bounded; no raw chunk or injection result produces one GUI event each.
    assert len(snapshots) <= 5
