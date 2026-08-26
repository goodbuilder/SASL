from __future__ import annotations

from contextlib import contextmanager
import json
import sys
import threading
import time
from types import SimpleNamespace

from PyQt5 import QtCore, QtWidgets

from Workstation.device_runtime_workers import PixhawkRtcmInjectionWorker
from Workstation.device_runtime_controller import DeviceRuntimeController
from Workstation.gnss_runtime_controller import GnssRuntimeController
from Workstation.gnss_runtime_workers import RtcmDataPlaneWorker
from Workstation.gnss_runtime_workers import GnssBaseSerialWorker, NtripNetworkWorker
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
        return 2


def _health(
    state: str,
    *,
    eligible: bool,
    generation: int,
    age_s: float,
) -> SimpleNamespace:
    return SimpleNamespace(
        state=SimpleNamespace(value=state),
        rtcm_eligible=eligible,
        connection_generation=generation,
        any_rx_age_s=age_s,
        stall_reason=("ALL_EXPECTED_RX_SILENT" if state == "ETH MAVLINK RX STALLED" else ""),
    )


class _HealthSubscriber:
    def __init__(self, health_by_side: dict[str, SimpleNamespace]) -> None:
        self.health_by_side = health_by_side
        self.sent: list[tuple[str, bytes]] = []

    def mavlink_health_snapshot(self, side: str, **_kwargs):
        return self.health_by_side[side]

    def send_rtcm_data_if_eligible(
        self,
        side: str,
        payload: bytes,
        *,
        expected_generation: int | None = None,
    ) -> dict[str, object]:
        health = self.health_by_side[side]
        if expected_generation is not None and health.connection_generation != expected_generation:
            return {
                "sent": False,
                "packet_count": 0,
                "health": health,
                "block_reason": "connection-generation-changed",
            }
        if not health.rtcm_eligible:
            return {
                "sent": False,
                "packet_count": 0,
                "health": health,
                "block_reason": "target-mavlink-ineligible",
            }
        self.sent.append((side, bytes(payload)))
        return {"sent": True, "packet_count": 2, "health": health, "block_reason": ""}


def test_gnss_base_serial_write_timeout_fails_and_finishes(monkeypatch) -> None:
    captured: dict[str, object] = {}

    class SerialPort:
        def __init__(self, port, baud, **kwargs) -> None:
            captured.update(port=port, baud=baud, **kwargs)

        def write(self, _payload: bytes) -> int:
            raise TimeoutError("blocked writer sentinel")

        def read(self, _size: int) -> bytes:
            return b""

        def close(self) -> None:
            captured["closed"] = True

    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=SerialPort))
    worker = GnssBaseSerialWorker(port="/dev/fake", baud=115200)
    failures: list[str] = []
    finished: list[bool] = []
    worker.failed.connect(failures.append)
    worker.finished.connect(lambda: finished.append(True))

    worker.run()

    assert captured["write_timeout"] == worker.WRITE_TIMEOUT_S
    assert failures == ["blocked writer sentinel"]
    assert finished == [True]
    assert captured["closed"] is True


def test_gnss_base_serial_write_queue_is_bounded_and_keeps_newest() -> None:
    worker = GnssBaseSerialWorker(port="/dev/fake", baud=115200)
    total = worker.MAX_WRITE_QUEUE + 10
    for index in range(total):
        worker.send_bytes(f"item-{index}", bytes((index % 256,)))

    queued = []
    while not worker._write_queue.empty():
        queued.append(worker._write_queue.get_nowait())

    assert len(queued) == worker.MAX_WRITE_QUEUE
    assert queued[0][0] == "item-10"
    assert queued[-1][0] == f"item-{total - 1}"


def test_rtcm_worker_replacement_never_waits_on_the_gui_thread() -> None:
    callbacks: list[object] = []
    calls: list[str] = []

    class Signal:
        def connect(self, callback, *_args) -> None:
            callbacks.append(callback)

    class Thread:
        finished = Signal()

        def isRunning(self) -> bool:  # noqa: N802
            return True

        def quit(self) -> None:
            calls.append("quit")

        def wait(self, _timeout: int) -> bool:
            raise AssertionError("GUI path must not call QThread.wait")

    old_subscriber = object()
    new_subscriber = object()
    thread = Thread()
    panel = SimpleNamespace(
        mavlink_pixhawk=new_subscriber,
        pixhawk_rtcm_thread=thread,
        pixhawk_rtcm_worker=SimpleNamespace(
            subscriber=old_subscriber,
            clear_pending=lambda: calls.append("clear"),
        ),
        rtcm_pipeline_worker=None,
        _rtcm_pipeline_injection_worker=None,
    )

    assert GnssRuntimeController(panel)._ensure_pixhawk_rtcm_worker() is None
    assert calls == ["clear", "quit"]
    assert callbacks


def test_injection_worker_echoes_pipeline_identity_with_delivery_result(monkeypatch) -> None:
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers.QtCore.QTimer, "singleShot", lambda *_args: None)
    monkeypatch.setattr(device_runtime_workers.time, "monotonic", lambda: 42.5)
    subscriber = _Subscriber()
    results: list[dict[str, object]] = []
    worker = PixhawkRtcmInjectionWorker(subscriber, max_injection_age_s=0.0)
    worker.injected.connect(results.append)

    worker.enqueue_envelope(
        {
            "pipeline_sequence": 17,
            "source": "ntrip",
            "targets": ("left", "right"),
            "payload": RTCM_1077,
            "payload_sha256": "fixture-sha",
            "receive_monotonic_ns": 123,
            "message_type": 1077,
        }
    )

    assert subscriber.sent == [("left", RTCM_1077), ("right", RTCM_1077)]
    assert results == [
        {
            "payload_bytes": len(RTCM_1077),
            "sent": (("left", 2), ("right", 2)),
            "failed": (),
            "dropped": 0,
            "pipeline_sequence": 17,
            "source": "ntrip",
            "targets": ("left", "right"),
            "payload_sha256": "fixture-sha",
            "receive_monotonic_ns": 123,
            "message_type": 1077,
            "delivery_monotonic_s_by_target": {"left": 42.5, "right": 42.5},
            "last_delivery_s": 42.5,
        }
    ]


def test_injection_worker_blocks_only_stalled_side_and_preserves_live_side(monkeypatch) -> None:
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers.QtCore.QTimer, "singleShot", lambda *_args: None)
    subscriber = _HealthSubscriber(
        {
            "left": _health(
                "ETH MAVLINK RX STALLED",
                eligible=False,
                generation=3,
                age_s=12.5,
            ),
            "right": _health("QUALIFYING", eligible=True, generation=8, age_s=0.1),
        }
    )
    results: list[dict[str, object]] = []
    worker = PixhawkRtcmInjectionWorker(subscriber, max_injection_age_s=0.0)
    worker.injected.connect(results.append)

    worker.enqueue(("left", "right"), RTCM_1077)

    assert subscriber.sent == [("right", RTCM_1077)]
    result = results[0]
    assert result["sent"] == (("right", 2),)
    assert result["failed"] == ()
    assert result["dropped"] == 0
    assert result["blocked_stall"] == 1
    assert result["blocked_generation"] == 0
    assert result["drop_reason"] == "target-mavlink-stalled"
    assert result["dropped_targets"] == ("left",)
    assert result["terminal_outcome"] == "partial-blocked"
    assert result["target_rx_age_s"] == {"left": 12.5}
    assert result["target_connection_generations"] == {"left": 3}


def test_injection_worker_rechecks_liveness_after_enqueue_before_send(monkeypatch) -> None:
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers.QtCore.QMetaObject, "invokeMethod", lambda *_args: None)
    subscriber = _HealthSubscriber(
        {
            "left": _health("QUALIFYING", eligible=True, generation=1, age_s=0.1),
            "right": _health("QUALIFYING", eligible=True, generation=2, age_s=0.1),
        }
    )
    results: list[dict[str, object]] = []
    worker = PixhawkRtcmInjectionWorker(subscriber, max_injection_age_s=0.0)
    worker.injected.connect(results.append)
    worker.submit_envelope(
        {"pipeline_sequence": 31, "targets": ("left", "right"), "payload": RTCM_1077}
    )

    subscriber.health_by_side["left"] = _health(
        "ETH MAVLINK RX STALLED",
        eligible=False,
        generation=1,
        age_s=10.0,
    )
    worker._drain_one()

    assert subscriber.sent == [("right", RTCM_1077)]
    assert results[0]["pipeline_sequence"] == 31
    assert results[0]["blocked_stall"] == 1
    assert results[0]["sent"] == (("right", 2),)


def test_injection_worker_rejects_stale_connection_generation(monkeypatch) -> None:
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers.QtCore.QMetaObject, "invokeMethod", lambda *_args: None)
    subscriber = _HealthSubscriber(
        {"left": _health("QUALIFYING", eligible=True, generation=4, age_s=0.1)}
    )
    results: list[dict[str, object]] = []
    worker = PixhawkRtcmInjectionWorker(subscriber, max_injection_age_s=0.0)
    worker.injected.connect(results.append)
    worker.submit_envelope(
        {"pipeline_sequence": 32, "targets": ("left",), "payload": RTCM_1077}
    )

    subscriber.health_by_side["left"] = _health(
        "QUALIFYING",
        eligible=True,
        generation=5,
        age_s=0.0,
    )
    worker._drain_one()

    assert subscriber.sent == []
    assert results[0]["blocked_generation"] == 1
    assert results[0]["blocked_stall"] == 0
    assert results[0]["drop_reason"] == "target-mavlink-generation-changed"
    assert results[0]["terminal_outcome"] == "blocked"


def test_rtcm_send_does_not_reset_mavlink_qualification(monkeypatch) -> None:
    from Workstation import device_runtime_workers
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    monkeypatch.setattr(device_runtime_workers.time, "monotonic", lambda: clock["value"])
    monkeypatch.setattr(device_runtime_workers.QtCore.QTimer, "singleShot", lambda *_args: None)

    sent_packets: list[tuple[int, int]] = []
    connection = SimpleNamespace(
        mav=SimpleNamespace(
            gps_rtcm_data_send=lambda flags, length, _data: sent_packets.append((flags, length))
        )
    )
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
    )
    subscriber.connections["left"] = connection
    model = subscriber._ensure_liveness_listener("left")
    for message_type in ("HEARTBEAT", "ATTITUDE"):
        assert model.observe_packet(
            system_id=1,
            component_id=1,
            message_type=message_type,
            received_at_s=0.0,
            generation=model.connection_generation,
        )
    before = subscriber.mavlink_health_snapshot("left", now_s=0.0)

    clock["value"] = 1.0
    worker = PixhawkRtcmInjectionWorker(subscriber, max_injection_age_s=0.0)
    worker.enqueue(("left",), RTCM_1077)
    after = subscriber.mavlink_health_snapshot("left", now_s=1.0)

    assert sent_packets == [(0, len(RTCM_1077))]
    assert before.qualification_started_monotonic_s == 0.0
    assert after.qualification_started_monotonic_s == 0.0
    assert after.active_command is None
    assert after.qualification_elapsed_s == 1.0


def test_injection_worker_rechecks_age_immediately_before_device_send(monkeypatch) -> None:
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers.QtCore.QTimer, "singleShot", lambda *_args: None)
    monkeypatch.setattr(
        device_runtime_workers,
        "time",
        SimpleNamespace(monotonic=lambda: 10.0, monotonic_ns=lambda: 10_000_000_000),
        raising=False,
    )
    subscriber = _Subscriber()
    results: list[dict[str, object]] = []
    worker = PixhawkRtcmInjectionWorker(
        subscriber,
        max_injection_age_s=0.5,
    )
    worker.injected.connect(results.append)

    worker.enqueue_envelope(
        {
            "pipeline_sequence": 19,
            "source": "base",
            "targets": ("left", "right"),
            "payload": RTCM_1077,
            "payload_sha256": "fixture-sha",
            "receive_monotonic_ns": 9_000_000_000,
            "message_type": 1077,
        }
    )

    assert subscriber.sent == []
    assert results == [
        {
            "payload_bytes": len(RTCM_1077),
            "sent": (),
            "failed": (),
            "dropped": 1,
            "pipeline_sequence": 19,
            "source": "base",
            "targets": ("left", "right"),
            "payload_sha256": "fixture-sha",
            "receive_monotonic_ns": 9_000_000_000,
            "message_type": 1077,
            "drop_reason": "stale-before-send",
            "age_s": 1.0,
            "dropped_targets": ("left", "right"),
            "terminal_outcome": "stale",
        }
    ]


def test_injection_worker_rechecks_freshness_between_targets(monkeypatch) -> None:
    from Workstation import device_runtime_workers

    monotonic_ns_values = iter((9_100_000_000, 10_000_000_000))
    monkeypatch.setattr(device_runtime_workers.QtCore.QTimer, "singleShot", lambda *_args: None)
    monkeypatch.setattr(
        device_runtime_workers,
        "time",
        SimpleNamespace(
            monotonic=lambda: 9.2,
            monotonic_ns=lambda: next(monotonic_ns_values),
        ),
    )
    subscriber = _Subscriber()
    results: list[dict[str, object]] = []
    worker = PixhawkRtcmInjectionWorker(subscriber, max_injection_age_s=0.5)
    worker.injected.connect(results.append)

    worker.enqueue_envelope(
        {
            "pipeline_sequence": 20,
            "source": "base",
            "targets": ("left", "right"),
            "payload": RTCM_1077,
            "receive_monotonic_ns": 9_000_000_000,
        }
    )

    assert subscriber.sent == [("left", RTCM_1077)]
    assert results[0]["sent"] == (("left", 2),)
    assert results[0]["dropped_targets"] == ("right",)
    assert results[0]["terminal_outcome"] == "partial-stale"
    assert results[0]["last_delivery_s"] == 9.2


def test_injection_queue_reports_exact_stale_envelope_while_already_draining(
    monkeypatch,
) -> None:
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers.QtCore.QTimer, "singleShot", lambda *_args: None)
    worker = PixhawkRtcmInjectionWorker(_Subscriber(), max_queue=1)
    results: list[dict[str, object]] = []
    worker.injected.connect(results.append)

    # First request is delivered immediately and leaves the test worker in its
    # scheduled-drain state because singleShot is intentionally held.
    worker.enqueue_envelope(
        {"pipeline_sequence": 1, "source": "base", "targets": ("left",), "payload": b"one"}
    )
    worker.enqueue_envelope(
        {"pipeline_sequence": 2, "source": "base", "targets": ("left",), "payload": b"two"}
    )
    worker.enqueue_envelope(
        {"pipeline_sequence": 3, "source": "base", "targets": ("left",), "payload": b"three"}
    )

    drop = results[-1]
    assert drop["dropped"] == 1
    assert drop["dropped_envelopes"] == (
        {"pipeline_sequence": 2, "source": "base", "targets": ("left",)},
    )


def test_retiring_injection_worker_emits_terminal_outcome_for_queued_backlog(
    monkeypatch,
) -> None:
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers.QtCore.QTimer, "singleShot", lambda *_args: None)
    subscriber = _Subscriber()
    worker = PixhawkRtcmInjectionWorker(subscriber, max_injection_age_s=0.0)
    results: list[dict[str, object]] = []
    worker.injected.connect(results.append)
    worker.enqueue_envelope(
        {
            "pipeline_sequence": 1,
            "source": "base",
            "targets": ("left",),
            "payload": b"first",
        }
    )
    worker.enqueue_envelope(
        {
            "pipeline_sequence": 2,
            "source": "base",
            "targets": ("left", "right"),
            "payload": b"queued",
            "evidence_context_token": "measurement-a:r0001",
        }
    )

    outcomes = worker.clear_pending(reason="subscriber-replaced")

    assert subscriber.sent == [("left", b"first")]
    assert outcomes == (
        {
            "payload_bytes": len(b"queued"),
            "sent": (),
            "failed": (),
            "dropped": 1,
            "pipeline_sequence": 2,
            "source": "base",
            "targets": ("left", "right"),
            "evidence_context_token": "measurement-a:r0001",
            "drop_reason": "subscriber-replaced",
            "dropped_targets": ("left", "right"),
            "terminal_outcome": "cancelled",
        },
    )
    assert results[-1] == outcomes[0]


def test_direct_injection_handoff_is_bounded_before_qt_event_queue() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    subscriber = _Subscriber()
    thread = QtCore.QThread()
    worker = PixhawkRtcmInjectionWorker(subscriber, max_queue=2)
    worker.moveToThread(thread)
    results: list[dict[str, object]] = []
    worker.injected.connect(results.append)

    for sequence in range(1, 6):
        worker.submit_envelope(
            {
                "pipeline_sequence": sequence,
                "source": "ntrip",
                "targets": ("left",),
                "payload": bytes((sequence,)),
            }
        )

    dropped_sequences = [
        int(envelope["pipeline_sequence"])
        for result in results
        for envelope in tuple(result.get("dropped_envelopes") or ())
    ]
    assert dropped_sequences == [1, 2, 3]

    thread.start()
    deadline = time.monotonic() + 2.0
    while len(subscriber.sent) < 2 and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    thread.quit()
    assert thread.wait(1000)
    assert subscriber.sent == [("left", b"\x04"), ("left", b"\x05")]


def test_data_plane_worker_coalesces_snapshots_but_emits_every_injection_request() -> None:
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0)
    worker.configure(
        {
            "ntrip_targets": ("left", "right"),
            "base_targets": (),
            "forward_ntrip_to_base": False,
            "base_serial_available": False,
        }
    )
    snapshots: list[dict[str, object]] = []
    injections: list[dict[str, object]] = []
    worker.snapshot_ready.connect(snapshots.append)
    worker.injection_requested.connect(injections.append)

    # Call the slots directly for this narrow cadence test. Thread-affinity is
    # covered separately by the controller integration wiring tests.
    worker.handle_ntrip_chunk(RTCM_1077)
    worker.handle_ntrip_chunk(RTCM_1077)

    assert len(injections) == 2
    assert [request["pipeline_sequence"] for request in injections] == [2, 4]
    assert snapshots == []
    for request in injections:
        worker.handle_injection_result(
            {
                "pipeline_sequence": request["pipeline_sequence"],
                "payload_bytes": request["payload_bytes"],
                "sent": (("left", 1), ("right", 1)),
                "failed": (),
                "dropped": 0,
            }
        )
    worker.request_stop()
    assert len(snapshots) == 1
    assert snapshots[0]["frames"] == 2
    assert snapshots[0]["integrity"]["records_received"] == 4


def test_data_plane_ingress_is_processed_on_its_owner_thread() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    thread = QtCore.QThread()
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0)
    worker.moveToThread(thread)
    worker.finished.connect(thread.quit)
    seen_threads: list[QtCore.QThread] = []
    requests: list[dict[str, object]] = []

    def capture(request: object) -> None:
        seen_threads.append(QtCore.QThread.currentThread())
        requests.append(dict(request))

    worker.injection_requested.connect(capture, QtCore.Qt.DirectConnection)
    thread.start()
    worker.configure_requested.emit(
        {
            "ntrip_targets": ("left", "right"),
            "base_targets": (),
            "forward_ntrip_to_base": False,
            "base_serial_available": False,
        }
    )
    worker.submit_ntrip_envelope(
        {
            "payload": RTCM_1077,
            "receive_utc_ns": time.time_ns(),
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )

    deadline = time.monotonic() + 2.0
    while not requests and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)

    assert len(requests) == 1
    assert seen_threads == [thread]
    assert seen_threads[0] is not QtCore.QThread.currentThread()

    worker.injection_result_received.emit(
        {
            "pipeline_sequence": requests[0]["pipeline_sequence"],
            "payload_bytes": requests[0]["payload_bytes"],
            "sent": (("left", 1), ("right", 1)),
            "failed": (),
            "dropped": 0,
        }
    )
    worker.stop_requested.emit()
    deadline = time.monotonic() + 2.0
    while thread.isRunning() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    assert thread.wait(1000)


def test_stale_injection_is_not_sent_but_remains_integrity_accounted() -> None:
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0, max_injection_age_s=0.1)
    worker.configure(
        {
            "ntrip_targets": (),
            "base_targets": ("left",),
            "forward_ntrip_to_base": False,
            "base_serial_available": False,
        }
    )
    injections: list[dict[str, object]] = []
    snapshots: list[dict[str, object]] = []
    worker.injection_requested.connect(injections.append)
    worker.snapshot_ready.connect(snapshots.append)
    worker.submit_base_envelope(
        {
            "payload": RTCM_1077,
            "receive_utc_ns": 10,
            "receive_monotonic_ns": 1,
        }
    )
    worker._drain_ingress()
    worker.request_stop()

    assert injections == []
    integrity = snapshots[-1]["integrity"]
    assert integrity["records_received"] == 1
    assert integrity["injection_dropped_stale"] == 1
    assert integrity["last_injection_drop"]["pipeline_sequence"] == 1
    assert integrity["last_injection_drop"]["reason"] == "stale-before-injection"


def test_successful_injection_records_last_delivery_monotonic_time() -> None:
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0)
    worker.configure({"base_targets": ("left",)})
    requests: list[dict[str, object]] = []
    worker.injection_requested.connect(requests.append)

    worker.handle_base_frame(RTCM_1077)
    worker.handle_injection_result(
        {
            "pipeline_sequence": requests[0]["pipeline_sequence"],
            "payload_bytes": len(RTCM_1077),
            "sent": (("left", 2),),
            "failed": (),
            "dropped": 0,
            "last_delivery_s": 123.25,
        }
    )

    snapshot = worker._plane.snapshot(now_monotonic_s=124.0)
    assert snapshot["injection"]["left"]["last_delivery_s"] == 123.25


def test_ingress_keeps_enqueued_evidence_context_after_session_switch(tmp_path) -> None:
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0)
    worker.configure({"base_targets": ()})
    root_a = tmp_path / "session-a"
    root_b = tmp_path / "session-b"
    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-a",
            "session_root": str(root_a),
            "context_token": "measurement-a:r0001",
        }
    )
    worker.submit_base_envelope(
        {
            "payload": RTCM_1077,
            "receive_utc_ns": 10,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )

    # Change the live context before the queued envelope is drained. Evidence
    # ownership must remain the immutable ingress snapshot, not session B.
    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-b",
            "session_root": str(root_b),
            "context_token": "measurement-b:r0001",
        }
    )
    worker._drain_ingress()

    binary_a = root_a / "raw" / "gnss" / "base_rtcm_frame.rtcm"
    index_a = binary_a.with_suffix(".rtcm.jsonl")
    assert binary_a.read_bytes() == RTCM_1077
    assert not root_b.exists()
    row = json.loads(index_a.read_text(encoding="utf-8"))
    assert row["session_id"] == "session-a"
    assert row["context_token"] == "measurement-a:r0001"


def test_evidence_drain_closes_old_token_then_accepts_new_measurement(
    tmp_path,
) -> None:
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0)
    worker.configure({"base_targets": ()})
    root_a = tmp_path / "measurement-a"
    root_b = tmp_path / "measurement-b"
    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-1",
            "session_root": str(root_a),
            "context_token": "measurement-a:r0001",
        }
    )
    drained: list[dict[str, object]] = []
    worker.evidence_drained.connect(drained.append)
    worker.submit_base_envelope(
        {
            "payload": RTCM_1077,
            "receive_utc_ns": 10,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )

    assert worker.close_evidence_context_and_drain("measurement-a:r0001") is True
    worker.submit_base_envelope(
        {
            "payload": b"POST-CLOSE",
            "receive_utc_ns": 11,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )
    worker._drain_ingress()

    assert len(drained) == 1
    assert drained[0]["context_token"] == "measurement-a:r0001"
    assert drained[0]["valid"] is True
    assert drained[0]["error"] == ""
    assert drained[0]["high_water"]["last_sequence"] == 1
    assert (
        root_a / "raw" / "gnss" / "base_rtcm_frame.rtcm"
    ).read_bytes() == RTCM_1077
    assert worker._accepting is True
    assert worker._finished is False

    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-1",
            "session_root": str(root_b),
            "context_token": "measurement-b:r0001",
        }
    )
    worker.submit_base_envelope(
        {
            "payload": b"NEW-MEASUREMENT",
            "receive_utc_ns": 12,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )
    worker._drain_ingress()
    assert (
        root_b / "raw" / "gnss" / "base_rtcm_frame.rtcm"
    ).read_bytes() == b"NEW-MEASUREMENT"


def test_evidence_drain_rejects_wrong_token_without_closing_new_owner(tmp_path) -> None:
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0)
    worker.configure({"base_targets": ()})
    root_b = tmp_path / "measurement-b"
    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-1",
            "session_root": str(root_b),
            "context_token": "measurement-b:r0001",
        }
    )
    drained: list[dict[str, object]] = []
    worker.evidence_drained.connect(drained.append)

    assert worker.close_evidence_context_and_drain("measurement-a:r0001") is False
    worker.submit_base_envelope(
        {
            "payload": RTCM_1077,
            "receive_utc_ns": 10,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )
    worker._drain_ingress()

    assert drained[0]["context_token"] == "measurement-a:r0001"
    assert drained[0]["valid"] is False
    assert "not the active ingress owner" in drained[0]["error"]
    assert (
        root_b / "raw" / "gnss" / "base_rtcm_frame.rtcm"
    ).read_bytes() == RTCM_1077


def test_package_snapshot_after_drain_cannot_reopen_old_context(tmp_path) -> None:
    token = "session-1:measurement-a:r0001"
    root = tmp_path / "measurement-a"
    panel = SimpleNamespace(
        _measurement_package_state="VERIFYING",
        recording_session_id="session-1",
        _measurement_id="measurement-a",
        _measurement_package_revision=1,
        _measurement_package_rtcm_drain_requested_token=token,
        _recording_session_dir=lambda _session_id: root,
    )
    closed_context = GnssRuntimeController(panel)._rtcm_evidence_context()
    assert closed_context.context_token == token
    assert closed_context.active is False

    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0)
    worker.configure({"base_targets": ()})
    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-1",
            "session_root": str(root),
            "context_token": token,
        }
    )
    worker.submit_base_envelope(
        {
            "payload": RTCM_1077,
            "receive_utc_ns": 10,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )
    assert worker.close_evidence_context_and_drain(token) is True
    worker._drain_ingress()
    before = (root / "raw" / "gnss" / "base_rtcm_frame.rtcm").read_bytes()

    # This is the package-changed sync that follows a valid ACK.
    worker.set_evidence_context(closed_context.as_dict())
    worker.submit_base_envelope(
        {
            "payload": b"MUST-NOT-APPEND",
            "receive_utc_ns": 11,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )
    worker._drain_ingress()
    assert (root / "raw" / "gnss" / "base_rtcm_frame.rtcm").read_bytes() == before


def test_evidence_drain_waits_for_associated_injection_terminal_outcome(
    tmp_path,
) -> None:
    worker = RtcmDataPlaneWorker(
        snapshot_interval_s=10.0,
        evidence_drain_timeout_s=1.0,
    )
    worker.configure({"base_targets": ("left",)})
    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-1",
            "session_root": str(tmp_path / "measurement-a"),
            "context_token": "measurement-a:r0001",
        }
    )
    requests: list[dict[str, object]] = []
    drained: list[dict[str, object]] = []
    worker.injection_requested.connect(requests.append)
    worker.evidence_drained.connect(drained.append)
    worker.submit_base_envelope(
        {
            "payload": RTCM_1077,
            "receive_utc_ns": 10,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )
    assert worker.close_evidence_context_and_drain("measurement-a:r0001") is True
    worker._drain_ingress()

    assert len(requests) == 1
    assert drained == []
    worker.handle_injection_result(
        {
            "pipeline_sequence": requests[0]["pipeline_sequence"],
            "payload_bytes": len(RTCM_1077),
            "sent": (("left", 1),),
            "failed": (),
            "dropped": 0,
            "last_delivery_s": 123.0,
        }
    )

    assert len(drained) == 1
    assert drained[0]["valid"] is True
    assert drained[0]["unconfirmed_injection_sequences"] == ()


def test_evidence_drain_timeout_marks_pending_injection_unconfirmed(tmp_path) -> None:
    worker = RtcmDataPlaneWorker(
        snapshot_interval_s=10.0,
        evidence_drain_timeout_s=1.0,
    )
    worker.configure({"base_targets": ("left",)})
    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-1",
            "session_root": str(tmp_path / "measurement-a"),
            "context_token": "measurement-a:r0001",
        }
    )
    drained: list[dict[str, object]] = []
    worker.evidence_drained.connect(drained.append)
    worker.submit_base_envelope(
        {
            "payload": RTCM_1077,
            "receive_utc_ns": 10,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )
    assert worker.close_evidence_context_and_drain("measurement-a:r0001") is True
    worker._drain_ingress()
    assert drained == []

    worker._pending_evidence_barriers[0].deadline_s = 0.0
    worker._poll_evidence_barriers()

    assert len(drained) == 1
    assert drained[0]["valid"] is False
    assert drained[0]["unconfirmed_injection_sequences"] == (1,)
    assert "did not drain" in drained[0]["error"]
    assert worker._plane.pending_injection_count == 0


def test_evidence_close_is_linearized_with_concurrent_ingress(tmp_path) -> None:
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0, max_ingress_queue=256)
    worker.configure({"base_targets": ()})
    root_a = tmp_path / "measurement-a"
    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-1",
            "session_root": str(root_a),
            "context_token": "measurement-a:r0001",
        }
    )
    worker.submit_base_envelope(
        {
            "payload": b"PRE",
            "receive_utc_ns": 9,
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )
    drained: list[dict[str, object]] = []
    worker.evidence_drained.connect(drained.append)
    start = threading.Barrier(2)
    closed = threading.Event()
    close_results: list[bool] = []

    def produce() -> None:
        start.wait()
        for _index in range(80):
            worker.submit_base_envelope(
                {
                    "payload": b"PRE",
                    "receive_utc_ns": 10,
                    "receive_monotonic_ns": time.monotonic_ns(),
                }
            )
            if closed.is_set():
                break
        closed.wait(1.0)
        for _index in range(10):
            worker.submit_base_envelope(
                {
                    "payload": b"POST",
                    "receive_utc_ns": 11,
                    "receive_monotonic_ns": time.monotonic_ns(),
                }
            )

    def close() -> None:
        start.wait()
        close_results.append(
            worker.close_evidence_context_and_drain("measurement-a:r0001")
        )
        closed.set()

    producer = threading.Thread(target=produce)
    closer = threading.Thread(target=close)
    producer.start()
    closer.start()
    producer.join(2.0)
    closer.join(2.0)
    assert not producer.is_alive() and not closer.is_alive()
    assert close_results == [True]

    while not worker._ingress.empty():
        worker._drain_ingress()

    binary = root_a / "raw" / "gnss" / "base_rtcm_frame.rtcm"
    assert b"POST" not in binary.read_bytes()
    assert len(drained) == 1 and drained[0]["valid"] is True


def test_pixhawk_worker_is_replaced_when_subscriber_identity_changes() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = QtCore.QObject()
    subscriber_a = _Subscriber()
    subscriber_b = _Subscriber()
    panel.mavlink_pixhawk = subscriber_a
    panel.pixhawk_rtcm_thread = None
    panel.pixhawk_rtcm_worker = None
    panel.rtcm_pipeline_worker = None
    panel._refresh_gnss_base_rtcm_stats = lambda: None
    panel._maybe_activate_gnss_fixed_base = lambda: None
    panel._refresh_gnss_operation_readiness = lambda: None
    controller = GnssRuntimeController(panel)
    controller._ensure_rtcm_pipeline = lambda: None
    controller._wire_rtcm_pipeline_injection_worker = lambda _worker: None
    panel._clear_pixhawk_rtcm_worker = controller._clear_pixhawk_rtcm_worker

    threads: list[QtCore.QThread] = []
    try:
        first = controller._ensure_pixhawk_rtcm_worker()
        first_thread = panel.pixhawk_rtcm_thread
        assert first is not None and first_thread is not None
        threads.append(first_thread)

        panel.mavlink_pixhawk = subscriber_b
        replacement_started_s = time.monotonic()
        second = controller._ensure_pixhawk_rtcm_worker()
        assert second is None
        assert time.monotonic() - replacement_started_s < 0.05
        deadline_s = time.monotonic() + 2.0
        while time.monotonic() < deadline_s:
            app.processEvents()
            second = panel.pixhawk_rtcm_worker
            if second is not None and second is not first:
                break
            time.sleep(0.005)
        second_thread = panel.pixhawk_rtcm_thread
        assert second is not None and second_thread is not None
        threads.append(second_thread)

        assert second is not first
        assert second.subscriber is subscriber_b
        assert second_thread is not first_thread
        assert not first_thread.isRunning()
        controller._clear_pixhawk_rtcm_worker(first_thread, first)
        assert panel.pixhawk_rtcm_thread is second_thread
        assert panel.pixhawk_rtcm_worker is second
    finally:
        for thread in dict.fromkeys(threads):
            if thread.isRunning():
                thread.quit()
                thread.wait(1000)
        app.processEvents()


def test_full_pixhawk_disconnect_retires_subscriber_bound_rtcm_worker() -> None:
    subscriber = SimpleNamespace(connections={})
    retired: list[tuple[object, bool]] = []
    runtime = SimpleNamespace(
        _retire_pixhawk_rtcm_worker=lambda *, expected_subscriber, wait: retired.append(
            (expected_subscriber, wait)
        )
    )
    label = SimpleNamespace(setText=lambda _text: None)
    panel = SimpleNamespace(
        mavlink_pixhawk=subscriber,
        pixhawk_connection_thread=None,
        pixhawk_attitude_times={"left": []},
        _px4_calibration_auto_refreshed_sides=set(),
        _gnss_runtime_controller=lambda: runtime,
        _clear_px4_mag_manager_connection_state=lambda _camera: None,
        _clear_px4_attitude_cal_connection_state=lambda _camera: None,
        _refresh_session_toggle_buttons=lambda: None,
        _refresh_mavlink_side_actions=lambda: None,
        _px4_mag_camera_label=lambda side: side,
        pixhawk_connection_label=label,
        _log=lambda _message: None,
        _refresh_operation_header=lambda: None,
    )

    DeviceRuntimeController(panel)._finish_pixhawk_disconnect(None, ("left",))

    assert panel.mavlink_pixhawk is None
    assert retired == [(subscriber, False)]


def test_evidence_write_failure_is_fail_closed_and_reports_sequence(tmp_path) -> None:
    bad_root = tmp_path / "not-a-directory"
    bad_root.write_text("sentinel")
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0)
    worker.configure({"base_targets": ()})
    worker.set_evidence_context(
        {
            "state": "COLLECTING",
            "session_id": "session-1",
            "session_root": str(bad_root),
        }
    )
    failures: list[dict[str, object]] = []
    snapshots: list[dict[str, object]] = []
    worker.integrity_failed.connect(failures.append)
    worker.snapshot_ready.connect(snapshots.append)

    worker.handle_base_frame(RTCM_1077)
    worker.request_stop()

    assert len(failures) == 1
    assert failures[0]["sequence"] == 1
    assert failures[0]["payload_sha256"]
    integrity = snapshots[-1]["integrity"]
    assert integrity["evidence_expected"] == 1
    assert integrity["evidence_persisted"] == 0
    assert integrity["evidence_failed"] == 1


def test_pipeline_stop_drains_every_queued_record_before_thread_finishes(tmp_path) -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    thread = QtCore.QThread()
    worker = RtcmDataPlaneWorker(snapshot_interval_s=10.0, max_ingress_queue=256)
    worker.moveToThread(thread)
    worker.finished.connect(thread.quit)
    final_snapshots: list[dict[str, object]] = []
    worker.snapshot_ready.connect(final_snapshots.append)
    thread.start()
    worker.configure_requested.emit({"ntrip_targets": ()})
    worker.evidence_context_requested.emit(
        {
            "state": "COLLECTING",
            "session_id": "drain-session",
            "session_root": str(tmp_path / "drain-session"),
        }
    )

    for sequence in range(100):
        worker.submit_ntrip_envelope(
            {
                "payload": RTCM_1077,
                "receive_utc_ns": 1_000 + sequence,
                "receive_monotonic_ns": time.monotonic_ns(),
            }
        )
    worker.stop_requested.emit()

    # Each of the 100 transport chunks produces two evidence records and each
    # binary/index pair is independently fsync'd. Under concurrent full-suite
    # disk pressure this durable drain can legitimately exceed five seconds;
    # the contract is lossless completion, not a soft UI cadence deadline.
    deadline = time.monotonic() + 15.0
    while thread.isRunning() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    assert thread.wait(1000)

    root = tmp_path / "drain-session" / "raw" / "gnss"
    assert (root / "ntrip_transport_chunk.rtcm").read_bytes() == RTCM_1077 * 100
    assert (root / "ntrip_rtcm_frame.rtcm").read_bytes() == RTCM_1077 * 100
    transport_rows = (root / "ntrip_transport_chunk.rtcm.jsonl").read_text().splitlines()
    frame_rows = (root / "ntrip_rtcm_frame.rtcm.jsonl").read_text().splitlines()
    assert len(transport_rows) == 100
    assert len(frame_rows) == 100
    integrity = final_snapshots[-1]["integrity"]
    assert integrity["records_received"] == 200
    assert integrity["evidence_expected"] == 200
    assert integrity["evidence_persisted"] == 200
    assert integrity["evidence_failed"] == 0


def test_controller_wires_only_coalesced_envelopes_not_legacy_raw_gui_signals() -> None:
    controller = GnssRuntimeController(object())
    pipeline = RtcmDataPlaneWorker()
    ntrip = NtripNetworkWorker(config=NetworkConfig())
    base = GnssBaseSerialWorker(port="/dev/null", baud=115200)

    controller._wire_rtcm_ntrip_source(ntrip, pipeline)
    controller._wire_rtcm_base_source(base, pipeline)

    assert ntrip.receivers(ntrip.chunk_received) == 0
    assert ntrip.receivers(ntrip.stats_changed) == 0
    assert ntrip.receivers(ntrip.chunk_envelope_received) == 1
    assert ntrip.receivers(ntrip.stats_snapshot_changed) == 1
    assert base.receivers(base.rtcm_frame_received) == 0
    assert base.receivers(base.rtcm_frame_envelope_received) == 1


def test_shutdown_timeout_marks_missing_injection_outcome_unconfirmed() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    thread = QtCore.QThread()
    worker = RtcmDataPlaneWorker(
        snapshot_interval_s=10.0,
        stop_drain_timeout_s=0.05,
    )
    worker.moveToThread(thread)
    worker.finished.connect(thread.quit)
    requests: list[dict[str, object]] = []
    failures: list[dict[str, object]] = []
    snapshots: list[dict[str, object]] = []
    worker.injection_requested.connect(requests.append)
    worker.integrity_failed.connect(failures.append)
    worker.snapshot_ready.connect(snapshots.append)
    thread.start()
    worker.configure_requested.emit({"base_targets": ("left",)})
    worker.submit_base_envelope(
        {
            "payload": RTCM_1077,
            "receive_utc_ns": time.time_ns(),
            "receive_monotonic_ns": time.monotonic_ns(),
        }
    )
    deadline = time.monotonic() + 2.0
    while not requests and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    assert [request["pipeline_sequence"] for request in requests] == [1]

    worker.stop_requested.emit()
    deadline = time.monotonic() + 2.0
    while thread.isRunning() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    assert thread.wait(1000)
    assert failures[-1]["unconfirmed_sequences"] == (1,)
    integrity = snapshots[-1]["integrity"]
    assert integrity["injection_pending"] == 0
    assert integrity["injection_unconfirmed"] == 1


def test_panel_owned_pipeline_starts_and_stops_without_leaking_qthread(
    monkeypatch,
    tmp_path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui, "APP_ROOT", tmp_path)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel()
    controller = panel._gnss_runtime_controller()
    received_drains: list[dict[str, object]] = []
    panel._handle_measurement_rtcm_evidence_drained = received_drains.append

    worker = controller._ensure_rtcm_pipeline()
    thread = panel.rtcm_pipeline_thread
    assert worker is panel.rtcm_pipeline_worker
    assert thread is not None and thread.isRunning()
    sentinel = {"schema": "workstation-rtcm-evidence-drain-v1", "context_token": "sentinel"}
    worker.evidence_drained.emit(sentinel)
    deadline = time.monotonic() + 1.0
    while not received_drains and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    assert received_drains == [sentinel]
    assert controller._stop_rtcm_pipeline() is False

    deadline = time.monotonic() + 2.0
    while panel.rtcm_pipeline_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    assert panel.rtcm_pipeline_thread is None
    assert panel.rtcm_pipeline_worker is None
    assert controller._stop_rtcm_pipeline() is True
    panel.deleteLater()
    app.processEvents()
