from __future__ import annotations

import hashlib
import multiprocessing
import threading
import time
from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtGui
import pytest

from Workstation.workstation_core.native_evidence_writer import (
    AsyncNativeEvidenceWriter,
    EVIDENCE_FRAME_UNAVAILABLE,
    reserve_ring_pair,
)
from Workstation.workstation_core.native_frame_ring import SharedNativeFrameRing
from Workstation.workstation_core.native_frame_store import NativeFrameStore
from Workstation.recording_controller import RecordingController


def _ring(side: str, *, generation: int = 3) -> SharedNativeFrameRing:
    return SharedNativeFrameRing.create(
        camera_id=side,
        stream_generation=generation,
        capacity=2,
        max_width=2,
        max_height=1,
        channels=3,
        mp_context=multiprocessing.get_context("spawn"),
    )


def _await_result(writer: AsyncNativeEvidenceWriter, request_id: str):
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        result = writer.result(request_id)
        if result is not None:
            return result
        time.sleep(0.01)
    raise AssertionError("native evidence result timeout")


def test_writer_pins_exact_pair_and_acknowledges_only_after_hash_persistence(
    tmp_path: Path,
) -> None:
    left = _ring("left")
    right = _ring("right")
    writer = AsyncNativeEvidenceWriter(tmp_path, max_pending=2)
    try:
        left_descriptor = left.publish_bytes(
            bytes([0, 0, 255, 0, 0, 255]),
            width=2,
            height=1,
            channels=3,
            timestamp_s=1.0,
        )
        right_descriptor = right.publish_bytes(
            bytes([255, 0, 0, 255, 0, 0]),
            width=2,
            height=1,
            channels=3,
            timestamp_s=1.01,
        )
        scheduled = writer.enqueue_ring_pair(
            request_id="request-1",
            pair_id="pair-1",
            purpose="accepted-measurement",
            frames=(
                ("left", left, left_descriptor.key, left_descriptor.to_mapping()),
                ("right", right, right_descriptor.key, right_descriptor.to_mapping()),
            ),
            context={"measurement_id": "m1"},
        )
        assert scheduled["evidence_state"] == "WRITING"
        assert left.metrics()["active_leases"] == 1
        assert right.metrics()["active_leases"] == 1

        result = _await_result(writer, "request-1")
        fields = result["fields"]
        assert fields["native_pair_status"] == "complete"
        assert fields["evidence_state"] == "SAVED"
        assert fields["evidence_artifact_count"] == 2
        assert fields["evidence_ack_sha256"]
        assert left.metrics()["active_leases"] == 0
        assert right.metrics()["active_leases"] == 0
        assert writer.pending_count == 0

        for side in ("left", "right"):
            path = tmp_path / str(fields[f"native_{side}_png_path"])
            assert path.is_file()
            payload = path.read_bytes()
            assert hashlib.sha256(payload).hexdigest() == fields[
                f"native_{side}_png_sha256"
            ]
            decoded = QtGui.QImage(str(path))
            assert not decoded.isNull()
            assert (decoded.width(), decoded.height()) == (2, 1)

        writer.acknowledge("request-1")
        assert writer.result("request-1") is None
        assert writer.close() is True
    finally:
        left.close()
        left.unlink()
        right.close()
        right.unlink()


def test_writer_rejects_pair_if_either_exact_frame_was_overwritten(
    tmp_path: Path,
) -> None:
    left = _ring("left")
    right = _ring("right")
    writer = AsyncNativeEvidenceWriter(tmp_path)
    try:
        left_descriptor = left.publish_bytes(
            bytes([1]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=1.0,
        )
        right_descriptor = right.publish_bytes(
            bytes([2]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=1.0,
        )
        # Capacity two: overwrite the selected right frame with two newer ones.
        for value in (3, 4):
            right.publish_bytes(
                bytes([value]) * 6,
                width=2,
                height=1,
                channels=3,
                timestamp_s=float(value),
            )

        rejected = writer.enqueue_ring_pair(
            request_id="request-missing",
            pair_id="pair-missing",
            purpose="accepted-measurement",
            frames=(
                ("left", left, left_descriptor.key, left_descriptor.to_mapping()),
                ("right", right, right_descriptor.key, right_descriptor.to_mapping()),
            ),
        )
        assert rejected["native_pair_status"] == EVIDENCE_FRAME_UNAVAILABLE
        assert rejected["evidence_state"] == "REJECTED"
        assert left.metrics()["active_leases"] == 0
        assert writer.pending_count == 0
        assert not list(tmp_path.rglob("*.png"))
        assert writer.close() is True
    finally:
        left.close()
        left.unlink()
        right.close()
        right.unlink()


@pytest.mark.parametrize("tamper_kind", ("frame_key", "descriptor_hash"))
def test_writer_fails_closed_on_tampered_frame_identity_or_descriptor_hash(
    tmp_path: Path,
    tamper_kind: str,
) -> None:
    left = _ring("left")
    right = _ring("right")
    writer = AsyncNativeEvidenceWriter(tmp_path)
    try:
        left_descriptor = left.publish_bytes(
            bytes([1]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=1.0,
        )
        right_descriptor = right.publish_bytes(
            bytes([2]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=1.01,
        )
        left_metadata = left_descriptor.to_mapping()
        if tamper_kind == "frame_key":
            left_metadata["frame_key"] = "left:3:999"
        else:
            left_metadata["frame_descriptor_sha256"] = "0" * 64

        scheduled = writer.enqueue_ring_pair(
            request_id=f"tampered-{tamper_kind}",
            pair_id="tampered-pair",
            purpose="accepted-measurement",
            frames=(
                ("left", left, left_descriptor.key, left_metadata),
                ("right", right, right_descriptor.key, right_descriptor.to_mapping()),
            ),
        )
        assert scheduled["evidence_state"] == "WRITING"
        result = _await_result(writer, f"tampered-{tamper_kind}")
        fields = result["fields"]
        assert fields["evidence_state"] == "FAILED"
        assert str(fields["native_pair_status"]).startswith("failed:")
        assert fields["evidence_artifact_count"] == 0
        assert not list(tmp_path.rglob("*.png"))
        assert left.metrics()["active_leases"] == 0
        assert right.metrics()["active_leases"] == 0
        assert writer.close() is True
    finally:
        left.close()
        left.unlink()
        right.close()
        right.unlink()


def test_writer_failure_releases_every_reserved_frame_and_reports_failed(
    tmp_path: Path,
    monkeypatch,
) -> None:
    left = _ring("left")
    right = _ring("right")
    writer = AsyncNativeEvidenceWriter(tmp_path)
    try:
        descriptors = {
            "left": left.publish_bytes(
                bytes([3]) * 6,
                width=2,
                height=1,
                channels=3,
                timestamp_s=2.0,
            ),
            "right": right.publish_bytes(
                bytes([4]) * 6,
                width=2,
                height=1,
                channels=3,
                timestamp_s=2.01,
            ),
        }

        def fail_persist(*_args, **_kwargs):
            raise OSError("injected native writer failure")

        monkeypatch.setattr(NativeFrameStore, "persist", fail_persist)
        scheduled = writer.enqueue_ring_pair(
            request_id="writer-failure",
            pair_id="writer-failure",
            purpose="accepted-measurement",
            frames=tuple(
                (
                    side,
                    ring,
                    descriptors[side].key,
                    descriptors[side].to_mapping(),
                )
                for side, ring in (("left", left), ("right", right))
            ),
        )
        assert scheduled["evidence_state"] == "WRITING"
        fields = _await_result(writer, "writer-failure")["fields"]
        assert fields["evidence_state"] == "FAILED"
        assert fields["evidence_artifact_count"] == 0
        assert left.metrics()["active_leases"] == 0
        assert right.metrics()["active_leases"] == 0
        assert writer.close() is True
    finally:
        left.close()
        left.unlink()
        right.close()
        right.unlink()


def test_pending_writer_stop_and_stream_reconnect_keep_old_reserved_pair_valid(
    tmp_path: Path,
    monkeypatch,
) -> None:
    left = _ring("left")
    right = _ring("right")
    replacement_left = _ring("left", generation=4)
    replacement_right = _ring("right", generation=4)
    entered = threading.Event()
    proceed = threading.Event()
    original_persist = NativeFrameStore.persist

    def blocked_persist(store, *args, **kwargs):
        entered.set()
        if not proceed.wait(timeout=2.0):
            raise TimeoutError("test writer release timeout")
        return original_persist(store, *args, **kwargs)

    monkeypatch.setattr(NativeFrameStore, "persist", blocked_persist)
    writer = AsyncNativeEvidenceWriter(tmp_path)
    try:
        left_descriptor = left.publish_bytes(
            bytes([5]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=3.0,
        )
        right_descriptor = right.publish_bytes(
            bytes([6]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=3.01,
        )
        scheduled = writer.enqueue_ring_pair(
            request_id="stop-reconnect",
            pair_id="stop-reconnect",
            purpose="accepted-measurement",
            frames=(
                ("left", left, left_descriptor.key, left_descriptor.to_mapping()),
                ("right", right, right_descriptor.key, right_descriptor.to_mapping()),
            ),
        )
        assert scheduled["evidence_state"] == "WRITING"
        assert entered.wait(timeout=1.0)
        assert writer.close() is False

        # A replacement stream can publish independently while the old rings
        # remain pinned until their durable evidence ACK completes.
        for replacement in (replacement_left, replacement_right):
            assert replacement.publish_bytes(
                bytes([7]) * 6,
                width=2,
                height=1,
                channels=3,
                timestamp_s=4.0,
            ).available
        assert left.metrics()["active_leases"] == 1
        assert right.metrics()["active_leases"] == 1

        proceed.set()
        fields = _await_result(writer, "stop-reconnect")["fields"]
        assert fields["evidence_state"] == "SAVED"
        assert left.metrics()["active_leases"] == 0
        assert right.metrics()["active_leases"] == 0
        assert writer.close() is True
    finally:
        proceed.set()
        for ring in (left, right, replacement_left, replacement_right):
            ring.close()
            ring.unlink()


def test_recording_controller_schedules_exact_ring_pair_without_gui_qimage(
    tmp_path: Path,
    monkeypatch,
) -> None:
    left = _ring("left")
    right = _ring("right")
    try:
        left_descriptor = left.publish_bytes(
            bytes([5]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=4.0,
        )
        right_descriptor = right.publish_bytes(
            bytes([6]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=4.01,
        )

        def pane(ring: SharedNativeFrameRing):
            return SimpleNamespace(
                native_ring_for_frame_key=lambda key: (
                    ring if ring.descriptor(key) is not None else None
                )
            )

        panel = SimpleNamespace(
            recording_session_id="ring-session",
            _recording_session_dir=lambda _session_id: tmp_path,
            left_pane=pane(left),
            right_pane=pane(right),
            distance_recording_rows=[],
            _recording_pending_native_request_rows={},
            _log=lambda _message: None,
        )
        controller = RecordingController(panel)
        delivered = []
        # This test inspects the producer result before the controller consumes
        # and acknowledges it. The normal callback removes acknowledged results.
        monkeypatch.setattr(controller, "_handle_async_native_frame_result", delivered.append)
        result = controller._persist_native_detection_pair(
            left_detection=SimpleNamespace(
                timestamp_metadata=left_descriptor.to_mapping(),
                app_receive_monotonic_s=4.0,
            ),
            right_detection=SimpleNamespace(
                timestamp_metadata=right_descriptor.to_mapping(),
                app_receive_monotonic_s=4.01,
            ),
            pair_id="pair-controller",
            purpose="accepted-measurement",
        )
        assert result["evidence_state"] == "WRITING"
        assert "native_left_source_frame_id" in result
        writer = panel._native_evidence_writers["ring-session"]
        completed = _await_result(writer, str(result["native_pair_request_id"]))
        assert completed["fields"]["evidence_state"] == "SAVED"
        assert not hasattr(panel.left_pane, "native_measurement_frame")
        assert writer.close()
    finally:
        left.close()
        left.unlink()
        right.close()
        right.unlink()


def test_recording_controller_consumes_pre_gui_reserved_pair_after_ring_depth_delay(
    tmp_path: Path,
    monkeypatch,
) -> None:
    left = _ring("left")
    right = _ring("right")
    try:
        left_descriptor = left.publish_bytes(
            bytes([8]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=5.0,
        )
        right_descriptor = right.publish_bytes(
            bytes([9]) * 6,
            width=2,
            height=1,
            channels=3,
            timestamp_s=5.01,
        )
        reservation = reserve_ring_pair(
            (
                ("left", left, left_descriptor.key, left_descriptor.to_mapping()),
                ("right", right, right_descriptor.key, right_descriptor.to_mapping()),
            )
        )
        assert reservation is not None

        for value in range(10, 16):
            for ring in (left, right):
                ring.publish_bytes(
                    bytes([value]) * 6,
                    width=2,
                    height=1,
                    channels=3,
                    timestamp_s=float(value),
                )

        def pane(ring: SharedNativeFrameRing):
            return SimpleNamespace(native_ring_for_frame_key=lambda _key: ring)

        panel = SimpleNamespace(
            recording_session_id="reserved-session",
            _recording_session_dir=lambda _session_id: tmp_path,
            left_pane=pane(left),
            right_pane=pane(right),
            distance_recording_rows=[],
            _recording_pending_native_request_rows={},
            _log=lambda _message: None,
        )
        controller = RecordingController(panel)
        delivered = []
        monkeypatch.setattr(controller, "_handle_async_native_frame_result", delivered.append)
        result = controller._persist_native_detection_pair(
            left_detection=SimpleNamespace(
                timestamp_metadata=left_descriptor.to_mapping(),
                app_receive_monotonic_s=5.0,
            ),
            right_detection=SimpleNamespace(
                timestamp_metadata=right_descriptor.to_mapping(),
                app_receive_monotonic_s=5.01,
            ),
            pair_id="pre-gui-reserved",
            purpose="accepted-measurement",
            reservation=reservation,
        )
        assert result["evidence_state"] == "WRITING"
        writer = panel._native_evidence_writers["reserved-session"]
        fields = _await_result(
            writer,
            str(result["native_pair_request_id"]),
        )["fields"]
        assert fields["evidence_state"] == "SAVED"
        assert fields["native_left_source_frame_id"] == left_descriptor.key.text
        assert fields["native_right_source_frame_id"] == right_descriptor.key.text
        assert writer.close()
    finally:
        left.close()
        left.unlink()
        right.close()
        right.unlink()
