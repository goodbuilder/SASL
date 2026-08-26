from __future__ import annotations

import hashlib
import multiprocessing
import time
from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtGui

from Workstation.workstation_core.native_evidence_writer import (
    AsyncNativeEvidenceWriter,
    EVIDENCE_FRAME_UNAVAILABLE,
)
from Workstation.workstation_core.native_frame_ring import SharedNativeFrameRing
from Workstation.recording_controller import RecordingController


def _ring(side: str) -> SharedNativeFrameRing:
    return SharedNativeFrameRing.create(
        camera_id=side,
        stream_generation=3,
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


def test_recording_controller_schedules_exact_ring_pair_without_gui_qimage(
    tmp_path: Path,
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
        result = RecordingController(panel)._persist_native_detection_pair(
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
