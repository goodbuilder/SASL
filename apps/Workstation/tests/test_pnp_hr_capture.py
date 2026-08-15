from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from Workstation.recording_controller import RecordingController
from Workstation.wiris_runtime_workers import (
    WirisOZPhotoCaptureWorker,
    WirisPnPPhotoCaptureWorker,
)


def test_pnp_worker_enables_hr_and_captures_both_cameras(monkeypatch) -> None:
    calls: list[tuple[str, str]] = []

    def fake_send(host: str, _port: int, _timeout_s: float, command: str) -> str:
        calls.append((host, command))
        return "0" if command == "ICPT" else "OK"

    monkeypatch.setattr(WirisPnPPhotoCaptureWorker, "_send", staticmethod(fake_send))
    worker = WirisPnPPhotoCaptureWorker(
        (
            ("left", "10.0.0.1", 2240, 0.5),
            ("right", "10.0.0.2", 2240, 0.5),
        ),
        capture_id="hr-001",
        completion_timeout_s=0.1,
    )
    completed: list[dict[str, object]] = []
    worker.completed.connect(completed.append)

    worker.run()

    assert len(completed) == 1
    assert completed[0]["capture_id"] == "hr-001"
    assert completed[0]["ftp_downloaded"] is False
    cameras = completed[0]["cameras"]
    assert cameras["left"]["stored_on_camera"] is True
    assert cameras["right"]["stored_on_camera"] is True
    for host in ("10.0.0.1", "10.0.0.2"):
        assert (host, "SIHR TRUE") in calls
        assert (host, "CPTR") in calls
        assert (host, "ICPT") in calls


def test_oz_worker_enables_visible_still_and_captures_both_cameras(monkeypatch) -> None:
    calls: list[tuple[str, str]] = []

    def fake_send(host: str, _port: int, _timeout_s: float, command: str) -> str:
        calls.append((host, command))
        return "0" if command == "ICPT" else "OK"

    monkeypatch.setattr(WirisOZPhotoCaptureWorker, "_send", staticmethod(fake_send))
    worker = WirisOZPhotoCaptureWorker(
        (
            ("left", "10.0.0.1", 2240, 0.5),
            ("right", "10.0.0.2", 2240, 0.5),
        ),
        capture_id="oz-001",
        completion_timeout_s=0.1,
    )
    completed: list[dict[str, object]] = []
    worker.completed.connect(completed.append)

    worker.run()

    assert len(completed) == 1
    assert completed[0]["capture_id"] == "oz-001"
    assert completed[0]["capture_channel"] == "oz"
    assert completed[0]["commands"] == ("SIVI TRUE", "CPTR", "ICPT")
    cameras = completed[0]["cameras"]
    assert cameras["left"]["oz_enabled"] is True
    assert cameras["right"]["oz_enabled"] is True
    for host in ("10.0.0.1", "10.0.0.2"):
        assert (host, "SIVI TRUE") in calls
        assert (host, "CPTR") in calls
        assert (host, "ICPT") in calls


def test_pnp_capture_records_append_to_separate_session_jsonl(tmp_path: Path) -> None:
    session_id = "session-pnp"
    panel = SimpleNamespace(
        recording_session_id=session_id,
        pnp_photo_capture_records=[],
        actual_visible_zoom={
            "left": {"index": 6, "updated_monotonic_s": 10.0},
            "right": {"index": 6, "updated_monotonic_s": 10.1},
        },
        zoom_level_spin=SimpleNamespace(value=lambda: 6),
        _recording_session_dir=lambda stamp: tmp_path / stamp,
    )
    controller = RecordingController(panel)

    controller._append_pnp_photo_capture_record(
        {
            "capture_id": "hr-001",
            "state": "completed",
            "cameras": {"left": {"state": "completed"}, "right": {"state": "completed"}},
        }
    )
    controller._append_pnp_photo_capture_record(
        {
            "capture_id": "hr-002",
            "state": "partial_or_failed",
            "cameras": {"left": {"state": "completed"}, "right": {"state": "timeout"}},
        }
    )

    event_path = tmp_path / session_id / "hr_captures.jsonl"
    rows = [json.loads(line) for line in event_path.read_text().splitlines()]
    assert [row["capture_id"] for row in rows] == ["hr-001", "hr-002"]
    assert [row["event_index"] for row in rows] == [1, 2]
    assert all(row["storage"] == "camera_internal" for row in rows)
    assert all(row["ftp_downloaded"] is False for row in rows)
    assert all(row["requested_zoom_level"] == 6 for row in rows)
    assert rows[0]["active_zoom"]["left"]["index"] == 6


def test_oz_capture_records_append_to_separate_session_jsonl(tmp_path: Path) -> None:
    session_id = "session-oz"
    panel = SimpleNamespace(
        recording_session_id=session_id,
        oz_photo_capture_records=[],
        actual_visible_zoom={
            "left": {"index": 8, "updated_monotonic_s": 10.0},
            "right": {"index": 8, "updated_monotonic_s": 10.1},
        },
        zoom_level_spin=SimpleNamespace(value=lambda: 8),
        _recording_session_dir=lambda stamp: tmp_path / stamp,
    )
    controller = RecordingController(panel)

    result = controller._append_oz_photo_capture_record(
        {
            "capture_id": "oz-001",
            "purpose": "pre",
            "state": "completed",
            "image_enable_command": "SIVI TRUE",
            "cameras": {"left": {"state": "completed"}, "right": {"state": "completed"}},
        }
    )

    event_path = tmp_path / session_id / "oz_captures.jsonl"
    rows = [json.loads(line) for line in event_path.read_text().splitlines()]
    assert rows == [result]
    assert result["capture_channel"] == "oz"
    assert result["purpose"] == "pre"
    assert result["event_index"] == 1
    assert result["storage"] == "camera_internal"
    assert result["ftp_downloaded"] is False
    assert result["image_enable_command"] == "SIVI TRUE"
    assert result["record_sha256"]


def test_oz_capture_without_camera_hosts_records_rejection(tmp_path: Path) -> None:
    panel = SimpleNamespace(
        recording_session_id="oz-rejected",
        recording_session_started_monotonic_ns=1_000,
        _session_save_finalized=False,
        _pnp_session_prepared=False,
        pnp_photo_capture_thread=None,
        oz_photo_capture_thread=None,
        oz_photo_capture_records=[],
        recording_status_label=SimpleNamespace(setText=lambda _value: None),
        actual_visible_zoom={"left": {}, "right": {}},
        zoom_level_spin=SimpleNamespace(value=lambda: 8),
        zoom_timeout_spin=SimpleNamespace(value=lambda: 0.5),
        zoom_hosts=lambda _target: (),
        _control_port_for_camera=lambda _side: 2240,
        _recording_session_dir=lambda stamp: tmp_path / stamp,
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)
    panel._append_oz_photo_capture_record = controller._append_oz_photo_capture_record

    assert controller.capture_pnp_oz_photos("post") is False

    rows = [
        json.loads(line)
        for line in (tmp_path / "oz-rejected" / "oz_captures.jsonl").read_text().splitlines()
    ]
    assert rows[0]["capture_id"] == "oz-001"
    assert rows[0]["capture_channel"] == "oz"
    assert rows[0]["purpose"] == "post"
    assert rows[0]["state"] == "not_started"
