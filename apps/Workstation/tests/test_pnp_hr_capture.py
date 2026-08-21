from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtWidgets

from Workstation.recording_controller import RecordingController
from Workstation.wiris_runtime_workers import (
    WirisOZPhotoCaptureWorker,
    WirisPnPPhotoCaptureWorker,
)


class _FakeFtpListing:
    def __init__(self) -> None:
        self.current = "/"
        self.entries = {
            "/ssd": ["old-session", "new-session", "latest-visible-session"],
            "/ssd/old-session/img": ["10-00-00-visible.jpg"],
            "/ssd/new-session/img": [
                "10-01-00-visible.jpg",
                "10-01-00-highRes.jpg",
            ],
            "/ssd/latest-visible-session/img": ["10-02-00-visible.jpg"],
            "/sdcard": [],
            "/flashd": [],
        }
        self.mtimes = {
            "10-00-00-visible.jpg": "20260820090000",
            "10-01-00-visible.jpg": "20260820100000",
            "10-01-00-highRes.jpg": "20260820100001",
            "10-02-00-visible.jpg": "20260820110000",
        }

    def cwd(self, path: str) -> None:
        self.current = path.rstrip("/") or "/"

    def nlst(self) -> list[str]:
        return list(self.entries.get(self.current, ()))

    def voidcmd(self, command: str) -> str:
        name = command.split(maxsplit=1)[1]
        return f"213 {self.mtimes[name]}"

    def retrbinary(self, _command: str, callback) -> None:
        callback(b"fake-image")

    def quit(self) -> None:
        return None

    def close(self) -> None:
        return None


def test_pnp_worker_discovers_wiris_session_img_directory_from_root() -> None:
    worker = WirisPnPPhotoCaptureWorker(
        (),
        capture_id="hr-remote-dir",
        image_enable_command="SIHR TRUE",
    )

    remote_dir, name = worker._latest_ftp_file_location(_FakeFtpListing(), "/")

    assert remote_dir == "/ssd/new-session/img"
    assert name == "10-01-00-highRes.jpg"


def test_pnp_worker_refuses_to_reuse_pre_capture_ftp_file() -> None:
    worker = WirisPnPPhotoCaptureWorker(
        (),
        capture_id="hr-stale-guard",
        image_enable_command="SIHR TRUE",
    )
    fake_ftp = _FakeFtpListing()
    baseline = {
        worker._ftp_candidate_token(candidate)
        for candidate in worker._ftp_image_candidates(fake_ftp, "/")
    }

    with pytest.raises(FileNotFoundError, match="stale"):
        worker._latest_ftp_file_location(fake_ftp, "/", baseline_tokens=baseline)

    fake_ftp.entries["/ssd"].append("fresh-session")
    fake_ftp.entries["/ssd/fresh-session/img"] = ["10-02-00-highRes.jpg"]
    fake_ftp.mtimes["10-02-00-highRes.jpg"] = "20260820120000"

    remote_dir, name = worker._latest_ftp_file_location(
        fake_ftp,
        "/",
        baseline_tokens=baseline,
    )
    assert remote_dir == "/ssd/fresh-session/img"
    assert name == "10-02-00-highRes.jpg"


def test_pnp_worker_separates_downloads_by_camera_side(tmp_path: Path, monkeypatch) -> None:
    worker = WirisPnPPhotoCaptureWorker(
        (),
        capture_id="calib-remote-layout",
        capture_channel="calib",
        image_enable_command="SIHR TRUE",
        ftp_output_dir=tmp_path,
        ftp_settings_by_label={"left": {"remote_dir": "/"}},
    )
    fake_ftp = _FakeFtpListing()
    monkeypatch.setattr(worker, "_connect_ftp", lambda *_args: fake_ftp)

    local_path, remote_path = worker._download_latest_ftp_file("10.0.0.1", "left")

    assert local_path.parent == tmp_path / "calib_capture" / "HR" / "left"
    assert local_path.name == "calib-remote-layout_10-01-00-highRes.jpg"
    assert remote_path == "/ssd/new-session/img/10-01-00-highRes.jpg"


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
    progress: list[dict[str, object]] = []
    worker.completed.connect(completed.append)
    worker.progress.connect(progress.append)

    worker.run()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.processEvents()

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
    assert {(row["side"], row["command"]) for row in progress if row["phase"] == "response"} >= {
        ("left", "SIHR TRUE"),
        ("left", "CPTR"),
        ("left", "ICPT"),
        ("right", "SIHR TRUE"),
        ("right", "CPTR"),
        ("right", "ICPT"),
    }


@pytest.mark.parametrize(
    "icpt_response",
    ("", "ERR", "ERROR 0", "OK", "garbage", "ICPT 0 trailing"),
)
def test_pnp_worker_rejects_non_explicit_icpt_completion(
    monkeypatch: pytest.MonkeyPatch,
    icpt_response: str,
) -> None:
    def fake_send(_host: str, _port: int, _timeout_s: float, command: str) -> str:
        return icpt_response if command == "ICPT" else "OK"

    monkeypatch.setattr(WirisPnPPhotoCaptureWorker, "_send", staticmethod(fake_send))
    worker = WirisPnPPhotoCaptureWorker(
        (("left", "10.0.0.1", 2240, 0.5),),
        capture_id="hr-invalid-icpt",
        completion_timeout_s=0.0,
    )
    completed: list[dict[str, object]] = []
    worker.completed.connect(completed.append)

    worker.run()

    camera = completed[0]["cameras"]["left"]
    assert camera["state"] == "failed"
    assert camera["capture_complete"] is False
    assert camera["stored_on_camera"] is False
    assert "invalid ICPT response" in camera["error"]


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
