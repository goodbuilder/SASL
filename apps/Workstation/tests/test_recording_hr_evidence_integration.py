from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from Workstation import recording_controller as recording_module
from Workstation.recording_controller import RecordingController


class _Label:
    def __init__(self) -> None:
        self.text = ""

    def setText(self, value: object) -> None:  # noqa: N802 - Qt-shaped test double.
        self.text = str(value)


def _panel(tmp_path: Path, *, session_id: str = "hr-session") -> SimpleNamespace:
    return SimpleNamespace(
        recording_session_id=session_id,
        recording_session_started_monotonic_ns=1_000,
        _session_save_finalized=False,
        _pnp_session_prepared=False,
        pnp_photo_capture_thread=None,
        pnp_photo_capture_worker=None,
        pnp_photo_capture_records=[],
        recording_status_label=_Label(),
        actual_visible_zoom={
            "left": {"index": 8, "updated_monotonic_s": 1.0},
            "right": {"index": 8, "updated_monotonic_s": 1.0},
        },
        zoom_level_spin=SimpleNamespace(value=lambda: 8),
        zoom_timeout_spin=SimpleNamespace(value=lambda: 0.5),
        zoom_hosts=lambda _target: (),
        _control_port_for_camera=lambda _side: 2240,
        _recording_session_dir=lambda stamp: tmp_path / stamp,
        _log=lambda _message: None,
    )


def _assert_record_hash(row: dict[str, object]) -> None:
    payload = dict(row)
    digest = str(payload.pop("record_sha256"))
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    assert digest == hashlib.sha256(encoded).hexdigest()


def test_capture_purpose_and_host_request_delta_are_append_only(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = _panel(tmp_path)
    controller = RecordingController(panel)
    panel._append_pnp_photo_capture_record = controller._append_pnp_photo_capture_record

    monotonic_values = iter((1_100, 1_300))
    utc_values = iter((9_000, 9_100))
    monkeypatch.setattr(recording_module.time, "monotonic_ns", lambda: next(monotonic_values))
    monkeypatch.setattr(recording_module.time, "time_ns", lambda: next(utc_values))

    # Missing hosts exercises the complete public request/event path without
    # starting a hardware thread.
    assert controller.capture_pnp_hr_photos("pre") is False

    event_file = tmp_path / "hr-session" / "hr_captures.jsonl"
    rows = [json.loads(line) for line in event_file.read_text().splitlines()]
    assert len(rows) == 1
    row = rows[0]
    assert row["capture_id"] == "hr-001"
    assert row["purpose"] == "pre"
    assert row["state"] == "not_started"
    assert row["request_utc_ns"] == 9_000
    assert row["request_monotonic_ns"] == 1_200
    assert row["request_clock_anchor_uncertainty_ns"] == 100
    assert row["request_session_delta_ns"] == 200
    assert row["request_session_delta_s"] == pytest.approx(2e-7)
    assert row["capture_time_semantics"].startswith("Workstation request UTC")
    assert row["event_sha256"]
    _assert_record_hash(row)

    # Signal/input dictionaries are detached from the persisted record.
    assert panel.pnp_photo_capture_records[0] == row
    assert controller._pending_hr_requests == {}
    with pytest.raises(ValueError, match="purpose"):
        controller.capture_pnp_hr_photos("training")


def test_completed_capture_records_signed_right_minus_left_request_delta(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = RecordingController(panel)
    panel._append_pnp_photo_capture_record = controller._append_pnp_photo_capture_record
    controller._pending_hr_requests["hr-001"] = {
        "capture_id": "hr-001",
        "purpose": "checkpoint",
        "request_utc_ns": 50,
        "request_monotonic_ns": 40,
        "request_session_delta_ns": -960,
        "request_session_delta_s": -9.6e-7,
    }

    controller._handle_pnp_photo_capture_completed(
        {
            "capture_id": "hr-001",
            "cameras": {
                "left": {"state": "completed", "capture_requested_unix_ns": 1_020},
                "right": {"state": "completed", "capture_requested_unix_ns": 990},
            },
        }
    )

    row = panel.pnp_photo_capture_records[0]
    assert row["state"] == "completed"
    assert row["purpose"] == "checkpoint"
    assert row["camera_request_delta_right_minus_left_ns"] == -30
    assert row["camera_request_delta_sign_contract"].startswith("right")
    _assert_record_hash(row)


def test_post_measurement_photo_reuses_the_just_finalized_session(tmp_path: Path) -> None:
    """A "post" bookend photo taken right after stopping recording must join
    the session that was just saved, not open a fresh one.

    Only the next explicit measurement start (``_pnp_session_prepared`` still
    False when it runs) is a "new session" boundary -- see
    ``_prepare_camera_evidence_session``'s docstring.
    """

    panel = _panel(tmp_path, session_id="will-be-replaced")
    panel.recording_session_id = None
    controller = RecordingController(panel)

    controller._prepare_camera_evidence_session("hr")
    first_session_id = panel.recording_session_id
    assert first_session_id
    assert panel._pnp_session_prepared is True

    # Measurement "starts": consumes the prepared session, same as
    # _start_session_recording's `if not self.panel._pnp_session_prepared`
    # branch would (that method needs more panel surface than this test
    # wires up, so drive its two relevant side effects directly).
    panel._pnp_session_prepared = False

    # Measurement "stops and saves".
    panel._session_save_finalized = True
    panel.pnp_photo_capture_records.append({"purpose": "pre"})

    # A "post" bookend photo, taken after the save finished.
    controller._prepare_camera_evidence_session("hr")

    assert panel.recording_session_id == first_session_id
    assert panel._session_save_finalized is True, "must not pretend a new save is pending"
    assert panel.pnp_photo_capture_records == [{"purpose": "pre"}], (
        "must not wipe records from the measurement that was just saved"
    )
    # Not consumed by a real measurement start yet, so it stays False -- the
    # next _start_session_recording call will see this and correctly mint a
    # brand new session instead of resuming this already-finalized one.
    assert panel._pnp_session_prepared is False


def test_import_hr_pair_retains_exact_session_owned_bytes_and_evidence(
    tmp_path: Path,
) -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    left_pixels = np.zeros((7, 11, 3), dtype=np.uint8)
    right_pixels = np.zeros((9, 13, 3), dtype=np.uint8)
    left_pixels[:, :, 1] = 123
    right_pixels[:, :, 2] = 231
    left_ok, left_encoded = cv2.imencode(".png", left_pixels)
    right_ok, right_encoded = cv2.imencode(".png", right_pixels)
    assert left_ok and right_ok
    left_bytes = bytes(left_encoded) + b"LEFT-EXACT-TRAILER"
    right_bytes = bytes(right_encoded) + b"RIGHT-EXACT-TRAILER"
    left_source = tmp_path / "source-left.png"
    right_source = tmp_path / "source-right.png"
    left_source.write_bytes(left_bytes)
    right_source.write_bytes(right_bytes)

    panel = _panel(tmp_path)
    panel.pnp_photo_capture_records.append(
        {
            "capture_id": "field/capture 7",
            "purpose": "post",
            "request_utc_ns": 12_345,
            "request_monotonic_ns": 456,
            "request_session_delta_ns": -544,
            "request_session_delta_s": -5.44e-7,
        }
    )
    linked_events: list[tuple[str, dict[str, object], dict[str, object]]] = []

    def append_run_event(
        event_type: str,
        payload: dict[str, object],
        **kwargs: object,
    ) -> None:
        linked_events.append((event_type, dict(payload), dict(kwargs)))

    panel.append_workstation_run_event = append_run_event
    controller = RecordingController(panel)

    result = controller.import_hr_files(
        "field/capture 7",
        left_path=left_source,
        right_path=right_source,
        left_camera_serial="WIRIS-L-001",
        right_camera_serial="WIRIS-R-002",
        expected_sha256_by_side={
            "left": hashlib.sha256(left_bytes).hexdigest(),
            "right": hashlib.sha256(right_bytes).hexdigest(),
        },
        expected_resolution_by_side={"left": (11, 7), "right": (13, 9)},
    )

    assert result["purpose"] == "post"
    assert result["capture_time_semantics"] == "Workstation_capture_request_utc"
    assert result["request_utc_ns"] == 12_345
    assert result["storage"] == "session_owned_exact_file"
    assert result["left"]["camera_serial"] == "WIRIS-L-001"
    assert result["right"]["camera_serial"] == "WIRIS-R-002"
    assert (result["left"]["width_px"], result["left"]["height_px"]) == (11, 7)
    assert (result["right"]["width_px"], result["right"]["height_px"]) == (13, 9)

    session_root = (tmp_path / "hr-session").resolve()
    left_stored = session_root / str(
        result["stored_files"]["left"]["session_relative_path"]
    )
    right_stored = session_root / str(
        result["stored_files"]["right"]["session_relative_path"]
    )
    assert left_stored.read_bytes() == left_bytes
    assert right_stored.read_bytes() == right_bytes
    assert left_stored.is_relative_to(session_root)
    assert right_stored.is_relative_to(session_root)

    # Changing the source after import cannot change session evidence.
    left_source.write_bytes(b"changed")
    assert left_stored.read_bytes() == left_bytes

    persisted = [
        json.loads(line)
        for line in (session_root / "hr_captures.jsonl").read_text().splitlines()
    ]
    assert len(persisted) == 1
    assert persisted[0] == result
    _assert_record_hash(persisted[0])
    assert linked_events[0][0] == "hr_capture_event"
    assert linked_events[0][1]["record_sha256"] == result["record_sha256"]
    assert "OpenCV optical RDF" in str(linked_events[0][2]["coordinate_contract"])


def test_import_rejects_missing_serial_before_writing_session_evidence(
    tmp_path: Path,
) -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    ok, encoded = cv2.imencode(".png", np.zeros((3, 4), dtype=np.uint8))
    assert ok
    source = tmp_path / "hr.png"
    source.write_bytes(bytes(encoded))
    panel = _panel(tmp_path)
    controller = RecordingController(panel)

    with pytest.raises(ValueError, match="explicit camera serial"):
        controller.import_hr_files("hr-001", side="left", path=source)
    assert not (tmp_path / "hr-session" / "hr_captures.jsonl").exists()
    assert not (tmp_path / "hr-session" / "hr").exists()
