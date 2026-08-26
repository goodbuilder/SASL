from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from Workstation.gnss_runtime_controller import GnssRuntimeController


def test_rtcm_bytes_and_offset_index_are_session_owned_and_exact(tmp_path: Path) -> None:
    activations: list[tuple[str, dict[str, object]]] = []
    panel = SimpleNamespace(
        _measurement_package_state="COLLECTING",
        recording_session_id="rtcm-session",
        _recording_session_dir=lambda session_id: tmp_path / session_id,
        activate_measurement_package_source=lambda source, **kwargs: activations.append(
            (source, kwargs)
        ),
    )
    controller = GnssRuntimeController(panel)
    first = b"\xd3\x00\x01\x00\x00\x00\x00"
    second = b"\xd3\x00\x02\x10\x20\x00\x00\x00"

    controller._append_rtcm_package_evidence(
        first,
        source="ntrip",
        record_kind="transport_chunk",
    )
    controller._append_rtcm_package_evidence(
        second,
        source="ntrip",
        record_kind="transport_chunk",
    )

    root = tmp_path / "rtcm-session" / "raw" / "gnss"
    binary = root / "ntrip_transport_chunk.rtcm"
    rows = [
        json.loads(line)
        for line in (root / "ntrip_transport_chunk.rtcm.jsonl").read_text().splitlines()
    ]
    assert binary.read_bytes() == first + second
    assert [(row["offset_bytes"], row["size_bytes"]) for row in rows] == [
        (0, len(first)),
        (len(first), len(second)),
    ]
    assert rows[0]["sha256"] == hashlib.sha256(first).hexdigest()
    assert rows[1]["sha256"] == hashlib.sha256(second).hexdigest()
    assert {source for source, _kwargs in activations} == {"gnss.rtcm"}
    assert activations[-1][1]["required_roles"] == ("corrections/rtcm/raw",)


def test_rtcm_bytes_are_not_written_without_an_open_measurement_package(tmp_path: Path) -> None:
    panel = SimpleNamespace(
        _measurement_package_state="IDLE",
        recording_session_id="rtcm-session",
        _recording_session_dir=lambda session_id: tmp_path / session_id,
    )
    GnssRuntimeController(panel)._append_rtcm_package_evidence(
        b"sentinel",
        source="base",
        record_kind="rtcm_frame",
    )
    assert not (tmp_path / "rtcm-session").exists()


def test_sessionless_rtcm_appends_only_while_recording_without_a_session(tmp_path: Path) -> None:
    recording = {"active": True}
    panel = SimpleNamespace(
        _measurement_session_hierarchy_enabled=False,
        _session_recording_active=lambda: recording["active"],
        _recording_session_dir=lambda _identity: tmp_path / "measurement_data",
    )
    controller = GnssRuntimeController(panel)
    context = controller._rtcm_evidence_context()
    assert context.active is True
    assert context.session_id == "measurement-data"

    controller._append_rtcm_package_evidence(
        b"first",
        source="base",
        record_kind="rtcm_frame",
    )
    controller._append_rtcm_package_evidence(
        b"second",
        source="base",
        record_kind="rtcm_frame",
    )
    binary = tmp_path / "measurement_data" / "raw" / "gnss" / "base_rtcm_frame.rtcm"
    assert binary.read_bytes() == b"firstsecond"
    index_rows = [
        json.loads(line)
        for line in binary.with_suffix(".rtcm.jsonl").read_text().splitlines()
    ]
    assert all(row["data_owner"] == "measurement_data_root" for row in index_rows)
    assert all("session_id" not in row for row in index_rows)

    recording["active"] = False
    assert controller._rtcm_evidence_context().active is False
    controller._append_rtcm_package_evidence(
        b"ignored",
        source="base",
        record_kind="rtcm_frame",
    )
    assert binary.read_bytes() == b"firstsecond"
