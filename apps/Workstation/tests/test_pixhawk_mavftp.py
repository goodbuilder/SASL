from __future__ import annotations

import json
import struct
import threading
import zlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from Workstation.workstation_core.pixhawk_mavftp import (
    FTP_DATA_BYTES,
    FTP_HEADER,
    FTP_PAYLOAD_BYTES,
    OP_ACK,
    OP_CALC_FILE_CRC32,
    OP_CREATE_FILE,
    OP_NACK,
    OP_OPEN_FILE_RO,
    OP_OPEN_FILE_WO,
    OP_READ_FILE,
    OP_TERMINATE_SESSION,
    OP_WRITE_FILE,
    MavFtpOffsetClient,
    atomic_write_transfer_evidence,
)


class _Reply:
    def __init__(self, payload: bytes) -> None:
        self.payload = payload

    def get_type(self) -> str:
        return "FILE_TRANSFER_PROTOCOL"


class _FakeMav:
    def __init__(self, owner: "_FakeConnection") -> None:
        self.owner = owner
        self.requests: list[tuple[int, int, int, int, bytes]] = []

    def file_transfer_protocol_send(
        self,
        _network: int,
        _target_system: int,
        _target_component: int,
        payload: bytes,
    ) -> None:
        raw = bytes(payload)
        assert len(raw) == FTP_PAYLOAD_BYTES
        seq, session, opcode, size, _req, _burst, _pad, offset = FTP_HEADER.unpack_from(raw)
        data = raw[FTP_HEADER.size : FTP_HEADER.size + min(size, FTP_DATA_BYTES)]
        self.requests.append((seq, session, opcode, offset, data))
        self.owner.handle(seq, session, opcode, size, offset, data)


class _FakeConnection:
    target_system = 3
    target_component = 1

    def __init__(self, remote: dict[str, bytes] | None = None) -> None:
        self.remote = dict(remote or {})
        self.active_path = ""
        self.session = 7
        self.replies: list[_Reply] = []
        self.mav = _FakeMav(self)

    def _reply(
        self,
        seq: int,
        session: int,
        req_opcode: int,
        *,
        offset: int = 0,
        payload: bytes = b"",
        ack: bool = True,
    ) -> None:
        header = FTP_HEADER.pack(
            (seq + 1) & 0xFFFF,
            session,
            OP_ACK if ack else OP_NACK,
            len(payload),
            req_opcode,
            0,
            0,
            offset,
        )
        self.replies.append(_Reply((header + payload).ljust(FTP_PAYLOAD_BYTES, b"\0")))

    def handle(self, seq: int, session: int, opcode: int, size: int, offset: int, data: bytes) -> None:
        if opcode in {OP_OPEN_FILE_RO, OP_OPEN_FILE_WO, OP_CREATE_FILE, OP_CALC_FILE_CRC32}:
            path = data.decode("utf-8")
        else:
            path = self.active_path
        if opcode == OP_CREATE_FILE:
            self.active_path = path
            self.remote[path] = b""
            self._reply(seq, self.session, opcode)
        elif opcode in {OP_OPEN_FILE_RO, OP_OPEN_FILE_WO}:
            if path not in self.remote:
                self._reply(seq, self.session, opcode, payload=b"\x0a", ack=False)
            else:
                self.active_path = path
                self._reply(seq, self.session, opcode, payload=struct.pack("<I", len(self.remote[path])))
        elif opcode == OP_WRITE_FILE:
            current = bytearray(self.remote[path])
            if len(current) < offset:
                current.extend(b"\0" * (offset - len(current)))
            current[offset : offset + len(data)] = data
            self.remote[path] = bytes(current)
            self._reply(seq, self.session, opcode, offset=offset)
        elif opcode == OP_READ_FILE:
            payload = self.remote[path][offset : offset + size]
            if not payload:
                self._reply(seq, self.session, opcode, offset=offset, payload=b"\x06", ack=False)
            else:
                self._reply(seq, self.session, opcode, offset=offset, payload=payload)
        elif opcode == OP_CALC_FILE_CRC32:
            crc = zlib.crc32(self.remote[path]) & 0xFFFFFFFF
            self._reply(seq, 0, opcode, payload=struct.pack("<I", crc))
        elif opcode == OP_TERMINATE_SESSION:
            self._reply(seq, session, opcode)
            self.active_path = ""
        else:
            raise AssertionError(f"unexpected opcode {opcode}")

    def recv_match(self, **_kwargs):
        return self.replies.pop(0) if self.replies else None


def _client(connection: _FakeConnection) -> MavFtpOffsetClient:
    return MavFtpOffsetClient(connection, send_lock=threading.RLock(), timeout_s=0.1)


def test_restart_upload_writes_offsets_and_verifies_remote_size_crc(tmp_path: Path) -> None:
    local = tmp_path / "firmware.px4"
    payload = bytes(range(256)) * 3
    local.write_bytes(payload)
    connection = _FakeConnection({"/stage/firmware.px4": b"old"})
    progress: list[tuple[int, int]] = []

    evidence = _client(connection).upload(
        local,
        "/stage/firmware.px4",
        resume=False,
        progress=lambda sent, total: progress.append((sent, total)),
    )

    assert connection.remote["/stage/firmware.px4"] == payload
    assert evidence.remote_size == len(payload)
    assert evidence.remote_crc32 == zlib.crc32(payload) & 0xFFFFFFFF
    assert evidence.local_sha256
    assert evidence.resumed_from == 0
    assert progress[-1] == (len(payload), len(payload))
    write_offsets = [offset for _seq, _session, opcode, offset, _data in connection.mav.requests if opcode == OP_WRITE_FILE]
    assert write_offsets == [0, FTP_DATA_BYTES, FTP_DATA_BYTES * 2, FTP_DATA_BYTES * 3]


def test_resume_requires_exact_remote_prefix_and_starts_at_verified_offset(tmp_path: Path) -> None:
    local = tmp_path / "large.bin"
    payload = b"prefix-verified-" * 40
    local.write_bytes(payload)
    partial_size = 277
    remote_path = "/data/large.bin"
    connection = _FakeConnection({remote_path: payload[:partial_size]})

    evidence = _client(connection).upload(local, remote_path, resume=True)

    assert evidence.resumed_from == partial_size
    assert connection.remote[remote_path] == payload
    first_write = next(row for row in connection.mav.requests if row[2] == OP_WRITE_FILE)
    assert first_write[3] == partial_size


def test_resume_rejects_changed_remote_prefix_without_writing(tmp_path: Path) -> None:
    local = tmp_path / "large.bin"
    local.write_bytes(b"correct bytes")
    connection = _FakeConnection({"/data/large.bin": b"wrong"})

    with pytest.raises(RuntimeError, match="Restart is required"):
        _client(connection).upload(local, "/data/large.bin", resume=True)

    assert all(row[2] != OP_WRITE_FILE for row in connection.mav.requests)


def test_transfer_evidence_is_written_atomically(tmp_path: Path) -> None:
    local = tmp_path / "file.bin"
    local.write_bytes(b"abc")
    evidence = _client(_FakeConnection()).upload(local, "/file.bin", resume=False)
    target = tmp_path / "evidence" / "transfer.json"

    atomic_write_transfer_evidence(target, evidence)

    payload = json.loads(target.read_text(encoding="utf-8"))
    assert payload["remote_path"] == "/file.bin"
    assert payload["local_size"] == 3
    assert list(target.parent.glob(".*.tmp")) == []


def test_download_restart_and_verified_resume_preserve_partial_until_complete(tmp_path: Path) -> None:
    remote_path = "/log/flight.ulg"
    payload = b"ULog" + bytes(range(251)) * 3
    destination = tmp_path / "flight.ulg"
    connection = _FakeConnection({remote_path: payload})

    restarted = _client(connection).download(remote_path, destination, resume=False)
    assert destination.read_bytes() == payload
    assert restarted.resumed_from == 0

    destination.unlink()
    partial = destination.with_name(f".{destination.name}.partial")
    partial.write_bytes(payload[:311])
    resumed = _client(_FakeConnection({remote_path: payload})).download(
        remote_path,
        destination,
        resume=True,
    )
    assert resumed.resumed_from == 311
    assert destination.read_bytes() == payload
    assert not partial.exists()


def test_download_resume_rejects_wrong_local_partial(tmp_path: Path) -> None:
    destination = tmp_path / "flight.ulg"
    partial = destination.with_name(f".{destination.name}.partial")
    partial.write_bytes(b"wrong")

    with pytest.raises(RuntimeError, match="Restart is required"):
        _client(_FakeConnection({"/log/flight.ulg": b"correct payload"})).download(
            "/log/flight.ulg",
            destination,
            resume=True,
        )

    assert partial.read_bytes() == b"wrong"
