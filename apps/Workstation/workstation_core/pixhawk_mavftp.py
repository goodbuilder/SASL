"""Offset-aware MAVLink FTP client used by explicit Pixhawk transfers."""

from __future__ import annotations

import hashlib
import os
import struct
import time
import zlib
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable


FTP_HEADER = struct.Struct("<HBBBBBBI")
FTP_PAYLOAD_BYTES = 251
FTP_DATA_BYTES = FTP_PAYLOAD_BYTES - FTP_HEADER.size

OP_TERMINATE_SESSION = 1
OP_OPEN_FILE_RO = 4
OP_READ_FILE = 5
OP_CREATE_FILE = 6
OP_WRITE_FILE = 7
OP_OPEN_FILE_WO = 11
OP_CALC_FILE_CRC32 = 14
OP_ACK = 128
OP_NACK = 129


@dataclass(frozen=True)
class MavFtpReply:
    seq: int
    session: int
    opcode: int
    size: int
    req_opcode: int
    offset: int
    payload: bytes


@dataclass(frozen=True)
class TransferEvidence:
    remote_path: str
    local_path: str
    local_size: int
    local_sha256: str
    local_crc32: int
    remote_size: int
    remote_crc32: int
    resumed_from: int

    def to_mapping(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class PartialVerification:
    remote_size: int
    local_prefix_sha256: str
    remote_prefix_sha256: str
    matches: bool


def local_file_identity(path: str | Path) -> tuple[int, str, int]:
    size = 0
    sha256 = hashlib.sha256()
    crc32 = 0
    with Path(path).open("rb") as handle:
        while True:
            chunk = handle.read(1024 * 1024)
            if not chunk:
                break
            size += len(chunk)
            sha256.update(chunk)
            crc32 = zlib.crc32(chunk, crc32)
    return size, sha256.hexdigest(), crc32 & 0xFFFFFFFF


class MavFtpOffsetClient:
    """Synchronous request/reply adapter with explicit offset semantics.

    The caller must own the connection receive path. ``send_lock`` is held
    only for one MAVLink packet so the RTCM owner can interleave higher-
    priority traffic between FTP chunks.
    """

    def __init__(
        self,
        connection: Any,
        *,
        send_lock: Any,
        timeout_s: float = 2.0,
        retries: int = 3,
        on_unmatched: Callable[[Any], None] | None = None,
    ) -> None:
        self.connection = connection
        self.send_lock = send_lock
        self.timeout_s = max(0.1, float(timeout_s))
        self.retries = max(1, int(retries))
        self.on_unmatched = on_unmatched
        self.seq = 0
        self.session = 0

    @staticmethod
    def decode_payload(payload: Any) -> MavFtpReply:
        raw = bytes(payload)
        if len(raw) < FTP_HEADER.size:
            raise ValueError("MAVFTP payload is shorter than its header")
        seq, session, opcode, size, req_opcode, _burst, _padding, offset = FTP_HEADER.unpack_from(raw)
        if size > FTP_DATA_BYTES:
            raise ValueError(f"MAVFTP reply size is invalid: {size}")
        return MavFtpReply(
            seq=seq,
            session=session,
            opcode=opcode,
            size=size,
            req_opcode=req_opcode,
            offset=offset,
            payload=raw[FTP_HEADER.size : FTP_HEADER.size + size],
        )

    def _encode(
        self,
        opcode: int,
        *,
        session: int,
        offset: int,
        payload: bytes,
        size_override: int | None = None,
    ) -> bytes:
        if len(payload) > FTP_DATA_BYTES:
            raise ValueError("MAVFTP request payload is too large")
        header = FTP_HEADER.pack(
            self.seq & 0xFFFF,
            session & 0xFF,
            opcode & 0xFF,
            len(payload) if size_override is None else int(size_override),
            0,
            0,
            0,
            int(offset),
        )
        return (header + payload).ljust(FTP_PAYLOAD_BYTES, b"\0")

    def _send(
        self,
        opcode: int,
        *,
        session: int = 0,
        offset: int = 0,
        payload: bytes = b"",
        size_override: int | None = None,
    ) -> None:
        target_system = int(getattr(self.connection, "target_system", 0) or 1)
        target_component = int(getattr(self.connection, "target_component", 0) or 1)
        encoded = self._encode(
            opcode,
            session=session,
            offset=offset,
            payload=payload,
            size_override=size_override,
        )
        with self.send_lock:
            self.connection.mav.file_transfer_protocol_send(
                0,
                target_system,
                target_component,
                encoded,
            )

    def _request(
        self,
        opcode: int,
        *,
        session: int = 0,
        offset: int = 0,
        payload: bytes = b"",
        size_override: int | None = None,
        allow_eof: bool = False,
    ) -> MavFtpReply:
        last_error = "timeout"
        for _attempt in range(self.retries):
            self.seq = (self.seq + 1) & 0xFFFF
            self._send(
                opcode,
                session=session,
                offset=offset,
                payload=payload,
                size_override=size_override,
            )
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                message = self.connection.recv_match(blocking=True, timeout=min(0.1, deadline - time.monotonic()))
                if message is None:
                    continue
                if getattr(message, "get_type", lambda: "")() != "FILE_TRANSFER_PROTOCOL":
                    if self.on_unmatched is not None:
                        self.on_unmatched(message)
                    continue
                reply = self.decode_payload(getattr(message, "payload", b""))
                if reply.req_opcode != opcode:
                    if self.on_unmatched is not None:
                        self.on_unmatched(message)
                    continue
                if opcode in {OP_READ_FILE, OP_WRITE_FILE} and reply.offset != offset:
                    if self.on_unmatched is not None:
                        self.on_unmatched(message)
                    continue
                if reply.opcode == OP_ACK:
                    return reply
                error_code = reply.payload[0] if reply.payload else -1
                if allow_eof and error_code == 6:
                    return reply
                last_error = f"NAK opcode={opcode} error={error_code}"
                break
        raise RuntimeError(f"MAVFTP request failed: {last_error}")

    def _open(self, remote_path: str, *, write: bool) -> tuple[int, int]:
        opcode = OP_OPEN_FILE_WO if write else OP_OPEN_FILE_RO
        reply = self._request(opcode, payload=str(remote_path).encode("utf-8"))
        self.session = int(reply.session)
        remote_size = struct.unpack("<I", reply.payload[:4])[0] if len(reply.payload) >= 4 else 0
        return self.session, remote_size

    def close(self) -> None:
        if self.session:
            try:
                self._request(OP_TERMINATE_SESSION, session=self.session)
            finally:
                self.session = 0

    def remote_size(self, remote_path: str) -> int:
        try:
            _session, size = self._open(remote_path, write=False)
            return size
        finally:
            self.close()

    def read_prefix(self, remote_path: str, size: int) -> bytes:
        requested = max(0, int(size))
        output = bytearray()
        try:
            session, remote_size = self._open(remote_path, write=False)
            requested = min(requested, remote_size)
            while len(output) < requested:
                count = min(FTP_DATA_BYTES, requested - len(output))
                reply = self._request(
                    OP_READ_FILE,
                    session=session,
                    offset=len(output),
                    size_override=count,
                    allow_eof=True,
                )
                if reply.opcode == OP_NACK or not reply.payload:
                    break
                output.extend(reply.payload[:count])
            return bytes(output)
        finally:
            self.close()

    def verify_partial(self, local_path: str | Path, remote_path: str) -> PartialVerification:
        remote_size = self.remote_size(remote_path)
        local_size = Path(local_path).stat().st_size
        if remote_size > local_size:
            return PartialVerification(remote_size, "", "", False)
        with Path(local_path).open("rb") as handle:
            local_prefix = handle.read(remote_size)
        remote_prefix = self.read_prefix(remote_path, remote_size)
        local_digest = hashlib.sha256(local_prefix).hexdigest()
        remote_digest = hashlib.sha256(remote_prefix).hexdigest()
        return PartialVerification(
            remote_size=remote_size,
            local_prefix_sha256=local_digest,
            remote_prefix_sha256=remote_digest,
            matches=len(remote_prefix) == remote_size and local_digest == remote_digest,
        )

    def remote_crc32(self, remote_path: str) -> int:
        reply = self._request(OP_CALC_FILE_CRC32, payload=str(remote_path).encode("utf-8"))
        if len(reply.payload) < 4:
            raise RuntimeError("MAVFTP CRC32 reply is incomplete")
        return struct.unpack("<I", reply.payload[:4])[0]

    def upload(
        self,
        local_path: str | Path,
        remote_path: str,
        *,
        resume: bool,
        progress: Callable[[int, int], None] | None = None,
        cancelled: Callable[[], bool] | None = None,
    ) -> TransferEvidence:
        local_path = Path(local_path)
        local_size, local_sha256, local_crc32 = local_file_identity(local_path)
        offset = 0
        if resume:
            partial = self.verify_partial(local_path, remote_path)
            if not partial.matches:
                raise RuntimeError("Remote partial does not match the local prefix; Restart is required")
            offset = partial.remote_size
            session, _remote_size = self._open(remote_path, write=True)
        else:
            reply = self._request(OP_CREATE_FILE, payload=str(remote_path).encode("utf-8"))
            session = int(reply.session)
            self.session = session
        resume_offset = offset
        try:
            with local_path.open("rb") as handle:
                handle.seek(offset)
                while offset < local_size:
                    if cancelled is not None and cancelled():
                        raise RuntimeError("MAVFTP transfer cancelled")
                    chunk = handle.read(FTP_DATA_BYTES)
                    if not chunk:
                        break
                    self._request(OP_WRITE_FILE, session=session, offset=offset, payload=chunk)
                    offset += len(chunk)
                    if progress is not None:
                        progress(offset, local_size)
        finally:
            self.close()
        remote_size = self.remote_size(remote_path)
        remote_crc32 = self.remote_crc32(remote_path)
        if remote_size != local_size or remote_crc32 != local_crc32:
            raise RuntimeError(
                f"MAVFTP completion mismatch: local={local_size}/{local_crc32:08x}, "
                f"remote={remote_size}/{remote_crc32:08x}"
            )
        return TransferEvidence(
            remote_path=str(remote_path),
            local_path=str(local_path),
            local_size=local_size,
            local_sha256=local_sha256,
            local_crc32=local_crc32,
            remote_size=remote_size,
            remote_crc32=remote_crc32,
            resumed_from=resume_offset,
        )

    def download(
        self,
        remote_path: str,
        local_path: str | Path,
        *,
        resume: bool,
        progress: Callable[[int, int], None] | None = None,
        cancelled: Callable[[], bool] | None = None,
    ) -> TransferEvidence:
        local_path = Path(local_path)
        local_path.parent.mkdir(parents=True, exist_ok=True)
        partial_path = local_path.with_name(f".{local_path.name}.partial")
        remote_size = self.remote_size(remote_path)
        remote_crc32 = self.remote_crc32(remote_path)
        offset = 0
        if resume:
            if not partial_path.is_file():
                raise RuntimeError("Local partial is missing; Restart is required")
            offset = partial_path.stat().st_size
            if offset > remote_size:
                raise RuntimeError("Local partial is larger than remote; Restart is required")
            local_prefix = partial_path.read_bytes()
            remote_prefix = self.read_prefix(remote_path, offset)
            if len(remote_prefix) != offset or hashlib.sha256(local_prefix).digest() != hashlib.sha256(remote_prefix).digest():
                raise RuntimeError("Local partial does not match remote prefix; Restart is required")
        mode = "ab" if resume else "wb"
        resume_offset = offset
        try:
            session, opened_size = self._open(remote_path, write=False)
            if opened_size != remote_size:
                raise RuntimeError("Remote file size changed during download preflight")
            with partial_path.open(mode) as handle:
                while offset < remote_size:
                    if cancelled is not None and cancelled():
                        raise RuntimeError("MAVFTP transfer cancelled")
                    count = min(FTP_DATA_BYTES, remote_size - offset)
                    reply = self._request(
                        OP_READ_FILE,
                        session=session,
                        offset=offset,
                        size_override=count,
                        allow_eof=True,
                    )
                    if reply.opcode == OP_NACK or not reply.payload:
                        raise RuntimeError(f"MAVFTP download ended early at offset {offset}")
                    handle.write(reply.payload)
                    offset += len(reply.payload)
                    if progress is not None:
                        progress(offset, remote_size)
        finally:
            self.close()
        local_size, local_sha256, local_crc32 = local_file_identity(partial_path)
        if local_size != remote_size or local_crc32 != remote_crc32:
            raise RuntimeError(
                f"MAVFTP download mismatch: local={local_size}/{local_crc32:08x}, "
                f"remote={remote_size}/{remote_crc32:08x}"
            )
        os.replace(partial_path, local_path)
        return TransferEvidence(
            remote_path=str(remote_path),
            local_path=str(local_path),
            local_size=local_size,
            local_sha256=local_sha256,
            local_crc32=local_crc32,
            remote_size=remote_size,
            remote_crc32=remote_crc32,
            resumed_from=resume_offset,
        )


def atomic_write_transfer_evidence(path: str | Path, evidence: TransferEvidence) -> None:
    import json

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    try:
        temporary.write_text(
            json.dumps(evidence.to_mapping(), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
