"""PX4 firmware bundle and verified uploader evidence helpers."""

from __future__ import annotations

import hashlib
import json
import os
import re
from dataclasses import asdict, dataclass
from pathlib import Path


PINNED_PX_UPLOADER_SHA256 = "5cae28e8ebb0cdb1c25035034afdb44b2fc09f3893c32d54f2a31e96e0ac144c"


@dataclass(frozen=True)
class Px4FirmwareIdentity:
    path: str
    sha256: str
    board_id: int
    board_revision: int
    image_size: int
    image_maxsize: int
    git_identity: str
    git_hash: str

    def to_mapping(self) -> dict[str, object]:
        return asdict(self)


def read_px4_firmware_identity(path: str | Path) -> Px4FirmwareIdentity:
    source = Path(path)
    raw = source.read_bytes()
    payload = json.loads(raw)
    if payload.get("magic") != "PX4FWv1":
        raise ValueError("firmware is not a PX4FWv1 bundle")
    board_id = int(payload.get("board_id") or 0)
    image_size = int(payload.get("image_size") or 0)
    image_maxsize = int(payload.get("image_maxsize") or 0)
    if board_id <= 0 or image_size <= 0 or image_maxsize <= 0 or image_size > image_maxsize:
        raise ValueError("firmware board/image metadata is invalid")
    return Px4FirmwareIdentity(
        path=str(source),
        sha256=hashlib.sha256(raw).hexdigest(),
        board_id=board_id,
        board_revision=int(payload.get("board_revision") or 0),
        image_size=image_size,
        image_maxsize=image_maxsize,
        git_identity=str(payload.get("git_identity") or ""),
        git_hash=str(payload.get("git_hash") or ""),
    )


def verify_pinned_uploader(path: str | Path) -> str:
    digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    if digest != PINNED_PX_UPLOADER_SHA256:
        raise ValueError(
            f"PX4 uploader hash mismatch: expected {PINNED_PX_UPLOADER_SHA256}, got {digest}"
        )
    return digest


_BOARD_PATTERN = re.compile(
    r"Found board id:\s*(?P<board>\d+),(?P<revision>\d+)\s+bootloader protocol revision\s+(?P<protocol>\d+)"
)


def parse_px_uploader_output(output: str) -> dict[str, object]:
    match = _BOARD_PATTERN.search(str(output))
    return {
        "detected_board_id": None if match is None else int(match.group("board")),
        "detected_board_revision": None if match is None else int(match.group("revision")),
        "bootloader_protocol_revision": None if match is None else int(match.group("protocol")),
        "erase": "PASS" if "Erase" in output else "UNKNOWN",
        "program": "PASS" if "Program" in output else "UNKNOWN",
        "verify": "PASS" if "Verify" in output else "UNKNOWN",
        "reboot": "PASS" if "Rebooting." in output else "UNKNOWN",
    }


def atomic_write_firmware_evidence(path: str | Path, payload: dict[str, object]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    try:
        temporary.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
