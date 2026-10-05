"""Small crash-tail recovery primitive for direct append-only JSONL ledgers."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any


@dataclass(frozen=True)
class JsonlRecovery:
    path: Path
    row_count: int
    repaired: bool = False
    normalized_newline: bool = False
    quarantined_tail: Path | None = None


def _decode_object(raw: bytes, *, path: Path, line_number: int) -> dict[str, Any]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant is forbidden: {value}")

    try:
        value = json.loads(raw.decode("utf-8"), parse_constant=reject_constant)
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise OSError(
            f"malformed JSONL record: path={path}, line={line_number}"
        ) from exc
    if not isinstance(value, dict):
        raise OSError(f"JSONL record is not an object: path={path}, line={line_number}")
    return value


def fsync_directory(path: Path) -> None:
    """Fsync one directory so a prior rename/write into it is durable.

    Canonical, consolidated implementation (see workstation_core/qimage_convert.py's
    sibling consolidation note for `sha256_file`/`safe_token`/`artifact_id`):
    always passes ``O_DIRECTORY`` (fails fast/clearly if *path* is not
    actually a directory) and always lets any ``OSError`` propagate -- no
    best-effort/"swallow the failure" mode. Call sites that previously
    tolerated a missing/unopenable directory must now handle that
    ``OSError`` explicitly themselves rather than relying on a silent no-op.
    """

    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def sha256_file(path: Path | str, *, chunk_size: int = 1024 * 1024) -> str:
    """Return the streaming SHA-256 hex digest of a file's bytes.

    Canonical, consolidated implementation -- see `fsync_directory`'s
    docstring for the sibling consolidation note.
    """

    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_replace(path: Path, payload: bytes) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".repair",
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def prepare_jsonl_append(path: Path | str) -> JsonlRecovery:
    """Validate all complete rows and recover only a torn final record."""

    resolved = Path(path)
    resolved.parent.mkdir(parents=True, exist_ok=True)
    if resolved.is_symlink():
        raise OSError(f"JSONL ledger must not be a symlink: {resolved}")
    if not resolved.exists():
        return JsonlRecovery(resolved, 0)
    if not resolved.is_file():
        raise OSError(f"JSONL ledger must be a regular file: {resolved}")
    payload = resolved.read_bytes()
    if not payload:
        return JsonlRecovery(resolved, 0)

    complete_payload = payload
    tail: bytes | None = None
    if not payload.endswith(b"\n"):
        prefix, separator, candidate = payload.rpartition(b"\n")
        complete_payload = prefix + separator if separator else b""
        tail = candidate

    complete_rows = complete_payload.splitlines(keepends=True)
    row_count = 0
    for line_number, raw_line in enumerate(complete_rows, start=1):
        if not raw_line.endswith(b"\n") or not raw_line[:-1].strip():
            raise OSError(
                f"malformed JSONL record: path={resolved}, line={line_number}"
            )
        _decode_object(raw_line[:-1], path=resolved, line_number=line_number)
        row_count += 1

    if tail is None:
        return JsonlRecovery(resolved, row_count)
    try:
        _decode_object(tail, path=resolved, line_number=row_count + 1)
    except OSError:
        quarantine_root = resolved.parent / ".quarantine"
        quarantine_root.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha256(tail).hexdigest()
        quarantine = quarantine_root / (
            f"{resolved.name}.{digest[:16]}.truncated-tail"
        )
        if quarantine.is_symlink():
            raise OSError(f"JSONL quarantine target must not be a symlink: {quarantine}")
        if quarantine.exists():
            if not quarantine.is_file() or quarantine.read_bytes() != tail:
                raise OSError("JSONL quarantine content-address collision")
        else:
            _atomic_replace(quarantine, tail)
        _atomic_replace(resolved, complete_payload)
        return JsonlRecovery(
            resolved,
            row_count,
            repaired=True,
            quarantined_tail=quarantine,
        )

    with resolved.open("ab") as handle:
        handle.write(b"\n")
        handle.flush()
        os.fsync(handle.fileno())
    return JsonlRecovery(
        resolved,
        row_count + 1,
        repaired=True,
        normalized_newline=True,
    )


def append_jsonl_record(
    path: Path | str,
    value: Mapping[str, Any],
    *,
    prepared: bool = False,
    fsync: bool = True,
) -> None:
    """Append one canonical JSON object after the one-time recovery gate."""

    output = Path(path)
    if output.is_symlink():
        raise OSError(f"JSONL ledger must not be a symlink: {output}")
    existed = output.exists()
    if not prepared:
        prepare_jsonl_append(output)
    data = json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"
    with output.open("ab") as handle:
        written = handle.write(data)
        if written != len(data):
            raise OSError(f"short JSONL append: {output}")
        handle.flush()
        if fsync:
            os.fsync(handle.fileno())
    if not existed and fsync:
        fsync_directory(output.parent)


__all__ = [
    "JsonlRecovery",
    "append_jsonl_record",
    "fsync_directory",
    "prepare_jsonl_append",
    "sha256_file",
]
