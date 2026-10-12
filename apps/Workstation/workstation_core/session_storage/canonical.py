"""Session canonical responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import hashlib
import json
import os
import tempfile
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from . import manifest as _storage_manifest
from . import schema as _storage_schema


def _authoritative_jsonl_row_bytes(row: dict[str, object]) -> bytes:
    try:
        encoded = json.dumps(
            row,
            ensure_ascii=False,
            sort_keys=True,
            allow_nan=False,
        )
    except ValueError as exc:
        raise ValueError(
            "authoritative JSONL row contains a non-finite number"
        ) from exc
    return (encoded + "\n").encode("utf-8")


def _apply_measurement_package_identity(
    rows: Iterable[dict[str, object]],
    *,
    measurement_id: str,
) -> None:
    """Add the package alias without changing ``measurement_id`` semantics."""

    for row in rows:
        declared = str(row.get("measurement_id") or "")
        alias = str(row.get("measurement_package_id") or "")
        if declared and declared != measurement_id:
            raise ValueError(
                f"row measurement_id {declared!r} does not match package {measurement_id!r}"
            )
        if alias and alias != measurement_id:
            raise ValueError(
                "row measurement_package_id does not match its measurement_id"
            )
        row["measurement_id"] = measurement_id
        row["measurement_package_id"] = measurement_id


def _apply_terminal_record_identities(
    rows: Iterable[dict[str, object]],
) -> None:
    """Materialize deterministic, non-positional terminal primary keys."""

    seen: set[str] = set()
    for row in rows:
        attempt_sha256 = str(row.get("measurement_attempt_sha256") or "")
        blocked_sha256 = str(row.get("blocked_pair_sha256") or "")
        populated = [value for value in (attempt_sha256, blocked_sha256) if value]
        if len(populated) > 1:
            raise ValueError("terminal row cannot be both an attempt and a blocked pair")
        if populated:
            digest = populated[0]
            if len(digest) != 64 or any(
                character not in "0123456789abcdef" for character in digest
            ):
                raise ValueError("terminal source identity must be lowercase SHA-256 hex")
            namespace = "attempt" if attempt_sha256 else "blocked"
            expected = f"{namespace}:{digest}"
        else:
            # Pre-G2/fixture rows may not carry a canonical attempt.  Their
            # complete content still supplies a stable, non-positional legacy
            # identity while new Workstation terminal rows use attempt/blocked.
            payload = {
                key: value
                for key, value in row.items()
                if key
                not in {
                    "terminal_record_id",
                    "sample_index",
                    "pair_sequence",
                }
            }
            try:
                encoded = json.dumps(
                    payload,
                    ensure_ascii=False,
                    allow_nan=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            except ValueError as exc:
                raise ValueError(
                    "terminal row contains a non-finite number"
                ) from exc
            expected = f"legacy:{hashlib.sha256(encoded).hexdigest()}"
        declared = str(row.get("terminal_record_id") or "")
        if declared and declared != expected:
            raise ValueError("terminal_record_id disagrees with canonical source identity")
        if expected in seen:
            raise ValueError(f"duplicate terminal_record_id: {expected}")
        row["terminal_record_id"] = expected
        seen.add(expected)


def _authoritative_jsonl_schema(
    rows: Iterable[dict[str, object]],
    *,
    normalization: str,
) -> dict[str, object]:
    return {
        "version": _storage_schema.AUTHORITATIVE_JSONL_SCHEMA_VERSION,
        "format": "jsonl",
        "encoding": "utf-8",
        "normalization": normalization,
        "fields": sorted({str(key) for row in rows for key in row}),
    }


@dataclass(frozen=True)
class JsonlReceipt:
    """Expected source bytes captured during the sole source iteration."""

    row_count: int
    size_bytes: int
    sha256: str
    fields: tuple[str, ...]


class JsonlRows:
    """Replay canonical disk rows without retaining row payloads in memory."""

    def __init__(self, path: Path, receipt: JsonlReceipt):
        self.path = path
        self.receipt = receipt

    def __len__(self):
        return self.receipt.row_count

    def __iter__(self):
        with self.path.open("rb") as handle:
            for raw_line in handle:
                yield json.loads(raw_line)


def _write_authoritative_jsonl(
    output: Path,
    rows: Iterable[dict[str, object]],
) -> JsonlReceipt:
    """Atomically write source rows once and retain only their byte receipt."""

    output.parent.mkdir(parents=True, exist_ok=True)
    temp_path: Path | None = None
    try:
        fd, temp_name = tempfile.mkstemp(
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
        )
        temp_path = Path(temp_name)
        digest = hashlib.sha256()
        fields: set[str] = set()
        row_count = 0
        size_bytes = 0
        with os.fdopen(fd, "wb") as handle:
            for row in rows:
                payload = _authoritative_jsonl_row_bytes(row)
                handle.write(payload)
                digest.update(payload)
                fields.update(str(key) for key in row)
                row_count += 1
                size_bytes += len(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, output)
        _storage_manifest._fsync_directory_path(output.parent)
        return JsonlReceipt(row_count, size_bytes, digest.hexdigest(), tuple(sorted(fields)))
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def _validate_authoritative_jsonl(
    output: Path,
    expected_rows: Sequence[dict[str, object]] | JsonlReceipt,
    *,
    role: str,
    artifact_path: str,
    normalization: str,
) -> dict[str, object]:
    """Re-read a staged JSONL artifact and return its verified manifest detail."""

    receipt = expected_rows if isinstance(expected_rows, JsonlReceipt) else None
    expected_count = receipt.row_count if receipt else len(expected_rows)
    digest = hashlib.sha256()
    fields: set[str] = set()
    row_count = 0
    try:
        with output.open("rb") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                digest.update(raw_line)
                if row_count >= expected_count:
                    raise OSError(
                        f"authoritative JSONL has an unexpected row: "
                        f"path={output}, line={line_number}"
                    )
                if receipt is None and raw_line != _authoritative_jsonl_row_bytes(expected_rows[row_count]):
                    raise OSError(
                        f"authoritative JSONL content mismatch: "
                        f"path={output}, line={line_number}"
                    )
                try:
                    decoded = json.loads(raw_line)
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    raise OSError(
                        f"authoritative JSONL is not valid UTF-8 JSON: "
                        f"path={output}, line={line_number}"
                    ) from exc
                if not isinstance(decoded, dict):
                    raise OSError(
                        f"authoritative JSONL row is not an object: "
                        f"path={output}, line={line_number}"
                    )
                try:
                    canonical_line = _authoritative_jsonl_row_bytes(decoded)
                except (TypeError, ValueError) as exc:
                    raise OSError(f"authoritative JSONL row is not canonical: {output}, line={line_number}") from exc
                if raw_line != canonical_line:
                    raise OSError(f"authoritative JSONL row is not canonical: {output}, line={line_number}")
                fields.update(str(key) for key in decoded)
                row_count += 1
    except OSError:
        raise

    if row_count != expected_count:
        raise OSError(
            f"authoritative JSONL row count mismatch: "
            f"path={output}, expected={expected_count}, actual={row_count}"
        )
    size = output.stat().st_size
    if receipt is not None and (
        size != receipt.size_bytes or digest.hexdigest() != receipt.sha256
        or tuple(sorted(fields)) != receipt.fields
    ):
        raise OSError(f"authoritative JSONL content mismatch: path={output}")
    schema = {
        "version": _storage_schema.AUTHORITATIVE_JSONL_SCHEMA_VERSION,
        "encoding": "utf-8",
        "normalization": normalization,
        "fields": sorted(fields),
    }
    return {
        "role": role,
        "media_type": "application/x-ndjson",
        "relative_path": artifact_path,
        "size_bytes": size,
        "sha256": digest.hexdigest(),
        "row_count": row_count,
        "schema_version": schema["version"],
        "authoritative": True,
        "derived_from": [],
        "encoding": schema["encoding"],
        "normalization": schema["normalization"],
        "fields": schema["fields"],
        "verified": True,
    }


def _scope_unpublished_artifact_details(
    manifest: dict[str, object],
    *,
    artifact_root: Path,
) -> None:
    """Keep failed-attempt details distinct from the stable published paths."""

    details = manifest.get("artifact_details")
    unpublished: dict[str, object] = {}
    if isinstance(details, dict):
        for name, value in details.items():
            if not isinstance(value, dict):
                continue
            unpublished[str(name)] = {
                **value,
                "path": (artifact_root / Path(str(name)).name).as_posix(),
                "published": False,
            }
    manifest["artifact_details"] = {}
    manifest["unpublished_artifact_details"] = unpublished
    manifest["authoritative_raw_artifacts_verified"] = False
