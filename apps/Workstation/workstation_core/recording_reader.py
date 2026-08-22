"""JSONL-first Workstation measurement reader with legacy Parquet fallback."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any


ROLE_SOURCES = {
    "raw/stereo_measurements": (
        "raw_stereo_measurements",
        "measurement_pairs",
        "measurement_pairs.parquet",
        None,
    ),
    "raw/camera_left": (
        "raw_camera_left",
        "camera_timeline",
        "camera_timeline.parquet",
        ("side", "left"),
    ),
    "raw/camera_right": (
        "raw_camera_right",
        "camera_timeline",
        "camera_timeline.parquet",
        ("side", "right"),
    ),
    "raw/px4_left": (
        "raw_px4_left",
        "pixhawk_timeline",
        "pixhawk_timeline.parquet",
        ("source_id", "left"),
    ),
    "raw/px4_right": (
        "raw_px4_right",
        "pixhawk_timeline",
        "pixhawk_timeline.parquet",
        ("source_id", "right"),
    ),
    "raw/px4_drone": (
        "raw_px4_drone",
        "pixhawk_timeline",
        "pixhawk_timeline.parquet",
        ("source_id", "drone"),
    ),
    "raw/gnss_base": (
        "raw_gnss_base",
        "gnss_base_timeline",
        "gnss_base_timeline.parquet",
        None,
    ),
    "raw/extrinsics_comparison": (
        "raw_extrinsics_comparison",
        "geometry_comparison",
        "geometry_comparison.parquet",
        None,
    ),
    "context/geometry_contexts": (
        "geometry_contexts",
        None,
        None,
        None,
    ),
    "context/time_sync_epochs": (
        "time_sync_epochs",
        None,
        None,
        None,
    ),
}

LEGACY_PARQUET_SUFFIXES = {
    "raw/stereo_measurements": (
        "measurement_pairs.parquet",
        "distance_measurements.parquet",
    ),
    "raw/camera_left": (
        "camera_timeline.parquet",
        "camera_left_timeline.parquet",
    ),
    "raw/camera_right": (
        "camera_timeline.parquet",
        "camera_right_timeline.parquet",
    ),
    "raw/px4_left": (
        "pixhawk_timeline.parquet",
        "px4_left_timeline.parquet",
    ),
    "raw/px4_right": (
        "pixhawk_timeline.parquet",
        "px4_right_timeline.parquet",
    ),
    "raw/px4_drone": (
        "pixhawk_timeline.parquet",
        "px4_drone_timeline.parquet",
    ),
    "raw/gnss_base": ("gnss_base_timeline.parquet",),
    "raw/extrinsics_comparison": ("geometry_comparison.parquet",),
}


@dataclass(frozen=True)
class RecordingStream:
    logical_role: str
    path: Path
    rows: tuple[dict[str, object], ...]
    authoritative: bool
    legacy_fallback: bool
    descriptor: Mapping[str, object]


def _strict_json(path: Path) -> dict[str, object]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant is forbidden: {value}")

    value = json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError(f"JSON file is not an object: {path}")
    return value


def _resolve_inside(root: Path, relative_path: object) -> Path:
    raw = str(relative_path or "")
    relative = Path(raw)
    if (
        not raw
        or relative.is_absolute()
        or any(part == ".." for part in relative.parts)
        or "\x00" in raw
    ):
        raise ValueError("recording artifact path is unsafe")
    resolved = (root / relative).resolve(strict=True)
    resolved.relative_to(root)
    if not resolved.is_file():
        raise ValueError("recording artifact is not a regular file")
    return resolved


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_verified_jsonl(
    path: Path,
    descriptor: Mapping[str, object],
) -> tuple[dict[str, object], ...]:
    expected_size = descriptor.get("size_bytes")
    if expected_size is None or int(expected_size) != path.stat().st_size:
        raise OSError("authoritative JSONL size does not match measurement manifest")
    expected_sha = str(descriptor.get("sha256") or "")
    if len(expected_sha) != 64 or _sha256(path) != expected_sha:
        raise OSError("authoritative JSONL SHA-256 does not match measurement manifest")
    payload = path.read_bytes()
    if payload and not payload.endswith(b"\n"):
        raise OSError("finalized authoritative JSONL has a torn final record")
    rows: list[dict[str, object]] = []
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant: {value}")

    for line_number, raw_line in enumerate(payload.splitlines(), start=1):
        if not raw_line.strip():
            raise OSError(f"blank authoritative JSONL row at line {line_number}")
        try:
            row = json.loads(
                raw_line.decode("utf-8"),
                parse_constant=reject_constant,
            )
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
            raise OSError(
                f"malformed authoritative JSONL row at line {line_number}"
            ) from exc
        if not isinstance(row, dict):
            raise OSError(
                f"authoritative JSONL row is not an object at line {line_number}"
            )
        rows.append(row)
    if descriptor.get("row_count") is None or int(descriptor["row_count"]) != len(rows):
        raise OSError("authoritative JSONL row count does not match measurement manifest")
    return tuple(rows)


def _legacy_resource_path(
    root: Path,
    manifest: Mapping[str, object],
    filename: str,
) -> Path | None:
    resources = manifest.get("resource_files")
    if isinstance(resources, Mapping):
        for value in resources.values():
            if str(value).endswith(filename):
                try:
                    return _resolve_inside(root, value)
                except (FileNotFoundError, OSError, ValueError):
                    continue
    candidates = (
        root / "data" / filename,
        root / filename,
    )
    for candidate in candidates:
        try:
            resolved = candidate.resolve(strict=True)
            resolved.relative_to(root)
        except (FileNotFoundError, OSError, ValueError):
            continue
        if resolved.is_file():
            return resolved
    return None


def _read_legacy_parquet(
    path: Path,
    artifact_key: str,
) -> tuple[dict[str, object], ...]:
    try:
        from Workstation.workstation_core.session_parquet import read_lossless_rows

        return tuple(read_lossless_rows(path, artifact_key))
    except (OSError, TypeError, ValueError):
        import pyarrow.parquet as pq

        return tuple(dict(row) for row in pq.read_table(path).to_pylist())


def read_measurement_stream(
    measurement_root: Path | str,
    logical_role: str = "raw/stereo_measurements",
) -> RecordingStream:
    """Read one logical stream, refusing Parquet fallback for a declared v3 JSONL."""

    root = Path(measurement_root).expanduser().resolve(strict=True)
    if not root.is_dir():
        raise ValueError("measurement root is not a directory")
    role = str(logical_role)
    if role not in ROLE_SOURCES:
        raise ValueError(f"unknown Workstation recording role: {role}")
    jsonl_key, parquet_key, parquet_filename, partition = ROLE_SOURCES[role]
    manifest_path = root / "measurement.json"
    manifest: dict[str, object] = {}
    if manifest_path.is_file():
        manifest = _strict_json(manifest_path)
    details = manifest.get("artifact_details")
    details = details if isinstance(details, Mapping) else {}
    descriptor = details.get(jsonl_key)
    is_jsonl_storage = str(manifest.get("schema") or "").endswith(
        "measurement-storage-v3"
    )
    if is_jsonl_storage and manifest.get("status") != "finalized":
        raise OSError("v3 measurement generation is not finalized")
    declared_measurement_id = str(manifest.get("measurement_id") or "")
    if (
        is_jsonl_storage
        and root.parent.name == "measurements"
        and declared_measurement_id != root.name
    ):
        raise OSError("v3 measurement identity does not match its directory")
    if isinstance(descriptor, Mapping):
        if descriptor.get("authoritative") is not True:
            raise OSError("declared canonical JSONL is not authoritative")
        if descriptor.get("media_type") != "application/x-ndjson":
            raise OSError("declared canonical JSONL has the wrong media type")
        path = _resolve_inside(root, descriptor.get("relative_path"))
        rows = _read_verified_jsonl(path, descriptor)
        return RecordingStream(role, path, rows, True, False, dict(descriptor))
    if is_jsonl_storage:
        raise OSError(f"v3 measurement manifest omits authoritative stream: {role}")
    if parquet_key is None or parquet_filename is None:
        raise FileNotFoundError(f"legacy package has no {role} stream")
    path = next(
        (
            candidate
            for filename in LEGACY_PARQUET_SUFFIXES.get(role, (parquet_filename,))
            for candidate in (_legacy_resource_path(root, manifest, filename),)
            if candidate is not None
        ),
        None,
    )
    if path is None:
        raise FileNotFoundError(f"legacy Parquet stream is missing: {role}")
    rows = _read_legacy_parquet(path, str(parquet_key))
    if partition is not None and path.name in {
        "camera_timeline.parquet",
        "pixhawk_timeline.parquet",
    }:
        field, expected = partition
        rows = tuple(row for row in rows if str(row.get(field) or "") == expected)
    return RecordingStream(
        role,
        path,
        rows,
        False,
        True,
        {
            "role": str(parquet_key),
            "media_type": "application/vnd.apache.parquet",
            "relative_path": path.relative_to(root).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
            "authoritative": True,
            "legacy_compatibility": "parquet_only",
        },
    )


__all__ = ["ROLE_SOURCES", "RecordingStream", "read_measurement_stream"]
