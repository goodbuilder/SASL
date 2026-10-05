"""Independently audit durable native-frame evidence for one Workstation run.

This verifier intentionally uses only the Python standard library.  It does
not import the native ring, writer, or frame-store implementations whose
output it checks.
"""

from __future__ import annotations

import argparse
import binascii
import hashlib
import json
import re
import struct
import zlib
from collections import Counter, defaultdict
from collections.abc import Iterator, Mapping, Sequence
from pathlib import Path
from typing import Any


_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_ORIGIN_FILENAMES = ("measurements.jsonl", "method_evaluations.jsonl")
_DESCRIPTOR_IDENTITY_KEYS = (
    "camera_id",
    "stream_generation",
    "frame_id",
    "frame_key",
    "timestamp_s",
    "source_timestamp_s",
    "native_width",
    "native_height",
    "native_channels",
    "native_byte_length",
    "native_pixel_format",
)


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_path(path: Path) -> str:
    # Deliberately NOT shared with workstation_core.strict_jsonl.sha256_file:
    # this verifier is stdlib-only by design (see module docstring) so it
    # never trusts the implementation it is independently auditing.
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()




def _iter_jsonl(path: Path) -> Iterator[tuple[int, str, dict[str, Any]]]:
    with path.open("r", encoding="utf-8") as handle:
        for line_number, text in enumerate(handle, 1):
            if not text.strip():
                continue
            try:
                payload = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"{path.name}:{line_number}: invalid JSON: {exc.msg}"
                ) from exc
            if not isinstance(payload, dict):
                raise ValueError(
                    f"{path.name}:{line_number}: JSONL row must be an object"
                )
            yield line_number, text, payload


def _canonical_descriptor_sha256(metadata: Mapping[str, Any]) -> str:
    identity = {
        key: metadata.get(key)
        for key in _DESCRIPTOR_IDENTITY_KEYS
    }
    canonical = json.dumps(
        identity,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return _sha256_bytes(canonical)


def _metadata_mapping(items: object) -> dict[str, Any] | None:
    if not isinstance(items, list):
        return None
    result: dict[str, Any] = {}
    for item in items:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            return None
        result[str(item[0])] = item[1]
    return result


def _collect_origin_descriptors(
    value: object,
    *,
    location: str,
    descriptors: dict[str, dict[str, list[dict[str, Any]]]],
    failures: list[str],
    stats: Counter[str],
    depth: int = 0,
) -> None:
    if depth > 30:
        failures.append(f"{location}: nested evidence exceeds depth 30")
        return
    if isinstance(value, Mapping):
        if "timestamp_metadata_items" in value:
            metadata = _metadata_mapping(value.get("timestamp_metadata_items"))
            if metadata is None:
                failures.append(f"{location}: malformed timestamp_metadata_items")
            else:
                frame_key = str(metadata.get("frame_key") or "")
                descriptor_sha256 = str(
                    metadata.get("frame_descriptor_sha256") or ""
                ).strip().lower()
                parts = frame_key.rsplit(":", 2)
                if len(parts) != 3 or not parts[0]:
                    failures.append(f"{location}: malformed frame_key {frame_key!r}")
                else:
                    try:
                        generation = int(parts[1])
                        frame_id = int(parts[2])
                        components_match = (
                            str(metadata.get("camera_id") or "") == parts[0]
                            and int(metadata.get("stream_generation")) == generation
                            and int(metadata.get("frame_id")) == frame_id
                            and str(metadata.get("source_native_frame_id") or "")
                            == frame_key
                        )
                    except (TypeError, ValueError):
                        components_match = False
                    if not components_match:
                        failures.append(
                            f"{location}: FrameKey compact/component mismatch {frame_key!r}"
                        )
                    else:
                        stats["origin_key_component_checks"] += 1
                if not _SHA256_RE.fullmatch(descriptor_sha256):
                    failures.append(
                        f"{location}: malformed descriptor SHA for {frame_key!r}"
                    )
                else:
                    missing_identity = [
                        key for key in _DESCRIPTOR_IDENTITY_KEYS if key not in metadata
                    ]
                    if missing_identity == ["timestamp_s"]:
                        stats["historical_origin_timestamp_missing"] += 1
                    elif missing_identity:
                        failures.append(
                            f"{location}: descriptor identity fields missing for "
                            f"{frame_key!r}: {','.join(missing_identity)}"
                        )
                    elif _canonical_descriptor_sha256(metadata) != descriptor_sha256:
                        failures.append(
                            f"{location}: descriptor SHA recomputation mismatch for "
                            f"{frame_key!r}"
                        )
                    else:
                        stats["origin_descriptor_recomputations"] += 1
                    descriptors[frame_key][descriptor_sha256].append(metadata)

        for key, item in value.items():
            _collect_origin_descriptors(
                item,
                location=f"{location}.{key}",
                descriptors=descriptors,
                failures=failures,
                stats=stats,
                depth=depth + 1,
            )
        return

    if isinstance(value, list):
        for index, item in enumerate(value):
            _collect_origin_descriptors(
                item,
                location=f"{location}[{index}]",
                descriptors=descriptors,
                failures=failures,
                stats=stats,
                depth=depth + 1,
            )
        return

    if (
        isinstance(value, str)
        and value[:1] in {"[", "{"}
        and len(value) <= 2_000_000
    ):
        try:
            decoded = json.loads(value)
        except json.JSONDecodeError:
            return
        stats["nested_json_records_checked"] += 1
        _collect_origin_descriptors(
            decoded,
            location=f"{location}<json>",
            descriptors=descriptors,
            failures=failures,
            stats=stats,
            depth=depth + 1,
        )


def _inspect_png(payload: bytes) -> dict[str, int]:
    if not payload.startswith(_PNG_SIGNATURE):
        raise ValueError("PNG signature missing")
    offset = len(_PNG_SIGNATURE)
    ihdr: tuple[int, int, int, int, int, int, int] | None = None
    compressed: list[bytes] = []
    saw_iend = False
    while offset < len(payload):
        if len(payload) - offset < 12:
            raise ValueError("truncated PNG chunk header")
        length = struct.unpack(">I", payload[offset : offset + 4])[0]
        chunk_type = payload[offset + 4 : offset + 8]
        data_start = offset + 8
        data_end = data_start + length
        crc_end = data_end + 4
        if crc_end > len(payload):
            raise ValueError("truncated PNG chunk payload")
        chunk = payload[data_start:data_end]
        stored_crc = struct.unpack(">I", payload[data_end:crc_end])[0]
        actual_crc = binascii.crc32(chunk_type + chunk) & 0xFFFFFFFF
        if stored_crc != actual_crc:
            raise ValueError(
                f"PNG {chunk_type.decode('ascii', errors='replace')} CRC mismatch"
            )
        if chunk_type == b"IHDR":
            if ihdr is not None or length != 13:
                raise ValueError("invalid PNG IHDR")
            ihdr = struct.unpack(">IIBBBBB", chunk)
        elif chunk_type == b"IDAT":
            compressed.append(chunk)
        elif chunk_type == b"IEND":
            if length != 0:
                raise ValueError("invalid PNG IEND")
            saw_iend = True
            offset = crc_end
            break
        offset = crc_end
    if ihdr is None or not compressed or not saw_iend or offset != len(payload):
        raise ValueError("PNG chunk sequence is incomplete or has trailing bytes")
    width, height, bit_depth, color_type, compression, filtering, interlace = ihdr
    if (
        width <= 0
        or height <= 0
        or bit_depth != 8
        or color_type != 2
        or compression != 0
        or filtering != 0
        or interlace != 0
    ):
        raise ValueError(
            "native PNG must be non-interlaced RGB8 with standard compression/filter"
        )
    try:
        scanlines = zlib.decompress(b"".join(compressed))
    except zlib.error as exc:
        raise ValueError(f"PNG IDAT decompression failed: {exc}") from exc
    row_bytes = width * 3
    stride = row_bytes + 1
    if len(scanlines) != height * stride:
        raise ValueError("PNG decompressed byte count does not match IHDR")
    if any(scanlines[index * stride] > 4 for index in range(height)):
        raise ValueError("PNG scanline uses an invalid filter type")
    return {
        "width": width,
        "height": height,
        "bit_depth": bit_depth,
        "color_type": color_type,
        "decompressed_bytes": len(scanlines),
    }


def _failure(failures: list[str], message: str) -> None:
    failures.append(message)


def _runtime_stream_path(root: Path, stem: str, runtime: str) -> Path:
    """Resolve a legacy or timestamped stream containing one runtime."""

    fixed = root / f"{stem}.jsonl"
    candidates = [fixed]
    candidates.extend(sorted(root.glob(f"{stem}_*.jsonl"), key=lambda path: path.name))
    for path in candidates:
        if not path.is_file() or path.is_symlink():
            continue
        try:
            for line in path.read_text(encoding="utf-8").splitlines():
                if runtime in line:
                    return path
        except (OSError, UnicodeError):
            continue
    # Apply the same symlink distrust as the search loop above: a symlinked
    # candidate must not be preferred just because nothing matched the
    # runtime marker, or tampering could swap in a symlink to attacker
    # controlled content and still have it silently accepted here.
    regular_candidates = [
        path for path in candidates if path.is_file() and not path.is_symlink()
    ]
    if fixed.is_file() and not fixed.is_symlink():
        return fixed
    if regular_candidates:
        return regular_candidates[-1]
    return candidates[-1]


def audit_native_evidence(
    data_root: str | Path,
    *,
    runtime_generation: str,
) -> dict[str, Any]:
    root = Path(data_root).expanduser().resolve()
    runtime = str(runtime_generation or "").strip()
    if not runtime:
        raise ValueError("runtime_generation is required")
    native_path = _runtime_stream_path(root, "native_frames", runtime)
    origin_paths = tuple(
        _runtime_stream_path(root, Path(name).stem, runtime)
        for name in _ORIGIN_FILENAMES
    )
    for path in (native_path, *origin_paths):
        if not path.is_file():
            raise ValueError(f"required evidence file is missing: {path}")

    failures: list[str] = []
    stats: Counter[str] = Counter()
    native_rows: dict[str, list[tuple[int, dict[str, Any]]]] = defaultdict(list)
    events: list[tuple[int, dict[str, Any]]] = []
    for line_number, _text, row in _iter_jsonl(native_path):
        if row.get("schema") == "native-frame-artifact-v1":
            relative_path = str(row.get("relative_path") or "")
            native_rows[relative_path].append((line_number, row))
        elif (
            row.get("event_type") == "native_frame_pair_persisted"
            and row.get("runtime_generation") == runtime
        ):
            events.append((line_number, row))

    if not events:
        failures.append(f"no native_frame_pair_persisted event for {runtime}")

    descriptors: dict[str, dict[str, list[dict[str, Any]]]] = defaultdict(
        lambda: defaultdict(list)
    )
    selected_origin_texts: list[str] = []
    input_hashes = {native_path.name: _sha256_path(native_path)}
    for path in origin_paths:
        input_hashes[path.name] = _sha256_path(path)
        for line_number, text, row in _iter_jsonl(path):
            if runtime not in text:
                continue
            selected_origin_texts.append(text)
            stats[f"{path.stem}_runtime_rows"] += 1
            _collect_origin_descriptors(
                row,
                location=f"{path.name}:{line_number}",
                descriptors=descriptors,
                failures=failures,
                stats=stats,
            )

    seen_requests: set[str] = set()
    seen_paths: set[str] = set()
    purposes: Counter[str] = Counter()
    manifest: list[dict[str, Any]] = []
    total_artifact_bytes = 0

    for line_number, event in events:
        request_id = str(event.get("request_id") or "")
        if not request_id:
            _failure(failures, f"native_frames.jsonl:{line_number}: request_id missing")
        elif request_id in seen_requests:
            _failure(
                failures,
                f"native_frames.jsonl:{line_number}: duplicate request {request_id}",
            )
        seen_requests.add(request_id)
        if not any(request_id in text for text in selected_origin_texts):
            _failure(
                failures,
                f"native_frames.jsonl:{line_number}: request absent from run origins",
            )
        if (
            event.get("evidence_state") != "SAVED"
            or event.get("native_pair_status") != "complete"
            or event.get("evidence_artifact_count") != 2
        ):
            _failure(
                failures,
                f"native_frames.jsonl:{line_number}: pair is not SAVED/complete/2",
            )

        left_frame_id = str(event.get("native_left_frame_id") or "")
        right_frame_id = str(event.get("native_right_frame_id") or "")
        if not left_frame_id or left_frame_id != right_frame_id:
            _failure(
                failures,
                f"native_frames.jsonl:{line_number}: left/right pair ID mismatch",
            )

        ack_rows: list[dict[str, Any]] = []
        for side in ("left", "right"):
            prefix = f"native_{side}_"
            relative_path = str(event.get(prefix + "png_path") or "")
            if relative_path in seen_paths:
                _failure(
                    failures,
                    f"native_frames.jsonl:{line_number}: duplicate path {relative_path}",
                )
            seen_paths.add(relative_path)
            parts = relative_path.split("/")
            purpose = parts[1] if len(parts) > 2 and parts[0] == "frames" else "?"
            purposes[purpose] += 1

            candidates = native_rows.get(relative_path, [])
            if len(candidates) != 1:
                _failure(
                    failures,
                    f"native_frames.jsonl:{line_number}: expected one artifact row for "
                    f"{relative_path}, found {len(candidates)}",
                )
                continue
            artifact_line, artifact = candidates[0]
            artifact_path = (root / relative_path).resolve()
            try:
                artifact_path.relative_to(root)
            except ValueError:
                _failure(
                    failures,
                    f"native_frames.jsonl:{line_number}: path escapes data root: "
                    f"{relative_path}",
                )
                continue
            if not artifact_path.is_file():
                _failure(
                    failures,
                    f"native_frames.jsonl:{line_number}: PNG missing: {relative_path}",
                )
                continue

            payload = artifact_path.read_bytes()
            actual_sha256 = _sha256_bytes(payload)
            actual_size = len(payload)
            total_artifact_bytes += actual_size
            try:
                png = _inspect_png(payload)
            except ValueError as exc:
                _failure(
                    failures,
                    f"{relative_path}: {exc}",
                )
                continue

            event_sha256 = str(event.get(prefix + "png_sha256") or "").lower()
            artifact_sha256 = str(artifact.get("sha256") or "").lower()
            source_key = str(event.get(prefix + "source_frame_id") or "")
            descriptor_sha256 = str(
                event.get(prefix + "frame_descriptor_sha256") or ""
            ).lower()
            origin_candidates = descriptors.get(source_key, {}).get(
                descriptor_sha256,
                [],
            )
            descriptor_reconstructed = False
            used_historical_timestamp_fill = False
            for origin_metadata in origin_candidates:
                identity = {
                    key: origin_metadata.get(key)
                    for key in _DESCRIPTOR_IDENTITY_KEYS
                }
                historical_fill = "timestamp_s" not in origin_metadata
                if historical_fill:
                    identity["timestamp_s"] = artifact.get("timestamp_s")
                if (
                    _canonical_descriptor_sha256(identity) == descriptor_sha256
                    and identity["timestamp_s"] == artifact.get("timestamp_s")
                    and identity["source_timestamp_s"]
                    == artifact.get("source_timestamp_s")
                    and identity["native_width"] == png["width"]
                    and identity["native_height"] == png["height"]
                    and identity["native_channels"] == 3
                    and identity["native_byte_length"]
                    == png["width"] * png["height"] * 3
                    and identity["native_pixel_format"] == "BGR24"
                ):
                    descriptor_reconstructed = True
                    used_historical_timestamp_fill = historical_fill
                    break
            if used_historical_timestamp_fill:
                stats["historical_descriptor_timestamp_fills"] += 1
            checks = (
                (actual_sha256 == event_sha256, "file/event SHA"),
                (actual_sha256 == artifact_sha256, "file/artifact SHA"),
                (actual_size == artifact.get("size_bytes"), "size"),
                (artifact.get("side") == side, "side"),
                (
                    str(artifact.get("frame_id") or "")
                    == str(event.get(prefix + "frame_id") or ""),
                    "frame ID",
                ),
                (
                    str(artifact.get("source_native_frame_id") or "") == source_key,
                    "source FrameKey",
                ),
                (
                    png["width"]
                    == event.get(prefix + "width_px")
                    == artifact.get("width"),
                    "width",
                ),
                (
                    png["height"]
                    == event.get(prefix + "height_px")
                    == artifact.get("height"),
                    "height",
                ),
                (
                    bool(_SHA256_RE.fullmatch(descriptor_sha256)),
                    "descriptor SHA syntax",
                ),
                (
                    bool(origin_candidates),
                    "run-scoped origin descriptor",
                ),
                (
                    descriptor_reconstructed,
                    "descriptor canonical reconstruction",
                ),
                (source_key.startswith(f"{side}:"), "source side"),
            )
            for accepted, label in checks:
                if not accepted:
                    _failure(
                        failures,
                        f"native_frames.jsonl:{line_number}/{artifact_line} "
                        f"{side}: {label} mismatch",
                    )

            ack_rows.append(
                {
                    "side": side,
                    "source_frame_key": source_key,
                    "frame_descriptor_sha256": descriptor_sha256,
                    "relative_path": relative_path,
                    "size_bytes": artifact.get("size_bytes"),
                    "sha256": artifact.get("sha256"),
                    "timestamp_s": artifact.get("timestamp_s"),
                    "source_timestamp_s": artifact.get("source_timestamp_s"),
                }
            )
            manifest.append(
                {
                    "request_id": request_id,
                    "side": side,
                    "relative_path": relative_path,
                    "sha256": actual_sha256,
                    "source_frame_key": source_key,
                    "frame_descriptor_sha256": descriptor_sha256,
                }
            )

        if len(ack_rows) == 2:
            canonical_ack = json.dumps(
                {
                    "request_id": request_id,
                    "pair_id": left_frame_id,
                    "artifacts": ack_rows,
                    "state": "SAVED",
                },
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
            if _sha256_bytes(canonical_ack) != event.get("evidence_ack_sha256"):
                _failure(
                    failures,
                    f"native_frames.jsonl:{line_number}: ACK SHA mismatch",
                )

    canonical_manifest = json.dumps(
        sorted(manifest, key=lambda item: (item["request_id"], item["side"])),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return {
        "schema": "workstation-native-evidence-qualification-v1",
        "passed": not failures,
        "data_root": str(root),
        "runtime_generation": runtime,
        "pair_events": len(events),
        "artifact_files": len(manifest),
        "total_artifact_bytes": total_artifact_bytes,
        "purposes": dict(sorted(purposes.items())),
        "unique_requests": len(seen_requests),
        "unique_paths": len(seen_paths),
        "origin_frame_keys": len(descriptors),
        "origin_key_component_checks": stats["origin_key_component_checks"],
        "origin_descriptor_recomputations": stats[
            "origin_descriptor_recomputations"
        ],
        "historical_origin_timestamp_missing": stats[
            "historical_origin_timestamp_missing"
        ],
        "historical_descriptor_timestamp_fills": stats[
            "historical_descriptor_timestamp_fills"
        ],
        "nested_json_records_checked": stats["nested_json_records_checked"],
        "origin_runtime_rows": {
            name: stats[f"{Path(name).stem}_runtime_rows"]
            for name in _ORIGIN_FILENAMES
        },
        "input_sha256": input_hashes,
        "aggregate_manifest_sha256": _sha256_bytes(canonical_manifest),
        "failure_count": len(failures),
        "failures": failures,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Audit one Workstation runtime generation's persisted native PNG, "
            "FrameKey, descriptor, origin, and ACK hashes without product imports."
        )
    )
    parser.add_argument("--data-root", required=True, help="Measurement data root")
    parser.add_argument(
        "--runtime-generation",
        required=True,
        help="Exact run-... runtime generation to audit",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        result = audit_native_evidence(
            args.data_root,
            runtime_generation=args.runtime_generation,
        )
    except (OSError, ValueError) as exc:
        result = {
            "schema": "workstation-native-evidence-qualification-v1",
            "passed": False,
            "failure_count": 1,
            "failures": [f"{type(exc).__name__}: {exc}"],
        }
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["audit_native_evidence", "main"]
