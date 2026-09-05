from __future__ import annotations

import binascii
import hashlib
import json
import struct
import zlib
from pathlib import Path

from Workstation.qualify_native_evidence import audit_native_evidence


def _chunk(chunk_type: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + chunk_type
        + payload
        + struct.pack(">I", binascii.crc32(chunk_type + payload) & 0xFFFFFFFF)
    )


def _rgb_png(width: int, height: int, seed: int) -> bytes:
    rows = b"".join(
        bytes([0]) + bytes([(seed + row) % 256]) * (width * 3)
        for row in range(height)
    )
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", zlib.compress(rows))
        + _chunk(b"IEND", b"")
    )


def _descriptor(metadata: dict[str, object]) -> str:
    identity = {
        key: metadata[key]
        for key in (
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
    }
    canonical = json.dumps(
        identity,
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return hashlib.sha256(canonical).hexdigest()


def _append_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def _fixture(tmp_path: Path) -> tuple[Path, str, dict[str, object]]:
    runtime = "run-current"
    request_id = "native-request-1"
    pair_id = "pair-1"
    artifact_rows: list[dict[str, object]] = []
    metadata_by_side: dict[str, dict[str, object]] = {}
    ack_rows: list[dict[str, object]] = []
    event: dict[str, object] = {
        "schema": "workstation-measurement-data-record-v1",
        "event_type": "native_frame_pair_persisted",
        "runtime_generation": runtime,
        "request_id": request_id,
        "native_pair_status": "complete",
        "evidence_state": "SAVED",
        "evidence_artifact_count": 2,
    }
    for index, side in enumerate(("left", "right"), 1):
        source_key = f"{side}:7:{index}"
        metadata: dict[str, object] = {
            "camera_id": side,
            "stream_generation": 7,
            "frame_id": index,
            "frame_key": source_key,
            "source_native_frame_id": source_key,
            "timestamp_s": 10.0 + index,
            "source_timestamp_s": 20.0 + index,
            "native_width": 4,
            "native_height": 3,
            "native_channels": 3,
            "native_byte_length": 36,
            "native_pixel_format": "BGR24",
        }
        metadata["frame_descriptor_sha256"] = _descriptor(metadata)
        metadata_by_side[side] = metadata
        relative_path = f"frames/accepted-measurement/{pair_id}-{side}.png"
        path = tmp_path / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = _rgb_png(4, 3, index)
        path.write_bytes(payload)
        sha256 = hashlib.sha256(payload).hexdigest()
        artifact = {
            "schema": "native-frame-artifact-v1",
            "side": side,
            "frame_id": pair_id,
            "source_native_frame_id": source_key,
            "relative_path": relative_path,
            "size_bytes": len(payload),
            "sha256": sha256,
            "timestamp_s": metadata["timestamp_s"],
            "source_timestamp_s": metadata["source_timestamp_s"],
            "width": 4,
            "height": 3,
        }
        artifact_rows.append(artifact)
        prefix = f"native_{side}_"
        event.update(
            {
                prefix + "frame_id": pair_id,
                prefix + "source_frame_id": source_key,
                prefix + "frame_descriptor_sha256": metadata[
                    "frame_descriptor_sha256"
                ],
                prefix + "png_path": relative_path,
                prefix + "png_sha256": sha256,
                prefix + "width_px": 4,
                prefix + "height_px": 3,
            }
        )
        ack_rows.append(
            {
                "side": side,
                "source_frame_key": source_key,
                "frame_descriptor_sha256": metadata[
                    "frame_descriptor_sha256"
                ],
                "relative_path": relative_path,
                "size_bytes": len(payload),
                "sha256": sha256,
                "timestamp_s": metadata["timestamp_s"],
                "source_timestamp_s": metadata["source_timestamp_s"],
            }
        )
    canonical_ack = json.dumps(
        {
            "request_id": request_id,
            "pair_id": pair_id,
            "artifacts": ack_rows,
            "state": "SAVED",
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    event["evidence_ack_sha256"] = hashlib.sha256(canonical_ack).hexdigest()
    _append_jsonl(tmp_path / "native_frames.jsonl", [*artifact_rows, event])
    _append_jsonl(
        tmp_path / "measurements.jsonl",
        [
            {
                "measurement_id": runtime,
                "native_pair_request_id": request_id,
                "left": {
                    "timestamp_metadata_items": list(
                        metadata_by_side["left"].items()
                    )
                },
                "right": {
                    "timestamp_metadata_items": list(
                        metadata_by_side["right"].items()
                    )
                },
            }
        ],
    )
    _append_jsonl(tmp_path / "method_evaluations.jsonl", [])
    return tmp_path, runtime, event


def test_real_byte_hash_descriptor_origin_and_ack_chain_passes(tmp_path: Path) -> None:
    data_root, runtime, _event = _fixture(tmp_path)

    result = audit_native_evidence(data_root, runtime_generation=runtime)

    assert result["passed"] is True, result
    assert result["pair_events"] == 1
    assert result["artifact_files"] == 2
    assert result["origin_key_component_checks"] == 2
    assert result["failure_count"] == 0


def test_qualification_resolves_timestamped_stream_ledgers(tmp_path: Path) -> None:
    data_root, runtime, _event = _fixture(tmp_path)
    stamp = "20260903_120000_000000001+0900"
    for stem in ("native_frames", "measurements", "method_evaluations"):
        (data_root / f"{stem}.jsonl").rename(data_root / f"{stem}_{stamp}.jsonl")

    result = audit_native_evidence(data_root, runtime_generation=runtime)

    assert result["passed"] is True, result


def test_png_byte_tampering_fails_file_and_png_integrity(tmp_path: Path) -> None:
    data_root, runtime, event = _fixture(tmp_path)
    path = data_root / str(event["native_left_png_path"])
    payload = bytearray(path.read_bytes())
    payload[-8] ^= 1
    path.write_bytes(payload)

    result = audit_native_evidence(data_root, runtime_generation=runtime)

    assert result["passed"] is False
    assert any(
        "CRC mismatch" in failure or "SHA" in failure
        for failure in result["failures"]
    )


def test_event_descriptor_tampering_fails_origin_and_ack_chain(tmp_path: Path) -> None:
    data_root, runtime, event = _fixture(tmp_path)
    event["native_left_frame_descriptor_sha256"] = "0" * 64
    _append_jsonl(
        data_root / "native_frames.jsonl",
        [
            *[
                row
                for _, _, row in _jsonl_rows(data_root / "native_frames.jsonl")
                if row.get("schema") == "native-frame-artifact-v1"
            ],
            event,
        ],
    )

    result = audit_native_evidence(data_root, runtime_generation=runtime)

    assert result["passed"] is False
    assert any("origin descriptor" in failure for failure in result["failures"])
    assert any("ACK SHA mismatch" in failure for failure in result["failures"])


def _jsonl_rows(path: Path):
    for number, text in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if text:
            yield number, text, json.loads(text)


def test_origin_descriptor_is_scoped_to_selected_runtime(tmp_path: Path) -> None:
    data_root, runtime, event = _fixture(tmp_path)
    rows = [row for _, _, row in _jsonl_rows(data_root / "measurements.jsonl")]
    selected = rows[0]
    selected["left"]["timestamp_metadata_items"] = [
        [key, ("f" * 64 if key == "frame_descriptor_sha256" else value)]
        for key, value in selected["left"]["timestamp_metadata_items"]
    ]
    other = json.loads(json.dumps(rows[0]))
    other["measurement_id"] = "run-other"
    other["left"]["timestamp_metadata_items"] = [
        [
            key,
            (
                event["native_left_frame_descriptor_sha256"]
                if key == "frame_descriptor_sha256"
                else value
            ),
        ]
        for key, value in other["left"]["timestamp_metadata_items"]
    ]
    _append_jsonl(data_root / "measurements.jsonl", [selected, other])

    result = audit_native_evidence(data_root, runtime_generation=runtime)

    assert result["passed"] is False
    assert any("descriptor SHA recomputation" in failure for failure in result["failures"])
    assert any("origin descriptor" in failure for failure in result["failures"])


def test_historical_origin_can_reconstruct_missing_timestamp_from_acked_artifact(
    tmp_path: Path,
) -> None:
    data_root, runtime, _event = _fixture(tmp_path)
    rows = [row for _, _, row in _jsonl_rows(data_root / "measurements.jsonl")]
    for side in ("left", "right"):
        rows[0][side]["timestamp_metadata_items"] = [
            [key, value]
            for key, value in rows[0][side]["timestamp_metadata_items"]
            if key != "timestamp_s"
        ]
    _append_jsonl(data_root / "measurements.jsonl", rows)

    result = audit_native_evidence(data_root, runtime_generation=runtime)

    assert result["passed"] is True, result
    assert result["historical_origin_timestamp_missing"] == 2
    assert result["historical_descriptor_timestamp_fills"] == 2
