from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import pytest

from Workstation.workstation_core.rtcm_pipeline import (
    RtcmDataPlane,
    RtcmEvidenceContext,
    RtcmRouteConfig,
    append_rtcm_evidence,
    load_rtcm_evidence_high_water,
)


# Independently frozen RTCM3 fixture: message type 1077, payload 43 50 00,
# trailing CRC-24Q 8d 44 be.  The pipeline test never calls the CRC encoder.
RTCM_1077 = bytes.fromhex("d300034350008d44be")


def test_data_plane_preserves_split_ntrip_bytes_and_routes_only_complete_frames() -> None:
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    plane.configure(
        RtcmRouteConfig(
            ntrip_targets=("left", "right"),
            base_targets=("drone",),
            forward_ntrip_to_base=True,
            base_serial_available=True,
        )
    )

    first = plane.process_ntrip_chunk(
        RTCM_1077[:4],
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    )
    second = plane.process_ntrip_chunk(
        RTCM_1077[4:],
        receive_utc_ns=11,
        receive_monotonic_ns=21,
    )

    assert [record.sequence for record in first.evidence_records] == [1]
    assert [record.sequence for record in second.evidence_records] == [2, 3]
    assert b"".join(
        result.evidence_records[0].payload for result in (first, second)
    ) == RTCM_1077
    assert second.evidence_records[1].payload == RTCM_1077
    assert second.evidence_records[1].message_type == 1077
    assert first.injection_requests == ()
    assert len(second.injection_requests) == 1
    assert second.injection_requests[0].targets == ("left", "right")
    assert second.injection_requests[0].payload == RTCM_1077
    assert first.base_writes == (("NTRIP RTCM -> fixed base", RTCM_1077[:4]),)
    assert second.base_writes == (("NTRIP RTCM -> fixed base", RTCM_1077[4:]),)


def test_base_and_ntrip_sources_keep_independent_source_sequences() -> None:
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    plane.configure(RtcmRouteConfig(ntrip_targets=("left",), base_targets=("drone",)))

    ntrip = plane.process_ntrip_chunk(
        RTCM_1077,
        receive_utc_ns=100,
        receive_monotonic_ns=200,
    )
    base = plane.process_base_frame(
        RTCM_1077,
        receive_utc_ns=101,
        receive_monotonic_ns=201,
    )

    assert [(row.record_kind, row.source_sequence) for row in ntrip.evidence_records] == [
        ("transport_chunk", 1),
        ("rtcm_frame", 1),
    ]
    assert [(row.record_kind, row.source_sequence) for row in base.evidence_records] == [
        ("rtcm_frame", 1)
    ]
    assert base.injection_requests[0].targets == ("drone",)
    assert [row.sequence for row in (*ntrip.evidence_records, *base.evidence_records)] == [
        1,
        2,
        3,
    ]


def test_corrupt_ntrip_frame_is_evidenced_but_never_injected() -> None:
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    plane.configure(RtcmRouteConfig(ntrip_targets=("left", "right")))
    corrupt = RTCM_1077[:-1] + bytes((RTCM_1077[-1] ^ 0xFF,))

    result = plane.process_ntrip_chunk(
        corrupt,
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    )

    assert len(result.evidence_records) == 1
    assert result.evidence_records[0].record_kind == "transport_chunk"
    assert result.evidence_records[0].payload == corrupt
    assert result.injection_requests == ()
    assert plane.snapshot(now_monotonic_s=1.0)["frames"] == 0


def test_required_base_forwarding_failure_is_explicit_not_silent() -> None:
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    plane.configure(
        RtcmRouteConfig(
            forward_ntrip_to_base=True,
            base_serial_available=False,
        )
    )

    result = plane.process_ntrip_chunk(
        b"sentinel",
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    )

    assert result.base_writes == ()
    assert result.base_forward_unavailable is True
    assert plane.snapshot(now_monotonic_s=1.0)["transport"][
        "base_forward_unavailable"
    ] == 1


def test_evidence_append_preserves_exact_bytes_offsets_hashes_and_sequences(
    tmp_path: Path,
) -> None:
    context = RtcmEvidenceContext(
        state="COLLECTING",
        session_id="session-1",
        session_root=tmp_path / "session-1",
    )
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    records = []
    records.extend(
        plane.process_ntrip_chunk(
            RTCM_1077[:4],
            receive_utc_ns=100,
            receive_monotonic_ns=200,
        ).evidence_records
    )
    records.extend(
        plane.process_ntrip_chunk(
            RTCM_1077[4:],
            receive_utc_ns=101,
            receive_monotonic_ns=201,
        ).evidence_records
    )

    results = [append_rtcm_evidence(context, record) for record in records]

    root = context.session_root / "raw" / "gnss"
    transport_path = root / "ntrip_transport_chunk.rtcm"
    transport_rows = [
        json.loads(line)
        for line in transport_path.with_suffix(".rtcm.jsonl").read_text().splitlines()
    ]
    frame_path = root / "ntrip_rtcm_frame.rtcm"
    assert transport_path.read_bytes() == RTCM_1077
    assert frame_path.read_bytes() == RTCM_1077
    assert [(row["sequence"], row["source_sequence"]) for row in transport_rows] == [
        (1, 1),
        (2, 2),
    ]
    assert [(row["offset_bytes"], row["size_bytes"]) for row in transport_rows] == [
        (0, 4),
        (4, len(RTCM_1077) - 4),
    ]
    assert transport_rows[0]["sha256"] == hashlib.sha256(RTCM_1077[:4]).hexdigest()
    assert results[-1]["message_type"] == 1077


def test_disabled_evidence_context_never_creates_session_files(tmp_path: Path) -> None:
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    record = plane.process_base_frame(
        RTCM_1077,
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    ).evidence_records[0]
    context = RtcmEvidenceContext(
        state="IDLE",
        session_id="session-1",
        session_root=tmp_path / "session-1",
    )

    assert append_rtcm_evidence(context, record) is None
    assert not (tmp_path / "session-1").exists()


def test_integrity_snapshot_tracks_persistence_injection_and_explicit_drops() -> None:
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    plane.configure(RtcmRouteConfig(base_targets=("left", "right")))
    result = plane.process_base_frame(
        RTCM_1077,
        receive_utc_ns=10,
        receive_monotonic_ns=2_000_000_000,
    )
    sequence = result.evidence_records[0].sequence
    plane.record_injection_expected(result.injection_requests[0])
    plane.record_evidence_expected(result.evidence_records[0])
    plane.record_evidence_written({"sequence": sequence})
    plane.record_injection_result(
        {
            "pipeline_sequence": sequence,
            "payload_bytes": len(RTCM_1077),
            "sent": (("left", 2),),
            "failed": (("right", "timeout"),),
            "dropped": 3,
        }
    )

    snapshot = plane.snapshot(now_monotonic_s=2.5)
    assert snapshot["integrity"] == {
        "last_received_sequence": sequence,
        "last_persisted_sequence": sequence,
        "last_injected_sequence": sequence,
        "records_received": 1,
        "evidence_expected": 1,
        "evidence_persisted": 1,
        "evidence_failed": 0,
        "injection_dropped_stale": 3,
        "last_injection_drop": None,
        "injection_expected": 1,
        "injection_outcomes": 1,
        "injection_pending": 0,
        "pending_injection_sequences": (),
        "injection_unconfirmed": 0,
        "last_evidence_error": "",
    }
    assert snapshot["injection"]["left"] == {
        "frames": 1,
        "bytes": len(RTCM_1077),
        "packets": 2,
        "fail": 0,
    }
    assert snapshot["injection"]["right"]["fail"] == 1


def test_runtime_reset_clears_transport_stats_without_reusing_sequence() -> None:
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    first = plane.process_base_frame(
        RTCM_1077,
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    )
    plane.reset_runtime(
        RtcmRouteConfig(base_targets=("left",)),
        started_monotonic_s=2.0,
    )
    second = plane.process_base_frame(
        RTCM_1077,
        receive_utc_ns=11,
        receive_monotonic_ns=21,
    )

    assert first.evidence_records[0].sequence == 1
    assert second.evidence_records[0].sequence == 2
    snapshot = plane.snapshot(now_monotonic_s=2.0)
    assert snapshot["frames"] == 1
    assert snapshot["integrity"]["records_received"] == 2


def test_recovered_evidence_high_water_prevents_sequence_reuse(tmp_path: Path) -> None:
    context = RtcmEvidenceContext(
        state="COLLECTING",
        session_id="session-1",
        session_root=tmp_path / "session-1",
    )
    first_plane = RtcmDataPlane(started_monotonic_s=1.0)
    for record in first_plane.process_ntrip_chunk(
        RTCM_1077,
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    ).evidence_records:
        assert append_rtcm_evidence(context, record) is not None

    last_sequence, source_sequences = load_rtcm_evidence_high_water(context)
    recovered = RtcmDataPlane(started_monotonic_s=2.0)
    recovered.restore_evidence_high_water(
        last_sequence=last_sequence,
        source_sequences=source_sequences,
    )
    next_records = recovered.process_ntrip_chunk(
        RTCM_1077,
        receive_utc_ns=11,
        receive_monotonic_ns=21,
    ).evidence_records

    assert [record.sequence for record in next_records] == [3, 4]
    assert [record.source_sequence for record in next_records] == [2, 2]


def test_recovery_fails_closed_on_unindexed_binary_tail(tmp_path: Path) -> None:
    context = RtcmEvidenceContext(
        state="COLLECTING",
        session_id="session-1",
        session_root=tmp_path / "session-1",
    )
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    record = plane.process_base_frame(
        RTCM_1077,
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    ).evidence_records[0]
    result = append_rtcm_evidence(context, record)
    assert result is not None
    with Path(result["binary_path"]).open("ab") as handle:
        handle.write(b"orphan")

    try:
        load_rtcm_evidence_high_water(context)
    except ValueError as exc:
        assert "unindexed trailing bytes" in str(exc)
    else:
        raise AssertionError("orphan RTCM bytes must fail recovery")


def test_recovery_fails_closed_on_binary_hash_mismatch(tmp_path: Path) -> None:
    context = RtcmEvidenceContext(
        state="COLLECTING",
        session_id="session-1",
        session_root=tmp_path / "session-1",
    )
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    record = plane.process_base_frame(
        RTCM_1077,
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    ).evidence_records[0]
    result = append_rtcm_evidence(context, record)
    assert result is not None
    binary_path = Path(result["binary_path"])
    corrupted = bytearray(binary_path.read_bytes())
    corrupted[0] ^= 0xFF
    binary_path.write_bytes(corrupted)

    try:
        load_rtcm_evidence_high_water(context)
    except ValueError as exc:
        assert "hash mismatch" in str(exc)
    else:
        raise AssertionError("corrupt RTCM evidence must fail recovery")


def test_append_rolls_back_binary_and_index_when_index_fsync_fails(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    """A failed durable index commit must not strand binary tail bytes."""

    from Workstation.workstation_core import rtcm_pipeline

    context = RtcmEvidenceContext(
        state="COLLECTING",
        session_id="session-rollback",
        session_root=tmp_path / "session-rollback",
        context_token="measurement-a:r0001",
    )
    plane = RtcmDataPlane(started_monotonic_s=1.0)
    first = plane.process_base_frame(
        RTCM_1077,
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    ).evidence_records[0]
    second = plane.process_base_frame(
        RTCM_1077,
        receive_utc_ns=11,
        receive_monotonic_ns=21,
    ).evidence_records[0]
    written = append_rtcm_evidence(context, first)
    assert written is not None
    binary_path = Path(written["binary_path"])
    index_path = Path(written["index_path"])
    binary_before = binary_path.read_bytes()
    index_before = index_path.read_bytes()

    real_fsync = os.fsync
    fsync_calls = 0

    def fail_second_fsync(descriptor: int) -> None:
        nonlocal fsync_calls
        fsync_calls += 1
        if fsync_calls == 2:
            raise OSError("injected index fsync failure")
        real_fsync(descriptor)

    monkeypatch.setattr(rtcm_pipeline.os, "fsync", fail_second_fsync)

    with pytest.raises(OSError, match="injected index fsync failure"):
        append_rtcm_evidence(context, second)

    assert binary_path.read_bytes() == binary_before
    assert index_path.read_bytes() == index_before
    assert fsync_calls >= 4  # binary/index commit plus index/binary rollback
    assert load_rtcm_evidence_high_water(context)[0] == first.sequence


def test_append_refuses_an_unpaired_existing_binary_without_mutation(
    tmp_path: Path,
) -> None:
    context = RtcmEvidenceContext(
        state="COLLECTING",
        session_id="session-unpaired",
        session_root=tmp_path / "session-unpaired",
        context_token="measurement-a:r0001",
    )
    binary_path = (
        context.session_root / "raw" / "gnss" / "base_rtcm_frame.rtcm"
    )
    binary_path.parent.mkdir(parents=True)
    binary_path.write_bytes(b"orphan-sentinel")
    record = RtcmDataPlane(started_monotonic_s=1.0).process_base_frame(
        RTCM_1077,
        receive_utc_ns=10,
        receive_monotonic_ns=20,
    ).evidence_records[0]

    with pytest.raises(ValueError, match="pair is incomplete"):
        append_rtcm_evidence(context, record)

    assert binary_path.read_bytes() == b"orphan-sentinel"
    assert not binary_path.with_suffix(".rtcm.jsonl").exists()
