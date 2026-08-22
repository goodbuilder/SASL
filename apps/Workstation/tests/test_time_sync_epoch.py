from __future__ import annotations

import copy
import hashlib
import json

from Workstation.workstation_core.recording_session import save_session_files
from Workstation.workstation_core.time_sync_epoch import (
    TIME_SYNC_EPOCH_SCHEMA_VERSION,
    materialize_time_sync_epochs,
)


def _camera_row(
    monotonic_ns: int,
    *,
    ssrc: int = 101,
    generation: int = 7,
    source_utc_ns: int = 1_700_000_000_000_000_123,
    invalid_reason: str = "",
) -> dict[str, object]:
    source_seconds = (
        f"{source_utc_ns // 1_000_000_000}."
        f"{source_utc_ns % 1_000_000_000:09d}"
    )
    return {
        "host_receive_monotonic_ns": monotonic_ns,
        "host_receive_utc_ns": 1_700_000_001_000_000_123,
        "source_utc_ns": source_utc_ns,
        "rtcp_frame_ntp_unix_s": source_seconds,
        "rtp_timestamp": 44_000,
        "rtp_ssrc": ssrc,
        "stream_generation": generation,
        "rtcp_sr_ntp_raw": 0xE8FE6F8000000000,
        "rtcp_sr_rtp_timestamp": 43_000,
        "rtcp_sr_received_count": 2,
        "rtcp_timing_invalid_reason": invalid_reason,
    }


def _metadata() -> dict[str, object]:
    return {
        "session_started_unix_ns": 9_007_199_254_740_993,
        "session_started_monotonic_ns": 9_007_199_254_741_111,
        "session_time_anchor_uncertainty_ns": 37,
        "pc_utc_sync": {
            "command": "STUS",
            "result": "success",
            "requested_utc_s": 1_699_999_999.25,
            "recorded_at_unix_s": 1_700_000_000.0,
        },
    }


def _materialize(
    left: list[dict[str, object]],
    right: list[dict[str, object]] | None = None,
    terminal: list[dict[str, object]] | None = None,
) -> list[dict[str, object]]:
    return materialize_time_sync_epochs(
        camera_rows={"left": left, "right": right or []},
        terminal_rows=terminal or [],
        metadata=_metadata(),
        session_id="session",
        measurement_package_id="m000001",
    )


def test_unknown_mapping_creates_no_epoch_and_keeps_null_reference() -> None:
    rows = [
        {
            "host_receive_monotonic_ns": 9_007_199_254_740_999,
            "rtp_ssrc": 101,
            "stream_generation": 7,
        }
    ]

    epochs = _materialize(rows)

    assert epochs == []
    assert rows[0]["time_sync_epoch_id"] is None


def test_epoch_identity_and_integer_anchor_are_exact_and_retry_stable() -> None:
    first_rows = [_camera_row(9_007_199_254_742_000)]
    second_rows = copy.deepcopy(first_rows)

    first = _materialize(first_rows)
    second = _materialize(second_rows)

    assert [row["time_sync_epoch_id"] for row in first] == [
        row["time_sync_epoch_id"] for row in second
    ]
    [epoch] = first
    identity_bytes = json.dumps(
        epoch["identity_payload"],
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    assert epoch["time_sync_epoch_sha256"] == hashlib.sha256(
        identity_bytes
    ).hexdigest()
    assert epoch["time_sync_epoch_id"] == f"tse:{epoch['time_sync_epoch_sha256']}"
    assert epoch["mapping_evidence"]["host_session_anchor"] == {
        "session_started_unix_ns": 9_007_199_254_740_993,
        "session_started_monotonic_ns": 9_007_199_254_741_111,
        "uncertainty_ns": 37,
    }
    assert isinstance(
        epoch["mapping_evidence"]["host_session_anchor"][
            "session_started_unix_ns"
        ],
        int,
    )
    assert epoch["uncertainty_ns"] is None
    assert epoch["utc_mapping_method"] == (
        "rtcp_sender_report_ntp_float_seconds_to_decimal_rounded_ns"
    )
    assert "not an original integer" in epoch["clock_domain_contract"][
        "source_utc_ns"
    ]


def test_ssrc_and_stream_generation_changes_create_observed_epochs() -> None:
    rows = [
        _camera_row(10_000, ssrc=101, generation=7),
        _camera_row(20_000, ssrc=202, generation=7),
        _camera_row(30_000, ssrc=202, generation=8),
    ]

    epochs = _materialize(rows)

    assert [epoch["trigger"] for epoch in epochs] == [
        "successful_stus_mapping_observed",
        "rtp_ssrc_change",
        "stream_generation_reset",
    ]
    assert len({row["time_sync_epoch_id"] for row in rows}) == 3


def test_explicit_invalidation_is_recorded_without_claiming_utc_mapping() -> None:
    rows = [
        _camera_row(10_000),
        _camera_row(20_000, invalid_reason="sr_pc_offset_exceeded"),
        _camera_row(30_000),
    ]

    epochs = _materialize(rows)

    assert [epoch["status"] for epoch in epochs] == ["valid", "invalid", "valid"]
    assert epochs[1]["trigger"] == "mapping_invalidation"
    assert epochs[1]["utc_mapping_method"] is None
    assert epochs[1]["mapping_evidence"]["rtcp_timing_invalid_reason"] == (
        "sr_pc_offset_exceeded"
    )
    assert epochs[2]["trigger"] == "mapping_revalidated"


def test_terminal_side_links_require_one_exact_epoch_and_stereo_id_remains_null() -> None:
    left = [_camera_row(10_000, ssrc=101, generation=7)]
    right = [_camera_row(10_100, ssrc=202, generation=9)]
    terminal = [
        {
            "left_app_receive_monotonic_ns": 10_000,
            "left_rtp_ssrc": 101,
            "left_stream_generation": 7,
            "right_app_receive_monotonic_ns": 10_100,
            "right_rtp_ssrc": 202,
            "right_stream_generation": 9,
        }
    ]

    epochs = _materialize(left, right, terminal)

    assert len(epochs) == 2
    assert terminal[0]["left_time_sync_epoch_id"] is not None
    assert terminal[0]["right_time_sync_epoch_id"] is not None
    assert terminal[0]["left_time_sync_epoch_id"] != terminal[0][
        "right_time_sync_epoch_id"
    ]
    assert terminal[0]["time_sync_epoch_id"] is None


def test_finalized_epoch_stream_preserves_request_and_exposure_time_separation(
    tmp_path,
) -> None:
    output = tmp_path / "recordings" / "time-epochs"
    camera_rows = {
        "left": [_camera_row(10_000, source_utc_ns=1_700_000_000_250_000_000)],
        "right": [],
    }

    saved = save_session_files(
        output,
        metadata={"session_id": output.name, **_metadata()},
        pair_rows=[],
        camera_rows=camera_rows,
    )
    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    epoch_path = output / manifest["resource_files"]["time_sync_epochs"]
    [epoch] = [json.loads(line) for line in epoch_path.read_text().splitlines()]
    context = json.loads((output / "data" / "context.json").read_text())

    assert epoch["schema_version"] == TIME_SYNC_EPOCH_SCHEMA_VERSION
    assert epoch["mapping_evidence"]["first_source_utc_ns"] == (
        1_700_000_000_250_000_000
    )
    assert context["metadata"]["pc_utc_sync"]["requested_utc_s"] == (
        1_699_999_999.25
    )
    assert context["metadata"]["pc_utc_sync"]["requested_utc_s"] != (
        float(epoch["mapping_evidence"]["source_utc_original_seconds"])
    )
    descriptor = manifest["artifact_details"]["time_sync_epochs"]
    assert descriptor["authoritative"] is True
    assert descriptor["row_count"] == 1
