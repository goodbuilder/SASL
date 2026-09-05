"""Observed camera time-mapping epochs without fabricated clock truth."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import hashlib
import json
from typing import Any

from Workstation.workstation_core.runtime_camera_status import is_sr_pc_diagnostic_reason


TIME_SYNC_EPOCH_SCHEMA_VERSION = "workstation-time-sync-epoch-v2"


def _canonical_bytes(value: Mapping[str, Any]) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _optional_int(value: object) -> int | None:
    if value in (None, "") or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _exact_json_int(value: object) -> int | None:
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def _stus_reference(metadata: Mapping[str, object]) -> dict[str, object] | None:
    evidence = metadata.get("pc_utc_sync")
    if not isinstance(evidence, Mapping):
        return None
    if str(evidence.get("command") or "").upper() != "STUS":
        return None
    if str(evidence.get("result") or "").lower() != "success":
        return None
    normalized = json.loads(_canonical_bytes(evidence).decode("utf-8"))
    return {
        "relative_reference": "context.json#/metadata/pc_utc_sync",
        "sha256": hashlib.sha256(_canonical_bytes(normalized)).hexdigest(),
        "result": "success",
    }


def _row_monotonic_ns(row: Mapping[str, object]) -> int | None:
    return _optional_int(
        row.get("host_receive_monotonic_ns")
        if row.get("host_receive_monotonic_ns") not in (None, "")
        else row.get("sample_monotonic_ns")
    )


def _mapping_state(
    row: Mapping[str, object],
) -> tuple[str, int | None, int | None, str, str] | None:
    generation = _optional_int(row.get("stream_generation"))
    ssrc = _optional_int(row.get("rtp_ssrc"))
    clock_epoch_id = str(row.get("clock_epoch_id") or "")
    clock_epoch_state = str(row.get("clock_epoch_state") or "")
    invalid_reason = str(row.get("rtcp_timing_invalid_reason") or "").strip()
    blocking_invalid_reason = invalid_reason and not is_sr_pc_diagnostic_reason(invalid_reason)
    if blocking_invalid_reason or (clock_epoch_state and clock_epoch_state != "current"):
        return "invalid", generation, ssrc, clock_epoch_id, clock_epoch_state
    source_utc_ns = _optional_int(row.get("source_utc_ns"))
    sender_report_count = _optional_int(row.get("rtcp_sr_received_count"))
    if (
        source_utc_ns is None
        or ssrc is None
        or sender_report_count is None
        or sender_report_count <= 0
    ):
        return None
    return "valid", generation, ssrc, clock_epoch_id, clock_epoch_state


def _transition_trigger(
    previous: tuple[str, int | None, int | None, str, str] | None,
    current: tuple[str, int | None, int | None, str, str],
    *,
    has_stus_reference: bool,
) -> str:
    status, generation, ssrc, clock_epoch_id, clock_epoch_state = current
    if previous is None:
        if status == "invalid":
            return (
                "stus_clock_epoch_transition"
                if clock_epoch_state == "transition"
                else "mapping_invalidation"
            )
        return (
            "successful_stus_mapping_observed"
            if has_stus_reference
            else "first_observed_rtcp_mapping"
        )
    (
        previous_status,
        previous_generation,
        previous_ssrc,
        previous_clock_epoch_id,
        previous_clock_epoch_state,
    ) = previous
    if status == "invalid":
        return (
            "stus_clock_epoch_transition"
            if clock_epoch_state == "transition"
            else "mapping_invalidation"
        )
    if previous_status == "invalid":
        return (
            "stus_clock_epoch_verified"
            if clock_epoch_id and clock_epoch_id == previous_clock_epoch_id
            else "mapping_revalidated"
        )
    if clock_epoch_id and clock_epoch_id != previous_clock_epoch_id:
        return "stus_clock_epoch_change"
    generation_changed = generation != previous_generation
    ssrc_changed = ssrc != previous_ssrc
    if generation_changed and ssrc_changed:
        return "stream_generation_and_ssrc_change"
    if generation_changed:
        return "stream_generation_reset"
    if ssrc_changed:
        return "rtp_ssrc_change"
    return "mapping_state_change"


def _epoch_record(
    *,
    accumulator: Mapping[str, object],
    end_monotonic_ns: int | None,
    session_id: str,
    measurement_package_id: str,
    session_anchor: Mapping[str, int | None],
    stus_reference: Mapping[str, object] | None,
) -> dict[str, object]:
    first_row = accumulator["first_row"]
    if not isinstance(first_row, Mapping):
        raise ValueError("time-sync epoch first row is invalid")
    status = str(accumulator["status"])
    identity_payload = {
        "schema_version": TIME_SYNC_EPOCH_SCHEMA_VERSION,
        "source": "wiris_rtcp_sender_report",
        "side": str(accumulator["side"]),
        "start_monotonic_ns": int(accumulator["start_monotonic_ns"]),
        "status": status,
        "trigger": str(accumulator["trigger"]),
        "rtp_ssrc": accumulator.get("rtp_ssrc"),
        "stream_generation": accumulator.get("stream_generation"),
        "clock_epoch_id": str(accumulator.get("clock_epoch_id") or ""),
        "clock_epoch_state": str(accumulator.get("clock_epoch_state") or ""),
        "stus_evidence_sha256": (
            None if stus_reference is None else stus_reference.get("sha256")
        ),
    }
    digest = hashlib.sha256(_canonical_bytes(identity_payload)).hexdigest()
    source_utc_ns = _optional_int(first_row.get("source_utc_ns"))
    original_source_seconds = first_row.get("rtcp_frame_ntp_unix_s")
    return {
        "schema_version": TIME_SYNC_EPOCH_SCHEMA_VERSION,
        "time_sync_epoch_id": f"tse:{digest}",
        "time_sync_epoch_sha256": digest,
        "identity_payload": identity_payload,
        "session_id": session_id,
        "measurement_package_id": measurement_package_id,
        "source": "wiris_rtcp_sender_report",
        "side": str(accumulator["side"]),
        "start_monotonic_ns": int(accumulator["start_monotonic_ns"]),
        "end_monotonic_ns": end_monotonic_ns,
        "end_contract": str(accumulator["end_contract"]),
        "utc_mapping_method": (
            "rtcp_sender_report_ntp_float_seconds_to_decimal_rounded_ns"
            if status == "valid"
            else None
        ),
        "uncertainty_ns": None,
        "status": status,
        "trigger": str(accumulator["trigger"]),
        "rtp_ssrc": accumulator.get("rtp_ssrc"),
        "stream_generation": accumulator.get("stream_generation"),
        "clock_epoch_id": str(accumulator.get("clock_epoch_id") or ""),
        "clock_epoch_state": str(accumulator.get("clock_epoch_state") or ""),
        "stus_evidence_reference": (
            None if stus_reference is None else dict(stus_reference)
        ),
        "mapping_evidence": {
            "first_host_receive_monotonic_ns": _optional_int(
                first_row.get("host_receive_monotonic_ns")
            ),
            "first_host_receive_utc_ns": _optional_int(
                first_row.get("host_receive_utc_ns")
            ),
            "first_source_utc_ns": source_utc_ns,
            "source_utc_original_seconds": original_source_seconds,
            "source_utc_ns_derivation": (
                "Decimal(str(seconds))*1e9 ROUND_HALF_UP"
                if source_utc_ns is not None
                else None
            ),
            "rtcp_sr_rtp_timestamp": _optional_int(
                first_row.get("rtcp_sr_rtp_timestamp")
            ),
            "rtcp_sr_ntp_raw": _optional_int(first_row.get("rtcp_sr_ntp_raw")),
            "rtcp_sr_received_count": _optional_int(
                first_row.get("rtcp_sr_received_count")
            ),
            "rtcp_timing_invalid_reason": str(
                first_row.get("rtcp_timing_invalid_reason") or ""
            ),
            "clock_epoch_id": str(first_row.get("clock_epoch_id") or ""),
            "clock_epoch_state": str(first_row.get("clock_epoch_state") or ""),
            "host_session_anchor": dict(session_anchor),
        },
        "clock_domain_contract": {
            "host_receive_monotonic_ns": "host CLOCK_MONOTONIC integer reading",
            "host_receive_utc_ns": "host CLOCK_REALTIME integer reading",
            "source_utc_ns": "derived from RTCP/NTP float seconds; not an original integer clock reading",
            "rtp_timestamp": "RTP media clock; never treated as Unix or monotonic time",
            "px4_and_gps_clocks": "not_part_of_camera_epoch",
        },
    }


def _side_epochs(
    side: str,
    rows: Sequence[dict[str, object]],
    *,
    session_id: str,
    measurement_package_id: str,
    session_anchor: Mapping[str, int | None],
    stus_reference: Mapping[str, object] | None,
) -> list[dict[str, object]]:
    epochs: list[dict[str, object]] = []
    active: dict[str, object] | None = None
    active_state: tuple[str, int | None, int | None, str, str] | None = None

    def close_active(*, end_ns: int | None, end_contract: str) -> None:
        nonlocal active
        if active is None:
            return
        active["end_contract"] = end_contract
        record = _epoch_record(
            accumulator=active,
            end_monotonic_ns=end_ns,
            session_id=session_id,
            measurement_package_id=measurement_package_id,
            session_anchor=session_anchor,
            stus_reference=stus_reference,
        )
        for index in active["row_indices"]:
            rows[int(index)]["time_sync_epoch_id"] = record["time_sync_epoch_id"]
        epochs.append(record)
        active = None

    for index, row in enumerate(rows):
        row["time_sync_epoch_id"] = None
        observed_ns = _row_monotonic_ns(row)
        state = _mapping_state(row)
        if observed_ns is None or state is None:
            continue
        if active_state != state:
            close_active(
                end_ns=observed_ns,
                end_contract="exclusive_transition_boundary",
            )
            active = {
                "side": side,
                "status": state[0],
                "stream_generation": state[1],
                "rtp_ssrc": state[2],
                "clock_epoch_id": state[3],
                "clock_epoch_state": state[4],
                "start_monotonic_ns": observed_ns,
                "last_monotonic_ns": observed_ns,
                "trigger": _transition_trigger(
                    active_state,
                    state,
                    has_stus_reference=stus_reference is not None,
                ),
                "first_row": dict(row),
                "row_indices": [index],
                "end_contract": "last_observed_row",
            }
            active_state = state
        else:
            if active is None:
                raise ValueError("time-sync epoch state lost its accumulator")
            if observed_ns < int(active["last_monotonic_ns"]):
                row["time_sync_epoch_id"] = None
                continue
            active["last_monotonic_ns"] = observed_ns
            active["row_indices"].append(index)
    close_active(
        end_ns=None if active is None else int(active["last_monotonic_ns"]),
        end_contract="last_observed_row_inclusive",
    )
    return epochs


def _link_terminal_rows(
    terminal_rows: Sequence[dict[str, object]],
    epochs: Sequence[Mapping[str, object]],
) -> None:
    valid_by_side = {
        side: [
            epoch
            for epoch in epochs
            if epoch.get("side") == side and epoch.get("status") == "valid"
        ]
        for side in ("left", "right")
    }
    for row in terminal_rows:
        side_ids: dict[str, str | None] = {}
        for side in ("left", "right"):
            observed_ns = _optional_int(row.get(f"{side}_app_receive_monotonic_ns"))
            ssrc = _optional_int(row.get(f"{side}_rtp_ssrc"))
            generation = _optional_int(row.get(f"{side}_stream_generation"))
            matches: list[Mapping[str, object]] = []
            if observed_ns is not None and ssrc is not None:
                for epoch in valid_by_side[side]:
                    start_ns = int(epoch["start_monotonic_ns"])
                    end_ns = _optional_int(epoch.get("end_monotonic_ns"))
                    end_is_exclusive = (
                        epoch.get("end_contract") == "exclusive_transition_boundary"
                    )
                    if observed_ns < start_ns or (
                        end_ns is not None
                        and (
                            observed_ns >= end_ns
                            if end_is_exclusive
                            else observed_ns > end_ns
                        )
                    ):
                        continue
                    if _optional_int(epoch.get("rtp_ssrc")) != ssrc:
                        continue
                    epoch_generation = _optional_int(epoch.get("stream_generation"))
                    if generation is not None and epoch_generation != generation:
                        continue
                    matches.append(epoch)
            side_ids[side] = (
                str(matches[0]["time_sync_epoch_id"])
                if len(matches) == 1
                else None
            )
            row[f"{side}_time_sync_epoch_id"] = side_ids[side]
        row["time_sync_epoch_id"] = (
            side_ids["left"]
            if side_ids["left"] is not None
            and side_ids["left"] == side_ids["right"]
            else None
        )


def materialize_time_sync_epochs(
    *,
    camera_rows: Mapping[str, Sequence[dict[str, object]]],
    terminal_rows: Sequence[dict[str, object]],
    metadata: Mapping[str, object],
    session_id: str,
    measurement_package_id: str,
) -> list[dict[str, object]]:
    """Attach only observed epoch references and return deterministic records."""

    session_anchor = {
        "session_started_unix_ns": _exact_json_int(
            metadata.get("session_started_unix_ns")
        ),
        "session_started_monotonic_ns": _exact_json_int(
            metadata.get("session_started_monotonic_ns")
        ),
        "uncertainty_ns": _exact_json_int(
            metadata.get("session_time_anchor_uncertainty_ns")
            if metadata.get("session_time_anchor_uncertainty_ns") is not None
            else metadata.get("measurement_time_anchor_uncertainty_ns")
        ),
    }
    stus_reference = _stus_reference(metadata)
    epochs: list[dict[str, object]] = []
    for side in ("left", "right"):
        epochs.extend(
            _side_epochs(
                side,
                camera_rows.get(side, ()),
                session_id=session_id,
                measurement_package_id=measurement_package_id,
                session_anchor=session_anchor,
                stus_reference=stus_reference,
            )
        )
    _link_terminal_rows(terminal_rows, epochs)
    return epochs


__all__ = [
    "TIME_SYNC_EPOCH_SCHEMA_VERSION",
    "materialize_time_sync_epochs",
]
