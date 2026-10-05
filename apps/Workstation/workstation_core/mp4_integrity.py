"""Fail-closed integrity validation for finalized H.264-copy MP4 recordings.

Container finalization and a zero decoder exit status are not sufficient
evidence that a raw-copy recording is usable.  This module combines three
independent observations of the finalized bytes:

* ffprobe sample count versus software-decoded frame count;
* per-sample MP4 timing; and
* FFmpeg ``trace_headers`` H.264 IDR/frame_num syntax.

The validator never attempts to repair or synthesize sender frames.  Missing
tools, timeouts, malformed output, or incomplete observations all fail closed.
"""

from __future__ import annotations

import json
import math
import os
import re
import stat
import struct
import subprocess
import tempfile
import threading
from pathlib import Path
from typing import Any, Callable, Mapping


H264_MP4_INTEGRITY_SCHEMA = "workstation-h264-mp4-integrity-v1"
DEFAULT_INTEGRITY_DECODE_TIMEOUT_SECONDS = 300.0
DEFAULT_INTEGRITY_PROBE_TIMEOUT_SECONDS = 30.0
DEFAULT_INTEGRITY_TRACE_TIMEOUT_SECONDS = 120.0
MAX_INTEGRITY_PROBE_OUTPUT_BYTES = 64 * 1024 * 1024
MAX_INTEGRITY_TRACE_OUTPUT_BYTES = 32 * 1024 * 1024
MAX_PLAUSIBLE_NOMINAL_FPS = 240.0
MAX_NOMINAL_TO_AVERAGE_FPS_RATIO = 4.0


Runner = Callable[..., Any]

_TRACE_FIELD = re.compile(
    r"\]\s+\d+\s+([A-Za-z0-9_]+)\s+.*?=\s*(-?\d+)\s*$"
)


def _positive_timeout(value: object, name: str) -> float:
    try:
        timeout = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite positive number") from exc
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return timeout


def _integer(value: object) -> int | None:
    if value is None or str(value).strip().lower() in {"", "n/a", "nan", "none"}:
        return None
    try:
        return int(str(value).strip())
    except (TypeError, ValueError, OverflowError):
        return None


def _positive_ratio(value: object) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    try:
        if "/" in text:
            numerator, denominator = text.split("/", 1)
            denominator_value = float(denominator)
            if denominator_value == 0.0:
                return None
            result = float(numerator) / denominator_value
        else:
            result = float(text)
    except (TypeError, ValueError, OverflowError, ZeroDivisionError):
        return None
    return result if math.isfinite(result) and result > 0.0 else None


def _result_output_bytes(result: object, attribute: str) -> tuple[bytes, str]:
    raw = getattr(result, attribute, b"")
    if isinstance(raw, bytes):
        return raw, raw.decode("utf-8", errors="replace")
    text = str(raw or "")
    return text.encode("utf-8", errors="replace"), text


def _iter_mp4_boxes(handle: Any, start: int, end: int):
    position = int(start)
    boundary = int(end)
    while position < boundary:
        if boundary - position < 8:
            raise ValueError("truncated MP4 box header")
        handle.seek(position)
        header = handle.read(8)
        if len(header) != 8:
            raise ValueError("truncated MP4 box header")
        size, box_type = struct.unpack(">I4s", header)
        header_size = 8
        if size == 1:
            extended = handle.read(8)
            if len(extended) != 8:
                raise ValueError("truncated MP4 extended box size")
            size = struct.unpack(">Q", extended)[0]
            header_size += 8
        elif size == 0:
            size = boundary - position
        if box_type == b"uuid":
            user_type = handle.read(16)
            if len(user_type) != 16:
                raise ValueError("truncated MP4 UUID box")
            header_size += 16
        if size < header_size or position + size > boundary:
            raise ValueError("invalid MP4 box size")
        payload_start = position + header_size
        box_end = position + size
        yield box_type, payload_start, box_end
        position = box_end


def _child_box(handle: Any, start: int, end: int, wanted: bytes) -> tuple[int, int] | None:
    for box_type, payload_start, box_end in _iter_mp4_boxes(handle, start, end):
        if box_type == wanted:
            return payload_start, box_end
    return None


def _mp4_handler_type(handle: Any, mdia_start: int, mdia_end: int) -> bytes | None:
    hdlr = _child_box(handle, mdia_start, mdia_end, b"hdlr")
    if hdlr is None:
        return None
    payload_start, box_end = hdlr
    if box_end - payload_start < 12:
        raise ValueError("truncated MP4 handler box")
    handle.seek(payload_start + 8)
    handler_type = handle.read(4)
    return handler_type if len(handler_type) == 4 else None


def inspect_mp4_video_stts(path: str | os.PathLike[str]) -> dict[str, Any]:
    """Inspect the video track's raw ``stts`` entries without decoder inference."""

    source = Path(path).expanduser()
    with source.open("rb") as handle:
        file_size = os.fstat(handle.fileno()).st_size
        moov = _child_box(handle, 0, file_size, b"moov")
        if moov is None:
            raise ValueError("MP4 moov box missing")
        moov_start, moov_end = moov
        video_tables: list[tuple[int, int]] = []
        for box_type, trak_start, trak_end in _iter_mp4_boxes(handle, moov_start, moov_end):
            if box_type != b"trak":
                continue
            mdia = _child_box(handle, trak_start, trak_end, b"mdia")
            if mdia is None:
                continue
            mdia_start, mdia_end = mdia
            if _mp4_handler_type(handle, mdia_start, mdia_end) != b"vide":
                continue
            minf = _child_box(handle, mdia_start, mdia_end, b"minf")
            if minf is None:
                raise ValueError("video track minf box missing")
            stbl = _child_box(handle, minf[0], minf[1], b"stbl")
            if stbl is None:
                raise ValueError("video track stbl box missing")
            stts = _child_box(handle, stbl[0], stbl[1], b"stts")
            if stts is None:
                raise ValueError("video track stts box missing")
            video_tables.append(stts)
        if len(video_tables) != 1:
            raise ValueError("exactly one video stts table is required")

        payload_start, box_end = video_tables[0]
        if box_end - payload_start < 8:
            raise ValueError("truncated MP4 stts header")
        handle.seek(payload_start)
        header = handle.read(8)
        if len(header) != 8:
            raise ValueError("truncated MP4 stts header")
        version = header[0]
        flags = header[1:4]
        entry_count = struct.unpack(">I", header[4:8])[0]
        if version != 0 or flags != b"\x00\x00\x00" or entry_count <= 0:
            raise ValueError("unsupported or empty MP4 stts table")
        if box_end - payload_start < 8 + entry_count * 8:
            raise ValueError("truncated MP4 stts entries")

        sample_count = 0
        zero_duration_count = 0
        final_sample_duration: int | None = None
        for _index in range(entry_count):
            entry = handle.read(8)
            if len(entry) != 8:
                raise ValueError("truncated MP4 stts entry")
            run_count, sample_duration = struct.unpack(">II", entry)
            if run_count <= 0:
                raise ValueError("MP4 stts entry has no samples")
            sample_count += run_count
            if sample_duration == 0:
                zero_duration_count += run_count
            final_sample_duration = sample_duration
        return {
            "mp4_stts_entry_count": entry_count,
            "mp4_stts_sample_count": sample_count,
            "mp4_stts_zero_duration_sample_count": zero_duration_count,
            "mp4_stts_final_sample_duration": final_sample_duration,
            "mp4_stts_timing_valid": bool(
                sample_count > 0
                and zero_duration_count == 0
                and final_sample_duration is not None
                and final_sample_duration > 0
            ),
        }


def _run_json_probe(
    command: list[str],
    *,
    runner: Runner,
    timeout_seconds: float,
    phase: str,
) -> tuple[object | None, dict[str, Any], str | None]:
    record: dict[str, Any] = {
        "tool": Path(command[0]).name or "ffprobe",
        "status": "failed",
        "timeout_seconds": timeout_seconds,
    }
    try:
        if runner is subprocess.run:
            with tempfile.TemporaryFile() as output_file:
                result = runner(
                    command,
                    check=False,
                    stdout=output_file,
                    stderr=subprocess.PIPE,
                    timeout=timeout_seconds,
                )
                output_size = output_file.seek(0, os.SEEK_END)
                if output_size > MAX_INTEGRITY_PROBE_OUTPUT_BYTES:
                    reason = f"integrity_{phase}_output_too_large"
                    record["reason"] = reason
                    return None, record, reason
                output_file.seek(0)
                output_bytes = output_file.read()
                output_text = output_bytes.decode("utf-8", errors="replace")
        else:
            result = runner(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout_seconds,
            )
            output_bytes, output_text = _result_output_bytes(result, "stdout")
    except subprocess.TimeoutExpired:
        reason = f"integrity_{phase}_timeout"
        record["reason"] = reason
        return None, record, reason
    except FileNotFoundError:
        reason = "integrity_ffprobe_unavailable"
        record["reason"] = reason
        return None, record, reason
    except OSError:
        reason = f"integrity_{phase}_execution_failed"
        record["reason"] = reason
        return None, record, reason

    return_code = int(getattr(result, "returncode", 1))
    record["return_code"] = return_code
    if return_code != 0:
        reason = f"integrity_{phase}_failed"
        record["reason"] = reason
        return None, record, reason
    if len(output_bytes) > MAX_INTEGRITY_PROBE_OUTPUT_BYTES:
        reason = f"integrity_{phase}_output_too_large"
        record["reason"] = reason
        return None, record, reason
    try:
        payload = json.loads(output_text)
    except (json.JSONDecodeError, TypeError, ValueError):
        reason = f"integrity_{phase}_invalid_json"
        record["reason"] = reason
        return None, record, reason
    record["status"] = "passed"
    return payload, record, None


def _parse_h264_trace_lines(lines: Any) -> dict[str, Any]:
    """Parse independent H.264 syntax evidence emitted by ``trace_headers``.

    Continuity is evaluated between reference pictures.  H.264 permits
    non-reference pictures to reuse the next reference ``frame_num``; treating
    every slice as an increment would therefore create false discontinuities.
    Multi-slice pictures are counted once through ``first_mb_in_slice == 0``.
    """

    pending_ref_idc: int | None = None
    current_nal_type: int | None = None
    current_ref_idc: int | None = None
    current_first_mb: int | None = None
    log2_max_frame_num_minus4: int | None = None
    gaps_allowed: bool | None = None
    previous_reference_frame_num: int | None = None
    access_unit_count = 0
    idr_access_unit_count = 0
    discontinuity_count = 0
    idr_frame_num_error_count = 0
    first_vcl_nal_type: int | None = None
    sps_ids: set[int] = set()
    pps_ids: set[int] = set()
    pps_sps_links: set[tuple[int, int]] = set()
    slice_pps_ids: set[int] = set()
    current_pps_id: int | None = None
    unsupported_vcl_nal_count = 0
    field_picture_count = 0
    mmco5_count = 0

    for line in lines:
        match = _TRACE_FIELD.search(line)
        if match is None:
            continue
        field, raw_value = match.groups()
        value = int(raw_value)
        if field == "nal_ref_idc":
            pending_ref_idc = value
            continue
        if field == "nal_unit_type":
            current_nal_type = value
            current_ref_idc = pending_ref_idc
            current_first_mb = None
            pending_ref_idc = None
            current_pps_id = None
            if value in {2, 3, 4, 19, 20, 21}:
                unsupported_vcl_nal_count += 1
            continue
        if field == "log2_max_frame_num_minus4":
            log2_max_frame_num_minus4 = value
            continue
        if field == "gaps_in_frame_num_allowed_flag":
            gaps_allowed = bool(value)
            continue
        if field == "seq_parameter_set_id":
            if current_nal_type == 7:
                sps_ids.add(value)
            elif current_nal_type == 8 and current_pps_id is not None:
                pps_sps_links.add((current_pps_id, value))
            continue
        if field == "pic_parameter_set_id":
            if current_nal_type == 8:
                current_pps_id = value
                pps_ids.add(value)
            elif current_nal_type in {1, 5}:
                slice_pps_ids.add(value)
            continue
        if field == "field_pic_flag" and value:
            field_picture_count += 1
            continue
        if field == "memory_management_control_operation" and value == 5:
            mmco5_count += 1
            continue
        if field == "first_mb_in_slice":
            current_first_mb = value
            continue
        if field != "frame_num" or current_nal_type not in {1, 5}:
            continue
        if current_first_mb != 0:
            continue

        access_unit_count += 1
        if first_vcl_nal_type is None:
            first_vcl_nal_type = current_nal_type
        frame_num = value
        if current_nal_type == 5:
            idr_access_unit_count += 1
            if frame_num != 0:
                idr_frame_num_error_count += 1
            previous_reference_frame_num = frame_num
            continue
        if not current_ref_idc:
            continue
        if previous_reference_frame_num is not None:
            if log2_max_frame_num_minus4 is None:
                discontinuity_count += 1
            else:
                max_frame_num = 1 << (log2_max_frame_num_minus4 + 4)
                expected = (previous_reference_frame_num + 1) % max_frame_num
                if frame_num != expected:
                    discontinuity_count += 1
        previous_reference_frame_num = frame_num

    return {
        "h264_access_unit_count": access_unit_count,
        "h264_idr_access_unit_count": idr_access_unit_count,
        "h264_usable_idr_present": idr_access_unit_count > 0,
        "h264_first_vcl_is_idr": first_vcl_nal_type == 5,
        "h264_frame_num_discontinuity_count": discontinuity_count,
        "h264_idr_frame_num_error_count": idr_frame_num_error_count,
        "h264_log2_max_frame_num_minus4": log2_max_frame_num_minus4,
        "h264_gaps_in_frame_num_allowed": gaps_allowed,
        "h264_sps_ids": sorted(sps_ids),
        "h264_pps_ids": sorted(pps_ids),
        "h264_pps_sps_links": [list(link) for link in sorted(pps_sps_links)],
        "h264_slice_pps_ids": sorted(slice_pps_ids),
        "h264_unsupported_vcl_nal_count": unsupported_vcl_nal_count,
        "h264_field_picture_count": field_picture_count,
        "h264_mmco5_count": mmco5_count,
    }


def parse_h264_trace_headers(output: str) -> dict[str, Any]:
    """Parse captured ``trace_headers`` text; production uses a streaming iterator."""

    return _parse_h264_trace_lines(str(output or "").splitlines())


def _trace_h264_headers(
    path: Path,
    *,
    runner: Runner | None,
    binary: str,
    timeout_seconds: float,
) -> tuple[dict[str, Any] | None, dict[str, Any], str | None]:
    record: dict[str, Any] = {
        "tool": Path(str(binary)).name or "ffmpeg",
        "filter": "trace_headers",
        "status": "failed",
        "timeout_seconds": timeout_seconds,
    }
    command = [
        str(binary),
        "-v",
        "info",
        "-nostdin",
        "-i",
        os.fspath(path),
        "-map",
        "0:v:0",
        "-c:v",
        "copy",
        "-bsf:v",
        "trace_headers",
        "-f",
        "null",
        "-",
    ]
    try:
        if runner is None:
            process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                errors="replace",
            )
            parsed_box: dict[str, Any] = {}

            def drain_trace() -> None:
                assert process.stderr is not None
                try:
                    parsed_box["value"] = _parse_h264_trace_lines(process.stderr)
                except Exception as exc:  # noqa: BLE001 - fail-closed at the call site.
                    parsed_box["error"] = exc

            drain_thread = threading.Thread(
                target=drain_trace,
                name="mp4-h264-trace-drain",
                daemon=True,
            )
            drain_thread.start()
            try:
                return_code = process.wait(timeout=timeout_seconds)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
                drain_thread.join(timeout=2.0)
                reason = "integrity_h264_trace_timeout"
                record["reason"] = reason
                return None, record, reason
            drain_thread.join(timeout=2.0)
            if drain_thread.is_alive():
                reason = "integrity_h264_trace_drain_failed"
                record["reason"] = reason
                return None, record, reason
            if "value" not in parsed_box:
                # The drain thread exited (join above proved it is no longer
                # alive) without ever recording a parsed value -- it must
                # have raised inside drain_trace(). Fail closed instead of
                # falling through with an empty `parsed` dict, which would
                # raise an uncaught KeyError below.
                reason = "integrity_h264_trace_drain_exception"
                record["reason"] = reason
                drain_error = parsed_box.get("error")
                if drain_error is not None:
                    record["drain_error"] = f"{type(drain_error).__name__}: {drain_error}"
                return None, record, reason
            parsed = parsed_box["value"]
        else:
            result = runner(
                command,
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                timeout=timeout_seconds,
            )
            return_code = int(getattr(result, "returncode", 1))
            output_bytes, output_text = _result_output_bytes(result, "stderr")
            if len(output_bytes) > MAX_INTEGRITY_TRACE_OUTPUT_BYTES:
                reason = "integrity_h264_trace_output_too_large"
                record["reason"] = reason
                return None, record, reason
    except subprocess.TimeoutExpired:
        reason = "integrity_h264_trace_timeout"
        record["reason"] = reason
        return None, record, reason
    except FileNotFoundError:
        reason = "integrity_ffmpeg_unavailable"
        record["reason"] = reason
        return None, record, reason
    except OSError:
        reason = "integrity_h264_trace_execution_failed"
        record["reason"] = reason
        return None, record, reason

    record["return_code"] = return_code
    if return_code != 0:
        reason = "integrity_h264_trace_failed"
        record["reason"] = reason
        return None, record, reason
    if runner is not None:
        parsed = parse_h264_trace_headers(output_text)
    if int(parsed["h264_access_unit_count"]) <= 0:
        reason = "integrity_h264_trace_no_access_units"
        record["reason"] = reason
        return None, record, reason
    record["status"] = "passed"
    return parsed, record, None


def _stream_metrics(payload: object) -> tuple[dict[str, Any], list[str]]:
    reasons: list[str] = []
    if not isinstance(payload, Mapping):
        return {}, ["integrity_decode_invalid_payload"]
    streams = payload.get("streams")
    if not isinstance(streams, list) or len(streams) != 1 or not isinstance(streams[0], Mapping):
        return {}, ["integrity_video_stream_missing"]
    stream = streams[0]
    codec = str(stream.get("codec_name") or "").strip().lower()
    sample_count = _integer(stream.get("nb_frames"))
    decoded_count = _integer(stream.get("nb_read_frames"))
    duration_ts = _integer(stream.get("duration_ts"))
    nominal_rate_text = str(stream.get("r_frame_rate") or "").strip()
    average_rate_text = str(stream.get("avg_frame_rate") or "").strip()
    time_base_text = str(stream.get("time_base") or "").strip()
    nominal_rate = _positive_ratio(nominal_rate_text)
    average_rate = _positive_ratio(average_rate_text)
    time_base = _positive_ratio(time_base_text)

    if codec != "h264":
        reasons.append("integrity_codec_not_h264")
    if sample_count is None or sample_count <= 0:
        reasons.append("integrity_sample_count_invalid")
    if decoded_count is None or decoded_count <= 0:
        reasons.append("integrity_decoded_frame_count_invalid")
    frame_count_match = bool(
        sample_count is not None
        and decoded_count is not None
        and sample_count > 0
        and decoded_count > 0
        and sample_count == decoded_count
    )
    if sample_count and decoded_count and not frame_count_match:
        reasons.append("integrity_decoder_visible_frame_loss")
    if duration_ts is None or duration_ts <= 0:
        reasons.append("integrity_stream_duration_invalid")
    if average_rate is None:
        reasons.append("integrity_average_frame_rate_invalid")
    if time_base is None:
        reasons.append("integrity_time_base_invalid")

    nominal_rate_plausible = bool(
        nominal_rate is not None
        and nominal_rate <= MAX_PLAUSIBLE_NOMINAL_FPS
        and (
            average_rate is None
            or nominal_rate <= average_rate * MAX_NOMINAL_TO_AVERAGE_FPS_RATIO
        )
    )
    if not nominal_rate_plausible:
        reasons.append("integrity_nominal_frame_rate_invalid")
    decoder_visible_loss = (
        max(0, sample_count - decoded_count)
        if sample_count is not None and decoded_count is not None
        else None
    )
    return {
        "codec": codec,
        "sample_count": sample_count,
        "decoded_frame_count": decoded_count,
        "decoder_visible_loss_count": decoder_visible_loss,
        "decoder_frame_count_match": frame_count_match,
        "stream_duration_ts": duration_ts,
        "r_frame_rate": nominal_rate_text,
        "avg_frame_rate": average_rate_text,
        "time_base": time_base_text,
        "average_frame_rate_valid": average_rate is not None,
        "time_base_valid": time_base is not None,
        "nominal_frame_rate_plausible": nominal_rate_plausible,
    }, reasons


def _timing_metrics(
    payload: object,
    *,
    expected_sample_count: int | None,
) -> tuple[dict[str, Any], list[str]]:
    reasons: list[str] = []
    if not isinstance(payload, Mapping) or not isinstance(payload.get("packets"), list):
        return {}, ["integrity_sample_timing_invalid_payload"]
    packets = payload["packets"]
    zero_or_negative_duration = 0
    missing_duration = 0
    missing_timestamp = 0
    non_monotonic_dts = 0
    non_contiguous_dts = 0
    previous_dts: int | None = None
    previous_duration: int | None = None
    for packet in packets:
        if not isinstance(packet, Mapping):
            missing_duration += 1
            missing_timestamp += 1
            continue
        duration = _integer(packet.get("duration"))
        if duration is None:
            missing_duration += 1
        elif duration <= 0:
            zero_or_negative_duration += 1
        pts = _integer(packet.get("pts"))
        dts = _integer(packet.get("dts"))
        if pts is None or dts is None:
            missing_timestamp += 1
        if dts is not None:
            if previous_dts is not None and dts <= previous_dts:
                non_monotonic_dts += 1
            if (
                previous_dts is not None
                and previous_duration is not None
                and dts != previous_dts + previous_duration
            ):
                non_contiguous_dts += 1
            previous_dts = dts
            previous_duration = duration
    packet_count = len(packets)
    sample_count_match = bool(
        expected_sample_count is not None
        and expected_sample_count > 0
        and packet_count == expected_sample_count
    )
    if not sample_count_match:
        reasons.append("integrity_packet_sample_count_mismatch")
    if missing_duration:
        reasons.append("integrity_sample_duration_missing")
    if zero_or_negative_duration:
        reasons.append("integrity_sample_duration_nonpositive")
    if missing_timestamp:
        reasons.append("integrity_sample_timestamp_missing")
    if non_monotonic_dts:
        reasons.append("integrity_sample_dts_non_monotonic")
    if non_contiguous_dts:
        reasons.append("integrity_sample_dts_non_contiguous")
    timing_valid = not reasons
    return {
        "packet_sample_count": packet_count,
        "packet_sample_count_match": sample_count_match,
        "sample_timing_valid": timing_valid,
        "missing_sample_duration_count": missing_duration,
        "nonpositive_sample_duration_count": zero_or_negative_duration,
        "missing_sample_timestamp_count": missing_timestamp,
        "non_monotonic_sample_dts_count": non_monotonic_dts,
        "non_contiguous_sample_dts_count": non_contiguous_dts,
    }, reasons


def validate_h264_copy_mp4_integrity(
    path: str | os.PathLike[str],
    *,
    probe_runner: Runner | None = None,
    trace_runner: Runner | None = None,
    ffprobe_binary: str = "ffprobe",
    ffmpeg_binary: str = "ffmpeg",
    decode_timeout_seconds: float = DEFAULT_INTEGRITY_DECODE_TIMEOUT_SECONDS,
    probe_timeout_seconds: float = DEFAULT_INTEGRITY_PROBE_TIMEOUT_SECONDS,
    trace_timeout_seconds: float = DEFAULT_INTEGRITY_TRACE_TIMEOUT_SECONDS,
) -> dict[str, Any]:
    """Return factual finalized-file integrity evidence for one raw H.264 MP4."""

    source = Path(path).expanduser()
    decode_timeout = _positive_timeout(decode_timeout_seconds, "decode_timeout_seconds")
    probe_timeout = _positive_timeout(probe_timeout_seconds, "probe_timeout_seconds")
    trace_timeout = _positive_timeout(trace_timeout_seconds, "trace_timeout_seconds")
    run_probe = probe_runner or subprocess.run
    reasons: list[str] = []
    evidence: dict[str, Any] = {
        "integrity_schema": H264_MP4_INTEGRITY_SCHEMA,
        "integrity_validation_status": "FAIL",
        "recording_usable": False,
        "recording_integrity_verified": False,
    }

    try:
        details = source.lstat()
    except OSError:
        reasons.append("integrity_video_file_unavailable")
        evidence["integrity_failure_reasons"] = reasons
        return evidence
    if not stat.S_ISREG(details.st_mode) or stat.S_ISLNK(details.st_mode) or details.st_size <= 0:
        reasons.append("integrity_video_file_invalid")
        evidence["integrity_failure_reasons"] = reasons
        return evidence

    try:
        stts_metrics = inspect_mp4_video_stts(source)
    except (OSError, ValueError, struct.error):
        reasons.append("integrity_mp4_stts_invalid")
    else:
        evidence.update(stts_metrics)
        if not bool(stts_metrics.get("mp4_stts_timing_valid")):
            reasons.append("integrity_mp4_stts_zero_sample_duration")

    decode_command = [
        str(ffprobe_binary),
        "-v",
        "error",
        "-threads",
        "4",
        "-count_frames",
        "-select_streams",
        "v:0",
        "-show_entries",
        (
            "stream=index,codec_type,codec_name,r_frame_rate,avg_frame_rate,"
            "time_base,duration_ts,duration,nb_frames,nb_read_frames"
        ),
        "-of",
        "json",
        os.fspath(source),
    ]
    decode_payload, decode_record, decode_reason = _run_json_probe(
        decode_command,
        runner=run_probe,
        timeout_seconds=decode_timeout,
        phase="decode_count",
    )
    evidence["integrity_decode_count_probe"] = decode_record
    stream_metrics: dict[str, Any] = {}
    if decode_reason is not None:
        reasons.append(decode_reason)
    else:
        stream_metrics, stream_reasons = _stream_metrics(decode_payload)
        evidence.update(stream_metrics)
        reasons.extend(stream_reasons)
        stts_sample_count = _integer(evidence.get("mp4_stts_sample_count"))
        decoded_sample_count = _integer(stream_metrics.get("sample_count"))
        if (
            stts_sample_count is not None
            and decoded_sample_count is not None
            and stts_sample_count != decoded_sample_count
        ):
            reasons.append("integrity_mp4_stts_sample_count_mismatch")

    timing_command = [
        str(ffprobe_binary),
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_packets",
        "-show_entries",
        "packet=pts,dts,duration",
        "-of",
        "json",
        os.fspath(source),
    ]
    timing_payload, timing_record, timing_reason = _run_json_probe(
        timing_command,
        runner=run_probe,
        timeout_seconds=probe_timeout,
        phase="sample_timing",
    )
    evidence["integrity_sample_timing_probe"] = timing_record
    if timing_reason is not None:
        reasons.append(timing_reason)
    else:
        expected_sample_count = stream_metrics.get("sample_count")
        if _integer(expected_sample_count) is None:
            expected_sample_count = evidence.get("mp4_stts_sample_count")
        timing_metrics, timing_reasons = _timing_metrics(
            timing_payload,
            expected_sample_count=_integer(expected_sample_count),
        )
        evidence.update(timing_metrics)
        reasons.extend(timing_reasons)

    trace_metrics, trace_record, trace_reason = _trace_h264_headers(
        source,
        runner=trace_runner,
        binary=ffmpeg_binary,
        timeout_seconds=trace_timeout,
    )
    evidence["integrity_h264_trace"] = trace_record
    if trace_reason is not None:
        reasons.append(trace_reason)
    else:
        evidence.update(trace_metrics or {})
        traced_access_units = _integer(
            (trace_metrics or {}).get("h264_access_unit_count")
        )
        expected_access_units = _integer(stream_metrics.get("sample_count"))
        if expected_access_units is None:
            expected_access_units = _integer(evidence.get("mp4_stts_sample_count"))
        access_unit_count_match = bool(
            traced_access_units is not None
            and expected_access_units is not None
            and traced_access_units == expected_access_units
        )
        evidence["h264_access_unit_sample_count_match"] = access_unit_count_match
        if not access_unit_count_match:
            reasons.append("integrity_h264_access_unit_sample_count_mismatch")
        if (trace_metrics or {}).get("h264_log2_max_frame_num_minus4") is None:
            reasons.append("integrity_h264_sps_frame_num_contract_missing")
        if (trace_metrics or {}).get("h264_gaps_in_frame_num_allowed") is None:
            reasons.append("integrity_h264_sps_gap_contract_missing")
        if bool((trace_metrics or {}).get("h264_gaps_in_frame_num_allowed")):
            reasons.append("integrity_h264_frame_num_gaps_allowed_unsupported")
        if len(tuple((trace_metrics or {}).get("h264_sps_ids", ()))) != 1:
            reasons.append("integrity_h264_multiple_or_missing_sps_unsupported")
        if len(tuple((trace_metrics or {}).get("h264_pps_ids", ()))) != 1:
            reasons.append("integrity_h264_multiple_or_missing_pps_unsupported")
        if len(tuple((trace_metrics or {}).get("h264_pps_sps_links", ()))) != 1:
            reasons.append("integrity_h264_pps_sps_mapping_unsupported")
        if len(tuple((trace_metrics or {}).get("h264_slice_pps_ids", ()))) != 1:
            reasons.append("integrity_h264_slice_pps_mapping_unsupported")
        if int((trace_metrics or {}).get("h264_unsupported_vcl_nal_count") or 0):
            reasons.append("integrity_h264_partition_or_extension_vcl_unsupported")
        if int((trace_metrics or {}).get("h264_field_picture_count") or 0):
            reasons.append("integrity_h264_field_picture_unsupported")
        if int((trace_metrics or {}).get("h264_mmco5_count") or 0):
            reasons.append("integrity_h264_mmco5_unsupported")
        if not bool((trace_metrics or {}).get("h264_usable_idr_present")):
            reasons.append("integrity_h264_usable_idr_missing")
        if not bool((trace_metrics or {}).get("h264_first_vcl_is_idr")):
            reasons.append("integrity_h264_first_vcl_not_idr")
        if int((trace_metrics or {}).get("h264_frame_num_discontinuity_count") or 0) > 0:
            reasons.append("integrity_h264_frame_num_discontinuity")
        if int((trace_metrics or {}).get("h264_idr_frame_num_error_count") or 0) > 0:
            reasons.append("integrity_h264_idr_frame_num_invalid")

    unique_reasons = list(dict.fromkeys(reasons))
    evidence["integrity_failure_reasons"] = unique_reasons
    if not unique_reasons:
        evidence.update(
            integrity_validation_status="PASS",
            recording_usable=True,
            recording_integrity_verified=True,
        )
    return evidence


__all__ = [
    "DEFAULT_INTEGRITY_DECODE_TIMEOUT_SECONDS",
    "DEFAULT_INTEGRITY_PROBE_TIMEOUT_SECONDS",
    "DEFAULT_INTEGRITY_TRACE_TIMEOUT_SECONDS",
    "H264_MP4_INTEGRITY_SCHEMA",
    "MAX_INTEGRITY_PROBE_OUTPUT_BYTES",
    "MAX_INTEGRITY_TRACE_OUTPUT_BYTES",
    "inspect_mp4_video_stts",
    "parse_h264_trace_headers",
    "validate_h264_copy_mp4_integrity",
]
