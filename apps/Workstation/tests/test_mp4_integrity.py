from __future__ import annotations

import json
import struct
import subprocess
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from Workstation.workstation_core.media_evidence import verify_video_evidence
from Workstation.workstation_core.mp4_integrity import (
    H264_MP4_INTEGRITY_SCHEMA,
    inspect_mp4_video_stts,
    parse_h264_trace_headers,
    validate_h264_copy_mp4_integrity,
)


def _box(box_type: bytes, payload: bytes) -> bytes:
    return struct.pack(">I4s", len(payload) + 8, box_type) + payload


def _write_mp4_with_stts(path: Path, durations: tuple[int, ...]) -> None:
    stts_entries = b"".join(
        struct.pack(">II", 1, duration) for duration in durations
    )
    stts = _box(b"stts", b"\x00\x00\x00\x00" + struct.pack(">I", len(durations)) + stts_entries)
    stbl = _box(b"stbl", stts)
    minf = _box(b"minf", stbl)
    hdlr = _box(b"hdlr", b"\x00\x00\x00\x00" + b"\x00" * 4 + b"vide")
    mdia = _box(b"mdia", hdlr + minf)
    trak = _box(b"trak", mdia)
    path.write_bytes(_box(b"moov", trak))


def _trace_field(name: str, value: int) -> str:
    return f"[trace_headers @ 0x1] 1 {name:<60} 0 = {value}"


def _trace_picture(*, nal_type: int, ref_idc: int, frame_num: int) -> list[str]:
    return [
        _trace_field("nal_ref_idc", ref_idc),
        _trace_field("nal_unit_type", nal_type),
        _trace_field("first_mb_in_slice", 0),
        _trace_field("pic_parameter_set_id", 0),
        _trace_field("frame_num", frame_num),
    ]


def _trace_parameter_sets() -> list[str]:
    return [
        _trace_field("nal_ref_idc", 3),
        _trace_field("nal_unit_type", 7),
        _trace_field("seq_parameter_set_id", 0),
        _trace_field("log2_max_frame_num_minus4", 0),
        _trace_field("gaps_in_frame_num_allowed_flag", 0),
        _trace_field("nal_ref_idc", 3),
        _trace_field("nal_unit_type", 8),
        _trace_field("pic_parameter_set_id", 0),
        _trace_field("seq_parameter_set_id", 0),
    ]


def _healthy_trace() -> str:
    return "\n".join(
        [
            *_trace_parameter_sets(),
            *_trace_picture(nal_type=5, ref_idc=3, frame_num=0),
            *_trace_picture(nal_type=1, ref_idc=2, frame_num=1),
            *_trace_picture(nal_type=1, ref_idc=2, frame_num=2),
        ]
    )


def _broken_trace() -> str:
    return "\n".join(
        [
            *_trace_parameter_sets(),
            *_trace_picture(nal_type=1, ref_idc=2, frame_num=4),
            *_trace_picture(nal_type=1, ref_idc=2, frame_num=5),
            *_trace_picture(nal_type=1, ref_idc=2, frame_num=7),
        ]
    )


def _leading_p_trace() -> str:
    return "\n".join(
        [
            *_trace_parameter_sets(),
            *_trace_picture(nal_type=1, ref_idc=2, frame_num=7),
            *_trace_picture(nal_type=5, ref_idc=3, frame_num=0),
            *_trace_picture(nal_type=1, ref_idc=2, frame_num=1),
        ]
    )


class IntegrityRunner:
    def __init__(
        self,
        *,
        sample_count: int,
        decoded_count: int,
        durations: tuple[int, ...],
        trace: str,
        r_frame_rate: str = "30/1",
    ) -> None:
        self.sample_count = sample_count
        self.decoded_count = decoded_count
        self.durations = durations
        self.trace = trace
        self.r_frame_rate = r_frame_rate
        self.calls: list[list[str]] = []

    def __call__(self, command: list[str], **_kwargs: Any) -> SimpleNamespace:
        self.calls.append(list(command))
        if Path(command[0]).name == "ffmpeg":
            return SimpleNamespace(returncode=0, stdout=b"", stderr=self.trace.encode())
        if "-count_frames" in command:
            payload = {
                "streams": [
                    {
                        "index": 0,
                        "codec_type": "video",
                        "codec_name": "h264",
                        "r_frame_rate": self.r_frame_rate,
                        "avg_frame_rate": "30/1",
                        "time_base": "1/30000",
                        "duration_ts": 3000,
                        "duration": "0.1",
                        "nb_frames": str(self.sample_count),
                        "nb_read_frames": str(self.decoded_count),
                    }
                ]
            }
        else:
            payload = {
                "packets": [
                    {"pts": index * 1000, "dts": index * 1000, "duration": duration}
                    for index, duration in enumerate(self.durations)
                ]
            }
        return SimpleNamespace(returncode=0, stdout=json.dumps(payload), stderr="")


def _standard_probe_runner(command: list[str], **_kwargs: Any) -> SimpleNamespace:
    payload = {
        "streams": [
            {
                "index": 0,
                "codec_type": "video",
                "codec_name": "h264",
                "width": 1280,
                "height": 720,
                "avg_frame_rate": "30/1",
                "r_frame_rate": "30/1",
                "nb_frames": "3",
                "duration": "0.1",
            }
        ],
        "format": {"format_name": "mov,mp4", "duration": "0.1"},
    }
    return SimpleNamespace(returncode=0, stdout=json.dumps(payload), stderr="")


def test_h264_trace_parser_counts_only_reference_picture_discontinuities() -> None:
    trace = "\n".join(
        [
            *_trace_parameter_sets(),
            *_trace_picture(nal_type=5, ref_idc=3, frame_num=0),
            *_trace_picture(nal_type=1, ref_idc=0, frame_num=1),
            *_trace_picture(nal_type=1, ref_idc=2, frame_num=1),
            *_trace_picture(nal_type=1, ref_idc=2, frame_num=3),
        ]
    )

    parsed = parse_h264_trace_headers(trace)

    assert parsed["h264_access_unit_count"] == 4
    assert parsed["h264_idr_access_unit_count"] == 1
    assert parsed["h264_usable_idr_present"] is True
    assert parsed["h264_frame_num_discontinuity_count"] == 1


def test_valid_finalized_h264_copy_mp4_passes_all_independent_checks(tmp_path: Path) -> None:
    video = tmp_path / "valid.mp4"
    _write_mp4_with_stts(video, (1000, 1000, 1000))
    runner = IntegrityRunner(
        sample_count=3,
        decoded_count=3,
        durations=(1000, 1000, 1000),
        trace=_healthy_trace(),
    )

    result = validate_h264_copy_mp4_integrity(
        video,
        probe_runner=runner,
        trace_runner=runner,
    )

    assert result["integrity_schema"] == H264_MP4_INTEGRITY_SCHEMA
    assert result["integrity_validation_status"] == "PASS"
    assert result["recording_usable"] is True
    assert result["recording_integrity_verified"] is True
    assert result["decoder_frame_count_match"] is True
    assert result["h264_usable_idr_present"] is True
    assert result["h264_frame_num_discontinuity_count"] == 0
    assert result["mp4_stts_timing_valid"] is True
    assert result["sample_timing_valid"] is True
    assert result["integrity_failure_reasons"] == []


def test_missing_idr_decoder_loss_frame_num_gap_and_zero_stts_fail_together(
    tmp_path: Path,
) -> None:
    video = tmp_path / "field-failure.mp4"
    _write_mp4_with_stts(video, (1000, 1000, 0))
    runner = IntegrityRunner(
        sample_count=3,
        decoded_count=2,
        durations=(1000, 1000, 0),
        trace=_broken_trace(),
        r_frame_rate="10000/1",
    )

    result = validate_h264_copy_mp4_integrity(
        video,
        probe_runner=runner,
        trace_runner=runner,
    )

    assert inspect_mp4_video_stts(video)["mp4_stts_final_sample_duration"] == 0
    assert result["integrity_validation_status"] == "FAIL"
    assert result["recording_usable"] is False
    assert result["recording_integrity_verified"] is False
    assert result["decoder_visible_loss_count"] == 1
    assert result["h264_idr_access_unit_count"] == 0
    assert result["h264_frame_num_discontinuity_count"] == 1
    assert result["mp4_stts_zero_duration_sample_count"] == 1
    assert result["nominal_frame_rate_plausible"] is False
    assert set(result["integrity_failure_reasons"]) >= {
        "integrity_mp4_stts_zero_sample_duration",
        "integrity_decoder_visible_frame_loss",
        "integrity_nominal_frame_rate_invalid",
        "integrity_sample_duration_nonpositive",
        "integrity_h264_usable_idr_missing",
        "integrity_h264_frame_num_discontinuity",
    }


def test_measurement_descriptor_cannot_be_present_when_copy_integrity_fails(
    tmp_path: Path,
) -> None:
    video = tmp_path / "raw_left.mp4"
    _write_mp4_with_stts(video, (1000, 1000, 0))
    runner = IntegrityRunner(
        sample_count=3,
        decoded_count=2,
        durations=(1000, 1000, 0),
        trace=_broken_trace(),
    )

    descriptor = verify_video_evidence(
        video,
        recording_backend="gstreamer",
        h264_copy_integrity_required=True,
        probe_runner=_standard_probe_runner,
        integrity_probe_runner=runner,
        integrity_trace_runner=runner,
    )

    assert descriptor["status"] == "failed"
    assert descriptor["metadata"]["recording_usable"] is False
    assert descriptor["metadata"]["recording_integrity_verified"] is False
    assert descriptor["metadata"]["integrity_validation_status"] == "FAIL"
    integrity = descriptor["metadata"]["h264_copy_integrity"]
    assert integrity["decoder_visible_loss_count"] == 1
    assert integrity["h264_usable_idr_present"] is False


def test_decode_timeout_fails_closed_without_inventing_sample_count_mismatches(
    tmp_path: Path,
) -> None:
    video = tmp_path / "decode-timeout.mp4"
    _write_mp4_with_stts(video, (1000, 1000, 1000))
    trace_runner = IntegrityRunner(
        sample_count=3,
        decoded_count=3,
        durations=(1000, 1000, 1000),
        trace=_healthy_trace(),
    )

    def probe_runner(command: list[str], **_kwargs: Any) -> SimpleNamespace:
        if "-count_frames" in command:
            raise subprocess.TimeoutExpired(command, timeout=0.01)
        return SimpleNamespace(
            returncode=0,
            stdout=json.dumps(
                {
                    "packets": [
                        {"pts": index * 1000, "dts": index * 1000, "duration": 1000}
                        for index in range(3)
                    ]
                }
            ),
            stderr="",
        )

    result = validate_h264_copy_mp4_integrity(
        video,
        probe_runner=probe_runner,
        trace_runner=trace_runner,
    )

    assert result["integrity_validation_status"] == "FAIL"
    assert result["recording_usable"] is False
    assert "integrity_decode_count_timeout" in result["integrity_failure_reasons"]
    assert "integrity_packet_sample_count_mismatch" not in result[
        "integrity_failure_reasons"
    ]
    assert "integrity_h264_access_unit_sample_count_mismatch" not in result[
        "integrity_failure_reasons"
    ]
    assert result["packet_sample_count_match"] is True
    assert result["h264_access_unit_sample_count_match"] is True


def test_missing_trace_tool_is_not_accepted_as_verified_integrity(tmp_path: Path) -> None:
    video = tmp_path / "missing-trace-tool.mp4"
    _write_mp4_with_stts(video, (1000, 1000, 1000))
    runner = IntegrityRunner(
        sample_count=3,
        decoded_count=3,
        durations=(1000, 1000, 1000),
        trace=_healthy_trace(),
    )

    def missing_trace(*_args: Any, **_kwargs: Any) -> None:
        raise FileNotFoundError("ffmpeg missing")

    result = validate_h264_copy_mp4_integrity(
        video,
        probe_runner=runner,
        trace_runner=missing_trace,
    )

    assert result["integrity_validation_status"] == "FAIL"
    assert result["recording_integrity_verified"] is False
    assert result["integrity_failure_reasons"] == [
        "integrity_ffmpeg_unavailable"
    ]


def test_later_idr_cannot_make_a_leading_predictive_sample_usable(tmp_path: Path) -> None:
    video = tmp_path / "leading-p.mp4"
    _write_mp4_with_stts(video, (1000, 1000, 1000))
    runner = IntegrityRunner(
        sample_count=3,
        decoded_count=3,
        durations=(1000, 1000, 1000),
        trace=_leading_p_trace(),
    )

    result = validate_h264_copy_mp4_integrity(
        video, probe_runner=runner, trace_runner=runner
    )

    assert result["h264_idr_access_unit_count"] == 1
    assert result["h264_first_vcl_is_idr"] is False
    assert result["integrity_validation_status"] == "FAIL"
    assert "integrity_h264_first_vcl_not_idr" in result["integrity_failure_reasons"]


def test_packet_timing_gaps_are_not_merely_treated_as_monotonic(tmp_path: Path) -> None:
    video = tmp_path / "timing-gap.mp4"
    _write_mp4_with_stts(video, (400, 400, 400))
    runner = IntegrityRunner(
        sample_count=3,
        decoded_count=3,
        durations=(400, 400, 400),
        trace=_healthy_trace(),
    )

    result = validate_h264_copy_mp4_integrity(
        video, probe_runner=runner, trace_runner=runner
    )

    assert result["non_contiguous_sample_dts_count"] == 2
    assert result["sample_timing_valid"] is False
    assert "integrity_sample_dts_non_contiguous" in result[
        "integrity_failure_reasons"
    ]


def test_package_bytes_cannot_override_live_terminal_integrity_failure(
    tmp_path: Path,
) -> None:
    video = tmp_path / "terminal-after-early-idr.mp4"
    _write_mp4_with_stts(video, (1000, 1000, 1000))
    runner = IntegrityRunner(
        sample_count=3,
        decoded_count=3,
        durations=(1000, 1000, 1000),
        trace=_healthy_trace(),
    )

    descriptor = verify_video_evidence(
        video,
        recording_backend="gstreamer",
        h264_copy_integrity_required=True,
        recording_finalization_evidence={
            "path": str(video),
            "stream_backend": "gstreamer",
            "container_finalization_success": True,
            "integrity_terminal_failure": "H264_IDR_RANDOM_ACCESS_UNAVAILABLE",
        },
        probe_runner=_standard_probe_runner,
        integrity_probe_runner=runner,
        integrity_trace_runner=runner,
    )

    assert descriptor["status"] == "failed"
    assert descriptor["metadata"]["reason"] == (
        "video_live_integrity_finalization_failed"
    )
    assert descriptor["metadata"]["recording_finalization_boundary_valid"] is False


def test_video_finalized_persistence_preserves_validated_pass(tmp_path: Path) -> None:
    from Workstation.camera_stream_controller import CameraStreamLifecycleController

    path = tmp_path / "valid.mp4"
    path.write_bytes(b"validated-placeholder")
    artifacts: list[dict[str, object]] = []
    finalization = {
        "path": str(path),
        "stream_backend": "gstreamer",
        "success": True,
        "container_finalization_success": True,
        "detail": "EOS received; mp4mux finalized; MP4 integrity PASS",
        "integrity_schema": H264_MP4_INTEGRITY_SCHEMA,
        "integrity_validation_status": "PASS",
        "integrity_failure_reasons": [],
        "recording_usable": True,
        "recording_integrity_verified": True,
    }
    panel = SimpleNamespace(
        _pending_video_finalization_artifacts={
            "paths": {"left": path},
            "backend": "gstreamer",
        },
        left_pane=SimpleNamespace(recording_finalizations={str(path): finalization}),
        append_workstation_data=lambda _stream, payload, **_kwargs: artifacts.append(
            dict(payload)
        ),
        flush_workstation_data=lambda: True,
        _log=lambda _message: None,
    )

    assert CameraStreamLifecycleController(
        panel
    )._append_pending_video_finalization_artifacts() is True

    assert len(artifacts) == 1
    assert artifacts[0]["success"] is True
    assert artifacts[0]["finalization_success"] is True
    assert artifacts[0]["container_finalization_success"] is True
    assert artifacts[0]["recording_usable"] is True
    assert artifacts[0]["recording_integrity_verified"] is True


def test_gstreamer_worker_wires_h264_copy_integrity_validator(tmp_path: Path) -> None:
    """The GStreamer EOS handler in yolo_stream.py finalizes a raw H.264 copy
    recording by calling ``finalization_config["_recording_integrity_validator"]``
    (see the comment above the corresponding EOS handling code). This used to
    be populated by the now-deleted measurement_package_controller.py's
    "package worker" as a later, separate pass -- which never actually ran in
    production since nothing wired it up (see workstation_core/yolo_stream.py's
    stale comment referencing it). It is now wired directly into
    YoloStreamWorker.config so real H.264 copy integrity validation actually
    happens live instead of leaving every such recording stuck at PENDING.
    """
    from Workstation.workstation_core.yolo_stream import YoloStreamWorker

    worker = YoloStreamWorker(
        url="rtsp://camera.invalid/visible",
        weights_path="weights.pt",
        device="cpu",
        input_size=640,
        conf_threshold=0.5,
        iou_threshold=0.45,
        max_rate_hz=0.0,
        enable_detection=False,
        stream_backend="gstreamer",
    )

    assert worker.config["_recording_integrity_validator"] is validate_h264_copy_mp4_integrity

    # A finalized MP4 with no usable H.264 access units must be reported as a
    # real integrity failure through the exact same helper the live EOS
    # handler calls -- not silently left as a "package worker will check this
    # later" placeholder.
    video_path = tmp_path / "raw_left.mp4"
    _write_mp4_with_stts(video_path, (1000, 1000, 1000))
    result = worker.config["_recording_integrity_validator"](
        video_path,
        probe_runner=_standard_probe_runner,
    )

    assert result["integrity_schema"] == H264_MP4_INTEGRITY_SCHEMA
    assert result["integrity_validation_status"] == "FAIL"
    assert result["recording_usable"] is False
    assert result["recording_integrity_verified"] is False
