import json
import shutil
import subprocess
from pathlib import Path

import pytest

from wiris_stereo_distance.session_recording_parquet import VideoMapping
import wiris_stereo_distance.session_video as video_module
from wiris_stereo_distance.session_video import MkvWriterAdapter, MkvWriterError, probe_mkv


def _make_h264_fixture(tmp_path: Path) -> Path:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        pytest.skip("ffmpeg is unavailable")
    fixture = tmp_path / "fixture.mp4"
    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "lavfi",
        "-i",
        "testsrc=size=32x24:rate=2",
        "-frames:v",
        "2",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(fixture),
    ]
    try:
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.CalledProcessError) as exc:
        pytest.skip(f"H.264 fixture unavailable: {exc}")
    return fixture


def test_local_h264_fixture_is_remuxed_to_real_mkv_with_timing(tmp_path: Path) -> None:
    fixture = _make_h264_fixture(tmp_path)
    output = tmp_path / "camera" / "left_video.mkv"
    writer = MkvWriterAdapter("left", output)
    result = writer.remux_fixture(fixture)
    assert writer.state == "ready"
    assert output.is_file()
    assert "matroska" in result.format_name.split(",")
    assert result.codec_name == "h264"
    assert result.frame_count >= 1
    assert result.decoded_frame_count == result.frame_count
    assert result.packet_count is not None
    assert result.time_base_num is not None
    assert result.time_base_den is not None
    assert result.packets
    assert any(packet.pts_ticks is not None for packet in result.packets)
    assert writer.close() == result
    assert probe_mkv(output).codec_name == "h264"


def test_video_mapping_can_be_stored_and_intentional_drop_is_explicit(tmp_path: Path) -> None:
    received: list[VideoMapping] = []
    sources = {
        "right-source-0": {"camera_side": "right", "frame_id": 0},
        "right-source-1": {"camera_side": "right", "frame_id": 1},
    }
    writer = MkvWriterAdapter(
        "right",
        tmp_path / "right_video.mkv",
        mapping_sink=received.append,
        source_record_resolver=sources.get,
    )
    writer.start()
    writer.mark_mapping(
        VideoMapping(
            source_record_id="right-source-0",
            stored_in_video=True,
            video_mapping_status="stored",
            video_frame_index=0,
            video_packet_pts_ticks=0,
            video_packet_dts_ticks=0,
        )
    )
    writer.mark_mapping(
        VideoMapping(
            source_record_id="right-source-1",
            stored_in_video=False,
            video_mapping_status="dropped",
            drop_reason="video_queue_full",
        )
    )
    assert [mapping.source_record_id for mapping in received] == ["right-source-0", "right-source-1"]
    assert received[1].stored_in_video is False
    assert received[1].drop_reason == "video_queue_full"
    writer.mark_failure("synthetic", "forced failure")
    writer.close()
    assert writer.state == "closed"


def test_writer_failure_does_not_delete_partial_artifact(tmp_path: Path) -> None:
    output = tmp_path / "left_video.mkv"
    writer = MkvWriterAdapter("left", output)
    writer.start()
    output.write_bytes(b"partial-mkv-artifact")
    writer.mark_failure("writer", "forced failure")
    writer.close()
    assert output.read_bytes() == b"partial-mkv-artifact"
    assert writer.failure == ("writer", "forced failure")


def test_mapping_sink_is_validated_before_adapter_state_changes(tmp_path: Path) -> None:
    def rejecting_sink(_mapping: VideoMapping) -> None:
        raise RuntimeError("sink rejected mapping")

    writer = MkvWriterAdapter(
        "left",
        tmp_path / "left_video.mkv",
        mapping_sink=rejecting_sink,
        source_record_resolver=lambda source_id: {"camera_side": "left", "frame_id": 0}
        if source_id == "left-source-0"
        else None,
    )
    writer.start()
    with pytest.raises(RuntimeError):
        writer.mark_mapping(
            VideoMapping(
                source_record_id="left-source-0",
                stored_in_video=True,
                video_mapping_status="stored",
                video_frame_index=0,
            )
        )
    assert writer.mappings == ()
    writer.close()


def test_invalid_drop_mapping_requires_a_reason(tmp_path: Path) -> None:
    writer = MkvWriterAdapter("right", tmp_path / "right_video.mkv")
    writer.start()
    with pytest.raises(MkvWriterError):
        writer.mark_mapping(
            VideoMapping(
                source_record_id="right-source-0",
                stored_in_video=False,
                video_mapping_status="dropped",
            )
        )
    assert writer.mappings == ()
    writer.close()


def test_standalone_mapping_requires_source_authority(tmp_path: Path) -> None:
    writer = MkvWriterAdapter("left", tmp_path / "left_video.mkv")
    writer.start()
    with pytest.raises(MkvWriterError, match="authority"):
        writer.mark_mapping(
            VideoMapping(
                source_record_id="left-source-0",
                stored_in_video=False,
                video_mapping_status="dropped",
                drop_reason="queue_full",
            )
        )
    writer.close()

    writer_with_untrusted_sink = MkvWriterAdapter(
        "left",
        tmp_path / "untrusted-sink.mkv",
        mapping_sink=lambda _mapping: None,
    )
    writer_with_untrusted_sink.start()
    with pytest.raises(MkvWriterError, match="authority"):
        writer_with_untrusted_sink.mark_mapping(
            VideoMapping(
                source_record_id="left-source-0",
                stored_in_video=False,
                video_mapping_status="dropped",
                drop_reason="queue_full",
            )
        )
    writer_with_untrusted_sink.close()


def test_source_resolver_rejects_missing_wrong_side_and_frame_mismatch(tmp_path: Path) -> None:
    sources = {
        "left-source-0": {"camera_side": "left", "frame_id": 0},
        "right-source-0": {"camera_side": "right", "frame_id": 0},
    }
    for source_id, expected in (
        ("missing", "unknown source_record_id"),
        ("right-source-0", "camera_side"),
    ):
        writer = MkvWriterAdapter("left", tmp_path / f"{source_id}.mkv", source_record_resolver=sources.get)
        writer.start()
        with pytest.raises(MkvWriterError, match=expected):
            writer.mark_mapping(
                VideoMapping(
                    source_record_id=source_id,
                    stored_in_video=False,
                    video_mapping_status="dropped",
                    drop_reason="queue_full",
                )
            )
        writer.close()

    writer = MkvWriterAdapter("left", tmp_path / "frame-mismatch.mkv", source_record_resolver=sources.get)
    writer.start()
    with pytest.raises(MkvWriterError, match="frame_id"):
        writer.mark_mapping(
            VideoMapping(
                source_record_id="left-source-0",
                stored_in_video=False,
                video_mapping_status="dropped",
                drop_reason="queue_full",
                source_frame_id=99,
            )
        )
    writer.close()


def test_adapter_rejects_duplicate_video_frame_index(tmp_path: Path) -> None:
    sources = {
        "left-source-0": {"camera_side": "left", "frame_id": 0},
        "left-source-1": {"camera_side": "left", "frame_id": 1},
    }
    writer = MkvWriterAdapter("left", tmp_path / "duplicate-frame.mkv", source_record_resolver=sources.get)
    writer.start()
    writer.mark_mapping(
        VideoMapping(
            source_record_id="left-source-0",
            stored_in_video=True,
            video_mapping_status="stored",
            video_frame_index=0,
        )
    )
    with pytest.raises(MkvWriterError, match="duplicate video frame index"):
        writer.mark_mapping(
            VideoMapping(
                source_record_id="left-source-1",
                stored_in_video=True,
                video_mapping_status="stored",
                video_frame_index=0,
            )
        )
    assert len(writer.mappings) == 1
    writer.close()


def test_existing_output_is_never_overwritten(tmp_path: Path) -> None:
    output = tmp_path / "left_video.mkv"
    output.write_bytes(b"existing")
    writer = MkvWriterAdapter("left", output)
    with pytest.raises(MkvWriterError):
        writer.start()
    assert output.read_bytes() == b"existing"


def test_ffprobe_separates_decoded_frames_from_packets(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "fixture.mkv"
    path.write_bytes(b"synthetic-probe-input")
    commands: list[list[str]] = []

    class Completed:
        stdout = json.dumps(
            {
                "streams": [
                    {
                        "codec_type": "video",
                        "codec_name": "h264",
                        "time_base": "1/90000",
                        "nb_frames": "N/A",
                        "nb_read_frames": "2",
                        "nb_read_packets": "3",
                    }
                ],
                "format": {"format_name": "matroska,webm"},
                "packets": [],
            }
        )

    def fake_run(command: list[str], **_: object) -> Completed:
        commands.append(command)
        return Completed()

    monkeypatch.setattr(video_module.subprocess, "run", fake_run)
    result = probe_mkv(path, ffprobe_binary="ffprobe")
    assert result.decoded_frame_count == 2
    assert result.packet_count == 3
    assert result.frame_count == 2
    assert "-count_frames" in commands[0]
    assert "-count_packets" in commands[0]


def test_ffprobe_unavailable_decoded_count_is_not_packet_fallback(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "fixture.mkv"
    path.write_bytes(b"synthetic-probe-input")

    class Completed:
        stdout = json.dumps(
            {
                "streams": [
                    {
                        "codec_type": "video",
                        "codec_name": "h264",
                        "time_base": "1/90000",
                        "nb_frames": "N/A",
                        "nb_read_frames": "N/A",
                        "nb_read_packets": "4",
                    }
                ],
                "format": {"format_name": "matroska,webm"},
            }
        )

    monkeypatch.setattr(video_module.subprocess, "run", lambda *_args, **_kwargs: Completed())
    result = probe_mkv(path, ffprobe_binary="ffprobe", include_packets=False)
    assert result.decoded_frame_count is None
    assert result.packet_count == 4
    with pytest.raises(MkvWriterError, match="decoded frame count"):
        _ = result.frame_count
