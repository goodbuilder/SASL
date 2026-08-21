from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from Workstation.workstation_core.media_evidence import (
    DEFAULT_PROBE_TIMEOUT_SECONDS,
    MAX_PROBE_OUTPUT_BYTES,
    VIDEO_EVIDENCE_SCHEMA,
    VideoFileObservation,
    observe_video_files,
    verify_video_evidence,
    video_files_stable,
)
from Workstation.workstation_core.measurement_package import MeasurementPackage


ARTIFACT_KEYS = {
    "artifact_id",
    "role",
    "classification",
    "relative_path",
    "producer",
    "source",
    "required",
    "status",
    "media_type",
    "schema",
    "size_bytes",
    "sha256",
    "metadata",
}


def _probe_payload(
    *,
    codec: str = "hevc",
    container: str = "mov,mp4,m4a,3gp,3g2,mj2",
    width: int = 1920,
    height: int = 1080,
    frame_count: str = "375",
    fps: str = "30000/1001",
    r_fps: str = "25/1",
    stream_duration: str = "12.5125",
    format_duration: str = "12.6000",
) -> str:
    # This fixture is built independently from the parser and keeps adjacent
    # fields deliberately distinct so swapped values cannot pass unnoticed.
    return json.dumps(
        {
            "streams": [
                {
                    "index": 3,
                    "codec_type": "video",
                    "codec_name": codec,
                    "width": width,
                    "height": height,
                    "avg_frame_rate": fps,
                    "r_frame_rate": r_fps,
                    "nb_frames": frame_count,
                    "duration": stream_duration,
                }
            ],
            "format": {"format_name": container, "duration": format_duration},
        }
    )


class FakeRunner:
    def __init__(
        self,
        *,
        stdout: str | bytes = "",
        returncode: int = 0,
        error: BaseException | None = None,
        callback: Any = None,
    ) -> None:
        self.stdout = stdout
        self.returncode = returncode
        self.error = error
        self.callback = callback
        self.calls: list[tuple[list[str], dict[str, Any]]] = []

    def __call__(self, command: list[str], **kwargs: Any) -> SimpleNamespace:
        self.calls.append((list(command), dict(kwargs)))
        if self.callback is not None:
            self.callback()
        if self.error is not None:
            raise self.error
        return SimpleNamespace(returncode=self.returncode, stdout=self.stdout, stderr="ignored")


def test_success_descriptor_hashes_bytes_parses_probe_and_is_portable(tmp_path: Path) -> None:
    video = tmp_path / "raw" / "left.mp4"
    video.parent.mkdir()
    exact_bytes = b"independent-video-file-bytes\x00\xff"
    video.write_bytes(exact_bytes)
    probe = FakeRunner(stdout=_probe_payload())

    descriptor = verify_video_evidence(
        video,
        session_root=tmp_path,
        role="left_oz_video",
        source=str(tmp_path / "camera" / "left"),
        recording_backend="/opt/media/bin/gstreamer",
        encoding_provenance={
            "encoder": "nvh265enc",
            "binary": tmp_path / "private" / "ffmpeg",
            "settings": {"bitrate_bps": 8_000_000},
        },
        probe_runner=probe,
    )

    assert set(descriptor) == ARTIFACT_KEYS
    assert descriptor["status"] == "present"
    assert descriptor["schema"] == VIDEO_EVIDENCE_SCHEMA
    assert descriptor["relative_path"] == "raw/left.mp4"
    assert descriptor["role"] == "left_oz_video"
    assert descriptor["classification"] == "raw"
    assert descriptor["required"] is True
    assert descriptor["size_bytes"] == len(exact_bytes)
    assert descriptor["sha256"] == hashlib.sha256(exact_bytes).hexdigest()
    assert descriptor["media_type"] == "video/mp4"

    metadata = descriptor["metadata"]
    assert metadata["recording_backend"] == "<absolute-path>/gstreamer"
    assert metadata["encoding_provenance"] == {
        "binary": "<absolute-path>/ffmpeg",
        "encoder": "nvh265enc",
        "settings": {"bitrate_bps": 8_000_000},
    }
    assert metadata["container"] == "mov,mp4,m4a,3gp,3g2,mj2"
    assert metadata["codec"] == "hevc"
    assert metadata["width_px"] == 1920
    assert metadata["height_px"] == 1080
    assert metadata["frame_count"] == 375
    assert metadata["frame_count_source"] == "nb_frames"
    assert metadata["fps"] == pytest.approx(30000 / 1001)
    assert metadata["fps_ratio"] == "30000/1001"
    assert metadata["duration_seconds"] == pytest.approx(12.5125)
    assert metadata["duration_source"] == "stream"
    assert metadata["video_stream_index"] == 3
    assert metadata["probe"] == {
        "tool": "ffprobe",
        "status": "passed",
        "timeout_seconds": DEFAULT_PROBE_TIMEOUT_SECONDS,
        "return_code": 0,
    }
    assert metadata["full_decode"]["status"] == "not_requested"
    assert "reason" not in metadata

    serialized = json.dumps(descriptor, sort_keys=True)
    assert str(tmp_path) not in serialized
    assert len(probe.calls) == 1
    command, options = probe.calls[0]
    assert command[-1] == str(video)
    assert command[:3] == ["ffprobe", "-v", "error"]
    assert "-select_streams" in command
    assert options["timeout"] == DEFAULT_PROBE_TIMEOUT_SECONDS
    assert options["capture_output"] is True
    assert options["text"] is True


def test_success_descriptor_registers_directly_with_measurement_package(tmp_path: Path) -> None:
    package = MeasurementPackage.create(tmp_path, session_id="video-descriptor-test")
    video = tmp_path / "raw" / "left.mp4"
    video.parent.mkdir()
    video.write_bytes(b"package-video-evidence")
    descriptor = verify_video_evidence(
        video,
        session_root=tmp_path,
        role="left_oz_video",
        source="left_camera",
        recording_backend="gstreamer",
        probe_runner=FakeRunner(stdout=_probe_payload()),
    )

    registered = package.register_descriptor(descriptor)

    assert registered.to_mapping() == descriptor


def test_stability_gate_is_pure_and_requires_two_equal_ready_observations() -> None:
    paths = (Path("session/raw/left.mp4"), Path("session/raw/right.mp4"))
    left = VideoFileObservation(True, True, False, 101, 1_000)
    right = VideoFileObservation(True, True, False, 202, 2_000)
    first = {paths[0]: left, paths[1]: right}
    identical = {str(paths[0]): left, str(paths[1]): right}

    assert video_files_stable(paths, first, identical) is True
    assert video_files_stable(paths, first) is False

    grew = dict(identical)
    grew[str(paths[1])] = VideoFileObservation(True, True, False, 203, 2_001)
    assert video_files_stable(paths, first, grew) is False

    zero = dict(identical)
    zero[str(paths[0])] = VideoFileObservation(True, True, False, 0, 1_000)
    assert video_files_stable(paths, first, zero) is False

    missing = dict(identical)
    missing[str(paths[0])] = VideoFileObservation(False, False, False, None, None, "missing")
    assert video_files_stable(paths, first, missing) is False


def test_observe_video_files_detects_growth_and_non_regular_paths(tmp_path: Path) -> None:
    video = tmp_path / "video.mp4"
    video.write_bytes(b"first")
    first = observe_video_files([video])
    second = observe_video_files([video])
    assert video_files_stable([video], first, second) is True

    video.write_bytes(b"longer-second-value")
    third = observe_video_files([video])
    assert video_files_stable([video], second, third) is False

    directory = tmp_path / "directory.mp4"
    directory.mkdir()
    directory_observation = observe_video_files([directory])
    assert video_files_stable([directory], directory_observation, directory_observation) is False


def test_missing_empty_directory_and_symlink_are_never_probed_or_present(tmp_path: Path) -> None:
    target = tmp_path / "target.mp4"
    target.write_bytes(b"target-video")
    empty = tmp_path / "empty.mp4"
    empty.touch()
    directory = tmp_path / "directory.mp4"
    directory.mkdir()
    link = tmp_path / "linked.mp4"
    link.symlink_to(target)

    def forbidden_runner(*_args: Any, **_kwargs: Any) -> None:
        raise AssertionError("invalid file must not reach ffprobe")

    cases = (
        (tmp_path / "missing.mp4", "missing", "video_file_missing"),
        (empty, "failed", "video_file_empty"),
        (directory, "failed", "video_file_not_regular"),
        (link, "failed", "video_file_symlink"),
    )
    for path, expected_status, expected_reason in cases:
        descriptor = verify_video_evidence(
            path,
            session_root=tmp_path,
            recording_backend="test-writer",
            probe_runner=forbidden_runner,
        )
        assert descriptor["status"] == expected_status
        assert descriptor["status"] != "present"
        assert descriptor["metadata"]["reason"] == expected_reason
        assert not Path(str(descriptor["relative_path"])).is_absolute()
        assert descriptor["relative_path"] == path.name


@pytest.mark.parametrize(
    ("runner", "expected_reason"),
    (
        (FakeRunner(returncode=9), "video_probe_failed"),
        (FakeRunner(stdout="not-json"), "video_probe_invalid_json"),
        (
            FakeRunner(stdout="x" * (MAX_PROBE_OUTPUT_BYTES + 1)),
            "video_probe_output_too_large",
        ),
        (
            FakeRunner(
                stdout=json.dumps(
                    {
                        "streams": [{"codec_type": "audio", "codec_name": "aac"}],
                        "format": {"format_name": "mp4", "duration": "2.0"},
                    }
                )
            ),
            "video_stream_missing",
        ),
        (
            FakeRunner(
                error=subprocess.TimeoutExpired(cmd=["ffprobe"], timeout=0.25)
            ),
            "video_probe_timeout",
        ),
        (FakeRunner(error=FileNotFoundError()), "video_probe_unavailable"),
        (FakeRunner(error=OSError()), "video_probe_execution_failed"),
    ),
)
def test_corrupt_non_video_and_timeout_probe_results_are_failed(
    tmp_path: Path,
    runner: FakeRunner,
    expected_reason: str,
) -> None:
    video = tmp_path / "candidate.mp4"
    content = b"not-assumed-valid-without-ffprobe"
    video.write_bytes(content)

    descriptor = verify_video_evidence(
        video,
        relative_path="raw/candidate.mp4",
        recording_backend="test-writer",
        probe_runner=runner,
        probe_timeout_seconds=0.25,
    )

    assert descriptor["status"] == "failed"
    assert descriptor["metadata"]["reason"] == expected_reason
    assert descriptor["metadata"]["probe"]["status"] == "failed"
    assert descriptor["size_bytes"] == len(content)
    assert descriptor["sha256"] == hashlib.sha256(content).hexdigest()


@pytest.mark.parametrize(
    ("payload", "expected_reason"),
    (
        (_probe_payload(width=0), "video_dimensions_invalid"),
        (_probe_payload(fps="0/0", r_fps="0/0"), "video_fps_invalid"),
        (_probe_payload(frame_count="0"), "video_frame_count_invalid"),
        (
            _probe_payload(frame_count="N/A", stream_duration="N/A", format_duration="0"),
            "video_duration_invalid",
        ),
        (_probe_payload(codec="N/A"), "video_codec_missing"),
        (_probe_payload(container="N/A"), "video_container_missing"),
    ),
)
def test_invalid_video_metadata_cannot_look_successful(
    tmp_path: Path,
    payload: str,
    expected_reason: str,
) -> None:
    video = tmp_path / "invalid-metadata.mp4"
    video.write_bytes(b"candidate")

    descriptor = verify_video_evidence(
        video,
        recording_backend="test-writer",
        probe_runner=FakeRunner(stdout=payload),
    )

    assert descriptor["status"] == "failed"
    assert descriptor["metadata"]["reason"] == expected_reason


def test_duration_can_be_derived_from_factual_frame_count_and_fps(tmp_path: Path) -> None:
    video = tmp_path / "derived-duration.mp4"
    video.write_bytes(b"candidate")
    payload = _probe_payload(
        frame_count="300",
        fps="30/1",
        stream_duration="N/A",
        format_duration="N/A",
    )

    descriptor = verify_video_evidence(
        video,
        recording_backend="test-writer",
        probe_runner=FakeRunner(stdout=payload),
    )

    assert descriptor["status"] == "present"
    assert descriptor["metadata"]["duration_seconds"] == pytest.approx(10.0)
    assert descriptor["metadata"]["duration_source"] == "derived_frame_count_fps"


def test_optional_full_decode_uses_separate_runner_and_records_success(tmp_path: Path) -> None:
    video = tmp_path / "raw" / "right.mp4"
    video.parent.mkdir()
    video.write_bytes(b"candidate")
    probe = FakeRunner(stdout=_probe_payload())
    decode = FakeRunner(returncode=0)

    descriptor = verify_video_evidence(
        video,
        session_root=tmp_path,
        recording_backend="ffmpeg-copy",
        probe_runner=probe,
        decode_runner=decode,
        full_decode=True,
        decode_timeout_seconds=45,
    )

    assert descriptor["status"] == "present"
    assert descriptor["metadata"]["full_decode"] == {
        "requested": True,
        "tool": "ffmpeg",
        "status": "passed",
        "timeout_seconds": 45.0,
        "return_code": 0,
    }
    assert len(decode.calls) == 1
    command, options = decode.calls[0]
    assert command[0] == "ffmpeg"
    assert command[-5:] == ["-map", "0:v:0", "-f", "null", "-"]
    assert options["timeout"] == 45.0


@pytest.mark.parametrize(
    ("decode", "expected_reason"),
    (
        (FakeRunner(returncode=1), "video_decode_failed"),
        (
            FakeRunner(error=subprocess.TimeoutExpired(cmd=["ffmpeg"], timeout=0.5)),
            "video_decode_timeout",
        ),
    ),
)
def test_requested_full_decode_failure_is_factual_failed_status(
    tmp_path: Path,
    decode: FakeRunner,
    expected_reason: str,
) -> None:
    video = tmp_path / "video.mp4"
    video.write_bytes(b"candidate")

    descriptor = verify_video_evidence(
        video,
        recording_backend="test-writer",
        probe_runner=FakeRunner(stdout=_probe_payload()),
        decode_runner=decode,
        full_decode=True,
        decode_timeout_seconds=0.5,
    )

    assert descriptor["status"] == "failed"
    assert descriptor["metadata"]["reason"] == expected_reason
    assert descriptor["metadata"]["full_decode"]["status"] == "failed"


def test_file_changed_by_probe_is_not_accepted_with_stale_hash(tmp_path: Path) -> None:
    video = tmp_path / "changing.mp4"
    video.write_bytes(b"before")

    def append_bytes() -> None:
        with video.open("ab") as handle:
            handle.write(b"-after")

    probe = FakeRunner(stdout=_probe_payload(), callback=append_bytes)
    descriptor = verify_video_evidence(
        video,
        recording_backend="test-writer",
        probe_runner=probe,
    )

    assert descriptor["status"] == "failed"
    assert descriptor["metadata"]["reason"] == "video_file_changed_during_verification"


def test_descriptor_rejects_absolute_escaping_or_mismatched_package_paths(tmp_path: Path) -> None:
    video = tmp_path / "raw" / "video.mp4"
    video.parent.mkdir()
    video.write_bytes(b"candidate")

    with pytest.raises(ValueError, match="must not be absolute"):
        verify_video_evidence(
            video,
            relative_path=str(video),
            recording_backend="test-writer",
        )
    with pytest.raises(ValueError, match="normalized portable"):
        verify_video_evidence(
            video,
            relative_path="../video.mp4",
            recording_backend="test-writer",
        )
    with pytest.raises(ValueError, match="does not identify"):
        verify_video_evidence(
            video,
            session_root=tmp_path,
            relative_path="raw/other.mp4",
            recording_backend="test-writer",
        )
