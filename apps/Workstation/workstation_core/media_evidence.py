"""Bounded verification and package descriptors for recorded video evidence.

This module verifies an already-flushed recording; it does not own the
recording process or wait for a writer to finish.  Callers can take two or
more :func:`observe_video_files` snapshots and use
:func:`video_files_stable` before verification so a file that is still being
written is never hashed as final evidence.

Authoritative artifact paths returned here are always portable paths relative
to the measurement session.  The local source path is used only to read the
file and invoke the media tools and is never included in the descriptor.
"""

from __future__ import annotations

import hashlib
import json
import math
import mimetypes
import os
import re
import stat
import subprocess
from dataclasses import dataclass
from enum import Enum
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Callable, Iterable, Mapping, Sequence


PRODUCER = "workstation.media_evidence"
VIDEO_EVIDENCE_SCHEMA = "workstation-video-evidence-v1"
DEFAULT_PROBE_TIMEOUT_SECONDS = 10.0
DEFAULT_DECODE_TIMEOUT_SECONDS = 300.0
MAX_PROBE_OUTPUT_BYTES = 1024 * 1024

_MISSING_VALUES = frozenset({"", "n/a", "nan", "none", "unknown"})
_POSIX_ABSOLUTE_IN_TEXT = re.compile(
    r"(?<![A-Za-z0-9_:/])/(?:[^/\s\"'`<>]+/)+[^/\s\"'`<>:,;)}\]]*"
)
_WINDOWS_ABSOLUTE_IN_TEXT = re.compile(
    r"(?<![A-Za-z0-9_])[A-Za-z]:\\(?:[^\\\s\"'`<>]+\\)+"
    r"[^\\\s\"'`<>:,;)}\]]*"
)


Runner = Callable[..., Any]


@dataclass(frozen=True)
class VideoFileObservation:
    """One non-opening filesystem observation used by the stability gate."""

    exists: bool
    is_regular_file: bool
    is_symlink: bool
    size_bytes: int | None
    mtime_ns: int | None
    observation_error: str | None = None

    @property
    def ready(self) -> bool:
        return (
            self.exists
            and self.is_regular_file
            and not self.is_symlink
            and self.size_bytes is not None
            and self.size_bytes > 0
            and self.mtime_ns is not None
            and self.observation_error is None
        )

    @property
    def stability_signature(self) -> tuple[int, int] | None:
        if not self.ready:
            return None
        return (int(self.size_bytes), int(self.mtime_ns))


def observe_video_file(path: str | os.PathLike[str]) -> VideoFileObservation:
    """Observe size and modification time without following a final symlink."""

    source = Path(path).expanduser()
    try:
        details = source.lstat()
    except FileNotFoundError:
        return VideoFileObservation(False, False, False, None, None, "missing")
    except OSError:
        return VideoFileObservation(False, False, False, None, None, "stat_failed")
    is_link = stat.S_ISLNK(details.st_mode)
    return VideoFileObservation(
        exists=True,
        is_regular_file=stat.S_ISREG(details.st_mode),
        is_symlink=is_link,
        size_bytes=int(details.st_size),
        mtime_ns=_mtime_ns(details),
    )


def observe_video_files(
    paths: Iterable[str | os.PathLike[str]],
) -> dict[str, VideoFileObservation]:
    """Return an observation set suitable for :func:`video_files_stable`."""

    return {_path_key(path): observe_video_file(path) for path in paths}


def video_files_stable(
    paths: Iterable[str | os.PathLike[str]],
    *observations: Mapping[str | os.PathLike[str], VideoFileObservation],
) -> bool:
    """Return whether every requested file stayed ready with equal size/mtime.

    This is deliberately a pure comparison: it never sleeps or reads the
    filesystem.  A finalize controller owns the sampling interval and passes
    at least two independently collected observation sets.
    """

    requested = tuple(_path_key(path) for path in paths)
    if not requested or len(set(requested)) != len(requested) or len(observations) < 2:
        return False

    normalized_sets: list[dict[str, VideoFileObservation]] = []
    for observed in observations:
        normalized: dict[str, VideoFileObservation] = {}
        for path, value in observed.items():
            if not isinstance(value, VideoFileObservation):
                return False
            normalized[_path_key(path)] = value
        normalized_sets.append(normalized)

    for path in requested:
        signatures: list[tuple[int, int]] = []
        for observed in normalized_sets:
            item = observed.get(path)
            if item is None or item.stability_signature is None:
                return False
            signatures.append(item.stability_signature)
        if any(signature != signatures[0] for signature in signatures[1:]):
            return False
    return True


def verify_video_evidence(
    path: str | os.PathLike[str],
    *,
    recording_backend: str,
    encoding_provenance: Mapping[str, Any] | Sequence[Any] | str | None = None,
    session_root: str | os.PathLike[str] | None = None,
    relative_path: str | os.PathLike[str] | None = None,
    role: str = "video_recording",
    source: str = "camera",
    classification: str = "raw",
    required: bool = True,
    full_decode: bool = False,
    probe_runner: Runner | None = None,
    decode_runner: Runner | None = None,
    ffprobe_binary: str = "ffprobe",
    ffmpeg_binary: str = "ffmpeg",
    probe_timeout_seconds: float = DEFAULT_PROBE_TIMEOUT_SECONDS,
    decode_timeout_seconds: float = DEFAULT_DECODE_TIMEOUT_SECONDS,
) -> dict[str, Any]:
    """Verify one recording and return a MeasurementPackage-ready descriptor.

    The default check asks ``ffprobe`` only for selected metadata fields and
    enforces a finite timeout and output-size ceiling.  Set ``full_decode`` to
    additionally decode the complete first video stream through ``ffmpeg``.
    Runners use the :func:`subprocess.run` calling convention and are injectable
    so boundary responses can be tested independently of installed binaries.
    """

    source_path = Path(path).expanduser()
    portable_relative = _relative_artifact_path(
        source_path,
        relative_path=relative_path,
        session_root=session_root,
    )
    role_text = _portable_text(_required_text(role, "role"))
    source_text = _portable_text(_required_text(source, "source"))
    backend_text = _portable_text(_required_text(recording_backend, "recording_backend"))
    classification_text = _portable_text(_required_text(classification, "classification"))
    probe_timeout = _positive_timeout(probe_timeout_seconds, "probe_timeout_seconds")
    decode_timeout = (
        _positive_timeout(decode_timeout_seconds, "decode_timeout_seconds")
        if full_decode
        else None
    )
    provenance = _portable_json_value(encoding_provenance or {})
    base_metadata: dict[str, Any] = {
        "recording_backend": backend_text,
        "encoding_provenance": provenance,
        "mp4_frame_index_contract": "not_available",
        "source_name": _portable_text(source_path.name or "video"),
        "probe": {
            "tool": Path(str(ffprobe_binary)).name or "ffprobe",
            "status": "not_run",
            "timeout_seconds": probe_timeout,
        },
        "full_decode": {
            "requested": bool(full_decode),
            "tool": Path(str(ffmpeg_binary)).name or "ffmpeg",
            "status": "not_run" if full_decode else "not_requested",
            "timeout_seconds": decode_timeout if full_decode else None,
        },
    }

    try:
        initial = source_path.lstat()
    except FileNotFoundError:
        return _descriptor(
            status="missing",
            reason="video_file_missing",
            relative_path=portable_relative,
            role=role_text,
            source=source_text,
            classification=classification_text,
            required=required,
            media_type=_media_type(source_path),
            metadata=base_metadata,
        )
    except OSError:
        return _descriptor(
            status="failed",
            reason="video_file_stat_failed",
            relative_path=portable_relative,
            role=role_text,
            source=source_text,
            classification=classification_text,
            required=required,
            media_type=_media_type(source_path),
            metadata=base_metadata,
        )

    if stat.S_ISLNK(initial.st_mode):
        return _descriptor(
            status="failed",
            reason="video_file_symlink",
            relative_path=portable_relative,
            role=role_text,
            source=source_text,
            classification=classification_text,
            required=required,
            media_type=_media_type(source_path),
            metadata=base_metadata,
        )
    if not stat.S_ISREG(initial.st_mode):
        return _descriptor(
            status="failed",
            reason="video_file_not_regular",
            relative_path=portable_relative,
            role=role_text,
            source=source_text,
            classification=classification_text,
            required=required,
            media_type=_media_type(source_path),
            metadata=base_metadata,
        )
    if int(initial.st_size) <= 0:
        return _descriptor(
            status="failed",
            reason="video_file_empty",
            relative_path=portable_relative,
            role=role_text,
            source=source_text,
            classification=classification_text,
            required=required,
            media_type=_media_type(source_path),
            size_bytes=0,
            metadata=base_metadata,
        )

    try:
        size_bytes, digest = _hash_unchanged_regular_file(source_path, initial)
    except _FileChanged:
        return _descriptor(
            status="failed",
            reason="video_file_changed_during_verification",
            relative_path=portable_relative,
            role=role_text,
            source=source_text,
            classification=classification_text,
            required=required,
            media_type=_media_type(source_path),
            metadata=base_metadata,
        )
    except OSError:
        return _descriptor(
            status="failed",
            reason="video_file_read_failed",
            relative_path=portable_relative,
            role=role_text,
            source=source_text,
            classification=classification_text,
            required=required,
            media_type=_media_type(source_path),
            size_bytes=int(initial.st_size),
            metadata=base_metadata,
        )

    probe_metadata, probe_record, probe_reason = _probe_video(
        source_path,
        runner=probe_runner or subprocess.run,
        binary=ffprobe_binary,
        timeout_seconds=probe_timeout,
    )
    base_metadata["probe"] = probe_record
    if probe_reason is not None:
        return _descriptor(
            status="failed",
            reason=probe_reason,
            relative_path=portable_relative,
            role=role_text,
            source=source_text,
            classification=classification_text,
            required=required,
            media_type=_media_type(source_path),
            size_bytes=size_bytes,
            sha256=digest,
            metadata=base_metadata,
        )
    base_metadata.update(probe_metadata or {})

    if full_decode:
        decode_record, decode_reason = _decode_video(
            source_path,
            runner=decode_runner or subprocess.run,
            binary=ffmpeg_binary,
            timeout_seconds=float(decode_timeout),
        )
        base_metadata["full_decode"] = decode_record
        if decode_reason is not None:
            return _descriptor(
                status="failed",
                reason=decode_reason,
                relative_path=portable_relative,
                role=role_text,
                source=source_text,
                classification=classification_text,
                required=required,
                media_type=_media_type(source_path),
                size_bytes=size_bytes,
                sha256=digest,
                metadata=base_metadata,
            )

    if not _path_matches_stat(source_path, initial):
        return _descriptor(
            status="failed",
            reason="video_file_changed_during_verification",
            relative_path=portable_relative,
            role=role_text,
            source=source_text,
            classification=classification_text,
            required=required,
            media_type=_media_type(source_path),
            size_bytes=size_bytes,
            sha256=digest,
            metadata=base_metadata,
        )

    return _descriptor(
        status="present",
        reason=None,
        relative_path=portable_relative,
        role=role_text,
        source=source_text,
        classification=classification_text,
        required=required,
        media_type=_media_type(source_path),
        size_bytes=size_bytes,
        sha256=digest,
        metadata=base_metadata,
    )


def verify_video_file(*args: Any, **kwargs: Any) -> dict[str, Any]:
    """Compatibility spelling for :func:`verify_video_evidence`."""

    return verify_video_evidence(*args, **kwargs)


class _FileChanged(Exception):
    pass


def _path_key(path: str | os.PathLike[str]) -> str:
    return os.fspath(Path(path).expanduser())


def _mtime_ns(details: os.stat_result) -> int:
    return int(getattr(details, "st_mtime_ns", round(details.st_mtime * 1_000_000_000)))


def _stat_signature(details: os.stat_result) -> tuple[int, int, int, int]:
    return (
        int(details.st_dev),
        int(details.st_ino),
        int(details.st_size),
        _mtime_ns(details),
    )


def _path_matches_stat(path: Path, expected: os.stat_result) -> bool:
    try:
        current = path.lstat()
    except OSError:
        return False
    return stat.S_ISREG(current.st_mode) and _stat_signature(current) == _stat_signature(expected)


def _hash_unchanged_regular_file(path: Path, expected: os.stat_result) -> tuple[int, str]:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    digest = hashlib.sha256()
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or _stat_signature(before) != _stat_signature(expected):
            raise _FileChanged
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
        after = os.fstat(descriptor)
        if _stat_signature(after) != _stat_signature(before):
            raise _FileChanged
    finally:
        os.close(descriptor)
    if not _path_matches_stat(path, expected):
        raise _FileChanged
    return int(expected.st_size), digest.hexdigest()


def _probe_video(
    path: Path,
    *,
    runner: Runner,
    binary: str,
    timeout_seconds: float,
) -> tuple[dict[str, Any] | None, dict[str, Any], str | None]:
    record: dict[str, Any] = {
        "tool": Path(str(binary)).name or "ffprobe",
        "status": "failed",
        "timeout_seconds": timeout_seconds,
    }
    command = [
        str(binary),
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        (
            "format=format_name,duration:"
            "stream=index,codec_type,codec_name,width,height,avg_frame_rate,"
            "r_frame_rate,nb_frames,nb_read_frames,duration"
        ),
        "-of",
        "json",
        os.fspath(path),
    ]
    try:
        result = runner(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
        )
    except subprocess.TimeoutExpired:
        record["reason"] = "video_probe_timeout"
        return None, record, "video_probe_timeout"
    except FileNotFoundError:
        record["reason"] = "video_probe_unavailable"
        return None, record, "video_probe_unavailable"
    except OSError:
        record["reason"] = "video_probe_execution_failed"
        return None, record, "video_probe_execution_failed"

    return_code = int(getattr(result, "returncode", 1))
    record["return_code"] = return_code
    if return_code != 0:
        record["reason"] = "video_probe_failed"
        return None, record, "video_probe_failed"
    stdout = getattr(result, "stdout", "")
    if isinstance(stdout, bytes):
        output_bytes = stdout
        output_text = stdout.decode("utf-8", errors="replace")
    else:
        output_text = str(stdout or "")
        output_bytes = output_text.encode("utf-8", errors="replace")
    if len(output_bytes) > MAX_PROBE_OUTPUT_BYTES:
        record["reason"] = "video_probe_output_too_large"
        return None, record, "video_probe_output_too_large"
    try:
        payload = json.loads(output_text)
    except (json.JSONDecodeError, TypeError, ValueError):
        record["reason"] = "video_probe_invalid_json"
        return None, record, "video_probe_invalid_json"
    metadata, reason = _parse_probe_payload(payload)
    if reason is not None:
        record["reason"] = reason
        return None, record, reason
    record["status"] = "passed"
    return metadata, record, None


def _parse_probe_payload(payload: Any) -> tuple[dict[str, Any] | None, str | None]:
    if not isinstance(payload, Mapping):
        return None, "video_probe_invalid_payload"
    raw_streams = payload.get("streams")
    if not isinstance(raw_streams, list):
        return None, "video_stream_missing"
    stream = next(
        (
            candidate
            for candidate in raw_streams
            if isinstance(candidate, Mapping)
            and str(candidate.get("codec_type") or "").strip().lower() == "video"
        ),
        None,
    )
    if stream is None:
        return None, "video_stream_missing"

    codec = str(stream.get("codec_name") or "").strip()
    if codec.lower() in _MISSING_VALUES:
        return None, "video_codec_missing"
    format_data = payload.get("format")
    if not isinstance(format_data, Mapping):
        return None, "video_container_missing"
    container = str(format_data.get("format_name") or "").strip()
    if container.lower() in _MISSING_VALUES:
        return None, "video_container_missing"
    try:
        width = int(stream.get("width"))
        height = int(stream.get("height"))
    except (TypeError, ValueError, OverflowError):
        return None, "video_dimensions_invalid"
    if width <= 0 or height <= 0:
        return None, "video_dimensions_invalid"

    fps: float | None = None
    fps_ratio: str | None = None
    for candidate in (stream.get("avg_frame_rate"), stream.get("r_frame_rate")):
        parsed = _positive_ratio(candidate)
        if parsed is not None:
            fps, fps_ratio = parsed
            break
    if fps is None:
        return None, "video_fps_invalid"

    frame_count: int | None = None
    frame_count_source: str | None = None
    for key in ("nb_frames", "nb_read_frames"):
        candidate = stream.get(key)
        if _is_missing(candidate):
            continue
        try:
            parsed_count = int(str(candidate).strip())
        except (TypeError, ValueError, OverflowError):
            return None, "video_frame_count_invalid"
        if parsed_count <= 0:
            return None, "video_frame_count_invalid"
        frame_count = parsed_count
        frame_count_source = key
        break

    duration: float | None = None
    duration_source: str | None = None
    for candidate, candidate_source in (
        (stream.get("duration"), "stream"),
        (format_data.get("duration"), "format"),
    ):
        parsed_duration = _positive_float(candidate)
        if parsed_duration is not None:
            duration = parsed_duration
            duration_source = candidate_source
            break
    if duration is None and frame_count is not None:
        duration = frame_count / fps
        duration_source = "derived_frame_count_fps"
    if duration is None or not math.isfinite(duration) or duration <= 0.0:
        return None, "video_duration_invalid"

    index: int | None
    try:
        index = int(stream.get("index")) if stream.get("index") is not None else None
    except (TypeError, ValueError, OverflowError):
        index = None
    return {
        "container": _portable_text(container),
        "codec": _portable_text(codec),
        "width_px": width,
        "height_px": height,
        "frame_count": frame_count,
        "frame_count_source": frame_count_source,
        "fps": fps,
        "fps_ratio": fps_ratio,
        "duration_seconds": duration,
        "duration_source": duration_source,
        "video_stream_index": index,
    }, None


def _decode_video(
    path: Path,
    *,
    runner: Runner,
    binary: str,
    timeout_seconds: float,
) -> tuple[dict[str, Any], str | None]:
    record: dict[str, Any] = {
        "requested": True,
        "tool": Path(str(binary)).name or "ffmpeg",
        "status": "failed",
        "timeout_seconds": timeout_seconds,
    }
    command = [
        str(binary),
        "-v",
        "error",
        "-xerror",
        "-nostdin",
        "-i",
        os.fspath(path),
        "-map",
        "0:v:0",
        "-f",
        "null",
        "-",
    ]
    try:
        result = runner(
            command,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=timeout_seconds,
        )
    except subprocess.TimeoutExpired:
        record["reason"] = "video_decode_timeout"
        return record, "video_decode_timeout"
    except FileNotFoundError:
        record["reason"] = "video_decode_unavailable"
        return record, "video_decode_unavailable"
    except OSError:
        record["reason"] = "video_decode_execution_failed"
        return record, "video_decode_execution_failed"
    return_code = int(getattr(result, "returncode", 1))
    record["return_code"] = return_code
    if return_code != 0:
        record["reason"] = "video_decode_failed"
        return record, "video_decode_failed"
    record["status"] = "passed"
    return record, None


def _positive_ratio(value: Any) -> tuple[float, str] | None:
    if _is_missing(value):
        return None
    text = str(value).strip()
    try:
        if "/" in text:
            numerator_text, denominator_text = text.split("/", 1)
            denominator = float(denominator_text)
            if denominator == 0.0:
                return None
            result = float(numerator_text) / denominator
        else:
            result = float(text)
    except (TypeError, ValueError, OverflowError, ZeroDivisionError):
        return None
    if not math.isfinite(result) or result <= 0.0:
        return None
    return result, text


def _positive_float(value: Any) -> float | None:
    if _is_missing(value):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) and result > 0.0 else None


def _is_missing(value: Any) -> bool:
    return value is None or str(value).strip().lower() in _MISSING_VALUES


def _positive_timeout(value: Any, name: str) -> float:
    try:
        timeout = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite positive number") from exc
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return timeout


def _relative_artifact_path(
    source: Path,
    *,
    relative_path: str | os.PathLike[str] | None,
    session_root: str | os.PathLike[str] | None,
) -> str:
    root = Path(session_root).expanduser().absolute() if session_root is not None else None
    if relative_path is None:
        if root is None:
            raw = source.name or "video"
        else:
            try:
                raw = source.absolute().relative_to(root).as_posix()
            except ValueError as exc:
                raise ValueError("video path is outside session_root") from exc
    else:
        raw = os.fspath(relative_path)
    windows = PureWindowsPath(raw)
    posix = PurePosixPath(str(raw).replace("\\", "/"))
    if windows.is_absolute() or posix.is_absolute():
        raise ValueError("relative_path must not be absolute")
    if not posix.parts or any(part in {"", ".", ".."} for part in posix.parts):
        raise ValueError("relative_path must be a normalized portable package path")
    portable = posix.as_posix()
    if "\x00" in portable:
        raise ValueError("relative_path must not contain NUL")
    if root is not None and relative_path is not None:
        expected = (root / Path(*posix.parts)).absolute()
        if source.absolute() != expected:
            raise ValueError("relative_path does not identify video path inside session_root")
    return portable


def _required_text(value: Any, name: str) -> str:
    text = str(value or "").strip()
    if not text or "\x00" in text:
        raise ValueError(f"{name} is required")
    return text


def _portable_text(value: str) -> str:
    stripped = str(value)
    if PurePosixPath(stripped).is_absolute():
        name = PurePosixPath(stripped).name
        return f"<absolute-path>/{name}" if name else "<absolute-path>"
    windows = PureWindowsPath(stripped)
    if windows.is_absolute():
        name = windows.name
        return f"<absolute-path>/{name}" if name else "<absolute-path>"

    def replace_posix(match: re.Match[str]) -> str:
        name = PurePosixPath(match.group(0)).name
        return f"<absolute-path>/{name}" if name else "<absolute-path>"

    def replace_windows(match: re.Match[str]) -> str:
        name = PureWindowsPath(match.group(0)).name
        return f"<absolute-path>/{name}" if name else "<absolute-path>"

    return _WINDOWS_ABSOLUTE_IN_TEXT.sub(
        replace_windows,
        _POSIX_ABSOLUTE_IN_TEXT.sub(replace_posix, stripped),
    )


def _portable_json_value(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("encoding_provenance contains a non-finite number")
        return value
    if isinstance(value, Enum):
        return _portable_json_value(value.value)
    if isinstance(value, Path):
        return _portable_text(str(value))
    if isinstance(value, str):
        return _portable_text(value)
    if isinstance(value, Mapping):
        return {
            _portable_text(str(key)): _portable_json_value(item)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
    if isinstance(value, (tuple, list)):
        return [_portable_json_value(item) for item in value]
    raise TypeError(f"unsupported encoding_provenance value: {type(value).__name__}")


def _media_type(path: Path) -> str:
    guessed, _encoding = mimetypes.guess_type(path.name)
    return guessed or "application/octet-stream"


def _artifact_id(*, role: str, relative_path: str, status: str, sha256: str | None) -> str:
    identity = json.dumps(
        {
            "relative_path": relative_path,
            "role": role,
            "sha256": sha256,
            "status": status,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return f"artifact-{hashlib.sha256(identity).hexdigest()[:24]}"


def _descriptor(
    *,
    status: str,
    reason: str | None,
    relative_path: str,
    role: str,
    source: str,
    classification: str,
    required: bool,
    media_type: str,
    metadata: Mapping[str, Any],
    size_bytes: int | None = None,
    sha256: str | None = None,
) -> dict[str, Any]:
    details = dict(_portable_json_value(metadata))
    if reason is not None:
        details["reason"] = reason
    return {
        "artifact_id": _artifact_id(
            role=role,
            relative_path=relative_path,
            status=status,
            sha256=sha256,
        ),
        "role": role,
        "classification": classification,
        "relative_path": relative_path,
        "producer": PRODUCER,
        "source": source,
        "required": bool(required),
        "status": status,
        "media_type": media_type,
        "schema": VIDEO_EVIDENCE_SCHEMA,
        "size_bytes": size_bytes,
        "sha256": sha256,
        "metadata": details,
    }


__all__ = [
    "DEFAULT_DECODE_TIMEOUT_SECONDS",
    "DEFAULT_PROBE_TIMEOUT_SECONDS",
    "MAX_PROBE_OUTPUT_BYTES",
    "PRODUCER",
    "VIDEO_EVIDENCE_SCHEMA",
    "VideoFileObservation",
    "observe_video_file",
    "observe_video_files",
    "verify_video_evidence",
    "verify_video_file",
    "video_files_stable",
]
