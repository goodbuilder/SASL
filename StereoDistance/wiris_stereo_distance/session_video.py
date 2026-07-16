"""Phase A local-fixture MKV adapter.

The adapter is intentionally independent of the live GStreamer/RTSP path.
It remuxes an existing H.264 fixture into Matroska with stream-copy and
exposes ffprobe evidence for tests.  Phase B can replace the fixture source
with a shared GStreamer tee without changing the mapping contract.
"""

from __future__ import annotations

import json
import os
import shutil
import sqlite3
import subprocess
import tempfile
import uuid
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Mapping

from .session_recording_parquet import VideoMapping, validate_video_mapping


class MkvWriterError(RuntimeError):
    """Raised when a fixture cannot be remuxed or probed as Matroska."""


@dataclass(frozen=True)
class VideoPacketTiming:
    packet_index: int
    stream_index: int
    pts_ticks: int | None
    dts_ticks: int | None


@dataclass(frozen=True)
class MkvProbeResult:
    path: Path
    format_name: str
    codec_name: str
    decoded_frame_count: int | None
    packet_count: int | None
    time_base_num: int | None
    time_base_den: int | None
    packets: tuple[VideoPacketTiming, ...] = ()

    @property
    def frame_count(self) -> int:
        """Compatibility alias for the authoritative decoded-frame count."""

        if self.decoded_frame_count is None:
            raise MkvWriterError("decoded frame count is unavailable")
        return self.decoded_frame_count


def _binary(name: str, explicit: str | Path | None) -> str:
    value = str(explicit) if explicit is not None else shutil.which(name)
    if not value:
        raise MkvWriterError(f"required executable is unavailable: {name}")
    return value


def _integer_or_none(value: object) -> int | None:
    if value is None or value == "N/A":
        return None
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise MkvWriterError(f"invalid media timestamp from ffprobe: {value!r}") from exc


def _parse_time_base(value: object) -> tuple[int | None, int | None]:
    if value in (None, "", "N/A"):
        return None, None
    text = str(value)
    if "/" not in text:
        raise MkvWriterError(f"invalid media time base from ffprobe: {text!r}")
    numerator, denominator = text.split("/", 1)
    try:
        numerator_i = int(numerator)
        denominator_i = int(denominator)
    except ValueError as exc:
        raise MkvWriterError(f"invalid media time base from ffprobe: {text!r}") from exc
    if numerator_i <= 0 or denominator_i <= 0:
        raise MkvWriterError(f"media time base must be positive: {text!r}")
    return numerator_i, denominator_i


def _safe_process_error(exc: BaseException, *, context: str) -> str:
    """Keep bounded diagnostic stderr without persisting credentials."""

    stderr = getattr(exc, "stderr", None)
    detail = str(stderr or exc).replace("\x00", "")
    for marker in ("rtsp://", "rtsps://"):
        if marker in detail:
            prefix, suffix = detail.split(marker, 1)
            detail = prefix + marker + "<redacted>" + ("" if "/" not in suffix else "/" + suffix.split("/", 1)[1])
    detail = detail[-4000:]
    return f"{context}: {detail}"


def probe_mkv(
    path: str | Path,
    *,
    ffprobe_binary: str | Path | None = None,
    timeout_s: float = 30.0,
    include_packets: bool = True,
) -> MkvProbeResult:
    """Probe a real Matroska video and return codec/count/timing evidence."""

    video_path = Path(path).expanduser()
    if not video_path.is_file() or video_path.stat().st_size <= 0:
        raise MkvWriterError(f"MKV file is missing or empty: {video_path}")
    binary = _binary("ffprobe", ffprobe_binary)
    command = [
        binary,
        "-v",
        "error",
        "-count_frames",
        "-count_packets",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=codec_type,codec_name,time_base,nb_frames,nb_read_frames,nb_read_packets",
        "-show_entries",
        "format=format_name",
        "-of",
        "json",
    ]
    if include_packets:
        command.extend(
            [
                "-show_packets",
                "-show_entries",
                "packet=stream_index,pts,dts",
            ]
        )
    command.append(str(video_path))
    try:
        completed = subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.CalledProcessError as exc:
        raise MkvWriterError(_safe_process_error(exc, context=f"ffprobe failed for {video_path.name}")) from exc
    except (OSError, subprocess.SubprocessError) as exc:
        raise MkvWriterError(f"ffprobe failed for {video_path.name}: {exc}") from exc
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise MkvWriterError(f"ffprobe returned invalid JSON for {video_path}") from exc

    format_name = str(payload.get("format", {}).get("format_name", ""))
    if "matroska" not in format_name.split(","):
        raise MkvWriterError(f"not a Matroska container: {format_name!r}")
    streams = payload.get("streams") or []
    if not streams:
        raise MkvWriterError(f"MKV has no video stream: {video_path}")
    stream = streams[0]
    if stream.get("codec_type") != "video":
        raise MkvWriterError(f"first selected stream is not video: {stream.get('codec_type')!r}")
    codec_name = str(stream.get("codec_name") or "")
    decoded_frame_count = _integer_or_none(stream.get("nb_read_frames"))
    packet_count = _integer_or_none(stream.get("nb_read_packets"))
    time_base_num, time_base_den = _parse_time_base(stream.get("time_base"))
    packets: list[VideoPacketTiming] = []
    for packet_index, packet in enumerate(payload.get("packets") or []):
        packets.append(
            VideoPacketTiming(
                packet_index=packet_index,
                stream_index=int(packet.get("stream_index", 0)),
                pts_ticks=_integer_or_none(packet.get("pts")),
                dts_ticks=_integer_or_none(packet.get("dts")),
            )
        )
    return MkvProbeResult(
        path=video_path,
        format_name=format_name,
        codec_name=codec_name,
        decoded_frame_count=decoded_frame_count,
        packet_count=packet_count,
        time_base_num=time_base_num,
        time_base_den=time_base_den,
        packets=tuple(packets),
    )


class MkvWriterAdapter:
    """Remux a local H.264 fixture into a Phase A camera MKV artifact."""

    def __init__(
        self,
        camera_side: str,
        output_path: str | Path,
        *,
        ffmpeg_binary: str | Path | None = None,
        ffprobe_binary: str | Path | None = None,
        timeout_s: float = 30.0,
        mapping_sink: Callable[[VideoMapping], None] | None = None,
        source_record_resolver: Callable[[str], object | None] | None = None,
    ) -> None:
        if camera_side not in {"left", "right"}:
            raise MkvWriterError(f"camera_side must be left or right: {camera_side!r}")
        self.camera_side = camera_side
        self.output_path = Path(output_path).expanduser()
        self.ffmpeg_binary = ffmpeg_binary
        self.ffprobe_binary = ffprobe_binary
        self.timeout_s = timeout_s
        self.mapping_sink = mapping_sink
        self.source_record_resolver = source_record_resolver
        self._state = "new"
        self._failure_stage: str | None = None
        self._failure_message: str | None = None
        self._result: MkvProbeResult | None = None
        self._association: sqlite3.Connection | None = None
        self._association_path: Path | None = None

    @property
    def state(self) -> str:
        return self._state

    @property
    def result(self) -> MkvProbeResult | None:
        return self._result

    @property
    def failure(self) -> tuple[str, str] | None:
        if self._failure_stage is None or self._failure_message is None:
            return None
        return self._failure_stage, self._failure_message

    @property
    def mappings(self) -> tuple[VideoMapping, ...]:
        if self._association is None:
            return ()
        rows = self._association.execute(
            """SELECT source_record_id,camera_side,stored_in_video,video_mapping_status,
                      video_frame_index,video_packet_pts_ticks,video_packet_dts_ticks,
                      drop_reason,source_frame_id
                 FROM video_mappings ORDER BY rowid"""
        )
        return tuple(
            VideoMapping(
                source_record_id=row[0],
                camera_side=row[1],
                stored_in_video=bool(row[2]),
                video_mapping_status=row[3],
                video_frame_index=row[4],
                video_packet_pts_ticks=row[5],
                video_packet_dts_ticks=row[6],
                drop_reason=row[7],
                source_frame_id=row[8],
            )
            for row in rows
        )

    def start(self) -> None:
        if self._state == "recording":
            return
        if self._state != "new":
            raise MkvWriterError(f"cannot start writer in state {self._state!r}")
        if self.output_path.exists():
            raise MkvWriterError(f"refusing to overwrite existing video: {self.output_path}")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            fd, path = tempfile.mkstemp(prefix="stereodistance-mkv-", suffix=".sqlite3")
            os.close(fd)
            self._association_path = Path(path)
            self._association = sqlite3.connect(path)
            self._association.executescript(
                """
                CREATE TABLE video_mappings (
                    source_record_id TEXT PRIMARY KEY,
                    camera_side TEXT NOT NULL,
                    stored_in_video INTEGER NOT NULL,
                    video_mapping_status TEXT NOT NULL,
                    video_frame_index INTEGER,
                    video_packet_pts_ticks INTEGER,
                    video_packet_dts_ticks INTEGER,
                    drop_reason TEXT,
                    source_frame_id INTEGER
                );
                CREATE UNIQUE INDEX video_mapping_frame_unique
                    ON video_mappings(video_frame_index)
                    WHERE stored_in_video = 1;
                """
            )
            self._association.commit()
        except (OSError, sqlite3.Error) as exc:
            self.mark_failure("association_index", str(exc))
            raise MkvWriterError(f"failed to open video association index: {exc}") from exc
        self._state = "recording"

    def remux_fixture(self, fixture_path: str | Path) -> MkvProbeResult:
        if self._state == "new":
            self.start()
        if self._state != "recording":
            raise MkvWriterError(f"cannot remux in state {self._state!r}")
        fixture = Path(fixture_path).expanduser()
        if not fixture.is_file() or fixture.stat().st_size <= 0:
            self.mark_failure("input", f"fixture is missing or empty: {fixture}")
            raise MkvWriterError(self._failure_message or "fixture is unavailable")
        try:
            binary = _binary("ffmpeg", self.ffmpeg_binary)
        except MkvWriterError as exc:
            self.mark_failure("dependency", str(exc))
            raise
        temporary_path = self.output_path.with_name(
            f".{self.output_path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp.mkv"
        )
        command = [
            binary,
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-n",
            "-i",
            str(fixture),
            "-map",
            "0:v:0",
            "-an",
            "-c:v",
            "copy",
            "-f",
            "matroska",
            str(temporary_path),
        ]
        try:
            subprocess.run(
                command,
                check=True,
                capture_output=True,
                text=True,
                timeout=self.timeout_s,
            )
            temporary_result = probe_mkv(
                temporary_path,
                ffprobe_binary=self.ffprobe_binary,
                timeout_s=self.timeout_s,
                include_packets=True,
            )
            if "matroska" not in temporary_result.format_name.split(","):
                raise MkvWriterError("remuxed output is not Matroska")
            if temporary_result.codec_name != "h264":
                raise MkvWriterError(f"remuxed output codec is not H.264: {temporary_result.codec_name!r}")
            if temporary_result.frame_count < 1:
                raise MkvWriterError("remuxed MKV contains no video frames")
            if self._association is None:
                raise MkvWriterError("video association index is unavailable")
            for (video_frame_index,) in self._association.execute(
                "SELECT video_frame_index FROM video_mappings WHERE stored_in_video=1"
            ):
                if video_frame_index is None or video_frame_index >= temporary_result.frame_count:
                    raise MkvWriterError("existing video mapping is outside the probed MKV frame range")
            # os.link is atomic and no-clobber: a racing creator cannot be
            # overwritten by this adapter.
            try:
                os.link(temporary_path, self.output_path)
            except FileExistsError as exc:
                raise MkvWriterError(f"refusing to overwrite existing video: {self.output_path}") from exc
            self._result = replace(temporary_result, path=self.output_path)
        except (OSError, subprocess.SubprocessError, MkvWriterError) as exc:
            message = _safe_process_error(exc, context="MKV remux failed")
            self.mark_failure("remux", message)
            raise MkvWriterError(message) from exc
        finally:
            try:
                temporary_path.unlink(missing_ok=True)
            except OSError:
                pass
        self._state = "ready"
        assert self._result is not None
        return self._result

    def mark_mapping(self, mapping: VideoMapping) -> None:
        if self._state not in {"recording", "ready"}:
            raise MkvWriterError(f"cannot mark mapping in state {self._state!r}")
        resolver = self.source_record_resolver
        sink_owner = getattr(self.mapping_sink, "__self__", None)
        if resolver is None and sink_owner is not None:
            candidate = getattr(sink_owner, "resolve_source_record", None)
            if callable(candidate):
                resolver = candidate
        if resolver is None:
            raise MkvWriterError(
                "mark_mapping requires a SessionRecorder sink or source_record_resolver authority"
            )
        if mapping.camera_side is None:
            mapping = replace(mapping, camera_side=self.camera_side)
        try:
            validate_video_mapping(mapping)
        except Exception as exc:
            raise MkvWriterError(f"invalid video mapping: {exc}") from exc
        if self._association is None:
            raise MkvWriterError("video association index is unavailable")
        if self._association.execute(
            "SELECT 1 FROM video_mappings WHERE source_record_id=?",
            (mapping.source_record_id,),
        ).fetchone():
            raise MkvWriterError(f"duplicate video mapping: {mapping.source_record_id}")
        if mapping.camera_side != self.camera_side:
            raise MkvWriterError("video mapping camera_side does not match adapter")
        if resolver is not None:
            try:
                source = resolver(mapping.source_record_id)
            except Exception as exc:
                raise MkvWriterError(f"source record resolver failed: {exc}") from exc
            if source is None:
                raise MkvWriterError(f"unknown source_record_id: {mapping.source_record_id}")
            if isinstance(source, Mapping):
                source_side = source.get("camera_side")
                source_frame_id = source.get("frame_id")
            else:
                source_side = getattr(source, "camera_side", None)
                source_frame_id = getattr(source, "frame_id", None)
            if source_side != self.camera_side or source_side != mapping.camera_side:
                raise MkvWriterError("source record camera_side does not match adapter/mapping")
            if mapping.source_frame_id is not None and source_frame_id != mapping.source_frame_id:
                raise MkvWriterError("source frame_id does not match video mapping")
        if mapping.stored_in_video:
            assert mapping.video_frame_index is not None
            if self._association.execute(
                "SELECT 1 FROM video_mappings WHERE stored_in_video=1 AND video_frame_index=?",
                (mapping.video_frame_index,),
            ).fetchone():
                raise MkvWriterError(f"duplicate video frame index: {mapping.video_frame_index}")
            if self._result is not None and mapping.video_frame_index >= self._result.frame_count:
                raise MkvWriterError("video_frame_index is outside probed MKV frame range")
        self._association.execute("SAVEPOINT mapping_insert")
        try:
            self._association.execute(
                """INSERT INTO video_mappings(
                    source_record_id,camera_side,stored_in_video,video_mapping_status,
                    video_frame_index,video_packet_pts_ticks,video_packet_dts_ticks,
                    drop_reason,source_frame_id
                ) VALUES(?,?,?,?,?,?,?,?,?)""",
                (
                    mapping.source_record_id,
                    mapping.camera_side,
                    int(mapping.stored_in_video),
                    mapping.video_mapping_status,
                    mapping.video_frame_index,
                    mapping.video_packet_pts_ticks,
                    mapping.video_packet_dts_ticks,
                    mapping.drop_reason,
                    mapping.source_frame_id,
                ),
            )
            if self.mapping_sink is not None:
                self.mapping_sink(mapping)
            self._association.execute("RELEASE SAVEPOINT mapping_insert")
            self._association.commit()
        except Exception:
            self._association.execute("ROLLBACK TO SAVEPOINT mapping_insert")
            self._association.execute("RELEASE SAVEPOINT mapping_insert")
            raise
        if mapping.video_mapping_status == "writer_failure":
            self.mark_failure("mapping", mapping.drop_reason or "writer failure mapping")

    def mark_failure(self, stage: str, message: str) -> None:
        if self._state == "closed":
            raise MkvWriterError("cannot mark failure after close")
        self._failure_stage = str(stage)
        self._failure_message = str(message)[-4000:]
        self._state = "incomplete"

    def close(self) -> MkvProbeResult | None:
        if self._state == "closed":
            return self._result
        if self._state == "recording" and self._result is None:
            self.mark_failure("close", "writer closed before a fixture was remuxed")
        if self._association is not None:
            self._association.close()
            self._association = None
        if self._association_path is not None:
            try:
                self._association_path.unlink(missing_ok=True)
            except OSError as exc:
                self._failure_stage = self._failure_stage or "association_index_cleanup"
                self._failure_message = self._failure_message or str(exc)[-4000:]
                self._state = "incomplete"
            self._association_path = None
        self._state = "closed"
        return self._result
