"""Phase B1 camera-session adapter for the Phase A recorder.

The live GStreamer workers publish structured, ordered camera events.  This
module keeps all synchronous Phase A recorder and media-probe calls on one
dedicated thread while exposing Qt signals suitable for the GUI lifecycle.
It deliberately owns no persistence schema and relies on Phase A for pending
video mappings and canonical artifact creation.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import threading
from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from PyQt5 import QtCore

from .session_recording_parquet import (
    SessionCloseResult,
    SessionRecorder,
    SessionRecordingConfig,
    SourceRecord,
    TimeAlignmentRecord,
    VideoArtifact,
    VideoMapping,
)
from .session_video import MkvProbeResult, probe_mkv


_CAMERA_SIDES = ("left", "right")


@dataclass(frozen=True)
class B1VideoProbe:
    """Read-only B1 codec evidence not exposed by the Phase A probe."""

    codec_name: str
    has_b_frames: int
    decoded_frame_count: int | None
    packet_count: int | None


def _integer_or_none(value: object) -> int | None:
    if value in (None, "", "N/A"):
        return None
    if isinstance(value, bool):
        raise ValueError("boolean is not a media count")
    return int(value)


def probe_b1_video(
    path: str | Path,
    *,
    ffprobe_binary: str | Path | None = None,
    timeout_s: float = 30.0,
) -> B1VideoProbe:
    """Return B-frame and count evidence without changing the Phase A API."""

    video_path = Path(path).expanduser()
    binary = str(ffprobe_binary) if ffprobe_binary is not None else shutil.which("ffprobe")
    if not binary:
        raise RuntimeError("required executable is unavailable: ffprobe")
    command = [
        binary,
        "-v",
        "error",
        "-count_frames",
        "-count_packets",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=codec_name,has_b_frames,nb_read_frames,nb_read_packets",
        "-of",
        "json",
        str(video_path),
    ]
    try:
        completed = subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError(f"B1 ffprobe failed for {video_path.name}: {exc}") from exc
    try:
        payload = json.loads(completed.stdout)
        stream = (payload.get("streams") or [])[0]
    except (IndexError, KeyError, TypeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"B1 ffprobe returned invalid stream data for {video_path}") from exc
    if "has_b_frames" not in stream:
        raise RuntimeError(f"B1 ffprobe omitted has_b_frames for {video_path}")
    try:
        if isinstance(stream["has_b_frames"], bool):
            raise ValueError("boolean is not has_b_frames")
        has_b_frames = int(stream["has_b_frames"])
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"B1 ffprobe returned invalid has_b_frames for {video_path}") from exc
    if has_b_frames < 0:
        raise RuntimeError(f"B1 ffprobe returned negative has_b_frames for {video_path}")
    try:
        decoded_frame_count = _integer_or_none(stream.get("nb_read_frames"))
        packet_count = _integer_or_none(stream.get("nb_read_packets"))
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"B1 ffprobe returned invalid media counts for {video_path}") from exc
    if decoded_frame_count is None or decoded_frame_count < 0:
        raise RuntimeError(f"B1 ffprobe did not return a nonnegative decoded frame count for {video_path}")
    if packet_count is None or packet_count < 0:
        raise RuntimeError(f"B1 ffprobe did not return a nonnegative packet count for {video_path}")
    return B1VideoProbe(
        codec_name=str(stream.get("codec_name") or ""),
        has_b_frames=has_b_frames,
        decoded_frame_count=decoded_frame_count,
        packet_count=packet_count,
    )


@dataclass
class _CameraState:
    expected_event_sequence: int = 0
    last_processed_event_sequence: int = -1
    last_accepted_event_sequence: int = -1
    accepted_event_count: int = 0
    processed_event_count: int = 0
    terminal: dict[str, Any] | None = None
    terminal_barrier_sequence: int | None = None
    terminal_barrier_event_count: int | None = None
    source_count: int = 0
    alignment_count: int = 0
    mapping_count: int = 0
    source_ids: set[str] = field(default_factory=set)
    source_frame_ids: list[int] = field(default_factory=list)
    video_frame_indices: list[int] = field(default_factory=list)
    expected_width_px: int | None = None
    expected_height_px: int | None = None
    last_pts_ticks: int | None = None
    last_dts_ticks: int | None = None


class SessionCameraRecordingController(QtCore.QObject):
    """Serialize structured camera events into one Phase A session recorder."""

    opened = QtCore.pyqtSignal(object)
    state_changed = QtCore.pyqtSignal(str, object)
    session_terminal = QtCore.pyqtSignal(object)

    def __init__(
        self,
        parent: QtCore.QObject | None = None,
        *,
        data_capacity: int = 1024,
        put_timeout_s: float = 1.0,
        recorder_factory: Callable[[SessionRecordingConfig], SessionRecorder] = SessionRecorder.open,
        mkv_probe: Callable[..., MkvProbeResult] = probe_mkv,
        codec_probe: Callable[[str | Path], B1VideoProbe] = probe_b1_video,
    ) -> None:
        super().__init__(parent)
        if data_capacity <= 0:
            raise ValueError("data_capacity must be positive")
        if put_timeout_s <= 0:
            raise ValueError("put_timeout_s must be positive")
        self.data_capacity = int(data_capacity)
        self.put_timeout_s = float(put_timeout_s)
        self._recorder_factory = recorder_factory
        self._mkv_probe = mkv_probe
        self._codec_probe = codec_probe

        self._condition = threading.Condition()
        self._data_events: deque[dict[str, Any]] = deque()
        self._control_events: deque[tuple[str, Any]] = deque()
        self._thread = threading.Thread(
            target=self._run,
            name="stereo-session-camera-recorder",
            daemon=True,
        )
        self._thread_started = False
        self._shutdown_requested = False
        self._accepting_data = False
        self._terminal_submitted: set[str] = set()
        self._public_state = "idle"
        self._session_id: str | None = None
        self._session_epoch = 0
        self._finalize_timeout_epoch: int | None = None
        self._finalize_timeout_reason: str | None = None
        self._close_started_epoch: int | None = None

        self._recorder: SessionRecorder | None = None
        self._config: SessionRecordingConfig | None = None
        self._camera = {side: _CameraState() for side in _CAMERA_SIDES}
        self._failures: list[dict[str, Any]] = []
        self._failure_keys: set[tuple[str, str | None]] = set()
        self._mark_failure_count = 0
        self._mark_failure_attempted = False
        self._logical_close_count = 0
        self._close_completed = False
        self._close_result: SessionCloseResult | None = None
        self._finalizing = False

    @property
    def state(self) -> str:
        with self._condition:
            return self._public_state

    def open_session(self, config: SessionRecordingConfig) -> bool:
        """Queue a Phase A open without performing filesystem I/O in the caller."""

        if not isinstance(config, SessionRecordingConfig):
            raise TypeError("config must be SessionRecordingConfig")
        with self._condition:
            if self._shutdown_requested or self._public_state not in {
                "idle",
                "complete",
                "incomplete",
                "failed",
            }:
                return False
            self._public_state = "starting"
            self._session_id = config.session_id
            self._session_epoch += 1
            self._finalize_timeout_epoch = None
            self._finalize_timeout_reason = None
            self._close_started_epoch = None
            self._accepting_data = True
            self._terminal_submitted.clear()
            self._control_events.append(("open", config))
            if not self._thread_started:
                self._thread_started = True
                self._thread.start()
            self._condition.notify_all()
        self.state_changed.emit("starting", {"session_id": config.session_id})
        return True

    def submit_recording_event(self, event: Mapping[str, Any]) -> bool:
        """Put an event into bounded ingress without blocking the GUI thread.

        The camera process owns the required blocking 1024-event/1-second IPC
        put.  This GUI-side handoff is deliberately immediate; exhausting it
        emits a stopping state synchronously and then records the failure on
        the recorder thread.
        """

        payload = dict(event)
        if payload.get("event_kind", payload.get("type")) == "camera_failure":
            with self._condition:
                side = str(payload.get("camera_side") or "")
                if (
                    self._shutdown_requested
                    or self._public_state not in {"starting", "recording", "stopping"}
                    or payload.get("session_id") != self._session_id
                    or side not in _CAMERA_SIDES
                ):
                    return False
                self._control_events.append(("camera_failure", payload))
                self._condition.notify_all()
            self.state_changed.emit("stopping", dict(payload))
            return True
        backpressure_detail: dict[str, Any] | None = None
        with self._condition:
            side = str(payload.get("camera_side") or "")
            if (
                self._shutdown_requested
                or not self._accepting_data
                or payload.get("session_id") != self._session_id
                or side not in _CAMERA_SIDES
                or side in self._terminal_submitted
            ):
                return False
            if len(self._data_events) >= self.data_capacity:
                self._accepting_data = False
                backpressure_detail = {
                    "camera_side": side,
                    "failure_stage": "recording_ipc_backpressure",
                    "message": "recorder-thread ingress capacity was exhausted",
                }
                self._control_events.append(("backpressure", backpressure_detail))
                self._condition.notify_all()
            if not self._accepting_data:
                accepted = False
            else:
                self._data_events.append(payload)
                sequence = payload.get("record_event_sequence")
                if isinstance(sequence, int) and not isinstance(sequence, bool):
                    self._camera[side].last_accepted_event_sequence = max(
                        self._camera[side].last_accepted_event_sequence,
                        sequence,
                    )
                self._camera[side].accepted_event_count += 1
                self._condition.notify_all()
                accepted = True
        if backpressure_detail is not None:
            self.state_changed.emit("stopping", backpressure_detail)
        return accepted

    def submit_camera_terminal(self, terminal: Mapping[str, Any]) -> bool:
        """Queue terminal control independently of data-channel saturation."""

        payload = dict(terminal)
        side = str(payload.get("camera_side") or "")
        with self._condition:
            if self._shutdown_requested or side not in _CAMERA_SIDES or side in self._terminal_submitted:
                return False
            if self._session_id is not None and payload.get("session_id") != self._session_id:
                return False
            self._terminal_submitted.add(side)
            self._control_events.append(("terminal", payload))
            self._condition.notify_all()
            return True

    def finalize_timeout(self, reason: str) -> bool:
        """Fail the session and synthesize terminal barriers for missing cameras."""

        message = str(reason).strip()
        if not message:
            raise ValueError("finalize timeout reason must be non-empty")
        with self._condition:
            if (
                self._shutdown_requested
                or self._public_state not in {"starting", "recording", "stopping"}
                or self._close_started_epoch == self._session_epoch
            ):
                return False
            self._accepting_data = False
            self._finalize_timeout_epoch = self._session_epoch
            self._finalize_timeout_reason = message
            self._control_events.append(
                ("timeout", {"epoch": self._session_epoch, "reason": message})
            )
            self._condition.notify_all()
            return True

    def shutdown(self, timeout_s: float = 5.0) -> bool:
        """Stop the adapter thread after any active session has been finalized."""

        with self._condition:
            if not self._shutdown_requested:
                self._shutdown_requested = True
                self._accepting_data = False
                self._control_events.append(("shutdown", None))
                self._condition.notify_all()
        if not self._thread_started or threading.current_thread() is self._thread:
            return True
        self._thread.join(timeout=max(0.0, float(timeout_s)))
        return not self._thread.is_alive()

    def _run(self) -> None:
        stop_thread = False
        while not stop_thread:
            with self._condition:
                while not self._control_events and not self._data_events:
                    if self._shutdown_requested and self._recorder is None:
                        return
                    self._condition.wait()
                if self._control_events:
                    kind, payload = self._control_events.popleft()
                    event = None
                else:
                    kind, payload = "data", None
                    event = self._data_events.popleft()
                    self._condition.notify_all()
            try:
                if kind == "open":
                    self._handle_open(payload)
                elif kind == "data" and event is not None:
                    self._handle_data(event)
                elif kind == "terminal":
                    self._handle_terminal(payload)
                elif kind == "backpressure":
                    self._add_failure(payload["failure_stage"], payload["message"], payload.get("camera_side"))
                    self._emit_state("stopping", payload)
                elif kind == "camera_failure":
                    self._handle_camera_failure(payload)
                elif kind == "timeout":
                    self._handle_timeout(payload)
                elif kind == "shutdown":
                    if self._recorder is not None and not self._finalizing:
                        self._handle_timeout(
                            {
                                "epoch": self._session_epoch,
                                "reason": "controller shutdown before camera finalization",
                            }
                        )
                    stop_thread = self._recorder is None
            except Exception as exc:  # keep the worker alive long enough to close evidence
                self._add_failure("session_camera_adapter", str(exc))
                self._emit_state("stopping", {"failure_stage": "session_camera_adapter", "message": str(exc)})
            self._maybe_finalize()
            if self._shutdown_requested and self._recorder is None:
                stop_thread = True

    def _reset_session_state(self, config: SessionRecordingConfig) -> None:
        self._config = config
        self._camera = {side: _CameraState() for side in _CAMERA_SIDES}
        self._failures.clear()
        self._failure_keys.clear()
        self._mark_failure_count = 0
        self._mark_failure_attempted = False
        self._logical_close_count = 0
        self._close_completed = False
        self._close_result = None
        self._finalizing = False

    def _handle_open(self, config: SessionRecordingConfig) -> None:
        self._reset_session_state(config)
        try:
            recorder = self._recorder_factory(config)
        except Exception as exc:
            with self._condition:
                self._accepting_data = False
                self._data_events.clear()
                self._condition.notify_all()
            self._add_failure("session_open", str(exc))
            self._emit_state("failed", {"session_id": config.session_id, "message": str(exc)})
            self.session_terminal.emit(
                {
                    "session_id": config.session_id,
                    "session_state": "failed",
                    "clean_shutdown": False,
                    "counts": {},
                    "camera": {},
                    "failures": list(self._failures),
                    "logical_close_count": 0,
                    "paths": None,
                }
            )
            return
        self._recorder = recorder
        paths = {
            "session_dir": recorder.paths.session_dir,
            "left_video": recorder.paths.left_video,
            "right_video": recorder.paths.right_video,
        }
        self.opened.emit({"session_id": config.session_id, "paths": paths})
        self._emit_state("recording", {"session_id": config.session_id, "paths": paths})

    def _handle_data(self, event: dict[str, Any]) -> None:
        if self._recorder is None or self._config is None:
            self._add_failure("recording_event_before_open", "camera event arrived before SessionRecorder.open")
            return
        side = str(event.get("camera_side") or "")
        if side not in _CAMERA_SIDES:
            self._add_failure("recording_event_invalid", f"unknown camera_side: {side!r}")
            return
        camera = self._camera[side]
        camera.processed_event_count += 1
        sequence = event.get("record_event_sequence")
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
            self._add_failure("recording_event_sequence_gap", f"invalid event sequence: {sequence!r}", side)
            return
        if event.get("session_id") != self._config.session_id:
            self._add_failure("recording_event_session_mismatch", "event session_id does not match recorder", side)
        if sequence != camera.expected_event_sequence:
            self._add_failure(
                "recording_event_sequence_gap",
                f"expected {camera.expected_event_sequence}, received {sequence}",
                side,
            )
        camera.expected_event_sequence = max(camera.expected_event_sequence, sequence + 1)
        camera.last_processed_event_sequence = max(camera.last_processed_event_sequence, sequence)

        event_kind = event.get("event_kind", event.get("type"))
        try:
            if event_kind == "camera_source":
                self._record_source(event, camera)
            elif event_kind == "video_mapping":
                self._record_mapping(event, camera)
            else:
                self._add_failure("recording_event_invalid", f"unknown event_kind: {event_kind!r}", side)
        except Exception as exc:
            self._add_failure(f"{event_kind or 'unknown'}_record", str(exc), side)

    def _record_source(self, event: Mapping[str, Any], camera: _CameraState) -> None:
        assert self._recorder is not None and self._config is not None
        side = str(event["camera_side"])
        source_id = str(event["source_record_id"])
        frame_id = int(event["frame_id"])
        camera_sequence = int(event["camera_sequence"])
        width_px = int(event["width_px"])
        height_px = int(event["height_px"])
        pts_ticks = event.get("pts_ticks")
        dts_ticks = event.get("dts_ticks")

        if source_id in camera.source_ids:
            self._add_failure("duplicate_source_id", source_id, side)
        if frame_id != camera.source_count or camera_sequence != camera.source_count:
            self._add_failure(
                "source_sequence_gap",
                f"expected frame/camera sequence {camera.source_count}, got {frame_id}/{camera_sequence}",
                side,
            )
        if width_px <= 0 or height_px <= 0:
            self._add_failure("camera_caps_invalid", f"invalid caps {width_px}x{height_px}", side)
        elif camera.expected_width_px is None:
            camera.expected_width_px = width_px
            camera.expected_height_px = height_px
        elif (width_px, height_px) != (camera.expected_width_px, camera.expected_height_px):
            self._add_failure(
                "camera_caps_changed",
                f"expected {camera.expected_width_px}x{camera.expected_height_px}, observed {width_px}x{height_px}",
                side,
            )
        if pts_ticks is None:
            self._add_failure("frame_reordering_detected", "countable AU has no PTS", side)
        elif camera.last_pts_ticks is not None and int(pts_ticks) <= camera.last_pts_ticks:
            self._add_failure("frame_reordering_detected", "PTS must be strictly increasing", side)
        if dts_ticks is not None and camera.last_dts_ticks is not None and int(dts_ticks) < camera.last_dts_ticks:
            self._add_failure("frame_reordering_detected", "DTS must be nondecreasing", side)
        if pts_ticks is not None:
            camera.last_pts_ticks = int(pts_ticks)
        if dts_ticks is not None:
            camera.last_dts_ticks = int(dts_ticks)

        source = SourceRecord(
            source_record_id=source_id,
            session_id=str(event["session_id"]),
            camera_side=side,
            camera_stream_id=str(event["camera_stream_id"]),
            frame_id=frame_id,
            camera_sequence=camera_sequence,
            pc_receive_monotonic_ns=int(event["pc_receive_monotonic_ns"]),
            pc_receive_utc_ns=_optional_int(event.get("pc_receive_utc_ns")),
            rtp_timestamp=_optional_int(event.get("rtp_timestamp")),
            rtp_ssrc=_optional_int(event.get("rtp_ssrc")),
            rtp_marker_seq=_optional_int(event.get("rtp_marker_seq")),
            rtcp_sr_ntp_raw=_optional_int(event.get("rtcp_sr_ntp_raw")),
            rtcp_frame_ntp_unix_ns=_optional_int(event.get("rtcp_frame_ntp_unix_ns")),
            pts_ticks=_optional_int(pts_ticks),
            dts_ticks=_optional_int(dts_ticks),
            time_base_num=_optional_int(event.get("time_base_num")),
            time_base_den=_optional_int(event.get("time_base_den")),
            width_px=width_px,
            height_px=height_px,
            stored_in_video=False,
            video_mapping_status="pending",
        )
        self._recorder.record_camera_source(source)
        alignment = TimeAlignmentRecord(
            alignment_id=f"alignment:{source_id}",
            record_type="camera_source",
            record_id=source_id,
            camera_side=side,
            alignment_reference_domain="pc_receive_monotonic",
            alignment_status="receive_time_fallback",
            aligned_pc_monotonic_ns=source.pc_receive_monotonic_ns,
            pc_receive_monotonic_ns=source.pc_receive_monotonic_ns,
            pc_receive_utc_ns=source.pc_receive_utc_ns,
            camera_rtp_timestamp=source.rtp_timestamp,
            camera_pts_ticks=source.pts_ticks,
            camera_dts_ticks=source.dts_ticks,
            camera_rtcp_ntp_unix_ns=source.rtcp_frame_ntp_unix_ns,
            camera_rtcp_to_pc_utc_delta_ns=None,
            px4_utc_to_pc_utc_delta_ns=None,
        )
        self._recorder.record_time_alignment(alignment)
        camera.source_ids.add(source_id)
        camera.source_frame_ids.append(frame_id)
        camera.source_count += 1
        camera.alignment_count += 1

    def _record_mapping(self, event: Mapping[str, Any], camera: _CameraState) -> None:
        assert self._recorder is not None
        side = str(event["camera_side"])
        frame_index = _optional_int(event.get("video_frame_index"))
        source_frame_id = _optional_int(event.get("source_frame_id", event.get("frame_id")))
        if frame_index != camera.mapping_count:
            self._add_failure(
                "video_index_gap",
                f"expected video index {camera.mapping_count}, got {frame_index}",
                side,
            )
        mapping = VideoMapping(
            source_record_id=str(event["source_record_id"]),
            camera_side=side,
            source_frame_id=source_frame_id,
            stored_in_video=bool(event.get("stored_in_video", True)),
            video_mapping_status=str(event.get("video_mapping_status", "stored")),
            video_frame_index=frame_index,
            video_packet_pts_ticks=_optional_int(event.get("video_packet_pts_ticks")),
            video_packet_dts_ticks=_optional_int(event.get("video_packet_dts_ticks")),
            drop_reason=event.get("drop_reason"),
        )
        self._recorder.mark_video_mapping(mapping)
        if frame_index is not None:
            camera.video_frame_indices.append(frame_index)
        camera.mapping_count += 1

    def _handle_terminal(self, terminal: dict[str, Any]) -> None:
        side = str(terminal["camera_side"])
        camera = self._camera[side]
        accepted = camera.last_accepted_event_sequence
        emitted = int(terminal.get("last_emitted_event_sequence", -1))
        camera.terminal = terminal
        camera.terminal_barrier_sequence = accepted
        camera.terminal_barrier_event_count = camera.accepted_event_count
        terminal_status = str(terminal.get("terminal_status") or "")
        if terminal_status != "complete":
            self._add_failure(
                "camera_terminal_incomplete",
                f"terminal_status={terminal_status or 'missing'}",
                side,
            )
        failure_stages = tuple(terminal.get("failure_stages") or ())
        failure_details = tuple(terminal.get("failure_details") or ())
        for index, failure_stage in enumerate(failure_stages):
            detail = failure_details[index] if index < len(failure_details) else terminal.get("pipeline_error")
            self._add_failure(str(failure_stage), str(detail or failure_stage), side)
        if int(terminal.get("rtp_association_mismatches", 0) or 0) > 0:
            self._add_failure(
                "rtp_au_association_failed",
                f"mismatches={terminal.get('rtp_association_mismatches')}",
                side,
            )
        if emitted != accepted:
            self._add_failure(
                "recording_event_sequence_gap",
                f"terminal emitted sequence {emitted} does not match accepted sequence {accepted}",
                side,
            )
        for flag, stage in (
            (not bool(terminal.get("eos_ok", False)), "camera_eos_failed"),
            (bool(terminal.get("ipc_backpressure", False)), "recording_ipc_backpressure"),
            (bool(terminal.get("caps_changed", False)), "camera_caps_changed"),
            (bool(terminal.get("pts_reordering_observed", False)), "frame_reordering_detected"),
        ):
            if flag:
                self._add_failure(stage, str(terminal.get("pipeline_error") or terminal.get("terminal_status") or stage), side)
        if terminal.get("pipeline_error"):
            self._add_failure("camera_pipeline_error", str(terminal["pipeline_error"]), side)
        if int(terminal.get("authority_fifo_residual", 0) or 0) != 0:
            self._add_failure("authority_fifo_residual", str(terminal.get("authority_fifo_residual")), side)
        self._emit_state("stopping", {"camera_side": side, "terminal_status": terminal.get("terminal_status")})

    def _handle_camera_failure(self, failure: Mapping[str, Any]) -> None:
        side = str(failure.get("camera_side") or "")
        self._add_failure(
            str(failure.get("failure_stage") or "camera_failure"),
            str(failure.get("detail") or failure.get("message") or "camera recording failure"),
            side if side in _CAMERA_SIDES else None,
        )
        self._emit_state("stopping", dict(failure))

    def _handle_timeout(self, payload: Mapping[str, Any]) -> None:
        epoch = int(payload.get("epoch", -1))
        reason = str(payload.get("reason") or "camera finalize timeout")
        with self._condition:
            if epoch != self._session_epoch or self._recorder is None or self._config is None:
                return
        self._add_failure("camera_finalize_timeout", reason)
        with self._condition:
            self._accepting_data = False
            self._terminal_submitted.update(_CAMERA_SIDES)
        for side in _CAMERA_SIDES:
            camera = self._camera[side]
            if camera.terminal is None:
                camera.terminal = {
                    "session_id": self._session_id,
                    "camera_side": side,
                    "terminal_status": "timeout",
                    "last_emitted_event_sequence": camera.last_accepted_event_sequence,
                    "last_source_sequence": camera.source_count - 1,
                    "last_video_frame_index": camera.mapping_count - 1,
                    "source_count": camera.source_count,
                    "mapping_count": camera.mapping_count,
                    "output_path": str(self._recorder.paths.video_path(side)) if self._recorder else "",
                    "eos_ok": False,
                    "pipeline_error": reason,
                    "ipc_backpressure": False,
                    "caps_changed": False,
                    "pts_reordering_observed": False,
                    "authority_fifo_residual": 0,
                }
                camera.terminal_barrier_sequence = camera.last_accepted_event_sequence
                camera.terminal_barrier_event_count = camera.accepted_event_count
        self._emit_state("stopping", {"failure_stage": "camera_finalize_timeout", "message": reason})

    def _maybe_finalize(self) -> None:
        if self._recorder is None or self._finalizing:
            return
        for camera in self._camera.values():
            if (
                camera.terminal is None
                or camera.terminal_barrier_sequence is None
                or camera.terminal_barrier_event_count is None
            ):
                return
            if camera.last_processed_event_sequence != camera.terminal_barrier_sequence:
                return
            if camera.processed_event_count != camera.terminal_barrier_event_count:
                return
        with self._condition:
            if self._data_events:
                return
        try:
            self._finalize_session()
        except Exception as exc:
            self._finalize_after_unexpected_exception(exc)

    def _finalize_session(self) -> None:
        assert self._recorder is not None and self._config is not None
        self._finalizing = True
        recorder = self._recorder
        camera_results: dict[str, dict[str, Any]] = {}
        for side in _CAMERA_SIDES:
            camera = self._camera[side]
            terminal = camera.terminal or {}
            if int(terminal.get("source_count", camera.source_count)) != camera.source_count:
                self._add_failure("camera_source_count_mismatch", "worker/controller source counts differ", side)
            if int(terminal.get("mapping_count", camera.mapping_count)) != camera.mapping_count:
                self._add_failure("camera_mapping_count_mismatch", "worker/controller mapping counts differ", side)
            if int(terminal.get("last_source_sequence", camera.source_count - 1)) != camera.source_count - 1:
                self._add_failure("camera_source_count_mismatch", "terminal last source sequence is inconsistent", side)
            if int(terminal.get("last_video_frame_index", camera.mapping_count - 1)) != camera.mapping_count - 1:
                self._add_failure("camera_mapping_count_mismatch", "terminal last video index is inconsistent", side)
            output_path = Path(str(terminal.get("output_path") or recorder.paths.video_path(side))).expanduser()
            expected_path = recorder.paths.video_path(side)
            if output_path.resolve(strict=False) != expected_path.resolve(strict=False):
                self._add_failure("camera_output_path_mismatch", f"expected {expected_path}, got {output_path}", side)
                output_path = expected_path

            phase_a_probe: MkvProbeResult | None = None
            codec_probe: B1VideoProbe | None = None
            try:
                phase_a_probe = self._mkv_probe(output_path, include_packets=True)
                codec_probe = self._codec_probe(output_path)
                if codec_probe.has_b_frames > 0:
                    self._add_failure("b_frame_not_supported", f"has_b_frames={codec_probe.has_b_frames}", side)
                if codec_probe.codec_name != "h264" or phase_a_probe.codec_name != "h264":
                    self._add_failure("camera_codec_invalid", f"codec={codec_probe.codec_name!r}", side)
                if codec_probe.decoded_frame_count is None or codec_probe.decoded_frame_count < 0:
                    self._add_failure("camera_probe_count_missing", "B1 decoded frame count is unavailable", side)
                if codec_probe.packet_count is None or codec_probe.packet_count < 0:
                    self._add_failure("camera_probe_count_missing", "B1 packet count is unavailable", side)
                if phase_a_probe.decoded_frame_count is None or phase_a_probe.decoded_frame_count < 0:
                    self._add_failure("camera_probe_count_missing", "Phase A decoded frame count is unavailable", side)
                if phase_a_probe.packet_count is None or phase_a_probe.packet_count < 0:
                    self._add_failure("camera_probe_count_missing", "Phase A packet count is unavailable", side)
                if codec_probe.decoded_frame_count != phase_a_probe.decoded_frame_count:
                    self._add_failure("camera_probe_count_mismatch", "B1 and Phase A decoded counts differ", side)
                if codec_probe.packet_count != phase_a_probe.packet_count:
                    self._add_failure("camera_probe_packet_count_mismatch", "B1 and Phase A packet counts differ", side)
            except Exception as exc:
                self._add_failure("camera_mkv_probe", str(exc), side)

            decoded_count = phase_a_probe.decoded_frame_count if phase_a_probe is not None else 0
            if camera.source_frame_ids != list(range(camera.source_count)):
                self._add_failure("source_sequence_gap", "source frame IDs are not contiguous from zero", side)
            if camera.video_frame_indices != list(range(camera.mapping_count)):
                self._add_failure("video_index_gap", "video frame indices are not contiguous from zero", side)
            if not (camera.source_count == camera.alignment_count == camera.mapping_count == decoded_count):
                self._add_failure(
                    "camera_count_mismatch",
                    f"source={camera.source_count}, alignment={camera.alignment_count}, mapping={camera.mapping_count}, decoded={decoded_count}",
                    side,
                )
            if camera.source_count < 1:
                self._add_failure("camera_empty_recording", "camera session contains no countable access unit", side)

            side_failed = any(item.get("camera_side") in (None, side) for item in self._failures)
            writer_status = "incomplete" if side_failed else "complete"
            artifact = VideoArtifact(
                camera_side=side,
                frame_count=int(decoded_count or 0),
                writer_status=writer_status,
                mapping_missing_count=0 if writer_status == "complete" else abs(camera.source_count - camera.mapping_count),
                packet_count=phase_a_probe.packet_count if phase_a_probe is not None else None,
            )
            try:
                recorder.register_video_artifact(artifact, probe=phase_a_probe)
            except Exception as exc:
                self._add_failure("video_artifact_registration", str(exc), side)
            camera_results[side] = {
                "terminal": dict(terminal),
                "last_emitted_event_sequence": terminal.get("last_emitted_event_sequence", -1),
                "last_processed_event_sequence": camera.last_processed_event_sequence,
                "accepted_event_count": camera.accepted_event_count,
                "processed_event_count": camera.processed_event_count,
                "source": camera.source_count,
                "alignment": camera.alignment_count,
                "mapping": camera.mapping_count,
                "decoded_mkv": decoded_count,
                "packet_count": phase_a_probe.packet_count if phase_a_probe is not None else None,
                "has_b_frames": codec_probe.has_b_frames if codec_probe is not None else None,
                "codec_name": codec_probe.codec_name if codec_probe is not None else None,
            }

        timeout_reason = self._begin_close_phase()
        if timeout_reason is not None:
            self._add_failure("camera_finalize_timeout", timeout_reason)
        if self._failures and recorder.state == "recording":
            first = self._failures[0]
            self._mark_failure_attempted = True
            recorder.mark_failure(str(first["stage"]), str(first["message"]))
            self._mark_failure_count = 1

        close_result, close_error = self._close_phase_a_recorder(recorder)
        if close_error is not None:
            self._add_failure("session_close", str(close_error))

        terminal_state = close_result.session_state if close_result is not None else "failed"
        clean_shutdown = bool(close_result.clean_shutdown) if close_result is not None else False
        if close_result is not None and terminal_state != "complete" and not self._failures:
            self._add_failure(
                "session_close_incomplete",
                f"Phase A close returned session_state={terminal_state}",
            )
        counts = dict(close_result.counts) if close_result is not None else {
            "source_records": sum(camera.source_count for camera in self._camera.values()),
            "time_alignment": sum(camera.alignment_count for camera in self._camera.values()),
            "pixhawk_samples": 0,
            "triangulation": 0,
        }
        if self._failures and terminal_state == "complete":
            terminal_state = "incomplete"
            clean_shutdown = False
        result = {
            "session_id": self._config.session_id,
            "session_state": terminal_state,
            "clean_shutdown": clean_shutdown,
            "counts": counts,
            "camera": camera_results,
            "failures": list(self._failures),
            "mark_failure_count": self._mark_failure_count,
            "logical_close_count": self._logical_close_count,
            "paths": asdict(close_result.paths) if close_result is not None else None,
        }
        self._publish_terminal(terminal_state, result)

    def _close_phase_a_recorder(
        self,
        recorder: SessionRecorder,
    ) -> tuple[SessionCloseResult | None, Exception | None]:
        """Perform one logical close, allowing only Phase A metadata retry."""

        if self._close_completed:
            return self._close_result, None
        self._begin_close_phase()
        self._logical_close_count = 1
        close_error: Exception | None = None
        for _attempt in range(2):
            try:
                self._close_result = recorder.close()
                self._close_completed = True
                return self._close_result, None
            except Exception as exc:
                close_error = exc
                try:
                    retry_metadata = recorder.state == "metadata_finalize_failed"
                except Exception:
                    retry_metadata = False
                if not retry_metadata:
                    break
        return None, close_error

    def _begin_close_phase(self) -> str | None:
        """Atomically freeze the timeout decision before persisted close."""

        with self._condition:
            if self._close_started_epoch != self._session_epoch:
                self._close_started_epoch = self._session_epoch
            if self._finalize_timeout_epoch != self._session_epoch:
                return None
            return self._finalize_timeout_reason

    def _finalize_after_unexpected_exception(self, exc: Exception) -> None:
        """Best-effort incomplete close and terminal delivery for any finalizer bug."""

        recorder = self._recorder
        config = self._config
        self._add_failure("session_finalize_exception", str(exc))
        close_result = self._close_result
        if recorder is not None:
            if not self._mark_failure_attempted:
                try:
                    if recorder.state == "recording":
                        self._mark_failure_attempted = True
                        recorder.mark_failure("session_finalize_exception", str(exc))
                        self._mark_failure_count = 1
                except Exception as mark_exc:
                    self._add_failure("session_finalize_mark_failure", str(mark_exc))
            if not self._close_completed:
                close_result, close_error = self._close_phase_a_recorder(recorder)
                if close_error is not None:
                    self._add_failure("session_close", str(close_error))

        counts = dict(close_result.counts) if close_result is not None else {
            "source_records": sum(camera.source_count for camera in self._camera.values()),
            "time_alignment": sum(camera.alignment_count for camera in self._camera.values()),
            "pixhawk_samples": 0,
            "triangulation": 0,
        }
        camera_results = {
            side: {
                "terminal": dict(camera.terminal or {}),
                "last_emitted_event_sequence": (camera.terminal or {}).get("last_emitted_event_sequence", -1),
                "last_processed_event_sequence": camera.last_processed_event_sequence,
                "accepted_event_count": camera.accepted_event_count,
                "processed_event_count": camera.processed_event_count,
                "source": camera.source_count,
                "alignment": camera.alignment_count,
                "mapping": camera.mapping_count,
                "decoded_mkv": None,
                "packet_count": None,
                "has_b_frames": None,
                "codec_name": None,
            }
            for side, camera in self._camera.items()
        }
        try:
            result_paths = asdict(close_result.paths) if close_result is not None else None
        except Exception:
            result_paths = None
        result = {
            "session_id": config.session_id if config is not None else self._session_id,
            "session_state": "incomplete",
            "clean_shutdown": False,
            "counts": counts,
            "camera": camera_results,
            "failures": list(self._failures),
            "mark_failure_count": self._mark_failure_count,
            "logical_close_count": self._logical_close_count,
            "paths": result_paths,
        }
        self._publish_terminal("incomplete", result)

    def _publish_terminal(self, terminal_state: str, result: Mapping[str, Any]) -> None:
        payload = dict(result)
        with self._condition:
            timeout_reason = (
                self._finalize_timeout_reason
                if self._finalize_timeout_epoch == self._session_epoch
                else None
            )
            if timeout_reason is not None:
                self._add_failure("camera_finalize_timeout", timeout_reason)
                payload["failures"] = list(self._failures)
                if terminal_state == "complete":
                    terminal_state = "incomplete"
                    payload["session_state"] = "incomplete"
                    payload["clean_shutdown"] = False
            self._accepting_data = False
            self._public_state = terminal_state
            self._data_events.clear()
            self._recorder = None
            self._config = None
            self._finalizing = False
            self._condition.notify_all()
        self.state_changed.emit(terminal_state, payload)
        self.session_terminal.emit(payload)

    def _add_failure(self, stage: str, message: str, camera_side: str | None = None) -> None:
        key = (str(stage), camera_side)
        if key in self._failure_keys:
            return
        self._failure_keys.add(key)
        self._failures.append(
            {
                "stage": str(stage),
                "message": str(message)[-4000:],
                "camera_side": camera_side,
            }
        )

    def _emit_state(self, state: str, detail: Mapping[str, Any]) -> None:
        with self._condition:
            self._public_state = state
        self.state_changed.emit(state, dict(detail))


def _optional_int(value: object) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise TypeError("boolean is not an integer timestamp")
    return int(value)
