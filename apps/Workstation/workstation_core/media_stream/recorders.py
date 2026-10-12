"""recorders owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import queue
import subprocess
import threading
import time
from pathlib import Path
from typing import Any
from .configuration import (
    _record_video_fps,
    _record_video_path,
    _record_video_queue_size,
    normalize_stream_backend,
    recording_integrity_contract,
)
from .events import _emit_recording_finalized, _emit_status
from .policy import GST_RECORDING_EOS_TIMEOUT_S, STREAM_BACKEND_GSTREAMER, STREAM_BACKEND_OPENCV


class AsyncVideoWriter:
    def __init__(
        self,
        cv2,
        *,
        path: str,
        fps: float,
        width: int,
        height: int,
        max_queue_size: int,
        output_queue,
        backend: str,
    ) -> None:
        self.cv2 = cv2
        self.path = str(path)
        self.fps = float(fps)
        self.width = int(width)
        self.height = int(height)
        self.output_queue = output_queue
        self.backend = normalize_stream_backend(backend)
        self.frames: queue.Queue[Any] = queue.Queue(maxsize=max_queue_size)
        self.stop_event = threading.Event()
        self.dropped_frames = 0
        self.written_frames = 0
        self._last_drop_status_s = 0.0
        self._writer_thread_dead_reported = False
        self.thread = threading.Thread(target=self._run, name="stereo-video-writer", daemon=True)
        self.thread.start()

    def enqueue(self, frame) -> None:
        if self.stop_event.is_set():
            return
        if not self.thread.is_alive():
            # The writer thread exited on its own (e.g. cv2.VideoWriter
            # failed to open) without stop_event being set. Stop pushing
            # into a queue nobody will ever drain, and report the failure
            # exactly once instead of spamming the same warning forever.
            self.stop_event.set()
            if not self._writer_thread_dead_reported:
                self._writer_thread_dead_reported = True
                _emit_status(
                    self.output_queue,
                    f"영상 녹화 writer 스레드가 예기치 않게 종료되어 프레임을 버립니다: {self.path}",
                )
            return
        try:
            self.frames.put_nowait(frame)
            return
        except queue.Full:
            pass
        try:
            self.frames.get_nowait()
            self.frames.task_done()
        except queue.Empty:
            pass
        self.dropped_frames += 1
        now_s = time.monotonic()
        if now_s - self._last_drop_status_s >= 1.0:
            self._last_drop_status_s = now_s
            _emit_status(
                self.output_queue,
                f"영상 녹화 queue 포화: 오래된 프레임 드롭 중 ({self.dropped_frames} dropped)",
            )
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            self.dropped_frames += 1

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=GST_RECORDING_EOS_TIMEOUT_S)
        if self.thread.is_alive():
            _emit_status(self.output_queue, f"영상 녹화 writer 종료 지연: {self.path}")

    def _run(self) -> None:
        writer = None
        success = False
        detail = "writer did not open"
        try:
            output_path = Path(self.path).expanduser()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = self.cv2.VideoWriter_fourcc(*"mp4v")
            writer = self.cv2.VideoWriter(str(output_path), fourcc, self.fps, (self.width, self.height))
            if not writer.isOpened():
                writer.release()
                writer = None
                detail = "VideoWriter open failed"
                _emit_status(self.output_queue, f"영상 녹화 파일 열기 실패: {output_path}")
                return
            _emit_status(self.output_queue, f"영상 녹화 시작: {output_path}")
            while not self.stop_event.is_set() or not self.frames.empty():
                try:
                    frame = self.frames.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    if frame.shape[1] != self.width or frame.shape[0] != self.height:
                        frame = self.cv2.resize(frame, (self.width, self.height), interpolation=self.cv2.INTER_AREA)
                    writer.write(frame)
                    self.written_frames += 1
                finally:
                    self.frames.task_done()
            success = True
            detail = "VideoWriter released"
        except Exception as exc:
            detail = f"{type(exc).__name__}: {exc}"
            _emit_status(self.output_queue, f"영상 녹화 writer 오류: {exc}")
        finally:
            if writer is not None:
                writer.release()
            _emit_recording_finalized(
                self.output_queue,
                path=self.path,
                backend=self.backend,
                success=success,
                detail=detail,
                metrics={
                    "written_frames": self.written_frames,
                    "dropped_frames": self.dropped_frames,
                },
            )
            status_text = "저장 완료" if success else "저장 실패"
            _emit_status(
                self.output_queue,
                f"영상 녹화 {status_text}: {self.path} "
                f"(written={self.written_frames}, dropped={self.dropped_frames}, detail={detail})",
            )


def ffmpeg_overwrite_flag(path: str | Path) -> str:
    """Return ``-y`` only for the app's own empty slot reservation.

    ``allocate_workstation_data_artifact`` reserves an output path with
    ``O_CREAT|O_EXCL`` and leaves a zero-byte file behind for the writer to
    fill. FFmpeg's ``-n`` treats that placeholder as existing evidence and
    exits ("File ... already exists"), which left every decoded NVENC MP4 at
    0 bytes. A non-empty file is real evidence and must still never be
    overwritten.
    """

    try:
        target = Path(path).expanduser()
        if target.is_file() and target.stat().st_size == 0:
            return "-y"
    except OSError:
        pass
    return "-n"


class DecodedNv12Recorder:
    """Own one isolated bounded raw-NV12 -> FFmpeg NVENC MP4 writer."""

    def __init__(
        self,
        Gst,
        *,
        path: str,
        width: int,
        height: int,
        fallback_fps: float,
        output_queue,
    ) -> None:
        self.Gst = Gst
        self.path = str(Path(path).expanduser())
        self.width = int(width)
        self.height = int(height)
        self.expected_frame_bytes = self.width * self.height * 3 // 2
        self.frame_period_ns = max(
            1,
            int(round(1_000_000_000.0 / max(1.0, float(fallback_fps)))),
        )
        self.output_queue = output_queue
        self.submitted_frames = 0
        self.written_frames = 0
        self.dropped_frames = 0
        self.duplicated_frames = 0
        self._first_source_pts_ns: int | None = None
        self._last_source_pts_ns: int | None = None
        self._failed_detail = ""
        self._closed = False
        self._accepting = True
        self._stderr_lock = threading.Lock()
        self._stderr_tail = b""
        self.frames: queue.Queue[tuple[bytes, int] | None] = queue.Queue(maxsize=2)

        output_path = Path(self.path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        command = [
            "ffmpeg",
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            ffmpeg_overwrite_flag(output_path),
            "-f",
            "rawvideo",
            "-pixel_format",
            "nv12",
            "-video_size",
            f"{self.width}x{self.height}",
            "-framerate",
            f"{max(1.0, float(fallback_fps)):.6f}",
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "h264_nvenc",
            "-preset",
            "losslesshp",
            "-tune",
            "lossless",
            "-rc",
            "constqp",
            "-qp",
            "0",
            "-bf",
            "0",
            "-g",
            "30",
            "-vsync",
            "0",
            self.path,
        ]
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        if self.process.stdin is None or self.process.stderr is None:
            self.process.terminate()
            raise RuntimeError("FFmpeg NVENC recorder pipes are unavailable")
        self.thread = threading.Thread(
            target=self._run,
            name="stereo-nvenc-video-writer",
            daemon=True,
        )
        self.thread.start()
        self._stderr_thread = threading.Thread(
            target=self._drain_stderr,
            name="stereo-nvenc-stderr-reader",
            daemon=True,
        )
        self._stderr_thread.start()
        _emit_status(
            self.output_queue,
            f"Decoded NV12 lossless FFmpeg/NVENC recording started: {self.path}",
        )

    def submit(self, buffer, *, observed_monotonic_ns: int) -> bool:
        if self._closed or not self._accepting or self._failed_detail:
            return False
        if self.process.poll() is not None:
            self._failed_detail = f"FFmpeg exited early: {self.process.returncode}"
            return False
        success, map_info = buffer.map(self.Gst.MapFlags.READ)
        if not success:
            return False
        try:
            data = bytes(map_info.data)
        finally:
            buffer.unmap(map_info)
        if len(data) != self.expected_frame_bytes:
            self._failed_detail = f"NV12 buffer size mismatch: expected={self.expected_frame_bytes}, actual={len(data)}"
            return False
        source_pts_ns = int(observed_monotonic_ns if buffer.pts == self.Gst.CLOCK_TIME_NONE else buffer.pts)
        if self._first_source_pts_ns is None:
            self._first_source_pts_ns = source_pts_ns
        self._last_source_pts_ns = source_pts_ns
        self.submitted_frames += 1
        item = (data, source_pts_ns)
        try:
            self.frames.put_nowait(item)
            return True
        except queue.Full:
            pass
        try:
            self.frames.get_nowait()
            self.frames.task_done()
            self.dropped_frames += 1
        except queue.Empty:
            pass
        try:
            self.frames.put_nowait(item)
            return True
        except queue.Full:
            self.dropped_frames += 1
            return False

    def _run(self) -> None:
        assert self.process.stdin is not None
        previous_data: bytes | None = None
        previous_source_pts_ns: int | None = None
        try:
            while True:
                item = self.frames.get()
                try:
                    if item is None:
                        return
                    data, source_pts_ns = item
                    if previous_data is not None and previous_source_pts_ns is not None:
                        elapsed_ns = max(0, source_pts_ns - previous_source_pts_ns)
                        missing_intervals = max(
                            0,
                            int(round(elapsed_ns / self.frame_period_ns)) - 1,
                        )
                        for _ in range(min(30, missing_intervals)):
                            self.process.stdin.write(previous_data)
                            self.written_frames += 1
                            self.duplicated_frames += 1
                    self.process.stdin.write(data)
                    self.written_frames += 1
                    previous_data = data
                    previous_source_pts_ns = source_pts_ns
                finally:
                    self.frames.task_done()
        except (BrokenPipeError, OSError) as exc:
            self._failed_detail = f"{type(exc).__name__}: {exc}"
        finally:
            try:
                self.process.stdin.close()
            except OSError:
                pass

    def _enqueue_stop(self) -> None:
        try:
            self.frames.put(None, timeout=GST_RECORDING_EOS_TIMEOUT_S)
        except queue.Full:
            self._failed_detail = "recorder queue did not drain for shutdown"

    def _drain_stderr(self) -> None:
        # ffmpeg's stderr must be serviced continuously while it runs: the
        # OS pipe buffer is small (~64KB), and once it fills, ffmpeg blocks
        # writing to stderr and stops reading stdin too, stalling the whole
        # recording pipeline. Keep only a bounded tail for diagnostics.
        if self.process.stderr is None:
            return
        try:
            while True:
                chunk = self.process.stderr.read(4096)
                if not chunk:
                    return
                with self._stderr_lock:
                    self._stderr_tail = (self._stderr_tail + chunk)[-16_384:]
        except (OSError, ValueError):
            return

    def _stderr_text(self) -> str:
        with self._stderr_lock:
            data = self._stderr_tail
        return data.decode("utf-8", errors="replace").strip()

    def close(self) -> bool:
        if self._closed:
            return not bool(self._failed_detail)
        self._closed = True
        self._accepting = False
        self._enqueue_stop()
        self.thread.join(timeout=1.0)
        if self.thread.is_alive():
            self._failed_detail = "recorder writer timeout"
            self.process.terminate()
            self.thread.join(timeout=1.0)
        try:
            returncode = self.process.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            returncode = self.process.wait(timeout=1.0)
            self._failed_detail = self._failed_detail or "FFmpeg exit timeout"
        self._stderr_thread.join(timeout=1.0)
        stderr_text = self._stderr_text()
        if returncode != 0 and not self._failed_detail:
            self._failed_detail = f"FFmpeg exitcode={returncode}" + (f": {stderr_text}" if stderr_text else "")
        elif returncode != 0 and stderr_text:
            self._failed_detail = f"{self._failed_detail}: {stderr_text}"

        output_path = Path(self.path)
        try:
            present = output_path.is_file() and output_path.stat().st_size > 0
        except OSError:
            present = False
        success = bool(returncode == 0 and present and not self._failed_detail)
        detail = (
            "FFmpeg closed; lossless NVENC MP4 finalized; output validation pending"
            if success
            else self._failed_detail or "lossless NVENC MP4 unavailable"
        )
        _emit_recording_finalized(
            self.output_queue,
            path=self.path,
            backend=STREAM_BACKEND_GSTREAMER,
            success=success,
            detail=detail,
            contract=recording_integrity_contract(STREAM_BACKEND_GSTREAMER),
            metrics={
                "container_finalization_success": success,
                "source_media_integrity_verified": False,
                "recording_file_integrity_verified": False,
                "integrity_validation_status": "PENDING" if success else "FAIL",
                "integrity_failure_reasons": (
                    ["finalized_mp4_output_validation_pending"] if success else ["decoded_nvenc_recording_failed"]
                ),
                "recording_usable": success,
                "submitted_frames": self.submitted_frames,
                "encoded_frames": self.written_frames,
                "dropped_frames": self.dropped_frames,
                "duplicated_frames": self.duplicated_frames,
                "recording_timing_basis": "source_pts_fixed_rate_gap_duplication",
                "source_duration_ns": (
                    0
                    if self._first_source_pts_ns is None or self._last_source_pts_ns is None
                    else max(
                        0,
                        self._last_source_pts_ns - self._first_source_pts_ns,
                    )
                ),
            },
        )
        _emit_status(
            self.output_queue,
            f"Decoded NVENC recording {'finalized' if success else 'failed'}: "
            f"{self.path} (submitted={self.submitted_frames}, "
            f"encoded={self.written_frames}, dropped={self.dropped_frames}, "
            f"detail={detail})",
        )
        return success


def _open_record_video_writer(cv2, config: dict[str, Any], output_queue, width: int, height: int):
    path = _record_video_path(config)
    if not path or width <= 0 or height <= 0:
        return None
    try:
        return AsyncVideoWriter(
            cv2,
            path=path,
            fps=_record_video_fps(config),
            width=width,
            height=height,
            max_queue_size=_record_video_queue_size(config),
            output_queue=output_queue,
            backend=str(config.get("stream_backend", STREAM_BACKEND_OPENCV)),
        )
    except Exception as exc:
        _emit_status(output_queue, f"영상 녹화 시작 실패: {exc}")
        return None


def _write_record_video_frame(cv2, writer_state, frame) -> None:
    if writer_state is None:
        return
    writer_state.enqueue(frame)


def _release_record_video_writer(output_queue, writer_state) -> None:
    if writer_state is None:
        return
    writer_state.close()


# Retain legacy pickle names without importing the facade.
AsyncVideoWriter.__module__ = "Workstation.workstation_core.yolo_stream"
DecodedNv12Recorder.__module__ = "Workstation.workstation_core.yolo_stream"
