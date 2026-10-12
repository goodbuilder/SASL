"""ipc owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import multiprocessing
import queue


class CameraIpcCounters:
    """Small shared counters for independently bounded camera IPC channels."""

    def __init__(self, mp_context: multiprocessing.context.BaseContext) -> None:
        self.preview_replaced = mp_context.Value("q", 0)
        self.preview_dropped = mp_context.Value("q", 0)
        self.metadata_replaced = mp_context.Value("q", 0)
        self.metadata_dropped = mp_context.Value("q", 0)
        self.control_dropped = mp_context.Value("q", 0)

    @staticmethod
    def _increment(counter) -> None:
        with counter.get_lock():
            counter.value += 1

    def snapshot(self) -> dict[str, int]:
        return {
            "preview_replaced": int(self.preview_replaced.value),
            "preview_dropped": int(self.preview_dropped.value),
            "metadata_replaced": int(self.metadata_replaced.value),
            "metadata_dropped": int(self.metadata_dropped.value),
            "control_dropped": int(self.control_dropped.value),
        }


class _BoundedProcessOutputQueue:
    METADATA_KINDS = frozenset({"detection", "stats"})
    CRITICAL_CONTROL_KINDS = frozenset(
        {
            "decoder_fallback",
            "fatal_error",
            "recording_finalized",
            "terminal_exit",
        }
    )

    def __init__(
        self,
        control_queue,
        frame_queue,
        metadata_queue=None,
        counters: CameraIpcCounters | None = None,
    ) -> None:
        # event_queue remains an alias for compatibility with tests and older
        # process targets. Actual Workstation runs provide a separate metadata
        # queue so image pressure cannot delay Stop/Error/control delivery.
        self.control_queue = control_queue
        self.event_queue = control_queue
        self.frame_queue = frame_queue
        self.metadata_queue = metadata_queue
        self.counters = counters

    def _count(self, name: str) -> None:
        if self.counters is not None:
            CameraIpcCounters._increment(getattr(self.counters, name))

    @staticmethod
    def _is_nal1_candidate_frame(message: object) -> bool:
        return bool(
            isinstance(message, tuple)
            and len(message) >= 2
            and message[0] == "frame"
            and isinstance(message[1], dict)
            and message[1].get("h264_nal1_non_idr_intra") is True
        )

    def _replace_latest(
        self,
        target_queue,
        message,
        *,
        replaced: str,
        dropped: str,
        protect_nal1_candidate: bool = False,
    ) -> None:
        try:
            target_queue.put_nowait(message)
            return
        except queue.Full:
            pass
        try:
            previous = target_queue.get_nowait()
        except queue.Empty:
            previous = None
        if (
            protect_nal1_candidate
            and self._is_nal1_candidate_frame(previous)
            and not self._is_nal1_candidate_frame(message)
        ):
            try:
                target_queue.put_nowait(previous)
            except queue.Full:
                pass
            self._count(dropped)
            return
        if previous is not None:
            self._count(replaced)
        try:
            target_queue.put_nowait(message)
        except queue.Full:
            self._count(dropped)

    def put(self, message) -> None:
        if message and message[0] == "frame":
            self._replace_latest(
                self.frame_queue,
                message,
                replaced="preview_replaced",
                dropped="preview_dropped",
                protect_nal1_candidate=True,
            )
            return
        kind = message[0] if message else ""
        if self.metadata_queue is not None and kind in self.METADATA_KINDS:
            self._replace_latest(
                self.metadata_queue,
                message,
                replaced="metadata_replaced",
                dropped="metadata_dropped",
            )
            return
        if kind in self.CRITICAL_CONTROL_KINDS:
            try:
                self.control_queue.put_nowait(message)
                return
            except queue.Full:
                try:
                    self.control_queue.get_nowait()
                    self._count("control_dropped")
                except queue.Empty:
                    pass
                try:
                    self.control_queue.put_nowait(message)
                except queue.Full:
                    self._count("control_dropped")
                return
        try:
            self.control_queue.put_nowait(message)
        except queue.Full:
            # Status is observational. Stop is an independent Event and fatal
            # messages use the bounded critical path above.
            self._count("control_dropped")


def _queue_has_pending_messages(message_queue) -> bool:
    if message_queue is None:
        return False
    try:
        return not message_queue.empty()
    except Exception:
        return False


def _get_queue_message(message_queue, *, timeout_s: float):
    if message_queue is None:
        raise queue.Empty
    return message_queue.get(timeout=timeout_s)


def _get_queue_message_nowait(message_queue):
    if message_queue is None:
        raise queue.Empty
    return message_queue.get_nowait()


# Retain legacy pickle names without importing the facade.
CameraIpcCounters.__module__ = "Workstation.workstation_core.yolo_stream"
_BoundedProcessOutputQueue.__module__ = "Workstation.workstation_core.yolo_stream"
