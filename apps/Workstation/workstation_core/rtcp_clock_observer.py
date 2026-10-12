"""Decoder-free RTCP Sender Report observer for Workstation clock sync."""

from __future__ import annotations

import threading
from collections import deque
from collections.abc import Callable, Mapping
from typing import Protocol

from PyQt5 import QtCore

from Workstation.workstation_core.media_stream.rtp_timing import GstRtpTimingTracker
from Workstation.workstation_core.media_stream.configuration import normalize_rtsp_transport


class ClockProbe(Protocol):
    def start(self) -> None: ...

    def poll(self) -> list[dict[str, object]]: ...

    def stop(self) -> None: ...


def _gst_string(value: object) -> str:
    escaped = str(value).replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


class GstRtcpClockProbe:
    """One RTSP session that discards media and retains only RTCP truth."""

    def __init__(
        self,
        side: str,
        *,
        url: str,
        latency_ms: int = 100,
        transport: str = "tcp",
        status_callback: Callable[[str], None] | None = None,
    ) -> None:
        self.side = str(side)
        self.url = str(url)
        self.latency_ms = max(0, int(latency_ms))
        self.transport = normalize_rtsp_transport(transport)
        self.status_callback = status_callback
        self.Gst = None
        self.pipeline = None
        self.bus = None
        self.tracker: GstRtpTimingTracker | None = None

    def _status(self, text: str) -> None:
        if self.status_callback is not None:
            self.status_callback(f"{self.side}: {text}")

    def start(self) -> None:
        try:
            import gi

            gi.require_version("Gst", "1.0")
            gi.require_version("GstRtp", "1.0")
            from gi.repository import Gst, GstRtp
        except Exception as exc:  # pragma: no cover - target runtime boundary.
            raise RuntimeError(f"GStreamer RTCP observer unavailable: {exc}") from exc
        Gst.init(None)
        pipeline_text = (
            "rtspsrc name=src "
            f"location={_gst_string(self.url)} latency={self.latency_ms} "
            f"protocols={self.transport} drop-on-latency=true do-rtcp=true "
            "! rtph264depay name=depay source-info=true "
            "! fakesink name=clock_sink sync=false async=false"
        )
        pipeline = Gst.parse_launch(pipeline_text)
        bus = pipeline.get_bus()
        if bus is None:
            pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("RTCP observer pipeline has no bus")
        tracker = GstRtpTimingTracker(
            Gst,
            GstRtp,
            status_callback=self._status,
        )
        tracker.attach(pipeline)
        state_result = pipeline.set_state(Gst.State.PLAYING)
        if state_result == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("RTCP observer failed to enter PLAYING")
        self.Gst = Gst
        self.pipeline = pipeline
        self.bus = bus
        self.tracker = tracker
        self._status("PLAYING; fresh Sender Report pending")

    def poll(self) -> list[dict[str, object]]:
        if self.pipeline is None or self.bus is None or self.tracker is None or self.Gst is None:
            raise RuntimeError("RTCP observer is not started")
        message = self.bus.timed_pop_filtered(
            0,
            self.Gst.MessageType.ERROR | self.Gst.MessageType.EOS,
        )
        if message is not None:
            if message.type == self.Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                raise RuntimeError(f"RTCP observer GStreamer error: {error}; {debug or ''}")
            raise RuntimeError("RTCP observer reached EOS")
        self.tracker.poll_sender_reports()
        return [dict(row) for row in self.tracker.sender_report_observations()]

    def stop(self) -> None:
        pipeline = self.pipeline
        Gst = self.Gst
        self.pipeline = None
        self.bus = None
        self.tracker = None
        if pipeline is not None and Gst is not None:
            pipeline.set_state(Gst.State.NULL)


class RtcpClockObserverWorker(QtCore.QObject):
    """Own two decoder-free RTSP probes on one non-GUI QThread."""

    observation_ready = QtCore.pyqtSignal(object)
    status_changed = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    # `ntp_raw` changes on almost every RTCP Sender Report, so the dedup set
    # of (side, ssrc, ntp_raw) identities would otherwise grow unbounded over
    # a long-running session. Cap it and evict the oldest entries FIFO.
    MAX_EMITTED_IDENTITIES = 4096

    def __init__(
        self,
        targets: Mapping[str, Mapping[str, object]],
        *,
        observer_generation: int,
        probe_factory: Callable[..., ClockProbe] = GstRtcpClockProbe,
        poll_interval_s: float = 0.25,
        max_poll_cycles: int | None = None,
    ) -> None:
        super().__init__()
        self.targets = {
            str(side): dict(config)
            for side, config in targets.items()
            if str(side) in {"left", "right"}
        }
        self.observer_generation = int(observer_generation)
        self.probe_factory = probe_factory
        self.poll_interval_s = max(0.001, float(poll_interval_s))
        self.max_poll_cycles = (
            None if max_poll_cycles is None else max(1, int(max_poll_cycles))
        )
        self._cancel_event = threading.Event()

    def cancel(self) -> None:
        self._cancel_event.set()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        probes: dict[str, ClockProbe] = {}
        emitted_identities: set[tuple[str, int, int]] = set()
        emitted_identities_order: deque[tuple[str, int, int]] = deque()
        try:
            if set(self.targets) != {"left", "right"}:
                raise ValueError("RTCP clock observer requires exact Left and Right targets")
            for side in ("left", "right"):
                target = self.targets[side]
                probe = self.probe_factory(
                    side,
                    url=str(target.get("url") or ""),
                    latency_ms=int(target.get("latency_ms") or 100),
                    transport=str(target.get("transport") or "tcp"),
                    status_callback=self.status_changed.emit,
                )
                probes[side] = probe
                probe.start()
            self.status_changed.emit(
                f"RTCP clock observer generation {self.observer_generation} started"
            )
            poll_cycles = 0
            while not self._cancel_event.is_set():
                poll_cycles += 1
                for side in ("left", "right"):
                    for row in probes[side].poll():
                        try:
                            identity = (
                                side,
                                int(row["ssrc"]),
                                int(row["ntp_raw"]),
                            )
                        except (KeyError, TypeError, ValueError):
                            continue
                        if identity in emitted_identities:
                            continue
                        emitted_identities.add(identity)
                        emitted_identities_order.append(identity)
                        while len(emitted_identities_order) > self.MAX_EMITTED_IDENTITIES:
                            evicted = emitted_identities_order.popleft()
                            emitted_identities.discard(evicted)
                        payload = {
                            "schema": "workstation-rtcp-clock-observation-v1",
                            "side": side,
                            "observer_generation": self.observer_generation,
                            **dict(row),
                        }
                        self.observation_ready.emit(payload)
                if self.max_poll_cycles is not None and poll_cycles >= self.max_poll_cycles:
                    break
                self._cancel_event.wait(self.poll_interval_s)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            for probe in probes.values():
                try:
                    probe.stop()
                except Exception:
                    pass
            self.finished.emit()


__all__ = [
    "GstRtcpClockProbe",
    "RtcpClockObserverWorker",
]
