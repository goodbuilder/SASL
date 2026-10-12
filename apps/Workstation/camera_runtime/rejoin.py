"""RTSP recovery rejoin transaction over captured targets and explicit effects.

The control runtime owns the transaction generation. Recording retains segment
ownership, the media pane retains its frame generation, and StreamRecovery
registers the first-frame admission. No image copy, clock conversion or new
recording/readiness gate is introduced here.
"""

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class ReactivationRequest:
    side: str
    generation: int
    enable_detection: bool


@dataclass(frozen=True)
class RejoinTarget:
    url: str
    start_stream: Callable
    read_state: Callable


@dataclass(frozen=True)
class RejoinRecordingSnapshot:
    active: bool
    run_state: str


@dataclass(frozen=True)
class ReactivationPorts:
    capture_target: Callable
    prepare_preview: Callable
    read_recording: Callable
    start_recording: Callable
    current_segment: Callable
    allocate_segment: Callable
    install_segment: Callable
    stream_options: Callable
    recording_options: Callable
    read_side: Callable
    publish_source: Callable
    set_availability: Callable
    report: Callable
    refresh: Callable
    log: Callable

    def __post_init__(self):
        if not all(callable(value) for value in vars(self).values()):
            raise TypeError("RTSP rejoin requires explicit recording, media and display handoffs")


def finish_reactivation(request, *, success, detail, current, recovery, ports: ReactivationPorts):
    """Complete only the still-current control transaction, preserving old policy."""
    side = request.side
    if not current():
        return
    if not success:
        ports.report(side, "failed", str(detail or "frame check failed"), False)
        if not current():
            return
        ports.publish_source(f"camera.{side}", "UNAVAILABLE", str(detail or "frame check failed"))
        if not current():
            return
        ports.publish_source(f"recording.{side}", "UNAVAILABLE", str(detail or "frame check failed"))
        if current():
            ports.refresh(False)
        return

    target = ports.capture_target(side)
    if not current():
        return
    ports.prepare_preview(side, request.enable_detection)
    if not current():
        return
    recording = ports.read_recording()
    if not current():
        return
    active = recording.active
    started_recording = False
    if not active and recording.run_state == "RUNNING":
        try:
            started_recording = bool(ports.start_recording())
        except Exception as exc:
            ports.log(f"[저장] 카메라 재합류 recording 준비 실패: {exc}")
        if not current():
            return
        active = ports.read_recording().active
        if not current():
            return
    if active:
        path = ports.current_segment(side) if started_recording else ports.allocate_segment(side)
        if not current():
            return
        if path is not None:
            ports.install_segment(side, path)
            if not current():
                return
    elif recording.run_state == "RUNNING":
        ports.publish_source(f"recording.{side}", "UNAVAILABLE", "recording could not join recovered camera")
        if not current():
            return

    # Options are sampled after recording preparation, as in the existing
    # call: the just-allocated segment must reach the original captured pane.
    stream_options = ports.stream_options(side)
    if not current():
        return
    recording_options = ports.recording_options(side)
    if not current():
        return
    started = target.start_stream(
        target.url, enable_detection=request.enable_detection, **stream_options, **recording_options
    )
    if not current():
        return
    if not started:
        ports.report(side, "preview-start-failed", "", False)
        if current():
            ports.refresh(False)
        return
    state = target.read_state()
    if not current():
        return
    availability = recovery.expect_manual_rejoin(side, state.generation, read_side=ports.read_side)
    if not current():
        return
    if availability is not None:
        active_sides, detection = availability
        ports.set_availability(active_sides, detection)
        if not current():
            return
    ports.report(side, "pending", "", availability is not None)
    if current():
        ports.refresh(True)
