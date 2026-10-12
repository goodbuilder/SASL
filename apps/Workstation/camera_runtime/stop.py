"""Stop sequencing across media retirement, recording evidence and UI cleanup.

Measurement Run and session-recording state remain behind their existing
commands. A newer accepted Start/Stop invalidates reentrant old completion.
"""

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class CameraStopPorts:
    stop_panes: Callable
    schedule_watchdog: Callable
    finish_recording: Callable
    finish_run_if_stopping: Callable
    publish_stopped: Callable
    log: Callable
    finalization: object

    def __post_init__(self):
        if not all(callable(value) for name, value in vars(self).items() if name != "finalization"):
            raise TypeError("Camera Stop requires media, recording and presentation handoffs")


class CameraStopRuntime:
    def __init__(self, finalization, lifecycle_timers):
        self.finalization = finalization
        self.lifecycle_timers = lifecycle_timers
        self._generation = 0

    def cancel(self):
        self._generation += 1

    def stop(self, *, wait, finalize_session, clock_snapshot, ports: CameraStopPorts):
        self._generation += 1
        generation = self._generation
        panes_stopped = ports.stop_panes(clear_previews=True, wait=wait)
        if generation != self._generation:
            return False
        if wait and not panes_stopped:
            ports.log("[종료] 카메라 worker가 아직 종료 중입니다. 다음 종료 시도에서 다시 확인합니다.")
            return False
        if not panes_stopped:
            ports.schedule_watchdog()
        if generation != self._generation:
            return False
        if finalize_session:
            ports.finish_recording(clock_snapshot)
        if generation != self._generation:
            return False
        if panes_stopped:
            finalized = self.finalization.record_presence(ports.finalization)
            if generation != self._generation:
                return False
            if finalized:
                self.lifecycle_timers.cancel_stop()
                ports.finish_run_if_stopping()
        if generation != self._generation:
            return False
        ports.publish_stopped()
        return bool(panes_stopped)
