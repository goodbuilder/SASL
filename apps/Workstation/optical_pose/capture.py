"""Frozen burst and WIRIS capture ownership, with explicit request inputs.

One GUI-affine owner holds HR/OZ jobs until actual QThread.finished delivery.
Every result is bound to its admitted session/burst revision and request-time
K,D, prior NED poses, GNSS, yaw reference and zoom. Device I/O remains in M11.
"""

from __future__ import annotations

import copy
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping
from PyQt5 import QtCore, QtGui

from .artifacts import OpticalArtifacts
from .observation_history import OpticalObservationHistory
from .selection import OpticalReviewState

CAPTURE_RESOURCE_FIELDS = (
    "optical_pose_wiris_ftp_thread",
    "optical_pose_wiris_ftp_worker",
    "optical_pose_wiris_ftp_oz_thread",
    "optical_pose_wiris_ftp_oz_worker",
)


@dataclass(frozen=True)
class FrozenCaptureInputs:
    session_id: str
    root: Path
    captured_monotonic_s: float
    profile: object
    prior: object
    _evidence: dict = field(repr=False)

    def __post_init__(self):
        object.__setattr__(self, "root", Path(self.root))
        object.__setattr__(self, "_evidence", copy.deepcopy(self._evidence))

    def evidence(self, key, default=None):
        """Public exports never hand out mutable request-time evidence aliases."""
        return copy.deepcopy(self._evidence.get(key, default))


@dataclass(frozen=True)
class FrozenNativeFrame:
    timestamp_s: float
    _image: QtGui.QImage = field(repr=False)
    _metadata: Mapping = field(repr=False)

    def __post_init__(self):
        object.__setattr__(self, "_image", self._image.copy())
        object.__setattr__(self, "_metadata", copy.deepcopy(dict(self._metadata)))

    def materialize(self):
        return self.timestamp_s, QtGui.QImage(self._image), copy.deepcopy(self._metadata)


@dataclass
class OpticalCaptureJob:
    token: int
    mode: str
    capture_id: str
    generation: int
    burst_revision: int
    inputs: FrozenCaptureInputs
    payload: dict
    thread: object
    worker: object
    relay: object = None
    terminal: bool = False
    started: threading.Event = field(default_factory=threading.Event)
    callback_depth: int = 0
    retire_requested: bool = False


class _CaptureRelay(QtCore.QObject):
    def __init__(self, owner, job):
        super().__init__(owner)
        self.owner, self.job = owner, job

    @QtCore.pyqtSlot(object)
    def completed(self, value):
        self.owner.deliver(self.job, value=value)

    @QtCore.pyqtSlot(str)
    def failed(self, error):
        self.owner.deliver(self.job, error=error)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner.retire(self.job)


class OpticalCaptureOwner(QtCore.QObject):
    def __init__(self, *, review: OpticalReviewState, emit, log, parent=None, publish_legacy=None):
        super().__init__(parent)
        self.review = review
        self.observations = OpticalObservationHistory(emit)
        self.artifacts = OpticalArtifacts()
        self._emit, self._log = emit, log
        self._publish_legacy = publish_legacy
        self._frozen_frames = {}
        self._frozen_intrinsics_profile = None
        self._frozen_prior_poses = None
        self._frozen_yaw90_reference = None
        self._frozen_capture_monotonic_s = None
        self.frozen_inputs = None
        self._legacy = {name: None for name in CAPTURE_RESOURCE_FIELDS}
        self._legacy_payload = {}
        self._generation = 0
        self._burst_revision = 0
        self._next_token = 0
        self.current = None
        self.closing = False
        self.callback_error = ""

    @property
    def has_admitted(self):
        return self._next_token > 0

    @property
    def busy(self):
        return self.current is not None or any(value is not None for value in self._legacy.values())

    @property
    def _pending_wiris_ftp_payload(self):
        return copy.deepcopy(self.current.payload if self.current is not None else self._legacy_payload)

    @_pending_wiris_ftp_payload.setter
    def _pending_wiris_ftp_payload(self, value):
        if self.current is not None:
            raise RuntimeError("An admitted capture owns its request payload")
        self._legacy_payload = copy.deepcopy(dict(value))

    def _resource(self, name):
        job = self.current
        prefix = "optical_pose_wiris_ftp_oz_" if job is not None and job.mode == "oz" else "optical_pose_wiris_ftp_"
        if job is not None and name.startswith(prefix) and name[len(prefix) :] in {"thread", "worker"}:
            return getattr(job, name[len(prefix) :])
        return self._legacy[name]

    def _set_resource(self, name, value):
        if self.current is not None:
            if value is not self._resource(name):
                raise RuntimeError("An admitted optical capture must retire through its owner")
            return
        self._legacy[name] = value

    def _publish(self):
        if self._publish_legacy is not None:
            try:
                self._publish_legacy(self)
            except Exception as exc:
                self.callback_error = f"capture resource publication: {exc}"

    def resource_inventory(self):
        return {name: self._resource(name) for name in CAPTURE_RESOURCE_FIELDS}

    def reset(self):
        self._generation += 1
        self._burst_revision += 1
        self._frozen_frames.clear()
        self._frozen_intrinsics_profile = None
        self._frozen_prior_poses = None
        self._frozen_yaw90_reference = None
        self._frozen_capture_monotonic_s = None
        self.frozen_inputs = None
        self._legacy_payload.clear()
        self.observations.reset()
        self.artifacts.reset()
        self.review.reset_burst()

    def _bind(self, inputs, frozen):
        self.frozen_inputs = inputs
        self._frozen_frames = frozen
        self._frozen_intrinsics_profile = inputs.profile
        self._frozen_prior_poses = inputs.prior
        self._frozen_capture_monotonic_s = inputs.captured_monotonic_s
        self._frozen_yaw90_reference = inputs.evidence("yaw90")
        self._burst_revision += 1
        self.review.reset_burst()

    def _burst_event(self, inputs, burst_id, frozen):
        return {
            "schema": "optical-pose-frozen-burst-v1",
            "burst_id": burst_id,
            "session_id": inputs.session_id,
            "frames": {side: frozen[side]["artifact"].to_mapping() for side in ("left", "right")},
            "operating_intrinsics_profile_sha256": "" if inputs.profile is None else inputs.profile.profile_sha256,
            "prior_pose_available": inputs.prior is not None,
            "coordinate_contract": "native decoded pixels; u right, v down; no resize/overlay",
        }

    def capture_native(self, inputs, frames, *, store_factory):
        if self.closing:
            raise RuntimeError("Optical capture is shutting down")
        burst_id = f"pose-burst-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        frozen = self.artifacts.native_frames(
            inputs.root,
            burst_id,
            {side: frame.materialize() for side, frame in frames.items()},
            inputs.evidence("zoom", {}),
            store_factory=store_factory,
        )
        self._bind(inputs, frozen)
        payload = self._burst_event(inputs, burst_id, frozen)
        payload["yaw90_reference"] = copy.deepcopy(self._frozen_yaw90_reference)
        observation = self.observations.capture(
            session_id=inputs.session_id,
            burst_id=burst_id,
            frozen=frozen,
            gnss_snapshot=inputs.evidence("gnss", {}),
            emit=False,
        )
        # Publish evidence only after the complete burst/history is bound. An
        # event sink can re-enter/reset; no old state is written after it does.
        self._emit("optical_pose_burst_captured", payload)
        self._emit("optical_pose_drone_rtk_observation_captured", copy.deepcopy(observation))

    def import_burst(self, inputs, sources, *, store_factory=None, ftp=False):
        # An admitted FTP completion may already have saved stills during close.
        # Closing refuses new admissions, but does not discard that result.
        if self.closing and not ftp:
            raise RuntimeError("Optical capture is shutting down")
        burst_id = f"pose-burst-import-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        kwargs = {} if store_factory is None else {"store_factory": store_factory}
        # Admission fixes K,D/GNSS/zoom. Import receipt is a different instant;
        # it must not be backdated to the request or claimed as exposure time.
        import_received_monotonic_s = time.monotonic() if ftp else inputs.captured_monotonic_s
        frozen, records = self.artifacts.imported_frames(
            inputs.root,
            burst_id,
            sources,
            inputs.evidence("metadata", {}),
            import_received_monotonic_s,
            **kwargs,
        )
        self._bind(inputs, frozen)
        payload = self._burst_event(inputs, burst_id, frozen)
        payload["import_sources"] = records
        yaw90 = copy.deepcopy(self._frozen_yaw90_reference)
        gnss = inputs.evidence("gnss")
        observation = None
        if isinstance(gnss, Mapping):
            observation = self.observations.capture(
                session_id=inputs.session_id,
                burst_id=burst_id,
                frozen=frozen,
                gnss_snapshot=gnss,
                gnss_snapshot_source="ftp_capture_request",
                emit=False,
            )
        self._emit("optical_pose_burst_imported", payload)
        self._emit(
            "optical_pose_yaw90_reference_bound",
            {
                "burst_id": burst_id,
                "frames": payload["frames"],
                "yaw90_reference": yaw90,
            },
        )
        if observation is not None:
            self._emit("optical_pose_drone_rtk_observation_captured", copy.deepcopy(observation))

    def start_ftp(self, *, mode, capture_id, inputs, payload, worker_factory):
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("Optical capture must start on the Qt owner thread")
        if self.closing:
            raise RuntimeError("Optical capture is shutting down")
        if self.busy:
            raise RuntimeError("a WIRIS FTP alignment capture is already in progress")
        if mode not in {"hr", "oz"}:
            raise ValueError("Unknown WIRIS optical capture mode")
        worker = worker_factory()
        thread = QtCore.QThread(self)
        self._next_token += 1
        job = OpticalCaptureJob(
            self._next_token,
            mode,
            capture_id,
            self._generation,
            self._burst_revision,
            inputs,
            copy.deepcopy(dict(payload)),
            thread,
            worker,
        )
        relay = _CaptureRelay(self, job)
        job.relay = relay
        try:
            thread.started.connect(job.started.set, QtCore.Qt.DirectConnection)
            thread.started.connect(worker.run)
            worker.completed.connect(relay.completed, QtCore.Qt.QueuedConnection)
            worker.failed.connect(relay.failed, QtCore.Qt.QueuedConnection)
            worker.finished.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            thread.finished.connect(relay.finished, QtCore.Qt.QueuedConnection)
            worker.moveToThread(thread)
        except Exception:
            worker.deleteLater()
            thread.deleteLater()
            relay.deleteLater()
            raise
        self.current = job
        self._publish()
        try:
            self._log(f"[정렬] WIRIS FTP{self._mode_label(mode)} Capture {capture_id}: 좌/우 카메라 내부 저장 중")
            if self.current is not job or self.closing or job.generation != self._generation:
                raise RuntimeError("Optical capture admission was cancelled before launch")
            thread.start()
            if not thread.isRunning() and not job.started.is_set():
                raise RuntimeError("Optical capture thread did not start")
        except Exception as exc:
            if job.started.is_set() or thread.isRunning():
                self.callback_error = f"start reported an error after launch: {exc}"
                return job
            self.deliver(job, error=str(exc))
            thread.quit()
            self.retire(job)
            worker.deleteLater()
            raise
        return job

    @staticmethod
    def _mode_label(mode):
        return " (OZ)" if mode == "oz" else ""

    def deliver(self, job, *, value=None, error=None):
        if self.current is not job or job.terminal:
            return
        job.terminal = True
        job.callback_depth += 1
        try:
            if job.generation != self._generation:
                return
            if error is not None:
                self._log(f"[정렬] WIRIS FTP{self._mode_label(job.mode)} Capture 실패: {error}")
                return
            record = value if isinstance(value, dict) else {}
            if str(record.get("capture_id") or "") != job.capture_id:
                raise ValueError("WIRIS optical capture result identity does not match admission")
            cameras = record.get("cameras")
            cameras = cameras if isinstance(cameras, dict) else {}
            failures = self.ftp_capture_failures(cameras)
            if set(cameras) != {"left", "right"} or failures:
                raise ValueError("; ".join(failures) or "camera result is missing left/right entries")
            if job.burst_revision != self._burst_revision:
                self._emit(
                    "optical_pose_capture_superseded",
                    {
                        "capture_id": job.capture_id,
                        "session_id": job.inputs.session_id,
                        "reason": "a newer frozen burst replaced the admitted capture",
                        "cameras": copy.deepcopy(cameras),
                    },
                )
                return
            sources = self.ftp_import_sources(capture_id=job.capture_id, cameras=cameras, pending_payload=job.payload)
            self._log(
                f"[정렬] WIRIS FTP{self._mode_label(job.mode)} Capture {job.capture_id}: 다운로드 완료, 프리즈 프레임 저장 중"
            )
            # Reentrant presentation can reset the session or freeze a new burst.
            if job.generation != self._generation or job.burst_revision != self._burst_revision:
                return
            self.import_burst(job.inputs, sources, ftp=True)
        except Exception as exc:
            self.callback_error = str(exc)
            try:
                self._log(
                    f"[정렬] WIRIS FTP{self._mode_label(job.mode)} Capture {job.capture_id}: 프리즈 프레임 저장 실패 - {exc}"
                )
            except Exception as log_error:
                self.callback_error += f"; presentation: {log_error}"
        finally:
            job.callback_depth -= 1
            if not job.callback_depth and job.retire_requested:
                self.retire(job)

    def retire(self, job):
        if self.current is not job:
            return
        if job.callback_depth:
            job.retire_requested = True
            return
        try:
            if job.thread.isRunning():
                return
        except RuntimeError:
            pass
        self.current = None
        job.terminal = True
        self._publish()
        job.relay.deleteLater()
        job.thread.deleteLater()

    def clear_legacy(self, mode):
        if self.has_admitted or self.current is not None:
            return
        prefix = "optical_pose_wiris_ftp_oz_" if mode == "oz" else "optical_pose_wiris_ftp_"
        self._legacy[prefix + "thread"] = self._legacy[prefix + "worker"] = None
        self._publish()

    def request_shutdown(self):
        self.closing = True
        for name, value in self.resource_inventory().items():
            if value is not None and name.endswith("_thread"):
                try:
                    value.quit()
                except RuntimeError:
                    pass
        return not self.busy

    @staticmethod
    def ftp_capture_failures(cameras: dict[object, object]) -> list[str]:
        failures: list[str] = []
        for side, raw_item in cameras.items():
            item = raw_item if isinstance(raw_item, dict) else {}
            state = str(item.get("state") or "failed")
            if state != "completed":
                reason = str(item.get("error") or state)
            elif not bool(item.get("ftp_downloaded")):
                reason = str(item.get("ftp_error") or "FTP download failed")
            else:
                continue
            failures.append(f"{side}: {reason}")
        return failures

    @staticmethod
    def ftp_import_sources(
        *,
        capture_id: str,
        cameras: Mapping[object, object],
        pending_payload: Mapping[str, Any],
    ) -> dict[str, dict[str, Any]]:
        """Build imported-still provenance without claiming an exposure time.

        CPTR request/completion and FTP publication times bracket the still,
        but none is the camera exposure timestamp.  Preserve those audit
        fields while marking the capture clock unknown.  A request-time GZVV
        snapshot may establish zoom; the later live value must not be used.
        """

        zoom_snapshot = pending_payload.get("_ftp_request_zoom_snapshot")
        zoom_snapshot = zoom_snapshot if isinstance(zoom_snapshot, Mapping) else {}
        request_monotonic_s = pending_payload.get("_ftp_request_monotonic_s")
        sources: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            raw_item = cameras.get(side)
            item = raw_item if isinstance(raw_item, Mapping) else {}
            raw_zoom = zoom_snapshot.get(side)
            zoom = raw_zoom if isinstance(raw_zoom, Mapping) else {}
            source: dict[str, Any] = {
                "path": str(item.get("ftp_local_path") or ""),
                "metadata": {
                    "timestamp_reference": "unknown",
                    "capture_timestamp_known": False,
                    "ftp_capture_id": capture_id,
                    "ftp_remote_path": str(item.get("ftp_remote_path") or ""),
                    "capture_requested_unix_ns": item.get("capture_requested_unix_ns"),
                    "capture_completed_unix_ns": item.get("capture_completed_unix_ns"),
                    "ftp_download_finished_unix_ns": item.get("finished_unix_ns"),
                    "capture_request_monotonic_s": request_monotonic_s,
                },
            }
            if zoom.get("index") is not None:
                source["zoom_index"] = zoom.get("index")
            if zoom.get("ratio") is not None:
                source["zoom_ratio"] = zoom.get("ratio")
            if zoom.get("updated_monotonic_s") is not None:
                source["zoom_updated_monotonic_s"] = zoom.get("updated_monotonic_s")
            sources[side] = source
        return sources


def _capture_resource(name):
    return property(lambda owner: owner._resource(name), lambda owner, value: owner._set_resource(name, value))


for _name in CAPTURE_RESOURCE_FIELDS:
    setattr(OpticalCaptureOwner, _name, _capture_resource(_name))
del _name
