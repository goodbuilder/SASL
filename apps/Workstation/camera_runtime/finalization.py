"""One pending video teardown publication; media and recording keep their resources.

Paths/runtime generation are captured when Stop is admitted. Container presence
never becomes an integrity PASS or a gate that stops ongoing observation.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Callable
from Workstation.workstation_core.media_stream.configuration import (
    RECORDING_ENCODING_DECODED_NVENC_LOSSLESS,
    recording_integrity_contract,
)


@dataclass(frozen=True)
class VideoFinalizationPorts:
    read_finalization: Callable
    consume_finalization: Callable
    publish_evidence: Callable
    append: Callable | None = None
    record_presence: Callable | None = None
    flush: Callable | None = None
    log: Callable | None = None

    def __post_init__(self):
        for callback in (self.read_finalization, self.consume_finalization, self.publish_evidence):
            if not callable(callback):
                raise TypeError("Finalization requires media/evidence handoffs")


class VideoFinalization:
    def __init__(self, publish_legacy=None):
        self._pending = None
        self._publish_legacy = publish_legacy

    @property
    def pending(self):
        return self._pending

    @pending.setter
    def pending(self, value):
        self._pending = value
        if self._publish_legacy is not None:
            self._publish_legacy(value)

    def begin(self, paths, *, backend, runtime_generation):
        self.pending = {"paths": dict(paths), "backend": backend, "runtime_generation": str(runtime_generation)}

    def finalize(self, ports: VideoFinalizationPorts) -> bool:
        pending = self.pending
        if not isinstance(pending, dict):
            return True
        paths = pending.get("paths")
        if not isinstance(paths, dict):
            self.pending = None
            return True
        backend = str(pending.get("backend") or "gstreamer")
        runtime_generation = str(pending.get("runtime_generation") or "")
        append = ports.append
        recorded_sides = {str(side) for side in tuple(pending.get("recorded_sides", ()) or ())}
        for side, raw_path in paths.items():
            side = str(side)
            if side in recorded_sides:
                continue
            path = Path(raw_path).expanduser()
            payload = dict(ports.read_finalization(side, str(path)) or {})
            side_backend = str(payload.get("stream_backend") or backend)
            default_contract = recording_integrity_contract(side_backend)
            encoding_mode = str(payload.get("recording_encoding_mode") or default_contract["recording_encoding_mode"])
            try:
                inferred_present = path.is_file()
            except OSError:
                inferred_present = False
            if not payload and inferred_present and encoding_mode == RECORDING_ENCODING_DECODED_NVENC_LOSSLESS:
                payload = {
                    **default_contract,
                    "path": str(path),
                    "stream_backend": side_backend,
                    "success": True,
                    "container_finalization_success": True,
                    "container_finalization_inferred": True,
                    "detail": "camera child closed; file existence observed",
                    "integrity_validation_status": "NOT_RUN",
                    "recording_integrity_verified": False,
                    "recording_file_integrity_verified": False,
                    "source_media_integrity_verified": False,
                    "recording_usable": False,
                    "integrity_failure_reasons": [],
                }
            container_success = bool(
                payload.get(
                    "container_finalization_success",
                    payload.get("success", False),
                )
            )
            success = bool(payload.get("success", False))
            detail = str(payload.get("detail") or "worker exited without recording-finalization evidence")
            try:
                present = path.is_file()
                size_bytes = path.stat().st_size if present else 0
            except OSError:
                present = False
                size_bytes = 0
            validation_status = str(payload.get("integrity_validation_status") or "NOT_RUN").upper()
            validated_usable = bool(
                validation_status == "PASS"
                and payload.get("recording_usable") is True
                and payload.get("recording_integrity_verified") is True
            )
            artifact = {
                "event_type": "video_finalized",
                "runtime_generation": runtime_generation,
                "side": side,
                "path": str(path),
                "media_type": "video/mp4",
                "stream_backend": side_backend,
                **default_contract,
                "recording_encoding_mode": encoding_mode,
                **payload,
                "success": success,
                "container_finalization_success": container_success,
                "finalization_success": success,
                "finalization_detail": detail,
                "present": present,
                "presence_state": "SAVED" if present else "FAILED",
                "size_bytes": size_bytes,
                "integrity_validation_status": ("PASS" if validated_usable else "NOT_RUN"),
                "recording_usable": validated_usable,
            }
            presence_recorder = ports.record_presence
            if callable(presence_recorder):
                presence_recorder(
                    side,
                    path,
                    detail=detail,
                    runtime_generation=runtime_generation,
                )
            accepted = True
            if callable(append):
                accepted = (
                    append(
                        "artifacts",
                        artifact,
                        priority=True,
                    )
                    is not False
                )
            if not accepted:
                logger = ports.log
                if callable(logger):
                    logger(f"[저장] {side} MP4 finalization evidence queue 실패; Measurement와 다음 실행은 계속합니다.")
            if self.pending is not pending:
                return False
            ports.publish_evidence(side, dict(artifact))
            if self.pending is not pending:
                return False
            recorded_sides.add(side)
            ports.consume_finalization(side, str(path))
            if self.pending is not pending:
                return False
            if not success:
                logger = ports.log
                if callable(logger):
                    logger(f"[저장] {side} MP4 finalization 검증 실패: {path} ({detail})")
        pending["recorded_sides"] = tuple(sorted(recorded_sides))
        if any(str(side) not in recorded_sides for side in paths):
            return False
        flush = ports.flush
        if callable(flush):
            try:
                flush()
            except Exception:
                pass
        if self.pending is not pending:
            return False
        self.pending = None
        return True

    def record_presence(self, ports: VideoFinalizationPorts) -> bool:
        """Record file presence without making persistence a teardown gate."""

        pending = self.pending
        if not isinstance(pending, dict):
            return True
        paths = pending.get("paths")
        if not isinstance(paths, dict):
            self.pending = None
            return True
        backend = str(pending.get("backend") or "gstreamer")
        runtime_generation = str(pending.get("runtime_generation") or "")
        append = ports.append
        presence_recorder = ports.record_presence
        for raw_side, raw_path in paths.items():
            side = str(raw_side)
            path = Path(raw_path).expanduser()
            payload = dict(ports.read_finalization(side, str(path)) or {})
            try:
                present = path.is_file()
                size_bytes = path.stat().st_size if present else 0
            except OSError:
                present = False
                size_bytes = 0
            detail = str(payload.get("detail") or "worker exited without recording-finalization evidence")
            validation_status = str(payload.get("integrity_validation_status") or "NOT_RUN").upper()
            validated_usable = bool(
                validation_status == "PASS"
                and payload.get("recording_usable") is True
                and payload.get("recording_integrity_verified") is True
            )
            artifact = {
                "event_type": "video_finalized",
                "runtime_generation": runtime_generation,
                "side": side,
                "path": str(path),
                "media_type": "video/mp4",
                "stream_backend": str(payload.get("stream_backend") or backend),
                **recording_integrity_contract(str(payload.get("stream_backend") or backend)),
                **payload,
                "success": bool(payload.get("success", False)),
                "container_finalization_success": bool(
                    payload.get(
                        "container_finalization_success",
                        payload.get("success", False),
                    )
                ),
                "finalization_success": bool(payload.get("success", False)),
                "finalization_detail": detail,
                "present": present,
                "presence_state": "SAVED" if present else "FAILED",
                "size_bytes": size_bytes,
                "integrity_validation_status": ("PASS" if validated_usable else "NOT_RUN"),
                "recording_usable": validated_usable,
            }
            if callable(presence_recorder):
                try:
                    presence_recorder(
                        side,
                        path,
                        detail=detail,
                        runtime_generation=runtime_generation,
                    )
                except Exception:
                    pass
            if callable(append):
                try:
                    append("artifacts", artifact, priority=True)
                except Exception:
                    pass
            if self.pending is not pending:
                return False
            ports.publish_evidence(side, dict(artifact))
            if self.pending is not pending:
                return False
            ports.consume_finalization(side, str(path))
            if self.pending is not pending:
                return False
        flush = ports.flush
        if callable(flush):
            try:
                flush()
            except Exception:
                pass
        if self.pending is not pending:
            return False
        self.pending = None
        return True
