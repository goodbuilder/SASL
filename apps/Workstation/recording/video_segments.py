"""Allocate per-generation video segments and record independent file presence.

Owns no decoder or camera stream; it records provenance supplied by those owners."""
from __future__ import annotations
import copy
import time
from collections.abc import Mapping
from pathlib import Path
from .state import (RecordingSessionState, VideoSegmentState)
from .ports import VideoSegmentsPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .presentation import Presentation


class VideoSegments:
    def __init__(self, *,
        session: RecordingSessionState,
        video_state: VideoSegmentState,
        boundary: VideoSegmentsPort,
    ) -> None:
        self.session = session
        self.video_state = video_state
        self.boundary = boundary

    def bind_collaborators(self, *,
        presentation: "Presentation",
    ) -> None:
        self.presentation = presentation

    def _video_segment_registry(self) -> dict[str, dict[str, list[dict[str, object]]]]:
        registry = getattr(
            self.video_state, "_measurement_video_segments_by_generation", None
        )
        if not isinstance(registry, dict):
            registry = {}
            self.video_state._measurement_video_segments_by_generation = registry
        return registry

    def _begin_measurement_video_generation(
        self, generation: str
    ) -> dict[str, list[dict[str, object]]]:
        normalized = str(generation or "manual")
        segments: dict[str, list[dict[str, object]]] = {
            "left": [],
            "right": [],
        }
        registry = self._video_segment_registry()
        registry[normalized] = segments
        self.video_state._measurement_video_segments = segments
        states = getattr(
            self.video_state, "_recording_presence_state_by_generation", None
        )
        if not isinstance(states, dict):
            states = {}
            self.video_state._recording_presence_state_by_generation = states
        states[normalized] = "IDLE"
        self.video_state._last_recording_presence_state = "IDLE"
        # Persistence owns history. Keep only a small runtime window so
        # repeated observations cannot grow GUI memory without bound.
        while len(registry) > 16:
            oldest = next(iter(registry))
            if oldest == normalized:
                break
            registry.pop(oldest, None)
            states.pop(oldest, None)
        return segments

    def measurement_video_segments_snapshot(self) -> dict[str, object]:
        generation = str(
            getattr(self.session, "recording_session_id", "")
            or getattr(self.boundary, "_measurement_run_generation", "")
            or getattr(self.boundary, "_measurement_run_last_generation", "")
            or ""
        )
        registry = self._video_segment_registry()
        segments = registry.get(generation, {}) if generation else getattr(
            self.video_state, "_measurement_video_segments", {}
        )
        states = getattr(
            self.video_state, "_recording_presence_state_by_generation", {}
        )
        state = (
            str(states.get(generation) or "IDLE")
            if generation and isinstance(states, dict)
            else str(
                getattr(self.video_state, "_last_recording_presence_state", "IDLE")
                or "IDLE"
            )
        )
        return {
            "schema": "workstation-video-segments-v1",
            "runtime_generation": generation,
            "state": state,
            "sides": {
                side: [dict(row) for row in tuple(dict(segments or {}).get(side, ()))]
                for side in ("left", "right")
            },
        }

    def allocate_measurement_video_segment(
        self,
        side: str,
        *,
        utc_ns: int | None = None,
    ) -> Path | None:
        camera = str(side)
        if camera not in {"left", "right"}:
            raise ValueError(f"unsupported camera side: {side}")
        allocator = getattr(self.boundary, "allocate_workstation_data_artifact", None)
        if not callable(allocator):
            return None
        generation = str(
            getattr(self.session, "recording_session_id", "")
            or getattr(self.boundary, "_measurement_run_generation", "")
            or "manual"
        )
        registry = self._video_segment_registry()
        segments = registry.get(generation)
        if not isinstance(segments, dict):
            segments = {"left": [], "right": []}
            registry[generation] = segments
        active_generation = str(
            getattr(self.session, "recording_session_id", "")
            or getattr(self.boundary, "_measurement_run_generation", "")
            or ""
        )
        if generation == active_generation:
            self.video_state._measurement_video_segments = segments
        side_segments = segments.setdefault(camera, [])
        segment_index = len(side_segments) + 1
        short_generation = generation.removeprefix("run-")[:12] or "manual"
        created_utc_ns = time.time_ns() if utc_ns is None else int(utc_ns)
        try:
            path = None
            # A running Measurement names its video after the Measurement
            # Start prefix so the MP4 sits next to its workbook.
            prefixed_allocator = getattr(self.boundary, "allocate_measurement_artifact", None)
            if callable(prefixed_allocator) and generation == active_generation:
                try:
                    prefixed = prefixed_allocator(label=camera, suffix=".mp4")
                except (OSError, RuntimeError, TypeError, ValueError):
                    prefixed = None
                if prefixed is not None:
                    path = Path(prefixed)
            if path is None:
                path = Path(
                    allocator(
                        "videos",
                        label=(
                            f"{camera}-{short_generation}-segment-{segment_index:02d}"
                        ),
                        suffix=".mp4",
                        utc_ns=created_utc_ns,
                    )
                )
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            logger = getattr(self.boundary, "_log", None)
            if callable(logger):
                logger(
                    f"[저장] {camera} video segment path unavailable; "
                    f"Measurement continues: {exc}"
                )
            return None
        row = {
            "segment_id": f"{generation}:{camera}:{segment_index}",
            "segment_index": segment_index,
            "side": camera,
            "path": str(path),
            "state": "ALLOCATED",
            "created_utc_ns": created_utc_ns,
            "created_monotonic_ns": time.monotonic_ns(),
            "closed_utc_ns": None,
            "closed_monotonic_ns": None,
            "file_present": False,
            "size_bytes": 0,
        }
        side_segments.append(row)
        writer = getattr(self.boundary, "append_workstation_data", None)
        if callable(writer):
            try:
                writer(
                    "artifacts",
                    {
                        "event_type": "video_segment_allocated",
                        "runtime_generation": generation,
                        **dict(row),
                        "media_type": "video/mp4",
                    },
                    priority=True,
                )
            except Exception:
                pass
        publisher = getattr(self.boundary, "measurement_source_transition", None)
        if callable(publisher):
            publisher(
                f"recording.{camera}",
                "WAITING_RESOURCE",
                f"video segment {segment_index} allocated",
            )
        return path

    def record_measurement_video_segment_presence(
        self,
        side: str,
        path: str | Path,
        *,
        detail: str = "",
        runtime_generation: str | None = None,
    ) -> dict[str, object]:
        camera = str(side)
        source_path = Path(path).expanduser()
        try:
            present = source_path.is_file()
            size_bytes = source_path.stat().st_size if present else 0
        except OSError:
            present = False
            size_bytes = 0
        if camera not in {"left", "right"}:
            raise ValueError(f"unsupported camera side: {side}")
        generation = str(
            runtime_generation
            or getattr(self.session, "recording_session_id", "")
            or getattr(self.boundary, "_measurement_run_generation", "")
            or "unknown"
        )
        registry = self._video_segment_registry()
        segments = registry.get(generation)
        if not isinstance(segments, dict):
            segments = {"left": [], "right": []}
            registry[generation] = segments
        active_generation = str(
            getattr(self.session, "recording_session_id", "")
            or getattr(self.boundary, "_measurement_run_generation", "")
            or ""
        )
        if generation == active_generation:
            self.video_state._measurement_video_segments = segments
        side_segments = segments.setdefault(camera, [])
        row = next(
            (
                value
                for value in reversed(side_segments)
                if str(value.get("path") or "") == str(source_path)
            ),
            None,
        )
        if row is None:
            row = {
                "segment_id": f"{generation}:{camera}:{len(side_segments) + 1}",
                "segment_index": len(side_segments) + 1,
                "side": camera,
                "path": str(source_path),
                "created_utc_ns": None,
                "created_monotonic_ns": None,
            }
            side_segments.append(row)
        row.update(
            state="SAVED" if present else "FAILED",
            closed_utc_ns=time.time_ns(),
            closed_monotonic_ns=time.monotonic_ns(),
            file_present=present,
            size_bytes=size_bytes,
            detail=str(detail),
        )
        side_present = {
            candidate: any(
                bool(value.get("file_present"))
                for value in tuple(dict(segments or {}).get(candidate, ()))
            )
            for candidate in ("left", "right")
        }
        aggregate = (
            "SAVED"
            if all(side_present.values())
            else "PARTIAL"
            if any(side_present.values())
            else "FAILED"
        )
        states = getattr(
            self.video_state, "_recording_presence_state_by_generation", None
        )
        if not isinstance(states, dict):
            states = {}
            self.video_state._recording_presence_state_by_generation = states
        states[generation] = aggregate
        if not active_generation or active_generation == generation:
            self.video_state._last_recording_presence_state = aggregate
        writer = getattr(self.boundary, "append_workstation_data", None)
        if callable(writer):
            try:
                writer(
                    "artifacts",
                    {
                        "event_type": "video_segment_closed",
                        "runtime_generation": generation,
                        **dict(row),
                        "media_type": "video/mp4",
                        "presence_state": aggregate,
                        "integrity_validation_status": "NOT_RUN",
                        "recording_usable": False,
                    },
                    priority=True,
                )
            except Exception:
                pass
        return dict(row)

    def video_recording_start_snapshot(self) -> dict[str, object]:
        snapshot = getattr(self.video_state, "_video_recording_start_snapshot", None)
        if not isinstance(snapshot, dict):
            return {
                "schema": "workstation-video-start-v1",
                "state": "IDLE",
                "detail": "video recording is not requested",
                "sides": {},
            }
        return copy.deepcopy(snapshot)

    def _set_video_recording_start_snapshot(
        self,
        sides: Mapping[str, Mapping[str, object]] | None,
        *,
        detail: str = "",
        refresh: bool = True,
    ) -> dict[str, object]:
        normalized_sides: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            raw = (sides or {}).get(side)
            if not isinstance(raw, Mapping):
                continue
            state = str(raw.get("state") or "FAILED").upper()
            if state not in {"WAITING_FOR_CAMERA", "STARTING", "RUNNING", "FAILED"}:
                state = "FAILED"
            normalized_sides[side] = {
                "state": state,
                "path": str(raw.get("path") or ""),
                "size_bytes": max(0, int(raw.get("size_bytes") or 0)),
                "detail": str(raw.get("detail") or ""),
            }
        states = tuple(row["state"] for row in normalized_sides.values())
        if not states:
            aggregate = "IDLE"
        elif any(state in {"WAITING_FOR_CAMERA", "STARTING"} for state in states):
            aggregate = "STARTING"
        elif all(state == "RUNNING" for state in states):
            aggregate = "RUNNING"
        elif any(state == "RUNNING" for state in states):
            aggregate = "PARTIAL"
        else:
            aggregate = "FAILED"
        snapshot = {
            "schema": "workstation-video-start-v1",
            "state": aggregate,
            "detail": str(detail),
            "sides": normalized_sides,
            "updated_monotonic_s": time.monotonic(),
        }
        previous = self.video_recording_start_snapshot()
        self.video_state._video_recording_start_snapshot = snapshot
        if aggregate == "FAILED" and not any(
            row["state"] == "RUNNING" for row in normalized_sides.values()
        ):
            # Distance/telemetry collection remains active.  This flag only
            # claims that a video branch has independently become usable.
            self.session.video_recording_enabled = False
        changed = (
            previous.get("state") != aggregate
            or previous.get("sides") != normalized_sides
            or previous.get("detail") != str(detail)
        )
        append = getattr(self.boundary, "append_workstation_data", None)
        if changed and callable(append):
            try:
                append(
                    "events",
                    {
                        "event_type": "video_start_state_changed",
                        "runtime_generation": str(
                            getattr(self.session, "recording_session_id", "") or ""
                        ),
                        "state": aggregate,
                        "detail": str(detail),
                        "sides": copy.deepcopy(normalized_sides),
                    },
                    priority=True,
                )
            except Exception:
                pass
        refresher = getattr(self.presentation, "_refresh_recording_status_label", None)
        if refresh and changed and callable(refresher):
            refresher()
        return self.video_recording_start_snapshot()
