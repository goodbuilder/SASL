"""Authoritative first-frame and MP4-growth readiness without UI access.

Native frames keep their producer stream_generation identity. Deadlines use
host monotonic seconds; accepted worker startup is distinct from observed
frames/file growth. No NED/body/camera coordinate conversion is performed.
The outer adapter retains pane effects, measurement publication and scheduling.
"""

from __future__ import annotations
from collections.abc import Mapping
from pathlib import Path
import time

READINESS_FIELDS = (
    "_camera_start_readiness_generation",
    "_camera_start_readiness",
    "_video_start_readiness_generation",
    "_video_start_readiness",
)


class StreamReadinessTracker:
    def __init__(self, publish_legacy=None):
        self._camera_start_readiness_generation = 0
        self._camera_start_readiness = None
        self._video_start_readiness_generation = 0
        self._video_start_readiness = None
        self._publish_legacy = publish_legacy

    def _publish(self):
        if self._publish_legacy is not None:
            self._publish_legacy(self)

    def cancel_camera(self):
        self._camera_start_readiness_generation = int(self._camera_start_readiness_generation) + 1
        self._camera_start_readiness = None
        self._publish()

    def cancel_video(self):
        self._video_start_readiness_generation = int(self._video_start_readiness_generation) + 1
        self._video_start_readiness = None
        self._publish()

    def begin_camera(self, started_sides, *, expected_stream_generations, enable_detection, switch_to_depth, timeout_s):
        self.cancel_camera()
        generation = int(self._camera_start_readiness_generation) + 1
        self._camera_start_readiness_generation = generation
        unavailable = tuple(side for side in ("left", "right") if side not in started_sides)
        self._camera_start_readiness = {
            "generation": generation,
            "started_sides": tuple(started_sides),
            "expected_stream_generations": dict(expected_stream_generations),
            "frame_ready_sides": set(),
            "unverified_frame_sides": set(),
            "failed_sides": {side: "camera worker did not accept start" for side in unavailable},
            "enable_detection": bool(enable_detection),
            "switch_to_depth": bool(switch_to_depth),
            "deadline_monotonic_s": time.monotonic() + timeout_s,
        }
        self._publish()
        return self._camera_start_readiness, unavailable

    def begin_video(self, started_sides, *, enabled, paths, baselines, timeout_s):
        self.cancel_video()
        if not enabled:
            return None
        paths = dict(paths or {})
        baselines = dict(baselines or {})
        rows: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            raw_path = paths.get(side)
            if raw_path is None:
                rows[side] = {
                    "state": "FAILED",
                    "path": "",
                    "size_bytes": 0,
                    "detail": "video path was not allocated",
                }
                continue
            path = Path(raw_path).expanduser()
            baseline = max(0, int(baselines.get(side, 0) or 0))
            if side not in started_sides:
                rows[side] = {
                    "state": "FAILED",
                    "path": str(path),
                    "size_bytes": baseline,
                    "detail": "camera worker did not accept start",
                }
                continue
            rows[side] = {
                "state": "STARTING",
                "path": str(path),
                "size_bytes": baseline,
                "detail": "waiting for positive MP4 byte growth",
            }
        generation = int(self._video_start_readiness_generation) + 1
        self._video_start_readiness_generation = generation
        self._video_start_readiness = {
            "generation": generation,
            "deadline_monotonic_s": time.monotonic() + timeout_s,
            "baseline_sizes": {side: max(0, int(baselines.get(side, 0) or 0)) for side in ("left", "right")},
            "rows": rows,
        }
        self._publish()
        return self._video_start_readiness

    def poll_video(self, generation: int, *, timeout_s):
        readiness = self._video_start_readiness
        if not isinstance(readiness, dict) or int(readiness.get("generation", -1)) != int(generation):
            return
        rows = {side: dict(row) for side, row in dict(readiness.get("rows") or {}).items() if isinstance(row, Mapping)}
        baselines = dict(readiness.get("baseline_sizes") or {})
        for side, row in rows.items():
            if row.get("state") != "STARTING":
                continue
            path = Path(str(row.get("path") or "")).expanduser()
            try:
                size_bytes = path.stat().st_size if path.is_file() else 0
            except OSError as exc:
                row["detail"] = f"video path stat failed: {exc}"
                size_bytes = 0
            row["size_bytes"] = max(0, int(size_bytes))
            if size_bytes > max(0, int(baselines.get(side, 0) or 0)):
                row["state"] = "RUNNING"
                row["detail"] = "positive MP4 byte growth observed"
        timed_out = time.monotonic() >= float(readiness.get("deadline_monotonic_s") or 0.0)
        if timed_out:
            for row in rows.values():
                if row.get("state") == "STARTING":
                    row["state"] = "FAILED"
                    row["detail"] = f"no positive MP4 byte growth within {timeout_s:.1f}s"
        readiness["rows"] = rows
        pending = any(row.get("state") == "STARTING" for row in rows.values())
        return readiness, rows, timed_out, pending

    def finish_video(self, readiness):
        if readiness is self._video_start_readiness:
            self._video_start_readiness = None
            self._publish()

    def fail_video_side(self, side: str, reason: str):
        readiness = self._video_start_readiness
        if not isinstance(readiness, dict):
            return
        rows = readiness.get("rows")
        if not isinstance(rows, dict) or side not in rows:
            return
        row = dict(rows[side])
        if row.get("state") == "RUNNING":
            return
        row["state"] = "FAILED"
        row["detail"] = str(reason)
        rows[side] = row
        return int(readiness["generation"])

    def fail_camera_side(self, camera: str, reason: str):
        readiness = self._camera_start_readiness
        if not isinstance(readiness, dict) or camera not in readiness.get("started_sides", ()):
            return
        failed = readiness.get("failed_sides")
        if not isinstance(failed, dict):
            failed = {}
            readiness["failed_sides"] = failed
        if camera in failed:
            return
        failed[camera] = str(reason)
        return readiness

    def evaluate_camera(self, readiness):
        if readiness is not self._camera_start_readiness:
            return
        started = tuple(str(side) for side in readiness.get("started_sides", ()))
        failed = dict(readiness.get("failed_sides") or {})
        ready = tuple(
            side
            for side in ("left", "right")
            if side in set(readiness.get("frame_ready_sides") or ()) and side in started and side not in failed
        )
        pending = tuple(side for side in started if side not in ready and side not in failed)
        return ready, pending, failed

    def admit_frame(self, camera: str, frame_metadata, observed_generation: int):
        readiness = self._camera_start_readiness
        if isinstance(readiness, dict) and frame_metadata.get("media_integrity_unverified") is not None:
            unverified = readiness.get("unverified_frame_sides")
            if not isinstance(unverified, set):
                unverified = set(unverified or ())
                readiness["unverified_frame_sides"] = unverified
            if bool(frame_metadata.get("media_integrity_unverified")):
                unverified.add(camera)
            else:
                unverified.discard(camera)
        if not isinstance(readiness, dict) or camera not in readiness.get("started_sides", ()):
            return None
        if camera in dict(readiness.get("failed_sides") or {}):
            return False
        try:
            expected_generation = int(dict(readiness.get("expected_stream_generations") or {})[camera])
        except (KeyError, TypeError, ValueError):
            return False
        if observed_generation != expected_generation:
            return False
        ready = readiness.get("frame_ready_sides")
        if not isinstance(ready, set):
            ready = set(ready or ())
            readiness["frame_ready_sides"] = ready
        ready.add(camera)
        return True

    def expire_camera(self, generation: int, *, timeout_s: float):
        readiness = self._camera_start_readiness
        if not isinstance(readiness, dict) or int(readiness.get("generation", -1)) != int(generation):
            return None
        if time.monotonic() < float(readiness.get("deadline_monotonic_s") or 0.0):
            return None
        ready = set(readiness.get("frame_ready_sides") or ())
        failed = readiness.get("failed_sides")
        if not isinstance(failed, dict):
            failed = {}
            readiness["failed_sides"] = failed
        expired = []
        for side in tuple(readiness.get("started_sides", ())):
            if side not in ready and side not in failed:
                failed[side] = f"first decoded frame timeout after {timeout_s:.1f}s"
                expired.append((side, failed[side]))
        return tuple(expired)

    def fail_pending_video(self, failure: str):
        video = self._video_start_readiness
        if not isinstance(video, dict):
            return None
        rows = video.get("rows")
        if not isinstance(rows, dict):
            return None
        for side, raw in tuple(rows.items()):
            row = dict(raw) if isinstance(raw, Mapping) else {}
            if row.get("state") != "RUNNING":
                row["state"] = "FAILED"
                row["detail"] = failure
                rows[side] = row
        return rows
