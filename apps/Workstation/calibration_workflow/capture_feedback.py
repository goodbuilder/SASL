"""Capture feedback counters and persisted-inventory observation.

Owns the current capture-root feedback only; no panel, widgets, camera
commands, solver configuration or frame conversion is accessible here.
"""

from __future__ import annotations

from pathlib import Path
from Workstation.calibration_workflow.calibration_sessions import scan_calibration_capture_inventory


class CalibrationCaptureFeedback:
    def __init__(self) -> None:
        self._capture_feedback_root_text = ""
        self._capture_feedback_run = self._empty_capture_feedback_run()
        self._capture_feedback_last = {"state": "IDLE", "capture_id": "", "origin": ""}
        self._capture_feedback_inventory_stamp = None
        self._capture_feedback_inventory = None

    @staticmethod
    def _empty_capture_feedback_run() -> dict[str, int]:
        return {
            "inspection_request_count": 0,
            "stereo_saved_count": 0,
            "intrinsics_only_saved_count": 0,
            "failed_count": 0,
            "inflight_count": 0,
        }

    def _ensure_capture_feedback_state(self) -> None:
        """Keep legacy/lightweight controller construction compatible."""

        if not isinstance(getattr(self, "_capture_feedback_run", None), dict):
            self._capture_feedback_run = self._empty_capture_feedback_run()
        if not isinstance(getattr(self, "_capture_feedback_last", None), dict):
            self._capture_feedback_last = {
                "state": "IDLE",
                "capture_id": "",
                "origin": "",
            }
        if not hasattr(self, "_capture_feedback_root_text"):
            self._capture_feedback_root_text = ""
        if not hasattr(self, "_capture_feedback_inventory_stamp"):
            self._capture_feedback_inventory_stamp = None
        if not hasattr(self, "_capture_feedback_inventory"):
            self._capture_feedback_inventory = None

    @staticmethod
    def _capture_feedback_root_id(root: Path) -> str:
        if root.name == "AutoCapture" and root.parent.name == "calib_capture":
            return f"{root.parent.parent.name}/AutoCapture"
        return root.name or str(root)

    def _capture_feedback_prepare_root(self, root: str | Path) -> Path:
        self._ensure_capture_feedback_state()
        resolved = Path(root).expanduser().resolve(strict=False)
        root_text = str(resolved)
        if root_text != self._capture_feedback_root_text:
            self._capture_feedback_root_text = root_text
            self._capture_feedback_run = self._empty_capture_feedback_run()
            self._capture_feedback_inventory_stamp = None
            self._capture_feedback_inventory = None
            self._capture_feedback_last = {
                "state": "IDLE",
                "capture_id": "",
                "origin": "",
            }
        return resolved

    def _capture_feedback_reset_run(self, *, origin: str) -> None:
        """Start a new visible run without changing persisted capture inventory."""

        self._ensure_capture_feedback_state()
        self._capture_feedback_run = self._empty_capture_feedback_run()
        self._capture_feedback_last = {
            "state": "WAITING",
            "capture_id": "",
            "origin": str(origin),
        }

    def _capture_feedback_begin(
        self,
        *,
        root: str | Path,
        capture_id: str,
        origin: str,
    ) -> None:
        self._capture_feedback_prepare_root(root)
        self._capture_feedback_run["inspection_request_count"] += 1
        self._capture_feedback_run["inflight_count"] += 1
        self._capture_feedback_last = {
            "state": "INFLIGHT",
            "capture_id": str(capture_id),
            "origin": str(origin),
        }

    def _capture_feedback_complete(
        self,
        *,
        capture_id: str,
        origin: str,
        saved_sides: tuple[str, ...],
        stereo_pair_saved: bool,
    ) -> None:
        self._ensure_capture_feedback_state()
        counted = self._capture_feedback_run["inflight_count"] > 0
        if counted:
            self._capture_feedback_run["inflight_count"] -= 1
            if stereo_pair_saved:
                self._capture_feedback_run["stereo_saved_count"] += 1
                state = "STEREO_SAVED"
            elif saved_sides:
                self._capture_feedback_run["intrinsics_only_saved_count"] += 1
                state = "INTRINSICS_ONLY_SAVED"
            else:
                self._capture_feedback_run["failed_count"] += 1
                state = "FAILED"
        else:
            state = "LATE_RESULT"
        self._capture_feedback_last = {
            "state": state,
            "capture_id": str(capture_id),
            "origin": str(origin),
        }
        self._capture_feedback_inventory_stamp = None

    def _capture_feedback_failed(self, *, origin: str) -> None:
        self._ensure_capture_feedback_state()
        counted = self._capture_feedback_run["inflight_count"] > 0
        if counted:
            self._capture_feedback_run["inflight_count"] -= 1
            self._capture_feedback_run["failed_count"] += 1
        self._capture_feedback_last = {
            "state": "FAILED" if counted else "LATE_FAILURE",
            "capture_id": str(self._capture_feedback_last.get("capture_id") or ""),
            "origin": str(origin),
        }
        self._capture_feedback_inventory_stamp = None

    @staticmethod
    def _capture_inventory_stamp(root: Path) -> tuple[object, ...]:
        stamp: list[object] = [str(root)]
        for path in (root, root / "left", root / "right", root / "capture_metadata"):
            try:
                stat = path.stat()
            except FileNotFoundError:
                stamp.append((str(path), "missing"))
            except OSError as exc:
                stamp.append((str(path), "error", type(exc).__name__, str(exc)))
            else:
                stamp.append(
                    (
                        str(path),
                        "dir" if path.is_dir() else "file",
                        int(stat.st_mtime_ns),
                        int(stat.st_size),
                    )
                )
        return tuple(stamp)

    def _capture_feedback_inventory_for(self, root: Path) -> dict[str, object]:
        stamp = self._capture_inventory_stamp(root)
        if self._capture_feedback_inventory is None or self._capture_feedback_inventory_stamp != stamp:
            self._capture_feedback_inventory = scan_calibration_capture_inventory(root).to_mapping()
            self._capture_feedback_inventory_stamp = stamp
        return dict(self._capture_feedback_inventory)
