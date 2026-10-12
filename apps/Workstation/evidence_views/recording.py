"""Pure recording status projection from captured storage/lifecycle evidence."""

from collections.abc import Mapping
from dataclasses import dataclass

from .snapshot import snapshot_mapping, thaw_snapshot


@dataclass(frozen=True)
class RecordingInputs:
    hierarchy: bool
    active: bool
    identity: Mapping
    files: Mapping
    progress: Mapping
    sample_counts: Mapping
    storage: Mapping
    run: Mapping

    def __post_init__(self):
        for name in ("identity", "files", "progress", "sample_counts", "storage", "run"):
            object.__setattr__(self, name, snapshot_mapping(getattr(self, name)))

    def project(self):
        if not self.hierarchy:
            return thaw_snapshot(
                {
                    "state": "ACTIVE" if self.active else str(self.storage.get("state") or "IDLE"),
                    "active": self.active,
                    "measurement_state": str(self.run.get("state") or "IDLE"),
                    "recording_mode": self.identity["mode"],
                    "output_dir": str(self.storage.get("root") or ""),
                    "video_paths": dict(self.files["video_paths"]),
                    "artifacts": (),
                    "elapsed_s": self.progress["elapsed_s"],
                    "status_label": self.progress["status_label"],
                    "retry_required": False,
                    "finalized": False,
                    "storage": self.storage,
                    "sample_counts": self.sample_counts,
                    "source": "MeasurementDataStore + transient capture state",
                    "clock": "per-record UTC/source clocks",
                }
            )
        progress = self.progress
        state = (
            "FINALIZING"
            if progress["saving"]
            else "RETRYABLE"
            if progress["retry"]
            else "FINALIZED"
            if progress["finalized"]
            else "ACTIVE"
            if self.active
            else "IDLE"
        )
        return thaw_snapshot(
            {
                "state": state,
                "active": self.active,
                "session_id": self.identity["session_id"],
                "measurement_id": self.identity["measurement_id"],
                "recording_mode": self.identity["mode"],
                "output_dir": self.files["output_dir"],
                "video_paths": dict(self.files["video_paths"]),
                "artifacts": tuple(self.files["artifacts"]),
                "elapsed_s": progress["elapsed_s"],
                "status_label": progress["status_label"],
                "retry_required": progress["retry"],
                "finalized": progress["finalized"],
                "sample_counts": self.sample_counts,
                "recovered": progress["recovered"],
                "source": "session spool rows + recording session/save worker",
                "clock": "session monotonic anchor; wall-clock ID retained",
            }
        )
