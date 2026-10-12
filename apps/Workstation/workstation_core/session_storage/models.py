"""Session models responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from .spool import SpoolRows


@dataclass(frozen=True)
class SavedSessionFiles:
    context_json: Path
    measurement_pairs_parquet: Path
    camera_timeline_parquet: Path
    pixhawk_timeline_parquet: Path
    gnss_base_timeline_parquet: Path
    geometry_comparison_parquet: Path
    review_xlsx: Path
    manifest: Path
    authoritative_jsonl_files: tuple[Path, ...] = ()
    finalized: bool = True
    error: str = ""
    review_error: str = ""
    review_pending: bool = False

    @property
    def session_static_json(self) -> Path:
        """Compatibility alias for the v2 frozen measurement context."""

        return self.context_json

    @property
    def distance_measurements_parquet(self) -> Path:
        """Compatibility alias for the derived pair Parquet."""

        return self.measurement_pairs_parquet

    @property
    def xlsx(self) -> Path:
        """Compatibility alias for the one session-level review workbook."""

        return self.review_xlsx

    @property
    def outputs(self) -> list[Path]:
        return [
            self.context_json,
            *self.authoritative_jsonl_files,
            self.measurement_pairs_parquet,
            self.camera_timeline_parquet,
            self.pixhawk_timeline_parquet,
            self.gnss_base_timeline_parquet,
            self.geometry_comparison_parquet,
            self.review_xlsx,
        ]

    @property
    def camera_left_parquet(self) -> Path:
        """Legacy API alias; left/right are combined in one typed timeline."""

        return self.camera_timeline_parquet


@dataclass(frozen=True)
class RetryableSession:
    output_dir: Path
    metadata: dict[str, object]
    pair_rows: "SpoolRows"
    camera_rows: dict[str, "SpoolRows"]
    px4_rows: dict[str, "SpoolRows"]
    gnss_base_rows: "SpoolRows"
    extrinsics_comparison_rows: "SpoolRows"
    method_evaluation_bundle_rows: "SpoolRows" = field(
        default_factory=lambda: SpoolRows()
    )


@dataclass(frozen=True)
class SessionSheetPreview:
    """One sheet of the session workbook, in the order it is written.

    ``kind`` is ``"table"`` (row-per-sample, ``columns`` applies) or
    ``"key_value"`` (flattened metadata, ``entries`` applies).
    """

    name: str
    kind: str
    columns: tuple[str, ...] = ()
    entries: tuple[tuple[str, object], ...] = ()

# Preserve existing class/pickle identity at the named compatibility boundary.
SavedSessionFiles.__module__ = "Workstation.workstation_core.recording_session"
RetryableSession.__module__ = "Workstation.workstation_core.recording_session"
SessionSheetPreview.__module__ = "Workstation.workstation_core.recording_session"
