from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass
from pathlib import Path


def session_id_now() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def session_dir(root: Path, session_id: str | None = None) -> Path:
    return root / "sessions" / (session_id or session_id_now())


def raw_video_paths(root: Path, session_id: str | None = None) -> dict[str, Path]:
    output_dir = session_dir(root, session_id)
    return {
        "left": output_dir / "video" / "raw_left.mp4",
        "right": output_dir / "video" / "raw_right.mp4",
    }


def format_video_paths(paths: dict[str, Path]) -> str:
    if not paths:
        return "저장 경로 없음"
    return ", ".join(f"{camera}={path}" for camera, path in paths.items())


@dataclass(frozen=True)
class SavedSessionFiles:
    metadata: Path
    distance_history: Path | None
    external_parameters: Path | None
    pixhawk_mag_status: Path | None
    uam_analysis: Path | None = None

    @property
    def outputs(self) -> list[Path]:
        return [
            path
            for path in (
                self.metadata,
                self.distance_history,
                self.external_parameters,
                self.pixhawk_mag_status,
                self.uam_analysis,
            )
            if path is not None
        ]


def write_dynamic_csv(output: Path, rows: list[dict[str, object]]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(dict.fromkeys(field for row in rows for field in row.keys()))
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(rows)


def save_session_files(
    output_dir: Path,
    *,
    metadata: dict[str, object],
    distance_rows: list[dict[str, object]] | None = None,
    external_parameter_rows: list[dict[str, object]] | None = None,
    pixhawk_mag_rows: list[dict[str, object]] | None = None,
    uam_analysis_rows: list[dict[str, object]] | None = None,
) -> SavedSessionFiles:
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata_output = output_dir / "metadata" / "session_metadata.json"
    metadata_output.parent.mkdir(parents=True, exist_ok=True)
    metadata_output.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    distance_output = None
    if distance_rows:
        distance_output = output_dir / "measurements" / "distance_history.csv"
        write_dynamic_csv(distance_output, distance_rows)
    external_output = None
    if external_parameter_rows:
        external_output = output_dir / "alignment" / "external_parameters.csv"
        write_dynamic_csv(external_output, external_parameter_rows)
    mag_output = None
    if pixhawk_mag_rows:
        mag_output = output_dir / "alignment" / "pixhawk_mag_status.csv"
        write_dynamic_csv(mag_output, pixhawk_mag_rows)
    uam_analysis_output = None
    if uam_analysis_rows:
        uam_analysis_output = output_dir / "measurements" / "uam_analysis.csv"
        write_dynamic_csv(uam_analysis_output, uam_analysis_rows)
    return SavedSessionFiles(
        metadata=metadata_output,
        distance_history=distance_output,
        external_parameters=external_output,
        pixhawk_mag_status=mag_output,
        uam_analysis=uam_analysis_output,
    )
