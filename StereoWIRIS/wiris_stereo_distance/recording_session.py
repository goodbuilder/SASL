from __future__ import annotations

import csv
import json
import os
import shutil
import tempfile
import threading
import time
from collections import deque
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

import xlsxwriter


PAIR_AXIS_COLUMNS = ("sample_index", "session_elapsed_s")
DEVICE_TIMELINE_COLUMNS = (
    "sequence",
    "session_id",
    "side",
    "sample_type",
    "row_contract",
    "sample_monotonic_ns",
    "sample_monotonic_s",
    "session_elapsed_s",
    "session_utc_ns",
    "session_utc_s",
    "session_utc_method",
    "session_utc_anchor_uncertainty_ns",
    "source_utc_ns",
    "source_utc_s",
    "source_utc_valid",
    "source_utc_method",
    "host_receive_monotonic_ns",
    "host_receive_utc_ns",
)
DYNAMIC_SHEET_RULES: dict[str, tuple[str, ...]] = {
    "Measurements": (
        "measurement",
        "distance",
        "object",
        "class",
        "track",
        "frame",
        "utc",
        "timestamp",
    ),
    "GPS": ("gps", "latitude", "longitude", "lat", "lon", "altitude", "alt"),
    "Attitude": (
        "attitude",
        "roll",
        "pitch",
        "yaw",
        "quaternion",
        "heading",
        "px4_timestamp",
    ),
    "Baseline": ("baseline",),
    "Quality": (
        "quality",
        "confidence",
        "valid",
        "status",
        "error",
        "delta",
        "sync",
        "matched",
        "interpolation",
    ),
}
CALIBRATION_KEYWORDS = (
    "calibration",
    "intrinsic",
    "extrinsic",
    "distortion",
    "camera_matrix",
    "projection_matrix",
    "rectification",
)


_SESSION_ID_LOCK = threading.Lock()
_LAST_SESSION_ID_NS = 0


def session_id_now() -> str:
    """Return a sortable session id that cannot collide within one process."""
    global _LAST_SESSION_ID_NS
    with _SESSION_ID_LOCK:
        now_ns = max(time.time_ns(), _LAST_SESSION_ID_NS + 1)
        _LAST_SESSION_ID_NS = now_ns
    stamp = datetime.fromtimestamp(now_ns / 1_000_000_000)
    return f"{stamp:%Y%m%d_%H%M%S}_{now_ns % 1_000_000_000:09d}"


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
    csv: Path
    xlsx: Path
    camera_left_csv: Path
    camera_right_csv: Path
    px4_left_csv: Path
    px4_right_csv: Path
    manifest: Path
    finalized: bool = True
    error: str = ""

    @property
    def outputs(self) -> list[Path]:
        return [
            self.csv,
            self.xlsx,
            self.camera_left_csv,
            self.camera_right_csv,
            self.px4_left_csv,
            self.px4_right_csv,
        ]


@dataclass(frozen=True)
class RetryableSession:
    output_dir: Path
    metadata: dict[str, object]
    pair_rows: "SpoolRows"
    camera_rows: dict[str, "SpoolRows"]
    px4_rows: dict[str, "SpoolRows"]


class SpoolRows(Sequence[dict[str, object]]):
    """Disk-backed append-only rows with a bounded in-memory preview."""

    def __init__(self, *, preview_limit: int = 256, supports_updates: bool = False) -> None:
        self.preview_limit = max(1, int(preview_limit))
        self.supports_updates = supports_updates
        self._preview: deque[dict[str, object]] = deque(maxlen=self.preview_limit)
        self._path: Path | None = None
        self._handle = None
        self._count = 0
        self._lock = threading.RLock()

    @property
    def spool_path(self) -> Path | None:
        return self._path

    @property
    def preview_count(self) -> int:
        return len(self._preview)

    def bind(self, path: Path) -> None:
        with self._lock:
            self.close()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.unlink(missing_ok=True)
            self._path = path
            self._handle = path.open("a", encoding="utf-8", buffering=1)
            self._preview.clear()
            self._count = 0

    def open_existing(self, path: Path) -> None:
        with self._lock:
            self.close()
            self._path = path
            self._preview.clear()
            self._count = 0
            if path.exists():
                for row in self._materialized_rows() if self.supports_updates else self._plain_rows():
                    self._preview.append(row)
                    self._count += 1
            path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = path.open("a", encoding="utf-8", buffering=1)

    def append(self, row: dict[str, object]) -> None:
        stored = {key: _serializable_value(value) for key, value in row.items()}
        with self._lock:
            if self._handle is not None:
                self._handle.write(json.dumps(stored, ensure_ascii=False, sort_keys=True) + "\n")
            self._preview.append(stored)
            self._count += 1

    def replace(self, index: int, row: dict[str, object]) -> None:
        if not self.supports_updates:
            raise TypeError("this spool does not support row replacement")
        resolved = index if index >= 0 else self._count + index
        if resolved < 0 or resolved >= self._count:
            raise IndexError(index)
        stored = {key: _serializable_value(value) for key, value in row.items()}
        record = {"__spool_update_index": resolved, "__spool_row": stored}
        with self._lock:
            if self._handle is not None:
                self._handle.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
            preview_start = self._count - len(self._preview)
            if resolved >= preview_start:
                self._preview[resolved - preview_start] = stored

    def pop(self, index: int = -1) -> dict[str, object]:
        rows = list(self)
        removed = rows.pop(index)
        self._rewrite(rows)
        return removed

    def close(self) -> None:
        with self._lock:
            if self._handle is not None:
                self._handle.flush()
                os.fsync(self._handle.fileno())
                self._handle.close()
                self._handle = None

    def flush(self) -> None:
        with self._lock:
            if self._handle is not None:
                self._handle.flush()

    def clear(self) -> None:
        with self._lock:
            self.close()
            if self._path is not None:
                self._path.unlink(missing_ok=True)
            self._path = None
            self._preview.clear()
            self._count = 0

    def discard_spool(self) -> None:
        """Remove the durable spool after every final artifact is committed."""
        with self._lock:
            self.close()
            path = self._path
            if path is not None:
                path.unlink(missing_ok=True)
                try:
                    path.parent.rmdir()
                except OSError:
                    pass
            self._path = None

    def __len__(self) -> int:
        return self._count

    @property
    def preview_rows(self) -> list[dict[str, object]]:
        return list(self._preview)

    def _plain_rows(self) -> Iterator[dict[str, object]]:
        path = self._path
        if path is None or not path.exists():
            yield from self._preview
            return
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                if not line.strip():
                    continue
                row = json.loads(line)
                if "__spool_update_index" not in row:
                    yield row

    def _materialized_rows(self) -> list[dict[str, object]]:
        path = self._path
        if path is None or not path.exists():
            return list(self._preview)
        rows: list[dict[str, object]] = []
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                if not line.strip():
                    continue
                record = json.loads(line)
                if "__spool_update_index" in record:
                    rows[int(record["__spool_update_index"])] = record["__spool_row"]
                else:
                    rows.append(record)
        return rows

    def _rewrite(self, rows: list[dict[str, object]]) -> None:
        path = self._path
        self.close()
        self._preview.clear()
        self._count = 0
        if path is None:
            for row in rows:
                self._preview.append(row)
                self._count += 1
            return
        path.unlink(missing_ok=True)
        self._handle = path.open("a", encoding="utf-8", buffering=1)
        for row in rows:
            self.append(row)

    def __iter__(self) -> Iterator[dict[str, object]]:
        with self._lock:
            self.flush()
            path = self._path
            preview = list(self._preview)
            count = self._count
        if path is None or not path.exists():
            yield from preview
            return
        yielded = 0
        source = self._materialized_rows() if self.supports_updates else self._plain_rows()
        for row in source:
            yielded += 1
            yield row
        if yielded != count:
            raise OSError(f"spool row count mismatch: expected={count}, read={yielded}, path={path}")

    def __getitem__(self, index):
        if isinstance(index, slice):
            return list(self)[index]
        resolved = index if index >= 0 else self._count + index
        if resolved < 0 or resolved >= self._count:
            raise IndexError(index)
        preview_start = self._count - len(self._preview)
        if resolved >= preview_start:
            return list(self._preview)[resolved - preview_start]
        for current, row in enumerate(self):
            if current == resolved:
                return row
        raise IndexError(index)

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Sequence):
            return list(self) == list(other)
        return False


def _ordered_fieldnames(
    rows: Iterable[dict[str, object]],
    required_columns: Iterable[str] = PAIR_AXIS_COLUMNS,
) -> list[str]:
    observed = dict.fromkeys(required_columns)
    for row in rows:
        observed.update(dict.fromkeys(row))
    return list(observed)


def _serializable_value(value: object) -> object:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    return json.dumps(value, ensure_ascii=False, sort_keys=True)


def _normalized_pair_rows(pair_rows: Iterable[dict[str, object]]) -> list[dict[str, object]]:
    normalized: list[dict[str, object]] = []
    for index, row in enumerate(pair_rows):
        normalized.append(
            {
                **{key: _serializable_value(value) for key, value in row.items()},
                "sample_index": _serializable_value(row.get("sample_index", index)),
                "session_elapsed_s": _serializable_value(row.get("session_elapsed_s")),
            }
        )
    return normalized


def _projected_columns(fieldnames: list[str], keywords: tuple[str, ...]) -> list[str]:
    selected = list(PAIR_AXIS_COLUMNS)
    selected.extend(
        field
        for field in fieldnames
        if field not in PAIR_AXIS_COLUMNS
        and any(keyword in field.lower() for keyword in keywords)
    )
    return selected


def _flatten_metadata(
    metadata: dict[str, object], prefix: str = ""
) -> list[tuple[str, object]]:
    flattened: list[tuple[str, object]] = []
    for key, value in metadata.items():
        qualified_key = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            flattened.extend(_flatten_metadata(value, qualified_key))
        else:
            flattened.append((qualified_key, _serializable_value(value)))
    return flattened


def _write_table(
    worksheet: xlsxwriter.worksheet.Worksheet,
    columns: list[str],
    rows: Iterable[dict[str, object]],
    header_format: xlsxwriter.format.Format,
) -> None:
    worksheet.freeze_panes(1, 0)
    for column_index, name in enumerate(columns):
        worksheet.write(0, column_index, name, header_format)
    for row_index, row in enumerate(rows, start=1):
        for column_index, name in enumerate(columns):
            value = row.get(name)
            if value is not None:
                worksheet.write(row_index, column_index, value)
    for column_index, name in enumerate(columns):
        worksheet.set_column(
            column_index, column_index, max(14, min(32, len(name) + 2))
        )


def _write_key_value_sheet(
    worksheet: xlsxwriter.worksheet.Worksheet,
    entries: Iterable[tuple[str, object]],
    header_format: xlsxwriter.format.Format,
) -> None:
    worksheet.write_row(0, 0, ["key", "value"], header_format)
    for row_index, (key, value) in enumerate(entries, start=1):
        worksheet.write(row_index, 0, key)
        if value is not None:
            worksheet.write(row_index, 1, value)
    worksheet.set_column(0, 0, 36)
    worksheet.set_column(1, 1, 64)


def write_dynamic_csv(
    output: Path,
    rows: Iterable[dict[str, object]],
    *,
    required_columns: Iterable[str] = PAIR_AXIS_COLUMNS,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if not isinstance(rows, Sequence):
        rows = list(rows)
    fieldnames = _ordered_fieldnames(rows, required_columns)
    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w", newline="", encoding="utf-8", dir=output.parent,
            prefix=f".{output.name}.", suffix=".tmp", delete=False,
        ) as handle:
            temp_path = Path(handle.name)
            writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, output)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def _write_xlsx(
    output: Path,
    metadata: dict[str, object],
    rows: list[dict[str, object]],
) -> None:
    fieldnames = _ordered_fieldnames(rows)
    metadata_entries = _flatten_metadata(metadata)
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(dir=output.parent, prefix=f".{output.name}.", suffix=".tmp")
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        with xlsxwriter.Workbook(temp_path) as workbook:
            header_format = workbook.add_format({"bold": True, "bg_color": "#D9EAF7"})
            _write_key_value_sheet(
                workbook.add_worksheet("Summary"), metadata_entries, header_format
            )
            for sheet_name, keywords in DYNAMIC_SHEET_RULES.items():
                columns = _projected_columns(fieldnames, keywords)
                _write_table(workbook.add_worksheet(sheet_name), columns, rows, header_format)
            calibration_entries = [
                entry
                for entry in metadata_entries
                if any(keyword in entry[0].lower() for keyword in CALIBRATION_KEYWORDS)
            ]
            _write_key_value_sheet(
                workbook.add_worksheet("Calibration"),
                calibration_entries,
                header_format,
            )
        os.replace(temp_path, output)
    finally:
        temp_path.unlink(missing_ok=True)


def _write_manifest(path: Path, payload: dict[str, object]) -> None:
    temp_path: Path | None = None
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=path.parent,
            prefix=f".{path.name}.", suffix=".tmp", delete=False,
        ) as handle:
            temp_path = Path(handle.name)
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, path)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def write_recovery_metadata(output_dir: Path, metadata: dict[str, object]) -> Path:
    path = output_dir / ".spool" / "session_metadata.json"
    _write_manifest(path, metadata)
    return path


def discover_retryable_sessions(recordings_root: Path) -> list[Path]:
    sessions_root = recordings_root / "sessions"
    if not sessions_root.is_dir():
        return []
    retryable: list[Path] = []
    for output_dir in sessions_root.iterdir():
        if not output_dir.is_dir() or not (output_dir / ".spool").is_dir():
            continue
        manifest_path = output_dir / "session_manifest.json"
        status = ""
        if manifest_path.is_file():
            try:
                status = str(json.loads(manifest_path.read_text(encoding="utf-8")).get("status", ""))
            except (OSError, ValueError):
                status = "failed"
        if status != "finalized":
            retryable.append(output_dir)
    return sorted(retryable, key=lambda path: path.name, reverse=True)


def load_retryable_session(output_dir: Path) -> RetryableSession:
    spool_dir = output_dir / ".spool"
    metadata_path = spool_dir / "session_metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    pair_rows = SpoolRows(supports_updates=True)
    pair_rows.open_existing(spool_dir / "pair_rows.jsonl")
    camera_rows: dict[str, SpoolRows] = {}
    px4_rows: dict[str, SpoolRows] = {}
    for side in ("left", "right"):
        camera_rows[side] = SpoolRows()
        camera_rows[side].open_existing(spool_dir / f"camera_{side}.jsonl")
        px4_rows[side] = SpoolRows()
        px4_rows[side].open_existing(spool_dir / f"px4_{side}.jsonl")
    return RetryableSession(output_dir, metadata, pair_rows, camera_rows, px4_rows)


def _cleanup_finalize_orphans(output_dir: Path) -> None:
    for path in output_dir.glob(".finalize-*"):
        if path.is_dir():
            shutil.rmtree(path, ignore_errors=True)
    generations = output_dir / ".generations"
    if generations.is_dir():
        published_name = ""
        published = output_dir / "published"
        if published.is_symlink():
            published_name = Path(os.readlink(published)).name
        for path in generations.glob("*.preparing"):
            shutil.rmtree(path, ignore_errors=True)
        for path in generations.iterdir():
            if path.is_dir() and not path.name.endswith(".preparing") and path.name != published_name:
                shutil.rmtree(path, ignore_errors=True)


def _ensure_stable_artifact_links(output_dir: Path, names: Iterable[str]) -> None:
    names = list(names)
    regular_paths = [output_dir / name for name in names if (output_dir / name).exists() and not (output_dir / name).is_symlink()]
    if regular_paths:
        generation_id = f"legacy-{session_id_now()}"
        legacy_dir = output_dir / ".generations" / generation_id
        legacy_dir.mkdir(parents=True, exist_ok=False)
        for path in regular_paths:
            shutil.copy2(path, legacy_dir / path.name)
        pointer_tmp = output_dir / f".published-{generation_id}.tmp"
        pointer_tmp.symlink_to(Path(".generations") / generation_id)
        os.replace(pointer_tmp, output_dir / "published")
        for path in regular_paths:
            link_tmp = output_dir / f".{path.name}.link.tmp"
            link_tmp.unlink(missing_ok=True)
            link_tmp.symlink_to(Path("published") / path.name)
            os.replace(link_tmp, path)
    for name in names:
        path = output_dir / name
        if os.path.lexists(path):
            if path.is_symlink():
                continue
            raise OSError(f"stable artifact path is not a symlink: {path}")
        path.symlink_to(Path("published") / name)


def _publish_generation(output_dir: Path, generation_dir: Path, names: Iterable[str]) -> None:
    _ensure_stable_artifact_links(output_dir, names)
    temp_link = output_dir / f".published-{generation_dir.name}.tmp"
    temp_link.unlink(missing_ok=True)
    temp_link.symlink_to(Path(".generations") / generation_dir.name)
    try:
        os.replace(temp_link, output_dir / "published")
    finally:
        temp_link.unlink(missing_ok=True)


def save_session_files(
    output_dir: Path,
    *,
    metadata: dict[str, object],
    pair_rows: Iterable[dict[str, object]],
    camera_rows: dict[str, Iterable[dict[str, object]]] | None = None,
    px4_rows: dict[str, Iterable[dict[str, object]]] | None = None,
) -> SavedSessionFiles:
    output_dir.mkdir(parents=True, exist_ok=True)
    session_id = Path(str(metadata.get("session_id") or output_dir.name)).name
    rows = _normalized_pair_rows(pair_rows)
    csv_output = output_dir / f"{session_id}.csv"
    xlsx_output = output_dir / f"{session_id}.xlsx"
    camera_left_output = output_dir / "camera_left.csv"
    camera_right_output = output_dir / "camera_right.csv"
    px4_left_output = output_dir / "px4_left.csv"
    px4_right_output = output_dir / "px4_right.csv"
    manifest_output = output_dir / "session_manifest.json"
    camera_rows = camera_rows or {}
    px4_rows = px4_rows or {}
    _cleanup_finalize_orphans(output_dir)
    generation_id = session_id_now()
    generations_root = output_dir / ".generations"
    generations_root.mkdir(parents=True, exist_ok=True)
    staging_dir = generations_root / f"{generation_id}.preparing"
    staging_dir.mkdir()
    csv_names_and_rows = (
        (csv_output.name, rows, PAIR_AXIS_COLUMNS),
        (camera_left_output.name, camera_rows.get("left", []), DEVICE_TIMELINE_COLUMNS),
        (camera_right_output.name, camera_rows.get("right", []), DEVICE_TIMELINE_COLUMNS),
        (px4_left_output.name, px4_rows.get("left", []), DEVICE_TIMELINE_COLUMNS),
        (px4_right_output.name, px4_rows.get("right", []), DEVICE_TIMELINE_COLUMNS),
    )
    manifest: dict[str, object] = {
        "session_id": session_id,
        "status": "preparing",
        "csv_authoritative": True,
        "artifacts": {},
    }
    try:
        for name, source_rows, required in csv_names_and_rows:
            staged_path = staging_dir / name
            write_dynamic_csv(staged_path, source_rows, required_columns=required)
            manifest["artifacts"][name] = "prepared"  # type: ignore[index]
    except Exception as exc:
        manifest.update(
            status="failed",
            phase="prepare_csv",
            error=f"CSV preparation failed: {exc}",
            retryable=True,
            staging_dir=str(staging_dir),
        )
        _write_manifest(manifest_output, manifest)
        raise

    manifest.update(status="prepared", phase="prepare_xlsx", generation_id=generation_id)
    _write_manifest(manifest_output, manifest)
    error = ""
    try:
        _write_xlsx(staging_dir / xlsx_output.name, metadata, rows)
        manifest["artifacts"][xlsx_output.name] = "prepared"  # type: ignore[index]
    except Exception as exc:
        error = f"XLSX finalization failed: {exc}"
        manifest["artifacts"][xlsx_output.name] = "failed"  # type: ignore[index]
    generation_dir = generations_root / generation_id
    generation_manifest = dict(manifest)
    generation_manifest.update(
        status="prepared", phase="publish", error=error, retryable=bool(error)
    )
    _write_manifest(staging_dir / "generation_manifest.json", generation_manifest)
    os.replace(staging_dir, generation_dir)
    try:
        _publish_generation(
            output_dir,
            generation_dir,
            [name for name, _rows, _required in csv_names_and_rows] + [xlsx_output.name],
        )
    except Exception as exc:
        manifest.update(
            status="failed", phase="publish", error=f"generation publish failed: {exc}",
            retryable=True, generation_id=generation_id,
        )
        _write_manifest(manifest_output, manifest)
        raise
    manifest.update(
        status="failed" if error else "finalized",
        phase="published",
        error=error,
        retryable=bool(error),
        generation_id=generation_id,
        published_pointer="published",
        publish_contract="all stable artifact links resolve through one atomically replaced generation pointer",
    )
    _write_manifest(manifest_output, manifest)
    saved = SavedSessionFiles(
        csv=csv_output,
        xlsx=xlsx_output,
        camera_left_csv=camera_left_output,
        camera_right_csv=camera_right_output,
        px4_left_csv=px4_left_output,
        px4_right_csv=px4_right_output,
        manifest=manifest_output,
        finalized=not error,
        error=error,
    )
    if saved.finalized:
        for source_rows in (pair_rows, *camera_rows.values(), *px4_rows.values()):
            if isinstance(source_rows, SpoolRows):
                source_rows.discard_spool()
        recovery_metadata = output_dir / ".spool" / "session_metadata.json"
        recovery_metadata.unlink(missing_ok=True)
        try:
            recovery_metadata.parent.rmdir()
        except OSError:
            pass
        _cleanup_finalize_orphans(output_dir)
    return saved
