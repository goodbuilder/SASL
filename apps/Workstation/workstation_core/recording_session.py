from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import shutil
import tempfile
import threading
import time
from collections import deque
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import Iterable

import pyarrow as pa
import pyarrow.parquet as pq
import xlsxwriter

from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_IDS,
    PIXHAWK_RTK_LIVE,
)


# Finalized structured records are canonical JSONL.  Typed Parquet tables and
# the operator-facing review.xlsx are reproducible derived views.
WRITE_PARQUET_ARTIFACTS = True

PAIR_AXIS_COLUMNS = ("sample_index", "session_elapsed_s")
DISTANCE_MEASUREMENT_COLUMNS = (
    "sample_index",
    "recording_time_ns",
    "estimated_utc_time_ns",
    "device_utc_time_ns",
    "pair_id",
    "measurement_valid",
    "pair_status",
    "class_name",
    "left_bbox_center_x_px",
    "left_bbox_center_y_px",
    "right_bbox_center_x_px",
    "right_bbox_center_y_px",
    "left_confidence",
    "right_confidence",
    "distance_m",
    "baseline_center_forward_m",
    "baseline_center_right_m",
    "baseline_center_down_m",
    "baseline_m",
    "sync_delta_ns",
    "quality",
    "warning_code",
)
CAMERA_TIMELINE_COMPACT_COLUMNS = (
    "sequence",
    "recording_time_ns",
    "estimated_utc_time_ns",
    "device_utc_time_ns",
    "image_width_px",
    "image_height_px",
    "timestamp_source",
    "rtp_timestamp",
    "rtp_ssrc",
    "rtp_marker_seq",
    "rtcp_sr_received_count",
    "rtcp_sr_rtp_timestamp",
    "rtcp_clock_rate_hz",
    "rtcp_sr_age_ns",
    "rtcp_sr_pc_offset_ns",
    "rtcp_receive_delay_ns",
    "rtcp_transport_delay_ns",
    "rtcp_camera_ntp_rate",
    "rtcp_camera_media_rate",
    "rtsp_transport",
    "timing_status",
)
PX4_TIMELINE_COMPACT_COLUMNS = (
    "sequence",
    "recording_time_ns",
    "session_utc_time_ns",
    "gps_utc_time_ns",
    "gps_utc_source",
    "sample_type",
    "topic",
    "attitude_rollspeed_rad_s",
    "attitude_pitchspeed_rad_s",
    "attitude_yawspeed_rad_s",
    "attitude_time_boot_ms",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "fix_type",
    "satellites_used",
    "eph_m",
    "epv_m",
    "gps1_latitude_deg",
    "gps1_longitude_deg",
    "gps1_altitude_m",
    "gps1_fix_type",
    "gps1_satellites_used",
    "gps1_eph_m",
    "gps1_epv_m",
    "gps1_heading_deg",
    "gps1_ground_speed_m_s",
    "gps1_yaw_deg",
    "gps1_heading_accuracy_deg",
    "gps2_latitude_deg",
    "gps2_longitude_deg",
    "gps2_altitude_m",
    "gps2_fix_type",
    "gps2_satellites_used",
    "gps2_eph_m",
    "gps2_epv_m",
    "gps2_heading_deg",
    "gps2_ground_speed_m_s",
    "gps2_yaw_deg",
    "gps2_heading_accuracy_deg",
    "heading_deg",
    "mag_sensor_slot",
    "mag_slot",
    "mag_cal_slot",
    "mag_value_units",
    "mag_x",
    "mag_y",
    "mag_z",
    "mag_vector_norm",
    "mag_temperature_c",
    "mag_device_id",
    "mag_node_id",
    "mag_priority",
    "mag_rotation",
    "mag_enabled",
    "mag_expected_selected",
    "mag_actual_selected",
    "mag_armed",
    "mag_status",
    "mag_warning_text",
    "mag_param_uavcan_enable",
    "mag_param_uavcan_sub_mag",
    "mag_param_ekf2_mag_type",
    "mag_param_snapshot_json",
    "mag_scaled_imu_x",
    "mag_scaled_imu_y",
    "mag_scaled_imu_z",
    "mag_scaled_imu_unit",
    "mag_scaled_imu2_x",
    "mag_scaled_imu2_y",
    "mag_scaled_imu2_z",
    "mag_scaled_imu2_unit",
    "mag_scaled_imu3_x",
    "mag_scaled_imu3_y",
    "mag_scaled_imu3_z",
    "mag_scaled_imu3_unit",
    "mag_scaled_imu_accel_x",
    "mag_scaled_imu_accel_y",
    "mag_scaled_imu_accel_z",
    "mag_scaled_imu_gyro_x",
    "mag_scaled_imu_gyro_y",
    "mag_scaled_imu_gyro_z",
    "mag_scaled_imu_time_boot_ms",
    "mag_scaled_imu2_accel_x",
    "mag_scaled_imu2_accel_y",
    "mag_scaled_imu2_accel_z",
    "mag_scaled_imu2_gyro_x",
    "mag_scaled_imu2_gyro_y",
    "mag_scaled_imu2_gyro_z",
    "mag_scaled_imu2_time_boot_ms",
    "mag_scaled_imu3_accel_x",
    "mag_scaled_imu3_accel_y",
    "mag_scaled_imu3_accel_z",
    "mag_scaled_imu3_gyro_x",
    "mag_scaled_imu3_gyro_y",
    "mag_scaled_imu3_gyro_z",
    "mag_scaled_imu3_time_boot_ms",
    "mavlink_message_name",
    "mavlink_time_boot_ms",
    "mavlink_time_usec",
    "mavlink_global_latitude_deg",
    "mavlink_global_longitude_deg",
    "mavlink_global_altitude_m",
    "mavlink_global_relative_altitude_m",
    "mavlink_global_velocity_n_m_s",
    "mavlink_global_velocity_e_m_s",
    "mavlink_global_velocity_d_m_s",
    "mavlink_global_heading_deg",
    "mavlink_local_x_m",
    "mavlink_local_y_m",
    "mavlink_local_z_m",
    "mavlink_local_velocity_x_m_s",
    "mavlink_local_velocity_y_m_s",
    "mavlink_local_velocity_z_m_s",
    "mavlink_vfr_airspeed_m_s",
    "mavlink_vfr_groundspeed_m_s",
    "mavlink_vfr_heading_deg",
    "mavlink_vfr_throttle_pct",
    "mavlink_vfr_altitude_m",
    "mavlink_vfr_climb_m_s",
    "mavlink_sys_voltage_battery_v",
    "mavlink_sys_current_battery_a",
    "mavlink_sys_battery_remaining_pct",
    "mavlink_sys_drop_rate_comm_pct",
    "mavlink_sys_errors_comm",
    "mavlink_battery_current_a",
    "mavlink_battery_remaining_pct",
    "mavlink_battery_temperature_c",
    "mavlink_battery_voltage_0_v",
    "mavlink_battery_voltage_1_v",
    "mavlink_battery_voltage_2_v",
    "mavlink_raw_accel_x",
    "mavlink_raw_accel_y",
    "mavlink_raw_accel_z",
    "mavlink_raw_gyro_x",
    "mavlink_raw_gyro_y",
    "mavlink_raw_gyro_z",
    "mavlink_raw_mag_x",
    "mavlink_raw_mag_y",
    "mavlink_raw_mag_z",
)
PX4_MAVLINK_MAG_WIDE_COLUMNS = (
    "mag_scaled_imu_x",
    "mag_scaled_imu_y",
    "mag_scaled_imu_z",
    "mag_scaled_imu_unit",
    "mag_scaled_imu2_x",
    "mag_scaled_imu2_y",
    "mag_scaled_imu2_z",
    "mag_scaled_imu2_unit",
    "mag_scaled_imu3_x",
    "mag_scaled_imu3_y",
    "mag_scaled_imu3_z",
    "mag_scaled_imu3_unit",
)
_PX4_MAVLINK_MAG_STREAMS = {
    "mavlink/SCALED_IMU": "mag_scaled_imu",
    "mavlink/SCALED_IMU2": "mag_scaled_imu2",
    "mavlink/SCALED_IMU3": "mag_scaled_imu3",
}
_PX4_MAVLINK_MAG_MATCH_TOLERANCE_NS = 25_000_000
PX4_GPS_WIDE_FIELDS = (
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "fix_type",
    "satellites_used",
    "eph_m",
    "epv_m",
    "heading_deg",
    "ground_speed_m_s",
    "yaw_deg",
    "heading_accuracy_deg",
)
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
GNSS_BASE_COMPACT_COLUMNS = DEVICE_TIMELINE_COLUMNS + (
    "latitude_deg",
    "longitude_deg",
    "ellipsoid_height_m",
    "msl_altitude_m",
    "height_type",
    "fixed",
    "device_id",
    "gnss_operation_state",
)
EXTRINSICS_COMPARISON_COMPACT_COLUMNS = (
    "pair_sequence",
    "timestamp_s",
    "method",
    "forward_m",
    "right_m",
    "down_m",
    "range_m",
    "status",
)
DYNAMIC_SHEET_RULES: dict[str, tuple[str, ...]] = {
    "Measurements": (
        "measurement",
        "distance",
        "range_m",
        "baseline_center",
        "source_left_camera_rdf",
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

AUTHORITATIVE_JSONL_SCHEMA_VERSION = "stereowiris-authoritative-jsonl-v1"
MEASUREMENT_STORAGE_SCHEMA = "workstation-measurement-storage-v3"
MEASUREMENT_STORAGE_SCHEMA_VERSION = 3
AUTHORITATIVE_JSONL_STREAMS = {
    "raw_stereo_measurements": (
        "raw/stereo_measurements",
        "data/raw/stereo_measurements.jsonl",
    ),
    "raw_camera_left": ("raw/camera_left", "data/raw/camera_left.jsonl"),
    "raw_camera_right": ("raw/camera_right", "data/raw/camera_right.jsonl"),
    "raw_px4_left": ("raw/px4_left", "data/raw/px4_left.jsonl"),
    "raw_px4_right": ("raw/px4_right", "data/raw/px4_right.jsonl"),
    "raw_gnss_base": ("raw/gnss_base", "data/raw/gnss_base.jsonl"),
    "geometry_contexts": (
        "context/geometry_contexts",
        "data/context/geometry_contexts.jsonl",
    ),
    "time_sync_epochs": (
        "context/time_sync_epochs",
        "data/context/time_sync_epochs.jsonl",
    ),
    "raw_px4_drone": ("raw/px4_drone", "data/raw/px4_drone.jsonl"),
    "raw_extrinsics_comparison": (
        "raw/extrinsics_comparison",
        "data/raw/extrinsics_comparison.jsonl",
    ),
    "method_evaluation_bundles": (
        "raw/method_evaluation_bundles",
        "data/raw/method_evaluation_bundles.jsonl",
    ),
}

for _method_result_id in MEASUREMENT_METHOD_IDS:
    AUTHORITATIVE_JSONL_STREAMS[f"method_result_{_method_result_id}"] = (
        f"raw/method_results/{_method_result_id}",
        f"data/raw/method_results/{_method_result_id}.jsonl",
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
    """Return the canonical v2 session directory directly below ``root``.

    Older Workstation builds inserted an extra ``sessions`` component.  New
    writers intentionally use ``recordings/<session-id>``; read/discovery
    paths keep explicit legacy fallback support instead of perpetuating the
    extra directory for new evidence.
    """

    selected = str(session_id or session_id_now())
    if (
        selected in {"", ".", "..", "sessions"}
        or selected.startswith(".")
        or Path(selected).name != selected
        or "\\" in selected
        or "\x00" in selected
    ):
        raise ValueError("session_id must be one safe non-reserved path component")
    return root / selected


def legacy_session_dir(root: Path, session_id: str) -> Path:
    """Return the pre-v2 ``recordings/sessions/<session-id>`` location."""

    return root / "sessions" / session_id


def resolve_session_dir(root: Path, session_id: str) -> Path:
    """Resolve an existing v2/legacy session or the canonical new location.

    A duplicate ID across the two layouts and any session-directory symlink
    are rejected rather than selecting an ambiguous evidence owner.
    """

    root = Path(root)
    legacy_root = root / "sessions"
    if root.is_symlink() or legacy_root.is_symlink():
        raise ValueError("recordings and legacy container directories must not be symlinks")
    canonical = session_dir(root, session_id)
    legacy = legacy_session_dir(root, session_id)
    for candidate in (canonical, legacy):
        if candidate.is_symlink():
            raise ValueError(f"session directories must not be symlinks: {candidate}")
    canonical_exists = canonical.exists()
    legacy_exists = legacy.exists()
    if canonical_exists and legacy_exists:
        raise FileExistsError(
            f"direct/legacy session ID collision: {session_id}"
        )
    if canonical_exists:
        return canonical
    if legacy_exists:
        return legacy
    return canonical


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
        self._recovery_warnings: list[str] = []
        self._recovery_artifacts: list[Path] = []

    @property
    def spool_path(self) -> Path | None:
        return self._path

    @property
    def preview_count(self) -> int:
        return len(self._preview)

    @property
    def recovery_warnings(self) -> tuple[str, ...]:
        return tuple(self._recovery_warnings)

    @property
    def recovery_artifacts(self) -> tuple[Path, ...]:
        return tuple(self._recovery_artifacts)

    def bind(self, path: Path) -> None:
        with self._lock:
            self.close()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.unlink(missing_ok=True)
            self._path = path
            self._handle = path.open("a", encoding="utf-8", buffering=1)
            self._preview.clear()
            self._count = 0
            self._recovery_warnings.clear()
            self._recovery_artifacts.clear()

    def open_existing(self, path: Path) -> None:
        with self._lock:
            self.close()
            self._path = path
            self._preview.clear()
            self._count = 0
            self._recovery_warnings.clear()
            self._recovery_artifacts.clear()
            if path.exists():
                self._repair_unterminated_tail(path)
                for row in self._materialized_rows() if self.supports_updates else self._plain_rows():
                    self._preview.append(row)
                    self._count += 1
            path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = path.open("a", encoding="utf-8", buffering=1)

    @staticmethod
    def _atomic_replace_bytes(path: Path, payload: bytes) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".repair",
        )
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "wb") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def _repair_unterminated_tail(self, path: Path) -> None:
        """Preserve a valid prefix and quarantine only a torn final write.

        A malformed newline-terminated row or malformed interior row is not a
        recoverable crash tail and remains a hard error when rows are parsed.
        """

        payload = path.read_bytes()
        if not payload or payload.endswith(b"\n"):
            return
        prefix, separator, tail = payload.rpartition(b"\n")
        try:
            decoded = json.loads(tail)
        except (UnicodeDecodeError, json.JSONDecodeError):
            quarantine_root = path.parent / ".quarantine"
            quarantine_root.mkdir(parents=True, exist_ok=True)
            digest = hashlib.sha256(tail).hexdigest()
            quarantine = quarantine_root / f"{path.name}.{digest[:16]}.truncated-tail"
            if not quarantine.exists():
                self._atomic_replace_bytes(quarantine, tail)
            valid_prefix = prefix + separator if separator else b""
            self._atomic_replace_bytes(path, valid_prefix)
            self._recovery_artifacts.append(quarantine)
            self._recovery_warnings.append(
                f"quarantined truncated final JSONL record: {quarantine.name}"
            )
            return
        if not isinstance(decoded, dict):
            raise OSError(f"unterminated spool row is not an object: {path}")
        self._atomic_replace_bytes(path, payload + b"\n")
        self._recovery_warnings.append("normalized valid unterminated final JSONL record")

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

    def discard_spool(self) -> bool:
        """Best-effort removal after every final artifact is committed.

        Cleanup can fail because of filesystem permissions or transient I/O,
        but verified published data must not be downgraded into a retry that
        has already removed some sibling spools.  A failed path remains bound
        for later maintenance; callers never lose the full materialized rows.
        """
        with self._lock:
            try:
                self.close()
            except OSError:
                return False
            path = self._path
            if path is not None:
                try:
                    path.unlink(missing_ok=True)
                except OSError:
                    return False
                try:
                    path.parent.rmdir()
                except OSError:
                    pass
            self._path = None
            return True

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
            for line_number, line in enumerate(handle, start=1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise OSError(
                        f"malformed spool JSONL row: path={path}, line={line_number}"
                    ) from exc
                if not isinstance(row, dict):
                    raise OSError(
                        f"spool JSONL row is not an object: path={path}, line={line_number}"
                    )
                if "__spool_update_index" not in row:
                    yield row

    def _materialized_rows(self) -> list[dict[str, object]]:
        path = self._path
        if path is None or not path.exists():
            return list(self._preview)
        rows: list[dict[str, object]] = []
        with path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise OSError(
                        f"malformed spool JSONL row: path={path}, line={line_number}"
                    ) from exc
                if not isinstance(record, dict):
                    raise OSError(
                        f"spool JSONL row is not an object: path={path}, line={line_number}"
                    )
                if "__spool_update_index" in record:
                    try:
                        update_index = int(record["__spool_update_index"])
                        update_row = record["__spool_row"]
                        if not isinstance(update_row, dict):
                            raise TypeError("update row is not an object")
                        rows[update_index] = update_row
                    except (IndexError, KeyError, TypeError, ValueError) as exc:
                        raise OSError(
                            f"invalid spool update record: path={path}, line={line_number}"
                        ) from exc
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
    if isinstance(value, Mapping):
        return {
            str(key): _serializable_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_serializable_value(item) for item in value]
    raise TypeError(f"session row contains unsupported value type: {type(value).__name__}")


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


def _normalized_generic_rows(rows: Iterable[dict[str, object]]) -> list[dict[str, object]]:
    return [
        {key: _serializable_value(value) for key, value in row.items()}
        for row in rows
    ]


def _materialize_method_evaluation_rows(
    rows: Iterable[dict[str, object]],
) -> tuple[list[dict[str, object]], dict[str, list[dict[str, object]]]]:
    """Validate atomic bundle rows and split them into eight method streams."""

    from Workstation.workstation_core.method_evaluation import MethodEvaluationBundle

    bundles: list[dict[str, object]] = []
    method_rows = {method_id: [] for method_id in MEASUREMENT_METHOD_IDS}
    seen: set[tuple[str, int]] = set()
    last_sequence_by_measurement: dict[str, int] = {}
    for raw_row in rows:
        normalized = {
            key: _serializable_value(value) for key, value in raw_row.items()
        }
        bundle = MethodEvaluationBundle.from_mapping(normalized)
        identity = (bundle.pair_id, bundle.request_generation)
        if identity in seen:
            raise ValueError(f"duplicate method evaluation bundle: {bundle.pair_id}")
        seen.add(identity)
        previous_sequence = last_sequence_by_measurement.get(bundle.measurement_id, -1)
        if bundle.pair_sequence <= previous_sequence:
            raise ValueError(
                "method evaluation bundles must have strictly increasing pair_sequence"
            )
        last_sequence_by_measurement[bundle.measurement_id] = bundle.pair_sequence
        bundle_record = bundle.to_record()
        bundles.append(bundle_record)
        bundle_sha256 = str(bundle_record["method_evaluation_bundle_sha256"])
        primary_sha256 = bundle.primary.canonical_sha256()
        for outcome in bundle.outcomes:
            method_rows[outcome.method_id].append(
                {
                    **outcome.to_record(),
                    "method_evaluation_bundle_sha256": bundle_sha256,
                    "primary_method_id": bundle.primary.method_id,
                    "primary_outcome_sha256": primary_sha256,
                }
            )
    return bundles, method_rows


def _iter_normalized_generic_rows(
    rows: Iterable[dict[str, object]],
) -> Iterator[dict[str, object]]:
    for row in rows:
        yield {key: _serializable_value(value) for key, value in row.items()}


def _seconds_to_ns(value: object) -> int | None:
    if value in (None, "") or isinstance(value, bool):
        return None
    try:
        seconds = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(seconds):
        return None
    return int(round(seconds * 1_000_000_000.0))


def _copy_first_present(
    row: dict[str, object],
    destination: str,
    *sources: str,
) -> None:
    if row.get(destination) not in (None, ""):
        return
    for source in sources:
        value = row.get(source)
        if value not in (None, ""):
            row[destination] = value
            return


def _project_pair_typed_fields(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    for row in rows:
        _copy_first_present(row, "recording_time_ns", "session_elapsed_ns")
        _copy_first_present(row, "estimated_utc_time_ns", "recorded_unix_ns")
        _copy_first_present(row, "device_utc_time_ns", "pair_mid_utc_ns")
        _copy_first_present(row, "sync_delta_ns", "rtcp_pair_delta_ns")
    return rows


def _project_camera_typed_fields(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    for row in rows:
        if row.get("recording_time_ns") in (None, ""):
            value = _seconds_to_ns(row.get("session_elapsed_s"))
            if value is not None:
                row["recording_time_ns"] = value
        _copy_first_present(row, "estimated_utc_time_ns", "session_utc_ns")
        _copy_first_present(row, "device_utc_time_ns", "source_utc_ns")
        for destination, source in (
            ("rtcp_sr_age_ns", "rtcp_sr_age_s"),
            ("rtcp_sr_pc_offset_ns", "rtcp_sr_pc_offset_s"),
            ("rtcp_receive_delay_ns", "rtcp_frame_receive_delay_s"),
            ("rtcp_transport_delay_ns", "rtcp_frame_transport_delay_s"),
        ):
            if row.get(destination) in (None, ""):
                value = _seconds_to_ns(row.get(source))
                if value is not None:
                    row[destination] = value
    return rows


def _project_px4_typed_fields(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    for row in rows:
        if row.get("recording_time_ns") in (None, ""):
            value = _seconds_to_ns(row.get("session_elapsed_s"))
            if value is not None:
                row["recording_time_ns"] = value
        _copy_first_present(row, "session_utc_time_ns", "session_utc_ns")
        _copy_first_present(row, "gps_utc_time_ns", "source_utc_ns")
        _copy_first_present(row, "gps_utc_source", "source_utc_method")
    return rows


def _iter_project_camera_typed_fields(
    rows: Iterable[dict[str, object]],
) -> Iterator[dict[str, object]]:
    for row in rows:
        yield _project_camera_typed_fields([row])[0]


def _iter_project_px4_typed_fields(
    rows: Iterable[dict[str, object]],
) -> Iterator[dict[str, object]]:
    for row in rows:
        yield _project_px4_typed_fields([row])[0]


def _authoritative_jsonl_row_bytes(row: dict[str, object]) -> bytes:
    try:
        encoded = json.dumps(
            row,
            ensure_ascii=False,
            sort_keys=True,
            allow_nan=False,
        )
    except ValueError as exc:
        raise ValueError(
            "authoritative JSONL row contains a non-finite number"
        ) from exc
    return (encoded + "\n").encode("utf-8")


def _apply_measurement_package_identity(
    rows: Iterable[dict[str, object]],
    *,
    measurement_id: str,
) -> None:
    """Add the package alias without changing ``measurement_id`` semantics."""

    for row in rows:
        declared = str(row.get("measurement_id") or "")
        alias = str(row.get("measurement_package_id") or "")
        if declared and declared != measurement_id:
            raise ValueError(
                f"row measurement_id {declared!r} does not match package {measurement_id!r}"
            )
        if alias and alias != measurement_id:
            raise ValueError(
                "row measurement_package_id does not match its measurement_id"
            )
        row["measurement_id"] = measurement_id
        row["measurement_package_id"] = measurement_id


def _apply_terminal_record_identities(
    rows: Iterable[dict[str, object]],
) -> None:
    """Materialize deterministic, non-positional terminal primary keys."""

    seen: set[str] = set()
    for row in rows:
        attempt_sha256 = str(row.get("measurement_attempt_sha256") or "")
        blocked_sha256 = str(row.get("blocked_pair_sha256") or "")
        populated = [value for value in (attempt_sha256, blocked_sha256) if value]
        if len(populated) > 1:
            raise ValueError("terminal row cannot be both an attempt and a blocked pair")
        if populated:
            digest = populated[0]
            if len(digest) != 64 or any(
                character not in "0123456789abcdef" for character in digest
            ):
                raise ValueError("terminal source identity must be lowercase SHA-256 hex")
            namespace = "attempt" if attempt_sha256 else "blocked"
            expected = f"{namespace}:{digest}"
        else:
            # Pre-G2/fixture rows may not carry a canonical attempt.  Their
            # complete content still supplies a stable, non-positional legacy
            # identity while new Workstation terminal rows use attempt/blocked.
            payload = {
                key: value
                for key, value in row.items()
                if key
                not in {
                    "terminal_record_id",
                    "sample_index",
                    "pair_sequence",
                }
            }
            try:
                encoded = json.dumps(
                    payload,
                    ensure_ascii=False,
                    allow_nan=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            except ValueError as exc:
                raise ValueError(
                    "terminal row contains a non-finite number"
                ) from exc
            expected = f"legacy:{hashlib.sha256(encoded).hexdigest()}"
        declared = str(row.get("terminal_record_id") or "")
        if declared and declared != expected:
            raise ValueError("terminal_record_id disagrees with canonical source identity")
        if expected in seen:
            raise ValueError(f"duplicate terminal_record_id: {expected}")
        row["terminal_record_id"] = expected
        seen.add(expected)


def _authoritative_jsonl_schema(
    rows: Iterable[dict[str, object]],
    *,
    normalization: str,
) -> dict[str, object]:
    return {
        "version": AUTHORITATIVE_JSONL_SCHEMA_VERSION,
        "format": "jsonl",
        "encoding": "utf-8",
        "normalization": normalization,
        "fields": sorted({str(key) for row in rows for key in row}),
    }


def _write_authoritative_jsonl(
    output: Path,
    rows: Iterable[dict[str, object]],
) -> None:
    """Atomically write every normalized source row without field selection."""

    output.parent.mkdir(parents=True, exist_ok=True)
    temp_path: Path | None = None
    try:
        fd, temp_name = tempfile.mkstemp(
            dir=output.parent,
            prefix=f".{output.name}.",
            suffix=".tmp",
        )
        temp_path = Path(temp_name)
        with os.fdopen(fd, "wb") as handle:
            for row in rows:
                handle.write(_authoritative_jsonl_row_bytes(row))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, output)
        _fsync_directory_path(output.parent)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def _validate_authoritative_jsonl(
    output: Path,
    expected_rows: Sequence[dict[str, object]],
    *,
    role: str,
    artifact_path: str,
    normalization: str,
) -> dict[str, object]:
    """Re-read a staged JSONL artifact and return its verified manifest detail."""

    digest = hashlib.sha256()
    row_count = 0
    try:
        with output.open("rb") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                digest.update(raw_line)
                if row_count >= len(expected_rows):
                    raise OSError(
                        f"authoritative JSONL has an unexpected row: "
                        f"path={output}, line={line_number}"
                    )
                if raw_line != _authoritative_jsonl_row_bytes(expected_rows[row_count]):
                    raise OSError(
                        f"authoritative JSONL content mismatch: "
                        f"path={output}, line={line_number}"
                    )
                try:
                    decoded = json.loads(raw_line)
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    raise OSError(
                        f"authoritative JSONL is not valid UTF-8 JSON: "
                        f"path={output}, line={line_number}"
                    ) from exc
                if not isinstance(decoded, dict):
                    raise OSError(
                        f"authoritative JSONL row is not an object: "
                        f"path={output}, line={line_number}"
                    )
                row_count += 1
    except OSError:
        raise

    if row_count != len(expected_rows):
        raise OSError(
            f"authoritative JSONL row count mismatch: "
            f"path={output}, expected={len(expected_rows)}, actual={row_count}"
        )
    size = output.stat().st_size
    schema = _authoritative_jsonl_schema(
        expected_rows,
        normalization=normalization,
    )
    return {
        "role": role,
        "media_type": "application/x-ndjson",
        "relative_path": artifact_path,
        "size_bytes": size,
        "sha256": digest.hexdigest(),
        "row_count": row_count,
        "schema_version": schema["version"],
        "authoritative": True,
        "derived_from": [],
        "encoding": schema["encoding"],
        "normalization": schema["normalization"],
        "fields": schema["fields"],
        "verified": True,
    }


def _scope_unpublished_artifact_details(
    manifest: dict[str, object],
    *,
    artifact_root: Path,
) -> None:
    """Keep failed-attempt details distinct from the stable published paths."""

    details = manifest.get("artifact_details")
    unpublished: dict[str, object] = {}
    if isinstance(details, dict):
        for name, value in details.items():
            if not isinstance(value, dict):
                continue
            unpublished[str(name)] = {
                **value,
                "path": (artifact_root / Path(str(name)).name).as_posix(),
                "published": False,
            }
    manifest["artifact_details"] = {}
    manifest["unpublished_artifact_details"] = unpublished
    manifest["authoritative_raw_artifacts_verified"] = False


def _none_if_empty(value: object) -> object | None:
    if value is None:
        return None
    if isinstance(value, str) and value == "":
        return None
    return value


def _optional_int(value: object) -> int | None:
    value = _none_if_empty(value)
    if value is None:
        return None
    if isinstance(value, bool):
        return int(value)
    try:
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(Decimal(str(value)).to_integral_value(rounding=ROUND_HALF_UP))
        except (InvalidOperation, ValueError):
            return None


def _optional_float(value: object) -> float | None:
    value = _none_if_empty(value)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_bool(value: object) -> bool | None:
    value = _none_if_empty(value)
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "y"}:
        return True
    if text in {"false", "0", "no", "n"}:
        return False
    return None


def _ns_from_seconds(value: object) -> int | None:
    value = _none_if_empty(value)
    if value is None:
        return None
    try:
        return int(
            (Decimal(str(value)) * Decimal(1_000_000_000)).to_integral_value(
                rounding=ROUND_HALF_UP
            )
        )
    except (InvalidOperation, ValueError):
        return None


def _first_present(row: dict[str, object], *keys: str) -> object | None:
    for key in keys:
        value = _none_if_empty(row.get(key))
        if value is not None:
            return value
    return None


def _recording_time_ns(row: dict[str, object], metadata: dict[str, object]) -> int | None:
    direct = _optional_int(_first_present(row, "recording_time_ns", "session_elapsed_ns"))
    if direct is not None:
        return direct
    from_seconds = _ns_from_seconds(
        _first_present(row, "session_elapsed_s", "recording_elapsed_s")
    )
    if from_seconds is not None:
        return from_seconds
    sample_monotonic_ns = _optional_int(
        _first_present(row, "sample_monotonic_ns", "pair_app_mid_monotonic_ns")
    )
    started_monotonic_ns = _optional_int(metadata.get("session_started_monotonic_ns"))
    if sample_monotonic_ns is not None and started_monotonic_ns is not None:
        return max(0, sample_monotonic_ns - started_monotonic_ns)
    return None


def _estimated_utc_time_ns(
    row: dict[str, object],
    metadata: dict[str, object],
    recording_time_ns: int | None,
) -> int | None:
    direct = _optional_int(
        _first_present(row, "estimated_utc_time_ns", "session_utc_ns")
    )
    if direct is not None:
        return direct
    started_unix_ns = _optional_int(metadata.get("session_started_unix_ns"))
    if started_unix_ns is not None and recording_time_ns is not None:
        return started_unix_ns + recording_time_ns
    return None


def _distance_device_utc_time_ns(row: dict[str, object]) -> int | None:
    return _optional_int(
        _first_present(
            row,
            "device_utc_time_ns",
            "pair_mid_utc_ns",
            "bbox_mid_utc_ns",
        )
    )


def _timeline_device_utc_time_ns(row: dict[str, object]) -> int | None:
    direct = _optional_int(_first_present(row, "device_utc_time_ns", "source_utc_ns"))
    if direct is not None:
        return direct
    gps_utc_us = _optional_int(row.get("gps_utc_us"))
    if gps_utc_us is not None:
        return gps_utc_us * 1000
    return None


def _paired_class_name(row: dict[str, object]) -> str:
    left = str(_first_present(row, "left_bbox_class_name", "left_class_name") or "")
    right = str(_first_present(row, "right_bbox_class_name", "right_class_name") or "")
    if left and right:
        return left if left == right else f"{left}/{right}"
    return left or right


def _warning_code(row: dict[str, object]) -> str:
    text = str(_first_present(row, "warnings", "measurement_attempt_warnings") or "")
    return text.split("; ", 1)[0].strip()


def _compact_distance_measurement_row(
    row: dict[str, object],
    metadata: dict[str, object],
) -> dict[str, object]:
    recording_time_ns = _recording_time_ns(row, metadata)
    sync_delta_ns = _optional_int(_first_present(row, "rtcp_pair_delta_ns", "sync_delta_ns"))
    if sync_delta_ns is None:
        sync_delta_ns = _ns_from_seconds(_first_present(row, "camera_delta_s", "rtcp_pair_delta_s"))
    compact = {
        "sample_index": _optional_int(row.get("sample_index")),
        "recording_time_ns": recording_time_ns,
        "estimated_utc_time_ns": _estimated_utc_time_ns(row, metadata, recording_time_ns),
        "device_utc_time_ns": _distance_device_utc_time_ns(row),
        "pair_id": str(
            _first_present(
                row,
                "measurement_attempt_id",
                "measurement_attempt_session_id",
                "measurement_detection_pair_key",
                "pair_sequence",
                "sample_index",
            )
            or ""
        ),
        "measurement_valid": _optional_bool(row.get("measurement_valid")),
        "pair_status": str(row.get("pair_status") or ""),
        "class_name": _paired_class_name(row),
        "left_bbox_center_x_px": _optional_float(_first_present(row, "left_bbox_center_x_px", "left_cx")),
        "left_bbox_center_y_px": _optional_float(_first_present(row, "left_bbox_center_y_px", "left_cy")),
        "right_bbox_center_x_px": _optional_float(_first_present(row, "right_bbox_center_x_px", "right_cx")),
        "right_bbox_center_y_px": _optional_float(_first_present(row, "right_bbox_center_y_px", "right_cy")),
        "left_confidence": _optional_float(_first_present(row, "left_bbox_confidence", "left_confidence")),
        "right_confidence": _optional_float(_first_present(row, "right_bbox_confidence", "right_confidence")),
        "distance_m": _optional_float(row.get("distance_m")),
        "baseline_center_forward_m": _optional_float(row.get("baseline_center_forward_m")),
        "baseline_center_right_m": _optional_float(row.get("baseline_center_right_m")),
        "baseline_center_down_m": _optional_float(row.get("baseline_center_down_m")),
        "baseline_m": _optional_float(_first_present(row, "baseline_m", "baseline_length_m")),
        "sync_delta_ns": sync_delta_ns,
        "quality": str(_first_present(row, "quality", "pair_quality_status") or ""),
        "warning_code": _warning_code(row),
    }
    return {column: compact.get(column) for column in DISTANCE_MEASUREMENT_COLUMNS}


def compact_distance_measurement_rows(
    rows: Iterable[dict[str, object]],
    metadata: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    metadata = metadata or {}
    return [_compact_distance_measurement_row(row, metadata) for row in rows]


def _compact_camera_timeline_row(
    row: dict[str, object],
    metadata: dict[str, object],
) -> dict[str, object]:
    recording_time_ns = _recording_time_ns(row, metadata)
    receive_delay_ns = _ns_from_seconds(row.get("rtcp_frame_receive_delay_s"))
    invalid_reason = str(row.get("rtcp_timing_invalid_reason") or "")
    compact = {
        "sequence": _optional_int(row.get("sequence")),
        "recording_time_ns": recording_time_ns,
        "estimated_utc_time_ns": _estimated_utc_time_ns(row, metadata, recording_time_ns),
        "device_utc_time_ns": _timeline_device_utc_time_ns(row),
        "image_width_px": _optional_int(row.get("image_width_px")),
        "image_height_px": _optional_int(row.get("image_height_px")),
        "timestamp_source": str(row.get("timestamp_source") or ""),
        "rtp_timestamp": _optional_int(row.get("rtp_timestamp")),
        "rtp_ssrc": _optional_int(row.get("rtp_ssrc")),
        "rtp_marker_seq": _optional_int(row.get("rtp_marker_seq")),
        "rtcp_sr_received_count": _optional_int(row.get("rtcp_sr_received_count")),
        "rtcp_sr_rtp_timestamp": _optional_int(row.get("rtcp_sr_rtp_timestamp")),
        "rtcp_clock_rate_hz": _optional_float(row.get("rtcp_clock_rate_hz")),
        "rtcp_sr_age_ns": _ns_from_seconds(row.get("rtcp_sr_age_s")),
        "rtcp_sr_pc_offset_ns": _ns_from_seconds(row.get("rtcp_sr_pc_offset_s")),
        "rtcp_receive_delay_ns": receive_delay_ns,
        "rtcp_transport_delay_ns": _ns_from_seconds(row.get("rtcp_frame_transport_delay_s")),
        "rtcp_camera_ntp_rate": _optional_float(row.get("rtcp_camera_ntp_rate")),
        "rtcp_camera_media_rate": _optional_float(row.get("rtcp_camera_media_rate")),
        "rtsp_transport": str(row.get("rtsp_transport") or ""),
        "timing_status": invalid_reason or "valid",
    }
    return {column: compact.get(column) for column in CAMERA_TIMELINE_COMPACT_COLUMNS}


def compact_camera_timeline_rows(
    rows: Iterable[dict[str, object]],
    metadata: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    metadata = metadata or {}
    return [_compact_camera_timeline_row(row, metadata) for row in rows]


def _compact_px4_timeline_row(
    row: dict[str, object],
    metadata: dict[str, object],
) -> dict[str, object]:
    recording_time_ns = _recording_time_ns(row, metadata)
    compact = {
        "sequence": _optional_int(row.get("sequence")),
        "recording_time_ns": recording_time_ns,
        "session_utc_time_ns": _estimated_utc_time_ns(row, metadata, recording_time_ns),
        "gps_utc_time_ns": _gps_utc_time_ns(row),
        "gps_utc_source": _gps_utc_source(row),
        "sample_type": str(row.get("sample_type") or ""),
        "topic": str(row.get("topic") or ""),
        "attitude_rollspeed_rad_s": _optional_float(row.get("attitude_rollspeed_rad_s")),
        "attitude_pitchspeed_rad_s": _optional_float(row.get("attitude_pitchspeed_rad_s")),
        "attitude_yawspeed_rad_s": _optional_float(row.get("attitude_yawspeed_rad_s")),
        "attitude_time_boot_ms": _optional_int(row.get("attitude_time_boot_ms")),
        "roll_deg": _optional_float(row.get("roll_deg")),
        "pitch_deg": _optional_float(row.get("pitch_deg")),
        "latitude_deg": _optional_float(_first_present(row, "latitude_deg", "lat_deg")),
        "longitude_deg": _optional_float(_first_present(row, "longitude_deg", "lon_deg")),
        "altitude_m": _optional_float(_first_present(row, "altitude_m", "alt_m")),
        "altitude_ellipsoid_m": _optional_float(row.get("altitude_ellipsoid_m")),
        "fix_type": _optional_int(row.get("fix_type")),
        "satellites_used": _optional_int(_first_present(row, "satellites_used", "satellites")),
        "eph_m": _optional_float(row.get("eph_m")),
        "epv_m": _optional_float(row.get("epv_m")),
        "rtk_label": str(row.get("rtk_label") or ""),
        "heading_deg": _optional_float(row.get("heading_deg")),
        "ground_speed_m_s": _optional_float(row.get("ground_speed_m_s")),
        "yaw_deg": _optional_float(row.get("gps_yaw_deg", row.get("yaw_deg"))),
        "heading_accuracy_deg": _optional_float(row.get("heading_accuracy_deg")),
        "velocity_n_m_s": _optional_float(row.get("velocity_n_m_s")),
        "velocity_e_m_s": _optional_float(row.get("velocity_e_m_s")),
        "velocity_d_m_s": _optional_float(row.get("velocity_d_m_s")),
        "velocity_ned_valid": _optional_bool(row.get("velocity_ned_valid")),
        "mag_sensor_slot": str(_first_present(row, "mag_sensor_slot", "sensor_slot") or ""),
        "mag_slot": _optional_int(row.get("mag_slot")),
        "mag_cal_slot": str(row.get("mag_cal_slot") or ""),
        "mag_value_units": str(row.get("mag_value_units") or ""),
        "mag_x": _optional_float(row.get("mag_x")),
        "mag_y": _optional_float(row.get("mag_y")),
        "mag_z": _optional_float(row.get("mag_z")),
        "mag_vector_norm": _optional_float(
            _first_present(row, "mag_vector_norm", "mag_magnitude")
        ),
        "mag_temperature_c": _optional_float(row.get("mag_temperature_c")),
        "mag_device_id": _optional_int(_first_present(row, "mag_device_id", "device_id")),
        "mag_node_id": _optional_int(_first_present(row, "mag_node_id", "node_id")),
        "mag_device_type": str(row.get("mag_device_type") or ""),
        "mag_interface": str(row.get("mag_interface") or ""),
        "mag_is_rm3100": _optional_bool(row.get("mag_is_rm3100")),
        "mag_priority": _optional_int(row.get("mag_priority")),
        "mag_rotation": _optional_int(row.get("mag_rotation")),
        "mag_enabled": _optional_bool(row.get("mag_enabled")),
        "mag_expected_selected": _optional_bool(row.get("mag_expected_selected")),
        "mag_actual_selected": _optional_bool(row.get("mag_actual_selected")),
        "mag_expected_selected_device_id": _optional_int(row.get("mag_expected_selected_device_id")),
        "mag_actual_selected_device_id": _optional_int(row.get("mag_actual_selected_device_id")),
        "mag_is_actual_rm3100": _optional_bool(row.get("mag_is_actual_rm3100")),
        "mag_armed": _optional_bool(row.get("mag_armed")),
        "mag_status": str(row.get("mag_status") or ""),
        "mag_warning_text": str(row.get("mag_warning_text") or ""),
        "mag_rm3100_primary_id": _optional_int(row.get("mag_rm3100_primary_id")),
        "mag_rm3100_backup_id": _optional_int(row.get("mag_rm3100_backup_id")),
        "mag_param_uavcan_enable": _optional_int(row.get("mag_param_uavcan_enable")),
        "mag_param_uavcan_sub_mag": _optional_int(row.get("mag_param_uavcan_sub_mag")),
        "mag_param_ekf2_mag_type": _optional_int(row.get("mag_param_ekf2_mag_type")),
        "mag_param_snapshot_json": str(row.get("mag_param_snapshot_json") or ""),
    }
    for stream_name in ("mag_scaled_imu", "mag_scaled_imu2", "mag_scaled_imu3"):
        for suffix in (
            "accel_x",
            "accel_y",
            "accel_z",
            "gyro_x",
            "gyro_y",
            "gyro_z",
            "time_boot_ms",
        ):
            compact[f"{stream_name}_{suffix}"] = row.get(f"{stream_name}_{suffix}")
    for field in (
        "mavlink_message_name",
        "mavlink_time_boot_ms",
        "mavlink_time_usec",
        "mavlink_global_latitude_deg",
        "mavlink_global_longitude_deg",
        "mavlink_global_altitude_m",
        "mavlink_global_relative_altitude_m",
        "mavlink_global_velocity_n_m_s",
        "mavlink_global_velocity_e_m_s",
        "mavlink_global_velocity_d_m_s",
        "mavlink_global_heading_deg",
        "mavlink_local_x_m",
        "mavlink_local_y_m",
        "mavlink_local_z_m",
        "mavlink_local_velocity_x_m_s",
        "mavlink_local_velocity_y_m_s",
        "mavlink_local_velocity_z_m_s",
        "mavlink_vfr_airspeed_m_s",
        "mavlink_vfr_groundspeed_m_s",
        "mavlink_vfr_heading_deg",
        "mavlink_vfr_throttle_pct",
        "mavlink_vfr_altitude_m",
        "mavlink_vfr_climb_m_s",
        "mavlink_sys_voltage_battery_v",
        "mavlink_sys_current_battery_a",
        "mavlink_sys_battery_remaining_pct",
        "mavlink_sys_drop_rate_comm_pct",
        "mavlink_sys_errors_comm",
        "mavlink_battery_current_a",
        "mavlink_battery_remaining_pct",
        "mavlink_battery_temperature_c",
        "mavlink_battery_voltage_0_v",
        "mavlink_battery_voltage_1_v",
        "mavlink_battery_voltage_2_v",
        "mavlink_raw_accel_x",
        "mavlink_raw_accel_y",
        "mavlink_raw_accel_z",
        "mavlink_raw_gyro_x",
        "mavlink_raw_gyro_y",
        "mavlink_raw_gyro_z",
        "mavlink_raw_mag_x",
        "mavlink_raw_mag_y",
        "mavlink_raw_mag_z",
    ):
        compact[field] = row.get(field)
    gps_prefix = _gps_wide_prefix(compact["topic"], compact["sample_type"])
    if gps_prefix is not None:
        for field in PX4_GPS_WIDE_FIELDS:
            compact[f"{gps_prefix}_{field}"] = compact[field]
    return {column: compact.get(column) for column in PX4_TIMELINE_COMPACT_COLUMNS}


def _gps_utc_time_ns(row: dict[str, object]) -> int | None:
    gps_utc_us = _optional_int(row.get("gps_utc_us"))
    return gps_utc_us * 1000 if gps_utc_us is not None else None


def _gps_utc_source(row: dict[str, object]) -> str:
    topic = str(row.get("topic") or "").lower()
    if any(token in topic for token in ("gps2", "sensor_gps_2", "vehicle_gps_position_2")):
        return "GPS2"
    if _gps_utc_time_ns(row) is not None and str(row.get("sample_type") or "").lower().startswith(("gps", "gnss")):
        return "GPS1"
    return ""


def _gps_wide_prefix(topic: object, sample_type: object) -> str | None:
    """Return the explicit GPS1/GPS2 column family for a timeline row."""
    topic_text = str(topic or "").lower()
    sample_text = str(sample_type or "").lower()
    if not any(token in sample_text for token in ("gps", "gnss")):
        return None
    if any(token in topic_text for token in ("gps2", "sensor_gps_2", "vehicle_gps_position_2")):
        return "gps2"
    return "gps1"


def compact_px4_timeline_rows(
    rows: Iterable[dict[str, object]],
    metadata: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    metadata = metadata or {}
    compact_rows = [_compact_px4_timeline_row(row, metadata) for row in rows]
    return _wide_mavlink_mag_rows(compact_rows)


def _compact_gnss_base_row(row: dict[str, object]) -> dict[str, object]:
    # Unlike PX4 rows, GNSS Base rows are already built from
    # ``_raw_timeline_row`` with every DEVICE_TIMELINE_COLUMNS field
    # (including session_utc_ns/host_receive_utc_ns) resolved at record
    # time, so no metadata-driven timestamp reconstruction is needed here.
    compact = {column: row.get(column) for column in DEVICE_TIMELINE_COLUMNS}
    compact.update(
        {
            "latitude_deg": _optional_float(row.get("latitude_deg")),
            "longitude_deg": _optional_float(row.get("longitude_deg")),
            "ellipsoid_height_m": _optional_float(row.get("ellipsoid_height_m")),
            "msl_altitude_m": _optional_float(row.get("msl_altitude_m")),
            "height_type": str(row.get("height_type") or ""),
            "fixed": _optional_bool(row.get("fixed")),
            "device_id": str(row.get("device_id") or ""),
            "gnss_operation_state": str(row.get("gnss_operation_state") or ""),
        }
    )
    return {column: compact.get(column) for column in GNSS_BASE_COMPACT_COLUMNS}


def compact_gnss_base_rows(
    rows: Iterable[dict[str, object]],
    metadata: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    del metadata  # kept for signature parity with the other compact_*_rows() helpers
    return [_compact_gnss_base_row(row) for row in rows]


def _mavlink_mag_stream_name(topic: object) -> str | None:
    topic_text = str(topic or "")
    if topic_text in _PX4_MAVLINK_MAG_STREAMS:
        return _PX4_MAVLINK_MAG_STREAMS[topic_text]
    return _PX4_MAVLINK_MAG_STREAMS.get(
        f"mavlink/{topic_text.rsplit('/', 1)[-1]}"
    )


def _wide_mavlink_mag_rows(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    """Combine nearby SCALED_IMU* callbacks into fixed-column PX4 rows.

    MAVLink delivers the three streams independently, so their receive times
    are not required to be byte-for-byte equal.  A 25 ms window is narrow
    enough for the requested 20 Hz stream and avoids merging adjacent samples.
    Non-MAVLink rows and the existing generic ``mag_x/y/z`` fields are kept.
    """
    events: list[dict[str, object]] = []
    pending: list[dict[str, object]] = []

    for row in rows:
        stream_name = _mavlink_mag_stream_name(row.get("topic"))
        if stream_name is None or row.get("sample_type") != "magnetometer":
            events.append({"kind": "row", "row": row})
            continue

        timestamp = row.get("recording_time_ns")
        timestamp_int = timestamp if isinstance(timestamp, int) else None
        candidate = None
        if timestamp_int is not None:
            candidates = [
                group
                for group in pending
                if group["row"].get(f"{stream_name}_x") is None
                and isinstance(group["recording_time_ns"], int)
                and abs(timestamp_int - group["recording_time_ns"])
                <= _PX4_MAVLINK_MAG_MATCH_TOLERANCE_NS
            ]
            if candidates:
                candidate = min(
                    candidates,
                    key=lambda group: abs(timestamp_int - group["recording_time_ns"]),
                )

        if candidate is None:
            wide_row = dict(row)
            for column in PX4_MAVLINK_MAG_WIDE_COLUMNS:
                wide_row[column] = None
            event = {"kind": "mag", "row": wide_row}
            events.append(event)
            candidate = {
                "row": wide_row,
                "recording_time_ns": timestamp_int,
                "event": event,
            }
            pending.append(candidate)

        wide_row = candidate["row"]
        wide_row[f"{stream_name}_x"] = row.get("mag_x")
        wide_row[f"{stream_name}_y"] = row.get("mag_y")
        wide_row[f"{stream_name}_z"] = row.get("mag_z")
        wide_row[f"{stream_name}_unit"] = row.get("mag_value_units")

        if all(
            wide_row[f"{name}_x"] is not None
            for name in _PX4_MAVLINK_MAG_STREAMS.values()
        ):
            pending.remove(candidate)

    return [event["row"] for event in events]


def _projected_columns(fieldnames: list[str], keywords: tuple[str, ...]) -> list[str]:
    selected = list(PAIR_AXIS_COLUMNS)
    selected.extend(
        field
        for field in fieldnames
        if field not in PAIR_AXIS_COLUMNS
        and any(keyword in field.lower() for keyword in keywords)
    )
    return selected


_INVALID_XLSX_SHEET_CHARS = frozenset("[]:\\/?*")


def _sanitize_xlsx_sheet_name(name: str, *, used: set[str]) -> str:
    """Fit an arbitrary method string into Excel's 31-char, symbol-free sheet name rule."""

    cleaned = "".join("_" if ch in _INVALID_XLSX_SHEET_CHARS else ch for ch in str(name)).strip()
    cleaned = (cleaned or "Sheet")[:31]
    candidate = cleaned
    suffix = 1
    while candidate in used:
        suffix_text = f"~{suffix}"
        candidate = cleaned[: max(1, 31 - len(suffix_text))] + suffix_text
        suffix += 1
    used.add(candidate)
    return candidate


def _extrinsics_comparison_sheet_groups(
    rows: Sequence[dict[str, object]],
) -> list[tuple[str, str, list[dict[str, object]]]]:
    """Group extrinsics-comparison rows by method into one sheet each.

    Method names are per-session dynamic (an ad-hoc PnP candidate id/method
    string, not a fixed enum), so sheets are discovered from the data itself
    rather than declared up front like the other fixed-category sheets.
    """

    grouped: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        method = str(row.get("method") or "unknown")
        grouped.setdefault(method, []).append(row)
    used_names: set[str] = set()
    return [
        (_sanitize_xlsx_sheet_name(f"Ext_{method}", used=used_names), method, method_rows)
        for method, method_rows in grouped.items()
    ]


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


def session_sheet_previews(
    metadata: dict[str, object],
    rows: Sequence[dict[str, object]],
    *,
    camera_rows: dict[str, Sequence[dict[str, object]]] | None = None,
    px4_rows: dict[str, Sequence[dict[str, object]]] | None = None,
    gnss_base_rows: Sequence[dict[str, object]] | None = None,
    extrinsics_comparison_rows: Sequence[dict[str, object]] | None = None,
) -> list[SessionSheetPreview]:
    """Describe the sheets ``write_session_workbook`` would produce.

    The workbook writer consumes this same description, so an on-screen
    preview cannot drift from the saved file's sheet names or columns.
    """

    metadata_entries = _flatten_metadata(metadata)
    camera_rows = camera_rows or {}
    px4_rows = px4_rows or {}
    previews = [
        SessionSheetPreview(name="Summary", kind="key_value", entries=tuple(metadata_entries))
    ]
    previews.append(
        SessionSheetPreview(
            name="DistanceMeasurements",
            kind="table",
            columns=tuple(DISTANCE_MEASUREMENT_COLUMNS),
        )
    )
    for sheet_name in ("CameraLeft", "CameraRight"):
        previews.append(
            SessionSheetPreview(
                name=sheet_name,
                kind="table",
                columns=tuple(CAMERA_TIMELINE_COMPACT_COLUMNS),
            )
        )
    for sheet_name in ("Px4Left", "Px4Right"):
        previews.append(
            SessionSheetPreview(
                name=sheet_name,
                kind="table",
                columns=tuple(PX4_TIMELINE_COMPACT_COLUMNS),
            )
        )
    previews.append(
        SessionSheetPreview(
            name="GnssBase",
            kind="table",
            columns=tuple(GNSS_BASE_COMPACT_COLUMNS),
        )
    )
    previews.extend(
        SessionSheetPreview(
            name=f"{name}Static",
            kind="key_value",
            entries=tuple(
                entry
                for entry in metadata_entries
                if any(keyword in entry[0].lower() for keyword in CALIBRATION_KEYWORDS)
            ),
        ) for name in ("Calibration",)
    )
    for sheet_name, _method, _method_rows in _extrinsics_comparison_sheet_groups(
        extrinsics_comparison_rows or ()
    ):
        previews.append(
            SessionSheetPreview(
                name=sheet_name,
                kind="table",
                columns=tuple(EXTRINSICS_COMPARISON_COMPACT_COLUMNS),
            )
        )
    return previews


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


PX4_XLSX_TIME_COLUMNS = (
    "recording_time_ns",
    "session_utc_time_ns",
    "gps_utc_time_ns",
)
PX4_XLSX_TAB_COLUMNS: dict[str, tuple[str, ...]] = {
    "ATTITUDE": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "attitude_rollspeed_rad_s",
        "attitude_pitchspeed_rad_s",
        "attitude_yawspeed_rad_s",
        "attitude_time_boot_ms",
    ),
    "GPS1": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "gps1_latitude_deg",
        "gps1_longitude_deg",
        "gps1_altitude_m",
        "gps1_fix_type",
        "gps1_satellites_used",
        "gps1_eph_m",
        "gps1_epv_m",
        "gps1_heading_deg",
        "gps1_ground_speed_m_s",
        "gps1_yaw_deg",
        "gps1_heading_accuracy_deg",
    ),
    "GPS2": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "gps2_latitude_deg",
        "gps2_longitude_deg",
        "gps2_altitude_m",
        "gps2_fix_type",
        "gps2_satellites_used",
        "gps2_eph_m",
        "gps2_epv_m",
        "gps2_heading_deg",
        "gps2_ground_speed_m_s",
        "gps2_yaw_deg",
        "gps2_heading_accuracy_deg",
    ),
    "MAG": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "mag_sensor_slot",
        "mag_value_units",
        "mag_x",
        "mag_y",
        "mag_z",
        "mag_vector_norm",
        "mag_temperature_c",
    )
    + tuple(
        field
        for stream in ("mag_scaled_imu", "mag_scaled_imu2", "mag_scaled_imu3")
        for field in (
            f"{stream}_x",
            f"{stream}_y",
            f"{stream}_z",
            f"{stream}_unit",
            f"{stream}_accel_x",
            f"{stream}_accel_y",
            f"{stream}_accel_z",
            f"{stream}_gyro_x",
            f"{stream}_gyro_y",
            f"{stream}_gyro_z",
            f"{stream}_time_boot_ms",
        )
    ),
    "GLOBAL_POSITION": PX4_XLSX_TIME_COLUMNS
    + tuple(
        (
            "topic",
            "mavlink_message_name",
            "mavlink_time_boot_ms",
            "mavlink_global_latitude_deg",
            "mavlink_global_longitude_deg",
            "mavlink_global_altitude_m",
            "mavlink_global_relative_altitude_m",
            "mavlink_global_velocity_n_m_s",
            "mavlink_global_velocity_e_m_s",
            "mavlink_global_velocity_d_m_s",
            "mavlink_global_heading_deg",
        )
    ),
    "LOCAL_POSITION_NED": PX4_XLSX_TIME_COLUMNS
    + tuple(
        (
            "topic",
            "mavlink_message_name",
            "mavlink_time_boot_ms",
            "mavlink_local_x_m",
            "mavlink_local_y_m",
            "mavlink_local_z_m",
            "mavlink_local_velocity_x_m_s",
            "mavlink_local_velocity_y_m_s",
            "mavlink_local_velocity_z_m_s",
        )
    ),
    "RAW_IMU": PX4_XLSX_TIME_COLUMNS
    + tuple(
        (
            "topic",
            "mavlink_message_name",
            "mavlink_time_usec",
            "mavlink_raw_accel_x",
            "mavlink_raw_accel_y",
            "mavlink_raw_accel_z",
            "mavlink_raw_gyro_x",
            "mavlink_raw_gyro_y",
            "mavlink_raw_gyro_z",
            "mavlink_raw_mag_x",
            "mavlink_raw_mag_y",
            "mavlink_raw_mag_z",
        )
    ),
    "MAG_CONFIG": PX4_XLSX_TIME_COLUMNS
    + (
        "topic",
        "mag_sensor_slot",
        "mag_slot",
        "mag_cal_slot",
        "mag_priority",
        "mag_rotation",
        "mag_enabled",
        "mag_expected_selected",
        "mag_actual_selected",
        "mag_armed",
        "mag_status",
        "mag_warning_text",
        "mag_param_uavcan_enable",
        "mag_param_uavcan_sub_mag",
        "mag_param_ekf2_mag_type",
        "mag_param_snapshot_json",
    ),
}


def _write_px4_workbook(output: Path, rows: Sequence[dict[str, object]]) -> None:
    buckets: dict[str, list[dict[str, object]]] = {name: [] for name in PX4_XLSX_TAB_COLUMNS}
    for row in rows:
        sample_type = str(row.get("sample_type") or "")
        topic = str(row.get("topic") or "")
        if sample_type == "pixhawk_attitude":
            bucket = "ATTITUDE"
        elif sample_type == "gps_rtk":
            bucket = "GPS2" if "GPS2" in topic.upper() else "GPS1"
        elif sample_type == "magnetometer":
            bucket = "MAG"
        elif sample_type == "mag_slot_status":
            bucket = "MAG_CONFIG"
        elif sample_type == "mavlink_global_position_int":
            bucket = "GLOBAL_POSITION"
        elif sample_type == "mavlink_local_position_ned":
            bucket = "LOCAL_POSITION_NED"
        elif sample_type == "mavlink_raw_imu":
            bucket = "RAW_IMU"
        else:
            continue
        buckets[bucket].append(row)

    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(dir=output.parent, prefix=f".{output.name}.", suffix=".tmp")
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        with xlsxwriter.Workbook(temp_path) as workbook:
            header_format = workbook.add_format({"bold": True, "bg_color": "#e0e0e0"})
            for name, columns in PX4_XLSX_TAB_COLUMNS.items():
                worksheet = workbook.add_worksheet(name)
                _write_table(worksheet, list(columns), buckets[name], header_format)
        os.replace(temp_path, output)
    finally:
        temp_path.unlink(missing_ok=True)


RTK_XLSX_COLUMNS = (
    "side",
    "recording_time_ns",
    "session_utc_time_ns",
    "topic",
    "heading_deg",
    "heading_accuracy_deg",
    "position_length_m",
    "accuracy_length_m",
    "gnss_fix_ok",
    "differential_solution",
    "relative_position_valid",
    "carrier_solution_floating",
    "carrier_solution_fixed",
    "moving_base_mode",
    "heading_valid",
    "device_id",
)


def _compact_rtk_rows(
    rows: dict[str, Sequence[dict[str, object]]], metadata: dict[str, object]
) -> list[dict[str, object]]:
    compact: list[dict[str, object]] = []
    for side, side_rows in rows.items():
        for row in side_rows:
            if row.get("sample_type") != "gnss_relative":
                continue
            recording_time_ns = _recording_time_ns(row, metadata)
            compact.append(
                {
                    "side": side,
                    "recording_time_ns": recording_time_ns,
                    "session_utc_time_ns": _estimated_utc_time_ns(row, metadata, recording_time_ns),
                    "topic": row.get("topic"),
                    "heading_deg": row.get("heading_deg"),
                    "heading_accuracy_deg": row.get("heading_accuracy_deg"),
                    "position_length_m": row.get("position_length_m"),
                    "accuracy_length_m": row.get("accuracy_length_m"),
                    "gnss_fix_ok": row.get("gnss_fix_ok"),
                    "differential_solution": row.get("differential_solution"),
                    "relative_position_valid": row.get("relative_position_valid"),
                    "carrier_solution_floating": row.get("carrier_solution_floating"),
                    "carrier_solution_fixed": row.get("carrier_solution_fixed"),
                    "moving_base_mode": row.get("moving_base_mode"),
                    "heading_valid": row.get("heading_valid"),
                    "device_id": row.get("device_id"),
                }
            )
    return compact


def _write_rtk_workbook(output: Path, rows: Sequence[dict[str, object]]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(dir=output.parent, prefix=f".{output.name}.", suffix=".tmp")
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        with xlsxwriter.Workbook(temp_path) as workbook:
            header_format = workbook.add_format({"bold": True, "bg_color": "#e0e0e0"})
            worksheet = workbook.add_worksheet("GPS_RTK")
            _write_table(worksheet, list(RTK_XLSX_COLUMNS), rows, header_format)
        os.replace(temp_path, output)
    finally:
        temp_path.unlink(missing_ok=True)


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


def write_parquet(
    output: Path,
    rows: Iterable[dict[str, object]],
    *,
    columns: Iterable[str],
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    columns = list(columns)
    records = [
        {column: row.get(column) for column in columns}
        for row in rows
    ]
    if records:
        table = pa.Table.from_pylist(records)
        table = table.select(columns)
    else:
        table = pa.Table.from_pydict({column: [] for column in columns})
    temp_path: Path | None = None
    try:
        fd, temp_name = tempfile.mkstemp(
            dir=output.parent, prefix=f".{output.name}.", suffix=".tmp"
        )
        os.close(fd)
        temp_path = Path(temp_name)
        pq.write_table(table, temp_path)
        os.replace(temp_path, output)
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def _write_session_static_json(
    output: Path,
    metadata: dict[str, object],
    *,
    resource_files: dict[str, str],
) -> None:
    payload = {
        "file_type": "stereowiris_session_static",
        "schema_version": "stereowiris-resource-session-static-v1",
        "time_axis": {
            "primary": "recording_time_ns",
            "definition": "nanoseconds elapsed from the session recording start",
        },
        "resource_files": resource_files,
        "metadata": metadata,
    }
    _write_manifest(output, payload)


def _write_xlsx(
    output: Path,
    metadata: dict[str, object],
    distance_rows: list[dict[str, object]],
    *,
    camera_rows: dict[str, list[dict[str, object]]] | None = None,
    px4_rows: dict[str, list[dict[str, object]]] | None = None,
    gnss_base_rows: list[dict[str, object]] | None = None,
    extrinsics_comparison_rows: Sequence[dict[str, object]] | None = None,
) -> None:
    camera_rows = camera_rows or {}
    px4_rows = px4_rows or {}
    gnss_base_rows = gnss_base_rows or []
    extrinsics_comparison_rows = extrinsics_comparison_rows or ()
    sheet_rows = {
        "DistanceMeasurements": distance_rows,
        "CameraLeft": camera_rows.get("left", []),
        "CameraRight": camera_rows.get("right", []),
        "Px4Left": px4_rows.get("left", []),
        "Px4Right": px4_rows.get("right", []),
        "GnssBase": gnss_base_rows,
    }
    for sheet_name, _method, method_rows in _extrinsics_comparison_sheet_groups(
        extrinsics_comparison_rows
    ):
        sheet_rows[sheet_name] = method_rows
    previews = session_sheet_previews(
        metadata,
        distance_rows,
        camera_rows=camera_rows,
        px4_rows=px4_rows,
        gnss_base_rows=gnss_base_rows,
        extrinsics_comparison_rows=extrinsics_comparison_rows,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(dir=output.parent, prefix=f".{output.name}.", suffix=".tmp")
    os.close(fd)
    temp_path = Path(temp_name)
    try:
        with xlsxwriter.Workbook(temp_path) as workbook:
            header_format = workbook.add_format({"bold": True, "bg_color": "#e0e0e0"})
            for preview in previews:
                worksheet = workbook.add_worksheet(preview.name)
                if preview.kind == "key_value":
                    _write_key_value_sheet(worksheet, list(preview.entries), header_format)
                else:
                    _write_table(
                        worksheet,
                        list(preview.columns),
                        sheet_rows.get(preview.name, []),
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
    """Find retryable v2 sessions and legacy ``sessions/`` descendants."""

    retryable: list[Path] = []
    candidates: list[Path] = []
    roots: list[Path] = []
    direct_roots: dict[str, Path] = {}
    if recordings_root.is_dir():
        direct_roots = {
            child.name: child
            for child in recordings_root.iterdir()
            if child.name != "sessions"
            and not child.name.startswith(".")
            and child.is_dir()
            and not child.is_symlink()
        }
        roots.extend(direct_roots.values())
    legacy_root = recordings_root / "sessions"
    if legacy_root.is_dir() and not legacy_root.is_symlink():
        legacy_roots = {
            child.name: child
            for child in legacy_root.iterdir()
            if child.is_dir() and not child.is_symlink()
        }

        def has_spool(root: Path) -> bool:
            if (root / ".spool").is_dir():
                return True
            measurements = root / "measurements"
            return measurements.is_dir() and any(
                (child / ".spool").is_dir()
                for child in measurements.iterdir()
                if child.is_dir() and not child.is_symlink()
            )

        conflicts = sorted(
            name
            for name in set(direct_roots) & set(legacy_roots)
            if has_spool(direct_roots[name]) or has_spool(legacy_roots[name])
        )
        if conflicts:
            raise FileExistsError(
                "direct/legacy retry session ID collision: "
                + ", ".join(conflicts)
            )
        roots.extend(legacy_roots.values())
    for session_root in roots:
        candidates.append(session_root)
        measurements_root = session_root / "measurements"
        if measurements_root.is_dir() and not measurements_root.is_symlink():
            candidates.extend(
                child
                for child in measurements_root.iterdir()
                if child.is_dir() and not child.is_symlink()
            )
    for output_dir in candidates:
        spool_dir = output_dir / ".spool"
        if (
            not spool_dir.is_dir()
            or not (spool_dir / "session_metadata.json").is_file()
        ):
            continue
        status = ""
        failure_path = output_dir / ".workstation" / "save_failure.json"
        manifest_path = output_dir / "measurement.json"
        if failure_path.is_file():
            status = "failed"
        elif not manifest_path.is_file():
            manifest_path = output_dir / "session_manifest.json"
        if not status and manifest_path.is_file():
            try:
                status = str(json.loads(manifest_path.read_text(encoding="utf-8")).get("status", ""))
            except (OSError, ValueError):
                status = "failed"
        if status != "finalized":
            retryable.append(output_dir)
    return sorted(
        retryable,
        key=lambda path: (path.stat().st_mtime_ns, path.as_posix()),
        reverse=True,
    )


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
    drone_spool = spool_dir / "px4_drone.jsonl"
    if drone_spool.exists():
        px4_rows["drone"] = SpoolRows()
        px4_rows["drone"].open_existing(drone_spool)
    gnss_base_rows = SpoolRows()
    # Sessions recorded before this field existed have no gnss_base.jsonl;
    # open_existing() tolerates a missing path (empty rows, fresh append handle).
    gnss_base_rows.open_existing(spool_dir / "gnss_base.jsonl")
    extrinsics_comparison_rows = SpoolRows()
    # Sessions recorded before this field existed have no extrinsics_comparison.jsonl;
    # open_existing() tolerates a missing path (empty rows, fresh append handle).
    extrinsics_comparison_rows.open_existing(spool_dir / "extrinsics_comparison.jsonl")
    method_evaluation_bundle_rows = SpoolRows()
    # Pre multi-method sessions have no bundle spool; an empty stream remains
    # a valid recovery input and is materialized into eight empty method files.
    method_evaluation_bundle_rows.open_existing(
        spool_dir / "method_evaluation_bundles.jsonl"
    )
    recovered_spools = {
        "pair": pair_rows,
        "camera_left": camera_rows["left"],
        "camera_right": camera_rows["right"],
        "px4_left": px4_rows["left"],
        "px4_right": px4_rows["right"],
        "gnss_base": gnss_base_rows,
        "extrinsics_comparison": extrinsics_comparison_rows,
        "method_evaluation_bundles": method_evaluation_bundle_rows,
    }
    if "drone" in px4_rows:
        recovered_spools["px4_drone"] = px4_rows["drone"]
    recovery_warnings = [
        {"stream": stream, "warning": warning}
        for stream, rows in recovered_spools.items()
        for warning in rows.recovery_warnings
    ]
    recovery_artifacts = [
        {
            "stream": stream,
            "path": artifact.relative_to(output_dir).as_posix(),
        }
        for stream, rows in recovered_spools.items()
        for artifact in rows.recovery_artifacts
    ]
    if recovery_warnings or recovery_artifacts:
        metadata["spool_recovery"] = {
            "status": "RECOVERED_WITH_WARNINGS",
            "warnings": recovery_warnings,
            "artifacts": recovery_artifacts,
        }
    return RetryableSession(
        output_dir,
        metadata,
        pair_rows,
        camera_rows,
        px4_rows,
        gnss_base_rows,
        extrinsics_comparison_rows,
        method_evaluation_bundle_rows,
    )


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


def _v2_publish_root(output_dir: Path) -> Path:
    return output_dir / ".workstation" / "publish"


def _fsync_directory_path(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _ensure_v2_data_link(output_dir: Path) -> None:
    """Expose one data link and one generation-bound measurement manifest."""

    links = {
        output_dir / "data": Path(".workstation") / "publish" / "current" / "data",
        output_dir / "measurement.json": (
            Path(".workstation") / "publish" / "current" / "measurement.json"
        ),
    }
    changed = False
    for link, expected in links.items():
        if os.path.lexists(link):
            if not link.is_symlink() or Path(os.readlink(link)) != expected:
                raise OSError(f"measurement v2 path is not a managed link: {link}")
            continue
        link.symlink_to(expected)
        changed = True
    if changed:
        _fsync_directory_path(output_dir)


def _publish_v2_generation(output_dir: Path, generation_dir: Path) -> None:
    publish_root = _v2_publish_root(output_dir)
    _ensure_v2_data_link(output_dir)
    current = publish_root / "current"
    temporary = publish_root / f".current-{generation_dir.name}.tmp"
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(Path("generations") / generation_dir.name)
    try:
        os.replace(temporary, current)
        _fsync_directory_path(publish_root)
    finally:
        temporary.unlink(missing_ok=True)


def _v2_current_target(output_dir: Path) -> Path | None:
    current = _v2_publish_root(output_dir) / "current"
    if not os.path.lexists(current):
        return None
    if not current.is_symlink():
        raise OSError(f"measurement generation pointer is not a symlink: {current}")
    target = Path(os.readlink(current))
    if target.is_absolute() or len(target.parts) != 2 or target.parts[0] != "generations":
        raise OSError(f"measurement generation pointer target is invalid: {target}")
    if target.parts[1] in {"", ".", ".."}:
        raise OSError(f"measurement generation pointer target is invalid: {target}")
    return target


def _restore_v2_current_target(
    output_dir: Path,
    *,
    failed_generation: Path,
    previous_target: Path | None,
) -> None:
    """Roll back only the pointer published by the failed finalization."""

    publish_root = _v2_publish_root(output_dir)
    current = publish_root / "current"
    expected_failed_target = Path("generations") / failed_generation.name
    if _v2_current_target(output_dir) != expected_failed_target:
        raise OSError("measurement generation pointer changed during validation")
    if previous_target is None:
        current.unlink()
        _fsync_directory_path(publish_root)
        return
    temporary = publish_root / f".current-rollback-{failed_generation.name}.tmp"
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(previous_target)
    try:
        os.replace(temporary, current)
        _fsync_directory_path(publish_root)
    finally:
        temporary.unlink(missing_ok=True)


def _strict_json_object(path: Path) -> dict[str, object]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant is forbidden: {value}")

    payload = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=reject_constant,
    )
    if not isinstance(payload, dict):
        raise OSError(f"JSON artifact is not an object: {path}")
    return payload


def _validate_published_v2_generation(
    output_dir: Path,
    *,
    generation_id: str,
    expected_manifest: Mapping[str, object],
    jsonl_streams: Sequence[
        tuple[str, str, str, Sequence[dict[str, object]], str]
    ],
) -> None:
    """Verify the bytes reached through the atomically published pointer."""

    from Workstation.workstation_core.session_parquet import verify_parquet_artifact

    publish_root = _v2_publish_root(output_dir)
    current = publish_root / "current"
    expected_target = Path("generations") / generation_id
    if _v2_current_target(output_dir) != expected_target:
        raise OSError("published measurement generation pointer is stale")
    expected_root = (publish_root / expected_target).resolve(strict=True)
    published_root = current.resolve(strict=True)
    if published_root != expected_root or not published_root.is_dir():
        raise OSError("published measurement generation resolves outside its target")

    published_manifest = _strict_json_object(current / "measurement.json")
    if published_manifest != dict(expected_manifest):
        raise OSError("published measurement manifest changed after staging")
    artifact_details = published_manifest.get("artifact_details")
    if not isinstance(artifact_details, Mapping):
        raise OSError("published measurement artifact_details is missing")

    for artifact_key, role, relative_path, rows, normalization in jsonl_streams:
        declared = artifact_details.get(artifact_key)
        if not isinstance(declared, Mapping):
            raise OSError(f"published JSONL descriptor is missing: {artifact_key}")
        verified = _validate_authoritative_jsonl(
            current / relative_path,
            rows,
            role=role,
            artifact_path=relative_path,
            normalization=normalization,
        )
        if verified != dict(declared):
            raise OSError(f"published JSONL descriptor mismatch: {artifact_key}")

    for artifact_key in (
        "measurement_pairs",
        "camera_timeline",
        "pixhawk_timeline",
        "gnss_base_timeline",
        "geometry_comparison",
    ):
        declared = artifact_details.get(artifact_key)
        if not isinstance(declared, Mapping):
            raise OSError(f"published Parquet descriptor is missing: {artifact_key}")
        relative_path = str(declared.get("relative_path") or "")
        verified = verify_parquet_artifact(
            current / relative_path,
            artifact_key,
        ).to_mapping(relative_to=current)
        derived_from = declared.get("derived_from")
        if not isinstance(derived_from, list):
            raise OSError(f"Parquet derived_from is invalid: {artifact_key}")
        for source_descriptor in derived_from:
            if not isinstance(source_descriptor, Mapping) or not any(
                dict(source_descriptor) == dict(candidate)
                and candidate.get("authoritative") is True
                and candidate.get("media_type") == "application/x-ndjson"
                for candidate in artifact_details.values()
                if isinstance(candidate, Mapping)
            ):
                raise OSError(
                    f"Parquet derived_from does not resolve: {artifact_key}"
                )
        verified["derived_from"] = [dict(value) for value in derived_from]
        if verified != dict(declared):
            raise OSError(f"published Parquet descriptor mismatch: {artifact_key}")

    context = artifact_details.get("context")
    if not isinstance(context, Mapping):
        raise OSError("published context descriptor is missing")
    context_path = current / str(context.get("relative_path") or "")
    if (
        not context_path.is_file()
        or context_path.stat().st_size != int(context.get("size_bytes") or -1)
        or _sha256_path(context_path) != str(context.get("sha256") or "")
    ):
        raise OSError("published context bytes do not match their descriptor")


def _cleanup_v2_generations(output_dir: Path) -> None:
    publish_root = _v2_publish_root(output_dir)
    generations = publish_root / "generations"
    if not generations.is_dir():
        return
    current_name = ""
    current = publish_root / "current"
    if current.is_symlink():
        current_name = Path(os.readlink(current)).name
    for candidate in generations.iterdir():
        if candidate.is_dir() and candidate.name != current_name:
            shutil.rmtree(candidate, ignore_errors=True)


def _owning_session_root(output_dir: Path) -> Path:
    if output_dir.parent.name == "measurements":
        return output_dir.parent.parent
    return output_dir


def _validate_measurement_storage_identity(
    output_dir: Path,
    metadata: Mapping[str, object],
) -> tuple[str, str]:
    raw_session_id = str(metadata.get("session_id") or output_dir.name)
    # Reuse the public session path contract without creating the path.
    session_id = session_dir(Path("."), raw_session_id).name
    raw_measurement_id = str(metadata.get("measurement_id") or output_dir.name)
    if (
        not raw_measurement_id
        or raw_measurement_id in {".", ".."}
        or raw_measurement_id.startswith(".")
        or Path(raw_measurement_id).name != raw_measurement_id
        or "\\" in raw_measurement_id
        or "\x00" in raw_measurement_id
    ):
        raise ValueError("measurement_id must be one safe non-hidden path component")
    if output_dir.parent.name == "measurements":
        physical_session_id = output_dir.parent.parent.name
        if session_id != physical_session_id:
            raise ValueError(
                f"metadata session_id {session_id!r} does not own {output_dir}"
            )
        if raw_measurement_id != output_dir.name:
            raise ValueError(
                f"metadata measurement_id {raw_measurement_id!r} does not own {output_dir}"
            )
    elif session_id != output_dir.name:
        raise ValueError(
            f"metadata session_id {session_id!r} does not own {output_dir}"
        )
    return session_id, raw_measurement_id


_LEGACY_REVIEW_METHOD_SCHEMAS = frozenset(
    {
        "",
        "stereowiris-paired-frame-v1",
        "stereowiris-paired-frame-v2",
    }
)
_LEGACY_REVIEW_METHOD_ALIASES = {
    "pixhawk": "pixhawk_rtk_live",
    "checkerboard": "checkerboard_stereo",
    "calibration": "checkerboard_stereo",
}


def _review_row_is_explicitly_blocked(row: Mapping[str, object]) -> bool:
    pair_status = str(row.get("pair_status") or "").strip().lower()
    geometry_kind = str(row.get("measurement_geometry_kind") or "").strip().lower()
    return bool(
        row.get("blocked_pair_sha256")
        or geometry_kind == "unresolved"
        or "blocked" in pair_status
        or "no_ready" in pair_status
    )


def _review_measurement_method_id(row: Mapping[str, object]) -> str:
    """Resolve current and legacy method identity without truthiness fallback."""

    if "measurement_method_id" in row:
        return str(row.get("measurement_method_id") or "").strip()
    if _review_row_is_explicitly_blocked(row):
        return ""
    schema = str(row.get("schema_version") or "").strip()
    if schema not in _LEGACY_REVIEW_METHOD_SCHEMAS:
        return ""
    legacy = str(row.get("distance_mode") or "").strip().lower()
    legacy = _LEGACY_REVIEW_METHOD_ALIASES.get(legacy, legacy)
    return legacy if legacy in MEASUREMENT_METHOD_IDS else ""


def _review_result_row(row: Mapping[str, object]) -> dict[str, object]:
    return {
        "pair_id": row.get("pair_id") or row.get("measurement_attempt_id"),
        "utc_time": row.get("estimated_utc_time_ns") or row.get("recorded_unix_ns"),
        "recording_time_ns": row.get("recording_time_ns") or row.get("pair_app_mid_monotonic_ns"),
        "object_class": row.get("class_name"),
        "distance_m": row.get("distance_m"),
        "forward_m": row.get("baseline_center_forward_m"),
        "right_m": row.get("baseline_center_right_m"),
        "down_m": row.get("baseline_center_down_m"),
        "measurement_method_id": _review_measurement_method_id(row),
        "measurement_method_instance_id": row.get("measurement_method_instance_id"),
        "measurement_geometry_kind": row.get("measurement_geometry_kind"),
        "measurement_geometry_sha256": row.get("measurement_geometry_sha256"),
        "measurement_selection_reason": row.get("measurement_selection_reason"),
        "measurement_component_sources": row.get("measurement_component_sources"),
        "valid": row.get("measurement_valid"),
        "warning_code": row.get("warning_code") or row.get("warnings"),
    }


def _review_quality_row(row: Mapping[str, object]) -> dict[str, object]:
    return {
        "pair_id": row.get("pair_id") or row.get("measurement_attempt_id"),
        "sync_delta_ns": row.get("sync_delta_ns") or row.get("pair_delta_ns"),
        "left_confidence": row.get("left_confidence") or row.get("left_bbox_confidence"),
        "right_confidence": row.get("right_confidence") or row.get("right_bbox_confidence"),
        "vertical_disparity_px": row.get("vertical_disparity_px") or row.get("vertical_delta_px"),
        "reprojection_error_px": row.get("checkerboard_extrinsics_reprojection_error_px"),
        "gnss_fix_type": row.get("rtk_fix_quality") or row.get("left_gps_fix_type"),
        "geometry_state": row.get("measurement_geometry_kind"),
        "status": row.get("pair_status") or row.get("quality"),
    }


def _review_warning_rows(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    warnings: list[dict[str, object]] = []
    for row in rows:
        value = str(row.get("warnings") or row.get("warning_code") or "").strip()
        if not value:
            continue
        warnings.append(
            {
                "utc_time": row.get("estimated_utc_time_ns") or row.get("recorded_unix_ns"),
                "pair_id": row.get("pair_id") or row.get("measurement_attempt_id"),
                "subsystem": "measurement",
                "severity": "WARNING",
                "code": row.get("warning_code") or "MEASUREMENT_WARNING",
                "description": value,
                "impact": row.get("pair_status"),
            }
        )
    return warnings


def _review_capture_rows(metadata: Mapping[str, object]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for key, capture_type in (("pnp_hr_photo_capture", "HR"), ("oz_photo_capture", "OZ")):
        section = metadata.get(key)
        values = section.get("captures", ()) if isinstance(section, Mapping) else ()
        for capture in values if isinstance(values, (list, tuple)) else ():
            if not isinstance(capture, Mapping):
                continue
            cameras = capture.get("cameras")
            if isinstance(cameras, Mapping):
                for side, camera in cameras.items():
                    camera_row = camera if isinstance(camera, Mapping) else {}
                    rows.append(
                        {
                            "capture_id": capture.get("capture_id"),
                            "capture_type": capture_type,
                            "purpose": capture.get("purpose"),
                            "side": side,
                            "requested_at": camera_row.get("capture_requested_unix_ns") or capture.get("request_utc_ns"),
                            "completed_at": camera_row.get("completed_unix_ns"),
                            "relative_path": camera_row.get("ftp_local_path"),
                            "sha256": camera_row.get("sha256"),
                            "status": camera_row.get("state") or capture.get("state"),
                        }
                    )
            else:
                rows.append(
                    {
                        "capture_id": capture.get("capture_id"),
                        "capture_type": capture_type,
                        "purpose": capture.get("purpose"),
                        "status": capture.get("state"),
                    }
                )
    return rows


def _sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _measurement_review_mapping(
    measurement_root: Path,
    *,
    session_root: Path,
) -> dict[str, object] | None:
    from Workstation.workstation_core.session_parquet import (
        verify_parquet_artifact,
    )
    from Workstation.workstation_core.recording_reader import (
        read_measurement_stream,
    )

    data = measurement_root / "data"
    pairs = data / "measurement_pairs.parquet"
    context_path = data / "context.json"
    manifest_path = measurement_root / "measurement.json"
    if not pairs.exists() and not context_path.exists():
        return None
    if not pairs.is_file() or not context_path.is_file() or not manifest_path.is_file():
        raise OSError(
            f"incomplete finalized measurement data view: {measurement_root}"
        )
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(manifest, Mapping) or manifest.get("status") != "finalized":
            raise ValueError("measurement manifest is not finalized")
        declared_details = manifest.get("artifact_details")
        if not isinstance(declared_details, Mapping):
            raise ValueError("measurement manifest has no artifact details")
        rows = list(
            read_measurement_stream(
                measurement_root,
                "raw/stereo_measurements",
            ).rows
        )
        method_results: dict[str, list[dict[str, object]]] = {
            method_id: [] for method_id in MEASUREMENT_METHOD_IDS
        }
        from Workstation.workstation_core.method_evaluation import (
            MethodEvaluationOutcome,
        )

        for method_id in MEASUREMENT_METHOD_IDS:
            artifact_key = f"method_result_{method_id}"
            if not isinstance(declared_details.get(artifact_key), Mapping):
                continue
            method_rows = list(
                read_measurement_stream(
                    measurement_root,
                    f"raw/method_results/{method_id}",
                ).rows
            )
            for method_row in method_rows:
                outcome = MethodEvaluationOutcome.from_mapping(method_row)
                if outcome.method_id != method_id:
                    raise ValueError("method result stream contains the wrong method")
            method_results[method_id] = method_rows
        context_payload = json.loads(context_path.read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise OSError(
            f"cannot rebuild review from measurement {measurement_root.name}: {exc}"
        ) from exc
    metadata = context_payload.get("metadata", {}) if isinstance(context_payload, Mapping) else {}
    metadata = metadata if isinstance(metadata, Mapping) else {}
    measurement_id = str(metadata.get("measurement_id") or measurement_root.name)
    valid_count = sum(
        str(row.get("measurement_valid", "")).lower() in {"true", "1", "yes"}
        or row.get("measurement_valid") is True
        for row in rows
    )
    artifacts: list[dict[str, object]] = []
    nested_measurement = measurement_root.parent.name == "measurements"
    artifact_keys = {
        "measurement_pairs.parquet": "measurement_pairs",
        "camera_timeline.parquet": "camera_timeline",
        "pixhawk_timeline.parquet": "pixhawk_timeline",
        "gnss_base_timeline.parquet": "gnss_base_timeline",
        "geometry_comparison.parquet": "geometry_comparison",
    }
    for path in sorted(data.glob("*.parquet")):
        artifact_key = artifact_keys.get(path.name)
        if artifact_key is None:
            continue
        verified = verify_parquet_artifact(path, artifact_key)
        declared = declared_details.get(artifact_key)
        if not isinstance(declared, Mapping):
            raise OSError(f"manifest does not declare {artifact_key}")
        expected_values = {
            "size_bytes": verified.size_bytes,
            "row_count": verified.row_count,
            "sha256": verified.sha256,
            "schema_sha256": verified.schema_sha256,
        }
        mismatched = [
            key
            for key, actual in expected_values.items()
            if declared.get(key) != actual
        ]
        if mismatched:
            raise OSError(
                f"{path.name} disagrees with measurement manifest: {', '.join(mismatched)}"
            )
        relative = path.relative_to(measurement_root).as_posix()
        artifact_row: dict[str, object] = {
            "role": path.stem,
            "format": "parquet",
            "size_bytes": verified.size_bytes,
            "sha256": verified.sha256,
            "required": path.name == "measurement_pairs.parquet",
            "status": "VERIFIED",
        }
        artifact_row[
            "measurement_relative_path" if nested_measurement else "relative_path"
        ] = relative
        artifacts.append(artifact_row)
    context_relative = context_path.relative_to(measurement_root).as_posix()
    declared_context = declared_details.get("context")
    context_sha256 = _sha256_path(context_path)
    if (
        not isinstance(declared_context, Mapping)
        or declared_context.get("size_bytes") != context_path.stat().st_size
        or declared_context.get("sha256") != context_sha256
    ):
        raise OSError("context.json disagrees with measurement manifest")
    context_artifact: dict[str, object] = {
        "role": "measurement_context",
        "format": "json",
        "size_bytes": context_path.stat().st_size,
        "sha256": context_sha256,
        "required": True,
        "status": "VERIFIED",
    }
    context_artifact[
        "measurement_relative_path" if nested_measurement else "relative_path"
    ] = context_relative
    artifacts.append(context_artifact)
    latest_path = measurement_root / "latest_revision.json"
    if latest_path.is_file():
        try:
            latest = json.loads(latest_path.read_text(encoding="utf-8"))
            relative_manifest = str(latest.get("manifest_path") or "")
            package_manifest_path = (measurement_root / relative_manifest).resolve(
                strict=True
            )
            package_manifest_path.relative_to(measurement_root.resolve())
            package_manifest = json.loads(
                package_manifest_path.read_text(encoding="utf-8")
            )
            package_artifacts = package_manifest.get("artifacts", ())
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise OSError(
                f"cannot read sealed artifact inventory for {measurement_root.name}: {exc}"
            ) from exc
        existing_keys = {
            (str(row.get("role") or ""), str(row.get("measurement_relative_path") or row.get("relative_path") or ""))
            for row in artifacts
        }
        for descriptor in package_artifacts if isinstance(package_artifacts, list) else ():
            if not isinstance(descriptor, Mapping) or descriptor.get("status") != "present":
                continue
            relative = str(descriptor.get("relative_path") or "")
            if not relative:
                continue
            role = str(descriptor.get("role") or "artifact")
            display_relative = relative
            data_marker = "/data/"
            if data_marker in relative and relative.endswith(
                (".parquet", "context.json")
            ):
                display_relative = "data/" + relative.rsplit(data_marker, 1)[1]
            key = (role, display_relative)
            if key in existing_keys:
                continue
            file_row: dict[str, object] = {
                "role": role,
                "format": Path(display_relative).suffix.lstrip(".") or "file",
                "size_bytes": descriptor.get("size_bytes"),
                "sha256": descriptor.get("sha256"),
                "required": descriptor.get("required"),
                "status": "VERIFIED",
            }
            file_row[
                "measurement_relative_path" if nested_measurement else "relative_path"
            ] = display_relative
            artifacts.append(file_row)
            existing_keys.add(key)
    calculation = metadata.get("calculation")
    calculation = calculation if isinstance(calculation, Mapping) else {}
    intrinsics = metadata.get("intrinsics")
    intrinsics = intrinsics if isinstance(intrinsics, Mapping) else {}
    geometry = [
        {
            "method_id": calculation.get("measurement_method_id"),
            "profile_id": calculation.get("measurement_method_instance_id"),
            "geometry_sha256": metadata.get("active_geometry_sha256")
            or intrinsics.get("content_sha256"),
            "zoom": intrinsics.get("active_zoom_level"),
            "baseline_m": calculation.get("baseline_m"),
            "rotation_source": calculation.get("mount_assumption_model"),
            "translation_source": calculation.get("baseline_source"),
            "status": "READY" if valid_count else "NO_VALID_RESULT",
        }
    ]
    devices = [
        {
            "device_id": name,
            "device_type": kind,
            "sample_count": metadata.get(count_key),
            "status": "RECORDED" if int(metadata.get(count_key) or 0) else "NO_DATA",
        }
        for name, kind, count_key in (
            ("camera-left", "camera", "camera_left_gui_frame_count"),
            ("camera-right", "camera", "camera_right_gui_frame_count"),
            ("pixhawk-left", "pixhawk", "px4_left_sample_count"),
            ("pixhawk-right", "pixhawk", "px4_right_sample_count"),
            ("pixhawk-drone", "pixhawk", "px4_drone_sample_count"),
            ("gnss-base", "gnss", "gnss_base_sample_count"),
        )
    ]
    return {
        "measurement_id": measurement_id,
        "summary": {
            "status": "VALID" if valid_count else "NO_VALID_RESULT",
            "method": PIXHAWK_RTK_LIVE,
            "valid_pair_count": valid_count,
            "blocked_pair_count": len(rows) - valid_count,
            "warning_count": len(_review_warning_rows(rows)),
        },
        "results": [_review_result_row(row) for row in rows],
        "quality": [_review_quality_row(row) for row in rows],
        "geometry": geometry,
        "devices": devices,
        "captures": _review_capture_rows(metadata),
        "warnings": _review_warning_rows(rows),
        "artifacts": artifacts,
        "method_results": method_results,
    }


def _rebuild_session_review(session_root: Path, session_id: str) -> Path:
    from Workstation.workstation_core.session_review import write_session_review_xlsx

    measurements: list[dict[str, object]] = []
    session_summary: dict[str, object] = {}
    state_path = session_root / "session.json"
    if state_path.is_file():
        try:
            value = json.loads(state_path.read_text(encoding="utf-8"))
            if isinstance(value, Mapping):
                session_summary = dict(value)
        except (OSError, ValueError):
            session_summary = {}
    required_measurements = {
        str(row.get("measurement_id") or "")
        for row in session_summary.get("measurements", ()) or ()
        if isinstance(row, Mapping)
        and str(row.get("state") or "") in {"SEALED_COMPLETE", "SEALED_INCOMPLETE"}
        and row.get("measurement_id")
    }
    measurement_parent = session_root / "measurements"
    if measurement_parent.is_dir():
        roots = sorted(
            child for child in measurement_parent.iterdir() if child.is_dir()
        )
    else:
        roots = [session_root]
    for root in roots:
        review = _measurement_review_mapping(root, session_root=session_root)
        if review is not None:
            measurements.append(review)
    included = {str(row.get("measurement_id") or "") for row in measurements}
    missing = sorted(required_measurements - included)
    if missing:
        raise OSError(
            "finalized measurements are missing from review inputs: "
            + ", ".join(missing)
        )
    return write_session_review_xlsx(
        session_root,
        session_id=session_id,
        measurements=measurements,
        session_summary=session_summary,
    )


def rebuild_session_review(session_root: Path, session_id: str) -> Path:
    """Regenerate the derived session workbook and persist its status."""

    session_root = Path(session_root)
    status_path = session_root / ".workstation" / "review.json"
    try:
        output = _rebuild_session_review(session_root, session_id)
    except Exception as exc:
        _write_manifest(
            status_path,
            {
                "status": "failed",
                "path": "review.xlsx",
                "error": str(exc),
                "regenerable": True,
            },
        )
        _fsync_directory_path(status_path.parent)
        raise
    _write_manifest(
        status_path,
        {
            "status": "generated",
            "regenerable": True,
            "artifact": {
                "role": "derived/review",
                "media_type": (
                    "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"
                ),
                "relative_path": output.relative_to(session_root).as_posix(),
                "size_bytes": output.stat().st_size,
                "sha256": _sha256_path(output),
                "row_count": None,
                "schema_version": "workstation-session-review-xlsx-v1",
                "authoritative": False,
                "derived_from": [],
                "verified": True,
            },
        },
    )
    _fsync_directory_path(status_path.parent)
    return output


def _save_session_files_legacy(
    output_dir: Path,
    *,
    metadata: dict[str, object],
    pair_rows: Iterable[dict[str, object]],
    camera_rows: dict[str, Iterable[dict[str, object]]] | None = None,
    px4_rows: dict[str, Iterable[dict[str, object]]] | None = None,
    gnss_base_rows: Iterable[dict[str, object]] | None = None,
    extrinsics_comparison_rows: Iterable[dict[str, object]] | None = None,
) -> SavedSessionFiles:
    output_dir.mkdir(parents=True, exist_ok=True)
    session_id = Path(str(metadata.get("session_id") or output_dir.name)).name
    rows = _project_pair_typed_fields(_normalized_pair_rows(pair_rows))
    camera_rows = camera_rows or {}
    px4_rows = px4_rows or {}
    gnss_base_full_rows = _normalized_generic_rows(gnss_base_rows or [])
    extrinsics_comparison_full_rows = _normalized_generic_rows(extrinsics_comparison_rows or [])
    camera_full_rows = {
        side: _normalized_generic_rows(camera_rows.get(side, []))
        for side in ("left", "right")
    }
    px4_sides = ("left", "right", *(("drone",) if "drone" in px4_rows else ()))
    px4_full_rows = {
        side: _normalized_generic_rows(px4_rows.get(side, []))
        for side in px4_sides
    }
    compact_distance_rows = compact_distance_measurement_rows(rows, metadata)
    compact_camera_rows = {
        side: compact_camera_timeline_rows(camera_full_rows[side], metadata)
        for side in ("left", "right")
    }
    compact_px4_rows = {
        side: compact_px4_timeline_rows(px4_full_rows[side], metadata)
        for side in ("left", "right")
    }
    compact_rtk_rows = _compact_rtk_rows(px4_full_rows, metadata)
    compact_gnss_base_row_list = compact_gnss_base_rows(gnss_base_full_rows, metadata)
    static_output = output_dir / f"{session_id}_session_static.json"
    distance_csv_output = output_dir / f"{session_id}_distance_measurements.csv"
    distance_parquet_output = output_dir / f"{session_id}_distance_measurements.parquet"
    xlsx_output = output_dir / f"{session_id}.xlsx"
    camera_left_output = output_dir / f"{session_id}_camera_left.csv"
    camera_left_parquet_output = output_dir / f"{session_id}_camera_left.parquet"
    camera_right_output = output_dir / f"{session_id}_camera_right.csv"
    camera_right_parquet_output = output_dir / f"{session_id}_camera_right.parquet"
    px4_left_output = output_dir / f"{session_id}_px4_left.csv"
    px4_left_parquet_output = output_dir / f"{session_id}_px4_left.parquet"
    px4_right_output = output_dir / f"{session_id}_px4_right.csv"
    px4_right_parquet_output = output_dir / f"{session_id}_px4_right.parquet"
    px4_left_xlsx_output = output_dir / f"{session_id}_px4_left.xlsx"
    px4_right_xlsx_output = output_dir / f"{session_id}_px4_right.xlsx"
    session_rtk_xlsx_output = output_dir / f"{session_id}_session_rtk.xlsx"
    gnss_base_output = output_dir / f"{session_id}_gnss_base.csv"
    gnss_base_parquet_output = output_dir / f"{session_id}_gnss_base.parquet"
    extrinsics_comparison_output = output_dir / f"{session_id}_extrinsics_comparison.csv"
    extrinsics_comparison_parquet_output = output_dir / f"{session_id}_extrinsics_comparison.parquet"
    raw_pair_output = output_dir / f"{session_id}_raw_pair_rows.jsonl"
    raw_camera_left_output = output_dir / f"{session_id}_raw_camera_left_rows.jsonl"
    raw_camera_right_output = output_dir / f"{session_id}_raw_camera_right_rows.jsonl"
    raw_px4_left_output = output_dir / f"{session_id}_raw_px4_left_rows.jsonl"
    raw_px4_right_output = output_dir / f"{session_id}_raw_px4_right_rows.jsonl"
    raw_px4_drone_output = output_dir / f"{session_id}_raw_px4_drone_rows.jsonl"
    raw_gnss_base_output = output_dir / f"{session_id}_raw_gnss_base_rows.jsonl"
    raw_extrinsics_comparison_output = output_dir / f"{session_id}_raw_extrinsics_comparison_rows.jsonl"
    manifest_output = output_dir / "session_manifest.json"
    resource_files = {
        "session_static": static_output.name,
        "distance_measurements_csv": distance_csv_output.name,
        "distance_measurements_parquet": distance_parquet_output.name,
        "camera_left_csv": camera_left_output.name,
        "camera_left_parquet": camera_left_parquet_output.name,
        "camera_right_csv": camera_right_output.name,
        "camera_right_parquet": camera_right_parquet_output.name,
        "px4_left_csv": px4_left_output.name,
        "px4_left_parquet": px4_left_parquet_output.name,
        "px4_right_csv": px4_right_output.name,
        "px4_right_parquet": px4_right_parquet_output.name,
        "px4_left_xlsx": px4_left_xlsx_output.name,
        "px4_right_xlsx": px4_right_xlsx_output.name,
        "session_rtk_xlsx": session_rtk_xlsx_output.name,
        "gnss_base_csv": gnss_base_output.name,
        "gnss_base_parquet": gnss_base_parquet_output.name,
        "extrinsics_comparison_csv": extrinsics_comparison_output.name,
        "extrinsics_comparison_parquet": extrinsics_comparison_parquet_output.name,
        "raw_pair_rows_jsonl": raw_pair_output.name,
        "raw_camera_left_rows_jsonl": raw_camera_left_output.name,
        "raw_camera_right_rows_jsonl": raw_camera_right_output.name,
        "raw_px4_left_rows_jsonl": raw_px4_left_output.name,
        "raw_px4_right_rows_jsonl": raw_px4_right_output.name,
        "raw_gnss_base_rows_jsonl": raw_gnss_base_output.name,
        "raw_extrinsics_comparison_rows_jsonl": raw_extrinsics_comparison_output.name,
        "workbook": xlsx_output.name,
    }
    if "drone" in px4_full_rows:
        resource_files["raw_px4_drone_rows_jsonl"] = raw_px4_drone_output.name
    if not WRITE_PARQUET_ARTIFACTS:
        for key in (
            "distance_measurements_parquet",
            "camera_left_parquet",
            "camera_right_parquet",
            "px4_left_parquet",
            "px4_right_parquet",
            "gnss_base_parquet",
            "extrinsics_comparison_parquet",
        ):
            resource_files.pop(key, None)
    _cleanup_finalize_orphans(output_dir)
    generation_id = session_id_now()
    generations_root = output_dir / ".generations"
    generations_root.mkdir(parents=True, exist_ok=True)
    staging_dir = generations_root / f"{generation_id}.preparing"
    staging_dir.mkdir()
    csv_names_and_rows = (
        (distance_csv_output.name, compact_distance_rows, DISTANCE_MEASUREMENT_COLUMNS),
        (camera_left_output.name, compact_camera_rows["left"], CAMERA_TIMELINE_COMPACT_COLUMNS),
        (camera_right_output.name, compact_camera_rows["right"], CAMERA_TIMELINE_COMPACT_COLUMNS),
        (px4_left_output.name, compact_px4_rows["left"], PX4_TIMELINE_COMPACT_COLUMNS),
        (px4_right_output.name, compact_px4_rows["right"], PX4_TIMELINE_COMPACT_COLUMNS),
        (gnss_base_output.name, compact_gnss_base_row_list, GNSS_BASE_COMPACT_COLUMNS),
        (
            extrinsics_comparison_output.name,
            extrinsics_comparison_full_rows,
            EXTRINSICS_COMPARISON_COMPACT_COLUMNS,
        ),
    )
    parquet_names_and_rows = (
        (
            (distance_parquet_output.name, compact_distance_rows, DISTANCE_MEASUREMENT_COLUMNS),
            (camera_left_parquet_output.name, compact_camera_rows["left"], CAMERA_TIMELINE_COMPACT_COLUMNS),
            (camera_right_parquet_output.name, compact_camera_rows["right"], CAMERA_TIMELINE_COMPACT_COLUMNS),
            (px4_left_parquet_output.name, compact_px4_rows["left"], PX4_TIMELINE_COMPACT_COLUMNS),
            (px4_right_parquet_output.name, compact_px4_rows["right"], PX4_TIMELINE_COMPACT_COLUMNS),
            (gnss_base_parquet_output.name, compact_gnss_base_row_list, GNSS_BASE_COMPACT_COLUMNS),
            (
                extrinsics_comparison_parquet_output.name,
                extrinsics_comparison_full_rows,
                EXTRINSICS_COMPARISON_COMPACT_COLUMNS,
            ),
        )
        if WRITE_PARQUET_ARTIFACTS
        else ()
    )
    raw_jsonl_names_and_rows = [
        (
            raw_pair_output.name,
            rows,
            "raw_pair_rows_jsonl",
            "_normalized_pair_rows",
        ),
        (
            raw_camera_left_output.name,
            camera_full_rows["left"],
            "raw_camera_left_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_camera_right_output.name,
            camera_full_rows["right"],
            "raw_camera_right_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_px4_left_output.name,
            px4_full_rows["left"],
            "raw_px4_left_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_px4_right_output.name,
            px4_full_rows["right"],
            "raw_px4_right_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_gnss_base_output.name,
            gnss_base_full_rows,
            "raw_gnss_base_rows_jsonl",
            "_normalized_generic_rows",
        ),
        (
            raw_extrinsics_comparison_output.name,
            extrinsics_comparison_full_rows,
            "raw_extrinsics_comparison_rows_jsonl",
            "_normalized_generic_rows",
        ),
    ]
    if "drone" in px4_full_rows:
        raw_jsonl_names_and_rows.append(
            (
                raw_px4_drone_output.name,
                px4_full_rows["drone"],
                "raw_px4_drone_rows_jsonl",
                "_normalized_generic_rows",
            )
        )
    manifest: dict[str, object] = {
        "session_id": session_id,
        "status": "preparing",
        "resource_files_authoritative": True,
        "resource_files": resource_files,
        "time_axis": "recording_time_ns",
        "artifacts": {},
        "artifact_details": {},
        "authoritative_raw_artifacts_verified": False,
        "artifact_roles": {
            static_output.name: "session_static_metadata",
            distance_csv_output.name: "distance_measurements_csv",
            distance_parquet_output.name: "distance_measurements_parquet",
            camera_left_output.name: "camera_timeline_csv",
            camera_left_parquet_output.name: "camera_timeline_parquet",
            camera_right_output.name: "camera_timeline_csv",
            camera_right_parquet_output.name: "camera_timeline_parquet",
            px4_left_output.name: "px4_timeline_csv",
            px4_left_parquet_output.name: "px4_timeline_parquet",
            px4_right_output.name: "px4_timeline_csv",
            px4_right_parquet_output.name: "px4_timeline_parquet",
            px4_left_xlsx_output.name: "px4_left_workbook",
            px4_right_xlsx_output.name: "px4_right_workbook",
            session_rtk_xlsx_output.name: "session_rtk_workbook",
            gnss_base_output.name: "gnss_base_timeline_csv",
            gnss_base_parquet_output.name: "gnss_base_timeline_parquet",
            extrinsics_comparison_output.name: "extrinsics_comparison_csv",
            extrinsics_comparison_parquet_output.name: "extrinsics_comparison_parquet",
            raw_pair_output.name: "raw_pair_rows_jsonl",
            raw_camera_left_output.name: "raw_camera_left_rows_jsonl",
            raw_camera_right_output.name: "raw_camera_right_rows_jsonl",
            raw_px4_left_output.name: "raw_px4_left_rows_jsonl",
            raw_px4_right_output.name: "raw_px4_right_rows_jsonl",
            raw_gnss_base_output.name: "raw_gnss_base_rows_jsonl",
            raw_extrinsics_comparison_output.name: "raw_extrinsics_comparison_rows_jsonl",
            xlsx_output.name: "integrated_review_workbook",
        },
    }
    if "drone" in px4_full_rows:
        manifest["artifact_roles"][raw_px4_drone_output.name] = "raw_px4_drone_rows_jsonl"  # type: ignore[index]
    if not WRITE_PARQUET_ARTIFACTS:
        artifact_roles = manifest["artifact_roles"]  # type: ignore[assignment]
        for name in (
            distance_parquet_output.name,
            camera_left_parquet_output.name,
            camera_right_parquet_output.name,
            px4_left_parquet_output.name,
            px4_right_parquet_output.name,
            gnss_base_parquet_output.name,
            extrinsics_comparison_parquet_output.name,
        ):
            artifact_roles.pop(name, None)  # type: ignore[union-attr]
    authoritative_raw_artifacts_verified = False
    try:
        _write_session_static_json(
            staging_dir / static_output.name,
            metadata,
            resource_files=resource_files,
        )
        manifest["artifacts"][static_output.name] = "prepared"  # type: ignore[index]
        for name, source_rows, role, normalization in raw_jsonl_names_and_rows:
            staged_path = staging_dir / name
            _write_authoritative_jsonl(staged_path, source_rows)
            detail = _validate_authoritative_jsonl(
                staged_path,
                source_rows,
                role=role,
                artifact_path=name,
                normalization=normalization,
            )
            manifest["artifacts"][name] = "prepared"  # type: ignore[index]
            manifest["artifact_details"][name] = detail  # type: ignore[index]
        authoritative_raw_artifacts_verified = True
        manifest["authoritative_raw_artifacts_verified"] = True
        for name, source_rows, required in csv_names_and_rows:
            staged_path = staging_dir / name
            write_dynamic_csv(staged_path, source_rows, required_columns=required)
            manifest["artifacts"][name] = "prepared"  # type: ignore[index]
        for name, source_rows, columns in parquet_names_and_rows:
            staged_path = staging_dir / name
            write_parquet(staged_path, source_rows, columns=columns)
            manifest["artifacts"][name] = "prepared"  # type: ignore[index]
        _write_px4_workbook(staging_dir / px4_left_xlsx_output.name, compact_px4_rows["left"])
        _write_px4_workbook(staging_dir / px4_right_xlsx_output.name, compact_px4_rows["right"])
        _write_rtk_workbook(staging_dir / session_rtk_xlsx_output.name, compact_rtk_rows)
        for name in (px4_left_xlsx_output.name, px4_right_xlsx_output.name, session_rtk_xlsx_output.name):
            manifest["artifacts"][name] = "prepared"  # type: ignore[index]
    except Exception as exc:
        _scope_unpublished_artifact_details(manifest, artifact_root=staging_dir)
        manifest.update(
            status="failed",
            phase="prepare_resource_files",
            error=f"resource file preparation failed: {exc}",
            retryable=True,
            staging_dir=str(staging_dir),
        )
        _write_manifest(manifest_output, manifest)
        raise

    manifest.update(status="prepared", phase="prepare_xlsx", generation_id=generation_id)
    _write_manifest(manifest_output, manifest)
    error = ""
    try:
        _write_xlsx(
            staging_dir / xlsx_output.name,
            metadata,
            compact_distance_rows,
            camera_rows=compact_camera_rows,
            px4_rows=compact_px4_rows,
            gnss_base_rows=compact_gnss_base_row_list,
            extrinsics_comparison_rows=extrinsics_comparison_full_rows,
        )
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
            [static_output.name]
            + [name for name, _rows, _required in csv_names_and_rows]
            + [name for name, _rows, _columns in parquet_names_and_rows]
            + [name for name, _rows, _role, _normalization in raw_jsonl_names_and_rows]
            + [px4_left_xlsx_output.name, px4_right_xlsx_output.name, session_rtk_xlsx_output.name]
            + [xlsx_output.name],
        )
    except Exception as exc:
        _scope_unpublished_artifact_details(manifest, artifact_root=generation_dir)
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
        context_json=static_output,
        measurement_pairs_parquet=distance_parquet_output,
        camera_timeline_parquet=camera_left_parquet_output,
        pixhawk_timeline_parquet=px4_left_parquet_output,
        gnss_base_timeline_parquet=gnss_base_parquet_output,
        geometry_comparison_parquet=extrinsics_comparison_parquet_output,
        review_xlsx=xlsx_output,
        manifest=manifest_output,
        finalized=not error,
        error=error,
    )
    if saved.finalized and authoritative_raw_artifacts_verified:
        for source_rows in (
            pair_rows,
            *camera_rows.values(),
            *px4_rows.values(),
            gnss_base_rows,
            extrinsics_comparison_rows,
        ):
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


def save_session_files(
    output_dir: Path,
    *,
    metadata: dict[str, object],
    pair_rows: Iterable[dict[str, object]],
    camera_rows: dict[str, Iterable[dict[str, object]]] | None = None,
    px4_rows: dict[str, Iterable[dict[str, object]]] | None = None,
    gnss_base_rows: Iterable[dict[str, object]] | None = None,
    extrinsics_comparison_rows: Iterable[dict[str, object]] | None = None,
    method_evaluation_bundle_rows: Iterable[dict[str, object]] | None = None,
) -> SavedSessionFiles:
    """Finalize canonical JSONL plus derived Parquet in one generation.

    The closed spool state is materialized once.  Complete source rows are
    written and reread as canonical JSONL before any derived Parquet is built.
    The generation is published atomically, verified again through the
    ``current`` pointer, and only then may the retry spool be discarded.
    """

    from Workstation.workstation_core.session_parquet import (
        CAMERA_TIMELINE_FILENAME,
        GEOMETRY_COMPARISON_FILENAME,
        GNSS_BASE_TIMELINE_FILENAME,
        MEASUREMENT_PAIRS_FILENAME,
        PX4_TIMELINE_FILENAME,
        finalize_session_parquet_artifacts,
    )
    from Workstation.workstation_core.geometry_context import (
        materialize_geometry_contexts,
    )
    from Workstation.workstation_core.time_sync_epoch import (
        materialize_time_sync_epochs,
    )

    output_dir = Path(output_dir)
    session_id, measurement_id = _validate_measurement_storage_identity(
        output_dir, metadata
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    pair_full_rows = _normalized_pair_rows(pair_rows)
    camera_sources = camera_rows or {}
    px4_sources = px4_rows or {}
    camera_full_rows = {
        side: _normalized_generic_rows(camera_sources.get(side, ()))
        for side in ("left", "right")
    }
    px4_full_rows = {
        source: _normalized_generic_rows(values)
        for source, values in px4_sources.items()
    }
    for source in ("left", "right"):
        px4_full_rows.setdefault(source, [])
    gnss_rows = _normalized_generic_rows(gnss_base_rows or ())
    geometry_rows = _normalized_generic_rows(
        extrinsics_comparison_rows or ()
    )
    method_bundle_rows, method_result_rows = _materialize_method_evaluation_rows(
        method_evaluation_bundle_rows or ()
    )
    def jsonl_stream(
        artifact_key: str,
        rows: Sequence[dict[str, object]],
        normalization: str,
    ) -> tuple[str, str, str, Sequence[dict[str, object]], str]:
        role, relative_path = AUTHORITATIVE_JSONL_STREAMS[artifact_key]
        return artifact_key, role, relative_path, rows, normalization

    geometry_context_rows: list[dict[str, object]] = []
    time_sync_epoch_rows: list[dict[str, object]] = []
    jsonl_streams = [
        jsonl_stream(
            "raw_stereo_measurements",
            pair_full_rows,
            "_normalized_pair_rows",
        ),
        jsonl_stream(
            "raw_camera_left",
            camera_full_rows["left"],
            "_normalized_generic_rows",
        ),
        jsonl_stream(
            "raw_camera_right",
            camera_full_rows["right"],
            "_normalized_generic_rows",
        ),
        jsonl_stream(
            "raw_px4_left",
            px4_full_rows["left"],
            "_normalized_generic_rows",
        ),
        jsonl_stream(
            "raw_px4_right",
            px4_full_rows["right"],
            "_normalized_generic_rows",
        ),
        jsonl_stream(
            "raw_gnss_base",
            gnss_rows,
            "_normalized_generic_rows",
        ),
        jsonl_stream(
            "geometry_contexts",
            geometry_context_rows,
            "geometry_context_record_v1",
        ),
        jsonl_stream(
            "time_sync_epochs",
            time_sync_epoch_rows,
            "time_sync_epoch_record_v1",
        ),
        jsonl_stream(
            "method_evaluation_bundles",
            method_bundle_rows,
            "workstation-method-evaluation-bundle-v1",
        ),
    ]
    jsonl_streams.extend(
        jsonl_stream(
            f"method_result_{method_id}",
            method_result_rows[method_id],
            "workstation-method-evaluation-v1",
        )
        for method_id in MEASUREMENT_METHOD_IDS
    )
    if "drone" in px4_full_rows:
        jsonl_streams.append(
            jsonl_stream(
                "raw_px4_drone",
                px4_full_rows["drone"],
                "_normalized_generic_rows",
            )
        )
    if geometry_rows:
        jsonl_streams.append(
            jsonl_stream(
                "raw_extrinsics_comparison",
                geometry_rows,
                "_normalized_generic_rows",
            )
        )

    manifest_output = output_dir / "measurement.json"
    generation_id = session_id_now()
    publish_root = _v2_publish_root(output_dir)
    generations_root = publish_root / "generations"
    generations_root.mkdir(parents=True, exist_ok=True)
    staging_dir = generations_root / f"{generation_id}.preparing"
    staging_dir.mkdir()
    data_dir = staging_dir / "data"
    data_dir.mkdir()
    previous_target: Path | None = None
    generation_dir: Path | None = None

    try:
        previous_target = _v2_current_target(output_dir)
        for source_rows in (
            pair_full_rows,
            *camera_full_rows.values(),
            *px4_full_rows.values(),
            gnss_rows,
            geometry_rows,
            method_bundle_rows,
            *method_result_rows.values(),
        ):
            _apply_measurement_package_identity(
                source_rows,
                measurement_id=measurement_id,
            )
        _apply_terminal_record_identities(pair_full_rows)
        geometry_context_rows.extend(
            materialize_geometry_contexts(pair_full_rows)
        )
        time_sync_epoch_rows.extend(
            materialize_time_sync_epochs(
                camera_rows=camera_full_rows,
                terminal_rows=pair_full_rows,
                metadata=metadata,
                session_id=session_id,
                measurement_package_id=measurement_id,
            )
        )
        parquet_pair_rows = _project_pair_typed_fields(
            [dict(row) for row in pair_full_rows]
        )
        parquet_camera_rows = {
            side: _project_camera_typed_fields([dict(row) for row in rows])
            for side, rows in camera_full_rows.items()
        }
        parquet_px4_rows = {
            source: _project_px4_typed_fields([dict(row) for row in rows])
            for source, rows in px4_full_rows.items()
        }
        jsonl_details: dict[str, dict[str, object]] = {}
        for artifact_key, role, relative_path, rows, normalization in jsonl_streams:
            staged_path = staging_dir / relative_path
            _write_authoritative_jsonl(staged_path, rows)
            jsonl_details[artifact_key] = _validate_authoritative_jsonl(
                staged_path,
                rows,
                role=role,
                artifact_path=relative_path,
                normalization=normalization,
            )
        artifacts = finalize_session_parquet_artifacts(
            data_dir,
            measurement_pairs=parquet_pair_rows,
            camera_rows=parquet_camera_rows,
            px4_rows=parquet_px4_rows,
            gnss_base_rows=gnss_rows,
            geometry_comparison_rows=geometry_rows,
        )
        resource_files = {
            **{
                artifact_key: relative_path
                for artifact_key, _role, relative_path, _rows, _normalization in jsonl_streams
            },
            "measurement_pairs_parquet": f"data/{MEASUREMENT_PAIRS_FILENAME}",
            "camera_timeline_parquet": f"data/{CAMERA_TIMELINE_FILENAME}",
            "pixhawk_timeline_parquet": f"data/{PX4_TIMELINE_FILENAME}",
            "gnss_base_timeline_parquet": f"data/{GNSS_BASE_TIMELINE_FILENAME}",
            "geometry_comparison_parquet": f"data/{GEOMETRY_COMPARISON_FILENAME}",
            "context": "data/context.json",
        }
        context_payload = {
            "file_type": "workstation_measurement_context",
            "schema_version": "workstation-measurement-context-v2",
            "session_id": session_id,
            "measurement_id": measurement_id,
            "measurement_package_id": measurement_id,
            "time_axis": {
                "primary": "recording_time_ns",
                "definition": "nanoseconds elapsed from measurement recording start",
            },
            "resource_files": resource_files,
            "geometry_context_registry": resource_files["geometry_contexts"],
            "time_sync_epoch_registry": resource_files["time_sync_epochs"],
            "metadata": metadata,
        }
        context_path = data_dir / "context.json"
        _write_manifest(context_path, context_payload)
        parquet_details = artifacts.to_mapping(relative_to=staging_dir)
        parquet_derivations = {
            "measurement_pairs": ("raw_stereo_measurements",),
            "camera_timeline": ("raw_camera_left", "raw_camera_right"),
            "pixhawk_timeline": (
                "raw_px4_left",
                "raw_px4_right",
                "raw_px4_drone",
            ),
            "gnss_base_timeline": ("raw_gnss_base",),
            "geometry_comparison": ("raw_extrinsics_comparison",),
        }
        for artifact_key, source_keys in parquet_derivations.items():
            parquet_details[artifact_key]["derived_from"] = [
                dict(jsonl_details[source_key])
                for source_key in source_keys
                if source_key in jsonl_details
            ]
        context_detail = {
            "role": "measurement_context",
            "media_type": "application/json",
            "relative_path": "data/context.json",
            "size_bytes": context_path.stat().st_size,
            "sha256": _sha256_path(context_path),
            "row_count": None,
            "schema_version": "workstation-measurement-context-v2",
            "authoritative": True,
            "derived_from": [],
            "verified": True,
        }
        required_jsonl_roles = [
            AUTHORITATIVE_JSONL_STREAMS[key][0]
            for key in (
                "raw_stereo_measurements",
                "raw_camera_left",
                "raw_camera_right",
                "raw_px4_left",
                "raw_px4_right",
                "raw_gnss_base",
                "geometry_contexts",
                "time_sync_epochs",
                "method_evaluation_bundles",
                *(
                    f"method_result_{method_id}"
                    for method_id in MEASUREMENT_METHOD_IDS
                ),
            )
        ]
        generation_manifest = {
            "schema": MEASUREMENT_STORAGE_SCHEMA,
            "schema_version": MEASUREMENT_STORAGE_SCHEMA_VERSION,
            "session_id": session_id,
            "measurement_id": measurement_id,
            "measurement_package_id": measurement_id,
            "status": "prepared",
            "generation_id": generation_id,
            "authority_contract": {
                "structured_records": "canonical_jsonl",
                "binary_evidence": "native_bytes",
                "static_metadata": "json",
                "parquet": "derived_analysis",
                "xlsx": "derived_review",
            },
            "resource_files_authoritative": False,
            "resource_files": resource_files,
            "artifact_details": {
                **jsonl_details,
                **parquet_details,
                "context": context_detail,
            },
            "authoritative_jsonl_artifacts_verified": True,
            "parquet_artifacts_verified": True,
            "authoritative_parquet_artifacts_verified": False,
            "completeness": {
                "complete": True,
                "required_authoritative_roles": required_jsonl_roles,
                "missing_required_authoritative_roles": [],
            },
            "time_axis": "recording_time_ns",
        }
        manifest = {
            **generation_manifest,
            "status": "finalized",
            "phase": "published",
            "retryable": False,
            "published_pointer": ".workstation/publish/current",
            "data_view": "data",
            "publish_contract": (
                "data and measurement.json resolve through one atomically replaced generation pointer"
            ),
        }
        _write_manifest(staging_dir / "generation_manifest.json", generation_manifest)
        _write_manifest(staging_dir / "measurement.json", manifest)
        _fsync_directory_path(data_dir)
        _fsync_directory_path(staging_dir)
        generation_dir = generations_root / generation_id
        os.replace(staging_dir, generation_dir)
        _fsync_directory_path(generations_root)
        _publish_v2_generation(output_dir, generation_dir)
        _validate_published_v2_generation(
            output_dir,
            generation_id=generation_id,
            expected_manifest=manifest,
            jsonl_streams=jsonl_streams,
        )
    except Exception as exc:
        rollback_error = ""
        if generation_dir is not None:
            try:
                current_target = _v2_current_target(output_dir)
                if current_target == Path("generations") / generation_dir.name:
                    _restore_v2_current_target(
                        output_dir,
                        failed_generation=generation_dir,
                        previous_target=previous_target,
                    )
            except Exception as rollback_exc:
                rollback_error = str(rollback_exc)
        failure = {
            "schema": MEASUREMENT_STORAGE_SCHEMA,
            "schema_version": MEASUREMENT_STORAGE_SCHEMA_VERSION,
            "session_id": session_id,
            "measurement_id": measurement_id,
            "status": "failed",
            "phase": "prepare_validate_or_publish_generation",
            "error": (
                str(exc)
                if not rollback_error
                else f"{exc}; generation pointer rollback failed: {rollback_error}"
            ),
            "retryable": True,
            "generation_id": generation_id,
        }
        failure_path = output_dir / ".workstation" / "save_failure.json"
        _write_manifest(failure_path, failure)
        _fsync_directory_path(failure_path.parent)
        if rollback_error:
            raise OSError(failure["error"]) from exc
        raise
    failure_path = output_dir / ".workstation" / "save_failure.json"
    failure_path.unlink(missing_ok=True)
    _fsync_directory_path(failure_path.parent)

    session_root = _owning_session_root(output_dir)
    review_path = session_root / "review.xlsx"
    review_status_path = session_root / ".workstation" / "review.json"
    review_error = ""
    review_pending = output_dir.parent.name == "measurements"
    if review_pending:
        _write_manifest(
            review_status_path,
            {
                "status": "pending_session_close",
                "path": "review.xlsx",
                "regenerable": True,
            },
        )
        _fsync_directory_path(review_status_path.parent)
    else:
        try:
            review_path = rebuild_session_review(session_root, session_id)
        except Exception as exc:
            # The workbook is a reproducible operator view, never the
            # measurement source of truth. Preserve an older review and keep
            # typed evidence finalized when regeneration fails.
            review_error = str(exc)

    data_view = output_dir / "data"
    saved = SavedSessionFiles(
        context_json=data_view / "context.json",
        measurement_pairs_parquet=data_view / MEASUREMENT_PAIRS_FILENAME,
        camera_timeline_parquet=data_view / CAMERA_TIMELINE_FILENAME,
        pixhawk_timeline_parquet=data_view / PX4_TIMELINE_FILENAME,
        gnss_base_timeline_parquet=data_view / GNSS_BASE_TIMELINE_FILENAME,
        geometry_comparison_parquet=data_view / GEOMETRY_COMPARISON_FILENAME,
        review_xlsx=review_path,
        manifest=manifest_output,
        authoritative_jsonl_files=tuple(
            output_dir / relative_path
            for _key, _role, relative_path, _rows, _normalization in jsonl_streams
        ),
        finalized=True,
        review_error=review_error,
        review_pending=review_pending,
    )
    for source_rows in (
        pair_rows,
        *camera_sources.values(),
        *px4_sources.values(),
        gnss_base_rows,
        extrinsics_comparison_rows,
        method_evaluation_bundle_rows,
    ):
        if isinstance(source_rows, SpoolRows):
            source_rows.discard_spool()
    recovery_metadata = output_dir / ".spool" / "session_metadata.json"
    try:
        recovery_metadata.unlink(missing_ok=True)
    except OSError:
        pass
    try:
        recovery_metadata.parent.rmdir()
    except OSError:
        pass
    _cleanup_v2_generations(output_dir)
    return saved
