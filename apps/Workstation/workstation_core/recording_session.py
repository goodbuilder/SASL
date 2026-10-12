"""Named compatibility exports for the session-storage owners.

Canonical JSONL, generation publication, spool lifetime and derived views
have distinct implementations under session_storage. No duplicate row store,
lock, transaction state or writer is kept here.
"""
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
from typing import Callable, Iterable
import pyarrow as pa
import pyarrow.parquet as pq
import xlsxwriter
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS, PIXHAWK_RTK_LIVE
from Workstation.workstation_core.measurement_data_store import ROW_WRITE_ADMITTED, ROW_WRITE_UNAVAILABLE, RowWriteAdmission
from Workstation.workstation_core.runtime_camera_status import is_sr_pc_diagnostic_reason
from Workstation.workstation_core.strict_jsonl import fsync_directory as _fsync_directory_path, sha256_file as _sha256_path

from Workstation.workstation_core.session_storage.schema import (
    WRITE_PARQUET_ARTIFACTS,
    PAIR_AXIS_COLUMNS,
    DISTANCE_MEASUREMENT_COLUMNS,
    CAMERA_TIMELINE_COMPACT_COLUMNS,
    PX4_TIMELINE_COMPACT_COLUMNS,
    PX4_MAVLINK_MAG_WIDE_COLUMNS,
    _PX4_MAVLINK_MAG_STREAMS,
    _PX4_MAVLINK_MAG_MATCH_TOLERANCE_NS,
    PX4_GPS_WIDE_FIELDS,
    DEVICE_TIMELINE_COLUMNS,
    GNSS_BASE_COMPACT_COLUMNS,
    EXTRINSICS_COMPARISON_COMPACT_COLUMNS,
    DYNAMIC_SHEET_RULES,
    CALIBRATION_KEYWORDS,
    AUTHORITATIVE_JSONL_SCHEMA_VERSION,
    MEASUREMENT_STORAGE_SCHEMA,
    MEASUREMENT_STORAGE_SCHEMA_VERSION,
    AUTHORITATIVE_JSONL_STREAMS,
    _INVALID_XLSX_SHEET_CHARS,
    PX4_XLSX_TIME_COLUMNS,
    PX4_XLSX_TAB_COLUMNS,
    RTK_XLSX_COLUMNS,
)
from Workstation.workstation_core.session_storage.paths import (
    session_id_now,
    session_dir,
    legacy_session_dir,
    resolve_session_dir,
    raw_video_paths,
    format_video_paths,
)
from Workstation.workstation_core.session_storage.models import (
    SavedSessionFiles,
    RetryableSession,
    SessionSheetPreview,
)
from Workstation.workstation_core.session_storage.spool import (
    SpoolRows,
)
from Workstation.workstation_core.session_storage.projection import (
    _ordered_fieldnames,
    _normalized_pair_rows,
    _normalized_generic_rows,
    _materialize_method_evaluation_rows,
    _iter_normalized_generic_rows,
    _copy_first_present,
    _project_pair_typed_fields,
    _project_camera_typed_fields,
    _project_px4_typed_fields,
    _iter_project_camera_typed_fields,
    _iter_project_px4_typed_fields,
    _recording_time_ns,
    _estimated_utc_time_ns,
    _distance_device_utc_time_ns,
    _timeline_device_utc_time_ns,
    _paired_class_name,
    _warning_code,
    _compact_distance_measurement_row,
    compact_distance_measurement_rows,
    _compact_camera_timeline_row,
    compact_camera_timeline_rows,
    _compact_px4_timeline_row,
    _gps_utc_time_ns,
    _gps_utc_source,
    _gps_wide_prefix,
    compact_px4_timeline_rows,
    _compact_gnss_base_row,
    compact_gnss_base_rows,
    _mavlink_mag_stream_name,
    _wide_mavlink_mag_rows,
)
from Workstation.workstation_core.session_storage.values import (
    _serializable_value,
    _seconds_to_ns,
    _none_if_empty,
    _optional_int,
    _optional_float,
    _optional_bool,
    _ns_from_seconds,
    _first_present,
)
from Workstation.workstation_core.session_storage.canonical import (
    _authoritative_jsonl_row_bytes,
    _apply_measurement_package_identity,
    _apply_terminal_record_identities,
    _authoritative_jsonl_schema,
    _write_authoritative_jsonl,
    _validate_authoritative_jsonl,
    _scope_unpublished_artifact_details,
)
from Workstation.workstation_core.session_storage.writers import (
    _projected_columns,
    _sanitize_xlsx_sheet_name,
    _extrinsics_comparison_sheet_groups,
    session_sheet_previews,
    _flatten_metadata,
    _write_table,
    _write_px4_workbook,
    _compact_rtk_rows,
    _write_rtk_workbook,
    _write_key_value_sheet,
    write_dynamic_csv,
    write_parquet,
    _write_session_static_json,
    _write_xlsx,
)
from Workstation.workstation_core.session_storage.manifest import (
    _write_manifest,
)
from Workstation.workstation_core.session_storage.recovery import (
    write_recovery_metadata,
    discover_retryable_sessions,
    load_retryable_session,
)
from Workstation.workstation_core.session_storage.generation import (
    _cleanup_finalize_orphans,
    _ensure_stable_artifact_links,
    _publish_generation,
    _v2_publish_root,
    _ensure_v2_data_link,
    _publish_v2_generation,
    _v2_current_target,
    _restore_v2_current_target,
    _strict_json_object,
    _validate_published_v2_generation,
    _cleanup_v2_generations,
    _owning_session_root,
    _validate_measurement_storage_identity,
)
from Workstation.workstation_core.session_storage.review import (
    _LEGACY_REVIEW_METHOD_SCHEMAS,
    _LEGACY_REVIEW_METHOD_ALIASES,
    _review_row_is_explicitly_blocked,
    _review_measurement_method_id,
    _review_result_row,
    _review_quality_row,
    _review_warning_rows,
    _review_capture_rows,
    _measurement_review_mapping,
    _rebuild_session_review,
    rebuild_session_review,
)
from Workstation.workstation_core.session_storage.legacy import (
    _save_session_files_legacy,
)
from Workstation.workstation_core.session_storage.finalize import (
    save_session_files,
)
