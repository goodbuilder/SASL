"""Optical capture/artifact I/O, independent from geometry and application state.

Frame evidence remains native u-right/v-down pixels. Imported timestamps retain
only declared clock provenance; importing never infers an exposure or live zoom.
"""

from __future__ import annotations
import copy
import hashlib
import json
import math
import os
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping
from PyQt5 import QtGui
from Workstation.workstation_core.native_frame_store import NativeFrameStore
from Workstation.workstation_core.run_evidence import canonical_json_bytes
from Workstation.workstation_core.terrain import inspect_geotiff
from Workstation.controllers.run_evidence_controller import persistence_failure_error


@dataclass(frozen=True)
class AlignmentPublication:
    target: Path
    version_path: Path
    sha256: str


def _load_json_mapping(path_value: object, *, label: str) -> tuple[dict[str, Any], str]:
    source = Path(str(path_value or "")).expanduser().resolve(strict=True)
    if not source.is_file():
        raise ValueError(f"{label} must be a regular JSON file")
    raw = source.read_bytes()
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{label} is not valid UTF-8 JSON: {exc}") from exc
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} root must be a JSON object")
    return dict(value), hashlib.sha256(raw).hexdigest()


_SUGGESTION_MODE_METHOD_IDS = {
    "mountain": "mountain_gnss",
    "cloud": "cloud_gnss",
    "horizon": "horizon_hybrid",
}


def accepted_suggestion_alignment_row(accepted: Mapping[str, Any]) -> dict[str, Any]:
    """Flat ``alignment`` stream row describing one accepted auto suggestion."""

    mode = str(accepted.get("mode") or "").lower()
    rotation = accepted.get("relative_rotation")
    rotation = rotation if isinstance(rotation, Mapping) else {}
    frames = accepted.get("source_frames")
    frames = frames if isinstance(frames, Mapping) else {}
    hashes = accepted.get("input_hashes")
    hashes = hashes if isinstance(hashes, Mapping) else {}
    baseline = accepted.get("gnss_baseline")
    baseline = baseline if isinstance(baseline, Mapping) else {}

    def _frame_sha(side: str) -> str:
        value = frames.get(side)
        return str(value.get("image_sha256") or "") if isinstance(value, Mapping) else ""

    return {
        "schema": "workstation-alignment-event-v1",
        "event_type": "optical_pose_auto_suggestion_accepted",
        "updated_utc_ns": int(accepted.get("accepted_utc_ns") or time.time_ns()),
        "method_id": _SUGGESTION_MODE_METHOD_IDS.get(mode, ""),
        "mode": mode,
        "source": str(accepted.get("suggestion_id") or ""),
        "source_sha256": str(hashes.get("gnss_baseline_sha256") or ""),
        "state": accepted.get("state"),
        "detection_method": accepted.get("detection_method"),
        "inlier_count": accepted.get("inlier_count"),
        "match_count": accepted.get("match_count"),
        "residual_px": accepted.get("residual_px"),
        "confidence": accepted.get("confidence"),
        "eligible": accepted.get("eligible"),
        "relative_rotation_available": bool(rotation.get("available")),
        "relative_yaw_right_minus_left_deg": rotation.get("yaw_right_minus_left_deg"),
        "relative_pitch_right_minus_left_deg": rotation.get("pitch_right_minus_left_deg"),
        "relative_roll_right_minus_left_deg": rotation.get("roll_right_minus_left_deg"),
        "relative_fit_inlier_count": rotation.get("fit_inlier_count"),
        "relative_fit_residual_px": rotation.get("fit_residual_px"),
        "far_field_du_median_px": rotation.get("far_field_du_median_px"),
        "far_field_dv_median_px": rotation.get("far_field_dv_median_px"),
        "left_image_sha256": _frame_sha("left"),
        "right_image_sha256": _frame_sha("right"),
        "gnss_baseline_m": baseline.get("baseline_m"),
        "gnss_baseline_fix_quality": baseline.get("fix_quality"),
    }


class OpticalArtifacts:
    def __init__(self):
        self.terrain_metadata = None
        self.terrain_snapshot = None

    def reset(self):
        self.terrain_metadata = None
        self.terrain_snapshot = None

    @staticmethod
    def native_frames(root, burst_id, frames, zoom_by_side, *, store_factory=NativeFrameStore):
        """Persist detached producer snapshots; source dimensions stay authoritative."""
        store = store_factory(root)
        frozen: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            result = frames.get(side)
            if not isinstance(result, tuple) or len(result) != 3:
                raise RuntimeError(f"{side} native frame is not available yet")
            timestamp_s, image, raw_metadata = result
            if not isinstance(image, QtGui.QImage) or image.isNull():
                raise RuntimeError(f"{side} native frame is null")
            metadata = copy.deepcopy(dict(raw_metadata)) if isinstance(raw_metadata, Mapping) else {}
            metadata.setdefault("timestamp_s", float(timestamp_s))
            # native_width/native_height must come from the frame producer
            # (e.g. the shared native-frame-ring descriptor). Backfilling them
            # from `image` here would make NativeFrameStore.persist()'s
            # dimension-mismatch check tautological -- it would compare the
            # image against itself and could never fail.
            if metadata.get("native_width") is None or metadata.get("native_height") is None:
                raise RuntimeError(
                    f"{side} native frame source did not report native_width/"
                    "native_height; refusing to backfill from the captured image"
                )
            metadata.setdefault("overlay_applied", False)
            zoom = copy.deepcopy(zoom_by_side.get(side, {}))
            metadata["capture_zoom_index"] = zoom.get("index")
            metadata["capture_zoom_ratio"] = zoom.get("ratio")
            metadata["capture_zoom_updated_monotonic_s"] = zoom.get("updated_monotonic_s")
            artifact = store.persist(
                side=side,
                purpose="optical-pose-burst",
                frame_id=burst_id,
                image=image.copy(),
                metadata=metadata,
            )
            frozen[side] = {
                "image": image.copy(),
                "metadata": metadata,
                "artifact": artifact,
                "zoom": dict(zoom),
            }

        return frozen

    @staticmethod
    def imported_frames(
        root, burst_id, sources, stable_metadata, captured_monotonic_s, *, store_factory=NativeFrameStore
    ):
        store = store_factory(root)
        frozen: dict[str, dict[str, Any]] = {}
        source_records: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            row = sources[side]
            source_path = str(row.get("path") or "").strip()
            if not source_path:
                raise ValueError(f"{side}: sources.{side}.path is required")
            resolved_source = Path(source_path).expanduser()
            if not resolved_source.is_file():
                raise ValueError(f"{side}: image file not found: {resolved_source}")
            image = QtGui.QImage(str(resolved_source))
            if image.isNull():
                raise ValueError(f"{side}: failed to decode image: {resolved_source}")
            source_sha256 = hashlib.sha256(resolved_source.read_bytes()).hexdigest()
            # Inherit only stable rig identity.  Copying the complete live
            # RTSP row would incorrectly attach that unrelated preview's
            # timestamp/source clock to an FTP or offline still.
            live_metadata = stable_metadata
            live_row = live_metadata.get(side, {}) if isinstance(live_metadata, Mapping) else {}
            metadata: dict[str, Any] = {}
            if isinstance(live_row, Mapping):
                for key in (
                    "camera_serial",
                    "serial",
                    "device_serial",
                    "camera_model",
                    "stabilization_state",
                    "stabilization",
                ):
                    if live_row.get(key) not in (None, ""):
                        metadata[key] = live_row[key]
            overrides = row.get("metadata")
            if isinstance(overrides, Mapping):
                metadata.update(copy.deepcopy(dict(overrides)))

            raw_timestamp = row.get("timestamp_s", metadata.get("timestamp_s"))
            raw_reference = row.get(
                "timestamp_reference",
                metadata.get("timestamp_reference"),
            )
            try:
                capture_timestamp_s = None if raw_timestamp is None else float(raw_timestamp)
            except (TypeError, ValueError, OverflowError):
                capture_timestamp_s = None
            reference = str(raw_reference or "").strip().lower()
            if capture_timestamp_s is None or not math.isfinite(capture_timestamp_s):
                # NativeFrameArtifact currently requires a numeric timestamp.
                # Store the truthful import-receive instant in its own clock
                # domain; never label it as the exposure/app-receive time.
                metadata["timestamp_s"] = captured_monotonic_s
                metadata["timestamp_reference"] = "import-receive"
                metadata["capture_timestamp_known"] = False
                metadata["import_received_monotonic_s"] = captured_monotonic_s
                if raw_timestamp is not None:
                    metadata["capture_timestamp_error"] = "non-finite capture timestamp"
            else:
                metadata["timestamp_s"] = capture_timestamp_s
                metadata["timestamp_reference"] = reference or "unknown"
                metadata["capture_timestamp_known"] = bool(reference) and reference not in {
                    "unknown",
                    "import-receive",
                }
                metadata["import_received_monotonic_s"] = captured_monotonic_s
            metadata["native_width"] = image.width()
            metadata["native_height"] = image.height()
            metadata["overlay_applied"] = False
            metadata["imported_source_path"] = str(resolved_source)
            metadata["imported_source_sha256"] = source_sha256

            # Zoom is capture evidence.  Only an explicit source value (for
            # FTP, the request-time GZVV snapshot) may populate it.
            raw_zoom_index = row.get("zoom_index", metadata.get("capture_zoom_index"))
            raw_zoom_ratio = row.get("zoom_ratio", metadata.get("capture_zoom_ratio"))
            raw_zoom_updated = row.get(
                "zoom_updated_monotonic_s",
                metadata.get("capture_zoom_updated_monotonic_s"),
            )
            try:
                zoom_index = None if raw_zoom_index is None else int(raw_zoom_index)
            except (TypeError, ValueError, OverflowError):
                zoom_index = None
            try:
                zoom_ratio = None if raw_zoom_ratio is None else float(raw_zoom_ratio)
            except (TypeError, ValueError, OverflowError):
                zoom_ratio = None
            if zoom_ratio is not None and not math.isfinite(zoom_ratio):
                zoom_ratio = None
            try:
                zoom_updated = None if raw_zoom_updated is None else float(raw_zoom_updated)
            except (TypeError, ValueError, OverflowError):
                zoom_updated = None
            if zoom_updated is not None and not math.isfinite(zoom_updated):
                zoom_updated = None
            metadata["capture_zoom_index"] = zoom_index
            metadata["capture_zoom_ratio"] = zoom_ratio
            metadata["capture_zoom_updated_monotonic_s"] = zoom_updated
            metadata["capture_zoom_known"] = zoom_index is not None
            artifact = store.persist(
                side=side,
                purpose="optical-pose-burst",
                frame_id=burst_id,
                image=image.copy(),
                metadata=metadata,
            )
            frozen[side] = {
                "image": image.copy(),
                "metadata": metadata,
                "artifact": artifact,
                "zoom": {
                    "index": metadata["capture_zoom_index"],
                    "ratio": metadata["capture_zoom_ratio"],
                    "updated_monotonic_s": metadata["capture_zoom_updated_monotonic_s"],
                },
            }
            source_records[side] = {
                "path": str(resolved_source),
                "sha256": source_sha256,
                "capture_timestamp_known": metadata["capture_timestamp_known"],
                "timestamp_reference": metadata["timestamp_reference"],
                "capture_zoom_known": metadata["capture_zoom_known"],
            }

        return frozen, source_records

    def import_terrain(self, payload, *, importer=None, inspect=inspect_geotiff):
        source_text = str(payload.get("path") or payload.get("source_path") or "").strip()
        if not source_text:
            raise ValueError("GeoTIFF path is required")
        vertical_datum = str(payload.get("vertical_datum") or "unknown")
        metadata = inspect(Path(source_text), vertical_datum=vertical_datum)
        artifact: Mapping[str, Any] | None = None
        if callable(importer):
            artifact = importer(
                source_text,
                role="terrain/geotiff",
                media_type="image/tiff; application=geotiff",
                metadata={
                    "sha256": metadata.sha256,
                    "horizontal_datum": metadata.horizontal_datum,
                    "vertical_datum": metadata.vertical_datum,
                    "crs_wkt": metadata.crs_wkt,
                },
            )
            artifact_error = persistence_failure_error(artifact)
            if artifact_error is not None:
                raise OSError(f"terrain artifact import failed: {artifact_error}")
        self.terrain_metadata = metadata
        self.terrain_snapshot = {
            **{key: value for key, value in metadata.to_mapping().items() if key != "path"},
            "source_name": Path(source_text).name,
            "artifact_relative_path": "" if artifact is None else str(artifact.get("relative_path") or ""),
        }
        return copy.deepcopy(self.terrain_snapshot)

    @staticmethod
    def persist_alignment(*, hierarchy_enabled, session_id, alignment_dir, snapshot, writer=None):
        if not hierarchy_enabled:
            target = alignment_dir.parent / "alignment.jsonl"
            payload = {
                "schema": "workstation-alignment-event-v1",
                "event_type": "active_alignment_updated",
                "updated_utc_ns": time.time_ns(),
                "alignment": snapshot,
            }
            encoded = canonical_json_bytes(payload)
            digest = hashlib.sha256(encoded).hexdigest()
            if callable(writer):
                writer("alignment", payload, priority=True)
            return AlignmentPublication(target, target, digest)
        alignment_dir.mkdir(parents=True, exist_ok=True)
        target = alignment_dir / "active_alignment.json"
        version_payload = {
            "schema": "workstation-session-alignment-snapshot-v1",
            "session_id": session_id,
            "updated_utc_ns": time.time_ns(),
            "alignment": snapshot,
        }
        version_bytes = canonical_json_bytes(version_payload) + b"\n"
        version_sha256 = hashlib.sha256(version_bytes).hexdigest()
        selections_dir = alignment_dir / "selections"
        selections_dir.mkdir(parents=True, exist_ok=True)
        version_path = selections_dir / f"alignment-{version_sha256}.json"
        if version_path.exists():
            if version_path.read_bytes() != version_bytes:
                raise RuntimeError(f"immutable Alignment hash collision at {version_path}")
        else:
            version_temporary = version_path.with_name(f".{version_path.name}.{uuid.uuid4().hex}.tmp")
            try:
                with version_temporary.open("xb") as handle:
                    handle.write(version_bytes)
                    handle.flush()
                    os.fsync(handle.fileno())
                try:
                    os.link(version_temporary, version_path)
                except FileExistsError:
                    if version_path.read_bytes() != version_bytes:
                        raise RuntimeError(f"immutable Alignment destination changed at {version_path}")
            finally:
                version_temporary.unlink(missing_ok=True)
        payload = {
            **version_payload,
            "selection_relative_path": version_path.relative_to(alignment_dir).as_posix(),
            "selection_sha256": version_sha256,
        }
        temporary = target.with_name(f".{target.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("xb") as handle:
                handle.write(canonical_json_bytes(payload) + b"\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, target)
            descriptor = os.open(alignment_dir, os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        finally:
            temporary.unlink(missing_ok=True)
        return AlignmentPublication(target, version_path, version_sha256)
