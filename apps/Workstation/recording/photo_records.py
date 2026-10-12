"""Build append-only HR/OZ capture events and import exact verified image bytes.

Capture purpose, signed right-minus-left request timing, and source hashes are
retained when registering the session-owned artifact."""
from __future__ import annotations
import hashlib
import json
import re
import time
import uuid
from pathlib import Path
from Workstation.workstation_core.hr_evidence import CapturePurpose, create_hr_capture_event, import_hr_image_file
from Workstation.workstation_core.strict_jsonl import append_jsonl_record
from .state import (PhotoCaptureState, RecordingSessionState)
from .ports import PhotoRecordsPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .photo_capture import PhotoCapture


class PhotoRecords:
    def __init__(self, *,
        capture_state: PhotoCaptureState,
        session: RecordingSessionState,
        boundary: PhotoRecordsPort,
    ) -> None:
        self.capture_state = capture_state
        self.session = session
        self.boundary = boundary

    def bind_collaborators(self, *,
        photo_capture: "PhotoCapture",
    ) -> None:
        self.photo_capture = photo_capture

    def _hr_capture_event_record(
        self,
        record: dict[str, object],
        *,
        capture_id: str,
        request: dict[str, object] | None = None,
        note: str = "camera-internal HR capture",
    ) -> dict[str, object]:
        """Return one hash-bound, append-only HR event record."""

        context = dict(request or {})
        purpose = self.photo_capture._hr_capture_purpose(
            context.get("purpose", record.get("purpose", CapturePurpose.UNSPECIFIED))
        )
        request_utc_ns = int(context.get("request_utc_ns") or time.time_ns())
        event = create_hr_capture_event(
            capture_id,
            purpose=purpose,
            captured_utc_ns=request_utc_ns,
            note=note,
        ).to_mapping()
        event.update(context)
        event.update(record)
        event["capture_id"] = str(capture_id)
        event["purpose"] = purpose.value
        event["capture_time_semantics"] = (
            "Workstation request UTC; HR exposure UTC remains unavailable"
        )
        request_delta = self.photo_capture._camera_hr_request_delta(event)
        event["camera_request_delta_right_minus_left_ns"] = request_delta
        event["camera_request_delta_sign_contract"] = (
            "right capture_requested_unix_ns minus left capture_requested_unix_ns"
        )
        event["pixel_coordinate_contract"] = (
            "stored image raster u-right/v-down; OpenCV optical frame RDF "
            "(right/down/forward); no pixel or pose conversion performed"
        )
        return event

    def _oz_capture_event_record(
        self,
        record: dict[str, object],
        *,
        capture_id: str,
        request: dict[str, object] | None = None,
        note: str = "camera-internal OZ still capture",
    ) -> dict[str, object]:
        """Return one hash-bound, append-only OZ still event record."""

        context = dict(request or {})
        purpose = self.photo_capture._hr_capture_purpose(
            context.get("purpose", record.get("purpose", CapturePurpose.UNSPECIFIED))
        )
        request_utc_ns = int(context.get("request_utc_ns") or time.time_ns())
        event = {
            "schema": "workstation-oz-capture-event-v1",
            "event_id": str(uuid.uuid4()),
            "capture_id": str(capture_id),
            "purpose": purpose.value,
            "captured_utc_ns": request_utc_ns,
            "note": note,
        }
        event.update(context)
        event.update(record)
        event["capture_id"] = str(capture_id)
        event["capture_channel"] = "oz"
        event["purpose"] = purpose.value
        event["capture_time_semantics"] = (
            "Workstation request UTC; OZ exposure UTC remains unavailable"
        )
        request_delta = self.photo_capture._camera_hr_request_delta(event)
        event["camera_request_delta_right_minus_left_ns"] = request_delta
        event["camera_request_delta_sign_contract"] = (
            "right capture_requested_unix_ns minus left capture_requested_unix_ns"
        )
        event["pixel_coordinate_contract"] = (
            "camera-native visible/OZ still raster u-right/v-down; OpenCV optical "
            "frame RDF (right/down/forward); no pixel or pose conversion performed"
        )
        return event

    def _append_pnp_photo_capture_record(
        self, record: dict[str, object]
    ) -> dict[str, object]:
        # JSON round-tripping detaches nested mappings from signal payloads.
        # The append-only JSONL record and in-memory row therefore cannot be
        # changed by a worker mutating its original result after this call.
        normalized = json.loads(
            json.dumps(record, ensure_ascii=False, allow_nan=False, sort_keys=True)
        )
        normalized["session_id"] = self.session.recording_session_id
        normalized["measurement_id"] = str(getattr(self.boundary, "_measurement_id", "") or "")
        normalized["event_index"] = len(self.capture_state.pnp_photo_capture_records) + 1
        normalized.setdefault("recorded_utc_ns", time.time_ns())
        # Preserve the legacy spelling for downstream readers while making
        # the UTC basis explicit in new records.
        normalized.setdefault("recorded_unix_ns", normalized["recorded_utc_ns"])
        normalized.setdefault("schema", "workstation-hr-capture-event-v1")
        normalized.setdefault("capture_channel", "hr")
        normalized.setdefault("event_id", str(uuid.uuid4()))
        normalized.setdefault("purpose", CapturePurpose.UNSPECIFIED.value)
        normalized.setdefault("active_zoom", {
            side: dict(self.boundary.actual_visible_zoom.get(side, {}))
            for side in ("left", "right")
        })
        normalized.setdefault("requested_zoom_level", self.boundary.zoom_level_spin.value())
        normalized.setdefault("storage", "camera_internal")
        normalized.setdefault("ftp_downloaded", False)
        normalized["record_sha256"] = hashlib.sha256(
            json.dumps(
                normalized,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        if not bool(getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)):
            normalized.pop("session_id", None)
            writer = getattr(self.boundary, "append_workstation_data", None)
            if callable(writer):
                writer("capture.events", normalized, priority=True)
            self.capture_state.pnp_photo_capture_records.append(normalized)
            return json.loads(
                json.dumps(normalized, ensure_ascii=False, allow_nan=False, sort_keys=True)
            )
        output_dir = self.boundary._recording_session_dir(self.session.recording_session_id)
        output_dir.mkdir(parents=True, exist_ok=True)
        event_path = output_dir / "hr_captures.jsonl"
        append_jsonl_record(event_path, normalized)
        self.capture_state.pnp_photo_capture_records.append(normalized)
        run_event_writer = getattr(self.boundary, "append_workstation_run_event", None)
        if callable(run_event_writer):
            try:
                run_event_writer(
                    "hr_capture_event",
                    normalized,
                    source="RecordingController",
                    coordinate_contract=str(
                        normalized.get("pixel_coordinate_contract", "not_applicable")
                    ),
                    source_timestamp_ns=normalized.get("request_utc_ns"),
                )
            except Exception as exc:
                logger = getattr(self.boundary, "_log", None)
                if callable(logger):
                    logger(f"[Capture HR Still] run evidence linkage failed: {exc}")
        return json.loads(
            json.dumps(normalized, ensure_ascii=False, allow_nan=False, sort_keys=True)
        )

    def _append_oz_photo_capture_record(
        self, record: dict[str, object]
    ) -> dict[str, object]:
        """Append one camera-internal OZ still event to session evidence."""

        normalized = json.loads(
            json.dumps(record, ensure_ascii=False, allow_nan=False, sort_keys=True)
        )
        records = getattr(self.capture_state, "oz_photo_capture_records", None)
        if not isinstance(records, list):
            records = []
            self.capture_state.oz_photo_capture_records = records
        normalized["session_id"] = self.session.recording_session_id
        normalized["measurement_id"] = str(getattr(self.boundary, "_measurement_id", "") or "")
        normalized["event_index"] = len(records) + 1
        normalized.setdefault("recorded_utc_ns", time.time_ns())
        normalized.setdefault("recorded_unix_ns", normalized["recorded_utc_ns"])
        normalized.setdefault("schema", "workstation-oz-capture-event-v1")
        normalized.setdefault("capture_channel", "oz")
        normalized.setdefault("event_id", str(uuid.uuid4()))
        normalized.setdefault("purpose", CapturePurpose.UNSPECIFIED.value)
        normalized.setdefault(
            "active_zoom",
            {
                side: dict(self.boundary.actual_visible_zoom.get(side, {}))
                for side in ("left", "right")
            },
        )
        normalized.setdefault("requested_zoom_level", self.boundary.zoom_level_spin.value())
        normalized.setdefault("storage", "camera_internal")
        normalized.setdefault("ftp_downloaded", False)
        normalized.setdefault("image_enable_command", "SIVI TRUE")
        normalized["record_sha256"] = hashlib.sha256(
            json.dumps(
                normalized,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        if not bool(getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)):
            normalized.pop("session_id", None)
            writer = getattr(self.boundary, "append_workstation_data", None)
            if callable(writer):
                writer("capture.events", normalized, priority=True)
            records.append(normalized)
            return json.loads(
                json.dumps(normalized, ensure_ascii=False, sort_keys=True)
            )
        output_dir = self.boundary._recording_session_dir(self.session.recording_session_id)
        output_dir.mkdir(parents=True, exist_ok=True)
        event_path = output_dir / "oz_captures.jsonl"
        append_jsonl_record(event_path, normalized)
        records.append(normalized)
        run_event_writer = getattr(self.boundary, "append_workstation_run_event", None)
        if callable(run_event_writer):
            try:
                run_event_writer(
                    "oz_capture_event",
                    normalized,
                    source="RecordingController",
                    coordinate_contract=str(
                        normalized.get("pixel_coordinate_contract", "not_applicable")
                    ),
                    source_timestamp_ns=normalized.get("request_utc_ns"),
                )
            except Exception as exc:
                logger = getattr(self.boundary, "_log", None)
                if callable(logger):
                    logger(f"[Capture OZ Still] run evidence linkage failed: {exc}")
        return json.loads(
            json.dumps(normalized, ensure_ascii=False, sort_keys=True)
        )

    @staticmethod
    def _hr_capture_path_component(capture_id: str) -> str:
        raw = str(capture_id or "").strip()
        if not raw:
            raise ValueError("capture_id is required")
        safe = re.sub(r"[^A-Za-z0-9._-]+", "-", raw).strip(".-")
        if not safe:
            safe = "capture"
        if safe != raw:
            # Prevent different unsafe IDs from collapsing onto one directory.
            safe = f"{safe}-{uuid.uuid5(uuid.NAMESPACE_URL, raw).hex[:10]}"
        return safe[:128]

    def _known_hr_purpose(self, capture_id: str) -> CapturePurpose:
        for record in reversed(self.capture_state.pnp_photo_capture_records):
            if str(record.get("capture_id") or "") == str(capture_id):
                try:
                    return self.photo_capture._hr_capture_purpose(
                        record.get("purpose", CapturePurpose.UNSPECIFIED)
                    )
                except ValueError:
                    break
        return CapturePurpose.UNSPECIFIED

    def import_hr_files(
        self,
        capture_id: str,
        *,
        side: str | None = None,
        path: str | Path | None = None,
        left_path: str | Path | None = None,
        right_path: str | Path | None = None,
        camera_serial: str | None = None,
        left_camera_serial: str | None = None,
        right_camera_serial: str | None = None,
        camera_serials: dict[str, str] | None = None,
        purpose: CapturePurpose | str | None = None,
        captured_utc_ns: int | None = None,
        expected_sha256_by_side: dict[str, str] | None = None,
        expected_resolution_by_side: dict[str, tuple[int, int]] | None = None,
    ) -> dict[str, object]:
        """Validate and atomically retain exact HR files under the session.

        Callers may supply one ``side``/``path`` pair or ``left_path`` and/or
        ``right_path``.  File decoding, byte hashing, camera-serial validation,
        atomic writes, and event persistence all remain controller-owned; UI
        code should pass paths only.

        Pixel coordinates, if later annotated, are native raster ``u`` right,
        ``v`` down and map to an OpenCV optical RDF frame.  This import performs
        no geometry or frame conversion.
        """

        capture_text = str(capture_id or "").strip()
        capture_component = self._hr_capture_path_component(capture_text)
        single_supplied = side is not None or path is not None
        pair_supplied = left_path is not None or right_path is not None
        if single_supplied and pair_supplied:
            raise ValueError("use side/path or left_path/right_path, not both")
        sources: dict[str, Path] = {}
        if single_supplied:
            if side is None or path is None:
                raise ValueError("side and path must be supplied together")
            normalized_side = str(side).strip().lower()
            if normalized_side not in {"left", "right"}:
                raise ValueError("side must be left or right")
            sources[normalized_side] = Path(path)
        else:
            if left_path is not None:
                sources["left"] = Path(left_path)
            if right_path is not None:
                sources["right"] = Path(right_path)
        if not sources:
            raise ValueError("at least one HR file path is required")

        serials = {
            str(key).strip().lower(): str(value or "").strip()
            for key, value in dict(camera_serials or {}).items()
        }
        if left_camera_serial is not None:
            serials["left"] = str(left_camera_serial).strip()
        if right_camera_serial is not None:
            serials["right"] = str(right_camera_serial).strip()
        if camera_serial is not None:
            if len(sources) != 1:
                raise ValueError(
                    "camera_serial is only valid for one side; use per-side serials for a pair"
                )
            serials[next(iter(sources))] = str(camera_serial).strip()
        missing_serials = [side_name for side_name in sources if not serials.get(side_name)]
        if missing_serials:
            raise ValueError(
                "explicit camera serial required for HR import: " + ", ".join(missing_serials)
            )

        expected_hashes = dict(expected_sha256_by_side or {})
        expected_resolutions = dict(expected_resolution_by_side or {})
        imported_utc_ns = time.time_ns()
        imported = {}
        # Validate every source before materializing any destination.  This
        # prevents a bad second side from producing a misleading pair event.
        for side_name, source_path in sources.items():
            expected_resolution = expected_resolutions.get(side_name)
            if expected_resolution is not None and len(expected_resolution) != 2:
                raise ValueError("expected resolution must be a (width_px, height_px) pair")
            imported[side_name] = import_hr_image_file(
                source_path,
                side=side_name,
                camera_serial=serials[side_name],
                expected_sha256=expected_hashes.get(side_name),
                expected_width_px=(
                    None if expected_resolution is None else int(expected_resolution[0])
                ),
                expected_height_px=(
                    None if expected_resolution is None else int(expected_resolution[1])
                ),
                imported_utc_ns=imported_utc_ns,
            )

        self.photo_capture._prepare_pnp_evidence_session()
        session_root = Path(
            self.boundary._recording_session_dir(self.session.recording_session_id)
        ).resolve()
        sessionless = not bool(
            getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)
        )
        destination_root = (
            session_root / "images" / "hr_imported"
            if sessionless
            else session_root / "hr" / "imported" / capture_component
        )
        stored_files: dict[str, dict[str, object]] = {}
        for side_name, image in imported.items():
            suffix = Path(image.source_name).suffix.lower()
            if not re.fullmatch(r"\.[a-z0-9]{1,10}", suffix):
                suffix = ".bin"
            if sessionless:
                source_stem = re.sub(
                    r"[^A-Za-z0-9._-]+",
                    "_",
                    Path(image.source_name).stem,
                ).strip("._") or "image"
                stem = (
                    f"{self.photo_capture._measurement_artifact_stamp(imported_utc_ns)}_"
                    f"{side_name}_{source_stem}"
                )
                destination = destination_root / f"{stem}{suffix}"
                sequence = 1
                while destination.exists() or destination.is_symlink():
                    destination = destination_root / f"{stem}-{sequence:02d}{suffix}"
                    sequence += 1
            else:
                destination = destination_root / f"{side_name}-{image.sha256}{suffix}"
            stored = image.write_exact(destination)
            resolved = stored.resolve(strict=True)
            try:
                relative_path = resolved.relative_to(session_root)
            except ValueError as exc:  # defensive: destination must be session-owned
                raise RuntimeError("HR destination escaped the session directory") from exc
            stored_files[side_name] = {
                **image.to_mapping(),
                "session_relative_path": relative_path.as_posix(),
            }

        normalized_purpose = (
            self._known_hr_purpose(capture_text)
            if purpose is None
            else self.photo_capture._hr_capture_purpose(purpose)
        )
        known_request = next(
            (
                dict(record)
                for record in reversed(self.capture_state.pnp_photo_capture_records)
                if str(record.get("capture_id") or "") == capture_text
            ),
            {},
        )
        source_capture_utc_ns = (
            int(captured_utc_ns)
            if captured_utc_ns is not None
            else int(known_request.get("request_utc_ns") or imported_utc_ns)
        )
        time_semantics = (
            "caller_supplied_source_capture_utc"
            if captured_utc_ns is not None
            else (
                "Workstation_capture_request_utc"
                if known_request.get("request_utc_ns") is not None
                else "import_utc_fallback_not_source_capture_truth"
            )
        )
        capture_event = create_hr_capture_event(
            capture_text,
            purpose=normalized_purpose,
            left=imported.get("left"),
            right=imported.get("right"),
            captured_utc_ns=source_capture_utc_ns,
            note=(
                "exact HR file import into Workstation data root"
                if sessionless
                else "exact HR file import into Workstation session"
            ),
        ).to_mapping()
        capture_event.update(
            {
                "event_type": "hr_file_imported",
                "state": "imported",
                "storage": (
                    "type_owned_exact_file"
                    if sessionless
                    else "session_owned_exact_file"
                ),
                "ftp_downloaded": False,
                "stored_files": stored_files,
                "capture_time_semantics": time_semantics,
                "pixel_coordinate_contract": (
                    "native image raster u-right/v-down; OpenCV optical RDF "
                    "(right/down/forward); exact bytes retained; no conversion"
                ),
            }
        )
        for key in (
            "request_utc_ns",
            "request_monotonic_ns",
            "request_clock_anchor_uncertainty_ns",
            "request_utc_method",
            "request_session_delta_ns",
            "request_session_delta_s",
            "request_time_semantics",
        ):
            if key in known_request:
                capture_event[key] = known_request[key]
        return self._append_pnp_photo_capture_record(capture_event)
