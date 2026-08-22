"""Explicit Workstation measurement-package lifecycle orchestration.

The recording controller owns producers and their asynchronous save workers.
This controller owns the higher-level operator contract:

``Start Session -> New Measurement -> one Detection interval -> Post Capture
-> Seal Measurement``.  A sealed measurement leaves the parent session open
for another child; ``Close Session`` seals the aggregate child-manifest index.

Finalization is deliberately resumable.  A click may move the package to
``VERIFYING`` while camera/capture/save workers drain; their completion
callbacks call :meth:`_continue_measurement_package_finalization` and sealing
only happens after every producer boundary is quiet.
"""

from __future__ import annotations

import hashlib
import inspect
import json
import mimetypes
import re
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from PyQt5 import QtCore, QtGui

from Workstation.runtime_paths import resolve_workstation_data_path
from Workstation.workstation_core.recording_session import session_id_now


MEASUREMENT_PACKAGE_METHODS = [
    "begin_measurement_session",
    "measurement_session_snapshot",
    "measurement_package_snapshot",
    "activate_measurement_package_source",
    "begin_measurement_detection",
    "cancel_measurement_detection_start",
    "complete_measurement_detection",
    "finalize_measurement_package",
    "_continue_measurement_package_finalization",
    "_handle_measurement_rtcm_evidence_drained",
    "begin_measurement",
    "begin_next_measurement",
    "close_measurement_session",
    "begin_measurement_revision",
    "verify_measurement_package",
    "verify_session",
    # Public lifecycle aliases used by non-Qt integrations.
    "begin_session",
    "end_measurement",
    "seal_session",
    "close_session",
    "create_revision",
    "verify_package",
]


_ACTIVE_STATES = {"COLLECTING", "POST_CAPTURE", "VERIFYING"}
_SEALED_STATES = {"SEALED_COMPLETE", "SEALED_INCOMPLETE"}
_PACKAGE_METADATA_FILES = {
    "package_working_state.json",
    "package_manifest.json",
    "package_manifest.sha256",
    "latest_revision.json",
    "measurement.json",
}


def _json_copy(value: object) -> object:
    """Detach snapshots from mutable producer/core mappings."""

    try:
        return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))
    except (TypeError, ValueError):
        return value


def _state_text(value: object, default: str = "IDLE") -> str:
    raw = getattr(value, "value", value)
    text = str(raw or default).strip().upper()
    aliases = {
        "COMPLETE": "SEALED_COMPLETE",
        "INCOMPLETE": "SEALED_INCOMPLETE",
        "SEALED": "SEALED_COMPLETE",
        "FAILED": "RECOVERABLE",
    }
    return aliases.get(text, text)


def _revision_number(value: object, default: int = 0) -> int:
    if value in (None, ""):
        return int(default)
    text = str(value).strip().lower()
    if text.startswith("r") and text[1:].isdigit():
        return int(text[1:])
    try:
        return int(text)
    except (TypeError, ValueError):
        return int(default)


def _call_supported(callable_obj, *args: object, **kwargs: object) -> object:
    """Call a core API while tolerating additive keyword-version skew."""

    try:
        signature = inspect.signature(callable_obj)
    except (TypeError, ValueError):
        return callable_obj(*args, **kwargs)
    if any(
        parameter.kind is inspect.Parameter.VAR_KEYWORD
        for parameter in signature.parameters.values()
    ):
        return callable_obj(*args, **kwargs)
    supported = {key: value for key, value in kwargs.items() if key in signature.parameters}
    return callable_obj(*args, **supported)


class MeasurementPackageController:
    """Coordinate the explicit package lifecycle around existing producers."""

    def __init__(self, panel: object) -> None:
        self.panel = panel
        defaults = {
            "_measurement_package": None,
            "_measurement_package_state": "IDLE",
            "_measurement_package_detail": "Begin Session to collect evidence",
            "_measurement_package_error": "",
            "_measurement_package_detection_started": False,
            "_measurement_package_detection_completed": False,
            "_measurement_package_finalize_requested": False,
            "_measurement_package_finalize_retry_scheduled": False,
            "_measurement_session_close_requested": False,
            "_measurement_package_registered_paths": set(),
            "_measurement_package_inventory_errors": [],
            "_measurement_package_revision": 0,
            "_measurement_package_calibration_collected": False,
            "_measurement_package_video_observations": None,
            "_measurement_package_rtcm_drain_requested_token": "",
            "_measurement_package_rtcm_drain_result": None,
            "_measurement_session": None,
            "_measurement_session_id": "",
            "_measurement_session_state": "IDLE",
            "_measurement_id": "",
            "_measurement_app_run_id": "",
            "_measurement_started_monotonic_s": None,
            "_measurement_detection_started_monotonic_s": None,
        }
        for name, value in defaults.items():
            if not hasattr(panel, name):
                setattr(panel, name, value)

    # ------------------------------------------------------------------
    # State and presentation

    def _hierarchy_enabled(self) -> bool:
        return bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", False))

    def _session_container_root(self, session_id: str | None = None) -> Path:
        selected = str(
            session_id
            or getattr(self.panel, "_measurement_session_id", "")
            or getattr(self.panel, "recording_session_id", "")
            or ""
        )
        if not selected:
            raise RuntimeError("measurement session ID is unavailable")
        resolver = getattr(self.panel, "_measurement_session_dir", None)
        if callable(resolver):
            return Path(resolver(selected)).resolve()
        recording_resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(recording_resolver):
            raise RuntimeError("measurement session directory owner is unavailable")
        candidate = Path(recording_resolver(selected)).resolve()
        measurement_id = str(getattr(self.panel, "_measurement_id", "") or "")
        if measurement_id and candidate.parts[-2:] == ("measurements", measurement_id):
            return candidate.parent.parent
        return candidate

    def _discover_open_session_id(self) -> str:
        resolver = getattr(self.panel, "_measurement_sessions_root", None)
        if callable(resolver):
            root = Path(resolver()).resolve()
        else:
            app_root = getattr(self.panel, "app_root", None)
            if app_root is None:
                return ""
            root = Path(app_root).resolve() / "recordings"
        if not root.is_dir():
            return ""
        candidates: list[tuple[int, str]] = []
        search_children = [
            child
            for child in root.iterdir()
            if child.name != "sessions"
        ]
        legacy_root = root / "sessions"
        if legacy_root.is_dir() and not legacy_root.is_symlink():
            search_children.extend(legacy_root.iterdir())
        owners: dict[str, Path] = {}
        collisions: set[str] = set()
        for child in search_children:
            if not child.is_dir() or child.is_symlink() or not (child / "session.json").is_file():
                continue
            previous = owners.setdefault(child.name, child)
            if previous.resolve() != child.resolve():
                collisions.add(child.name)
        if collisions:
            self.panel._measurement_session_state = "RECOVERABLE"
            self.panel._measurement_package_error = (
                "direct/legacy session ID collision requires explicit recovery: "
                + ", ".join(sorted(collisions))
            )
            return ""
        for child in search_children:
            state_path = child / "session.json"
            if not child.is_dir() or child.is_symlink() or not state_path.is_file():
                continue
            try:
                payload = json.loads(state_path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, ValueError):
                continue
            if not isinstance(payload, Mapping):
                continue
            if str(payload.get("state") or "") not in {"OPEN", "CLOSING"}:
                continue
            candidates.append(
                (int(payload.get("updated_utc_ns") or 0), str(payload.get("session_id") or child.name))
            )
        if len(candidates) > 1:
            self.panel._measurement_session_state = "RECOVERABLE"
            self.panel._measurement_package_error = (
                "multiple open measurement sessions require explicit recovery: "
                + ", ".join(value for _updated, value in sorted(candidates, reverse=True))
            )
            return ""
        return candidates[0][1] if candidates else ""

    @property
    def session(self) -> object | None:
        if not self._hierarchy_enabled():
            return None
        current = getattr(self.panel, "_measurement_session", None)
        if current is not None:
            return current
        session_id = str(
            getattr(self.panel, "_measurement_session_id", "")
            or getattr(self.panel, "recording_session_id", "")
            or self._discover_open_session_id()
            or ""
        )
        if not session_id:
            return None
        try:
            root = self._session_container_root(session_id)
        except RuntimeError:
            return None
        if not (root / "session.json").is_file():
            return None
        try:
            from Workstation.workstation_core.measurement_session import MeasurementSession

            current = MeasurementSession.open(root)
        except Exception as exc:
            self.panel._measurement_session_state = "RECOVERABLE"
            self.panel._measurement_package_error = f"session recovery failed: {exc}"
            return None
        snapshot = current.snapshot()
        app_run_id = str(getattr(self.panel, "_measurement_app_run_id", "") or "")
        known_runs = {
            str(row.get("app_run_id") or "")
            for row in snapshot.get("app_runs", ()) or ()
            if isinstance(row, Mapping)
        }
        new_app_run = bool(app_run_id and app_run_id not in known_runs)
        if new_app_run and current.state == "OPEN":
            current.add_app_run(app_run_id)
            snapshot = current.snapshot()
        self.panel._measurement_session = current
        self.panel._measurement_session_id = current.session_id
        self.panel.recording_session_id = current.session_id
        self.panel._measurement_session_state = str(snapshot.get("state") or "OPEN")
        selected = current.active_measurement_id
        if selected is None:
            rows = list(snapshot.get("measurements") or [])
            selected = str(rows[-1].get("measurement_id") or "") if rows else None
        if selected:
            self.panel._measurement_id = selected
            if new_app_run and current.active_measurement_id == selected:
                try:
                    package = current.measurement_package(selected)
                    if package.state in {"COLLECTING", "POST_CAPTURE"}:
                        package.transition("RECOVERABLE")
                        current.refresh_measurement(selected, package=package)
                except Exception as exc:
                    self.panel._measurement_package_error = (
                        f"measurement interruption recovery failed: {exc}"
                    )
        return current

    def _sync_measurement_session(self) -> None:
        session = self.session
        package = getattr(self.panel, "_measurement_package", None)
        measurement_id = str(getattr(self.panel, "_measurement_id", "") or "")
        refresher = getattr(session, "refresh_measurement", None)
        if session is None or package is None or not measurement_id or not callable(refresher):
            return
        refresher(measurement_id, package=package)
        snapshotter = getattr(session, "snapshot", None)
        snapshot = snapshotter() if callable(snapshotter) else {}
        self.panel._measurement_session_state = str(
            snapshot.get("state") if isinstance(snapshot, Mapping) else "OPEN"
        )

    @property
    def package(self) -> object | None:
        package = getattr(self.panel, "_measurement_package", None)
        if package is not None:
            return package
        if self._hierarchy_enabled():
            session = self.session
            if session is None:
                return None
            measurement_id = str(getattr(self.panel, "_measurement_id", "") or "")
            if not measurement_id:
                return None
            try:
                package = session.measurement_package(measurement_id)
                core = package.snapshot()
            except Exception as exc:
                self.panel._measurement_package_state = "RECOVERABLE"
                self.panel._measurement_package_error = f"package recovery failed: {exc}"
                return None
            self.panel._measurement_package = package
            self.panel._measurement_method_config_token = None
            self.panel._measurement_package_state = _state_text(
                core.get("state"), "RECOVERABLE"
            )
            self.panel._measurement_package_revision = _revision_number(
                core.get("revision_number", core.get("revision")), 0
            )
            lifecycle = dict(core.get("metadata") or {}).get("controller_lifecycle")
            if isinstance(lifecycle, Mapping):
                self.panel._measurement_package_detection_started = bool(
                    lifecycle.get("detection_started", False)
                )
                self.panel._measurement_package_detection_completed = bool(
                    lifecycle.get("detection_completed", False)
                )
                self.panel._measurement_package_finalize_requested = bool(
                    lifecycle.get("finalize_requested", False)
                )
            return package
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not session_id or not callable(resolver):
            return None
        root = Path(resolver(session_id))
        if not (
            (root / "package_working_state.json").is_file()
            or (root / "latest_revision.json").is_file()
        ):
            return None
        try:
            from Workstation.workstation_core.measurement_package import MeasurementPackage

            package = MeasurementPackage.open(root)
            core = package.snapshot()
            if (
                bool(getattr(self.panel, "_session_save_retry_required", False))
                and str(core.get("state") or "").upper() == "COLLECTING"
            ):
                package.transition("RECOVERABLE")
                core = package.snapshot()
        except Exception as exc:
            self.panel._measurement_package_state = "RECOVERABLE"
            self.panel._measurement_package_error = f"package recovery failed: {exc}"
            return None
        self.panel._measurement_package = package
        self.panel._measurement_method_config_token = None
        self.panel._measurement_package_state = _state_text(core.get("state"), "RECOVERABLE")
        self.panel._measurement_package_revision = _revision_number(
            core.get("revision_number", core.get("revision")), 0
        )
        lifecycle = dict(core.get("metadata") or {}).get("controller_lifecycle")
        if isinstance(lifecycle, Mapping):
            self.panel._measurement_package_detection_started = bool(
                lifecycle.get("detection_started", False)
            )
            self.panel._measurement_package_detection_completed = bool(
                lifecycle.get("detection_completed", False)
            )
            self.panel._measurement_package_finalize_requested = bool(
                lifecycle.get("finalize_requested", False)
            )
        return package

    def _persist_controller_lifecycle(self) -> None:
        package = getattr(self.panel, "_measurement_package", None)
        updater = getattr(package, "update_metadata", None)
        if not callable(updater):
            return
        try:
            updater(
                {
                    "controller_lifecycle": {
                        "state": _state_text(
                            getattr(self.panel, "_measurement_package_state", "IDLE")
                        ),
                        "detection_started": bool(
                            getattr(self.panel, "_measurement_package_detection_started", False)
                        ),
                        "detection_completed": bool(
                            getattr(self.panel, "_measurement_package_detection_completed", False)
                        ),
                        "finalize_requested": bool(
                            getattr(self.panel, "_measurement_package_finalize_requested", False)
                        ),
                        "updated_utc_ns": time.time_ns(),
                    }
                }
            )
        except RuntimeError:
            # The immutable manifest owns terminal sealed state; lifecycle
            # flags were already persisted before the seal began.
            return

    def _set_state(self, state: str, detail: str = "", error: str = "") -> None:
        self.panel._measurement_package_state = _state_text(state)
        if detail:
            self.panel._measurement_package_detail = str(detail)
        self.panel._measurement_package_error = str(error)
        self._persist_controller_lifecycle()
        self._publish_snapshot()

    def _publish_snapshot(self) -> None:
        snapshot = self.measurement_package_snapshot()
        signal = getattr(self.panel, "measurement_package_changed", None)
        emit = getattr(signal, "emit", None)
        if callable(emit):
            emit(snapshot)
        for refresh_name in (
            "refresh_measurement_package_status",
            "_refresh_measurement_package_status",
            "_refresh_operation_header",
        ):
            refresh = getattr(self.panel, refresh_name, None)
            if callable(refresh):
                try:
                    refresh()
                except (AttributeError, RuntimeError):
                    # A partially constructed/tearing-down Qt view must not
                    # corrupt the persistence lifecycle.
                    pass

    def _core_snapshot(self) -> dict[str, object]:
        package = self.package
        snapshotter = getattr(package, "snapshot", None)
        if not callable(snapshotter):
            return {}
        try:
            raw = snapshotter()
        except Exception as exc:
            return {"snapshot_error": str(exc)}
        if isinstance(raw, Mapping):
            return dict(_json_copy(dict(raw)))
        mapper = getattr(raw, "to_mapping", None)
        if callable(mapper):
            mapped = mapper()
            return dict(_json_copy(dict(mapped))) if isinstance(mapped, Mapping) else {}
        return {}

    def measurement_package_snapshot(self) -> dict[str, object]:
        core = self._core_snapshot()
        state = _state_text(getattr(self.panel, "_measurement_package_state", "IDLE"))
        session_id = str(
            getattr(self.panel, "_measurement_session_id", "")
            or getattr(self.panel, "recording_session_id", "")
            or ""
        )
        measurement_id = str(getattr(self.panel, "_measurement_id", "") or "")
        root = ""
        if session_id:
            resolver = getattr(self.panel, "_recording_session_dir", None)
            if callable(resolver):
                try:
                    root = str(Path(resolver(session_id)))
                except Exception:
                    root = ""
        revision_number = _revision_number(
            core.get("revision_number", core.get("revision")),
            getattr(self.panel, "_measurement_package_revision", 0),
        )
        revision = str(core.get("revision") or (f"r{revision_number:04d}" if revision_number else ""))
        result = dict(core)
        result.update(
            {
                "schema": "workstation-measurement-package-controller-v1",
                "state": state,
                "core_state": _state_text(core.get("state"), state) if core else state,
                "session_id": session_id or str(core.get("session_id") or ""),
                "session_dir": (
                    str(self._session_container_root(session_id))
                    if self._hierarchy_enabled() and session_id
                    else root or str(core.get("root") or core.get("session_dir") or "")
                ),
                "measurement_id": measurement_id,
                "measurement_dir": root or str(core.get("root") or ""),
                "session_state": str(
                    getattr(self.panel, "_measurement_session_state", "IDLE") or "IDLE"
                ).upper(),
                "revision": revision,
                "revision_number": revision_number,
                "detection_started": bool(
                    getattr(self.panel, "_measurement_package_detection_started", False)
                ),
                "detection_completed": bool(
                    getattr(self.panel, "_measurement_package_detection_completed", False)
                ),
                "finalize_requested": bool(
                    getattr(self.panel, "_measurement_package_finalize_requested", False)
                ),
                "detail": str(getattr(self.panel, "_measurement_package_detail", "") or ""),
                "error": str(getattr(self.panel, "_measurement_package_error", "") or ""),
                "inventory_errors": list(
                    getattr(self.panel, "_measurement_package_inventory_errors", ()) or ()
                ),
            }
        )
        return dict(_json_copy(result))

    def measurement_session_snapshot(self) -> dict[str, object]:
        session = self.session
        if session is None:
            state = str(
                getattr(self.panel, "_measurement_session_state", "IDLE") or "IDLE"
            ).upper()
            return {
                "schema": "workstation-measurement-session-controller-v1",
                "state": state,
                "session_id": "",
                "active_measurement_id": None,
                "measurements": [],
                "error": str(
                    getattr(self.panel, "_measurement_package_error", "") or ""
                ),
                "current_measurement": self.measurement_package_snapshot(),
            }
        snapshotter = getattr(session, "snapshot", None)
        raw = snapshotter() if callable(snapshotter) else {}
        result = dict(raw) if isinstance(raw, Mapping) else {}
        current_measurement = self.measurement_package_snapshot()
        session_state = str(result.get("state") or "OPEN").upper()
        measurement_state = _state_text(current_measurement.get("state"), "IDLE")
        result.update(
            {
                "schema": "workstation-measurement-session-controller-v1",
                "state": session_state,
                "session_id": str(result.get("session_id") or ""),
                "session_dir": str(getattr(session, "root", "") or ""),
                "measurement_count": len(result.get("measurements") or []),
                "selected_measurement_id": str(
                    getattr(self.panel, "_measurement_id", "") or ""
                ),
                "current_measurement": current_measurement,
                "can_begin_measurement": (
                    session_state == "OPEN"
                    and measurement_state in _SEALED_STATES | {"IDLE"}
                ),
                "can_finalize_measurement": measurement_state in {"POST_CAPTURE", "RECOVERABLE"},
                "can_close_session": (
                    session_state in {"OPEN", "CLOSING", "RECOVERABLE"}
                    and measurement_state in _SEALED_STATES | {"IDLE"}
                ),
            }
        )
        return dict(_json_copy(result))

    # ------------------------------------------------------------------
    # Begin / source registration / one detection interval

    def _create_core_package(
        self,
        root: Path,
        session_id: str,
        metadata: Mapping[str, object],
    ) -> object:
        from Workstation.workstation_core.measurement_package import MeasurementPackage

        creator = getattr(MeasurementPackage, "create", None)
        if callable(creator):
            return _call_supported(
                creator,
                root,
                session_id=session_id,
                metadata=dict(metadata),
            )
        begin = getattr(MeasurementPackage, "begin", None)
        if callable(begin):
            return _call_supported(
                begin,
                root,
                session_id=session_id,
                metadata=dict(metadata),
            )
        return MeasurementPackage(root, session_id=session_id, metadata=dict(metadata))

    def _measurement_method_config_for_new_package(self) -> dict[str, object]:
        owner_factory = getattr(self.panel, "_measurement_view_controller", None)
        owner = owner_factory() if callable(owner_factory) else None
        builder = getattr(owner, "measurement_method_config_for_new_measurement", None)
        if not callable(builder):
            return {}
        try:
            value = builder()
        except (AttributeError, TypeError, ValueError):
            return {}
        return dict(value) if isinstance(value, Mapping) else {}

    def _apply_automatic_run_identity(self, session_id: str) -> None:
        applier = getattr(self.panel, "apply_workstation_run_setup", None)
        if not callable(applier):
            return
        mode = "drone"
        platform_widget = getattr(self.panel, "run_platform_combo", None)
        platform_reader = getattr(platform_widget, "currentData", None)
        if callable(platform_reader):
            mode = str(platform_reader() or mode)
        try:
            applier(
                {
                    "experiment_id": "unassigned",
                    "run_id": session_id,
                    "condition_id": "unspecified",
                    "mode": mode,
                    "condition": {
                        "identity_source": "automatic_session_identity",
                        "operator_metadata": "not_collected",
                    },
                }
            )
        except Exception as exc:
            inventory_errors = getattr(self.panel, "_measurement_package_inventory_errors", [])
            inventory_errors.append(f"automatic run identity: {exc}")

    def _initialize_measurement_package(
        self,
        package: object,
        *,
        session_id: str,
        measurement_id: str = "",
        apply_run_identity: bool,
    ) -> None:
        self.panel._measurement_package = package
        self.panel.recording_session_id = session_id
        if measurement_id:
            self.panel._measurement_id = measurement_id
        self.panel._measurement_method_config_token = None
        self.panel._measurement_package_detection_started = False
        self.panel._measurement_package_detection_completed = False
        self.panel._measurement_started_monotonic_s = time.monotonic()
        self.panel._measurement_detection_started_monotonic_s = None
        self.panel._measurement_package_finalize_requested = False
        self.panel._measurement_package_registered_paths = set()
        self.panel._measurement_package_inventory_errors = []
        initial_core = self._core_snapshot()
        self.panel._measurement_package_revision = _revision_number(
            initial_core.get("revision_number", initial_core.get("revision")), 1
        )
        self.panel._measurement_package_calibration_collected = False
        self.panel._measurement_package_video_observations = None
        self.panel._measurement_package_rtcm_drain_requested_token = ""
        self.panel._measurement_package_rtcm_drain_result = None
        # Pre-capture belongs to the newly prepared measurement.  The
        # RecordingController consumes this flag instead of allocating another
        # legacy recording ID when Detection begins.
        self.panel._pnp_session_prepared = True
        self.panel._automatic_session_recording = False
        self.panel._recording_mode = "explicit_measurement_package"
        self.panel._session_save_finalized = False
        self.panel._session_save_retry_required = False
        previous_rows = getattr(self.panel, "distance_recording_rows", None)
        closer = getattr(previous_rows, "close", None)
        if callable(closer):
            closer()
        self.panel.distance_recording_rows = []
        keys = getattr(self.panel, "distance_recording_keys", None)
        if hasattr(keys, "clear"):
            keys.clear()
        pair_index = getattr(self.panel, "_recording_pair_row_index", None)
        if hasattr(pair_index, "clear"):
            pair_index.clear()
        self.panel.camera_recording_rows = {"left": [], "right": []}
        self.panel.px4_recording_rows = {"left": [], "right": []}
        self.panel.gnss_base_recording_rows = []
        self.panel.extrinsics_comparison_recording_rows = []
        self.panel.pnp_photo_capture_records = []
        self.panel.oz_photo_capture_records = []
        if apply_run_identity:
            self._apply_automatic_run_identity(session_id)
        detail_id = f"{session_id}/{measurement_id}" if measurement_id else session_id
        self._set_state("COLLECTING", f"Measurement {detail_id} is collecting pre evidence")

        self.activate_measurement_package_source(
            "camera.left",
            required_roles=("video/left/raw",),
            metadata={"side": "left"},
        )
        self.activate_measurement_package_source(
            "camera.right",
            required_roles=("video/right/raw",),
            metadata={"side": "right"},
        )
        self.activate_measurement_package_source(
            "measurement",
            required_roles=("measurements/pairs_parquet",),
        )
        self.activate_measurement_package_source(
            "session.persistence",
            required_roles=("session/finalization_manifest",),
        )
        if bool(getattr(self.panel, "_measurement_package_collect_provenance", True)):
            self._collect_begin_provenance()

    def begin_measurement_session(
        self,
        metadata: Mapping[str, object] | None = None,
        *,
        session_id: str | None = None,
    ) -> bool:
        current = _state_text(getattr(self.panel, "_measurement_package_state", "IDLE"))
        if self._hierarchy_enabled():
            existing_session = self.session
            existing_state = str(getattr(existing_session, "state", "") or "")
            if existing_session is not None and existing_state in {"OPEN", "CLOSING"}:
                self._set_state(
                    current,
                    f"Close the current session {existing_session.session_id} before starting another",
                )
                return False
        if current in _ACTIVE_STATES:
            self._set_state(current, "A measurement package is already active")
            return False
        active = getattr(self.panel, "_session_recording_active", None)
        if callable(active) and active():
            self._set_state(current, "Stop the existing recording session before Begin Session")
            return False
        if getattr(self.panel, "_session_save_thread", None) is not None:
            self._set_state(current, "Wait for the existing session save worker")
            return False
        if bool(getattr(self.panel, "_session_save_retry_required", False)):
            self._set_state(current, "Resolve the current measurement save retry before starting another")
            return False
        if any(
            getattr(self.panel, name, None) is not None
            for name in ("pnp_photo_capture_thread", "oz_photo_capture_thread")
        ):
            self._set_state(current, "Wait for current measurement capture workers")
            return False

        resolved_id = str(session_id or session_id_now()).strip()
        if not resolved_id or Path(resolved_id).name != resolved_id:
            raise ValueError("session_id must be one safe path component")
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            raise RuntimeError("recording session directory owner is unavailable")
        root = Path(resolver(resolved_id))
        method_config = self._measurement_method_config_for_new_package()
        package_metadata = {
            "initiator": "workstation_explicit_begin_session",
            "recording_mode": "explicit_measurement_package",
            **dict(metadata or {}),
            **method_config,
        }
        measurement_id = ""
        if self._hierarchy_enabled():
            from Workstation.workstation_core.measurement_session import MeasurementSession

            session_root = self._session_container_root(resolved_id)
            session = MeasurementSession.create(
                session_root,
                session_id=resolved_id,
                metadata={
                    "initiator": "workstation_explicit_begin_session",
                    **dict(metadata or {}),
                },
                app_run_id=(
                    str(getattr(self.panel, "_measurement_app_run_id", "") or "")
                    or f"run-{resolved_id}"
                ),
            )
            self.panel._measurement_session = session
            self.panel._measurement_session_id = resolved_id
            self.panel._measurement_session_state = "OPEN"
            self.panel.recording_session_id = resolved_id
            self.panel._measurement_id = ""
            self.panel._measurement_package = None
            self.panel._measurement_package_state = "IDLE"
            self.panel._measurement_package_detection_started = False
            self.panel._measurement_package_detection_completed = False
            self.panel._measurement_package_finalize_requested = False
            self.panel._measurement_session_close_requested = False
            self.panel._measurement_package_registered_paths = set()
            self.panel._measurement_package_inventory_errors = []
            self.panel._measurement_package_rtcm_drain_requested_token = ""
            self.panel._measurement_package_rtcm_drain_result = None
            self.panel._measurement_package_revision = 0
            previous_rows = getattr(self.panel, "distance_recording_rows", None)
            closer = getattr(previous_rows, "close", None)
            if callable(closer):
                closer()
            self.panel.distance_recording_rows = []
            self.panel.camera_recording_rows = {"left": [], "right": []}
            self.panel.px4_recording_rows = {"left": [], "right": []}
            self.panel.gnss_base_recording_rows = []
            self.panel.extrinsics_comparison_recording_rows = []
            self.panel.pnp_photo_capture_records = []
            self.panel.oz_photo_capture_records = []
            self.panel.video_recording_paths = {}
            self.panel._pnp_session_prepared = False
            self.panel._session_save_finalized = False
            self.panel._session_save_retry_required = False
            self._apply_automatic_run_identity(resolved_id)
            self._set_state(
                "IDLE",
                f"Session {resolved_id} is open; create a measurement",
            )
            logger = getattr(self.panel, "_log", None)
            if callable(logger):
                logger(f"[패키지] Begin Session: {resolved_id}")
            return True
        else:
            package = self._create_core_package(root, resolved_id, package_metadata)
        self._initialize_measurement_package(
            package,
            session_id=resolved_id,
            measurement_id=measurement_id,
            apply_run_identity=True,
        )
        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            suffix = f", measurement={measurement_id}" if measurement_id else ""
            logger(f"[패키지] Begin Session: {resolved_id}{suffix}")
        return True

    def activate_measurement_package_source(
        self,
        source_id: str,
        *,
        active: bool = True,
        required_roles: Sequence[str] = (),
        metadata: Mapping[str, object] | None = None,
    ) -> bool:
        package = self.package
        if package is None:
            return False
        setter = getattr(package, "set_source", None) or getattr(package, "record_source", None)
        if not callable(setter):
            return False
        _call_supported(
            setter,
            str(source_id),
            active=bool(active),
            required_roles=tuple(str(role) for role in required_roles),
            metadata=dict(metadata or {}),
        )
        return True

    def _register_descriptor(self, descriptor: Mapping[str, object]) -> None:
        package = self.package
        registrar = getattr(package, "register_descriptor", None)
        if not callable(registrar):
            raise RuntimeError("measurement package does not expose register_descriptor()")
        registrar(dict(descriptor))
        relative = str(descriptor.get("relative_path") or "")
        if relative and str(descriptor.get("status") or "present") == "present":
            registered = getattr(self.panel, "_measurement_package_registered_paths", set())
            registered.add((str(descriptor.get("role") or "descriptor"), relative))
            self.panel._measurement_package_registered_paths = registered

    def _register_descriptor_failure(self, role: str, error: Exception, source: str) -> None:
        try:
            self._register_missing(
                role,
                str(error),
                producer="workstation.package_provenance",
                source=source,
                failed=True,
            )
        except Exception:
            inventory_errors = getattr(
                self.panel, "_measurement_package_inventory_errors", []
            )
            inventory_errors.append(f"{role}: {error}")

    def _effective_settings_snapshot(self) -> dict[str, object]:
        values: dict[str, object] = {}
        reader = getattr(self.panel, "workstation_settings_snapshot", None)
        if callable(reader):
            try:
                snapshot = reader()
            except Exception as exc:
                values["settings_snapshot_error"] = str(exc)
            else:
                if isinstance(snapshot, Mapping):
                    values["settings"] = dict(snapshot)
        elif hasattr(self.panel, "settings"):
            settings = getattr(self.panel, "settings")
            values["settings"] = dict(getattr(settings, "__dict__", {}) or {})
        yolo_reader = getattr(self.panel, "yolo_kwargs", None)
        if callable(yolo_reader):
            try:
                values["yolo_runtime"] = yolo_reader()
            except Exception as exc:
                values["yolo_runtime_error"] = str(exc)
        values["recording_mode"] = "explicit_measurement_package"
        return values

    @staticmethod
    def _first_existing_path(*values: object) -> Path | None:
        for value in values:
            text = str(value or "").strip()
            if not text:
                continue
            candidate = Path(text).expanduser()
            if not candidate.is_absolute():
                candidate = resolve_workstation_data_path(candidate)
            if candidate.is_file():
                return candidate
        return None

    def _active_input_paths(self) -> dict[str, Path]:
        settings = getattr(self.panel, "settings", None)
        weights_widget = getattr(self.panel, "weights_edit", None)
        weights_text = getattr(weights_widget, "text", None)
        weights = self._first_existing_path(
            weights_text() if callable(weights_text) else None,
            getattr(settings, "yolo_weights_path", None),
        )
        intrinsics = self._first_existing_path(
            getattr(self.panel, "_active_intrinsics_profile_path", None)
        )
        extrinsics = self._first_existing_path(
            getattr(self.panel, "_active_extrinsics_profile_path", None)
        )
        return {
            role: path
            for role, path in (
                ("active_yolo_model", weights),
                ("active_intrinsics_profile", intrinsics),
                ("active_extrinsics_profile", extrinsics),
            )
            if path is not None
        }

    def _calibration_source_roots(self, profiles: Mapping[str, Path]) -> tuple[Path, ...]:
        candidates: list[object] = [path.parent for path in profiles.values()]
        candidates.extend(
            (
                getattr(self.panel, "_calibration_session_root", None),
                getattr(self.panel, "calibration_diagnostics_root", None),
            )
        )
        settings = getattr(self.panel, "settings", None)
        candidates.extend(
            (
                getattr(settings, "calibration_root", None),
                getattr(settings, "calibration_validation_root", None),
            )
        )
        widget = getattr(self.panel, "calib_dir_edit", None)
        reader = getattr(widget, "text", None)
        if callable(reader):
            candidates.append(reader())
        base = Path(getattr(self.panel, "app_root", Path.cwd())).expanduser().resolve()
        roots: list[Path] = []
        for raw in candidates:
            text = str(raw or "").strip()
            if not text:
                continue
            path = Path(text).expanduser()
            if not path.is_absolute():
                path = base / path
            resolved = path.resolve(strict=False)
            if resolved.is_dir() and resolved not in roots:
                roots.append(resolved)
        return tuple(roots)

    def _collect_begin_provenance(self) -> None:
        from Workstation.workstation_core.package_provenance import (
            collect_software_provenance,
            snapshot_input_file,
            write_atomic_json_snapshot,
        )

        root = self._session_root()
        try:
            descriptor = write_atomic_json_snapshot(
                root,
                "inputs/config/effective_settings.json",
                self._effective_settings_snapshot(),
                role="effective_settings",
                source="workstation_effective_settings",
                data_schema="workstation-effective-settings-v1",
            )
            self._register_descriptor(descriptor)
        except Exception as exc:
            self._register_descriptor_failure(
                "effective_settings", exc, "workstation_effective_settings"
            )

        try:
            repository_root = Path(__file__).resolve().parents[2]
            for descriptor in collect_software_provenance(root, repository_root):
                self._register_descriptor(descriptor)
        except Exception as exc:
            self._register_descriptor_failure(
                "software_provenance", exc, "runtime_and_repository"
            )

        for role, source_path in self._active_input_paths().items():
            destination = "inputs/models" if role == "active_yolo_model" else "inputs/calibration/active"
            try:
                descriptor = snapshot_input_file(
                    source_path,
                    root,
                    role=role,
                    destination_subdir=destination,
                    source="measurement.inputs",
                )
                self._register_descriptor(descriptor)
            except Exception as exc:
                self._register_descriptor_failure(role, exc, source_path.name)

    def _ensure_measurement_input_evidence(self) -> None:
        paths = self._active_input_paths()
        expected_roles: list[str] = []
        weights_widget = getattr(self.panel, "weights_edit", None)
        weights_reader = getattr(weights_widget, "text", None)
        if callable(weights_reader) and str(weights_reader() or "").strip():
            expected_roles.append("active_yolo_model")
        if getattr(self.panel, "active_intrinsics", None) is not None:
            expected_roles.append("active_intrinsics_profile")
        if str(getattr(self.panel, "_active_extrinsics_profile_path", "") or "").strip():
            expected_roles.append("active_extrinsics_profile")
        if not expected_roles:
            return
        self.activate_measurement_package_source(
            "measurement.inputs",
            required_roles=tuple(expected_roles),
            metadata={"activated_utc_ns": time.time_ns()},
        )
        from Workstation.workstation_core.package_provenance import snapshot_input_file

        for role in expected_roles:
            source_path = paths.get(role)
            if source_path is None:
                self._register_missing(
                    role,
                    "active measurement input has no readable source file",
                    producer="MeasurementPackageController",
                    source="measurement.inputs",
                )
                continue
            destination = "inputs/models" if role == "active_yolo_model" else "inputs/calibration/active"
            try:
                self._register_descriptor(
                    snapshot_input_file(
                        source_path,
                        self._session_root(),
                        role=role,
                        destination_subdir=destination,
                        source="measurement.inputs",
                    )
                )
            except Exception as exc:
                self._register_descriptor_failure(role, exc, "measurement.inputs")

    def _collect_finalize_calibration_bundle(self) -> None:
        if bool(getattr(self.panel, "_measurement_package_calibration_collected", False)):
            return
        paths = self._active_input_paths()
        profiles = {
            role: path
            for role, path in paths.items()
            if role in {"active_intrinsics_profile", "active_extrinsics_profile"}
        }
        if not profiles:
            self.panel._measurement_package_calibration_collected = True
            return
        from Workstation.workstation_core.package_provenance import collect_calibration_bundle

        try:
            for descriptor in collect_calibration_bundle(
                self._session_root(),
                profiles,
                source_roots=self._calibration_source_roots(profiles),
            ):
                self._register_descriptor(descriptor)
        except Exception as exc:
            self._register_descriptor_failure(
                "calibration_bundle_index", exc, "calibration_collector"
            )
        self.panel._measurement_package_calibration_collected = True

    def _activate_runtime_sources(self) -> None:
        now_ns = time.time_ns()
        latest_extrinsics = getattr(self.panel, "latest_extrinsics", {}) or {}
        latest_gps = getattr(self.panel, "latest_gps", {}) or {}
        px4_runtime = getattr(self.panel, "mavlink_pixhawk", None)
        for side in ("left", "right"):
            if px4_runtime is not None or side in latest_extrinsics or side in latest_gps:
                self.activate_measurement_package_source(
                    f"px4.{side}",
                    required_roles=(f"telemetry/px4/{side}/parquet",),
                    metadata={"activated_utc_ns": now_ns, "side": side},
                )

        gnss_state = ""
        manager = getattr(self.panel, "gnss_operation_manager", None)
        metadata_reader = getattr(manager, "metadata", None)
        if callable(metadata_reader):
            try:
                metadata = metadata_reader()
                if isinstance(metadata, Mapping):
                    gnss_state = str(metadata.get("gnss_operation_state") or "")
            except Exception:
                gnss_state = ""
        if (
            bool(getattr(self.panel, "gnss_base_serial_connected", False))
            or bool(getattr(self.panel, "gnss_network_connected", False))
            or gnss_state.upper() not in {"", "DISCONNECTED", "IDLE"}
        ):
            self.activate_measurement_package_source(
                "gnss.base",
                required_roles=("telemetry/gnss/base/parquet",),
                metadata={"activated_utc_ns": now_ns, "operation_state": gnss_state},
            )
        network_phase = str(getattr(self.panel, "gnss_network_phase", "") or "").upper()
        if bool(getattr(self.panel, "gnss_network_connected", False)) or network_phase not in {
            "",
            "DISCONNECTED",
            "IDLE",
        }:
            self.activate_measurement_package_source(
                "gnss.rtcm",
                required_roles=("corrections/rtcm/raw",),
                metadata={"activated_utc_ns": now_ns, "network_phase": network_phase},
            )

        drone_state = str(
            getattr(self.panel, "drone_telemetry_connection_state", "") or ""
        ).upper()
        if drone_state not in {"", "DISCONNECTED", "IDLE"}:
            self.activate_measurement_package_source(
                "px4.drone",
                required_roles=("telemetry/px4/drone/parquet",),
                metadata={"activated_utc_ns": now_ns, "connection_state": drone_state},
            )

        target_reader = getattr(self.panel, "target_drone_stream_snapshot", None)
        if callable(target_reader):
            try:
                target = target_reader()
            except Exception:
                target = None
            if isinstance(target, Mapping):
                phase = str(target.get("phase") or "").upper()
                if phase not in {"", "UNCONFIGURED", "DEFERRED"}:
                    self.activate_measurement_package_source(
                        "target.drone",
                        required_roles=("telemetry/target/raw",),
                        metadata={"activated_utc_ns": now_ns, "phase": phase},
                    )

        ulog_reader = getattr(self.panel, "workstation_ulog_snapshot", None)
        if callable(ulog_reader):
            try:
                ulog = ulog_reader()
            except Exception:
                ulog = None
            if isinstance(ulog, Mapping):
                for raw in ulog.get("artifacts", ()) or ():
                    if not isinstance(raw, Mapping):
                        continue
                    source_id = self._safe_role_component(raw.get("source_id") or "unknown")
                    self.activate_measurement_package_source(
                        f"ulog.{source_id}",
                        required_roles=(f"ulog/{source_id}",),
                        metadata={"activated_utc_ns": now_ns},
                    )

    def begin_measurement_detection(self) -> bool:
        state = _state_text(getattr(self.panel, "_measurement_package_state", "IDLE"))
        if state != "COLLECTING":
            self._set_state(state, "Detection requires an explicit COLLECTING session")
            return False
        if bool(getattr(self.panel, "_measurement_package_detection_started", False)):
            self._set_state(state, "This session already used its single Detection interval")
            return False
        self.panel._measurement_package_detection_started = True
        self.panel._measurement_detection_started_monotonic_s = time.monotonic()
        self._ensure_measurement_input_evidence()
        self._activate_runtime_sources()
        self.activate_measurement_package_source("detector.yolo", metadata={"interval_count": 1})
        self._set_state("COLLECTING", "Detection interval is running")
        return True

    def cancel_measurement_detection_start(self, reason: str = "Detection did not start") -> bool:
        """Release a reservation only before any detector/recording producer ran."""

        if bool(getattr(self.panel, "_measurement_package_detection_completed", False)):
            return False
        active = getattr(self.panel, "_session_recording_active", None)
        if callable(active) and active():
            return False
        self.panel._measurement_package_detection_started = False
        self._set_state("COLLECTING", str(reason))
        return True

    def complete_measurement_detection(self) -> bool:
        state = _state_text(getattr(self.panel, "_measurement_package_state", "IDLE"))
        if state == "POST_CAPTURE" and bool(
            getattr(self.panel, "_measurement_package_detection_completed", False)
        ):
            return True
        if state != "COLLECTING" or not bool(
            getattr(self.panel, "_measurement_package_detection_started", False)
        ):
            self._set_state(state, "No active package Detection interval can be stopped")
            return False
        self.panel._measurement_package_detection_completed = True
        package = self.package
        transition = getattr(package, "mark_post_capture", None)
        if callable(transition):
            transition()
        else:
            transition = getattr(package, "transition", None)
            if callable(transition):
                _call_supported(transition, "POST_CAPTURE")
        self._sync_measurement_session()
        self._set_state("POST_CAPTURE", "Detection ended; collect post evidence, then Finalize")
        return True

    # ------------------------------------------------------------------
    # Finalization and inventory

    def _schedule_finalize_retry(self) -> None:
        if bool(getattr(self.panel, "_measurement_package_finalize_retry_scheduled", False)):
            return
        self.panel._measurement_package_finalize_retry_scheduled = True

        def retry() -> None:
            self.panel._measurement_package_finalize_retry_scheduled = False
            if bool(getattr(self.panel, "_measurement_package_finalize_requested", False)):
                self._continue_measurement_package_finalization()

        QtCore.QTimer.singleShot(100, retry)

    @staticmethod
    def _thread_running(thread: object | None) -> bool:
        if thread is None:
            return False
        checker = getattr(thread, "isRunning", None)
        if callable(checker):
            try:
                return bool(checker())
            except RuntimeError:
                return False
        return True

    def _producer_wait_reason(self) -> str:
        if getattr(self.panel, "pnp_photo_capture_thread", None) is not None:
            return "waiting for HR capture/FTP worker"
        if getattr(self.panel, "oz_photo_capture_thread", None) is not None:
            return "waiting for OZ capture/FTP worker"
        if getattr(self.panel, "_session_save_thread", None) is not None:
            return "waiting for session persistence worker"
        if getattr(self.panel, "_pending_camera_start", None) is not None:
            return "waiting for Detection camera writers to close before preview restart"

        camera_connected = bool(getattr(self.panel, "camera_connected", False))
        if not camera_connected:
            for side in ("left", "right"):
                pane = getattr(self.panel, f"{side}_pane", None)
                thread = getattr(pane, "thread", None)
                if self._thread_running(thread):
                    stopper = getattr(pane, "stop", None)
                    if callable(stopper):
                        stopper(wait=False)
                    return f"waiting for {side} camera writer"
        return ""

    def _video_stability_wait_reason(self) -> str:
        if not bool(getattr(self.panel, "_measurement_package_verify_media", True)):
            return ""
        from Workstation.workstation_core.media_evidence import (
            observe_video_files,
            video_files_stable,
        )

        root = self._session_root()
        paths = tuple(root / "video" / f"raw_{side}.mp4" for side in ("left", "right"))
        observed = observe_video_files(paths)
        # Missing/empty/failed video is a completeness result, not a reason to
        # wait forever once every producer has already stopped.
        if not all(item.ready for item in observed.values()):
            self.panel._measurement_package_video_observations = None
            return ""
        previous = getattr(self.panel, "_measurement_package_video_observations", None)
        self.panel._measurement_package_video_observations = observed
        if not isinstance(previous, Mapping) or not video_files_stable(paths, previous, observed):
            return "confirming that finalized video bytes are stable"
        return ""

    def _rtcm_context_token(self) -> str:
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        measurement_id = str(getattr(self.panel, "_measurement_id", "") or "")
        revision = max(
            0,
            int(getattr(self.panel, "_measurement_package_revision", 0) or 0),
        )
        if not session_id:
            return ""
        return f"{session_id}:{measurement_id or 'session'}:r{revision:04d}"

    def _handle_measurement_rtcm_evidence_drained(self, payload: object) -> bool:
        """Accept one exact RTCM evidence barrier result and resume sealing."""

        if not isinstance(payload, Mapping) or str(payload.get("schema") or "") != (
            "workstation-rtcm-evidence-drain-v1"
        ):
            return False
        token = str(payload.get("context_token") or "")
        expected = str(
            getattr(self.panel, "_measurement_package_rtcm_drain_requested_token", "")
            or ""
        )
        if not token or token != expected or token != self._rtcm_context_token():
            return False
        result = dict(_json_copy(dict(payload)))
        self.panel._measurement_package_rtcm_drain_result = result
        if not bool(result.get("valid")):
            error = str(result.get("error") or "RTCM evidence drain integrity failed")
            inventory_errors = getattr(
                self.panel, "_measurement_package_inventory_errors", []
            )
            if isinstance(inventory_errors, list) and error not in inventory_errors:
                inventory_errors.append(error)
            self._set_state(
                "RECOVERABLE",
                "RTCM evidence did not reach a verified durable barrier",
                error,
            )
            return False
        self._set_state("VERIFYING", "RTCM evidence drained and verified")
        if bool(getattr(self.panel, "_measurement_package_finalize_requested", False)):
            QtCore.QTimer.singleShot(0, self._continue_measurement_package_finalization)
        return True

    def _ensure_rtcm_evidence_drained(self) -> bool:
        pipeline = getattr(self.panel, "rtcm_pipeline_worker", None)
        thread = getattr(self.panel, "rtcm_pipeline_thread", None)
        if pipeline is None or thread is None:
            return True
        running = getattr(thread, "isRunning", None)
        if callable(running):
            try:
                if not bool(running()):
                    return True
            except RuntimeError:
                return True

        token = self._rtcm_context_token()
        if not token:
            self._set_state(
                "RECOVERABLE",
                "RTCM evidence barrier could not resolve the measurement identity",
            )
            return False
        result = getattr(self.panel, "_measurement_package_rtcm_drain_result", None)
        if isinstance(result, Mapping) and str(result.get("context_token") or "") == token:
            if bool(result.get("valid")):
                return True
            self._set_state(
                "RECOVERABLE",
                "RTCM evidence did not reach a verified durable barrier",
                str(result.get("error") or "RTCM evidence drain integrity failed"),
            )
            return False

        requested = str(
            getattr(self.panel, "_measurement_package_rtcm_drain_requested_token", "")
            or ""
        )
        if requested == token:
            self._set_state("VERIFYING", "waiting for RTCM evidence drain barrier")
            self._schedule_finalize_retry()
            return False

        closer = getattr(self.panel, "_close_rtcm_evidence_context_and_drain", None)
        if not callable(closer):
            self._set_state(
                "RECOVERABLE",
                "RTCM pipeline is active but its evidence drain boundary is unavailable",
            )
            return False
        self.panel._measurement_package_rtcm_drain_requested_token = token
        self.panel._measurement_package_rtcm_drain_result = None
        accepted = bool(closer(token))
        immediate = getattr(self.panel, "_measurement_package_rtcm_drain_result", None)
        immediate_valid = (
            isinstance(immediate, Mapping)
            and str(immediate.get("context_token") or "") == token
            and bool(immediate.get("valid"))
        )
        if not accepted and not immediate_valid:
            self._set_state(
                "RECOVERABLE",
                "RTCM evidence drain request was rejected",
                str(
                    immediate.get("error")
                    if isinstance(immediate, Mapping)
                    else "RTCM pipeline rejected the active context token"
                ),
            )
            return False
        if immediate_valid:
            return True
        self._set_state("VERIFYING", "waiting for RTCM evidence drain barrier")
        self._schedule_finalize_retry()
        return False

    def finalize_measurement_package(self) -> bool:
        state = _state_text(getattr(self.panel, "_measurement_package_state", "IDLE"))
        if state not in {"POST_CAPTURE", "VERIFYING", "RECOVERABLE"}:
            self._set_state(state, "Finalize requires one completed Detection interval")
            return False
        if not bool(getattr(self.panel, "_measurement_package_detection_completed", False)):
            self._set_state(state, "Finalize requires Detection stop/Post Capture")
            return False
        self.panel._measurement_package_finalize_requested = True
        self._set_state("VERIFYING", "Draining producers and verifying session artifacts")
        return self._continue_measurement_package_finalization()

    def _continue_measurement_package_finalization(self) -> bool:
        if not bool(getattr(self.panel, "_measurement_package_finalize_requested", False)):
            return False

        existing_core_state = _state_text(self._core_snapshot().get("state"), "IDLE")
        if self._hierarchy_enabled() and existing_core_state in _SEALED_STATES:
            try:
                self._sync_measurement_session()
            except Exception as exc:
                self.panel._measurement_package_finalize_requested = False
                self.panel._measurement_session_state = "RECOVERABLE"
                self._set_state(
                    "RECOVERABLE",
                    "Measurement is sealed, but the parent session index still needs recovery",
                    str(exc),
                )
                return False
            self.panel._measurement_package_finalize_requested = False
            self._set_state(existing_core_state, f"Package sealed as {existing_core_state}")
            if bool(getattr(self.panel, "_measurement_session_close_requested", False)):
                self.panel._measurement_session_close_requested = False
                return self.close_measurement_session()
            return True

        active = getattr(self.panel, "_session_recording_active", None)
        if callable(active) and active():
            stopper = getattr(self.panel, "stop_automatic_session_recording", None)
            if callable(stopper):
                stopper()
            if active():
                self._set_state("VERIFYING", "waiting for recording producers to stop")
                self._schedule_finalize_retry()
                return False

        wait_reason = self._producer_wait_reason()
        if wait_reason:
            self._set_state("VERIFYING", wait_reason)
            self._schedule_finalize_retry()
            return False

        stability_reason = self._video_stability_wait_reason()
        if stability_reason:
            self._set_state("VERIFYING", stability_reason)
            self._schedule_finalize_retry()
            return False

        # Session persistence is a hard barrier, not merely another inventory
        # item.  A failed/torn Parquet generation must retain its working spool
        # and revision for an explicit retry; sealing an incomplete revision
        # here would make that retry impossible.
        if bool(getattr(self.panel, "_session_save_retry_required", False)):
            self._set_state(
                "RECOVERABLE",
                "Session persistence failed; retry the session save before sealing",
                str(getattr(self.panel, "_session_save_last_error", "") or ""),
            )
            return False

        # A normal Detection stop starts this save.  If a boundary returned
        # early, start it here once instead of sealing a misleading package.
        if (
            hasattr(self.panel, "_session_save_finalized")
            and not bool(getattr(self.panel, "_session_save_finalized", False))
        ):
            saver = getattr(self.panel, "save_distance_history_csv", None)
            if callable(saver):
                saver()
                if getattr(self.panel, "_session_save_thread", None) is not None:
                    self._set_state("VERIFYING", "waiting for session persistence worker")
                    self._schedule_finalize_retry()
                    return False
            if bool(getattr(self.panel, "_session_save_retry_required", False)):
                self._set_state(
                    "RECOVERABLE",
                    "Session persistence failed; retry the session save before sealing",
                    str(getattr(self.panel, "_session_save_last_error", "") or ""),
                )
                return False
            if not bool(getattr(self.panel, "_session_save_finalized", False)):
                self._set_state(
                    "RECOVERABLE",
                    "Session persistence did not publish a verified generation; sealing was blocked",
                )
                return False

        if not self._ensure_rtcm_evidence_drained():
            return False

        try:
            self._activate_runtime_sources()
            self._ensure_measurement_input_evidence()
            self._collect_finalize_calibration_bundle()
            self._register_session_inventory()
            package = self.package
            if package is None:
                raise RuntimeError("measurement package owner is unavailable")
            sealer = getattr(package, "seal", None)
            if not callable(sealer):
                raise RuntimeError("measurement package does not expose seal()")
            outcome = self._measurement_outcome()
            manifest = _call_supported(sealer, measurement_outcome=outcome)
        except Exception as exc:
            # Never discard or replace a working revision here.  The core's
            # recovery area and the recording spool remain the retry source.
            self.panel._measurement_package_finalize_requested = False
            try:
                self._sync_measurement_session()
            except Exception as sync_exc:
                self.panel._measurement_session_state = "RECOVERABLE"
                inventory_errors = getattr(
                    self.panel, "_measurement_package_inventory_errors", []
                )
                inventory_errors.append(f"session index recovery: {sync_exc}")
            self._set_state(
                "RECOVERABLE",
                "Package sealing failed; working files and spool were preserved",
                str(exc),
            )
            logger = getattr(self.panel, "_log", None)
            if callable(logger):
                logger(f"[패키지] Finalize 실패; 복구 가능 상태를 보존했습니다: {exc}")
            return False

        manifest_mapping = dict(manifest) if isinstance(manifest, Mapping) else self._core_snapshot()
        final_state = _state_text(
            manifest_mapping.get("state")
            or manifest_mapping.get("status")
            or self._core_snapshot().get("state"),
            "SEALED_INCOMPLETE",
        )
        if final_state not in _SEALED_STATES:
            complete = manifest_mapping.get("complete")
            final_state = "SEALED_COMPLETE" if complete is True else "SEALED_INCOMPLETE"
        self.panel._measurement_package_finalize_requested = False
        try:
            self._sync_measurement_session()
        except Exception as exc:
            self.panel._measurement_session_state = "RECOVERABLE"
            self._set_state(
                "RECOVERABLE",
                "Measurement sealed; parent session index update failed",
                str(exc),
            )
            return False
        self._set_state(final_state, f"Package sealed as {final_state}")
        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            logger(f"[패키지] Finalize 완료: {final_state}")
        if bool(getattr(self.panel, "_measurement_session_close_requested", False)):
            self.panel._measurement_session_close_requested = False
            return self.close_measurement_session()
        return True

    def _measurement_outcome(self) -> dict[str, object]:
        rows = getattr(self.panel, "distance_recording_rows", ()) or ()
        valid = 0
        rejected = 0
        try:
            iterator = list(rows)
        except TypeError:
            iterator = []
        for row in iterator:
            if not isinstance(row, Mapping):
                continue
            value = str(row.get("measurement_valid", "")).lower()
            if value in {"true", "1", "yes"} or row.get("measurement_valid") is True:
                valid += 1
            else:
                rejected += 1
        result = "VALID" if valid else "NO_VALID_RESULT"
        return {
            "result": result,
            "valid_measurement_count": valid,
            "rejected_or_blocked_count": rejected,
            "evidence_completeness_is_independent": True,
        }

    def _session_root(self) -> Path:
        if self._hierarchy_enabled():
            package = self.package
            package_root = getattr(package, "root", None)
            if package_root is not None:
                return Path(package_root).resolve()
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not session_id or not callable(resolver):
            raise RuntimeError("measurement session root is unavailable")
        return Path(resolver(session_id)).resolve()

    @staticmethod
    def _classification(relative: str) -> str:
        lower = relative.lower()
        if (
            lower.startswith((".spool/", ".workstation/recovery/"))
            or "raw_" in Path(lower).name
            or lower.endswith((".ulog", ".parquet"))
        ):
            return "raw"
        if lower.startswith(("frames/", "hr/")) or lower.endswith((".png", ".jpg", ".jpeg")):
            return "input"
        if lower.endswith((".md", ".xlsx")):
            return "report"
        return "derived"

    @staticmethod
    def _producer_source(relative: str) -> tuple[str, str]:
        lower = relative.lower()
        if lower.startswith("video/"):
            side = "left" if "left" in lower else "right" if "right" in lower else "cameras"
            return "CameraPane/YoloStreamWorker", f"camera.{side}"
        if lower.startswith(("frames/hr_capture/", "hr/")):
            return "WirisPnPPhotoCaptureWorker", "wiris.hr"
        if lower.startswith("frames/oz_capture/"):
            return "WirisOZPhotoCaptureWorker", "wiris.oz"
        if lower.endswith(".ulog"):
            return "ULogController", "px4"
        if lower.startswith(".spool/"):
            return "RecordingController", "session.persistence"
        return "SessionSaveWorker", "session.persistence"

    @staticmethod
    def _media_type(path: Path) -> str:
        explicit = {
            ".jsonl": "application/x-ndjson",
            ".parquet": "application/vnd.apache.parquet",
            ".ulog": "application/vnd.px4.ulog",
            ".xlsx": "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
        }
        return explicit.get(path.suffix.lower()) or mimetypes.guess_type(path.name)[0] or "application/octet-stream"

    def _register_artifact(
        self,
        role: str,
        path: Path,
        *,
        required: bool,
        producer: str,
        source: str,
        classification: str | None = None,
        metadata: Mapping[str, object] | None = None,
    ) -> bool:
        root = self._session_root()
        try:
            resolved = path.resolve(strict=True)
            relative = resolved.relative_to(root).as_posix()
        except (FileNotFoundError, ValueError):
            return False
        if not resolved.is_file():
            return False
        registered = getattr(self.panel, "_measurement_package_registered_paths", set())
        key = (str(role), relative)
        if key in registered:
            return True
        package = self.package
        registrar = getattr(package, "register_artifact", None)
        if not callable(registrar):
            raise RuntimeError("measurement package does not expose register_artifact()")
        _call_supported(
            registrar,
            relative,
            role=str(role),
            classification=classification or self._classification(relative),
            producer=str(producer),
            source=str(source),
            required=bool(required),
            status="present",
            media_type=self._media_type(resolved),
            metadata=dict(metadata or {}),
        )
        registered.add(key)
        self.panel._measurement_package_registered_paths = registered
        return True

    def _register_missing(
        self,
        role: str,
        reason: str,
        *,
        producer: str,
        source: str,
        failed: bool = False,
        metadata: Mapping[str, object] | None = None,
    ) -> None:
        package = self.package
        method_name = "register_failed" if failed else "register_missing"
        registrar = getattr(package, method_name, None)
        if not callable(registrar) and failed:
            registrar = getattr(package, "register_missing", None)
        if not callable(registrar):
            raise RuntimeError(f"measurement package does not expose {method_name}()")
        _call_supported(
            registrar,
            str(role),
            reason=str(reason),
            producer=str(producer),
            source=str(source),
            required=True,
            metadata=dict(metadata or {}),
        )

    def _required_file(
        self,
        role: str,
        path: Path | None,
        *,
        producer: str,
        source: str,
        reason: str,
        metadata: Mapping[str, object] | None = None,
        allow_empty: bool = False,
    ) -> None:
        if path is not None and path.is_file() and (allow_empty or path.stat().st_size > 0):
            if self._register_artifact(
                role,
                path,
                required=True,
                producer=producer,
                source=source,
                metadata=metadata,
            ):
                return
        self._register_missing(role, reason, producer=producer, source=source, metadata=metadata)

    def _actual_generation_file(self, suffix: str) -> Path | None:
        root = self._session_root()
        manifest_path = root / "measurement.json"
        if manifest_path.is_file():
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                resources = manifest.get("resource_files")
                resources = resources if isinstance(resources, Mapping) else {}
                declared = next(
                    (
                        str(value)
                        for value in resources.values()
                        if str(value).endswith(suffix)
                    ),
                    "",
                )
            except (OSError, ValueError):
                declared = ""
            if declared:
                candidate = root / declared
                try:
                    resolved = candidate.resolve(strict=True)
                    resolved.relative_to(root)
                except (OSError, ValueError):
                    resolved = None
                if resolved is not None and resolved.is_file():
                    return resolved
        # Legacy fallback for pre-v2 finalized sessions.
        legacy_manifest = root / "session_manifest.json"
        if legacy_manifest.is_file():
            try:
                manifest = json.loads(legacy_manifest.read_text(encoding="utf-8"))
                generation = str(manifest.get("generation_id") or "")
            except (OSError, ValueError):
                generation = ""
            if generation:
                matches = sorted(
                    (root / ".generations" / generation).glob(f"*{suffix}")
                )
                if matches:
                    return matches[0]
        current_data = root / "data"
        matches = sorted(current_data.glob(f"*{suffix}")) if current_data.is_dir() else []
        if matches:
            return matches[0].resolve()
        matches = sorted(
            path
            for path in (root / ".generations").glob(f"*/*{suffix}")
            if path.is_file()
        )
        return matches[-1] if matches else None

    @staticmethod
    def _canonical_payload_digest(value: object) -> str | None:
        if not isinstance(value, str) or not value:
            return None
        try:
            payload = json.loads(value)
            encoded = json.dumps(
                payload,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        except (TypeError, ValueError):
            return None
        return hashlib.sha256(encoded).hexdigest()

    def _register_embedded_pair_payload(
        self,
        *,
        raw_path: Path,
        row: Mapping[str, object],
        row_index: int,
        role: str,
        source: str,
        value_field: str,
        hash_field: str,
    ) -> None:
        actual = self._canonical_payload_digest(row.get(value_field))
        expected = str(row.get(hash_field) or "")
        if actual is None or not expected or actual != expected:
            self._register_missing(
                role,
                f"{value_field} is absent, malformed, or does not match {hash_field}",
                producer="RecordingController",
                source=source,
                failed=True,
                metadata={"row_index": row_index, "embedded_field": value_field},
            )
            return
        self._register_artifact(
            role,
            raw_path,
            required=True,
            producer="RecordingController/SessionSaveWorker",
            source=source,
            classification="raw",
            metadata={
                "row_index": row_index,
                "embedded_field": value_field,
                "payload_sha256": actual,
            },
        )

    def _register_measurement_pair_evidence(self, raw_path: Path | None) -> None:
        if raw_path is None or not raw_path.is_file():
            return
        try:
            if raw_path.suffix.lower() == ".parquet":
                from Workstation.workstation_core.session_parquet import (
                    iter_lossless_rows,
                    verify_parquet_artifact,
                )

                verify_parquet_artifact(raw_path, "measurement_pairs")
                rows = iter_lossless_rows(
                    raw_path, "measurement_pairs", verify=False
                )
            else:
                with raw_path.open("r", encoding="utf-8") as handle:
                    rows = [json.loads(line) for line in handle if line.strip()]
        except (OSError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._register_missing(
                "measurements/pair_evidence_index",
                f"authoritative pair data could not be parsed: {exc}",
                producer="SessionSaveWorker",
                source="measurement",
                failed=True,
            )
            return

        root = self._session_root()
        for row_index, value in enumerate(rows):
            if not isinstance(value, Mapping):
                continue
            row = dict(value)
            identity = str(
                row.get("measurement_attempt_id")
                or row.get("blocked_pair_sha256")
                or row.get("pair_id")
                or row_index
            )
            component = self._safe_role_component(identity)
            source_id = f"measurement.pair.{component}"
            prefix = f"measurements/pairs/{component}"
            input_field = (
                "measurement_attempt_canonical_json"
                if row.get("measurement_attempt_canonical_json")
                else "blocked_pair_canonical_json"
            )
            input_hash = (
                "measurement_attempt_sha256"
                if input_field == "measurement_attempt_canonical_json"
                else "blocked_pair_sha256"
            )
            required_roles = [
                f"{prefix}/input",
                f"{prefix}/result",
                f"{prefix}/frame/left",
                f"{prefix}/frame/right",
            ]
            if row.get("measurement_readiness_sha256"):
                required_roles.append(f"{prefix}/readiness")
            self.activate_measurement_package_source(
                source_id,
                required_roles=tuple(required_roles),
                metadata={"row_index": row_index, "pair_identity": identity},
            )
            self._register_embedded_pair_payload(
                raw_path=raw_path,
                row=row,
                row_index=row_index,
                role=f"{prefix}/input",
                source=source_id,
                value_field=input_field,
                hash_field=input_hash,
            )
            self._register_embedded_pair_payload(
                raw_path=raw_path,
                row=row,
                row_index=row_index,
                role=f"{prefix}/result",
                source=source_id,
                value_field="measurement_result_canonical_json",
                hash_field="measurement_result_sha256",
            )
            if row.get("measurement_readiness_sha256"):
                self._register_embedded_pair_payload(
                    raw_path=raw_path,
                    row=row,
                    row_index=row_index,
                    role=f"{prefix}/readiness",
                    source=source_id,
                    value_field="measurement_readiness_canonical_json",
                    hash_field="measurement_readiness_sha256",
                )
            for side in ("left", "right"):
                relative = str(row.get(f"native_{side}_png_path") or "")
                native: Path | None = None
                if relative:
                    try:
                        candidate = (root / relative).resolve(strict=False)
                        candidate.relative_to(root)
                        native = candidate
                    except (OSError, RuntimeError, ValueError):
                        native = None
                expected_sha = str(row.get(f"native_{side}_png_sha256") or "")
                role = f"{prefix}/frame/{side}"
                if native is None or not native.is_file():
                    self._register_missing(
                        role,
                        f"exact evaluated-pair {side} native PNG is missing",
                        producer="NativeFrameStore",
                        source=source_id,
                        metadata={"row_index": row_index, "side": side},
                    )
                    continue
                actual_sha = hashlib.sha256(native.read_bytes()).hexdigest()
                if not expected_sha or actual_sha != expected_sha:
                    self._register_missing(
                        role,
                        f"{side} native PNG does not match row SHA-256",
                        producer="NativeFrameStore",
                        source=source_id,
                        failed=True,
                        metadata={"row_index": row_index, "side": side},
                    )
                    continue
                self._register_artifact(
                    role,
                    native,
                    required=True,
                    producer="NativeFrameStore",
                    source=source_id,
                    classification="raw",
                    metadata={
                        "row_index": row_index,
                        "side": side,
                        "pixel_contract": "native decoded pixels; lossless PNG; no overlay",
                    },
                )

    def _register_runtime_source_inventory(self) -> None:
        root = self._session_root()
        pixhawk_parquet = self._actual_generation_file("pixhawk_timeline.parquet")
        pixhawk_sources: set[str] = set()
        if pixhawk_parquet is not None and pixhawk_parquet.is_file():
            try:
                from Workstation.workstation_core.session_parquet import iter_lossless_rows

                pixhawk_sources = {
                    str(row.get("source_id") or "")
                    for row in iter_lossless_rows(
                        pixhawk_parquet, "pixhawk_timeline"
                    )
                }
            except (OSError, TypeError, ValueError):
                pixhawk_sources = set()
        for side in ("left", "right"):
            source = f"px4.{side}"
            expected = any(
                row.get("source_id") == source and bool(row.get("active"))
                for row in self._core_snapshot().get("source_expectations", ()) or ()
                if isinstance(row, Mapping)
            )
            if expected:
                if side in pixhawk_sources:
                    self._register_artifact(
                        f"telemetry/px4/{side}/parquet",
                        pixhawk_parquet,
                        required=True,
                        producer="SessionSaveWorker",
                        source=source,
                        classification="raw",
                        metadata={"partition": {"source_id": side}},
                    )
                else:
                    self._register_missing(
                        f"telemetry/px4/{side}/parquet",
                        f"active PX4 {side} source produced no typed timeline rows",
                        producer="SessionSaveWorker",
                        source=source,
                    )

        expectations = {
            str(row.get("source_id") or "")
            for row in self._core_snapshot().get("source_expectations", ()) or ()
            if isinstance(row, Mapping) and bool(row.get("active"))
        }
        if "gnss.base" in expectations:
            gnss_parquet = self._actual_generation_file("gnss_base_timeline.parquet")
            gnss_has_rows = False
            if gnss_parquet is not None and gnss_parquet.is_file():
                try:
                    from Workstation.workstation_core.session_parquet import iter_lossless_rows

                    gnss_has_rows = next(
                        iter(
                            iter_lossless_rows(
                                gnss_parquet, "gnss_base_timeline"
                            )
                        ),
                        None,
                    ) is not None
                except (OSError, TypeError, ValueError):
                    gnss_has_rows = False
            if gnss_has_rows:
                self._register_artifact(
                    "telemetry/gnss/base/parquet",
                    gnss_parquet,
                    required=True,
                    producer="SessionSaveWorker",
                    source="gnss.base",
                    classification="raw",
                )
            else:
                self._register_missing(
                    "telemetry/gnss/base/parquet",
                    "active Base GNSS source produced no typed timeline rows",
                    producer="SessionSaveWorker",
                    source="gnss.base",
                )
        if "gnss.rtcm" in expectations:
            rtcm_root = root / "raw" / "gnss"
            binaries = sorted(rtcm_root.glob("*.rtcm")) if rtcm_root.is_dir() else []
            raw_rtcm = next(
                (path for path in binaries if path.is_file() and path.stat().st_size),
                None,
            )
            drain = getattr(self.panel, "_measurement_package_rtcm_drain_result", None)
            drain_metadata = (
                {
                    "context_token": str(drain.get("context_token") or ""),
                    "high_water": dict(drain.get("high_water") or {}),
                    "unconfirmed_injection_sequences": list(
                        drain.get("unconfirmed_injection_sequences") or ()
                    ),
                    "integrity": dict(drain.get("integrity") or {}),
                    "semantic_validation": "passed",
                }
                if isinstance(drain, Mapping) and bool(drain.get("valid"))
                else {}
            )
            self._required_file(
                "corrections/rtcm/raw",
                raw_rtcm,
                producer="GnssRuntimeController",
                source="gnss.rtcm",
                reason="active NTRIP/RTCM source has no exact correction-byte artifact",
                metadata=drain_metadata,
            )
            for binary in binaries:
                if not binary.is_file():
                    continue
                self._register_artifact(
                    f"corrections/rtcm/raw/{binary.stem}",
                    binary,
                    required=True,
                    producer="RtcmDataPlaneWorker",
                    source="gnss.rtcm",
                    classification="raw",
                    metadata=drain_metadata,
                )
                index = binary.with_suffix(binary.suffix + ".jsonl")
                if index.is_file():
                    self._register_artifact(
                        f"corrections/rtcm/index/{binary.stem}",
                        index,
                        required=True,
                        producer="RtcmDataPlaneWorker",
                        source="gnss.rtcm",
                        classification="raw",
                        metadata={
                            **drain_metadata,
                            "offset_index_for": binary.name,
                        },
                    )
        if "px4.drone" in expectations:
            if "drone" in pixhawk_sources:
                self._register_artifact(
                    "telemetry/px4/drone/parquet",
                    pixhawk_parquet,
                    required=True,
                    producer="DroneTelemetryController/SessionSaveWorker",
                    source="px4.drone",
                    classification="raw",
                    metadata={"partition": {"source_id": "drone"}},
                )
            else:
                self._register_missing(
                    "telemetry/px4/drone/parquet",
                    "active Drone PX4 source has no typed timeline rows",
                    producer="DroneTelemetryController/SessionSaveWorker",
                    source="px4.drone",
                )
        if "target.drone" in expectations:
            candidates = sorted(root.glob("raw/target_drone/**/*.jsonl"))
            raw_target = next((path for path in candidates if path.is_file() and path.stat().st_size), None)
            self._required_file(
                "telemetry/target/raw",
                raw_target,
                producer="TargetDroneController",
                source="target.drone",
                reason="active Target Drone source has no sealed raw artifact",
            )
        for source in sorted(value for value in expectations if value.startswith("ulog.")):
            component = source.removeprefix("ulog.")
            candidates = sorted(root.glob(f"ulog/{component}/**/*.ulg")) + sorted(
                root.glob(f"assets/ulog_{component}/*.ulg")
            )
            artifact = next((path for path in candidates if path.is_file() and path.stat().st_size), None)
            self._required_file(
                f"ulog/{component}",
                artifact,
                producer="ULogController",
                source=source,
                reason=f"active ULog source {component} has no exact .ulg artifact",
            )

    @staticmethod
    def _safe_role_component(value: object) -> str:
        component = re.sub(r"[^A-Za-z0-9._-]+", "-", str(value or "capture")).strip(".-")
        return component or "capture"

    def _capture_side_path(self, records: Sequence[Mapping[str, object]], side: str) -> Path | None:
        root = self._session_root().resolve()

        def session_owned(raw_path: object, *, field: str) -> Path:
            value = Path(str(raw_path or ""))
            if value.is_absolute() and field == "session_relative_path":
                raise ValueError("capture session_relative_path must be relative")
            candidate = (
                value.expanduser().resolve(strict=False)
                if value.is_absolute()
                else (root / value).resolve(strict=False)
            )
            try:
                candidate.relative_to(root)
            except ValueError as exc:
                raise ValueError(f"capture {field} escapes the session root") from exc
            return candidate

        for record in reversed(records):
            stored_files = record.get("stored_files")
            if isinstance(stored_files, Mapping):
                item = stored_files.get(side)
                if isinstance(item, Mapping) and item.get("session_relative_path"):
                    return session_owned(
                        item["session_relative_path"],
                        field="session_relative_path",
                    )
            cameras = record.get("cameras")
            if isinstance(cameras, Mapping):
                item = cameras.get(side)
                if isinstance(item, Mapping) and item.get("ftp_local_path"):
                    # Measurement capture workers stage FTP originals below the
                    # session root.  A recovered JSONL row is evidence, not
                    # authority to import an arbitrary host path.
                    return session_owned(item["ftp_local_path"], field="ftp_local_path")
        return None

    @staticmethod
    def _capture_side_detail(
        records: Sequence[Mapping[str, object]], side: str
    ) -> dict[str, object]:
        for record in reversed(records):
            for container_name in ("stored_files", "cameras"):
                container = record.get(container_name)
                if isinstance(container, Mapping) and isinstance(container.get(side), Mapping):
                    return dict(container[side])
        return {}

    @staticmethod
    def _validated_capture_metadata(path: Path, detail: Mapping[str, object]) -> dict[str, object]:
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        expected_sha = str(detail.get("sha256") or "")
        expected_size = detail.get("size_bytes", detail.get("byte_size"))
        if expected_sha and expected_sha != digest:
            raise ValueError("downloaded image bytes do not match capture SHA-256")
        if expected_size not in (None, "") and int(expected_size) != len(payload):
            raise ValueError("downloaded image size does not match capture metadata")
        image = QtGui.QImage(str(path))
        if image.isNull():
            raise ValueError("downloaded capture is not a decodable image")
        expected_width = detail.get("width_px", detail.get("width"))
        expected_height = detail.get("height_px", detail.get("height"))
        if expected_width not in (None, "") and int(expected_width) != image.width():
            raise ValueError("downloaded image width does not match capture metadata")
        if expected_height not in (None, "") and int(expected_height) != image.height():
            raise ValueError("downloaded image height does not match capture metadata")
        return {
            "sha256": digest,
            "size_bytes": len(payload),
            "width_px": image.width(),
            "height_px": image.height(),
            "camera_serial": str(detail.get("camera_serial") or ""),
            "decode_verified": True,
        }

    @staticmethod
    def _capture_missing_reason(records: Sequence[Mapping[str, object]], side: str) -> str:
        for record in reversed(records):
            cameras = record.get("cameras")
            if isinstance(cameras, Mapping) and isinstance(cameras.get(side), Mapping):
                item = cameras[side]
                return str(
                    item.get("ftp_error")
                    or item.get("error")
                    or f"{side} exact original was not downloaded (state={item.get('state', 'unknown')})"
                )
            if record.get("error"):
                return str(record["error"])
        return f"{side} exact original is missing from the session"

    def _register_capture_records(self, channel: str, raw_records: object) -> None:
        if not isinstance(raw_records, Sequence) or isinstance(raw_records, (str, bytes)):
            return
        grouped: dict[str, list[Mapping[str, object]]] = {}
        for value in raw_records:
            if not isinstance(value, Mapping):
                continue
            capture_id = str(value.get("capture_id") or "unknown")
            grouped.setdefault(capture_id, []).append(value)
        for capture_id, records in grouped.items():
            component = self._safe_role_component(capture_id)
            source_id = f"wiris.{channel}.{component}"
            roles = tuple(f"captures/{channel}/{component}/{side}" for side in ("left", "right"))
            self.activate_measurement_package_source(
                source_id,
                required_roles=roles,
                metadata={"capture_id": capture_id, "channel": channel},
            )
            producer = (
                "WirisPnPPhotoCaptureWorker" if channel == "hr" else "WirisOZPhotoCaptureWorker"
            )
            for side, role in zip(("left", "right"), roles):
                detail = self._capture_side_detail(records, side)
                try:
                    path = self._capture_side_path(records, side)
                except ValueError as exc:
                    self._register_missing(
                        role,
                        str(exc),
                        producer=producer,
                        source=source_id,
                        failed=True,
                        metadata={
                            "capture_id": capture_id,
                            "side": side,
                            "channel": channel,
                        },
                    )
                    continue
                if path is not None and path.is_file():
                    try:
                        verified_metadata = self._validated_capture_metadata(path, detail)
                    except (OSError, TypeError, ValueError) as exc:
                        self._register_missing(
                            role,
                            f"exact {side} image validation failed: {exc}",
                            producer=producer,
                            source=source_id,
                            failed=True,
                            metadata={"capture_id": capture_id, "side": side, "channel": channel},
                        )
                        continue
                self._required_file(
                    role,
                    path,
                    producer=producer,
                    source=source_id,
                    reason=self._capture_missing_reason(records, side),
                    metadata={
                        "capture_id": capture_id,
                        "side": side,
                        "channel": channel,
                        **(
                            verified_metadata
                            if path is not None and path.is_file()
                            else {}
                        ),
                    },
                )

    def _register_session_inventory(self) -> None:
        root = self._session_root()
        inventory_errors: list[str] = []
        self.panel._measurement_package_inventory_errors = inventory_errors

        verify_media = bool(getattr(self.panel, "_measurement_package_verify_media", True))
        if verify_media:
            from Workstation.workstation_core.media_evidence import verify_video_evidence

            backend_widget = getattr(self.panel, "stream_backend_combo", None)
            backend_reader = getattr(backend_widget, "currentData", None)
            backend = str(backend_reader() if callable(backend_reader) else "workstation-camera")
            decoder_widget = getattr(self.panel, "decoder_combo", None)
            decoder_reader = getattr(decoder_widget, "currentData", None)
            decoder = str(decoder_reader() if callable(decoder_reader) else "unknown")
            for side in ("left", "right"):
                path = root / "video" / f"raw_{side}.mp4"
                descriptor = verify_video_evidence(
                    path,
                    session_root=root,
                    role=f"video/{side}/raw",
                    source=f"camera.{side}",
                    recording_backend=backend,
                    encoding_provenance={
                        "decoder": decoder,
                        "contract": (
                            "continuous MP4 plus exact evaluated-pair native PNG; "
                            "MP4 may be encoded pass-through or decoded-frame recording"
                        ),
                    },
                    full_decode=bool(
                        getattr(self.panel, "_measurement_package_full_video_decode", False)
                    ),
                    probe_runner=getattr(
                        self.panel, "_measurement_package_video_probe_runner", None
                    ),
                    decode_runner=getattr(
                        self.panel, "_measurement_package_video_decode_runner", None
                    ),
                )
                self._register_descriptor(descriptor)
        else:
            for side in ("left", "right"):
                self._required_file(
                    f"video/{side}/raw",
                    root / "video" / f"raw_{side}.mp4",
                    producer="CameraPane/YoloStreamWorker",
                    source=f"camera.{side}",
                    reason=f"{side} continuous MP4 is missing or empty",
                )

        session_manifest = root / "measurement.json"
        if bool(getattr(self.panel, "_session_save_finalized", False)):
            self._required_file(
                "session/finalization_manifest",
                session_manifest,
                producer="SessionSaveWorker",
                source="session.persistence",
                reason="finalized measurement.json is missing",
            )
        else:
            reason = "session persistence did not finalize"
            if bool(getattr(self.panel, "_session_save_retry_required", False)):
                reason += "; retryable spool was preserved"
            self._register_missing(
                "session/finalization_manifest",
                reason,
                producer="SessionSaveWorker",
                source="session.persistence",
                failed=True,
            )

        raw_pair_path = self._actual_generation_file("measurement_pairs.parquet")
        self._required_file(
            "measurements/pairs_parquet",
            raw_pair_path,
            producer="SessionSaveWorker",
            source="measurement",
            reason="authoritative measurement-pair Parquet is missing",
            allow_empty=True,
        )
        self._register_measurement_pair_evidence(raw_pair_path)
        self._register_runtime_source_inventory()

        self._register_capture_records(
            "hr", getattr(self.panel, "pnp_photo_capture_records", ())
        )
        self._register_capture_records(
            "oz", getattr(self.panel, "oz_photo_capture_records", ())
        )

        for path in sorted(root.rglob("*")):
            if not path.is_file() or path.is_symlink():
                continue
            relative = path.relative_to(root).as_posix()
            if path.name in _PACKAGE_METADATA_FILES or relative.startswith("revisions/"):
                continue
            registered = getattr(self.panel, "_measurement_package_registered_paths", set())
            if any(item[1] == relative for item in registered):
                continue
            producer, source = self._producer_source(relative)
            try:
                self._register_artifact(
                    f"session_file/{relative}",
                    path,
                    required=False,
                    producer=producer,
                    source=source,
                )
            except Exception as exc:
                inventory_errors.append(f"{relative}: {exc}")
        self.panel._measurement_package_inventory_errors = inventory_errors

    # ------------------------------------------------------------------
    # Revision and verification

    def begin_next_measurement(
        self,
        metadata: Mapping[str, object] | None = None,
    ) -> bool:
        if not self._hierarchy_enabled():
            self._set_state(
                _state_text(getattr(self.panel, "_measurement_package_state", "IDLE")),
                "Multiple measurements require the hierarchical session layout",
            )
            return False
        state = _state_text(getattr(self.panel, "_measurement_package_state", "IDLE"))
        if state not in _SEALED_STATES | {"IDLE"}:
            self._set_state(state, "Seal the current measurement before starting another")
            return False
        active = getattr(self.panel, "_session_recording_active", None)
        if callable(active) and active():
            self._set_state(state, "Stop current recording producers before starting another measurement")
            return False
        if getattr(self.panel, "_session_save_thread", None) is not None:
            self._set_state(state, "Wait for current measurement persistence to finish")
            return False
        if bool(getattr(self.panel, "_session_save_retry_required", False)):
            self._set_state(state, "Resolve the current measurement save retry before starting another")
            return False
        if bool(getattr(self.panel, "_measurement_package_finalize_requested", False)):
            self._set_state(state, "Wait for current measurement finalization")
            return False
        if any(
            getattr(self.panel, name, None) is not None
            for name in ("pnp_photo_capture_thread", "oz_photo_capture_thread")
        ):
            self._set_state(state, "Wait for current measurement capture workers")
            return False
        session = self.session
        if session is None:
            self._set_state("RECOVERABLE", "Measurement session owner is unavailable")
            return False
        if str(getattr(session, "state", "")) != "OPEN":
            self._set_state(state, "The measurement session is already closed")
            return False
        method_config = self._measurement_method_config_for_new_package()
        try:
            self._sync_measurement_session()
            package = session.begin_measurement(
                metadata={
                    "initiator": "workstation_explicit_next_measurement",
                    "recording_mode": "explicit_measurement_package",
                    **dict(metadata or {}),
                    **method_config,
                }
            )
            measurement_id = str(package.snapshot()["metadata"]["measurement_id"])
        except Exception as exc:
            self._set_state(state, "Could not create the next measurement", str(exc))
            return False
        self._initialize_measurement_package(
            package,
            session_id=str(getattr(session, "session_id", "")),
            measurement_id=measurement_id,
            apply_run_identity=False,
        )
        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            logger(
                f"[패키지] New Measurement: {session.session_id}/{measurement_id}"
            )
        return True

    def begin_measurement(
        self,
        metadata: Mapping[str, object] | None = None,
    ) -> bool:
        return self.begin_next_measurement(metadata)

    def close_measurement_session(self) -> bool:
        if not self._hierarchy_enabled():
            self._set_state(
                _state_text(getattr(self.panel, "_measurement_package_state", "IDLE")),
                "Session close requires the hierarchical session layout",
            )
            return False
        state = _state_text(getattr(self.panel, "_measurement_package_state", "IDLE"))
        if state not in _SEALED_STATES | {"IDLE"}:
            detection_started = bool(
                getattr(self.panel, "_measurement_package_detection_started", False)
            )
            detection_completed = bool(
                getattr(self.panel, "_measurement_package_detection_completed", False)
            )
            if state == "COLLECTING" and detection_started and not detection_completed:
                self._set_state(
                    state,
                    "Stop Detection before closing; Session will close automatically after save and seal",
                )
                return False
            if (
                state in {"COLLECTING", "RECOVERABLE"}
                and not detection_started
                and not detection_completed
            ):
                # A lifecycle-only child may be closed without inventing a
                # Detection interval.  This also covers an empty child moved
                # to RECOVERABLE when a new app run reopens its parent
                # Session.  Preserve it as an explicit empty measurement,
                # run the normal persistence/inventory gates, and let
                # completeness resolve to COMPLETE or INCOMPLETE.
                self.panel._measurement_package_detection_completed = True
                package = self.package
                transition = getattr(package, "mark_post_capture", None)
                if callable(transition):
                    transition()
                else:
                    transition = getattr(package, "transition", None)
                    if callable(transition):
                        _call_supported(transition, "POST_CAPTURE")
                self._sync_measurement_session()
                self._set_state(
                    "POST_CAPTURE",
                    "Empty measurement will be saved and sealed before Session close",
                )
                state = "POST_CAPTURE"
                detection_completed = True
            if state in {"POST_CAPTURE", "VERIFYING", "RECOVERABLE"} and detection_completed:
                self.panel._measurement_session_close_requested = True
                if bool(getattr(self.panel, "_measurement_package_finalize_requested", False)):
                    return self._continue_measurement_package_finalization()
                return self.finalize_measurement_package()
            self._set_state(state, "Measurement could not enter the automatic close workflow")
            return False
        session = self.session
        if session is None:
            self._set_state("RECOVERABLE", "Measurement session owner is unavailable")
            return False
        try:
            self._sync_measurement_session()
            resume_close = (
                str(getattr(session, "state", "") or "") == "CLOSING"
                or Path(getattr(session, "manifest_path", "")).is_file()
            )
            if not resume_close:
                try:
                    from Workstation.workstation_core.recording_session import (
                        rebuild_session_review,
                    )

                    rebuild_session_review(Path(session.root), session.session_id)
                except Exception as review_exc:
                    logger = getattr(self.panel, "_log", None)
                    if callable(logger):
                        logger(
                            "[패키지] verified Parquet은 유지하고 review.xlsx 생성 실패를 "
                            f"기록합니다: {review_exc}"
                        )
            manifest = session.seal()
        except Exception as exc:
            self.panel._measurement_session_state = "RECOVERABLE"
            self._set_state(state, "Session sealing failed", str(exc))
            return False
        self.panel._measurement_session_state = str(manifest.get("state") or "SEALED")
        self._set_state(state, f"Session {session.session_id} sealed")
        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            logger(f"[패키지] Session Close: {session.session_id}")
        return True

    def begin_measurement_revision(self) -> bool:
        state = _state_text(getattr(self.panel, "_measurement_package_state", "IDLE"))
        session = self.session
        if self._hierarchy_enabled() and str(getattr(session, "state", "")) != "OPEN":
            self._set_state(state, "A closed session cannot accept measurement revisions")
            return False
        if state not in _SEALED_STATES | {"INTEGRITY_FAILED"}:
            self._set_state(state, "Create Revision requires a sealed package")
            return False
        package = self.package
        creator = getattr(package, "create_revision", None) or getattr(
            package, "begin_revision", None
        )
        if not callable(creator):
            self._set_state(state, "Measurement package does not support revisions")
            return False
        try:
            revised = creator()
        except Exception as exc:
            self._set_state(state, "Could not create package revision", str(exc))
            return False
        if revised is not None:
            self.panel._measurement_package = revised
            self.panel._measurement_method_config_token = None
        self._sync_measurement_session()
        core = self._core_snapshot()
        self.panel._measurement_package_revision = _revision_number(
            core.get("revision_number", core.get("revision")),
            getattr(self.panel, "_measurement_package_revision", 0) + 1,
        )
        self.panel._measurement_package_registered_paths = set()
        self.panel._measurement_package_inventory_errors = []
        self.panel._measurement_package_finalize_requested = False
        self.panel._measurement_package_calibration_collected = False
        self.panel._measurement_package_video_observations = None
        self.panel._measurement_package_rtcm_drain_requested_token = ""
        self.panel._measurement_package_rtcm_drain_result = None
        # Revisions append post-session evidence; they never grant another
        # Detection interval for the same measurement session.
        self._set_state("POST_CAPTURE", "Revision is open for additional evidence")
        return True

    def verify_measurement_package(self) -> dict[str, object]:
        package = self.package
        verifier = getattr(package, "verify", None)
        try:
            if callable(verifier):
                result = verifier()
            else:
                from Workstation.workstation_core.measurement_package import verify_package

                result = verify_package(self._session_root())
        except Exception as exc:
            self._set_state("INTEGRITY_FAILED", "Package verification failed", str(exc))
            return {"valid": False, "state": "INTEGRITY_FAILED", "error": str(exc)}
        if isinstance(result, Mapping):
            mapped = dict(result)
        else:
            mapper = getattr(result, "to_mapping", None)
            mapped = dict(mapper()) if callable(mapper) else dict(getattr(result, "__dict__", {}))
        valid = bool(mapped.get("valid", mapped.get("ok", False)))
        if not valid:
            self._set_state(
                "INTEGRITY_FAILED",
                "Package files no longer match the sealed manifest",
                str(mapped.get("error") or mapped.get("detail") or "integrity mismatch"),
            )
        else:
            sealed = _state_text(self._core_snapshot().get("state"), "SEALED_COMPLETE")
            self._set_state(
                sealed if sealed in _SEALED_STATES else "SEALED_COMPLETE",
                "Package integrity verified",
            )
        return dict(_json_copy(mapped))

    def verify_session(self) -> dict[str, object]:
        session = self.session
        if session is None:
            return {
                "valid": False,
                "state": "INTEGRITY_FAILED",
                "error": "measurement session owner is unavailable",
            }
        verifier = getattr(session, "verify", None)
        try:
            result = verifier() if callable(verifier) else None
        except Exception as exc:
            return {"valid": False, "state": "INTEGRITY_FAILED", "error": str(exc)}
        mapper = getattr(result, "to_mapping", None)
        return dict(mapper()) if callable(mapper) else dict(getattr(result, "__dict__", {}))

    # Stable plan/API aliases.
    def begin_session(self, *args: object, **kwargs: object) -> bool:
        return self.begin_measurement_session(*args, **kwargs)

    def end_measurement(self) -> bool:
        return self.complete_measurement_detection()

    def seal_session(self) -> bool:
        return self.finalize_measurement_package()

    def close_session(self) -> bool:
        return self.close_measurement_session()

    def create_revision(self) -> bool:
        return self.begin_measurement_revision()

    def verify_package(self) -> dict[str, object]:
        return self.verify_measurement_package()
