from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtWidgets

from Workstation.evidence import EvidenceAdapter
from Workstation.status_model import StatusModel, StatusSeverity
from Workstation.workspaces import MeasurementWorkspace, RecordingWorkspace, SessionWorkspace
from Workstation.workstation_core.measurement_methods import (
    MEASUREMENT_METHOD_IDS,
    MEASUREMENT_METHOD_LABELS,
)


class _PackagePanel:
    def __init__(self) -> None:
        self.settings = SimpleNamespace()
        self.distance_history: list[object] = []
        self.calls: list[str] = []
        self.package: dict[str, object] = {
            "schema": "workstation-measurement-package-v1",
            "schema_version": 1,
            "state": "IDLE",
            "session_id": "",
            "revision": "r0001",
            "detection_started": False,
            "detection_completed": False,
            "source_expectations": [],
            "artifacts": [],
            "evidence_completeness": {
                "status": "PENDING",
                "complete": False,
                "missing_required": [],
            },
        }

    def measurement_package_snapshot(self) -> dict[str, object]:
        return dict(self.package)

    def begin_measurement_session(self) -> bool:
        self.calls.append("begin")
        self.package.update(
            state="COLLECTING",
            session_id="20260821T120000.000000Z",
            detection_started=False,
            detection_completed=False,
        )
        return True

    def finalize_measurement_package(self) -> bool:
        self.calls.append("finalize")
        self.package.update(state="VERIFYING", detail="waiting for producer drain")
        return False


class _PackageBackend:
    def __init__(self, package: dict[str, object]) -> None:
        self.panel = SimpleNamespace(settings=SimpleNamespace())
        self.package = package
        self.calls: list[str] = []

    def measurement_package_snapshot(self) -> dict[str, object]:
        return dict(self.package)

    def verify_measurement_package(self) -> dict[str, object]:
        self.calls.append("verify")
        return {
            "state": "INTEGRITY_FAILED",
            "issues": [
                {
                    "code": "hash_mismatch",
                    "message": "raw pair changed",
                    "path": "revisions/r0007/raw/pairs.jsonl",
                    "revision": "r0007",
                }
            ],
        }

    def begin_measurement_revision(self) -> bool:
        self.calls.append("revision")
        self.package.update(
            state="POST_CAPTURE",
            revision="r0008",
            revision_number=8,
            evidence_completeness={
                "status": "PENDING",
                "complete": False,
                "missing_required": [],
            },
        )
        return True


class _HierarchicalPackageBackend:
    def __init__(self) -> None:
        self.panel = SimpleNamespace(settings=SimpleNamespace())
        self.calls: list[str] = []
        self.session: dict[str, object] = {
            "state": "IDLE",
            "session_id": "",
            "active_measurement_id": None,
            "measurements": [],
        }
        self.package: dict[str, object] = {
            "state": "IDLE",
            "session_state": "IDLE",
            "session_id": "",
            "measurement_id": "",
            "revision": "",
            "detection_started": False,
            "detection_completed": False,
            "artifacts": [],
            "evidence_completeness": {"status": "PENDING", "complete": False},
        }

    def measurement_session_snapshot(self) -> dict[str, object]:
        return dict(self.session)

    def measurement_package_snapshot(self) -> dict[str, object]:
        return dict(self.package)

    def measurement_methods_snapshot(self) -> dict[str, object]:
        active = {
            "method_id": "pixhawk_rtk_live",
            "label": "Live Pixhawk + RTK",
            "instance_id": "live:pixhawk-rtk",
            "ready": True,
            "reasons": [],
            "required_inputs": ["fresh Pixhawk attitude", "GNSS baseline"],
            "input_values": {
                "uses_live_pixhawk": True,
                "uses_live_gnss": True,
                "live_attitude_rpy_deg": {
                    "left": {
                        "roll_deg": 1.0,
                        "pitch_deg": 2.0,
                        "yaw_deg": 3.0,
                        "age_s": 0.1,
                    },
                    "right": {
                        "roll_deg": 4.0,
                        "pitch_deg": 5.0,
                        "yaw_deg": 6.0,
                        "age_s": 0.2,
                    },
                },
                "live_baseline_ned_m": {
                    "north_m": 0.1,
                    "east_m": 1.2,
                    "down_m": -0.2,
                },
                "candidate_pose_virtual_frd_to_world_ned": {},
                "stereo_extrinsics_opencv_rdf": None,
            },
        }
        catalog = [active]
        catalog.extend(
            {
                "method_id": method_id,
                "label": MEASUREMENT_METHOD_LABELS[method_id],
                "instance_id": "",
                "ready": False,
                "reasons": [f"no {MEASUREMENT_METHOD_LABELS[method_id]} candidate"],
                "required_inputs": [f"{MEASUREMENT_METHOD_LABELS[method_id]} input"],
                "input_values": {},
            }
            for method_id in MEASUREMENT_METHOD_IDS
            if method_id != "pixhawk_rtk_live"
        )
        return {
            "preferred_method_id": "pixhawk_rtk_live",
            "selected_method_id": "pixhawk_rtk_live",
            "selected_instance_id": "live:pixhawk-rtk",
            "selection_reason": "preferred",
            "method_capabilities": [active],
            "method_catalog": catalog,
        }

    def begin_measurement_session(self) -> bool:
        self.calls.append("session.begin")
        self.session.update(state="OPEN", session_id="session-001")
        self.package.update(session_state="OPEN", session_id="session-001")
        return True

    def begin_measurement(self) -> bool:
        self.calls.append("measurement.begin")
        measurement_id = f"m{len(self.session['measurements']) + 1:06d}"
        self.session["active_measurement_id"] = measurement_id
        self.session["measurements"] = [
            *list(self.session["measurements"]),
            {"measurement_id": measurement_id, "state": "COLLECTING"},
        ]
        self.package.update(
            state="COLLECTING",
            session_state="OPEN",
            session_id="session-001",
            measurement_id=measurement_id,
            revision="r0001",
            detection_started=False,
            detection_completed=False,
        )
        return True

    def finalize_measurement_package(self) -> bool:
        self.calls.append("measurement.seal")
        self.package.update(state="SEALED_COMPLETE", detection_completed=True)
        rows = list(self.session["measurements"])
        rows[-1] = {**rows[-1], "state": "SEALED_COMPLETE"}
        self.session.update(active_measurement_id=None, measurements=rows)
        return True

    def close_measurement_session(self) -> bool:
        self.calls.append("session.close")
        self.session["state"] = "SEALED"
        self.package["session_state"] = "SEALED"
        return True


def _model_row(model: object, row: int) -> tuple[str, ...]:
    return tuple(model.item(row, column).text() for column in range(model.columnCount()))


def test_measurement_package_controls_follow_explicit_lifecycle() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    panel = _PackagePanel()
    commands: list[str] = []
    workspace = MeasurementWorkspace(
        SimpleNamespace(panel=panel),
        StatusModel(),
        lambda command: commands.append(command) or True,
    )

    assert workspace.start_session_button.objectName() == "measurementPackageStart"
    assert workspace.start_session_button.isEnabled()
    assert not workspace.control_buttons["detection.toggle"].isEnabled()
    assert not workspace.finalize_session_button.isEnabled()

    workspace.start_session_button.click()
    assert panel.calls == ["begin"]
    assert "COLLECTING" in workspace.session_status_label.text()
    assert "Pre evidence" in workspace.session_step_label.text()
    assert workspace.control_buttons["detection.toggle"].isEnabled()
    workspace.control_buttons["detection.toggle"].click()
    assert commands == ["detection.toggle"]

    panel.package.update(
        state="POST_CAPTURE",
        detection_started=True,
        detection_completed=True,
        artifacts=[
            {
                "artifact_id": "pair-jsonl",
                "relative_path": "revisions/r0001/raw/pairs.jsonl",
                "required": True,
                "status": "present",
            },
            {
                "artifact_id": "left-video",
                "relative_path": "revisions/r0001/video/left.mp4",
                "required": True,
                "status": "missing",
            },
        ],
        evidence_completeness={
            "status": "INCOMPLETE",
            "complete": False,
            "missing_required": ["left-video"],
        },
    )
    workspace.refresh({})
    assert not workspace.control_buttons["detection.toggle"].isEnabled()
    assert workspace.finalize_session_button.isEnabled()
    assert workspace.evidence_summary_label.text() == (
        "Evidence: INCOMPLETE · present=1, missing=1"
    )

    workspace.finalize_session_button.click()
    assert panel.calls == ["begin", "finalize"]
    assert "VERIFYING" in workspace.session_status_label.text()
    assert "draining producers" in workspace.session_step_label.text()
    assert not workspace.finalize_session_button.isEnabled()

    workspace.deleteLater()
    app.processEvents()


def test_measurement_without_package_facade_preserves_legacy_detection_control() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    panel = SimpleNamespace(settings=SimpleNamespace(), distance_history=[])
    workspace = MeasurementWorkspace(
        SimpleNamespace(panel=panel),
        StatusModel(),
        lambda _command: True,
    )

    workspace.refresh({})
    assert not workspace.start_session_button.isEnabled()
    assert not workspace.finalize_session_button.isEnabled()
    assert workspace.control_buttons["detection.toggle"].isEnabled()
    assert "automatic" in workspace.session_status_label.text()

    workspace.deleteLater()
    app.processEvents()


def test_hierarchical_controls_drive_two_measurements_then_close() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    workspace = MeasurementWorkspace(backend, StatusModel(), lambda _command: True)

    workspace.refresh({})
    assert workspace.start_session_button.isEnabled()
    assert not workspace.new_measurement_button.isEnabled()

    workspace.start_session_button.click()
    assert "Session: OPEN" in workspace.session_status_label.text()
    assert workspace.measurement_status_label.text() == "Measurement: NONE"
    assert workspace.new_measurement_button.isEnabled()
    assert workspace.close_session_button.isEnabled()

    workspace.new_measurement_button.click()
    assert "m000001" in workspace.measurement_status_label.text()
    assert workspace.control_buttons["detection.toggle"].isEnabled()
    backend.package.update(state="POST_CAPTURE", detection_started=True, detection_completed=True)
    workspace.refresh({})
    workspace.finalize_session_button.click()
    assert workspace.new_measurement_button.isEnabled()

    workspace.new_measurement_button.click()
    assert "m000002" in workspace.measurement_status_label.text()
    backend.package.update(state="POST_CAPTURE", detection_started=True, detection_completed=True)
    workspace.refresh({})
    workspace.finalize_session_button.click()
    workspace.close_session_button.click()

    assert backend.calls == [
        "session.begin",
        "measurement.begin",
        "measurement.seal",
        "measurement.begin",
        "measurement.seal",
        "session.close",
    ]
    assert "Session: SEALED" in workspace.session_status_label.text()
    workspace.deleteLater()
    app.processEvents()


def test_session_workspace_promotes_daily_lifecycle_and_observes_method_inputs() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    model = StatusModel()
    for key, label in (
        ("camera.left", "Left camera"),
        ("camera.right", "Right camera"),
        ("synchronization", "Time sync"),
    ):
        model.set_status(
            key,
            label,
            "READY",
            StatusSeverity.OK,
            detail="verified evidence",
            source="test fixture",
        )
    workspace = SessionWorkspace(backend, model, lambda: None)

    assert workspace.start_session_button.isEnabled()
    assert not workspace.new_measurement_button.isEnabled()
    workspace.start_session_button.click()
    workspace.new_measurement_button.click()
    workspace.refresh(model.snapshot())

    assert workspace.session_status_label.text().startswith("Session: OPEN · session-001")
    assert "m000001" in workspace.measurement_status_label.text()
    assert workspace.readiness_summary_label.text() == (
        "Evidence collection: READY · Valid measurement: READY"
    )
    assert workspace.readiness_model.rowCount() == 0
    assert workspace.readiness_table.isHidden()
    assert workspace.method_inputs_model.rowCount() == len(MEASUREMENT_METHOD_IDS)
    method_row = _model_row(workspace.method_inputs_model, 0)
    assert method_row[:2] == ("Live Pixhawk + RTK", "EFFECTIVE")
    assert "L R/P/Y=1.00/2.00/3.00°" in workspace.method_detail_values.text()
    assert "GNSS P1→P2 N/E/D=0.100/1.200/-0.200m" in workspace.method_detail_values.text()
    mountain_row = next(
        _model_row(workspace.method_inputs_model, row)
        for row in range(workspace.method_inputs_model.rowCount())
        if _model_row(workspace.method_inputs_model, row)[0] == "Mountain + GNSS"
    )
    assert mountain_row[1] == "BLOCKED"
    assert "no Mountain + GNSS candidate" in mountain_row[2]
    mountain_index = next(
        row
        for row in range(workspace.method_inputs_model.rowCount())
        if _model_row(workspace.method_inputs_model, row)[0] == "Mountain + GNSS"
    )
    workspace.method_inputs_table.setCurrentIndex(
        workspace.method_inputs_model.index(mountain_index, 0)
    )
    assert workspace.method_detail_title.text() == "Mountain + GNSS · BLOCKED"
    assert "Required: Mountain + GNSS input" in workspace.method_detail_values.text()
    assert workspace.next_action_label.text() == (
        "Next: Review blockers, then open Measurement"
    )
    assert workspace.open_measurement_button.isDefault()
    assert workspace.measurements_model.rowCount() == 1
    assert _model_row(workspace.measurements_model, 0)[1:3] == (
        "m000001",
        "COLLECTING",
    )

    workspace.deleteLater()
    app.processEvents()


def test_session_workspace_readiness_table_contains_only_collection_blockers() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    backend.begin_measurement_session()
    backend.begin_measurement()
    model = StatusModel()
    model.set_status(
        "camera.left",
        "Left camera",
        "READY",
        StatusSeverity.OK,
        detail="verified",
    )
    workspace = SessionWorkspace(backend, model)

    workspace.refresh(model.snapshot())

    assert workspace.readiness_summary_label.text() == (
        "Evidence collection: BLOCKED · Valid measurement: BLOCKED"
    )
    blockers = [
        _model_row(workspace.readiness_model, row)
        for row in range(workspace.readiness_model.rowCount())
    ]
    assert blockers == [
        ("COLLECTION", "Right camera: NO DATA"),
        ("COLLECTION", "Time sync: NO DATA"),
    ]
    assert all("Left camera" not in detail for _scope, detail in blockers)
    assert all("Measurement method" not in detail for _scope, detail in blockers)

    workspace.deleteLater()
    app.processEvents()


def test_session_workspace_reports_valid_measurement_blocked_without_ready_method() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    backend.begin_measurement_session()
    backend.begin_measurement()
    backend.measurement_methods_snapshot = lambda: {
        "preferred_method_id": "checkerboard_stereo",
        "selected_method_id": "",
        "selection_reason": "no_ready_method",
        "method_capabilities": [
            {
                "method_id": "checkerboard_stereo",
                "label": "Checkerboard stereo R,t",
                "instance_id": "profile:missing",
                "ready": False,
                "reasons": ["exact operating K,D missing", "checkerboard stereo R,t missing"],
                "input_values": {},
            }
        ],
    }
    model = StatusModel()
    for key in ("camera.left", "camera.right", "synchronization"):
        model.set_status(key, key, "READY", StatusSeverity.OK, detail="ready")
    workspace = SessionWorkspace(backend, model)

    workspace.refresh(model.snapshot())

    assert workspace.readiness_summary_label.text() == (
        "Evidence collection: READY · Valid measurement: BLOCKED"
    )
    assert workspace.readiness_model.rowCount() == 1
    assert _model_row(workspace.readiness_model, 0)[0] == "VALIDITY"
    assert "no ready method" in _model_row(workspace.readiness_model, 0)[1]
    assert "exact operating K,D missing" in _model_row(workspace.method_inputs_model, 0)[2]
    assert "blocker=exact operating K,D missing" in workspace.method_detail_meta.text()

    workspace.deleteLater()
    app.processEvents()


def test_saved_measurement_selection_is_not_overwritten_by_live_sibling(
    monkeypatch,
    tmp_path,
) -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    session_id = "session-001"
    records = tuple(
        {
            "session_id": session_id,
            "measurement_id": measurement_id,
            "catalog_key": f"{session_id}::{measurement_id}",
            "layout": "nested-v2",
            "path": str(tmp_path / session_id / "measurements" / measurement_id),
            "status": state,
            "state": state,
            "revision": "r0001",
            "artifacts": (),
            "artifact_details": (),
            "source_expectations": (),
        }
        for measurement_id, state in (
            ("m000001", "SEALED_COMPLETE"),
            ("m000002", "COLLECTING"),
        )
    )
    monkeypatch.setattr(EvidenceAdapter, "session_catalog", lambda _self: records)
    backend = _PackageBackend(
        {
            "session_id": session_id,
            "measurement_id": "m000002",
            "state": "COLLECTING",
            "artifacts": [],
        }
    )
    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)
    first_key = f"{session_id}::m000001"
    first_index = workspace.session_selector.findData(first_key)
    assert first_index >= 0
    workspace.session_selector.setCurrentIndex(first_index)

    selected = workspace._selected_package_snapshot()

    assert workspace.session_selector.count() == 2
    assert selected["measurement_id"] == "m000001"
    assert selected["state"] == "SEALED_COMPLETE"
    assert selected["_actions_available"] is False
    workspace.deleteLater()
    app.processEvents()


def test_session_details_show_package_inventory_and_revision_actions(
    monkeypatch,
    tmp_path,
) -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    session_id = "20260821T120000.000000Z"
    package = {
        "schema": "workstation-measurement-package-v1",
        "schema_version": 1,
        "state": "SEALED_INCOMPLETE",
        "session_id": session_id,
        "revision": "r0007",
        "revision_number": 7,
        "source_expectations": [
            {
                "source_id": "camera.left",
                "status": "active",
                "active": True,
                "required_roles": ["continuous_video", "pair_frame"],
                "metadata": {
                    "active_from": "pair-0001",
                    "active_until": "pair-0042",
                },
            },
            {
                "source_id": "target_drone",
                "status": "not_applicable",
                "active": False,
                "required_roles": [],
                "metadata": {"reason": "not configured"},
            },
        ],
        "artifacts": [
            {
                "artifact_id": "raw-pairs",
                "role": "raw_pair_rows",
                "classification": "raw",
                "relative_path": "revisions/r0007/raw/pairs.jsonl",
                "producer": "measurement-controller",
                "source": "stereo-pair",
                "required": True,
                "status": "present",
                "media_type": "application/x-ndjson",
                "schema": "measurement-pair-v1",
                "size_bytes": 321,
                "sha256": "a" * 64,
                "metadata": {},
            },
            {
                "artifact_id": "left-video",
                "role": "continuous_video",
                "classification": "raw",
                "relative_path": "revisions/r0007/video/left.mp4",
                "producer": "recording-controller",
                "source": "camera.left",
                "required": True,
                "status": "failed",
                "media_type": "video/mp4",
                "schema": "",
                "size_bytes": 0,
                "sha256": "",
                "metadata": {"failure_reason": "encoder flush timed out"},
            },
        ],
        "evidence_completeness": {
            "status": "INCOMPLETE",
            "complete": False,
            "missing_required": ["left-video"],
        },
        "integrity_status": "VERIFIED",
        "hash_verification": {
            "manifest_status": "verified",
            "artifact_hashes_declared": 2,
            "artifacts_not_run": 0,
        },
    }
    catalog_record = {
        **package,
        "path": str(tmp_path / session_id),
        "status": "SEALED_INCOMPLETE",
        "completeness_state": "INCOMPLETE",
        "evidence_complete": False,
        "missing_reasons": ("left-video",),
        "failed_reasons": ("encoder flush timed out",),
        "artifact_details": tuple(package["artifacts"]),
        "artifacts": tuple(
            str(row["relative_path"])
            for row in package["artifacts"]
        ),
    }
    monkeypatch.setattr(EvidenceAdapter, "session_catalog", lambda _self: (catalog_record,))
    monkeypatch.setattr(
        EvidenceAdapter,
        "session_artifact_rows",
        lambda _self, _session_id, _artifact: {
            "columns": (),
            "rows": (),
            "error": "binary or unavailable in focused UI test",
        },
    )
    backend = _PackageBackend(package)
    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)

    assert workspace.package_state_label.text() == (
        f"State: SEALED_INCOMPLETE · {session_id}"
    )
    assert workspace.package_completeness_label.text() == (
        "Evidence: INCOMPLETE · missing=1, failed=1"
    )
    assert workspace.package_revision_label.text() == "Revision: r0007"
    assert workspace.package_verification_label.text() == (
        "Verification: VERIFIED · manifest=verified, artifacts=2"
    )
    assert workspace.package_source_model.rowCount() == 2
    assert _model_row(workspace.package_source_model, 0) == (
        "camera.left",
        "continuous_video, pair_frame",
        "active · pair-0001 → pair-0042",
        "--",
    )
    assert workspace.package_artifact_model.rowCount() == 2
    failed_row = _model_row(workspace.package_artifact_model, 1)
    assert failed_row == (
        "revisions/r0007/video/left.mp4",
        "continuous_video",
        "yes",
        "failed",
        "0",
        "--",
        "r0007",
        "encoder flush timed out",
    )
    assert workspace._selected_artifact_name() == (
        "revisions/r0007/raw/pairs.jsonl"
    )
    assert workspace._selected_artifact_path() == (
        Path(catalog_record["path"]) / "revisions/r0007/raw/pairs.jsonl"
    )
    assert workspace.verify_package_button.isEnabled()
    assert workspace.create_revision_button.isEnabled()

    workspace.verify_package_button.click()
    assert backend.calls == ["verify"]
    assert workspace.package_verification_label.text() == (
        "Verification: INTEGRITY_FAILED · issues=1"
    )
    workspace.create_revision_button.click()
    assert backend.calls == ["verify", "revision"]
    assert workspace.package_state_label.text() == (
        f"State: POST_CAPTURE · {session_id}"
    )
    assert workspace.package_revision_label.text() == "Revision: r0008"
    assert not workspace.create_revision_button.isEnabled()

    workspace.deleteLater()
    app.processEvents()


def test_session_details_keep_selected_sealed_record_when_live_owner_is_idle(
    monkeypatch,
    tmp_path,
) -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    record = {
        "session_id": "sealed-session",
        "path": str(tmp_path / "sealed-session"),
        "status": "SEALED_COMPLETE",
        "lifecycle_state": "SEALED_COMPLETE",
        "completeness_state": "COMPLETE",
        "evidence_complete": True,
        "revision": "r0003",
        "artifacts": (),
        "artifact_details": (),
        "source_expectations": (),
        "integrity_status": "VERIFIED",
    }
    monkeypatch.setattr(EvidenceAdapter, "session_catalog", lambda _self: (record,))
    backend = _PackageBackend(
        {
            "state": "IDLE",
            "session_id": "",
            "revision": "",
            "source_expectations": [],
            "artifacts": [],
        }
    )
    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)

    assert workspace.package_state_label.text() == "State: SEALED_COMPLETE · sealed-session"
    assert workspace.package_revision_label.text() == "Revision: r0003"
    assert not workspace.verify_package_button.isEnabled()
    assert not workspace.create_revision_button.isEnabled()

    workspace.deleteLater()
    app.processEvents()


class _RunSetupBackend:
    def __init__(self) -> None:
        self.panel = SimpleNamespace(
            settings=SimpleNamespace(),
            pnp_photo_capture_records=[],
            pnp_photo_capture_thread=None,
            oz_photo_capture_records=[],
            oz_photo_capture_thread=None,
        )
        self.calls: list[tuple[str, object]] = []
        self.snapshot: dict[str, object] = {
            "state": "PENDING",
            "locked": False,
            "source": "owner has no applied setup",
            "experiment_id": "owner-old-experiment",
            "run_id": "owner-old-run",
            "condition_id": "owner-old-condition",
            "target_platform": "drone",
            "condition": {},
        }

    def begin_measurement_session(self) -> bool:
        self.calls.append(("begin", None))
        return True

    def run_setup_snapshot(self) -> dict[str, object]:
        return dict(self.snapshot)

    def apply_run_setup(self, payload: dict[str, object]) -> dict[str, object]:
        values = dict(payload)
        self.calls.append(("apply", values))
        self.snapshot = {
            **values,
            "state": "APPLIED",
            "locked": False,
            "source": "operator draft accepted",
        }
        return dict(self.snapshot)


def test_run_setup_draft_remains_editable_and_survives_unlocked_owner_poll(
    monkeypatch,
) -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    monkeypatch.setattr(EvidenceAdapter, "session_catalog", lambda _self: ())
    backend = _RunSetupBackend()
    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)

    # Merely exposing Begin Session is not evidence and must not lock identity.
    assert all(widget.isEnabled() for widget in workspace.run_setup_widgets.values())
    workspace.run_setup_widgets["experiment_id"].setText("EXP-visible")
    workspace.run_setup_widgets["run_id"].setText("RUN-visible")
    workspace.run_setup_widgets["condition_id"].setText("COND-visible")
    workspace.run_platform_combo.setCurrentIndex(
        workspace.run_platform_combo.findData("uam")
    )
    workspace.run_camera_serial_edits["left"].setText("LEFT-SERIAL")
    workspace.run_camera_serial_edits["right"].setText("RIGHT-SERIAL")
    workspace.run_camera_stabilization_combos["left"].setCurrentIndex(
        workspace.run_camera_stabilization_combos["left"].findData("disabled")
    )
    workspace.run_camera_stabilization_combos["right"].setCurrentIndex(
        workspace.run_camera_stabilization_combos["right"].findData("enabled")
    )

    workspace.refresh_run_setup_from_owner()
    assert workspace.run_setup_widgets["experiment_id"].text() == "EXP-visible"
    assert workspace.run_setup_widgets["run_id"].text() == "RUN-visible"
    assert workspace.run_setup_widgets["condition_id"].text() == "COND-visible"
    assert workspace.run_platform_combo.currentData() == "uam"
    assert workspace.run_setup_state_label.text().startswith("DRAFT")

    workspace.run_setup_apply_button.click()
    assert backend.calls == [
        (
            "apply",
            {
                "experiment_id": "EXP-visible",
                "run_id": "RUN-visible",
                "condition_id": "COND-visible",
                "target_platform": "uam",
                "condition": {
                    "left_serial": "LEFT-SERIAL",
                    "right_serial": "RIGHT-SERIAL",
                    "left_stabilization_state": "disabled",
                    "right_stabilization_state": "enabled",
                    "camera_identity_source": "operator_verified_run_setup",
                },
            },
        )
    ]
    assert workspace.run_setup_state_label.text() == "APPLIED · operator draft accepted"
    assert all(widget.isEnabled() for widget in workspace.run_setup_widgets.values())

    backend.snapshot.update(state="LOCKED", locked=True, source="first evidence")
    workspace.refresh_run_setup_from_owner()
    assert not any(widget.isEnabled() for widget in workspace.run_setup_widgets.values())
    assert workspace.run_setup_state_label.text() == "LOCKED · first evidence"

    workspace.deleteLater()
    app.processEvents()


def test_auto_default_run_setup_is_explicit_and_never_claims_operator_identity(
    monkeypatch,
) -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    monkeypatch.setattr(EvidenceAdapter, "session_catalog", lambda _self: ())
    backend = _RunSetupBackend()
    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)
    automatic = {
        "state": "APPLIED",
        "locked": False,
        "source": "identity will lock on first evidence",
        "experiment_id": "unassigned",
        "run_id": "session-automatic",
        "condition_id": "unspecified",
        "mode": "drone",
        "condition": {"identity_source": "automatic_session_identity"},
    }

    workspace._apply_run_setup_snapshot(automatic, recording_locked=False)
    assert workspace.run_setup_state_label.text() == (
        "AUTO_DEFAULT · EDITABLE · operator identity was not captured; "
        "fallback IDs preserve raw evidence"
    )
    assert workspace.run_setup_widgets["experiment_id"].isEnabled()
    assert workspace.validated_run_setup_payload(allow_auto_default=True) is None

    workspace._apply_run_setup_snapshot(
        {**automatic, "state": "LOCKED", "locked": True},
        recording_locked=False,
    )
    assert "AUTO_DEFAULT · LOCKED" in workspace.run_setup_state_label.text()
    assert "was not captured" in workspace.run_setup_state_label.text()
    assert not workspace.run_setup_widgets["experiment_id"].isEnabled()

    workspace.deleteLater()
    app.processEvents()


def test_same_thread_hr_oz_and_ulog_results_are_not_overwritten_by_pending_text(
    monkeypatch,
) -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    monkeypatch.setattr(EvidenceAdapter, "session_catalog", lambda _self: ())
    backend = _RunSetupBackend()
    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)

    def finish_still(command_id: str, _payload: object) -> None:
        label = (
            workspace.pnp_capture_status_label
            if command_id.startswith("recording.hr")
            else workspace.oz_capture_status_label
        )
        label.setText(f"{command_id}: COMPLETED")

    workspace.hr_command_requested.connect(finish_still)
    workspace.pnp_capture_button.click()
    assert workspace.pnp_capture_status_label.text() == "recording.hr.capture: COMPLETED"
    workspace.oz_capture_button.click()
    assert workspace.oz_capture_status_label.text() == "recording.oz.capture: COMPLETED"

    workspace.ulog_command_requested.connect(
        lambda _command_id, _payload: workspace.apply_ulog_snapshot(
            {
                "artifacts": [
                    {
                        "source_id": "p1",
                        "acquisition": "imported",
                        "ownership": "EXTERNAL",
                        "size_bytes": 42,
                        "sha256": "a" * 64,
                        "relative_path": "assets/ulog/p1.ulg",
                    }
                ]
            }
        )
    )
    workspace.ulog_import_button.click()
    assert workspace.ulog_status_label.text().startswith("ULog artifacts: 1")
    assert "awaiting owner" not in workspace.ulog_status_label.text()

    workspace.deleteLater()
    app.processEvents()
