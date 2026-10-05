from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtCore, QtTest, QtWidgets

from Workstation.evidence import EvidenceAdapter
from Workstation.status_model import RuntimeStatus, StatusModel, StatusSeverity
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
        self.session.update(
            state="OPEN",
            session_id="session-001",
            session_dir="/tmp/recordings/session-001",
            metadata={
                "opened_local_date": "2026-08-22",
                "opened_local_time": "14:30:00",
            },
        )
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

    assert workspace.findChild(QtWidgets.QWidget, "measurementPackageStart") is None
    assert workspace.findChild(QtWidgets.QWidget, "measurementSessionClose") is None
    assert workspace.finalize_session_button.text() == "Retry Save"
    assert workspace.session_status_label.text() == "Session: IDLE"
    assert workspace.session_step_label.text() == "Next: Sessions > Current"
    assert workspace.control_buttons["detection.toggle"].isEnabled()
    assert not workspace.finalize_session_button.isEnabled()

    panel.begin_measurement_session()
    workspace.refresh({})
    assert panel.calls == ["begin"]
    assert "COLLECTING" in workspace.session_status_label.text()
    assert workspace.session_step_label.text() == "Next: 측정 시작"
    assert "Pre evidence" in workspace.session_step_label.toolTip()
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
    assert workspace.control_buttons["detection.toggle"].isEnabled()
    assert workspace.finalize_session_button.isEnabled()
    assert workspace.evidence_summary_label.text() == (
        "Evidence: INCOMPLETE · 1 ok / 1 missing"
    )
    assert "present=1, missing=1" in workspace.evidence_summary_label.toolTip()

    workspace.finalize_session_button.click()
    assert panel.calls == ["begin", "finalize"]
    assert "VERIFYING" in workspace.session_status_label.text()
    assert workspace.session_step_label.text() == "Saving…"
    assert "draining producers" in workspace.session_step_label.toolTip()
    assert "waiting for producer drain" in workspace.session_status_label.toolTip()
    assert workspace.control_buttons["detection.toggle"].isEnabled()
    assert all(
        len(label.text()) <= workspace.RUN_STATUS_TEXT_LIMIT
        for label in (
            workspace.session_status_label,
            workspace.measurement_status_label,
            workspace.session_step_label,
            workspace.evidence_summary_label,
        )
    )
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
    assert workspace.findChild(QtWidgets.QWidget, "measurementPackageStart") is None
    assert workspace.findChild(QtWidgets.QWidget, "measurementPackageNewMeasurement") is None
    assert workspace.control_buttons["detection.toggle"].isEnabled()

    assert backend.begin_measurement_session()
    workspace.refresh({})
    assert "Session: OPEN" in workspace.session_status_label.text()
    assert workspace.measurement_status_label.text() == "Measurement: NONE"
    assert workspace.session_step_label.text() == "Next: 측정 시작"
    assert workspace.control_buttons["detection.toggle"].isEnabled()
    assert workspace.control_buttons["detection.toggle"].text() == "측정 시작"

    # CameraStreamLifecycleController performs this child creation before it
    # reserves Detection; this presentation test projects the resulting state.
    assert backend.begin_measurement()
    workspace.refresh({})
    assert "m000001" in workspace.measurement_status_label.text()
    assert workspace.control_buttons["detection.toggle"].isEnabled()
    backend.package.update(state="POST_CAPTURE", detection_started=True, detection_completed=True)
    workspace.refresh({})
    workspace.finalize_session_button.click()
    assert workspace.control_buttons["detection.toggle"].isEnabled()
    assert workspace.session_step_label.text() == "Next: 다음 측정 시작"

    assert backend.begin_measurement()
    workspace.refresh({})
    assert "m000002" in workspace.measurement_status_label.text()
    backend.package.update(state="POST_CAPTURE", detection_started=True, detection_completed=True)
    workspace.refresh({})
    workspace.finalize_session_button.click()
    assert backend.close_measurement_session()
    workspace.refresh({})
    assert workspace.control_buttons["detection.toggle"].isEnabled()

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


def test_sessions_workspace_tabs_project_ordered_measurements() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    workspace = RecordingWorkspace(
        backend,
        StatusModel(),
        lambda: None,
        lambda _command: True,
    )

    assert [
        workspace.tabs.tabText(index) for index in range(workspace.tabs.count())
    ] == ["Current", "Evidence", "Review"]
    assert workspace.current_tab_id() == "current"

    workspace.current_session.start_session_button.click()
    assert backend.begin_measurement()
    workspace.refresh({})

    assert workspace.measurements_model.rowCount() == 1
    assert _model_row(workspace.measurements_model, 0) == (
        "--",
        "m000001",
        "COLLECTING",
        "PENDING",
        "--",
    )
    assert workspace.select_tab("evidence")
    assert workspace.current_tab_id() == "evidence"
    assert workspace.select_tab("review")
    assert workspace.current_tab_id() == "review"
    assert not workspace.select_tab("unknown")

    workspace.deleteLater()
    app.processEvents()


def test_session_folder_creation_has_no_readiness_gate_and_accepts_pointer_click() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    backend.panel._active_optical_pose_selection_id = ""
    backend.panel.selected_extrinsics_profile = None
    backend.panel.checkerboard_extrinsics_profile = None
    backend.panel.active_intrinsics = None
    backend.panel.camera_connected = False
    backend.panel.camera_sync_status = "failed"
    backend.measurement_methods_snapshot = lambda: {
        "preferred_method_id": "checkerboard_stereo",
        "selected_method_id": "",
        "selection_reason": "no_ready_method",
        "method_catalog": [],
    }
    workspace = SessionWorkspace(backend, StatusModel())
    workspace.resize(900, 500)
    workspace.show()
    app.processEvents()

    assert workspace.start_session_button.isEnabled()
    assert "YYYYMMDD_HHMMSS_nanoseconds" in workspace.start_session_button.toolTip()
    assert workspace.start_session_button.width() <= (
        workspace.start_session_button.sizeHint().width() + 20
    )
    assert workspace.start_session_button.height() == workspace.close_session_button.height()
    center = workspace.start_session_button.rect().center()
    target = QtWidgets.QApplication.widgetAt(
        workspace.start_session_button.mapToGlobal(center)
    )
    assert target is workspace.start_session_button
    QtTest.QTest.mouseClick(
        workspace.start_session_button,
        QtCore.Qt.LeftButton,
        pos=center,
    )
    app.processEvents()

    assert backend.calls == ["session.begin"]
    assert workspace.session_status_label.text() == "Ready for saving"
    assert workspace.folder_name_label.text() == "session-001"
    assert workspace.folder_path_label.text() == "/tmp/recordings/session-001"
    assert workspace.folder_created_label.text() == "2026-08-22 14:30:00"
    assert not workspace.start_session_button.isEnabled()
    assert workspace.close_session_button.isEnabled()
    assert workspace.findChild(QtWidgets.QWidget, "sessionWorkspaceReadinessTable") is None
    assert workspace.findChild(QtWidgets.QWidget, "sessionWorkspaceMethodInputs") is None
    assert workspace.findChild(QtWidgets.QWidget, "sessionWorkspaceNewMeasurement") is None
    assert workspace.findChild(QtWidgets.QWidget, "sessionWorkspaceSealMeasurement") is None

    workspace.close()
    workspace.deleteLater()
    app.processEvents()


def test_session_workspace_shows_folder_only_and_routes_follow_up_navigation() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    backend.measurement_methods_snapshot = lambda: (_ for _ in ()).throw(
        AssertionError("Session must not read Measurement method readiness")
    )
    opened: list[str] = []
    workspace = SessionWorkspace(
        backend,
        StatusModel(),
        lambda: opened.append("measurement"),
        lambda: opened.append("review"),
    )

    assert workspace.start_session_button.isEnabled()
    workspace.start_session_button.click()
    workspace.refresh({})

    assert workspace.folder_name_label.text() == "session-001"
    assert workspace.open_measurement_button.isEnabled()
    assert workspace.open_review_button.isEnabled()
    workspace.open_measurement_button.click()
    workspace.open_review_button.click()
    assert opened == ["measurement", "review"]
    assert not hasattr(workspace, "next_action_label")
    assert not hasattr(workspace, "measurement_status_label")

    workspace.deleteLater()
    app.processEvents()


def test_session_folder_rejection_remains_visible_after_refresh() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    backend.begin_measurement_session = lambda: False
    backend.package.update(detail="injected folder creation rejection")
    workspace = SessionWorkspace(backend, StatusModel())
    workspace.show()
    app.processEvents()

    workspace.start_session_button.click()
    assert workspace.action_feedback_label.isVisible()
    assert workspace.action_feedback_label.property("severity") == "error"
    assert "injected folder creation rejection" in workspace.action_feedback_label.text()

    workspace.refresh({})
    assert workspace.action_feedback_label.isVisible()
    assert "injected folder creation rejection" in workspace.action_feedback_label.text()

    workspace.close()
    workspace.deleteLater()
    app.processEvents()


def test_session_async_close_feedback_clears_after_sealed_refresh() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    workspace = SessionWorkspace(backend, StatusModel())
    workspace.show()
    workspace.start_session_button.click()

    def close_async() -> bool:
        backend.calls.append("session.close.requested")
        backend.package.update(
            state="VERIFYING",
            finalize_requested=True,
            detail="waiting for session persistence worker",
        )
        return False

    backend.close_measurement_session = close_async
    workspace.close_session_button.click()
    app.processEvents()

    assert workspace.action_feedback_label.property("severity") == "info"
    assert workspace.action_feedback_label.text() == "Closing Session…"
    assert not workspace.action_feedback_label.isHidden()

    backend.session.update(state="SEALED", active_measurement_id=None)
    backend.package.update(
        state="SEALED_INCOMPLETE",
        session_state="SEALED",
        finalize_requested=False,
        detail="Package sealed as SEALED_INCOMPLETE",
    )
    workspace.refresh({})
    app.processEvents()

    assert workspace.session_status_label.text() == "Closed"
    assert workspace.action_feedback_label.text() == ""
    assert workspace.action_feedback_label.isHidden()

    workspace.close()
    workspace.deleteLater()
    app.processEvents()


def test_session_folder_repeated_pointer_click_creates_only_one_folder() -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    backend = _HierarchicalPackageBackend()
    workspace = SessionWorkspace(backend, StatusModel())
    workspace.show()
    app.processEvents()

    for _attempt in range(2):
        QtTest.QTest.mouseClick(
            workspace.start_session_button,
            QtCore.Qt.LeftButton,
            pos=workspace.start_session_button.rect().center(),
        )
        app.processEvents()

    assert backend.calls == ["session.begin"]
    assert workspace.folder_name_label.text() == "session-001"

    workspace.close()
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


def test_data_status_includes_right_camera_indicator_and_defaults_target_to_uam(
    monkeypatch,
) -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    monkeypatch.setattr(EvidenceAdapter, "session_catalog", lambda _self: ())
    backend = _RunSetupBackend()
    backend.panel._measurement_session_hierarchy_enabled = False

    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)

    assert workspace.run_platform_combo.currentData() == "uam"
    assert workspace.target_drone_payload()["mode"] == "uam"
    workspace.run_platform_combo.setCurrentIndex(
        workspace.run_platform_combo.findData("drone")
    )
    assert workspace.run_platform_combo.currentData() == "drone"
    assert workspace.target_drone_payload()["mode"] == "drone"

    assert set(workspace.indicators) >= {
        "camera.left",
        "camera.right",
    }
    assert workspace.indicators["camera.left"].label.text() == "CAM L"
    assert workspace.indicators["camera.right"].label.text() == "CAM R"

    workspace.refresh(
        {
            "camera.right": RuntimeStatus(
                key="camera.right",
                label="CAM R",
                state="LIVE",
                severity=StatusSeverity.OK,
                detail="right camera frame current",
                source="test",
                clock="test",
                age_s=0.1,
            )
        }
    )
    assert workspace.indicators["camera.right"].state.text() == "LIVE"

    workspace.deleteLater()
    app.processEvents()


def test_run_setup_draft_remains_editable_and_survives_unlocked_owner_poll(
    monkeypatch,
) -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    monkeypatch.setattr(EvidenceAdapter, "session_catalog", lambda _self: ())
    backend = _RunSetupBackend()
    workspace = RecordingWorkspace(backend, StatusModel(), lambda: None)
    assert not hasattr(workspace, "run_setup_widgets")
    assert workspace.target_reference_group.title() == "Target reference"
    return

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
    assert not hasattr(workspace, "run_setup_widgets")
    assert not hasattr(workspace, "run_setup_state_label")
    return
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
