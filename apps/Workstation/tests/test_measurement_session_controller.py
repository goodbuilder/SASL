from __future__ import annotations

import json
import hashlib
import re
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtTest, QtWidgets

from Workstation.camera_stream_controller import CameraStreamLifecycleController
from Workstation.measurement_package_controller import MeasurementPackageController
from Workstation.status_model import StatusModel
from Workstation.workspaces import SessionWorkspace
from Workstation.workstation_core import measurement_session as measurement_session_module
from Workstation.workstation_core.measurement_session import verify_measurement_session
from Workstation.workstation_core.recording_session import save_session_files
from Workstation.workstation_core.measurement_methods import (
    HORIZON_HYBRID,
    MEASUREMENT_FILL_NONE,
    default_measurement_method_fill_policy,
)


def _panel(tmp_path: Path) -> SimpleNamespace:
    panel = SimpleNamespace(
        recording_session_id="",
        _measurement_session_hierarchy_enabled=True,
        _measurement_package_explicit_lifecycle=True,
        _measurement_package_collect_provenance=False,
        _measurement_package_verify_media=False,
        _session_save_thread=None,
        _session_save_finalized=False,
        _session_save_retry_required=False,
        _automatic_session_recording=False,
        _recording_mode="manual",
        _pnp_session_prepared=False,
        pnp_photo_capture_records=[],
        oz_photo_capture_records=[],
        distance_recording_rows=[],
        camera_connected=False,
        _pending_camera_start=None,
        pnp_photo_capture_thread=None,
        oz_photo_capture_thread=None,
        left_pane=None,
        right_pane=None,
        _session_recording_active=lambda: False,
        _log=lambda _message: None,
    )

    def session_root(session_id: str) -> Path:
        return tmp_path / "recordings" / session_id

    def measurement_root(session_id: str) -> Path:
        root = session_root(session_id)
        measurement_id = str(getattr(panel, "_measurement_id", "") or "")
        if (
            str(getattr(panel, "_measurement_session_id", "") or "") == session_id
            and measurement_id
        ):
            return root / "measurements" / measurement_id
        return root

    panel._measurement_session_dir = session_root
    panel._measurement_sessions_root = lambda: tmp_path / "recordings"
    panel._recording_session_dir = measurement_root
    return panel


def _write_outputs(panel: SimpleNamespace, *, valid_count: int) -> Path:
    root = panel._recording_session_dir(panel.recording_session_id)
    (root / "video").mkdir(parents=True, exist_ok=True)
    (root / "video" / "raw_left.mp4").write_bytes(b"left")
    (root / "video" / "raw_right.mp4").write_bytes(b"right")
    rows = [
        {"pair_id": f"{panel._measurement_id}-pair-{index}", "measurement_valid": True}
        for index in range(valid_count)
    ]
    save_session_files(
        root,
        metadata={
            "session_id": panel.recording_session_id,
            "measurement_id": panel._measurement_id,
        },
        pair_rows=rows,
        camera_rows={"left": [], "right": []},
        px4_rows={"left": [], "right": []},
    )
    panel.distance_recording_rows = rows
    panel._session_save_finalized = True
    return root


def test_pointer_click_creates_timestamp_folder_and_displays_exact_identity(
    tmp_path: Path,
) -> None:
    app = QtWidgets.QApplication.instance()
    assert app is not None
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    backend = SimpleNamespace(
        panel=panel,
        begin_measurement_session=controller.begin_measurement_session,
        measurement_session_snapshot=controller.measurement_session_snapshot,
        measurement_package_snapshot=controller.measurement_package_snapshot,
        close_measurement_session=controller.close_measurement_session,
    )
    workspace = SessionWorkspace(backend, StatusModel())
    workspace.resize(900, 500)
    workspace.show()
    app.processEvents()

    center = workspace.start_session_button.rect().center()
    assert QtWidgets.QApplication.widgetAt(
        workspace.start_session_button.mapToGlobal(center)
    ) is workspace.start_session_button
    QtTest.QTest.mouseClick(
        workspace.start_session_button,
        QtCore.Qt.LeftButton,
        pos=center,
    )
    app.processEvents()

    snapshot = controller.measurement_session_snapshot()
    session_id = str(snapshot["session_id"])
    root = tmp_path / "recordings" / session_id
    assert re.fullmatch(r"\d{8}_\d{6}_\d{9}", session_id)
    assert root.is_dir()
    assert (root / "session.json").is_file()
    assert Path(str(snapshot["session_dir"])) == root
    assert root.name == session_id
    assert not (root / "measurements").exists()
    assert workspace.folder_name_label.text() == root.name
    assert workspace.folder_path_label.text() == str(root)
    assert workspace.folder_created_label.text() != "—"
    assert workspace.session_status_label.text() == "Ready for saving"

    QtTest.QTest.mouseClick(
        workspace.start_session_button,
        QtCore.Qt.LeftButton,
        pos=center,
    )
    app.processEvents()
    assert [path for path in (tmp_path / "recordings").iterdir() if path.is_dir()] == [root]

    workspace.close()
    workspace.deleteLater()
    app.processEvents()


def test_one_explicit_session_contains_two_isolated_measurements(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)

    assert controller.begin_measurement_session(session_id="20260821_120000_000000001")
    session_id = panel.recording_session_id
    assert panel._measurement_id == ""
    assert controller.measurement_session_snapshot()["state"] == "OPEN"
    assert not (tmp_path / "recordings" / session_id / "measurements").exists()
    assert controller.begin_measurement()
    assert panel._measurement_id == "m000001"
    first_root = panel._recording_session_dir(session_id)
    assert first_root == tmp_path / "recordings" / session_id / "measurements" / "m000001"
    assert controller.begin_measurement_detection()
    assert controller.complete_measurement_detection()
    _write_outputs(panel, valid_count=1)
    assert controller.finalize_measurement_package()
    first_manifest = first_root / "revisions" / "r0001" / "package_manifest.json"
    first_bytes = first_manifest.read_bytes()

    assert controller.measurement_session_snapshot()["state"] == "OPEN"
    assert controller.begin_next_measurement()
    assert panel.recording_session_id == session_id
    assert panel._measurement_id == "m000002"
    second_root = panel._recording_session_dir(session_id)
    assert second_root == tmp_path / "recordings" / session_id / "measurements" / "m000002"
    assert second_root != first_root
    assert panel.distance_recording_rows == []
    assert controller.begin_measurement_detection()
    assert controller.complete_measurement_detection()
    _write_outputs(panel, valid_count=0)
    assert controller.finalize_measurement_package()
    assert first_manifest.read_bytes() == first_bytes

    session_snapshot = controller.measurement_session_snapshot()
    assert [row["measurement_id"] for row in session_snapshot["measurements"]] == [
        "m000001",
        "m000002",
    ]
    assert controller.close_measurement_session()
    assert controller.measurement_session_snapshot()["state"] == "SEALED"
    session_root = tmp_path / "recordings" / session_id
    assert (session_root / "review.xlsx").is_file()
    assert verify_measurement_session(session_root).valid


def test_measurement_start_creates_each_child_before_detection(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    calls: list[object] = []
    panel.measurement_package_snapshot = controller.measurement_package_snapshot
    panel.begin_measurement = controller.begin_measurement
    panel.begin_measurement_detection = controller.begin_measurement_detection
    panel.start_automatic_session_recording = (
        lambda: calls.append("recording.begin") or True
    )
    panel._start_camera_streams = (
        lambda **kwargs: calls.append(("streams.begin", kwargs))
    )

    assert controller.begin_measurement_session(session_id="measurement-start-session")
    assert panel._measurement_id == ""

    camera = CameraStreamLifecycleController(panel)
    assert camera.start_yolo() is None
    assert panel._measurement_id == "m000001"
    assert controller.measurement_package_snapshot()["detection_started"] is True
    assert calls == [
        "recording.begin",
        ("streams.begin", {"enable_detection": True, "switch_to_depth": True}),
    ]

    assert controller.complete_measurement_detection()
    _write_outputs(panel, valid_count=1)
    assert controller.finalize_measurement_package()

    calls.clear()
    assert camera.start_yolo() is None
    assert panel._measurement_id == "m000002"
    assert controller.measurement_package_snapshot()["detection_started"] is True
    assert calls == [
        "recording.begin",
        ("streams.begin", {"enable_detection": True, "switch_to_depth": True}),
    ]


def test_close_session_auto_seals_empty_measurement(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    session_id = "20260822_181438_641156758"
    assert controller.begin_measurement_session(session_id=session_id)
    assert controller.begin_measurement()
    assert controller.measurement_package_snapshot()["state"] == "COLLECTING"

    panel.save_distance_history_csv = lambda: _write_outputs(panel, valid_count=0)

    assert controller.close_measurement_session() is True
    assert controller.measurement_package_snapshot()["state"] in {
        "SEALED_COMPLETE",
        "SEALED_INCOMPLETE",
    }
    assert controller.measurement_session_snapshot()["state"] == "SEALED"
    assert verify_measurement_session(tmp_path / "recordings" / session_id).valid


def test_close_session_auto_seals_recovered_empty_measurement(tmp_path: Path) -> None:
    first_panel = _panel(tmp_path)
    first_panel._measurement_app_run_id = "run-a"
    first = MeasurementPackageController(first_panel)
    session_id = "recovered-empty-close"
    assert first.begin_measurement_session(session_id=session_id)
    assert first.begin_measurement()
    assert first.measurement_package_snapshot()["state"] == "COLLECTING"

    restarted_panel = _panel(tmp_path)
    restarted_panel._measurement_app_run_id = "run-b"
    restarted = MeasurementPackageController(restarted_panel)
    recovered = restarted.measurement_package_snapshot()
    assert recovered["state"] == "RECOVERABLE"
    assert recovered["detection_started"] is False
    assert recovered["detection_completed"] is False

    restarted_panel.save_distance_history_csv = lambda: _write_outputs(
        restarted_panel,
        valid_count=0,
    )
    assert restarted.close_measurement_session() is True
    assert restarted.measurement_package_snapshot()["state"] in {
        "SEALED_COMPLETE",
        "SEALED_INCOMPLETE",
    }
    assert restarted.measurement_session_snapshot()["state"] == "SEALED"
    assert verify_measurement_session(tmp_path / "recordings" / session_id).valid


def test_close_session_still_requires_active_detection_to_stop(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    assert controller.begin_measurement_session(session_id="active-detection-close")
    assert controller.begin_measurement()
    assert controller.begin_measurement_detection()

    assert controller.close_measurement_session() is False
    assert controller.measurement_session_snapshot()["state"] == "OPEN"
    assert panel._measurement_session_close_requested is False
    assert "Stop Detection before closing" in controller.measurement_package_snapshot()["detail"]


def test_new_measurement_preserves_evidence_without_alignment_geometry(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    assert controller.begin_measurement_session(session_id="20260822_150000_000000001")

    assert controller.begin_measurement() is True
    snapshot = controller.measurement_package_snapshot()
    assert snapshot["state"] == "COLLECTING"
    assert snapshot["measurement_id"] == "m000001"


def test_measurement_snapshots_exact_session_alignment_and_rejects_foreign_paths(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    session_id = "20260822_150000_000000009"
    assert controller.begin_measurement_session(session_id=session_id)
    alignment = tmp_path / "recordings" / session_id / "alignment" / "selections" / "selected.json"
    alignment.parent.mkdir(parents=True, exist_ok=True)
    payload = b'{"schema":"alignment-selection-test-v1","selection":"sentinel"}\n'
    alignment.write_bytes(payload)
    panel._active_alignment_snapshot_path = str(alignment)
    panel._active_alignment_snapshot_sha256 = hashlib.sha256(payload).hexdigest()

    assert controller.begin_measurement()
    assert controller.begin_measurement_detection()

    package = controller.measurement_package_snapshot()
    row = next(
        item
        for item in package["artifacts"]
        if item["role"] == "active_alignment_selection"
    )
    copied = Path(controller.package.root) / row["relative_path"]
    assert copied.read_bytes() == payload
    assert row["sha256"] == hashlib.sha256(payload).hexdigest()
    assert row["relative_path"].startswith("inputs/alignment/active/")

    with pytest.raises(ValueError, match="escapes"):
        controller._register_descriptor(
            {
                **row,
                "relative_path": "../m000010/foreign.json",
            }
        )


def test_yolo_model_is_not_copied_into_session_or_measurement_package(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.workstation_core import package_provenance

    model = tmp_path / "runtime-models" / "detector-sentinel.pt"
    model.parent.mkdir(parents=True)
    model.write_bytes(b"runtime-owned-yolo-weights")
    panel = _panel(tmp_path)
    panel._measurement_package_collect_provenance = True
    panel.settings = SimpleNamespace(yolo_weights_path=str(model))
    panel.weights_edit = SimpleNamespace(text=lambda: str(model))
    panel.yolo_kwargs = lambda: {
        "weights": str(model),
        "resolved_weights": str(model),
    }
    monkeypatch.setattr(
        package_provenance,
        "collect_software_provenance",
        lambda *_args, **_kwargs: [],
    )
    controller = MeasurementPackageController(panel)
    session_id = "no-yolo-model-copy"

    assert controller.begin_measurement_session(session_id=session_id)
    assert controller.begin_measurement()
    assert controller.begin_measurement_detection()

    session_root = tmp_path / "recordings" / session_id
    measurement_root = session_root / "measurements" / "m000001"
    snapshot = controller.measurement_package_snapshot()
    assert not (session_root / "inputs" / "models").exists()
    assert not (measurement_root / "inputs" / "models").exists()
    assert not list(session_root.rglob("*.pt"))
    assert "active_yolo_model" not in {
        str(row.get("role") or "") for row in snapshot["artifacts"]
    }
    assert all(
        "active_yolo_model" not in tuple(row.get("required_roles") or ())
        for row in snapshot["source_expectations"]
    )


def test_new_measurement_freezes_method_fill_policy_in_child_metadata(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    policy = default_measurement_method_fill_policy()
    policy[HORIZON_HYBRID]["left.yaw_deg"] = MEASUREMENT_FILL_NONE
    owner = SimpleNamespace(
        measurement_method_config_for_new_measurement=lambda: {
            "measurement_method_config_schema": "measurement-method-config-v1",
            "measurement_preferred_method": HORIZON_HYBRID,
            "measurement_method_fallback_order": [HORIZON_HYBRID],
            "measurement_method_fill_policy": policy,
        }
    )
    panel._measurement_view_controller = lambda: owner
    controller = MeasurementPackageController(panel)

    assert controller.begin_measurement_session(session_id="method-policy-session")
    assert controller.begin_measurement()

    metadata = panel._measurement_package.snapshot()["metadata"]
    assert metadata["measurement_method_config_schema"] == "measurement-method-config-v1"
    assert metadata["measurement_preferred_method"] == HORIZON_HYBRID
    assert metadata["measurement_method_fill_policy"][HORIZON_HYBRID][
        "left.yaw_deg"
    ] == MEASUREMENT_FILL_NONE


def test_next_measurement_requires_current_measurement_to_be_sealed(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    assert controller.begin_measurement_session(session_id="session")
    assert controller.begin_measurement()

    assert controller.begin_next_measurement() is False
    assert panel._measurement_id == "m000001"
    assert [path.name for path in (tmp_path / "recordings" / "session" / "measurements").iterdir()] == [
        "m000001"
    ]


def test_direct_and_legacy_session_id_collision_fails_closed(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    for root in (
        tmp_path / "recordings" / "collision",
        tmp_path / "recordings" / "sessions" / "collision",
    ):
        root.mkdir(parents=True)
        (root / "session.json").write_text(
            json.dumps(
                {
                    "schema": "workstation-measurement-session-v1",
                    "schema_version": 1,
                    "session_id": "collision",
                    "state": "OPEN",
                    "updated_utc_ns": 1,
                    "measurements": [],
                }
            ),
            encoding="utf-8",
        )

    controller = MeasurementPackageController(panel)
    snapshot = controller.measurement_session_snapshot()

    assert snapshot["state"] == "RECOVERABLE"
    assert "collision" in snapshot["error"]


def test_restart_discovers_open_session_and_marks_interrupted_measurement_recoverable(
    tmp_path: Path,
) -> None:
    first_panel = _panel(tmp_path)
    first_panel._measurement_app_run_id = "run-a"
    first = MeasurementPackageController(first_panel)
    assert first.begin_measurement_session(session_id="restart-session")
    assert first.begin_measurement()
    assert first.begin_measurement_detection()

    restarted_panel = _panel(tmp_path)
    restarted_panel._measurement_app_run_id = "run-b"
    restarted = MeasurementPackageController(restarted_panel)
    session = restarted.measurement_session_snapshot()
    package = restarted.measurement_package_snapshot()

    assert session["session_id"] == "restart-session"
    assert session["state"] == "OPEN"
    assert restarted_panel.recording_session_id == "restart-session"
    assert restarted_panel._measurement_id == "m000001"
    assert package["state"] == "RECOVERABLE"
    assert restarted.begin_next_measurement() is False


def test_sealed_measurement_retries_only_parent_index_after_sync_failure(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    assert controller.begin_measurement_session(session_id="sync-retry")
    assert controller.begin_measurement()
    assert controller.begin_measurement_detection()
    assert controller.complete_measurement_detection()
    root = _write_outputs(panel, valid_count=1)
    session = panel._measurement_session
    original_refresh = session.refresh_measurement
    failed = False

    def fail_once(*args: object, **kwargs: object) -> object:
        nonlocal failed
        if not failed:
            failed = True
            raise OSError("injected parent index failure")
        return original_refresh(*args, **kwargs)

    session.refresh_measurement = fail_once
    assert controller.finalize_measurement_package() is False
    manifest = root / "revisions" / "r0001" / "package_manifest.json"
    before = manifest.read_bytes()
    sealed_state = json.loads(before)["state"]
    assert controller.measurement_package_snapshot()["state"] == "RECOVERABLE"

    assert controller.finalize_measurement_package() is True
    assert manifest.read_bytes() == before
    assert controller.measurement_package_snapshot()["state"] == sealed_state


def test_parent_close_retry_reuses_manifested_review_bytes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    assert controller.begin_measurement_session(session_id="parent-close-retry")
    assert controller.begin_measurement()
    assert controller.begin_measurement_detection()
    assert controller.complete_measurement_detection()
    _write_outputs(panel, valid_count=1)
    assert controller.finalize_measurement_package()
    session_root = tmp_path / "recordings" / "parent-close-retry"
    review = session_root / "review.xlsx"
    original_write = measurement_session_module._atomic_write_bytes
    failed = False

    def fail_sidecar_once(path: Path, payload: bytes, *, replace: bool = True) -> None:
        nonlocal failed
        if path.name == "measurement_session_manifest.sha256" and not failed:
            failed = True
            raise OSError("injected sidecar failure")
        original_write(path, payload, replace=replace)

    monkeypatch.setattr(
        measurement_session_module,
        "_atomic_write_bytes",
        fail_sidecar_once,
    )
    assert controller.close_measurement_session() is False
    assert (session_root / "measurement_session_manifest.json").is_file()
    review_before = review.read_bytes()

    assert controller.close_measurement_session() is True
    assert review.read_bytes() == review_before
    assert verify_measurement_session(session_root).valid is True
