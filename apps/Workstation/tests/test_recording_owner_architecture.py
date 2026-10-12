"""Independent contracts for the recording ownership decomposition."""
from __future__ import annotations

import ast
import inspect
from pathlib import Path
from types import SimpleNamespace

import pytest

from Workstation.recording import state as state_module
from Workstation.recording.binding import recording_state_for
from Workstation.recording.formatting import Formatting
from Workstation.recording.ports import FieldBinding, TimingFieldsPort
from Workstation.recording.state import RecordingSessionState
from Workstation.recording.timing_fields import TimingFields
from Workstation.recording_controller import RecordingController


def _read(value: object) -> FieldBinding:
    return FieldBinding(lambda: value)


def test_schema_builder_operates_from_explicit_inputs_without_panel() -> None:
    formatter = Formatting(session=RecordingSessionState())
    boundary = TimingFieldsPort(
        _camera_time_sync_metrics=_read(lambda: {}),
        _yolo_pair_max_delta_s=_read(lambda: 0.1),
        camera_sync_reason=_read("fixture"),
        camera_sync_status=_read("READY"),
    )
    builder = TimingFields(boundary=boundary)
    builder.bind_collaborators(formatting=formatter)
    detection = SimpleNamespace(
        x1=10.0, y1=20.0, x2=30.0, y2=50.0,
        center=(20.0, 35.0), confidence=0.75, class_id=3,
        class_name="target",
    )
    row = builder._record_bbox_parameters(SimpleNamespace(left=detection, right=None))
    # Independently supplied optical pixel values: x right, y down.
    assert row["left_bbox_center_x_px"] == 20.0
    assert row["left_bbox_center_y_px"] == 35.0
    assert row["left_width_px"] == 20.0
    assert row["left_height_px"] == 30.0
    assert row["right_bbox_center_x_px"] is None
    assert not hasattr(builder, "panel")
    with pytest.raises(AttributeError):
        boundary.camera_sync_status = "INVALID"
    with pytest.raises(AttributeError):
        boundary.unrelated_panel_setting = 3


def test_real_state_owns_shared_native_admission_ledger_and_resource_handles() -> None:
    class Host:
        pass

    host = Host()
    original_rows = []
    native_writers = {"previous-generation": object()}
    host.distance_recording_rows = original_rows
    host._native_evidence_writers = native_writers
    first_state = recording_state_for(host)
    owner = RecordingController(host)
    assert owner.recording_state is first_state
    assert owner.owners.primary_persistence.ledger is owner.owners.native_completion.ledger
    assert owner.owners.primary_persistence.native_state is owner.owners.native_completion.native_state
    assert owner.owners.primary_persistence.ledger._primary_persistence_lock is owner._primary_persistence_lock
    assert owner.recording_state.rows.distance_recording_rows is original_rows
    assert "distance_recording_rows" not in vars(host)
    replacement = [{"measurement_frame": "baseline_center_frd"}]
    host.distance_recording_rows = replacement
    assert owner.recording_state.rows.distance_recording_rows is replacement
    assert owner.resource_inventory()["_native_evidence_writers"] is native_writers
    assert owner.resource_inventory()["_pending_native_completions"] is owner._pending_native_completions
    assert owner.resource_inventory()["_pending_primary_record_rows"] is owner._pending_primary_record_rows


def test_native_terminal_state_survives_new_compatibility_facade() -> None:
    host = SimpleNamespace()
    first = RecordingController(host)
    first._native_conflicts_recorded.add("older-generation-request")
    second = RecordingController(host)
    assert second.recording_state is first.recording_state
    assert second._native_conflicts_recorded == {"older-generation-request"}
    assert second._primary_persistence_lock is first._primary_persistence_lock


def test_core_owners_have_no_panel_controller_import_or_unbounded_forwarding() -> None:
    package = Path(inspect.getfile(state_module)).parent
    boundaries = {"binding.py", "compatibility.py", "composition.py", "ports.py"}
    for path in sorted(package.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                assert node.name not in {"__getattr__", "__setattr__"}, path.name
            if path.name in boundaries:
                continue
            if isinstance(node, ast.ImportFrom):
                assert node.module not in {
                    "Workstation.standalone_gui", "Workstation.recording_controller",
                    "Workstation.measurement_view_controller",
                }, path.name
            if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
                if node.value.id == "self":
                    assert node.attr not in {"panel", "controller", "context"}, path.name
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                assert node.value not in {
                    "_measurement_view_controller_owner", "_target_drone_controller_owner",
                    "_data_storage_controller",
                }, path.name
