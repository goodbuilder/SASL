"""Shared calibration/optical geometry ownership and coherent publication."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from Workstation.calibration_geometry_state import apply_geometry_values
from Workstation.tests.test_panel_composition import panels
from Workstation.workstation_core.active_geometry import ActiveGeometryStore


def test_camera_and_calibration_use_one_geometry_store(panels):
    panel = panels()
    store = panel.application.geometry_store
    calibration = panel.application.features["calibration"].state
    camera = panel.application.features["camera"].state
    assert calibration.geometry_store is store
    assert camera.geometry_store is store
    assert "active_intrinsics" not in vars(calibration)
    assert "_active_extrinsics_source_zoom_level" not in vars(camera)
    assert panel.active_intrinsics is store.read_legacy("active_intrinsics")
    assert panel._active_extrinsics_source_zoom_level == store.read_legacy("_active_extrinsics_source_zoom_level")


def test_prepared_geometry_bundle_publishes_as_one_revision():
    store = ActiveGeometryStore()
    panel = SimpleNamespace(application=SimpleNamespace(geometry_store=store))
    before = store.snapshot()
    profile = {"matrix": [[1.0, 0.0], [0.0, 1.0]]}
    apply_geometry_values(panel, {
        "active_intrinsics": profile, "_active_intrinsics_profile_hash": "a" * 64,
        "_active_extrinsics_source_zoom_level": 7,
    })
    snapshot = store.snapshot()
    assert snapshot.revision == before.revision + 1
    assert snapshot.values["_active_intrinsics_profile_hash"] == "a" * 64
    assert snapshot.values["_active_extrinsics_source_zoom_level"] == 7
    assert store.read_legacy("active_intrinsics") is profile
    snapshot.values["active_intrinsics"]["matrix"][0][0] = 999.0
    assert profile["matrix"][0][0] == 1.0
    with pytest.raises(TypeError):
        snapshot.values["_active_intrinsics_profile_hash"] = "changed"


def test_unknown_geometry_field_rejects_the_whole_update():
    store = ActiveGeometryStore()
    store.update({"_active_intrinsics_profile_hash": "old"})
    before = store.snapshot()
    with pytest.raises(ValueError, match="Unknown active geometry fields"):
        store.update({"_active_intrinsics_profile_hash": "new", "unrelated_widget": object()})
    assert store.snapshot() == before


def test_named_legacy_read_and_delete_keep_missing_attribute_semantics():
    store = ActiveGeometryStore()
    with pytest.raises(AttributeError):
        store.read_legacy("active_intrinsics")
    store.write_legacy("active_intrinsics", None)
    assert store.read_legacy("active_intrinsics") is None
    store.remove_legacy("active_intrinsics")
    with pytest.raises(AttributeError):
        store.read_legacy("active_intrinsics")


def test_restore_geometry_fields_is_one_publication_and_preserves_value_identity():
    store = ActiveGeometryStore()
    original = object()
    store.update({"active_intrinsics": original, "_active_intrinsics_profile_hash": "old"})
    before = {"active_intrinsics": (True, original), "_active_intrinsics_profile_hash": (True, "old"),
              "_active_optical_pose_selection_id": (False, None)}
    store.update({"active_intrinsics": object(), "_active_intrinsics_profile_hash": "new",
                  "_active_optical_pose_selection_id": "selection"})
    revision = store.snapshot().revision
    store.restore_fields(before)
    assert store.snapshot().revision == revision + 1
    assert store.read_legacy("active_intrinsics") is original
    assert store.read_legacy("_active_intrinsics_profile_hash") == "old"
    with pytest.raises(AttributeError):
        store.read_legacy("_active_optical_pose_selection_id")
