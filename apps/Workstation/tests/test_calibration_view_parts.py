"""Calibration view ownership and the input/quality/preview handoffs."""
from __future__ import annotations

import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workspace_views.calibration import CalibrationWorkspace
from Workstation.workspace_views.calibration_parts.commands import CalibrationCommandWidgets
from Workstation.workspace_views.calibration_parts.inputs import CalibrationInputPicker
from Workstation.workspace_views.calibration_parts.preview import CalibrationPreview
from Workstation.workspace_views.calibration_parts.quality import CalibrationQualityPanel
from Workstation.workspace_views.calibration_parts.selection_files import write_outlier_selection
from Workstation.workspace_views.calibration_parts.contracts import PreviewChannel


PARTS = Path(__file__).resolve().parents[1] / "workspace_views" / "calibration_parts"


def _assert_component_boundary(source: str) -> None:
    """Reject broad owner injection/traversal, not documentation vocabulary."""
    broad = {"workspace", "host", "panel", "backend", "application", "context", "window"}
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.ImportFrom):
            module = node.module or ""
            assert "workspaces" not in module.split(".")
            assert module not in {"calibration", "Workstation.workspace_views.calibration"}
            assert not any(alias.name in {"CalibrationWorkspace", "workspaces", "_evidence"} for alias in node.names)
        elif isinstance(node, ast.Import):
            assert not any(alias.name in {
                "Workstation.workspaces", "Workstation.workspace_views.calibration",
            } for alias in node.names)
        elif isinstance(node, ast.FunctionDef):
            assert node.name not in {"__getattr__", "__setattr__", "get_panel", "get_backend"}
            if node.name == "__init__":
                assert not ({arg.arg for arg in (*node.args.posonlyargs, *node.args.args, *node.args.kwonlyargs)} & broad)
        elif isinstance(node, ast.Attribute):
            assert node.attr not in broad | {"get_panel", "get_backend"}
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in {"getattr", "setattr"}:
            if len(node.args) > 1 and isinstance(node.args[1], ast.Constant):
                assert node.args[1].value not in broad


def test_calibration_components_never_import_or_accept_the_workspace():
    for path in PARTS.glob("*.py"):
        _assert_component_boundary(path.read_text())


@pytest.mark.parametrize("source", [
    "from ..calibration import CalibrationWorkspace",
    "import Workstation.workspace_views.calibration",
    "from Workstation import workspaces",
    "class Owner:\n    def __init__(self, workspace): pass",
    "class Owner:\n    def __init__(self, *, backend): self.source = backend",
    "class Owner:\n    def read(self): return self.backend.calibration_setup()",
    "class Owner:\n    def read(self, source): return getattr(source, 'panel', source)",
    "class Owner:\n    def read(self): return self.source.panel.last_calibration_result",
    "class Owner:\n    def read(self): return self.source.get_panel()",
    "from ..shared import _evidence",
])
def test_calibration_boundary_rejects_forbidden_fixture(source):
    with pytest.raises(AssertionError):
        _assert_component_boundary(source)


def _write_images(root: Path, count: int = 2):
    paths = {"left": [], "right": []}
    for side in paths:
        (root / side).mkdir(parents=True, exist_ok=True)
        for index in range(count):
            path = root / side / f"{side}_{index:03d}.png"
            image = QtGui.QImage(32, 24, QtGui.QImage.Format_RGB888)
            image.fill(QtGui.QColor("black"))
            assert image.save(str(path))
            paths[side].append(path)
    return paths


def _input_picker(root: Path, parent: QtWidgets.QWidget):
    setup = {"calibration_root_resolved": str(root)}
    picker = CalibrationInputPicker(
        read_setup=lambda: setup,
        default_root=str(root), parent=parent,
    )
    picker._build_calibration_file_selection_group().setParent(parent)
    picker._calibration_session_explicit_root = str(root)
    return picker


def test_left_and_right_file_dialogs_share_the_solver_selection(tmp_path, monkeypatch):
    from Workstation.calibration_controller import CalibrationWorkflowController

    parent = QtWidgets.QWidget()
    root = tmp_path / "selected-session"
    paths = _write_images(root)
    picker = _input_picker(root, parent)
    responses = iter((
        ([str(paths["left"][1]), str(paths["left"][0])], ""),
        ([str(paths["right"][1]), str(paths["right"][0])], ""),
    ))
    monkeypatch.setattr(QtWidgets.QFileDialog, "getOpenFileNames", lambda *_args: next(responses))
    try:
        picker._choose_calibration_files("left")
        picker._choose_calibration_files("right")
        saved = json.loads((root / "calibration_pair_selection.json").read_text())
        assert saved["root"] == "."
        assert saved["left_paths"] == ["left/left_001.png", "left/left_000.png"]
        assert saved["right_paths"] == ["right/right_001.png", "right/right_000.png"]
        controller = CalibrationWorkflowController.__new__(CalibrationWorkflowController)
        controller.panel = SimpleNamespace(
            _load_calibration_pair_selection=picker._load_calibration_pair_selection,
            _calibration_dirs=lambda: (root, root / "left", root / "right"),
        )
        left, right, explicit = controller._selected_calibration_file_lists(root, [], [])
        assert explicit
        assert left == list(reversed(paths["left"]))
        assert right == list(reversed(paths["right"]))
        assert picker.calibration_selected_files_table.item(0, 1).text() == "left_001.png"
        assert picker.calibration_selected_files_table.item(0, 2).text() == "right_001.png"
    finally:
        picker.cancel_pending_thumbnails()
        parent.deleteLater()


def test_file_refresh_preserves_same_session_scroll_and_column_width(tmp_path):
    parent = QtWidgets.QWidget()
    paths = _write_images(tmp_path, count=30)
    picker = _input_picker(tmp_path, parent)
    table = picker.calibration_selected_files_table
    table.setParent(None)
    table.resize(620, 180)
    try:
        picker._sync_calibration_file_selection()
        table.show()
        QtWidgets.QApplication.processEvents()
        table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Interactive)
        table.setColumnWidth(1, 177)
        table.verticalScrollBar().setValue(8)
        table.setCurrentCell(10, 1)
        QtWidgets.QApplication.processEvents()
        before = table.verticalScrollBar().value()
        metadata = tmp_path / "capture_metadata"
        metadata.mkdir()
        (metadata / "capture_000.json").write_text(json.dumps({"capture_recorded_unix_ns": 1700000000000000000}))
        picker._sync_calibration_file_selection()
        assert table.verticalScrollBar().value() == before
        assert table.columnWidth(1) == 177
        assert table.currentRow() == 10
        assert paths["left"][0].name == table.item(0, 1).text()
    finally:
        table.close()
        table.deleteLater()
        parent.deleteLater()


def test_root_change_retires_old_thumbnail_before_it_can_touch_items(tmp_path):
    parent = QtWidgets.QWidget()
    picker = _input_picker(tmp_path, parent)
    picker._build_calibration_session_group().setParent(parent)
    picker._calibration_session_explicit_root = str(tmp_path / "previous-session")
    picker._calibration_pair_picker_thumbnail_generation = 7
    picker._calibration_pair_picker_thumbnail_paths = [("old", tmp_path / "old.png")]
    picker._calibration_pair_picker_items = {"old": SimpleNamespace(setIcon=lambda _icon: pytest.fail("stale thumbnail applied"))}
    picker.calibration_session_combo.setCurrentIndex(0)
    try:
        picker._on_calibration_session_selected(0)
        assert picker._calibration_pair_picker_thumbnail_generation > 7
        picker._load_next_calibration_pair_thumbnail(7)
        assert picker._calibration_pair_picker_thumbnail_paths == []
    finally:
        parent.deleteLater()


def test_quality_confirmation_is_owned_and_resets_for_a_new_result():
    parent = QtWidgets.QWidget()
    commands = CalibrationCommandWidgets()
    for command in ("calibration.apply_bundle", "calibration.save_intrinsics"):
        commands.primary[command] = QtWidgets.QPushButton(command, parent)
    panel = CalibrationQualityPanel(
        commands, read_selection=lambda: {},
        selection_saved=lambda: None, refresh_labels=lambda: None, parent=parent,
        read_quality=lambda: {}, read_diagnostics=lambda: (), read_setup=lambda: {},
    )
    panel._build_calibration_quality_group().setParent(parent)
    try:
        warning = {"state": "WARNING", "quality_id": "first", "reasons": ["coverage"]}
        panel.apply_calibration_quality_snapshot(warning)
        assert not commands.primary["calibration.apply_bundle"].isEnabled()
        assert commands.primary["calibration.save_intrinsics"].isEnabled()
        panel.confirm_calibration_quality_warning(True)
        assert commands.primary["calibration.apply_bundle"].isEnabled()
        panel.confirm_calibration_quality_warning(False)
        assert not commands.primary["calibration.apply_bundle"].isEnabled()
        panel.confirm_calibration_quality_warning(True)
        panel.apply_calibration_quality_snapshot({**warning, "quality_id": "second"})
        assert not panel._calibration_quality_warning_confirmed
        assert not commands.primary["calibration.apply_bundle"].isEnabled()
    finally:
        parent.deleteLater()


def test_preview_owner_keeps_overlay_and_manual_ftp_mode(tmp_path):
    parent = QtWidgets.QWidget()
    preview = CalibrationPreview(parent)
    preview.camera_mode_tabs.setParent(parent)
    preview.build_simple_previews(parent)
    paths = _write_images(tmp_path, count=1)
    overlay = QtGui.QImage(10, 10, QtGui.QImage.Format_RGB888)
    overlay.fill(QtGui.QColor("red"))
    raw = QtGui.QImage(10, 10, QtGui.QImage.Format_RGB888)
    raw.fill(QtGui.QColor("blue"))
    try:
        preview._on_calibration_preview_changed({"camera": "left", "image": overlay, "kind": "overlay"})
        preview._on_camera_frame("left", raw, {"source": "raw"})
        assert preview._checkerboard_overlay_active["left"]
        assert preview._camera_preview_metadata["left"] == {"source": "raw"}
        preview.camera_mode_tabs.setCurrentIndex(1)
        preview._on_camera_mode_tab_clicked(1)
        preview._show_ftp_pair_preview({"paths": {side: str(files[0]) for side, files in paths.items()}})
        assert preview.camera_mode_tabs.currentIndex() == 1
        assert not preview._ftp_preview_timer.isActive()
        preview._clear_camera_previews()
        assert not any(preview._checkerboard_overlay_active.values())
        assert preview.camera_mode_tabs.currentIndex() == 1
    finally:
        preview.stop()
        parent.deleteLater()


def test_workspace_compatibility_aliases_have_one_owner_and_explicit_timers(tmp_path, monkeypatch):
    from Workstation.workspace_views import calibration as module

    monkeypatch.setattr(module, "DEFAULT_CALIBRATION_ROOT", str(tmp_path))
    workspace = CalibrationWorkspace(SimpleNamespace(panel=SimpleNamespace(settings=SimpleNamespace())), lambda: None)
    try:
        assert workspace.calibration_quality_values is workspace._quality.calibration_quality_values
        assert workspace.calibration_selected_files_table is workspace._inputs.calibration_selected_files_table
        assert workspace.simple_calibration_previews is workspace._preview.simple_calibration_previews
        assert workspace.zoom_level_spin is workspace._capture.zoom_level_spin
        assert "_calibration_quality_state" not in vars(workspace)
        assert "_calibration_pair_picker_thumbnail_generation" not in vars(workspace)
        for part in (workspace._capture, workspace._preview, workspace._inputs, workspace._quality, workspace._validation):
            assert not {"backend", "panel", "workspace", "_evidence_reader"} & vars(part).keys()
        assert [timer.interval() for timer in workspace.view_timers()] == [500, 1000, 700]
        assert len({id(timer) for timer in workspace.view_timers()}) == 3
        for timer in workspace.view_timers():
            timer.start()
        workspace.dispose_view_resources()
        assert all(not timer.isActive() for timer in workspace.view_timers())
    finally:
        workspace.dispose_view_resources()
        workspace.deleteLater()


class PreviewProducer(QtCore.QObject):
    frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)
    detection_changed = QtCore.pyqtSignal(str, object)
    stats_changed = QtCore.pyqtSignal(str, object)
    cleared = QtCore.pyqtSignal()
    overlay_changed = QtCore.pyqtSignal(object)


def test_preview_disposal_fences_queued_signals_and_keeps_other_consumers(monkeypatch, workstation_qapplication):
    parent = QtWidgets.QWidget()
    producer = PreviewProducer()
    preview = CalibrationPreview(
        parent, channels=(PreviewChannel("left", producer.frame_changed,
                                        producer.detection_changed, producer.stats_changed),),
        clear_signal=producer.cleared, overlay_signal=producer.overlay_changed,
    )
    preview.camera_mode_tabs.setParent(parent)
    calls, other = [], []
    image = QtGui.QImage(8, 6, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("red"))
    producer.frame_changed.connect(lambda *_args: other.append("frame"))
    producer.detection_changed.connect(lambda *_args: other.append("detection"))
    producer.stats_changed.connect(lambda *_args: other.append("stats"))
    try:
        preview._connect_camera_panes()
        preview._connect_calibration_preview_signal()
        for method in ("set_frame", "set_detection", "set_metadata"):
            monkeypatch.setattr(preview.previews["left"], method,
                                lambda *_args, method=method: calls.append((method, QtCore.QThread.currentThread())))
        producer.frame_changed.emit("left", image, {})
        producer.detection_changed.emit("left", {})
        producer.stats_changed.emit("left", {})
        assert [call[0] for call in calls] == ["set_frame", "set_detection", "set_metadata"]
        assert all(call[1] is preview.thread() for call in calls)
        receiver = preview._side_receivers[0]
        QtCore.QMetaObject.invokeMethod(receiver, "frame", QtCore.Qt.QueuedConnection,
                                       QtCore.Q_ARG(str, "left"), QtCore.Q_ARG(QtGui.QImage, image), QtCore.Q_ARG(object, {}))
        for method in ("detection", "stats"):
            QtCore.QMetaObject.invokeMethod(receiver, method, QtCore.Qt.QueuedConnection,
                                           QtCore.Q_ARG(str, "left"), QtCore.Q_ARG(object, {}))
        QtCore.QMetaObject.invokeMethod(preview, "_on_calibration_preview_changed", QtCore.Qt.QueuedConnection,
                                       QtCore.Q_ARG(object, {"camera": "left", "image": image, "kind": "overlay"}))
        QtCore.QMetaObject.invokeMethod(preview, "_clear_camera_previews", QtCore.Qt.QueuedConnection)
        preview.stop()
        # Delete the widgets while the receiver still exists, then consume the
        # old queued events. The disposed owner must not touch those widgets.
        preview.camera_mode_tabs.deleteLater()
        workstation_qapplication.sendPostedEvents(preview.camera_mode_tabs, QtCore.QEvent.DeferredDelete)
        workstation_qapplication.processEvents()
        producer.frame_changed.emit("left", image, {})
        producer.detection_changed.emit("left", {})
        producer.stats_changed.emit("left", {})
        producer.cleared.emit()
        producer.overlay_changed.emit({"camera": "left", "image": image})
        assert len(calls) == 3
        assert other == ["frame", "detection", "stats"] * 2
        assert not preview._signal_connections
        assert not any(timer.isActive() for timer in (preview._checkerboard_preview_timer, preview._ftp_preview_timer))
    finally:
        preview.stop()
        parent.deleteLater()
        producer.deleteLater()


def test_calibration_part_boundary_requires_named_readers():
    with pytest.raises(TypeError, match="setup and apply-command readers"):
        CalibrationInputPicker(read_setup=None)
    with pytest.raises(TypeError, match="refresh-command reader"):
        CalibrationPreview(preview_command=None)


def test_outlier_io_keeps_pair_membership_and_portable_paths(tmp_path):
    paths = _write_images(tmp_path, count=1)
    rows = tuple({"side": side, "path": str(files[0]), "capture_key": "000"} for side, files in paths.items())
    selection = {"root": ".", "manual_mode": True, "left_paths": ["left/left_000.png"], "right_paths": ["right/right_000.png"]}
    changed, _message = write_outlier_selection(
        tmp_path, selection, rows,
        {"scope": "pair", "side": "left", "path": str(paths["left"][0]), "capture_key": "000"},
        restore=False,
    )
    assert changed
    saved = json.loads((tmp_path / "calibration_pair_selection.json").read_text())
    assert saved["excluded_pair_keys"] == ["000"]
    assert saved["excluded_left_paths"] == ["left/left_000.png"]
    assert saved["excluded_right_paths"] == ["right/right_000.png"]
    assert all(path.exists() for side in paths.values() for path in side)
