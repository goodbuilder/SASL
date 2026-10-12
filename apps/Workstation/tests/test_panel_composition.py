"""Independent ownership, callback affinity and resource evidence after G2."""
from __future__ import annotations

from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtTest, QtWidgets

from Workstation import standalone_gui
from Workstation.app_lifecycle_controller import AppLifecycleController
from Workstation.panel_application import FEATURE_TYPES
from Workstation.panel_contracts import FIELD_OWNERS, METHOD_OWNERS
from Workstation.standalone_backend import StandalonePanelBackend


@pytest.fixture
def panels(monkeypatch, tmp_path):
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    created = []
    def create():
        panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path / str(len(created)))
        created.append(panel)
        return panel
    yield create
    for panel in created:
        panel.close()
        panel.deleteLater()
    QtWidgets.QApplication.instance().processEvents()


def test_panel_has_one_feature_owner_per_value_and_no_cross_instance_state(panels):
    first, second = panels(), panels()
    camera = first.application.features["camera"]
    calibration = first.application.features["calibration"]
    first.camera_connected = True
    assert camera.state.camera_connected is True
    assert second.camera_connected is False
    assert "camera_connected" not in vars(first)
    assert "latest_calibration_preview_images" not in vars(first)
    assert first.latest_calibration_preview_images is calibration.state.latest_calibration_preview_images
    assert first.latest_calibration_preview_images is not second.latest_calibration_preview_images
    assert first.tabs is first.application.features["presentation"].views.tabs
    assert first.left_pane is not second.left_pane
    first._manual_recording_active = True
    assert "_manual_recording_active" not in vars(first)
    assert second._manual_recording_active is False


def test_feature_ports_do_not_grant_the_whole_application(panels):
    panel = panels()
    for kind in FEATURE_TYPES:
        feature = panel.application.features[kind.feature_name]
        assert not hasattr(feature, "panel")
        assert "__getattr__" not in kind.__dict__
        assert "__setattr__" not in kind.__dict__
        assert len(kind.dependency_names) < len(FIELD_OWNERS) // 2
        assert "not_a_declared_feature_field" not in feature.ports
    gnss = panel.application.features["gnss"]
    assert not hasattr(gnss, "_native_frame_async_stores")
    with pytest.raises(AttributeError):
        panel.application.command("not_a_command")
    with pytest.raises(AttributeError):
        panel.application.command("__getattribute__")


def test_cross_feature_field_reads_have_explicit_ports():
    """Changing a field owner must keep its existing readers wired at startup."""
    import ast
    import inspect
    owners = {**FIELD_OWNERS, **METHOD_OWNERS}
    for kind in FEATURE_TYPES:
        reads = {
            node.attr for node in ast.walk(ast.parse(inspect.getsource(kind)))
            if isinstance(node, ast.Attribute)
            and isinstance(node.value, ast.Name) and node.value.id == "self"
            and node.attr in owners
        }
        missing = {
            name for name in reads
            if owners[name] != kind.feature_name and name not in kind.dependency_names
        }
        assert not missing, (kind.feature_name, sorted(missing))


def test_optical_capture_fields_bind_lazily_to_actual_owner(panels):
    panel = panels()
    assert getattr(panel, "_optical_pose_controller_owner", None) is None
    assert panel.optical_pose_wiris_ftp_thread is None
    assert getattr(panel, "_optical_pose_controller_owner", None) is None
    controller = panel._optical_pose_controller()
    capture = controller.capture_owner
    thread = QtCore.QThread(panel)
    try:
        panel.optical_pose_wiris_ftp_thread = thread
        assert capture.optical_pose_wiris_ftp_thread is thread
        assert panel.application.resource_inventory()["optical_pose_wiris_ftp_thread"] is thread
        assert not panel.application.features["calibration"].state._optical_capture_bootstrap
        capture.optical_pose_wiris_ftp_thread = None
        assert panel.optical_pose_wiris_ftp_thread is None
    finally:
        capture.optical_pose_wiris_ftp_thread = None
        thread.deleteLater()


def test_query_inventory_preserves_multiple_runtime_threads(panels):
    panel = panels()
    class Runtime(QtCore.QObject):
        def __init__(self):
            super().__init__(panel)
            self.thread = QtCore.QThread(self)

        def resource_inventory(self):
            return {"thread": self.thread}

        def request_shutdown(self):
            return True

    first, second = Runtime(), Runtime()
    panel.application.register_evidence_query_runtime(first)
    panel.application.register_evidence_query_runtime(second)
    panel.application.register_evidence_query_runtime(first)
    resources = panel.application.resource_inventory()
    assert resources["evidence_query_0_thread"] is first.thread
    assert resources["evidence_query_1_thread"] is second.thread
    assert panel.application.evidence_query_runtimes == [first, second]


def test_pixhawk_connection_widgets_and_candidates_have_one_diagnostics_owner(panels):
    from Workstation.device_runtime.connection_view import PIXHAWK_CONNECTION_VIEW_FIELDS
    panel = panels()
    diagnostics = panel.application.features["diagnostics"]
    camera = panel.application.features["camera"]
    connection = diagnostics.views.connection
    for name in PIXHAWK_CONNECTION_VIEW_FIELDS:
        assert FIELD_OWNERS[name] == "diagnostics", name
        assert getattr(panel, name) is getattr(connection, name), name
        assert name not in camera.dependency_names, name
    panel._set_pixhawk_candidate_rows([
        {"kind": "mavlink", "camera": "left", "url": "udp:test-only", "status": "candidate"}
    ], source="mavlink")
    assert panel._pixhawk_candidate_rows_by_source is connection._pixhawk_candidate_rows_by_source
    assert panel._pixhawk_candidate_rows_by_source["mavlink"][0]["url"] == "udp:test-only"
    panel.clear_pixhawk_candidate_history("mavlink")
    assert panel._pixhawk_candidate_rows_by_source["mavlink"] == []


def test_backend_and_legacy_override_use_the_same_feature_command(panels, monkeypatch):
    panel = panels()
    seen = []
    monkeypatch.setattr(panel, "check_cameras", lambda: seen.append("camera"))
    StandalonePanelBackend(panel).camera_check_rtsp()
    assert seen == ["camera"]
    assert panel.check_cameras is panel.application.features["camera"].check_cameras
    assert "check_cameras" not in vars(panel)


def test_hidden_feature_thread_is_visible_to_shutdown_resource_check(panels):
    panel = panels()
    thread = QtCore.QThread(panel)
    camera = panel.application.features["camera"]
    camera.camera_auto_detect_thread = thread
    thread.start()
    try:
        QtTest.QTest.qWait(10)
        assert thread.isRunning()
        assert "camera_auto_detect_thread" not in vars(panel)
        assert panel.application.resource_inventory()["camera_auto_detect_thread"] is thread
        report = AppLifecycleController(panel)._shutdown_resource_snapshot()
        assert report["complete"] is False
        assert "camera_auto_detect_thread" in report["running_threads"]
    finally:
        thread.quit()
        assert thread.wait(2000)
        camera.camera_auto_detect_thread = None
        thread.deleteLater()


def test_worker_signal_executes_moved_feature_callback_on_gui_thread(panels, monkeypatch):
    panel = panels()
    feature = panel.application.features["lifecycle"]
    observed = []
    monkeypatch.setattr(panel, "_log", lambda text: observed.append((text, QtCore.QThread.currentThread())))

    class Worker(QtCore.QObject):
        ready = QtCore.pyqtSignal(str, str)
        @QtCore.pyqtSlot()
        def run(self):
            self.ready.emit("test-interface", "ready")

    thread = QtCore.QThread(panel)
    worker = Worker()
    worker.moveToThread(thread)
    thread.started.connect(worker.run)
    worker.ready.connect(feature._apply_interface_status)
    thread.finished.connect(worker.deleteLater)
    thread.start()
    try:
        for _ in range(100):
            if observed:
                break
            QtTest.QTest.qWait(5)
        assert observed
        assert observed[0][1] is QtWidgets.QApplication.instance().thread()
        assert feature.parent() is panel
        assert panel.application.features["camera"]._camera_sync_timer.parent() is panel
    finally:
        thread.quit()
        assert thread.wait(2000)
        thread.deleteLater()


def test_inventory_includes_measurement_actor_and_recording_native_owners(panels):
    panel = panels()
    measurement = panel._measurement_view_controller()
    recording = panel._recording_controller()
    state = panel._measurement_feature_state
    timer = QtCore.QTimer(panel)
    queue = object()
    actor = SimpleNamespace(is_alive=lambda: True)
    coordinator = SimpleNamespace(_thread=actor, _input=queue, _results=object())
    writers = {"owned-native-writer": object()}
    state.capture._detection_pair_timer = timer
    state.capture._detection_pair_coordinator = coordinator
    native = recording.recording_state.native_state
    previous = getattr(native, "_native_evidence_writers", None)
    native._native_evidence_writers = writers
    try:
        inventory = panel.application.resource_inventory()
        assert inventory["pairing.timer"] is timer
        assert inventory["pairing.coordinator"] is coordinator
        assert inventory["pairing.input"] is queue
        assert inventory["_native_evidence_writers"] is writers
        assert inventory["pairing.worker_thread"] is actor
        report = AppLifecycleController(panel)._shutdown_resource_snapshot()
        assert "pairing.worker_thread" in report["running_threads"]
        assert report["complete"] is False
    finally:
        state.capture._detection_pair_timer = None
        state.capture._detection_pair_coordinator = None
        native._native_evidence_writers = previous
        timer.deleteLater()


def test_inventory_includes_pixhawk_transport_without_duplicating_connection(panels):
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    panel = panels()
    previous = panel.mavlink_pixhawk
    subscriber = DualPixhawkMavlinkSubscriber(left_url="left", right_url="")
    closed = []
    connection = SimpleNamespace(close=lambda: closed.append("left"))
    subscriber.connections["left"] = connection
    panel.mavlink_pixhawk = subscriber
    try:
        inventory = panel.application.resource_inventory()
        assert inventory["pixhawk.left.connection"] is connection
        report = AppLifecycleController(panel)._shutdown_resource_snapshot()
        assert "mavlink:left" in report["open_sockets"]
        subscriber.stop("left")
        assert "pixhawk.left.connection" not in panel.application.resource_inventory()
        assert closed == ["left"]
    finally:
        subscriber.stop()
        panel.mavlink_pixhawk = previous
        subscriber.deleteLater()


@pytest.mark.parametrize(("feature_name", "command_names"), [
    ("recording_view", ("allocate_measurement_video_segment", "measurement_storage_context", "data_storage_snapshot")),
    ("camera", ("_camera_stream_start_status", "allocate_measurement_video_segment",
                "measurement_source_transition", "native_frame_persistence_pending_count",
                "record_measurement_video_segment_presence", "start_automatic_session_recording")),
    ("presentation", ("append_workstation_data",)),
    ("calibration", ("_decorate_clock_epoch_metadata",)),
    ("diagnostics", ("_update_mag_sensor",)),
])
def test_optional_controller_callbacks_remain_bound_in_feature_ports(panels, feature_name, command_names):
    """A missing optional callback must not silently disable recording/evidence."""
    feature = panels().application.features[feature_name]
    for name in command_names:
        assert callable(getattr(feature, name, None)), (feature_name, name)
        with pytest.raises(AttributeError, match="read-only"):
            setattr(feature, name, lambda: None)


def test_feature_data_never_remains_on_the_qobject_after_composition(panels):
    panel = panels()
    for name, feature in panel.application.features.items():
        # Named test/embedding command overrides are supported, but construction
        # itself must place every widget, mutable value and handle in a bundle.
        assert set(vars(feature)) == {"widget", "state", "views", "ports"}, name


def test_px4_tab_registry_keeps_owned_views_and_legacy_aliases(panels):
    panel = panels()
    diagnostics = panel.application.features["diagnostics"]
    tabs = diagnostics.views.px4_mag_tab_widgets
    assert panel.px4_mag_tab_widget_manager is tabs["manager"]
    assert panel.px4_mag_tab_widget_detail is tabs["detail"]
    assert tabs["manager"] is not tabs["detail"]
    assert "px4_mag_tab_widget_manager" not in vars(diagnostics)
    extra_view = QtWidgets.QWidget(panel)
    layout = QtWidgets.QVBoxLayout(extra_view)
    diagnostics._build_px4_mag_detail_panel(layout, panel_key="additional-view")
    assert "additional-view" in tabs
    assert "px4_mag_tab_widget_additional-view" not in vars(diagnostics)


def _outer_facade_imports(source):
    import ast

    forbidden = {
        "Workstation.standalone_gui", "Workstation.workspaces",
        "Workstation.recording_controller", "Workstation.measurement_view_controller",
    }
    found = []
    for node in ast.walk(ast.parse(source)):
        names = []
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            if node.level == 1:
                module = "Workstation" + ("." + module if module else "")
            names = [module, *(module + "." + alias.name for alias in node.names)]
        found.extend(name for name in names if name in forbidden)
    return found


@pytest.mark.parametrize("source", (
    "from Workstation.recording_controller import helper",
    "import Workstation.measurement_view_controller as legacy",
    "from Workstation import standalone_gui",
    "from .workspaces import View",
    "from . import recording_controller",
))
def test_feature_facade_import_guard_recognizes_import_forms(source):
    assert _outer_facade_imports(source)
    assert not _outer_facade_imports("from Workstation.recording.contracts import helper")


def test_panel_features_import_concrete_owners_instead_of_outer_facades():
    import inspect

    for kind in FEATURE_TYPES:
        module = inspect.getmodule(kind)
        assert module is not None
        assert not _outer_facade_imports(inspect.getsource(module)), module.__name__


def _feature_receiver_writes(method):
    """Inspect one receiver's writes without borrowing nested classes' self."""
    import ast

    class Visitor(ast.NodeVisitor):
        def __init__(self):
            self.literal = set()
            self.dynamic = []

        def visit_ClassDef(self, node):
            return

        def visit_FunctionDef(self, node):
            arguments = node.args.posonlyargs + node.args.args + node.args.kwonlyargs
            if not any(argument.arg == "self" for argument in arguments):
                self.generic_visit(node)

        visit_AsyncFunctionDef = visit_FunctionDef
        visit_Lambda = visit_FunctionDef

        def visit_Attribute(self, node):
            if (isinstance(node.value, ast.Name) and node.value.id == "self"
                    and isinstance(node.ctx, (ast.Store, ast.Del))):
                self.literal.add(node.attr)
            self.generic_visit(node)

        def visit_Call(self, node):
            if (isinstance(node.func, ast.Name) and node.func.id in {"setattr", "delattr"}
                    and len(node.args) > 1 and isinstance(node.args[0], ast.Name)
                    and node.args[0].id == "self"):
                target = node.args[1]
                if isinstance(target, ast.Constant) and isinstance(target.value, str):
                    self.literal.add(target.value)
                else:
                    self.dynamic.append(target)
            self.generic_visit(node)

    visitor = Visitor()
    for statement in method.body:
        visitor.visit(statement)
    return visitor.literal, visitor.dynamic


def test_write_audit_distinguishes_nested_receivers_and_captured_self():
    import ast
    method = ast.parse('''
def action(self):
    self.outer = []
    setattr(self, "literal", None)
    class Worker:
        def __init__(self):
            self.worker_thread = None
    def captured():
        self.captured_state = {}
    def separate(self):
        self.other_owner = {}
''').body[0]
    fields, dynamic = _feature_receiver_writes(method)
    assert fields == {"outer", "literal", "captured_state"}
    assert dynamic == []


def test_feature_writes_require_owned_storage_or_explicit_writable_ports():
    import ast
    import inspect
    from Workstation.panel_application import EXTERNAL_STATE_FIELDS

    for kind in FEATURE_TYPES:
        definition = ast.parse(inspect.getsource(kind)).body[0]
        for method in definition.body:
            if not isinstance(method, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            fields, dynamic = _feature_receiver_writes(method)
            if method.name == "__init__":
                fields.discard("views")  # A feature may select its typed view bundle.
            if dynamic:
                # The sole current dynamic write restores this explicit frozen
                # list after a rejected calibration apply. New computed writes
                # require an ownership review instead of escaping this guard.
                assert (kind.feature_name, method.name) == ("calibration", "_auto_load_intrinsics")
                assert all(isinstance(name, ast.Name) and name.id == "name" for name in dynamic)
                names = next(statement.value for statement in method.body
                             if isinstance(statement, ast.Assign)
                             and any(isinstance(target, ast.Name) and target.id == "state_names"
                                     for target in statement.targets))
                fields.update(ast.literal_eval(names))
            for name in fields:
                owner = FIELD_OWNERS.get(name)
                if owner == kind.feature_name:
                    continue
                evidence = (kind.feature_name, method.name, name, owner)
                assert owner is not None or name in EXTERNAL_STATE_FIELDS, evidence
                assert name in kind.dependency_names, evidence
                assert name in kind.writable_dependencies, evidence
