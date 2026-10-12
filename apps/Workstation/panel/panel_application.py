"""Application composition for the standalone Workstation panel.

The QWidget is an outer Qt/legacy boundary. Feature instances own their state,
views and behavior; named ports declare the remaining cross-feature contracts.
"""
from __future__ import annotations

from functools import wraps
from PyQt5 import QtCore

from Workstation.panel.panel_contracts import FIELD_OWNERS, METHOD_OWNERS, VIEW_FIELDS
from Workstation.panel.panel_controller_bindings import _CONTROLLER_DELEGATE_BINDINGS
from Workstation.panel.panel_feature_runtime import DependencyField, FeaturePort, OwnedField
from Workstation.panel.panel_settings import SettingsFeature
from Workstation.panel.panel_lifecycle import LifecycleFeature
from Workstation.panel.panel_presentation import PresentationFeature
from Workstation.panel.panel_camera import CameraFeature
from Workstation.panel.panel_detection import DetectionFeature
from Workstation.panel.panel_alignment import AlignmentFeature
from Workstation.panel.panel_gnss import GnssFeature
from Workstation.panel.panel_diagnostics import DiagnosticsFeature
from Workstation.panel.panel_calibration import CalibrationFeature
from Workstation.panel.panel_geometry import GeometryFeature
from Workstation.panel.panel_recording_view import RecordingViewFeature
from Workstation.panel.panel_view_composition import build_panel_views
from Workstation.measurement.state import MEASUREMENT_PANEL_FIELDS, measurement_state_property
from Workstation.recording.state import RECORDING_STATE_FIELDS
from Workstation.workstation_core.active_geometry import ActiveGeometryStore
from Workstation.camera_runtime.readiness import StreamReadinessTracker

FEATURE_TYPES = (
    SettingsFeature, LifecycleFeature, PresentationFeature, CameraFeature,
    DetectionFeature, AlignmentFeature, GnssFeature, DiagnosticsFeature,
    CalibrationFeature, GeometryFeature, RecordingViewFeature,
)
EXTERNAL_STATE_FIELDS = frozenset(MEASUREMENT_PANEL_FIELDS) | frozenset(RECORDING_STATE_FIELDS)
CONTROLLER_COMMANDS = frozenset(
    name for binding in _CONTROLLER_DELEGATE_BINDINGS for name in binding.method_names
)


def _legacy_state_property(name):
    def read(panel):
        return panel.application.read_field(name)
    def write(panel, value):
        panel.application.write_field(name, value)
    def delete(panel):
        panel.application.delete_field(name)
    return property(read, write, delete)


class FeatureCommand:
    """Named legacy method bridge, including supported instance overrides."""
    def __init__(self, owner_name, name, method):
        self.owner_name, self.name, self.method = owner_name, name, method
        @wraps(method)
        def invoke(panel, *args, **kwargs):
            application = getattr(panel, "application", None)
            if application is None:
                return method(panel, *args, **kwargs)
            return getattr(application.features[owner_name], name)(*args, **kwargs)
        self.invoke = invoke

    def __get__(self, panel, panel_type=None):
        if panel is None:
            return self.invoke
        return getattr(panel.application.features[self.owner_name], self.name)

    def __set__(self, panel, value):
        setattr(panel.application.features[self.owner_name], self.name, value)

    def __delete__(self, panel):
        delattr(panel.application.features[self.owner_name], self.name)


def _feature_delegate(owner_name, name, descriptor):
    if isinstance(descriptor, staticmethod):
        return staticmethod(descriptor.__func__)
    return FeatureCommand(owner_name, name, descriptor)


def install_panel_compatibility(panel_type):
    """Install a finite API; never intercept arbitrary attribute access."""
    for name in FIELD_OWNERS:
        if name not in EXTERNAL_STATE_FIELDS:
            setattr(panel_type, name, _legacy_state_property(name))
    for name in MEASUREMENT_PANEL_FIELDS:
        setattr(panel_type, name, measurement_state_property(name))
    from Workstation.recording.binding import install_recording_state_properties
    install_recording_state_properties(panel_type)
    for name, owner_name in METHOD_OWNERS.items():
        feature_type = next(t for t in FEATURE_TYPES if t.feature_name == owner_name)
        setattr(panel_type, name, _feature_delegate(owner_name, name, feature_type.__dict__[name]))


class PanelApplication:
    def __init__(self, widget):
        self.widget = widget
        self.optical_auto_runtimes = []
        self.evidence_query_runtimes = []
        self.geometry_store = ActiveGeometryStore()
        self.stream_readiness = StreamReadinessTracker()
        def build_feature(kind):
            if kind is CameraFeature:
                return kind(widget, geometry_store=self.geometry_store, stream_readiness=self.stream_readiness)
            if kind is CalibrationFeature:
                return kind(widget, geometry_store=self.geometry_store)
            if kind is PresentationFeature:
                return kind(widget, stream_readiness=self.stream_readiness)
            return kind(widget)
        self.features = {kind.feature_name: build_feature(kind) for kind in FEATURE_TYPES}
        self._bind_ports()
        self.features["calibration"].state.bind_optical_capture_owner(
            lambda: getattr(getattr(self.widget, "_optical_pose_controller_owner", None), "capture_owner", None)
        )
        # Recording state exists before any constructor field is assigned; the
        # controller later reuses this exact owner rather than migrating a copy.
        from Workstation.recording.binding import recording_state_for
        recording_state_for(widget)

    def read_field(self, name):
        owner = self.features[FIELD_OWNERS[name]]
        return getattr(owner.views if name in VIEW_FIELDS else owner.state, name)

    def write_field(self, name, value):
        owner = self.features[FIELD_OWNERS[name]]
        setattr(owner.views if name in VIEW_FIELDS else owner.state, name, value)

    def delete_field(self, name):
        owner = self.features[FIELD_OWNERS[name]]
        delattr(owner.views if name in VIEW_FIELDS else owner.state, name)

    def command(self, name):
        """Resolve only commands registered at the public compatibility boundary."""
        if name in METHOD_OWNERS:
            return getattr(self.features[METHOD_OWNERS[name]], name)
        if name not in CONTROLLER_COMMANDS:
            raise AttributeError(name)
        method = getattr(self.widget, name, None)
        if not callable(method):
            raise AttributeError(name)
        return method

    def _bind_ports(self):
        for feature in self.features.values():
            kind = type(feature)
            owned = {name for name, owner in FIELD_OWNERS.items() if owner == feature.feature_name}
            for name in owned - EXTERNAL_STATE_FIELDS:
                if name not in kind.__dict__:
                    setattr(kind, name, OwnedField(name, view=name in VIEW_FIELDS))
            dependencies = set(feature.dependency_names) | (owned & EXTERNAL_STATE_FIELDS)
            for name in dependencies:
                if name in EXTERNAL_STATE_FIELDS:
                    port = FeaturePort(
                        lambda name=name: getattr(self.widget, name),
                        lambda value, name=name: setattr(self.widget, name, value),
                    )
                elif name in FIELD_OWNERS:
                    port = FeaturePort(
                        lambda name=name: self.read_field(name),
                        lambda value, name=name: self.write_field(name, value),
                    )
                elif name in METHOD_OWNERS:
                    port = FeaturePort(lambda name=name: self.command(name))
                else:
                    # Named Qt signals/methods and existing controller commands.
                    port = FeaturePort(lambda name=name: getattr(self.widget, name))
                if name not in feature.writable_dependencies and name not in owned:
                    port = FeaturePort(port.read)
                feature.ports[name] = port
                if name not in kind.__dict__:
                    setattr(kind, name, DependencyField(name))

    def initialize(self, *, auto_start=False, auto_start_pixhawk=False,
                   app_root=None, settings_path=None, use_default_roots=False):
        options = dict(auto_start=auto_start, auto_start_pixhawk=auto_start_pixhawk,
                       app_root=app_root, settings_path=settings_path)
        self.features['settings'].initialize_state(**options)
        if use_default_roots:
            from Workstation.runtime_paths import WORKSTATION_CALIBRATION_ROOT, WORKSTATION_MEASUREMENT_DATA_ROOT
            self.features['settings'].default_calibration_root = WORKSTATION_CALIBRATION_ROOT
            self.features['settings'].default_measurement_data_root = WORKSTATION_MEASUREMENT_DATA_ROOT
        for name in ('gnss', 'diagnostics', 'calibration', 'detection', 'geometry',
                     'alignment', 'camera', 'recording_view', 'presentation', 'lifecycle'):
            self.features[name].initialize_state(**options)
        build_panel_views(self)
        # The composition root preserves embedding/startup override hooks.
        QtCore.QTimer.singleShot(0, self.widget._initial_status)
        QtCore.QTimer.singleShot(0, self.widget._auto_load_intrinsics)

    def endpoint_usage_snapshot(self):
        """Composition-owned projection; consumers do not inspect feature widgets."""
        panel = self.widget
        endpoints = []
        subscriber = getattr(panel, "mavlink_pixhawk", None)
        if subscriber is not None:
            urls = getattr(subscriber, "urls", {})
            sides = set(getattr(subscriber, "connections", {}))
            if getattr(panel, "pixhawk_connection_thread", None) is not None:
                sides.update(urls)
            for side in sides:
                endpoints.append({"endpoint":str(urls.get(side, "")), "owner":f"Pixhawk/{side}"})
        base = getattr(panel, "gnss_base_serial_worker", None)
        if base is not None:
            endpoints.append({"endpoint":str(getattr(base, "port", "")), "owner":"Base GNSS"})
        scans = ("pixhawk_usb_discovery_thread", "pixhawk_network_discovery_thread",
                 "drone_telemetry_usb_discovery_thread", "drone_telemetry_network_discovery_thread")
        return {"endpoints":endpoints,
                "discovery_busy":any(getattr(panel, name, None) is not None for name in scans)}

    def register_optical_auto_runtime(self, runtime):
        if runtime in self.optical_auto_runtimes:
            return
        self.optical_auto_runtimes.append(runtime)
        def unregister(_object=None):
            if runtime in self.optical_auto_runtimes:
                self.optical_auto_runtimes.remove(runtime)
        runtime.destroyed.connect(unregister)

    def register_evidence_query_runtime(self, runtime):
        if runtime in self.evidence_query_runtimes:
            return
        self.evidence_query_runtimes.append(runtime)
        def unregister(_object=None):
            if runtime in self.evidence_query_runtimes:
                self.evidence_query_runtimes.remove(runtime)
        runtime.destroyed.connect(unregister)

    def resource_inventory(self):
        resources = {}
        for feature in self.features.values():
            resources.update(feature.resource_inventory())
        for runtime in tuple(self.optical_auto_runtimes):
            resources.update(runtime.resource_inventory())
        for index, runtime in enumerate(tuple(self.evidence_query_runtimes)):
            resources.update({f"evidence_query_{index}_{name}": value for name, value in runtime.resource_inventory().items()})
        # Service inventories include queues, native stores and subprocesses
        # owned below the compatibility fields. Do not instantiate services just
        # to inspect an application which never started them.
        for accessor in ("_measurement_view_controller_owner", "_recording_controller_owner", "_target_drone_controller_owner", "_gnss_runtime_controller_owner", "_optical_pose_controller_owner", "_calibration_workflow_controller_owner", "_camera_stream_lifecycle_controller_owner", "mavlink_pixhawk"):
            controller = getattr(self.widget, accessor, None)
            inventory = getattr(controller, "resource_inventory", None)
            if callable(inventory):
                resources.update(inventory())
        # External measurement/recording state is another explicit owner, not a
        # second copied field store. Read initialized declared handles as well.
        for name in EXTERNAL_STATE_FIELDS:
            if name.endswith(("_thread", "_threads", "_worker", "_workers", "_timer", "_timers")):
                try:
                    resources[name] = getattr(self.widget, name)
                except AttributeError:
                    pass
        return resources
