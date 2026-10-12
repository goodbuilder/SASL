"""Application adapter for the finite recording compatibility contract.

Only this module accesses a panel. Production properties route directly into
recording state; namespace fixtures synchronize the same named fields at calls.
"""
from __future__ import annotations
from contextlib import contextmanager
from functools import wraps
from typing import Iterator
from .ports import FieldBinding
from .state import RECORDING_STATE_FIELDS, RecordingState

_MISSING = object()

def recording_state_for(panel: object) -> RecordingState:
    """Return the single recording owner, including during panel construction."""
    state = vars(panel).get("_recording_state")
    if state is None:
        state = RecordingState()
        for name in RECORDING_STATE_FIELDS:
            if name in vars(panel):
                setattr(state.state_for_field(name), name, vars(panel).pop(name))
        setattr(panel, "_recording_state", state)
    return state

def _recording_property(name: str) -> property:
    def read(panel):
        state = vars(panel).get("_recording_state")
        if state is None:
            value = vars(panel).get(name, _MISSING)
            if value is _MISSING:
                raise AttributeError(name)
            return value
        return getattr(state.state_for_field(name), name)
    def write(panel, value):
        state = vars(panel).get("_recording_state")
        if state is None:
            vars(panel)[name] = value
        else:
            setattr(state.state_for_field(name), name, value)
    def delete(panel):
        state = vars(panel).get("_recording_state")
        if state is None:
            if name not in vars(panel):
                raise AttributeError(name)
            del vars(panel)[name]
        else:
            delattr(state.state_for_field(name), name)
    return property(read, write, delete)


def install_recording_state_properties(panel_type: type) -> bool:
    """Install only the declared recording fields; immutable fixture types opt out."""
    if panel_type.__module__ in {"builtins", "types"}:
        return False
    for name in RECORDING_STATE_FIELDS:
        setattr(panel_type, name, _recording_property(name))
    return True


def bind_recording_panel_state(panel: object, state: RecordingState) -> bool:
    values = {name: getattr(panel, name, _MISSING) for name in RECORDING_STATE_FIELDS}
    installed = install_recording_state_properties(type(panel))
    for name, value in values.items():
        if value is not _MISSING:
            setattr(state.state_for_field(name), name, value)
    setattr(panel, "_recording_state", state)
    if installed:
        for name in RECORDING_STATE_FIELDS:
            vars(panel).pop(name, None)
    return installed


class RecordingPanelBinding:
    """Outer boundary only; never passed into a recording service."""
    def __init__(self, panel: object, state: RecordingState) -> None:
        self.panel = panel
        self.state = state
        self.properties_installed = bind_recording_panel_state(panel, state)
        self._depth = 0

    def import_fixture_fields(self) -> None:
        if self.properties_installed:
            return
        for name in RECORDING_STATE_FIELDS:
            owner = self.state.state_for_field(name)
            value = getattr(self.panel, name, _MISSING)
            if value is _MISSING:
                if hasattr(owner, name):
                    delattr(owner, name)
            else:
                setattr(owner, name, value)

    def publish_fixture_fields(self) -> None:
        if self.properties_installed:
            return
        for name in RECORDING_STATE_FIELDS:
            owner = self.state.state_for_field(name)
            value = getattr(owner, name, _MISSING)
            if value is _MISSING:
                if hasattr(self.panel, name):
                    delattr(self.panel, name)
            else:
                setattr(self.panel, name, value)

    @contextmanager
    def call_scope(self) -> Iterator[None]:
        if self._depth == 0:
            self.import_fixture_fields()
        self._depth += 1
        try:
            yield
        finally:
            self._depth -= 1
            if self._depth == 0:
                self.publish_fixture_fields()

    def field(self, name: str, *, writable: bool = False, deletable: bool = False) -> FieldBinding:
        """Build one declared external capability without exposing the host."""
        if name == "qobject_parent":
            return FieldBinding(lambda: self.panel)
        if name == "mavlink_link_provenance":
            from Workstation.application.evidence import mavlink_link_provenance
            return FieldBinding(
                lambda: lambda side: mavlink_link_provenance(self.panel, side),
            )
        if name in {"method_evaluation_pending_count", "target_drone_stream_snapshot", "stop_target_drone_stream"}:
            owner_name = (
                "_measurement_view_controller_owner"
                if name == "method_evaluation_pending_count"
                else "_target_drone_controller_owner"
            )
            return FieldBinding(lambda: getattr(getattr(self.panel, owner_name), name))
        if name in {"recording_store", "native_frames_path"}:
            def provide():
                owner_factory = getattr(self.panel, "_data_storage_controller", None)
                controller = owner_factory() if callable(owner_factory) else None
                store = getattr(controller, "store", None)
                if name == "recording_store":
                    return store
                path_for = getattr(store, "path_for", None)
                return path_for("native.frames") if callable(path_for) else None
            return FieldBinding(lambda: provide)
        if name in RECORDING_STATE_FIELDS:
            owner = self.state.state_for_field(name)
            return FieldBinding(
                lambda: getattr(owner, name),
                (lambda value: setattr(owner, name, value)) if writable else None,
                (lambda: delattr(owner, name)) if deletable else None,
            )
        def read():
            value = getattr(self.panel, name)
            # QObject-bound slots must retain their receiver/thread affinity.
            if self.properties_installed or not callable(value):
                return value
            @wraps(value)
            def boundary_call(*args, **kwargs):
                self.publish_fixture_fields()
                try:
                    return value(*args, **kwargs)
                finally:
                    self.import_fixture_fields()
            return boundary_call
        return FieldBinding(
            read,
            (lambda value: setattr(self.panel, name, value)) if writable else None,
            (lambda: delattr(self.panel, name)) if deletable else None,
        )

    def fixture_override(self, name: str):
        """Honor explicit legacy test/embedding callbacks, never panel delegates."""
        value = vars(self.panel).get(name)
        if value is None and not type(self.panel).__module__.endswith(".standalone_gui"):
            value = getattr(self.panel, name, None)
        if not callable(value):
            return None
        if getattr(value, "__module__", "").endswith(".controller_delegation"):
            return None
        return self.field(name).read()
