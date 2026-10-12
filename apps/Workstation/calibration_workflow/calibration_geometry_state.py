"""Outer feature bindings for the shared active-geometry value owner."""

from __future__ import annotations

from types import SimpleNamespace

from Workstation.workstation_core.active_geometry import (
    ActiveGeometryStore,
    CALIBRATION_GEOMETRY_FIELDS,
    CAMERA_GEOMETRY_FIELDS,
    GEOMETRY_FIELDS,
)


def _geometry_field(name: str) -> property:
    return property(
        lambda state: state.geometry_store.read_legacy(name),
        lambda state, value: state.geometry_store.write_legacy(name, value),
        lambda state: state.geometry_store.remove_legacy(name),
    )


class CalibrationGeometryState(SimpleNamespace):
    def __init__(self, store: ActiveGeometryStore) -> None:
        if not isinstance(store, ActiveGeometryStore):
            raise TypeError("Calibration state requires an ActiveGeometryStore")
        super().__init__(geometry_store=store)


class CameraGeometryState(SimpleNamespace):
    def __init__(self, store: ActiveGeometryStore) -> None:
        if not isinstance(store, ActiveGeometryStore):
            raise TypeError("Camera state requires an ActiveGeometryStore")
        super().__init__(geometry_store=store)


for _name in CALIBRATION_GEOMETRY_FIELDS:
    setattr(CalibrationGeometryState, _name, _geometry_field(_name))
for _name in CAMERA_GEOMETRY_FIELDS:
    setattr(CameraGeometryState, _name, _geometry_field(_name))
del _name


def _geometry_store_for(panel: object):
    application = getattr(panel, "application", None)
    store = getattr(application, "geometry_store", None)
    if store is None:
        store = getattr(getattr(panel, "state", None), "geometry_store", None)
    return store


def apply_geometry_values(panel: object, values: dict[str, object]) -> None:
    store = _geometry_store_for(panel)
    if isinstance(store, ActiveGeometryStore):
        store.update(values)
        return
    # Named legacy embedders lack the production feature state descriptors.
    for name, value in values.items():
        setattr(panel, name, value)


def restore_geometry_values(panel: object, attributes) -> None:
    store = _geometry_store_for(panel)
    owned = set(attributes) & GEOMETRY_FIELDS if isinstance(store, ActiveGeometryStore) else set()
    if owned:
        store.restore_fields({name: attributes[name] for name in owned})
    for name, (existed, value) in attributes.items():
        if name in owned:
            continue
        if existed:
            setattr(panel, name, value)
        elif hasattr(panel, name):
            delattr(panel, name)
