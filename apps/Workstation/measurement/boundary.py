"""Qt/legacy boundary for Measurement; service code never receives this object."""

from __future__ import annotations
from functools import partial
from types import SimpleNamespace
from typing import Any, Callable
from PyQt5 import QtCore
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.optical_inputs import OpticalMeasurementInputs
from Workstation.measurement.state import (
    MEASUREMENT_PANEL_FIELDS,
    UNSET,
    measurement_state_for,
)


class MeasurementBoundary:
    def __init__(self, host: object) -> None:
        self._host = host
        self.state = measurement_state_for(host)
        self._legacy = isinstance(host, SimpleNamespace)
        self.refresh_legacy_state()

    def refresh_legacy_state(self) -> None:
        # Production compatibility properties address the same domain objects.
        # Descriptor-free embedders only are copied at the outer call boundary.
        if not self._legacy:
            return
        for name, domain in MEASUREMENT_PANEL_FIELDS.items():
            if hasattr(self._host, name):
                setattr(getattr(self.state, domain), name, getattr(self._host, name))

    def publish_legacy_state(self) -> None:
        if not self._legacy:
            return
        for name, domain in MEASUREMENT_PANEL_FIELDS.items():
            value = getattr(getattr(self.state, domain), name)
            if value is not UNSET:
                setattr(self._host, name, value)

    def invoke(self, method: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
        self.refresh_legacy_state()
        try:
            return method(*args, **kwargs)
        finally:
            self.publish_legacy_state()

    def value(self, name: str, *, writable: bool = False) -> ValueInput:
        return ValueInput(
            lambda default: getattr(self._host, name, default),
            (lambda value: setattr(self._host, name, value)) if writable else None,
        )

    def optical_inputs(self) -> OpticalMeasurementInputs:
        """Bind only enumerated optical evidence operations; never return its owner."""

        def source(name: str) -> ValueInput:
            def read_value(default: Any) -> Any:
                factory = getattr(self._host, "_optical_pose_controller", None)
                owner = factory() if callable(factory) else None
                evidence_fields = {
                    "_active_selection": "active_selection",
                    "_candidates": "candidates",
                    "_candidate_priors": "candidate_priors",
                    "_candidate_drone_rotation_context": "candidate_drone_rotation_context",
                    "_last_accepted_auto_suggestion": "last_accepted_auto_suggestion",
                }
                reader = getattr(owner, "workstation_optical_pose_measurement_evidence", None)
                if name in evidence_fields and callable(reader):
                    return reader().get(evidence_fields[name], default)
                return getattr(owner, name, default)

            return ValueInput(read_value)

        return OpticalMeasurementInputs(
            read_intrinsics_snapshot=source("_intrinsics_snapshot"),
            read_operating_intrinsics_profile_for_geometry=source(
                "operating_intrinsics_profile_for_geometry"
            ),
            read_operating_intrinsics_profile=source("_operating_intrinsics_profile"),
            read_active_selection=source("_active_selection"),
            read_candidate_mapping_with_prior=source("_candidate_mapping_with_prior"),
            read_candidates=source("_candidates"),
            read_candidate_priors=source("_candidate_priors"),
            read_candidate_drone_rotation_context=source(
                "_candidate_drone_rotation_context"
            ),
            read_last_accepted_auto_suggestion=source("_last_accepted_auto_suggestion"),
            read_workstation_optical_pose_snapshot=source(
                "workstation_optical_pose_snapshot"
            ),
        )

    def release_primary_retry(self) -> ValueInput:
        return ValueInput(
            lambda default: getattr(
                getattr(self._host, "_recording_controller_owner", None),
                "_release_primary_retry_ownership",
                default,
            )
        )

    def persist_compact_primary(self) -> ValueInput:
        return ValueInput(
            lambda default: getattr(
                getattr(self._host, "_recording_controller_owner", None),
                "_persist_compact_primary_measurement",
                default,
            )
        )

    def qt_parent(self) -> ValueInput:
        parent = self._host if isinstance(self._host, QtCore.QObject) else None
        return ValueInput(lambda default: parent)

    def camera_pane(self, side: str) -> object | None:
        if side not in ("left", "right"):
            raise ValueError("camera side must be left or right")
        return getattr(self._host, f"{side}_pane", None)

    def widget_value(self, widget: str, method: str) -> Callable[[], Any]:
        def value() -> Any:
            control = getattr(self._host, widget, None)
            if control is None and widget == "baseline_spin":
                return None
            return getattr(control, method)()

        return value

    def optional_action(self, name: str, target: Callable[..., Any]) -> ValueInput:
        # The optional callback is sampled when invoked, so legacy embedders can
        # bind a screen after construction. Production always calls the service.
        return ValueInput(
            lambda default: (
                getattr(self._host, name, default) if self._legacy else target
            )
        )


class service_method:
    """One explicitly declared compatibility entrypoint, never wildcard routing."""

    def __init__(self, domain: str, name: str, service_type: type) -> None:
        self.domain = domain
        self.name = name
        self.service_type = service_type

    def __get__(self, controller: object | None, owner: type | None = None) -> Any:
        if controller is None:
            return getattr(self.service_type, self.name)
        service = getattr(controller.services, self.domain)
        return partial(controller.boundary.invoke, getattr(service, self.name))

    def __set__(self, controller: object, value: Any) -> None:
        setattr(getattr(controller.services, self.domain), self.name, value)


def service_state(domain: str, name: str) -> property:
    def get(controller: object) -> Any:
        return getattr(getattr(controller.boundary.state, domain), name)

    def set_value(controller: object, value: Any) -> None:
        setattr(getattr(controller.boundary.state, domain), name, value)

    return property(get, set_value)
