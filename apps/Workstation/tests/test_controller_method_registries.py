"""Validate the live controller bindings exposed by the panel."""

from __future__ import annotations

import pytest

from Workstation import standalone_gui


CONTROLLER_DELEGATE_BINDINGS = standalone_gui._CONTROLLER_DELEGATE_BINDINGS
_IDS = [binding.controller_cls.__name__ for binding in CONTROLLER_DELEGATE_BINDINGS]


@pytest.mark.parametrize("binding", CONTROLLER_DELEGATE_BINDINGS, ids=_IDS)
def test_controller_delegate_binding_contract(binding):
    controller_cls = binding.controller_cls
    method_names = tuple(binding.method_names)

    assert len(method_names) == len(set(method_names)), (
        f"{controller_cls.__name__} method registry contains duplicate names"
    )
    assert callable(getattr(standalone_gui.WirisStereoDistancePanel, binding.accessor_name, None)), (
        f"{controller_cls.__name__} accessor is missing: {binding.accessor_name}"
    )

    missing_controller_methods = [
        name for name in method_names if not callable(getattr(controller_cls, name, None))
    ]
    assert not missing_controller_methods, (
        f"{controller_cls.__name__} registry references missing/non-callable methods: "
        f"{missing_controller_methods}"
    )

    missing_panel_methods = [
        name
        for name in method_names
        if not callable(getattr(standalone_gui.WirisStereoDistancePanel, name, None))
    ]
    assert not missing_panel_methods, (
        f"{controller_cls.__name__} methods not bound on WirisStereoDistancePanel: {missing_panel_methods}"
    )


def test_controller_delegate_method_names_are_globally_unique():
    owners: dict[str, list[str]] = {}
    for binding in CONTROLLER_DELEGATE_BINDINGS:
        for method_name in binding.method_names:
            owners.setdefault(method_name, []).append(binding.controller_cls.__name__)

    collisions = {name: names for name, names in owners.items() if len(names) > 1}
    assert collisions == {}, f"controller delegate method collisions: {collisions}"
