from types import SimpleNamespace

import pytest

from workstation_core.mavlink_parameter_store import (
    MavlinkAttitudeParameterStore,
    MavlinkParameterStore,
)


def test_integer_store_requires_matching_readback():
    calls = []
    subscriber = SimpleNamespace(
        read_param_int=lambda camera, name, timeout_s: calls.append(
            ("read", camera, name, timeout_s)
        ) or 7,
        set_param_int=lambda camera, name, value, timeout_s: calls.append(
            ("set", camera, name, value, timeout_s)
        ) or value,
    )
    store = MavlinkParameterStore(subscriber, "left")

    assert store.get_param("CAL_MAG0_ID") == 7
    store.set_param("CAL_MAG0_PRIO", 50.9)
    assert calls[-1] == ("set", "left", "CAL_MAG0_PRIO", 50, 2.0)

    subscriber.set_param_int = lambda *_args, **_kwargs: 49
    with pytest.raises(RuntimeError, match="readback mismatch"):
        store.set_param("CAL_MAG0_PRIO", 50)


def test_attitude_store_preserves_real32_and_standalone_reexports():
    subscriber = SimpleNamespace(
        read_param=lambda camera, name, timeout_s: 0.125,
    )
    store = MavlinkAttitudeParameterStore(subscriber, "right")

    assert store.get_param("CAL_ACC0_XOFF") == pytest.approx(0.125)

    from workstation import standalone_gui

    assert standalone_gui.MavlinkParameterStore is MavlinkParameterStore
    assert standalone_gui.MavlinkAttitudeParameterStore is MavlinkAttitudeParameterStore
