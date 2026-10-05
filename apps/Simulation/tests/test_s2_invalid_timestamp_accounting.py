from __future__ import annotations

from types import SimpleNamespace

import pytest

from Simulation.stereo_ground_sim.stereo_ground_sim.time_alignment import StereoImagePairer


@pytest.mark.parametrize(
    "message",
    [
        object(),
        SimpleNamespace(header=object()),
        SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec="bad", nanosec=0))),
        SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=1_000_000_000))),
        SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1 << 63, nanosec=0))),
    ],
)
def test_invalid_message_timestamp_is_counted(message) -> None:
    pairer = StereoImagePairer()
    with pytest.raises(ValueError):
        pairer.add_message("left", message)
    assert pairer.invalid_timestamp_count == 1
