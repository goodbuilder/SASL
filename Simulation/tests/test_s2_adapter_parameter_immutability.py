from __future__ import annotations

from types import SimpleNamespace

from Simulation.stereo_ground_sim.stereo_ground_sim.gazebo_ground_truth_adapter import (
    GazeboGroundTruthAdapter,
)


class Result:
    def __init__(self, *, successful=True, reason=""):
        self.successful = successful
        self.reason = reason


def test_ground_truth_identity_parameters_are_immutable_as_one_batch() -> None:
    adapter = GazeboGroundTruthAdapter.__new__(GazeboGroundTruthAdapter)
    adapter._parent_model_name = "x500_15m"
    adapter._source_link_name = "base_link"
    adapter._set_parameters_result_type = Result
    for name in ("parent_model_name", "source_link_name"):
        result = adapter._reject_identity_parameter_update(
            [SimpleNamespace(name="unrelated"), SimpleNamespace(name=name)]
        )
        assert not result.successful
        assert result.reason == "Ground Truth entity identity parameters are immutable for one run"
        assert adapter._parent_model_name == "x500_15m"
        assert adapter._source_link_name == "base_link"
    assert adapter._reject_identity_parameter_update([SimpleNamespace(name="unrelated")]).successful
