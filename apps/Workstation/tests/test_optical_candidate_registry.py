"""Candidate/selection ownership and immutable pose preview contracts."""
from __future__ import annotations

import pytest

from Workstation.optical_pose.candidates import OpticalCandidateRegistry
from Workstation.workstation_core.optical_pose import CameraPoseNed, OpticalPoseCandidate, OpticalPoseSelection, PoseDofMask, POSE_DOF_NAMES


def poses(yaw):
    return tuple(CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=center, roll_deg=0.0, pitch_deg=0.0, yaw_deg=yaw,
    ) for center in ((1.0, 2.0, 3.0), (4.0, 7.0, 3.0)))


def candidate(identity, when, method="fixture"):
    left, right = poses(-179.0)
    return OpticalPoseCandidate(
        candidate_id=identity, method=method, left_pose_ned=left, right_pose_ned=right,
        optimized_dofs=PoseDofMask.from_names(("yaw_deg",)),
        observability=tuple((name, name == "yaw_deg") for name in POSE_DOF_NAMES),
        created_utc_ns=when,
    )


def test_candidate_registration_preserves_ned_pose_and_wrapped_prior_delta():
    owner = OpticalCandidateRegistry()
    value = candidate("first", 1)
    prior = poses(179.0)
    result = owner.register(value, prior)
    assert owner._candidates["first"] is value
    assert owner._candidate_priors["first"] is prior
    assert result["left_pose"]["center_ned_m"] == [1.0, 2.0, 3.0]
    assert result["left_delta_rpy_deg"][2] == pytest.approx(2.0)
    with pytest.raises(ValueError, match="duplicate"):
        owner.register(value, prior)


def test_preview_is_latest_per_method_without_applying_a_candidate():
    owner = OpticalCandidateRegistry()
    owner.register(candidate("old", 1), poses(0.0))
    owner.register(candidate("new", 2), poses(0.0))
    preview = owner.latest_poses()
    assert [row["candidate_id"] for row in preview] == ["new"]
    assert preview[0]["candidate_applied"] is False
    assert owner._active_selection is None
    assert owner._active_poses is None
    preview[0]["candidate_warnings"].append("external edit")
    assert owner.latest_poses()[0]["candidate_warnings"] == []


def test_selection_restore_preserves_the_single_history_list():
    owner = OpticalCandidateRegistry()
    value = candidate("selected", 1)
    prior = poses(0.0)
    owner.register(value, prior)
    history = owner._selection_history
    selection = OpticalPoseSelection("selection", "selected", value.optimized_dofs, 2, 123)
    owner.select(selection, prior)
    assert owner.latest_poses()[0]["candidate_applied"] is True
    owner.restore_selection({"active_selection": None, "active_poses": None, "selection_history": ()})
    assert owner._selection_history is history
    assert history == []
    assert owner._active_selection is None
