"""Auto-selection must run from detached inputs without a live controller."""
from __future__ import annotations

from PyQt5 import QtGui
import pytest

from Workstation.tests.test_optical_pose_controller_integration import _panel
import Workstation.optical_pose_controller as controller_module
from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.workstation_core.optical_pose_auto_select import AutoSelectionResult, MountainAutoSelection


class ForbiddenPanel:
    def __getattribute__(self, name):
        raise AssertionError(f"background job read live panel: {name}")


def test_horizon_job_keeps_its_image_and_profile_after_owner_changes(tmp_path, monkeypatch):
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    controller.capture_workstation_optical_pose_burst({})
    observed = []
    initial = controller._frozen_frames["left"]["image"].pixelColor(0, 0).name()

    def detect(image, **kwargs):
        observed.append((image.pixelColor(0, 0).name(), kwargs["camera_matrix"]))
        image.fill(QtGui.QColor("magenta"))
        return AutoSelectionResult(
            mode="horizon", state="GOOD", detection_method="fixture",
            points=((5.0, 100.0), (630.0, 100.0)), inlier_count=2,
            confidence=0.8, eligible=True,
        )

    monkeypatch.setattr(controller_module, "auto_select_horizon", detect)
    job = controller.prepare_workstation_optical_pose_auto_selection("horizon", {})
    assert not hasattr(job, "controller")
    profile_hash = job.inputs.profile.profile_sha256
    controller._frozen_frames["left"]["image"].fill(QtGui.QColor("green"))
    panel.active_intrinsics = None
    panel.camera_identity_evidence = {"left": {"serial": "changed"}}
    controller.panel = ForbiddenPanel()

    first = job.run()
    second = job.run()

    assert observed[0][0] == initial
    assert observed[2][0] == initial  # detector mutation did not change the frozen image
    assert first["input_hashes"]["operating_intrinsics_profile_sha256"] == profile_hash
    assert second["source_frames"] == first["source_frames"]


def test_mountain_job_freezes_explicit_ned_override_and_scalar_settings(tmp_path, monkeypatch):
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    controller.capture_workstation_optical_pose_burst({})
    calls = []
    points = ((10.0, 20.0), (120.0, 210.0), (260.0, 55.0), (430.0, 300.0))
    sides = [AutoSelectionResult(
        mode="mountain", state="GOOD", detection_method="fixture", points=points,
        confidence=0.8, inlier_count=4, spatial_rank=2, eligible=True,
    ) for _ in range(2)]

    def detect(*images, **kwargs):
        calls.append(kwargs)
        return MountainAutoSelection(
            left=sides[0], right=sides[1], state="GOOD", detection_method="fixture",
            match_count=4, inlier_count=4, confidence=0.8, eligible=True,
        )

    monkeypatch.setattr(controller_module, "auto_select_mountain", detect)
    payload = {"gnss_baseline_ned_m": [3.0, 4.0, -2.0]}
    controller._mountain_max_points = 27
    job = controller.prepare_workstation_optical_pose_auto_selection("mountain", payload)
    payload["gnss_baseline_ned_m"][0] = 999.0
    controller._mountain_max_points = 99
    controller.panel = ForbiddenPanel()
    with pytest.raises(TypeError):
        job.inputs.baseline_details["source"] = "modified"

    result = job.run()

    assert calls[0]["baseline_ned_m"] == (3.0, 4.0, -2.0)
    assert calls[0]["max_points"] == 27
    assert result["gnss_baseline"]["ned_m"] == [3.0, 4.0, -2.0]
    assert "explicit override" in result["gnss_baseline"]["source"]
