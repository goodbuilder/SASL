from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtGui

from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.recording_controller import RecordingController
from Workstation.workstation_core.recording_session import save_session_files

from test_optical_pose_controller_integration import _panel


def test_auto_suggestion_and_annotation_review_are_durable_audit_data(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    artifacts = frozen["frozen_frame_artifacts"]

    suggestion = {
        "schema": "optical-pose-auto-suggestion-v1",
        "suggestion_id": "auto-horizon-persisted",
        "mode": "horizon",
        "state": "GOOD",
        "provenance_state": "AUTO_PROPOSED",
        "detection_method": "test-gradient",
        "confidence": 0.9,
        "inlier_count": 32,
        "residual_px": 0.6,
        "eligible": True,
        "sides": {
            side: {
                "side": side,
                "frame_id": artifacts[side]["frame_id"],
                "image_sha256": artifacts[side]["sha256"],
                "source_width_px": artifacts[side]["width"],
                "source_height_px": artifacts[side]["height"],
                "points": [{"u_px": 20.0, "v_px": 220.0}, {"u_px": 620.0, "v_px": 230.0}],
                "horizon_line": [{"u_px": 20.0, "v_px": 220.0}, {"u_px": 620.0, "v_px": 230.0}],
            }
            for side in ("left", "right")
        },
    }

    snapshot = controller._record_auto_suggestion(suggestion)
    assert snapshot["auto_suggestions"][0]["sides"]["left"]["points"]
    assert (events_payload := panel.events[-1][1])
    assert events_payload["suggestion"]["sides"]["right"]["points"]

    accepted = controller.accept_workstation_optical_pose_suggestion(
        {"suggestion_id": "auto-horizon-persisted"}
    )
    assert accepted["annotation_review"]["provenance_state"] == "AUTO_ACCEPTED"
    assert panel.events[-1][0] == "optical_pose_auto_suggestion_accepted"
    assert panel.events[-1][1]["suggestion"]["provenance_state"] == "AUTO_ACCEPTED"

    annotation = {
        "frame_id": artifacts["left"]["frame_id"],
        "image_sha256": artifacts["left"]["sha256"],
        "source_width_px": artifacts["left"]["width"],
        "source_height_px": artifacts["left"]["height"],
        "horizon_line": [{"u_px": 25.0, "v_px": 221.0}, {"u_px": 615.0, "v_px": 231.0}],
        "ridge_polyline": [],
        "target_point": [],
    }
    updated = controller.record_workstation_optical_pose_annotation(
        {
            "suggestion_id": "auto-horizon-persisted",
            "provenance_state": "MANUAL_EDITED",
            "annotation_mode": "horizon",
            "annotations": {"left": annotation},
        }
    )
    assert updated["annotation_review"]["provenance_state"] == "MANUAL_EDITED"
    assert updated["annotation_review"]["annotations"]["left"]["horizon_line"][0]["u_px"] == 25.0
    assert panel.events[-1][0] == "optical_pose_annotations_updated"


def test_optical_pose_session_projection_excludes_qimages_and_is_published(
    tmp_path: Path,
) -> None:
    image = QtGui.QImage(8, 6, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("#444444"))

    class Owner:
        def workstation_optical_pose_snapshot(self, *, include_frames: bool = True):
            assert include_frames is False
            return {
                "state": "ACTIVE",
                "frozen_frames": {"left": {"image": image}},
                "frozen_frame_artifacts": {
                    "left": {"frame_id": "frame-left", "relative_path": "frames/left.png"}
                },
                "auto_suggestions": [{"suggestion_id": "suggestion-1", "sides": {"left": {"points": [(1, 2)]}}}],
                "annotation_review": {
                    "provenance_state": "MANUAL_EDITED",
                    "annotations": {"left": {"frame_id": "frame-left"}},
                },
            }

    projection = RecordingController(Owner())._recording_optical_pose_session_data()
    assert projection["state"] == "ACTIVE"
    assert "frozen_frames" not in projection["snapshot"]
    assert projection["snapshot"]["auto_suggestions"][0]["suggestion_id"] == "suggestion-1"

    output_dir = tmp_path / "sessions" / "persisted-session"
    save_session_files(
        output_dir,
        metadata={"session_id": "persisted-session", "optical_pose": projection},
        pair_rows=[],
        camera_rows={"left": [], "right": []},
        px4_rows={"left": [], "right": []},
        gnss_base_rows=[],
    )
    static_path = output_dir / "persisted-session_session_static.json"
    saved = json.loads(static_path.read_text(encoding="utf-8"))
    assert saved["metadata"]["optical_pose"] == projection
