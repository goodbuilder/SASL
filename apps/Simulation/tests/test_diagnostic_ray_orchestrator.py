from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.diagnostic_ray_pipeline import DiagnosticRayOrchestrator


def _computed(tag):
    return {
        "computation_status": "computed",
        "forward_m": 10.0,
        "right_m": 1.0,
        "down_m": -2.0,
        "range_m": 10.25,
        "output_frame": "baseline_center_frd",
        "tag": tag,
    }


def test_orchestrator_keeps_world_pose_inputs_and_frd_outputs_separate():
    calls = []

    def attitude_poses(source, **kwargs):
        calls.append(("attitude", source, kwargs))
        return (("left_world_enu", "right_world_enu"), "available", {"pose": "px4"})

    def true_north_attitude_poses(source, **kwargs):
        calls.append(("true_north", source, kwargs))
        return (
            ("left_true_north_enu", "right_true_north_enu"),
            "available",
            {"pose": "px4_true_north"},
        )

    def gps_poses(source, **kwargs):
        calls.append(("gps", source, kwargs))
        return (("left_enu", "right_enu"), "available", {"baseline": "enu"})

    def triangulate(*pixels, left_camera_pose, right_camera_pose):
        calls.append(("triangulate", left_camera_pose, right_camera_pose, pixels))
        return _computed(f"{left_camera_pose}:{right_camera_pose}")

    result = DiagnosticRayOrchestrator(
        {
            "attitude_poses": attitude_poses,
            "true_north_attitude_poses": true_north_attitude_poses,
            "gps_poses": gps_poses,
            "triangulate": triangulate,
        },
        gps_baseline_anchor_policy="nominal_left_plus_relative_enu",
    ).compute(1, 2, 3, 4, 1280, 720, "gazebo_snapshot",
              operational_snapshot="pair_snapshot", observation_timestamp_ns=42)

    assert result["attitude"]["output_frame"] == "baseline_center_frd"
    assert result["attitude_true_north"]["output_frame"] == "baseline_center_frd"
    assert result["operational"]["output_frame"] == "baseline_center_frd"
    assert result["operational"]["baseline_anchor_policy"] == "nominal_left_plus_relative_enu"
    # Call order: attitude ray (pose + triangulate), attitude_true_north ray
    # (pose + triangulate), then the gps-baseline-fed operational ray (pose
    # + triangulate).
    assert calls[0][1] == "gazebo_snapshot"
    assert calls[1][1] == "left_world_enu"
    assert calls[2][1] == "gazebo_snapshot"
    assert calls[3][1] == "left_true_north_enu"
    assert calls[4][1] == "gps_baseline"
    assert calls[5][1] == ("left_enu", "right_enu")
    assert calls[6][1] == "left_world_enu"
    assert calls[0][2]["observation_timestamp_ns"] == 42


def test_orchestrator_reports_gps_and_attitude_failures_without_triangulating():
    def attitude_poses(source, **_kwargs):
        if source == "gazebo_snapshot":
            return (None, "stale_attitude", {"pose": "px4"})
        return (None, "invalid_px4_attitude", {"pose": "px4"})

    def true_north_attitude_poses(_source, **_kwargs):
        return (None, "missing_left_gps_fix", {"pose": "px4_true_north"})

    def gps_poses(_source, **_kwargs):
        return (None, "stale_gps", {"baseline": "enu"})

    def unexpected_triangulation(*_args, **_kwargs):
        raise AssertionError("invalid sources must not reach triangulation")

    result = DiagnosticRayOrchestrator(
        {
            "attitude_poses": attitude_poses,
            "true_north_attitude_poses": true_north_attitude_poses,
            "gps_poses": gps_poses,
            "triangulate": unexpected_triangulation,
        },
        gps_baseline_anchor_policy="nominal_left_plus_relative_enu",
    ).compute(1, 2, 3, 4, 1280, 720, "gazebo_snapshot")

    assert result["attitude"]["failure_reason"] == "stale_attitude"
    assert result["attitude_true_north"]["failure_reason"] == "missing_left_gps_fix"
    assert result["operational"]["failure_reason"] == "gps:stale_gps;attitude:stale_attitude"
    assert result["operational"]["valid"] is False
