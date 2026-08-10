from pathlib import Path
import sys
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.gazebo_target_controller import (
    GazeboTargetController,
    pose_request,
)


def test_pose_request_maps_mission_distance_to_world_enu_y_and_yaw_about_up():
    request = pose_request("target", 2.0, 10.0, 3.0, 90.0)
    assert 'name: "target"' in request
    assert "position: {x: 2.000000, y: 10.000000, z: 3.000000}" in request
    assert "z: 0.707106781" in request
    assert "w: 0.707106781" in request


def test_mission_commands_are_throttled_without_changing_world_enu_contract():
    times = iter((1.0, 1.2, 1.6))
    calls = []

    def runner(command, **_kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0)

    controller = GazeboTargetController(
        "target", warning=lambda _message: None, monotonic=lambda: next(times), runner=runner
    )
    state = {"distance_m": 8.0, "altitude_m": 4.0}
    assert controller.command_mission_state(
        spawn_world_enu_x_m=1.0, state=state, yaw_world_enu_deg=0.0
    ) is True
    assert controller.command_mission_state(
        spawn_world_enu_x_m=1.0, state=state, yaw_world_enu_deg=0.0
    ) is None
    assert controller.command_mission_state(
        spawn_world_enu_x_m=1.0, state=state, yaw_world_enu_deg=0.0
    ) is True
    assert len(calls) == 2


def test_nonzero_gazebo_exit_is_reported_as_failure():
    warnings = []
    controller = GazeboTargetController(
        "target",
        warning=warnings.append,
        runner=lambda *_args, **_kwargs: SimpleNamespace(returncode=4),
    )
    assert controller.set_pose(0, 0, 0, 0) is False
    assert warnings == ["Gazebo set_pose returned exit code 4"]
