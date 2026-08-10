from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.mission_timeline import MissionTimelineConfig, mission_state


def _config(profile="takeoff_forward_land", approach_duration_s=4.0):
    return MissionTimelineConfig(
        profile=profile,
        start_distance_m=10.0,
        end_distance_m=2.0,
        initial_world_enu_z_m=1.0,
        hover_world_enu_z_m=5.0,
        takeoff_duration_s=2.0,
        first_hover_duration_s=1.0,
        approach_duration_s=approach_duration_s,
        final_hover_duration_s=1.0,
        land_duration_s=2.0,
    )


def test_manual_profile_is_stationary_at_initial_world_enu_altitude():
    assert mission_state(_config("manual"), 100.0) == {
        "phase": "manual_qgc",
        "distance_m": 10.0,
        "altitude_m": 1.0,
        "velocity_x_mps": 0.0,
        "velocity_z_mps": 0.0,
    }


@pytest.mark.parametrize(
    ("elapsed_s", "phase", "distance_m", "altitude_m", "velocity_x", "velocity_z"),
    (
        (0.0, "takeoff", 10.0, 1.0, 0.0, 2.0),
        (1.0, "takeoff", 10.0, 3.0, 0.0, 2.0),
        (2.0, "hover_before_approach", 10.0, 5.0, 0.0, 0.0),
        (3.0, "approach", 10.0, 5.0, -2.0, 0.0),
        (5.0, "approach", 6.0, 5.0, -2.0, 0.0),
        (7.0, "hover_after_approach", 2.0, 5.0, 0.0, 0.0),
        (8.0, "landing", 2.0, 5.0, 0.0, -2.0),
        (9.0, "landing", 2.0, 3.0, 0.0, -2.0),
        (10.0, "complete", 2.0, 1.0, 0.0, 0.0),
    ),
)
def test_takeoff_forward_land_phase_boundaries(
    elapsed_s, phase, distance_m, altitude_m, velocity_x, velocity_z
):
    state = mission_state(_config(), elapsed_s)
    assert state["phase"] == phase
    assert state["distance_m"] == pytest.approx(distance_m)
    assert state["altitude_m"] == pytest.approx(altitude_m)
    assert state["velocity_x_mps"] == pytest.approx(velocity_x)
    assert state["velocity_z_mps"] == pytest.approx(velocity_z)


def test_simple_approach_clamps_negative_and_completed_elapsed_time():
    config = _config(profile="approach_only")
    assert mission_state(config, -1.0)["distance_m"] == pytest.approx(10.0)
    completed = mission_state(config, 5.0)
    assert completed["phase"] == "complete"
    assert completed["distance_m"] == pytest.approx(2.0)
    assert completed["velocity_x_mps"] == 0.0


def test_zero_duration_approach_skips_directly_to_post_approach_hover():
    state = mission_state(_config(approach_duration_s=0.0), 3.0)
    assert state["phase"] == "hover_after_approach"
    assert state["distance_m"] == 2.0
