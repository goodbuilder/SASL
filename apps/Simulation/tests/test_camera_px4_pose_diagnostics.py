from __future__ import annotations

from array import array
from dataclasses import asdict
import json
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.camera_px4_pose_diagnostics import (
    SCHEMA,
    _gazebo_base_pose_from_message,
    compare_px4_ned_to_gazebo_base_link,
    ned_delta_to_enu,
)


def test_ned_displacement_maps_to_enu_without_changing_the_origin() -> None:
    assert ned_delta_to_enu((3.0, 4.0, 5.0)) == (4.0, 3.0, -5.0)
    assert ned_delta_to_enu(array("f", (3.0, 4.0, 5.0))) == (4.0, 3.0, -5.0)


def test_px4_local_ned_pose_is_bootstrapped_to_observed_gazebo_base_link() -> None:
    comparison = compare_px4_ned_to_gazebo_base_link(
        camera_label="left",
        px4_odometry_topic="/camera_left/fmu/out/vehicle_odometry",
        px4_timestamp_us=123,
        px4_position_ned_m=(13.0, 24.0, 35.0),
        px4_bootstrap_position_ned_m=(10.0, 20.0, 30.0),
        gazebo_stamp_ns=456,
        gazebo_bootstrap_base_link_position_world_enu_m=(-12.5, 0.0, 10.0),
        observed_gazebo_base_link_position_world_enu_m=(-8.5, 3.0, 5.0),
        px4_reset_counter=7,
    )

    assert comparison.schema == SCHEMA
    assert comparison.px4_relative_position_world_enu_m == (4.0, 3.0, -5.0)
    assert comparison.px4_estimated_base_link_position_world_enu_m == (-8.5, 3.0, 5.0)
    assert comparison.px4_to_gazebo_base_link_delta_world_enu_m == (0.0, 0.0, 0.0)
    assert comparison.px4_to_gazebo_base_link_delta_norm_m == pytest.approx(0.0)
    assert json.loads(json.dumps(asdict(comparison)))["px4_pose_frame"] == "local_ned"


def test_px4_comparison_rejects_invalid_timestamps_and_positions() -> None:
    common = dict(
        camera_label="left",
        px4_odometry_topic="/camera_left/fmu/out/vehicle_odometry",
        px4_timestamp_us=123,
        px4_position_ned_m=(0.0, 0.0, 0.0),
        px4_bootstrap_position_ned_m=(0.0, 0.0, 0.0),
        gazebo_stamp_ns=456,
        gazebo_bootstrap_base_link_position_world_enu_m=(0.0, 0.0, 0.0),
        observed_gazebo_base_link_position_world_enu_m=(0.0, 0.0, 0.0),
        px4_reset_counter=0,
    )
    with pytest.raises(ValueError, match="timestamps"):
        compare_px4_ned_to_gazebo_base_link(**{**common, "px4_timestamp_us": 0})
    with pytest.raises(ValueError, match="finite"):
        compare_px4_ned_to_gazebo_base_link(**{**common, "px4_position_ned_m": (float("nan"), 0.0, 0.0)})


def test_gazebo_camera_diagnostic_requires_the_observed_base_link_contract() -> None:
    with pytest.raises(ValueError, match="schema"):
        _gazebo_base_pose_from_message("{}")
    stamp, position = _gazebo_base_pose_from_message(
        json.dumps(
            {
                "schema": "camera_pose_comparison/v1",
                "gazebo_stamp_ns": 5,
                "observed_base_link_position_world_enu_m": [-12.5, 0.0, 10.0],
            }
        )
    )
    assert stamp == 5
    assert position == (-12.5, 0.0, 10.0)
