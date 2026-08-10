from pathlib import Path
import sys
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.research_record_fields import (
    camera_oracle_record_fields,
    diagnostic_ray_record_fields,
)


def test_diagnostic_midpoint_columns_preserve_frd_component_order():
    fields = diagnostic_ray_record_fields(
        "operational",
        {
            "valid": True,
            "computation_status": "computed",
            "midpoint_baseline_center_frd_m": (1, 2, 3),
        },
    )
    assert fields["operational_midpoint_x_m"] == 1
    assert fields["operational_midpoint_y_m"] == 2
    assert fields["operational_midpoint_z_m"] == 3


def test_camera_oracle_columns_preserve_world_enu_and_xyzw_orders():
    observation = SimpleNamespace(
        gazebo_stamp_ns=1, base_entity_name="base", base_entity_id=2,
        optical_entity_name="optical", optical_entity_id=3,
        optical_position_world_enu_m=(4, 5, 6),
        optical_orientation_world_from_entity_xyzw=(0.1, 0.2, 0.3, 0.9),
    )
    fields = camera_oracle_record_fields(
        (observation, observation), expected_sdf_sha256="abc"
    )
    assert fields["left_camera_oracle_optical_x_m"] == 4
    assert fields["left_camera_oracle_optical_y_m"] == 5
    assert fields["left_camera_oracle_optical_z_m"] == 6
    assert fields["left_camera_oracle_optical_qw"] == 0.9
    assert fields["camera_oracle_expected_sdf_sha256"] == "abc"
