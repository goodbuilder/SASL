from __future__ import annotations

from wiris_stereo_distance.measurement_widgets import DistanceHistoryPlot, DistanceHistoryPoint
from wiris_stereo_distance.gui import RECORDED_SAMPLE_TABLE_COLUMNS


def test_distance_history_point_uses_explicit_rdf_axis_names() -> None:
    point = DistanceHistoryPoint(
        timestamp_s=1.0,
        distance_m=12.1,
        quality="GOOD",
        method="two_ray_closest_midpoint",
        right_m=1.2,
        down_m=-0.4,
        forward_m=12.0,
    )

    assert (point.right_m, point.down_m, point.forward_m) == (1.2, -0.4, 12.0)
    assert [field for field, _title, _unit, _color in DistanceHistoryPlot.SERIES[:3]] == [
        "right_m",
        "down_m",
        "forward_m",
    ]
    table_fields = [field for field, _label in RECORDED_SAMPLE_TABLE_COLUMNS]
    assert {"raw_forward_m", "raw_right_m", "raw_down_m", "point_frame", "output_frame"} <= set(
        table_fields
    )
    assert not {"x_m", "y_m", "z_m", "raw_x_m", "raw_y_m", "raw_z_m"} & set(table_fields)
