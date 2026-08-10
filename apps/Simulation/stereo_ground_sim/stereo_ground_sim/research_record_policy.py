"""Pure validation policy for completed research pair records."""

import math

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME


def _finite(record, names):
    return all(
        record.get(name) is not None and math.isfinite(float(record[name]))
        for name in names
    )


def sample_eligible(record, *, terminal_status, measurement_status, evaluation_status):
    """Accept only evaluated baseline-center FRD samples with finite inputs.

    Pixel fields are OpenCV optical `(u=right,v=down)` in px. Nominal,
    corrected, and GT vectors are baseline-center `[forward,right,down]` in m.
    """

    return bool(
        terminal_status == "evaluated"
        and measurement_status == "processed"
        and evaluation_status == "evaluated"
        and _finite(record, ("left_u_px", "left_v_px", "right_u_px", "right_v_px"))
        and record.get("nominal_pose_computation_status") == "computed"
        and record.get("corrected_computation_status") == "computed"
        and _finite(
            record,
            (
                "nominal_pose_forward_m", "nominal_pose_right_m",
                "nominal_pose_down_m", "nominal_pose_range_m",
                "corrected_forward_m", "corrected_right_m", "corrected_down_m",
                "corrected_range_m", "gt_forward_m", "gt_right_m", "gt_down_m",
                "gt_range_m",
            ),
        )
        and record.get("output_frame") == BASELINE_CENTER_FRD_FRAME
    )
