"""Pure, pair-aligned diagnostics for the operator view."""
from dataclasses import dataclass
import math
from typing import Mapping, Any


@dataclass(frozen=True)
class OperatorPairDiagnostics:
    epoch: int
    pair_id: str
    pair_midpoint_stamp_ns: int
    output_frame: str
    position_unit: str
    angle_unit: str
    values: Mapping[str, float]


def build_pair_diagnostics(final: Mapping[str, Any]) -> OperatorPairDiagnostics | None:
    """Build a snapshot only from one completed evaluated pair."""
    if final.get("terminal_status") not in ("evaluated", "valid"):
        return None
    if final.get("output_frame") != "baseline_center_frd" or final.get("position_unit") != "m":
        return None
    if final.get("angle_unit") != "deg":
        return None
    epoch, pair_id, stamp = final.get("epoch"), final.get("pair_id"), final.get("pair_midpoint_stamp_ns")
    if type(epoch) is not int or type(pair_id) is not str or type(stamp) is not int:
        return None
    names = ("raw_position_error_m", "corrected_position_error_m", "raw_forward_error_m",
             "corrected_forward_error_m", "raw_right_error_m", "corrected_right_error_m",
             "raw_down_error_m", "corrected_down_error_m", "raw_range_error_m",
             "corrected_range_error_m", "raw_ray_gap_m", "corrected_ray_gap_m",
             "raw_intersection_angle_deg", "corrected_intersection_angle_deg")
    values = {}
    for name in names:
        value = final.get(name)
        if type(value) not in (int, float) or isinstance(value, bool) or not math.isfinite(value):
            return None
        values[name] = float(value)
    return OperatorPairDiagnostics(epoch, pair_id, stamp, "baseline_center_frd", "m", "deg", values)
