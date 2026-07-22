"""Pure, immutable pair-aligned operator diagnostics."""
from dataclasses import dataclass
import math
from typing import Any, Mapping

METRIC_NAMES = (
    "raw_position_error_m", "corrected_position_error_m", "raw_forward_error_m",
    "corrected_forward_error_m", "raw_right_error_m", "corrected_right_error_m",
    "raw_down_error_m", "corrected_down_error_m", "raw_range_error_m",
    "corrected_range_error_m", "raw_ray_gap_m", "corrected_ray_gap_m",
    "raw_intersection_angle_deg", "corrected_intersection_angle_deg",
)


@dataclass(frozen=True)
class OperatorPairDiagnostics:
    epoch: int
    pair_id: str
    pair_midpoint_stamp_ns: int
    output_frame: str
    position_unit: str
    angle_unit: str
    values: tuple[tuple[str, float], ...]

    def __post_init__(self) -> None:
        if type(self.epoch) is not int or self.epoch < 0:
            raise ValueError("invalid_epoch")
        if type(self.pair_id) is not str or not self.pair_id.strip():
            raise ValueError("invalid_pair_id")
        if type(self.pair_midpoint_stamp_ns) is not int or self.pair_midpoint_stamp_ns < 0:
            raise ValueError("invalid_pair_midpoint_stamp")
        if (self.output_frame, self.position_unit, self.angle_unit) != ("baseline_center_frd", "m", "deg"):
            raise ValueError("operator_coordinate_contract_mismatch")
        if type(self.values) is not tuple or tuple(k for k, _ in self.values) != tuple(k for k in METRIC_NAMES if k in dict(self.values)):
            raise ValueError("invalid_metric_order")
        if any(type(item) is not tuple or len(item) != 2 or type(k) is not str or type(v) not in (int, float) or isinstance(v, bool) or not math.isfinite(v) for item in self.values for k, v in (item,)):
            raise ValueError("invalid_metric_value")

    def value_map(self) -> dict[str, float]:
        return dict(self.values)


def _identity(source: Mapping[str, Any]) -> tuple[int, str, int]:
    epoch, pair_id, stamp = source.get("epoch"), source.get("pair_id"), source.get("pair_midpoint_stamp_ns")
    if type(epoch) is not int or epoch < 0 or type(pair_id) is not str or not pair_id.strip() or type(stamp) is not int or stamp < 0:
        raise ValueError("invalid_pair_identity")
    return epoch, pair_id, stamp


def build_pair_diagnostics(*, pair: Mapping[str, Any], measurement: Mapping[str, Any], alignment: Mapping[str, Any]) -> OperatorPairDiagnostics | None:
    try:
        identities = [_identity(source) for source in (pair, measurement, alignment)]
    except ValueError:
        return None
    if identities.count(identities[0]) != len(identities):
        return None
    if pair.get("terminal_status") != "evaluated" or measurement.get("measurement_status") != "processed" or alignment.get("evaluation_status") != "evaluated":
        return None
    if pair.get("output_frame") != "baseline_center_frd" or pair.get("position_unit") != "m" or pair.get("angle_unit") != "deg":
        return None
    values = []
    for name in METRIC_NAMES:
        value = pair.get(name)
        if value is None:
            continue
        if type(value) not in (int, float) or isinstance(value, bool) or not math.isfinite(value):
            continue
        values.append((name, float(value)))
    if not values:
        return None
    epoch, pair_id, stamp = identities[0]
    return OperatorPairDiagnostics(epoch, pair_id, stamp, "baseline_center_frd", "m", "deg", tuple(values))
