"""Pure, immutable diagnostics built from one evaluated terminal record."""

from dataclasses import dataclass
import math
from typing import Any, Mapping

METRIC_NAMES = (
    "raw_position_error_m", "corrected_position_error_m",
    "raw_forward_error_m", "corrected_forward_error_m",
    "raw_right_error_m", "corrected_right_error_m",
    "raw_down_error_m", "corrected_down_error_m",
    "raw_range_error_m", "corrected_range_error_m",
    "raw_ray_gap_m", "corrected_ray_gap_m",
    "raw_intersection_angle_deg", "corrected_intersection_angle_deg",
)
_FRAME_CONTRACT = ("baseline_center_frd", "m", "deg")


def _identity_values(epoch: Any, pair_id: Any, stamp: Any) -> tuple[int, int, int]:
    if type(epoch) is not int or epoch < 0:
        raise ValueError("operator_diagnostics_invalid_epoch")
    if type(pair_id) is not int or pair_id <= 0:
        raise ValueError("operator_diagnostics_invalid_pair_id")
    if type(stamp) is not int or stamp < 0:
        raise ValueError("operator_diagnostics_invalid_pair_midpoint_stamp")
    return epoch, pair_id, stamp


def _pair_identity(pair: Any) -> tuple[int, int, int]:
    try:
        return _identity_values(pair.epoch, pair.pair_id, pair.pair_midpoint_stamp_ns)
    except AttributeError as exc:
        raise ValueError("operator_diagnostics_pair_identity_missing") from exc


def _record_identity(record: Mapping[str, Any]) -> tuple[int, int, int]:
    return _identity_values(record.get("epoch"), record.get("pair_id"), record.get("pair_midpoint_stamp_ns"))


@dataclass(frozen=True)
class OperatorPairDiagnostics:
    epoch: int
    pair_id: int
    pair_midpoint_stamp_ns: int
    output_frame: str
    position_unit: str
    angle_unit: str
    values: tuple[tuple[str, float], ...]

    def __post_init__(self) -> None:
        _identity_values(self.epoch, self.pair_id, self.pair_midpoint_stamp_ns)
        if (self.output_frame, self.position_unit, self.angle_unit) != _FRAME_CONTRACT:
            raise ValueError("operator_diagnostics_coordinate_contract_mismatch")
        if type(self.values) is not tuple:
            raise ValueError("operator_diagnostics_values_not_tuple")
        if not self.values:
            raise ValueError("operator_diagnostics_values_empty")
        keys: list[str] = []
        for item in self.values:
            if type(item) is not tuple:
                raise ValueError("operator_diagnostics_metric_item_not_tuple")
            if len(item) != 2:
                raise ValueError("operator_diagnostics_metric_item_shape")
            key, value = item
            if type(key) is not str or key not in METRIC_NAMES:
                raise ValueError("operator_diagnostics_metric_key_invalid")
            if key in keys:
                raise ValueError("operator_diagnostics_metric_duplicate")
            if type(value) is not float or not math.isfinite(value):
                raise ValueError("operator_diagnostics_metric_value_invalid")
            keys.append(key)
        expected = tuple(name for name in METRIC_NAMES if name in keys)
        if tuple(keys) != expected:
            raise ValueError("operator_diagnostics_metric_order_invalid")

    def value_map(self) -> dict[str, float]:
        return dict(self.values)


def build_pair_diagnostics_from_terminal_record(*, pair: Any, terminal_record: Mapping[str, Any]) -> OperatorPairDiagnostics | None:
    """Project one already-assembled evaluated terminal record.

    No lookup, I/O, geometry, correction, ROS, or recorder access occurs here.
    """
    if not isinstance(terminal_record, Mapping):
        return None
    try:
        pair_identity = _pair_identity(pair)
        record_identity = _record_identity(terminal_record)
    except ValueError:
        return None
    if pair_identity != record_identity:
        return None
    if (
        terminal_record.get("terminal_status") != "evaluated"
        or terminal_record.get("measurement_status") != "processed"
        or terminal_record.get("evaluation_status") != "evaluated"
    ):
        return None
    if (
        terminal_record.get("output_frame"),
        terminal_record.get("position_unit"),
        terminal_record.get("angle_unit"),
    ) != _FRAME_CONTRACT:
        return None
    values: list[tuple[str, float]] = []
    for name in METRIC_NAMES:
        value = terminal_record.get(name)
        if value is None:
            continue
        if type(value) not in (int, float) or isinstance(value, bool):
            continue
        if not math.isfinite(value):
            continue
        values.append((name, float(value)))
    if not values:
        return None
    epoch, pair_id, stamp = pair_identity
    return OperatorPairDiagnostics(epoch, pair_id, stamp, *_FRAME_CONTRACT, tuple(values))


__all__ = ["METRIC_NAMES", "OperatorPairDiagnostics", "build_pair_diagnostics_from_terminal_record"]
