from __future__ import annotations

import ast
import math
import time
from pathlib import Path


OPERATOR_PANEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "stereo_ground_sim"
    / "stereo_ground_sim"
    / "operator_panel.py"
)


def _load_pure_helpers():
    tree = ast.parse(OPERATOR_PANEL_PATH.read_text(encoding="utf-8"))
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in {"_shortest_angle_delta_deg", "_mavlink_attitude_sample"}
    ]
    namespace = {"math": math, "time": time}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(OPERATOR_PANEL_PATH), "exec"), namespace)
    return namespace


class _FakeAttitude:
    roll = math.pi / 6.0
    pitch = -math.pi / 12.0
    yaw = math.radians(179.0)
    time_boot_ms = 123456

    @staticmethod
    def get_srcSystem():
        return 2

    @staticmethod
    def get_srcComponent():
        return 1


def test_raw_mavlink_attitude_is_converted_from_radians_with_distinct_timestamp_contract():
    helpers = _load_pure_helpers()

    sample = helpers["_mavlink_attitude_sample"](
        _FakeAttitude(), "udpin:0.0.0.0:14551", receive_monotonic_ns=99
    )

    assert all(
        math.isclose(actual, expected, abs_tol=1.0e-12)
        for actual, expected in zip(sample["rpy"], (30.0, -15.0, 179.0))
    )
    assert sample["time_boot_ms"] == 123456
    assert sample["sysid"] == 2
    assert sample["compid"] == 1
    assert sample["receive_monotonic_ns"] == 99
    assert sample["url"] == "udpin:0.0.0.0:14551"


def test_yaw_comparison_wraps_at_the_minus_plus_180_boundary():
    helpers = _load_pure_helpers()

    delta = helpers["_shortest_angle_delta_deg"](-179.0, 179.0)

    assert math.isclose(delta, 2.0)
