from __future__ import annotations

import inspect
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.stereo_distance_node import StereoDistanceNode


ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = ROOT / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py"
LAUNCH_SOURCE = ROOT / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py"
OPERATOR_SOURCE = ROOT / "run_operator_view.sh"


def test_actual_distance_is_gazebo_cache_only_and_unavailable_without_gt() -> None:
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.cached_ground_truth_position = None
    node.cached_actual_distance = 123.0
    assert node._actual_distance() is None
    assert node._read_actual_distance() is None

    node.cached_ground_truth_position = {"ground_truth_source": "gazebo_vehicle_reference_link_world_pose"}
    assert node._actual_distance() == 123.0
    assert node._read_actual_distance() == 123.0


def test_read_actual_distance_has_no_planned_fallback_argument() -> None:
    signature = inspect.signature(StereoDistanceNode._read_actual_distance)
    assert tuple(signature.parameters) == ("self",)
    source = NODE_SOURCE.read_text()
    assert "_read_actual_distance(self, fallback)" not in source
    assert "_read_actual_distance(state[\"distance_m\"])" not in source
    assert "_read_actual_distance(state['distance_m'])" not in source


def test_active_launch_and_operator_contract_do_not_select_planned_truth() -> None:
    node_source = NODE_SOURCE.read_text()
    launch_source = LAUNCH_SOURCE.read_text()
    operator_source = OPERATOR_SOURCE.read_text()
    for source in (node_source, launch_source, operator_source):
        assert "actual_distance_source" not in source
        assert "actual_distance_read_hz" not in source
    assert "raw = fused_raw" not in node_source
