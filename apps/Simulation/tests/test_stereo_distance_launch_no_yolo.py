from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
LAUNCH_PATH = ROOT_DIR / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py"


def test_stereo_distance_node_is_not_conditioned_on_yolo_enablement() -> None:
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    node_start = source.index('name="stereo_distance_node"')
    node_end = source.index("output=\"screen\"", node_start)

    assert 'condition=IfCondition(LaunchConfiguration("enable_yolo"))' not in source[node_start:node_end]
