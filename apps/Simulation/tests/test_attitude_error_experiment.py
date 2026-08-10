from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


class DummyNode:
    pass


def load_attitude_error_experiment_module():
    previous_modules = {
        name: sys.modules.get(name)
        for name in ("rclpy", "rclpy.node", "std_msgs", "std_msgs.msg")
    }
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    rclpy_node.Node = DummyNode
    std_msgs_msg.Float32 = type("Float32", (), {})
    std_msgs_msg.Float32MultiArray = type("Float32MultiArray", (), {})
    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": rclpy_node,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
        }
    )
    try:
        spec = importlib.util.spec_from_file_location(
            "attitude_error_experiment_under_test",
            ROOT_DIR / "scripts" / "attitude_error_experiment.py",
        )
        if spec is None or spec.loader is None:
            raise RuntimeError("cannot load attitude_error_experiment.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, previous in previous_modules.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


class AttitudeErrorExperimentTests(unittest.TestCase):
    def test_summarize_distance_preserves_frd_alias_contract(self) -> None:
        module = load_attitude_error_experiment_module()

        summary = module.summarize_distance(
            [
                {
                    "drone_x_m": "101.0",
                    "drone_y_m": "202.0",
                    "drone_z_m": "303.0",
                    "estimated_x_m": "404.0",
                    "estimated_y_m": "505.0",
                    "estimated_z_m": "606.0",
                    "estimated_range_m": "707.0",
                    "ground_truth_m": "800.0",
                    "measured_m": "802.5",
                    "error_m": "2.5",
                }
            ]
        )

        # attitude_error_experiment CSV rows use x=forward, y=right, z=down.
        self.assertEqual(summary["last_drone_forward_m"], "101.0")
        self.assertEqual(summary["last_drone_right_m"], "202.0")
        self.assertEqual(summary["last_drone_down_m"], "303.0")
        self.assertEqual(summary["last_estimated_forward_m"], "404.0")
        self.assertEqual(summary["last_estimated_right_m"], "505.0")
        self.assertEqual(summary["last_estimated_down_m"], "606.0")
        self.assertNotIn("last_drone_x_m", summary)
        self.assertNotIn("last_estimated_x_m", summary)


if __name__ == "__main__":
    unittest.main()
