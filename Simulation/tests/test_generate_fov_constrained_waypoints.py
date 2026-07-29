from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


def load_waypoint_module():
    spec = importlib.util.spec_from_file_location(
        "generate_fov_constrained_waypoints_under_test",
        ROOT_DIR / "scripts" / "generate_fov_constrained_waypoints.py",
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load generate_fov_constrained_waypoints.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class GenerateFovConstrainedWaypointTests(unittest.TestCase):
    def test_generates_disjoint_waypoints_inside_left_and_right_fov(self) -> None:
        module = load_waypoint_module()
        args = type(
            "Args",
            (),
            {
                "output_dir": Path("unused"),
                "seed": 42,
                "calibration_count": 6,
                "validation_count": 5,
                "min_range_m": 500.0,
                "max_range_m": 900.0,
                "camera_baseline_m": 25.0,
                "camera_height_m": 0.5,
                "horizontal_fov_rad": 0.154997710698,
                "image_width_px": 1280,
                "image_height_px": 720,
                "fov_margin_rad": 0.01,
                "pixel_margin_px": 32.0,
                "min_set_separation_m": 10.0,
                "max_attempts": 10000,
            },
        )()

        calibration, validation, manifest = module.generate_waypoints(args)

        self.assertEqual(len(calibration), 6)
        self.assertEqual(len(validation), 5)
        self.assertTrue(manifest["sets_disjoint"])
        usable_h = args.horizontal_fov_rad * 0.5 - args.fov_margin_rad
        usable_v = module.vertical_fov_rad(
            args.horizontal_fov_rad,
            args.image_width_px,
            args.image_height_px,
        ) * 0.5 - args.fov_margin_rad
        for row in calibration + validation:
            self.assertEqual(row["baseline_frame"], module.BASELINE_CENTER_FRD_FRAME)
            self.assertEqual(row["px4_waypoint_frame"], module.PX4_WAYPOINT_FRAME)
            self.assertGreaterEqual(float(row["baseline_forward_m"]), args.min_range_m)
            self.assertLessEqual(float(row["baseline_forward_m"]), args.max_range_m)
            self.assertLess(abs(float(row["horizontal_angle_rad"])), usable_h)
            self.assertLess(abs(float(row["vertical_angle_rad"])), usable_v)
            for key, limit in (
                ("left_u_px", args.image_width_px),
                ("right_u_px", args.image_width_px),
                ("left_v_px", args.image_height_px),
                ("right_v_px", args.image_height_px),
            ):
                value = float(row[key])
                self.assertGreaterEqual(value, args.pixel_margin_px)
                self.assertLessEqual(value, float(limit) - args.pixel_margin_px)

        for left in calibration:
            for right in validation:
                self.assertGreaterEqual(
                    module.baseline_distance(left, right),
                    args.min_set_separation_m,
                )

    def test_main_writes_px4_mission_csvs(self) -> None:
        module = load_waypoint_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            old_argv = sys.argv
            try:
                sys.argv = [
                    "generate_fov_constrained_waypoints.py",
                    "--output-dir",
                    tmpdir,
                    "--calibration-count",
                    "2",
                    "--validation-count",
                    "2",
                    "--seed",
                    "7",
                ]
                self.assertEqual(module.main(), 0)
            finally:
                sys.argv = old_argv

            tmp = Path(tmpdir)
            self.assertTrue((tmp / "calibration_waypoints.csv").exists())
            self.assertTrue((tmp / "validation_waypoints.csv").exists())
            self.assertTrue((tmp / "calibration_mission.csv").exists())
            self.assertTrue((tmp / "validation_mission.csv").exists())
            manifest = (tmp / "waypoint_manifest.json").read_text(encoding="utf-8")

        self.assertIn("baseline_center_frd", manifest)
        self.assertIn("px4_local_ned_setpoint", manifest)


if __name__ == "__main__":
    unittest.main()
