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
                "px4_origin_world_east_m": 12.0,
                "px4_origin_world_north_m": 2100.0,
                "px4_origin_world_up_m": 6.0,
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
            self.assertAlmostEqual(
                float(row["north_m"]),
                float(row["world_y_m"]) - args.px4_origin_world_north_m,
            )
            self.assertAlmostEqual(
                float(row["east_m"]),
                float(row["world_x_m"]) - args.px4_origin_world_east_m,
            )
            self.assertAlmostEqual(
                float(row["altitude_agl_m"]),
                float(row["world_z_m"]) - args.px4_origin_world_up_m,
            )
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
        self.assertEqual(
            manifest["px4_origin_world_enu_m"],
            {"east": 12.0, "north": 2100.0, "up": 6.0},
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

    def test_range_count_includes_endpoints_and_zigzags_inside_fov(self) -> None:
        module = load_waypoint_module()
        args = type(
            "Args",
            (),
            {
                "output_dir": Path("unused"),
                "seed": 42,
                "calibration_count": 0,
                "validation_count": 8,
                "min_range_m": 200.0,
                "max_range_m": 400.0,
                "range_count": 8,
                "range_step_m": None,
                "lateral_pattern": "zigzag",
                "zigzag_amplitude_m": 2.0,
                "camera_baseline_m": 25.0,
                "camera_height_m": 0.1,
                "px4_origin_world_east_m": 0.0,
                "px4_origin_world_north_m": 400.0,
                "px4_origin_world_up_m": 6.0,
                "cam_l_optical_roll_deg": 0.0,
                "cam_l_optical_pitch_deg": 20.0,
                "cam_l_optical_yaw_deg": 0.0,
                "cam_r_optical_roll_deg": 0.0,
                "cam_r_optical_pitch_deg": 20.0,
                "cam_r_optical_yaw_deg": 0.0,
                "horizontal_fov_rad": 0.154997710698,
                "image_width_px": 1280,
                "image_height_px": 720,
                "fov_margin_rad": 0.01,
                "pixel_margin_px": 32.0,
                "min_set_separation_m": 10.0,
                "max_attempts": 10000,
            },
        )()

        _, validation, manifest = module.generate_waypoints(args)

        self.assertEqual(len(validation), 8)
        self.assertEqual(float(validation[0]["baseline_forward_m"]), 200.0)
        self.assertEqual(float(validation[-1]["baseline_forward_m"]), 400.0)
        self.assertEqual(
            [float(row["baseline_right_m"]) for row in validation],
            [2.0, -2.0, 2.0, -2.0, 2.0, -2.0, 2.0, -2.0],
        )
        self.assertEqual(manifest["range_count"], 8)
        self.assertEqual(manifest["lateral_pattern"], "zigzag")

    def test_explicit_world_rfu_waypoints_are_preserved_and_overlap_scaled(self) -> None:
        module = load_waypoint_module()
        args = type(
            "Args",
            (),
            {
                "output_dir": Path("unused"),
                "seed": 42,
                "calibration_count": 0,
                "validation_count": 8,
                "min_range_m": 1.0,
                "max_range_m": 2.0,
                "range_count": None,
                "range_step_m": None,
                "waypoints_rfu_m": "0,415,154;0,365,137;0,315,121;0,265,104;0,215,88;0,165,71;0,115,55;0,65,38",
                "lateral_pattern": "zigzag",
                "zigzag_amplitude_m": 2.0,
                "zigzag_overlap_ratio": 0.4,
                "camera_baseline_m": 25.0,
                "camera_height_m": 0.1,
                "px4_origin_world_east_m": 0.0,
                "px4_origin_world_north_m": 415.0,
                "px4_origin_world_up_m": 6.0,
                "cam_l_optical_roll_deg": 0.0,
                "cam_l_optical_pitch_deg": 20.0,
                "cam_l_optical_yaw_deg": 0.0,
                "cam_r_optical_roll_deg": 0.0,
                "cam_r_optical_pitch_deg": 20.0,
                "cam_r_optical_yaw_deg": 0.0,
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

        self.assertEqual(calibration, [])
        self.assertEqual(len(validation), 8)
        self.assertEqual(
            [float(row["world_rfu_front_m"]) for row in validation],
            [415.0, 365.0, 315.0, 265.0, 215.0, 165.0, 115.0, 65.0],
        )
        self.assertEqual(
            [round(float(row["world_rfu_right_m"]), 3) for row in validation],
            [8.062, -6.504, 4.956, -3.398, 1.85, -0.292, 0.0, 0.0],
        )
        self.assertEqual(
            [round(float(row["north_m"]), 3) for row in validation],
            [0.0, -50.0, -100.0, -150.0, -200.0, -250.0, -300.0, -350.0],
        )
        self.assertEqual(
            [round(float(row["altitude_agl_m"]), 3) for row in validation],
            [148.1, 131.1, 115.1, 98.1, 82.1, 65.1, 49.1, 32.1],
        )
        self.assertEqual(
            [bool(row["nominal_stereo_visible"]) for row in validation],
            [True, True, True, True, True, False, False, False],
        )
        self.assertEqual(manifest["waypoint_mode"], "explicit_baseline_center_world_rfu")
        self.assertEqual(manifest["nominal_stereo_visible_count"], 5)

        with tempfile.TemporaryDirectory() as tmpdir:
            module.write_mission_path(Path(tmpdir) / "mission.csv", validation)
            lines = (Path(tmpdir) / "mission.csv").read_text().splitlines()
        self.assertEqual(lines[1].split(",")[0], "0.000000")
        self.assertEqual(lines[-1].split(",")[0], "-350.000000")


if __name__ == "__main__":
    unittest.main()
