from __future__ import annotations

import importlib.util
import json
import math
import sys
import types
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPO_ROOT / "Simulation" / "stereo_ground_sim"
SCRIPT_PATH = REPO_ROOT / "Simulation" / "scripts" / "phase3c_camera_equivalence_probe.py"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def load_probe_module():
    spec = importlib.util.spec_from_file_location("phase3c_camera_equivalence_probe_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to load probe module")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


probe = load_probe_module()


class DummyImage:
    def __init__(self, encoding: str, array: np.ndarray):
        self.encoding = encoding
        self.height = int(array.shape[0])
        self.width = int(array.shape[1])
        self.step = int(array.shape[1] * 3)
        self.data = array.tobytes()


class Phase3CCameraEquivalenceProbeTests(unittest.TestCase):
    def test_marker_points_round_trip_and_expected_world_coordinates(self) -> None:
        for name, expected_world in probe.EXPECTED_MARKER_POINTS_WORLD_ENU_M.items():
            actual_world = probe.marker_world_point(name)
            for actual, expected in zip(actual_world, expected_world):
                self.assertAlmostEqual(actual, expected, delta=1.0e-12)
            round_trip = probe.marker_baseline_point_from_world(actual_world)
            for actual, expected in zip(round_trip, probe.MARKER_POINTS_BASELINE_FRD_M[name]):
                self.assertAlmostEqual(actual, expected, delta=1.0e-12)
            self.assertGreater(actual_world[2], 1.0)

    def test_all_standard_marker_analytic_projections_are_valid(self) -> None:
        zero_case = probe.standard_cases()[0]
        self.assertEqual(zero_case.name, "zero")
        for marker_name in zero_case.marker_names:
            projection = probe.analytic_projection(marker_name, zero_case)
            self.assertTrue(projection["valid"], marker_name)
            self.assertEqual(projection["status"], "ok")
            for side in ("left", "right"):
                self.assertGreaterEqual(projection[side]["u_px"], 0.0)
                self.assertLess(projection[side]["u_px"], probe.WIDTH_PX)
                self.assertGreaterEqual(projection[side]["v_px"], 0.0)
                self.assertLess(projection[side]["v_px"], probe.HEIGHT_PX)

    def test_known_sensitivity_reference_is_preserved(self) -> None:
        zero = probe.AttitudeCase(
            "zero",
            probe.CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
            probe.CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
            ("center",),
        )
        yaw = probe.AttitudeCase(
            "left_yaw_sensitivity_0p01",
            probe.CameraAttitudeErrorDeg(0.0, 0.0, 0.01),
            probe.CameraAttitudeErrorDeg(0.0, 0.0, 0.0),
            ("center",),
        )
        zero_px = probe.analytic_projection("center", zero)
        yaw_px = probe.analytic_projection("center", yaw)
        self.assertAlmostEqual(yaw_px["left"]["u_px"] - zero_px["left"]["u_px"], -1.4388772994, delta=1.0e-6)

    def test_sdf_rpy_matrix_uses_rz_ry_rx(self) -> None:
        roll, pitch, yaw = math.radians(0.3), math.radians(-0.2), math.radians(0.1)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.asarray(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
        ry = np.asarray(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
        rz = np.asarray(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
        actual = np.asarray(probe.rotation_from_sdf_rpy(roll, pitch, yaw))
        self.assertLess(float(np.max(np.abs(actual - (rz @ ry @ rx)))), 1.0e-12)

    def test_quaternion_xyzw_reordered_to_wxyz(self) -> None:
        self.assertEqual(
            probe.quaternion_xyzw_to_wxyz({"x": 1.0, "y": 2.0, "z": 3.0, "w": 4.0}),
            (4.0, 1.0, 2.0, 3.0),
        )

    def test_generated_world_marker_insertion(self) -> None:
        source = """<sdf version="1.9"><world name="w"><model name="camera"/><include><uri>model://x</uri></include></world></sdf>"""
        world = probe.add_marker_to_world_xml(source, (1.0, 2.0, 3.0))
        root = probe.ET.fromstring(world)
        marker = root.find("world/model[@name='phase3c_equivalence_marker']")
        self.assertIsNotNone(marker)
        self.assertEqual(marker.findtext("static"), "true")
        self.assertIn("1.000000000 2.000000000 3.000000000", marker.findtext("pose"))
        self.assertIsNone(marker.find("link/collision"))
        self.assertEqual(marker.findtext("link/visual/material/emissive"), "1 0 1 1")

    def test_production_generate_world_call_path(self) -> None:
        calls = []

        class FakeModule:
            def _generate_world(self, context, generated_world_path=None):
                calls.append((dict(context.launch_configurations), generated_world_path))
                Path(generated_world_path).write_text("<sdf/>", encoding="utf-8")
                return generated_world_path

        original_loader = probe.load_production_launch_module
        original_launch = sys.modules.get("launch")
        launch = types.ModuleType("launch")

        class LaunchContext:
            def __init__(self):
                self.launch_configurations = {}

        launch.LaunchContext = LaunchContext
        sys.modules["launch"] = launch
        try:
            probe.load_production_launch_module = lambda _share: FakeModule()
            path = probe.generate_production_world(probe.smoke_cases()[0], Path("/tmp/phase3c_test_world.sdf"), package_share=Path("/tmp/share"))
        finally:
            probe.load_production_launch_module = original_loader
            if original_launch is None:
                sys.modules.pop("launch", None)
            else:
                sys.modules["launch"] = original_launch
        self.assertEqual(path, Path("/tmp/phase3c_test_world.sdf"))
        self.assertEqual(calls[0][0]["baseline_m"], "25.0")
        self.assertEqual(calls[0][0]["left_yaw_error_deg"], "0.0")

    def test_image_decoding_rgb_bgr_8uc3_and_encoding_mismatch(self) -> None:
        rgb = np.zeros((probe.HEIGHT_PX, probe.WIDTH_PX, 3), dtype=np.uint8)
        rgb[10, 20] = (255, 0, 255)
        arr, status = probe.decode_image_to_rgb_array(DummyImage("rgb8", rgb))
        self.assertEqual(status, "ok")
        self.assertEqual(tuple(arr[10, 20]), (255, 0, 255))
        bgr_source = rgb[:, :, ::-1].copy()
        arr, status = probe.decode_image_to_rgb_array(DummyImage("bgr8", bgr_source))
        self.assertEqual(status, "ok")
        self.assertEqual(tuple(arr[10, 20]), (255, 0, 255))
        arr, status = probe.decode_image_to_rgb_array(DummyImage("8UC3", rgb))
        self.assertEqual(status, "ok")
        self.assertEqual(tuple(arr[10, 20]), (255, 0, 255))
        bad = DummyImage("mono8", rgb)
        arr, status = probe.decode_image_to_rgb_array(bad)
        self.assertIsNone(arr)
        self.assertEqual(status, "encoding_mismatch")

    def test_marker_mask_connected_components_and_weighted_centroid(self) -> None:
        image = np.zeros((probe.HEIGHT_PX, probe.WIDTH_PX, 3), dtype=np.uint8)
        image[100:103, 200:204] = (255, 0, 255)
        image[101, 203] = (255, 0, 255)
        detection = probe.detect_marker(image, threshold=80.0, min_area_px=3)
        self.assertTrue(detection.valid)
        self.assertEqual(detection.status, "ok")
        self.assertAlmostEqual(detection.u_px, 201.5, delta=0.6)
        self.assertAlmostEqual(detection.v_px, 101.0, delta=0.6)
        self.assertEqual(detection.component_count, 1)

    def test_marker_invalid_cases(self) -> None:
        image = np.zeros((probe.HEIGHT_PX, probe.WIDTH_PX, 3), dtype=np.uint8)
        self.assertEqual(probe.detect_marker(image).status, "marker_not_found")
        image[10:12, 10:12] = (255, 0, 255)
        self.assertEqual(probe.detect_marker(image, min_area_px=20).status, "area_out_of_range")
        image = np.zeros((probe.HEIGHT_PX, probe.WIDTH_PX, 3), dtype=np.uint8)
        image[20:26, 20:26] = (255, 0, 255)
        image[50:56, 50:56] = (255, 0, 255)
        self.assertEqual(probe.detect_marker(image, min_area_px=5).status, "multiple_components")
        image = np.zeros((probe.HEIGHT_PX, probe.WIDTH_PX, 3), dtype=np.uint8)
        image[0:10, 0:10] = (255, 0, 255)
        self.assertEqual(probe.detect_marker(image, min_area_px=5).status, "marker_clipped")

    def test_camera_info_intrinsic_delta_extraction(self) -> None:
        intr = probe.intrinsics()
        result = probe.intrinsic_delta_from_camera_info(
            {"k": [intr.fx_px + 1.0, 0.0, intr.cx_px - 2.0, 0.0, intr.fy_px + 3.0, intr.cy_px - 4.0]},
            intr,
        )
        self.assertAlmostEqual(result["fx_delta_px"], 1.0)
        self.assertAlmostEqual(result["fy_delta_px"], 3.0)
        self.assertAlmostEqual(result["cx_delta_px"], -2.0)
        self.assertAlmostEqual(result["cy_delta_px"], -4.0)

    def test_absolute_and_differential_residuals(self) -> None:
        absolute = probe.residual({"u_px": 12.0, "v_px": 9.0}, {"u_px": 10.0, "v_px": 5.0})
        self.assertEqual(absolute["du_px"], 2.0)
        self.assertEqual(absolute["dv_px"], 4.0)
        self.assertAlmostEqual(absolute["pixel_error_px"], math.sqrt(20.0))
        diff = probe.differential_residual(
            {"rendered": {"u_px": 12.0, "v_px": 8.0}, "analytic": {"u_px": 11.0, "v_px": 7.0}},
            {"rendered": {"u_px": 10.0, "v_px": 5.0}, "analytic": {"u_px": 10.0, "v_px": 5.0}},
        )
        self.assertEqual(diff["delta_u_residual_px"], 1.0)
        self.assertEqual(diff["delta_v_residual_px"], 1.0)

    def test_verdicts_equivalent_not_equivalent_and_inconclusive(self) -> None:
        equivalent = {
            "infrastructure_valid": True,
            "pose_coverage_rate": 1.0,
            "marker_detection_success_rate": 0.99,
            "required_case_coverage_rate": 1.0,
            "absolute_residual_summary": {"median": 0.2, "max": 0.5},
            "differential_residual_summary": {"median": 0.1, "max": 0.2},
            "composite_differential_residual_summary": {"median": 0.2, "max": 0.4},
            "centroid_stddev_summary": {"max": 0.05},
            "single_axis_sign_agreement": {"matched": 12, "total": 12},
        }
        self.assertEqual(probe.determine_verdict(equivalent), "equivalent")
        bad = dict(equivalent)
        bad["absolute_residual_summary"] = {"median": 2.0, "max": 3.0}
        self.assertEqual(probe.determine_verdict(bad), "not_equivalent")
        bad["infrastructure_valid"] = False
        self.assertEqual(probe.determine_verdict(bad), "inconclusive")

    def test_json_finite_null_policy(self) -> None:
        payload = {"ok": 1.0, "nan": math.nan, "inf": math.inf, "items": [1.0, -math.inf]}
        encoded = json.dumps(probe.json_safe(payload), allow_nan=False)
        self.assertNotIn("NaN", encoded)
        self.assertNotIn("Infinity", encoded)
        decoded = json.loads(encoded)
        self.assertIsNone(decoded["nan"])
        self.assertIsNone(decoded["inf"])
        self.assertIsNone(decoded["items"][1])

    def test_current_mapping_hypothesis_algebra_metadata(self) -> None:
        error = probe.CameraAttitudeErrorDeg(0.1, -0.2, 0.3)
        self.assertEqual(probe.candidate_sdf_error_from_camera_error(error), (0.1, 0.2, -0.3))

    def test_cleanup_targets_own_process_group_only_source_contract(self) -> None:
        source = SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("start_new_session=True", source)
        self.assertIn("os.killpg(process.pid, signal.SIGTERM)", source)
        self.assertIn("os.killpg(process.pid, signal.SIGKILL)", source)
        self.assertNotIn("pkill", source)
        self.assertNotIn("killall", source)


if __name__ == "__main__":
    unittest.main()
