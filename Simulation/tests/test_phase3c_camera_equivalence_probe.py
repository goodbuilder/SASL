from __future__ import annotations

import importlib.util
import inspect
import json
import math
import sys
import tempfile
import types
import unittest
from collections import deque
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
    def write_camera_info(self, root: Path, *, valid: bool = True) -> None:
        intr = probe.intrinsics()
        info = {}
        for topic in ("/left_camera/camera_info", "/right_camera/camera_info"):
            info[topic] = {
                "width": probe.WIDTH_PX if valid else 1,
                "height": probe.HEIGHT_PX,
                "k": [intr.fx_px, 0.0, intr.cx_px, 0.0, intr.fy_px, intr.cy_px, 0.0, 0.0, 1.0],
                "p": [intr.fx_px, 0.0, intr.cx_px, 0.0, 0.0, intr.fy_px, intr.cy_px, 0.0, 0.0, 0.0, 1.0, 0.0],
            }
        (root / "camera_info.json").write_text(json.dumps(info), encoding="utf-8")

    def make_fake_image_collector(self, *, maxlen: int = 3, initial_count: int = 10):
        collector = object.__new__(probe.RosImageCollector)
        topics = ("/left_camera/image", "/right_camera/image")
        collector.images = {topic: deque(maxlen=maxlen) for topic in topics}
        collector.received_image_counts = {topic: initial_count for topic in topics}
        return collector, topics

    def test_receive_counter_exceeds_deque_maxlen_and_is_topic_independent(self) -> None:
        collector, topics = self.make_fake_image_collector(maxlen=3, initial_count=0)
        for index in range(5):
            collector._record_image(topics[0], index)
        collector._record_image(topics[1], "right")
        self.assertEqual(len(collector.images[topics[0]]), 3)
        self.assertEqual(collector.received_image_counts[topics[0]], 5)
        self.assertEqual(len(collector.images[topics[1]]), 1)
        self.assertEqual(collector.received_image_counts[topics[1]], 1)

    def test_sampling_uses_monotonic_count_when_deque_is_full(self) -> None:
        collector, topics = self.make_fake_image_collector(maxlen=3, initial_count=10)
        image = np.zeros((probe.HEIGHT_PX, probe.WIDTH_PX, 3), dtype=np.uint8)
        image[350:354, 630:634] = (255, 0, 255)
        messages = [DummyImage("rgb8", image), DummyImage("rgb8", image)]
        for topic in topics:
            collector.images[topic].extend([(0.0, messages[0])] * 3)

        def spin_until(predicate, timeout_s):
            for message in messages:
                for topic in topics:
                    collector._record_image(topic, message)
            return predicate()

        collector.spin_until = spin_until
        with tempfile.TemporaryDirectory() as tmpdir:
            result = probe.sample_marker_from_images(
                collector,
                discard_frames=0,
                sample_frames=2,
                marker_threshold=80.0,
                output_dir=Path(tmpdir),
                case_name="zero",
                marker_name="center",
            )
        self.assertTrue(result["valid"])
        self.assertEqual(result["frame_wait_diagnostics"]["start_received_counts"][topics[0]], 10)
        self.assertEqual(result["frame_wait_diagnostics"]["target_received_counts"][topics[0]], 12)
        self.assertEqual(result["frame_wait_diagnostics"]["final_received_counts"][topics[0]], 12)
        self.assertEqual(result["frame_wait_diagnostics"]["final_deque_lengths"][topics[0]], 3)

    def test_image_wait_diagnostics_include_counts_and_deque_length(self) -> None:
        collector, topics = self.make_fake_image_collector(maxlen=3, initial_count=250)
        start = {topic: 250 for topic in topics}
        target = {topic: 270 for topic in topics}
        diagnostics = probe._image_wait_diagnostics(
            collector, topics, start, target, discard_frames=10, sample_frames=10,
        )
        self.assertEqual(diagnostics["target_received_counts"][topics[0]], 270)
        self.assertEqual(diagnostics["final_received_counts"][topics[0]], 250)
        self.assertEqual(diagnostics["final_deque_lengths"][topics[0]], 0)
        self.assertEqual(diagnostics["deque_maxlen"], 3)

    def test_sampling_waits_on_received_counts_not_deque_length(self) -> None:
        marker_source = inspect.getsource(probe.sample_marker_from_images)
        absolute_source = inspect.getsource(probe.sample_absolute_offset_from_images)
        old_wait = "len(collector.images[topic]) + discard_frames + sample_frames"
        self.assertNotIn(old_wait, marker_source)
        self.assertNotIn(old_wait, absolute_source)
        self.assertIn("collector.received_image_counts", marker_source)
        self.assertIn("collector.received_image_counts", absolute_source)

    def test_timeout_returns_frame_wait_diagnostics(self) -> None:
        collector, topics = self.make_fake_image_collector(maxlen=3, initial_count=250)
        collector.spin_until = lambda predicate, timeout_s: False
        with tempfile.TemporaryDirectory() as tmpdir:
            result = probe.sample_marker_from_images(
                collector,
                discard_frames=10,
                sample_frames=10,
                marker_threshold=80.0,
                output_dir=Path(tmpdir),
                case_name="zero",
                marker_name="center",
            )
        diagnostics = result["frame_wait_diagnostics"]
        self.assertEqual(result["status"], "image_topic_timeout")
        self.assertEqual(diagnostics["start_received_counts"][topics[0]], 250)
        self.assertEqual(diagnostics["target_received_counts"][topics[0]], 270)
        self.assertEqual(diagnostics["final_received_counts"][topics[0]], 250)
        self.assertEqual(diagnostics["final_deque_lengths"][topics[0]], 0)

    def make_result(
        self,
        *,
        case: str = "zero",
        marker: str = "center",
        valid_frames: int = 10,
        requested_frames: int = 10,
        pose_valid: bool = True,
        left_shift: tuple[float, float] = (0.0, 0.0),
        right_shift: tuple[float, float] = (0.0, 0.0),
        analytic_left_shift: tuple[float, float] = (0.0, 0.0),
        analytic_right_shift: tuple[float, float] = (0.0, 0.0),
    ) -> dict:
        analytic = probe.analytic_projection(marker, probe.AttitudeCase(case, probe.CameraAttitudeErrorDeg(0.0, 0.0, 0.0), probe.CameraAttitudeErrorDeg(0.0, 0.0, 0.0), (marker,)))
        left = analytic["left"]
        right = analytic["right"]
        analytic = json.loads(json.dumps(analytic))
        analytic["left"]["u_px"] += analytic_left_shift[0]
        analytic["left"]["v_px"] += analytic_left_shift[1]
        analytic["right"]["u_px"] += analytic_right_shift[0]
        analytic["right"]["v_px"] += analytic_right_shift[1]

        def camera_payload(base, shift):
            return {
                "valid": valid_frames / requested_frames >= 0.95,
                "u_px": base["u_px"] + shift[0],
                "v_px": base["v_px"] + shift[1],
                "stddev_px": 0.0,
                "requested_frame_count": requested_frames,
                "decoded_frame_count": requested_frames,
                "valid_detection_frame_count": valid_frames,
                "invalid_detection_frame_count": requested_frames - valid_frames,
                "frame_detection_success_rate": valid_frames / requested_frames,
            }

        checks = [{"name": f"check_{idx}", "value": 0.0, "tolerance": 1.0, "passed": pose_valid} for idx in range(probe.REQUIRED_POSE_CHECKS_PER_MARKER_SAMPLE)]
        return {
            "case": case,
            "marker": marker,
            "valid": pose_valid and valid_frames / requested_frames >= 0.95,
            "pose_evidence": {
                "valid": pose_valid,
                "confirmed_check_count": probe.REQUIRED_POSE_CHECKS_PER_MARKER_SAMPLE if pose_valid else 0,
                "required_check_count": probe.REQUIRED_POSE_CHECKS_PER_MARKER_SAMPLE,
                "checks": checks,
                "left_generated_runtime_center_error_m": 0.0 if pose_valid else None,
                "right_generated_runtime_center_error_m": 0.0 if pose_valid else None,
                "left_generated_runtime_rotation_max_error": 0.0 if pose_valid else None,
                "right_generated_runtime_rotation_max_error": 0.0 if pose_valid else None,
                "left_model_link_center_error_m": 0.0 if pose_valid else None,
                "right_model_link_center_error_m": 0.0 if pose_valid else None,
                "left_model_link_rotation_max_error": 0.0 if pose_valid else None,
                "right_model_link_rotation_max_error": 0.0 if pose_valid else None,
                "baseline_length_error_m": 0.0 if pose_valid else None,
                "baseline_midpoint_error_m": 0.0 if pose_valid else None,
                "marker_commanded_runtime_position_error_m": 0.0 if pose_valid else None,
            },
            "sample": {
                "cameras": {
                    "/left_camera/image": camera_payload(left, left_shift),
                    "/right_camera/image": camera_payload(right, right_shift),
                }
            },
            "analytic_projection": analytic,
            "log_paths": {"gazebo": "/tmp/gazebo.log", "bridge": "/tmp/bridge.log"},
        }

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

    def test_standard_cases_use_right_positive_for_roll_observability(self) -> None:
        cases = probe.standard_cases()
        self.assertEqual(cases[0].name, "zero")
        self.assertIn("right_positive", cases[0].marker_names)
        single_axis = [
            case
            for case in cases
            if case.name.startswith(("left_", "right_"))
            and case.name.endswith(("_positive", "_negative"))
        ]
        self.assertEqual(len(single_axis), 12)
        for case in single_axis:
            if "_roll_" in case.name:
                self.assertEqual(case.marker_names, ("right_positive",))
            else:
                self.assertEqual(case.marker_names, ("center",))

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

    def test_generated_and_runtime_pose_residuals(self) -> None:
        world = probe.add_marker_to_world_xml(
            """<sdf version="1.9"><world name="w"><model name="left_stereo_camera"><pose>-12.5 0 5 0 0 0</pose></model><model name="right_stereo_camera"><pose>12.5 0 5 0 0 0</pose></model></world></sdf>""",
            (20.0, 1852.0, 11.0),
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "world.sdf"
            path.write_text(world, encoding="utf-8")
            generated = probe.parse_generated_model_poses(path)
        runtime = {
            "left_stereo_camera": {"name": "left_stereo_camera", "position": {"x": -12.5, "y": 0.0, "z": 5.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
            "left_stereo_camera::link": {"name": "left_stereo_camera::link", "position": {"x": -12.5, "y": 0.0, "z": 5.0}, "orientation": {"w": 1}},
            "right_stereo_camera": {"name": "right_stereo_camera", "position": {"x": 12.5, "y": 0.0, "z": 5.0}, "orientation": {"w": 1}},
            "right_stereo_camera::link": {"name": "right_stereo_camera::link", "position": {"x": 12.5, "y": 0.0, "z": 5.0}, "orientation": {"w": 1}},
            probe.MARKER_MODEL_NAME: {"name": probe.MARKER_MODEL_NAME, "position": {"x": 20.0, "y": 1852.0, "z": 11.0}, "orientation": {"w": 1}},
        }
        evidence = probe.pose_evidence_from_records(
            generated_poses=generated,
            runtime_records=runtime,
            commanded_marker_world_enu_m=(20.0, 1852.0, 11.0),
        )
        self.assertTrue(evidence["valid"])
        self.assertEqual(evidence["confirmed_check_count"], probe.REQUIRED_POSE_CHECKS_PER_MARKER_SAMPLE)
        self.assertAlmostEqual(evidence["baseline_length_error_m"], 0.0)
        self.assertAlmostEqual(evidence["marker_commanded_runtime_position_error_m"], 0.0)

    def test_missing_or_stale_pose_is_infrastructure_invalid(self) -> None:
        generated = {
            "left_stereo_camera": {"position_world_enu_m": (-12.5, 0.0, 5.0), "rotation_world_model": np.eye(3).tolist()},
            "right_stereo_camera": {"position_world_enu_m": (12.5, 0.0, 5.0), "rotation_world_model": np.eye(3).tolist()},
            probe.MARKER_MODEL_NAME: {"position_world_enu_m": (20.0, 1852.0, 11.0), "rotation_world_model": np.eye(3).tolist()},
        }
        runtime = {
            "left_stereo_camera": {"position": {"x": -12.5, "y": 0, "z": 5}, "orientation": {"w": 1}},
            "left_stereo_camera::link": {"position": {"x": -12.5, "y": 0, "z": 5}, "orientation": {"w": 1}},
            "right_stereo_camera": {"position": {"x": 12.5, "y": 0, "z": 5}, "orientation": {"w": 1}},
            "right_stereo_camera::link": None,
            probe.MARKER_MODEL_NAME: {"position": {"x": 21.0, "y": 1852.0, "z": 11.0}, "orientation": {"w": 1}},
        }
        evidence = probe.pose_evidence_from_records(
            generated_poses=generated,
            runtime_records=runtime,
            commanded_marker_world_enu_m=(20.0, 1852.0, 11.0),
        )
        self.assertFalse(evidence["valid"])
        self.assertLess(evidence["confirmed_check_count"], evidence["required_check_count"])
        self.assertGreater(evidence["marker_commanded_runtime_position_error_m"], 0.9)

    def test_static_identity_link_records_are_completed_from_exact_model_pose(self) -> None:
        generated = {
            "left_stereo_camera": {"has_identity_link": True},
            "right_stereo_camera": {"has_identity_link": True},
        }
        runtime = {
            "left_stereo_camera": {"name": "left_stereo_camera", "position": {"x": -12.5, "y": 0.0, "z": 5.0}, "orientation": {"w": 1}},
            "left_stereo_camera::link": None,
            "right_stereo_camera": {"name": "right_stereo_camera", "position": {"x": 12.5, "y": 0.0, "z": 5.0}, "orientation": {"w": 1}},
            "right_stereo_camera::link": None,
        }
        completed = probe.complete_static_identity_link_records(generated, runtime)
        self.assertEqual(completed["left_stereo_camera::link"]["name"], "left_stereo_camera::link")
        self.assertEqual(completed["left_stereo_camera::link"]["pose_source"], "implicit_static_identity_link_from_model_pose")

    def test_camera_info_validation(self) -> None:
        intr = probe.intrinsics()
        valid_info = {
            "/left_camera/camera_info": {"width": 1280, "height": 720, "k": [intr.fx_px, 0, intr.cx_px, 0, intr.fy_px, intr.cy_px, 0, 0, 1], "p": [intr.fx_px, 0, intr.cx_px, 0, 0, intr.fy_px, intr.cy_px, 0, 0, 0, 1, 0]},
            "/right_camera/camera_info": {"width": 1280, "height": 720, "k": [intr.fx_px, 0, intr.cx_px, 0, intr.fy_px, intr.cy_px, 0, 0, 1], "p": [intr.fx_px, 0, intr.cx_px, 0, 0, intr.fy_px, intr.cy_px, 0, 0, 0, 1, 0]},
        }
        self.assertTrue(probe.validate_camera_info(valid_info)["valid"])
        missing = probe.validate_camera_info({"/left_camera/camera_info": valid_info["/left_camera/camera_info"]})
        self.assertFalse(missing["valid"])
        bad_dims = json.loads(json.dumps(valid_info))
        bad_dims["/left_camera/camera_info"]["width"] = 1
        self.assertEqual(probe.validate_camera_info(bad_dims)["sides"]["left"]["status"], "dimension_mismatch")
        bad_k = json.loads(json.dumps(valid_info))
        bad_k["/right_camera/camera_info"]["k"][0] = math.nan
        self.assertEqual(probe.validate_camera_info(bad_k)["sides"]["right"]["status"], "invalid_k_or_p")

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

    def test_pixel_coordinate_helper_preserves_fractional_and_negative_values(self) -> None:
        converted = probe.array_index_to_pixel_center_px(1.25, -2.75)
        self.assertEqual(converted.convention, probe.PIXEL_CONVENTION_COMPARISON_CENTER)
        self.assertEqual(converted.u_px, 1.75)
        self.assertEqual(converted.v_px, -2.25)

    def test_pixel_coordinate_helper_validates_without_rounding_or_clipping(self) -> None:
        for value in (True, "1", math.nan, math.inf, -math.inf):
            with self.assertRaises((TypeError, ValueError)):
                probe.array_index_to_pixel_center_px(value, 0.0)
        with self.assertRaises(ValueError):
            probe.PixelCoordinate2D(0.0, 0.0, "")
        self.assertEqual(probe.array_index_to_pixel_center_px(0.0, 0.0).u_px, 0.5)

    def test_normalized_verdict_is_separate_from_raw_verdict(self) -> None:
        summary = {
            "infrastructure_valid": True,
            "pose_coverage_rate": 1.0,
            "marker_detection_success_rate": 1.0,
            "required_case_coverage_rate": 1.0,
            "absolute_residual_summary": {"median": 0.70, "max": 0.80},
            "raw_absolute_residual_summary": {"median": 0.70, "max": 0.80},
            "normalized_absolute_residual_summary": {"median": 0.05, "max": 0.20},
            "differential_residual_summary": {"median": 0.1, "max": 0.2},
            "composite_differential_residual_summary": {"median": 0.2, "max": 0.4},
            "centroid_stddev_summary": {"max": 0.05},
            "single_axis_sign_agreement": {"matched": 12, "total": 12},
            "pixel_coordinate_contract": {
                "rendered_detector_output_convention": probe.PIXEL_CONVENTION_INTEGER_INDEX_LATTICE,
                "rendered_comparison_convention": probe.PIXEL_CONVENTION_COMPARISON_CENTER,
                "analytic_projection_convention": probe.PIXEL_CONVENTION_ANALYTIC_CONTINUOUS,
                "integer_index_to_comparison_center_offset_px": [0.5, 0.5],
                "evidence_commit": probe.PIXEL_CONVENTION_EVIDENCE_COMMIT,
                "evidence_cause_verdict": probe.PIXEL_CONVENTION_EVIDENCE_CAUSE_VERDICT,
            },
        }
        self.assertEqual(probe.determine_verdict(summary), "not_equivalent")
        self.assertEqual(probe.determine_pixel_convention_normalized_verdict(summary), "equivalent")

    def test_normalized_verdict_rejects_missing_or_invalid_contract(self) -> None:
        summary = {
            "infrastructure_valid": True,
            "normalized_absolute_residual_summary": {"median": 0.1, "max": 0.2},
        }
        self.assertEqual(probe.determine_pixel_convention_normalized_verdict(summary), "inconclusive")
        summary["pixel_coordinate_contract"] = {"evidence_commit": "wrong"}
        self.assertEqual(probe.determine_pixel_convention_normalized_verdict(summary), "inconclusive")

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

    def test_attitude_semantics_verdict_is_separate_from_absolute_equivalence(self) -> None:
        complete = {
            "infrastructure_valid": True,
            "single_axis_sign_agreement": {"matched": 12, "total": 12, "evaluable": 12},
            "differential_residual_summary": {"median": 0.20, "max": 0.70},
            "composite_differential_residual_summary": {"median": 0.40, "max": 0.90},
            "centroid_stddev_summary": {"max": 0.09},
            "absolute_residual_summary": {"median": 0.80, "max": 0.90},
        }
        self.assertEqual(probe.determine_attitude_semantics_verdict(complete), "aligned")
        self.assertEqual(
            probe.determine_verdict({
                **complete,
                "pose_coverage_rate": 1.0,
                "marker_detection_success_rate": 0.99,
                "required_case_coverage_rate": 1.0,
            }),
            "not_equivalent",
        )

        direction_mismatch = {
            **complete,
            "single_axis_sign_agreement": {"matched": 11, "total": 12, "evaluable": 12},
        }
        self.assertEqual(probe.determine_attitude_semantics_verdict(direction_mismatch), "not_aligned")

        nine_evaluable = {
            **complete,
            "single_axis_sign_agreement": {"matched": 9, "total": 12, "evaluable": 9},
        }
        self.assertEqual(probe.determine_attitude_semantics_verdict(nine_evaluable), "inconclusive")

        invalid_infrastructure = {**complete, "infrastructure_valid": False}
        self.assertEqual(probe.determine_attitude_semantics_verdict(invalid_infrastructure), "inconclusive")

        differential_failure = {
            **complete,
            "differential_residual_summary": {"median": 0.20, "max": 0.80},
        }
        self.assertEqual(probe.determine_attitude_semantics_verdict(differential_failure), "not_aligned")

        missing_composite = {
            **complete,
            "composite_differential_residual_summary": {"median": None, "max": None},
        }
        self.assertEqual(probe.determine_attitude_semantics_verdict(missing_composite), "inconclusive")

    def test_summary_uses_frame_detection_pose_camera_info_and_required_case_coverage(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_camera_info(root)
            results = [self.make_result(case="zero", marker=name) for name in probe.MARKER_POINTS_BASELINE_FRD_M]
            for case in probe.standard_cases()[1:]:
                marker = case.marker_names[0]
                if case.name.startswith("left_") and case.name.endswith(("_positive", "_negative")):
                    results.append(self.make_result(case=case.name, marker=marker, left_shift=(1.0, 0.0), analytic_left_shift=(1.0, 0.0)))
                elif case.name.startswith("right_") and case.name.endswith(("_positive", "_negative")):
                    results.append(self.make_result(case=case.name, marker=marker, right_shift=(1.0, 0.0), analytic_right_shift=(1.0, 0.0)))
                else:
                    results.append(self.make_result(case=case.name, marker=marker))
            summary = probe.summarize_runtime_results(results, root, "standard", {"marker_detection_success_min": 0.95})
            self.assertEqual(summary["pose_coverage_rate"], 1.0)
            self.assertEqual(summary["required_case_coverage_rate"], 1.0)
            self.assertEqual(summary["marker_detection_success_rate"], 1.0)
            self.assertTrue(summary["camera_info_valid"])
            self.assertTrue(summary["infrastructure_valid"])
            self.assertEqual(summary["base_commit"], "c5e0e609b3990948347d26cc5d57865ef18ee280")
            self.assertEqual(summary["roll_observability_marker"], "right_positive")
            self.assertIsInstance(summary["pre_adapter_evidence_available"], bool)
            self.assertEqual(summary["pre_adapter_evidence_reference"], "/tmp/sasl-phase3c-a-standard-review-fix")
            self.assertFalse(summary["gazebo_sdf_equivalence_verified"])
            self.assertEqual(summary["attitude_semantics_verdict"], "aligned")
            self.assertEqual(summary["equivalence_verdict"], summary["raw_equivalence_verdict"])
            self.assertEqual(summary["absolute_residual_summary"], summary["raw_absolute_residual_summary"])
            self.assertIn("pixel_coordinate_contract", summary)
            self.assertIn("pixel_coordinate_observations", summary)
            self.assertFalse(summary["full_rendered_pixel_equivalence_verified"])

            root2 = Path(tmpdir) / "bad"
            root2.mkdir()
            self.write_camera_info(root2)
            bad_results = [self.make_result(case="zero", marker="center", valid_frames=1)]
            bad_summary = probe.summarize_runtime_results(bad_results, root2, "smoke", {})
            self.assertAlmostEqual(bad_summary["marker_detection_success_rate"], 0.1)
            self.assertFalse(bad_summary["infrastructure_valid"])
            self.assertEqual(bad_summary["equivalence_verdict"], "inconclusive")

    def test_required_case_coverage_is_not_case_name_only(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_camera_info(root)
            results = [self.make_result(case="zero", marker="center")]
            summary = probe.summarize_runtime_results(results, root, "standard", {})
            self.assertLess(summary["required_case_coverage_rate"], 1.0)
            self.assertEqual(summary["equivalence_verdict"], "inconclusive")

    def test_single_axis_direction_evaluability(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self.write_camera_info(root)
            zero = self.make_result(case="zero", marker="center")
            aligned = self.make_result(
                case="left_yaw_positive",
                marker="center",
                left_shift=(-2.0, 0.5),
                analytic_left_shift=(-1.0, 0.25),
            )
            opposite = self.make_result(
                case="left_yaw_negative",
                marker="center",
                left_shift=(+2.0, -0.5),
                analytic_left_shift=(-1.0, 0.25),
            )
            near_zero = self.make_result(
                case="left_roll_positive",
                marker="center",
                left_shift=(0.1, 0.0),
                analytic_left_shift=(0.1, 0.0),
            )
            summary = probe.summarize_runtime_results([zero, aligned, opposite, near_zero], root, "smoke", {})
            details = {item["case"]: item for item in summary["single_axis_sign_details"]}
            self.assertTrue(details["left_yaw_positive"]["matched"])
            self.assertFalse(details["left_yaw_negative"]["matched"])
            self.assertFalse(details["left_roll_positive"]["evaluable"])

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

    def test_absolute_offset_radius_and_case_matrix(self) -> None:
        source = "<sdf version=\"1.9\"><world name=\"w\"/></sdf>"
        with tempfile.TemporaryDirectory() as tmpdir:
            base, out = Path(tmpdir) / "base.sdf", Path(tmpdir) / "out.sdf"
            base.write_text(source, encoding="utf-8")
            probe.write_probe_world(base, out, (0.0, 0.0, 3.0), marker_radius_m=2.0)
            root = probe.ET.fromstring(out.read_text(encoding="utf-8"))
            self.assertEqual(root.findtext("world/model/link/visual/geometry/sphere/radius"), "2.000000000")
            with self.assertRaises(ValueError):
                probe.write_probe_world(base, out, (0.0, 0.0, 3.0), marker_radius_m=3.0)
        cases = probe.case_matrix("absolute_offset", ("center",))
        self.assertEqual(cases[0].name, "absolute_offset")
        self.assertEqual(cases[0].marker_names, ("center",))

    def test_absolute_offset_csv_validation(self) -> None:
        self.assertEqual(probe.parse_csv_values("40,80", kind="threshold-sweep"), (40.0, 80.0))
        with self.assertRaises(ValueError):
            probe.parse_csv_values("center,center", kind="marker-set")
        with self.assertRaises(ValueError):
            probe.parse_csv_values("not_a_marker", kind="marker-set")
        with self.assertRaises(ValueError):
            probe.parse_csv_values("bad_mode", kind="centroid-modes")

    def test_centroid_modes_use_same_component_and_core_failure(self) -> None:
        image = np.zeros((probe.HEIGHT_PX, probe.WIDTH_PX, 3), dtype=np.uint8)
        image[100:104, 200:204] = (255, 0, 255)
        binary = probe.detect_marker_variant(image, threshold=80, centroid_mode="binary_component_centroid", min_area_px=3)
        weighted = probe.detect_marker(image, threshold=80, min_area_px=3)
        self.assertTrue(binary.valid and weighted.valid)
        self.assertEqual(binary.bbox, weighted.bbox)
        self.assertEqual(binary.pixel_count, weighted.pixel_count)
        core = probe.detect_marker_variant(image, threshold=80, centroid_mode="high_confidence_core", min_area_px=3)
        self.assertTrue(core.valid)
        tiny = np.zeros_like(image)
        tiny[100:102, 200:202] = (160, 100, 160)
        self.assertEqual(probe.detect_marker_variant(tiny, threshold=40, centroid_mode="high_confidence_core", min_area_px=3).status, "high_confidence_core_too_small")


if __name__ == "__main__":
    unittest.main()
