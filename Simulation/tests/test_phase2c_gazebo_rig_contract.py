from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_SDF = REPO_ROOT / "Simulation/stereo_ground_sim/models/x500_15m/model.sdf"
WORLD_SDF = REPO_ROOT / "Simulation/stereo_ground_sim/worlds/stereo_ground_drone.sdf"


def parse_sdf(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def parse_floats(text: str) -> tuple[float, ...]:
    return tuple(float(part) for part in text.split())


def assert_tuple_close(testcase: unittest.TestCase, actual, expected, delta: float) -> None:
    testcase.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        testcase.assertAlmostEqual(actual_value, expected_value, delta=delta)


class Phase2CGazeboRigContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.model_root = parse_sdf(MODEL_SDF)
        self.world_root = parse_sdf(WORLD_SDF)
        self.model = self.model_root.find("model")
        self.world = self.world_root.find("world")
        self.assertIsNotNone(self.model)
        self.assertIsNotNone(self.world)

    def model_links(self) -> list[ET.Element]:
        return list(self.model.findall("link"))

    def model_joints(self) -> list[ET.Element]:
        return list(self.model.findall("joint"))

    def test_target_reference_link_contract(self) -> None:
        self.assertEqual(self.model.attrib.get("name"), "x500_15m")

        reference_links = [link for link in self.model_links() if link.attrib.get("name") == "target_reference_link"]
        self.assertEqual(len(reference_links), 1)
        reference_link = reference_links[0]

        reference_joints = [
            joint for joint in self.model_joints() if joint.attrib.get("name") == "target_reference_joint"
        ]
        self.assertEqual(len(reference_joints), 1)
        reference_joint = reference_joints[0]
        self.assertEqual(reference_joint.attrib.get("type"), "fixed")
        self.assertEqual(reference_joint.findtext("parent"), "base_link")
        self.assertEqual(reference_joint.findtext("child"), "target_reference_link")

        pose = reference_link.find("pose")
        self.assertIsNotNone(pose)
        self.assertEqual(pose.attrib.get("relative_to"), "base_link")
        pose_values = parse_floats(pose.text or "")
        assert_tuple_close(self, pose_values[:3], (0.0, 0.0, 0.0), 1.0e-9)
        assert_tuple_close(self, pose_values[3:], (0.0, 0.0, 0.0), 1.0e-12)

        self.assertEqual(reference_link.findall("visual"), [])
        self.assertEqual(reference_link.findall("collision"), [])
        self.assertEqual(reference_link.findall("sensor"), [])
        self.assertEqual(reference_link.findtext("gravity"), "false")
        self.assertEqual(reference_link.findtext("self_collide"), "false")

        inertial = reference_link.find("inertial")
        self.assertIsNotNone(inertial)
        mass = float(inertial.findtext("mass"))
        self.assertGreater(mass, 0.0)
        self.assertLessEqual(mass, 1.0e-4)
        inertia = inertial.find("inertia")
        self.assertIsNotNone(inertia)
        for field in ("ixx", "iyy", "izz"):
            value = float(inertia.findtext(field))
            self.assertGreater(value, 0.0)
            self.assertLessEqual(value, 1.0e-8)
        for field in ("ixy", "ixz", "iyz"):
            self.assertEqual(float(inertia.findtext(field)), 0.0)

    def test_existing_px4_model_elements_preserved(self) -> None:
        link_names = {link.attrib.get("name") for link in self.model_links()}
        self.assertTrue({"base_link", "rotor_0", "rotor_1", "rotor_2", "rotor_3"}.issubset(link_names))

        joint_names = {joint.attrib.get("name") for joint in self.model_joints()}
        self.assertTrue({"rotor_0_joint", "rotor_1_joint", "rotor_2_joint", "rotor_3_joint"}.issubset(joint_names))

        motor_plugins = [
            plugin
            for plugin in self.model.findall("plugin")
            if plugin.attrib.get("filename") == "gz-sim-multicopter-motor-model-system"
        ]
        self.assertEqual(len(motor_plugins), 4)

    def test_world_camera_rig_contract(self) -> None:
        self.assertEqual(self.world.findtext("spherical_coordinates/world_frame_orientation"), "ENU")

        camera_models = {
            model.attrib.get("name"): model
            for model in self.world.findall("model")
            if model.attrib.get("name") in {"left_stereo_camera", "right_stereo_camera"}
        }
        self.assertEqual(set(camera_models), {"left_stereo_camera", "right_stereo_camera"})

        expected_poses = {
            "left_stereo_camera": (-12.5, 0.0, 5.0, 0.0, 0.0, 1.570796327),
            "right_stereo_camera": (12.5, 0.0, 5.0, 0.0, 0.0, 1.570796327),
        }
        centers = []
        for name, expected_pose in expected_poses.items():
            model = camera_models[name]
            pose = parse_floats(model.findtext("pose"))
            assert_tuple_close(self, pose, expected_pose, 1.0e-9)
            centers.append(pose[:3])

            sensor = model.find("link/sensor[@name='camera']")
            self.assertIsNotNone(sensor)
            self.assertEqual(sensor.attrib.get("type"), "camera")
            assert_tuple_close(self, parse_floats(sensor.findtext("pose")), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0e-12)
            self.assertEqual(int(sensor.findtext("camera/image/width")), 1280)
            self.assertEqual(int(sensor.findtext("camera/image/height")), 720)
            self.assertEqual(float(sensor.findtext("update_rate")), 20.0)

        midpoint = tuple((left + right) / 2.0 for left, right in zip(centers[0], centers[1]))
        assert_tuple_close(self, midpoint, (0.0, 0.0, 5.0), 1.0e-9)

    def test_target_spawn_default_preserved(self) -> None:
        includes = [include for include in self.world.findall("include") if include.findtext("uri") == "model://x500_15m"]
        self.assertEqual(len(includes), 1)
        target_pose = parse_floats(includes[0].findtext("pose"))
        assert_tuple_close(self, target_pose[:3], (0.0, 2000.0, 0.75), 1.0e-9)
        assert_tuple_close(self, target_pose[3:], (0.0, 0.0, -1.570796327), 1.0e-12)


if __name__ == "__main__":
    unittest.main()
