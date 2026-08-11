from __future__ import annotations

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import (
    rpy_to_rotation_world_from_link,
    sdf_link_rpy_from_optical_error,
    sdf_link_rpy_from_px4_frd_error,
)


def _launch_module():
    path = PACKAGE_ROOT / "launch" / "stereo_ground_sim.launch.py"
    spec = importlib.util.spec_from_file_location("camera_px4_optical_launch", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **overrides: float) -> LaunchContext:
    context = LaunchContext()
    declarations = {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    for name, declaration in declarations.items():
        context.launch_configurations[name] = "".join(
            substitution.perform(context) for substitution in declaration.default_value
        )
    context.launch_configurations.update({name: str(value) for name, value in overrides.items()})
    return context


def _camera_model(root: ET.Element, name: str) -> ET.Element:
    models = [model for model in root.findall("./world/model") if model.attrib.get("name") == name]
    assert len(models) == 1
    return models[0]


def _pose_rpy(element: ET.Element) -> tuple[float, float, float]:
    values = tuple(float(value) for value in element.findtext("pose").split())
    return values[3], values[4], values[5]


def _mat_mul(left, right):
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def _global_optical_rotation(model: ET.Element):
    parent = rpy_to_rotation_world_from_link(*_pose_rpy(model))
    optical = model.find("./link[@name='optical_link']")
    assert optical is not None
    relative = rpy_to_rotation_world_from_link(*_pose_rpy(optical))
    return _mat_mul(parent, relative)


def _assert_matrix_close(actual, expected) -> None:
    for actual_row, expected_row in zip(actual, expected):
        assert actual_row == pytest.approx(expected_row, abs=1.0e-9)


def test_one_canonical_default_env_and_no_legacy_attitude_names() -> None:
    root_env_files = sorted(path.name for path in (ROOT / "config").glob("*.env"))
    assert root_env_files == ["simulation.env"]
    config = (ROOT / "config" / "simulation.env").read_text(encoding="utf-8")
    for side in ("L", "R"):
        for axis in ("ROLL", "PITCH", "YAW"):
            assert f"CAM_{side}_OPTICAL_{axis}_DEG=" in config
            assert f"CAM_{side}_PX4_ATTITUDE_{axis}_DEG=" in config
            assert f"CAM_{side}_PX4_ATTITUDE_ERROR_{axis}_DEG=" not in config
    assert "LEFT_ROLL_ERROR_DEG" not in config
    assert "RIGHT_ROLL_ERROR_DEG" not in config


def test_px4_attitude_names_are_independent_and_error_names_are_removed() -> None:
    config = (ROOT / "config" / "simulation.env").read_text(encoding="utf-8")
    for side in ("L", "R"):
        for axis in ("ROLL", "PITCH", "YAW"):
            assert f"CAM_{side}_PX4_{axis}_DEG=" not in config
            assert f"CAM_{side}_PX4_ATTITUDE_ERROR_{axis}_DEG=" not in config


def test_every_runner_uses_the_unified_config() -> None:
    for name in (
        "run_operator_view.sh",
        "run_mission.sh",
        "run_px4_waypoint_mission.sh",
    ):
        source = (ROOT / name).read_text(encoding="utf-8")
        assert "config/simulation.env" in source
        for legacy in (
            "config/camera_attitude.env",
            "config/camera_model.env",
            "config/drone_mission.env",
            "config/yolo.env",
            "config/waypoints.env",
            "config/attitude_error.env",
        ):
            assert legacy not in source


def test_camera_models_expose_px4_sensors_and_separate_optical_links() -> None:
    root = ET.parse(PACKAGE_ROOT / "worlds" / "stereo_ground_drone.sdf").getroot()
    required_sensors = {
        "imu_sensor",
        "magnetometer_sensor",
        "air_pressure_sensor",
        "navsat_sensor",
    }
    for model_name in ("left_stereo_camera", "right_stereo_camera"):
        model = _camera_model(root, model_name)
        base = model.find("./link[@name='base_link']")
        optical = model.find("./link[@name='optical_link']")
        assert base is not None and optical is not None
        assert {sensor.attrib["name"] for sensor in base.findall("./sensor")} == required_sensors
        assert optical.find("./sensor[@name='camera']") is not None
        joint = model.find("./joint[@name='optical_mount']")
        assert joint is not None and joint.attrib["type"] == "fixed"
        assert joint.findtext("parent") == "base_link"
        assert joint.findtext("child") == "optical_link"
        pose_publisher = model.find("./plugin[@name='gz::sim::systems::PosePublisher']")
        assert pose_publisher is not None
        assert pose_publisher.findtext("publish_model_pose") == "true"
        assert pose_publisher.findtext("publish_link_pose") == "true"
        assert pose_publisher.findtext("use_pose_vector_msg") == "true"
        assert pose_publisher.findtext("static_publisher") == "true"


def test_camera_px4_sensors_include_nonzero_noise_for_px4_freshness() -> None:
    root = ET.parse(PACKAGE_ROOT / "worlds" / "stereo_ground_drone.sdf").getroot()
    for model_name in ("left_stereo_camera", "right_stereo_camera"):
        model = _camera_model(root, model_name)
        base = model.find("./link[@name='base_link']")
        assert base is not None
        noise_paths = (
            "./sensor[@name='air_pressure_sensor']/air_pressure/pressure/noise/stddev",
            "./sensor[@name='magnetometer_sensor']/magnetometer/x/noise/stddev",
            "./sensor[@name='magnetometer_sensor']/magnetometer/y/noise/stddev",
            "./sensor[@name='magnetometer_sensor']/magnetometer/z/noise/stddev",
            "./sensor[@name='imu_sensor']/imu/angular_velocity/x/noise/stddev",
            "./sensor[@name='imu_sensor']/imu/angular_velocity/y/noise/stddev",
            "./sensor[@name='imu_sensor']/imu/angular_velocity/z/noise/stddev",
            "./sensor[@name='imu_sensor']/imu/linear_acceleration/x/noise/stddev",
            "./sensor[@name='imu_sensor']/imu/linear_acceleration/y/noise/stddev",
            "./sensor[@name='imu_sensor']/imu/linear_acceleration/z/noise/stddev",
        )
        assert all(float(base.findtext(path)) > 0.0 for path in noise_paths)


def test_generated_world_uses_independent_body_frd_mag_and_gyro_noise_bias(tmp_path: Path, monkeypatch) -> None:
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PACKAGE_ROOT))
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    generated = module._generate_world(
        _context(
            module,
            cam_l_mag_noise_x_gauss=0.001,
            cam_l_mag_noise_y_gauss=0.002,
            cam_l_mag_noise_z_gauss=0.003,
            cam_r_mag_noise_x_gauss=0.004,
            cam_r_mag_noise_y_gauss=0.005,
            cam_r_mag_noise_z_gauss=0.006,
            cam_l_mag_bias_mean_x_gauss=-0.011,
            cam_l_mag_bias_stddev_y_gauss=0.012,
            cam_r_mag_bias_mean_z_gauss=0.013,
            cam_r_mag_bias_stddev_x_gauss=0.014,
            cam_l_gyro_noise_z_rad_s=0.021,
            cam_l_gyro_bias_mean_y_rad_s=-0.022,
            cam_l_gyro_bias_stddev_x_rad_s=0.023,
            cam_r_gyro_noise_x_rad_s=0.024,
            cam_r_gyro_bias_mean_z_rad_s=0.025,
            cam_r_gyro_bias_stddev_y_rad_s=0.026,
        )
    )
    root = ET.parse(generated).getroot()
    for model_name, expected in (
        ("left_stereo_camera", (0.001, 0.002, 0.003)),
        ("right_stereo_camera", (0.004, 0.005, 0.006)),
    ):
        model = _camera_model(root, model_name)
        actual = tuple(
            float(model.findtext(
                f"./link[@name='base_link']/sensor[@name='magnetometer_sensor']/magnetometer/{axis}/noise/stddev"
            ))
            for axis in ("x", "y", "z")
        )
        assert actual == pytest.approx(expected)
    expected_fields = {
        "left_stereo_camera": {
            ("magnetometer_sensor", "magnetometer", "x", "bias_mean"): -0.011,
            ("magnetometer_sensor", "magnetometer", "y", "bias_stddev"): 0.012,
            ("imu_sensor", "imu/angular_velocity", "z", "stddev"): 0.021,
            ("imu_sensor", "imu/angular_velocity", "y", "bias_mean"): -0.022,
            ("imu_sensor", "imu/angular_velocity", "x", "bias_stddev"): 0.023,
        },
        "right_stereo_camera": {
            ("magnetometer_sensor", "magnetometer", "z", "bias_mean"): 0.013,
            ("magnetometer_sensor", "magnetometer", "x", "bias_stddev"): 0.014,
            ("imu_sensor", "imu/angular_velocity", "x", "stddev"): 0.024,
            ("imu_sensor", "imu/angular_velocity", "z", "bias_mean"): 0.025,
            ("imu_sensor", "imu/angular_velocity", "y", "bias_stddev"): 0.026,
        },
    }
    for model_name, fields in expected_fields.items():
        model = _camera_model(root, model_name)
        for (sensor_name, group, axis, field), expected in fields.items():
            path = (
                f"./link[@name='base_link']/sensor[@name='{sensor_name}']/"
                f"{group}/{axis}/noise/{field}"
            )
            assert float(model.findtext(path)) == pytest.approx(expected)


@pytest.mark.parametrize(
    "override",
    (
        {"cam_l_mag_noise_y_gauss": -0.001},
        {"cam_r_mag_bias_stddev_z_gauss": -0.001},
        {"cam_l_gyro_noise_x_rad_s": -0.001},
        {"cam_r_gyro_bias_stddev_y_rad_s": -0.001},
    ),
)
def test_generated_world_rejects_negative_sensor_stddev(tmp_path: Path, monkeypatch, override) -> None:
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PACKAGE_ROOT))
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    with pytest.raises(ValueError, match="standard deviations"):
        module._generate_world(_context(module, **override))


def test_generated_world_rejects_non_finite_sensor_value(tmp_path: Path, monkeypatch) -> None:
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PACKAGE_ROOT))
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    with pytest.raises(ValueError, match="must be finite"):
        module._generate_world(_context(module, cam_l_mag_bias_mean_x_gauss="nan"))


def test_camera_px4_sensors_use_sdf_defined_elements_only() -> None:
    root = ET.parse(PACKAGE_ROOT / "worlds" / "stereo_ground_drone.sdf").getroot()
    for model_name in ("left_stereo_camera", "right_stereo_camera"):
        model = _camera_model(root, model_name)
        base = model.find("./link[@name='base_link']")
        assert base is not None
        assert base.findall(".//gz_frame_id") == []


def test_px4_frd_errors_are_explicitly_converted_to_gazebo_flu() -> None:
    nominal_yaw = math.pi / 2.0
    roll, pitch, yaw = sdf_link_rpy_from_px4_frd_error(
        nominal_yaw,
        math.radians(3.0),
        math.radians(4.0),
        math.radians(5.0),
    )
    assert roll == pytest.approx(math.radians(3.0))
    assert pitch == pytest.approx(-math.radians(4.0))
    assert yaw == pytest.approx(nominal_yaw - math.radians(5.0))


def test_optical_only_attitude_does_not_change_px4_body_pose(tmp_path: Path, monkeypatch) -> None:
    """An optical-only change rotates the camera sensor but leaves the
    independently configured PX4 body pose unchanged."""

    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PACKAGE_ROOT))
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    generated = module._generate_world(_context(module, cam_l_optical_yaw_deg=7.0))
    model = _camera_model(ET.parse(generated).getroot(), "left_stereo_camera")
    assert _pose_rpy(model) == pytest.approx((0.0, 0.0, math.pi / 2.0), abs=1.0e-9)
    optical = model.find("./link[@name='optical_link']")
    assert optical is not None
    assert _pose_rpy(optical) != pytest.approx((0.0, 0.0, 0.0), abs=1.0e-9)
    expected_world_rpy = sdf_link_rpy_from_optical_error(math.pi / 2.0, 0.0, 0.0, math.radians(7.0))
    expected = rpy_to_rotation_world_from_link(*expected_world_rpy)
    _assert_matrix_close(_global_optical_rotation(model), expected)


def test_independent_px4_attitude_does_not_rotate_optical_sensor(tmp_path: Path, monkeypatch) -> None:
    """Configured PX4 attitude rotates base_link independently, while the
    optical_link compensation keeps the camera's true optical pose fixed."""

    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PACKAGE_ROOT))
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    generated = module._generate_world(_context(module, cam_l_px4_attitude_roll_deg=5.0))
    model = _camera_model(ET.parse(generated).getroot(), "left_stereo_camera")
    assert _pose_rpy(model) != pytest.approx((0.0, 0.0, math.pi / 2.0), abs=1.0e-9)
    nominal = rpy_to_rotation_world_from_link(0.0, 0.0, math.pi / 2.0)
    _assert_matrix_close(_global_optical_rotation(model), nominal)


def test_operator_launches_isolated_target_left_and_right_px4_instances() -> None:
    source = (ROOT / "run_operator_view.sh").read_text(encoding="utf-8")
    assert '"$px4_binary" -d -i "$instance"' in source
    assert 'start_px4_instance 0 "$target_model" ""' in source
    assert 'start_px4_instance "$left_instance" "$left_model" "$left_namespace"' in source
    assert 'start_px4_instance "$right_instance" "$right_model" "$right_namespace"' in source
    assert "PX4_SIM_MODEL" not in source
