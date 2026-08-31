from __future__ import annotations

import importlib.util
import math
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HELPER_PATH = ROOT / "scripts" / "prepare_px4_gz_magnetometer_frame.py"
FIELD_VERIFIER_PATH = (
    ROOT / "stereo_ground_sim" / "tools" / "verify_magnetic_field.py"
)


def _load_helper():
    spec = importlib.util.spec_from_file_location("prepare_px4_gz_magnetometer_frame", HELPER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


HELPER = _load_helper()


def _load_field_verifier():
    spec = importlib.util.spec_from_file_location("verify_magnetic_field", FIELD_VERIFIER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


FIELD_VERIFIER = _load_field_verifier()


def _bridge_source(y_expression: str) -> str:
    return f"""
void GZBridge::magnetometerCallback(const gz::msgs::Magnetometer &msg)
{{
\t// FIMEX: once we're on jetty or later
\t// The magnetometer plugin publishes in units of gauss and in a weird left handed coordinate system
\t// https://github.com/gazebosim/gz-sim/pull/2460
\treport.x = -msg.field_tesla().y();
\treport.y = {y_expression};
\treport.z = msg.field_tesla().z();

\t_sensor_mag_pub.publish(report);
}}
"""


def _corrected_source() -> str:
    return """
void GZBridge::magnetometerCallback(const gz::msgs::Magnetometer &msg)
{
\t// gz::msgs::Magnetometer is sensor-local FLU. Convert to body FRD,
\t// matching imuCallback so arbitrary roll/pitch/yaw remain valid.
\treport.x = msg.field_tesla().x();
\treport.y = -msg.field_tesla().y();
\treport.z = -msg.field_tesla().z();

\t_sensor_mag_pub.publish(report);
}
"""


@pytest.mark.parametrize(
    ("source", "variant"),
    (
        (_bridge_source("-msg.field_tesla().x()"), "stock_v1_17"),
        (_bridge_source("msg.field_tesla().x()"), "legacy_y_sign_workaround"),
    ),
)
def test_known_predecessor_bridge_mappings_prepare_to_flu_frd(source: str, variant: str) -> None:
    updated, before, changed = HELPER.prepare_bridge_source(source)

    assert before == variant
    assert changed is True
    assert HELPER.bridge_variant(updated)[0] == "sensor_flu_to_body_frd"
    assert "report.x = msg.field_tesla().x();" in updated
    assert "report.y = -msg.field_tesla().y();" in updated
    assert "report.z = -msg.field_tesla().z();" in updated
    assert "sensor-local FLU" in updated


def test_correct_bridge_mapping_is_idempotent() -> None:
    source = _corrected_source()

    updated, before, changed = HELPER.prepare_bridge_source(source)

    assert updated == source
    assert before == "sensor_flu_to_body_frd"
    assert changed is False


def test_unknown_bridge_mapping_fails_closed() -> None:
    source = _bridge_source("msg.field_tesla().z()")

    with pytest.raises(HELPER.PreparationError, match="not a known"):
        HELPER.prepare_bridge_source(source)


def _matmul(left, right):
    columns = tuple(zip(*right))
    return tuple(
        tuple(sum(left[row][item] * columns[column][item] for item in range(3)) for column in range(3))
        for row in range(3)
    )


def _transpose(matrix):
    return tuple(tuple(matrix[row][column] for row in range(3)) for column in range(3))


def _matvec(matrix, vector):
    return tuple(sum(row[index] * vector[index] for index in range(3)) for row in matrix)


def _rpy(roll_deg: float, pitch_deg: float, yaw_deg: float):
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    rx = (
        (1.0, 0.0, 0.0),
        (0.0, math.cos(roll), -math.sin(roll)),
        (0.0, math.sin(roll), math.cos(roll)),
    )
    ry = (
        (math.cos(pitch), 0.0, math.sin(pitch)),
        (0.0, 1.0, 0.0),
        (-math.sin(pitch), 0.0, math.cos(pitch)),
    )
    rz = (
        (math.cos(yaw), -math.sin(yaw), 0.0),
        (math.sin(yaw), math.cos(yaw), 0.0),
        (0.0, 0.0, 1.0),
    )
    return _matmul(_matmul(rz, ry), rx)


@pytest.mark.parametrize(
    "world_enu_rpy_deg",
    (
        (0.0, 0.0, 90.0),
        (0.0, -20.0, 90.0),
        (0.0, 20.0, 90.0),
        (15.0, 0.0, 90.0),
        (10.0, -20.0, 75.0),
    ),
)
def test_enu_world_to_sensor_flu_to_body_frd_matches_independent_ned_oracle(
    world_enu_rpy_deg,
) -> None:
    # Distinct sentinels expose every adjacent-field swap and sign inversion.
    field_ned = (0.21, 0.03, 0.44)
    enu_from_ned = (
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, -1.0),
    )
    flu_from_frd = (
        (1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, -1.0),
    )
    rotation_enu_from_flu = _rpy(*world_enu_rpy_deg)
    field_enu = _matvec(enu_from_ned, field_ned)
    field_flu = _matvec(_transpose(rotation_enu_from_flu), field_enu)
    actual_frd = _matvec(flu_from_frd, field_flu)

    rotation_ned_from_frd = _matmul(
        _matmul(enu_from_ned, rotation_enu_from_flu),
        flu_from_frd,
    )
    expected_frd = _matvec(_transpose(rotation_ned_from_frd), field_ned)

    assert actual_frd == pytest.approx(expected_frd, abs=1.0e-12)


def test_legacy_level_only_mapping_fails_for_twenty_degree_pitch() -> None:
    field_ned = (0.2174328663, 0.0129398487, 0.4304112612)
    enu_from_ned = (
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, -1.0),
    )
    flu_from_frd = (
        (1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, -1.0),
    )
    rotation_enu_from_flu = _rpy(0.0, -20.0, 90.0)
    field_enu = _matvec(enu_from_ned, field_ned)
    field_flu = _matvec(_transpose(rotation_enu_from_flu), field_enu)
    expected_frd = _matvec(flu_from_frd, field_flu)

    # Previous PX4 workaround: x=-gz.y, y=+gz.x, z=+gz.z.
    legacy_frd = (-field_flu[1], field_flu[0], field_flu[2])

    assert max(abs(left - right) for left, right in zip(legacy_frd, expected_frd)) > 0.4


def test_world_field_verifier_accepts_explicit_enu_plugin_contract() -> None:
    world = ROOT / "stereo_ground_sim/worlds/stereo_ground_drone.sdf"

    latitude, longitude, field = FIELD_VERIFIER.parse_world(world)

    assert latitude == pytest.approx(47.397971057728974)
    assert longitude == pytest.approx(8.546163739800146)
    assert field == pytest.approx(
        (1.293984869535e-06, 2.174328662843e-05, -4.304112611929e-05)
    )


def test_world_field_verifier_rejects_ned_plugin_in_enu_world(tmp_path: Path) -> None:
    source = (ROOT / "stereo_ground_sim/worlds/stereo_ground_drone.sdf").read_text(
        encoding="utf-8"
    )
    broken = tmp_path / "broken-world.sdf"
    broken.write_text(
        source.replace(
            "<use_earth_frame_ned>false</use_earth_frame_ned>",
            "<use_earth_frame_ned>true</use_earth_frame_ned>",
            1,
        ),
        encoding="utf-8",
    )

    with pytest.raises(SystemExit, match="must explicitly emit the earth field in ENU"):
        FIELD_VERIFIER.parse_world(broken)


def _init_checkout(tmp_path: Path, bridge_source: str) -> tuple[Path, Path, Path]:
    px4_root = tmp_path / "PX4-Autopilot"
    bridge = px4_root / HELPER.BRIDGE_RELATIVE_PATH
    bridge.parent.mkdir(parents=True)
    bridge.write_text(bridge_source, encoding="utf-8")
    airframe = tmp_path / "4090_camera_px4"
    airframe.write_text("param set-default EKF2_MAG_TYPE 1\n", encoding="utf-8")
    binary = px4_root / "build/px4_sitl_default/bin/px4"
    binary.parent.mkdir(parents=True)
    binary.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    binary.chmod(0o755)
    subprocess.run(["git", "init", "-q", str(px4_root)], check=True)
    subprocess.run(["git", "-C", str(px4_root), "add", "."], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(px4_root),
            "-c",
            "user.name=Simulation Test",
            "-c",
            "user.email=simulation-test@example.invalid",
            "commit",
            "-qm",
            "fixture",
        ],
        check=True,
    )
    subprocess.run(["git", "-C", str(px4_root), "tag", "v1.17.0"], check=True)
    return px4_root, airframe, binary


def test_checkout_prepare_and_build_stamp_are_idempotent_and_hash_bound(tmp_path: Path) -> None:
    px4_root, airframe, binary = _init_checkout(
        tmp_path, _bridge_source("-msg.field_tesla().x()")
    )

    applied = HELPER.prepare_checkout(px4_root, apply=True)
    second = HELPER.prepare_checkout(px4_root, apply=True)
    recorded = HELPER.write_build_stamp(
        px4_root, airframe=airframe, binary=binary
    )
    verified = HELPER.check_build_stamp(
        px4_root, airframe=airframe, binary=binary
    )

    assert applied["status"] == "applied"
    assert applied["before_variant"] == "stock_v1_17"
    assert second["status"] == "already_correct"
    assert recorded["status"] == "recorded"
    assert verified["status"] == "verified"

    airframe.write_text("param set-default EKF2_MAG_TYPE 0\n", encoding="utf-8")
    with pytest.raises(HELPER.PreparationError, match="stamp is stale"):
        HELPER.check_build_stamp(px4_root, airframe=airframe, binary=binary)


def test_checkout_unknown_mapping_is_not_modified(tmp_path: Path) -> None:
    source = _bridge_source("msg.field_tesla().z()")
    px4_root, _airframe, _binary = _init_checkout(tmp_path, source)
    bridge = px4_root / HELPER.BRIDGE_RELATIVE_PATH

    with pytest.raises(HELPER.PreparationError, match="not a known"):
        HELPER.prepare_checkout(px4_root, apply=True)

    assert bridge.read_text(encoding="utf-8") == source


def test_build_stamp_requires_exact_px4_version(tmp_path: Path) -> None:
    px4_root, airframe, binary = _init_checkout(tmp_path, _corrected_source())

    with pytest.raises(HELPER.PreparationError, match="version mismatch"):
        HELPER.write_build_stamp(
            px4_root,
            airframe=airframe,
            binary=binary,
            required_version="v9.9.9",
        )
