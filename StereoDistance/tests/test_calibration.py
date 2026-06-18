import json
from pathlib import Path

from wiris_stereo_distance.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoIntrinsicsResult,
    load_intrinsics_json,
    opencv_available,
    save_intrinsics_json,
)


def test_checkerboard_config_pattern_and_square_size() -> None:
    config = CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03)

    assert config.pattern_size == (9, 6)
    assert config.square_size_m == 0.03


def test_save_intrinsics_json(tmp_path: Path) -> None:
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.1, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.4,
        image_count=20,
    )
    result = StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
    )
    output = tmp_path / "intrinsics.json"

    save_intrinsics_json(output, result, metadata={"test": True})

    text = output.read_text()
    assert '"fx"' not in text  # Matrix form is stored for direct OpenCV use.
    assert '"camera_matrix"' in text
    assert '"reprojection_error_px": 0.4' in text
    assert '"extrinsics"' not in text


def test_load_intrinsics_json_ignores_legacy_extrinsics(tmp_path: Path) -> None:
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.4,
        image_count=20,
    )
    output = tmp_path / "intrinsics.json"
    result = StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
    )

    save_intrinsics_json(output, result)
    payload = json.loads(output.read_text())
    payload["extrinsics"] = {
        "rotation_left_to_right": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "rotation_right_to_left": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "translation_left_to_right_m": [-0.3, 0.0, 0.0],
        "translation_right_in_left_m": [0.3, 0.0, 0.0],
        "reprojection_error_px": 0.22,
        "pair_count": 18,
    }
    output.write_text(json.dumps(payload))

    loaded = load_intrinsics_json(output)

    assert loaded.left.camera_matrix == intrinsics.camera_matrix
    assert loaded.right.image_count == 20
    assert not hasattr(loaded, "extrinsics")


def test_opencv_available_returns_status_tuple() -> None:
    available, error = opencv_available()

    assert isinstance(available, bool)
    assert error is None or isinstance(error, str)
