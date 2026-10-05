from pathlib import Path

from wiris_stereo_distance.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
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


def test_load_intrinsics_json_preserves_checkerboard_extrinsics(tmp_path: Path) -> None:
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.4,
        image_count=20,
    )
    output = tmp_path / "intrinsics.json"
    extrinsics = StereoExtrinsics(
        rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 0.999, -0.001], [0.0, 0.001, 0.999]],
        translation_left_to_right_m=[0.3, 0.0, 0.0],
        reprojection_error_px=0.22,
        pair_count=18,
    )
    result = StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
        extrinsics=extrinsics,
    )

    save_intrinsics_json(output, result)

    loaded = load_intrinsics_json(output)

    assert loaded.left.camera_matrix == intrinsics.camera_matrix
    assert loaded.right.image_count == 20
    assert loaded.extrinsics is not None
    assert loaded.extrinsics.translation_left_to_right_m == [0.3, 0.0, 0.0]
    assert loaded.extrinsics.pair_count == 18
    assert "opencv_optical:RDF" in loaded.extrinsics.frame_contract


def test_opencv_available_returns_status_tuple() -> None:
    available, error = opencv_available()

    assert isinstance(available, bool)
    assert error is None or isinstance(error, str)
