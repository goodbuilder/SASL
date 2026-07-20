from types import SimpleNamespace

from wiris_stereo_distance import gui
from wiris_stereo_distance.gui import WirisStereoDistancePanel


def test_measurement_translation_uses_supplied_detection_time_pose_samples(monkeypatch) -> None:
    selected_left = SimpleNamespace(q=(0.1, 0.2, 0.3, 0.4), timestamp_s=10.0)
    selected_right = SimpleNamespace(q=(0.5, 0.6, 0.7, 0.8), timestamp_s=10.1)
    latest_left = SimpleNamespace(q=(1.0, 0.0, 0.0, 0.0), timestamp_s=20.0)
    latest_right = SimpleNamespace(q=(0.0, 1.0, 0.0, 0.0), timestamp_s=20.1)
    captured: dict[str, object] = {}

    def camera_translation(**kwargs):
        captured.update(kwargs)
        return (0.3, 0.0, 0.0)

    monkeypatch.setattr(gui, "camera_translation_right_in_left_camera", camera_translation)

    class Harness:
        _measurement_translation_right_in_left_m = (
            WirisStereoDistancePanel._measurement_translation_right_in_left_m
        )
        _rtk_translation_right_in_left_m_for_extrinsics = (
            WirisStereoDistancePanel._rtk_translation_right_in_left_m_for_extrinsics
        )

        def __init__(self) -> None:
            self.latest_extrinsics = {"left": latest_left, "right": latest_right}
            self.latest_rtk_baseline = SimpleNamespace(north_m=0.25, east_m=-0.02, down_m=0.01)

        @staticmethod
        def _distance_mode() -> str:
            return "pixhawk"

        @staticmethod
        def _gps_baseline_usable_for_translation(_baseline) -> tuple[bool, str]:
            return True, "ok"

    result = Harness()._measurement_translation_right_in_left_m(
        left_extrinsic=selected_left,
        right_extrinsic=selected_right,
    )

    assert result == (0.3, 0.0, 0.0)
    assert captured == {
        "baseline_right_in_left_world_m": (0.25, -0.02, 0.01),
        "left_body_to_world_q": selected_left.q,
        "right_body_to_world_q": selected_right.q,
        "scale_to_m": None,
    }


def test_refresh_distance_wires_the_same_detection_time_samples_to_translation_and_rays(monkeypatch) -> None:
    left = SimpleNamespace(timestamp_s=10.0)
    right = SimpleNamespace(timestamp_s=10.2)
    pair = SimpleNamespace(left=left, right=right)
    selected_left = SimpleNamespace(q=(1.0, 0.0, 0.0, 0.0), timestamp_s=10.05)
    selected_right = SimpleNamespace(q=(0.0, 1.0, 0.0, 0.0), timestamp_s=10.15)
    nearest_midpoints: list[float] = []
    translation_pose_samples: list[tuple[object, object]] = []
    captured_input: dict[str, object] = {}

    class Buffer:
        def __init__(self, sample) -> None:
            self.sample = sample

        def nearest(self, timestamp_s: float):
            nearest_midpoints.append(timestamp_s)
            return SimpleNamespace(sample=self.sample, age_s=abs(self.sample.timestamp_s - timestamp_s))

    def measurement_translation(*, left_extrinsic, right_extrinsic):
        translation_pose_samples.append((left_extrinsic, right_extrinsic))
        return (0.3, 0.0, 0.0)

    def calculate(inputs):
        captured_input["value"] = inputs
        return SimpleNamespace(
            match=None,
            triangulated=None,
            estimate=None,
            warnings=(),
            triangulation_reason="not_reached",
        )

    monkeypatch.setattr(gui, "calculate_measurement_geometry", calculate)
    panel = SimpleNamespace(
        attitude_buffers={"left": Buffer(selected_left), "right": Buffer(selected_right)},
        active_intrinsics=object(),
        latest_rtk_baseline=None,
        _rtsp_urls_are_same_stream=lambda: False,
        _matched_detection_pair=lambda: pair,
        _measurement_display_mode=lambda: "none",
        _clear_measurement_pair_display=lambda: None,
        _active_baseline_m=lambda: (0.3, "test", ()),
        _yolo_pair_max_delta_s=lambda: 0.25,
        _yolo_vertical_max_delta_px=lambda: 80.0,
        _effective_yolo_time_basis=lambda: "detection",
        _triangulation_extrinsics_source=lambda: "pixhawk",
        _measurement_translation_right_in_left_m=measurement_translation,
        _clear_triangulation_quality=lambda _reason: None,
        _set_distance_text=lambda *_args, **_kwargs: None,
        _stereo_geometry_wait_detail=lambda _pair: "waiting",
    )

    WirisStereoDistancePanel._refresh_distance(panel)

    assert nearest_midpoints == [10.1, 10.1]
    assert translation_pose_samples == [(selected_left, selected_right)]
    geometry_input = captured_input["value"]
    assert geometry_input.left_extrinsic is selected_left
    assert geometry_input.right_extrinsic is selected_right
    assert geometry_input.translation_right_in_left_m == (0.3, 0.0, 0.0)


def test_grid_mismatch_has_actionable_gui_detail() -> None:
    detail = WirisStereoDistancePanel._triangulation_wait_detail(
        object(),
        (
            "left detection pixel grid 1920x1080 "
            "does not match left intrinsics grid 1280x720",
        ),
    )

    assert "pixel grid" in detail
    assert "width×height" in detail
    assert "1920x1080" in detail
    assert "1280x720" in detail
