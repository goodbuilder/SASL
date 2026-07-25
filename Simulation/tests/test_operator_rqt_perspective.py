from __future__ import annotations

import json
from pathlib import Path
from xml.etree import ElementTree


ROOT_DIR = Path(__file__).resolve().parents[1]
PERSPECTIVE_PATH = ROOT_DIR / "config" / "operator_rqt.perspective"
LAUNCHER_PATH = ROOT_DIR / "run_operator_view.sh"
RQT_PATCH_PATH = ROOT_DIR / "python_patches" / "sitecustomize.py"
RECORDER_PANEL_PATH = ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "recorder_panel.py"
OPERATOR_PANEL_PATH = ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "operator_panel.py"
PLUGIN_XML_PATH = ROOT_DIR / "stereo_ground_sim" / "plugin.xml"
PREFLIGHT_PERSPECTIVE_PATH = ROOT_DIR / "config" / "operator_rqt_preflight.perspective"


def test_default_perspective_loads_the_single_tabbed_operator_panel() -> None:
    perspective = json.loads(PERSPECTIVE_PATH.read_text(encoding="utf-8"))
    running = perspective["groups"]["pluginmanager"]["keys"]["running-plugins"]["repr"]

    assert running == "{'stereo_ground_sim/OperatorPanel': [1]}"
    assert "plugin__stereo_ground_sim__OperatorPanel__1" in perspective["groups"]


def test_operator_plugin_id_matches_the_perspective_identifier() -> None:
    root = ElementTree.parse(PLUGIN_XML_PATH).getroot()
    classes = {entry.attrib["name"]: entry.attrib["type"] for entry in root.findall("class")}

    assert classes["OperatorPanel"] == "stereo_ground_sim.operator_panel.OperatorPanel"


def test_launcher_uses_the_tabbed_perspective_without_a_second_recorder_plugin() -> None:
    source = LAUNCHER_PATH.read_text(encoding="utf-8")

    assert '--perspective-file "$perspective_file"' in source
    assert "--standalone stereo_ground_sim.operator_panel.OperatorPanel" not in source
    assert "--command-start-plugin stereo_ground_sim/RecorderPanel" not in source
    assert "Setup, Flight, Analysis, and Debug tabs" in source


def test_launcher_cleanup_matches_rqt_with_force_discover_between_its_flags() -> None:
    source = LAUNCHER_PATH.read_text(encoding="utf-8")
    cleanup_pattern = (
        'rqt_gui .*--clear-config .*--perspective-file .*operator_rqt.*\\\\.perspective'
    )

    assert source.count(cleanup_pattern) == 3
    assert 'rqt_gui --clear-config --perspective-file $ROOT_DIR/config/operator_rqt' not in source


def test_operator_panel_separates_setup_flight_analysis_and_debug() -> None:
    source = OPERATOR_PANEL_PATH.read_text(encoding="utf-8")

    assert "QTabWidget" in source
    assert "QTimer.singleShot(250" in source
    for tab_name in ("Setup", "Flight", "Analysis", "Record", "Debug"):
        assert f'"{tab_name}"' in source
    assert "RecordingControls" in source
    assert "/stereo/raw_gps_baseline_position" in source
    assert "/stereo/global_position_baseline_position" not in source
    assert "/stereo/ideal_position" in source
    assert '"raw": "Nominal stereo"' in source
    assert '"optical": "Optical stereo"' in source
    assert '"ideal": "Ideal reference"' in source
    assert "bbox x1,y1,x2,y2" in source
    assert '("Range", 3)' in source
    assert "FRDR comparison" in source
    assert "legend_x = right + 12" in source
    assert "_ASPECT_HEIGHT_PER_WIDTH = 9.0 / 16.0" in source
    assert "QSizePolicy.Ignored, QSizePolicy.MinimumExpanding" in source
    assert "layout.addLayout(cameras)" in source
    assert "layout.addStretch(1)" in source
    assert "StereoCAM run parameters (/stereo_distance_node, read-only)" in source
    assert "ListParameters" in source
    assert "Refresh run parameters" in source
    assert "Live algorithm controls" not in source
    assert "Apply live controls" not in source
    assert "set_parameters" not in source
    assert "QComboBox" not in source
    assert "QDoubleSpinBox" not in source
    assert "QLineEdit" not in source
    assert 'QPushButton("Open ROS graph")' in source
    assert '["ros2", "run", "rqt_graph", "rqt_graph"]' in source
    assert "Live saved CSV preview" in source
    assert "read_recent_csv_lines(csv_path, data_row_count=3)" in source
    assert 'QPushButton("Open session")' in source
    assert 'QPushButton("Open CSV")' in source
    assert 'QPushButton("Open XLSX")' in source
    assert 'QPushButton("Export analysis CSV")' in source
    assert "read_recent_csv_rows" in source
    for record_view in ("Summary", "Raw GPS", "BBox", "PX4 / Camera", "Raw CSV"):
        assert f'"{record_view}"' in source


def test_rqt_runtime_patch_keeps_only_raw_gps_baseline_in_axis_plots() -> None:
    source = RQT_PATCH_PATH.read_text(encoding="utf-8")

    for index in range(3):
        assert f"/stereo/ideal_position/data[{index}]" in source
        assert f"/stereo/raw_gps_baseline_plot/data[{index}]" in source
        assert f"/stereo/global_position_baseline_plot/data[{index}]" not in source


def test_flight_toolbar_exposes_csv_mp4_and_debug_recording_actions() -> None:
    source = RECORDER_PANEL_PATH.read_text(encoding="utf-8")

    assert 'QLabel("Flight recording")' in source
    assert '("lite", PRIMARY_RECORD_BUTTON_TEXT)' in source
    assert '("full", "Save MP4")' in source
    assert '("debug", "Save Debug")' in source
    assert 'self._start_recording(record_mode)' in source
    assert 'QPushButton("Start Cal CSV")' not in source


def test_preflight_perspective_uses_standard_readiness_plugins_without_recorder_controls() -> None:
    perspective = json.loads(PREFLIGHT_PERSPECTIVE_PATH.read_text(encoding="utf-8"))
    running = perspective["groups"]["pluginmanager"]["keys"]["running-plugins"]["repr"]
    launcher = LAUNCHER_PATH.read_text(encoding="utf-8")

    assert "rqt_image_view/ImageView" in running
    assert "rqt_topic/Topic" in running
    assert "--preflight-view" in launcher
    assert "operator_rqt_preflight.perspective" in launcher
