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
    assert "/stereo/gps_baseline_position" in source
    assert "/stereo/uam_raw_gps_reference_position" in source
    assert "/stereo/ideal_position" in source
    assert '"nominal_pose": "Nominal Pose"' in source
    assert '"optical_pose": "Optical Pose"' in source
    assert '"ideal": "Ideal"' in source
    assert '"uam_raw_gps_reference": "UAM GPS"' in source
    assert "bbox x1,y1,x2,y2" in source
    assert '("Range", 3)' in source
    assert "FRDR comparison" not in source
    assert "legend_x = right + 12" in source
    assert "legend_x + 35" in source
    assert "legend_y += 18" in source
    assert "legend_label_width" in source
    assert "set_series_visible" in source
    assert "QCheckBox" in source
    assert "self._facts = None" in source
    assert "layout.addWidget(self._facts)" not in source
    assert "group.setCheckable(True)" not in source
    assert "group.toggled.connect(content.setVisible)" not in source
    assert "_ASPECT_HEIGHT_PER_WIDTH = 9.0 / 16.0" in source
    assert "QSizePolicy.Ignored, QSizePolicy.MinimumExpanding" in source
    assert "layout.addLayout(cameras)" in source
    recorder_source = RECORDER_PANEL_PATH.read_text(encoding="utf-8")
    assert "self._recording_controls.respawn_button" in source
    assert 'self.respawn_button = QPushButton("Respawn Drone")' in recorder_source
    assert "controls.addWidget(self.respawn_button)" in recorder_source
    assert '"/stereo/respawn_drone"' in source
    assert "Trigger.Request()" in source
    assert "respawn_finished = Signal()" in source
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
    assert "CSV preview" in source
    assert "update_live_sample" in source
    assert "update_live_px4_raw_gps" in source
    assert "_show_live_preview()" in source
    assert "No CSV session has been started." not in source
    assert "persisted telemetry rows here" not in source
    assert "CSV_PREVIEW_TABLE_ROWS = 40" in source
    assert "CSV_PREVIEW_RAW_ROWS = 20" in source
    assert 'RQT_IMAGE_RENDER_HZ = _env_float("RQT_IMAGE_RENDER_HZ", 5.0, 0.5)' in source
    assert 'RQT_FACTS_RENDER_HZ = _env_float("RQT_FACTS_RENDER_HZ", 5.0, 0.5)' in source
    assert 'RQT_PREVIEW_RENDER_HZ = _env_float("RQT_PREVIEW_RENDER_HZ", 2.0, 0.5)' in source
    assert "_emit_image_sample" in source
    assert "_flush_live_preview" in source
    assert "_flush_repaint" in source
    assert "_refresh_if_pending" in source
    assert "read_recent_csv_lines(csv_path, data_row_count=CSV_PREVIEW_RAW_ROWS)" in source
    assert 'QPushButton("Open session")' in source
    assert 'QPushButton("Open CSV")' in source
    assert 'QPushButton("Open XLSX")' in source
    assert 'QPushButton("Export analysis CSV")' in source
    assert "read_recent_csv_rows(csv_path, data_row_count=CSV_PREVIEW_TABLE_ROWS)" in source
    assert '("gps_baseline_range_m", "GPS Baseline range (m)")' in source
    assert '("gps_baseline_forward_m", "GPS Baseline F (m)")' in source
    assert '("nominal_pose_ray_midpoint_forward_m", "F Nominal Pose")' in source
    assert '("optical_pose_ray_midpoint_right_m", "Rt Optical Pose")' in source
    assert '("ideal_ray_range_m", "Rg Ideal")' in source
    assert 'ray_metrics = Signal(str)' in source
    assert '"/stereo/ray_metrics"' in source
    assert "update_live_ray_metrics" in source
    assert 'from px4_msgs.msg import SensorGps, VehicleAttitude, VehicleGlobalPosition, VehicleOdometry' in source
    assert 'QGroupBox("Published VehicleAttitude (actual)")' in source
    assert 'PX4 timestamp_sample [us]' in source
    assert 'QGroupBox("MAVLink ATTITUDE (raw)")' in source
    assert 'recv_match(' in source
    assert 'type="ATTITUDE"' in source
    assert 'RQT_MAVLINK_ATTITUDE_URL' in source
    assert '"udpin:0.0.0.0:14551"' in source
    assert 'MAVLINK_ATTITUDE_SIDE_BY_SYSID = {2: "left", 3: "right"}' in source
    assert 'time_boot_ms' in source
    assert 'Δyaw ROS2-MAVLink [deg]' in source
    assert 'self._bridge.mavlink_attitude.connect' in source
    assert 'MAVLink ATTITUDE: listening on' not in source
    assert 'value_label.setWordWrap(False)' in source
    assert 'actual_grid.setRowMinimumHeight(row, 24)' in source
    assert 'self._bridge.px4_attitude.connect(self._ray_diagnostics.update_actual_attitude)' in source
    assert 'VehicleAttitude,' in source
    assert 'PX4_ATTITUDE_TOPICS.items()' in source
    assert '("midpoint_frd_m", "midpoint")' in source
    for record_view in ("Summary", "GPS", "BBox", "Ray summary", "Left PX4", "Right PX4", "UAM PX4", "Raw CSV"):
        assert f'"{record_view}"' in source


def test_respawn_service_contract_is_declared_by_distance_node() -> None:
    source = (ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "stereo_distance_node.py").read_text(
        encoding="utf-8"
    )

    assert 'self.create_service(\n            Trigger, "/stereo/respawn_drone"' in source
    runtime_control = (ROOT_DIR / "stereo_ground_sim" / "stereo_ground_sim" / "runtime_control_coordinator.py").read_text(encoding="utf-8")
    assert "self.node._set_gazebo_target_pose(" in runtime_control
    for parameter in ("target_spawn_x_m", "target_spawn_y_m", "target_spawn_z_m", "target_spawn_yaw_deg"):
        assert f'"{parameter}"' in source


def test_rqt_runtime_patch_keeps_gps_position_series_in_axis_plots() -> None:
    source = RQT_PATCH_PATH.read_text(encoding="utf-8")

    for index in range(3):
        assert f"/stereo/ideal_position/data[{index}]" in source
        assert f"/stereo/gps_baseline_plot/data[{index}]" in source
        assert f"/stereo/uam_raw_gps_reference_plot/data[{index}]" in source


def test_flight_toolbar_exposes_csv_mp4_and_debug_recording_actions() -> None:
    source = RECORDER_PANEL_PATH.read_text(encoding="utf-8")

    assert 'QLabel("Flight recording")' in source
    assert '("lite", PRIMARY_RECORD_BUTTON_TEXT)' in source
    assert '("full", "Save MP4")' in source
    assert '("debug", "Save Debug")' in source
    assert 'QPushButton("Mission")' in source
    assert 'QPushButton("Waypoint")' in source
    assert 'QPushButton("Auto")' in source
    assert '"run_mission.sh"' in source
    assert '"run_auto.sh"' in source
    assert '"run_px4_waypoint_mission.sh"' in source
    assert "build_mission_command(kind" in source
    assert 'self._start_recording(record_mode)' in source
    assert "self._start_mission(mission_kind)" in source
    assert 'QPushButton("Start Cal CSV")' not in source


def test_preflight_perspective_uses_standard_readiness_plugins_without_recorder_controls() -> None:
    perspective = json.loads(PREFLIGHT_PERSPECTIVE_PATH.read_text(encoding="utf-8"))
    running = perspective["groups"]["pluginmanager"]["keys"]["running-plugins"]["repr"]
    launcher = LAUNCHER_PATH.read_text(encoding="utf-8")

    assert "rqt_image_view/ImageView" in running
    assert "rqt_topic/Topic" in running
    assert "--preflight-view" in launcher
    assert "operator_rqt_preflight.perspective" in launcher
