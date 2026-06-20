from pathlib import Path

from wiris_stereo_distance.settings import (
    DEFAULT_CLOCK_DOWNLOAD_DIR,
    DEFAULT_INTRINSICS_PATH,
    AppSettings,
    load_settings,
    resolve_settings_path,
    save_settings,
)


def test_load_settings_defaults_when_missing(tmp_path: Path) -> None:
    path = tmp_path / "missing.json"
    settings = load_settings(path)

    assert settings.auto_load_intrinsics is True
    assert settings.manual_focal_override is False
    assert settings.use_triangulation is True
    assert settings.intrinsics_path == DEFAULT_INTRINSICS_PATH
    assert settings.yolo_weights_path == "/home/moontaehyung/Capstone/models/Capstone.pt"
    assert settings.interface_name == "enx588694fce07e"
    assert settings.left_rtsp_url == "rtsp://10.0.0.231:8554/visible"
    assert settings.right_rtsp_url == "rtsp://10.0.0.232:8554/visible"
    assert settings.left_control_host == "10.0.0.231"
    assert settings.right_control_host == "10.0.0.232"
    assert settings.yolo_input_size == 416
    assert settings.preview_width == 1920
    assert settings.preview_rate_hz == 15.0
    assert settings.rtsp_decoder == "nvidia"
    assert settings.stream_backend == "gstreamer"
    assert settings.rtsp_transport == "udp"
    assert settings.measurement_display_mode == "detection"
    assert settings.ros_domain_id == 0
    assert settings.pixhawk_host_address == "10.0.0.10"
    assert settings.left_pixhawk_eth_ip == "10.0.0.2"
    assert settings.right_pixhawk_eth_ip == "10.0.0.3"
    assert settings.left_mavlink_url == "udpin:0.0.0.0:14550"
    assert settings.right_mavlink_url == "udpin:0.0.0.0:14551"
    assert settings.uam_telemetry_source == "ros2"
    assert settings.uam_ros2_topic == "/uav_0/fmu/out/vehicle_local_position"
    assert settings.uam_ros2_x_field == "x"
    assert settings.uam_ros2_y_field == "y"
    assert settings.uam_ros2_z_field == "z"
    assert settings.uam_mavlink_side == "left"
    assert settings.uam_mavlink_message == "LOCAL_POSITION_NED"
    assert settings.uam_mavlink_x_field == "x"
    assert settings.uam_mavlink_y_field == "y"
    assert settings.uam_mavlink_z_field == "z"
    assert settings.uam_compare_max_dt_s == 0.25
    assert settings.uam_compare_time_offset_s == 0.0
    assert settings.clock_ftp_user == "anonymous"
    assert settings.clock_ftp_port == 21
    assert settings.clock_ftp_remote_dir == "/"
    assert settings.clock_download_dir == DEFAULT_CLOCK_DOWNLOAD_DIR
    assert settings.clock_capture_wait_s == 10.0
    assert path.exists()


def test_save_and_load_settings_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "settings.json"
    settings = AppSettings(
        intrinsics_path="custom/intrinsics.json",
        yolo_weights_path="yolov8n.pt",
        interface_name="enx-test",
        camera_host_address="10.0.0.10/24",
        left_rtsp_url="rtsp://10.0.0.10:8554/visible",
        right_rtsp_url="rtsp://10.0.0.11:8554/visible",
        left_control_host="10.0.0.10",
        right_control_host="10.0.0.11",
        left_stip_ip="10.0.0.110",
        right_stip_ip="10.0.0.111",
        stip_subnet_mask="255.255.255.0",
        stip_reboot=False,
        wiris_control_port=2241,
        wiris_control_timeout_s=1.5,
        yolo_device="cuda:0",
        yolo_input_size=800,
        yolo_conf_threshold=0.4,
        yolo_iou_threshold=0.6,
        preview_width=1280,
        preview_rate_hz=30.0,
        rtsp_latency_ms=250,
        rtsp_decoder="software",
        stream_backend="opencv",
        rtsp_transport="tcp",
        ros_domain_id=7,
        left_namespace="uav_left",
        right_namespace="uav_right",
        pixhawk_host_address="192.168.0.254",
        left_pixhawk_eth_ip="192.168.0.2",
        right_pixhawk_eth_ip="192.168.0.3",
        left_mavlink_url="/dev/ttyACM0",
        right_mavlink_url="/dev/ttyACM1",
        left_mavlink_baud=115200,
        right_mavlink_baud=57600,
        mavlink_configure_streams=False,
        mavlink_stream_rate=1_000_000,
        mavlink_configure_uxrce_agent=False,
        uxrce_port=9999,
        auto_load_intrinsics=False,
        manual_focal_override=True,
        use_triangulation=False,
        measurement_display_mode="preview",
        use_rtk_baseline=False,
        require_rtk_fixed=False,
        uam_telemetry_source="mavlink",
        uam_ros2_topic="/vehicle/local",
        uam_ros2_x_field="position[0]",
        uam_ros2_y_field="position[1]",
        uam_ros2_z_field="position[2]",
        uam_mavlink_side="right",
        uam_mavlink_message="LOCAL_POSITION_NED",
        uam_mavlink_x_field="x",
        uam_mavlink_y_field="y",
        uam_mavlink_z_field="z",
        uam_compare_max_dt_s=0.5,
        uam_compare_time_offset_s=-0.012345,
        clock_ftp_user="operator",
        clock_ftp_password="secret",
        clock_ftp_port=2121,
        clock_ftp_remote_dir="/media",
        clock_download_dir="clock_test",
        clock_capture_wait_s=5.5,
    )

    save_settings(settings, path)
    loaded = load_settings(path)

    assert loaded == settings
    assert "legacy_removed_mount_key" not in path.read_text()


def test_load_settings_ignores_unknown_legacy_keys(tmp_path: Path) -> None:
    path = tmp_path / "legacy-settings.json"
    path.write_text(
        """
        {
          "legacy_removed_mount_key": 1.0,
          "legacy_removed_runtime_key": -2.0,
          "measurement_display_mode": "preview"
        }
        """
    )

    settings = load_settings(path)

    assert settings.measurement_display_mode == "preview"
    assert not hasattr(settings, "legacy_removed_mount_key")
    assert not hasattr(settings, "legacy_removed_runtime_key")


def test_load_settings_accepts_legacy_mavlink_start_ros2_key(tmp_path: Path) -> None:
    path = tmp_path / "legacy-uxrce-settings.json"
    path.write_text('{"mavlink_start_ros2": false}')

    settings = load_settings(path)

    assert settings.mavlink_configure_uxrce_agent is False


def test_load_settings_normalizes_removed_calibration_extrinsics_source(tmp_path: Path) -> None:
    path = tmp_path / "settings.json"
    path.write_text('{"triangulation_extrinsics_source": "calibration"}')

    settings = load_settings(path)

    assert settings.triangulation_extrinsics_source == "pixhawk"


def test_resolve_settings_path_uses_base_for_relative_paths(tmp_path: Path) -> None:
    resolved = resolve_settings_path("calibration/intrinsics.json", base_dir=tmp_path)

    assert resolved == tmp_path / "calibration/intrinsics.json"


def test_save_settings_creates_parent_directory(tmp_path: Path) -> None:
    path = tmp_path / "nested" / "settings.json"

    save_settings(AppSettings(), path)

    assert path.exists()
