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
    assert settings.yolo_weights_path == "yolov8m.pt"
    assert settings.interface_name == "enx588694fce07e"
    assert settings.left_rtsp_url == "rtsp://10.0.0.231:8554/visible"
    assert settings.right_rtsp_url == "rtsp://10.0.0.232:8554/visible"
    assert settings.left_control_host == "10.0.0.231"
    assert settings.right_control_host == "10.0.0.232"
    assert settings.yolo_input_size == 416
    assert settings.preview_width == 1920
    assert settings.yolo_event_queue_size == 128
    assert settings.yolo_frame_queue_size == 2
    assert settings.record_video_queue_size == 300
    assert settings.ffmpeg_stimeout_us == 2_000_000
    assert settings.ffmpeg_max_delay_us == 500_000
    assert settings.rtsp_decoder == "nvidia"
    assert settings.stream_backend == "gstreamer"
    assert settings.rtsp_transport == "udp"
    assert settings.tracking_process_accel_std_mps2 == 4.0
    assert settings.tracking_max_normalized_innovation == 12.0
    assert settings.baseline_bad_std_m == 0.10
    assert settings.baseline_jump_m == 0.20
    assert settings.live_attitude_max_age_s == 0.25
    assert settings.live_attitude_max_pair_delta_s == 0.10
    assert settings.yolo_gate_class_enabled is False
    assert settings.yolo_gate_min_bbox_enabled is False
    assert settings.yolo_gate_matching_enabled is False
    assert settings.yolo_gate_time_delta_enabled is False
    assert settings.yolo_gate_vertical_delta_enabled is False
    assert settings.yolo_gate_time_basis == "camera_rtc_required"
    assert settings.ros_domain_id == 0
    assert settings.pixhawk_host_address == "10.0.0.10"
    assert settings.left_pixhawk_eth_ip == "10.0.0.2"
    assert settings.right_pixhawk_eth_ip == "10.0.0.3"
    assert settings.left_mavlink_url == "udpin:0.0.0.0:14550"
    assert settings.right_mavlink_url == "udpin:0.0.0.0:14551"
    assert settings.left_pixhawk_bootstrap_url == ""
    assert settings.right_pixhawk_bootstrap_url == ""
    assert settings.left_pixhawk_bootstrap_baud == 57600
    assert settings.right_pixhawk_bootstrap_baud == 57600
    assert settings.gnss_base_serial_port == "/dev/ttyACM0"
    assert settings.gnss_base_baud == 460800
    assert settings.gnss_base_survey_in_min_duration_s == 300
    assert settings.gnss_base_survey_in_accuracy_m == 1.0
    assert settings.gnss_base_rtcm_injection_target == "both"
    assert settings.uam_telemetry_source == "mavlink"
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
        rtsp_latency_ms=250,
        yolo_event_queue_size=256,
        yolo_frame_queue_size=4,
        record_video_queue_size=512,
        ffmpeg_stimeout_us=3_000_000,
        ffmpeg_max_delay_us=750_000,
        rtsp_decoder="software",
        stream_backend="opencv",
        rtsp_transport="tcp",
        tracking_process_accel_std_mps2=5.0,
        tracking_initial_position_std_m=30.0,
        tracking_initial_velocity_std_mps=15.0,
        tracking_reset_after_s=4.0,
        tracking_max_normalized_innovation=10.0,
        tracking_min_lateral_std_m=0.2,
        tracking_min_forward_std_m=1.5,
        tracking_max_lateral_std_m=40.0,
        tracking_max_forward_std_m=200.0,
        baseline_bad_std_m=0.15,
        baseline_jump_m=0.25,
        baseline_max_timestamp_delta_ms=250.0,
        baseline_good_rtk_ratio=0.90,
        baseline_good_std_m=0.03,
        baseline_good_h_acc_m=0.04,
        baseline_good_v_acc_m=0.06,
        baseline_good_timestamp_delta_ms=60.0,
        baseline_warning_rtk_ratio=0.65,
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
        left_pixhawk_bootstrap_url="/dev/serial/by-id/usb-PX4-left",
        right_pixhawk_bootstrap_url="/dev/serial/by-id/usb-PX4-right",
        left_pixhawk_bootstrap_baud=921600,
        right_pixhawk_bootstrap_baud=115200,
        mavlink_configure_streams=False,
        mavlink_stream_rate=1_000_000,
        mavlink_configure_uxrce_agent=False,
        uxrce_port=9999,
        auto_load_intrinsics=False,
        manual_focal_override=True,
        use_triangulation=False,
        triangulation_extrinsics_source="checkerboard_stereo_calibration",
        live_attitude_max_age_s=0.75,
        live_attitude_max_pair_delta_s=0.20,
        use_rtk_baseline=False,
        require_rtk_fixed=False,
        gnss_base_serial_port="/dev/ttyUSB9",
        gnss_base_baud=115200,
        gnss_base_survey_in_min_duration_s=600,
        gnss_base_survey_in_accuracy_m=0.25,
        gnss_base_rtcm_injection_target="left",
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

    assert settings.live_attitude_max_age_s == 0.25
    assert settings.live_attitude_max_pair_delta_s == 0.10
    assert not hasattr(settings, "measurement_display_mode")
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
