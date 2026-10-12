"""Compose feature views using named shell layout dependencies."""

def build_panel_views(application):
    shell = application.features['presentation'].build_shell()
    application.features['camera'].build_connection_controls(shell.camera_form)
    application.features['camera'].build_connection_layout(
        shell.camera_form,
        shell.camera_group,
        shell.camera_layout,
        shell.camera_layout_outer,
        shell.camera_network_group,
        shell.pixhawk_connection_layout,
        shell.pixhawk_form,
        shell.pixhawk_group,
        shell.yolo_form,
        shell.yolo_group,
        shell.yolo_layout_outer,
    )
    application.features['geometry'].build_geometry_settings(shell.stereo_layout)
    application.features['diagnostics'].build_sensor_tabs(
        shell.advanced_tabs,
        shell.alignment_attitude_layout,
        shell.alignment_mag_layout,
        shell.alignment_overview_layout,
        shell.alignment_tabs,
        shell.baseline_field_browser_layout,
        shell.baseline_field_browser_tab,
        shell.connection_tabs,
        shell.gnss_base_layout,
        shell.mag_management_layout,
        shell.optical_alignment_layout,
        shell.px4_mag_advanced_layout,
        shell.px4_mag_advanced_tab,
        shell.rtk_layout,
        shell.setup_layout,
        shell.wiris_command_panel,
    )
    (zoom_group,) = application.features['camera'].build_camera_commands(shell.wiris_command_layout)
    (calibration_group, calibration_preview_row) = application.features['calibration'].build_calibration_capture()
    application.features['calibration'].build_calibration_validation(
        calibration_group,
        calibration_preview_row,
        shell.checkerboard_calibration_layout,
        shell.checkerboard_validation_layout,
        zoom_group,
    )
    application.features['calibration'].build_calibration_profiles(shell.zoom_intrinsics_layout)
    application.features['geometry'].build_parameter_views(shell.extrinsic_parameters_layout, shell.intrinsic_parameters_layout)
    application.features['camera'].build_live_camera(shell.runtime_layout)
    application.features['geometry'].build_measurement_quality(shell.triangulation_quality_layout)
    application.features['recording_view'].build_recording_preview(shell.save_layout, shell.time_layout)
    application.features['presentation'].build_presentation_finish()
    application.features['lifecycle'].build_lifecycle()
