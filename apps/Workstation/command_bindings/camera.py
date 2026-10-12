"""camera command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_camera_primary(registry, *, toggle_camera) -> None:
    registry.register(
        CommandSpec(
            "camera.toggle",
            "Preview",
            "Start/stop the camera preview through the existing CameraPane owner.",
            "camera",
            toggle_camera,
            shortcut="Ctrl+P",
        )
    )


def register_camera(
    registry,
    *,
    camera_apply_oz_only_image_format,
    camera_auto_connect,
    camera_check_control,
    camera_check_rtsp,
    camera_discover_host_ip,
    camera_enable_rtsp,
    camera_reactivate,
    camera_refresh_image_format_status,
    camera_save_settings,
    camera_swap,
) -> None:
    for command_id, text, tooltip, category, handler in (
        (
            "camera.discover_host_ip",
            "Find camera network",
            "Discover the camera-network host/interface through the existing owner.",
            "camera",
            camera_discover_host_ip,
        ),
        (
            "camera.auto_connect",
            "Connect cameras",
            "Discover WIRIS control candidates, verify HIWS, enable RTSP, and apply the verified camera pair.",
            "camera",
            camera_auto_connect,
        ),
        (
            "camera.check_rtsp",
            "Check video",
            "Check RTSP ports and frame reception through the existing camera worker path.",
            "camera",
            camera_check_rtsp,
        ),
        (
            "camera.check_control",
            "Check control",
            "Check WIRIS control ports through the existing network path.",
            "camera",
            camera_check_control,
        ),
        (
            "camera.enable_rtsp",
            "Enable stream",
            "Enable WIRIS RTSP using the existing control and camera owner.",
            "camera",
            camera_enable_rtsp,
        ),
        (
            "camera.reactivate_left",
            "Reactivate CAM L",
            "Safely send SETH TRUE for the left WIRIS camera, verify frames, and reconnect only the left preview.",
            "camera",
            lambda: camera_reactivate("left"),
        ),
        (
            "camera.reactivate_right",
            "Reactivate CAM R",
            "Safely send SETH TRUE for the right WIRIS camera, verify frames, and reconnect only the right preview.",
            "camera",
            lambda: camera_reactivate("right"),
        ),
        (
            "camera.swap",
            "Swap L/R",
            "Swap the existing Left/Right camera mapping without changing coordinate contracts.",
            "camera",
            camera_swap,
        ),
        (
            "camera.save_settings",
            "Save camera",
            "Save the current camera settings through the existing settings owner.",
            "camera",
            camera_save_settings,
        ),
        (
            "camera.refresh_image_format_status",
            "새로고침",
            "Query GIVI/GIRJ/GISR/GIHR/GILC on both WIRIS cameras.",
            "camera",
            camera_refresh_image_format_status,
        ),
        (
            "camera.apply_oz_only_image_format",
            "OZ 전용 초기화",
            "Send SIRJ FALSE, SISR FALSE, SIHR FALSE, SIVI TRUE to both WIRIS cameras.",
            "camera",
            camera_apply_oz_only_image_format,
        ),
    ):
        registry.register(CommandSpec(command_id, text, tooltip, category, handler))
