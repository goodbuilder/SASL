"""completion command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_completion(registry) -> None:
    for command_id in (
        "camera.toggle",
        "detection.toggle",
        "recording.toggle",
        "camera.discover_host_ip",
        "camera.auto_connect",
        "camera.check_rtsp",
        "camera.check_control",
        "camera.enable_rtsp",
        "camera.reactivate_left",
        "camera.reactivate_right",
        "time.set_pc",
        "time.check_sync",
        "pixhawk.toggle_mavlink",
        "pixhawk.connect_mavlink",
        "pixhawk.disconnect_mavlink",
        "pixhawk.reset_level_calibration",
        "pixhawk.refresh_parameters.p1",
        "pixhawk.refresh_parameters.p2",
        "pixhawk.mag.refresh.p1",
        "pixhawk.mag.refresh.p2",
        "pixhawk.board_rotation.refresh.p1",
        "pixhawk.board_rotation.refresh.p2",
        "pixhawk.mag.apply.p1",
        "pixhawk.mag.apply.p2",
        "pixhawk.recover_eth_mavlink.p1",
        "pixhawk.recover_eth_mavlink.p2",
        "pixhawk.discover_usb",
        "pixhawk.discover_host_ip",
        "gnss.base_discover",
        "gnss.base_connect",
        "gnss.base_disconnect",
        "gnss.survey_start",
        "gnss.survey_stop",
        "gnss.ntrip_start",
        "gnss.ntrip_stop",
        "calibration.capture_images",
        "calibration.run_intrinsic",
        "calibration.run_extrinsics",
        "calibration.load_intrinsics",
        "calibration.load_extrinsics",
        "advanced.wiris.send",
        "advanced.zoom.in",
        "advanced.zoom.out",
        "advanced.zoom.stop",
        "advanced.zoom.set",
        "advanced.watch.discover_mavlink",
        "advanced.px4.refresh_mag",
        "advanced.px4.refresh_attitude",
        "advanced.px4.shell",
        "advanced.px4.apply_mag",
        "advanced.px4.save_mag",
        "advanced.px4.reboot",
        "advanced.preview.refresh",
        "advanced.media.open_video",
    ):
        registry.set_deferred_completion(command_id)
