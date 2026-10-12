"""advanced command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_advanced(
    registry,
    *,
    advanced_baseline_scan,
    advanced_media_open_video,
    advanced_preview_refresh,
    advanced_px4_apply_mag,
    advanced_px4_reboot,
    advanced_px4_refresh_attitude,
    advanced_px4_refresh_mag,
    advanced_px4_save_mag,
    advanced_px4_shell,
    advanced_watch_add_mavlink,
    advanced_watch_discover_mavlink,
    advanced_watch_remove,
    advanced_wiris_send,
    advanced_zoom,
) -> None:
    for command_id, text, tooltip, handler in (
        (
            "advanced.wiris.send",
            "Send WIRIS SDK command",
            "Send the selected raw WIRIS SDK command through the existing camera control owner.",
            advanced_wiris_send,
        ),
        ("advanced.zoom.in", "Zoom in", "Send the existing WIRIS zoom-in command.", lambda: advanced_zoom("in")),
        ("advanced.zoom.out", "Zoom out", "Send the existing WIRIS zoom-out command.", lambda: advanced_zoom("out")),
        (
            "advanced.zoom.stop",
            "Stop zoom",
            "Stop the existing WIRIS focus/zoom action.",
            lambda: advanced_zoom("stop"),
        ),
        (
            "advanced.zoom.set",
            "Set zoom level",
            "Send the existing absolute WIRIS visible zoom command.",
            lambda: advanced_zoom("set"),
        ),
        (
            "advanced.watch.add_mavlink",
            "Add MAVLink watch",
            "Add a MAVLink field watch to the existing Pixhawk telemetry timeline.",
            advanced_watch_add_mavlink,
        ),
        (
            "advanced.watch.discover_mavlink",
            "Discover MAVLink watch",
            "Open the existing MAVLink message/field browser.",
            advanced_watch_discover_mavlink,
        ),
        (
            "advanced.watch.remove",
            "Remove field watch",
            "Remove the selected custom field watch from the existing subscriber timeline.",
            advanced_watch_remove,
        ),
        (
            "advanced.baseline.scan",
            "Scan baseline fields",
            "Refresh MAVLink baseline field evidence through the existing owner.",
            advanced_baseline_scan,
        ),
        (
            "advanced.px4.refresh_mag",
            "Refresh PX4 MAG",
            "Read the selected PX4 magnetometer status through the existing parameter manager.",
            advanced_px4_refresh_mag,
        ),
        (
            "advanced.px4.refresh_attitude",
            "Refresh attitude calibration",
            "Read selected PX4 accelerometer/gyro/board-rotation calibration state.",
            advanced_px4_refresh_attitude,
        ),
        ("advanced.px4.shell", "Open PX4 Shell", "Open the existing per-Pixhawk PX4 Shell dialog.", advanced_px4_shell),
        (
            "advanced.px4.apply_mag",
            "Apply PX4 MAG priorities",
            "Apply selected CAL_MAG priority values through the existing guarded manager.",
            advanced_px4_apply_mag,
        ),
        (
            "advanced.px4.save_mag",
            "Save PX4 MAG parameters",
            "Save PX4 MAG parameters through the existing MAVLink/PX4 shell path.",
            advanced_px4_save_mag,
        ),
        (
            "advanced.px4.reboot",
            "Reboot PX4",
            "Reboot the selected Pixhawk through the existing guarded MAVLink path.",
            advanced_px4_reboot,
        ),
        (
            "advanced.preview.refresh",
            "Refresh calibration preview",
            "Refresh checkerboard preview through the existing lazy preview worker.",
            advanced_preview_refresh,
        ),
        (
            "advanced.media.open_video",
            "Open video",
            "Open the selected recorded video with the system-associated player.",
            advanced_media_open_video,
        ),
    ):
        registry.register(CommandSpec(command_id, text, tooltip, "advanced", handler))
