"""telemetry command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_telemetry(
    registry,
    *,
    telemetry_apply,
    telemetry_connect,
    telemetry_disconnect,
    telemetry_discover_network,
    telemetry_discover_usb,
    telemetry_save_settings,
    telemetry_use_network_candidate,
) -> None:
    for command_id, text, tooltip, category, handler in (
        (
            "telemetry.apply",
            "Apply drone telemetry",
            "Apply the active Drone Pixhawk endpoint and RTCM forwarding settings.",
            "device",
            telemetry_apply,
        ),
        (
            "telemetry.connect",
            "Connect drone telemetry",
            "Connect the active Drone Pixhawk MAVLink endpoint and receive its GPS position.",
            "device",
            telemetry_connect,
        ),
        (
            "telemetry.disconnect",
            "Disconnect drone telemetry",
            "Disconnect the active Drone Pixhawk MAVLink endpoint.",
            "device",
            telemetry_disconnect,
        ),
        (
            "telemetry.save_settings",
            "Save drone telemetry",
            "Persist the active Drone Pixhawk endpoint and RTCM forwarding settings.",
            "device",
            telemetry_save_settings,
        ),
        (
            "telemetry.discover_usb",
            "Find USB telemetry radio",
            "Probe /dev/serial/by-id, ttyACM, and ttyUSB candidates for a MAVLink heartbeat and fill the URL.",
            "device",
            telemetry_discover_usb,
        ),
        (
            "telemetry.discover_network",
            "Find Network drone telemetry",
            "Passively find wired MAVLink sources and identify Drone sysid 3 without changing networking.",
            "device",
            telemetry_discover_network,
        ),
        (
            "telemetry.use_network_candidate",
            "Use selected Drone network candidate",
            "Fill the MAVLink URL from the selected current sysid 3 candidate without connecting.",
            "device",
            telemetry_use_network_candidate,
        ),
    ):
        registry.register(CommandSpec(command_id, text, tooltip, category, handler))
