"""devices command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec
from Workstation.workstation_core.mavlink_liveness import MAVLINK_QUALIFICATION_S
from Workstation.workstation_core.px4_calibration_groups import PX4_CALIBRATION_GROUPS


def register_devices(
    registry,
    *,
    pixhawk_board_rotation_refresh,
    pixhawk_connect_mavlink,
    pixhawk_disconnect_mavlink,
    pixhawk_discover_host_ip,
    pixhawk_discover_mavlink_candidates,
    pixhawk_discover_usb,
    pixhawk_mag_apply,
    pixhawk_mag_refresh,
    pixhawk_recover_eth_mavlink,
    pixhawk_refresh_parameters,
    pixhawk_reset_level_calibration,
    pixhawk_toggle_mavlink,
) -> None:
    for command_id, text, tooltip, category, handler in (
        (
            "pixhawk.toggle_mavlink",
            "Connect Pixhawk",
            "Legacy toggle for the existing MAVLink Pixhawk path.",
            "device",
            pixhawk_toggle_mavlink,
        ),
        (
            "pixhawk.connect_mavlink",
            "Connect Pixhawk",
            "Connect the configured P1/P2 MAVLink Pixhawk endpoints.",
            "device",
            pixhawk_connect_mavlink,
        ),
        (
            "pixhawk.disconnect_mavlink",
            "Disconnect Pixhawk",
            "Disconnect the configured P1/P2 MAVLink Pixhawk endpoints.",
            "device",
            pixhawk_disconnect_mavlink,
        ),
        (
            "pixhawk.reset_level_calibration",
            "Reset level cal.",
            "Reset only PX4 SENS_BOARD_X_OFF/Y_OFF to firmware defaults on connected Pixhawks.",
            "device",
            pixhawk_reset_level_calibration,
        ),
        (
            "pixhawk.refresh_parameters.p1",
            "Refresh P1 parameters",
            "Read P1 PX4 calibration parameters written by QGC and show their raw sensor-axis values.",
            "device",
            lambda: pixhawk_refresh_parameters("left"),
        ),
        (
            "pixhawk.refresh_parameters.p2",
            "Refresh P2 parameters",
            "Read P2 PX4 calibration parameters written by QGC and show their raw sensor-axis values.",
            "device",
            lambda: pixhawk_refresh_parameters("right"),
        ),
        (
            "pixhawk.mag.refresh.p1",
            "Refresh P1 MAG",
            "Read P1 MAG priorities and active-selection evidence.",
            "device",
            lambda: pixhawk_mag_refresh("left"),
        ),
        (
            "pixhawk.mag.refresh.p2",
            "Refresh P2 MAG",
            "Read P2 MAG priorities and active-selection evidence.",
            "device",
            lambda: pixhawk_mag_refresh("right"),
        ),
        (
            "pixhawk.board_rotation.refresh.p1",
            "Refresh P1 board rotation",
            "Read P1 SENS_BOARD_ROT so internal/GPS MAG rows can show a CAL_MAG-corrected yaw.",
            "device",
            lambda: pixhawk_board_rotation_refresh("left"),
        ),
        (
            "pixhawk.board_rotation.refresh.p2",
            "Refresh P2 board rotation",
            "Read P2 SENS_BOARD_ROT so internal/GPS MAG rows can show a CAL_MAG-corrected yaw.",
            "device",
            lambda: pixhawk_board_rotation_refresh("right"),
        ),
        (
            "pixhawk.mag.apply.p1",
            "Apply P1 MAG",
            "Apply P1's visible MAG priorities and begin independent active-device verification.",
            "device",
            lambda: pixhawk_mag_apply("left"),
        ),
        (
            "pixhawk.mag.apply.p2",
            "Apply P2 MAG",
            "Apply P2's visible MAG priorities and begin independent active-device verification.",
            "device",
            lambda: pixhawk_mag_apply("right"),
        ),
        (
            "pixhawk.recover_eth_mavlink.p1",
            "Recover P1 via USB",
            f"For a confirmed P1 ETH RX stall, revalidate a unique USB heartbeat sysid=1, ask for confirmation, reboot only P1, then require {MAVLINK_QUALIFICATION_S:.0f}-second requalification.",
            "device",
            lambda: pixhawk_recover_eth_mavlink("left"),
        ),
        (
            "pixhawk.recover_eth_mavlink.p2",
            "Recover P2 via USB",
            f"For a confirmed P2 ETH RX stall, revalidate a unique USB heartbeat sysid=2, ask for confirmation, reboot only P2, then require {MAVLINK_QUALIFICATION_S:.0f}-second requalification.",
            "device",
            lambda: pixhawk_recover_eth_mavlink("right"),
        ),
        (
            "pixhawk.discover_usb",
            "Find USB Pixhawks",
            "List USB serial candidates and MAVLink heartbeat evidence without auto-assigning P1/P2.",
            "device",
            pixhawk_discover_usb,
        ),
        (
            "pixhawk.discover_mavlink_candidates",
            "Find network links",
            "Read wired NIC, IPv4 and P1/P2 kernel-route choices without changing host networking.",
            "device",
            pixhawk_discover_mavlink_candidates,
        ),
        (
            "pixhawk.discover_host_ip",
            "Find Pixhawk host IP",
            "List local IPv4 interfaces that can be used for Pixhawk Ethernet candidates.",
            "device",
            pixhawk_discover_host_ip,
        ),
    ):
        registry.register(CommandSpec(command_id, text, tooltip, category, handler))


def register_device_resets(registry, *, pixhawk_reset_calibration) -> None:
    for side_code, camera in (("p1", "left"), ("p2", "right")):
        for calibration in PX4_CALIBRATION_GROUPS:
            if not calibration.resettable:
                continue
            command_id = f"pixhawk.reset_calibration.{side_code}.{calibration.key}"
            registry.register(
                CommandSpec(
                    command_id,
                    f"Reset {side_code.upper()} {calibration.label}",
                    f"Reset only {', '.join(calibration.reset_parameter_names)} to the connected PX4 firmware defaults.",
                    "device",
                    lambda camera=camera, group=calibration.key: pixhawk_reset_calibration(camera, group),
                )
            )


def register_device_completion(registry) -> None:
    for side_code in ("p1", "p2"):
        for calibration in PX4_CALIBRATION_GROUPS:
            if calibration.resettable:
                registry.set_deferred_completion(f"pixhawk.reset_calibration.{side_code}.{calibration.key}")
