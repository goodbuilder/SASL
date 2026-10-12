"""measurement command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_measurement_primary(registry, *, toggle_detection) -> None:
    registry.register(
        CommandSpec(
            "detection.toggle",
            "측정 시작",
            "Create a Measurement and start Detection, or stop and automatically save the active Measurement.",
            "detection",
            toggle_detection,
            shortcut="Ctrl+D",
        )
    )


def register_measurement_alignment(
    registry,
    *,
    alignment_load_profile,
    alignment_save_profile,
    alignment_scan_sources,
    alignment_start_sampling,
    alignment_stop_sampling,
    alignment_swap_pixhawks,
    calibration_apply_setup,
    calibration_save_setup,
    measurement_apply_settings,
    measurement_save_settings,
) -> None:
    for command_id, text, tooltip, handler in (
        (
            "measurement.apply_settings",
            "Apply",
            "Apply Workstation measurement fields through the existing owner.",
            measurement_apply_settings,
        ),
        (
            "measurement.save_settings",
            "Save",
            "Persist the current measurement settings without changing measurement schemas or coordinate frames.",
            measurement_save_settings,
        ),
        (
            "calibration.apply_setup",
            "Apply setup",
            "Apply checkerboard geometry, calculation mode, and capture path through the existing owner.",
            calibration_apply_setup,
        ),
        (
            "calibration.save_setup",
            "Save setup",
            "Persist the current Workstation calibration setup through the existing settings owner.",
            calibration_save_setup,
        ),
        (
            "alignment.scan_sources",
            "Find sources",
            "Refresh MAVLink alignment source evidence through the existing owner.",
            alignment_scan_sources,
        ),
        (
            "alignment.start_sampling",
            "Start sampling",
            "Start the existing GNSS baseline quality sampling workflow.",
            alignment_start_sampling,
        ),
        (
            "alignment.stop_sampling",
            "Stop sampling",
            "Stop the existing GNSS baseline quality sampling workflow.",
            alignment_stop_sampling,
        ),
        (
            "alignment.save_profile",
            "Save profile",
            "Save the existing alignment profile without changing NED or camera-frame contracts.",
            alignment_save_profile,
        ),
        (
            "alignment.load_profile",
            "Load profile",
            "Load the existing alignment profile through the owner.",
            alignment_load_profile,
        ),
        (
            "alignment.swap_pixhawks",
            "Swap P1/P2",
            "Swap P1/P2 mapping through the existing owner; coordinate labels remain explicit.",
            alignment_swap_pixhawks,
        ),
    ):
        category = (
            "measurement"
            if command_id.startswith("measurement.")
            else "calibration"
            if command_id.startswith("calibration.")
            else "alignment"
        )
        registry.register(CommandSpec(command_id, text, tooltip, category, handler))
