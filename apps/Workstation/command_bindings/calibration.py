"""calibration command declarations; caller supplies only named actions.

Labels, payload closures, order and deferred-completion policy are preserved.
This module does not read the window, backend, settings or feature state.
"""

from __future__ import annotations
from Workstation.application.command_registry import CommandSpec


def register_calibration(
    registry,
    *,
    calibration_apply_bundle,
    calibration_capture_images,
    calibration_capture_images_wiris_ftp,
    calibration_capture_images_wiris_ftp_oz,
    calibration_capture_images_wiris_oz_capture_only,
    calibration_load_extrinsics,
    calibration_load_intrinsics,
    calibration_run_extrinsics,
    calibration_run_intrinsic,
    calibration_save_extrinsics,
    calibration_save_intrinsics,
    calibration_sync_images_wiris_oz_ftp,
    calibration_toggle_auto_capture,
) -> None:
    for command_id, text, tooltip, handler in (
        (
            "calibration.capture_images",
            "Capture Live Pair",
            "Capture the newest decoded native pair from the live RTSP streams through the existing checkerboard owner.",
            calibration_capture_images,
        ),
        (
            "calibration.capture_images_wiris_ftp",
            "Capture HR Still (FTP)",
            "Trigger a WIRIS HR still on both cameras and download it via FTP into this calibration run, instead of the live RTSP feed.",
            calibration_capture_images_wiris_ftp,
        ),
        (
            "calibration.capture_images_wiris_ftp_oz",
            "Capture OZ Still (FTP)",
            "Same as Capture HR Still (FTP), using the lower-resolution operating-zoom still (SIVI) instead of the HR still -- faster, no camera reboot risk.",
            calibration_capture_images_wiris_ftp_oz,
        ),
        (
            "calibration.capture_images_wiris_oz_capture_only",
            "OZ 캡처",
            "Trigger an OZ still on both cameras and leave it on camera storage -- no FTP download, so it can be pressed repeatedly and quickly.",
            calibration_capture_images_wiris_oz_capture_only,
        ),
        (
            "calibration.sync_images_wiris_oz_ftp",
            "FTP 자동 전송",
            "Download every OZ still accumulated on camera storage since the last sync, pairing left/right by trigger order, not by camera clock.",
            calibration_sync_images_wiris_oz_ftp,
        ),
        (
            "calibration.auto_capture",
            "Auto capture",
            "Start/stop validated automatic checkerboard pair capture through the existing owner.",
            calibration_toggle_auto_capture,
        ),
        (
            "calibration.run_intrinsic",
            "Run calibration",
            "Run the existing intrinsic calibration pipeline without duplicating camera math.",
            calibration_run_intrinsic,
        ),
        (
            "calibration.run_extrinsics",
            "Calculate R,t only",
            "Estimate OpenCV optical RDF left-to-right R,t with the selected K,D fixed.",
            calibration_run_extrinsics,
        ),
        (
            "calibration.save_intrinsics",
            "Save intrinsics",
            "Save the current intrinsic profile through the existing calibration owner.",
            calibration_save_intrinsics,
        ),
        (
            "calibration.save_extrinsics",
            "Save R,t",
            "Save the selected stereo R,t as an independent profile.",
            calibration_save_extrinsics,
        ),
        (
            "calibration.load_intrinsics",
            "Load intrinsics",
            "Choose an intrinsic JSON file and apply it.",
            calibration_load_intrinsics,
        ),
        (
            "calibration.load_extrinsics",
            "Load R,t",
            "Load a standalone R,t profile or legacy intrinsics JSON with embedded extrinsics.",
            calibration_load_extrinsics,
        ),
        (
            "calibration.apply_bundle",
            "Apply K,D + R,t",
            "Apply the independently selected K,D and R,t; source-zoom mismatch is informational only.",
            calibration_apply_bundle,
        ),
    ):
        registry.register(CommandSpec(command_id, text, tooltip, "calibration", handler))
