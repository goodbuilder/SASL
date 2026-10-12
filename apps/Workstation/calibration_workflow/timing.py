"""Capture pairing evidence; camera optical pixels and host/exposure clocks stay distinct."""
from __future__ import annotations
import math
from typing import Mapping
from Workstation.workstation_core.calibration_timing import (
    CAMERA_TIMESTAMP_VERIFIED, EXPOSURE_TIMESTAMP_VERIFIED, HOST_DISPATCH_PROXY,
    HOST_RECEIVE_PROXY, UNKNOWN_TIMING_EVIDENCE, CalibrationTimingEvidence,
)
CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S = 0.100


class CaptureTiming:
    @staticmethod
    def _app_receive_monotonic_ns_from_metadata(
        metadata: Mapping[str, object],
    ) -> tuple[int | None, str]:
        """Return one decoded-frame host receipt stamp and its field source.

        Normal camera metadata carries the exact integer field.  Native-ring
        descriptors also guarantee that ``timestamp_s`` is the producer's
        app-receive monotonic clock, so it remains a bounded compatibility
        source when preview throttling omitted the redundant integer field.
        """

        exact = metadata.get("app_receive_monotonic_ns")
        if exact is not None:
            if isinstance(exact, bool):
                return None, ""
            try:
                if isinstance(exact, float) and (
                    not math.isfinite(exact) or not exact.is_integer()
                ):
                    raise ValueError
                exact_ns = int(exact)
            except (TypeError, ValueError, OverflowError):
                return None, ""
            if exact_ns <= 0:
                return None, ""
            return exact_ns, "app_receive_monotonic_ns"

        compatibility = metadata.get("timestamp_s")
        if compatibility is None or isinstance(compatibility, bool):
            return None, ""
        try:
            compatibility_s = float(compatibility)
        except (TypeError, ValueError, OverflowError):
            return None, ""
        if not math.isfinite(compatibility_s) or compatibility_s <= 0.0:
            return None, ""
        return int(round(compatibility_s * 1_000_000_000.0)), "timestamp_s"


    def _checkerboard_capture_pair_timing_evidence(
        self,
        metadata_by_camera: dict[str, dict[str, object]] | None = None,
        *,
        maximum_delta_s: float | None = None,
    ) -> CalibrationTimingEvidence:
        metadata_by_camera = metadata_by_camera or {}
        left_metadata = metadata_by_camera.get("left") or {}
        right_metadata = metadata_by_camera.get("right") or {}
        limit_s = (
            CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S
            if maximum_delta_s is None
            else float(maximum_delta_s)
        )
        if not math.isfinite(limit_s) or limit_s < 0.0:
            limit_s = CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S

        def exposure_timestamp(metadata: Mapping[str, object]) -> float | None:
            if metadata.get("exposure_timestamp_verified") is not True:
                return None
            try:
                if metadata.get("exposure_timestamp_unix_ns") is not None:
                    return int(metadata["exposure_timestamp_unix_ns"]) * 1.0e-9
                if metadata.get("exposure_timestamp_unix_s") is not None:
                    return float(metadata["exposure_timestamp_unix_s"])
            except (TypeError, ValueError, OverflowError):
                return None
            return None

        left_exposure = exposure_timestamp(left_metadata)
        right_exposure = exposure_timestamp(right_metadata)
        if left_exposure is not None or right_exposure is not None:
            if left_exposure is None or right_exposure is None:
                return CalibrationTimingEvidence(
                    UNKNOWN_TIMING_EVIDENCE,
                    False,
                    False,
                    False,
                    "camera-exposure",
                    None,
                    limit_s,
                    "verified exposure timestamp is missing for one camera",
                )
            delta_s = right_exposure - left_exposure
            allowed = abs(delta_s) <= limit_s
            return CalibrationTimingEvidence(
                EXPOSURE_TIMESTAMP_VERIFIED,
                allowed,
                allowed,
                True,
                "camera-exposure",
                delta_s,
                limit_s,
                "verified exposure timestamp pair within gate"
                if allowed
                else (
                    f"exposure timestamp delta {delta_s * 1000.0:.1f} ms exceeds "
                    f"{limit_s * 1000.0:.1f} ms"
                ),
            )

        left_ftp_id = str(left_metadata.get("ftp_capture_id") or "")
        right_ftp_id = str(right_metadata.get("ftp_capture_id") or "")
        if left_ftp_id or right_ftp_id:
            contract = "same_capture_id_and_synchronized_cptr_dispatch_v1"
            if not left_ftp_id or left_ftp_id != right_ftp_id:
                return CalibrationTimingEvidence(
                    HOST_DISPATCH_PROXY, False, False, False,
                    "ftp-cptr-dispatch", None, limit_s, "FTP capture_id mismatch",
                )
            if (
                left_metadata.get("capture_pairing_contract") != contract
                or right_metadata.get("capture_pairing_contract") != contract
            ):
                return CalibrationTimingEvidence(
                    HOST_DISPATCH_PROXY,
                    False,
                    False,
                    False,
                    "ftp-cptr-dispatch",
                    None,
                    limit_s,
                    "FTP still lacks the synchronized CPTR dispatch pairing contract",
                )
            timestamp_fields = (
                ("capture_requested_monotonic_ns", 1.0e-9, "ftp-cptr-dispatch-monotonic"),
                ("capture_requested_unix_ns", 1.0e-9, "ftp-cptr-dispatch-unix"),
            )
            for field, scale, basis in timestamp_fields:
                try:
                    left_value = int(left_metadata.get(field))
                    right_value = int(right_metadata.get(field))
                except (TypeError, ValueError, OverflowError):
                    continue
                if left_value <= 0 or right_value <= 0:
                    continue
                delta_s = (right_value - left_value) * scale
                if abs(delta_s) > limit_s:
                    return CalibrationTimingEvidence(
                        HOST_DISPATCH_PROXY,
                        False,
                        False,
                        False,
                        basis,
                        delta_s,
                        limit_s,
                        (
                            f"FTP CPTR dispatch delta {delta_s * 1000.0:.1f} ms exceeds "
                            f"{limit_s * 1000.0:.1f} ms; exposure timestamps remain unknown"
                        ),
                    )
                return CalibrationTimingEvidence(
                    HOST_DISPATCH_PROXY,
                    True,
                    False,
                    False,
                    basis,
                    delta_s,
                    limit_s,
                    (
                        "same FTP capture_id; synchronized CPTR dispatch within gate; "
                        "camera exposure timestamps unknown"
                    ),
                )
            return CalibrationTimingEvidence(
                HOST_DISPATCH_PROXY,
                False,
                False,
                False,
                "ftp-cptr-dispatch",
                None,
                limit_s,
                "FTP still lacks per-camera CPTR dispatch timestamps",
            )

        left_receive_ns, left_receive_source = (
            self._app_receive_monotonic_ns_from_metadata(left_metadata)
        )
        right_receive_ns, right_receive_source = (
            self._app_receive_monotonic_ns_from_metadata(right_metadata)
        )
        if left_receive_ns is None or right_receive_ns is None:
            return CalibrationTimingEvidence(
                UNKNOWN_TIMING_EVIDENCE,
                False,
                False,
                False,
                "app-receive-monotonic",
                None,
                limit_s,
                "missing decoded-frame app-receive monotonic timestamp for left or right frame",
            )
        delta_ns = right_receive_ns - left_receive_ns
        delta_s = delta_ns * 1.0e-9
        source_detail = (
            "exact integer metadata"
            if left_receive_source == right_receive_source == "app_receive_monotonic_ns"
            else (
                "native-frame timestamp_s compatibility"
                if left_receive_source == right_receive_source == "timestamp_s"
                else "mixed exact/compatibility app-receive metadata"
            )
        )
        if abs(delta_s) > limit_s:
            return CalibrationTimingEvidence(
                HOST_RECEIVE_PROXY,
                False,
                False,
                False,
                "app-receive-monotonic",
                delta_s,
                limit_s,
                f"decoded-frame app-receive pair delta {delta_s * 1000.0:.1f} ms exceeds "
                f"{limit_s * 1000.0:.1f} ms ({source_detail}); camera exposure remains unverified",
            )
        return CalibrationTimingEvidence(
            HOST_RECEIVE_PROXY,
            True,
            False,
            False,
            "app-receive-monotonic",
            delta_s,
            limit_s,
            f"decoded-frame app-receive delta within gate ({source_detail}); "
            "camera exposure remains unverified",
        )


    def _checkerboard_capture_pair_timing(
        self,
        metadata_by_camera: dict[str, dict[str, object]] | None = None,
        *,
        maximum_delta_s: float | None = None,
    ) -> tuple[bool, str, float | None, str]:
        evidence = self._checkerboard_capture_pair_timing_evidence(
            metadata_by_camera,
            maximum_delta_s=maximum_delta_s,
        )
        compatibility_detail = (
            "ok"
            if evidence.pair_persistence_allowed
            and evidence.evidence_class == CAMERA_TIMESTAMP_VERIFIED
            else evidence.detail
        )
        return (
            evidence.pair_persistence_allowed,
            evidence.timestamp_reference,
            evidence.pair_delta_s,
            compatibility_detail,
        )


