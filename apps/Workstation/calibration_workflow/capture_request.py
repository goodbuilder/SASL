"""Owned capture input snapshot shared by manual, auto and downloaded stills.

Image arrays stay native optical pixels (x right/y down). Timing carries the
existing host proxy/exposure classification without altering any quality gate.
"""
from __future__ import annotations
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from PyQt5 import QtGui
from Workstation.workstation_core.calibration_quality import CalibrationViewGeometry
from Workstation.workstation_core.calibration_timing import UNKNOWN_TIMING_EVIDENCE, CalibrationTimingEvidence
from .timing import CaptureTiming
from Workstation.calibration_workflow.calibration_sessions import write_calibration_session_manifest


@dataclass(frozen=True)
class CaptureRequest:
    root: Path
    stem: str
    origin: str
    frames: dict
    config: object
    opencv_error: object
    timing: CalibrationTimingEvidence
    metadata_payload: dict

    def worker_arguments(self):
        return dict(
            root=self.root, frames=dict(self.frames), stem=self.stem,
            config=self.config, opencv_error=self.opencv_error,
            pair_timing_ok=self.timing.pair_persistence_allowed,
            pair_time_basis=self.timing.timestamp_reference,
            pair_delta_s=self.timing.pair_delta_s,
            pair_timing_detail=self.timing.detail,
            metadata_payload=deepcopy(self.metadata_payload),
            timing_evidence=self.timing.to_mapping(),
        )


class CaptureRequestBuilder:
    @staticmethod
    def build(*, root, stem, frames, metadata_by_camera, preview_metadata,
              capture_origin, pair_max_delta_s, zoom_snapshot, config,
              opencv_error, pose_views=None, pose_novelty=None, copy_frames=True):
        if copy_frames:
            frames = {
                side: None if image is None else image.convertToFormat(QtGui.QImage.Format_RGB888).copy()
                for side, image in frames.items()
            }
        else:
            # The outer native-source adapter transferred copies made before
            # session I/O. Keep the original capture-time pixel identity.
            frames = dict(frames)
        metadata_by_camera = deepcopy(metadata_by_camera)
        preview_metadata = deepcopy(preview_metadata)
        pair_candidate = frames["left"] is not None and frames["right"] is not None
        timing_evidence = (
            CaptureTiming()._checkerboard_capture_pair_timing_evidence(
                metadata_by_camera,
                maximum_delta_s=pair_max_delta_s,
            )
            if pair_candidate
            else CalibrationTimingEvidence(
                UNKNOWN_TIMING_EVIDENCE,
                False,
                False,
                False,
                "none",
                None,
                pair_max_delta_s,
                "single-camera capture",
            )
        )
        pair_timing_ok = timing_evidence.pair_persistence_allowed
        pair_time_basis = timing_evidence.timestamp_reference
        pair_delta_s = timing_evidence.pair_delta_s
        pair_timing_detail = timing_evidence.detail
        app_receive_samples = {
            side: (
                CaptureTiming._app_receive_monotonic_ns_from_metadata(
                    metadata_by_camera.get(side) or {}
                )
                if frames.get(side) is not None
                else (None, "")
            )
            for side in ("left", "right")
        }
        left_app_receive_ns = app_receive_samples["left"][0]
        right_app_receive_ns = app_receive_samples["right"][0]
        app_receive_pair_delta_ns = (
            right_app_receive_ns - left_app_receive_ns
            if left_app_receive_ns is not None and right_app_receive_ns is not None
            else None
        )
        metadata_payload = {
            "capture_origin": str(capture_origin),
            "capture_recorded_unix_ns": time.time_ns(),
            "pair_time_basis": pair_time_basis,
            "pair_delta_s": pair_delta_s,
            "pair_delta_ns": (
                app_receive_pair_delta_ns
                if pair_time_basis == "app-receive-monotonic"
                else None
            ),
            "pair_max_delta_s": pair_max_delta_s,
            "pair_timing_status": pair_timing_detail,
            "calibration_timing_evidence": timing_evidence.to_mapping(),
            "timing_evidence_class": timing_evidence.evidence_class,
            "pair_persistence_allowed": timing_evidence.pair_persistence_allowed,
            "precision_timing_qualified": timing_evidence.precision_qualified,
            "actual_exposure_verified": timing_evidence.actual_exposure_verified,
            "left_app_receive_monotonic_ns": left_app_receive_ns,
            "right_app_receive_monotonic_ns": right_app_receive_ns,
            "app_receive_pair_delta_ns": app_receive_pair_delta_ns,
            "left_rtcp_frame_ntp_unix_s": (metadata_by_camera.get("left") or {}).get("rtcp_frame_ntp_unix_s"),
            "right_rtcp_frame_ntp_unix_s": (metadata_by_camera.get("right") or {}).get("rtcp_frame_ntp_unix_s"),
            "frame_contract": (
                "OpenCV source image pixels (x right, y down); no application resize/crop/overlay; "
                "native/preview/saved grids are explicit per side; "
                f"stereo pairing gate uses {pair_time_basis}; decoded-frame app receipt and "
                "FTP CPTR dispatch are host-side pairing proxies and never exposure timestamps"
            ),
            "frame_grid_by_side": {
                side: {
                    "frame_id": str((metadata_by_camera.get(side) or {}).get("frame_id") or ""),
                    "frame_key": str(
                        (metadata_by_camera.get(side) or {}).get("frame_key") or ""
                    ),
                    "h264_access_unit_kind": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_access_unit_kind"),
                    "h264_access_unit_nal_types": list(
                        (metadata_by_camera.get(side) or {}).get(
                            "h264_access_unit_nal_types", ()
                        )
                        or ()
                    ),
                    "h264_access_unit_is_delta": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_access_unit_is_delta"),
                    "h264_access_unit_pts_ns": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_access_unit_pts_ns"),
                    "decoded_buffer_pts_ns": (
                        metadata_by_camera.get(side) or {}
                    ).get("decoded_buffer_pts_ns"),
                    "h264_nal1_non_idr_intra": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_nal1_non_idr_intra"),
                    "h264_random_access_guaranteed": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_random_access_guaranteed"),
                    "app_receive_monotonic_ns": app_receive_samples[side][0],
                    "app_receive_timestamp_source": app_receive_samples[side][1],
                    "source_native_width": int(
                        (metadata_by_camera.get(side) or {}).get("native_width")
                        or (frames.get(side).width() if frames.get(side) is not None else 0)
                    ),
                    "source_native_height": int(
                        (metadata_by_camera.get(side) or {}).get("native_height")
                        or (frames.get(side).height() if frames.get(side) is not None else 0)
                    ),
                    "saved_width": frames.get(side).width() if frames.get(side) is not None else 0,
                    "saved_height": frames.get(side).height() if frames.get(side) is not None else 0,
                    "preview_width": int(
                        (preview_metadata.get(side) or {}).get(
                            "preview_width", 0
                        )
                    ),
                    "preview_height": int(
                        (preview_metadata.get(side) or {}).get(
                            "preview_height", 0
                        )
                    ),
                    "resize_scale_xy": [1.0, 1.0],
                    "crop_xywh": None,
                    "overlay_applied": False,
                    "native_lossless_source": bool(
                        (metadata_by_camera.get(side) or {}).get("native_lossless_source", False)
                    ),
                }
                for side in ("left", "right")
            },
            "auto_pose_geometry_by_side": {
                side: view.to_mapping()
                for side, view in (
                    pose_views or {}
                ).items()
                if isinstance(view, CalibrationViewGeometry)
            },
            "auto_pose_novelty_by_side": dict(
                pose_novelty or {}
            ),
            **zoom_snapshot,
        }
        return CaptureRequest(
            root=Path(root), stem=str(stem), origin=str(capture_origin),
            frames=frames, config=config, opencv_error=opencv_error,
            timing=timing_evidence, metadata_payload=deepcopy(metadata_payload),
        )


@dataclass(frozen=True)
class CaptureRootResolution:
    root: Path | None
    session_roots: dict
    session_key: str = ""
    source: str = ""
    activate: bool = False
    cache_changed: bool = False
    created: bool = False
    pre_session: bool = False


class CaptureDirectories:
    """Capture folder layout and manifest I/O over an explicit Session snapshot.

    Session identity and configured paths are sampled by the application adapter.
    The supplied session_roots dict is its one live path cache; only this
    operation mutates that named cache. No UI, Session resolver or live settings
    object crosses this boundary. Manifest publication remains a separate step
    so the adapter can preserve its existing path-state/failure ordering.
    """

    write_manifest = staticmethod(write_calibration_session_manifest)

    @staticmethod
    def current_root(*, configured_root, default_root, hierarchy_enabled, session_root):
        if session_root is not None and (configured_root == default_root or hierarchy_enabled):
            return Path(session_root)
        return configured_root

    @staticmethod
    def resolve_session_root(
        *, configured_root: Path, default_root: Path | None,
        hierarchy_enabled: bool, session_state: str, session_id: str,
        session_directory: Path | None, capture_origin: str, session_roots,
        stamp: str | None = None,
    ) -> CaptureRootResolution:
        roots = session_roots if isinstance(session_roots, dict) else {}
        if hierarchy_enabled:
            if session_state == "OPEN":
                if not session_id or session_directory is None:
                    raise RuntimeError("open Session calibration owner is unavailable")
                session_base = Path(session_directory) / "calibration"
            else:
                configured_root.mkdir(parents=True, exist_ok=True)
                return CaptureRootResolution(configured_root, roots, source=f"pre_session:{capture_origin}", pre_session=True)
        else:
            if configured_root != default_root:
                return CaptureRootResolution(None, roots)
            session_base = default_root
        origin_key = "auto" if str(capture_origin).strip().lower() == "auto" else "default"
        session_key = f"{session_id}:{origin_key}" if hierarchy_enabled else origin_key
        existing = roots.get(session_key)
        if existing is not None:
            existing_path = Path(existing)
            try:
                existing_path.resolve(strict=False).relative_to(session_base.resolve(strict=False))
            except ValueError:
                roots.pop(session_key, None)
            else:
                return CaptureRootResolution(existing_path, roots, session_key=session_key, source=origin_key, activate=True)
        session_base.mkdir(parents=True, exist_ok=True)
        stamp = stamp if stamp is not None else time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        candidate = session_base / stamp
        suffix = 1
        while candidate.exists():
            candidate = session_base / f"{stamp}_{suffix:02d}"
            suffix += 1
        if origin_key == "auto":
            candidate = candidate / "calib_capture" / "AutoCapture"
        candidate.mkdir(parents=True, exist_ok=False)
        roots[session_key] = candidate
        return CaptureRootResolution(candidate, roots, session_key=session_key, source=origin_key, activate=True, cache_changed=True, created=True)

    @staticmethod
    def prepare_capture_family(root: Path, capture_family: str) -> Path:
        capture_root = root / "calib_capture" / capture_family
        for side in ("left", "right"):
            (capture_root / side).mkdir(parents=True, exist_ok=True)
        return capture_root
