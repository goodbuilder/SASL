"""Mutable optical capture history, separate from pure observation computation.

This owner validates persisted NativeFrameArtifact instances and retains the
append-only capture observations and latest target revision for each burst.
Its artifact dependency belongs to capture/runtime, never to solver imports.
Native pixels stay u-right/v-down; GNSS collections use the first READY P1
origin and north/east/down metres, with the existing clock evidence gates.
"""

from __future__ import annotations

import copy
import time
import uuid
from typing import Any, Mapping

from Workstation.workstation_core.native_frame_store import NativeFrameArtifact
from Workstation.workstation_core.rtk_geometry import GpsPositionState, ned_delta_m
from .geometry_values import _payload_sha256
from .observations import (
    annotation_points,
    _gps_state_observation_mapping,
    _gps_state_from_capture_mapping,
    _drone_rtk_sync_mapping,
    _drone_rtk_observation_state,
)
from .policy import _DRONE_RTK_MULTIVIEW_MIN_OBSERVATIONS

_DRONE_RTK_OBSERVATION_SCHEMA = "optical-pose-drone-rtk-capture-observation-v1"

_DRONE_RTK_MULTIVIEW_SAMPLE_SCHEMA = "optical-pose-drone-rtk-multiview-sample-v1"

_DRONE_RTK_MULTIVIEW_COLLECTION_SCHEMA = "optical-pose-drone-rtk-multiview-collection-v1"


class OpticalObservationHistory:
    """Append-only capture evidence and latest target revision per burst.

    All inputs name their frozen burst. No live telemetry/UI object is retained.
    Native u/v pixels are bound to WGS84 samples; collection origin remains the
    first READY left/P1 GNSS and target/centres remain north/east/down metres.
    """

    def __init__(self, emit):
        self._append_event = emit
        self._drone_rtk_observations = []
        self._drone_rtk_multiview_samples = {}

    def reset(self):
        self._drone_rtk_observations.clear()
        self._drone_rtk_multiview_samples.clear()

    def capture(
        self,
        *,
        session_id: str,
        burst_id: str,
        frozen: Mapping[str, dict[str, Any]],
        gnss_snapshot: Mapping[str, Any] | None = None,
        gnss_snapshot_source: str = "live_capture_request",
        emit: bool = True,
    ) -> dict[str, Any]:
        """Snapshot Left/Right/Drone GNSS against the just-frozen native burst.

        This is evidence capture only: it never touches candidates, the
        active selection, or any solver. Missing/degraded GNSS is recorded
        as such (state INCOMPLETE/WARNING); it never raises, so a live or FTP
        Horizon/Mountain/Target Capture with no GNSS still succeeds.
        """

        raw_gps = gnss_snapshot
        raw_gps = raw_gps if isinstance(raw_gps, Mapping) else {}
        left_artifact: NativeFrameArtifact = frozen["left"]["artifact"]
        right_artifact: NativeFrameArtifact = frozen["right"]["artifact"]
        left_state = raw_gps.get("left")
        right_state = raw_gps.get("right")
        drone_state = raw_gps.get("drone")
        left_gnss = _gps_state_observation_mapping(left_state, role="left")
        right_gnss = _gps_state_observation_mapping(right_state, role="right")
        drone_gnss = _gps_state_observation_mapping(drone_state, role="drone")
        sync = _drone_rtk_sync_mapping(
            left_artifact=left_artifact,
            right_artifact=right_artifact,
            drone_state=(
                drone_state
                if isinstance(drone_state, GpsPositionState)
                else _gps_state_from_capture_mapping(drone_state)
            ),
        )
        state = _drone_rtk_observation_state(
            left_gnss=left_gnss,
            right_gnss=right_gnss,
            drone_gnss=drone_gnss,
            sync=sync,
        )
        observation = {
            "schema": _DRONE_RTK_OBSERVATION_SCHEMA,
            "observation_id": f"drone-rtk-obs-{uuid.uuid4().hex}",
            "sequence": len(self._drone_rtk_observations),
            "session_id": session_id,
            "burst_id": burst_id,
            "frames": {
                "left": left_artifact.to_mapping(),
                "right": right_artifact.to_mapping(),
            },
            "gnss": {
                "left": left_gnss,
                "right": right_gnss,
                "drone": drone_gnss,
            },
            "gnss_snapshot_source": gnss_snapshot_source,
            "synchronization": sync,
            "state": state,
            "created_utc_ns": time.time_ns(),
        }
        self._drone_rtk_observations.append(observation)
        if emit:
            self._append_event("optical_pose_drone_rtk_observation_captured", copy.deepcopy(observation))
        return copy.deepcopy(observation)

    def for_burst(self, frozen) -> dict[str, Any] | None:
        """Return the Drone-RTK observation captured with the frozen burst.

        Matches on ``burst_id`` (== the frozen left artifact's ``frame_id``)
        so a later live-GPS change or a later Capture can never be silently
        substituted for the GNSS evidence that belonged to *this* frozen
        image. Returns ``None`` for an imported burst (which never appends
        an observation) or an empty history.
        """

        left_frozen = frozen.get("left")
        artifact = left_frozen.get("artifact") if isinstance(left_frozen, Mapping) else None
        burst_id = getattr(artifact, "frame_id", None)
        if not burst_id:
            return None
        for observation in reversed(self._drone_rtk_observations):
            if observation.get("burst_id") == burst_id:
                return copy.deepcopy(observation)
        return None

    def record_sample(
        self,
        review: Mapping[str, Any],
        *,
        frozen,
        profile,
        intrinsics_snapshot,
    ) -> dict[str, Any] | None:
        """Bind one frozen target pair to that burst's captured RTK evidence.

        A partially clicked pair is not a sample. Once both sides contain one
        target pixel, the pair is recorded even when its RTK/synchronization
        evidence is blocked; this keeps the failure auditable while ensuring
        only ``READY`` samples can enter the full-R,t solve.
        """

        if str(review.get("annotation_mode") or "").lower() != "target":
            return None
        annotations = review.get("annotations")
        if not isinstance(annotations, Mapping):
            return None
        normalized: dict[str, dict[str, Any]] = {}
        target_pixels: dict[str, dict[str, float]] = {}
        for side in ("left", "right"):
            annotation = annotations.get(side)
            if not isinstance(annotation, Mapping):
                return None
            points = annotation_points(annotation, "target_point")
            if len(points) != 1:
                return None
            normalized[side] = copy.deepcopy(dict(annotation))
            target_pixels[side] = {"u_px": points[0][0], "v_px": points[0][1]}

        left_frozen = frozen.get("left")
        right_frozen = frozen.get("right")
        left_artifact = left_frozen.get("artifact") if isinstance(left_frozen, Mapping) else None
        right_artifact = right_frozen.get("artifact") if isinstance(right_frozen, Mapping) else None
        if not isinstance(left_artifact, NativeFrameArtifact) or not isinstance(right_artifact, NativeFrameArtifact):
            return None
        burst_id = left_artifact.frame_id
        observation = self.for_burst(frozen)
        reasons: list[str] = []
        frozen_profile = profile
        frozen_profile_sha256 = "" if frozen_profile is None else str(frozen_profile.profile_sha256)
        intrinsics_applicability = intrinsics_snapshot.get("applicability")
        intrinsics_applicability = intrinsics_applicability if isinstance(intrinsics_applicability, Mapping) else {}
        if not bool(intrinsics_applicability.get("geometry_allowed", False)):
            intrinsics_reasons = intrinsics_applicability.get("reasons")
            reason_text = ", ".join(str(value) for value in intrinsics_reasons or ())
            reasons.append(
                "frozen exact operating K,D is not geometry-applicable" + (f": {reason_text}" if reason_text else "")
            )
        if observation is None:
            reasons.append("no capture-associated Drone/P1/P2 GNSS observation")
            observation_id = ""
            gnss: Mapping[str, Any] = {}
            synchronization: Mapping[str, Any] = {}
            capture_state = "INCOMPLETE"
        else:
            observation_id = str(observation.get("observation_id") or "")
            gnss_value = observation.get("gnss")
            gnss = gnss_value if isinstance(gnss_value, Mapping) else {}
            sync_value = observation.get("synchronization")
            synchronization = sync_value if isinstance(sync_value, Mapping) else {}
            capture_state = str(observation.get("state") or "INCOMPLETE")
            if capture_state != "EVIDENCE_READY":
                reasons.append(f"capture evidence state is {capture_state}, not EVIDENCE_READY")
            if synchronization.get("state") != "SYNC_OK":
                reasons.append(str(synchronization.get("reason") or "frame/Drone GNSS synchronization is not verified"))
            frames = observation.get("frames")
            frames = frames if isinstance(frames, Mapping) else {}
            for side, artifact in (("left", left_artifact), ("right", right_artifact)):
                frame = frames.get(side)
                if not isinstance(frame, Mapping) or str(frame.get("frame_id") or "") != artifact.frame_id:
                    reasons.append(f"{side} frame identity does not match capture observation")

        previous = self._drone_rtk_multiview_samples.get(burst_id)
        sample_id = (
            str(previous.get("sample_id") or "") if isinstance(previous, Mapping) else ""
        ) or f"drone-rtk-multiview-sample-{uuid.uuid4().hex}"
        revision = int(previous.get("revision", 0)) + 1 if isinstance(previous, Mapping) else 1
        sequence = (
            int(previous.get("sequence", 0))
            if isinstance(previous, Mapping)
            else len(self._drone_rtk_multiview_samples)
        )
        sample = {
            "schema": _DRONE_RTK_MULTIVIEW_SAMPLE_SCHEMA,
            "sample_id": sample_id,
            "sequence": sequence,
            "revision": revision,
            "session_id": str(observation.get("session_id") or "") if observation else "",
            "burst_id": burst_id,
            "capture_observation_id": observation_id,
            "capture_state": capture_state,
            "operating_intrinsics_profile_sha256": frozen_profile_sha256,
            "capture_zoom": {side: copy.deepcopy(dict(frozen[side].get("zoom") or {})) for side in ("left", "right")},
            "state": "READY" if not reasons else "BLOCKED",
            "reasons": list(dict.fromkeys(reasons)),
            "frames": {
                "left": left_artifact.to_mapping(),
                "right": right_artifact.to_mapping(),
            },
            "annotations": normalized,
            "target_pixels": target_pixels,
            "annotation_hashes": {side: _payload_sha256(normalized[side]) for side in ("left", "right")},
            "gnss": copy.deepcopy(dict(gnss)),
            "synchronization": copy.deepcopy(dict(synchronization)),
            "coordinate_contract": {
                "pixel": "native OpenCV pixels; u right, v down",
                "gnss": "WGS84 latitude/longitude degrees and altitude metres",
                "solve_world": "local NED metres; origin selected when collection is solved",
            },
            "updated_utc_ns": time.time_ns(),
        }
        self._drone_rtk_multiview_samples[burst_id] = sample
        self._append_event("optical_pose_drone_rtk_multiview_sample_recorded", copy.deepcopy(sample))
        return copy.deepcopy(sample)

    def collection(self, *, profile) -> dict[str, Any]:
        """Build the existing solver contract from READY in-session samples."""

        samples = sorted(
            (
                copy.deepcopy(sample)
                for sample in self._drone_rtk_multiview_samples.values()
                if sample.get("state") == "READY"
            ),
            key=lambda sample: int(sample.get("sequence", 0)),
        )
        if len(samples) < _DRONE_RTK_MULTIVIEW_MIN_OBSERVATIONS:
            blocked_count = sum(sample.get("state") != "READY" for sample in self._drone_rtk_multiview_samples.values())
            raise ValueError(
                "Drone RTK Multi-view requires at least four READY accumulated Frozen target "
                f"observations; ready={len(samples)}, blocked={blocked_count}"
            )

        profile_hashes = {str(sample.get("operating_intrinsics_profile_sha256") or "") for sample in samples}
        if "" in profile_hashes or len(profile_hashes) != 1:
            raise ValueError("READY Multi-view samples must share one exact operating K,D profile")
        current_profile_hash = "" if profile is None else str(profile.profile_sha256)
        if current_profile_hash not in profile_hashes:
            raise ValueError("accumulated Multi-view K,D profile does not match the current frozen burst")

        first_gnss = samples[0].get("gnss")
        first_gnss = first_gnss if isinstance(first_gnss, Mapping) else {}
        origin = _gps_state_from_capture_mapping(first_gnss.get("left"))
        first_right = _gps_state_from_capture_mapping(first_gnss.get("right"))
        if origin is None or first_right is None:
            raise ValueError("first READY Multi-view sample lacks a usable P1/P2 GNSS origin")
        right_center = ned_delta_m(origin, first_right)
        if right_center is None:
            raise ValueError("first READY Multi-view sample cannot derive the P2-P1 NED baseline")

        observations: dict[str, list[dict[str, Any]]] = {"left": [], "right": []}
        for sample in samples:
            gnss = sample.get("gnss")
            gnss = gnss if isinstance(gnss, Mapping) else {}
            drone_row = gnss.get("drone")
            drone = _gps_state_from_capture_mapping(drone_row)
            if drone is None:
                raise ValueError(f"READY Multi-view sample {sample.get('sample_id')} lacks usable Drone GNSS")
            target_ned = ned_delta_m(origin, drone)
            if target_ned is None:
                raise ValueError(f"READY Multi-view sample {sample.get('sample_id')} cannot derive target NED")
            pixels = sample.get("target_pixels")
            pixels = pixels if isinstance(pixels, Mapping) else {}
            frames = sample.get("frames")
            frames = frames if isinstance(frames, Mapping) else {}
            for side in ("left", "right"):
                pixel = pixels.get(side)
                frame = frames.get(side)
                if not isinstance(pixel, Mapping) or not isinstance(frame, Mapping):
                    raise ValueError(
                        f"READY Multi-view sample {sample.get('sample_id')} lacks {side} pixel/frame evidence"
                    )
                observations[side].append(
                    {
                        "observation_id": str(sample.get("sample_id") or ""),
                        "capture_observation_id": str(sample.get("capture_observation_id") or ""),
                        "burst_id": str(sample.get("burst_id") or ""),
                        "frame_id": str(frame.get("frame_id") or ""),
                        "image_sha256": str(frame.get("sha256") or ""),
                        "pixel_uv": [float(pixel["u_px"]), float(pixel["v_px"])],
                        "target_point_ned_m": [float(value) for value in target_ned],
                        "drone_gps": copy.deepcopy(dict(drone_row)),
                        "timestamp_alignment_verified": True,
                    }
                )

        collection_evidence = {
            "sample_ids": [str(sample.get("sample_id") or "") for sample in samples],
            "capture_observation_ids": [str(sample.get("capture_observation_id") or "") for sample in samples],
            "annotation_hashes": [sample.get("annotation_hashes") for sample in samples],
            "operating_intrinsics_profile_sha256": next(iter(profile_hashes)),
        }
        return {
            "schema": _DRONE_RTK_MULTIVIEW_COLLECTION_SCHEMA,
            "source": "session_accumulated_frozen_targets",
            "origin_gps": copy.deepcopy(dict(first_gnss["left"])),
            "camera_centers_ned_m": {
                "left": [0.0, 0.0, 0.0],
                "right": [float(value) for value in right_center],
            },
            "observations": observations,
            "collection_sample_ids": collection_evidence["sample_ids"],
            "collection_sha256": _payload_sha256(collection_evidence),
            "coordinate_contract": {
                "origin": "P1/left GNSS from the first READY accumulated sample",
                "world": "local NED metres: north, east, down",
                "target": "Drone GNSS minus origin converted to local NED metres",
                "ray_input": "native OpenCV pixel (u right, v down), undistorted with frozen K,D",
                "camera_center_initialization": "P1=(0,0,0), P2=P2_GNSS-P1_GNSS; optical centers are solved",
            },
        }
