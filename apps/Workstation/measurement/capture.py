"""Pair admission, retained telemetry, native reservations and frozen evidence.

Attitudes: body FRD -> earth NED, wxyz. GPS baselines: NED metres.
Optical R,t: X_right_RDF = R_right_from_left X_left_RDF + t_m.
Public point_m: baseline-center FRD; source_point_left_rdf_m: left optical RDF.
"""

from __future__ import annotations
import math
import time
from collections.abc import Mapping
from dataclasses import asdict, dataclass, replace
from PyQt5 import QtCore
from Workstation.workstation_core.gnss_operation import ReadinessConfig
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    DetectionSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
    detection_pair_key,
    live_pair_timing_evidence,
)
from Workstation.workstation_core.method_evaluation import FrozenMeasurementPairSnapshot
from Workstation.workstation_core.detection_pair_coordinator import (
    DetectionPairCoordinator,
    PairTimingDecision,
    PairingPolicy,
)
from Workstation.workstation_core.native_evidence_writer import (
    RingPairReservation,
    reserve_ring_pair,
)
from Workstation.workstation_core.native_frame_ring import FrameKey
from Workstation.workstation_core.preview_measurement_worker import (
    PreviewMeasurementCoordinator,
    PreviewMeasurementResult,
)
from Workstation.workstation_core.settings import normalize_live_distance_time_basis
from Workstation.workstation_core.stereo import (
    DetectionObservation,
    TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC,
    detection_timestamp_for_basis,
)
from typing import Any, Callable
from Workstation.measurement import limits
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.frozen_inputs import FrozenPairInputs
from Workstation.measurement.state import (
    UNSET,
    read,
    PairCaptureState,
    ResultCommitState,
    MeasurementPresentationState,
    WorkerRuntimeState,
)


@dataclass(frozen=True)
class PairCaptureInputs:
    camera_pane: Callable[..., Any]
    qt_parent: ValueInput
    yolo_gate_time_basis_combo: ValueInput
    settings: ValueInput
    _workstation_setting: ValueInput
    _measurement_id: ValueInput
    camera_sync_status: ValueInput
    frame_timing_status: ValueInput
    _yolo_pair_max_delta_s: ValueInput
    _yolo_vertical_max_delta_px: ValueInput
    optical_target_distance_spin_value: Callable[..., Any]
    _decorate_clock_epoch_metadata: ValueInput
    latest_left: ValueInput
    latest_right: ValueInput
    detection_buffers: ValueInput
    gnss_operation_manager: ValueInput
    gnss_fixed_base_profile: ValueInput
    latest_rtcm_reference_station: ValueInput
    latest_extrinsics: ValueInput
    _measurement_started_monotonic_s: ValueInput
    _selected_baseline_gps_states: ValueInput
    _pixhawk_gnss_source: ValueInput
    attitude_buffers: ValueInput
    _persist_native_detection_pair: ValueInput


@dataclass(frozen=True)
class PairCaptureActions:
    freeze_geometry_inputs: Callable[..., Any]
    _measurement_method_state: Callable[..., Any]
    panel__refresh_distance: ValueInput
    panel__refresh_optical_alignment_view: ValueInput
    panel__optical_alignment_expected_dy_px: ValueInput
    panel__optical_alignment_mean_focal_px: ValueInput
    _measurement_registry: Callable[..., Any]


class PairCaptureService:
    """Pair admission, retained telemetry, native reservations and frozen evidence."""

    def __init__(
        self,
        *,
        state: PairCaptureState,
        inputs: PairCaptureInputs,
        actions: PairCaptureActions,
        commit_state: ResultCommitState,
        presentation_state: MeasurementPresentationState,
        runtime_state: WorkerRuntimeState,
    ) -> None:
        self.state = state
        self.inputs = inputs
        self.actions = actions
        self.commit_state = commit_state
        self.presentation_state = presentation_state
        self.runtime_state = runtime_state

    @staticmethod
    def _release_detection_pair_reservation(reservation: object | None) -> None:
        release = getattr(reservation, "release", None)
        if callable(release):
            release()

    def _exact_pair_reservation_supported(self) -> bool:
        return all(
            (
                callable(
                    getattr(
                        self.inputs.camera_pane(side), "native_ring_for_frame_key", None
                    )
                )
                for side in ("left", "right")
            )
        )

    @staticmethod
    def _reserve_detection_pair(
        pair: object, sources: Mapping[str, object | None]
    ) -> RingPairReservation | None:
        frames = []
        for side, observation in (("left", pair.left), ("right", pair.right)):
            metadata = getattr(observation, "timestamp_metadata", {})
            if not isinstance(metadata, Mapping):
                raise ValueError(f"{side} frame metadata is unavailable")
            key = FrameKey.from_mapping(metadata)
            if key.camera_id != side or key.stream_generation <= 0 or key.frame_id <= 0:
                raise ValueError(f"{side} exact FrameKey is invalid")
            ring = sources.get(side)
            if ring is None:
                raise ValueError(f"{side} exact native ring is unavailable")
            frames.append((side, ring, key, metadata))
        return reserve_ring_pair(frames)

    def _ensure_detection_pair_coordinator(self) -> DetectionPairCoordinator:
        coordinator = self.state._detection_pair_coordinator
        if coordinator is None:
            coordinator = DetectionPairCoordinator(
                reservation_factory=(
                    self._reserve_detection_pair
                    if self._exact_pair_reservation_supported()
                    else None
                )
            )
            self.state._detection_pair_coordinator = coordinator
        if (
            self.state._detection_pair_timer is None
            and QtCore.QCoreApplication.instance() is not None
        ):
            parent = (
                self.inputs.qt_parent.get(None)
                if self.inputs.qt_parent.get(None) is not None
                else None
            )
            timer = QtCore.QTimer(parent)
            timer.setInterval(20)
            timer.timeout.connect(self._poll_detection_pairs)
            timer.start()
            self.state._detection_pair_timer = timer
        return coordinator

    def _configured_time_basis(self) -> str:
        combo = self.inputs.yolo_gate_time_basis_combo.get(None)
        current_data = getattr(combo, "currentData", None)
        if callable(current_data):
            value = current_data()
            if value is not None:
                return normalize_live_distance_time_basis(value)
        settings = self.inputs.settings.get(None)
        default = getattr(
            settings, "yolo_gate_time_basis", TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC
        )
        setting_reader = self.inputs._workstation_setting.get(None)
        value = (
            setting_reader("yolo_gate_time_basis", default)
            if callable(setting_reader)
            else default
        )
        return normalize_live_distance_time_basis(value)

    def requested_time_basis(self) -> str:
        """Return the current Measurement-frozen request or the idle setting."""
        measurement_id = str(self.inputs._measurement_id.get("") or "")
        if not measurement_id:
            return self._configured_time_basis()
        self.actions._measurement_method_state()
        return normalize_live_distance_time_basis(
            read(
                self.state._measurement_active_time_basis_requested,
                self._configured_time_basis(),
            )
        )

    def _rtcp_pairing_health(self, requested_basis: str) -> tuple[bool | None, str]:
        if requested_basis != "camera_rtcp":
            return (None, "")
        if str(self.inputs.camera_sync_status.get("")) != "verified":
            return (False, "CLOCK_NOT_VERIFIED")
        if str(self.inputs.frame_timing_status.get("")) != "current":
            return (False, "FRAME_TIMING_NOT_CURRENT")
        return (True, "")

    def _pairing_policy_snapshot(self) -> PairingPolicy:
        configured_delta_s = float(self.inputs._yolo_pair_max_delta_s.get()())
        app_delta_limit_s = (
            min(limits.LIVE_DISTANCE_MAX_PAIR_DELTA_S, configured_delta_s)
            if math.isfinite(configured_delta_s)
            else limits.LIVE_DISTANCE_MAX_PAIR_DELTA_S
        )
        requested_basis = self.requested_time_basis()
        rtcp_verified, rtcp_reason = self._rtcp_pairing_health(requested_basis)
        measurement_generation = str(self.inputs._measurement_id.get("") or "idle")
        token = (
            measurement_generation,
            requested_basis,
            rtcp_verified,
            rtcp_reason,
            app_delta_limit_s,
            float(self.inputs._yolo_vertical_max_delta_px.get()()),
        )
        if token != self.state._pairing_policy_token:
            self.state._pairing_policy_token = token
            self.state._pairing_policy_revision += 1
            previous_reservation = read(
                self.state._coordinated_detection_pair_reservation, None
            )
            self._release_detection_pair_reservation(previous_reservation)
            self.state._coordinated_detection_pair_reservation = None
            self.state._coordinated_detection_pair = None
            self.state._coordinated_pairing_result = None
            self.state._last_frozen_measurement_pair = None
        return PairingPolicy(
            timestamp_basis=requested_basis,
            max_delta_s=app_delta_limit_s,
            max_vertical_delta_px=float(
                self.inputs._yolo_vertical_max_delta_px.get()()
            ),
            allow_app_fallback=requested_basis == "camera_rtcp",
            rtcp_clock_verified=rtcp_verified,
            rtcp_unavailable_reason=rtcp_reason,
            require_current_clock_epoch=requested_basis == "camera_rtcp",
            measurement_generation=measurement_generation,
            max_observation_age_s=limits.DETECTION_PAIR_MAX_OBSERVATION_AGE_S,
            revision=self.state._pairing_policy_revision,
        )

    def _poll_detection_pairs(self) -> None:
        coordinator = self.state._detection_pair_coordinator
        if coordinator is None:
            return
        result = coordinator.poll_latest()
        if result is None:
            return
        current_policy = self._pairing_policy_snapshot()
        if result.policy_revision != current_policy.revision:
            self._release_detection_pair_reservation(result.reservation)
            self.state._coordinated_detection_pair = None
            self.state._coordinated_pairing_result = None
            return
        previous_reservation = read(
            self.state._coordinated_detection_pair_reservation, None
        )
        if previous_reservation is not result.reservation:
            self._release_detection_pair_reservation(previous_reservation)
        self.state._coordinated_detection_pair_reservation = result.reservation
        self.state._detection_pair_coordinator_active = True
        self.state._coordinated_detection_pair = result.pair
        self.state._coordinated_pairing_result = result
        self.state._last_correspondence_rejection_reason = result.rejection_reason
        self.state._last_pair_advisories = result.advisories
        self.state._runtime_pair_uses_app_fallback = result.uses_app_fallback
        self.state._detection_pair_metrics = coordinator.metrics()
        try:
            self.actions.panel__refresh_distance.get(None)()
            self.actions.panel__refresh_optical_alignment_view.get(None)()
        finally:
            current_reservation = read(
                self.state._coordinated_detection_pair_reservation, None
            )
            if current_reservation is result.reservation:
                self._release_detection_pair_reservation(current_reservation)
                self.state._coordinated_detection_pair_reservation = None

    def reset_detection_pair_coordinator(self) -> None:
        coordinator = self.state._detection_pair_coordinator
        self.state._detection_pair_coordinator = None
        if coordinator is not None:
            coordinator.close()
        self.state._detection_pair_coordinator_active = False
        self.state._coordinated_detection_pair = None
        self.state._coordinated_pairing_result = None
        self._release_detection_pair_reservation(
            read(self.state._coordinated_detection_pair_reservation, None)
        )
        self.state._coordinated_detection_pair_reservation = None
        self.state._detection_pair_metrics = {}
        preview_coordinator = self.state._preview_measurement_coordinator
        self.state._preview_measurement_coordinator = None
        if preview_coordinator is not None:
            preview_coordinator.close()
        self.state._preview_measurement_results.clear()
        self.state._preview_measurement_pending.clear()
        self.state._latest_preview_measurement_result = None
        self.state._latest_preview_pair_identity = None

    def begin_pairing_generation(self) -> None:
        """Retire every pre-Measurement candidate/result before freezing settings."""
        self.state._pairing_policy_token = None
        self.commit_state._primary_outcome_hash_by_key.clear()
        self.commit_state._primary_normalized_outcome_by_key.clear()
        self.commit_state._primary_measurement_persisted.clear()
        self.commit_state._primary_history_projected.clear()
        self.presentation_state._primary_display_high_water = -1
        self.presentation_state._primary_displayed_key = None
        self.runtime_state._measurement_recording_method_id = ""
        self.reset_detection_pair_coordinator()
        self.state._last_pair_advisories = ()
        self.state._last_correspondence_rejection_reason = ""
        self.state._runtime_pair_uses_app_fallback = False

    def _ensure_preview_measurement_coordinator(self) -> PreviewMeasurementCoordinator:
        coordinator = self.state._preview_measurement_coordinator
        if coordinator is None:
            coordinator = PreviewMeasurementCoordinator()
            self.state._preview_measurement_coordinator = coordinator
        if (
            self.state._preview_measurement_timer is None
            and QtCore.QCoreApplication.instance() is not None
        ):
            parent = (
                self.inputs.qt_parent.get(None)
                if self.inputs.qt_parent.get(None) is not None
                else None
            )
            timer = QtCore.QTimer(parent)
            timer.setInterval(20)
            timer.timeout.connect(self._poll_preview_measurements)
            timer.start()
            self.state._preview_measurement_timer = timer
        return coordinator

    def _poll_preview_measurements(self) -> None:
        coordinator = self.state._preview_measurement_coordinator
        if coordinator is None:
            return
        result = coordinator.poll_latest()
        if result is None:
            return
        self.state._preview_measurement_pending.discard(result.key)
        self.state._preview_measurement_results[result.key] = result
        while len(self.state._preview_measurement_results) > 8:
            self.state._preview_measurement_results.pop(
                next(iter(self.state._preview_measurement_results))
            )
        self.state._preview_measurement_metrics = coordinator.metrics()
        self.state._latest_preview_measurement_result = result
        if result.attempt is not None:
            self.state._latest_preview_pair_identity = (
                str(
                    result.attempt.left_detection.timestamp_metadata.get(
                        "frame_key",
                        result.attempt.left_detection.app_receive_monotonic_s,
                    )
                ),
                str(
                    result.attempt.right_detection.timestamp_metadata.get(
                        "frame_key",
                        result.attempt.right_detection.app_receive_monotonic_s,
                    )
                ),
            )
        self.actions.panel__refresh_distance.get(None)()
        self.actions.panel__refresh_optical_alignment_view.get(None)()

    def _preview_optical_payload(self, pair: object) -> dict[str, object]:
        expected_dy_px, _expected_source, _expected_level = (
            self.actions.panel__optical_alignment_expected_dy_px.get(None)()
        )
        mean_fy_px, _fy_source = (
            self.actions.panel__optical_alignment_mean_focal_px.get(None)("y")
        )
        mean_fx_px, _fx_source = (
            self.actions.panel__optical_alignment_mean_focal_px.get(None)("x")
        )
        timing = self._pair_timing_decision(pair)
        return {
            "left_center": tuple(pair.left.center),
            "right_center": tuple(pair.right.center),
            "expected_dy_px": expected_dy_px,
            "mean_fy_px": mean_fy_px,
            "mean_fx_px": mean_fx_px,
            "target_distance_m": self.inputs.optical_target_distance_spin_value(),
            "left_pair_timestamp_s": timing.left_timestamp_s,
            "right_pair_timestamp_s": timing.right_timestamp_s,
        }

    def _preview_measurement_result(
        self, attempt: MeasurementAttempt, pair: object
    ) -> PreviewMeasurementResult | None:
        key = attempt.canonical_sha256()
        cached = self.state._preview_measurement_results.get(key)
        if cached is not None:
            return cached
        if key not in self.state._preview_measurement_pending:
            self.state._preview_measurement_pending.add(key)
            self._ensure_preview_measurement_coordinator().submit(
                key=key,
                attempt=attempt,
                optical_payload=self._preview_optical_payload(pair),
            )
        return None

    def _update_detection(
        self, camera: str, detection: DetectionObservation | None
    ) -> None:
        if detection is not None:
            decorate = self.inputs._decorate_clock_epoch_metadata.get(None)
            if callable(decorate):
                detection = replace(
                    detection,
                    timestamp_metadata=decorate(
                        camera, dict(detection.timestamp_metadata)
                    ),
                )
        if camera == "left":
            self.inputs.latest_left.set(detection)
        elif camera == "right":
            self.inputs.latest_right.set(detection)
        if detection is not None:
            self.inputs.detection_buffers.get()[camera].add(detection)
            registry = self.actions._measurement_registry()
            if registry is not None:
                registry.observe_detection(camera, detection)
        coordinator = self._ensure_detection_pair_coordinator()
        reservation_source = None
        if detection is not None and self._exact_pair_reservation_supported():
            try:
                frame_key = FrameKey.from_mapping(detection.timestamp_metadata)
            except (TypeError, ValueError):
                frame_key = None
            pane = self.inputs.camera_pane(camera)
            resolver = getattr(pane, "native_ring_for_frame_key", None)
            if frame_key is not None and callable(resolver):
                reservation_source = resolver(frame_key)
        coordinator.submit(
            camera,
            detection,
            self._pairing_policy_snapshot(),
            reservation_source=reservation_source,
        )

    def _gnss_readiness_config(self) -> object:
        manager = self.inputs.gnss_operation_manager.get(None)
        config = getattr(manager, "config", None)
        if config is None:
            config = getattr(self.inputs.settings.get(None), "gnss_operation", None)
        return getattr(config, "readiness", None)

    def _measurement_correction_frame_token(self) -> str:
        """Return the identity of the frame the retained rover positions resolve in.

        P1 and P2 can only be connected one at a time, so their retained
        positions are captured minutes apart.  Two RTK fixes are comparable as
        one P1->P2 baseline only while the correction source and the Base
        coordinate they resolve against stay the same; a re-survey, a
        mountpoint/caster change, or an operation-mode change silently moves
        the frame.  Binding the retained pair to this token drops it instead.
        """
        manager = self.inputs.gnss_operation_manager.get(None)
        identity = getattr(manager, "source_identity", None)
        source_id = (
            "" if identity is None else str(getattr(identity, "stable_id", "") or "")
        )
        profile = self.inputs.gnss_fixed_base_profile.get(None)
        base_hash = (
            "" if profile is None else str(getattr(profile, "profile_hash", "") or "")
        )
        reference = self.inputs.latest_rtcm_reference_station.get(None)
        station = ""
        if isinstance(reference, Mapping):
            ecef = reference.get("ecef_01mm")
            try:
                coordinates = ",".join((str(int(value)) for value in ecef or ()))
            except (TypeError, ValueError):
                coordinates = ""
            if coordinates:
                station = f"{reference.get('station_id')}:{coordinates}"
        return f"{source_id}\x00{base_hash}\x00{station}"

    @staticmethod
    def _attitude_state_is_valid_for_measurement_cache(state: object) -> bool:
        if state is None:
            return False
        try:
            timestamp_s = float(getattr(state, "timestamp_s", float("nan")))
        except (TypeError, ValueError):
            return False
        if not math.isfinite(timestamp_s):
            return False
        try:
            quaternion = tuple(
                (float(value) for value in getattr(state, "q", ()) or ())
            )
        except (TypeError, ValueError):
            return False
        if len(quaternion) != 4 or not all(
            (math.isfinite(value) for value in quaternion)
        ):
            return False
        return any((value != 0.0 for value in quaternion))

    def _cached_measurement_attitude_states(
        self,
    ) -> tuple[object | None, object | None, tuple[str, ...]]:
        """Update each P1/P2 side independently and retain last valid attitudes.

        This is the attitude twin of `_cached_measurement_gps_states`.  Both
        sensor halves of the live Pixhawk method therefore reuse their own last
        valid process-local sample instead of one half being retained while the
        other is re-read live and blocks on freshness.
        """
        cache = read(self.state._measurement_latest_valid_attitude, None)
        if not isinstance(cache, dict):
            cache = {"left": None, "right": None}
            self.state._measurement_latest_valid_attitude = cache
        live = self.inputs.latest_extrinsics.get({}) or {}
        measurement_start = self.inputs._measurement_started_monotonic_s.get(None)
        reused: list[str] = []
        for side in ("left", "right"):
            sample = live.get(side) if isinstance(live, Mapping) else None
            if self._attitude_state_is_valid_for_measurement_cache(sample):
                previous = cache.get(side)
                previous_timestamp = float(
                    getattr(previous, "timestamp_s", -1.0) or -1.0
                )
                sample_timestamp = float(getattr(sample, "timestamp_s", -1.0) or -1.0)
                if previous is None or sample_timestamp >= previous_timestamp:
                    cache[side] = sample
            selected = cache.get(side)
            selected_timestamp = getattr(selected, "timestamp_s", None)
            reused_from_before_measurement = bool(
                selected is not None
                and measurement_start is not None
                and (selected_timestamp is not None)
                and (float(selected_timestamp) < float(measurement_start))
            )
            if selected is not None and (
                selected is not sample or reused_from_before_measurement
            ):
                reused.append(f"{side}_attitude")
        return (cache.get("left"), cache.get("right"), tuple(reused))

    def _gps_state_is_valid_for_measurement_cache(self, state: object) -> bool:
        defaults = ReadinessConfig()
        readiness = self._gnss_readiness_config()
        minimum_fix_type = int(
            getattr(readiness, "minimum_fix_type", defaults.minimum_fix_type)
        )
        if state is None or int(getattr(state, "fix_type", 0) or 0) < minimum_fix_type:
            return False
        values = (
            getattr(state, "latitude_deg", None),
            getattr(state, "longitude_deg", None),
            getattr(state, "altitude_m", None),
        )
        try:
            return all(
                (value is not None and math.isfinite(float(value)) for value in values)
            )
        except (TypeError, ValueError):
            return False

    def _cached_measurement_gps_states(
        self,
    ) -> tuple[object | None, object | None, tuple[str, ...]]:
        """Update each P1/P2 side independently and retain last valid 3D fixes."""
        cache = read(self.state._measurement_latest_valid_gps, None)
        if not isinstance(cache, dict):
            cache = {"left": None, "right": None}
            self.state._measurement_latest_valid_gps = cache
        source_cache = read(self.state._measurement_latest_valid_gps_source, None)
        if not isinstance(source_cache, dict):
            source_cache = {}
            self.state._measurement_latest_valid_gps_source = source_cache
        frame_cache = read(
            self.state._measurement_latest_valid_gps_correction_frame, None
        )
        if not isinstance(frame_cache, dict):
            frame_cache = {}
            self.state._measurement_latest_valid_gps_correction_frame = frame_cache
        frame_barrier = read(
            self.state._measurement_gps_correction_frame_barrier_s, None
        )
        if not isinstance(frame_barrier, dict):
            frame_barrier = {}
            self.state._measurement_gps_correction_frame_barrier_s = frame_barrier
        correction_frame = self._measurement_correction_frame_token()
        current_left, current_right = self.inputs._selected_baseline_gps_states.get()()
        current = {"left": current_left, "right": current_right}
        measurement_start = self.inputs._measurement_started_monotonic_s.get(None)
        reused: list[str] = []
        for side in ("left", "right"):
            source_reader = self.inputs._pixhawk_gnss_source.get(None)
            source = (
                str(source_reader(side))
                if callable(source_reader)
                else (
                    "GPS2"
                    if str(getattr(current[side], "topic", "")).endswith("GPS2_RAW")
                    else "GPS1"
                )
            )
            if source_cache.get(side) != source:
                cache[side] = None
                source_cache[side] = source
            previous_frame = frame_cache.get(side)
            if previous_frame != correction_frame:
                frame_cache[side] = correction_frame
                if previous_frame is not None:
                    cache[side] = None
                    frame_barrier[side] = time.monotonic()
            sample = current[side]
            barrier_s = frame_barrier.get(side)
            admissible = self._gps_state_is_valid_for_measurement_cache(sample)
            if admissible and barrier_s is not None:
                try:
                    admissible = float(sample.timestamp_s) >= float(barrier_s)
                except (AttributeError, TypeError, ValueError):
                    admissible = False
            if admissible:
                previous = cache.get(side)
                previous_timestamp = float(
                    getattr(previous, "timestamp_s", -1.0) or -1.0
                )
                sample_timestamp = float(getattr(sample, "timestamp_s", -1.0) or -1.0)
                if previous is None or sample_timestamp >= previous_timestamp:
                    cache[side] = sample
            selected = cache.get(side)
            selected_timestamp = getattr(selected, "timestamp_s", None)
            reused_from_before_measurement = bool(
                selected is not None
                and measurement_start is not None
                and (selected_timestamp is not None)
                and (float(selected_timestamp) < float(measurement_start))
            )
            if selected is not None and (
                selected is not sample or reused_from_before_measurement
            ):
                reused.append(f"{side}_gps")
        self.state._measurement_gps_correction_frame_dropped = tuple(
            (
                side
                for side in ("left", "right")
                if cache.get(side) is None and frame_barrier.get(side) is not None
            )
        )
        return (cache.get("left"), cache.get("right"), tuple(reused))

    @staticmethod
    def _ftp_command_round_sync(pair: object) -> dict[str, object]:
        left_metadata = (
            getattr(getattr(pair, "left", None), "timestamp_metadata", {}) or {}
        )
        right_metadata = (
            getattr(getattr(pair, "right", None), "timestamp_metadata", {}) or {}
        )
        if not isinstance(left_metadata, Mapping) or not isinstance(
            right_metadata, Mapping
        ):
            return {}
        left_capture = str(left_metadata.get("capture_id") or "")
        right_capture = str(right_metadata.get("capture_id") or "")
        left_round = str(
            left_metadata.get("command_round")
            or left_metadata.get("capture_round")
            or ""
        )
        right_round = str(
            right_metadata.get("command_round")
            or right_metadata.get("capture_round")
            or ""
        )

        def capture_ready(metadata: Mapping[str, object]) -> bool:
            downloaded = bool(
                metadata.get("ftp_downloaded", metadata.get("downloaded", False))
            )
            return bool(
                metadata.get("capture_command_sent", metadata.get("command_sent", True))
                and metadata.get("stored", downloaded)
                and downloaded
                and metadata.get("decoded", False)
            )

        left_ready = capture_ready(left_metadata)
        right_ready = capture_ready(right_metadata)
        if not (
            left_capture
            and left_capture == right_capture
            and left_round
            and (left_round == right_round)
            and left_ready
            and right_ready
        ):
            return {}
        return {
            "capture_id": left_capture,
            "command_round": left_round,
            "time_basis": "command_round_assumed_sync",
            "pair_delta_s": None,
            "operator_approved_sync_assumption": True,
            "warning": "FTP command-round simultaneity is operator-approved; it is not a measured exposure timestamp",
        }

    def _pair_timing_decision(self, pair: object) -> PairTimingDecision:
        result = read(self.state._coordinated_pairing_result, None)
        if result is not None and getattr(result, "pair", None) is pair:
            timing = getattr(result, "timing", None)
            if isinstance(timing, PairTimingDecision):
                return timing
        if bool(read(self.state._detection_pair_coordinator_active, False)):
            raise ValueError(
                "matched detection pair has no coordinated timing decision"
            )
        requested_basis = self.requested_time_basis()
        effective_basis = requested_basis
        fallback_reason = ""
        left_timestamp_s = detection_timestamp_for_basis(pair.left, requested_basis)
        right_timestamp_s = detection_timestamp_for_basis(pair.right, requested_basis)
        if requested_basis == "camera_rtcp" and (
            left_timestamp_s is None or right_timestamp_s is None
        ):
            effective_basis = TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC
            fallback_reason = "RTCP_TIMESTAMP_UNAVAILABLE"
            left_timestamp_s = detection_timestamp_for_basis(pair.left, effective_basis)
            right_timestamp_s = detection_timestamp_for_basis(
                pair.right, effective_basis
            )
        if left_timestamp_s is None or right_timestamp_s is None:
            raise ValueError(
                "matched detection pair has no timestamp on its selected basis"
            )
        if effective_basis == TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC:
            left_ns = pair.left.app_receive_monotonic_ns
            right_ns = pair.right.app_receive_monotonic_ns
            signed_delta_ns = right_ns - left_ns
            left_timestamp_s = left_ns * 1e-09
            right_timestamp_s = right_ns * 1e-09
        else:
            signed_delta_ns = int(
                round(
                    (float(right_timestamp_s) - float(left_timestamp_s)) * 1000000000.0
                )
            )
        return PairTimingDecision(
            requested_basis=requested_basis,
            effective_basis=effective_basis,
            left_timestamp_s=float(left_timestamp_s),
            right_timestamp_s=float(right_timestamp_s),
            signed_delta_ns=signed_delta_ns,
            abs_delta_ns=abs(signed_delta_ns),
            fallback_reason=fallback_reason,
        )

    def _freeze_measurement_pair_inputs(self, pair: object) -> FrozenPairInputs:
        """Persist native pixels, then freeze one shared, byte-free pair snapshot."""
        timing = self._pair_timing_decision(pair)
        requested_timestamp_basis = timing.requested_basis
        timestamp_basis = timing.effective_basis
        left_pairing_timestamp_s = timing.left_timestamp_s
        right_pairing_timestamp_s = timing.right_timestamp_s
        physical_key = detection_pair_key(
            triangulation_source="all_methods",
            timestamp_basis=timestamp_basis,
            left=pair.left,
            right=pair.right,
            left_pairing_timestamp_s=left_pairing_timestamp_s,
            right_pairing_timestamp_s=right_pairing_timestamp_s,
        )
        cached = read(self.state._last_frozen_measurement_pair, None)
        if isinstance(cached, Mapping) and tuple(
            cached.get("physical_key", ())
        ) == tuple(physical_key):
            reservation = read(self.state._coordinated_detection_pair_reservation, None)
            self._release_detection_pair_reservation(reservation)
            self.state._coordinated_detection_pair_reservation = None
            return cached
        created_monotonic_ns = time.monotonic_ns()
        created_unix_ns = time.time_ns()
        detection_midpoint_s = (
            float(pair.left.timestamp_s) + float(pair.right.timestamp_s)
        ) / 2.0
        left_detection = DetectionSnapshot.from_observation(
            pair.left,
            pairing_timestamp_basis=timestamp_basis,
            pairing_timestamp_s=left_pairing_timestamp_s,
        )
        right_detection = DetectionSnapshot.from_observation(
            pair.right,
            pairing_timestamp_basis=timestamp_basis,
            pairing_timestamp_s=right_pairing_timestamp_s,
        )
        left_nearest = self.inputs.attitude_buffers.get()["left"].nearest(
            float(pair.left.timestamp_s)
        )
        right_nearest = self.inputs.attitude_buffers.get()["right"].nearest(
            float(pair.right.timestamp_s)
        )
        measurement_start = self.inputs._measurement_started_monotonic_s.get(None)
        retained_left_attitude, retained_right_attitude, reused_attitude_inputs = (
            self._cached_measurement_attitude_states()
        )
        left_attitude_state = (
            retained_left_attitude if left_nearest is None else left_nearest.sample
        )
        right_attitude_state = (
            retained_right_attitude if right_nearest is None else right_nearest.sample
        )
        left_attitude = (
            None
            if left_attitude_state is None
            else AttitudeSnapshot.from_state(left_attitude_state)
        )
        right_attitude = (
            None
            if right_attitude_state is None
            else AttitudeSnapshot.from_state(right_attitude_state)
        )
        left_attitude_age_s = (
            None
            if left_attitude is None
            else abs(float(pair.left.timestamp_s) - float(left_attitude.timestamp_s))
        )
        right_attitude_age_s = (
            None
            if right_attitude is None
            else abs(float(pair.right.timestamp_s) - float(right_attitude.timestamp_s))
        )
        selected_left_gps, selected_right_gps, reused_gps_inputs = (
            self._cached_measurement_gps_states()
        )
        reused_inputs = tuple(
            dict.fromkeys((*reused_gps_inputs, *reused_attitude_inputs))
        )
        left_gps = (
            None
            if selected_left_gps is None
            else GpsSnapshot.from_state(selected_left_gps)
        )
        right_gps = (
            None
            if selected_right_gps is None
            else GpsSnapshot.from_state(selected_right_gps)
        )
        pair_sequence = int(read(self.state._measurement_attempt_counter, 0)) + 1
        self.state._measurement_attempt_counter = pair_sequence
        generation_id = str(
            read(self.state._measurement_attempt_generation_id, "")
            or self.inputs._measurement_id.get("")
            or "measurement"
        )
        measurement_id = str(self.inputs._measurement_id.get("") or generation_id)
        pair_id = f"{generation_id}:{pair_sequence:020d}"
        native_fields: dict[str, object] = {}
        persister = self.inputs._persist_native_detection_pair.get(None)
        pair_reservation = read(
            self.state._coordinated_detection_pair_reservation, None
        )
        if not isinstance(pair_reservation, RingPairReservation):
            pair_reservation = None
        if callable(persister):
            try:
                native_fields = dict(
                    persister(
                        left_detection=left_detection,
                        right_detection=right_detection,
                        pair_id=f"pair-{pair_sequence}",
                        purpose="method-evaluation",
                        reservation=pair_reservation,
                    )
                    or {}
                )
            except Exception:
                self._release_detection_pair_reservation(pair_reservation)
                self.state._coordinated_detection_pair_reservation = None
                raise
        if pair_reservation is not None and (not pair_reservation.active):
            self.state._coordinated_detection_pair_reservation = None
        if native_fields.get("native_pair_status") == "EVIDENCE_FRAME_UNAVAILABLE":
            raise ValueError(
                "EVIDENCE_FRAME_UNAVAILABLE: exact selected native frame is no longer available"
            )
        capture_sync = self._ftp_command_round_sync(pair)
        if not capture_sync:
            configured_delta_s = float(self.inputs._yolo_pair_max_delta_s.get()())
            timing_limit_ns = (
                100000000
                if not math.isfinite(configured_delta_s)
                else min(
                    100000000, max(0, int(round(configured_delta_s * 1000000000.0)))
                )
            )
            capture_sync = live_pair_timing_evidence(
                left_detection,
                right_detection,
                requested_basis=requested_timestamp_basis,
                effective_basis=timestamp_basis,
                fallback_reason=timing.fallback_reason,
                max_abs_delta_ns=timing_limit_ns,
            )
        snapshot = FrozenMeasurementPairSnapshot(
            measurement_id=measurement_id,
            pair_id=pair_id,
            pair_sequence=pair_sequence,
            created_monotonic_ns=created_monotonic_ns,
            created_unix_ns=created_unix_ns,
            timestamp_basis=timestamp_basis,
            left_detection=asdict(left_detection),
            right_detection=asdict(right_detection),
            left_attitude=None if left_attitude is None else asdict(left_attitude),
            right_attitude=None if right_attitude is None else asdict(right_attitude),
            left_gps=None if left_gps is None else asdict(left_gps),
            right_gps=None if right_gps is None else asdict(right_gps),
            native_frames=native_fields,
            capture_sync=capture_sync,
            reused_inputs=reused_inputs,
        )
        frozen = {
            "physical_key": tuple(physical_key),
            "snapshot": snapshot,
            "requested_timestamp_basis": requested_timestamp_basis,
            "timestamp_basis": timestamp_basis,
            "timing_fallback_reason": timing.fallback_reason,
            "pair_timing_decision": timing,
            "left_pairing_timestamp_s": float(left_pairing_timestamp_s),
            "right_pairing_timestamp_s": float(right_pairing_timestamp_s),
            "created_monotonic_s": created_monotonic_ns * 1e-09,
            "created_wall_s": created_unix_ns * 1e-09,
            "detection_midpoint_s": detection_midpoint_s,
            "left_detection": left_detection,
            "right_detection": right_detection,
            "left_attitude": left_attitude,
            "right_attitude": right_attitude,
            "left_attitude_age_s": left_attitude_age_s,
            "right_attitude_age_s": right_attitude_age_s,
            "left_gps": left_gps,
            "right_gps": right_gps,
            "measurement_start": measurement_start,
            "reused_inputs": reused_inputs,
        }
        captured = FrozenPairInputs(**frozen)
        captured = replace(
            captured, geometry_inputs=self.actions.freeze_geometry_inputs(captured)
        )
        self.state._last_frozen_measurement_pair = captured
        return captured
