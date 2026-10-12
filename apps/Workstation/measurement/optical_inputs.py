"""The only optical-pose operations/data visible to Measurement input capture."""

from dataclasses import dataclass
from typing import Any
from Workstation.measurement.ports import ValueInput


@dataclass(frozen=True)
class OpticalMeasurementInputs:
    read_intrinsics_snapshot: ValueInput
    read_operating_intrinsics_profile_for_geometry: ValueInput
    read_operating_intrinsics_profile: ValueInput
    read_active_selection: ValueInput
    read_candidate_mapping_with_prior: ValueInput
    read_candidates: ValueInput
    read_candidate_priors: ValueInput
    read_candidate_drone_rotation_context: ValueInput
    read_last_accepted_auto_suggestion: ValueInput
    read_workstation_optical_pose_snapshot: ValueInput

    @property
    def _intrinsics_snapshot(self) -> Any:
        return self.read_intrinsics_snapshot.get()

    @property
    def operating_intrinsics_profile_for_geometry(self) -> Any:
        return self.read_operating_intrinsics_profile_for_geometry.get()

    @property
    def _operating_intrinsics_profile(self) -> Any:
        return self.read_operating_intrinsics_profile.get()

    @property
    def _active_selection(self) -> Any:
        return self.read_active_selection.get()

    @property
    def _candidate_mapping_with_prior(self) -> Any:
        return self.read_candidate_mapping_with_prior.get()

    @property
    def _candidates(self) -> Any:
        return self.read_candidates.get()

    @property
    def _candidate_priors(self) -> Any:
        return self.read_candidate_priors.get()

    @property
    def _candidate_drone_rotation_context(self) -> Any:
        return self.read_candidate_drone_rotation_context.get()

    @property
    def _last_accepted_auto_suggestion(self) -> Any:
        return self.read_last_accepted_auto_suggestion.get()

    @property
    def workstation_optical_pose_snapshot(self) -> Any:
        return self.read_workstation_optical_pose_snapshot.get()
