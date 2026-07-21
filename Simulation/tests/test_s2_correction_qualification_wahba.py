from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.coordinate_frames import optical_rpy_error_rotation
from stereo_ground_sim.correction_qualification import (
    CandidateProfile,
    WahbaObservation,
    fit_wahba,
)


RAYS = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 2.0, 3.0),
    (-2.0, 1.0, 4.0),
    (3.0, -1.0, 2.0),
)


def _unit(value) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=float)
    array /= np.linalg.norm(array)
    return tuple(float(item) for item in array)


def _observations(rotation) -> list[WahbaObservation]:
    matrix = np.asarray(rotation, dtype=float)
    return [
        WahbaObservation(
            source_ray_actual_rdf=_unit(ray),
            target_ray_nominal_rdf=_unit(matrix @ np.asarray(_unit(ray))),
            weight=1.0,
            case_id=f"case-{index}",
            camera="left" if index % 2 else "right",
        )
        for index, ray in enumerate(RAYS)
    ]


def _rotation_error(actual, expected) -> float:
    relative = np.asarray(actual) @ np.asarray(expected).T
    sine = 0.5 * math.sqrt(
        (relative[2, 1] - relative[1, 2]) ** 2
        + (relative[0, 2] - relative[2, 0]) ** 2
        + (relative[1, 0] - relative[0, 1]) ** 2
    )
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) * 0.5))
    return abs(math.atan2(sine, cosine))


def test_identity_wahba_recovery() -> None:
    result = fit_wahba(_observations(np.eye(3)))
    assert result.status == "fitted"
    assert _rotation_error(result.rotation_cnominal_from_cactual, np.eye(3)) <= 1.0e-12
    assert result.support_count == len(RAYS)
    assert result.case_count == len(RAYS)
    assert result.determinant == pytest.approx(1.0, abs=1.0e-12)
    assert result.orthonormality_error <= 1.0e-12


@pytest.mark.parametrize(
    "rpy_deg",
    (
        (0.1, 0.0, 0.0),
        (-0.1, 0.0, 0.0),
        (0.0, 0.1, 0.0),
        (0.0, -0.1, 0.0),
        (0.0, 0.0, 0.1),
        (0.0, 0.0, -0.1),
        (0.05, -0.08, 0.12),
    ),
)
def test_single_axis_and_mixed_rotation_direction_recovery(rpy_deg) -> None:
    expected = optical_rpy_error_rotation(*(math.radians(value) for value in rpy_deg))
    result = fit_wahba(_observations(expected))
    assert _rotation_error(result.rotation_cnominal_from_cactual, expected) <= 1.0e-10
    source = np.asarray(_unit((0.4, -0.2, 1.0)))
    assert np.asarray(result.rotation_cnominal_from_cactual) @ source == pytest.approx(
        np.asarray(expected) @ source, abs=1.0e-10
    )


def test_reflection_input_is_corrected_to_proper_rotation() -> None:
    reflection = np.diag((-1.0, 1.0, 1.0))
    result = fit_wahba(_observations(reflection))
    matrix = np.asarray(result.rotation_cnominal_from_cactual)
    assert np.linalg.det(matrix) == pytest.approx(1.0, abs=1.0e-12)
    assert matrix.T @ matrix == pytest.approx(np.eye(3), abs=1.0e-12)


def test_parallel_observations_are_explicitly_degenerate() -> None:
    observations = [
        WahbaObservation((0.0, 0.0, 1.0), (0.0, 0.0, 1.0), 1.0, f"case-{i}", "left")
        for i in range(3)
    ]
    with pytest.raises(ValueError, match="degenerate"):
        fit_wahba(observations)


@pytest.mark.parametrize(
    "source,target,weight",
    (
        ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 1.0),
        ((math.nan, 0.0, 1.0), (0.0, 0.0, 1.0), 1.0),
        ((0.0, 0.0, 1.0), (math.inf, 0.0, 1.0), 1.0),
        ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0), 0.0),
        ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0), math.inf),
    ),
)
def test_invalid_wahba_observation_is_rejected(source, target, weight) -> None:
    with pytest.raises(ValueError):
        WahbaObservation(source, target, weight, "case", "left")


def test_invalid_rotation_matrix_is_rejected_by_candidate_contract() -> None:
    reflection = ((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match="proper orthonormal rotation"):
        CandidateProfile("raw_nominal", "profile", reflection, reflection, None, None)
