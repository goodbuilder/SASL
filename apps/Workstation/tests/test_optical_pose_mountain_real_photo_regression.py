"""Opt-in Mountain auto-select regression guards against local real photos.

Every earlier synthetic test for ``detect_mountain`` passed while the real
bug (matching clouds/sky instead of the ridge) was still present, and each
subsequent fix that looked correct on one real photo produced a new failure
mode on the next real photo. These tests pin the two real photos that
exposed those failures so a future change to the sky/ground classifier or
the boundary-band filter cannot silently regress back to either one.

The source photos are not committed to the repository. Set
``WORKSTATION_RUN_LOCAL_PHOTO_REGRESSION=1`` to include these slow,
machine-local checks; the default suite skips this module.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from Workstation.workstation_core.optical_pose_auto_select import detect_mountain

_DOWNLOADS = Path.home() / "다운로드"
_TREE_SUNSET_PHOTO = _DOWNLOADS / "f6f187f04281b84b7ee1440b7730a087.jpg"
_MOUNTAIN_LEFT_PHOTO = _DOWNLOADS / "2L.png"
_MOUNTAIN_RIGHT_PHOTO = _DOWNLOADS / "2R.png"

# Ideal pinhole placeholders (fx=fy=max(width,height), principal point at
# image center, zero distortion) matching each photo's native resolution --
# see intrinsics_dummy_3024x4032.json / intrinsics_dummy_1672x941.json. Not
# a real calibration; only exercises the undistort/epipolar-bias code path
# instead of the "no K,D at all" degraded path.
_TREE_PHOTO_K = ((4032.0, 0.0, 1512.0), (0.0, 4032.0, 2016.0), (0.0, 0.0, 1.0))
_MOUNTAIN_PHOTO_K = ((1672.0, 0.0, 836.0), (0.0, 1672.0, 470.5), (0.0, 0.0, 1.0))
_ZERO_DISTORTION = (0.0, 0.0, 0.0, 0.0, 0.0)


def _require_photo(path: Path) -> None:
    if not path.exists():
        pytest.skip(f"real regression photo is not present on this machine: {path}")


def test_mountain_regression_partial_width_silhouette_stays_off_sky() -> None:
    """A single foreground tree against a dramatic sunset sky, occupying only
    roughly a third of the frame's width, with the rest open sky/cloud. This
    was the photo that first exposed Mountain matching high-contrast
    cloud/sun-glare edges instead of the ridge, because a single
    full-width straight-line skyline fit gets pulled toward whichever edge
    dominates the column-wise gradient search.
    """

    _require_photo(_TREE_SUNSET_PHOTO)
    image = cv2.imread(str(_TREE_SUNSET_PHOTO))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    height, _width = gray.shape[:2]
    # No true right-eye capture exists for this single photo; a horizontal
    # roll stands in for it, matching every earlier ad-hoc verification of
    # this photo in this investigation.
    right = np.roll(gray, -20, axis=1)

    result = detect_mountain(
        gray,
        right,
        left_camera_matrix=_TREE_PHOTO_K,
        right_camera_matrix=_TREE_PHOTO_K,
        left_distortion_coefficients=_ZERO_DISTORTION,
        right_distortion_coefficients=_ZERO_DISTORTION,
        baseline_ned_m=(0.0, 0.5, 0.0),
    )

    assert result.state == "GOOD"
    assert result.match_count >= 4
    # default max_points=0 keeps every RANSAC inlier
    assert len(result.left.points) == result.inlier_count >= 4
    # The sky/cloud band fills roughly the top ~45% of this frame; the tree
    # silhouette and grass horizon sit well below that. The originally
    # broken line-fit put points as high as the top few percent of the
    # frame, so this threshold has ample margin on both sides.
    sky_boundary_y = 0.42 * height
    bad_points = [(x, y) for x, y in result.left.points if y <= sky_boundary_y]
    assert not bad_points, f"points landed back in the sky/cloud band: {bad_points}"


def test_mountain_regression_real_stereo_ridge_pair_stays_off_cloud() -> None:
    """A genuine stereo pair (not a single photo with a synthetic shift) of
    an actual rocky mountain ridge. Fixing the tree photo above first
    introduced a boundary-band filter that only checked a keypoint's
    proximity to the (sometimes locally wrong) estimated boundary curve --
    not whether the point itself was ground-like -- which let two isolated
    cloud keypoints through here even though the ridge itself was matched
    correctly everywhere else.
    """

    _require_photo(_MOUNTAIN_LEFT_PHOTO)
    _require_photo(_MOUNTAIN_RIGHT_PHOTO)
    left = cv2.imread(str(_MOUNTAIN_LEFT_PHOTO))
    right = cv2.imread(str(_MOUNTAIN_RIGHT_PHOTO))
    left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    height, _width = left_gray.shape[:2]

    result = detect_mountain(
        left_gray,
        right_gray,
        left_camera_matrix=_MOUNTAIN_PHOTO_K,
        right_camera_matrix=_MOUNTAIN_PHOTO_K,
        left_distortion_coefficients=_ZERO_DISTORTION,
        right_distortion_coefficients=_ZERO_DISTORTION,
        baseline_ned_m=(0.0, 0.5, 0.0),
    )

    assert result.state == "GOOD"
    assert result.match_count >= 4
    # default max_points=0 keeps every RANSAC inlier
    assert len(result.left.points) == result.inlier_count >= 4
    # Known bad points from the original failure were (1250, 209) and
    # (142, 81) -- isolated cloud keypoints far above the true ridge
    # (~470-513). This threshold sits well between the two clusters.
    sky_boundary_y = 0.30 * height
    bad_points = [(x, y) for x, y in result.left.points if y <= sky_boundary_y]
    assert not bad_points, f"points landed back on cloud, not the ridge: {bad_points}"
