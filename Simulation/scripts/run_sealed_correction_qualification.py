#!/usr/bin/env python3
"""Prepare and freeze a sealed Simulation correction qualification cycle.

R4-A fits candidates from Calibration artifacts and selects one global method
family from Validation artifacts.  The sealed plan is committed but is not
executed by this CLI during R4-A.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPOSITORY_ROOT / "Simulation" / "stereo_ground_sim"
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_qualification import (  # noqa: E402
    FREEZE_ANCHOR_REPOSITORY_PATH,
    QualificationSpec,
    authorize_unseal,
    fit_select_freeze,
    prepare_plans,
    qualification_status,
    verify_freeze_anchor,
)


def _command_output(command: Sequence[str], *, cwd: Path) -> str:
    return subprocess.check_output(command, cwd=cwd, text=True).strip()


def verify_git_provenance(
    repository_root: Path,
    *,
    expected_commit_sha: str,
    expected_branch: str,
) -> dict[str, str]:
    """Verify that qualification code is the requested clean exact revision."""

    root = Path(repository_root).resolve(strict=True)
    if _command_output(("git", "rev-parse", "--show-toplevel"), cwd=root) != str(root):
        raise ValueError("script repository root does not match Git toplevel")
    if len(expected_commit_sha) != 40 or any(
        character not in "0123456789abcdef" for character in expected_commit_sha
    ):
        raise ValueError("expected commit SHA must be 40 lowercase hexadecimal characters")
    if not str(expected_branch).strip():
        raise ValueError("expected branch must be non-empty")
    actual_sha = _command_output(("git", "rev-parse", "HEAD"), cwd=root)
    actual_branch = _command_output(("git", "branch", "--show-current"), cwd=root)
    status = _command_output(("git", "status", "--short"), cwd=root)
    if actual_sha != expected_commit_sha:
        raise ValueError(f"HEAD mismatch: expected {expected_commit_sha}, got {actual_sha}")
    if actual_branch != expected_branch:
        raise ValueError(
            f"branch mismatch: expected {expected_branch}, got {actual_branch}"
        )
    if status:
        raise ValueError("working tree must be clean")
    return {
        "repository_root": str(root),
        "branch": actual_branch,
        "commit_sha": actual_sha,
        "git_status": "clean",
    }


def _absolute_external_path(path: Path, *, name: str, must_exist: bool) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        raise ValueError(f"{name} must be an absolute path")
    resolved = candidate.resolve(strict=must_exist)
    repository = REPOSITORY_ROOT.resolve(strict=True)
    if resolved == repository or repository in resolved.parents:
        raise ValueError(f"{name} must be outside the repository")
    return resolved


def validate_prepare_output(path: Path) -> Path:
    output = _absolute_external_path(path, name="output directory", must_exist=False)
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError("prepare output directory must be new or empty")
    output.mkdir(parents=True, exist_ok=True)
    return output


def _existing_external_directory(path: Path, *, name: str) -> Path:
    resolved = _absolute_external_path(path, name=name, must_exist=True)
    if not resolved.is_dir():
        raise ValueError(f"{name} must be a directory")
    return resolved


def _add_exact_revision_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--expected-commit-sha", required=True)
    parser.add_argument("--expected-branch", required=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    prepare = subparsers.add_parser(
        "prepare-plans",
        help="Create deterministic Calibration, Validation, and unexecuted sealed plans.",
    )
    prepare.add_argument("--output-dir", type=Path, required=True)
    prepare.add_argument("--qualification-id", required=True)
    prepare.add_argument("--repository-name", required=True)
    _add_exact_revision_arguments(prepare)

    freeze = subparsers.add_parser(
        "fit-select-freeze",
        help="Fit from Calibration, select from Validation, and freeze the catalog.",
    )
    freeze.add_argument("--qualification-root", type=Path, required=True)
    freeze.add_argument("--calibration-sweep-root", type=Path, required=True)
    freeze.add_argument("--validation-sweep-root", type=Path, required=True)
    _add_exact_revision_arguments(freeze)

    status = subparsers.add_parser(
        "status", help="Validate and report the current qualification state read-only."
    )
    status.add_argument("--qualification-root", type=Path, required=True)
    _add_exact_revision_arguments(status)

    verify_anchor = subparsers.add_parser(
        "verify-freeze-anchor",
        help="Verify the Git freeze anchor and external R4-A artifacts read-only.",
    )
    verify_anchor.add_argument("--qualification-root", type=Path, required=True)
    verify_anchor.add_argument("--freeze-anchor", type=Path, required=True)
    verify_anchor.add_argument("--repository-root", type=Path, required=True)

    unseal = subparsers.add_parser(
        "unseal-evaluate",
        help="Authorize R4-B sealed execution using exact frozen artifact hashes.",
    )
    unseal.add_argument("--qualification-root", type=Path, required=True)
    unseal.add_argument("--freeze-anchor", type=Path, required=True)
    unseal.add_argument("--repository-root", type=Path, required=True)
    unseal.add_argument("--frozen-catalog-sha256", required=True)
    unseal.add_argument("--sealed-commitment-sha256", required=True)
    _add_exact_revision_arguments(unseal)
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    return build_parser().parse_args(argv)


def execute(args: argparse.Namespace) -> Mapping[str, Any]:
    if args.command == "verify-freeze-anchor":
        repository_root = Path(args.repository_root).resolve(strict=True)
        anchor_path = Path(args.freeze_anchor)
        if not anchor_path.is_absolute():
            anchor_path = repository_root / anchor_path
        return verify_freeze_anchor(
            _existing_external_directory(
                args.qualification_root, name="qualification root"
            ),
            anchor_path,
            repository_root=repository_root,
        )

    provenance = verify_git_provenance(
        REPOSITORY_ROOT,
        expected_commit_sha=args.expected_commit_sha,
        expected_branch=args.expected_branch,
    )
    if args.command == "prepare-plans":
        if not str(args.qualification_id).strip():
            raise ValueError("qualification ID must be non-empty")
        if not str(args.repository_name).strip():
            raise ValueError("repository name must be non-empty")
        output = validate_prepare_output(args.output_dir)
        spec = QualificationSpec(
            qualification_id=args.qualification_id,
            repository_name=args.repository_name,
            branch=provenance["branch"],
            commit_sha=provenance["commit_sha"],
        )
        return prepare_plans(output, spec)
    if args.command == "fit-select-freeze":
        return fit_select_freeze(
            qualification_root=_existing_external_directory(
                args.qualification_root, name="qualification root"
            ),
            calibration_sweep_root=_existing_external_directory(
                args.calibration_sweep_root, name="calibration sweep root"
            ),
            validation_sweep_root=_existing_external_directory(
                args.validation_sweep_root, name="validation sweep root"
            ),
            expected_branch=provenance["branch"],
            expected_commit_sha=provenance["commit_sha"],
        )
    if args.command == "status":
        return qualification_status(
            _existing_external_directory(
                args.qualification_root, name="qualification root"
            )
        )
    if args.command == "unseal-evaluate":
        repository_root = Path(args.repository_root).resolve(strict=True)
        if repository_root != REPOSITORY_ROOT.resolve(strict=True):
            raise ValueError("unseal repository root must be the CLI repository")
        anchor_path = Path(args.freeze_anchor)
        if not anchor_path.is_absolute():
            anchor_path = repository_root / anchor_path
        if anchor_path.relative_to(repository_root).as_posix() != FREEZE_ANCHOR_REPOSITORY_PATH:
            raise ValueError("unseal freeze anchor path mismatch")
        return authorize_unseal(
            _existing_external_directory(
                args.qualification_root, name="qualification root"
            ),
            freeze_anchor_path=anchor_path,
            repository_root=repository_root,
            frozen_catalog_sha256=args.frozen_catalog_sha256,
            sealed_commitment_sha256=args.sealed_commitment_sha256,
        )
    raise ValueError(f"unsupported command: {args.command}")


def main(argv: Sequence[str] | None = None) -> int:
    result = execute(parse_args(argv))
    print(
        json.dumps(
            dict(result),
            ensure_ascii=False,
            sort_keys=True,
            allow_nan=False,
            separators=(",", ":"),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
