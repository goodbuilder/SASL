#!/usr/bin/env python3
"""Reject newly tracked machine-local or generated repository state."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN_PARTS = {"__pycache__", ".pytest_cache", "build", "install", "log"}
FORBIDDEN_PREFIXES = (
    "Simulation/.venv",
    "Simulation/PX4-Autopilot/",
    "workstation_data/",
)
FORBIDDEN_EXACT = {"perf.data"}
RUNTIME_RECORDING_PREFIX = "StereoWIRIS/recordings/"
RUNTIME_RECORDING_ALLOW = {"StereoWIRIS/recordings/README.md"}


def tracked_paths() -> list[str]:
    result = subprocess.run(
        ["git", "ls-files"], cwd=ROOT, check=True, capture_output=True, text=True
    )
    return [line for line in result.stdout.splitlines() if line]


def violation(path: str) -> str | None:
    parts = set(Path(path).parts)
    if path in FORBIDDEN_EXACT:
        return "machine-local profiler output"
    if parts & FORBIDDEN_PARTS:
        return "generated cache/build/install/log path"
    if path.startswith(FORBIDDEN_PREFIXES):
        return "machine-local dependency or runtime state"
    if path.startswith(RUNTIME_RECORDING_PREFIX) and path not in RUNTIME_RECORDING_ALLOW:
        return "runtime recording outside tests/fixtures"
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--allow-existing-debt",
        action="store_true",
        help="report existing tracked artifacts but do not fail; migration-only escape hatch",
    )
    args = parser.parse_args()
    failures = [(path, reason) for path in tracked_paths() if (reason := violation(path))]
    if not failures:
        print("repository policy: PASS")
        return 0
    for path, reason in failures:
        print(f"repository policy: {reason}: {path}")
    if args.allow_existing_debt:
        print(f"repository policy: DEBT ({len(failures)} tracked paths; no failure by request)")
        return 0
    print(f"repository policy: FAIL ({len(failures)} tracked paths)")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
