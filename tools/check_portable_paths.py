#!/usr/bin/env python3
"""Reject tracked checkout/user-home paths outside historical evidence.

This check intentionally does not reject system contracts such as ``/dev``,
``/opt/ros``, or bounded ``/tmp`` runtime files.  It targets paths that bind a
tracked script/configuration to one user or to one dated checkout directory.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import subprocess
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCANNED_SUFFIXES = {
    ".bash",
    ".cfg",
    ".ini",
    ".json",
    ".md",
    ".py",
    ".sh",
    ".toml",
    ".yaml",
    ".yml",
}
HISTORICAL_PREFIXES = (
    "apps/Simulation/reports/",
    "apps/StereoWIRIS/calibration_captures/",
    "apps/StereoWIRIS/docs/changes/",
    "common/firmware/artifacts/",
)
EXEMPT_EXACT = {
    "AGENTS.md",
    "apps/StereoWIRIS/UPDATE.md",
    "apps/Workstation/intrinsics_zoom_08.json",
    "apps/Workstation/tests/test_portable_path_policy.py",
    "apps/Workstation/tests/test_portable_paths.py",
    "tools/check_portable_paths.py",
}
# Reviewed immutable provenance, not runtime setup instructions. Match both
# path and bytes so edits/new files cannot acquire a historical exemption.
# P1 records where its settings were observed; the remaining files preserve
# previous executions/current-behavior observations, including failed runs.
HISTORICAL_CONTENT_SHA256 = {
    "apps/Workstation/Pixhawk/P1/workstation_endpoint.json":
        "8827d4ad1af5fcf98981907d14838922aa3adb3c89941423aa06a3ae01e84ad3",
    "apps/Workstation/docs/changes/20260829-calibration-app-receive-timing/current-behavior.md":
        "240e1ebe311ca1ccc03ed22442cc56d465c7a185ba810c19be0c80c2572d8438",
    "apps/Workstation/docs/changes/20260829-calibration-app-receive-timing/verification.md":
        "e805a4a3e40b5932a4a38c4c9d2b86478a53f5dd686c6aac15231972cbda21ca",
    "apps/Workstation/docs/changes/20260903-jsonl-csv-mirror/current-behavior.md":
        "76b9041981761ec01ae0ab25ee4baa0e38664aaa720a9271317bab032c588aba",
    "apps/Workstation/docs/changes/20260904-magnetometer-yaw-display/current-behavior.md":
        "5fbbbbd313d912e3adea0541052566582a52560b4c4431d1cdd6a3f50d3a51ea",
    "apps/Workstation/docs/changes/20260904-magnetometer-yaw-display/verification.md":
        "e969ce802811990d2e9f1b3ffd427bfcdb2d82d65df2462616c003887c32c0dc",
    "apps/Workstation/docs/changes/20260904-mountain-rt-ui-display/current-behavior.md":
        "9057bae87da03a0d3824eb18a0b2e977d8d973e72102d349688799cf09da6260",
}
FORBIDDEN_PATTERNS = (
    ("POSIX user-home absolute path", re.compile(r"/(?:home|Users)/[^/\s'\"`]+/")),
    ("Windows user-home absolute path", re.compile(r"[A-Za-z]:\\Users\\[^\\\s'\"`]+\\")),
    ("dated SASL checkout name", re.compile(r"\bSASL-[0-9]{8}\b")),
)


@dataclass(frozen=True)
class Violation:
    path: str
    line_number: int
    reason: str
    line: str


def tracked_paths(root: Path = ROOT) -> tuple[str, ...]:
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    )
    return tuple(line for line in result.stdout.splitlines() if line)


def is_exempt_path(path: str) -> bool:
    return path in EXEMPT_EXACT or path.startswith(HISTORICAL_PREFIXES)


def scan_text(path: str, text: str) -> tuple[Violation, ...]:
    if is_exempt_path(path) or Path(path).suffix.lower() not in SCANNED_SUFFIXES:
        return ()
    historical_hash = HISTORICAL_CONTENT_SHA256.get(path)
    if historical_hash and hashlib.sha256(text.encode("utf-8")).hexdigest() == historical_hash:
        return ()
    violations: list[Violation] = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        for reason, pattern in FORBIDDEN_PATTERNS:
            if pattern.search(line):
                violations.append(Violation(path, line_number, reason, line.strip()))
    return tuple(violations)


def repository_violations(
    root: Path = ROOT,
    paths: Iterable[str] | None = None,
) -> tuple[Violation, ...]:
    violations: list[Violation] = []
    for relative in tracked_paths(root) if paths is None else paths:
        path = root / relative
        if not path.is_file():
            continue
        try:
            # Do not normalize CRLF before the immutable provenance hash check.
            text = path.read_bytes().decode("utf-8")
        except UnicodeDecodeError:
            continue
        violations.extend(scan_text(relative, text))
    return tuple(violations)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Reject tracked user-home and dated-checkout absolute paths."
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    failures = repository_violations(args.root.resolve())
    if not failures:
        print("portable path policy: PASS")
        return 0
    for failure in failures:
        print(
            f"portable path policy: {failure.reason}: "
            f"{failure.path}:{failure.line_number}: {failure.line}"
        )
    print(f"portable path policy: FAIL ({len(failures)} violations)")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
