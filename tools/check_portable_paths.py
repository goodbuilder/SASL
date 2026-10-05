#!/usr/bin/env python3
"""Reject tracked checkout/user-home paths outside historical evidence.

This check intentionally does not reject system contracts such as ``/dev``,
``/opt/ros``, or bounded ``/tmp`` runtime files.  It targets paths that bind a
tracked script/configuration to one user or to one dated checkout directory.
"""

from __future__ import annotations

import argparse
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
            text = path.read_text(encoding="utf-8")
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
