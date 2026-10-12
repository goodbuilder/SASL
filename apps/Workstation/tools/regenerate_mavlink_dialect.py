#!/usr/bin/env python3
"""Reproduce the recorded MAVLink dialect from repo-owned, hash-pinned inputs."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import tempfile


DIALECT_ROOT = Path(__file__).resolve().parents[1] / "workstation_core/mavlink_dialects"
DEFINITIONS = DIALECT_ROOT / "definitions"
MANIFEST_PATH = DIALECT_ROOT / "generation_manifest.json"
INPUT_FILES = frozenset({"common.xml", "standard.xml", "minimal.xml", "mavschema.xsd", "COPYING"})


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_inputs(definitions: Path, manifest: dict) -> None:
    if set(manifest["inputs"]) != INPUT_FILES:
        raise ValueError("MAVLink input manifest must name the complete recorded input set")
    for name, expected in manifest["inputs"].items():
        if sha256(definitions / name) != expected:
            raise ValueError(f"MAVLink source hash mismatch: {name}")


def generate_bytes(*, definitions: Path = DEFINITIONS, manifest_path: Path = MANIFEST_PATH) -> bytes:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    verify_inputs(definitions, manifest)
    generator = manifest["generator"]
    installed = importlib.metadata.version("pymavlink")
    if installed != generator["version"]:
        raise RuntimeError(f"pymavlink {generator['version']} is required; installed {installed}")
    # Fail explicitly instead of accepting mavgen's optional no-validation fallback.
    import lxml.etree  # noqa: F401
    from pymavlink.generator import mavgen

    generator_root = Path(mavgen.__file__).resolve().parent
    for name, expected in generator["files"].items():
        if sha256(generator_root / name) != expected:
            raise RuntimeError(f"MAVLink generator source hash mismatch: {name}")
    with tempfile.TemporaryDirectory(prefix="workstation-mavlink-") as temporary:
        # Keep the canonical basename even for a caller-selected output location:
        # mavgen derives the Python DIALECT constant from this filename.
        output = Path(temporary) / manifest["generated"]["filename"]
        options = mavgen.Opts(
            str(output),
            wire_protocol=generator["wire_protocol"],
            language=generator["language"],
            validate=True,
            strict_units=generator["strict_units"],
        )
        previous_schema = mavgen.schemaFile
        try:
            # The firmware definitions use newer metadata than the wheel's XSD.
            # Use their recorded XSD while preserving XML validation.
            mavgen.schemaFile = str(definitions / "mavschema.xsd")
            try:
                generated = mavgen.mavgen(options, [str(definitions / "common.xml")])
            except SystemExit as exc:
                validation = "enabled" if options.validate else "disabled"
                raise RuntimeError(f"MAVLink generation failed with XML validation {validation}: {exc}") from exc
            if not generated or not options.validate:
                raise RuntimeError("MAVLink generation did not complete with XML validation enabled")
        finally:
            mavgen.schemaFile = previous_schema
        return output.read_bytes()


def check() -> str:
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    generated = generate_bytes()
    actual = (DIALECT_ROOT / manifest["generated"]["filename"]).read_bytes()
    digest = hashlib.sha256(generated).hexdigest()
    if digest != manifest["generated"]["sha256"] or generated != actual:
        raise RuntimeError("Regenerated dialect differs from the recorded runtime artifact")
    return digest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="Verify exact regeneration (the default)")
    mode.add_argument("--output", type=Path, help="Write regenerated Python to an explicitly selected file")
    args = parser.parse_args(argv)
    try:
        if args.output is None:
            print(f"MAVLink reproduction PASS: {check()}")
        else:
            content = generate_bytes()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_bytes(content)
            print(f"Generated {args.output}: {hashlib.sha256(content).hexdigest()}")
    except Exception as exc:
        print(f"MAVLink reproduction failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
