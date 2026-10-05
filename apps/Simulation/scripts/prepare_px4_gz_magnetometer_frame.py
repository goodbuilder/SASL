#!/usr/bin/env python3
"""Prepare and verify the pinned PX4 Gazebo magnetometer frame boundary.

Gazebo magnetometer samples are sensor-local FLU vectors.  PX4 ``sensor_mag``
expects body FRD, so the bridge must use the same ``(x, -y, -z)`` rotation as
the IMU callback.  PX4 v1.17.0 contains an older Gazebo-specific mapping and a
previous SASL checkout may contain a one-line sign workaround.  This helper
accepts only those two known predecessors or the final mapping; any other
source fails closed without being edited.

The helper also owns a build fingerprint used by ``run_operator_view.sh``.
Skipping the PX4 build is allowed only when the current bridge source,
camera-airframe source, PX4 revision, and built binary match a stamp written
after a successful build.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


SCHEMA = "sasl-px4-gz-magnetometer-frame-v1"
DEFAULT_REQUIRED_VERSION = "v1.17.0"
BRIDGE_RELATIVE_PATH = Path("src/modules/simulation/gz_bridge/GZBridge.cpp")
DEFAULT_STAMP_RELATIVE_PATH = Path(
    "build/px4_sitl_default/.sasl-gz-magnetometer-frame-v1.json"
)

_ASSIGNMENT_PATTERN = re.compile(
    r"(?P<indent>[ \t]*)report\.x\s*=\s*(?P<x>[^;\n]+);\s*\n"
    r"(?P=indent)report\.y\s*=\s*(?P<y>[^;\n]+);\s*\n"
    r"(?P=indent)report\.z\s*=\s*(?P<z>[^;\n]+);"
)

_VARIANTS = {
    (
        "-msg.field_tesla().y()",
        "-msg.field_tesla().x()",
        "msg.field_tesla().z()",
    ): "stock_v1_17",
    (
        "-msg.field_tesla().y()",
        "msg.field_tesla().x()",
        "msg.field_tesla().z()",
    ): "legacy_y_sign_workaround",
    (
        "msg.field_tesla().x()",
        "-msg.field_tesla().y()",
        "-msg.field_tesla().z()",
    ): "sensor_flu_to_body_frd",
}

_OLD_COMMENT = (
    "\t// FIMEX: once we're on jetty or later\n"
    "\t// The magnetometer plugin publishes in units of gauss and in a weird left handed coordinate system\n"
    "\t// https://github.com/gazebosim/gz-sim/pull/2460\n"
)
_FINAL_COMMENT = (
    "\t// gz::msgs::Magnetometer is sensor-local FLU. Convert to body FRD,\n"
    "\t// matching imuCallback so arbitrary roll/pitch/yaw remain valid.\n"
)


class PreparationError(RuntimeError):
    """Raised when a PX4 checkout cannot be changed or trusted safely."""


def _normalize(expression: str) -> str:
    return "".join(str(expression).split())


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_output(px4_root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(px4_root), *args],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        raise PreparationError(
            f"PX4 git query failed ({' '.join(args)}): {result.stderr.strip()}"
        )
    return result.stdout.strip()


def px4_identity(
    px4_root: Path, *, required_version: str = DEFAULT_REQUIRED_VERSION
) -> dict[str, str]:
    root = px4_root.resolve()
    if not (root / ".git").exists():
        raise PreparationError(f"PX4 checkout is not a Git worktree: {root}")
    version = _git_output(root, "describe", "--tags", "--exact-match", "HEAD")
    if version != required_version:
        raise PreparationError(
            f"PX4 exact version mismatch: required={required_version}, actual={version}"
        )
    return {
        "px4_version": version,
        "px4_head": _git_output(root, "rev-parse", "HEAD"),
    }


def _callback_region(source: str) -> tuple[int, int]:
    start = source.find("void GZBridge::magnetometerCallback")
    if start < 0:
        raise PreparationError("PX4 GZBridge magnetometerCallback was not found")
    publish = source.find("_sensor_mag_pub.publish(report);", start)
    if publish < 0:
        raise PreparationError("PX4 GZBridge magnetometer publish boundary was not found")
    return start, publish


def bridge_variant(source: str) -> tuple[str, re.Match[str]]:
    start, end = _callback_region(source)
    matches = list(_ASSIGNMENT_PATTERN.finditer(source, start, end))
    if len(matches) != 1:
        raise PreparationError(
            "PX4 GZBridge magnetometer assignment block must occur exactly once "
            f"inside magnetometerCallback (found {len(matches)})"
        )
    match = matches[0]
    expressions = tuple(
        _normalize(match.group(axis)) for axis in ("x", "y", "z")
    )
    variant = _VARIANTS.get(expressions)
    if variant is None:
        raise PreparationError(
            "PX4 GZBridge magnetometer mapping is not a known stock, legacy, "
            f"or corrected variant: x={expressions[0]}, y={expressions[1]}, "
            f"z={expressions[2]}"
        )
    return variant, match


def prepare_bridge_source(source: str) -> tuple[str, str, bool]:
    variant, match = bridge_variant(source)
    if variant == "sensor_flu_to_body_frd":
        return source, variant, False

    indent = match.group("indent")
    replacement = (
        f"{indent}report.x = msg.field_tesla().x();\n"
        f"{indent}report.y = -msg.field_tesla().y();\n"
        f"{indent}report.z = -msg.field_tesla().z();"
    )
    updated = source[: match.start()] + replacement + source[match.end() :]
    if _OLD_COMMENT in updated:
        updated = updated.replace(_OLD_COMMENT, _FINAL_COMMENT, 1)
    elif _FINAL_COMMENT not in updated:
        assignment_start = updated.find(replacement, _callback_region(updated)[0])
        updated = updated[:assignment_start] + _FINAL_COMMENT + updated[assignment_start:]

    final_variant, _ = bridge_variant(updated)
    if final_variant != "sensor_flu_to_body_frd":
        raise PreparationError("internal error: prepared bridge source is not corrected")
    return updated, variant, True


def prepare_checkout(
    px4_root: Path,
    *,
    required_version: str = DEFAULT_REQUIRED_VERSION,
    apply: bool,
) -> dict[str, Any]:
    root = px4_root.resolve()
    identity = px4_identity(root, required_version=required_version)
    bridge_path = root / BRIDGE_RELATIVE_PATH
    if not bridge_path.is_file():
        raise PreparationError(f"PX4 GZBridge source is missing: {bridge_path}")
    source = bridge_path.read_text(encoding="utf-8")
    updated, before_variant, changed = prepare_bridge_source(source)
    current_source_sha256 = hashlib.sha256(source.encode("utf-8")).hexdigest()
    prepared_source_sha256 = hashlib.sha256(updated.encode("utf-8")).hexdigest()

    status = "already_correct"
    if changed:
        status = "patch_required"
        if apply:
            bridge_path.write_text(updated, encoding="utf-8")
            status = "applied"

    inspected = updated if apply and changed else source
    after_variant, _ = bridge_variant(inspected)
    result: dict[str, Any] = {
        "schema": SCHEMA,
        **identity,
        "mode": "apply" if apply else "check",
        "status": status,
        "before_variant": before_variant,
        "after_variant": after_variant,
        "bridge_relative_path": BRIDGE_RELATIVE_PATH.as_posix(),
        "bridge_source_sha256": _sha256(bridge_path),
        "current_bridge_source_sha256": current_source_sha256,
        "prepared_bridge_source_sha256": prepared_source_sha256,
    }
    return result


def _default_stamp_path(px4_root: Path) -> Path:
    return px4_root.resolve() / DEFAULT_STAMP_RELATIVE_PATH


def build_fingerprint(
    px4_root: Path,
    *,
    airframe: Path,
    binary: Path,
    required_version: str = DEFAULT_REQUIRED_VERSION,
) -> dict[str, Any]:
    root = px4_root.resolve()
    prepared = prepare_checkout(
        root, required_version=required_version, apply=False
    )
    if prepared["after_variant"] != "sensor_flu_to_body_frd" or prepared[
        "status"
    ] == "patch_required":
        raise PreparationError("PX4 GZBridge source is not prepared for FLU to FRD")
    airframe_path = airframe.resolve()
    binary_path = binary.resolve()
    if not airframe_path.is_file():
        raise PreparationError(f"camera airframe source is missing: {airframe_path}")
    if not binary_path.is_file() or not os.access(binary_path, os.X_OK):
        raise PreparationError(f"PX4 SITL binary is missing or not executable: {binary_path}")
    return {
        "schema": SCHEMA,
        "px4_version": prepared["px4_version"],
        "px4_head": prepared["px4_head"],
        "bridge_relative_path": BRIDGE_RELATIVE_PATH.as_posix(),
        "bridge_variant": prepared["after_variant"],
        "bridge_source_sha256": prepared["bridge_source_sha256"],
        "camera_airframe_sha256": _sha256(airframe_path),
        "px4_binary_sha256": _sha256(binary_path),
    }


def write_build_stamp(
    px4_root: Path,
    *,
    airframe: Path,
    binary: Path,
    stamp: Path | None = None,
    required_version: str = DEFAULT_REQUIRED_VERSION,
) -> dict[str, Any]:
    payload = build_fingerprint(
        px4_root,
        airframe=airframe,
        binary=binary,
        required_version=required_version,
    )
    stamp_path = (stamp or _default_stamp_path(px4_root)).resolve()
    stamp_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=stamp_path.parent,
        prefix=f".{stamp_path.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")
        temporary = Path(handle.name)
    os.replace(temporary, stamp_path)
    return {**payload, "status": "recorded", "stamp": stamp_path.name}


def check_build_stamp(
    px4_root: Path,
    *,
    airframe: Path,
    binary: Path,
    stamp: Path | None = None,
    required_version: str = DEFAULT_REQUIRED_VERSION,
) -> dict[str, Any]:
    stamp_path = (stamp or _default_stamp_path(px4_root)).resolve()
    if not stamp_path.is_file():
        raise PreparationError(
            f"PX4 build stamp is missing: {stamp_path}; run once without --skip-px4-build"
        )
    try:
        recorded = json.loads(stamp_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PreparationError(f"PX4 build stamp is unreadable: {exc}") from exc
    expected = build_fingerprint(
        px4_root,
        airframe=airframe,
        binary=binary,
        required_version=required_version,
    )
    mismatches = {
        key: {"recorded": recorded.get(key), "current": value}
        for key, value in expected.items()
        if recorded.get(key) != value
    }
    if mismatches:
        raise PreparationError(
            "PX4 build stamp is stale; run without --skip-px4-build: "
            + json.dumps(mismatches, sort_keys=True)
        )
    return {**expected, "status": "verified", "stamp": stamp_path.name}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    prepare = subparsers.add_parser("prepare", help="inspect or apply the bridge mapping")
    prepare.add_argument("--px4-root", required=True, type=Path)
    prepare.add_argument("--required-version", default=DEFAULT_REQUIRED_VERSION)
    prepare.add_argument("--check", action="store_true", help="do not edit; fail if a patch is required")

    for name in ("record-build", "check-build"):
        command = subparsers.add_parser(name)
        command.add_argument("--px4-root", required=True, type=Path)
        command.add_argument("--airframe", required=True, type=Path)
        command.add_argument("--binary", required=True, type=Path)
        command.add_argument("--stamp", type=Path)
        command.add_argument("--required-version", default=DEFAULT_REQUIRED_VERSION)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "prepare":
            result = prepare_checkout(
                args.px4_root,
                required_version=args.required_version,
                apply=not args.check,
            )
            if args.check and result["status"] == "patch_required":
                print(json.dumps(result, ensure_ascii=False, sort_keys=True))
                return 1
        elif args.command == "record-build":
            result = write_build_stamp(
                args.px4_root,
                airframe=args.airframe,
                binary=args.binary,
                stamp=args.stamp,
                required_version=args.required_version,
            )
        else:
            result = check_build_stamp(
                args.px4_root,
                airframe=args.airframe,
                binary=args.binary,
                stamp=args.stamp,
                required_version=args.required_version,
            )
    except PreparationError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
