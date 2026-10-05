#!/usr/bin/env python3
"""Hash a target SDF and its size/geometry contract independently."""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _text(element: ET.Element, path: str) -> str:
    value = element.findtext(path)
    return "" if value is None else value.strip()


def _element_payload(element: ET.Element | None) -> Any:
    if element is None:
        return None
    return {
        "tag": element.tag,
        "attributes": dict(sorted(element.attrib.items())),
        "text": (element.text or "").strip(),
        "children": [_element_payload(child) for child in element],
    }


def protected_geometry_payload(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    model = root.find("model")
    if model is None:
        raise ValueError(f"SDF has no model element: {path}")

    links = []
    for link in model.findall("link"):
        links.append(
            {
                "name": link.get("name", ""),
                "pose": _text(link, "pose"),
                "visuals": [
                    {
                        "name": item.get("name", ""),
                        "pose": _text(item, "pose"),
                        "geometry": _element_payload(item.find("geometry")),
                    }
                    for item in link.findall("visual")
                ],
                "collisions": [
                    {
                        "name": item.get("name", ""),
                        "pose": _text(item, "pose"),
                        "geometry": _element_payload(item.find("geometry")),
                    }
                    for item in link.findall("collision")
                ],
            }
        )

    joints = []
    for joint in model.findall("joint"):
        joints.append(
            {
                "name": joint.get("name", ""),
                "type": joint.get("type", ""),
                "pose": _text(joint, "pose"),
                "parent": _text(joint, "parent"),
                "child": _text(joint, "child"),
                "axis_xyz": _text(joint, "axis/xyz"),
            }
        )

    frames = [
        {
            "name": frame.get("name", ""),
            "attached_to": frame.get("attached_to", ""),
            "pose": _text(frame, "pose"),
        }
        for frame in model.findall("frame")
    ]
    motor_bindings = [
        {
            "name": plugin.get("name", ""),
            "filename": plugin.get("filename", ""),
            "joint_name": _text(plugin, "jointName"),
            "link_name": _text(plugin, "linkName"),
            "motor_number": _text(plugin, "motorNumber"),
            "turning_direction": _text(plugin, "turningDirection"),
            "command_topic": _text(plugin, "commandSubTopic"),
        }
        for plugin in model.findall("plugin")
        if plugin.find("motorNumber") is not None
    ]
    return {
        "schema_version": "sdf_protected_geometry_v1",
        "model_name": model.get("name", ""),
        "model_pose": _text(model, "pose"),
        "links": links,
        "joints": joints,
        "frames": frames,
        "motor_bindings": motor_bindings,
    }


def protected_geometry_sha256(path: Path) -> str:
    payload = protected_geometry_payload(path)
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def sdf_contract(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    return {
        "schema_version": "sdf_contract_v1",
        "path": str(path),
        "sha256": sha256_file(path),
        "protected_geometry_sha256": protected_geometry_sha256(path),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", type=Path, required=True)
    parser.add_argument(
        "--field",
        choices=("sha256", "protected_geometry_sha256", "path"),
    )
    args = parser.parse_args()
    payload = sdf_contract(args.sdf)
    if args.field:
        print(payload[args.field])
    else:
        print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
