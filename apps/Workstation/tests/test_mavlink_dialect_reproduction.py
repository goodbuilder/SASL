"""Original firmware XML reproduces the vendored dialect and raw CAN contract."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
import struct
from xml.etree import ElementTree

import pytest

from Workstation.tools.regenerate_mavlink_dialect import (
    DEFINITIONS,
    DIALECT_ROOT,
    MANIFEST_PATH,
    check,
    generate_bytes,
    verify_inputs,
)


def test_repo_inputs_regenerate_every_runtime_byte_without_external_checkout():
    from pymavlink.generator import mavgen

    previous_schema = mavgen.schemaFile
    assert check() == "7ed861c4d01a48e1c972cd1936b4410e29f209f1e428178f704564868f4ea2ba"
    assert mavgen.schemaFile == previous_schema


def test_changed_input_fails_before_generation(tmp_path):
    copied = tmp_path / "definitions"
    shutil.copytree(DEFINITIONS, copied)
    with (copied / "common.xml").open("a") as handle:
        handle.write("\n")
    with pytest.raises(ValueError, match="source hash mismatch: common.xml"):
        verify_inputs(copied, json.loads(MANIFEST_PATH.read_text()))


def test_generation_keeps_xml_validation_enabled_for_intentionally_revised_inputs(tmp_path):
    copied = tmp_path / "definitions"
    shutil.copytree(DEFINITIONS, copied)
    common = copied / "common.xml"
    common.write_text(
        common.read_text().replace('<message id="180"', '<message invalid_attribute="forbidden" id="180"')
    )
    manifest = json.loads(MANIFEST_PATH.read_text())
    manifest["inputs"]["common.xml"] = hashlib.sha256(common.read_bytes()).hexdigest()
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(RuntimeError, match="XML validation enabled"):
        generate_bytes(definitions=copied, manifest_path=manifest_path)


def test_generated_can_sample_matches_independent_payload_and_golden_wire_bytes(tmp_path):
    destination = tmp_path / "regenerated.py"
    destination.write_bytes(generate_bytes())
    spec = importlib.util.spec_from_file_location("regenerated_sasl_dialect", destination)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    mav = module.MAVLink(None, srcSystem=7, srcComponent=8)
    packet = module.MAVLink_sasl_can_mag_message(123456789, 0x10203, 2, 42, 1.25, -2.5, 0.125).pack(mav)
    assert packet.hex() == "fd1a0000000708b4000015cd5b0700000000030201000000a03f000020c00000003e022af704"
    assert struct.unpack("<QIfffBB", packet[10:-2]) == (123456789, 0x10203, 1.25, -2.5, 0.125, 2, 42)
    assert module.MAVLink_sasl_can_mag_message.crc_extra == 233
    assert len(module.mavlink_map) == 235
    message = ElementTree.parse(DEFINITIONS / "common.xml").find('messages/message[@id="180"]')
    assert [field.attrib["name"] for field in message.findall("field")] == [
        "time_usec",
        "device_id",
        "can_port",
        "node_id",
        "xmag",
        "ymag",
        "zmag",
    ]
    assert [field.attrib["units"] for field in message.findall("field") if field.attrib["name"].endswith("mag")] == [
        "Gauss"
    ] * 3
    assert "sensor-frame" in message.find("description").text
    manifest = json.loads(MANIFEST_PATH.read_text())
    assert (DIALECT_ROOT / manifest["provenance"]["patch"]).resolve().is_file()
