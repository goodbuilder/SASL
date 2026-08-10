from pathlib import Path
import struct

import pytest

from workstation_core.gnss_operation import HeightType
from workstation_core.gnss_protocol import (
    Rtcm3StreamParser,
    UbxStreamParser,
    parse_rtcm3_message_type,
    parse_ubx_cfg_tmode3,
    parse_ubx_nav_pvt_coordinate_sample,
    preferred_gnss_base_serial_port,
    ubx_cfg_tmode3_fixed_frame,
    ubx_frame,
)


def test_workstation_protocol_is_owned_outside_the_qt_panel_and_reexported():
    from workstation import standalone_gui

    source = Path(standalone_gui.__file__).read_text(encoding="utf-8")
    assert "def ubx_frame(" not in source
    assert "class UbxStreamParser" not in source
    assert standalone_gui.ubx_frame is ubx_frame
    assert standalone_gui.UbxStreamParser is UbxStreamParser


def test_fixed_base_frame_retains_wgs84_ecef_contract():
    parser = UbxStreamParser()
    messages = parser.feed(
        ubx_cfg_tmode3_fixed_frame(
            latitude_deg=37.0,
            longitude_deg=127.0,
            ellipsoid_height_m=52.0,
            fixed_position_accuracy_m=0.05,
        )
    )

    assert len(messages) == 1
    message_class, message_id, payload = messages[0]
    assert (message_class, message_id) == (0x06, 0x71)
    decoded = parse_ubx_cfg_tmode3(payload)
    assert decoded is not None
    assert decoded["mode"] == 2
    assert decoded["ecef_cm"] != (0, 0, 0)
    assert decoded["fixed_accuracy_0_1mm"] == 500


def test_preferred_base_port_promotes_u_blox_by_id_for_generic_default(monkeypatch):
    import workstation_core.gnss_protocol as protocol

    stable = "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
    monkeypatch.setattr(protocol, "gnss_base_serial_port_candidates", lambda: ["/dev/ttyACM0", stable])

    assert preferred_gnss_base_serial_port("/dev/ttyACM0") == stable
    assert preferred_gnss_base_serial_port("/dev/custom-gnss") == "/dev/custom-gnss"


def test_nav_pvt_keeps_geodetic_heights_distinct_from_px4_fix_type():
    payload = bytearray(92)
    payload[20] = 3
    payload[21] = 0x81
    struct.pack_into("<i", payload, 24, int(127.0 * 1e7))
    struct.pack_into("<i", payload, 28, int(37.0 * 1e7))
    struct.pack_into("<i", payload, 32, 52_000)
    struct.pack_into("<i", payload, 36, 48_000)

    sample = parse_ubx_nav_pvt_coordinate_sample(
        0x01, 0x07, bytes(payload), timestamp_s=10.0,
        height_type=HeightType.ELLIPSOID,
    )

    assert sample is not None and sample.fixed is True
    assert sample.ellipsoid_height_m == pytest.approx(52.0)
    assert sample.msl_altitude_m == pytest.approx(48.0)
    payload[20] = 6
    assert parse_ubx_nav_pvt_coordinate_sample(
        0x01, 0x07, bytes(payload), timestamp_s=10.0,
        height_type=HeightType.ELLIPSOID,
    ).fixed is False


def test_rtcm_parser_and_message_identity_remain_transport_only():
    message_type = 1077
    payload = bytes((message_type >> 4, (message_type & 0x0F) << 4, 0))
    frame = b"\xd3\x00\x03" + payload + b"\x00\x00\x00"
    parser = Rtcm3StreamParser()

    assert parser.feed(frame[:4]) == []
    assert parser.feed(frame[4:]) == [frame]
    assert parse_rtcm3_message_type(frame) == 1077
    assert UbxStreamParser().feed(ubx_frame(1, 2, b"abc")) == [(1, 2, b"abc")]
