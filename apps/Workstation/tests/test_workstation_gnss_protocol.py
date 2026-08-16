from pathlib import Path
import struct

import pytest

from Workstation.workstation_core.gnss_operation import HeightType
from Workstation.workstation_core.gnss_protocol import (
    Rtcm3StreamParser,
    UbxStreamParser,
    _crc24q,
    parse_rtcm3_message_type,
    parse_ubx_cfg_tmode3,
    parse_ubx_nav_pvt_coordinate_sample,
    preferred_gnss_base_serial_port,
    ubx_cfg_tmode3_fixed_frame,
    ubx_cfg_tmode3_survey_in_frame,
    ubx_frame,
)


def _rtcm3_frame(payload: bytes) -> bytes:
    header = bytes((0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF)) + payload
    return header + _crc24q(header).to_bytes(3, "big")


def test_workstation_protocol_is_owned_outside_the_qt_panel_and_reexported():
    from Workstation import standalone_gui

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


def test_cfg_tmode3_trailing_u4_fields_land_at_their_real_ublox_offsets():
    # UBX-CFG-TMODE3's three trailing U4 fields are, in wire order,
    # fixedPosAcc (offset 20) / svinMinDur (offset 24) / svinAccLimit
    # (offset 28) -- independent of this module's own parser, so a bug that
    # shifts the encoder and parser together in the same wrong direction
    # can't hide from this test the way it hid from a pure round-trip check.
    fixed_payload = ubx_cfg_tmode3_fixed_frame(
        latitude_deg=37.0, longitude_deg=127.0, ellipsoid_height_m=52.0, fixed_position_accuracy_m=0.05,
    )[6:-2]
    assert struct.unpack_from("<I", fixed_payload, 20)[0] == 500  # fixedPosAcc
    assert struct.unpack_from("<I", fixed_payload, 24)[0] == 0  # svinMinDur (unused)
    assert struct.unpack_from("<I", fixed_payload, 28)[0] == 0  # svinAccLimit (unused)

    survey_payload = ubx_cfg_tmode3_survey_in_frame(min_duration_s=120, accuracy_m=0.1)[6:-2]
    assert struct.unpack_from("<I", survey_payload, 20)[0] == 0  # fixedPosAcc (unused)
    assert struct.unpack_from("<I", survey_payload, 24)[0] == 120  # svinMinDur
    assert struct.unpack_from("<I", survey_payload, 28)[0] == 1000  # svinAccLimit


def test_parse_cfg_tmode3_reads_fixed_pos_acc_from_the_spec_slot():
    # Construct a receiver payload independently of the frame builder. This
    # catches a parser that still reads svinAccLimit as fixedPosAcc.
    payload = struct.pack(
        "<BBHlllbbbBIII8s",
        0,
        0,
        2,
        100,
        200,
        300,
        1,
        -2,
        3,
        0,
        501,   # fixedPosAcc, offset 20
        120,   # svinMinDur, offset 24
        1000,  # svinAccLimit, offset 28
        b"\x00" * 8,
    )

    decoded = parse_ubx_cfg_tmode3(payload)

    assert decoded is not None
    assert decoded["fixed_accuracy_0_1mm"] == 501


def test_preferred_base_port_promotes_u_blox_by_id_for_generic_default(monkeypatch):
    import Workstation.workstation_core.gnss_protocol as protocol

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
    frame = _rtcm3_frame(payload)
    parser = Rtcm3StreamParser()

    assert parser.feed(frame[:4]) == []
    assert parser.feed(frame[4:]) == [frame]
    assert parse_rtcm3_message_type(frame) == 1077
    assert UbxStreamParser().feed(ubx_frame(1, 2, b"abc")) == [(1, 2, b"abc")]


def test_rtcm_parser_rejects_bad_crc_and_resyncs_on_the_next_frame():
    good_payload = bytes((0x43, 0x50, 0))
    good_frame = _rtcm3_frame(good_payload)
    corrupted = bytearray(good_frame)
    corrupted[-1] ^= 0xFF  # flip the trailing CRC byte so it no longer matches
    parser = Rtcm3StreamParser()

    frames = parser.feed(bytes(corrupted) + good_frame)

    assert frames == [good_frame]
