"""Workstation-owned UBX/RTCM protocol and field-base serial probing.

Coordinate contract:
- fixed-base input is WGS84 latitude/longitude plus ellipsoid height in metres;
- CFG-TMODE3 wire coordinates are WGS84 ECEF centimetres plus signed 0.1 mm;
- NAV-PVT fields retain geodetic/ellipsoid/MSL meanings;
- no value in this module is a PX4 NED, body FRD, or camera-frame vector.
"""

from collections.abc import Sequence
import os
from pathlib import Path
import struct
import time

from Workstation.workstation_core.gnss_operation import HeightType
from Workstation.workstation_core.gnss_profile import GnssCoordinateSample, geodetic_to_ecef_m


GNSS_BASE_SURVEY_TREND_WINDOW_S = 60.0
GNSS_BASE_SURVEY_STAGNANT_DELTA_M = 0.01
GNSS_BASE_SURVEY_IMPROVING_DELTA_M = 0.05
GNSS_BASE_LOW_CNO_DBHZ = 30
GNSS_BASE_LOW_ELEVATION_DEG = 15
GNSS_ID_LABELS = {
    0: "GPS",
    1: "SBAS",
    2: "Galileo",
    3: "BeiDou",
    5: "QZSS",
    6: "GLONASS",
    7: "NavIC",
}


def _ubx_checksum(data: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for value in data:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def ubx_frame(message_class: int, message_id: int, payload: bytes) -> bytes:
    header = bytes((message_class & 0xFF, message_id & 0xFF)) + len(payload).to_bytes(2, "little")
    ck_a, ck_b = _ubx_checksum(header + payload)
    return b"\xb5\x62" + header + payload + bytes((ck_a, ck_b))


def ubx_cfg_tmode3_survey_in_frame(*, min_duration_s: int, accuracy_m: float) -> bytes:
    min_duration_s = max(1, int(min_duration_s))
    accuracy_0_1mm = max(1, int(round(float(accuracy_m) / 0.0001)))
    payload = struct.pack(
        "<BBHlllbbbBIII8s",
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        min_duration_s,
        accuracy_0_1mm,
        b"\x00" * 8,
    )
    return ubx_frame(0x06, 0x71, payload)


def ubx_cfg_tmode3_disabled_frame() -> bytes:
    payload = struct.pack(
        "<BBHlllbbbBIII8s",
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        b"\x00" * 8,
    )
    return ubx_frame(0x06, 0x71, payload)


def _ubx_tmode3_ecef_components(value_m: float) -> tuple[int, int]:
    """Encode ECEF metres as CFG-TMODE3 centimetres plus 0.1 mm high part."""

    coarse_cm = int(round(float(value_m) * 100.0))
    high_part = int(round((float(value_m) - coarse_cm * 0.01) / 0.0001))
    while high_part > 99:
        coarse_cm += 1
        high_part -= 100
    while high_part < -99:
        coarse_cm -= 1
        high_part += 100
    return coarse_cm, high_part


def ubx_cfg_tmode3_fixed_frame(
    *,
    latitude_deg: float,
    longitude_deg: float,
    ellipsoid_height_m: float,
    fixed_position_accuracy_m: float = 0.05,
) -> bytes:
    """Build UBX-CFG-TMODE3 fixed-base command from WGS84 geodetic data.

    The receiver command uses ECEF centimetres and 0.1 mm high parts.  The
    input height is explicitly ellipsoid height; MSL values must be converted
    with a geoid model before calling this function.
    """

    ecef = geodetic_to_ecef_m(latitude_deg, longitude_deg, ellipsoid_height_m)
    coarse_x, high_x = _ubx_tmode3_ecef_components(ecef[0])
    coarse_y, high_y = _ubx_tmode3_ecef_components(ecef[1])
    coarse_z, high_z = _ubx_tmode3_ecef_components(ecef[2])
    accuracy_0_1mm = max(1, int(round(float(fixed_position_accuracy_m) / 0.0001)))
    # CFG-TMODE3's three trailing U4 fields are, in wire order, fixedPosAcc /
    # svinMinDur / svinAccLimit. Fixed-base mode only uses fixedPosAcc (the
    # other two are survey-in-only), so it goes first, not last.
    payload = struct.pack(
        "<BBHlllbbbBIII8s",
        0,
        0,
        2,
        coarse_x,
        coarse_y,
        coarse_z,
        high_x,
        high_y,
        high_z,
        0,
        accuracy_0_1mm,
        0,
        0,
        b"\x00" * 8,
    )
    return ubx_frame(0x06, 0x71, payload)


def ubx_poll_frame(message_class: int, message_id: int) -> bytes:
    return ubx_frame(message_class, message_id, b"")


def ubx_nav_svin_poll_frame() -> bytes:
    return ubx_poll_frame(0x01, 0x3B)


def ubx_nav_pvt_poll_frame() -> bytes:
    return ubx_poll_frame(0x01, 0x07)


def ubx_cfg_tmode3_poll_frame() -> bytes:
    """Poll the receiver's active CFG-TMODE3 configuration."""

    return ubx_poll_frame(0x06, 0x71)


def parse_ubx_ack(message_class: int, message_id: int, payload: bytes) -> tuple[bool, int, int] | None:
    """Return ``(accepted, target_class, target_id)`` for UBX ACK messages."""

    if message_class != 0x05 or message_id not in {0x00, 0x01} or len(payload) < 2:
        return None
    return message_id == 0x01, int(payload[0]), int(payload[1])


def parse_ubx_cfg_tmode3(payload: bytes) -> dict[str, object] | None:
    """Decode the fixed/survey-in ECEF representation used by CFG-TMODE3.

    Coordinate contract: the three coordinate components are WGS84 ECEF
    centimetres plus signed 0.1 mm high parts.  They are intentionally never
    treated as rover NED or camera-frame values.  The trailing U4 fields are
    fixedPosAcc at payload offset 20, svinMinDur at offset 24, and
    svinAccLimit at offset 28; fixed_accuracy_0_1mm always names the first of
    those fields.
    """

    if len(payload) < 40:
        return None
    # Wire order of the trailing U4 fields is fixedPosAcc / svinMinDur /
    # svinAccLimit -- match ubx_cfg_tmode3_fixed_frame's field order above.
    (
        _version,
        _reserved0,
        flags,
        x_cm,
        y_cm,
        z_cm,
        x_hp,
        y_hp,
        z_hp,
        _reserved1,
        fixed_accuracy_0_1mm,
        _svin_min_duration_s,
        _svin_accuracy_0_1mm,
        _reserved2,
    ) = struct.unpack_from("<BBHlllbbbBIII8s", payload, 0)
    return {
        "mode": int(flags) & 0xFF,
        "ecef_cm": (int(x_cm), int(y_cm), int(z_cm)),
        "ecef_hp_0_1mm": (int(x_hp), int(y_hp), int(z_hp)),
        "fixed_accuracy_0_1mm": int(fixed_accuracy_0_1mm),
    }


def parse_ubx_nav_pvt_coordinate_sample(
    message_class: int,
    message_id: int,
    payload: bytes,
    *,
    timestamp_s: float,
    height_type: HeightType = HeightType.ELLIPSOID,
) -> GnssCoordinateSample | None:
    """Parse UBX-NAV-PVT into the WGS84 coordinate contract used by fixed-base."""

    if message_class != 0x01 or message_id != 0x07 or len(payload) < 48:
        return None
    fix_type = int(payload[20])
    flags = int(payload[21])
    latitude_deg = struct.unpack_from("<i", payload, 28)[0] * 1e-7
    longitude_deg = struct.unpack_from("<i", payload, 24)[0] * 1e-7
    ellipsoid_height_m = struct.unpack_from("<i", payload, 32)[0] * 1e-3
    msl_altitude_m = struct.unpack_from("<i", payload, 36)[0] * 1e-3
    gnss_fix_ok = bool(flags & 0x01)
    carrier_solution = (flags >> 6) & 0x03
    return GnssCoordinateSample(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        ellipsoid_height_m=ellipsoid_height_m,
        msl_altitude_m=msl_altitude_m,
        height_type=height_type,
        timestamp_s=float(timestamp_s),
        # Raw u-blox NAV-PVT is not PX4 GPS_RAW_INT: RTK fixed is encoded by
        # gnssFixOK plus carrSoln=2, while fixType remains a 3D/GNSS+DR value.
        fixed=fix_type in {3, 4} and gnss_fix_ok and carrier_solution == 2,
        device_id="field-base",
        fix_type=fix_type,
        carrier_solution=carrier_solution,
    )


def parse_ubx_nav_svin(payload: bytes) -> dict[str, object] | None:
    if len(payload) < 40:
        return None
    (
        _version,
        _reserved0,
        _itow_ms,
        duration_s,
        mean_x_cm,
        mean_y_cm,
        mean_z_cm,
        mean_x_hp_0_1mm,
        mean_y_hp_0_1mm,
        mean_z_hp_0_1mm,
        _reserved1,
        mean_acc_0_1mm,
        observations,
        valid,
        active,
        _reserved2,
    ) = struct.unpack_from("<B3sLLlllbbbBLLBBH", payload, 0)
    return {
        "duration_s": int(duration_s),
        "mean_accuracy_m": float(mean_acc_0_1mm) * 0.0001,
        "observations": int(observations),
        "valid": bool(valid),
        "active": bool(active),
        "mean_x_m": float(mean_x_cm) * 0.01 + float(mean_x_hp_0_1mm) * 0.0001,
        "mean_y_m": float(mean_y_cm) * 0.01 + float(mean_y_hp_0_1mm) * 0.0001,
        "mean_z_m": float(mean_z_cm) * 0.01 + float(mean_z_hp_0_1mm) * 0.0001,
    }


def parse_ubx_base_satellite_count(message_class: int, message_id: int, payload: bytes) -> int | None:
    if message_class != 0x01:
        return None
    if message_id == 0x07 and len(payload) >= 24:
        return int(payload[23])
    if message_id == 0x35 and len(payload) >= 8:
        return int(payload[5])
    return None


def parse_ubx_nav_sat_quality(message_class: int, message_id: int, payload: bytes) -> dict[str, object] | None:
    if message_class != 0x01 or message_id != 0x35 or len(payload) < 8:
        return None
    reported_count = int(payload[5])
    available_count = max(0, (len(payload) - 8) // 12)
    record_count = min(reported_count, available_count)
    if record_count <= 0:
        return {
            "tracked": 0,
            "used": 0,
            "avg_cno_dbhz": None,
            "low_cno": 0,
            "low_elevation": 0,
            "constellations": {},
        }
    cno_values: list[int] = []
    used = 0
    low_cno = 0
    low_elevation = 0
    constellations: dict[str, int] = {}
    for index in range(record_count):
        offset = 8 + index * 12
        gnss_id = int(payload[offset])
        cno = int(payload[offset + 2])
        elevation = struct.unpack_from("<b", payload, offset + 3)[0]
        flags = struct.unpack_from("<I", payload, offset + 8)[0]
        label = GNSS_ID_LABELS.get(gnss_id, f"GNSS{gnss_id}")
        constellations[label] = constellations.get(label, 0) + 1
        if flags & 0x01:
            used += 1
        if cno > 0:
            cno_values.append(cno)
            if cno < GNSS_BASE_LOW_CNO_DBHZ:
                low_cno += 1
        if elevation < GNSS_BASE_LOW_ELEVATION_DEG:
            low_elevation += 1
    avg_cno = sum(cno_values) / len(cno_values) if cno_values else None
    return {
        "tracked": record_count,
        "used": used,
        "avg_cno_dbhz": avg_cno,
        "low_cno": low_cno,
        "low_elevation": low_elevation,
        "constellations": constellations,
    }


def parse_nmea_gga_satellite_count(line: str) -> tuple[int, int] | None:
    text = line.strip()
    if not text.startswith(("$GNGGA", "$GPGGA", "$GAGGA", "$GLGGA", "$GBGGA")):
        return None
    if "*" in text:
        body, checksum_text = text[1:].split("*", 1)
        checksum = 0
        for char in body:
            checksum ^= ord(char)
        try:
            expected = int(checksum_text[:2], 16)
        except ValueError:
            return None
        if checksum != expected:
            return None
        fields = body.split(",")
    else:
        fields = text[1:].split(",")
    if len(fields) <= 7:
        return None
    try:
        fix_quality = int(fields[6] or "0")
        satellites = int(fields[7] or "0")
    except ValueError:
        return None
    return satellites, fix_quality


def parse_rtcm3_message_type(frame: bytes) -> int | None:
    if len(frame) < 6 or frame[0] != 0xD3:
        return None
    payload_len = ((frame[1] & 0x03) << 8) | frame[2]
    if payload_len < 2 or len(frame) < 3 + payload_len + 3:
        return None
    return ((int(frame[3]) << 4) | (int(frame[4]) >> 4)) & 0x0FFF


RTCM_REFERENCE_STATION_MESSAGE_TYPES = (1005, 1006)


def parse_rtcm3_reference_station_arp(
    frame: bytes,
) -> tuple[int, tuple[int, int, int]] | None:
    """Return ``(station_id, ECEF)`` from an RTCM 1005/1006 frame.

    The ECEF triple stays in the wire unit of 0.1 mm as exact integers, so two
    reports can be compared without reintroducing float noise.  Returns
    ``None`` for any other message type or a truncated frame.
    """

    if len(frame) < 6 or frame[0] != 0xD3:
        return None
    payload_len = ((frame[1] & 0x03) << 8) | frame[2]
    if payload_len < 19 or len(frame) < 3 + payload_len + 3:
        return None
    payload = frame[3 : 3 + payload_len]

    def bits(offset: int, length: int) -> int:
        value = 0
        for index in range(offset, offset + length):
            value = (value << 1) | ((payload[index // 8] >> (7 - index % 8)) & 1)
        return value

    def signed(offset: int, length: int) -> int:
        value = bits(offset, length)
        return value - (1 << length) if value & (1 << (length - 1)) else value

    if bits(0, 12) not in RTCM_REFERENCE_STATION_MESSAGE_TYPES:
        return None
    station_id = bits(12, 12)
    return station_id, (signed(34, 38), signed(74, 38), signed(114, 38))


def gnss_base_serial_port_candidates() -> list[str]:
    candidates: list[str] = []
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        for path in sorted(Path("/").glob(pattern.removeprefix("/"))):
            text = str(path)
            if text not in candidates:
                candidates.append(text)
    return candidates


def preferred_gnss_base_serial_port(configured_port: str = "") -> str:
    """Choose a stable u-blox USB path when the configured port is generic.

    The by-id path is preferred only for the legacy/default ``ttyACM0`` or an
    empty setting.  An operator-selected custom port remains authoritative.
    """

    configured = str(configured_port).strip()
    candidates = gnss_base_serial_port_candidates()
    ublox_paths = [
        path for path in candidates
        if "/dev/serial/by-id/" in path.lower() and "u-blox" in path.lower()
    ]
    if ublox_paths and configured in {"", "/dev/ttyACM0"}:
        return ublox_paths[0]
    if configured:
        return configured
    return candidates[0] if candidates else ""


def probe_gnss_base_serial_ports(candidates: Sequence[str], *, baud: int, timeout_s: float = 0.6) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    try:
        import serial  # type: ignore[import-not-found]
    except Exception as exc:
        return [
            {
                "port": str(candidate),
                "target": os.path.realpath(str(candidate)),
                "baud": int(baud),
                "status": f"pyserial 필요: {exc}",
                "ubx": False,
                "messages": "",
            }
            for candidate in candidates
        ]
    for candidate in candidates:
        port = str(candidate)
        parser = UbxStreamParser()
        nmea_buffer = ""
        messages_seen: set[str] = set()
        total_bytes = 0
        status = "no data"
        ubx_detected = False
        try:
            with serial.Serial(port, int(baud), timeout=0.1) as serial_obj:
                deadline = time.monotonic() + max(0.1, float(timeout_s))
                while time.monotonic() < deadline:
                    chunk = serial_obj.read(4096)
                    if not chunk:
                        continue
                    total_bytes += len(chunk)
                    nmea_buffer += chunk.decode("ascii", "ignore")
                    if "\n" in nmea_buffer:
                        lines = nmea_buffer.splitlines()
                        nmea_buffer = "" if nmea_buffer.endswith(("\n", "\r")) else lines.pop() if lines else nmea_buffer
                        for line in lines:
                            gga = parse_nmea_gga_satellite_count(line)
                            if gga is not None:
                                satellites, fix_quality = gga
                                messages_seen.add(f"NMEA-GGA sats={satellites} fix={fix_quality}")
                    for message_class, message_id, payload in parser.feed(chunk):
                        ubx_detected = True
                        if message_class == 0x01 and message_id == 0x3B:
                            messages_seen.add("NAV-SVIN")
                        elif message_class == 0x01 and message_id == 0x07:
                            sat_count = parse_ubx_base_satellite_count(message_class, message_id, payload)
                            messages_seen.add("NAV-PVT" if sat_count is None else f"NAV-PVT sats={sat_count}")
                        elif message_class == 0x01 and message_id == 0x35:
                            sat_count = parse_ubx_base_satellite_count(message_class, message_id, payload)
                            messages_seen.add("NAV-SAT" if sat_count is None else f"NAV-SAT sats={sat_count}")
                        else:
                            messages_seen.add(f"UBX {message_class:02X}/{message_id:02X}")
                    if messages_seen:
                        break
            if messages_seen:
                status = "GNSS data detected"
            elif total_bytes:
                status = f"raw data only ({total_bytes} bytes)"
        except Exception as exc:
            status = str(exc)
        rows.append(
            {
                "port": port,
                "target": os.path.realpath(port),
                "baud": int(baud),
                "status": status,
                "ubx": ubx_detected,
                "messages": ", ".join(sorted(messages_seen)),
            }
        )
    return rows


class UbxStreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        self._buffer.extend(data)
        messages: list[tuple[int, int, bytes]] = []
        while True:
            sync_index = self._buffer.find(b"\xb5\x62")
            if sync_index < 0:
                if len(self._buffer) > 1:
                    del self._buffer[:-1]
                break
            if sync_index:
                del self._buffer[:sync_index]
            if len(self._buffer) < 8:
                break
            payload_len = int.from_bytes(self._buffer[4:6], "little")
            frame_len = 8 + payload_len
            if len(self._buffer) < frame_len:
                break
            frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]
            ck_a, ck_b = _ubx_checksum(frame[2:-2])
            if ck_a != frame[-2] or ck_b != frame[-1]:
                continue
            messages.append((frame[2], frame[3], frame[6:-2]))
        return messages


def _crc24q(data: bytes) -> int:
    """RTCM3's CRC-24Q (Qualcomm) checksum: poly 0x1864CFB, init 0, MSB-first."""

    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        crc &= 0xFFFFFF
    return crc


class Rtcm3StreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self._buffer.extend(data)
        frames: list[bytes] = []
        while True:
            sync_index = self._buffer.find(b"\xd3")
            if sync_index < 0:
                self._buffer.clear()
                break
            if sync_index:
                del self._buffer[:sync_index]
            if len(self._buffer) < 3:
                break
            if self._buffer[1] & 0xFC:
                del self._buffer[0]
                continue
            payload_len = ((self._buffer[1] & 0x03) << 8) | self._buffer[2]
            frame_len = 3 + payload_len + 3
            if len(self._buffer) < frame_len:
                break
            frame = bytes(self._buffer[:frame_len])
            if _crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
                # A corrupt length field can make this candidate encompass a
                # complete valid frame that follows the false sync byte.  Drop
                # only that sync byte so the next pass can rescan every
                # remaining byte for 0xD3; deleting ``frame_len`` bytes here
                # would silently discard the enclosed valid correction.
                del self._buffer[0]
                continue
            del self._buffer[:frame_len]
            frames.append(frame)
        return frames
