"""Workstation GNSS base serial, NTRIP, and discovery workers."""

from __future__ import annotations

import queue
import threading
import time

from PyQt5 import QtCore

from workstation_core.gnss_network import NtripClient
from workstation_core.gnss_operation import HeightType, NetworkConfig
from workstation_core.gnss_protocol import (
    Rtcm3StreamParser,
    UbxStreamParser,
    gnss_base_serial_port_candidates,
    parse_nmea_gga_satellite_count,
    parse_ubx_base_satellite_count,
    parse_ubx_nav_pvt_coordinate_sample,
    parse_ubx_nav_sat_quality,
    parse_ubx_nav_svin,
    probe_gnss_base_serial_ports,
    ubx_nav_pvt_poll_frame,
    ubx_nav_svin_poll_frame,
)


class GnssBaseSerialWorker(QtCore.QObject):
    connected = QtCore.pyqtSignal(str, int)
    stats_changed = QtCore.pyqtSignal(int, float, float)
    survey_in_changed = QtCore.pyqtSignal(object)
    satellite_count_changed = QtCore.pyqtSignal(int, str)
    satellite_quality_changed = QtCore.pyqtSignal(object)
    position_changed = QtCore.pyqtSignal(object)
    ubx_message_received = QtCore.pyqtSignal(int, int, object)
    rtcm_frame_received = QtCore.pyqtSignal(object)
    command_sent = QtCore.pyqtSignal(str, int)
    failed = QtCore.pyqtSignal(str)
    disconnected = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, port: str, baud: int, height_type: HeightType = HeightType.ELLIPSOID) -> None:
        super().__init__()
        self.port = port
        self.baud = int(baud)
        self.height_type = HeightType(height_type)
        self._stop_requested = False
        self._write_queue: queue.Queue[tuple[str, bytes]] = queue.Queue()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        serial_obj = None
        total_bytes = 0
        started_s = time.monotonic()
        last_emit_s = started_s
        last_rx_s: float | None = None
        last_nav_svin_poll_s = 0.0
        ubx_parser = UbxStreamParser()
        rtcm_parser = Rtcm3StreamParser()
        nmea_buffer = ""
        try:
            try:
                import serial  # type: ignore[import-not-found]
            except Exception as exc:
                self.failed.emit(f"pyserial 필요: {exc}")
                return
            serial_obj = serial.Serial(self.port, self.baud, timeout=0.2)
            self.connected.emit(self.port, self.baud)
            while not self._stop_requested:
                self._drain_writes(serial_obj)
                now_s = time.monotonic()
                if now_s - last_nav_svin_poll_s >= 2.0:
                    serial_obj.write(ubx_nav_svin_poll_frame())
                    serial_obj.write(ubx_nav_pvt_poll_frame())
                    try:
                        serial_obj.flush()
                    except Exception:
                        pass
                    last_nav_svin_poll_s = now_s
                chunk = serial_obj.read(4096)
                if chunk:
                    total_bytes += len(chunk)
                    last_rx_s = now_s
                    for rtcm_frame in rtcm_parser.feed(chunk):
                        self.rtcm_frame_received.emit(rtcm_frame)
                    nmea_buffer += chunk.decode("ascii", "ignore")
                    if "\n" in nmea_buffer:
                        lines = nmea_buffer.splitlines()
                        nmea_buffer = "" if nmea_buffer.endswith(("\n", "\r")) else lines.pop() if lines else nmea_buffer
                        for line in lines:
                            gga = parse_nmea_gga_satellite_count(line)
                            if gga is not None:
                                satellites, fix_quality = gga
                                self.satellite_count_changed.emit(satellites, f"NMEA-GGA fix={fix_quality}")
                    for message_class, message_id, payload in ubx_parser.feed(chunk):
                        self.ubx_message_received.emit(message_class, message_id, payload)
                        if message_class == 0x01 and message_id == 0x3B:
                            survey_in = parse_ubx_nav_svin(payload)
                            if survey_in is not None:
                                self.survey_in_changed.emit(survey_in)
                        if message_class == 0x01 and message_id == 0x07:
                            coordinate = parse_ubx_nav_pvt_coordinate_sample(
                                message_class,
                                message_id,
                                payload,
                                timestamp_s=now_s,
                                height_type=self.height_type,
                            )
                            if coordinate is not None:
                                self.position_changed.emit(coordinate)
                        satellite_count = parse_ubx_base_satellite_count(message_class, message_id, payload)
                        if satellite_count is not None:
                            source = "NAV-PVT" if message_id == 0x07 else "NAV-SAT"
                            self.satellite_count_changed.emit(satellite_count, source)
                        satellite_quality = parse_ubx_nav_sat_quality(message_class, message_id, payload)
                        if satellite_quality is not None:
                            self.satellite_quality_changed.emit(satellite_quality)
                if now_s - last_emit_s >= 0.5:
                    elapsed_s = max(1e-6, now_s - started_s)
                    rate_bps = total_bytes / elapsed_s
                    age_s = -1.0 if last_rx_s is None else max(0.0, now_s - last_rx_s)
                    self.stats_changed.emit(total_bytes, rate_bps, age_s)
                    last_emit_s = now_s
            self.disconnected.emit("disconnected")
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            if serial_obj is not None:
                try:
                    serial_obj.close()
                except Exception:
                    pass
            self.finished.emit()
    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_requested = True

    @QtCore.pyqtSlot(str, bytes)
    def send_bytes(self, label: str, payload: bytes) -> None:
        self._write_queue.put((label, bytes(payload)))

    def _drain_writes(self, serial_obj) -> None:
        while True:
            try:
                label, payload = self._write_queue.get_nowait()
            except queue.Empty:
                return
            serial_obj.write(payload)
            try:
                serial_obj.flush()
            except Exception:
                pass
            self.command_sent.emit(label, len(payload))


class NtripNetworkWorker(QtCore.QObject):
    """Qt adapter for the blocking, dependency-free NTRIP client."""

    connected = QtCore.pyqtSignal(str, object)
    chunk_received = QtCore.pyqtSignal(object)
    reconnecting = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, config: NetworkConfig, gga_provider=None) -> None:
        super().__init__()
        self.config = config
        self.gga_provider = gga_provider
        self._stop_event = threading.Event()
        self._client: NtripClient | None = None

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            while not self._stop_event.is_set():
                client = NtripClient(self.config)
                self._client = client
                try:
                    initial_gga = self.gga_provider() if self.gga_provider is not None else None
                    status_line, initial_payload = client.connect(gga_sentence=initial_gga)
                    self.connected.emit(status_line, client.stats)
                    for chunk in client.iter_chunks(
                        self._stop_event,
                        gga_provider=self.gga_provider,
                        initial_payload=initial_payload,
                    ):
                        self.chunk_received.emit(chunk)
                    if self._stop_event.is_set():
                        break
                    raise RuntimeError("NTRIP stream ended without a stop request")
                except Exception as exc:
                    if self._stop_event.is_set():
                        break
                    self.reconnecting.emit(str(exc))
                    # The failed client is closed before retrying.  A fresh
                    # client owns the interruptible reconnect delay because a
                    # closed client intentionally never retries.
                    client.close()
                    self._client = None
                    retry_waiter = NtripClient(self.config)
                    if not retry_waiter.wait_for_reconnect(self._stop_event):
                        break
                    continue
                finally:
                    client.close()
                    if self._client is client:
                        self._client = None
        finally:
            self._client = None
            self.finished.emit()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_event.set()
        client = self._client
        if client is not None:
            client.close()


class GnssBaseSerialDetectWorker(QtCore.QObject):
    detected = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, baud: int) -> None:
        super().__init__()
        self.baud = int(baud)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            candidates = gnss_base_serial_port_candidates()
            rows = probe_gnss_base_serial_ports(candidates, baud=self.baud, timeout_s=0.6)
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.detected.emit(rows)
        finally:
            self.finished.emit()
