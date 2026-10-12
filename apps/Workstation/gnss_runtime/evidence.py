"""Measurement identity to immutable RTCM evidence context and drain handoff.

Only the outside adapter resolves recording paths and package fields. This
owner preserves original context tokens, session attribution, UTC/monotonic
receive clocks and source/pipeline sequence high-water contracts.
"""

import time
from dataclasses import dataclass
from pathlib import Path

from Workstation.workstation_core.gnss_protocol import parse_rtcm3_message_type
from Workstation.workstation_core.rtcm_pipeline import (
    RtcmEvidenceContext,
    RtcmEvidenceRecord,
    append_rtcm_evidence,
    load_rtcm_evidence_high_water,
)
from Workstation.workstation_core.session_storage.spool import SpoolRows


@dataclass(frozen=True)
class GnssEvidenceRowsPort:
    """Only the Recorder-owned GNSS row collection, including replacements."""

    read: object
    replace: object

    def __post_init__(self):
        if not callable(self.read) or not callable(self.replace):
            raise TypeError("GNSS evidence requires Recorder row read/replace operations")


class GnssEvidenceState:
    """GNSS row view; the Recorder remains the production storage owner.

    Standalone legacy adapters have no Recorder and retain their local backing
    collection. Production GnssRuntimeState requires an explicit Recorder port.
    """

    def __init__(self, rows_port=None):
        if rows_port is None:
            self._standalone_rows = SpoolRows()
            rows_port = GnssEvidenceRowsPort(
                lambda: self._standalone_rows,
                lambda rows: setattr(self, "_standalone_rows", rows),
            )
        self._rows_port = rows_port

    @property
    def gnss_base_recording_rows(self):
        return self._rows_port.read()

    @gnss_base_recording_rows.setter
    def gnss_base_recording_rows(self, rows):
        self._rows_port.replace(rows)


EVIDENCE_FIELDS = ("gnss_base_recording_rows",)


@dataclass(frozen=True)
class EvidenceCapture:
    hierarchy: bool
    active: bool = False
    root: object = None
    state: str = "IDLE"
    session_id: str = ""
    measurement_id: str = ""
    revision: int = 0
    drain_requested_token: str = ""
    file_stamp: str = ""


class RtcmEvidenceRuntime:
    def __init__(self, state, *, capture, activated):
        self.state = state
        self.capture, self.activated = capture, activated
        if not callable(capture) or not callable(activated):
            raise TypeError("RTCM evidence capture and activation adapters are required")

    def context(self):
        value = self.capture()
        root = None if value.root is None else Path(value.root)
        if not value.hierarchy:
            return RtcmEvidenceContext(
                state="COLLECTING" if value.active else "IDLE",
                session_id="measurement-data" if value.active and root is not None else "",
                session_root=root if value.active else None,
                context_token=(value.measurement_id or "measurement-data-live")
                if value.active and root is not None
                else "",
                file_stamp=value.file_stamp,
            )
        token = (
            f"{value.session_id}:{value.measurement_id or 'session'}:r{max(0, int(value.revision)):04d}"
            if value.session_id and root is not None
            else ""
        )
        return RtcmEvidenceContext(
            state="IDLE" if token and value.drain_requested_token == token else value.state,
            session_id=value.session_id,
            session_root=root,
            context_token=token,
        )

    def close_and_drain(self, pipeline, context_token):
        if pipeline is None:
            return False
        current = self.context()
        return bool(
            pipeline.close_evidence_context_and_drain(
                {
                    "context_token": str(context_token),
                    "session_id": current.session_id,
                    "session_root": None if current.session_root is None else str(current.session_root),
                }
            )
        )

    def append_legacy(self, payload, *, source, record_kind):
        if not payload:
            return
        context = self.context()
        if not context.active:
            return
        last_sequence, source_sequences = load_rtcm_evidence_high_water(context)
        key = (str(source), str(record_kind))
        evidence_record = RtcmEvidenceRecord(
            sequence=last_sequence + 1,
            source_sequence=int(source_sequences.get(key, 0)) + 1,
            source=str(source),
            record_kind=str(record_kind),
            payload=payload,
            receive_utc_ns=time.time_ns(),
            receive_monotonic_ns=time.monotonic_ns(),
            message_type=parse_rtcm3_message_type(payload) if record_kind == "rtcm_frame" else None,
        )
        record = append_rtcm_evidence(context, evidence_record)
        if record is not None:
            self.activated({"activated_utc_ns": record["receive_utc_ns"], "source": str(source)})

    def record_base_position(self, sample, *, operation_state, timeline, append_external):
        """Retain NAV-PVT producer datum/units and original receive timeline."""
        rows = self.state.gnss_base_recording_rows
        row = timeline(
            side="base",
            sample_type="gnss_base_position",
            row_contract="one row per GNSS Base position sample (NAV-PVT) received by the GUI",
            sample_monotonic_s=sample.timestamp_s,
            host_receive_monotonic_ns=time.monotonic_ns(),
            host_receive_utc_ns=time.time_ns(),
            source_utc_ns=None,
            source_utc_method="unavailable",
            sequence=len(rows),
        )
        if row is None:
            return
        row.update(
            {
                "latitude_deg": sample.latitude_deg,
                "longitude_deg": sample.longitude_deg,
                "ellipsoid_height_m": sample.ellipsoid_height_m,
                "msl_altitude_m": sample.msl_altitude_m,
                "height_type": sample.height_type.value,
                "fixed": sample.fixed,
                "device_id": sample.device_id,
                "gnss_operation_state": operation_state,
            }
        )
        rows.append(row)
        append_external(row)
