"""One-way compatibility fields for the named GNSS capability state owners."""

from types import SimpleNamespace

from .base import BASE_FIELDS, BaseReceiverState
from .corrections import CORRECTION_FIELDS, CorrectionRuntimeState
from .evidence import EVIDENCE_FIELDS, GnssEvidenceState
from .gga import GGA_STATE_FIELDS, GgaSentenceOwner
from .ntrip import NTRIP_FIELDS, NtripSessionState
from .operation import OPERATION_FIELDS, GnssOperationRuntimeState


class GnssRuntimeState(SimpleNamespace):
    def __init__(self, *, evidence_rows):
        super().__init__(
            gga=GgaSentenceOwner(),
            ntrip=NtripSessionState(),
            operation=GnssOperationRuntimeState(),
            base=BaseReceiverState(),
            corrections=CorrectionRuntimeState(),
            evidence=GnssEvidenceState(evidence_rows),
        )


def _gga_field(name):
    return property(lambda state: getattr(state.gga, name), lambda state, value: setattr(state.gga, name, value))


for _name in GGA_STATE_FIELDS:
    setattr(GnssRuntimeState, _name, _gga_field(_name))
del _name


def _ntrip_field(name):
    return property(lambda state: getattr(state.ntrip, name), lambda state, value: setattr(state.ntrip, name, value))


for _name in NTRIP_FIELDS:
    setattr(GnssRuntimeState, _name, _ntrip_field(_name))
del _name


def _runtime_field(owner, name):
    return property(
        lambda state: getattr(getattr(state, owner), name),
        lambda state, value: setattr(getattr(state, owner), name, value),
    )


for _owner, _fields in (
    ("operation", OPERATION_FIELDS),
    ("base", BASE_FIELDS),
    ("corrections", CORRECTION_FIELDS),
    ("evidence", EVIDENCE_FIELDS),
):
    for _name in _fields:
        setattr(GnssRuntimeState, _name, _runtime_field(_owner, _name))
del _owner, _fields, _name
