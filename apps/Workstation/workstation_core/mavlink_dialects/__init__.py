"""SASL private MAVLink dialect (adds SASL_CAN_MAG, id 180).

``sasl_can_mag_dialect.py`` is generated from the recorded firmware's XML
definitions. The original inputs, validation schema and hashes are kept in
this package; the regeneration check reproduces the exact current bytes.
Hardware deployment is tracked separately in the firmware manifests.
The dialect is not shipped inside pymavlink, so it must be registered into
``sys.modules`` before
``mavutil.mavlink_connection(..., dialect="sasl_can_mag_dialect")`` is called
-- see :func:`ensure_registered`.

Named distinctly from the unrelated ``common/python/sasl_common`` package
(stereo geometry) elsewhere in this repo to avoid any confusion.

To verify regeneration from the SASL repository root:

    .venv/bin/python -B apps/Workstation/tools/regenerate_mavlink_dialect.py --check

See this package's README.md for the pinned generator and preview command.
"""

from __future__ import annotations

import os
import sys

DIALECT_NAME = "sasl_can_mag_dialect"

_registered = False


def ensure_registered() -> str:
    """Register the vendored dialect module so pymavlink can find it by name.

    Returns the dialect name to pass as ``mavutil.mavlink_connection(dialect=...)``.
    Safe to call repeatedly.
    """
    global _registered
    if not _registered:
        # pymavlink's mavutil.set_dialect() resolves dialect modules under
        # pymavlink.dialects.v10.* unless MAVLINK20 is set, even though the
        # wire protocol itself auto-negotiates to v2 -- without this, our
        # sys.modules registration below (under the v20 path) is never found.
        os.environ.setdefault("MAVLINK20", "1")

        import pymavlink.dialects.v20 as _v20_pkg

        from . import sasl_can_mag_dialect

        sys.modules[f"pymavlink.dialects.v20.{DIALECT_NAME}"] = sasl_can_mag_dialect
        # mavutil.set_dialect() does getattr(pymavlink.dialects.v20, DIALECT_NAME)
        # after the sys.modules short-circuit above skips a real import, so the
        # attribute must be set explicitly too.
        setattr(_v20_pkg, DIALECT_NAME, sasl_can_mag_dialect)
        _registered = True
    return DIALECT_NAME
