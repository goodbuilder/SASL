# Verification

## Requirement-to-evidence matrix

- PASS pure protocol: raw MAVLink v2 HEARTBEAT yields exact sysid/component; truncated data ignored.
- PASS boundary: bounded UDP receive reports source/listen port, route NIC/source and sysid.
- PASS validation: sysid 1/2 rejected; current sysid 3 candidate accepted.
- PASS UI: Find Network/candidate table/Use selected, dirty edit and accepted Apply/Save URL.
- PASS persistence/restart: settings retained `drone_pixhawk_transport=ethernet` and
  `drone_telemetry_url=udpin:0.0.0.0:14551`.
- PASS hardware: Workstation owns UDP 14551 and reports Drone LIVE/heartbeat OK/GNSS LIVE.

## Focused test evidence

`QT_QPA_PLATFORM=offscreen .../.venv/bin/python -m pytest -q
tests/test_drone_network_discovery.py tests/test_workstation_telemetry.py
tests/test_pixhawk_network_assignment.py tests/test_workstation_lifecycle.py
tests/test_workstation_ui.py tests/test_workstation_contracts.py`: 122 passed.

After generated delegate refresh, focused direct set: 14 passed.

## Regression and application evidence

Full `pytest -q`: 1901 passed, 1 skipped, 2 failed before the generated delegate refresh. The
directly related stale-delegate failure was repaired and passed afterward. The remaining failure is
the pre-existing NVIDIA synthetic lossless recorder rejecting its second frame; no Drone/network
code appears in that path.

## Performance, load, and target-hardware evidence

PASS: one 3-second bounded worker, no GPU/media work, no persistent scan. Live discovery returned
three heartbeats without UI stall. Existing P1/P2 status continued independently.

## Feature isolation and contract evidence

PASS focused lifecycle and Pixhawk assignment regressions. Discovery has its own thread/result and
uses the existing shared connection owner only after explicit Connect.

## Visual authenticity and dynamic-layout stability evidence

PASS screenshot `/tmp/drone-network-live-telemetry.png`: native compact Qt action row, bounded table,
selected factual row, stable lower table and no decorative replacement styling.

## Visual and hardware/manual evidence

PASS baseline: `10.0.0.3:14551` produced sysid-3 HEARTBEAT/ATTITUDE/GPS.
PASS implemented UI flow: Find Network showed `DRONE VERIFIED`, NIC `enx0023a40a2e39`, source
`10.0.0.3`, UDP 14551, sysid 3. Use selected produced `udpin:0.0.0.0:14551`; Apply/Save retained it;
runtime reached `LIVE · heartbeat=OK`. Drone GPS row reached LIVE/3D_FIX with 32 satellites.

## Residual conditions

One unrelated environment residual: the full suite's NVIDIA synthetic decoded recorder test
rejected frame two. Existing startup also reports optional `nvh264dec` unavailable and selects
software decode. This is outside the approved Drone scope and did not block network verification.
