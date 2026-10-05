# P1/P2 Radio minimal profile — permanent Workstation behavior

2026-09-08. User authorized permanent adoption of the measured minimum stream profile.

## Contract and scope

Only left/right with transport `radio`. ATTITUDE 5 Hz, GPS_RAW_INT 5 Hz,
GPS2_RAW 5 Hz, SYS_STATUS 1 Hz, BATTERY_STATUS 1 Hz. HEARTBEAT is untouched.
Explicit interval -1 stops the measured Normal-mode background IDs plus optional
RAW_IMU, GLOBAL_POSITION_INT, DEBUG_VECT and GPS_RTK/GPS2_RTK demands that an
older client may have enabled. This is not a strict packet whitelist: protocol,
event, ACK and local RADIO_STATUS messages remain possible.

No PX4 parameter/EEPROM write, MAV_0 rate/mode change, modem change, RTCM routing
change or Shell polling change. USB/Ethernet and Drone profiles remain unchanged.
P1/P2 Radio optional-tier escalation is bypassed. Stale ATTITUDE retries use 5 Hz.
Disconnect clears the request marker; existing confirmed FC reboot detection
also clears it. The next live heartbeat reapplies the full profile. A send
exception clears the marker for retry. Sending commands does not prove device
ACK/readback or actual receive Hz. Radio loss is still an independent limit.

## Coordinate review

ATTITUDE orientation remains PX4 body FRD relative to local NED, with existing
angle conversion unchanged. GPS_RAW_INT/GPS2_RAW remain geodetic lat/lon and
MSL altitude with existing scaling. No coordinate arrays, transforms, CSV fields
or recording schemas are changed. Disabled position streams are not substitutes
for GPS. Disabled IMU/debug streams may leave their observation panels stale;
USB/Ethernet remains available for those observations.

## Verification slices

- Policy / measured profile in minimal-profile-p1-p2-rtcm.md -> fixed demands:
  PASS in targeted tests, expected numeric intervals specified independently.
- Boundary / MAV_CMD_SET_MESSAGE_INTERVAL 511 -> positive intervals and -1:
  PASS in fake sender tests; keep IDs excluded from disable set.
- State / healthy dwell, missing IMU, duplicate request, send failure, rearm:
  PASS in targeted subscriber tests. Existing reboot detector tests retained.
- Isolation / Drone and direct transports: PASS in targeted tests.
- Persistence / code in repository, auto request per connection: implemented;
  running temporary observer was not restarted. No new hardware run performed.
- Operations / approved Workstation README updated; full suite result below.

Targeted command: `.venv/bin/python -s -m pytest tests/test_pixhawk_radio_policy.py
 tests/test_mavlink_liveness.py -q` from Workstation using repository venv:
41 passed (0.90 s). Full suite launched with `python -s -m pytest -q` from Workstation.

Full suite: **2047 passed, 2 skipped in 156.04 s**, exit 0. Final focused rerun:
**41 passed in 0.77 s**. `git diff --check`: passed. No field hardware retest.
The change report is a new repo-local untracked file; existing unrelated edits
were preserved and no commit was created.
