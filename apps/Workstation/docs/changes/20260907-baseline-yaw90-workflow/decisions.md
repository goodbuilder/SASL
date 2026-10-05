# Decisions

2026-09-07: User chose existing Method tab row, not a Navigation subpage. Preserve Pixhawk shared alignment. Move Telemetry installation table and Measurement Reverse. Remove Calibration Center line only. Physical yaw/leveling is iterative. Omit level target 0. Show angles in tables and keep P1→P2 azimuth permanently visible. User explicitly authorized implementation in Default mode.

2026-09-07 review follow-up: User approved fixing two reproduced UI defects. On setup-tab exit,
clear only the owner-tagged Inspector selection, preserving actual table/annotation/candidate
state. Observe the QScrollArea viewport rather than relying on child resize; coalesce updates
through an owned single-shot timer and stop it while hidden. No sensor, geometry, recording or
Reverse persistence changes are included.

2026-09-07 display-query failure follow-up: the user requires continued distance calculation with
the last valid P1/P2 inputs after reception stops. A UI query exception is a separate program error.
Retain the last confirmed display snapshot and grow its ages; show 조회 오류 in both table and
Inspector, keep missing rows explicit, preserve pending controls, and restore source state after
recovery. This display fallback never writes Measurement caches or claims its old values are the
current geometry inputs. Malformed containers must be rejected before updating visible rows.
