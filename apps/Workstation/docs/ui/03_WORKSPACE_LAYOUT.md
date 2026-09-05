# StereoWIRIS Workspace Layout

- Status: Revised after audit; adapter-backed implementation present
- Depends on: 00_PRODUCT_VISION.md, 01_USER_WORKFLOW.md, 02_INFORMATION_ARCHITECTURE.md

## 1. Main Window Structure

QMainWindow
- Menu Bar
- Primary Toolbar
- Left Navigation Dock
- Central Workspace
- Right Inspector Dock
- Bottom Diagnostics Dock
- Status Bar

Concept:

┌──────────────────────────────────────────────────────────────┐
│ Menu Bar                                                     │
├──────────────────────────────────────────────────────────────┤
│ Primary Toolbar                                              │
├──────────────┬───────────────────────────────┬───────────────┤
│ Navigation   │       Central Workspace       │ Inspector     │
├──────────────┴───────────────────────────────┴───────────────┤
│ Diagnostics                                                  │
├──────────────────────────────────────────────────────────────┤
│ Global Status                                                │
└──────────────────────────────────────────────────────────────┘

## 2. Priority

1. Workspace
2. Navigation
3. Inspector
4. Diagnostics
5. Toolbar / Status

## 3. Target Display

Primary target: 1920×1080 or larger, mouse+keyboard, landscape. 1366×768 remains usable with collapsible docks.

## 4. Menu / Toolbar

Toolbar height target: 28–34 px. Candidate groups: connection, preview/detection, recording, data, diagnostics.

## 5. Navigation Dock

Default width candidate: 220–280 px, initial ~240 px, minimum ~180 px. Use hierarchical tree.

## 6. Workspace

Should receive approximately 55–65% or more of horizontal space under default layout. Local tabs are allowed inside one engineering context.

Camera and Pixhawk network candidate tables retain compact native QTableView geometry while adding
an `Assigned` role-set projection. Candidate count is runtime-variable. `Assign Camera NIC/P1/P2`
opens one native before/after confirmation; asynchronous STOPPING/APPLYING/VERIFYING/RECONNECTING/
ROLLING BACK status stays in the existing bounded status region and never rebuilds the table, moves
selection, changes scroll, or resizes adjacent controls.

## 7. Inspector Dock

Default width candidate: 280–340 px, initial ~300 px, minimum ~240 px. Contextual, compact, collapsible.

The Status group remains common where a source-backed channel exists. The section below it
changes its title and field labels for the active context (for example Camera pair, Base
operation, Physical alignment, or Data recording). Selecting a supported table row or
engineering value temporarily switches that section to Selection details. Inspector values
remain read-only projections and preserve the producer's frame, unit, and timestamp contract.

## 8. Diagnostics Dock

Default height candidate: 180–240 px, initial ~200 px. Initial tabs may be System, Log, Measurement. Opening Diagnostics shall not replace Workspace.

## 9. Status Bar

Height target: 22–28 px. Candidate indicators: CAM L/R, P1/P2, RTK, DET, MEAS, REC, FPS.

## 10. Collapsible Regions

Navigation, Inspector, Diagnostics may be hidden. Workspace remains dominant. Focus mode may temporarily hide docks.

## 11. Docking Behavior

Use stable default locations. Resizing required. Floating optional. Complex free docking is not necessary in first implementation.

## 12. Workspace Switching

Persistent shell remains unchanged; only Workspace and Inspector context changes.

## 13. Camera Workspace

Image-dominant, possibly left/right panes with compact facts below or beside. Preserve aspect ratio.

Camera readiness actions are ordered as IP discovery, WIRIS control, RTSP enablement,
and RTSP frame check. Left/right mapping and saving camera settings are kept in a separate
mapping/settings group.

The details region also contains a fixed-height `RTP / decode` text group for each side. It shows
worker-measured five-second rolling RTP packets/s, sequence-gap missing packets/s, estimated loss,
H.264 access-units/s, and decoded Hz without changing preview geometry.

## 14. Time / Sync Workspace

The field clock sequence is visible together: set WIRIS time from the PC, edit/apply/save
RTCP/NTP sync limits, and check stream evidence. Workstation timing evidence includes WIRIS
RTC/STUS-GDTI, live RTCP/NTP frame metadata, and the editable sync-limit contract; still-image
capture, FTP, and EXIF comparison remain Legacy-only.

## 15. Data and Measurement Workspaces

`Data` is the top-level storage/capture surface with `Status` and `Capture` tabs. Status owns the
fixed JSONL/CSV stream table, writer/queue/drop state, data-folder access, Retry Storage, and a file-manager
link to legacy recordings. A fixed second Files action row owns `Update Latest XLSX` and
`Open Latest XLSX`; a fixed two-line status strip shows target/published generation, state, row
count, and compact error while the tooltip retains full path/detail. Capture owns Target reference,
HR/OZ events/import, and ULog tools. Data does not expose Session lifecycle, package completeness,
legacy `review.xlsx`, or legacy catalog rows. The latest XLSX is a replaceable one-Measurement view,
not a restored Review workspace.

Measurement owns one `측정 시작`/`측정 종료` control and transient
`IDLE/QUEUED/STARTING/RUNNING/STOPPING/ERROR` status. The control stays enabled when storage is
degraded. Actual camera contention appears as QUEUED and resumes automatically. The readiness and
method tables remain evidence projections and preserve PX4, GNSS NED, candidate virtual-FRD/NED,
and OpenCV RDF contracts.

## 16. Calibration Workspace

The primary surface keeps the local `촬영`, `계산·적용`, and `프로파일` tabs. `촬영` places
checkerboard status, capture commands, current-run/current-set counters, and the latest factual
status before an equal-width Left/Right overlay pair. This keeps critical evidence and commands
inside the unscrolled 1366x768 viewport even when Navigation, Inspector, and Diagnostics are open;
the preview continues immediately below. Live status rows have fixed height and elide long detail
into a tooltip, so they do not move controls, focus, selection, or scroll. Photo-set roots, exact
file selection, setup, diagnostics, raw K,D/R,t evidence and detailed command responses remain under
collapsed `고급 캘리브레이션`; the hidden owner UI is not a fallback.

## 17. Pixhawk / GNSS / Detection / Recording / Analysis

Pixhawk, Base GNSS, and Alignment have separate Workstation pages. Pixhawk presents P1/P2
attitude, rover fix numbers, and NED/FRD baseline evidence. Directly below Connection it keeps a
bounded, horizontally scrollable NIC/address/route candidate table plus stable P1/P2 assignment
summaries; live status refresh must not rebuild that table or move its selection. Its `Pixhawk Parameters`
group has P1/P2 tabs, each split into Accelerometer, Gyroscope, Compass/Magnetometer,
Level Horizon, and manual Board Orientation groups. The first four groups provide a guarded
reset button that sends PX4 `param reset` only for that group's parameters and then refreshes
the readback; Board Orientation is explicitly manual and has no calibration reset. Priority
values are shown with their PX4 semantic label and raw number. These values retain the PX4
sensor-axis/board-parameter contract and are not converted to NED, ENU, or camera frames.
The P1/P2 MAG Priority cards also show a bounded read-only `sensor_mag instance → device_id →
role/CAL_MAG slot` topology map, a priority candidate preview, and the actual
`vehicle_magnetometer` device. The priority combos remain editable for all roles; candidate and
actual are deliberately separate and topology refresh never blocks the controls or moves focus.
The IMU magnetometer table shows the three `SCALED_IMU/2/3` rows plus the existing dedicated
RM3100 `DEBUG_VECT` `CAN1_MAG`/`CAN2_MAG` rows, preserving each row's original topic and unit.
Stable PX4 Shell `sensor_mag` readback may populate those same RM3100 rows with a
`px4-shell/sensor_mag/<instance>` topic and Gauss unit; the source remains visible.
The IMU and dedicated CAN magnetometer tables additionally show fixed `RAW yaw°` and
`Corrected yaw°` columns. RAW is an explicitly labelled producer-axis `atan2(-y,x)` diagnostic;
Corrected yaw applies the read-back `CAL_MAG` offset/scale/misalignment/rotation when the user
has manually refreshed parameters. Before readback, the cell remains `PARAM 대기`. PX4 yaw/Δ
remain backend diagnostic evidence rather than operator-table columns. Missing, stale, and
uncalibrated values use bounded text and do not resize the table, move selection, or alter
priority/calibration ownership.
Base GNSS presents the Base receiver, Survey-in/NTRIP, discovered receiver candidates, and
RTCM delivery; Alignment presents physical sampling and iteration evidence. Detection remains
with Cameras, writer/capture status is Data, and Manual record/video files are Advanced.
The Alignment page places compact readiness and shared frozen native left/right annotation views
below the page header. The previews are side by side when each can retain at least 560 px width and
stack vertically otherwise. Below them, promoted `Horizon`, `Mountain + GNSS`, `Terrain PnP`,
`Drone Rotation`, `Drone Multi-view`, `References`, and `Combine` tabs replace the former nested
`R,t methods` structure. Candidate residuals/side-specific observability and immutable Apply/Revert
remain in one common review area: right of the tabs at widths of at least 1200 px and below them on
narrower screens. Overview shows eight read-only method cards and Measurement shows all frozen/live
method details; Pixhawk/RTK remains the fixed representative. Alignment Apply/Revert/Reset is
app-lifetime state and is not a second calibration owner.
The Alignment evidence keeps raw RTK NED `[N,E,D]` and the separate P1 body-FRD
`[F,R,D]` projection in adjacent groups so the operator can use the intuitive P1
orientation without confusing it with camera optical RDF or public measurement output.
The Pixhawk page keeps the raw P1/P2 ATTITUDE diagnostic separate from GNSS installation alignment.
The Telemetry page places one fixed `GNSS BASELINE_CENTER F/R/D/Rg/Angle` group immediately below its
three-row `Live position` table and before RTCM delivery. A non-selectable one-row table keeps
State, signed Forward/Right/Down metres, Rg, the acute horizontal baseline↔Drone angle, absolute
90° error, maximum age, maximum three-GNSS receive delta, and P1/P2/Drone fix labels in stable
columns. Live updates replace values in place and never
resize the group, move controls, change focus/selection, or jump the outer scroll position.
Immediately below this baseline/yaw group, Pixhawk adds one compact read-only `Drone Pixhawk
geometry · F/R/D/Rg` table with `Dr from Baseline Center`, `Dr from P1`, and `Dr from P2` rows.
The midpoint row is GNSS baseline-center local-level FRD; the P1/P2 rows are their respective
PX4 body-FRD vectors. Its separate `∠Dr-P1-P2` and `∠Dr-P2-P1` values use horizontal F/R only,
are unsigned internal angles, and do not add a 3D/yaw verdict. Fixed formats and in-place refresh
preserve group height, focus, row selection, and outer scroll.

Its Discovery candidate table uses fixed source-neutral `Source`, `NIC / Device`, `Link / Kind`,
`Evidence`, and `Role` columns. A new USB/network search intentionally clears prior table selection
and feedback without moving the assignment cards, controls, docks, splitters, or outer scroll.
Network rows may say `P1, P2 (shared NIC)`; USB rows show one physical endpoint and the observed
P1/P2/Drone sysid separately, so a shared host NIC cannot be mistaken for a shared Radio.
The bounded reason and static coordinate-contract hint
remain visible without moving controls, focus, selection, scroll, docks, or splitters at the
existing visible-workspace refresh cadence.
Data Capture keeps Target reference connectivity, HR purpose/import events, and a PX4 ULog table.
UAM target telemetry is shown as deferred rather
than erroneous; target Drone data is a separate receive-only source and never replaces P1/P2
attitude, GNSS baseline, or nominal stereo geometry.

## 18. Density Targets

- margins 4–8 px
- section spacing 4–8 px
- toolbar 28–34 px
- control 24–30 px
- tree/table row 22–28 px
- status 22–28 px

## 19. Anti-Patterns

Avoid giant scrolling settings pages, excessive nested top-level tabs, dashboard cards, duplicated status, routine modal monitoring, separate windows per subsystem, oversized bars, hidden critical state.

## 20. Phase 1 Minimum

Menu, Toolbar, Navigation, Central Workspace, Diagnostics, Status Bar. Inspector may initially be minimal but architecture reserves the region.

## 21. Acceptance Criteria

- Workspace visually dominant
- critical state visible without navigation
- Diagnostics opens without replacing Workspace
- Navigation and commands separate
- major regions resizable
- no new top-level tab per feature
- desktop layout remains usable

Implementation note: the current Workstation keeps one hidden runtime owner and provides
source-backed Overview, Camera, Pixhawk, Base GNSS, Measurement, Alignment, Calibration,
Recording/Review/Export, and Advanced pages. The internal owner UI is not a user-visible
fallback. 1366×768, 1600×900, and 1920×1080 offscreen construction was checked, while
field data density and long-running finalization remain conditions.
