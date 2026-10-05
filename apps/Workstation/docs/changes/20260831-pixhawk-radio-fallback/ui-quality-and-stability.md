# UI Quality and Stability

Reuse the existing PX4/MAVLink settings group, connection actions, status rows, Diagnostics tables,
and Inspector detail. Add no new dashboard or decorative status surface.

Transport-labelled state text must reserve stable width, keep focus/selection/scroll, and never
resize docks during heartbeat, RSSI, packet-loss, RTCM-age, or reconnect updates.

Active-RTCM administrative actions add a bounded informational Log row only. They never produce a
modal, banner, toast, focus change, dock movement, or command-layout change.

Discovery keeps the existing table, assignment cards, buttons, fonts, spacing, and fixed column
geometry. Reset clears selection and stale feedback intentionally at command start, without moving
focus outside the Pixhawk workspace or changing scroll/dock/splitter geometry. Source-neutral column
labels and bounded role text replace the misleading network-only headings; full values remain in
tooltips.

## Approved guided topology surface

Reuse `QGroupBox`, `EngineeringValue`, the existing source-neutral table style, `statusStrip`, and
normal command buttons. Place one fixed-height five-row table below Board transport; do not add a
wizard window, dashboard cards, animated progress, decorative icons, or auto-expanding sections.

Step state uses bounded text (`IDLE`, `CHECKING`, `WAITING FOR OPERATOR`, `WRITING`, `VERIFYING`,
`PARTIAL`, `READY`, `COMPLETE`) plus a textual reason, never color alone. Reserve the evidence and
action columns for the longest expected identity/path/error strings with elision and full tooltips.
Live heartbeat/carrier updates may replace cell text but must not resize rows, columns, the workspace,
or adjacent controls; they must not steal focus, change selection, or jump scroll.
