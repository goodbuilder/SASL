# Measurement optional-timeline non-finite FIELD RED — 2026-08-27

## Preconditions

- Latest integrated tree had passed `1444 passed, 1 skipped` and runtime/delegate/compile/diff
  checks.
- Replacement P1/P2 RTL8156B NIC `enx0023a40a2e39` was UP at `10.0.0.10/24` with zero errors.
- P1/P2 were fresh and high-rate; both camera sides were truthfully unavailable because firmware
  2.18.72 emitted no verified H.264 IDR despite `GEIR VISIBLE=1` readback.
- The toolbar `측정 시작` and Manual record actions were enabled.

## Real failure

One explicit Measurement start allocated runtime generation
`run-83017fc8d247428889263d266bc1a593` and waited for current-generation camera/MP4 readiness. During
that bounded 30-second wait, an optional MAVLink telemetry callback repeatedly attempted to append a
raw timeline row containing a non-finite float. `measurement_data_store._json_safe` correctly
rejected it, but the caller allowed the exception to escape into the Qt callback boundary.

The current run log contains exactly 315 `Unhandled Workstation Qt callback exception` records and
315 matching `ValueError: measurement data contains a non-finite number` records. Their expanded
tracebacks produced 6,184 lines / 520,471 bytes in roughly 30 seconds. This is a reproducible GUI/log
amplifier and a plausible long-run freeze trigger.

The persistent stack is:

```text
device_runtime_controller._update_mavlink_optional_timeline
  -> recording_controller._record_external_timeline_row
  -> measurement_data_store.LiveRows.append
  -> _json_safe
  -> ValueError(non-finite)
```

Independent decode of `/tmp/rtl8156b_latest_app_mavlink_20260827.pcap` confirms the exact active
source: both P1 and P2 transmit `VFR_HUD.airspeed=NaN` at roughly 5 Hz. The complete capture holds
1,035 such P1 messages and 1,032 P2 messages; the bounded Measurement window therefore explains the
315 callback exceptions. Other wire messages also truthfully use NaN for unavailable fields, but
VFR_HUD is the optional timeline type exercised here. The protocol helper converted present values
with `float(...)` but did not require `math.isfinite`.
The strict storage rejection is correct and must remain; the producer boundary must represent
undefined protocol floats as null plus explicit field evidence.

## Failure isolation that passed

Camera readiness ended with both sides `INTEGRITY_RECOVERY_TIMEOUT`. Durable events then recorded,
in order, `recording_stopped` and `measurement_start_failed`; neither 595-byte MP4 stub was promoted
to a usable recording. Application Quit exited zero and the subsequent
`shutdown_resource_verification` reported no live processes, sockets, threads or shared memory.
Thus the start transaction and cleanup fail closed, while the optional-timeline callback itself is
the open CRITICAL defect.

## Required GREEN

- every MAVLink optional float/scaled/custom numeric rejects NaN/Inf to `None`;
- the timeline row records exact non-finite source field names rather than inventing a value;
- a non-finite source timestamp falls back to host receive time with explicit provenance;
- no Qt exception/log storm occurs;
- failed camera start still rolls back and the next Measurement receives a fresh run ID.

## Software and hardware GREEN

The implementation now normalizes every optional/scaled/custom MAVLink numeric through a finite
gate. A non-finite source becomes JSON `null`, while `nonfinite_source_fields` and
`nonfinite_source_reasons` preserve the exact message path and class. The recording boundary also
recursively normalizes any residual nested payload and records host-time fallback provenance for a
non-finite sample timestamp. The strict store rejection remains unchanged as defense in depth.

- focused and adjacent regression: **126 passed**;
- latest complete Workstation suite including the timer-affinity fix: **1448 passed, 1 skipped**;
- compile, delegate, runtime doctor and `git diff --check`: PASS.

The same physical failure path was then executed twice consecutively. Both camera generations
correctly timed out after 30 seconds because the sender still had no IDR; each run durably recorded
`recording_stopped` then `measurement_start_failed`, and the second action was immediately admitted
under a distinct ID:

- `run-aae872f7ddb941eea4ddc6b7baf467d6`;
- `run-47c62ee3a8214a20ae655f149a8e8835`.

Across both runs the fresh log contained zero unhandled Qt callback exceptions and zero non-finite
storage errors. Persisted P1 and P2 VFR rows contain
`mavlink_vfr_airspeed_m_s=null`, `nonfinite_source_fields=["VFR_HUD.airspeed"]`, and
`nonfinite_source_reasons={"VFR_HUD.airspeed":"nan"}`. Final Quit returned exit zero with empty
stderr; no `QObject::killTimer` warning, process, socket or shared-memory residue remained.

Replacement-NIC wire artifact `/tmp/rtl8156b_measurement_nonfinite_green_20260827.pcap` is
5,452,830 bytes, SHA-256
`9371e52bb416ec8f83a8eb9b12438f7b84c983b19711d27d2678a0e2c589cf49`, capture drop zero. It
contains P1 27,766 packets over 158.725 seconds (174.93/s, maximum gap 29.8 ms) and P2 27,573 over
158.720 seconds (173.72/s, maximum gap 25.1 ms). The replacement NIC retained zero link errors and
drops. This slice is FIELD PASS.
