# RTCM injection starved by sensor_mag Shell polling (P1/P2 stuck at 3D FIX)

## Symptom
With NTRIP (RTS2 VRS-RTCM32) connected and streaming ~1.3 kB/s, both rovers
stayed at fix_type 3. `RTCM 주입` status lines appeared only every ~34 s per
side although they are emitted at most once per 5 s while sends succeed.

## Root cause
- The stable-PX4 `sensor_mag` Shell poller runs six `listener` commands per
  side every 2 s inside one `exclusive_command`, which held the per-side
  MAVLink **send lock** for the whole cycle.
- `_read_mavlink_shell_output` always waited out its full timeout
  (0.25 s + 0.25 s probes + 4.0 s command read ≈ 4.5 s per command), so one
  poll cycle held the lock for ~27 s.
- `send_rtcm_data_if_eligible` waits on the same lock; RTCM frames could only
  leave in the ~2 s gap between cycles.
- The NTRIP → `_handle_gnss_base_rtcm_frame` path enqueued frames **without a
  receive timestamp**, so the injection worker's 2 s stale guard never fired
  and the backlog was flushed late in a burst. The receiver saw corrections
  tens of seconds old and never entered RTK.

## Change
1. `_read_mavlink_shell_output` returns as soon as the NSH prompt is back
   (single-line commands) or after a 0.5 s idle gap; `timeout_s` remains the
   hard bound. Multi-line payloads keep reading past intermediate prompts.
2. `_send_mavlink_shell_command` takes the side's send lock only around each
   SERIAL_CONTROL write. `PixhawkShellCommandWorker` opens the exclusive
   command with `hold_send_lock=False` (reboot keeps the lock).
3. `_handle_gnss_base_rtcm_frame` emits `request_envelope` with
   `receive_monotonic_ns`, so frames older than `MAX_INJECTION_AGE_S` (2 s)
   are dropped instead of delivered stale. Drop logging is throttled to one
   line per 5 s carrying the pending count.

Poll cadence (2 s) is unchanged: the CAL_MAG-slot table marks shell rows
STALE above 6 s, and the shorter per-command wait already cuts the cycle to a
few seconds.

## Related (same session, separate change)
`PixhawkWorkspace` automatic CAL_MAG PARAM readback now waits for a verified
heartbeat and retries up to 3 times (10 s apart) instead of firing once during
CONNECTING and giving up; success/request lines are logged as `[PX4 MAG]`.

## Follow-up (same day)
- PARAM reads/writes (`read_param`, `read_param_int`, `set_param_int`) now open
  the exclusive command with `hold_send_lock=False` and lock only the
  request frame. A 75-parameter CAL_MAG readback over the radio otherwise held
  the send lock for most of a minute and, because the injection worker sends
  P1 then P2 per frame, starved both sides.
- `_apply_pixhawk_rtcm_result` logs blocked/failed targets to `run.log`
  (`[GNSS] RTCM 주입 제한 ...`, throttled to 5 s). Previously the reason was
  only visible in the GNSS tab health label.

## Shutdown follow-up (same day)
Closing the app waited up to `SHUTDOWN_DEADLINE_S` (30 s) whenever a Pixhawk
shell or PARAM worker was mid-sequence: `quit()` cannot stop a running slot,
no worker honoured `requestInterruption()`, and MAVLink disconnect is only
attempted once those workers stop. `PixhawkShellCommandWorker` and
`PixhawkParameterWorker` now check `isInterruptionRequested()` before every
radio round trip and fail fast with "interrupted by shutdown";
`_stop_pixhawk_operation_workers` calls `requestInterruption()` before
`quit()`. Worst-case close cost from these workers drops to one PARAM
timeout (1.5 s) or one shell command (a few hundred ms).
