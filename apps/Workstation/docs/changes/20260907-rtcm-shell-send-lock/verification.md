# Verification

## Automated
- `tests/test_mavlink_liveness.py`: existing lock-depth test updated (writes
  now take the per-write lock: depth 2 under a held exclusive command); new
  `test_shell_command_without_held_send_lock_only_locks_each_write` asserts
  reads run at depth 0 and writes at depth 1 with `hold_send_lock=False`.
- `tests/test_workstation_pixhawk_gnss.py`:
  `test_pixhawk_shell_read_stops_at_prompt_and_on_idle` (prompt exit < 0.5 s,
  idle exit at 0.5 s, empty reply still waits the full timeout, multi-line
  keeps reading) and
  `test_pixhawk_shell_worker_awaits_replies_without_holding_send_lock`.
- `tests/test_rtcm_pipeline_workers.py`:
  `test_base_rtcm_frame_is_queued_with_receive_timestamp` and
  `test_rtcm_drop_log_is_throttled_with_pending_count`.
- Suites run green on 2026-09-07: liveness/status-model 36, pixhawk GNSS 100,
  RTCM pipeline/evidence/telemetry 49, magnetometer workspace 26.

## Field check after restart
1. `run.log` shows `왼쪽/오른쪽 RTCM 주입` lines about every 5 s per side.
2. No sustained `[GNSS] RTCM 오래된 대기 프레임 N개를 폐기했습니다.` stream
   (occasional single drops during PARAM readback are expected).
3. Receiver table: P1/P2 GPS1 fix number moves 3 → 5 (RTK Float) → 6 (Fixed)
   within roughly a minute of continuous corrections.
4. `listener sensor_gps -n 1` on either Pixhawk shows a non-zero
   `rtcm_injection_rate` and `rtcm_msg_used` = 2 (used).

## Shutdown follow-up
- `test_pixhawk_parameter_worker_stops_when_its_thread_is_interrupted`,
  `test_pixhawk_shell_worker_stops_between_commands_when_interrupted`,
  `test_stop_pixhawk_operation_workers_requests_interruption_before_quit`
  in `tests/test_workstation_pixhawk_gnss.py`.
- Field check: close the app while `[PX4 MAG] ... 자동 readback 요청` is in
  progress; the window should close within a couple of seconds and
  `run.log` should show `PARAM 작업 실패 (mag_refresh): PARAM read ...
  interrupted by shutdown` rather than a `terminate()` line.
