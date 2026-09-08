# Verification

## Automated
- `tests/test_pixhawk_radio_policy.py`: existing 30 s / 15 s boundary and
  regression tests unchanged; new
  `test_radio_regression_caps_escalation_below_the_failed_tier`,
  `test_radio_regression_at_first_optional_tier_caps_at_critical`,
  `test_radio_transport_reset_forgets_the_cap`.
- `tests/test_mavlink_liveness.py`: new
  `test_subscriber_transition_skips_qualification_preserving_command_blip`
  and `test_subscriber_transition_still_logs_qualification_resetting_command`.

## Field check after restart
1. Log tab: `오른쪽/Drone MAVLink liveness: LIVE` should appear once per
   (re)qualification, not every 2 s. PARAM readback still shows a
   `QUALIFYING; qualification=00:00/00:10` line.
2. After the first `ATTITUDE STALE` at tier T, `Radio optional stream tier=`
   should climb back only to T-1 and stay there for 5 min before trying T
   again; the status detail shows `optional-cap=<T-1>` meanwhile.
