# MAVLink Connect GUI freeze — 2026-08-26

## Field failure

Latest Workstation PID 892994 was launched with Preview, NTRIP, Measurement and recording stopped.
Pixhawk Connect created exclusive UDP listeners 14550/14551, but the visible state remained
DISCONNECTED and the window stopped accepting input. Both Linux Recv-Q values reached 213,760.
After evidence capture the process was terminated and all UDP/RTSP/NTRIP/child ownership released.

SIGUSR1 captured the GUI thread at:

```text
app.exec_
  -> _handle_mavlink_status_changed
  -> _refresh_session_toggle_buttons
  -> _set_button_role
  -> button.style().polish(button)
```

The worker inserts sockets and emits progress before its completion callback starts the GUI-owned
20 ms receive timer. Generic progress therefore changed a topology-button role and entered native
QStyle polish inside the completion critical path.

## Independent wire oracle

`/tmp/mavlink_latest_connect_20260826.pcap`, SHA-256
`70856311071ecefea91c1e7166a5961f9e17b22ca3436931cd7c545938dcca42`, contains 12,724 records over
49.134 seconds. Every datagram has one valid MAVLink2 frame; sequence loss, duplicate, reorder and
capture truncation are zero.

- P1/P2 unique unicast rates: 66.939/66.084 messages/s.
- ANY maximum gaps: 38.222/37.221 ms.
- ATTITUDE maximum gaps: 38.222/38.084 ms.
- HEARTBEAT maximum gaps: about 1.003/1.003 s.
- Host sent one GCS 255/190 heartbeat to each side and no other packet.
- P2/P1 replied with heartbeat after 0.213/0.739 s.
- `COMMAND_LONG` and `MAV_CMD_SET_MESSAGE_INTERVAL(511)`: zero.

The profile and remote-reboot paths were not reached. Wire input stayed healthy and non-bursty
while application drain stopped before the first inbound heartbeat handler.

## External amplifier

Orca PID 5654 was already runaway at about 64% CPU, 3.4 GiB RSS and 2.9 GiB swap. The AT-SPI bus
reported hundreds of thousands of `destination has a full message queue` rejections. Screen reader
and toolkit accessibility were intentionally enabled. This is strong correlation with the native
style stall, not native-stack proof of sole causality. No accessibility service was killed or
disabled.

## Repair contract and software evidence

- Queue subscriber status/shell and connect completed/failed/cleanup callbacks explicitly to GUI.
- Generic status cannot refresh topology-button styling before completion.
- Coalesce side-action/LIVE-calibration refresh within one event-loop turn.
- Start the subscriber timer only on its QObject owner thread.
- Preserve button text and `role` without synchronous QStyle unpolish/polish.
- Emit successful RTCM progress at most once per side per five seconds; preserve exact counters and
  every failure.

Focused regression: 3 passed. Full Pixhawk/GNSS file: 90 passed. Dependent regressions: 107 passed.
Root combined MAVLink/liveness/status/RTCM rerun: 143 passed. A 69.63-second real post-fix Connect
remained responsive, drained both sockets and sent the exact 15-command Stock profile once per side.
Later isolated-adapter application ten-minute gates also remained responsive; the latest complete
Workstation suite is 1448 passed, 1 skipped.
