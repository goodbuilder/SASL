# Integration Flow

## Action-to-postcondition traces

```text
device/controller message
  -> panel._log(message, optional metadata)
  -> GUI-thread normalization + sequence/timestamp
  -> panel-owned bounded DiagnosticEventStore
  -> StandalonePanelBackend incremental read facade
  -> Diagnostics Log model
  -> filter proxy/table/follow or pending indicator
```

기존 side path도 유지한다:

```text
panel._log
  -> existing application audit persistence when active
  -> existing wiris/pixhawk/camera_yolo widgets
  -> existing FTP lifecycle buffer when applicable
```

## Transition tables

| UI state | New event | Visible result |
|---|---|---|
| following | filter match | append and remain at bottom |
| scrolled up | any | preserve position, increment new count |
| paused | any | do not change rows, increment pending count |
| resumed | pending available | append in sequence order |
| paused/hidden overflow | oldest expired | show explicit expired count, continue |
| filter changed | existing events | recompute visible rows only |

## Failure and recovery flows

Structured facade unavailable → legacy text fallback. Malformed batch → factual unavailable state →
next poll retry. UI model/filter failure must not alter store or device behavior. Store overflow →
explicit loss notice, no unbounded memory growth.

## Cross-feature contracts and dependency direction

Device/controller → panel owner → neutral diagnostic read contract → Workstation Diagnostics.
Diagnostics는 device widgets/private workers를 직접 읽거나 제어하지 않는다.
