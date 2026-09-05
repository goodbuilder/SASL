# Integration Flow

## Action-to-postcondition traces

```text
Capture Live Pair / Auto tick
  -> latest exact native Left/Right frames and metadata
  -> normalize same-host app-receive monotonic ns
  -> signed Right-minus-Left delta + absolute threshold gate
  -> HOST_RECEIVE_PROXY evidence frozen
  -> CheckerboardCaptureWorker encode/detect
  -> stereo pair or K,D-only decision persisted
  -> status + Photo set manifest/input snapshot
```

FTP remains:

```text
CPTR pair -> same capture ID + host dispatch timestamps -> HOST_DISPATCH_PROXY -> existing worker
```

## Transition tables

| Inputs | Timing result | Persistence | Visible result |
| --- | --- | --- | --- |
| both app receive, within/equal gate | allowed | stereo pair when both boards detected | app-receive basis |
| both app receive, over gate | rejected | valid sides K,D-only | signed delta/limit reason |
| one/both app receive missing/invalid | unknown/rejected | valid sides K,D-only | receive evidence missing |
| verified exposure pair | existing verified path | existing behavior | camera-exposure basis |
| valid FTP dispatch pair | existing proxy path | existing behavior | FTP dispatch basis |

## Failure and recovery flows

Timing rejection never deletes a checkerboard-valid side and never promotes it to R,t. A new fresh capture is the
recovery. Worker/save failure follows the existing failed state and does not mutate prior photo-set evidence.

## Cross-feature contracts and dependency direction

`Camera decoded-frame metadata -> Calibration timing controller -> Calibration worker/persistence -> UI evidence`.
There is no reverse dependency and no Measurement/Time-Sync owner mutation.
