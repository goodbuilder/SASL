# Integration Flow

## Action-to-postcondition traces

```text
Preview -> controller -> one rtspsrc/side -> integrity gate -> native ring/decoder
        -> current FrameKey -> latest-only UI + accepted-pair evidence

MAVLink Connect -> subscriber -> raw HB/ATT/ANY -> liveness reducer
                -> qualification -> side status -> RTCM route admission

NTRIP Start -> credential resolver -> caster HTTP -> CRC-valid RTCM3
            -> route freshness recheck -> healthy-side injection -> counters/status

STUS -> both-side bounded exact-OK HIWS preflight
     -> attempt 1 Left exact OK -> Right exact OK -> RTSP generation replacement
     -> 3 unique stable post-ACK pad-probe SRs/side
     -> within limits: final verify
        or one signed correction: attempt 2 Left exact OK -> Right exact OK -> replace/reobserve
     -> final hard quality gate -> continuous 1 Hz reducer -> VERIFIED, FAILED or revoked evidence

Measurement/Record -> resource coordinator -> new run/writers -> frames/results/MP4
                   -> durable ACK/hash/EOS -> visible completion -> cleanup
```

## Transition tables

| Capability | Start | Qualified | Failure | Recovery |
|---|---|---|---|---|
| Camera | STARTING | current frame + decoder | boundary-specific ERROR/RECOVERING | bounded reconnect or named physical action |
| MAVLink | CONNECTING | 10-minute HB+ATT | STALE or RX_STALLED | verified-side recovery and requalification |
| NTRIP | CONNECTING | STREAMING valid RTCM | AUTH/NO_DATA/STALE | correct credential/network and reconnect |
| STUS | SYNCING | current live thresholds continuously satisfied | SYNC_UNVERIFIED/FAILED | stop recording if active, manual STUS, then requalify |
| Recording | STARTING | bytes advancing | per-side/storage ERROR | stop/drain, fix boundary, new run |

## Failure and recovery flows

Every failure latches enough provenance to explain it after recovery. Recovery never reuses a stale
FrameKey, old run identity, stale RTCM frame or pre-STUS timing epoch. Partial success remains visible.

## Cross-feature contracts and dependency direction

Controllers coordinate through explicit state/read models; UI does not access worker internals.
Camera ring/evidence infrastructure remains feature-neutral. MAVLink liveness owns RTCM route
admission. Storage reports health but does not own Measurement admission.
