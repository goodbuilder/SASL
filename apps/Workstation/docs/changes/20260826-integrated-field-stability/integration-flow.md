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
| MAVLink | CONNECTING | 60-second fresh HB+ATT | STALE or RX_STALLED | verified-side recovery and 60-second requalification |
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

## Proposed health-aware pair timing flow — not yet approved

```text
Clock snapshot + manual-seed history + logical epoch
  -> transition/mismatch: drop pair candidate
  -> VERIFIED: select nearest pair by RTCP/NTP, <=100 ms
  -> previously VERIFIED then revoked/unavailable:
       flush buffers -> advance timing-basis generation
       -> select nearest pair by app-receive monotonic, <=100 ms
       -> distance geometry unchanged
       -> result ESTIMATED/DEGRADED_TIMING

selected pair
  -> persist selected basis + selected delta
  -> persist exact app-receive monotonic values/delta
  -> persist NTP/SR/RTP values/delta as diagnostic even in fallback
  -> persist Clock state/reason + timing-basis generation + fallback reason

fresh Clock VERIFIED
  -> flush fallback buffers -> advance generation -> future pairs use NTP
```

The pairing coordinator, Measurement snapshot and persistence writer exchange explicit public
fields. No widget reads Clock worker internals, and no storage component chooses a timing basis.

## Superseding fixed app-receive live-distance flow — awaiting approval

```text
decoded appsink sample/side
  -> exact host app_receive_monotonic_ns
  -> frame metadata + native exact-frame identity
  -> bounded Left/Right coordinator
  -> nearest app delta <=100 ms
       -> frozen pair basis=app_receive_monotonic
       -> unchanged method/geometry evaluation
       -> normal VALID when other requirements pass
     else -> drop/wait

parallel record-only branch
  RTCP SR/RTP mapping/NTP frame time + Clock state/epoch
    -> selected pair/session evidence only
    -> never feeds pair selection, delta gate or distance validity

STUS/Auto/Clock transition
  -> Time / Sync + record-only evidence
  -> no live-distance buffer flush, pause or reclassification
```

## Proposed indoor experiment flow — awaiting approval

```text
strict default
  -> NAL5 + exact decoded PTS only

explicit experiment process
  RTP/AU startup or discontinuity
    -> latch all media consumers
    -> parser-complete SPS/PPS + non-delta NAL1 + valid PTS + clean flags
    -> exact candidate PTS enters existing decoder
    -> exact decoded PTS returns
    -> Camera status EXPERIMENTAL NON-IDR INTRA
    -> Preview/YOLO/native/app-time pair may continue
    -> any next gap/error relatches immediately

five-minute preflight
  -> camera current + P1/P2 QUALIFYING/fresh HB+ATT + HTTP 200/RTCM/injection
     + recording growth + GUI/resources
  -> PASS: start 3,600-second checkpoint monitor
  -> FAIL: stop candidate and preserve evidence

60-second operational checkpoint
  -> P1/P2 LIVE after continuous qualification

10-minute checkpoint
  -> P1/P2 still current as independent sustained-link evidence

stop
  -> Measurement/record drain -> MP4 EOS + strict expected FAIL audit
  -> NTRIP/MAVLink/camera stop -> process quit -> resource verification
  -> normal strict-mode relaunch
```

## Manual-only PX4 calibration PARAM flow

```text
P1/P2 reaches LIVE
  -> no PARAM command, no PARAM SYNC, no qualification reset

operator Refresh P1/P2 parameters
  -> existing command registry/backend facade
  -> one side-specific PixhawkParameterWorker calibration_refresh
  -> 75 MAG + 49 attitude PARAM_REQUEST_READ responses
  -> calibration caches/tables and optional later MAG-slot evidence
  -> existing post-command qualification reset

explicit reset/apply/save
  -> existing mutation owner -> explicit readback -> visible result/error
```

## 60-second operational qualification flow

```text
expected-sysid HEARTBEAT + ATTITUDE become fresh
  -> QUALIFYING/RECOVERING 00:00 / 01:00
  -> both remain fresh continuously for 60.000 s
  -> LIVE -> attitude-dependent method input eligible

ATTITUDE stale or all expected RX stalled
  -> immediate stale/stalled state and dependent eligibility blocked
  -> fresh recovery evidence -> new 60-second interval

explicit/manual PARAM command
  -> PARAM SYNC -> post-command fresh HB+ATT -> new 60-second interval

10-minute comparison / 60-minute integrated soak
  -> separate external stability evidence; no product-timer mutation
```

## Mountain GNSS 3D-fix flow

```text
Capture Live Pair
  -> immutable same-burst Left/Right GNSS position + numeric fix_type
  -> Solve GNSS + Mountain R,t
       positions finite + non-zero finite NED baseline
       both fix_type >= 3 -> translation axes observable
       either fix_type < 3/missing -> translation axes blocked
       robust Mountain correspondence solve -> independent rotation axes
  -> Draft with exact fix/baseline/warning evidence

next Measurement start
  -> freeze latest Mountain Draft before Detection
  -> accepted target pair -> mountain_gnss method evaluation
       non-fixed usable input -> ESTIMATED
  -> append method_evaluations evidence

parallel representative path
  -> pixhawk_rtk_live -> measurements.jsonl representative result unchanged
```

## Mountain optional-copy flow — approved 2026-08-29

```text
Auto-select Mountain
  -> background proposal + current frozen frame IDs/hashes
  -> quality GOOD / LOW_CONFIDENCE / NO_DATA retained as detector evidence
  -> owner structural check after paired exclusions
       usable -> overlay says COPY READY
       unusable -> COPY BLOCKED reason + Manual Mountain fallback

Use suggestion
  -> owner filters/validates paired exclusions
  -> filtered-input hash + counts/exclusions + quality warning evidence
  -> copy exact retained points to editable annotation buffer

Solve GNSS + Mountain R,t
  -> complete manual/accepted annotations exist: use them
  -> missing/partial annotations: reject through existing behavior
  -> unchanged K,D + GNSS + robust rotation gates
  -> candidate

Measurement start
  -> unchanged latest-Draft freeze and mountain_gnss comparison fan-out
```

## Horizon optional-copy flow — approved 2026-08-29

```text
Auto-select Horizon
  -> background endpoint proposal + detector quality + current frozen identities
  -> owner/UI structural check: two finite in-frame endpoints/side, >=25% width apart
       usable -> quality text + COPY READY
       unusable -> quality text + COPY BLOCKED reason

Use suggestion
  -> controller revalidates identity/dimensions/endpoints
  -> validated-input hash + original quality warning evidence
  -> copy endpoints to editable annotation buffer

Solve Horizon Roll/Pitch -> unchanged exact K,D/model/endpoint/observability checks
Apply candidate -> unchanged explicit action
```

## Approved Settings-preferred live-distance flow — 2026-08-31

```text
Settings Apply/Save
  -> normalize requested timing preference (default app receive)
  -> active Measurement: persist for next Measurement
  -> idle: publish policy generation -> flush old detection candidates

decoded frame pair
  -> one immutable requested/effective timing policy
  -> selected RTCP + VERIFIED/current/same-epoch/finite/in-limit: pair on RTCP/NTP
  -> any RTCP timing fault: pair on exact app-receive monotonic within the same bound
  -> freeze the same effective basis/delta/reason
  -> unchanged method workers and geometry
  -> persist requested + effective + fallback reason + separate app/NTP evidence
```

No widget, storage reader or clock observer may independently choose another basis. PX4 attitude
lookup remains host-monotonic and fallback provenance alone does not degrade distance validity.
