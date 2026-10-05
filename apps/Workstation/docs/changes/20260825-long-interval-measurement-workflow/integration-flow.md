# Integration Flow

## Measurement start

```text
click Measurement Start
  -> if Manual record active: request its camera/video stop
  -> if camera operation busy: state QUEUED with reason
  -> allocate transient generation and timestamped video paths
  -> best-effort append start event / retry storage (failure only degrades Storage)
  -> reserve Detection and start available camera sides
  -> RUNNING when actual source state confirms start
```

An old OPEN/RECOVERABLE Session, retry spool, malformed data file, or missing storage permission is
never consulted before the camera request.

## Measurement stop and next start

```text
click Measurement Stop
  -> mark generation non-current and stop accepting new method/native work
  -> request Detection/camera/video release
  -> best-effort append stop event and async flush
  -> IDLE when the actual camera owner is released
  -> next queued/start request may run immediately
```

Late method/native/RTCM/ULog/artifact work reports against its original generation/time and cannot
change current state. No save, verification, package, review, or completeness barrier follows.

## Manual record

```text
Manual record -> timestamped video + fixed structured streams
Measurement Start -> stop Manual camera/video -> queued Measurement start
Measurement Stop -> IDLE; Manual record remains off
```

Starting Manual record during RUNNING Measurement is rejected with `end Measurement first`.

## Failure isolation

| Failure | Required result |
| --- | --- |
| one structured stream write | mark only that stream DEGRADED; measure and other streams continue |
| all storage unavailable | Storage UNAVAILABLE; Measurement still starts/runs |
| bulk queue full | drop oldest bulk records; preserve reserved result/event queue |
| result queue full | report critical drops; never block GUI/device callback |
| RTCM evidence failure | routing/injection continues |
| one camera unavailable | keep the available side and expose partial state |
| real camera owner busy | queue one start with factual reason |
| old Session/retry evidence | ignore for admission and leave bytes unchanged |
