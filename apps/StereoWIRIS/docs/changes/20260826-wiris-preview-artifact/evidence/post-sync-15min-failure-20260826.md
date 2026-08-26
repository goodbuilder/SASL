# Post-sync long-run preview/recording failure

## Window

- Application PID: 534818.
- Recording session: `20260826_155250_121849311`.
- User-triggered STUS restart completed; stable recording workers were
  `538456/538457`, both `nvh264dec`.
- Conservative visual/identity window start: 15:56:35.559 KST.
- 15-minute checkpoint: 16:12:01.736 KST (926 seconds).

## Boundary evidence

- NIC RX/TX error/drop/missed counters remained zero.
- Workers and NVIDIA decoder remained fixed through the 15-minute checkpoint.
- Identity JSONL had 24,070 records at the checkpoint; producer-to-queue,
  queue-to-QImage mismatch and reorder counts were zero.
- At 16:12:02 KST the Right preview contained clear lower-half block/mosaic and
  previous-region mixing during person/target motion. The full screenshot and
  Right crop are:
  - `/tmp/wiris-final-post-sync-acceptance-15m-app-20260826T1611KST.png`
  - `/tmp/wiris-final-post-sync-acceptance-15m-right-crop2-20260826T1611KST.png`
- During 16:11:58–16:12:06, Left emitted 81 identity frames and Right 78;
  every hash boundary matched and reorder count was zero. The corruption was
  already present in Right decoded pixels, before process queue/QImage/Qt.
- By 16:14:25 the same Right view was clean again without worker replacement.

## Recording failure after the checkpoint

At 16:16:20, Right `mp4mux` posted:

```text
Could not multiplex stream: Buffer has no PTS.
```

The application classified the bus error as an NVIDIA first-frame timeout,
ended the active recording and restarted both sides in `avdec_h264`. This was
an unexpected fallback and reset both RTCP Sender Report histories.

- Left MP4: 9,337,215,815 bytes, duration 1369.2145 s, container readable.
- Right MP4: 8,504,947,939 bytes, missing `moov`, ffprobe invalid.

This window fails preview artifact, unexpected fallback, GStreamer error,
continuous sync and stored-file acceptance criteria.
