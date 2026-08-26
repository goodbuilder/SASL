# Recording-start filesink root cause

## Reproduction

- At `2026-08-26T15:19:49+09:00`, recording session
  `20260826_151949_360723872` replaced preview workers `512605/512606` with
  `513832/513833`.
- Both workers exhausted the 0.5/1/2/4 second initial-PLAYING retry. Neither
  `raw_left.mp4` nor `raw_right.mp4` was created and both panes remained at
  `YOLO 시작 중...`.
- The first filesystem defect was that the session had `.spool/` but no
  `video/` directory. A RED regression proved that prepared raw-video path
  parents did not exist.

## Exact bus error after creating `video/`

`GST_DEBUG=2` on session `20260826_153342_386571311` exposed the remaining
failure:

```text
filesink: Could not open file "'/home/moontaehyung/바탕화면/.../video/raw_left.mp4'" for writing.
filesink: system error: No such file or directory
```

The single quotes were part of the filesink property value. `_gst_location()`
used shell-oriented `shlex.quote()`. A non-ASCII path such as `바탕화면`
therefore became a single-quoted token, while `Gst.parse_launch()` preserves
single quotes literally. The process tried to open a path whose first byte was
`'`, not `/`.

A direct property round trip demonstrated the parser behavior:

```text
"'/tmp/한글 path/test.mp4'" => "'/tmp/한글 path/test.mp4'"
'"/tmp/한글 path/test.mp4"' => '/tmp/한글 path/test.mp4'
```

## Fix and verification

- `_prepare_video_recording_paths()` creates each computed path's parent.
- `_gst_location()` uses GStreamer double-quoted property serialization and
  escapes backslashes/double quotes; it no longer uses shell quoting.
- The regression round-trips a non-ASCII path containing a space and a double
  quote through a real `Gst.parse_launch()` filesink property.
- `tests/test_yolo_stream.py`: `61 passed`.
- Focused startup/video-parent selection: `4 passed`.
- New process PID `527641`, session `20260826_153835_987796434`:
  preview workers `527898/527899` were replaced by recording workers
  `528513/528514`; both stayed on `nvh264dec`, both RTSP sessions connected,
  and Left/Right MP4 bytes increased.

The PLAYING retry remains bounded defensive recovery, but it did not fix or
mask this deterministic path-serialization failure.
