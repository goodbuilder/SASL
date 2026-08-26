# Pre-change Saved MP4 Strict Decode — 2026-08-26

Sample session: `recordings/sessions/20260825_155445_030256238/video`

## Left

- File: `raw_left.mp4`, 18,719,091 bytes, 38 H.264 packets, 1.948 s
- ffprobe reports `Invalid level prefix`, `corrupted macroblock`, and macroblock decode errors.
- `ffmpeg -loglevel error -xerror -i raw_left.mp4 -map 0:v:0 -f null -` exits 1.
- Tolerant command without `-xerror` emits the errors but exits 0.

## Right

- File: `raw_right.mp4`, 15,046,074 bytes, 36 H.264 packets, 1.846 s
- ffprobe reports invalid intra prediction, invalid level prefix, and macroblock decode errors.
- Strict full decode exits 1; tolerant decode emits errors/non-monotonic DTS warnings but exits 0.

## Interpretation

The GStreamer recording tee is upstream of `nvh264dec`, so NVIDIA-created concealment pixels are
not written to MP4.  The encoded stream is nevertheless damaged.  A tolerant software player hides
the difference by dropping or concealing unusable dependency frames, whereas NVIDIA emits many of
them as visually corrupted frames.  “Looks normal in a player” is therefore not evidence of a clean
stored bitstream.
