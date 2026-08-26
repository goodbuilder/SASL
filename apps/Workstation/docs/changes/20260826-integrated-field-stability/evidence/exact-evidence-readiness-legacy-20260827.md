# Exact evidence, method readiness, and Legacy recording — 2026-08-27

## R8 — independent native-frame evidence audit

### RED and fix

The first independent recomputation found that the durable detection projection omitted
`timestamp_s`, even though that value is an input to `frame_descriptor_sha256`. The existing real
run had 440 repeated origin references whose compact/component FrameKey and stored descriptor agreed,
but whose descriptor digest was not self-recalculating from that origin row alone.

`DETECTION_METADATA_ALLOWLIST` now retains `timestamp_s`. New canonical Measurement and method rows
therefore carry every descriptor identity field. A focused RED test failed with
`KeyError: 'timestamp_s'` before the change and passes afterward.

`qualify_native_evidence.py` is a standard-library-only verifier. It does not import the product
ring, writer, or frame store. For one exact runtime generation it checks:

- strict JSONL and run-scoped request/origin linkage;
- compact FrameKey against camera/generation/frame components;
- descriptor SHA recomputation from all canonical identity fields;
- root-confined, unique artifact paths and unique request IDs;
- complete PNG chunk CRCs, IDAT decompression, RGB8/non-interlaced IHDR, dimensions and byte count;
- file bytes against artifact/event size and SHA-256;
- source FrameKey/descriptor against the selected run, never another run with a colliding key;
- pair ACK SHA-256 reconstructed from the two artifact rows; and
- one aggregate manifest SHA over every audited path, file hash, source key and descriptor.

For historical rows created before the allowlist fix, the verifier permits exactly one recovery:
`timestamp_s` may be supplied by the same saved artifact row whose path, bytes and fields are bound
into the pair ACK. No other descriptor input may be missing.

### Real artifact result

- Data root: canonical Workstation `measurement_data`.
- Runtime: `run-43dbe5aa53de410588ee0e2d4ba4a681`.
- 88 complete pair events: 44 method-evaluation pairs and 44 accepted-measurement pairs.
- 176 unique 1280×720 RGB8 PNGs, 487,531,968 total bytes.
- 88 unique request IDs and 176 unique paths.
- 440 compact/component origin checks and 176 historical timestamp reconstructions.
- Failure count: 0.
- Aggregate manifest SHA-256:
  `1849c402a0955feb154fe4733a9e7a9e8ad9856f7e6cc198158332e443a89ff2`.
- Qualification JSON SHA-256:
  `2785060f76d7aef4420b021167febe84c38fd43cf013a8e0e2e36064006fff7f`.
- Input JSONL SHA-256:
  - `native_frames.jsonl`: `81ea740f2e73fcc598fb15a5112ef3e1de7b10cf959eb83d68a88cc7b5d34137`;
  - `measurements.jsonl`: `838cdb9b8cbfc27dc6bd53b38228144cfd774caa074a897ff3646cead29d84ef`;
  - `method_evaluations.jsonl`: `15d50f55fc70aa2623d6d0439cd46a11305eb6257099ead49ebf026465041bb1`.

Focused qualifier/writer/ring/canonical evidence tests: 37 passed. The complete Workstation suite
after R8/R12/R13 changes is 1,466 passed and 1 skipped.

## R12 — method readiness is not a global Start gate

The same real runtime is a field boundary for the all-blocked case:

- one `measurement_start_requested` preceded the capture;
- exact operating K,D was inactive in all 88 Measurement rows;
- camera clock evidence was `pc-offset-failed` in 72 rows and `pair-delta-failed` in 16;
- 44 unique correspondence pairs were retained;
- every one of the eight method IDs produced one terminal outcome per pair;
- all 352 outcomes were truthfully `BLOCKED`; and
- the run still stopped normally instead of fabricating a valid result or disabling collection.

An offscreen real MainWindow matrix separately covered missing K,D, operating-zoom mismatch, missing
stereo R,t, missing correspondence, missing live Pixhawk/GNSS, and failed STUS/RTCP. In every case
the method catalog remained visibly BLOCKED while both the command and visible `측정 시작` button
were enabled and invoked the lifecycle owner. Related readiness/fill/canonical/UI regressions:
108 passed.

This is an admission PASS, not a geometry or timing PASS. R3 remains CRITICAL and no BLOCKED method
outcome is promoted to a Measurement result.

## R13 — Legacy no-mutation, restart, and MP4-start boundary

### Network and encoder ownership preflight

The Legacy saved/default camera host address was still `10.0.0.10/24`, colliding with the isolated
Pixhawk NIC's connected subnet. Both Workstation and Legacy defaults, plus the Legacy saved field,
now use camera source `10.0.0.11/32`; explicit `.231/.232` host routes remain the network owner.

Source audit found no automatic caller of the stored encoder-profile SET helper. A capture covering
Legacy startup, `영상 수신 확인`, Preview, two normal recordings, two restarts, and the new direct
record-start path contains exactly zero TCP/2240 packets:

- pcap: `/tmp/legacy_r13_control_20260827.pcap`;
- size: 24-byte pcap header;
- tcpdump: 0 captured, 0 received by filter, 0 kernel drops;
- SHA-256: `704e5e5b3234433c01fcfd1b20a306e77e985038120492dc53965c3edd38a4ea`.

Legacy `--check --mavlink-only --ui legacy` passed the canonical interpreter, GStreamer, NVIDIA,
settings, model and import gates. Startup itself generated no SDK traffic. `영상 수신 확인` returned
Left/Right `frame-ok`, and Preview reported 1280×720 at 18.8/15.2 Hz without SDK traffic.

### Recording RED and GREEN

With Preview already connected, session `20260827_050056_101888877` recorded and finalized both MP4
files. A clean restart then exposed the stronger RED: pressing `세션 기록 시작` without first
starting Preview created session `20260827_050359_648097447` and displayed recording active, but
owned no camera worker/socket and created no MP4. It nevertheless finalized an empty session.

Legacy recording now treats the action as a camera/video request:

- both configured sides are expected before their first GUI frame;
- a disconnected Preview causes both recording streams to start immediately;
- a synchronous stream-start rejection rolls visible recording state back, retains a retryable
  failure and restores `세션 기록 시작`; and
- later side failure remains an explicit missing/zero/invalid MP4 finalization failure.

The repeated blank-start GREEN created distinct session `20260827_050834_833389066`, immediately
owned two camera children, produced both MP4s, completed normal Stop/EOS/save, and finalized its
manifest. Final sizes and byte hashes were:

- Left: 293,564,454 bytes,
  `fd2dca19c2fc5e42e0f40bcbf86512ba1e5009fb1ac8cd0a3a4b45df63b78d45`;
- Right: 255,449,104 bytes,
  `c46a196f2ae05c4e0323fcf9a54088a523960bb423f135a8596c43ab37bcce45`;
- manifest:
  `e9575b509265dd40e92b4709f9798dd53bec8b91ee3f204e379192f6eb255f38`.

After Quit there was no Legacy process, camera child, established SDK/RTSP socket, Python shared
memory or Workstation shared memory.
Independent post-run GETs returned the same value on both sides: `HIWS=OK`, `GFWV=2.18.72`,
`GETH=TRUE`, visible `0/1/30/1`, `GSSS=0`, and `IRCR=FALSE`.

### MP4 remains independently failed

Creation/EOS does not close R7. The latest strict validator found:

- Left: 745 samples, 669 decoded, 76 decoder-visible loss, IDR 0, 56 frame-number gaps, final
  `stts=0`;
- Right: 795 samples, 745 decoded, 50 decoder-visible loss, IDR 0, 28 frame-number gaps, final
  `stts=0`.

Both are `recording_usable=false`. This is the same FW 2.18.72 sender blocker as Workstation, not a
remaining Legacy admission/path-allocation defect.

Legacy support/settings/network/runtime tests passed 95; focused encoder/session/start/save tests
passed 16. The monolithic `test_stereo.py` process still reproduced the known order-dependent native
Qt segmentation fault near `test_gnss_base_candidate_rows_apply_selected_port`; the changed tests
pass in the isolated selections above.

Coordinate contract checked: this slice adds timestamp/byte identity and lifecycle ownership only;
it does not move or reinterpret NED, PX4/body FRD, OpenCV optical RDF, stereo-center, left/right
camera, GPS/RTK, or public `baseline_center_frd` values.
