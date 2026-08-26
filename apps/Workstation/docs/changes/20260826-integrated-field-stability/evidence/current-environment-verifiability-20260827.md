# Current-environment verifiability — 2026-08-27

## Available boundaries

- Workstation/StereoWIRIS/camera children were initially absent; UDP 14550/14551 were free.
- The devices were later physically isolated: cameras use `enx588694fce07e` at `10.0.0.11/32`
  with preferred-source host routes. The first P1/P2 adapter `enx8cb0e9e8f8ee` was physically
  replaced; the current P1/P2 RTL8156B is `enx0023a40a2e39` at `10.0.0.10/24` with explicit
  `.2/.3` routes. Both current NICs are UP/LOWER_UP and all four devices answer ping.
- Both RTSP 8554 endpoints and NGII caster 2101 accepted TCP; two PX4 FMUv6X USB CDC devices were
  present as ttyACM0/1 (both expose ambiguous serial `0`).
- RTX 4050/driver 595.84, required GStreamer factories/NVDEC, ffmpeg/ffprobe, NTP and about 1 TiB
  free storage were available.
- Orca/AT-SPI remained saturated. Accessibility was not disabled, making this a valid adversarial
  condition for the post-fix Connect gate.
- Session credential entry enabled a definitive isolated NTRIP Stop/reconnect run: HTTP 200,
  120 valid GGA, station/MSM content and 636.5 seconds of dual injection failure zero. The secret
  remains unsaved and must be re-entered after a future process restart.
- R10 clean-host reproduction is now complete in a separate official Ubuntu 22.04.5 KVM guest:
  exact pushed source, manifest-only runtime installation, package closure, both launchers and
  isolated HOME all pass.

## Verifiable now, severity order

| Severity | Slice | Current-environment oracle |
|---|---|---|
| Critical | post-fix Pixhawk Connect freeze | UI click/page progress, Recv-Q drain, correct sysids/HB/ATT, 15 command-511 requests per side, continuing GCS heartbeat |
| Critical | high-rate telemetry-only and NIC-isolated camera-load MAVLink | first isolated adapter Preview 10m PASS; replacement RTL8156B direct load 45s and latest-app 10m/PARAM 124-of-124 PASS; cold boots/final 60m pending |
| Critical | real dual MP4/evidence/resource integrity | growth/EOS/`ftyp`/ffprobe/strict decode are necessary but insufficient; require first-VCL IDR, sample-vs-decoded equality, frame-number continuity, positive final sample duration, sane rate, FrameKey/hash and zero residual resources |
| High | session admission and abnormal restart | current-frame ACK, new run/file IDs, disposable data-root SIGKILL, restart and new session without adopting old mutable state |
| Critical | real STUS continuous health | visibly Apply 7.0 s, exact Left OK before Right OK, replacement/new SR, limits held for at least 25 minutes |
| High | no automatic camera mutation | Workstation and Legacy automatic-path zero-SET FIELD PASS; operator later approved only IDR `0->1`, so current hardware oracle is `0/1/30/1`, not stored JSON |
| High | method admission negative paths | Start remains available while only dependent methods are BLOCKED; no invalid result becomes valid |

Hardware owners must run sequentially. Camera-only can collect MP4, evidence and resource metrics in
one run, but NTRIP and camera A/B remain separate until each independent branch passes.

## Conditionally verifiable

| Slice | Missing condition |
|---|---|
| RTK convergence | credential plus fresh LEFT_GNSS GGA, station/MSM RTCM, open-sky antennas and stable fix_type 6 on both sides |
| positive geometry/method result | operator-selected verified current-zoom intrinsics/extrinsics; configured active path is missing |
| visual corruption/freeze authenticity | controlled visible motion in front of both cameras; metadata alone cannot prove current imagery |

## Not verifiable without new physical authority or external change

| Slice | Reason |
|---|---|
| exact PX4 #25956 IOB/sendto proof and firmware root fix | needs PX4 task/IOB instrumentation or diagnostic firmware approval |
| remote-reboot profile rearm on the same host socket | needs named-side physical power cycle or approved reboot action |
| three independent cold boots | needs user physical power cycles after every short gate passes |
| corruption-free result on current known-defective sender | prior camera-origin RTP omission/non-IDR evidence; successful resolution needs vendor/fixed sender plus motion |
| final 60-minute integrated acceptance | blocked until all short gates pass, NTRIP credential is available and physical cold boots complete |
