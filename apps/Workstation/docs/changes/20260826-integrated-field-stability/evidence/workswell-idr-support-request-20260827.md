# Ready-to-send Workswell support request — visible H.264 IDR

## Suggested subject

WIRIS Enterprise FW 2.18.72: `GEIR VISIBLE=1` but RTSP `/visible` emits no H.264 IDR NAL units

## Message

Hello Workswell Technical Support,

We operate two WIRIS Enterprise cameras through the WIRIS Ethernet Stream SDK and RTSP. Both units
show the same visible-stream random-access problem on firmware 2.18.72.

Device identification:

| Side | Serial number | Article number | Firmware |
|---|---|---|---|
| Left | `20204-046-2505` | `WWE-US-640-SNL-256-LRF` | `2.18.72` |
| Right | `20207-046-2507` | `WWE-US-640-FNL-128-LRF` | `2.18.72` |

Final device state on both cameras:

```text
HIWS = OK
IACT = TRUE
GETH = TRUE
GEBR VISIBLE = 0
GEQZ VISIBLE = 1
GEGS VISIBLE = 30
GEIR VISIBLE = 1
GSSS = 0
IRCR = FALSE
RTSP = rtsp://<camera>:8554/visible
```

We applied the documented `SEIR VISIBLE 1` command separately to each camera with no RTSP client
active. Each command returned exact `OK`, and `GEIR VISIBLE` returned `1`. Fresh 15-second elementary
stream captures still contained zero NAL type 5 access units. FFmpeg marked every observed I-picture
as `key_frame=0`.

We then rebooted one camera at a time using `REBT`, verified an actual down/up transition, restored
the approved operating profile where necessary, and repeated the test. IDR reset to `0` after reboot.
Reapplying `SEIR VISIBLE 1` both before RTSP connection and while RTSP was active again returned
`OK`/readback `1`, but every fresh stream still contained zero NAL5. The streams instead contain
periodic non-IDR NAL1 I-pictures and reference/frame-number discontinuities.

Representative results:

- Left after reboot: 219 NAL1 VCL AUs, seven SPS/PPS sets, zero NAL5.
- Left after active-stream SET: 214 AUs, five non-IDR I-pictures, zero NAL5.
- Right after restored profile: 214 AUs, eight non-IDR I-pictures, zero NAL5.
- Right after active-stream SET: 269 AUs, eight non-IDR I-pictures, zero NAL5.

The issue is observable over RTSP/TCP on an isolated camera NIC with zero NIC errors or drops. Two
independent MP4 recordings also show zero IDR, decoder-visible frame loss, reference discontinuity,
and invalid final sample timing. Decoder concealment can display frames, but it does not provide a
safe random-access/recovery boundary after packet loss.

Could you please confirm:

1. On FW 2.18.72, is `GEIR VISIBLE=1` intended to produce H.264 NAL type 5 IDR pictures, or does the
   setting only control non-IDR intra pictures?
2. Is there a documented visible encoder restart or force-IDR command not present in Ethernet Stream
   SDK manual revision 241106EN?
3. Which firmware version fixes periodic IDR emission for WIRIS Enterprise `/visible` RTSP?
4. Please provide the approved firmware image/download, SHA-256, compatibility notes, installation
   steps, rollback procedure, and whether encoder settings persist across that update.

Our acceptance criterion is a fresh RTSP client observing SPS/PPS followed by a genuine NAL type 5
IDR at a bounded periodic interval, with independent software decode from that boundary and no
reference/frame-number discontinuity in an otherwise loss-free interval.

We can provide the elementary-stream and control pcaps listed below on request.

Thank you.

## Attachment inventory

| Artifact | SHA-256 |
|---|---|
| Initial `SEIR VISIBLE 1` control pcap | `f77ee9b52aead41dd322a97afa4c74f41d443583c08f19c741f45d4627dc91ca` |
| Left post-reboot H.264 | `e2a4225444e8b77a8edb2a99212e629f89c37f176a5a9e3b75538e38573fde15` |
| Left active-SET H.264 | `80dc48f16a5d7888575b1b664fe8a134470f478084037313ceee64af38c02aa3` |
| Right reboot control pcap | `27069cbf8754c2244955fdd9e41d9679bae86ab842adb7e159849abfb7bcf14e` |
| Right restore control pcap | `1781f7d4b3dd8efe8644a4c6b3b3f5f2eb3a9fa4ad8d33ead37f631189991a06` |
| Right post-restore H.264 | `920496ee0c2b03a381b6dc3e46c0f03f682c15c5c0a35f1c52068bb85783997e` |
| Right active-SET H.264 | `f863130be13517ba856d3d4b4a2bdc642ce7e9fbb69a48775721f39a521511a0` |

Do not attach settings files, NTRIP credentials, keyring data, unrelated telemetry, or complete
operator recordings. Verify each selected attachment against the hash above before sending.
