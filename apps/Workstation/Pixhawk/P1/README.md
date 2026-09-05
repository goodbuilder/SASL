# P1 Pixhawk configuration snapshot

현재 P1 Ethernet RX 장애의 통합 진단 기록은
[`ETHERNET_RX_DIAGNOSIS.md`](ETHERNET_RX_DIAGNOSIS.md)를 참조한다.

Captured at 2026-08-30T14:57:59+09:00 from the USB device verified as MAVLink
system ID 1. The capture was read-only: no parameter, SD-card file, reboot, or
firmware command was sent.

## Artifacts

- extras.txt: byte-exact MAVLink FTP copy of /fs/microsd/etc/extras.txt.
- net.cfg.missing.json: records that /fs/microsd/net.cfg was absent.
- ethernet_params.json: PX4 parameter-storage status and P1 MAVLink Ethernet
  parameter readback.
- board_identity.txt: PX4 and board identity returned by ver all.
- ethernet_runtime.txt: eth0 and MAVLink runtime status at capture time.
- workstation_endpoint.json: the P1 values configured on this Workstation.
- parameters_backup.bson: byte-exact microSD parameter backup after decoding
  parameters_backup.bson.base64.
- manifest.json: provenance, remote paths, byte counts, and hashes.

The PX4 parameter store reported /fs/mtd_params as its live storage and
/fs/microsd/parameters_backup.bson as its backup. MAVLink FTP exposes
/fs/mtd_params as a zero-byte virtual path, so it is recorded in the manifest
and is not represented as a usable empty backup.

## 2026-08-31 official stock firmware reinstall

At the operator's request, P1 was reinstalled with the exact repository-pinned
official PX4 v1.17.0 FMUv6X artifact. The uploader detected `board_id=53`
before erase and used no force override. Erase, program, verify and reboot all
passed. Fresh pre-flash backups are:

- `parameters-backup-pre-flash-20260831T1003KST.bson`
- `extras-pre-flash-20260831T1003KST.txt`

Post-flash USB readback retained P1 sysid `1`, all required `MAV_2_*` values,
`10.0.0.2/24`, `extras.txt`, and the same PX4 git hash. The reinstall did not
repair P1 Ethernet receive: ARP remained `90 requests / 0 replies`, while P2
was `5/5`. During the first 89.049 seconds of independent post-flash capture,
P1 sent 87 heartbeat and 1,299 ATTITUDE messages but zero `SCALED_IMU/2/3`;
the messages returned only after verified USB runtime stream activation.

See `firmware-reinstall-20260831T1003KST.json` for firmware/uploader hashes,
bootloader identity, exact readback, pcap provenance and the restored state.
The remaining fault is P1's board-side Ethernet PHY/driver/NuttX RX boundary,
not an unverified firmware-image installation. Physical damage is not yet
proven. The latest source/runtime comparison has, however, confirmed a more
specific failure mechanism: P1 records `partner IP: 10.0.0.2`, transmits every
payload once to itself and once by broadcast, and receives almost exactly two
copies of every MAVLink sequence. Current Ethernet rates remain about
`2.18 kB/s TX` and `4.35 kB/s RX`, with only self `sysid=1/component=1` and
reported loss `126.9%`; ARP remains `INCOMPLETE` and host ping remains 100%
loss. Exact IPv4-ID and sequence math, likely PHY/RMII/duplex causes, lower
probability resource hypotheses, and the ordered next-test matrix are in
[`ETHERNET_RX_DIAGNOSIS.md`](ETHERNET_RX_DIAGNOSIS.md). Do not change P1
network files or parameters, or claim USB-free Ethernet recovery, without
following that one-variable-at-a-time plan.

### Ten-minute post-reflash ATTITUDE/GNSS transport check

An independent 619.397-second Ethernet capture recorded 32,034 consecutive
P1 MAVLink 2 packets with zero duplicate, missing or reordered sequence and
zero kernel capture drops. ATTITUDE was 15.0 Hz with a 71.5 ms maximum gap;
GPS1 was 5.0 Hz with a 204.4 ms maximum gap; GPS2 was 1.0 Hz with a 1.003 s
maximum gap; heartbeat was 1.0 Hz. Twenty-one UI checkpoints kept P1 `LIVE`
with no all-RX stall.

This is a transport PASS, not a GNSS solution PASS. Both GPS messages carried
`fix_type=0`, `satellites_visible=0`, and zero latitude/longitude, so the UI
correctly remained `NO FIX`. P1's outbound Ethernet telemetry is stable while
its inbound ARP/GCS/RTCM path remains failed. See the JSON record for exact
counts, gaps and pcap provenance.
