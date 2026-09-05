# Proposed P2 Ethernet configuration

These files were derived from the P1 layout and the Workstation P2 contract,
with P2-specific identity values substituted:

- MAVLink system ID 2
- eth0 address 10.0.0.3/24
- host/router 10.0.0.10
- MAVLink UDP local and remote port 14551

The approved values were applied and verified after reboot on
2026-08-30T15:26:25+09:00. Board-specific calibration values were never
copied. net.cfg passed byte readback before reboot but was absent after boot;
extras.txt remained present and byte-exact.
