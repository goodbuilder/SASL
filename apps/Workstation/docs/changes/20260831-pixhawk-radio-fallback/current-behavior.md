# Current Behavior

- Settings expose Left/Right MAVLink URL and baud as raw fields.
- `DualPixhawkMavlinkSubscriber` can create pymavlink connections from those endpoints and its RTCM
  sender is transport-neutral.
- Connect preflight always validates NIC, host address, peer IP, endpoint, and route as Ethernet.
- Post-connect stream setup can invoke Ethernet-specific `MAV_2_CONFIG`, ports, and rate handling.
- First-heartbeat/post-connect paths can start automatic parameter/calibration refresh work that
  reserves the receive path and adds traffic without a distinct operator command.
- Ethernet liveness states and RTCM fail-close are implemented per side.
- P1 TELEM1 readback showed bidirectional MAVLink at `/dev/ttyS6 @ 57600`; actual radio+RTCM field
  qualification has not been run.

## 2026-08-31 connected ground-radio inventory

- One unowned USB serial adapter was present: FTDI FT231X `0403:6015`, serial `DU0E49VU`, stable
  `/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E49VU-if00-port0`.
- USB descriptors identify the UART bridge, not the telemetry-radio product or air data rate.
- An eight-second read-only 57600-baud probe received zero bytes/messages. This does not reject the
  radio; the air peer may be absent/unlinked or the UART configuration may differ.
- A second independent ground radio was not present in this inventory.

## 2026-08-31 dual-radio powered inventory

- Two FT231X ground endpoints were present and unowned: `DU0E49VU` (`ttyUSB0`) and `D30JNMVH`
  (`ttyUSB1`). Two direct Pixhawk USB endpoints were also present.
- Ten-second passive 57600-baud Radio reads: `DU0E49VU` received zero bytes/messages;
  `D30JNMVH` received 141 parsed messages including 10 heartbeat and identities from both
  `sysid=1/component=1` and `sysid=2/component=1`.
- Independent five-second direct USB reads confirmed `ttyACM0` is P1 sysid 1 and `ttyACM1` is P2
  sysid 2, so both boards are powered and emitting telemetry.
- The current RF configuration is therefore not two isolated pairs: one ground endpoint receives
  both boards while the other receives neither.

## Read-only SiK AT inventory

- Both FT231X ground radios accepted SiK command mode at 57600 without any write/save/reboot command.
- `DU0E49VU` local `ATI5`: format 26, serial 57, air 64, NETID 25, TX power 20, ECC 0,
  MAVLink framing 1, 915000–928000 kHz, 50 channels, RTSCTS 0, MAX_WINDOW 131. Remote `RTI/RTI5`
  produced no response.
- `D30JNMVH` identified as `RFD SiK 2.0 on HM-TRP`; local and responding remote `ATI5/RTI5`
  matched on air 64, NETID 25, ECC 0, MAVLink 1, 915000–928000 kHz, 50 channels, and MAX_WINDOW 131.
- Both radios received `ATO`. A following passive check confirmed `D30JNMVH` returned to data mode
  and still received both sysid 1/2; `DU0E49VU` still received none.

## P1-only physical pair mapping

- With both ground radios connected, P2 powered off, and only P1 powered, simultaneous eight-second
  passive probes received `169` parsed messages and `8` heartbeat on `D30JNMVH`, all board telemetry
  from `sysid=1/component=1`.
- `DU0E49VU` received zero bytes/messages in the same interval.
- P1 air radio therefore currently maps to ground radio `D30JNMVH`; this is observed RF delivery,
  not a persistent app assignment or settings change.

## P2-only physical pair mapping

- With P1 powered off and only P2 powered, simultaneous eight-second passive probes received `154`
  parsed messages and `8` heartbeat on `D30JNMVH`, with board telemetry from
  `sysid=2/component=1`.
- `DU0E49VU` again received zero bytes/messages.
- Both air radios therefore currently participate in the same NETID-25 RF network received by
  `D30JNMVH`; `DU0E49VU` has no responding air peer.

## P2 NETID-26 split attempt

- With P1 off and P2 alone, operator approved changing only the P2 air remote and `DU0E49VU` ground
  from NETID 25 to 26.
- Through `D30JNMVH`, P2 remote `RTI5` confirmed NETID 25; `RTS3=26` returned OK, `RTS3?` returned
  26, `RT&W` returned OK, and `RTZ` was sent. D30 local remained NETID 25.
- `DU0E49VU` local `ATI5` confirmed NETID 25; `ATS3=26` returned OK, `ATS3?` returned 26, `AT&W`
  returned OK, and `ATZ` was sent. Later `ATI5` independently confirmed persisted local NETID 26
  and `ATO` data mode.
- P2 direct USB remained live as sysid 2, but two passive cross-checks (10 seconds and then 20
  seconds) received zero messages on both ground radios. The NETID-26 RF pair has not formed yet.
- No further setting write was attempted; a full P2 air-radio power reapply is the next boundary.

## P2 NETID-26 post-power-cycle recovery

- After a full P2/air-radio power reapply, a 15-second simultaneous passive check received `298`
  parsed messages and `15` heartbeat from `sysid=2/component=1` on `DU0E49VU`.
- `D30JNMVH` received zero bytes/messages in the same interval, while direct P2 USB independently
  remained live as sysid 2.
- P2 is now isolated onto the DU0E49VU/NETID-26 pair. Final acceptance still requires simultaneous
  P1+P2 power with zero cross-sysid delivery on both ground endpoints.

## Simultaneous dual-pair isolation

- With P1 and P2 powered together, a simultaneous 20-second passive check received only P2 sysid 2
  on `DU0E49VU`/NETID26 and only P1 sysid 1 on `D30JNMVH`/NETID25.
- DU/P2: 393 parsed messages, 19 heartbeat, 19.52 messages/s, maximum observed heartbeat gap 1.932 s.
- D30/P1: 400 parsed messages, 20 heartbeat, 19.95 messages/s, maximum observed heartbeat gap 1.124 s.
- Cross-board sysid delivery was zero on both endpoints. `(0,0)` radio/local frames remain separate
  from board identity and do not violate the expected P1/P2 mapping.

## Conflict

Typing a serial device into the existing URL field looks supported but is not a safe end-to-end
radio workflow because controller preflight and setup remain Ethernet-specific.

## 2026-09-01 Discovery UI defect

- `Find USB` appends its result to the existing candidate groups, so stale network/PC rows remain
  visible and the Discovery status no longer describes the displayed table.
- A serial endpoint with heartbeat evidence is appended twice, once as `USB` and again as
  `MAVLink`. Two physical FT231X radios therefore rendered as `FOUND 3 USB ENDPOINTS`.
- `Find network` replaces only the network group and leaves stale USB/PC rows in the same projected
  table.
- The table uses network-specific `Local IPv4` and `Assigned` columns for Radio rows. Radio rows
  show `--` even when heartbeat sysid identifies a role, while a shared Ethernet NIC can show
  `P1,P2`; this makes two independent Radio transports look merged.
- Runtime evidence showed `D30JNMVH` once with no heartbeat and `DU0E49VU` once with
  `sysid=2/component=1`, plus the duplicate `MAVLink` row for `DU0E49VU`. Saved P1/P2 Radio device
  fields remained empty; discovery itself did not assign or connect either side.

## 2026-09-02 active Drone Ethernet readiness and UI gap

- Direct USB identity verified the active Drone board as `sysid=3`, board version 53, UID2
  `0006000000003837393834335109002d004a`, disarmed.
- Read-only PX4 shell evidence shows a real `eth0` (`72:0e:d2:20:be:a6`) with address
  `10.0.0.3`, destination/router `10.0.0.10`, and link state `DOWN`; no Ethernet cable/carrier was
  present during the read.
- `/fs/microsd/etc/extras.txt` brings up `eth0` as `10.0.0.3/24`; `/fs/microsd/net.cfg` is absent.
- PX4 already owns one parameter-created Ethernet MAVLink instance with `MAV_2_CONFIG=1000`, mode
  0, broadcast 1, rate 2,000,000, and UDP local/remote port `14551`.
- These values duplicate P2 (`10.0.0.3:14551`) and must not be connected to the same Ethernet
  segment. The candidate Drone profile is a unique address such as `10.0.0.4` with Workstation
  port `14552`, subject to approval and a live address-conflict check.
- Workstation settings, subscriber ownership, endpoint selection, preflight, expected sysid 3,
  RTCM routing, and persistent PX4 setup code all contain a Drone Ethernet branch with default
  receive port 14552.
- The operator workflow is incomplete: `pixhawk_network_assignments`, network candidate Apply,
  network-role ownership, and visible persistent Ethernet/Shell/Reboot controls currently admit
  only P1/P2. Drone Ethernet therefore cannot be configured and connected safely end to end from
  the current UI even though its lower-level transport path exists.
- Read-only inspection changed no PX4, host-network, Workstation setting, or source file. The
  temporary probe was removed and all serial owners were closed.

## 2026-09-02 two-ground-radio topology evidence

- Drone identity was corrected and persisted as `sysid=3`; P1/P2 remain `sysid=1/2`.
- Both attempted shared P2P layouts failed before RTCM load: P1+Drone on D30 produced heartbeat
  gaps up to 6.0/12.0 seconds, while P1+P2 on D30 produced gaps up to 6.9/18.1 seconds.
- The dedicated peer on DU remained healthy in each comparison. Code-level sysid demultiplexing
  cannot repair the observed RF collision under the installed peer-to-peer SiK firmware.
- The experimental RF NETID swap was rolled back: D30 remains NETID25, P2 was restored to the
  DU/NETID26 pair, and Drone was restored to NETID25. The corrected Drone sysid3 was deliberately
  retained.

## 2026-09-03 removal baseline

The guided `Field topology setup` amendment was withdrawn before field use. The current application
uses the existing per-board transport and explicit Connect/Verify flow; the withdrawn topology
journal/controller/worker is no longer loaded or exposed. Existing saved device settings remain
unchanged. Runtime hardware state still requires independent field verification.

## 2026-09-04 P1/P2 IMU follow-up

- The operator stopped RTCM transmission and reconnected P1/P2; the IMU stream still did not
  recover. This removes current RTCM traffic as the primary explanation for the missing samples.
- The active Workstation settings file still selects `ethernet` for both `left_pixhawk_transport`
  and `right_pixhawk_transport`, while the P1/P2 Radio device paths are populated separately.
- The latest application log records repeated P1/P2 Ethernet `ROUTE MISMATCH` preflight failures;
  it contains no Radio connection or Radio IMU stream-request evidence. Therefore the observed
  reconnect did not reach the Telemetry Radio owner.
- The Radio subscriber starts with the reduced CRITICAL profile, which excludes
  `SCALED_IMU/2/3` until its optional-tier policy advances. A Telemetry-only acceptance run must
  first select the Radio transport, explicitly Connect, verify the expected sysid, and capture the
  subsequent Radio IMU request/sample evidence with RTCM priority preserved.
