---
type: protocol
year: 2003
zotero_item: PQM6RCY3
bibtex_key: schulzrinne_rtp_2003
tags:
  - workstation
  - RTCP
---

# RFC 3550 — RTP: A Transport Protocol for Real-Time Applications

**Henning Schulzrinne, Stephen Casner, Ron Frederick, Van Jacobson.** RFC 3550 (2003). [IETF RFC Editor](https://datatracker.ietf.org/doc/rfc3550/)

**Read status:** Section 6.4.1 checked against the IETF Datatracker version; metadata verified against the RFC record.

## Relevant contract

An RTCP Sender Report associates an NTP timestamp with an RTP timestamp representing the same sampling instant. The RFC explains that this pair can synchronize streams on a common timebase and can be used to estimate nominal RTP clock frequency. This is the protocol basis for interpreting the two rate estimates in [[Workstation - Camera clock drift]]. It does not guarantee that a camera timestamp equals exposure time; that requires device-specific evidence.

