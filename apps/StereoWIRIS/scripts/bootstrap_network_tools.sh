#!/usr/bin/env bash
set -euo pipefail

if command -v arp-scan >/dev/null 2>&1; then
  echo "PASS network.arp_scan: arp-scan is available."
  exit 0
fi

printf '%s\n' \
  "WARN network.arp_scan: arp-scan is unavailable; bounded TCP discovery remains available." \
  "  Recovery: sudo apt-get update" \
  "  Recovery: sudo apt-get install arp-scan"
exit 0
