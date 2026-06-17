#!/usr/bin/env bash
set -euo pipefail

if command -v arp-scan >/dev/null 2>&1; then
  echo "[deps] arp-scan already available."
  exit 0
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "[deps] apt-get unavailable; skipping arp-scan install." >&2
  exit 0
fi

if [[ "$(id -u)" -eq 0 ]]; then
  apt-get update
  apt-get install -y arp-scan
  exit 0
fi

if ! command -v sudo >/dev/null 2>&1; then
  echo "[deps] sudo unavailable; install arp-scan manually for ARP camera discovery." >&2
  exit 0
fi

if ! sudo -n true >/dev/null 2>&1; then
  echo "[deps] sudo requires a password; install arp-scan manually or run sudo once before launching." >&2
  exit 0
fi

sudo apt-get update
sudo apt-get install -y arp-scan
