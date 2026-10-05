#!/usr/bin/env bash
set -euo pipefail

# Install Gazebo Sim Harmonic on Ubuntu Jammy, matching the notebook's 8.x
# runtime. The script intentionally leaves the existing Fortress packages in
# place because ROS 2 Humble's apt ros_gz packages are Fortress-linked.

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y curl lsb-release gnupg
"${SUDO[@]}" curl -fsSL \
  https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

repo_file=/etc/apt/sources.list.d/gazebo-stable.list
printf 'deb [arch=%s signed-by=%s] https://packages.osrfoundation.org/gazebo/ubuntu-stable %s main\n' \
  "$(dpkg --print-architecture)" \
  /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  "$(lsb_release -cs)" | "${SUDO[@]}" tee "${repo_file}" >/dev/null

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y gz-harmonic

gz sim --version
