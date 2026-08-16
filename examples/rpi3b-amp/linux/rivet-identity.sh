#!/usr/bin/env bash
# Give the combined system an identity of its own.
#
# Stock Pi OS presents itself as Pi OS with a daemon running, which is not
# what this is. These are the files the rest of the userland actually
# reads: hostnamectl, the login prompt, and anything that parses
# os-release all pick this up without being told.
#
# /etc/os-release keeps its Debian-derived fields (ID_LIKE, VERSION_CODENAME
# and friends) because package tooling depends on them. Only the
# presentation layer changes.
set -euo pipefail

ROOT="${1:-}"           # empty for the running system, or a mounted rootfs
VERSION="${2:-0.3.0}"

f() { printf '%s' "$ROOT$1"; }

# Preserve whatever the distro said, then re-badge the presentation.
if [ ! -f "$(f /etc/os-release).stock" ]; then
    sudo cp -p "$(f /etc/os-release)" "$(f /etc/os-release).stock"
fi

sudo tee "$(f /etc/rivet-release)" >/dev/null <<REL
RIVET_SYSTEM_VERSION=$VERSION
RIVET_SYSTEM_NAME="Rivet RTOS + Linux"
RIVET_BOARD="Raspberry Pi 3 Model B"
RIVET_RTOS_CORE=3
REL

# Rewrite only the presentation fields, keeping the rest verbatim.
sudo awk -v ver="$VERSION" '
    /^PRETTY_NAME=/ { print "PRETTY_NAME=\"Rivet RTOS + Linux " ver " (Raspberry Pi 3B)\""; next }
    /^NAME=/        { print "NAME=\"Rivet\""; next }
    /^VERSION=/     { print "VERSION=\"" ver "\""; next }
    /^HOME_URL=/    { next }
    { print }
    END {
        print "HOME_URL=\"https://github.com/habeelali/rivet-rtos\""
        print "RIVET_SYSTEM_VERSION=\"" ver "\""
        print "VARIANT=\"RTOS on core 3, Linux on cores 0-2\""
        print "VARIANT_ID=rivet-amp"
    }
' "$(f /etc/os-release).stock" > /tmp/os-release.new
sudo cp /tmp/os-release.new "$(f /etc/os-release)"
rm -f /tmp/os-release.new

# Pre-login banner. \4 is the IP, filled in by agetty.
sudo tee "$(f /etc/issue)" >/dev/null <<'ISSUE'

  Rivet RTOS + Linux  \s \r \m
  \4

ISSUE

sudo tee "$(f /etc/motd)" >/dev/null <<'MOTD'

   ██████  ██ ██    ██ ███████ ████████
   ██   ██ ██ ██    ██ ██         ██
   ██████  ██ ██    ██ █████      ██
   ██   ██ ██  ██  ██  ██         ██
   ██   ██ ██   ████   ███████    ██   RTOS core 3 / Linux cores 0-2

   rivet status     rivet images     rivet --help

MOTD

echo "identity set to version $VERSION"
