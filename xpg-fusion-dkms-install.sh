#!/bin/bash
# Install xpg-fusion via DKMS (dkms install).
# Usage: sudo ./xpg-fusion-dkms-install.sh [kernel-version]
set -euo pipefail

PKG_NAME="xpg-fusion"
PKG_VERSION="1.0"
MOD_NAME="xpg-fusion"
KERNEL="${1:-$(uname -r)}"

[[ $(id -u) -eq 0 ]] || { echo "ERROR: run as root (sudo)" >&2; exit 1; }

if ! dkms status | grep -q "^${PKG_NAME}/${PKG_VERSION}"; then
    echo "ERROR: ${PKG_NAME}/${PKG_VERSION} not added. Run the build script first." >&2
    exit 1
fi

dkms install "${PKG_NAME}/${PKG_VERSION}" -k "${KERNEL}"

# Если БП уже подключён, udev не повторяет modalias-matching — подтолкнуть:
udevadm trigger

echo ">> Installed:"
ls -l "/lib/modules/${KERNEL}/updates/dkms/${MOD_NAME}.ko"*
