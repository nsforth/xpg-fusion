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

# If the PSU is already connected, no new 'add' uevent will arrive, so udev
# never runs the modalias modprobe (it only does so for add/remove events).
# Load the driver directly: the usb core probes already-present devices when
# a driver registers. If the PSU is not connected yet, the driver simply
# waits for it — same as any other USB driver.
# modprobe only acts on the RUNNING kernel: skip it for cross-kernel installs
# and never fail the script over it (the install itself already succeeded).
if [[ "${KERNEL}" == "$(uname -r)" ]]; then
    modprobe "${MOD_NAME}" || echo "WARN: modprobe ${MOD_NAME} failed (install itself succeeded)" >&2
else
    echo ">> Skipping load: ${KERNEL} is not the running kernel; run 'modprobe ${MOD_NAME}' after booting it"
fi

echo ">> Installed:"
ls -l "/lib/modules/${KERNEL}/updates/dkms/${MOD_NAME}.ko"*
