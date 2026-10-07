#!/bin/bash
# Build xpg-fusion via DKMS (dkms add + dkms build).
# Usage: sudo ./xpg-fusion-dkms-build.sh [kernel-version]
set -euo pipefail

PKG_NAME="xpg-fusion"
PKG_VERSION="1.0"
MOD_NAME="xpg-fusion"
SRC_DIR="${SRC_DIR:-$(cd "$(dirname "$0")" && pwd)}"
KERNEL="${1:-$(uname -r)}"

[[ $(id -u) -eq 0 ]] || { echo "ERROR: run as root (sudo)" >&2; exit 1; }
[[ -f "${SRC_DIR}/dkms.conf" ]] || { echo "ERROR: no dkms.conf in ${SRC_DIR}" >&2; exit 1; }

if dkms status | grep -q "^${PKG_NAME}/${PKG_VERSION}"; then
    echo ">> ${PKG_NAME}/${PKG_VERSION} already added, skipping dkms add"
else
    dkms add "${SRC_DIR}"
fi

dkms build "${PKG_NAME}/${PKG_VERSION}" -k "${KERNEL}"

echo ">> Built for ${KERNEL}:"
find "/var/lib/dkms/${PKG_NAME}/${PKG_VERSION}" \( -name "${MOD_NAME}.ko" -o -name "${MOD_NAME}.ko.zst" \) -print 2>/dev/null || true
