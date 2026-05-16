#!/bin/bash
##
##  scripts/setup_pi.sh — Internal helper for `./launch.sh setup`
##  Author: bugrASl
##  Date:   April 2026
##  Version: v2.7
##
##  ────────────────────────────────────────────────────────────────────
##  THIS IS A HELPER SCRIPT — invoked only by ./launch.sh setup.
##  Users should never invoke this directly. The user-facing API is:
##
##      ./launch.sh setup
##
##  ────────────────────────────────────────────────────────────────────
##
##  WHAT IT DOES (one-time-per-Pi setup):
##      1. Self-elevates via sudo (one prompt regardless of how many
##         privileged steps it runs).
##      2. Installs the build toolchain and Python deps.
##      3. Enables SPI and I²C in /boot/firmware/config.txt.
##      4. Appends isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3 to
##         /boot/firmware/cmdline.txt (real-time core isolation).
##      5. Creates spi/i2c/gpio groups, drops a udev rule, adds the
##         real user to all three groups.
##      6. Creates /opt/cpcu/{bin,scripts,python,models,test} and
##         /var/log/cpcu and chowns them to the real user.
##      7. Symlinks /opt/cpcu/config.json -> <repo>/config/runtime.json.
##
##  Idempotent — safe to re-run. Skips work that's already done.
##
##  Exit codes:
##      0   success, no reboot needed
##      10  success, reboot required (caller should prompt the user)
##      *   error
##
##  v2.7 changes:
##      - Moved from cpcu_v2/setup_pi.sh to cpcu_v2/scripts/setup_pi.sh.
##      - REPO_RUNTIME path resolution climbs one directory up.
##      - Added /opt/cpcu/python/ to the directory list.
##      - Returns exit code 10 instead of printing reboot prompts.

set -e

##============= SELF-ELEVATE ===============================================================

if [ "$(id -u)" -ne 0 ]; then
    echo "[setup_pi] Re-exec under sudo (you'll be prompted for your password)..."
    exec sudo --preserve-env=HOME,USER,SUDO_USER "$0" "$@"
fi

REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"
echo "=== CPCU v2.7 Raspberry Pi Setup ==="
echo "  Acting as: root  (real user: ${REAL_USER})"
echo ""

##============= APT DEPENDENCIES ===========================================================

echo "[1/6] Installing system packages..."
apt update
apt install -y \
    build-essential \
    cmake \
    libncurses-dev \
    i2c-tools \
    tmux \
    python3-numpy \
    python3-scipy \
    python3-pip

##============= PYTHON DEPENDENCIES ========================================================

echo "[2/6] Installing Python packages..."
pip3 install --break-system-packages \
    joblib \
    scikit-learn

##============= DIRECTORY STRUCTURE ========================================================

echo "[3/6] Creating directory structure..."
mkdir -p /opt/cpcu/bin
mkdir -p /opt/cpcu/scripts
mkdir -p /opt/cpcu/python
mkdir -p /opt/cpcu/models
mkdir -p /opt/cpcu/test
mkdir -p /var/log/cpcu

chown -R "${REAL_USER}:${REAL_USER}" /opt/cpcu /var/log/cpcu

##============= RUNTIME CONFIG SYMLINK =====================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_RUNTIME="${REPO_DIR}/config/runtime.json"
SYS_CONFIG="/opt/cpcu/config.json"

# Source the shared runtime.json emitter.
. "${SCRIPT_DIR}/_default_runtime_json.sh"

# v2.7: if runtime.json is missing (fresh checkout, accidentally
# deleted, etc.), write a known-good default so the kernel can boot.
# Operator can customize later via the TUI editor or by editing the
# file directly.
if [ ! -f "${REPO_RUNTIME}" ]; then
    echo "  ${REPO_RUNTIME} missing — writing default..."
    emit_default_runtime_json "${REPO_RUNTIME}"
    chown "${REAL_USER}:${REAL_USER}" "${REPO_RUNTIME}" 2>/dev/null || true
    chown "${REAL_USER}:${REAL_USER}" "${REPO_DIR}/config" 2>/dev/null || true
    echo "  Wrote default runtime.json (you can edit ${REPO_RUNTIME} or use the TUI editor later)"
fi

if [ -L "${SYS_CONFIG}" ] && [ "$(readlink "${SYS_CONFIG}")" = "${REPO_RUNTIME}" ]; then
    echo "  config.json symlink already correct"
else
    ln -sfn "${REPO_RUNTIME}" "${SYS_CONFIG}"
    echo "  Linked ${SYS_CONFIG} -> ${REPO_RUNTIME}"
fi

##============= KERNEL CONFIG CHECK ========================================================

echo "[4/6] Checking kernel configuration..."

NEEDS_REBOOT=0

CONFIG="/boot/firmware/config.txt"
if [ -f "${CONFIG}" ]; then
    if ! grep -q "dtparam=spi=on" "${CONFIG}"; then
        echo "  Adding SPI enable to ${CONFIG}"
        echo "dtparam=spi=on" >> "${CONFIG}"
        NEEDS_REBOOT=1
    fi
    if ! grep -q "i2c_arm_baudrate=400000" "${CONFIG}"; then
        echo "  Adding I2C 400kHz to ${CONFIG}"
        echo "dtparam=i2c_arm_baudrate=400000" >> "${CONFIG}"
        NEEDS_REBOOT=1
    fi
    if ! grep -q "arm_freq=2800" "${CONFIG}"; then
        echo "  NOTE: Overclock (arm_freq=2800) not set. Add manually if desired."
    fi
    if ! grep -q "dtoverlay=disable-bt" "${CONFIG}"; then
        echo "  Adding Bluetooth disable to ${CONFIG}"
        echo "dtoverlay=disable-bt" >> "${CONFIG}"
        NEEDS_REBOOT=1
    fi
else
    echo "  WARNING: ${CONFIG} not found"
fi

CMDLINE="/boot/firmware/cmdline.txt"
if [ -f "${CMDLINE}" ]; then
    if ! grep -q "isolcpus" "${CMDLINE}"; then
        echo "  Adding core isolation to ${CMDLINE}"
        sed -i 's/$/ isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3/' "${CMDLINE}"
        NEEDS_REBOOT=1
    else
        echo "  Core isolation already configured"
    fi
else
    echo "  WARNING: ${CMDLINE} not found"
fi

##============= PERMISSIONS ================================================================

echo "[5/6] Setting permissions..."

if ! getent group spi >/dev/null 2>&1; then
    groupadd spi
fi
if ! getent group i2c >/dev/null 2>&1; then
    groupadd i2c
fi

cat > /etc/udev/rules.d/90-cpcu.rules << 'EOF'
# CPCU: spi/i2c group access; gpiochip0 access for nrf24l01_linux's CE pin
SUBSYSTEM=="spidev",   GROUP="spi", MODE="0660"
SUBSYSTEM=="i2c-dev",  GROUP="i2c", MODE="0660"
SUBSYSTEM=="gpio",     GROUP="gpio", MODE="0660"
KERNEL=="gpiochip[0-9]*", GROUP="gpio", MODE="0660"
EOF

udevadm control --reload-rules 2>/dev/null || true

if [ "${REAL_USER}" != "root" ]; then
    usermod -aG spi  "${REAL_USER}" 2>/dev/null || true
    usermod -aG i2c  "${REAL_USER}" 2>/dev/null || true
    usermod -aG gpio "${REAL_USER}" 2>/dev/null || true
    echo "  Added ${REAL_USER} to spi, i2c, gpio groups"
fi

##============= VERIFY =====================================================================

echo "[6/6] Verification..."

echo "  Python3: $(python3 --version 2>&1)"
echo "  NumPy:   $(python3 -c 'import numpy; print(numpy.__version__)' 2>&1)"
echo "  SciPy:   $(python3 -c 'import scipy; print(scipy.__version__)' 2>&1)"
echo "  sklearn: $(python3 -c 'import sklearn; print(sklearn.__version__)' 2>&1)"
echo "  CMake:   $(cmake --version 2>&1 | head -1)"
echo "  GCC:     $(gcc --version 2>&1 | head -1)"
echo "  tmux:    $(tmux -V 2>&1 || echo 'NOT INSTALLED')"

[ -e /dev/spidev0.0 ] && echo "  SPI0:    OK (/dev/spidev0.0 exists)" \
                       || echo "  SPI0:    NOT READY (reboot needed)"
[ -e /dev/i2c-1 ]     && echo "  I2C-1:   OK (/dev/i2c-1 exists)" \
                       || echo "  I2C-1:   NOT READY (reboot needed)"

ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "none")
echo "  Isolated cores: ${ISOLATED}"

echo ""
echo "=== Setup helper finished ==="

if [ "${NEEDS_REBOOT:-0}" -eq 1 ]; then
    exit 10
fi
exit 0
