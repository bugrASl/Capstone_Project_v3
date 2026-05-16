#!/bin/bash
## setup_uart.sh — enable UART0 on Pi 5 GPIO14/GPIO15 for debug output.
## Invoked by: ./launch.sh setup-uart
##
## Wiring (3 wires, use your existing USB-to-UART adapter):
##
##   Pi 5 GPIO14 TX (pin  8)  →  Adapter RX
##   Pi 5 GPIO15 RX (pin 10)  →  Adapter TX  (optional, for bidirectional)
##   Pi 5 GND       (pin 14)  →  Adapter GND
##
##   Do NOT connect adapter VCC to Pi (Pi has its own power).
##   Baud: 115200 8N1.
##
##   On host PC:
##     Linux:   screen /dev/ttyUSB0 115200
##     macOS:   screen /dev/tty.usbserial* 115200
##     Windows: PuTTY → Serial → COM port → 115200
set -euo pipefail

G='\033[32m'; Y='\033[33m'; C='\033[36m'; N='\033[0m'
ok()   { echo -e "  ${G}✓${N} $*"; }
info() { echo -e "  ${C}▶${N} $*"; }
warn() { echo -e "  ${Y}⚠${N} $*"; }

BOOT_CFG="/boot/firmware/config.txt"
CMDLINE="/boot/firmware/cmdline.txt"
NEED_REBOOT=0

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "${C}  UART Debug Setup (Pi 5)${N}"
echo -e "${C}═════════════════════════════════════════════${N}"
echo

# enable UART in config.txt
if grep -q "^enable_uart=1" "${BOOT_CFG}" 2>/dev/null; then
    ok "UART already enabled in config.txt."
else
    info "Enabling UART..."
    sudo cp "${BOOT_CFG}" "${BOOT_CFG}.bak"
    {
        echo ""
        echo "# UART debug output on GPIO14/GPIO15"
        echo "enable_uart=1"
    } | sudo tee -a "${BOOT_CFG}" >/dev/null
    ok "Added enable_uart=1."
    NEED_REBOOT=1
fi

# remove serial console from cmdline.txt (frees UART for our use)
if grep -q "console=serial0" "${CMDLINE}" 2>/dev/null; then
    info "Removing serial console from cmdline.txt..."
    sudo cp "${CMDLINE}" "${CMDLINE}.bak"
    sudo sed -i 's/ console=serial0,[0-9]*//g' "${CMDLINE}"
    ok "Serial console removed (UART now free for debug)."
    NEED_REBOOT=1
elif grep -q "console=ttyAMA0" "${CMDLINE}" 2>/dev/null; then
    sudo cp "${CMDLINE}" "${CMDLINE}.bak"
    sudo sed -i 's/ console=ttyAMA0,[0-9]*//g' "${CMDLINE}"
    ok "Serial console removed."
    NEED_REBOOT=1
else
    ok "Serial console already disabled."
fi

# install pyserial if missing
python3 -c "import serial" 2>/dev/null || {
    info "Installing pyserial..."
    pip3 install pyserial --break-system-packages -q
    ok "pyserial installed."
}

# check device exists
if [ -e /dev/ttyAMA0 ]; then
    ok "UART device: /dev/ttyAMA0"
    # set permissions
    sudo usermod -aG dialout "$(whoami)" 2>/dev/null || true
elif [ -e /dev/serial0 ]; then
    ok "UART device: /dev/serial0"
else
    warn "UART device not found (reboot may be needed)."
fi

echo
if [ ${NEED_REBOOT} -eq 1 ]; then
    warn "Reboot required to activate UART."
    read -rp "  Reboot now? (y/n): " r
    [[ "$r" =~ ^[yY] ]] && sudo reboot
else
    ok "UART ready."
fi

echo
echo -e "  ${C}Usage:${N}"
echo "    ./launch.sh tui --uart              Enable UART debug output"
echo "    ./launch.sh tui --uart --audio      + audio feedback"
echo
echo -e "  ${C}Wiring:${N}"
echo "    Pi GPIO14 TX (pin  8)  →  USB-UART adapter RX"
echo "    Pi GPIO15 RX (pin 10)  →  USB-UART adapter TX (optional)"
echo "    Pi GND       (pin 14)  →  USB-UART adapter GND"
echo "    Baud: 115200 8N1"
echo
echo -e "  ${C}Host PC:${N}"
echo "    Linux:   screen /dev/ttyUSB0 115200"
echo "    Windows: PuTTY → Serial → COM port → 115200"
echo "    Or:      python3 scripts/uart_monitor.py --port /dev/ttyUSB0"
echo
