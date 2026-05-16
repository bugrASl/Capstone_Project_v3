#!/bin/bash
## setup_audio.sh — configure I2S audio: PCM5102A DAC + PAM8403 amp.
## Invoked by: ./launch.sh setup-audio
##
## Hardware:
##   PCM5102A  — I2S stereo DAC, line-level 3.5mm output
##   PAM8403   — 2×3W Class-D amplifier (drives speaker from line level)
##   8Ω speaker — any small 2W speaker
##
## Wiring (no resistors, no capacitors):
##
##   Pi 5 → PCM5102A:
##     GPIO18 (pin 12)  →  BCK
##     GPIO19 (pin 35)  →  LCK  (also labeled LRCK/WS)
##     GPIO21 (pin 40)  →  DIN  (also labeled DATA)
##     3.3V   (pin  1)  →  VIN
##     GND    (pin  6)  →  GND
##     (nothing)        →  SCK  →  tie SCK to GND (on the PCM5102A board)
##
##   PCM5102A → PAM8403:
##     3.5mm L out  →  PAM8403 L-IN
##     3.5mm GND    →  PAM8403 GND-IN
##       (or solder directly from PCM5102A L/R pads)
##
##   PAM8403 → Speaker:
##     5V  (from Pi pin 4)  →  PAM8403 VCC
##     GND (from Pi pin 6)  →  PAM8403 GND
##     L+/L-               →  Speaker +/−
##
##   IMPORTANT: Tie PCM5102A SCK pin to GND. This enables the
##   internal oscillator so no external master clock is needed.
set -euo pipefail

G='\033[32m'; Y='\033[33m'; C='\033[36m'; N='\033[0m'
ok()   { echo -e "  ${G}✓${N} $*"; }
info() { echo -e "  ${C}▶${N} $*"; }
warn() { echo -e "  ${Y}⚠${N} $*"; }

BOOT_CFG="/boot/firmware/config.txt"
NEED_REBOOT=0

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "${C}  PCM5102A + PAM8403 Audio Setup${N}"
echo -e "${C}═════════════════════════════════════════════${N}"
echo

# enable I2S overlay
if grep -q "hifiberry-dac" "${BOOT_CFG}" 2>/dev/null; then
    ok "I2S overlay already enabled."
else
    info "Adding I2S overlay to ${BOOT_CFG}..."
    sudo cp "${BOOT_CFG}" "${BOOT_CFG}.bak"
    {
        echo ""
        echo "# PCM5102A I2S DAC (gesture audio feedback)"
        echo "dtoverlay=hifiberry-dac"
    } | sudo tee -a "${BOOT_CFG}" >/dev/null
    ok "I2S overlay added."
    NEED_REBOOT=1
fi

# install espeak-ng
if ! command -v espeak-ng >/dev/null 2>&1; then
    info "Installing espeak-ng..."
    sudo apt-get install -y -qq espeak-ng
    ok "espeak-ng installed."
else
    ok "espeak-ng present."
fi

# install alsa-utils
if ! command -v aplay >/dev/null 2>&1; then
    info "Installing alsa-utils..."
    sudo apt-get install -y -qq alsa-utils
    ok "alsa-utils installed."
else
    ok "alsa-utils present."
fi

# set default volume
if command -v amixer >/dev/null 2>&1; then
    amixer set 'PCM' 80% 2>/dev/null && ok "Volume: 80%." || true
fi

echo
if [ ${NEED_REBOOT} -eq 1 ]; then
    warn "Reboot required for I2S to activate."
    echo
    read -rp "  Reboot now? (y/n): " r
    [[ "$r" =~ ^[yY] ]] && sudo reboot
else
    info "Verify with: aplay -l"
    info "Test with:   aplay config/audio_cues/voice_flex.wav"
fi

echo
echo -e "  ${C}Wiring reference:${N}"
echo
echo "    Pi 5               PCM5102A           PAM8403        Speaker"
echo "    ────               ────────           ───────        ───────"
echo "    GPIO18 (pin 12) →  BCK"
echo "    GPIO19 (pin 35) →  LCK"
echo "    GPIO21 (pin 40) →  DIN"
echo "    3.3V   (pin  1) →  VIN"
echo "    GND    (pin  6) →  GND"
echo "                       SCK → GND (tie!)"
echo "                       3.5mm out    →    L-IN / GND"
echo "    5V     (pin  4)                 →    VCC"
echo "    GND    (pin  6)                 →    GND"
echo "                                         L+ / L-    →   + / −"
echo
echo "    No resistors. No capacitors. SCK must be tied to GND."
echo
