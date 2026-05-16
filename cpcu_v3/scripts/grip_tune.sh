#!/bin/bash
## grip_tune.sh — interactive gripper calibration.
## Invoked by: ./launch.sh grip-tune
## Steps the gripper servo, asks touch/firm/stall, writes result.
set -euo pipefail

REPO="${REPO_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
RT_JSON="${REPO}/config/runtime.json"
GS_JSON="${REPO}/config/gestures.json"

GRIP_CH=5
GRIP_MIN=976
GRIP_MAX=1733
STEP=20
START=1200

G='\033[32m'; Y='\033[33m'; R='\033[31m'; C='\033[36m'; N='\033[0m'
ok()   { echo -e "  ${G}✓${N} $*"; }
info() { echo -e "  ${C}▶${N} $*"; }
warn() { echo -e "  ${Y}⚠${N} $*"; }
err()  { echo -e "  ${R}✗${N} $*"; }

ask_yn() { local r; read -rp "  $1 (y/n): " r; [[ "$r" =~ ^[yY] ]]; }

## write one servo pulse via smbus2 (no kernel needed)
set_servo() {
    python3 << PYEOF
import smbus2, time
bus = smbus2.SMBus(1)
PCA = 0x40
bus.write_byte_data(PCA, 0x00, 0x10)      # sleep
time.sleep(0.005)
bus.write_byte_data(PCA, 0xFE, 121)        # 50 Hz prescale
bus.write_byte_data(PCA, 0x00, 0x20)       # wake + auto-inc
time.sleep(0.005)
count = int($1 * 4096 / 20000)
reg = 0x06 + 4 * ${GRIP_CH}
bus.write_byte_data(PCA, reg,   0)
bus.write_byte_data(PCA, reg+1, 0)
bus.write_byte_data(PCA, reg+2, count & 0xFF)
bus.write_byte_data(PCA, reg+3, (count >> 8) & 0x0F)
bus.close()
PYEOF
}

## write grip_firm_us to both config files
write_firm() {
    python3 << PYEOF
import json
for path in ["${RT_JSON}", "${GS_JSON}"]:
    try:
        with open(path) as f: d = json.load(f)
        if "grip_firm_us" in d:
            d["grip_firm_us"] = $1
        if "gestures" in d and "hand" in d["gestures"]:
            d["gestures"]["hand"]["grip_firm_us"] = $1
        with open(path, "w") as f: json.dump(d, f, indent=4)
    except Exception: pass
PYEOF
}

## write gripper rate + snap to gestures.json
write_rate() {
    python3 << PYEOF
import json
with open("${GS_JSON}") as f: d = json.load(f)
g = d.setdefault("gestures", {}).setdefault("hand", {})
ch = g.setdefault("channels", {}).setdefault("S5_Gripper", {})
ch["rate_us_s"] = $1
ch["snap"] = $2
with open("${GS_JSON}", "w") as f: json.dump(d, f, indent=4)
PYEOF
}

## preflight
if [ ! -e /dev/i2c-1 ]; then err "/dev/i2c-1 missing"; exit 1; fi
if ! python3 -c "import smbus2" 2>/dev/null; then
    err "smbus2 not installed: pip3 install smbus2"; exit 1
fi

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "${C}  InfiniTech Grip Tuning Wizard${N}"
echo -e "${C}═════════════════════════════════════════════${N}"
echo
info "Servo 5 range: ${GRIP_MIN} (closed) → ${GRIP_MAX} (open)"
info "Opening gripper..."
set_servo ${START}
sleep 0.5

echo
info "Place an object in the gripper jaws."
read -rp "  Press Enter when ready... "

## find touch point
touch_us=0; us=${START}
while [ ${us} -ge ${GRIP_MIN} ]; do
    set_servo ${us}; sleep 0.3
    if ask_yn "[${us} µs] Jaws touching?"; then touch_us=${us}; break; fi
    us=$((us - STEP))
done
[ ${touch_us} -eq 0 ] && { err "No touch detected."; set_servo 1500; exit 1; }
ok "Touch: ${touch_us} µs"

## find firm point
firm_us=0; us=$((touch_us - STEP))
while [ ${us} -ge ${GRIP_MIN} ]; do
    set_servo ${us}; sleep 0.3
    if ask_yn "[${us} µs] Grip firm?"; then firm_us=${us}; break; fi
    us=$((us - STEP))
done
[ ${firm_us} -eq 0 ] && firm_us=${GRIP_MIN}
ok "Firm: ${firm_us} µs"

## find stall point
stall_us=0; us=$((firm_us - STEP))
while [ ${us} -ge ${GRIP_MIN} ]; do
    set_servo ${us}; sleep 0.3
    if ask_yn "[${us} µs] Servo whining?"; then stall_us=${us}; break; fi
    us=$((us - STEP))
done
[ ${stall_us} -eq 0 ] && stall_us=${GRIP_MIN}
ok "Stall: ${stall_us} µs"

set_servo 1500  # neutral

rec=$(( (firm_us + stall_us) / 2 ))
margin=$(( firm_us - stall_us ))

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "  Touch: ${touch_us}  Firm: ${firm_us}  Stall: ${stall_us}"
echo -e "  Margin: ${margin} µs"
echo -e "  ${Y}Recommended grip_firm_us = ${rec}${N}"
echo -e "${C}═════════════════════════════════════════════${N}"

if ask_yn "Apply grip_firm_us = ${rec}?"; then
    write_firm ${rec}
    ok "Written to runtime.json + gestures.json"
fi

echo
echo -e "  Gripper speed:"
echo "    [1] Slow   (200 µs/s, ~3s)"
echo "    [2] Medium (400 µs/s, ~1.5s)"
echo "    [3] Fast   (600 µs/s, ~1s)"
echo "    [4] Snap   (bypass smoother)"
read -rp "  Choice [1-4]: " c
case "$c" in
    1) write_rate -200 false; ok "Slow" ;;
    2) write_rate -400 false; ok "Medium" ;;
    3) write_rate -600 false; ok "Fast" ;;
    4) write_rate -600 true;  ok "Snap" ;;
    *) warn "Kept current speed." ;;
esac

echo
info "Restart to apply: ./launch.sh stop && ./launch.sh tui"
