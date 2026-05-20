#!/bin/bash
## grip_tune.sh — interactive gripper calibration.
## Invoked by: ./launch.sh grip-tune
##
## Reads gestures.json to find which PCA channel the gripper is wired
## to (no hardcoding!). At startup parks the whole arm at neutral so
## the calibration starts from a known pose. Steps the gripper down
## µs by µs, asks the operator to mark touch / firm / stall points,
## writes the recommended grip_firm_us into runtime.json + the active
## gesture_groups schema.
set -euo pipefail

REPO="${REPO_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
RT_JSON="${REPO}/config/runtime.json"
GS_JSON="${REPO}/config/gestures.json"

STEP=20
START=1500     # midway pulse — close enough to "open" for any gripper

G='\033[32m'; Y='\033[33m'; R='\033[31m'; C='\033[36m'; N='\033[0m'
ok()   { echo -e "  ${G}✓${N} $*"; }
info() { echo -e "  ${C}▶${N} $*"; }
warn() { echo -e "  ${Y}⚠${N} $*"; }
err()  { echo -e "  ${R}✗${N} $*"; }

ask_yn() { local r; read -rp "  $1 (y/n): " r; [[ "$r" =~ ^[yY] ]]; }

## ─────────────────────────────────────────────────────────────────────
## Pull config from gestures.json: which PCA channel is the gripper on,
## what's its safe pulse range, and what's the configured neutral.
## Falls back to broad SG90 defaults if any field is missing.
## ─────────────────────────────────────────────────────────────────────
GRIP_CH=$(python3 - "${GS_JSON}" << 'PYEOF'
import json, sys
try:
    gs = json.load(open(sys.argv[1]))
    print(int(gs["servo_channels"]["Gripper"]["pca_ch"]))
except Exception:
    print(15)
PYEOF
)
GRIP_MIN=$(python3 - "${GS_JSON}" << 'PYEOF'
import json, sys
try:
    gs = json.load(open(sys.argv[1]))
    print(int(gs["servo_channels"]["Gripper"]["min_us"]))
except Exception:
    print(976)
PYEOF
)
GRIP_MAX=$(python3 - "${GS_JSON}" << 'PYEOF'
import json, sys
try:
    gs = json.load(open(sys.argv[1]))
    print(int(gs["servo_channels"]["Gripper"]["max_us"]))
except Exception:
    print(1733)
PYEOF
)
GRIP_NEUTRAL=$(python3 - "${GS_JSON}" << 'PYEOF'
import json, sys
try:
    gs = json.load(open(sys.argv[1]))
    print(int(gs["servo_channels"]["Gripper"]["neutral_us"]))
except Exception:
    print(1500)
PYEOF
)

## ─────────────────────────────────────────────────────────────────────
## Single PCA9685 initialiser + servo writer. Used by both the "park
## the whole arm" step and the per-step gripper jog. Keeping the bus
## open across writes would be faster but the calibration is human-
## paced (one Y/N per step) so the overhead is invisible.
## ─────────────────────────────────────────────────────────────────────
set_servo_on_channel() {
    local ch="$1" us="$2"
    python3 - "${ch}" "${us}" << 'PYEOF'
import smbus2, time, sys
ch, us = int(sys.argv[1]), int(sys.argv[2])
bus = smbus2.SMBus(1)
PCA = 0x40
bus.write_byte_data(PCA, 0x00, 0x10)      # sleep
time.sleep(0.005)
bus.write_byte_data(PCA, 0xFE, 121)        # 50 Hz prescale (~20 ms tick)
bus.write_byte_data(PCA, 0x00, 0x20)       # wake + auto-inc
time.sleep(0.005)
count = int(us * 4096 / 20000)
reg = 0x06 + 4 * ch
bus.write_byte_data(PCA, reg,   0)
bus.write_byte_data(PCA, reg+1, 0)
bus.write_byte_data(PCA, reg+2, count & 0xFF)
bus.write_byte_data(PCA, reg+3, (count >> 8) & 0x0F)
bus.close()
PYEOF
}

## Park the entire arm at each servo's configured neutral_us. Reads
## the servo list (name → pca_ch + neutral_us) from gestures.json so
## this works regardless of how the operator has things wired.
park_arm_at_neutral() {
    info "Parking arm at neutral..."
    python3 - "${GS_JSON}" << 'PYEOF'
import json, sys, time, smbus2
gs = json.load(open(sys.argv[1]))
bus = smbus2.SMBus(1)
PCA = 0x40
bus.write_byte_data(PCA, 0x00, 0x10); time.sleep(0.005)
bus.write_byte_data(PCA, 0xFE, 121)
bus.write_byte_data(PCA, 0x00, 0x20); time.sleep(0.005)
for name, sd in gs.get("servo_channels", {}).items():
    ch = int(sd.get("pca_ch", -1))
    us = int(sd.get("neutral_us", 1500))
    if 0 <= ch <= 15:
        count = int(us * 4096 / 20000)
        reg = 0x06 + 4 * ch
        bus.write_byte_data(PCA, reg,   0)
        bus.write_byte_data(PCA, reg+1, 0)
        bus.write_byte_data(PCA, reg+2, count & 0xFF)
        bus.write_byte_data(PCA, reg+3, (count >> 8) & 0x0F)
        print(f"  {name} (PCA{ch:>2}) -> {us} us")
bus.close()
PYEOF
    sleep 0.5
}

set_grip() { set_servo_on_channel "${GRIP_CH}" "$1"; }

## ─────────────────────────────────────────────────────────────────────
## Write grip_firm_us into both runtime.json (the live cpcu_io
## handle) and the per-arm gesture defs (gesture_groups schema).
## Falls back to no-op if the schema is unrecognised.
## ─────────────────────────────────────────────────────────────────────
write_firm() {
    python3 - "${RT_JSON}" "${GS_JSON}" "$1" << 'PYEOF'
import json, sys
rt_path, gs_path, val = sys.argv[1], sys.argv[2], int(sys.argv[3])

# runtime.json: top-level grip_firm_us field
try:
    with open(rt_path) as f: rt = json.load(f)
    rt["grip_firm_us"] = val
    with open(rt_path, "w") as f: json.dump(rt, f, indent=4)
except Exception as e:
    print(f"  (skipped runtime.json: {e})")

# gestures.json: every group's "hand" gesture, per-group override
try:
    with open(gs_path) as f: gs = json.load(f)
    groups = gs.get("gesture_groups", {})
    if groups:
        for gname, gdef in groups.items():
            hand = gdef.get("gestures", {}).get("hand")
            if hand is not None:
                hand["grip_firm_us"] = val
    with open(gs_path, "w") as f: json.dump(gs, f, indent=4)
except Exception as e:
    print(f"  (skipped gestures.json: {e})")
PYEOF
}

## Set the gripper close rate + snap on each "hand" gesture across
## ALL gesture_groups. Works on the current schema only; the old flat
## gestures dict isn't touched.
write_rate() {
    python3 - "${GS_JSON}" "$1" "$2" << 'PYEOF'
import json, sys
gs_path, rate, snap = sys.argv[1], int(sys.argv[2]), sys.argv[3].lower() == "true"
with open(gs_path) as f: gs = json.load(f)
groups = gs.get("gesture_groups")
if not groups:
    print(f"  (no gesture_groups in {gs_path} — rate not written)")
    sys.exit(0)
for gname, gdef in groups.items():
    hand = gdef.get("gestures", {}).get("hand")
    if hand is None: continue
    chs = hand.setdefault("channels", {})
    gr = chs.setdefault("Gripper", {})
    gr["rate_us_s"] = rate
    gr["snap"]      = snap
with open(gs_path, "w") as f: json.dump(gs, f, indent=4)
PYEOF
}

## ─────────────────────────────────────────────────────────────────────
## PREFLIGHT
## ─────────────────────────────────────────────────────────────────────
if [ ! -e /dev/i2c-1 ]; then err "/dev/i2c-1 missing"; exit 1; fi
if ! python3 -c "import smbus2" 2>/dev/null; then
    err "smbus2 not installed: pip3 install smbus2"; exit 1
fi
if pgrep -f cpcu_io >/dev/null 2>&1 || pgrep -f cpcu_kernel >/dev/null 2>&1; then
    warn "cpcu_kernel/io is running — it will fight our I²C writes."
    warn "Run './launch.sh stop' first, then re-run grip-tune."
    exit 1
fi

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "${C}  InfiniTech Grip Tuning Wizard${N}"
echo -e "${C}═════════════════════════════════════════════${N}"
echo
info "Gripper resolved from gestures.json:"
info "  PCA channel ${GRIP_CH}   range ${GRIP_MIN} ↔ ${GRIP_MAX} µs"

park_arm_at_neutral

echo
info "Closing gripper to ${START} µs (start position)..."
set_grip ${START}
sleep 0.6

echo
info "Place an object in the gripper jaws."
read -rp "  Press Enter when ready... "

## ─────────────────────────────────────────────────────────────────────
## CALIBRATION SWEEP — three reference points
##   touch_us : jaws first contact the object (no force yet)
##   firm_us  : noticeable hold force, no servo strain
##   stall_us : audible whine / current spike — DON'T leave here
## ─────────────────────────────────────────────────────────────────────
touch_us=0; us=${START}
while [ ${us} -ge ${GRIP_MIN} ]; do
    set_grip ${us}; sleep 0.3
    if ask_yn "[${us} µs] Jaws touching?"; then touch_us=${us}; break; fi
    us=$((us - STEP))
done
if [ ${touch_us} -eq 0 ]; then
    err "No touch detected across the full range."
    set_grip ${GRIP_NEUTRAL}
    exit 1
fi
ok "Touch: ${touch_us} µs"

firm_us=0; us=$((touch_us - STEP))
while [ ${us} -ge ${GRIP_MIN} ]; do
    set_grip ${us}; sleep 0.3
    if ask_yn "[${us} µs] Grip firm?"; then firm_us=${us}; break; fi
    us=$((us - STEP))
done
[ ${firm_us} -eq 0 ] && firm_us=${GRIP_MIN}
ok "Firm: ${firm_us} µs"

stall_us=0; us=$((firm_us - STEP))
while [ ${us} -ge ${GRIP_MIN} ]; do
    set_grip ${us}; sleep 0.3
    if ask_yn "[${us} µs] Servo whining / straining?"; then stall_us=${us}; break; fi
    us=$((us - STEP))
done
[ ${stall_us} -eq 0 ] && stall_us=${GRIP_MIN}
ok "Stall: ${stall_us} µs"

# Park gripper back at its neutral so we don't leave the servo loaded
set_grip ${GRIP_NEUTRAL}

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
echo "  Gripper close speed:"
echo "    [1] Slow   (200 µs/s, ~3 s)"
echo "    [2] Medium (400 µs/s, ~1.5 s)"
echo "    [3] Fast   (600 µs/s, ~1 s)"
echo "    [4] Snap   (bypass smoother — instant)"
echo "    [5] Skip"
read -rp "  Choice [1-5]: " c
case "$c" in
    1) write_rate -200 false; ok "Slow" ;;
    2) write_rate -400 false; ok "Medium" ;;
    3) write_rate -600 false; ok "Fast" ;;
    4) write_rate -600 true;  ok "Snap" ;;
    5|*) warn "Kept current speed." ;;
esac

echo
info "Restart to apply: ./launch.sh stop && ./launch.sh tui"
