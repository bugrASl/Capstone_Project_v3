#!/bin/bash
## add_gesture.sh — add a new gesture to gestures.json with audio cues.
## Invoked by: ./launch.sh add-gesture
set -euo pipefail

REPO="${REPO_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
GS="${REPO}/config/gestures.json"
AUDIO_DIR="${REPO}/config/audio_cues"

G='\033[32m'; Y='\033[33m'; C='\033[36m'; N='\033[0m'
ok()   { echo -e "  ${G}✓${N} $*"; }
info() { echo -e "  ${C}▶${N} $*"; }
warn() { echo -e "  ${Y}⚠${N} $*"; }

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "${C}  Add New Gesture${N}"
echo -e "${C}═════════════════════════════════════════════${N}"
echo

# ── step 1: name ──
read -rp "  Gesture name (lowercase, no spaces): " GNAME
GNAME=$(echo "$GNAME" | tr '[:upper:]' '[:lower:]' | tr ' ' '_')

if python3 -c "
import json, os
with open('${GS}') as f: g = json.load(f)
if '${GNAME}' in g.get('gestures', {}):
    print('EXISTS')
" 2>/dev/null | grep -q EXISTS; then
    warn "'${GNAME}' already exists in gestures.json"
    read -rp "  Overwrite? (y/n): " ow
    [[ "$ow" =~ ^[yY] ]] || exit 0
fi

# ── step 2: mode ──
echo
echo "  Mode:"
echo "    [1] velocity  (servo moves while gesture held)"
echo "    [2] freeze    (hold current position)"
read -rp "  Choice [1]: " mc
MODE="velocity"
[ "$mc" = "2" ] && MODE="freeze"

# ── step 3: servo channels ──
CHANNELS="{}"
if [ "$MODE" = "velocity" ]; then
    echo
    echo "  Available servos:"
    echo "    S0_Base  S1_Elbow  S2_Forearm  S3_Wrist1  S4_Wrist2  S5_Gripper"
    echo
    read -rp "  Which servos? (e.g. S1_Elbow S5_Gripper): " SERVOS

    CH_JSON="{"
    first=1
    for s in $SERVOS; do
        read -rp "  ${s} rate (µs/s, negative=reverse) [200]: " rate
        rate="${rate:-200}"
        snap="false"
        if [ "$s" = "S5_Gripper" ]; then
            read -rp "  ${s} snap mode? (y/n) [n]: " sn
            [[ "$sn" =~ ^[yY] ]] && snap="true"
        fi
        [ $first -eq 0 ] && CH_JSON="${CH_JSON},"
        CH_JSON="${CH_JSON} \"${s}\": {\"rate_us_s\": ${rate}, \"snap\": ${snap}}"
        first=0
    done
    CH_JSON="${CH_JSON} }"
    CHANNELS="$CH_JSON"
fi

# ── step 4: audio ──
echo
echo "  Audio feedback:"

# pick next unused frequency (step by 110 Hz from 330)
USED_FREQS=$(python3 -c "
import json, os
with open('${GS}') as f: g = json.load(f)
for gd in g.get('gestures',{}).values():
    a = gd.get('audio',{})
    if 'freq_hz' in a: print(a['freq_hz'])
" 2>/dev/null)
FREQ=330
while echo "$USED_FREQS" | grep -q "^${FREQ}$" 2>/dev/null; do
    FREQ=$((FREQ + 110))
done

read -rp "  Tone frequency Hz [${FREQ}]: " uf
FREQ="${uf:-$FREQ}"
read -rp "  Tone duration ms [80]: " ud
DUR="${ud:-80}"

# generate voice cue if espeak-ng available
VOICE_NAME="voice_${GNAME}"
if command -v espeak-ng >/dev/null 2>&1; then
    mkdir -p "${AUDIO_DIR}"
    espeak-ng -v en -s 180 -p 40 -a 150 -w "${AUDIO_DIR}/${VOICE_NAME}.wav" "${GNAME}" 2>/dev/null
    ok "Voice cue: ${VOICE_NAME}.wav"
else
    warn "espeak-ng not found — voice cue skipped (run ./launch.sh generate-cues later)"
fi

# ── step 5: write to gestures.json ──
python3 << PYEOF
import json, os

with open("${GS}") as f:
    gs = json.load(f)

gesture = {
    "mode": "${MODE}",
    "channels": ${CHANNELS},
    "audio": {
        "voice": "${VOICE_NAME}",
        "freq_hz": ${FREQ},
        "freq_ms": ${DUR}
    }
}

# find first group or create one
gg = gs.setdefault("gesture_groups", {})
if not gg:
    gg["gesture_0"] = {"gestures": {}, "emg_channels": {"active": [0,1,2]},
                        "model_path": "", "confidence": {"curve":"quadratic","floor_pct":40,"ceil_pct":85},
                        "hysteresis": {"rest_to_active":4,"active_to_rest":2,"active_to_active":6}}
# add to first group by default (use --group to specify)
group = os.environ.get("CPCU_GESTURE_GROUP", list(gg.keys())[0])
if group not in gg:
    print(f"Group '{group}' not found. Available: {list(gg.keys())}")
    exit(1)
gg[group].setdefault("gestures", {})["${GNAME}"] = gesture
print(f"  Added to group: {group}")

with open("${GS}", "w") as f:
    json.dump(gs, f, indent=4)
PYEOF

ok "Added '${GNAME}' to gestures.json"

# generate freq tone wav
python3 -c "
import wave, numpy as np, os
sr=22050; fhz=${FREQ}; ms=${DUR}; n=int(sr*ms/1000)
t=np.linspace(0,ms/1000,n,False)
fade=min(int(sr*0.005),n//4)
env=np.ones(n)
if fade>0: env[:fade]=np.linspace(0,1,fade); env[-fade:]=np.linspace(1,0,fade)
d=(np.sin(2*np.pi*fhz*t)*env*16000).astype(np.int16)
p=os.path.join('${AUDIO_DIR}','_gen_${GNAME}_${FREQ}hz.wav')
with wave.open(p,'w') as w:
    w.setnchannels(1);w.setsampwidth(2);w.setframerate(sr);w.writeframes(d.tobytes())
print(f'  Tone: ${FREQ}Hz/${DUR}ms')
" 2>/dev/null && ok "Freq tone generated" || true

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "  ${G}Gesture '${GNAME}' created.${N}"
echo
echo "  Next steps:"
echo "    1. Collect training data:  ./launch.sh collect"
echo "    2. Retrain the model with the new class"
echo "    3. Restart:  ./launch.sh reload"
echo -e "${C}═════════════════════════════════════════════${N}"
