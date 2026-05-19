#!/bin/bash
## generate_voice_cues.sh — create voice .wav for gestures + system events.
## Reads gestures.json, generates via espeak-ng.
## Invoked by: ./launch.sh generate-cues
set -euo pipefail

REPO="${REPO_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
GS="${REPO}/config/gestures.json"
OUT="${REPO}/config/audio_cues"
mkdir -p "${OUT}"

G='\033[32m'; R='\033[31m'; C='\033[36m'; N='\033[0m'

if ! command -v espeak-ng >/dev/null 2>&1; then
    echo "  Installing espeak-ng..."
    sudo apt-get install -y -qq espeak-ng
fi

echo
echo -e "${C}  Generating voice cues from gestures.json...${N}"
echo

python3 << PYEOF
import json, subprocess, os

with open("${GS}") as f:
    gs = json.load(f)

out_dir = "${OUT}"
count = 0

def generate(voice_name, text):
    global count
    wav = os.path.join(out_dir, f"{voice_name}.wav")
    try:
        subprocess.run(
            ["espeak-ng", "-v", "en", "-s", "180", "-p", "40",
             "-a", "150", "-w", wav, text],
            check=True, capture_output=True)
        sz = os.path.getsize(wav)
        print(f'  \033[32m✓\033[0m {voice_name}.wav ({sz} B) "{text}"')
        count += 1
    except Exception as e:
        print(f'  \033[31m✗\033[0m {voice_name}: {e}')

# gesture cues
print("  Gestures:")
for gname, gdef in gs.get("gestures", {}).items():
    vname = gdef.get("audio", {}).get("voice")
    if vname:
        generate(vname, gname.replace("_", " "))

# system event cues
events = gs.get("audio_events", {})
if events:
    print("\n  System events:")
    event_texts = {
        "system_start":     "system ready",
        "system_stop":      "shutting down",
        "safe_state":       "safe state",
        "link_lost":        "link lost",
        "link_recovered":   "link recovered",
        "low_battery":      "low battery",
        "calibration_done": "calibration complete",
    }
    for ename, edef in events.items():
    if isinstance(edef, str) or ename.startswith("_"): continue
        vname = edef.get("voice")
        if vname:
            text = event_texts.get(ename, ename.replace("_", " "))
            generate(vname, text)

print(f"\n  {count} voice cues in {out_dir}/")
PYEOF

echo
echo "  Test: aplay ${OUT}/voice_flex.wav"
