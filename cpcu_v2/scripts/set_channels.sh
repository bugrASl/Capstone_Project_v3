#!/bin/bash
## set_channels.sh — update active EMG channels in gestures.json.
## Validates against current model and warns on mismatch.
## Invoked by: ./launch.sh set-channels <ch0> <ch1> ... <chN>
##
## Examples:
##   ./launch.sh set-channels 0 1 2           # 3-channel (current)
##   ./launch.sh set-channels 0 1 2 3 4       # 5-channel
##   ./launch.sh set-channels 0 1 2 3 4 5 6 7 # 8-channel (full)
set -euo pipefail

REPO="${REPO_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
GS="${REPO}/config/gestures.json"

G='\033[32m'; Y='\033[33m'; R='\033[31m'; C='\033[36m'; N='\033[0m'
ok()   { echo -e "  ${G}✓${N} $*"; }
info() { echo -e "  ${C}▶${N} $*"; }
warn() { echo -e "  ${Y}⚠${N} $*"; }
err()  { echo -e "  ${R}✗${N} $*"; }

if [ $# -lt 1 ]; then
    echo "Usage: ./launch.sh set-channels <ch0> <ch1> ... <chN>"
    echo
    echo "  Channel indices match BSAU ADC channels (0-7)."
    echo "  Order matters: ch0 is the first feature group in the model."
    echo
    echo "  Examples:"
    echo "    ./launch.sh set-channels 0 1 2             # 3ch"
    echo "    ./launch.sh set-channels 0 1 2 3 4         # 5ch"
    echo "    ./launch.sh set-channels 0 1 2 3 4 5 6 7   # 8ch (full)"
    exit 1
fi

# build channel list
CHANNELS="["
FIRST=1
for ch in "$@"; do
    # validate: must be 0-7
    if ! [[ "$ch" =~ ^[0-7]$ ]]; then
        err "Invalid channel: $ch (must be 0-7)"
        exit 1
    fi
    [ $FIRST -eq 0 ] && CHANNELS="${CHANNELS}, "
    CHANNELS="${CHANNELS}${ch}"
    FIRST=0
done
CHANNELS="${CHANNELS}]"
NUM_CH=$#

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "${C}  EMG Channel Configuration${N}"
echo -e "${C}═════════════════════════════════════════════${N}"
echo

# show current
python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
old = g.get('emg_channels', {})
print(f'  Current: ch{old.get(\"active\", [])}')
print(f'  Names:   {old.get(\"names\", [])}')
"
echo "  New:     ch${CHANNELS}"
echo

# ask for muscle names
NAMES="["
FIRST=1
for ch in "$@"; do
    read -rp "  Muscle name for channel ${ch}: " mname
    mname="${mname:-ch${ch}}"
    [ $FIRST -eq 0 ] && NAMES="${NAMES}, "
    NAMES="${NAMES}\"${mname}\""
    FIRST=0
done
NAMES="${NAMES}]"

echo

# update gestures.json
python3 << PYEOF
import json, os, sys

with open("${GS}") as f:
    gs = json.load(f)

gs["emg_channels"]["active"] = ${CHANNELS}
gs["emg_channels"]["names"]  = ${NAMES}

with open("${GS}", "w") as f:
    json.dump(gs, f, indent=4)

print("  \033[32m✓\033[0m gestures.json updated.")

# validate model compatibility
model_path = gs.get("model_path", "")
if not os.path.isabs(model_path):
    model_path = os.path.join("${REPO}", model_path)

features_per_ch = 4
expected_features = ${NUM_CH} * features_per_ch

try:
    import joblib
    cp = joblib.load(model_path)
    if isinstance(cp, dict):
        model = cp.get("model")
        scaler = cp.get("scaler")
    else:
        model = cp
        scaler = None

    if scaler and hasattr(scaler, "n_features_in_"):
        model_features = scaler.n_features_in_
    elif hasattr(model, "n_features_in_"):
        model_features = model.n_features_in_
    else:
        model_features = None

    if model_features is not None:
        if model_features == expected_features:
            print(f"  \033[32m✓\033[0m Model expects {model_features} features "
                  f"({model_features // features_per_ch} channels) — matches!")
        else:
            model_ch = model_features // features_per_ch
            print(f"  \033[33m⚠\033[0m Model expects {model_features} features "
                  f"({model_ch} channels)")
            print(f"    New config produces {expected_features} features "
                  f"({NUM_CH} channels)")
            print(f"    \033[31mModel will NOT work — retrain required.\033[0m")
            print()
            print("  Next steps:")
            print("    1. ./launch.sh collect             (record data)")
            print("    2. Retrain model with ${NUM_CH} channels")
            print("    3. Place .pkl in models/")
            print(f"    4. Update gestures.json model_path")
            print("    5. ./launch.sh reload")
    else:
        print("  \033[33m⚠\033[0m Could not determine model feature count.")

    if hasattr(model, "classes_"):
        print(f"  Model classes: {list(model.classes_)}")

except FileNotFoundError:
    print(f"  \033[33m⚠\033[0m Model not found: {model_path}")
except ImportError:
    print("  \033[33m⚠\033[0m joblib not installed — can't validate model.")
except Exception as e:
    print(f"  \033[33m⚠\033[0m Model check failed: {e}")
PYEOF

echo
echo -e "${C}═════════════════════════════════════════════${N}"

# suggest model naming
echo
info "Model naming convention:"
echo "    models/model_${NUM_CH}ch.pkl     (${NUM_CH}-channel model)"
echo "    models/model_8ch.pkl     (full 8-channel model)"
echo
info "After retraining, update gestures.json:"
echo "    \"model_path\": \"models/model_${NUM_CH}ch.pkl\""
echo
