#!/bin/bash
## calibrate.sh — rest noise + velocity preference (0-10 scale).
## Invoked by: ./launch.sh calibrate [--operator <name>]
set -euo pipefail

REPO="${REPO_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
GS_JSON="${REPO}/config/gestures.json"
PY_DIR="${REPO}/python"
MODELS="${REPO}/models"

G='\033[32m'; Y='\033[33m'; C='\033[36m'; N='\033[0m'
ok()   { echo -e "  ${G}✓${N} $*"; }
info() { echo -e "  ${C}▶${N} $*"; }

OPERATOR="default"
REST_SEC=10
DO_REST=1
DO_VEL=1

while [ $# -gt 0 ]; do
    case "$1" in
        --operator)   OPERATOR="$2"; shift 2 ;;
        --rest-only)  DO_VEL=0; shift ;;
        --vel-only)   DO_REST=0; shift ;;
        --duration)   REST_SEC="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: ./launch.sh calibrate [--operator NAME] [--rest-only|--vel-only] [--duration SEC]"
            exit 0 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

VMAP="${MODELS}/velocity_map.json"
[ "${OPERATOR}" != "default" ] && VMAP="${MODELS}/velocity_map_${OPERATOR}.json"

if [ ! -f /dev/shm/cpcu_ipc ]; then
    echo "  System not running. Start with: ./launch.sh tui"
    exit 1
fi

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "${C}  InfiniTech Calibration — ${OPERATOR}${N}"
echo -e "${C}═════════════════════════════════════════════${N}"

if [ ${DO_REST} -eq 1 ]; then
    echo
    info "Phase 1: Rest calibration (${REST_SEC}s)"
    info "Relax your arm. Keep electrodes on."
    read -rp "  Press Enter to start... "
    python3 "${PY_DIR}/cpcu_dsp.py" --calibrate "${REST_SEC}" 2>&1 | sed 's/^/  /'
    ok "Rest thresholds saved."
fi

if [ ${DO_VEL} -eq 1 ]; then
    echo
    info "Phase 2: Velocity preference (0-10 scale)"
    info "  0=stop  5=comfortable  10=max speed"
    echo
    python3 "${PY_DIR}/cpcu_calibrate.py" \
        --gestures "${GS_JSON}" --output "${VMAP}" --operator "${OPERATOR}" 2>&1 | sed 's/^/  /'
    ok "Velocity map: ${VMAP}"
fi

echo
echo -e "${C}═════════════════════════════════════════════${N}"
echo -e "  ${G}Done.${N} Use:  ./launch.sh tui --operator ${OPERATOR}"
echo -e "${C}═════════════════════════════════════════════${N}"
