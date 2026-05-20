#!/bin/bash
# calibrate.sh — operator calibration: rest-noise floor + velocity profile.
#
# ROLE
#   Thin wrapper around the Python pieces that produce per-operator
#   calibration files. Two phases, gated by --rest-only / --vel-only:
#     1. Rest-noise floor : invokes cpcu_dsp.py --calibrate to record
#        N seconds of muscle rest, computes a per-channel envelope
#        threshold (mean+3*std), and writes
#        models/dynamic_noise_thresholds_<operator>.json.
#     2. Velocity preference : invokes the Python velocity wizard to
#        record the operator's preferred per-gesture speed on a
#        0-10 scale, written to models/velocity_map_<operator>.json.
#
# DEPENDENCIES
#   config/gestures.json     : source of channel + gesture metadata.
#   python/cpcu_dsp.py       : performs the actual rest recording.
#   python/cpcu_calibrate.py : performs the velocity preference flow.
#   models/                  : output directory; never overwritten
#                              files for "default" — those are
#                              Aleyna's canonical set.
#
# DOWNSTREAM
#   cpcu_dsp.py at runtime   : Loads the per-operator JSON files
#                              when invoked with --operator NAME
#                              ($CPCU_OPERATOR carries this through
#                              launch.sh).
#   cpcu_tui CONFIG page     : Surfaces which operator profile is
#                              currently loaded (via the digest files
#                              cpcu_dsp.py drops in /tmp).
#
# CROSS-MODULE EFFECTS
#   - Changing the thresholds JSON schema requires matching parser
#     updates in cpcu_dsp.py::_load_dynamic_thresholds.
#   - The "default" operator name is a sentinel — calibrating it
#     would overwrite the canonical files. The Python side refuses
#     to write to those paths (re-tagged operator names are forced
#     into _<name>.json suffixed paths).
#
# USAGE
#   ./launch.sh calibrate [--operator NAME] [--rest-only | --vel-only]
#                         [--duration SEC]

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
    info "Phase 1: Rest calibration (${REST_SEC}s) — operator '${OPERATOR}'"
    info "Relax your arm. Keep electrodes on."
    info "  Writes models/dynamic_noise_thresholds_${OPERATOR}.json"
    info "  (Aleyna's canonical dynamic_noise_thresholds.json is untouched.)"
    read -rp "  Press Enter to start... "
    # Pass --operator so cpcu_dsp.py writes the operator-specific
    # noise file. Without this it would only see CPCU_OPERATOR via
    # env, which works, but explicit is clearer for a per-task
    # script.
    python3 "${PY_DIR}/cpcu_dsp.py" --calibrate "${REST_SEC}" \
                                    --operator "${OPERATOR}" 2>&1 | sed 's/^/  /'
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
