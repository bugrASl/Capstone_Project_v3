#!/bin/bash
## test_launch_commands.sh — verify all v3.0 launch.sh commands work.
## Run from the cpcu_v2 directory.
##
## Usage:
##   ./test/test_launch_commands.sh           # all tests
##   ./test/test_launch_commands.sh --quick   # config-only (no hardware)
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
GS="${REPO}/config/gestures.json"
RT="${REPO}/config/runtime.json"
MODELS="${REPO}/models"
PASS=0; FAIL=0; SKIP=0
QUICK=0
[ "${1:-}" = "--quick" ] && QUICK=1

G='\033[32m'; R='\033[31m'; Y='\033[33m'; C='\033[36m'; N='\033[0m'

pass() { echo -e "  ${G}PASS${N}  $1"; PASS=$((PASS+1)); }
fail() { echo -e "  ${R}FAIL${N}  $1: $2"; FAIL=$((FAIL+1)); }
skip() { echo -e "  ${Y}SKIP${N}  $1"; SKIP=$((SKIP+1)); }

assert_file()    { [ -f "$1" ] && pass "$2" || fail "$2" "file missing: $1"; }
assert_json()    { python3 -c "import json; json.load(open('$1'))" 2>/dev/null && pass "$2" || fail "$2" "invalid JSON: $1"; }
assert_key()     { python3 -c "import json; d=json.load(open('$1')); assert '$2' in d" 2>/dev/null && pass "$3" || fail "$3" "key '$2' missing"; }
assert_cmd()     { eval "$1" >/dev/null 2>&1 && pass "$2" || fail "$2" "command failed"; }
assert_contains(){ grep -q "$2" "$1" 2>/dev/null && pass "$3" || fail "$3" "'$2' not found in $(basename $1)"; }

echo
echo -e "${C}═══════════════════════════════════════════════════${N}"
echo -e "${C}  InfiniTech v3.0 — Launch Command Tests${N}"
echo -e "${C}═══════════════════════════════════════════════════${N}"
echo

# ═══════════════════════════════════════════════════
# GROUP 1: File structure
# ═══════════════════════════════════════════════════
echo -e "${C}[1] File Structure${N}"
assert_file "${GS}" "gestures.json exists"
assert_file "${RT}" "runtime.json exists"
assert_file "${MODELS}/velocity_map.json" "velocity_map.json exists"
assert_file "${REPO}/python/cpcu_dsp.py" "cpcu_dsp.py exists"
assert_file "${REPO}/python/cpcu_audio_daemon.py" "cpcu_audio_daemon.py exists"
assert_file "${REPO}/python/cpcu_calibrate.py" "cpcu_calibrate.py exists"
assert_file "${REPO}/scripts/grip_tune.sh" "grip_tune.sh exists"
assert_file "${REPO}/scripts/calibrate.sh" "calibrate.sh exists"
assert_file "${REPO}/scripts/add_gesture.sh" "add_gesture.sh exists"
assert_file "${REPO}/scripts/set_channels.sh" "set_channels.sh exists"
assert_file "${REPO}/scripts/setup_audio.sh" "setup_audio.sh exists"
assert_file "${REPO}/scripts/setup_uart.sh" "setup_uart.sh exists"
assert_file "${REPO}/scripts/generate_voice_cues.sh" "generate_voice_cues.sh exists"
echo

# ═══════════════════════════════════════════════════
# GROUP 2: JSON validity
# ═══════════════════════════════════════════════════
echo -e "${C}[2] JSON Validity${N}"
assert_json "${GS}" "gestures.json valid JSON"
assert_json "${RT}" "runtime.json valid JSON"
assert_json "${MODELS}/velocity_map.json" "velocity_map.json valid JSON"
echo

# ═══════════════════════════════════════════════════
# GROUP 3: gestures.json schema
# ═══════════════════════════════════════════════════
echo -e "${C}[3] gestures.json Schema${N}"
assert_key "${GS}" "schema_version" "has schema_version"
assert_key "${GS}" "model_path" "has model_path"
assert_key "${GS}" "audio_mode" "has audio_mode"
assert_key "${GS}" "audio_volume_pct" "has audio_volume_pct"
assert_key "${GS}" "servo_channels" "has servo_channels"
assert_key "${GS}" "emg_channels" "has emg_channels"
assert_key "${GS}" "gestures" "has gestures"
assert_key "${GS}" "audio_events" "has audio_events"
assert_key "${GS}" "confidence" "has confidence"
assert_key "${GS}" "hysteresis" "has hysteresis"

# verify servo_channels use pca_ch (not idx)
python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
for name, sd in g['servo_channels'].items():
    if 'pca_ch' not in sd:
        print(f'FAIL: {name} missing pca_ch')
        sys.exit(1)
    if 'idx' in sd:
        print(f'FAIL: {name} has legacy idx field')
        sys.exit(1)
" 2>/dev/null && pass "servo_channels use pca_ch (not idx)" || fail "servo_channels field naming" "pca_ch/idx mismatch"

# verify rest gesture exists
python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
assert 'rest' in g['gestures'], 'no rest gesture'
assert g['gestures']['rest']['mode'] == 'freeze', 'rest not freeze'
" 2>/dev/null && pass "rest gesture exists (freeze mode)" || fail "rest gesture" "missing or wrong mode"

# verify gesture audio has both voice and freq_hz
python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
for gname, gdef in g['gestures'].items():
    audio = gdef.get('audio', {})
    if 'voice' not in audio:
        print(f'FAIL: {gname} missing audio.voice')
        sys.exit(1)
    if 'freq_hz' not in audio:
        print(f'FAIL: {gname} missing audio.freq_hz')
        sys.exit(1)
" 2>/dev/null && pass "all gestures have voice + freq audio" || fail "gesture audio" "missing voice or freq_hz"

# verify gesture channels reference valid servo names
python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
servos = set(g['servo_channels'].keys())
for gname, gdef in g['gestures'].items():
    for sname in gdef.get('channels', {}):
        if sname not in servos:
            print(f'FAIL: gesture {gname} references unknown servo {sname}')
            sys.exit(1)
" 2>/dev/null && pass "gesture servo references valid" || fail "gesture servo refs" "unknown servo name"
echo

# ═══════════════════════════════════════════════════
# GROUP 4: runtime.json schema
# ═══════════════════════════════════════════════════
echo -e "${C}[4] runtime.json Schema${N}"
python3 -c "
import json, sys
with open('${RT}') as f: g = json.load(f)
required = ['servo_min_us','servo_max_us','smooth_velocity_us_per_s',
            'smooth_accel_us_per_s2','grip_firm_us','hysteresis_votes']
for k in required:
    assert k in g, f'missing {k}'
# no legacy comment fields
for k in g:
    assert not k.startswith('//'), f'legacy comment field: {k}'
" 2>/dev/null && pass "runtime.json has required fields, no legacy comments" || fail "runtime.json schema" "check failed"
echo

# ═══════════════════════════════════════════════════
# GROUP 5: Python imports
# ═══════════════════════════════════════════════════
echo -e "${C}[5] Python Syntax${N}"
python3 -c "import ast; ast.parse(open('${REPO}/python/cpcu_dsp.py').read())" 2>/dev/null \
    && pass "cpcu_dsp.py syntax OK" || fail "cpcu_dsp.py" "syntax error"
python3 -c "import ast; ast.parse(open('${REPO}/python/cpcu_calibrate.py').read())" 2>/dev/null \
    && pass "cpcu_calibrate.py syntax OK" || fail "cpcu_calibrate.py" "syntax error"
python3 -c "import ast; ast.parse(open('${REPO}/python/cpcu_audio_daemon.py').read())" 2>/dev/null \
    && pass "cpcu_audio_daemon.py syntax OK" || fail "cpcu_audio_daemon.py" "syntax error"

# verify cpcu_dsp.py reads pca_ch not idx
assert_contains "${REPO}/python/cpcu_dsp.py" "pca_ch" "cpcu_dsp.py uses pca_ch"
python3 -c "
import sys
code = open('${REPO}/python/cpcu_dsp.py').read()
if '\"idx\"' in code:
    print('FAIL: still references idx')
    sys.exit(1)
" 2>/dev/null && pass "cpcu_dsp.py no legacy idx references" || fail "cpcu_dsp.py" "still uses idx"
echo

# ═══════════════════════════════════════════════════
# GROUP 6: Shell script syntax
# ═══════════════════════════════════════════════════
echo -e "${C}[6] Shell Script Syntax${N}"
for f in "${REPO}"/scripts/*.sh; do
    name=$(basename "$f")
    bash -n "$f" 2>/dev/null && pass "${name} syntax OK" || fail "${name}" "syntax error"
done
echo

# ═══════════════════════════════════════════════════
# GROUP 7: Audio cue files
# ═══════════════════════════════════════════════════
echo -e "${C}[7] Audio Files${N}"
python3 -c "
import json, os, sys
with open('${GS}') as f: g = json.load(f)
audio_dir = '${REPO}/config/audio_cues'
missing = []
for gname, gdef in g['gestures'].items():
    vname = gdef.get('audio', {}).get('voice')
    if vname:
        wav = os.path.join(audio_dir, f'{vname}.wav')
        if not os.path.exists(wav):
            missing.append(vname)
if missing:
    print(f'Missing voice cues: {missing}')
    print('Run: ./launch.sh generate-cues')
    sys.exit(1)
" 2>/dev/null && pass "all gesture voice cues present" || skip "voice cues (run generate-cues)"
echo

# ═══════════════════════════════════════════════════
# GROUP 8: Model validation
# ═══════════════════════════════════════════════════
echo -e "${C}[8] Model${N}"
python3 -c "
import json, os, sys
with open('${GS}') as f: g = json.load(f)
mp = g.get('model_path', '')
if not os.path.isabs(mp):
    mp = os.path.join('${REPO}', mp)
if not os.path.exists(mp):
    print(f'Model not found: {mp}')
    sys.exit(1)
try:
    import joblib
    cp = joblib.load(mp)
    m = cp.get('model', cp) if isinstance(cp, dict) else cp
    s = cp.get('scaler') if isinstance(cp, dict) else None
    if hasattr(m, 'classes_'):
        classes = list(m.classes_)
        gestures = list(g['gestures'].keys())
        unmatched = [c for c in classes if c not in gestures]
        if unmatched:
            print(f'Model classes not in gestures.json: {unmatched}')
            sys.exit(1)
    nf = s.n_features_in_ if s else getattr(m, 'n_features_in_', None)
    if nf:
        ch = g.get('emg_channels', {}).get('active', [])
        expect = len(ch) * 4
        if nf != expect:
            print(f'Feature mismatch: model={nf} config={expect}')
            sys.exit(1)
except ImportError:
    print('joblib not installed')
    sys.exit(2)
" 2>/dev/null
rc=$?
[ $rc -eq 0 ] && pass "model valid + matches config"
[ $rc -eq 1 ] && fail "model validation" "mismatch"
[ $rc -eq 2 ] && skip "model validation (joblib missing)"
echo

# ═══════════════════════════════════════════════════
# GROUP 9: Cross-compatibility checks
# ═══════════════════════════════════════════════════
echo -e "${C}[9] Cross-Compatibility${N}"

# gestures.json hysteresis has all 3 fields
python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
h = g.get('hysteresis', {})
for k in ['rest_to_active','active_to_rest','active_to_active']:
    assert k in h, f'missing hysteresis.{k}'
    assert isinstance(h[k], int), f'hysteresis.{k} not int'
" 2>/dev/null && pass "hysteresis fields valid" || fail "hysteresis" "missing or invalid"

# confidence curve
python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
c = g.get('confidence', {})
assert c.get('curve') in ('quadratic','linear'), 'bad curve type'
assert 0 <= c.get('floor_pct',0) < c.get('ceil_pct',100) <= 100, 'floor >= ceil'
" 2>/dev/null && pass "confidence curve valid" || fail "confidence" "invalid"

# runtime.json array lengths = 6
python3 -c "
import json, sys
with open('${RT}') as f: g = json.load(f)
for k in ['servo_min_us','servo_max_us','smooth_velocity_us_per_s','smooth_accel_us_per_s2']:
    assert len(g[k]) == 6, f'{k} length != 6'
" 2>/dev/null && pass "runtime arrays length=6" || fail "runtime arrays" "wrong length"

# emg_channels.active and names same length
python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
ec = g['emg_channels']
assert len(ec['active']) == len(ec['names']), 'active/names length mismatch'
for ch in ec['active']:
    assert 0 <= ch <= 7, f'channel {ch} out of range'
" 2>/dev/null && pass "emg_channels active/names consistent" || fail "emg_channels" "mismatch"
echo

# ═══════════════════════════════════════════════════
# GROUP 10: Hardware tests (skip if --quick)
# ═══════════════════════════════════════════════════
if [ ${QUICK} -eq 0 ]; then
    echo -e "${C}[10] Hardware (skip with --quick)${N}"
    [ -e /dev/i2c-1 ] && pass "/dev/i2c-1 exists" || skip "I2C not available"
    [ -e /dev/spidev0.0 ] && pass "/dev/spidev0.0 exists" || skip "SPI not available"
    [ -e /dev/ttyAMA0 ] && pass "UART /dev/ttyAMA0 exists" || skip "UART not enabled"
    command -v aplay >/dev/null && pass "aplay available" || skip "aplay not installed"
    command -v espeak-ng >/dev/null && pass "espeak-ng available" || skip "espeak-ng not installed"
    command -v amixer >/dev/null && pass "amixer available" || skip "amixer not installed"
    echo
fi

# ═══════════════════════════════════════════════════
# SUMMARY
# ═══════════════════════════════════════════════════
TOTAL=$((PASS + FAIL + SKIP))
echo -e "${C}═══════════════════════════════════════════════════${N}"
echo -e "  ${G}PASS: ${PASS}${N}  ${R}FAIL: ${FAIL}${N}  ${Y}SKIP: ${SKIP}${N}  TOTAL: ${TOTAL}"
echo -e "${C}═══════════════════════════════════════════════════${N}"

[ ${FAIL} -eq 0 ] && echo -e "  ${G}All checks passed.${N}" || echo -e "  ${R}Fix ${FAIL} failures before continuing.${N}"
echo
exit ${FAIL}
