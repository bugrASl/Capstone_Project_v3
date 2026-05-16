#!/bin/bash
## ═══════════════════════════════════════════════════════════════════
##  launch.sh v3.0 — ALL new commands, helpers, and help text.
##  Paste into existing launch.sh at marked locations.
## ═══════════════════════════════════════════════════════════════════

GS="${CPCU_ROOT}/config/gestures.json"

# ══════════════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════════════

_run_script() {
    local s="${SCRIPTS_DIR}/$1"; shift
    ensure_helper_executable "${s}"
    REPO_ROOT="${CPCU_ROOT}" "${s}" "$@"
}

run_grip_tune()      { _run_script grip_tune.sh "$@"; }
run_calibrate()      { _run_script calibrate.sh "$@"; }
run_add_gesture()    { _run_script add_gesture.sh "$@"; }
run_generate_cues()  { _run_script generate_voice_cues.sh "$@"; }
run_set_channels()   { _run_script set_channels.sh "$@"; }
cmd_setup_audio()    { _run_script setup_audio.sh "$@"; }
cmd_setup_uart()     { _run_script setup_uart.sh "$@"; }

# ── show-config: print EVERYTHING ──
cmd_show_config() {
    python3 << 'PYEOF'
import json, os
gs_path = os.environ.get("CPCU_GS", "") or "${GS}"
with open(gs_path) as f: g = json.load(f)

C="\033[36m"; G="\033[32m"; Y="\033[33m"; N="\033[0m"; B="\033[1m"
def h(t): print(f"\n  {C}{t}{N}")
def sep(): print("  " + "─" * 58)

h("SYSTEM CONFIGURATION")
sep()
print(f"  Model:      {g.get('model_path','?')}")
print(f"  Audio:      {g.get('audio_mode','off')} @ {g.get('audio_volume_pct',80)}%")
cc = g.get("confidence", {})
print(f"  Confidence: {cc.get('curve','?')} floor={cc.get('floor_pct','?')}% ceil={cc.get('ceil_pct','?')}%")
hy = g.get("hysteresis", {})
print(f"  Hysteresis: rest→active={hy.get('rest_to_active','?')} "
      f"active→active={hy.get('active_to_active','?')} "
      f"active→rest={hy.get('active_to_rest','?')}")

h("SERVO MOTORS")
sep()
fmt = "  {:<12s} PCA={:<2d}  range=[{:>4d}, {:>4d}]  neutral={:>4d}"
for name, sd in g.get("servo_channels", {}).items():
    print(fmt.format(name, sd["pca_ch"], sd["min_us"], sd["max_us"], sd["neutral_us"]))

h("EMG CHANNELS")
sep()
ec = g.get("emg_channels", {})
active = ec.get("active", [])
names  = ec.get("names", [])
for i, ch in enumerate(active):
    nm = names[i] if i < len(names) else "?"
    print(f"  ADC ch{ch}  →  {nm}")

h("GESTURES")
sep()
fmt2 = "  {:<15s} {:<8s} {:<30s} {:s}"
print(fmt2.format("NAME", "MODE", "SERVOS", "AUDIO"))
sep()
for gn, gd in g.get("gestures", {}).items():
    mode = gd.get("mode", "?")
    chs = gd.get("channels", {})
    ch_s = ", ".join(f"{s}={d.get('rate_us_s',0)}" for s,d in chs.items()) or "-"
    a = gd.get("audio", {})
    a_s = a.get("voice", "-")
    print(fmt2.format(gn, mode, ch_s, a_s))

h("AUDIO EVENTS")
sep()
for en, ed in g.get("audio_events", {}).items():
    print(f"  {en:<20s} voice={ed.get('voice','-'):<28s} {ed.get('freq_hz','-')}Hz")

print()
PYEOF
}

# ── show-gestures: compact version ──
cmd_show_gestures() {
    cmd_show_config
}

# ── audio: mode/volume/test/show ──
cmd_audio() {
    local sub="${1:-show}"; shift 2>/dev/null || true
    case "${sub}" in
        off|voice|freq)
            python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
g['audio_mode'] = '${sub}'
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
"
            ok "Audio mode: ${sub}. Run './launch.sh reload --audio' to apply." ;;
        volume|vol)
            local pct="${1:-}"
            if [ -z "${pct}" ]; then
                python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
print(f'  Volume: {g.get(\"audio_volume_pct\", 80)}%')
"
            else
                python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
g['audio_volume_pct'] = max(0, min(100, int(${pct})))
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
"
                amixer set 'PCM' "${pct}%" 2>/dev/null && ok "Volume: ${pct}%" || ok "Saved."
            fi ;;
        test)
            local wav="${CPCU_ROOT}/config/audio_cues/voice_flex.wav"
            [ -f "${wav}" ] && aplay -q "${wav}" 2>/dev/null && ok "Working." || \
                err "No audio. Run './launch.sh setup-audio' and './launch.sh generate-cues'." ;;
        show|"") cmd_show_config ;;
        *) err "Usage: ./launch.sh audio [off|voice|freq|volume N|test]" ;;
    esac
}

# ── rename-motor ──
cmd_rename_motor() {
    local old="${1:-}" new="${2:-}"
    if [ -z "$old" ] || [ -z "$new" ]; then
        err "Usage: ./launch.sh rename-motor <old_name> <new_name>"
        python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
print('  Current motors: ' + ', '.join(g.get('servo_channels',{}).keys()))
"
        exit 1
    fi
    python3 << PYEOF
import json, sys
with open("${GS}") as f: g = json.load(f)
sc = g.get("servo_channels", {})
if "${old}" not in sc:
    print(f"  Motor '${old}' not found. Available: {list(sc.keys())}")
    sys.exit(1)
# rename in servo_channels
sc["${new}"] = sc.pop("${old}")
# rename in all gesture channel references
for gn, gd in g.get("gestures", {}).items():
    ch = gd.get("channels", {})
    if "${old}" in ch:
        ch["${new}"] = ch.pop("${old}")
    so = gd.get("smoother_override", {})
    if "${old}" in so:
        so["${new}"] = so.pop("${old}")
with open("${GS}", "w") as f: json.dump(g, f, indent=4)
print("  \033[32m✓\033[0m Renamed '${old}' → '${new}' (servo + all gesture refs)")
PYEOF
}

# ── remove-gesture ──
cmd_remove_gesture() {
    local name="${1:-}"
    if [ -z "$name" ]; then
        err "Usage: ./launch.sh remove-gesture <name>"
        exit 1
    fi
    python3 << PYEOF
import json, sys
with open("${GS}") as f: g = json.load(f)
if "${name}" not in g.get("gestures", {}):
    print(f"  '${name}' not found.")
    sys.exit(1)
if "${name}" == "rest":
    print("  Cannot remove 'rest'.")
    sys.exit(1)
del g["gestures"]["${name}"]
with open("${GS}", "w") as f: json.dump(g, f, indent=4)
print("  \033[32m✓\033[0m Removed '${name}'. Run './launch.sh reload' to apply.")
PYEOF
}

# ── rename-gesture ──
cmd_rename_gesture() {
    local old="${1:-}" new="${2:-}"
    if [ -z "$old" ] || [ -z "$new" ]; then
        err "Usage: ./launch.sh rename-gesture <old_name> <new_name>"
        exit 1
    fi
    python3 << PYEOF
import json, sys
with open("${GS}") as f: g = json.load(f)
gs = g.get("gestures", {})
if "${old}" not in gs:
    print(f"  '${old}' not found.")
    sys.exit(1)
gs["${new}"] = gs.pop("${old}")
with open("${GS}", "w") as f: json.dump(g, f, indent=4)
print("  \033[32m✓\033[0m Renamed '${old}' → '${new}'.")
print("  \033[33m⚠\033[0m Model classes must match — retrain if class name changed.")
PYEOF
}

# ── edit-gesture (change servo mapping for existing gesture) ──
cmd_edit_gesture() {
    local name="${1:-}"
    if [ -z "$name" ]; then
        err "Usage: ./launch.sh edit-gesture <name>"
        python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
print('  Gestures: ' + ', '.join(g.get('gestures',{}).keys()))
"
        exit 1
    fi
    python3 << PYEOF
import json, sys
with open("${GS}") as f: g = json.load(f)
gs = g.get("gestures", {})
if "${name}" not in gs:
    print(f"  '${name}' not found.")
    sys.exit(1)
gd = gs["${name}"]
sc = g.get("servo_channels", {})
motors = list(sc.keys())
print(f"\n  Editing gesture: ${name}")
print(f"  Mode: {gd.get('mode', '?')}")
print(f"  Available motors: {motors}")
print(f"  Current mapping: {gd.get('channels', {})}")
print(f"\n  Enter new servo mappings (empty line to finish):")
channels = {}
while True:
    line = input("  Motor Rate(us/s) [e.g. Gripper -400]: ").strip()
    if not line: break
    parts = line.split()
    if len(parts) != 2:
        print("  Format: MotorName Rate")
        continue
    mname, rate = parts[0], int(parts[1])
    if mname not in sc:
        print(f"  Unknown motor: {mname}. Available: {motors}")
        continue
    channels[mname] = {"rate_us_s": rate}
    snap = input(f"  {mname} snap mode? (y/n) [n]: ").strip()
    if snap.lower() == 'y':
        channels[mname]["snap"] = True
if channels:
    gd["channels"] = channels
    with open("${GS}", "w") as f: json.dump(g, f, indent=4)
    print(f"\n  \033[32m✓\033[0m Updated '{name}' → {channels}")
else:
    print("  No changes.")
PYEOF
}

# ── add-motor ──
cmd_add_motor() {
    local name="${1:-}" pca="${2:-}"
    if [ -z "$name" ] || [ -z "$pca" ]; then
        err "Usage: ./launch.sh add-motor <name> <pca_channel>"
        echo "  Example: ./launch.sh add-motor Thumb 6"
        exit 1
    fi
    python3 << PYEOF
import json
with open("${GS}") as f: g = json.load(f)
sc = g.get("servo_channels", {})
# check PCA channel conflict
for mn, md in sc.items():
    if md["pca_ch"] == ${pca}:
        print(f"  PCA channel ${pca} already used by '{mn}'.")
        exit(1)
sc["${name}"] = {
    "pca_ch": ${pca},
    "min_us": 500,
    "max_us": 2500,
    "neutral_us": 1500
}
with open("${GS}", "w") as f: json.dump(g, f, indent=4)
print("  \033[32m✓\033[0m Added motor '${name}' on PCA ch${pca}.")
print("  Tune limits with: ./launch.sh edit-motor ${name}")
PYEOF
}

# ── edit-motor (change limits) ──
cmd_edit_motor() {
    local name="${1:-}"
    if [ -z "$name" ]; then
        err "Usage: ./launch.sh edit-motor <name>"
        exit 1
    fi
    python3 << PYEOF
import json, sys
with open("${GS}") as f: g = json.load(f)
sc = g.get("servo_channels", {})
if "${name}" not in sc:
    print(f"  Motor '${name}' not found. Available: {list(sc.keys())}")
    sys.exit(1)
sd = sc["${name}"]
print(f"\n  Motor: ${name}  PCA ch{sd['pca_ch']}")
print(f"  Current: min={sd['min_us']} max={sd['max_us']} neutral={sd['neutral_us']}")
mn = input(f"  min_us [{sd['min_us']}]: ").strip()
mx = input(f"  max_us [{sd['max_us']}]: ").strip()
ne = input(f"  neutral_us [{sd['neutral_us']}]: ").strip()
if mn: sd["min_us"] = int(mn)
if mx: sd["max_us"] = int(mx)
if ne: sd["neutral_us"] = int(ne)
with open("${GS}", "w") as f: json.dump(g, f, indent=4)
print(f"  \033[32m✓\033[0m Updated: min={sd['min_us']} max={sd['max_us']} neutral={sd['neutral_us']}")
PYEOF
}

# ── set-model ──
cmd_set_model() {
    local path="${1:-}"
    if [ -z "$path" ]; then
        err "Usage: ./launch.sh set-model <path>"
        echo "  Example: ./launch.sh set-model models/model_5ch.pkl"
        echo
        echo "  Available models:"
        ls -1 "${CPCU_ROOT}"/models/*.pkl 2>/dev/null | while read f; do
            printf "    %s\n" "$(basename "$f")"
        done
        exit 1
    fi
    local full="${path}"
    [ ! -f "${full}" ] && full="${CPCU_ROOT}/${path}"
    if [ ! -f "${full}" ]; then
        err "File not found: ${path}"
        exit 1
    fi
    python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
g['model_path'] = '${path}'
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
"
    ok "Model: ${path}"
    python3 -c "
import joblib, json, os
with open('${GS}') as f: g = json.load(f)
p = g['model_path']
if not os.path.isabs(p): p = os.path.join('${CPCU_ROOT}', p)
cp = joblib.load(p)
m = cp.get('model', cp) if isinstance(cp, dict) else cp
s = cp.get('scaler') if isinstance(cp, dict) else None
nf = s.n_features_in_ if s else getattr(m, 'n_features_in_', '?')
cl = list(m.classes_) if hasattr(m, 'classes_') else '?'
ch = g.get('emg_channels',{}).get('active',[])
expect = len(ch) * 4
print(f'  Features: model={nf} config={expect} ({len(ch)} ch x 4)')
print(f'  Classes:  {cl}')
if nf == expect: print('  \033[32m✓ Compatible.\033[0m')
else: print('  \033[31m✗ Feature count mismatch.\033[0m')
" 2>/dev/null || warn "Could not validate (joblib missing?)."
    echo
    info "./launch.sh reload to apply."
}

# ── reload ──
cmd_reload() {
    local what="${1:-all}"
    case "${what}" in
        --dsp|dsp)    pkill -f cpcu_dsp.py 2>/dev/null || true; ok "DSP restarting." ;;
        --audio|audio) pkill -f cpcu_audio_daemon.py 2>/dev/null || true; ok "Audio restarting." ;;
        --all|all|"")
            pkill -HUP cpcu_kernel 2>/dev/null || true; sleep 0.3
            pkill -f cpcu_dsp.py 2>/dev/null || true
            pkill -f cpcu_audio_daemon.py 2>/dev/null || true
            ok "Full reload." ;;
    esac
}
# ══════════════════════════════════════════════════════════════════
#  CASE ENTRIES — add to case "${MODE}" in ... esac
# ══════════════════════════════════════════════════════════════════
#
#   setup-audio)        cmd_setup_audio "$@" ;;
#   setup-uart)         cmd_setup_uart "$@" ;;
#   grip-tune)          run_grip_tune "$@" ;;
#   calibrate)          run_calibrate "$@" ;;
#   add-gesture)        run_add_gesture "$@" ;;
#   remove-gesture)     cmd_remove_gesture "$@" ;;
#   rename-gesture)     cmd_rename_gesture "$@" ;;
#   edit-gesture)       cmd_edit_gesture "$@" ;;
#   add-motor)          cmd_add_motor "$@" ;;
#   edit-motor)         cmd_edit_motor "$@" ;;
#   rename-motor)       cmd_rename_motor "$@" ;;
#   set-channels)       run_set_channels "$@" ;;
#   set-model)          cmd_set_model "$@" ;;
#   generate-cues)      run_generate_cues "$@" ;;
#   audio)              cmd_audio "$@" ;;
#   show-config)        cmd_show_config ;;
#   show-gestures)      cmd_show_gestures ;;
#   reload)             cmd_reload "$@" ;;
# ══════════════════════════════════════════════════════════════════
#  FLAG PARSING — add to the flag-stripping loop
# ══════════════════════════════════════════════════════════════════
#
#   WITH_AUDIO=0; OPERATOR="default"; WITH_UART=0
#   --audio)    WITH_AUDIO=1 ;;
#   --uart)     WITH_UART=1 ;;
#   --operator) shift; OPERATOR="${1}" ;;
#   export CPCU_OPERATOR="${OPERATOR}"
#   [ "${WITH_UART}" = "1" ] && export CPCU_UART_DEBUG="/dev/ttyAMA0"
#
#   In tmux session, add audio pane if WITH_AUDIO=1:
#     tmux split-window -t "$SESSION" -v \
#       "CPCU_ROOT=${CPCU_ROOT} python3 ${PY}/cpcu_audio_daemon.py 2>&1 | tee ${LOG}"
# ══════════════════════════════════════════════════════════════════
#  HELP TEXT
# ══════════════════════════════════════════════════════════════════

print_v3_help() {
cat << 'HELPEOF'

  ═══════════════════════════════════════════════════════════════
   InfiniTech CPCU v3.0 — Complete Command Reference
  ═══════════════════════════════════════════════════════════════

  SETUP
  ─────────────────────────────────────────────────────────────
    ./launch.sh setup                         Pi one-time config
    ./launch.sh setup-audio                   I2S DAC + speaker
    ./launch.sh setup-uart                    UART debug output
    ./launch.sh build                         Compile + install
    ./launch.sh check                         Verify readiness

  RUNNING
  ─────────────────────────────────────────────────────────────
    ./launch.sh tui                           Dashboard
    ./launch.sh tui --audio                   + voice feedback
    ./launch.sh tui --uart                    + UART debug to PC
    ./launch.sh tui --with-ws                 + web dashboard
    ./launch.sh tui --audio --uart --with-ws  All features
    ./launch.sh tui --operator NAME           Operator profile
    ./launch.sh ws                            Web only
    ./launch.sh stop                          Stop everything
    ./launch.sh attach                        Re-attach tmux

  TESTING
  ─────────────────────────────────────────────────────────────
    ./launch.sh test-sw                       Software tests
    ./launch.sh test-hw                       + hardware probes
    ./launch.sh test-pca                      Servo check
    ./launch.sh test-system                   Full verification

  EMG CHANNELS
  ─────────────────────────────────────────────────────────────
    ./launch.sh set-channels 0 1 2            3-channel mode
    ./launch.sh set-channels 0 1 2 3 4        5-channel mode
    ./launch.sh set-channels 0 1 2 3 4 5 6 7  8-channel (full)

  SERVO MOTORS
  ─────────────────────────────────────────────────────────────
    ./launch.sh show-config                   Show all motors
    ./launch.sh add-motor Thumb 6             Add motor on PCA ch6
    ./launch.sh edit-motor Gripper            Edit limits
    ./launch.sh rename-motor Gripper Claw     Rename motor

  GESTURES
  ─────────────────────────────────────────────────────────────
    ./launch.sh show-config                   Show all gestures
    ./launch.sh add-gesture                   Wizard (+ audio)
    ./launch.sh edit-gesture flex             Change servo map
    ./launch.sh rename-gesture hand grip      Rename gesture
    ./launch.sh remove-gesture biceps         Delete gesture

  CALIBRATION
  ─────────────────────────────────────────────────────────────
    ./launch.sh grip-tune                     Gripper wizard
    ./launch.sh calibrate                     Rest + velocity
    ./launch.sh calibrate --operator ali      Per-person profile

  AUDIO (PCM5102A + PAM8403)
  ─────────────────────────────────────────────────────────────
    ./launch.sh audio                         Show audio config
    ./launch.sh audio off                     Disable
    ./launch.sh audio voice                   Spoken words
    ./launch.sh audio freq                    Frequency tones
    ./launch.sh audio volume 80               Set volume 0-100
    ./launch.sh audio test                    Play test sound
    ./launch.sh generate-cues                 Generate voice wavs

  CONFIG & RELOAD
  ─────────────────────────────────────────────────────────────
    ./launch.sh show-config                   Everything at once
    ./launch.sh set-model models/model_5ch.pkl Set active model
    ./launch.sh set-model                      List available models
    ./launch.sh reload                        Apply all changes
    ./launch.sh reload --dsp                  DSP only
    ./launch.sh reload --audio                Audio only

  UART DEBUG (to host PC via USB-UART adapter)
  ─────────────────────────────────────────────────────────────
    ./launch.sh setup-uart                    Enable UART on Pi
    ./launch.sh tui --uart                    Run with UART debug
    Host PC: python3 scripts/uart_monitor.py --port /dev/ttyUSB0
    Host PC: python3 scripts/uart_monitor.py --port COM3 --log data.csv

  EXAMPLES
  ─────────────────────────────────────────────────────────────

    # First-time setup:
    ./launch.sh setup && ./launch.sh setup-audio
    ./launch.sh build && ./launch.sh generate-cues

    # Daily use:
    ./launch.sh tui --audio --with-ws

    # Add a 7th servo motor:
    ./launch.sh add-motor Thumb 6
    ./launch.sh edit-motor Thumb

    # Create a new gesture using the new motor:
    ./launch.sh add-gesture
    ./launch.sh collect && # retrain...
    ./launch.sh reload

    # Upgrade from 3 to 5 EMG channels:
    ./launch.sh set-channels 0 1 2 3 4
    ./launch.sh collect && # retrain...
    ./launch.sh set-model models/model_5ch.pkl
    ./launch.sh reload

    # Rename things:
    ./launch.sh rename-motor Gripper Claw
    ./launch.sh rename-gesture hand grip

    # Calibrate for a new operator:
    ./launch.sh calibrate --operator nisa
    ./launch.sh tui --operator nisa --audio

    # Quick audio mode switch:
    ./launch.sh audio freq && ./launch.sh reload --audio

HELPEOF
}
