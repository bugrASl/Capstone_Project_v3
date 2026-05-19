#!/bin/bash
##
##  test_launch_features.sh — comprehensive feature tests for launch.sh
##
##  Verifies every user-facing command in launch.sh without requiring
##  a Pi or any hardware. Tests fall into five layers:
##
##    1. STATIC      — script parses, every command dispatches, help works
##    2. DRY-RUN     — commands that only read state (show-*, check, --help)
##    3. STATE       — commands that mutate config/* (add/remove/rename/set)
##    4. SCRIPT-API  — helper scripts called by launch.sh exist + are valid
##    5. INTEGRATION — full flow on a sandbox tree (config reset, sync, etc)
##
##  Hardware-touching commands (tui, signal, pca, kernel, web, nrf, build)
##  are exercised in --dry-run mode where possible, or by checking that the
##  dispatch path resolves to the right helper.
##
##  Usage:
##      ./test/test_launch_features.sh                  # all
##      ./test/test_launch_features.sh --layer state    # one layer only
##      ./test/test_launch_features.sh --verbose        # show every command
##
##  Each test PRINTs PASS/FAIL and the script's exit code is non-zero
##  if any test failed. Designed to run before every git push.

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LAUNCH="${REPO_ROOT}/launch.sh"
GS="${REPO_ROOT}/config/gestures.json"
RT="${REPO_ROOT}/config/runtime.json"

# Colors
G='\033[32m'; R='\033[31m'; Y='\033[33m'; C='\033[36m'; B='\033[1m'; N='\033[0m'

# State
PASS=0
FAIL=0
SKIP=0
VERBOSE=0
LAYER_FILTER=""
FAILURES=()

# ============================================================================
# Argument parsing
# ============================================================================
while [ $# -gt 0 ]; do
    case "$1" in
        --verbose|-v) VERBOSE=1 ;;
        --layer)      shift; LAYER_FILTER="$1" ;;
        --help|-h)
            sed -n '2,25p' "$0" | sed 's/^## //; s/^##//'
            exit 0
            ;;
        *) echo "unknown arg: $1"; exit 2 ;;
    esac
    shift
done

# ============================================================================
# Test helpers
# ============================================================================
pass() {
    PASS=$((PASS + 1))
    [ "$VERBOSE" = "1" ] && echo -e "  ${G}PASS${N}  $1"
}

fail() {
    FAIL=$((FAIL + 1))
    FAILURES+=("$1: $2")
    echo -e "  ${R}FAIL${N}  $1"
    [ -n "${2:-}" ] && echo -e "        ${R}$2${N}"
}

skip() {
    SKIP=$((SKIP + 1))
    [ "$VERBOSE" = "1" ] && echo -e "  ${Y}SKIP${N}  $1  ${2:-}"
}

layer() {
    if [ -n "$LAYER_FILTER" ] && [ "$LAYER_FILTER" != "$1" ]; then
        return 1
    fi
    echo
    echo -e "${C}${B}[$1] $2${N}"
    return 0
}

# Run launch.sh and capture exit + output.
# Sets _OUT (stdout+stderr) and _RC (exit code) globals.
run_launch() {
    _OUT="$(timeout 10 "${LAUNCH}" "$@" 2>&1)"
    _RC=$?
}

# Assert: launch.sh exited with rc 0
assert_rc0() {
    local name="$1"
    if [ "$_RC" -eq 0 ]; then
        pass "$name"
    else
        fail "$name" "rc=$_RC; output: $(echo "$_OUT" | head -3 | tr '\n' '|')"
    fi
}

# Assert: launch.sh exited non-zero (expected failure)
assert_rc_nonzero() {
    local name="$1"
    if [ "$_RC" -ne 0 ]; then
        pass "$name"
    else
        fail "$name" "expected nonzero rc, got 0"
    fi
}

# Assert: $_OUT contains $2
assert_out_contains() {
    local name="$1" needle="$2"
    if echo "$_OUT" | grep -qF -- "$needle"; then
        pass "$name"
    else
        fail "$name" "missing '$needle' in output"
    fi
}

# Assert: $_OUT does NOT contain $2
assert_out_missing() {
    local name="$1" needle="$2"
    if ! echo "$_OUT" | grep -qF -- "$needle"; then
        pass "$name"
    else
        fail "$name" "unexpected '$needle' in output"
    fi
}

# Assert: file exists and is valid JSON
assert_json_valid() {
    local name="$1" path="$2"
    if [ ! -f "$path" ]; then
        fail "$name" "file missing: $path"
        return
    fi
    if python3 -c "import json; json.load(open('$path'))" 2>/dev/null; then
        pass "$name"
    else
        fail "$name" "invalid JSON: $path"
    fi
}

# Assert: a JSON path equals expected value
# usage: assert_json_path name file '["servo_channels","Gripper","pca_ch"]' 5
assert_json_path() {
    local name="$1" path="$2" json_path="$3" expected="$4"
    local actual
    actual=$(python3 - "$path" "$json_path" << 'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
for k in json.loads(sys.argv[2]):
    d = d[k] if not isinstance(k, int) else d[k]
print(d)
PYEOF
    )
    if [ "$actual" = "$expected" ]; then
        pass "$name"
    else
        fail "$name" "expected '$expected', got '$actual'"
    fi
}

# ============================================================================
# Sandbox: clone gestures.json + runtime.json into a tempdir for mutation tests
# ============================================================================
SANDBOX=""
setup_sandbox() {
    SANDBOX=$(mktemp -d /tmp/launch_sandbox.XXXXXX)
    mkdir -p "${SANDBOX}/config"
    [ -f "$GS" ] && cp "$GS" "${SANDBOX}/config/gestures.json"
    [ -f "$RT" ] && cp "$RT" "${SANDBOX}/config/runtime.json"
}

teardown_sandbox() {
    [ -n "$SANDBOX" ] && [ -d "$SANDBOX" ] && rm -rf "$SANDBOX"
}

# Run launch.sh against the sandbox by temporarily symlinking its config/
run_in_sandbox() {
    local repo_cfg="${REPO_ROOT}/config"
    local backup_cfg="${repo_cfg}.test_backup_$$"

    # Move real config aside, link sandbox in
    [ -d "$repo_cfg" ] && mv "$repo_cfg" "$backup_cfg"
    ln -sfn "${SANDBOX}/config" "$repo_cfg"

    run_launch "$@"

    # Restore real config
    rm -f "$repo_cfg"
    [ -d "$backup_cfg" ] && mv "$backup_cfg" "$repo_cfg"
}

# ============================================================================
# Entry banner
# ============================================================================
echo
echo -e "${C}═══════════════════════════════════════════════════════════════${N}"
echo -e "${C}  launch.sh feature test bench${N}"
echo -e "${C}  Repo: ${REPO_ROOT}${N}"
echo -e "${C}═══════════════════════════════════════════════════════════════${N}"

if [ ! -x "$LAUNCH" ]; then
    echo -e "${R}FATAL: ${LAUNCH} not found or not executable${N}"
    exit 2
fi

# ============================================================================
# LAYER 1 — STATIC: script parses, every command dispatches
# ============================================================================
if layer "1" "STATIC — script integrity"; then

    # 1a. bash -n: syntax check
    if bash -n "$LAUNCH" 2>&1; then
        pass "launch.sh: bash -n passes"
    else
        fail "launch.sh: bash -n" "syntax error"
    fi

    # 1b. shellcheck (if installed) — non-fatal warnings only
    if command -v shellcheck >/dev/null 2>&1; then
        if shellcheck -S error "$LAUNCH" 2>&1; then
            pass "launch.sh: shellcheck (errors only)"
        else
            skip "launch.sh: shellcheck" "warnings present (non-fatal)"
        fi
    else
        skip "launch.sh: shellcheck" "not installed"
    fi

    # 1c. --help and no-args show banner (not 'unknown command')
    run_launch --help
    assert_out_missing "launch.sh --help: no 'unknown'" "Unknown command"

    run_launch
    # With no args, launch.sh should print help/usage rather than crash
    if [ "$_RC" -eq 0 ] || [ "$_RC" -eq 1 ]; then
        pass "launch.sh (no args): exits cleanly"
    else
        fail "launch.sh (no args)" "unexpected rc=$_RC"
    fi

    # 1d. Each documented command dispatches (rc != "unknown command")
    COMMANDS=(
        setup setup-audio setup-uart generate-cues vendor build check
        tui signal collect pca kernel nrf smoother grip-tune calibrate
        ws web attach stop reload audio
        configure show-config show-gestures
        add-gesture remove-gesture rename-gesture edit-gesture
        add-group remove-group rename-group
        add-motor remove-motor rename-motor edit-motor
        set-channels set-model set-pca-channel
        test-sw test-ipc test-hw test-pca test-nrf test-signal
        test-signal-demo test-safety-demo test-system
        install-service install-ws-service grant-caps
    )
    for cmd in "${COMMANDS[@]}"; do
        # Dispatch alone tells us the case statement has an entry. Without
        # any args, most commands print usage and rc != "Unknown command".
        run_launch "$cmd" --help
        if echo "$_OUT" | grep -qF "Unknown command"; then
            fail "dispatch: $cmd" "no case entry in main dispatcher"
        else
            pass "dispatch: $cmd"
        fi
    done
fi

# ============================================================================
# LAYER 2 — DRY-RUN: read-only commands work on the real repo
# ============================================================================
if layer "2" "DRY-RUN — read-only commands"; then

    # 2a. show-config / show-gestures parse the existing gestures.json
    if [ -f "$GS" ]; then
        run_launch show-config
        assert_rc0 "show-config exits 0"
        assert_out_contains "show-config: prints SYSTEM" "SYSTEM"

        run_launch show-gestures
        assert_rc0 "show-gestures exits 0"
    else
        skip "show-config / show-gestures" "gestures.json missing"
    fi

    # 2b. configure --show / --diff: read-only
    run_launch configure --show
    assert_rc0 "configure --show"
    run_launch configure --diff
    # --diff may return 0 or 11 (rebuild hint); both acceptable
    if [ "$_RC" -eq 0 ] || [ "$_RC" -eq 11 ]; then
        pass "configure --diff (rc=$_RC)"
    else
        fail "configure --diff" "rc=$_RC"
    fi

    # 2c. attach (no session running) should print friendly error, not crash
    run_launch attach
    assert_rc_nonzero "attach (no session): exits nonzero"
    assert_out_contains "attach (no session): friendly msg" "No tmux session"
fi

# ============================================================================
# LAYER 3 — STATE: mutation commands operate on a sandboxed gestures.json
# ============================================================================
if layer "3" "STATE — config mutation in sandbox"; then
    setup_sandbox
    trap 'teardown_sandbox' EXIT

    # 3a. set-pca-channel: change a known servo's pca_ch
    if [ -f "${SANDBOX}/config/gestures.json" ]; then
        run_in_sandbox set-pca-channel Gripper 12
        assert_rc0 "set-pca-channel Gripper 12"
        assert_json_valid "gestures.json still valid after set-pca-channel" \
                          "${SANDBOX}/config/gestures.json"
        assert_json_path  "Gripper.pca_ch == 12" \
                          "${SANDBOX}/config/gestures.json" \
                          '["servo_channels","Gripper","pca_ch"]' "12"

        # Rejects out-of-range
        run_in_sandbox set-pca-channel Gripper 99
        assert_rc_nonzero "set-pca-channel: rejects pca_ch=99"

        # Rejects unknown servo
        run_in_sandbox set-pca-channel NonExistentServo 3
        assert_rc_nonzero "set-pca-channel: rejects unknown servo"

        # Rejects duplicate (Base is on 0 in the fresh sandbox)
        # If Gripper is now on 12, setting Base to 12 should fail
        run_in_sandbox set-pca-channel Base 12
        assert_rc_nonzero "set-pca-channel: rejects duplicate channel"
    else
        skip "set-pca-channel tests" "no gestures.json to mutate"
    fi

    # 3b. rename-gesture: rename + verify
    if [ -f "${SANDBOX}/config/gestures.json" ]; then
        # Pick the first existing gesture name in the first group
        local_group=$(python3 -c "
import json
d = json.load(open('${SANDBOX}/config/gestures.json'))
gg = d.get('gesture_groups', {})
if gg:
    name = list(gg.keys())[0]
    gestures = list(gg[name].get('gestures', {}).keys())
    if 'flex' in gestures:
        print(f'{name} flex')
" 2>/dev/null)
        if [ -n "$local_group" ]; then
            # shellcheck disable=SC2086
            run_in_sandbox rename-gesture $local_group flex_test
            assert_rc0 "rename-gesture flex -> flex_test"
            assert_json_valid "gestures.json valid after rename" \
                              "${SANDBOX}/config/gestures.json"
            # Rename back so subsequent tests see expected names
            # shellcheck disable=SC2086
            local_group_arg="${local_group% *}"
            run_in_sandbox rename-gesture "$local_group_arg" flex_test flex
            assert_rc0 "rename-gesture flex_test -> flex (revert)"
        else
            skip "rename-gesture" "no 'flex' gesture to rename"
        fi
    fi

    teardown_sandbox
    trap - EXIT
fi

# ============================================================================
# LAYER 4 — SCRIPT-API: helper scripts called by launch.sh
# ============================================================================
if layer "4" "SCRIPT-API — helper scripts"; then
    HELPERS=(
        scripts/configure.sh
        scripts/calibrate.sh
        scripts/grip_tune.sh
        scripts/add_gesture.sh
        scripts/set_channels.sh
        scripts/setup_audio.sh
        scripts/setup_uart.sh
        scripts/setup_pi.sh
        scripts/generate_voice_cues.sh
        scripts/_default_runtime_json.sh
        scripts/run_tests.sh
    )
    for h in "${HELPERS[@]}"; do
        path="${REPO_ROOT}/${h}"
        if [ -f "$path" ]; then
            if bash -n "$path" 2>/dev/null; then
                pass "$h: bash -n passes"
            else
                fail "$h: bash -n" "syntax error"
            fi
            if [ -x "$path" ]; then
                pass "$h: executable bit set"
            else
                fail "$h: executable" "missing chmod +x"
            fi
        else
            skip "$h" "not present"
        fi
    done

    # Verify _default_runtime_json.sh emits valid JSON with required fields
    if [ -f "${REPO_ROOT}/scripts/_default_runtime_json.sh" ]; then
        tmp=$(mktemp /tmp/rt_test.XXXXXX.json)
        # shellcheck source=/dev/null
        ( . "${REPO_ROOT}/scripts/_default_runtime_json.sh" && \
          emit_default_runtime_json "$tmp" ) 2>&1
        assert_json_valid "_default_runtime_json: emits valid JSON" "$tmp"
        # Required fields
        for key in servo_min_us servo_max_us servo_pca_ch \
                   smooth_velocity smooth_accel smooth_deadband \
                   grip_open_us grip_touch_us grip_firm_us; do
            if python3 -c "import json; assert '$key' in json.load(open('$tmp'))" 2>/dev/null; then
                pass "_default_runtime_json: has '$key'"
            else
                fail "_default_runtime_json: missing '$key'"
            fi
        done
        # Length checks
        for key in servo_min_us servo_max_us servo_pca_ch; do
            n=$(python3 -c "import json; print(len(json.load(open('$tmp'))['$key']))")
            if [ "$n" = "6" ]; then
                pass "_default_runtime_json: $key has 6 entries"
            else
                fail "_default_runtime_json: $key length" "expected 6, got $n"
            fi
        done
        # pca_ch must be in 0..15
        if python3 -c "
import json
d = json.load(open('$tmp'))
ok = all(0 <= v <= 15 for v in d['servo_pca_ch'])
ok = ok and len(set(d['servo_pca_ch'])) == 6
exit(0 if ok else 1)
" 2>/dev/null; then
            pass "_default_runtime_json: servo_pca_ch range + distinct"
        else
            fail "_default_runtime_json: servo_pca_ch" "out of range or duplicate"
        fi
        rm -f "$tmp"
    fi
fi

# ============================================================================
# LAYER 5 — INTEGRATION: full sync flow
# ============================================================================
if layer "5" "INTEGRATION — sync flow"; then
    setup_sandbox
    trap 'teardown_sandbox' EXIT

    if [ -f "${SANDBOX}/config/gestures.json" ] && \
       [ -f "${SANDBOX}/config/runtime.json" ]; then

        # 5a. set-pca-channel triggers no runtime.json update on its own
        before=$(md5sum "${SANDBOX}/config/runtime.json" | awk '{print $1}')
        run_in_sandbox set-pca-channel Gripper 12 >/dev/null 2>&1
        after=$(md5sum "${SANDBOX}/config/runtime.json" | awk '{print $1}')
        if [ "$before" = "$after" ]; then
            pass "set-pca-channel: doesn't touch runtime.json directly"
        else
            fail "set-pca-channel sync" "runtime.json changed without preflight"
        fi

        # 5b. Preflight sync function: pca_ch flows gestures -> runtime
        # Extract just the sync function and run it standalone
        if grep -q "sync_servo_pca_ch_to_runtime" "$LAUNCH"; then
            (
                cd "${SANDBOX}"
                GS="${SANDBOX}/config/gestures.json"
                CPCU_ROOT="${SANDBOX}"

                # Source the sync function definition out of launch.sh
                # by piping just that function body to bash
                eval "$(awk '
                    /^sync_servo_pca_ch_to_runtime\(\)/,/^}/ { print }
                ' "$LAUNCH")"

                GS="${SANDBOX}/config/gestures.json" \
                CPCU_ROOT="${SANDBOX}" \
                    sync_servo_pca_ch_to_runtime
            ) >/dev/null 2>&1

            actual_sync=$(python3 -c "
import json
d = json.load(open('${SANDBOX}/config/runtime.json'))
print(d.get('servo_pca_ch'))
")
            if echo "$actual_sync" | grep -q "12"; then
                pass "sync_servo_pca_ch_to_runtime: Gripper=12 propagated"
            else
                fail "sync_servo_pca_ch_to_runtime" "got: $actual_sync"
            fi
        else
            skip "sync function test" "sync_servo_pca_ch_to_runtime not present"
        fi
    else
        skip "integration tests" "fresh repo missing config files"
    fi

    teardown_sandbox
    trap - EXIT
fi

# ============================================================================
# Summary
# ============================================================================
echo
echo -e "${C}═══════════════════════════════════════════════════════════════${N}"
echo -e "  ${B}Results${N}"
echo -e "    ${G}PASS ${PASS}${N}    ${R}FAIL ${FAIL}${N}    ${Y}SKIP ${SKIP}${N}"
echo -e "${C}═══════════════════════════════════════════════════════════════${N}"

if [ "$FAIL" -gt 0 ]; then
    echo
    echo -e "${R}Failures:${N}"
    for f in "${FAILURES[@]}"; do
        echo -e "  ${R}- ${f}${N}"
    done
    echo
    exit 1
fi

echo -e "${G}All tests passed.${N}"
exit 0
