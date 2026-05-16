#!/bin/bash
##
##  scripts/_default_runtime_json.sh — Shared helper sourced by setup_pi.sh
##  and configure.sh to emit a known-good default config/runtime.json.
##
##  Usage:
##      source "${SCRIPT_DIR}/_default_runtime_json.sh"
##      emit_default_runtime_json /path/to/runtime.json
##
##  Defaults match what cpcu_dsp.py and cpcu_config.c bake in. Operator
##  can customize via the TUI editor or by editing the file directly.
##
##  Why this exists:
##      - setup_pi.sh creates a default runtime.json on a fresh Pi if
##        none is present (so the kernel can boot).
##      - configure.sh --reset --runtime wants to regenerate the same
##        default on demand.
##      Rather than duplicate the JSON template across two scripts, both
##      source this file and call emit_default_runtime_json.

emit_default_runtime_json() {
    local target="$1"
    if [ -z "${target}" ]; then
        echo "[_default_runtime_json] ERROR: target path required" >&2
        return 1
    fi
    mkdir -p "$(dirname "${target}")"
    cat > "${target}" << 'RUNTIMEEOF'
{
    "// schema_version": "REQUIRED. Bump when removing/renaming fields.",
    "schema_version": 1,
    "// servo_min_us": "Per-servo minimum pulse width. Order: S0=Base, S1=Upper, S2=Last, S3=Joint1, S4=Joint2, S5=Gripper.",
    "servo_min_us": [ 498, 1074, 1074, 1001, 1001,  976 ],
    "// servo_max_us": "Per-servo maximum pulse width. Same order.",
    "servo_max_us": [ 2500, 1953, 1953, 2002, 2002, 1733 ],
    "// servo_bias_us": "Per-servo trim offset, signed micros (-100..+100).",
    "servo_bias_us": [ 0, 0, 0, 0, 0, 0 ],
    "// smooth_velocity": "Max us/tick (50 Hz tick) per servo.",
    "smooth_velocity": [ 12, 12, 12, 8, 8, 8 ],
    "// smooth_accel": "Max us/tick² per servo.",
    "smooth_accel":    [ 2, 2, 2, 1, 1, 1 ],
    "// smooth_deadband": "Hold-pose deadband in micros per servo (jitter suppression).",
    "smooth_deadband": [ 10, 10, 10, 10, 10, 10 ],
    "// interp_conf_floor_pct": "SVM confidence floor for class hold (0-100).",
    "interp_conf_floor_pct": 40,
    "// interp_conf_ceil_pct": "SVM confidence ceiling above which class commits (0-100).",
    "interp_conf_ceil_pct": 85,
    "// hysteresis_votes": "Number of consecutive votes needed for class commit.",
    "hysteresis_votes": 3,
    "// grip_firm_us": "Soft-grip firm-hold pulse width.",
    "grip_firm_us": 1100,
    "// grip_touch_us": "Soft-grip touch pulse width.",
    "grip_touch_us": 1200,
    "// grip_open_us": "Soft-grip fully-open pulse width.",
    "grip_open_us": 1500,
    "// grip_stall_recover_ms": "Time before stall watchdog retreats to grip_touch_us.",
    "grip_stall_recover_ms": 2000,
    "// gesture_velocity": "v2.3.5 hybrid velocity-mode gestures.",
    "gesture_velocity": {
        "rest":         [ 0, 0, 0, 0, 0, 0 ],
        "biceps_flex":  [ 0, 200, 0, 0, 0, 0 ],
        "hand_flex":    [ 0, 0, 0, 100, 100, 100 ]
    }
}
RUNTIMEEOF
    return $?
}
