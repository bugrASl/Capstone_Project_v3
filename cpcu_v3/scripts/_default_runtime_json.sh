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
    "schema_version": 2,

    "// servo_min_us": "Per-servo minimum pulse width. Order matches sorted-by-pca_ch slot order: S0=Base, S1=Elbow, S2=Forearm, S3=Wrist1, S4=Wrist2, S5=Gripper.",
    "servo_min_us": [ 498, 1074, 1074, 1001, 1001,  976 ],
    "// servo_max_us": "Per-servo maximum pulse width. Same order.",
    "servo_max_us": [ 2500, 1953, 1953, 2002, 2002, 1733 ],
    "// servo_bias_us": "Per-servo trim offset, signed micros (-100..+100).",
    "servo_bias_us": [ 0, 0, 0, 0, 0, 0 ],

    "// smooth_velocity": "Max pulse rate per servo, us/s. The 50 Hz smoother multiplies by dt=0.02s, so 3000 us/s = 60 us per tick. SG90 mech max ~6666 us/s; we cap at 45% for gearbox margin.",
    "smooth_velocity": [ 3000, 3000, 3000, 3000, 3000, 1500 ],
    "// smooth_accel":    "Max acceleration per servo, us/s^2. v_max reached in ~5 ticks (100 ms) for snappy-but-jerk-free motion.",
    "smooth_accel":    [ 30000, 30000, 30000, 30000, 30000, 15000 ],
    "// smooth_deadband": "Hold-pose deadband in micros. 10 us ~= 2 PCA9685 LSBs.",
    "smooth_deadband": [ 10, 10, 10, 10, 10, 10 ],

    "// servo_pca_ch": "Logical slot (0..5) -> physical PCA9685 channel (0..15). launch.sh preflight syncs this from gestures.json's pca_ch fields; manual edits here are overwritten on the next launch.",
    "servo_pca_ch":    [ 0, 1, 2, 3, 4, 5 ],

    "// gravity_dir":  "Per-servo gravity-comp direction: +1 = gravity pulls TOWARD increasing-us, -1 = pulls toward decreasing-us, 0 = no comp. Elbow falls toward lower us (lift up needs help), Forearm tilts toward higher us.",
    "gravity_dir":        [ 0, -1, 1, 0, 0, 0 ],
    "// gravity_scale_pct": "Compensation strength as % of nominal velocity. 30 = boost UP-direction by 30%, dampen DOWN-direction by 30%. Tune in pca_testbench with G + ',' / '.' keys.",
    "gravity_scale_pct":  [ 0, 30, 30, 0, 0, 0 ],

    "// interp_conf_floor_pct": "Classifier confidence floor for class hold (0-100).",
    "interp_conf_floor_pct": 40,
    "// interp_conf_ceil_pct":  "Classifier confidence ceiling above which class commits (0-100).",
    "interp_conf_ceil_pct":  85,

    "// grip_firm_us":           "Soft-grip firm-hold pulse width.",
    "grip_firm_us": 1100,
    "// grip_touch_us":          "Soft-grip touch pulse width.",
    "grip_touch_us": 1200,
    "// grip_open_us":           "Soft-grip fully-open pulse width.",
    "grip_open_us": 1500,
    "// grip_stall_recover_ms":  "Time before stall watchdog retreats to grip_touch_us.",
    "grip_stall_recover_ms": 2000,

    "// safety_ignore_battery": "Set to 1 for bench tests with no battery wired. Disables BATT_CRITICAL detection so the FSM doesn't latch SAFE on first packet.",
    "safety_ignore_battery": 1,

    "// gravity_dir":  "Per-servo gravity-comp direction: +1 = gravity pulls TOWARD increasing-us, -1 = pulls toward decreasing-us, 0 = no comp. Default is OFF (all zeros) — turn on per servo only after empirical tuning.",
    "gravity_dir":        [ 0, 0, 0, 0, 0, 0 ],
    "// gravity_scale_pct": "Compensation strength as % of nominal velocity (only used when gravity_dir != 0). 35 = boost UP-direction by 35%, dampen DOWN-direction by 35%. Empirically tuned for Elbow/Forearm slots; harmless when their dir is 0.",
    "gravity_scale_pct":  [ 0, 35, 35, 0, 0, 0 ]
}
RUNTIMEEOF
    return $?
}
