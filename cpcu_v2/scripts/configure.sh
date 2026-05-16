#!/usr/bin/env bash
# =============================================================================
#  scripts/configure.sh — Internal helper for `./launch.sh configure`
# =============================================================================
#  Author : bugrASl
#  Date   : April 2026
#  Version: v2.7
#
#  ───────────────────────────────────────────────────────────────────────────
#  THIS IS A HELPER SCRIPT — invoked only by ./launch.sh configure.
#  Users should never invoke this directly. The user-facing API is:
#
#      ./launch.sh configure              (interactive)
#      ./launch.sh configure --show       (list current values)
#      ./launch.sh configure --radio-timeout 1000   (set one)
#      ./launch.sh configure --reset      (restore defaults)
#  ───────────────────────────────────────────────────────────────────────────
#
#  WHY THIS EXISTS
#    Some constants in this project are SAFETY thresholds — radio timeout,
#    battery cutoffs, thermal limits, packet/IPC schema versions. Making
#    these runtime-tunable would mean a misclick could disable safety. So
#    they live as #defines in headers and require a rebuild to change.
#    This script is a documented sed-wrapper that:
#      1. Knows where each tunable lives.
#      2. Validates new values against sane ranges.
#      3. Reminds the launcher to rebuild after every edit.
#
#    For runtime-tunable knobs (servo limits, gesture velocities, deadband,
#    grip levels, per-servo bias) edit cpcu_v2/config/runtime.json directly,
#    or use the CPCU TUI's edit mode.
#
#  v2.7 changes:
#    - Moved from cpcu_v2/configure.sh to cpcu_v2/scripts/configure.sh.
#    - CPCU_ROOT path resolution climbs one directory (we live in
#      scripts/ now, not at the repo root).
#    - All "next steps" prose stripped — the launcher prints user-facing
#      messages from the rebuild-required exit code.
# =============================================================================

set -e

#------------------------------------------------------------------------------
# Locate the cpcu_v2/ root and the repo parent. Robust against being invoked
# from anywhere thanks to BASH_SOURCE.
#------------------------------------------------------------------------------
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CPCU_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"
REPO_ROOT="$( cd "${CPCU_ROOT}/.." && pwd )"
BSAU_ROOT="${REPO_ROOT}/bsau_v2"

# Sanity check: cpcu_v2/include/cpcu_safety.h must exist where we expect it.
if [ ! -f "${CPCU_ROOT}/include/cpcu_safety.h" ]; then
    echo "[configure.sh] ERROR: can't locate cpcu_v2/include/cpcu_safety.h"
    echo "                  expected layout: <repo>/cpcu_v2/scripts/configure.sh"
    exit 1
fi

# Color helpers
if [ -t 1 ]; then
    R='\033[31m'; G='\033[32m'; Y='\033[33m'; B='\033[36m'; N='\033[0m'
else
    R=''; G=''; Y=''; B=''; N=''
fi

#------------------------------------------------------------------------------
# Tunable registry — one entry per editable #define
#------------------------------------------------------------------------------
# Format: name|file|define|default|min|max|description
#   - name        : flag name (without --)
#   - file        : path relative to repo root
#   - define      : the #define identifier as it appears in the header
#   - default     : the original value (used by --reset and --diff)
#   - min,max     : validation bounds (numeric); use "-" to skip
#   - description : human-readable one-line summary
#------------------------------------------------------------------------------
TUNABLES=(
    # CPCU safety thresholds
    "radio-timeout|cpcu_v2/include/cpcu_safety.h|SAFETY_RADIO_TIMEOUT_MS|750|150|10000|Silence (ms) before RUNNING -> DEGRADED"
    "radio-safe|cpcu_v2/include/cpcu_safety.h|SAFETY_RADIO_SAFE_MS|1500|500|30000|DEGRADED duration (ms) before SAFE"
    "boot-grace|cpcu_v2/include/cpcu_safety.h|SAFETY_RADIO_BOOT_GRACE_MS|5000|1000|60000|Cold-start grace (ms) before radio fault arms"
    "vbat-low|cpcu_v2/include/cpcu_safety.h|SAFETY_VBAT_LOW_V|3.0f|2.5|4.0|Battery LOW threshold (V), float (suffix f)"
    "vbat-crit|cpcu_v2/include/cpcu_safety.h|SAFETY_VBAT_CRITICAL_V|2.7f|2.0|3.5|Battery CRITICAL threshold (V), float (suffix f)"
    "thermal-warn|cpcu_v2/include/cpcu_safety.h|SAFETY_THERMAL_WARN_C|75|50|85|Thermal WARN (deg C)"
    "thermal-crit|cpcu_v2/include/cpcu_safety.h|SAFETY_THERMAL_CRITICAL_C|82|60|90|Thermal CRITICAL (deg C)"
    "i2c-max|cpcu_v2/include/cpcu_safety.h|SAFETY_I2C_MAX_ERRORS|5|1|100|Consecutive I2C failures before SAFE"
    "ring-overflow|cpcu_v2/include/cpcu_safety.h|SAFETY_RING_OVERFLOW_LIMIT|100|10|10000|Ring overflows (delta) before fault"

    # BSAU radio (channel only — BSAU_MODE selection is multi-line and
    # not amenable to simple #define editing; edit bsau_config.h by hand
    # for now and rebuild via STM32CubeIDE)
    "nrf-channel|bsau_v2/Core/Inc/bsau_app.h|NRF_CHANNEL|76|0|125|NRF24L01 channel (0-125)"
)

#------------------------------------------------------------------------------
# Helpers
#------------------------------------------------------------------------------

print_help() {
    cat <<'EOF'
configure.sh — edit compile-time #defines (safety/profile thresholds).

USAGE
  ./configure.sh                       interactive walkthrough
  ./configure.sh --show                show all current values
  ./configure.sh --diff                show values that differ from defaults
  ./configure.sh --reset               reset all to defaults
  ./configure.sh --reset --runtime     also regenerate cpcu_v2/config/runtime.json
  ./configure.sh --<name>              show one current value
  ./configure.sh --<name> <value>      set one value
  ./configure.sh --<name1> <v1> --<name2> <v2> ...
                                       set multiple values

SCOPE PREFIXES (optional)
  ./configure.sh --bsau --mode ...     limit interactive to BSAU tunables
  ./configure.sh --cpcu --vbat-low ... limit interactive to CPCU tunables

NOTES
  - Every change is followed by a "rebuild required" reminder. You must
    re-run cmake/make for CPCU and re-flash for BSAU before changes take
    effect.
  - For RUNTIME-tunable knobs (servo limits, smoother params, gesture
    velocities, etc.) edit cpcu_v2/config/runtime.json or use the CPCU TUI
    edit mode (v2.3.4+).
  - See cpcu_v2/docs/RUNTIME_CONFIG.md for the full runtime/compile split.
EOF
}

# Look up a tunable entry by flag name. Echoes the entry line.
lookup() {
    local name="$1"
    for entry in "${TUNABLES[@]}"; do
        local flagname="${entry%%|*}"
        if [ "${flagname}" = "${name}" ]; then
            echo "${entry}"
            return 0
        fi
    done
    return 1
}

# Read the current value of a #define from a header file.
# Captures the first whitespace-delimited token after the define name.
# This deliberately ignores trailing /* multi-line */ comments and any
# // line comments — only the bare value is returned.
read_value() {
    local file="$1" define="$2"
    local full="${REPO_ROOT}/${file}"
    if [ ! -f "${full}" ]; then
        echo "<file missing>"
        return 1
    fi
    awk -v def="${define}" '
        /^[[:space:]]*#define[[:space:]]/ && $2 == def {
            # $3 is the first whitespace-delimited token after the name.
            # Strip trailing block-comment glue defensively.
            v = $3
            sub(/\/\*.*$/, "", v)
            sub(/\/\/.*$/, "", v)
            print v
            exit
        }
    ' "${full}"
}

# Validate a candidate value against min/max. Pass "-" to skip.
# Numbers may have an optional 'f' suffix (C float literal); strip it
# for the numeric comparison but preserve in storage.
validate() {
    local name="$1" value="$2" min="$3" max="$4"
    if [ "${min}" = "-" ] || [ "${max}" = "-" ]; then
        return 0     # non-numeric tunable, skip range check
    fi
    local stripped="${value%f}"   # remove trailing f if present
    # Use awk for robust numeric (int+float) comparison.
    if ! awk -v v="${stripped}" 'BEGIN{ if (v+0 != v) exit 1 }'; then
        echo -e "${R}[configure]${N} ${name}: '${value}' is not numeric (range ${min}..${max})"
        return 1
    fi
    if ! awk -v v="${stripped}" -v lo="${min}" -v hi="${max}" \
            'BEGIN{ exit !(v+0 >= lo+0 && v+0 <= hi+0) }'; then
        echo -e "${R}[configure]${N} ${name}: ${value} out of range ${min}..${max}"
        return 1
    fi
    return 0
}

# Edit a single #define in a file. Atomic via tmpfile + mv.
write_value() {
    local file="$1" define="$2" newval="$3"
    local full="${REPO_ROOT}/${file}"
    local tmp="${full}.cfgtmp.$$"
    if [ ! -f "${full}" ]; then
        echo -e "${R}[configure]${N} file not found: ${full}"
        return 1
    fi
    # Replace the value portion of the matching #define line. Preserve any
    # trailing /* comment */ — that's where the rationale lives.
    awk -v def="${define}" -v val="${newval}" '
        /^[[:space:]]*#define[[:space:]]+/ && $2 == def {
            # Reconstruct: indent + "#define" + name + spaces + new value
            # + (trailing comment if any)
            split($0, parts, /\/\*/)
            comment = (parts[2] != "" ? "/*" parts[2] : "")
            printf("#define %-32s %s%s%s\n", def, val,
                   (comment != "" ? "    " : ""), comment)
            next
        }
        { print }
    ' "${full}" > "${tmp}"
    mv "${tmp}" "${full}"
}

# Apply a single named change.
do_set() {
    local name="$1" newval="$2"
    local entry; entry="$(lookup "${name}")" || {
        echo -e "${R}[configure]${N} unknown tunable: ${name}"
        echo "Use ./configure.sh --show for the list."
        return 1
    }
    IFS='|' read -r flagname file define defval minv maxv descr <<< "${entry}"
    validate "${flagname}" "${newval}" "${minv}" "${maxv}" || return 1
    local old; old="$(read_value "${file}" "${define}")"
    if [ "${old}" = "${newval}" ]; then
        echo -e "${B}[configure]${N} ${flagname}: already = ${newval}, no change"
        return 0
    fi
    write_value "${file}" "${define}" "${newval}"
    echo -e "${G}[configure]${N} ${flagname}: ${old} -> ${newval}"
    echo -e "    ${descr}"
    echo -e "    File: ${file}"
    NEED_REBUILD=1
}

# Show one current value.
do_show_one() {
    local name="$1"
    local entry; entry="$(lookup "${name}")" || {
        echo -e "${R}[configure]${N} unknown tunable: ${name}"
        return 1
    }
    IFS='|' read -r flagname file define defval minv maxv descr <<< "${entry}"
    local val; val="$(read_value "${file}" "${define}")"
    printf "%-20s = %-20s  (default %s, range %s..%s)\n" \
        "${define}" "${val}" "${defval}" "${minv}" "${maxv}"
    echo "    ${descr}"
    echo "    ${file}"
}

# Show every current value.
do_show_all() {
    local scope="$1"
    echo -e "${B}=== Compile-time tunables ===${N}"
    echo "(For runtime knobs see cpcu_v2/config/runtime.json + RUNTIME_CONFIG.md)"
    echo
    for entry in "${TUNABLES[@]}"; do
        IFS='|' read -r flagname file define defval minv maxv descr <<< "${entry}"
        case "${scope}" in
            bsau) [[ "${file}" == bsau_v2/* ]] || continue ;;
            cpcu) [[ "${file}" == cpcu_v2/* ]] || continue ;;
        esac
        local val; val="$(read_value "${file}" "${define}")"
        local marker=""
        [ "${val}" != "${defval}" ] && marker=" ${Y}[modified]${N}"
        printf "  %-30s %-12s ${B}default=%-12s${N}%s\n" \
            "${define}" "${val}" "${defval}" "$(echo -e ${marker})"
    done
}

# Show only modified values.
do_diff() {
    echo -e "${B}=== Modifications from defaults ===${N}"
    local count=0
    for entry in "${TUNABLES[@]}"; do
        IFS='|' read -r flagname file define defval minv maxv descr <<< "${entry}"
        local val; val="$(read_value "${file}" "${define}")"
        if [ "${val}" != "${defval}" ]; then
            printf "  %-30s ${G}%-12s${N} (default ${B}%s${N})\n" \
                "${define}" "${val}" "${defval}"
            count=$((count + 1))
        fi
    done
    if [ "${count}" -eq 0 ]; then
        echo "  (no compile-time tunables modified)"
    fi
}

# Reset all to defaults.
do_reset() {
    local also_runtime="$1"
    echo -e "${Y}[configure]${N} Resetting compile-time tunables to defaults..."
    for entry in "${TUNABLES[@]}"; do
        IFS='|' read -r flagname file define defval minv maxv descr <<< "${entry}"
        local val; val="$(read_value "${file}" "${define}")"
        if [ "${val}" != "${defval}" ]; then
            write_value "${file}" "${define}" "${defval}"
            echo "  ${define}: ${val} -> ${defval}"
            NEED_REBUILD=1
        fi
    done
    echo -e "${G}[configure]${N} Done."

    if [ "${also_runtime}" = "1" ]; then
        local runtime="${CPCU_ROOT}/config/runtime.json"
        local example="${CPCU_ROOT}/config/runtime.json.example"

        # Backup any existing runtime.json before clobbering, so the
        # operator can recover if --reset was the wrong call.
        if [ -f "${runtime}" ]; then
            cp -p "${runtime}" "${runtime}.bak"
            echo -e "${Y}[configure]${N} Backed up existing runtime.json -> runtime.json.bak"
        fi

        if [ -f "${example}" ]; then
            echo -e "${Y}[configure]${N} Regenerating runtime.json from runtime.json.example..."
            cp "${example}" "${runtime}"
        else
            # v2.7: source the shared emitter and write known-good defaults.
            # No more "no example found, can't help" dead-end.
            local emit_helper="${SCRIPT_DIR}/_default_runtime_json.sh"
            if [ -f "${emit_helper}" ]; then
                . "${emit_helper}"
                echo -e "${Y}[configure]${N} Regenerating runtime.json with embedded defaults..."
                emit_default_runtime_json "${runtime}"
            else
                echo -e "${R}[configure]${N} ERROR: ${emit_helper} missing; can't regenerate runtime.json."
                echo "  Restore from git: git checkout HEAD -- config/runtime.json"
                return 1
            fi
        fi
        echo -e "${G}[configure]${N} Wrote ${runtime}"
        echo "  Reload kernel to apply: kill -HUP \$(pgrep cpcu_kernel)"
    fi
}

# Interactive walkthrough — prompts for every tunable in turn.
do_interactive() {
    local scope="$1"
    echo -e "${B}=== configure.sh interactive ===${N}"
    echo "Press ENTER to keep the current value. Enter 'q' to abort."
    echo
    for entry in "${TUNABLES[@]}"; do
        IFS='|' read -r flagname file define defval minv maxv descr <<< "${entry}"
        case "${scope}" in
            bsau) [[ "${file}" == bsau_v2/* ]] || continue ;;
            cpcu) [[ "${file}" == cpcu_v2/* ]] || continue ;;
        esac
        local val; val="$(read_value "${file}" "${define}")"
        echo -e "${B}${define}${N}  (${descr})"
        echo "  range  : ${minv}..${maxv}"
        echo "  default: ${defval}"
        echo -e "  current: ${G}${val}${N}"
        read -r -p "  new value (ENTER=keep, q=quit): " ans
        case "${ans}" in
            "")     echo "  (kept)" ;;
            q|Q)    echo "Aborted."; exit 0 ;;
            *)      do_set "${flagname}" "${ans}" || true ;;
        esac
        echo
    done
}

#------------------------------------------------------------------------------
# Main argument parser
#------------------------------------------------------------------------------
NEED_REBUILD=0
ALSO_RUNTIME=0
SCOPE=""

if [ $# -eq 0 ]; then
    do_interactive ""
    exit_with_rebuild_hint=1
else
    # First pass — extract scope/help/show/diff/reset
    args=("$@")
    i=0
    while [ ${i} -lt ${#args[@]} ]; do
        case "${args[${i}]}" in
            -h|--help)
                print_help; exit 0 ;;
            --show)
                do_show_all "${SCOPE}"; exit 0 ;;
            --diff)
                do_diff; exit 0 ;;
            --reset)
                # Check next-but-one for --runtime
                ;;
            --runtime)
                ALSO_RUNTIME=1 ;;
            --bsau)
                SCOPE="bsau" ;;
            --cpcu)
                SCOPE="cpcu" ;;
        esac
        i=$((i + 1))
    done
    # Second pass — execute --reset / sets
    i=0
    while [ ${i} -lt ${#args[@]} ]; do
        a="${args[${i}]}"
        case "${a}" in
            --reset)
                do_reset "${ALSO_RUNTIME}"; exit_with_rebuild_hint=1 ;;
            --bsau|--cpcu|--runtime)
                ;;
            --*)
                name="${a#--}"
                # Look ahead for value
                j=$((i + 1))
                if [ ${j} -lt ${#args[@]} ] && [[ ! "${args[${j}]}" =~ ^-- ]]; then
                    do_set "${name}" "${args[${j}]}"
                    i=$((i + 1))
                else
                    do_show_one "${name}"
                fi
                ;;
            *)
                echo -e "${R}[configure]${N} unexpected positional arg: ${a}"
                exit 1 ;;
        esac
        i=$((i + 1))
    done
fi

#------------------------------------------------------------------------------
# Rebuild signaling — exit 11 if a rebuild is needed. The caller (launch.sh)
# prints the user-facing prompt; this script just signals.
#------------------------------------------------------------------------------
if [ "${NEED_REBUILD}" = "1" ]; then
    exit 11
fi
exit 0
