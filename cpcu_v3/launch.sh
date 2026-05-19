#!/bin/bash
##
##  launch.sh — InfiniTech CPCU Unified User API
##  Author:  bugrASl
##  Date:    May 2026
##  Version: v3.0
##
##  ════════════════════════════════════════════════════════════════════
##  THIS IS THE ONLY SCRIPT YOU NEED TO INVOKE. EVERY SYSTEM OPERATION
##  GOES THROUGH `./launch.sh <command>`.
##  ════════════════════════════════════════════════════════════════════
##
##  Run `./launch.sh help` for the full command reference, or
##  `./launch.sh help <command>` for command-specific detail.
##
##  Quick map:
##
##    SETUP / BUILD (once-per-Pi, then once-per-source-change):
##      ./launch.sh setup                  Configure the Pi (one-time)
##      ./launch.sh build                  Compile + install the project
##      ./launch.sh check                  Verify everything is ready
##
##    TESTING (verify subsystems before running live):
##      ./launch.sh test-sw                Software-only tests (233 PASS)
##      ./launch.sh test-ipc               + IPC validation (kernel needed)
##      ./launch.sh test-hw                + Pi hardware probes
##      ./launch.sh test-pca               Interactive servo motion check
##      ./launch.sh test-signal            Live signal-integrity TUI (needs BSAU)
##      ./launch.sh test-signal-demo       Same TUI with synthetic data
##      ./launch.sh test-safety-demo       Fault-injection demo (no hardware)
##      ./launch.sh test-system            System requirements verification (live)
##
##
##    RUNTIME / COMPILE-TIME TUNING:
##      ./launch.sh configure              Interactive (compile-time)
##      ./launch.sh configure --show       Show all values
##      ./launch.sh configure --diff       Show changes from defaults
##      ./launch.sh configure --reset      Restore defaults
##      ./launch.sh configure --<name> <value>   Set one knob
##
##    OPERATING THE LIVE SYSTEM:
##      ./launch.sh tui                    Interactive: kernel + TUI dashboard
##      ./launch.sh signal                 Interactive: kernel + signal_testbench
##      ./launch.sh collect                Interactive: kernel + TUI for dataset capture
##      ./launch.sh smoother               Servo motion profile exerciser (needs kernel)
##      ./launch.sh pca                    Direct PCA9685 calibration (no kernel)
##      ./launch.sh kernel                 Kernel only, foreground (for systemd)
##      ./launch.sh ws                     Web dashboard (browser-accessible)
##      ./launch.sh menu                   Interactive picker (default on TTY)
##      ./launch.sh attach                 Re-attach to a running session
##      ./launch.sh stop                   Stop the running session
##
##    SERVICES (start at boot):
##      ./launch.sh install-service        systemd unit for the kernel
##      ./launch.sh install-ws-service     systemd unit for the web dashboard
##
##    INTERNAL:
##      ./launch.sh grant-caps             Re-apply RT capabilities to binaries
##      ./launch.sh help [<cmd>]           Help text
##      ./launch.sh version                Show version
##
##  v2.7 architectural changes:
##      - launch.sh moved to project root (was scripts/launch.sh).
##      - scripts/ contains internal shell helpers only
##        (setup_pi.sh, configure.sh, run_tests.sh).
##      - python/ contains Python modules (cpcu_dsp.py, etc).
##      - launch.sh exposes ALL helper functionality. Users never invoke
##        the helpers directly.
##
##  v2.7.1 changes:
##      - Every tmux session now includes a SHELL window (window 1)
##        between KERNEL (0) and the tool window (2+). Ctrl-b 1 gives
##        you a bash prompt for ./launch.sh stop without detaching.
##      - cpcu_ws: fixed broadcast() field name (mgr->conns), added
##        --version flag, fixed mg_http_serve_opts struct.
##      - CMakeLists.txt: web/static/ now installed to /opt/cpcu/ws_static.
##      - cpcu_kernel.c: removed duplicate #define block.
##      - cpcu_io.c: fixed version banner (v2.2 → v2.3.7).
##

set -e

# ─── Self-locate ─────────────────────────────────────────────────────
# launch.sh lives at the project root. Find ourselves robustly so the
# script works regardless of cwd or symlinks.
LAUNCH_SCRIPT="$(readlink -f "${BASH_SOURCE[0]}")"
CPCU_ROOT="$(dirname "${LAUNCH_SCRIPT}")"
SCRIPTS_DIR="${CPCU_ROOT}/scripts"
PYTHON_DIR="${CPCU_ROOT}/python"

# ─── Install paths (must match CMakeLists.txt) ──────────────────────
CPCU_DIR="/opt/cpcu"
BIN_DIR="${CPCU_DIR}/bin"
PYTHON_INSTALL_DIR="${CPCU_DIR}/python"
SCRIPTS_INSTALL_DIR="${CPCU_DIR}/scripts"
MODEL_DIR="${CPCU_DIR}/models"
LOG_DIR="${CPCU_ROOT}/log"
DATASETS_DIR="${CPCU_ROOT}/datasets"
WS_INFO_FILE="/tmp/cpcu_ws_active.txt"

SESSION_NAME="cpcu"
LAUNCH_MODE=""          # set by each command for log labeling

# Cleanup-flag: 1 while we own a tmux session that the user hasn't yet
# attached to. Cleared after attach returns.
TMUX_OWNED=""
KERNEL_PID=""

# ─── ANSI colors ─────────────────────────────────────────────────────
if [ -t 1 ]; then
    C_RED="\033[31m";  C_GRN="\033[32m";  C_YEL="\033[33m"
    C_CYN="\033[36m";  C_BLD="\033[1m";   C_RST="\033[0m"
else
    C_RED=""; C_GRN=""; C_YEL=""; C_CYN=""; C_BLD=""; C_RST=""
fi

log()   { echo -e "${C_CYN}[LAUNCH]${C_RST} $*"; }
warn()  { echo -e "${C_YEL}[LAUNCH] WARN:${C_RST} $*"; }
err()   { echo -e "${C_RED}[LAUNCH] ERROR:${C_RST} $*" >&2; }
fatal() { err "$*"; exit 1; }
ok()    { echo -e "${C_GRN}[LAUNCH] OK:${C_RST} $*"; }

# Per-process log path: log/log_{process}_{mode}_{timestamp}.txt
make_log_path() {
    local proc_name="$1"
    mkdir -p "${LOG_DIR}" 2>/dev/null || true
    local ts
    ts=$(date +%Y%m%d_%H%M%S)
    echo "${LOG_DIR}/log_${proc_name}_${LAUNCH_MODE:-cli}_${ts}.txt"
}

# Write web dashboard connection info so TUI can display it
write_ws_info() {
    local ip="${1:-$(hostname -I 2>/dev/null | awk '{print $1}')}"
    local port="${2:-8765}"
    mkdir -p "$(dirname "${WS_INFO_FILE}")" 2>/dev/null || true
    cat > "${WS_INFO_FILE}" << WSINFO
# Web dashboard active — written by launch.sh, read by cpcu_tui
url=http://${ip}:${port}
ip=${ip}
port=${port}
started=$(date -Iseconds)
pid=$$
WSINFO
}

clear_ws_info() {
    rm -f "${WS_INFO_FILE}" 2>/dev/null || true
}

# Verify a helper script is runnable. If it exists but isn't executable
# (common right after a fresh clone — git doesn't always preserve the
# +x bit across some hosts), self-heal by chmod +x'ing it. This avoids
# the confusing "missing — incomplete source tree?" error when the file
# is actually present.
ensure_helper_executable() {
    local helper="$1"
    if [ -x "${helper}" ]; then
        return 0
    fi
    if [ -f "${helper}" ]; then
        warn "${helper} exists but isn't executable. Fixing with chmod +x..."
        if chmod +x "${helper}" 2>/dev/null; then
            ok "Fixed ${helper}. Continuing."
            return 0
        fi
        fatal "Couldn't chmod +x ${helper}. Try: chmod +x scripts/*.sh"
    fi
    fatal "Missing ${helper} — is your source tree complete?"
}

# Prompt-yes-or-no (default depends on second arg)
prompt_yn() {
    local msg="$1" default="${2:-n}" reply
    if [ "${default}" = "y" ]; then
        read -rp "  ${msg} [Y/n]: " reply
        reply="${reply:-y}"
    else
        read -rp "  ${msg} [y/N]: " reply
        reply="${reply:-n}"
    fi
    case "${reply}" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}


# ════════════════════════════════════════════════════════════════════════
#  PRE-FLIGHT CHECKS
# ════════════════════════════════════════════════════════════════════════

preflight_kernel() {
    log "Pre-flight..."

    mkdir -p "${LOG_DIR}" 2>/dev/null || true
    chmod 755 "${LOG_DIR}" 2>/dev/null || true

    [ -x "${BIN_DIR}/cpcu_kernel" ] \
        || fatal "Missing ${BIN_DIR}/cpcu_kernel — run './launch.sh build'"
    [ -x "${BIN_DIR}/cpcu_io" ] \
        || fatal "Missing ${BIN_DIR}/cpcu_io — run './launch.sh build'"

    if ! python3 -c "import numpy, scipy, joblib" 2>/dev/null; then
        warn "Python deps missing — DSP will run in feature-only mode."
        warn "Install with: ./launch.sh setup"
    fi

    [ -f "${PYTHON_INSTALL_DIR}/cpcu_dsp.py" ] \
        || warn "${PYTHON_INSTALL_DIR}/cpcu_dsp.py missing — re-run './launch.sh build'"

    if ! ls "${MODEL_DIR}"/*.pkl >/dev/null 2>&1; then
        warn "No ML model (.pkl) in ${MODEL_DIR} — DSP will run feature-only"
        warn "  Place your trained model files in that directory and"
        warn "  send 'kill -HUP \$(pgrep cpcu_kernel)' to reload."
    fi

    local isolated
    isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")
    if [ -z "${isolated}" ]; then
        warn "No CPU cores isolated — real-time guarantees void."
        warn "Run './launch.sh setup' and reboot to fix."
    else
        log "Isolated cores: ${isolated}"
    fi

    [ -e /dev/spidev0.0 ] || warn "/dev/spidev0.0 missing — run './launch.sh setup'"
    [ -e /dev/i2c-1 ]     || warn "/dev/i2c-1 missing — run './launch.sh setup'"
}

preflight_pca()    { resolve_bin pca_testbench;    }
preflight_nrf()    { resolve_bin nrf_testbench;    }
preflight_tui()    { resolve_bin cpcu_tui;          }
preflight_signal() { resolve_bin signal_testbench;  }

# Locate a binary, preferring the installed copy under /opt/cpcu/bin/
# but falling back to ${CPCU_ROOT}/build/ for developers who haven't
# (yet) run `cmake --install`. Sets the global ${RESOLVED_BIN} on
# success; fatals on failure.
#
# Why both paths: the v2.7 layout installs testbenches to
# /opt/cpcu/bin/ via CMakeLists.txt's install() rules. Older trees
# (or trees that built without installing) leave them in build/
# only — running ./launch.sh signal there should "just work" rather
# than nag the user to re-install.
resolve_bin() {
    local name="$1"
    local installed="${BIN_DIR}/${name}"
    local local_build="${CPCU_ROOT}/build/${name}"
    if [ -x "${installed}" ]; then
        RESOLVED_BIN="${installed}"
    elif [ -x "${local_build}" ]; then
        RESOLVED_BIN="${local_build}"
        warn "Using build/${name} (not installed). Run './launch.sh build' to install."
    else
        fatal "${name} not found in ${BIN_DIR} or ${CPCU_ROOT}/build — run './launch.sh build'"
    fi
}


# ════════════════════════════════════════════════════════════════════════
#  TMUX HELPERS
# ════════════════════════════════════════════════════════════════════════

require_tmux() { command -v tmux >/dev/null 2>&1; }

tmux_kill_existing() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Killing existing tmux session '$SESSION_NAME'..."
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        sleep 0.5
    fi
}

tmux_create_with_kernel() {
    tmux_kill_existing

    # Create per-process log files labeled with the launch mode
    local kernel_log
    kernel_log=$(make_log_path "kernel")
    log "Creating tmux session '$SESSION_NAME', spawning KERNEL..."
    log "  Kernel log: ${kernel_log}"

    # Get terminal dimensions from stty (most reliable source).
    # Default to 80x24 if not running from a real terminal.
    local _cols _rows
    _cols=$(stty size 2>/dev/null | awk '{print $2}')
    _rows=$(stty size 2>/dev/null | awk '{print $1}')
    _cols=${_cols:-80}
    _rows=${_rows:-24}

    tmux new-session -d -s "$SESSION_NAME" -n "KERNEL" \
        -x "$_cols" -y "$_rows" \
        "bash -c 'cd ${CPCU_ROOT} && exec taskset -c 0 ${BIN_DIR}/cpcu_kernel --log 2>&1 | tee -a ${kernel_log}'"

    # Wait briefly for the new session to be reachable. tmux's set-option
    # calls below otherwise race with the daemon and emit harmless but
    # noisy "no server running" stderr warnings.
    for i in 1 2 3 4 5; do
        if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
            break
        fi
        sleep 0.1
    done

    tmux set-option -t "$SESSION_NAME" remain-on-exit on        >/dev/null 2>&1
    tmux set-option -t "$SESSION_NAME" status on                >/dev/null 2>&1
    tmux set-option -t "$SESSION_NAME" status-justify centre    >/dev/null 2>&1
    tmux set-option -t "$SESSION_NAME" status-left  " #S "      >/dev/null 2>&1
    tmux set-option -t "$SESSION_NAME" status-right " %H:%M "   >/dev/null 2>&1

    TMUX_OWNED=1

    # v2.7.1: add a spare SHELL window so the user can type
    # `./launch.sh stop` (or any other command) without detaching.
    # Placed after KERNEL but before the caller adds its own window,
    # so window order is:  0=KERNEL  1=SHELL  2=<TUI|SIGNAL|WS|...>
    tmux_add_window "SHELL" "bash --login"

    log "Waiting for shared memory..."
    for i in $(seq 1 30); do
        if [ -e /dev/shm/cpcu_ipc ]; then
            log "Shared memory ready after $((i*500))ms"
            sleep 1
            return 0
        fi
        sleep 0.5
    done

    err "Kernel didn't bring up /dev/shm/cpcu_ipc within 15s"
    err "Inspect what happened: ./launch.sh attach"
    return 1
}

tmux_add_window() {
    tmux new-window -t "$SESSION_NAME" -n "$1" "$2" 2>/dev/null
}

tmux_attach_at() {
    local target_window="$1"
    tmux select-window -t "${SESSION_NAME}:${target_window}" 2>/dev/null

    # If the session is gone (kernel crashed and ate it), bail with
    # a clear message instead of silently trying to attach to nothing.
    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        err "tmux session '${SESSION_NAME}' is no longer running."
        err "The kernel probably crashed at startup. Check what went wrong:"
        err "  /opt/cpcu/bin/cpcu_kernel       # run it directly to see the error"
        err "  tail -50 ${LOG_DIR}/cpcu.log    # check the log"
        return 1
    fi

    echo
    log "${C_BLD}Session ready.${C_RST} Attaching to ${C_GRN}${target_window}${C_RST}..."
    log "  ${C_GRN}Ctrl-b d${C_RST} to detach  |  ${C_GRN}./launch.sh stop${C_RST} to stop"
    sleep 0.5

    if [ -n "${TMUX:-}" ]; then
        # already inside tmux — switch client instead of attach
        tmux switch-client -t "${SESSION_NAME}:${target_window}" 2>/dev/null \
            || tmux attach-session -t "$SESSION_NAME"
    else
        exec tmux attach-session -t "$SESSION_NAME"
    fi
    TMUX_OWNED=""

    echo
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Detached. Session ${C_BLD}${SESSION_NAME}${C_RST} keeps running."
        log "  Re-attach: ${C_BLD}./launch.sh attach${C_RST}"
        log "  Stop:      ${C_BLD}./launch.sh stop${C_RST}"
    else
        log "Session ended."
    fi
}


# ════════════════════════════════════════════════════════════════════════
#  FALLBACK (NO TMUX)
# ════════════════════════════════════════════════════════════════════════

kernel_start_background_fallback() {
    log "Starting cpcu_kernel in background (logs → ${LOG_DIR}/cpcu.log)..."
    cd "${BIN_DIR}"
    ( taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a "${LOG_DIR}/cpcu.log" ) &
    KERNEL_PID=$!

    log "Waiting for shared memory..."
    for i in $(seq 1 30); do
        [ -e /dev/shm/cpcu_ipc ] && { sleep 1; return 0; }
        sleep 0.5
    done
    err "Timed out waiting for /dev/shm/cpcu_ipc"
    return 1
}

kernel_stop_background_fallback() {
    [ -z "${KERNEL_PID}" ] && return
    kill -0 "${KERNEL_PID}" 2>/dev/null || return

    log "Stopping cpcu_kernel (pid=${KERNEL_PID})..."
    kill -TERM "-${KERNEL_PID}" 2>/dev/null || kill -TERM "${KERNEL_PID}" 2>/dev/null || true
    for i in $(seq 1 10); do
        kill -0 "${KERNEL_PID}" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 "${KERNEL_PID}" 2>/dev/null; then
        warn "Kernel didn't exit cleanly, sending SIGKILL"
        kill -KILL "${KERNEL_PID}" 2>/dev/null || true
    fi
    wait "${KERNEL_PID}" 2>/dev/null || true
    KERNEL_PID=""
}


# ════════════════════════════════════════════════════════════════════════
#  CLEANUP TRAP
# ════════════════════════════════════════════════════════════════════════

cleanup() {
    if [ -n "${TMUX_OWNED}" ] && command -v tmux >/dev/null 2>&1 && \
       tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        warn "Cleanup: tearing down tmux session $SESSION_NAME"
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null
    fi
    kernel_stop_background_fallback
}
trap cleanup EXIT INT TERM


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: setup
# ════════════════════════════════════════════════════════════════════════

cmd_setup() {
    # Self-heal: ensure launch.sh and all helpers are executable. This
    # lets the user invoke us once via `bash launch.sh setup` and never
    # need to type chmod themselves, even if their checkout came from a
    # source that strips the +x bit (zip download, FAT32, scp from
    # Windows, etc.).
    chmod +x "${LAUNCH_SCRIPT}" 2>/dev/null || true
    chmod +x "${SCRIPTS_DIR}"/*.sh 2>/dev/null || true
    ensure_helper_executable "${SCRIPTS_DIR}/setup_pi.sh"

    log "Running one-time Pi setup..."
    log "This will:"
    log "  - Install build tools and Python libraries (apt + pip)"
    log "  - Enable SPI and I²C in /boot/firmware/config.txt"
    log "  - Reserve cores 1-3 for real-time use (isolcpus)"
    log "  - Add you to the spi/i2c/gpio groups"
    log "  - Create /opt/cpcu/{bin,scripts,python,models} owned by you"
    echo
    log "You'll see one sudo password prompt."
    echo

    local rc=0
    "${SCRIPTS_DIR}/setup_pi.sh" "$@" || rc=$?

    case "${rc}" in
        0)
            ok "Setup complete. No reboot needed."
            log "Next step: ${C_BLD}./launch.sh build${C_RST}"
            ;;
        10)
            echo
            warn "════════════════════════════════════════════════════════════"
            warn "  REBOOT REQUIRED"
            warn ""
            warn "  Setup modified /boot/firmware/config.txt and/or"
            warn "  cmdline.txt. The Pi must reboot for those to take effect."
            warn "════════════════════════════════════════════════════════════"
            echo
            if prompt_yn "Reboot now?" n; then
                log "Rebooting in 3 seconds... (Ctrl-C to cancel)"
                sleep 3
                sudo reboot
            else
                warn "Reboot deferred. Run 'sudo reboot' before building."
            fi
            ;;
        *)
            fatal "setup_pi.sh exited with code ${rc}"
            ;;
    esac
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: build
# ════════════════════════════════════════════════════════════════════════

# Lazy cmake configure: only re-runs if CMakeCache.txt is missing or
# CMakeLists.txt is newer.
cmake_configure_if_needed() {
    if [ ! -f "${CPCU_ROOT}/build/CMakeCache.txt" ]; then
        log "First build — running cmake configure..."
        ( cd "${CPCU_ROOT}" && cmake -S . -B build ) || fatal "cmake configure failed"
    elif [ "${CPCU_ROOT}/CMakeLists.txt" -nt "${CPCU_ROOT}/build/CMakeCache.txt" ]; then
        log "CMakeLists.txt is newer than cache — re-running cmake configure..."
        ( cd "${CPCU_ROOT}" && cmake -S . -B build ) || fatal "cmake configure failed"
    fi
}

cmd_build() {
    [ -f "${CPCU_ROOT}/CMakeLists.txt" ] \
        || fatal "No CMakeLists.txt at ${CPCU_ROOT} — is this a CPCU source tree?"

    if [ ! -d /opt/cpcu ]; then
        fatal "/opt/cpcu doesn't exist. Run './launch.sh setup' first."
    fi

    # --clean forces a fresh build directory. Use this if you've
    # changed CMakeLists.txt in a way that requires regenerating the
    # install plan (added/removed targets), or if the cached config
    # has otherwise gone stale.
    if [ "${1:-}" = "--clean" ]; then
        log "Removing build/ directory for a clean rebuild..."
        rm -rf "${CPCU_ROOT}/build"
    fi

    cmake_configure_if_needed

    log "Building..."
    ( cd "${CPCU_ROOT}" && cmake --build build -j4 ) || fatal "build failed"

    log "Installing to /opt/cpcu..."
    ( cd "${CPCU_ROOT}" && cmake --install build ) || fatal "install failed"

    # Sanity check: did the testbenches actually land in /opt/cpcu/bin?
    # If they built but didn't install, the user's CMakeLists.txt is
    # older than the cmake cache (preserved-mtime cp, or someone
    # changed the install rules without invalidating the cache).
    # Detect and offer the recovery command.
    local missing=()
    for tb in pca_testbench signal_testbench editor_testbench; do
        if [ -x "${CPCU_ROOT}/build/${tb}" ] && [ ! -x "${BIN_DIR}/${tb}" ]; then
            missing+=("${tb}")
        fi
    done
    if [ "${#missing[@]}" -gt 0 ]; then
        warn "Built but not installed: ${missing[*]}"
        warn "Your cmake build cache is stale. Recovering with a clean rebuild..."
        rm -rf "${CPCU_ROOT}/build"
        cmake_configure_if_needed
        ( cd "${CPCU_ROOT}" && cmake --build build -j4 ) || fatal "clean rebuild failed"
        ( cd "${CPCU_ROOT}" && cmake --install build ) || fatal "clean install failed"
    fi

    log "Re-applying real-time capabilities (you'll see one sudo prompt)..."
    cmd_grant_caps_internal

    ok "Build complete. Run '${C_BLD}./launch.sh check${C_RST}' to verify, or '${C_BLD}./launch.sh tui${C_RST}' to start."
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: check
# ════════════════════════════════════════════════════════════════════════

cmd_check() {
    log "Standalone pre-flight..."
    local fatal_count=0 warn_count=0

    [ -x "${BIN_DIR}/cpcu_kernel" ] \
        || { err "MISSING ${BIN_DIR}/cpcu_kernel — run './launch.sh build'"; fatal_count=$((fatal_count+1)); }
    [ -x "${BIN_DIR}/cpcu_io" ] \
        || { err "MISSING ${BIN_DIR}/cpcu_io — run './launch.sh build'"; fatal_count=$((fatal_count+1)); }
    [ -x "${BIN_DIR}/cpcu_tui" ] \
        || { warn "${BIN_DIR}/cpcu_tui not installed"; warn_count=$((warn_count+1)); }

    if [ -x "${BIN_DIR}/cpcu_kernel" ]; then
        if ! getcap "${BIN_DIR}/cpcu_kernel" 2>/dev/null | grep -q cap_sys_nice; then
            err "cpcu_kernel missing CAP_SYS_NICE — run './launch.sh grant-caps'"
            fatal_count=$((fatal_count+1))
        fi
    fi

    if ! python3 -c "import numpy, scipy, joblib" 2>/dev/null; then
        warn "Python deps missing — DSP will run feature-only"
        warn_count=$((warn_count+1))
    fi

    if ! ls "${MODEL_DIR}"/*.pkl >/dev/null 2>&1; then
        warn "ML model not in ${MODEL_DIR} — DSP will run feature-only"
        warn_count=$((warn_count+1))
    fi

    if [ ! -f "/opt/cpcu/config.json" ]; then
        err "No /opt/cpcu/config.json — symlink missing (re-run './launch.sh setup')"
        fatal_count=$((fatal_count+1))
    fi

    local isolated
    isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")
    if [ -z "${isolated}" ]; then
        err "No cores isolated — re-run './launch.sh setup' + reboot"
        fatal_count=$((fatal_count+1))
    fi

    [ -e /dev/spidev0.0 ] || { err "/dev/spidev0.0 missing — re-run './launch.sh setup'"; fatal_count=$((fatal_count+1)); }
    [ -e /dev/i2c-1 ]     || { err "/dev/i2c-1 missing — re-run './launch.sh setup'"; fatal_count=$((fatal_count+1)); }

    if command -v i2cdetect >/dev/null 2>&1; then
        if ! i2cdetect -y 1 2>/dev/null | grep -q " 40 "; then
            warn "PCA9685 not detected at I²C 0x40 — check wiring + power"
            warn_count=$((warn_count+1))
        fi
    fi

    if ! groups | grep -qE '\bspi\b' || ! groups | grep -qE '\bi2c\b'; then
        warn "Not in spi/i2c groups — log out and back in (or re-run './launch.sh setup')"
        warn_count=$((warn_count+1))
    fi

    echo
    if [ "${fatal_count}" -gt 0 ]; then
        err "${fatal_count} fatal issue(s), ${warn_count} warning(s) — system will NOT start"
        exit 1
    elif [ "${warn_count}" -gt 0 ]; then
        warn "${warn_count} warning(s) — system can start with degraded behavior"
        exit 0
    else
        ok "All checks passed — system is ready to launch"
        exit 0
    fi
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: configure
# ════════════════════════════════════════════════════════════════════════

cmd_configure() {
    ensure_helper_executable "${SCRIPTS_DIR}/configure.sh"

    local rc=0
    "${SCRIPTS_DIR}/configure.sh" "$@" || rc=$?

    case "${rc}" in
        0) : ;;  # nothing to do, no rebuild needed
        11)
            echo
            warn "════════════════════════════════════════════════════════════"
            warn "  REBUILD REQUIRED"
            warn ""
            warn "  You changed compile-time values. The new values won't"
            warn "  take effect until the binaries are rebuilt and installed."
            warn "════════════════════════════════════════════════════════════"
            echo
            if prompt_yn "Rebuild now?" y; then
                cmd_build
            else
                warn "Rebuild deferred. Run './launch.sh build' before launching."
            fi
            ;;
        *) fatal "configure.sh exited with code ${rc}" ;;
    esac
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: test*
# ════════════════════════════════════════════════════════════════════════

cmd_test_phase() {
    local phases="$1"
    ensure_helper_executable "${SCRIPTS_DIR}/run_tests.sh"
    "${SCRIPTS_DIR}/run_tests.sh" ${phases}
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: tui / signal / collect / pca / kernel
# ════════════════════════════════════════════════════════════════════════

run_kernel_only() {
    LAUNCH_MODE="kernel"
    preflight_kernel
    local kernel_log
    kernel_log=$(make_log_path "kernel")
    log "Mode: KERNEL (foreground; systemd path, no tmux)"
    log "  Log: ${kernel_log}"
    trap - EXIT INT TERM
    cd "${BIN_DIR}"
    exec taskset -c 0 ./cpcu_kernel --log 2>&1 | tee -a "${kernel_log}"
}

run_tui_tmux() {
    LAUNCH_MODE="tui"
    preflight_kernel
    preflight_tui
    local tui_bin="${RESOLVED_BIN}"
    local features="KERNEL + SHELL + TUI"
    [ "${WITH_WS:-0}" = "1" ] && features="${features} + WS"
    [ "${WITH_AUDIO:-0}" = "1" ] && features="${features} + AUDIO"
    [ "${WITH_UART:-0}" = "1" ] && features="${features} + UART"
    [ "${OPERATOR}" != "default" ] && features="${features} (operator: ${OPERATOR})"
    if [ "${WITH_WS:-0}" = "1" ]; then
        with_ws_preflight || fatal "WS preflight failed"
    fi
    log "Mode: ${features}"
    tmux_create_with_kernel || fatal "Couldn't bring up kernel"
    local tui_log
    tui_log=$(make_log_path "tui")
    tmux_add_window "TUI" "${tui_bin}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        local ws_log pi_ip
        ws_log=$(make_log_path "ws")
        pi_ip=$(hostname -I | awk '{print $1}')
        write_ws_info
        tmux_add_window "WS" "bash -c '$(ws_window_cmd) 2>&1 | tee -a ${ws_log}'"
        log "═══════════════════════════════════════════════════"
        log "Web dashboard:"
        log "  Same network:  http://${pi_ip}:8765"
        log "  Remote (SSH):  ssh -L 8765:localhost:8765 $(whoami)@${pi_ip}"
        log "                 then http://localhost:8765"
        log "═══════════════════════════════════════════════════"
    fi
    if [ "${WITH_AUDIO:-0}" = "1" ]; then
        local audio_log
        audio_log=$(make_log_path "audio")
        local py_dir="${PYTHON_INSTALL_DIR:-${CPCU_ROOT:-$(dirname "$0")}/python}"
        tmux_add_window "AUDIO" "bash -c 'CPCU_ROOT=${CPCU_ROOT:-$(dirname "$0")} python3 ${py_dir}/cpcu_audio_daemon.py 2>&1 | tee -a ${audio_log}'"
        log "Audio feedback: $(python3 -c "import json; print(json.load(open('${CPCU_ROOT:-$(dirname "$0")}/config/gestures.json')).get('audio_mode','off'))" 2>/dev/null || echo "?")"
    fi
    log "Logs: ${LOG_DIR}/"
    tmux_attach_at "TUI"
}

run_collect_tmux() {
    LAUNCH_MODE="collect"
    preflight_kernel
    preflight_tui
    local tui_bin="${RESOLVED_BIN}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        with_ws_preflight    || fatal "WS preflight failed"
        log "Mode: COLLECT + WS (tmux: KERNEL + SHELL + TUI + WS, capture-focused)"
    else
        log "Mode: COLLECT (tmux: KERNEL + SHELL + TUI, capture-focused)"
    fi
    tmux_create_with_kernel || fatal "Couldn't bring up kernel"
    local tui_log
    tui_log=$(make_log_path "tui")
    tmux_add_window "TUI" "${tui_bin}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        local ws_log
        ws_log=$(make_log_path "ws")
        write_ws_info
        tmux_add_window "WS" "bash -c '$(ws_window_cmd) 2>&1 | tee -a ${ws_log}'"
        log "Web dashboard at http://$(hostname -I | awk '{print $1}'):8765"
    fi

    echo
    log "${C_BLD}Capture workflow:${C_RST}"
    log "  1. In the TUI, press ${C_GRN}7${C_RST} to jump to the DATASET page"
    log "  2. ${C_GRN}←/→${C_RST} cycle labels (REST, H_OPN, A_BND<, ...)"
    log "  3. ${C_GRN}t${C_RST} toggle RAW ↔ FILTERED  (default FILTERED)"
    log "  4. ${C_GRN}s${C_RST} or SPACE: start, again to stop+save"
    log "  5. ${C_GRN}r${C_RST} cancel + delete partial capture"
    log "  6. ${C_GRN}q${C_RST} quit. Files land in ${BIN_DIR}/datasets/"
    echo

    tmux_attach_at "TUI"
}

run_signal_tmux() {
    LAUNCH_MODE="signal"
    preflight_kernel
    preflight_signal
    local sig_bin="${RESOLVED_BIN}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        with_ws_preflight    || fatal "WS preflight failed"
        log "Mode: SIGNAL + WS (tmux: KERNEL + SHELL + SIGNAL + WS)"
    else
        log "Mode: SIGNAL (tmux: KERNEL + SHELL + SIGNAL)"
    fi
    tmux_create_with_kernel || fatal "Couldn't bring up kernel"
    local sig_log
    sig_log=$(make_log_path "signal")
    tmux_add_window "SIGNAL" "${sig_bin}"
    if [ "${WITH_WS:-0}" = "1" ]; then
        local ws_log
        ws_log=$(make_log_path "ws")
        write_ws_info
        tmux_add_window "WS" "bash -c '$(ws_window_cmd) 2>&1 | tee -a ${ws_log}'"
        log "Web dashboard at http://$(hostname -I | awk '{print $1}'):8765"
    fi
    log "Logs: ${LOG_DIR}/"
    tmux_attach_at "SIGNAL"
}

# Helpers for --with-ws composition. Both invoked from the run_*_tmux
# functions above. Kept as separate functions so they're visible at the
# top of this section and easy to find.
ws_static_dir() {
    if [ -d /opt/cpcu/ws_static ]; then
        echo "/opt/cpcu/ws_static"
    elif [ -d "${CPCU_ROOT}/web/static" ]; then
        echo "${CPCU_ROOT}/web/static"
    else
        return 1
    fi
}

# Run before adding the WS window — abort early on missing binary or
# stub builds so we don't bring up the whole tmux session only to have
# the WS window die immediately.
with_ws_preflight() {
    [ -x "${BIN_DIR}/cpcu_ws" ] \
        || { err "${BIN_DIR}/cpcu_ws not found — run './launch.sh build'"; return 1; }
    if cpcu_ws_is_stub; then
        err "cpcu_ws is a STUB build — run './launch.sh vendor' first."
        return 1
    fi
    ws_static_dir >/dev/null \
        || { err "Web static dir not found (looked at /opt/cpcu/ws_static and ${CPCU_ROOT}/web/static)"; return 1; }
    return 0
}

ws_window_cmd() {
    local sd
    sd="$(ws_static_dir)"
    echo "${BIN_DIR}/cpcu_ws --static ${sd}"
}

# Demo variant: signal_testbench --demo. Generates synthetic 100 Hz
# sines on all 8 channels internally — no kernel, no /dev/shm/cpcu_ipc,
# no BSAU needed. Useful for screenshots, verifying TUI rendering,
# and sanity-checking that the testbench builds correctly.
run_signal_demo() {
    preflight_signal
    local sig_bin="${RESOLVED_BIN}"
    log "Mode: SIGNAL DEMO (synthetic data; no kernel, no shared memory)"
    log "  Inside the TUI:  w cycle wave types  [/] change frequency  q quit"
    sleep 0.5
    trap - EXIT INT TERM
    cd "$(dirname "${sig_bin}")"
    exec "${sig_bin}" --demo
}

run_pca() {
    preflight_pca
    local pca_bin="${RESOLVED_BIN}"
    log "Mode: PCA (no kernel; direct I²C servo calibration)"
    log "Controls: arrows / m / M / n / 0 / A / q — press '?' inside for help"
    sleep 0.5
    trap - EXIT INT TERM
    local cfg_arg=""
    if [ -r "${CPCU_ROOT}/config/runtime.json" ]; then
        cfg_arg="--config ${CPCU_ROOT}/config/runtime.json"
    elif [ -r "/opt/cpcu/config.json" ]; then
        cfg_arg="--config /opt/cpcu/config.json"
    else
        warn "No runtime.json found — pca_testbench will use compile-time defaults"
    fi
    cd "$(dirname "${pca_bin}")"
    "${pca_bin}" ${cfg_arg}
    local rc=$?

    # sync servo limits from runtime.json → gestures.json
    if [ -f "${CPCU_ROOT}/config/runtime.json" ] && [ -f "${GS}" ]; then
        log "Syncing servo limits from runtime.json → gestures.json..."
        python3 << PYEOF
import json
with open("${CPCU_ROOT}/config/runtime.json") as f: rt = json.load(f)
with open("${GS}") as f: gs = json.load(f)
mn = rt.get("servo_min_us", [])
mx = rt.get("servo_max_us", [])
bias = rt.get("servo_bias_us", [])
# sort servo_channels by pca_ch to match array indices
servos = sorted(gs.get("servo_channels", {}).items(), key=lambda x: x[1].get("pca_ch", 0))
changed = 0
for i, (name, sd) in enumerate(servos):
    if i < len(mn) and sd.get("min_us") != mn[i]:
        sd["min_us"] = mn[i]; changed += 1
    if i < len(mx) and sd.get("max_us") != mx[i]:
        sd["max_us"] = mx[i]; changed += 1
if changed:
    with open("${GS}", "w") as f: json.dump(gs, f, indent=4)
    print(f"  \033[32m✓\033[0m Synced {changed} values to gestures.json")
else:
    print("  No changes to sync.")
PYEOF
    fi
    return $rc
}

run_nrf() {
    preflight_nrf
    local nrf_bin="${RESOLVED_BIN}"
    log "Mode: NRF (self-test + optional packet reception)"
    trap - EXIT INT TERM
    exec "${nrf_bin}" "$@"
}

run_smoother() {
    # Resolve the Python script — prefer installed, fall back to repo
    local script=""
    if [ -f "${PYTHON_INSTALL_DIR}/smoother_scenario.py" ]; then
        script="${PYTHON_INSTALL_DIR}/smoother_scenario.py"
    elif [ -f "${PYTHON_DIR}/smoother_scenario.py" ]; then
        script="${PYTHON_DIR}/smoother_scenario.py"
    else
        fatal "smoother_scenario.py not found in ${PYTHON_INSTALL_DIR} or ${PYTHON_DIR}"
    fi

    # Require a running kernel (motor commands go through IPC)
    if [ ! -e /dev/shm/cpcu_ipc ]; then
        fatal "cpcu_kernel is not running. Start with: ./launch.sh tui (in another terminal)"
    fi

    log "Mode: SMOOTHER (servo motion profile exerciser)"
    exec python3 "${script}" "$@"
}

# ───── Fallbacks (no tmux installed) ─────

run_tui_fallback() {
    preflight_kernel; preflight_tui
    local tui_bin="${RESOLVED_BIN}"
    log "Mode: TUI (background-mode fallback — kernel logs may overlap UI)"
    kernel_start_background_fallback || fatal "Kernel failed"
    sleep 1
    cd "$(dirname "${tui_bin}")" && "${tui_bin}"
    log "cpcu_tui exited"
}

run_collect_fallback() {
    preflight_kernel; preflight_tui
    local tui_bin="${RESOLVED_BIN}"
    log "Mode: COLLECT (background-mode fallback)"
    kernel_start_background_fallback || fatal "Kernel failed"
    sleep 1
    log "Press 7 inside the TUI to capture."
    cd "$(dirname "${tui_bin}")" && "${tui_bin}"
}

run_signal_fallback() {
    preflight_kernel; preflight_signal
    local sig_bin="${RESOLVED_BIN}"
    log "Mode: SIGNAL (background-mode fallback)"
    kernel_start_background_fallback || fatal "Kernel failed"
    sleep 1
    cd "$(dirname "${sig_bin}")" && "${sig_bin}"
}

run_tui()    { if require_tmux; then run_tui_tmux;    else warn "tmux not installed — install via './launch.sh setup'"; run_tui_fallback;    fi; }
run_collect(){ if require_tmux; then run_collect_tmux; else warn "tmux not installed — install via './launch.sh setup'"; run_collect_fallback; fi; }
run_signal() { if require_tmux; then run_signal_tmux; else warn "tmux not installed — install via './launch.sh setup'"; run_signal_fallback; fi; }


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: vendor (fetch third-party deps, currently just Mongoose)
# ════════════════════════════════════════════════════════════════════════

cmd_vendor() {
    local fetcher="${CPCU_ROOT}/web/vendor/fetch.sh"
    local mongoose_c="${CPCU_ROOT}/web/vendor/mongoose.c"
    local mongoose_h="${CPCU_ROOT}/web/vendor/mongoose.h"

    if [ ! -f "${fetcher}" ]; then
        fatal "${fetcher} not found — is this a CPCU source tree?"
    fi

    # Self-heal +x bit. Files freshly written by web upload, scp, or
    # tar without -p commonly lose the executable bit.
    if [ ! -x "${fetcher}" ]; then
        warn "${fetcher} is missing the executable bit — fixing..."
        chmod +x "${fetcher}" || fatal "couldn't chmod +x ${fetcher}"
    fi

    # Skip the network round-trip if the files are already there
    # AND the user hasn't asked for --force. Mongoose is version-pinned
    # in the script itself, so a re-fetch only matters when the script
    # gets bumped.
    if [ -f "${mongoose_c}" ] && [ -f "${mongoose_h}" ] && [ "${1:-}" != "--force" ]; then
        ok "Mongoose already vendored at web/vendor/mongoose.{c,h}."
        log "  Pass --force to re-download."
    else
        log "Fetching Mongoose source into web/vendor/..."
        ( cd "${CPCU_ROOT}/web/vendor" && bash "${fetcher}" ) \
            || fatal "fetch.sh failed — check network or web/vendor/ permissions"
        ok "Mongoose fetched."
    fi

    # CMake's WS_HAS_MONGOOSE branch is decided at configure-time, not
    # build-time. So even though the files are now in place, the
    # cached install plan still says "build cpcu_ws as a stub". Force
    # a clean reconfigure so cpcu_ws picks up the real source.
    log "Triggering a clean rebuild so cpcu_ws picks up Mongoose..."
    cmd_build --clean
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: ws (web dashboard)
# ════════════════════════════════════════════════════════════════════════

# Detects whether the installed cpcu_ws is the "stub" build (no
# Mongoose, no real HTTP server). The stub prints a recognisable line
# in its startup banner; we grep for it here. Returns 0 if it's a stub.
cpcu_ws_is_stub() {
    [ -x "${BIN_DIR}/cpcu_ws" ] || return 1
    "${BIN_DIR}/cpcu_ws" --version 2>&1 | grep -q "BUILT WITHOUT MONGOOSE" && return 0
    # Older builds that don't support --version: fall back to checking
    # whether mongoose.c was vendored at build time. Heuristic — if
    # it's not in the source tree, the binary almost certainly is a
    # stub.
    [ ! -f "${CPCU_ROOT}/web/vendor/mongoose.c" ] && return 0
    return 1
}

cmd_ws() {
    LAUNCH_MODE="ws"
    log "${C_BLD}Web Dashboard Setup${C_RST}"
    echo

    # ── Step 1: Check if the project is built at all ──
    if [ ! -x "${BIN_DIR}/cpcu_ws" ] && [ ! -x "${BIN_DIR}/cpcu_kernel" ]; then
        log "Step 1/4: ${C_YEL}Project not built yet.${C_RST}"
        if [ ! -d /opt/cpcu ]; then
            log "  /opt/cpcu doesn't exist — running one-time Pi setup first."
            log "  This installs build tools, enables SPI/I2C, and isolates CPU cores."
            echo
            cmd_setup
            echo
        fi
        log "  Building the project (cmake + make + install)..."
        cmd_build
        echo
    else
        log "Step 1/4: ${C_GRN}Project built.${C_RST}"
    fi

    # ── Step 2: Check if Mongoose is vendored ──
    if [ ! -x "${BIN_DIR}/cpcu_ws" ] || cpcu_ws_is_stub; then
        log "Step 2/4: ${C_YEL}Mongoose not vendored — downloading...${C_RST}"
        log "  Mongoose is the embedded HTTP/WebSocket library (~26K lines)."
        log "  Fetching from GitHub and rebuilding cpcu_ws..."
        echo
        cmd_vendor
        echo
    else
        log "Step 2/4: ${C_GRN}Mongoose vendored and cpcu_ws built.${C_RST}"
    fi

    # ── Step 3: Resolve static files directory ──
    local static_dir
    if [ -d /opt/cpcu/ws_static ]; then
        static_dir=/opt/cpcu/ws_static
    elif [ -d "${CPCU_ROOT}/web/static" ]; then
        static_dir="${CPCU_ROOT}/web/static"
    else
        fatal "Web static directory not found. Re-run './launch.sh build'."
    fi
    log "Step 3/4: ${C_GRN}Static files at ${static_dir}${C_RST}"

    # ── Step 4: Start the dashboard ──
    local pi_ip
    pi_ip=$(hostname -I 2>/dev/null | awk '{print $1}')
    local pi_host
    pi_host="${HOSTNAME:-$(hostname 2>/dev/null)}"

    log "Step 4/4: ${C_GRN}Starting web dashboard...${C_RST}"
    echo
    echo -e "  ${C_BLD}╔══════════════════════════════════════════════════════════╗${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}                                                          ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}   Open in any browser on the same network:               ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}                                                          ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}      ${C_GRN}${C_BLD}http://${pi_ip}:8765${C_RST}                          ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}                                                          ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}   or: ${C_CYN}http://${pi_host}.local:8765${C_RST}  (if mDNS works)   ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}                                                          ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}   Anyone on your LAN can view (read-only, multi-viewer). ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}   To restrict: --bind ws://127.0.0.1:8765                ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}║${C_RST}                                                          ${C_BLD}║${C_RST}"
    echo -e "  ${C_BLD}╚══════════════════════════════════════════════════════════╝${C_RST}"
    echo

    # If kernel not running, start it in tmux alongside the dashboard
    if [ ! -e /dev/shm/cpcu_ipc ]; then
        if ! require_tmux; then
            err "tmux not installed — run './launch.sh setup' to install it."
            exit 1
        fi
        write_ws_info
        log "Starting kernel + web dashboard in tmux..."
        write_ws_info
        log "  Ctrl-b 0 = KERNEL  |  Ctrl-b 1 = SHELL  |  Ctrl-b 2 = WS"
        tmux_create_with_kernel || fatal "Couldn't bring up kernel"
        tmux_add_window "WS" "${BIN_DIR}/cpcu_ws --static \"${static_dir}\" $*"
        tmux_attach_at "WS"
        return $?
    fi

    # Kernel already running — attach directly
    write_ws_info
    log "Kernel already running. Starting cpcu_ws in foreground (Ctrl+C to stop)."
    exec "${BIN_DIR}/cpcu_ws" --static "${static_dir}" "$@"
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: attach / stop
# ════════════════════════════════════════════════════════════════════════

cmd_attach() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Attaching to existing session '$SESSION_NAME'..."
        trap - EXIT INT TERM
        exec tmux attach -t "$SESSION_NAME"
    else
        err "No tmux session named '$SESSION_NAME' is running."
        err "Start one with: ./launch.sh tui  (or signal, collect)"
        exit 1
    fi
}

cmd_stop() {
    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "No active session."
        pkill -f cpcu_kernel 2>/dev/null || true
        pkill -f cpcu_dsp.py 2>/dev/null || true
        pkill -f cpcu_audio_daemon.py 2>/dev/null || true
        clear_ws_info
        return
    fi
    log "Stopping (safe servo shutdown)..."
    # SIGTERM → kernel → cpcu_io runs cleanup:
    #   PCA_SetAllNeutral → sleep 300ms → PCA_AllOff → NRF_PowerDown
    pkill -TERM cpcu_kernel 2>/dev/null || true
    pkill -TERM -f cpcu_dsp.py 2>/dev/null || true
    pkill -TERM -f cpcu_audio_daemon.py 2>/dev/null || true
    log "Waiting for servo neutral + disable..."
    sleep 1.5
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
    sleep 0.5
    pkill -9 -f cpcu_kernel 2>/dev/null || true
    pkill -9 -f cpcu_dsp.py 2>/dev/null || true
    clear_ws_info
    ok "Stopped. Servos neutral and disabled."
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: grant-caps
# ════════════════════════════════════════════════════════════════════════

cmd_grant_caps_internal() {
    if [ "$(id -u)" -ne 0 ]; then
        log "  (one sudo prompt for setcap)..."
        exec sudo "$0" grant-caps
    fi
    for B in cpcu_io cpcu_kernel; do
        if [ -x "${BIN_DIR}/${B}" ]; then
            setcap 'cap_sys_nice,cap_ipc_lock+ep' "${BIN_DIR}/${B}" \
                && log "  setcap OK: ${BIN_DIR}/${B}" \
                || warn "  setcap FAILED on ${BIN_DIR}/${B}"
        else
            warn "  ${BIN_DIR}/${B} not found — install first with './launch.sh build'"
        fi
    done
}

cmd_grant_caps() {
    log "Re-applying CAP_SYS_NICE + CAP_IPC_LOCK to installed binaries..."
    cmd_grant_caps_internal
    ok "Capabilities re-applied. They persist until the next rebuild."
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: install-service / install-ws-service
# ════════════════════════════════════════════════════════════════════════

cmd_install_service() {
    if [ "$(id -u)" -ne 0 ]; then
        log "install-service writes /etc/systemd/system/cpcu.service (one sudo prompt)..."
        exec sudo "$0" install-service
    fi

    REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"

    cat > /etc/systemd/system/cpcu.service << SVCEOF
[Unit]
Description=CPCU Prosthetic Hand Controller
After=network.target

[Service]
Type=simple
User=${REAL_USER}
ExecStart=${CPCU_DIR}/launch.sh kernel
Restart=on-failure
RestartSec=2
StandardOutput=append:${LOG_DIR}/cpcu.log
StandardError=append:${LOG_DIR}/cpcu.log

# RT scheduling needs nice + mlockall
AmbientCapabilities=CAP_SYS_NICE CAP_IPC_LOCK
LimitMEMLOCK=infinity
LimitRTPRIO=99

[Install]
WantedBy=multi-user.target
SVCEOF
    log "Wrote /etc/systemd/system/cpcu.service (User=${REAL_USER})"

    for B in cpcu_io cpcu_kernel; do
        if [ -x "${BIN_DIR}/${B}" ]; then
            setcap 'cap_sys_nice,cap_ipc_lock+ep' "${BIN_DIR}/${B}" \
                && log "  setcap OK: ${BIN_DIR}/${B}"
        fi
    done

    systemctl daemon-reload
    systemctl enable cpcu.service
    ok "Service enabled. The kernel will auto-start at next boot."
    log "Manual control:"
    log "  sudo systemctl start cpcu"
    log "  sudo systemctl stop cpcu"
    log "  sudo systemctl status cpcu"
    log "  journalctl -u cpcu -f              (no sudo needed)"
}

cmd_install_ws_service() {
    if [ "$(id -u)" -ne 0 ]; then
        log "install-ws-service writes /etc/systemd/system/cpcu_ws.service (one sudo prompt)..."
        exec sudo "$0" install-ws-service
    fi
    REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"

    local static_dir
    if [ -d /opt/cpcu/ws_static ]; then
        static_dir=/opt/cpcu/ws_static
    else
        static_dir="${CPCU_ROOT}/web/static"
    fi

    cat > /etc/systemd/system/cpcu_ws.service << WSEOF
[Unit]
Description=CPCU Dashboard — read-only web bridge
After=cpcu.service network.target
Requires=cpcu.service

[Service]
Type=simple
User=${REAL_USER}
ExecStart=${BIN_DIR}/cpcu_ws --bind ws://0.0.0.0:8765 --static ${static_dir}
Restart=on-failure
RestartSec=2
StandardOutput=append:${LOG_DIR}/cpcu_ws.log
StandardError=append:${LOG_DIR}/cpcu_ws.log

[Install]
WantedBy=multi-user.target
WSEOF
    log "Wrote /etc/systemd/system/cpcu_ws.service (User=${REAL_USER})"
    log "  bind:        ws://0.0.0.0:8765 (LAN-shared)"
    log "  static dir:  ${static_dir}"
    systemctl daemon-reload
    systemctl enable cpcu_ws.service
    ok "Web dashboard service enabled. Will start at next boot or:"
    log "  sudo systemctl start cpcu_ws"
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: menu
# ════════════════════════════════════════════════════════════════════════

show_menu() {
    cat <<'EOF'

  ┌──────────────────────────────────────────────────────────────────┐
  │              CPCU v3.0 — Mode Selection (tmux)                   │
  ├──────────────────────────────────────────────────────────────────┤
  │  1) kernel    Run kernel only  (foreground, no UI)               │
  │  2) tui       tmux: [KERNEL][SHELL][TUI]    — main dashboard     │
  │  3) collect   tmux: [KERNEL][SHELL][TUI]    — capture workflow   │
  │  4) signal    tmux: [KERNEL][SHELL][SIGNAL] — signal testbench   │
  │  5) pca       PCA9685 servo calibration only  (no kernel)        │
  │  6) ws        Web dashboard (browser-accessible)                 │
  │  7) smoother  Servo motion profile exerciser  (needs kernel)     │
  │  q) quit                                                         │
  ├──────────────────────────────────────────────────────────────────┤
  │  Ctrl-b 0 = KERNEL | Ctrl-b 1 = SHELL | Ctrl-b 2 = tool window  │
  └──────────────────────────────────────────────────────────────────┘
EOF
    while true; do
        read -p "  Choice [1-7/q]: " choice
        case "${choice}" in
            1|kernel)   run_kernel_only;  return $? ;;
            2|tui)      run_tui;          return $? ;;
            3|collect)  run_collect;      return $? ;;
            4|signal)   run_signal;       return $? ;;
            5|pca)      run_pca;          return $? ;;
            6|ws)       cmd_ws;           return $? ;;
            7|smoother) run_smoother;     return $? ;;
            q|Q|quit)  log "Exiting without launching."; return 0 ;;
            *)         echo "  Invalid choice." ;;
        esac
    done
}


# ════════════════════════════════════════════════════════════════════════
#  COMMAND: help
# ════════════════════════════════════════════════════════════════════════


print_v3_help() {
cat << 'HELPEOF'

  launch.sh — InfiniTech CPCU v3.0
  ═══════════════════════════════════════════════════════════════
  THIS IS THE ONLY SCRIPT YOU NEED. EVERY OPERATION GOES
  THROUGH ./launch.sh <command>.
  ═══════════════════════════════════════════════════════════════

  SETUP & BUILD (once per Pi, then once per source change)
  ─────────────────────────────────────────────────────────────
    ./launch.sh setup                         Pi one-time config
    ./launch.sh setup-audio                   I2S DAC + speaker
    ./launch.sh setup-uart                    UART debug output
    ./launch.sh build                         Compile + install
    ./launch.sh check                         Verify readiness

  RUNNING THE SYSTEM
  ─────────────────────────────────────────────────────────────
    ./launch.sh tui                           TUI dashboard
    ./launch.sh tui --audio                   + voice/tone feedback
    ./launch.sh tui --uart                    + UART debug to PC
    ./launch.sh tui --with-ws                 + web dashboard
    ./launch.sh tui --audio --uart --with-ws  all features
    ./launch.sh tui --operator NAME           operator velocity profile
    ./launch.sh ws                            web dashboard only
    ./launch.sh kernel                        kernel only (for systemd)
    ./launch.sh collect                       dataset capture mode
    ./launch.sh signal                        signal testbench (live)
    ./launch.sh smoother                      servo motion exerciser
    ./launch.sh pca                           direct PCA9685 calibration
    ./launch.sh menu                          interactive mode picker
    ./launch.sh attach                        re-attach tmux session
    ./launch.sh stop                          stop (safe servo shutdown)

  TESTING
  ─────────────────────────────────────────────────────────────
    ./launch.sh test-sw                       software tests (233 checks)
    ./launch.sh test-ipc                      + IPC validation
    ./launch.sh test-hw                       + hardware probes
    ./launch.sh test-pca                      interactive servo check
    ./launch.sh test-signal                   live signal integrity
    ./launch.sh test-signal-demo              synthetic signal (no BSAU)
    ./launch.sh test-safety-demo              fault injection demo
    ./launch.sh test-system                   full system verification

  EMG CHANNELS
  ─────────────────────────────────────────────────────────────
    ./launch.sh set-channels 0 1 2            3-channel mode
    ./launch.sh set-channels 0 1 2 3 4        5-channel mode
    ./launch.sh set-channels 0 1 2 3 4 5 6 7  8-channel (full)
                                               validates model match

  SERVO MOTORS
  ─────────────────────────────────────────────────────────────
    ./launch.sh add-motor Thumb 6             add on PCA channel 6
    ./launch.sh edit-motor Gripper            edit limits (min/max/neutral)
    ./launch.sh rename-motor Gripper Claw     rename (updates all refs)

  GESTURES
  ─────────────────────────────────────────────────────────────
    ./launch.sh add-gesture                   guided wizard (+ audio)
    ./launch.sh edit-gesture flex             change servo mapping
    ./launch.sh rename-gesture hand grip      rename gesture
    ./launch.sh remove-gesture biceps         delete gesture

  CALIBRATION & TUNING
  ─────────────────────────────────────────────────────────────
    ./launch.sh grip-tune                     gripper firmness wizard
    ./launch.sh calibrate                     rest noise + velocity (0-10)
    ./launch.sh calibrate --operator NAME     per-operator profile
    ./launch.sh calibrate --rest-only         noise floor only
    ./launch.sh calibrate --vel-only          velocity preference only

  AUDIO FEEDBACK (PCM5102A + PAM8403 + speaker)
  ─────────────────────────────────────────────────────────────
    ./launch.sh audio                         show audio config
    ./launch.sh audio off                     disable audio
    ./launch.sh audio voice                   spoken word cues
    ./launch.sh audio freq                    frequency tone cues
    ./launch.sh audio volume 80               set volume (0-100%)
    ./launch.sh audio test                    play test sound
    ./launch.sh generate-cues                 generate voice .wav files

  MODEL MANAGEMENT
  ─────────────────────────────────────────────────────────────
    ./launch.sh set-model models/model_5ch.pkl  set active model
    ./launch.sh set-model                       list available models

  CONFIGURATION
  ─────────────────────────────────────────────────────────────
    ./launch.sh show-config                   print full system config
    ./launch.sh configure                     compile-time settings
    ./launch.sh configure --show              show all compile values
    ./launch.sh configure --diff              changes from defaults
    ./launch.sh configure --reset             restore defaults

  RELOAD (apply config changes without full restart)
  ─────────────────────────────────────────────────────────────
    ./launch.sh reload                        all (runtime + DSP + audio)
    ./launch.sh reload --dsp                  DSP pipeline only
    ./launch.sh reload --audio                audio daemon only

  UART DEBUG (to host PC via USB-UART adapter)
  ─────────────────────────────────────────────────────────────
    ./launch.sh setup-uart                    enable UART on Pi 5
    ./launch.sh tui --uart                    run with UART debug
    Host: python3 scripts/uart_monitor.py --port /dev/ttyUSB0
    Host: python3 scripts/uart_monitor.py --port COM3 --log data.csv

  SERVICES (auto-start at boot)
  ─────────────────────────────────────────────────────────────
    ./launch.sh install-service               kernel systemd unit
    ./launch.sh install-ws-service            web dashboard service
    ./launch.sh grant-caps                    re-apply RT capabilities

  EXAMPLES
  ─────────────────────────────────────────────────────────────

    # fresh Pi setup (once):
    ./launch.sh setup && ./launch.sh setup-audio
    ./launch.sh build && ./launch.sh generate-cues && ./launch.sh check

    # daily operation:
    ./launch.sh tui --audio --with-ws

    # add a servo motor and use it in a gesture:
    ./launch.sh add-motor Thumb 6
    ./launch.sh edit-motor Thumb
    ./launch.sh add-gesture               # wizard asks which motors
    ./launch.sh collect                    # record training data
    ./launch.sh set-model models/new.pkl   # after retraining
    ./launch.sh reload

    # upgrade from 3 to 5 EMG channels:
    ./launch.sh set-channels 0 1 2 3 4
    ./launch.sh collect
    ./launch.sh set-model models/model_5ch.pkl
    ./launch.sh reload

    # calibrate for a new operator:
    ./launch.sh calibrate --operator ali
    ./launch.sh tui --operator ali --audio

    # rename things:
    ./launch.sh rename-motor Gripper Claw
    ./launch.sh rename-gesture hand grip

    # switch audio mode:
    ./launch.sh audio freq
    ./launch.sh reload --audio

    # tune gripper firmness:
    ./launch.sh grip-tune

    # UART debug to laptop:
    ./launch.sh setup-uart    # once, then reboot
    ./launch.sh tui --uart
    # on laptop: python3 scripts/uart_monitor.py --port /dev/ttyUSB0

HELPEOF
}

cmd_help() {
    local topic="${1:-}"
    case "${topic}" in
        ""|main)
            print_v3_help
            ;;
        setup)
            cat <<'EOF'

./launch.sh setup
─────────────────
Configures the Raspberry Pi for CPCU. Run this once on a fresh Pi.

What it does:
  - Installs build tools (gcc, cmake, libncurses-dev, tmux, i2c-tools)
  - Installs Python libs (numpy, scipy, joblib, scikit-learn)
  - Enables SPI and I²C in /boot/firmware/config.txt
  - Reserves CPU cores 1-3 for real-time use (isolcpus boot parameter)
  - Disables Bluetooth (frees up kernel interrupts)
  - Adds you to the spi/i2c/gpio groups
  - Creates /opt/cpcu/{bin,scripts,python,models,test} owned by you
  - Symlinks /opt/cpcu/config.json to your config/runtime.json

Side effects: requires sudo (one prompt), may require reboot after.
The script is idempotent — safe to re-run.

EOF
            ;;
        build)
            cat <<'EOF'

./launch.sh build [--clean]
───────────────────────────
Compiles the C binaries and installs them to /opt/cpcu.

What it does:
  - Runs cmake configure if needed (lazy: only when CMakeLists.txt changed)
  - Compiles all binaries (cpcu_io, cpcu_kernel, cpcu_tui, testbenches)
  - Copies binaries to /opt/cpcu/bin
  - Copies Python helpers to /opt/cpcu/python
  - Copies launch.sh to /opt/cpcu/launch.sh
  - Re-applies real-time capabilities (CAP_SYS_NICE, CAP_IPC_LOCK)
  - Auto-detects "built but not installed" inconsistencies and
    self-recovers with a clean rebuild

Pass --clean to force a fresh build directory. Use this when you've
replaced CMakeLists.txt and the cmake cache has gone stale (added or
removed install targets, changed compile flags, etc.). Equivalent to
`rm -rf build && ./launch.sh build`.

Run this every time you change C source. You'll see one sudo prompt
at the end (for the capability granting step).

EOF
            ;;
        check)
            cat <<'EOF'

./launch.sh check
─────────────────
Verifies the system is ready to launch. Reports what's wrong without
trying to start anything.

Pass criteria:
  - cpcu_kernel + cpcu_io binaries exist and are installed
  - Real-time capabilities applied
  - Cores 1-3 isolated
  - SPI and I²C enabled
  - PCA9685 detected at I²C 0x40
  - Runtime config symlink exists
  - You're in the spi/i2c groups

Warning-level (system can still start):
  - ML model files missing → DSP runs in feature-only mode
  - Python libs missing → DSP degrades

Exit code: 0 if launch is OK, 1 if there's a fatal-class problem.

EOF
            ;;
        test-sw|test-ipc|test-hw|test-pca|test-signal|test-signal-demo|test-safety-demo)
            cat <<'EOF'

./launch.sh test*
─────────────────
Test commands run subsystem verifications. Each command runs its own
phase plus all earlier phases.

  ./launch.sh test-sw
    Phase 1: software-only tests. Codec, safety FSM, smoother, DSP
    pipeline, runtime config loader, TUI editor, JSON serializer.
    Expected: 233 PASS. No hardware needed.

  ./launch.sh test-ipc
    Phase 1 + Phase 2: also validates IPC bridge offsets between C
    and Python by spinning up the kernel briefly.

  ./launch.sh test-hw
    Phase 1 + 2 + 3: also probes Pi hardware (SPI, I²C, PCA9685,
    core isolation, CPU temp).

  ./launch.sh test-pca
    Interactive PCA9685 servo calibration TUI. Direct I²C, no kernel
    running. Use arrows to select servo, m/M for min/max, etc.

  ./launch.sh test-signal
    Live signal-integrity TUI. Needs cpcu_kernel running and BSAU
    transmitting. Plots raw 8-channel ADC streams. Function generator
    on PA0 is the typical input.

  ./launch.sh test-signal-demo
    Same TUI but with synthetic 100 Hz sine waves. No hardware needed.

  ./launch.sh test-safety-demo
    cpcu_tui --demo with fault-injection hotkeys (F=radio loss,
    B=battery low, G=seq-gap storm, etc.).

EOF
            ;;
        configure)
            cat <<'EOF'

./launch.sh configure [args...]
───────────────────────────────
Edit compile-time #defines in safety headers (radio timeout, battery
thresholds, thermal limits, NRF channel). After editing, prompts to
rebuild.

Sub-commands (passed through to configure.sh):
  ./launch.sh configure                        Interactive walkthrough
  ./launch.sh configure --show                 Show all current values
  ./launch.sh configure --diff                 Show only changes from defaults
  ./launch.sh configure --reset                Restore all defaults
  ./launch.sh configure --reset --runtime      Also regenerate runtime.json
  ./launch.sh configure --<name>               Show one current value
  ./launch.sh configure --<name> <value>       Set one value

Available knobs (--name):
  --radio-timeout    silence (ms) before RUNNING → DEGRADED
  --radio-safe       DEGRADED duration (ms) before SAFE
  --boot-grace       cold-start grace before radio fault arms
  --vbat-low         battery LOW threshold (V)
  --vbat-crit        battery CRITICAL threshold (V)
  --thermal-warn     thermal WARN (°C)
  --thermal-crit     thermal CRITICAL (°C)
  --i2c-max          consecutive I²C failures before SAFE
  --ring-overflow    ring overflows before fault
  --nrf-channel      BSAU NRF channel (0-125)

Note: this is for SAFETY thresholds. For RUNTIME tunables (servo
limits, gesture velocities, smoother knobs, grip levels), edit
config/runtime.json directly OR use the TUI's edit mode (press 'e'
on the CONFIG page).

EOF
            ;;
        tui|signal|collect|pca|kernel|ws|vendor|menu|attach|stop|install-service|install-ws-service|grant-caps)
            cat <<EOF

./launch.sh ${topic}
$(printf '─%.0s' $(seq 1 $((${#topic} + 13))))

EOF
            case "${topic}" in
                tui)
                    cat <<'EOF'
Bring up the kernel + the ncurses dashboard inside a tmux session.
Three windows: KERNEL (logs), SHELL (for ./launch.sh stop), TUI (dashboard).

Inside the TUI, the 7 pages are:
  1=Overview   2=Radio/IO   3=DSP/AI   4=Waves
  5=Health     6=Dataset    7=Config
Press 'e' on the Config page to enter live edit mode (arm parks).
Press 'q' to quit the TUI (kernel keeps running).

Windows:  Ctrl-b 0 = KERNEL  |  Ctrl-b 1 = SHELL  |  Ctrl-b 2 = TUI
Detach with Ctrl-b d. Re-attach later with './launch.sh attach'.

Combine with the web dashboard:
  ./launch.sh tui --with-ws
This adds a fourth tmux window (WS) running cpcu_ws so others can
watch from a browser at http://<pi-ip>:8765 while you work in the
TUI. Both views share one kernel and one IPC region — no duplication.
EOF
                    ;;
                signal)
                    cat <<'EOF'
Bring up the kernel + signal-integrity testbench inside a tmux session.
Three windows: KERNEL (logs), SHELL (for commands), SIGNAL (testbench).
Plots raw 8-channel ADC data straight off the IPC ring.

Pass criteria when a function generator drives PA0 with a 100 Hz sine,
0.6 V amplitude, 1.65 V DC offset:
  - Clean sinusoid on each channel
  - Dominant frequency ≈ 100 Hz
  - Vpp ≈ 1.2 V, SNR > 20 dB
  - Packet rate ≈ 1000/s, loss < 0.1%

Windows:  Ctrl-b 0 = KERNEL  |  Ctrl-b 1 = SHELL  |  Ctrl-b 2 = SIGNAL
Press TAB for all-channel view, q to quit.

Combine with the web dashboard:
  ./launch.sh signal --with-ws
This adds a fourth tmux window (WS). Useful for showing the live
signal stream to someone in a browser while you watch it locally.
EOF
                    ;;
                collect)
                    cat <<'EOF'
Like 'tui' but with on-screen reminders for the dataset capture
workflow. Use this when you're recording EMG to .csv files for
training. Three windows: KERNEL, SHELL, TUI.

Inside the TUI:
  - Press 7 to jump to the DATASET page
  - ←/→ cycle gesture labels
  - t to toggle RAW/FILTERED capture
  - s or SPACE to start, again to stop and save
  - r to cancel and delete a partial capture
  - q to quit. Files land in /opt/cpcu/bin/datasets/.

Windows:  Ctrl-b 0 = KERNEL  |  Ctrl-b 1 = SHELL  |  Ctrl-b 2 = TUI

Combine with the web dashboard:
  ./launch.sh collect --with-ws
The browser dashboard mirrors the data being captured, useful when
recording with someone watching remotely.
EOF
                    ;;
                pca)
                    cat <<'EOF'
Interactive PCA9685 servo calibration. Direct I²C, NO kernel running
— use this to tune per-servo limits and bias before running the live
system.

Inside the TUI:
  - ↑/↓ select servo (S0..S5)
  - ←/→ jog ±10 µs
  - PgUp/PgDn jog ±50 µs
  - m / M jog to current MIN / MAX
  - [ set current jog as MIN
  - ] set current jog as MAX
  - b set current deviation as bias
  - B clear bias
  - s save changes to config/runtime.json
  - q quit (prompts if dirty)
EOF
                    ;;
                smoother)
                    cat <<'EOF'
Servo motion profile exerciser. Sends motor commands through IPC so
cpcu_io's trapezoidal smoother handles the interpolation — you see
the REAL servo motion with your current runtime.json settings.

Requires cpcu_kernel + cpcu_io to be running (start with ./launch.sh tui
in another terminal).

Usage:
  ./launch.sh smoother                            Interactive menu
  ./launch.sh smoother --servo 0 --scenario step  One-shot
  ./launch.sh smoother --servo all --scenario sweep
  ./launch.sh smoother --list                     Show all scenarios

Scenarios:
  step        Step response (neutral → max → neutral)
  sweep       Full sweep (neutral → min → max → neutral)
  triangle    Triangle wave (two full cycles)
  small_step  ±50 µs around neutral (deadband test)
  burst       Rapid min↔max toggles (stress test)
  slow_creep  Neutral → neutral+200 (see the accel ramp)

Tuning workflow:
  1. Run a scenario, watch the servo
  2. In TUI CONFIG page: e → edit vel/accel/deadband → Ctrl+S
  3. Re-run the same scenario — motion changes immediately
  4. Repeat until the feel is right
EOF
                    ;;
                kernel)
                    cat <<'EOF'
Run cpcu_kernel only, in the foreground. This is the path systemd
uses. You won't see the TUI; logs go to /var/log/cpcu/cpcu.log.

For interactive use, prefer ./launch.sh tui instead.
EOF
                    ;;
                ws)
                    cat <<'EOF'
Start the web dashboard on http://<pi-ip>:8765. Read-only,
multi-viewer, browser-accessible from any device on the LAN.

If the kernel isn't already running, this spawns it in a tmux session:
  Ctrl-b 0 = KERNEL  |  Ctrl-b 1 = SHELL  |  Ctrl-b 2 = WS

Tabs in the dashboard:
  Overview    System state, packet rate, classification
  Waves       8-channel rolling raw + filtered plots
  Spectrum    FFT + waterfall on a selected channel
  Tools       NRF / PCA / DSP diagnostics

Default bind is 0.0.0.0 — anyone on your LAN can view. To restrict
to localhost only, run: ./launch.sh ws --bind ws://127.0.0.1:8765

If cpcu_ws was built without Mongoose (the embedded HTTP/WS library),
this command will detect that and offer to fix it for you in one
prompt — the alternative is './launch.sh vendor' run manually.
EOF
                    ;;
                vendor)
                    cat <<'EOF'
Fetch third-party dependencies into the source tree. Currently this
means downloading Mongoose (the embedded HTTP+WebSocket library used
by cpcu_ws) into web/vendor/, then doing a clean rebuild so cpcu_ws
links the real library instead of the no-network stub.

Why this is a separate command:
  - Mongoose is ~26k lines of vendored source. Keeping it out of the
    main source tree makes git history cleaner and lets us version-
    pin via web/vendor/fetch.sh (currently 7.14).
  - The fetch is a one-time step per checkout. Once vendored, the
    files persist until you delete them.

Options:
  --force    re-download even if web/vendor/mongoose.{c,h} exist
             (use after editing fetch.sh to bump the pinned version)

Network: needs internet access to raw.githubusercontent.com. If your
Pi is on an air-gapped network, fetch on a connected machine, copy
web/vendor/mongoose.{c,h} over, then run './launch.sh build --clean'.
EOF
                    ;;
                menu)
                    cat <<'EOF'
Show the interactive mode picker. This is the default if you run
./launch.sh with no argument from a TTY.
EOF
                    ;;
                attach)
                    cat <<'EOF'
Re-attach to a previously detached tmux session. If you ran
./launch.sh tui and pressed Ctrl-b d, the session keeps running in
the background; this brings you back to it.
EOF
                    ;;
                stop)
                    cat <<'EOF'
Kill a running tmux session and all its child processes (kernel, io,
dsp, TUI). Use this when you're done for the day.
EOF
                    ;;
                install-service)
                    cat <<'EOF'
Install a systemd unit so cpcu_kernel starts automatically at boot.
Writes /etc/systemd/system/cpcu.service (one sudo prompt).

After installing:
  sudo systemctl start cpcu      # start now
  sudo systemctl stop cpcu       # stop
  sudo systemctl status cpcu     # check
  journalctl -u cpcu -f          # live log tail
EOF
                    ;;
                install-ws-service)
                    cat <<'EOF'
Install a systemd unit for the web dashboard. Depends on cpcu.service.
Writes /etc/systemd/system/cpcu_ws.service (one sudo prompt).
EOF
                    ;;
                grant-caps)
                    cat <<'EOF'
Re-apply CAP_SYS_NICE (real-time scheduling) and CAP_IPC_LOCK (page
locking) capabilities to cpcu_io and cpcu_kernel. Capabilities are
stored on the binary's inode and lost when a rebuild creates a new
file, so you need to re-run this after every rebuild.

Note: ./launch.sh build runs grant-caps automatically. You only need
to run it manually if something went wrong.
EOF
                    ;;
            esac
            ;;
        *)
            err "No help topic '${topic}'."
            log "Try: ./launch.sh help        (top-level)"
            log "  or: ./launch.sh help setup"
            exit 1
            ;;
    esac
}

cmd_version() { echo "InfiniTech CPCU launch.sh v3.0 (April 2026)"; }




# ════════════════════════════════════════════════════════════════════════
#  COMMAND: test-system (live system-level requirements verification)
# ════════════════════════════════════════════════════════════════════════

cmd_test_system() {
    LAUNCH_MODE="test-system"

    if [ ! -e /dev/shm/cpcu_ipc ]; then
        fatal "Kernel not running. Start with: ./launch.sh tui (then run test-system in SHELL window)"
    fi

    local test_script=""
    if [ -f "${CPCU_ROOT}/test/system_test.py" ]; then
        test_script="${CPCU_ROOT}/test/system_test.py"
    elif [ -f "/opt/cpcu/test/system_test.py" ]; then
        test_script="/opt/cpcu/test/system_test.py"
    else
        fatal "system_test.py not found. Re-run './launch.sh build'."
    fi

    log "Running system-level requirements verification..."
    log "  This requires cpcu_kernel + cpcu_io + cpcu_dsp to be running."
    log "  The test monitors live IPC data for the specified duration."
    echo
    exec python3 "${test_script}" "$@"
}

# ════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════

MODE="${1:-}"
shift || true

# Composable flag: --with-ws (or --ws) appended to a kernel-aware
# command makes the tmux session also include a WS window. Examples:
#   ./launch.sh tui --with-ws
#   ./launch.sh signal --with-ws
#   ./launch.sh collect --with-ws
# We strip the flag here from $@ so the remaining args pass cleanly
# through to the underlying command.

# ══════════════════════════════════════════════════════════════════════
#  v3.0 COMMANDS — merged from launch_additions.sh
# ══════════════════════════════════════════════════════════════════════
#!/bin/bash
## ═══════════════════════════════════════════════════════════════════
##  launch.sh v3.0 — ALL new commands, helpers, and help text.
##  Paste into existing launch.sh at marked locations.
## ═══════════════════════════════════════════════════════════════════

GS="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/config/gestures.json"

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
    python3 << PYEOF
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
    if isinstance(ed, str) or en.startswith("_"): continue
    print(f"  {en:<20s} P{ed.get('priority','?')}  voice={ed.get('voice','-'):<24s} {ed.get('freq_hz','-')}Hz")

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


# generate voice + freq audio for one gesture (called after add/rename)
_regen_gesture_audio() {
    local gname="$1"
    local gs="${GS}"
    python3 -c "
import json, subprocess, os, wave, numpy as np
with open('${gs}') as f: g = json.load(f)
gdef = g.get('gestures', {}).get('${gname}')
if not gdef: exit(0)
audio = gdef.get('audio', {})
audio_dir = os.path.join(os.path.dirname('${gs}'), 'audio_cues')
os.makedirs(audio_dir, exist_ok=True)
# voice cue
vname = audio.get('voice')
if vname:
    wav = os.path.join(audio_dir, f'{vname}.wav')
    text = '${gname}'.replace('_', ' ')
    try:
        subprocess.run(['espeak-ng','-v','en','-s','180','-p','40','-a','150','-w',wav,text],
                       check=True, capture_output=True)
        print(f'  voice: {vname}.wav')
    except FileNotFoundError:
        print(f'  espeak-ng not found — run ./launch.sh generate-cues later')
# freq tone
fhz = audio.get('freq_hz', 0)
fms = audio.get('freq_ms', 80)
if fhz > 0:
    sr = 22050; n = int(sr * fms / 1000)
    t = np.linspace(0, fms/1000, n, False)
    fade = min(int(sr*0.005), n//4)
    env = np.ones(n)
    if fade > 0: env[:fade] = np.linspace(0,1,fade); env[-fade:] = np.linspace(1,0,fade)
    data = (np.sin(2*np.pi*fhz*t)*env*16000).astype(np.int16)
    tp = os.path.join(audio_dir, f'_gen_${gname}_{fhz}hz.wav')
    with wave.open(tp,'w') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr); w.writeframes(data.tobytes())
    print(f'  tone: {fhz}Hz/{fms}ms')
" 2>/dev/null
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
    python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
if '${name}' not in g.get('gestures',{}):
    print(f\"  '${name}' not found.\"); sys.exit(1)
gd = g['gestures']['${name}']
print(f\"  Gesture: ${name}  mode={gd.get('mode','?')}\")
print(f\"  Motors: {list(g.get('servo_channels',{}).keys())}\")
print(f\"  Current: {gd.get('channels',{})}\")
"
    echo "  Enter motor mappings (MotorName Rate), empty to finish:"
    local channels="{"
    local first=1
    while true; do
        read -rp "  Motor Rate: " line
        [ -z "$line" ] && break
        local mname=$(echo "$line" | awk '{print $1}')
        local rate=$(echo "$line" | awk '{print $2}')
        [ -z "$rate" ] && { echo "  Format: MotorName Rate"; continue; }
        [ $first -eq 0 ] && channels="${channels},"
        read -rp "  ${mname} snap? (y/n) [n]: " sn
        local snap="false"
        [[ "$sn" =~ ^[yY] ]] && snap="true"
        channels="${channels} \"${mname}\": {\"rate_us_s\": ${rate}, \"snap\": ${snap}}"
        first=0
    done
    channels="${channels} }"
    [ "$first" -eq 1 ] && { echo "  No changes."; return; }
    python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
g['gestures']['${name}']['channels'] = ${channels}
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
print('  \033[32m✓\033[0m Updated.')
"
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

# ── remove-motor ──
cmd_remove_motor() {
    local name="${1:-}"
    if [ -z "$name" ]; then
        err "Usage: ./launch.sh remove-motor <name>"
        python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
print('  Motors: ' + ', '.join(g.get('servo_channels',{}).keys()))
"
        exit 1
    fi
    python3 << PYEOF
import json, sys
with open("${GS}") as f: g = json.load(f)
sc = g.get("servo_channels", {})
if "${name}" not in sc:
    print(f"  Motor '${name}' not found. Available: {list(sc.keys())}")
    sys.exit(1)
refs = [gn for gn, gd in g.get("gestures", {}).items() if "${name}" in gd.get("channels", {})]
if refs:
    print(f"  \033[33m⚠\033[0m Gestures {refs} reference '${name}' — they will break.")
del sc["${name}"]
with open("${GS}", "w") as f: json.dump(g, f, indent=4)
print(f"  \033[32m✓\033[0m Removed motor '${name}'.")
PYEOF
}

# ── edit-motor (change limits) ──
cmd_edit_motor() {
    local name="${1:-}"
    if [ -z "$name" ]; then
        err "Usage: ./launch.sh edit-motor <name>"
        exit 1
    fi
    python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
sc = g.get('servo_channels', {})
if '${name}' not in sc:
    print(f\"  Not found. Available: {list(sc.keys())}\"); sys.exit(1)
sd = sc['${name}']
print(f\"  Motor: ${name}  PCA ch{sd['pca_ch']}\")
print(f\"  Current: min={sd['min_us']} max={sd['max_us']} neutral={sd['neutral_us']}\")
"
    read -rp "  min_us: " mn
    read -rp "  max_us: " mx
    read -rp "  neutral_us: " ne
    [ -z "$mn" ] && [ -z "$mx" ] && [ -z "$ne" ] && { echo "  No changes."; return; }
    python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
sd = g['servo_channels']['${name}']
mn, mx, ne = '${mn}', '${mx}', '${ne}'
if mn: sd['min_us'] = int(mn)
if mx: sd['max_us'] = int(mx)
if ne: sd['neutral_us'] = int(ne)
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
print(f\"  \033[32m✓\033[0m min={sd['min_us']} max={sd['max_us']} neutral={sd['neutral_us']}\")
"
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


WITH_WS=0
WITH_AUDIO=0
WITH_UART=0
OPERATOR="default"
NEW_ARGS=()
for arg in "$@"; do
    case "${arg}" in
        --with-ws|--ws) WITH_WS=1 ;;
        --audio)        WITH_AUDIO=1 ;;
        --uart)         WITH_UART=1 ;;
        --operator)     _NEXT_IS_OPERATOR=1 ;;
        *)
            if [ "${_NEXT_IS_OPERATOR:-0}" = "1" ]; then
                OPERATOR="${arg}"
                _NEXT_IS_OPERATOR=0
            else
                NEW_ARGS+=("${arg}")
            fi
            ;;
    esac
done
set -- "${NEW_ARGS[@]+"${NEW_ARGS[@]}"}"
export CPCU_OPERATOR="${OPERATOR}"
[ "${WITH_UART}" = "1" ] && export CPCU_UART_DEBUG="/dev/ttyAMA0"

case "${MODE}" in
    -h|--help|help)         cmd_help "$@" ;;
    -v|--version|version)   cmd_version ;;

    setup)                  cmd_setup "$@" ;;
    build)                  cmd_build "$@" ;;
    vendor)                 cmd_vendor "$@" ;;
    check)                  cmd_check ;;
    configure)              cmd_configure "$@" ;;

    test-sw)                cmd_test_phase "1" ;;
    test-ipc)               cmd_test_phase "1 2" ;;
    test-hw)                cmd_test_phase "1 2 3" ;;
    # The interactive testbench dispatches go through launch.sh's own
    # kernel-aware helpers (which spawn cpcu_kernel inside a tmux
    # session as needed), not through run_tests.sh — `test-pca`,
    # `test-signal`, and `test-signal-demo` all become aliases for
    # the equivalent operating-mode commands so users get a usable
    # session out of the box without having to start the kernel
    # manually first.
    test-pca)               run_pca ;;
    test-nrf)               run_nrf "$@" ;;
    test-signal)            run_signal ;;
    test-signal-demo)       run_signal_demo ;;
    test-safety-demo)       cmd_test_phase "safety-demo" ;;
    test-system)            cmd_test_system "$@" ;;

    kernel|"")
        if [ -z "${MODE}" ] && [ -t 0 ] && [ -t 1 ]; then
            show_menu
        else
            run_kernel_only
        fi
        ;;
    tui)                    run_tui ;;
    collect)                run_collect ;;
    signal)                 run_signal ;;
    pca)                    run_pca ;;
    nrf)                    run_nrf "$@" ;;
    smoother)               run_smoother "$@" ;;
    menu)                   show_menu ;;
    ws)                     cmd_ws "$@" ;;

    attach)                 cmd_attach ;;
    stop)                   cmd_stop ;;

    grant-caps)             cmd_grant_caps ;;
    install-service)        cmd_install_service ;;
    install-ws-service)     cmd_install_ws_service ;;

    # v3.0 commands
    setup-audio)            cmd_setup_audio "$@" ;;
    setup-uart)             cmd_setup_uart "$@" ;;
    grip-tune)              run_grip_tune "$@" ;;
    calibrate)              run_calibrate "$@" ;;
    add-gesture)            run_add_gesture "$@" ;;
    remove-gesture)         cmd_remove_gesture "$@" ;;
    rename-gesture)         cmd_rename_gesture "$@" ;;
    edit-gesture)           cmd_edit_gesture "$@" ;;
    add-motor)              cmd_add_motor "$@" ;;
    remove-motor)           cmd_remove_motor "$@" ;;
    edit-motor)             cmd_edit_motor "$@" ;;
    rename-motor)           cmd_rename_motor "$@" ;;
    set-channels)           run_set_channels "$@" ;;
    set-model)              cmd_set_model "$@" ;;
    generate-cues)          run_generate_cues "$@" ;;
    audio)                  cmd_audio "$@" ;;
    show-config)            cmd_show_config ;;
    show-gestures)          cmd_show_gestures ;;
    reload)                 cmd_reload "$@" ;;

    *)
        err "Unknown command: ${MODE}"
        echo
        echo "Run './launch.sh help' for the full command reference."
        echo "Common commands: setup, build, check, test-sw, tui, ws, stop, help"
        exit 2
        ;;
esac


