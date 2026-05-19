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

    # runtime.json is THE #1 cause of "TUI doesn't attach" — the kernel
    # reads it at startup and aborts if any field is malformed or absent,
    # then the tmux session dies before the user has a chance to attach
    # and see why. Validate up front so the error message lands in their
    # current shell instead of inside a tmux pane that's already gone.
    local cfg_path=""
    for p in /opt/cpcu/config.json "${CPCU_ROOT}/config/runtime.json"; do
        if [ -r "$p" ]; then
            cfg_path="$p"
            break
        fi
    done
    if [ -z "${cfg_path}" ]; then
        err "════════════════════════════════════════════════════════════"
        err "  runtime.json NOT FOUND"
        err ""
        err "  cpcu_kernel needs config/runtime.json (or its installed"
        err "  symlink /opt/cpcu/config.json) and will crash without it."
        err "  Regenerate the defaults with:"
        err ""
        err "      ./launch.sh configure --reset --runtime"
        err ""
        err "  Then re-run ./launch.sh tui."
        err "════════════════════════════════════════════════════════════"
        fatal "Pre-flight aborted: missing runtime.json"
    fi
    log "  Runtime config: ${cfg_path}"

    if ! python3 -c "import numpy, scipy, joblib" 2>/dev/null; then
        warn "Python deps missing — DSP will run in feature-only mode."
        warn "Install with: ./launch.sh setup"
    fi

    # Resolve cpcu_dsp.py location and export CPCU_DSP_PATH so the kernel
    # doesn't have to guess. The kernel itself has a 3-way fallback
    # (installed → repo → legacy), but pre-resolving here gives a clear
    # error message in the user's shell instead of letting it surface as
    # a respawn loop inside the tmux KERNEL window.
    DSP_PATH=""
    for p in "${PYTHON_INSTALL_DIR}/cpcu_dsp.py" \
             "${CPCU_ROOT}/python/cpcu_dsp.py" \
             "${CPCU_ROOT}/cpcu_dsp.py"; do
        if [ -r "$p" ]; then
            DSP_PATH="$p"
            break
        fi
    done
    if [ -z "${DSP_PATH}" ]; then
        err "════════════════════════════════════════════════════════════"
        err "  cpcu_dsp.py NOT FOUND in any expected location:"
        err "    ${PYTHON_INSTALL_DIR}/cpcu_dsp.py    (installed)"
        err "    ${CPCU_ROOT}/python/cpcu_dsp.py      (repo source)"
        err "    ${CPCU_ROOT}/cpcu_dsp.py             (legacy)"
        err ""
        err "  Re-run:  ./launch.sh build"
        err "════════════════════════════════════════════════════════════"
        fatal "Pre-flight aborted: missing cpcu_dsp.py"
    fi
    export CPCU_DSP_PATH="${DSP_PATH}"
    log "  DSP script: ${DSP_PATH}"

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
        # Give tmux a moment to actually tear down — has-session returns
        # immediately but the server may still be cleaning up sockets.
        for i in 1 2 3 4 5; do
            tmux has-session -t "$SESSION_NAME" 2>/dev/null || break
            sleep 0.1
        done
    fi
    # Also kill any zombie tmux processes that may be wedged on stale
    # sockets from a previous crash — these can stop `new-session`
    # from registering even though `has-session` says no session exists.
    if pgrep -x tmux >/dev/null 2>&1 && \
       ! tmux list-sessions >/dev/null 2>&1; then
        warn "Found stale tmux server with no sessions — restarting it"
        tmux kill-server 2>/dev/null || true
        sleep 0.3
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

    # Wrap the kernel invocation in a holder shell that stays alive
    # after the kernel exits. Without this, an immediate kernel crash
    # (bad runtime.json, missing /opt/cpcu/config.json, segfault on
    # startup) kills the only window in the session, and tmux destroys
    # the session before we can apply `remain-on-exit on` — leaving the
    # caller with the misleading "tmux refused to create session" error.
    #
    # The holder also prints a clearly-visible exit-code banner so the
    # user sees the kernel's return value the moment they attach, instead
    # of having to dig through the kernel log.
    #
    # The trailing `exec bash --login` gives a usable shell in the pane
    # so the user can poke around (run `tail`, `cat`, etc.) without
    # having to swap to the SHELL window.
    local kernel_inner="cd ${CPCU_ROOT} && \
export CPCU_DSP_PATH=\"${CPCU_DSP_PATH:-}\" && \
taskset -c 0 ${BIN_DIR}/cpcu_kernel --log 2>&1 | tee -a ${kernel_log} ; \
rc=\$? ; \
printf '\\n\\033[31m========================================\\n' ; \
printf '[KERNEL exited with code %d]\\n' \$rc ; \
printf 'This pane stays open for inspection.\\n' ; \
printf 'Ctrl-b 1 = SHELL window, Ctrl-b d = detach.\\n' ; \
printf '========================================\\033[0m\\n\\n' ; \
exec bash --login"

    tmux new-session -d -s "$SESSION_NAME" -n "KERNEL" \
        -x "$_cols" -y "$_rows" \
        "bash -c '${kernel_inner}'"

    # Wait briefly for the new session to be reachable. tmux's set-option
    # calls below otherwise race with the daemon and emit harmless but
    # noisy "no server running" stderr warnings.
    for i in 1 2 3 4 5; do
        if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
            break
        fi
        sleep 0.1
    done

    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        err "tmux refused to create session '${SESSION_NAME}' — is tmux installed?"
        return 1
    fi

    # remain-on-exit keeps dead windows visible so the user can read
    # the kernel's crash output when something goes wrong, instead of
    # the whole session vanishing the moment cpcu_kernel exits.
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

    # Poll /dev/shm/cpcu_ipc up to 15 s. The kernel typically wins this
    # race in <300 ms when runtime.json is healthy, so a long wait means
    # something's wrong. Check the KERNEL window each iteration: if the
    # kernel process has died, bail immediately with the actual error
    # message instead of waiting out the full timeout.
    local attempts=30
    for i in $(seq 1 ${attempts}); do
        if [ -e /dev/shm/cpcu_ipc ]; then
            log "Shared memory ready after $((i*500))ms"
            sleep 1   # let kernel finish init (cpcu_io fork, capability drop)
            tmux_verify_kernel_alive || return 1
            return 0
        fi
        # Has the kernel window died? (remain-on-exit keeps it visible
        # but reports dead state via #{window_dead} format flag.)
        local dead
        dead=$(tmux list-windows -t "${SESSION_NAME}:KERNEL" \
                  -F '#{window_dead}' 2>/dev/null || echo "1")
        if [ "${dead}" = "1" ]; then
            err "════════════════════════════════════════════════════════════"
            err "  cpcu_kernel died during startup."
            err "  Last log lines:"
            err "════════════════════════════════════════════════════════════"
            tail -n 20 "${kernel_log}" 2>/dev/null | sed 's/^/   | /' >&2
            err "════════════════════════════════════════════════════════════"
            err "  Full log: ${kernel_log}"
            err "  To inspect interactively (window survives until you stop):"
            err "    ./launch.sh attach    # Ctrl-b 0 = KERNEL pane"
            err "    ./launch.sh stop      # then tear it down"
            err "════════════════════════════════════════════════════════════"
            # leave session alive on purpose so user can attach
            TMUX_OWNED=""
            return 1
        fi
        sleep 0.5
    done

    err "Kernel didn't bring up /dev/shm/cpcu_ipc within 15 s"
    err "  Inspect: ./launch.sh attach   (Ctrl-b 0 = KERNEL pane)"
    err "  Stop:    ./launch.sh stop"
    err "  Log:     ${kernel_log}"
    TMUX_OWNED=""   # leave session alive so user can investigate
    return 1
}

# Verify the kernel window is still alive *after* shared memory came up
# — catches the rare case where cpcu_io forks, runtime.json validation
# fails mid-init, and the kernel exits in the brief window between
# mmap'ing /dev/shm/cpcu_ipc and reaching the main loop. Without this
# check we'd open the TUI window onto a dead supervisor.
tmux_verify_kernel_alive() {
    local dead
    dead=$(tmux list-windows -t "${SESSION_NAME}:KERNEL" \
              -F '#{window_dead}' 2>/dev/null || echo "1")
    if [ "${dead}" = "1" ]; then
        err "Kernel crashed right after mapping /dev/shm/cpcu_ipc."
        err "  Inspect: ./launch.sh attach  (Ctrl-b 0)"
        TMUX_OWNED=""
        return 1
    fi
    return 0
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
    export CPCU_DSP_PATH="${CPCU_DSP_PATH:-}"
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
    export CPCU_DSP_PATH="${CPCU_DSP_PATH:-}"
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
    local sig_args
    sig_args="$(signal_cli_args)"
    tmux_add_window "SIGNAL" "${sig_bin} ${sig_args}"
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

# Build the --ch-names and --active-channels CLI string for
# signal_testbench by reading gestures.json. Each gesture group contributes
# its emg_channels.{active,names}; if multiple groups overlap on a channel,
# the first group's name wins (deterministic order matches dict insertion
# order from gestures.json, which is what the user sees in show-config).
# Empty on any error so the testbench falls back to its "ch<N>" defaults.
signal_cli_args() {
    [ -r "${GS}" ] || return 0
    python3 - "${GS}" << 'PYEOF' 2>/dev/null || true
import json, sys
try:
    with open(sys.argv[1]) as f: g = json.load(f)
    names  = [""] * 8
    active = set()
    for grp in g.get("gesture_groups", {}).values():
        ec   = grp.get("emg_channels", {})
        acts = ec.get("active", [])
        nms  = ec.get("names",  [])
        for i, ch in enumerate(acts):
            if 0 <= ch < 8:
                active.add(ch)
                if i < len(nms) and not names[ch]:
                    names[ch] = nms[i]
    name_csv = ",".join(names)
    act_csv  = ",".join(str(c) for c in sorted(active))
    parts = []
    if any(names): parts += ["--ch-names",        name_csv]
    if active:     parts += ["--active-channels", act_csv]
    print(" ".join(parts))
except Exception:
    pass
PYEOF
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
# and sanity-checking that the testbench builds correctly. Still gets
# --ch-names/--active-channels so the demo plot's labels match the
# user's config too.
run_signal_demo() {
    preflight_signal
    local sig_bin="${RESOLVED_BIN}"
    log "Mode: SIGNAL DEMO (synthetic data; no kernel, no shared memory)"
    log "  Inside the TUI:  w cycle wave types  [/] change frequency  q quit"
    sleep 0.5
    trap - EXIT INT TERM
    local sig_args
    sig_args="$(signal_cli_args)"
    cd "$(dirname "${sig_bin}")"
    exec "${sig_bin}" --demo ${sig_args}
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

    # ── Pull servo names from gestures.json so the TUI shows the user's
    #    current names instead of compile-time strings. Sort by pca_ch so
    #    the CSV index matches the testbench's logical servo index. Silent
    #    fallback if gestures.json is missing or unparseable.
    local names_csv=""
    if [ -r "${GS}" ]; then
        names_csv=$(python3 - "${GS}" << 'PYEOF' 2>/dev/null || true
import json, sys
try:
    with open(sys.argv[1]) as f: g = json.load(f)
    sc = sorted(g.get("servo_channels", {}).items(),
                key=lambda x: x[1].get("pca_ch", 0))
    print(",".join(n for n, _ in sc))
except Exception:
    pass
PYEOF
)
    fi
    local names_arg=""
    [ -n "${names_csv}" ] && names_arg="--names ${names_csv}"
    [ -n "${names_csv}" ] && log "Servo names from gestures.json: ${names_csv}"

    cd "$(dirname "${pca_bin}")"
    "${pca_bin}" ${cfg_arg} ${names_arg}
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

    if [ "${rc}" -eq 0 ]; then
        log "  Next steps:"
        log "    ./launch.sh show-config       # verify saved values"
        log "    ./launch.sh tui               # run live with new limits"
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
    local sig_args
    sig_args="$(signal_cli_args)"
    cd "$(dirname "${sig_bin}")" && "${sig_bin}" ${sig_args}
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

    ok "Vendor + rebuild complete."
    log "  Next steps:"
    log "    ./launch.sh check               # verify everything is ready"
    log "    ./launch.sh ws                  # launch the web dashboard"
    log "    ./launch.sh tui --with-ws       # TUI + browser dashboard together"
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

  ORDER OF OPERATIONS  (do these in this sequence on a fresh Pi)
  ─────────────────────────────────────────────────────────────
   ┌── Stage 1 — One-time Pi setup ─────────────────────────────┐
   │   1. ./launch.sh setup            Pi config + isolcpus     │
   │   2. sudo reboot                  (required after step 1)  │
   │   3. ./launch.sh setup-audio      I2S DAC for voice cues   │
   │   4. ./launch.sh setup-uart       UART debug (optional)    │
   │   5. sudo reboot                  (if 3 or 4 changed boot) │
   └────────────────────────────────────────────────────────────┘
   ┌── Stage 2 — Build the tree (once per checkout) ────────────┐
   │   6. ./launch.sh vendor           fetch Mongoose (web srv) │
   │   7. ./launch.sh build            cmake + make + install   │
   │   8. ./launch.sh generate-cues    voice .wav files         │
   │   9. ./launch.sh check            verify everything ready  │
   └────────────────────────────────────────────────────────────┘
   ┌── Stage 3 — Configure (only if defaults need changing) ────┐
   │  10. ./launch.sh configure --show      see compile knobs   │
   │  11. ./launch.sh configure --reset --runtime  regen JSON   │
   │  12. ./launch.sh show-config           print full config   │
   └────────────────────────────────────────────────────────────┘
   ┌── Stage 4 — Verify subsystems (before running live) ───────┐
   │  13. ./launch.sh test-sw          software-only tests      │
   │  14. ./launch.sh test-ipc         + IPC offset validation  │
   │  15. ./launch.sh test-hw          + Pi hardware probes     │
   │  16. ./launch.sh test-nrf         NRF24L01+ self-test      │
   │  17. ./launch.sh test-pca         interactive servo cal    │
   │  18. ./launch.sh test-signal-demo signal TUI (synthetic)   │
   │  19. ./launch.sh test-signal      signal TUI (live, BSAU)  │
   │  20. ./launch.sh test-safety-demo fault injection demo     │
   └────────────────────────────────────────────────────────────┘
   ┌── Stage 5 — Calibrate per operator & tune motors ──────────┐
   │  21. ./launch.sh test-pca         set MIN/MAX/BIAS/smoother│
   │  22. ./launch.sh grip-tune        gripper firmness         │
   │  23. ./launch.sh calibrate        rest noise + velocity    │
   │      ./launch.sh calibrate --operator NAME  (per-user)     │
   └────────────────────────────────────────────────────────────┘
   ┌── Stage 6 — Customise gestures / motors / audio ───────────┐
   │  24. ./launch.sh add-group  NAME      new classifier       │
   │  25. ./launch.sh add-motor  NAME CH   new servo            │
   │  26. ./launch.sh add-gesture [group]  guided wizard        │
   │  27. ./launch.sh set-model  PATH      pick .pkl            │
   │  28. ./launch.sh audio voice          enable voice cues    │
   │  29. ./launch.sh set-channels 0 1 2   EMG channel mask     │
   │  30. ./launch.sh show-config          verify changes       │
   └────────────────────────────────────────────────────────────┘
   ┌── Stage 7 — Run the live system ───────────────────────────┐
   │  31. ./launch.sh test-system      requirements (live)      │
   │  32. ./launch.sh tui --audio --with-ws   daily operation   │
   │      ./launch.sh signal                live signal scope   │
   │      ./launch.sh collect               dataset capture     │
   │      ./launch.sh ws                    web dashboard only  │
   │      ./launch.sh smoother              servo motion exer.  │
   └────────────────────────────────────────────────────────────┘
   ┌── Stage 8 — Auto-start at boot (optional) ─────────────────┐
   │  33. ./launch.sh install-service       kernel as systemd   │
   │  34. ./launch.sh install-ws-service    web dashboard       │
   └────────────────────────────────────────────────────────────┘

  ─── REFERENCE — every command grouped by purpose ───────────

  SETUP & BUILD
  ─────────────────────────────────────────────────────────────
    ./launch.sh setup                         Pi one-time config
    ./launch.sh setup-audio                   I2S DAC + speaker
    ./launch.sh setup-uart                    UART debug output
    ./launch.sh vendor                        fetch Mongoose
    ./launch.sh build [--clean]               compile + install
    ./launch.sh generate-cues                 voice .wav files
    ./launch.sh check                         verify readiness

  RUNNING THE SYSTEM
  ─────────────────────────────────────────────────────────────
    ./launch.sh tui                           TUI dashboard
    ./launch.sh tui --audio                   + voice/tone feedback
    ./launch.sh tui --uart                    + UART debug to PC
    ./launch.sh tui --with-ws                 + web dashboard
    ./launch.sh tui --audio --uart --with-ws  all features
    ./launch.sh tui --operator NAME           operator velocity profile
    ./launch.sh ws                            web dashboard only
    ./launch.sh kernel                        kernel only (systemd path)
    ./launch.sh collect                       dataset capture mode
    ./launch.sh signal                        signal testbench (live)
    ./launch.sh smoother                      servo motion exerciser
    ./launch.sh pca                           direct PCA9685 (alias for test-pca)
    ./launch.sh nrf                           NRF utility/self-test
    ./launch.sh menu                          interactive picker
    ./launch.sh attach                        re-attach tmux session
    ./launch.sh stop                          stop (safe servo shutdown)

  TESTING
  ─────────────────────────────────────────────────────────────
    ./launch.sh test-sw                       software tests (233 PASS)
    ./launch.sh test-ipc                      + IPC validation
    ./launch.sh test-hw                       + hardware probes
    ./launch.sh test-pca                      interactive servo TUI
                                              (--names from gestures.json)
    ./launch.sh test-nrf                      NRF self-test + packet rx
    ./launch.sh test-signal                   live signal integrity
                                              (--ch-names from gestures.json)
    ./launch.sh test-signal-demo              synthetic signal (no BSAU)
    ./launch.sh test-safety-demo              fault injection demo
    ./launch.sh test-system                   live requirements check
                                              (needs running kernel)

  CALIBRATION & TUNING
  ─────────────────────────────────────────────────────────────
    ./launch.sh grip-tune                     gripper firmness wizard
    ./launch.sh calibrate                     rest noise + velocity (0-10)
    ./launch.sh calibrate --operator NAME     per-operator profile
    ./launch.sh calibrate --rest-only         noise floor only
    ./launch.sh calibrate --vel-only          velocity preference only

  EMG CHANNELS
  ─────────────────────────────────────────────────────────────
    ./launch.sh set-channels 0 1 2            3-channel mode
    ./launch.sh set-channels 0 1 2 3 4        5-channel mode
    ./launch.sh set-channels 0 1 2 3 4 5 6 7  8-channel (full)
                                               validates model match

  SERVO MOTORS
  ─────────────────────────────────────────────────────────────
    ./launch.sh add-motor    Thumb 6          add on PCA channel 6
    ./launch.sh edit-motor   Gripper          edit limits (min/max/neutral)
    ./launch.sh rename-motor Gripper Claw     rename (updates all refs)
    ./launch.sh remove-motor Thumb            remove a servo

  GESTURE GROUPS  (v5: multiple classifiers, each with own EMG + model)
  ─────────────────────────────────────────────────────────────
    ./launch.sh show-config                   show all groups + gestures
    ./launch.sh show-gestures                 (alias of show-config)
    ./launch.sh add-group    right_arm        create group (asks EMG + model)
    ./launch.sh remove-group right_arm        delete entire group
    ./launch.sh rename-group right_arm right  rename a group

  GESTURES  (each gesture belongs to a group)
  ─────────────────────────────────────────────────────────────
    ./launch.sh add-gesture                   add to first group (wizard)
    ./launch.sh add-gesture right_arm         add to specific group
    ./launch.sh edit-gesture flex             change servo mapping
    ./launch.sh rename-gesture hand grip      rename gesture
    ./launch.sh remove-gesture right_arm flex delete gesture from group

  AUDIO FEEDBACK  (PCM5102A + PAM8403 + speaker)
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
    ./launch.sh configure                     compile-time settings (interactive)
    ./launch.sh configure --show              show all compile values
    ./launch.sh configure --diff              changes from defaults
    ./launch.sh configure --reset             restore defaults
    ./launch.sh configure --reset --runtime   also regenerate runtime.json
    ./launch.sh configure --<knob> <value>    set one knob

  RELOAD  (apply config changes without full restart)
  ─────────────────────────────────────────────────────────────
    ./launch.sh reload                        all (runtime + DSP + audio)
    ./launch.sh reload --dsp                  DSP pipeline only
    ./launch.sh reload --audio                audio daemon only

  UART DEBUG  (to host PC via USB-UART adapter)
  ─────────────────────────────────────────────────────────────
    ./launch.sh setup-uart                    enable UART on Pi 5
    ./launch.sh tui --uart                    run with UART debug
    Host: python3 scripts/uart_monitor.py --port /dev/ttyUSB0
    Host: python3 scripts/uart_monitor.py --port COM3 --log data.csv

  SERVICES  (auto-start at boot)
  ─────────────────────────────────────────────────────────────
    ./launch.sh install-service               kernel systemd unit
    ./launch.sh install-ws-service            web dashboard service
    ./launch.sh grant-caps                    re-apply RT capabilities

  META
  ─────────────────────────────────────────────────────────────
    ./launch.sh help [<command>]              this message / topic
    ./launch.sh version                       version string

  COMMON RECIPES
  ─────────────────────────────────────────────────────────────

    # daily operation:
    ./launch.sh tui --audio --with-ws

    # full setup of a fresh Pi:
    ./launch.sh setup && sudo reboot                # then later:
    ./launch.sh setup-audio && ./launch.sh vendor
    ./launch.sh build && ./launch.sh generate-cues
    ./launch.sh check && ./launch.sh tui --audio

    # add a servo motor + use it in a gesture:
    ./launch.sh add-motor    Thumb 6
    ./launch.sh edit-motor   Thumb
    ./launch.sh add-gesture                       # wizard asks which motors
    ./launch.sh collect                           # record training data
    ./launch.sh set-model    models/new.pkl       # after retraining
    ./launch.sh reload

    # multi-group setup (right arm + left arm):
    ./launch.sh add-group right_arm               # EMG 0,1,2 + right_arm.pkl
    ./launch.sh add-group left_arm                # EMG 3,4,5 + left_arm.pkl
    ./launch.sh add-gesture right_arm
    ./launch.sh remove-gesture right_arm flex

    # calibrate for a new operator:
    ./launch.sh calibrate --operator ali
    ./launch.sh tui --operator ali --audio

    # rename things (all gesture refs follow):
    ./launch.sh rename-motor Gripper Claw

    # switch audio mode:
    ./launch.sh audio freq && ./launch.sh reload --audio

    # tune gripper firmness:
    ./launch.sh grip-tune

    # servo calibration (syncs to gestures.json on exit):
    ./launch.sh test-pca

    # UART debug to laptop:
    ./launch.sh setup-uart && sudo reboot
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
run_add_gesture() {
    local group="${1:-}"
    if [ -n "$group" ]; then
        # check if first arg is an existing group name
        local is_group
        is_group=$(python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
print('yes' if '${group}' in g.get('gesture_groups', {}) else 'no')
" 2>/dev/null)
        if [ "$is_group" = "yes" ]; then
            shift
            CPCU_GESTURE_GROUP="$group" _run_script add_gesture.sh "$@"
            return $?
        fi
    fi
    _run_script add_gesture.sh "$@"
}
run_generate_cues()  { _run_script generate_voice_cues.sh "$@"; }
run_set_channels()   { _run_script set_channels.sh "$@"; }
cmd_setup_audio() {
    _run_script setup_audio.sh "$@"
    local rc=$?
    if [ "${rc}" -eq 0 ]; then
        ok "Audio setup complete."
        log "  Next steps:"
        log "    sudo reboot                       # if boot config changed"
        log "    ./launch.sh generate-cues         # render voice .wav files"
        log "    ./launch.sh audio test            # verify speaker"
        log "    ./launch.sh tui --audio           # run live with cues"
    fi
    return $rc
}
cmd_setup_uart() {
    _run_script setup_uart.sh "$@"
    local rc=$?
    if [ "${rc}" -eq 0 ]; then
        ok "UART setup complete."
        log "  Next steps:"
        log "    sudo reboot                       # if boot config changed"
        log "    ./launch.sh tui --uart            # run with UART debug stream"
        log "    # on the host PC, in parallel:"
        log "    python3 scripts/uart_monitor.py --port /dev/ttyUSB0"
    fi
    return $rc
}

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
print(f"  Schema:     v{g.get('schema_version','?')}")
print(f"  Audio:      {g.get('audio_mode','off')} @ {g.get('audio_volume_pct',80)}%")
gg = g.get("gesture_groups", {})
print(f"  Groups:     {len(gg)} gesture group(s)")

h("SERVO MOTORS")
sep()
fmt = "  {:<12s} PCA={:<2d}  range=[{:>4d}, {:>4d}]  neutral={:>4d}"
for name, sd in g.get("servo_channels", {}).items():
    print(fmt.format(name, sd["pca_ch"], sd["min_us"], sd["max_us"], sd["neutral_us"]))

h("GESTURE GROUPS")
sep()
if not gg:
    print("  (none — run ./launch.sh add-group)")
for grp_name, grp in gg.items():
    ec = grp.get("emg_channels", {})
    chs = ec.get("active", [])
    names = ec.get("names", [])
    model = grp.get("model_path", "-")
    cc = grp.get("confidence", {})
    hy = grp.get("hysteresis", {})
    print(f"\n  {B}{grp_name}{N}")
    ch_str = ", ".join(f"ch{c}={names[i] if i<len(names) else '?'}" for i,c in enumerate(chs))
    print(f"    EMG:        {ch_str}")
    print(f"    Model:      {model}")
    print(f"    Confidence: {cc.get('curve','?')} floor={cc.get('floor_pct','?')}% ceil={cc.get('ceil_pct','?')}%")
    print(f"    Hysteresis: r→a={hy.get('rest_to_active','?')} a→a={hy.get('active_to_active','?')} a→r={hy.get('active_to_rest','?')}")
    gestures = grp.get("gestures", {})
    if gestures:
        print(f"    {'NAME':<14s} {'MODE':<9s} {'SERVOS':<28s} {'AUDIO'}")
        for gn, gd in gestures.items():
            mode = gd.get("mode", "?")
            chs2 = gd.get("channels", {})
            ch_s = ", ".join(f"{s}={d.get('rate_us_s',0)}" for s,d in chs2.items()) or "-"
            a_s = gd.get("audio", {}).get("voice", "-")
            print(f"    {gn:<14s} {mode:<9s} {ch_s:<28s} {a_s}")
    else:
        print("    (no gestures — run ./launch.sh add-gesture)")

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

# ── add-group ──
cmd_add_group() {
    local name="${1:-}"
    if [ -z "$name" ]; then
        err "Usage: ./launch.sh add-group <name>"
        echo "  Example: ./launch.sh add-group gesture_2"
        exit 1
    fi
    read -rp "  EMG channels (comma-separated, e.g. 3,4,5): " channels
    read -rp "  Model path (e.g. models/left_arm.pkl): " mpath
    [ -z "$mpath" ] && mpath="models/model.pkl"
    python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
gg = g.setdefault('gesture_groups', {})
if '${name}' in gg: print(f\"  Already exists.\"); sys.exit(1)
chs = [int(c.strip()) for c in '${channels}'.split(',') if c.strip()]
gg['${name}'] = {
    'emg_channels': {'active': chs, 'names': [f'ch{c}' for c in chs]},
    'model_path': '${mpath}',
    'confidence': {'curve': 'quadratic', 'floor_pct': 40, 'ceil_pct': 85},
    'hysteresis': {'rest_to_active': 4, 'active_to_rest': 2, 'active_to_active': 6},
    'gestures': {'rest': {'mode': 'freeze', 'audio': {'voice': 'voice_rest_${name}', 'freq_hz': 330, 'freq_ms': 60}}}
}
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
print(f\"  \033[32m✓\033[0m Created '${name}' with EMG {chs}\")
"
}

# ── remove-group ──
cmd_remove_group() {
    local name="${1:-}"
    if [ -z "$name" ]; then
        err "Usage: ./launch.sh remove-group <name>"
        python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
print('  Groups: ' + ', '.join(g.get('gesture_groups',{}).keys()))
"
        exit 1
    fi
    python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
gg = g.get('gesture_groups', {})
if '${name}' not in gg: print(f\"  Not found.\"); sys.exit(1)
del gg['${name}']
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
print(f\"  \033[32m✓\033[0m Removed '${name}'.\")
"
}

# ── rename-group ──
cmd_rename_group() {
    local old="${1:-}" new="${2:-}"
    if [ -z "$old" ] || [ -z "$new" ]; then
        err "Usage: ./launch.sh rename-group <old_name> <new_name>"
        python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
print('  Groups: ' + ', '.join(g.get('gesture_groups',{}).keys()))
"
        exit 1
    fi
    python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
gg = g.get('gesture_groups', {})
if '${old}' not in gg: print(f\"  '${old}' not found.\"); sys.exit(1)
if '${new}' in gg: print(f\"  '${new}' already exists.\"); sys.exit(1)
# preserve order: rebuild dict
new_gg = {}
for k, v in gg.items():
    new_gg['${new}' if k == '${old}' else k] = v
g['gesture_groups'] = new_gg
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
print(f\"  \033[32m✓\033[0m Renamed '${old}' → '${new}'.\")
"
}

# ── remove-gesture ──
cmd_remove_gesture() {
    local group="${1:-}" name="${2:-}"
    if [ -z "$group" ] || [ -z "$name" ]; then
        err "Usage: ./launch.sh remove-gesture <group> <gesture>"
        echo "  Example: ./launch.sh remove-gesture gesture_0 flex"
        python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
for gn, gd in g.get('gesture_groups', {}).items():
    print(f'  {gn}: {list(gd.get(\"gestures\",{}).keys())}')
"
        exit 1
    fi
    python3 -c "
import json, sys
with open('${GS}') as f: g = json.load(f)
gg = g.get('gesture_groups', {})
if '${group}' not in gg: print(f\"  Group not found.\"); sys.exit(1)
gs = gg['${group}'].get('gestures', {})
if '${name}' not in gs: print(f\"  '${name}' not in ${group}.\"); sys.exit(1)
if '${name}' == 'rest': print('  Cannot remove rest.'); sys.exit(1)
del gs['${name}']
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
print(f\"  \033[32m✓\033[0m Removed '${name}' from ${group}.\")
"
    rm -f "${GS%/*}/audio_cues/voice_${name}.wav" "${GS%/*}/audio_cues/_gen_${name}_"*.wav 2>/dev/null
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
# v5: usage is `rename-gesture <group> <old> <new>`. The 2-arg legacy
# form `rename-gesture <old> <new>` is still accepted: it scans all
# groups, renames only if exactly one match is found, otherwise asks
# the user to disambiguate by passing the group name explicitly.
cmd_rename_gesture() {
    local a1="${1:-}" a2="${2:-}" a3="${3:-}"
    local group="" old="" new=""
    if [ -n "$a3" ]; then
        group="$a1"; old="$a2"; new="$a3"
    elif [ -n "$a2" ]; then
        # legacy 2-arg form — let the python helper find the group
        old="$a1"; new="$a2"
    else
        err "Usage: ./launch.sh rename-gesture <group> <old> <new>"
        echo "  Example: ./launch.sh rename-gesture right_arm flex bend"
        python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
for gn, gd in g.get('gesture_groups', {}).items():
    print(f'  {gn}: {list(gd.get(\"gestures\",{}).keys())}')
"
        exit 1
    fi
    python3 << PYEOF
import json, sys
with open("${GS}") as f: g = json.load(f)
gg = g.get("gesture_groups", {})
if not gg:
    print("  No gesture_groups in gestures.json (v5 schema required).")
    sys.exit(1)

group, old, new = "${group}", "${old}", "${new}"

# Resolve group: explicit > unique-match across all groups.
if not group:
    matches = [gn for gn, gd in gg.items()
               if old in gd.get("gestures", {})]
    if not matches:
        print(f"  '{old}' not found in any group.")
        sys.exit(1)
    if len(matches) > 1:
        print(f"  '{old}' exists in multiple groups: {matches}")
        print(f"  Run: ./launch.sh rename-gesture <group> {old} {new}")
        sys.exit(1)
    group = matches[0]

if group not in gg:
    print(f"  Group '{group}' not found. Available: {list(gg.keys())}")
    sys.exit(1)

gestures = gg[group].setdefault("gestures", {})
if old not in gestures:
    print(f"  '{old}' not in group '{group}'. Available: {list(gestures.keys())}")
    sys.exit(1)
if new in gestures:
    print(f"  '{new}' already exists in '{group}'.")
    sys.exit(1)
if old == "rest":
    print(f"  Cannot rename 'rest' (reserved).")
    sys.exit(1)

# Preserve insertion order: rebuild the dict so 'new' takes 'old's slot.
new_gestures = {}
for k, v in gestures.items():
    new_gestures[new if k == old else k] = v
gg[group]["gestures"] = new_gestures

with open("${GS}", "w") as f: json.dump(g, f, indent=4)
print(f"  \033[32m✓\033[0m Renamed '{old}' → '{new}' in group '{group}'.")
print(f"  \033[33m⚠\033[0m Model classes must match — retrain if the class label changed.")
PYEOF
}

# ── edit-gesture (change servo mapping for existing gesture) ──
# v5: usage is `edit-gesture <group> <name>`. 1-arg legacy form
# `edit-gesture <name>` works when the gesture exists in exactly one
# group. Servo names accept the legacy `S#_Foo` prefix (auto-stripped),
# same convention as add_gesture.sh, and are validated against
# servo_channels so a typo can't silently break the gesture.
cmd_edit_gesture() {
    local a1="${1:-}" a2="${2:-}"
    local group="" name=""
    if [ -n "$a2" ]; then
        group="$a1"; name="$a2"
    elif [ -n "$a1" ]; then
        name="$a1"  # legacy — resolve group below
    else
        err "Usage: ./launch.sh edit-gesture <group> <name>"
        echo "  Example: ./launch.sh edit-gesture right_arm flex"
        python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
for gn, gd in g.get('gesture_groups', {}).items():
    print(f'  {gn}: {list(gd.get(\"gestures\",{}).keys())}')
"
        exit 1
    fi

    # Resolve group + dump current state. The script's stdout here also
    # captures the resolved group so the shell can read it back.
    local resolved
    resolved=$(python3 << PYEOF
import json, sys
with open("${GS}") as f: g = json.load(f)
gg = g.get("gesture_groups", {})
group, name = "${group}", "${name}"
if not group:
    matches = [gn for gn, gd in gg.items()
               if name in gd.get("gestures", {})]
    if not matches:
        print(f"ERR:'{name}' not found in any group.")
        sys.exit(1)
    if len(matches) > 1:
        print(f"ERR:'{name}' in multiple groups: {matches}. Pass the group explicitly.")
        sys.exit(1)
    group = matches[0]
if group not in gg or name not in gg[group].get("gestures", {}):
    print(f"ERR:'{name}' not in group '{group}'.")
    sys.exit(1)
gd       = gg[group]["gestures"][name]
print(f"GROUP={group}")
print(f"  Gesture:        {name}  (group: {group}, mode: {gd.get('mode','?')})")
print(f"  Available motors: {list(g.get('servo_channels',{}).keys())}")
print(f"  Current channels: {gd.get('channels',{})}")
PYEOF
)
    local rc=$?
    if [ $rc -ne 0 ] || echo "${resolved}" | grep -q '^ERR:'; then
        echo "${resolved}" | sed 's/^ERR://'
        exit 1
    fi
    # Print everything the python helper produced except the GROUP= line,
    # then extract the resolved group name for the write step.
    echo "${resolved}" | grep -v '^GROUP='
    group=$(echo "${resolved}" | sed -n 's/^GROUP=//p')

    echo "  Enter motor mappings (MotorName Rate), empty line to finish:"
    echo "  Legacy 'S#_Name' tokens are auto-stripped."
    local channels="{"
    local first=1
    while true; do
        read -rp "  Motor Rate: " line
        [ -z "$line" ] && break
        local mname rate snap
        mname=$(echo "$line" | awk '{print $1}')
        rate=$(echo "$line"  | awk '{print $2}')
        [ -z "$rate" ] && { echo "  Format: MotorName Rate"; continue; }
        # Strip legacy S#_ prefix
        local m_stripped
        m_stripped=$(echo "${mname}" | sed -E 's/^S[0-9]+_//')
        [ "${m_stripped}" != "${mname}" ] && \
            echo "  ↳ stripped to '${m_stripped}'"
        mname="${m_stripped}"
        # Validate against current servo_channels
        if ! python3 - "${GS}" "${mname}" << 'PYV' 2>/dev/null
import json, sys
with open(sys.argv[1]) as f: g = json.load(f)
sys.exit(0 if sys.argv[2] in g.get("servo_channels", {}) else 1)
PYV
        then
            warn "'${mname}' isn't a known motor — skipping."
            continue
        fi
        read -rp "  ${mname} snap? (y/n) [n]: " sn
        snap="False"
        [[ "$sn" =~ ^[yY] ]] && snap="True"
        [ $first -eq 0 ] && channels="${channels},"
        channels="${channels} \"${mname}\": {\"rate_us_s\": ${rate}, \"snap\": ${snap}}"
        first=0
    done
    channels="${channels} }"
    if [ "$first" -eq 1 ]; then
        echo "  No changes."
        return
    fi

    python3 -c "
import json
with open('${GS}') as f: g = json.load(f)
# The inline channels dict uses Python booleans (True/False); convert
# them to JSON booleans during dump.
g['gesture_groups']['${group}']['gestures']['${name}']['channels'] = ${channels}
with open('${GS}', 'w') as f: json.dump(g, f, indent=4)
print('  \033[32m✓\033[0m Updated ${name} in group ${group}.')
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
    add-group)              cmd_add_group "$@" ;;
    remove-group)           cmd_remove_group "$@" ;;
    rename-group)           cmd_rename_group "$@" ;;
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


