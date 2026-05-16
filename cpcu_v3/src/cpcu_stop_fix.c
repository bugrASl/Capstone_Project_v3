## ═══════════════════════════════════════════════════════════════════
##  STOP COMMAND FIX — safe servo shutdown before tmux kill
##
##  PROBLEM: tmux kill-session sends SIGHUP, but cpcu_io only catches
##  SIGTERM/SIGINT. SIGHUP kills cpcu_io instantly without running
##  PCA_SetAllNeutral + PCA_AllOff. Arm stays powered in last position.
##
##  FIX: Send SIGTERM to kernel → kernel SIGTERMs children → cpcu_io
##  runs cleanup (neutral + alloff) → wait → then kill tmux.
## ═══════════════════════════════════════════════════════════════════

## REPLACE the existing cmd_stop() in launch.sh with:

cmd_stop() {
    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "No active session."
        # kill any orphaned processes
        pkill -f cpcu_kernel 2>/dev/null || true
        pkill -f cpcu_dsp.py 2>/dev/null || true
        pkill -f cpcu_audio_daemon.py 2>/dev/null || true
        clear_ws_info
        return
    fi

    log "Stopping system (safe servo shutdown)..."

    # step 1: SIGTERM to kernel → it SIGTERMs cpcu_io and cpcu_dsp
    # cpcu_io catches SIGTERM, exits main loop, runs:
    #   PCA_SetAllNeutral (servos to 1500µs)
    #   sleep_ms(300)     (wait for servos to reach neutral)
    #   PCA_AllOff        (disable all PWM, servos go limp)
    #   NRF_PowerDown     (radio off)
    pkill -TERM cpcu_kernel 2>/dev/null || true
    pkill -TERM -f cpcu_dsp.py 2>/dev/null || true
    pkill -TERM -f cpcu_audio_daemon.py 2>/dev/null || true

    # step 2: wait for cpcu_io cleanup (needs ~500ms for neutral + alloff)
    log "Waiting for servo shutdown..."
    sleep 1.5

    # step 3: kill tmux session (cleanup any remaining processes)
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

    # step 4: verify nothing survived
    sleep 0.5
    pkill -9 -f cpcu_kernel 2>/dev/null || true
    pkill -9 -f cpcu_dsp.py 2>/dev/null || true

    clear_ws_info
    ok "Stopped. Servos neutral and disabled."
}

## ═══════════════════════════════════════════════════════════════════
##  ALSO: Add SIGHUP to cpcu_io.c signal handler as a safety net
##
##  In cpcu_io.c, after line 247:
##    signal(SIGINT,  on_signal);
##    signal(SIGTERM, on_signal);
##
##  ADD:
##    signal(SIGHUP,  on_signal);
##
##  This ensures cpcu_io runs its cleanup even if SIGHUP arrives
##  directly (e.g., from a terminal close or tmux crash).
## ═══════════════════════════════════════════════════════════════════
