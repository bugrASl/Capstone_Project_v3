/* ═══════════════════════════════════════════════════════════════════
 *  cpcu_io.c — v3.0 CHANGES
 *
 *  Two patches to the existing main loop:
 *    PATCH A: After IPC_ReadMotorCmd, apply smoother overrides + snap
 *    PATCH B: Timestamp SMOOTH_Update + PCA writes, publish to latency
 * ═══════════════════════════════════════════════════════════════════ */

/* ── PATCH A: Insert AFTER line 578 (SMOOTH_SetAllTargets) ──
 *
 * Current code:
 *   PCA_SafetyClamp(&pca, servo_us);
 *   SMOOTH_SetAllTargets(&smooth, servo_us);
 *
 * Add this block right after SMOOTH_SetAllTargets:
 */

/* v3.0: per-gesture smoother overrides from motor_cmd.
 * DSP writes these alongside servo targets. Zero = use
 * global config, non-zero = override for this gesture. */
{
    uint16_t v_ovr[IPC_NUM_SERVOS], a_ovr[IPC_NUM_SERVOS];
    uint8_t  snap;

    /* read overrides from motor command (same seqlock read) */
    memcpy(v_ovr, ipc.motor->smooth_velocity_override, sizeof(v_ovr));
    memcpy(a_ovr, ipc.motor->smooth_accel_override,    sizeof(a_ovr));
    snap = ipc.motor->snap_flags;

    for(int s = 0; s < PCA_SERVO_COUNT; s++)
    {
        /* velocity: gesture override > global config > default */
        if(v_ovr[s] > 0)
            SMOOTH_SetVelocity(&smooth, s, v_ovr[s]);
        else if(cfg_cache.smooth_velocity_us_per_s[s] > 0)
            SMOOTH_SetVelocity(&smooth, s, cfg_cache.smooth_velocity_us_per_s[s]);

        /* accel: same priority */
        if(a_ovr[s] > 0)
            SMOOTH_SetAccel(&smooth, s, a_ovr[s]);
        else if(cfg_cache.smooth_accel_us_per_s2[s] > 0)
            SMOOTH_SetAccel(&smooth, s, cfg_cache.smooth_accel_us_per_s2[s]);

        /* snap mode: bypass smoother for this servo */
        bool should_snap = (snap >> s) & 1;
        SMOOTH_SetEnabled(&smooth, s, !should_snap);
    }
}


/* ── PATCH B: Wrap SMOOTH_Update + PCA writes with timestamps ──
 *
 * Current code (around line 583):
 *   SMOOTH_Update(&smooth, servo_dt);
 *   ...
 *   (PCA write loop)
 *
 * Replace with:
 */

/* v3.0: timestamp the smoother + I2C path */
uint64_t t_smooth_start = t;  /* 't' is already the loop timestamp */

SMOOTH_Update(&smooth, servo_dt);

/* ... (existing PCA write loop unchanged) ... */

/* After the PCA write loop completes, before the gripper watchdog: */
{
    uint64_t t_smooth_end = monotonic_us();
    uint32_t smoother_us = (uint32_t)(t_smooth_end - t_smooth_start);
    atomic_store_explicit(&ipc.latency->t_smoother,
                          smoother_us, memory_order_relaxed);
}


/* ═══════════════════════════════════════════════════════════════════
 * NOTE: The snap_flags bitmask approach:
 *
 *   bit 0 = S0_Base, bit 1 = S1_Elbow, ..., bit 5 = S5_Gripper
 *
 * cpcu_dsp.py sets bit 5 when gestures.json has "snap": true on
 * S5_Gripper for the active gesture. When the gesture changes, dsp
 * publishes the new snap_flags for the new gesture, and cpcu_io
 * applies it on the next motor_cmd read.
 *
 * SMOOTH_SetEnabled(false) bypasses the trapezoidal profile —
 * the servo immediately receives the target PWM value. The SG90's
 * internal gearbox provides mechanical damping.
 *
 * On gesture transition back to a non-snap gesture, the bit clears,
 * SMOOTH_SetEnabled(true) re-engages the profile, and the servo
 * resumes smooth motion from its current position.
 * ═══════════════════════════════════════════════════════════════════ */
