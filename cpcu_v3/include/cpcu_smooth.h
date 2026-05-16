/**
 *  @file   cpcu_smooth.h
 *  @brief  Servo smoother API — trapezoidal profile, deadband, gravity compensation.
 *
 *  Per-servo state: target, current position, velocity, acceleration limits,
 *  hold-pose deadband, gravity bias. SMOOTH_Update() advances all channels
 *  by dt microseconds. SMOOTH_ShouldWrite() gates PCA writes to suppress
 *  jitter on settled servos.
 */

#ifndef CPCU_SMOOTH_H
#define CPCU_SMOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cpcu_pca9685.h"

#include <stdint.h>
#include <stdbool.h>

/*============= DEFAULTS ===================================================*/

#define SMOOTH_DEFAULT_VELOCITY     2000     /* us per second */
#define SMOOTH_DEFAULT_ACCEL        8000     /* us per second² */
#define SMOOTH_SETTLE_THRESH        2        /* within this distance = settled */
#define SMOOTH_DEFAULT_DEADBAND     10       /* ≈0.9° hold-pose deadband */

/*============= CONTEXT ====================================================*/

typedef struct
{
    /* Live state per servo */
    float       current_f[PCA_SERVO_COUNT];     /* sub-us position */
    float       velocity_f[PCA_SERVO_COUNT];    /* signed, us/s */
    uint16_t    current[PCA_SERVO_COUNT];       /* integer pose for PCA */
    uint16_t    target[PCA_SERVO_COUNT];

    /* Configuration per servo */
    uint16_t    max_velocity[PCA_SERVO_COUNT];  /* us/s */
    uint16_t    max_accel[PCA_SERVO_COUNT];     /* us/s² */
    bool        enabled[PCA_SERVO_COUNT];       /* false = bypass smoother */

    /* hold-pose deadband. See SMOOTH_ShouldWrite/MarkWritten. */
    uint16_t    hold_deadband_us[PCA_SERVO_COUNT];   /* 0 = disabled */
    uint16_t    last_written_us[PCA_SERVO_COUNT];    /* shadow of last PCA value */
    bool        ever_written[PCA_SERVO_COUNT];       /* false until first write */

    /* gravity compensation. For joints where arm weight accelerates
     * downward motion beyond what the servo can track, reduce max_velocity
     * when moving in the gravity-assisted direction.
     *   gravity_dir:   +1 = gravity helps positive motion (increasing us)
     *                  -1 = gravity helps negative motion (decreasing us)
     *                   0 = no gravity effect (default)
     *   gravity_scale: 0.1-1.0, multiplied into max_velocity when moving
     *                  in the gravity direction. 1.0 = no reduction (default).
     *                  0.3 = servo moves at 30% speed during gravity drop. */
    int8_t      gravity_dir[PCA_SERVO_COUNT];        /* +1, -1, or 0 */
    float       gravity_scale[PCA_SERVO_COUNT];      /* 0.1 .. 1.0 */

    /* Status per servo */
    bool        settled[PCA_SERVO_COUNT];
} SMOOTH_Context;

/*============= API ========================================================*/

/*  Initialise all channels to start_us with the default profile and
 *  enabled=true. Call SMOOTH_SetEnabled afterwards to bypass specific
 *  channels (e.g. Gripper).                                              */
void SMOOTH_Init(SMOOTH_Context *ctx, uint16_t start_us);

/*  Configure one channel. */
void SMOOTH_SetEnabled (SMOOTH_Context *ctx, int channel, bool enabled);
void SMOOTH_SetVelocity(SMOOTH_Context *ctx, int channel, uint16_t v_us_per_s);
void SMOOTH_SetAccel   (SMOOTH_Context *ctx, int channel, uint16_t a_us_per_s2);

/* gravity compensation.
 *  dir:   +1  gravity assists positive motion (servo_us increasing)
 *         -1  gravity assists negative motion (servo_us decreasing)
 *          0  disabled (default)
 *  scale: 0.1-1.0, velocity multiplier for gravity-assisted direction.
 *         1.0 = no reduction. 0.3 = 30% speed when dropping. */
void SMOOTH_SetGravity (SMOOTH_Context *ctx, int channel, int8_t dir, float scale);

/* hold-pose deadband configuration.
 *  Once a servo settles at its target, SMOOTH_ShouldWrite() returns
 *  false until the target moves more than `deadband_us` from the
 *  last-written PCA value. Suppresses static jitter on cheap hobby
 *  servos. Pass 0 to disable. */
void SMOOTH_SetDeadband(SMOOTH_Context *ctx, int channel, uint16_t deadband_us);

/* compatibility shim — old code called this "SetSpeed". Maps to
 *  SetVelocity previously. Keep using SMOOTH_SetVelocity in new code.      */
static inline void SMOOTH_SetSpeed(SMOOTH_Context *ctx, int channel,
                                   uint16_t speed_us_per_s)
{
    SMOOTH_SetVelocity(ctx, channel, speed_us_per_s);
}

/*  Target setting. */
void SMOOTH_SetTarget    (SMOOTH_Context *ctx, int channel, uint16_t target_us);
void SMOOTH_SetAllTargets(SMOOTH_Context *ctx, const uint16_t targets[PCA_SERVO_COUNT]);

/*  Advance positions by dt. dt_us = 20000 for 50 Hz tick. */
void SMOOTH_Update(SMOOTH_Context *ctx, uint32_t dt_us);

/*  Force every channel to its target now (zero velocity). */
void SMOOTH_Snap(SMOOTH_Context *ctx);

/* should the consumer issue a fresh PCA write for this channel?
 *  Returns true when:
 *      - the channel has never been written, OR
 *      - the smoother is not settled (motion in progress), OR
 *      - the smoother is settled but |current - last_written| exceeds the
 *        deadband (target moved enough to warrant an update).
 *  Returns false when the channel is settled at a value within the
 *  deadband of what the PCA already has — in that case, skipping the
 *  write avoids re-triggering the servo's internal correction loop. */
bool SMOOTH_ShouldWrite(const SMOOTH_Context *ctx, int channel);

/* caller MUST invoke this after each successful PCA write so
 *  the deadband logic knows what's currently latched in the hardware. */
void SMOOTH_MarkWritten(SMOOTH_Context *ctx, int channel, uint16_t written_us);

/*  Diagnostic */
bool SMOOTH_AllSettled(const SMOOTH_Context *ctx);

#ifdef __cplusplus
}
#endif

#endif /* CPCU_SMOOTH_H */

