/**
 *  @file   cpcu_smooth.c
 *  @brief  Per-servo trapezoidal motion smoother with hold-pose deadband.
 *
 *  Converts step-function motor commands into smooth trapezoidal profiles
 *  (acceleration-limited ramp to velocity, then deceleration to target).
 *  Hold-pose deadband suppresses redundant PCA writes when a servo has
 *  settled, eliminating static jitter from the servo's internal P controller.
 *
 *  Gravity compensation biases velocity asymmetrically for weight-bearing
 *  joints (faster when fighting gravity, slower when assisted).
 */

#include "cpcu_smooth.h"

#include <math.h>
#include <stdlib.h>     /* abs() */

/*============= INIT =======================================================*/

void SMOOTH_Init(SMOOTH_Context *ctx, uint16_t start_us)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        ctx->current_f[i]        = (float)start_us;
        ctx->velocity_f[i]       = 0.0f;
        ctx->current[i]          = start_us;
        ctx->target[i]           = start_us;
        ctx->max_velocity[i]     = SMOOTH_DEFAULT_VELOCITY;
        ctx->max_accel[i]        = SMOOTH_DEFAULT_ACCEL;
        ctx->enabled[i]          = true;
        ctx->settled[i]          = true;

        /* deadband state. start_us has not been written yet
         * — ever_written stays false until cpcu_io confirms the
         * first PCA write via SMOOTH_MarkWritten. */
        ctx->hold_deadband_us[i] = SMOOTH_DEFAULT_DEADBAND;
        ctx->last_written_us[i]  = 0;
        ctx->ever_written[i]     = false;

        /* gravity compensation — disabled by default */
        ctx->gravity_dir[i]      = 0;
        ctx->gravity_scale[i]    = 1.0f;
    }
}

/*============= CONFIG =====================================================*/

void SMOOTH_SetEnabled(SMOOTH_Context *ctx, int channel, bool enabled)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->enabled[channel] = enabled;
    if(!enabled)
    {
        /* Bypassed: snap immediately so we don't hold a stale interim pose */
        ctx->current_f[channel]  = (float)ctx->target[channel];
        ctx->current[channel]    = ctx->target[channel];
        ctx->velocity_f[channel] = 0.0f;
        ctx->settled[channel]    = true;
    }
}

void SMOOTH_SetVelocity(SMOOTH_Context *ctx, int channel, uint16_t v)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->max_velocity[channel] = v;
}

void SMOOTH_SetAccel(SMOOTH_Context *ctx, int channel, uint16_t a)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->max_accel[channel] = (a > 0) ? a : 1;   /* avoid div-by-0 */
}

void SMOOTH_SetGravity(SMOOTH_Context *ctx, int channel, int8_t dir, float scale)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->gravity_dir[channel]   = (dir > 0) ? 1 : (dir < 0) ? -1 : 0;
    ctx->gravity_scale[channel] = (scale < 0.1f) ? 0.1f
                                : (scale > 1.0f) ? 1.0f
                                : scale;
}

/*============= TARGET =====================================================*/

void SMOOTH_SetTarget(SMOOTH_Context *ctx, int channel, uint16_t target_us)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;

    ctx->target[channel] = target_us;

    int diff = (int)target_us - (int)ctx->current[channel];
    if(diff < -SMOOTH_SETTLE_THRESH || diff > SMOOTH_SETTLE_THRESH)
        ctx->settled[channel] = false;
}

void SMOOTH_SetAllTargets(SMOOTH_Context *ctx,
                          const uint16_t targets[PCA_SERVO_COUNT])
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
        SMOOTH_SetTarget(ctx, i, targets[i]);
}

/*============= UPDATE — TRAPEZOIDAL PROFILE ===============================*/
/**
 *  Each tick, for each enabled servo:
 *
 *  1. dist  = target - current_f               (signed)
 *     |dist| / 2*a is the "stopping distance" if we decel from
 *     current velocity at max_accel.
 *  2. If we're inside or at the stopping distance OR very close to
 *     target → decelerate (|v| -= a*dt, clamp to ≥ 0).
 *  3. Else if |v| < max_v → accelerate (|v| += a*dt, clamp to max_v).
 *  4. Else → cruise (|v| unchanged).
 *
 *  Direction is the sign of `dist`. When dist crosses zero (overshoot
 *  guard) we snap to target and zero the velocity — settled.
 */
void SMOOTH_Update(SMOOTH_Context *ctx, uint32_t dt_us)
{
    const float dt_s = (float)dt_us * 1.0e-6f;

    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        /* Bypass: just write target straight through */
        if(!ctx->enabled[i])
        {
            ctx->current_f[i]   = (float)ctx->target[i];
            ctx->current[i]     = ctx->target[i];
            ctx->velocity_f[i]  = 0.0f;
            ctx->settled[i]     = true;
            continue;
        }

        if(ctx->settled[i] && ctx->current[i] == ctx->target[i])
            continue;

        float pos      = ctx->current_f[i];
        float vel      = ctx->velocity_f[i];
        float tgt      = (float)ctx->target[i];
        float dist     = tgt - pos;
        float dist_abs = fabsf(dist);
        float vel_abs  = fabsf(vel);

        float a        = (float)ctx->max_accel[i];
        float v_max    = (float)ctx->max_velocity[i];

        /* gravity compensation. When moving in the gravity-assisted
         * direction, reduce max velocity so the smoother commands a slower
         * trajectory. The servo's internal PID can then track without the
         * arm weight causing overshoot. */
        if(ctx->gravity_dir[i] != 0)
        {
            float move_dir = (dist >= 0.0f) ? 1.0f : -1.0f;
            if((int)move_dir == ctx->gravity_dir[i])
                v_max *= ctx->gravity_scale[i];
        }

        /* If sitting close enough, finish */
        if(dist_abs < (float)SMOOTH_SETTLE_THRESH && vel_abs < a * dt_s)
        {
            ctx->current_f[i]   = tgt;
            ctx->current[i]     = ctx->target[i];
            ctx->velocity_f[i]  = 0.0f;
            ctx->settled[i]     = true;
            continue;
        }

        /* Distance needed to stop from current velocity.
         * This must match the actual discrete decel distance, which
         * depends on using trapezoidal integration below. With the
         * midpoint rule, discrete decel distance = v²/(2a) exactly. */
        float decel_dist = (vel_abs * vel_abs) / (2.0f * a);

        /* Phase decision */
        float new_vel_abs;
        if(dist_abs <= decel_dist)
        {
            /* Decelerate */
            new_vel_abs = vel_abs - a * dt_s;
            if(new_vel_abs < 0.0f) new_vel_abs = 0.0f;
        }
        else if(vel_abs < v_max)
        {
            /* Accelerate */
            new_vel_abs = vel_abs + a * dt_s;
            if(new_vel_abs > v_max) new_vel_abs = v_max;
        }
        else
        {
            /* Cruise */
            new_vel_abs = v_max;
        }

        /* Sign comes from current direction to target. If we somehow
         * crossed zero on the previous tick, the velocity sign needs
         * to flip — handled by the dist_abs sign check below. */
        float dir = (dist >= 0.0f) ? 1.0f : -1.0f;

        /* If we were moving the wrong way (rare: target changed mid-flight),
         * decelerate first before accelerating in the new direction. */
        if((vel > 0.0f && dist < 0.0f) || (vel < 0.0f && dist > 0.0f))
        {
            new_vel_abs = vel_abs - a * dt_s;
            if(new_vel_abs < 0.0f) new_vel_abs = 0.0f;
            dir = (vel > 0.0f) ? 1.0f : -1.0f;   /* keep current direction while braking */
        }

        /* Trapezoidal integration: use average of old and new velocity
         * for the position step. Plain Euler (pos += new_vel * dt)
         * over-steps during decel because it applies the reduced
         * velocity for the full tick, causing overshoot → snap. The
         * midpoint rule matches the continuous trapezoidal profile. */
        float avg_vel = (vel_abs + new_vel_abs) * 0.5f;
        float new_vel = new_vel_abs * dir;
        float new_pos = pos + avg_vel * dir * dt_s;

        /* Overshoot guard */
        if((dist > 0.0f && new_pos > tgt) || (dist < 0.0f && new_pos < tgt))
        {
            new_pos = tgt;
            new_vel = 0.0f;
            ctx->settled[i] = true;
        }
        else
        {
            ctx->settled[i] = false;
        }

        ctx->current_f[i]   = new_pos;
        ctx->velocity_f[i]  = new_vel;
        ctx->current[i]     = (uint16_t)(new_pos + 0.5f);
    }
}

/*============= SNAP =======================================================*/

void SMOOTH_Snap(SMOOTH_Context *ctx)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        ctx->current_f[i]   = (float)ctx->target[i];
        ctx->current[i]     = ctx->target[i];
        ctx->velocity_f[i]  = 0.0f;
        ctx->settled[i]     = true;
    }
}

/*============= QUERY ======================================================*/

bool SMOOTH_AllSettled(const SMOOTH_Context *ctx)
{
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
        if(!ctx->settled[i]) return false;
    return true;
}

/*============= DEADBAND ==============================================*/
/*
 *  Three-rule logic in SMOOTH_ShouldWrite:
 *
 *      1. Channel never written before  -> ALWAYS write (initial PWM).
 *      2. Smoother in motion (!settled) -> ALWAYS write (track motion).
 *      3. Smoother settled              -> write only if
 *                                          |current - last_written| > deadband.
 *
 *  Rule (1) prevents a freshly-initialised servo from being silent on
 *  startup just because its starting target equals SMOOTH_Init's value.
 *  Rule (2) ensures the smoother's per-tick interpolation always reaches
 *  the PCA — the deadband only suppresses redundant *holding* writes.
 *  Rule (3) is the actual jitter killer: when current matches what the
 *  PCA already has, sending the same pulse width again forces the
 *  servo's internal P controller to re-evaluate, which can perturb the
 *  hold position. Skip the write -> the servo coasts on its existing
 *  PWM signal -> no re-trigger.
 *
 *  When deadband_us == 0 the channel always writes (deadband disabled,
 *  legacy behaviour).
 */

void SMOOTH_SetDeadband(SMOOTH_Context *ctx, int channel, uint16_t deadband_us)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->hold_deadband_us[channel] = deadband_us;
}

bool SMOOTH_ShouldWrite(const SMOOTH_Context *ctx, int channel)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return false;

    /* Rule 1: never written before */
    if(!ctx->ever_written[channel]) return true;

    /* Rule 2: motion in progress */
    if(!ctx->settled[channel])      return true;

    /* Rule 3: settled — gate on deadband */
    uint16_t db = ctx->hold_deadband_us[channel];
    if(db == 0) return true;        /* deadband disabled */

    int diff = (int)ctx->current[channel] - (int)ctx->last_written_us[channel];
    if(diff < 0) diff = -diff;
    return diff > (int)db;
}

void SMOOTH_MarkWritten(SMOOTH_Context *ctx, int channel, uint16_t written_us)
{
    if(channel < 0 || channel >= PCA_SERVO_COUNT) return;
    ctx->last_written_us[channel] = written_us;
    ctx->ever_written[channel]    = true;
}

