/**
 *  @file   cpcu_safety.c
 *  @brief  Safety monitor — 7-source fault detection with deterministic FSM.
 *
 *  Monitors radio timeout, battery voltage, DSP stall, I2C bus health,
 *  ring-buffer overflow, thermal limits, and NRF hardware status.
 *  Drives a RUNNING -> DEGRADED -> SAFE state machine with hysteresis
 *  and auto-recovery. Cold-start grace period suppresses radio faults
 *  during the first 5 seconds after boot.
 *
 *  Called from cpcu_io.c's main loop in fixed order:
 *    FeedPacket -> CheckTimeout -> CheckDSP -> FeedRingOverflow ->
 *    FeedI2C -> FeedTemperature -> UpdateState
 */

#include "cpcu_safety.h"

#include <string.h>
#include <stdio.h>
#include <time.h>

/*============= INTERNAL TIMING ============================================*/
/*
 *  Local CLOCK_MONOTONIC sampler for the ring-overflow recovery
 *  timer. We could have added a now_us argument to FeedRingOverflow,
 *  but the testbench (test/safety_testbench.c) and cpcu_io.c both use
 *  the single-argument form, and keeping that contract avoids a
 *  user-facing API change.
 */
static uint64_t safety_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/*============= INIT =======================================================*/

void SAFETY_Init(SAFETY_Context *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state              =   RADIO_INIT;
    ctx->last_fault         =   SAFETY_OK;

    /* anchor the cold-start grace period.
     * Note that memset above already cleared first_packet_seen to false. */
    ctx->boot_us            =   safety_now_us();
}

/*============= SEQUENCE GAP ===============================================*/

uint32_t SAFETY_SeqGap(SAFETY_Context *ctx, uint8_t seq)
{
    if(!ctx->seq_init)
    {
        ctx->expected_seq   =   seq;
        ctx->seq_init       =   true;
        return 0;
    }
    uint8_t gap         =   (seq - ctx->expected_seq) & 0xFF;
    ctx->expected_seq   =   (seq + 1) & 0xFF;
    return (gap <= 1) ? 0 : gap;
}

/*============= LINK FEED ==================================================*/

static void link_feed(LINK_Stats *l, const WL_Packet *pkt, uint32_t gap,
                      uint64_t now_us)
{
    l->w_packets++;
    l->w_gaps       +=  gap;
    l->w_retry_sum  +=  pkt->tx_retry;

    if(l->w_packets >= SAFETY_LINK_WINDOW)
    {
        l->mean_retry           =   (float)l->w_retry_sum / (float)l->w_packets;
        uint32_t total_expected =   l->w_packets + l->w_gaps;
        l->loss_rate            =   (total_expected > 0)
                                     ? (float)l->w_gaps / (float)total_expected
                                     : 0.0f;

        LINK_Quality prev_q = l->quality;
        if(l->mean_retry < SAFETY_LINK_RETRY_GOOD &&
           l->loss_rate < SAFETY_LINK_LOSS_GOOD)
            l->quality = LINK_GOOD;
        else if(l->mean_retry < SAFETY_LINK_RETRY_DEGRADED &&
                l->loss_rate < SAFETY_LINK_LOSS_DEGRADED)
            l->quality = LINK_DEGRADED;
        else
            l->quality = LINK_POOR;

        if(l->quality == LINK_GOOD)
        {
            if(prev_q != LINK_GOOD || l->good_since_us == 0)
                l->good_since_us = now_us;
        }
        else
        {
            l->good_since_us = 0;
        }

        l->w_packets    =   0;
        l->w_gaps       =   0;
        l->w_retry_sum  =   0;
    }
}

/*============= FEED PACKET ================================================*/

void SAFETY_FeedPacket(SAFETY_Context *ctx, const WL_Packet *pkt,
                       uint64_t now_us)
{
    /* any successful FeedPacket call lifts the cold-start grace.
     * We don't gate this on FIRST_PACKET — what matters is that we're
     * receiving traffic, not whether the BSAU labelled this as its first.
     * (FIRST_PACKET still controls expected_seq init, below.) */
    ctx->first_packet_seen  =   true;

    if(pkt->flags & WL_FLAG_FIRST_PACKET)
    {
        ctx->expected_seq   =   pkt->seq;
        ctx->seq_init       =   true;
    }

    uint32_t gap            =   SAFETY_SeqGap(ctx, pkt->seq);

    /* suppress link stats during boot grace period.
     * Startup gaps (from BSAU powering up after CPCU) should not poison
     * the link quality window. We still count the gap for diagnostics
     * (the caller increments io_seq_gaps), but we don't feed it into
     * link_feed until the boot grace has elapsed or we've received
     * enough consecutive packets to establish a stable baseline.
     *
     * Additionally, when first_packet_seen transitions true, reset
     * the link stats window so any gaps accumulated during the very
     * first packets don't persist. */
    bool in_boot_grace = (now_us - ctx->boot_us) / 1000 < SAFETY_RADIO_BOOT_GRACE_MS;

    if(in_boot_grace)
    {
        /* During grace: feed gap as 0 so link stats stay clean */
        link_feed(&ctx->link, pkt, 0, now_us);
    }
    else
    {
        /* After grace: if this is the first post-grace packet, flush
         * any stale stats from the boot period */
        if(!ctx->boot_grace_flushed)
        {
            ctx->boot_grace_flushed = true;
            memset(&ctx->link, 0, sizeof(ctx->link));
        }
        link_feed(&ctx->link, pkt, gap, now_us);
    }

    /* Battery with hysteresis */
    ctx->battery.raw        =   pkt->vbat_raw;
    ctx->battery.voltage    =   pkt->vbat_raw * (3.3f / 4095.0f) * SAFETY_VBAT_DIVIDER;
    ctx->battery.level      =   WL_BATT_GET(pkt->flags);

    if(!ctx->battery.critical)
    {
        ctx->battery.critical = (ctx->battery.level == WL_BATT_CRIT)
                              || (ctx->battery.voltage < SAFETY_VBAT_CRITICAL_V
                                  && ctx->battery.voltage > 0.1f);
        if(ctx->battery.critical)
            ctx->last_fault = SAFETY_ERR_BATTERY;
    }
    else if(ctx->battery.voltage > SAFETY_VBAT_RECOVER_V &&
            ctx->battery.level != WL_BATT_CRIT)
    {
        ctx->battery.critical = false;
    }

    ctx->last_pkt_rcv_us    =   now_us;

    /* Radio FSM (battery + thermal + others handled in UpdateState) */
    switch(ctx->state)
    {
        case RADIO_INIT:
            ctx->state          =   RADIO_RUNNING;
            ctx->last_fault     =   SAFETY_OK;
            break;

        case RADIO_DEGRADED:
            ctx->state          =   RADIO_RECOVERING;
            ctx->recovery_cnt   =   (gap == 0) ? 1 : 0;
            break;

        case RADIO_RECOVERING:
            if(gap == 0 && ++ctx->recovery_cnt >= RECOVERY_PKT_COUNT)
            {
                ctx->state          =   RADIO_RUNNING;
                ctx->last_fault     =   SAFETY_OK;
            }
            else if(gap > 0)
            {
                ctx->recovery_cnt   =   0;
            }
            /* safety net — if we've been in RECOVERING for
             * 10+ seconds with packets flowing, force to RUNNING.
             * Prevents permanent lockout from boot-time gap bursts
             * that poisoned the link stats window. */
            if(ctx->state == RADIO_RECOVERING &&
               ctx->degraded_entry_us > 0 &&
               (now_us - ctx->degraded_entry_us) / 1000 > 10000)
            {
                ctx->state          =   RADIO_RUNNING;
                ctx->last_fault     =   SAFETY_OK;
            }
            break;

        case RADIO_RUNNING:
        case RADIO_SAFE:
            /* Other transitions are driven from UpdateState. */
            break;
    }
}

/*============= TIMEOUT CHECK ==============================================*/

void SAFETY_CheckTimeout(SAFETY_Context *ctx, uint64_t now_us)
{
    if(ctx->state == RADIO_INIT) return;
    if(ctx->state == RADIO_SAFE) return;     /* recovery handled by UpdateState */

    /* cold-start grace.
     *
     * If we have never received a packet AND we are still inside the boot
     * grace window, we are not in a fault — we are in initial sync. The
     * BSAU may simply not have powered on yet, or may still be holding
     * its NRF in reset. Returning early here keeps the FSM in
     * RADIO_RUNNING (which is the post-Init default after the first
     * UpdateState pass) and prevents a spurious SAFE on cold boot.
     *
     * Once first_packet_seen flips true (in SAFETY_FeedPacket), or
     * SAFETY_RADIO_BOOT_GRACE_MS has elapsed, this guard is bypassed
     * and the normal timeout logic resumes. So a genuinely-dead
     * BSAU still trips a fault — just SAFETY_RADIO_BOOT_GRACE_MS
     * later than a runtime drop would. */
    if(!ctx->first_packet_seen &&
       (now_us - ctx->boot_us) / 1000 < SAFETY_RADIO_BOOT_GRACE_MS)
    {
        return;
    }

    uint64_t silence_ms = (now_us - ctx->last_pkt_rcv_us) / 1000;

    if((ctx->state == RADIO_RUNNING || ctx->state == RADIO_RECOVERING) &&
       silence_ms > SAFETY_RADIO_TIMEOUT_MS)
    {
        ctx->state              =   RADIO_DEGRADED;
        ctx->degraded_entry_us  =   now_us;
        ctx->recovery_cnt       =   0;
        ctx->last_fault         =   SAFETY_ERR_RADIO;
    }
    else if(ctx->state == RADIO_DEGRADED &&
            (now_us - ctx->degraded_entry_us) / 1000 > SAFETY_RADIO_SAFE_MS)
    {
        ctx->state              =   RADIO_SAFE;
        ctx->last_fault         =   SAFETY_ERR_RADIO;
        ctx->safe_entry_us      =   now_us;
        ctx->safe_clear_since_us = 0;
    }
}

/*============= DSP HEALTH =================================================*/

void SAFETY_FeedMotorCMD(SAFETY_Context *ctx, uint64_t now_us)
{
    ctx->dsp.last_cmd_us    =   now_us;
    ctx->dsp.active         =   true;
    ctx->dsp.stalled        =   false;
}

void SAFETY_CheckDSP(SAFETY_Context *ctx, uint64_t now_us)
{
    if(!ctx->dsp.active) return;

    uint64_t ms = (now_us - ctx->dsp.last_cmd_us) / 1000;
    bool was_stalled = ctx->dsp.stalled;
    ctx->dsp.stalled = (ms > SAFETY_DSP_STALL_MS);

    if(ctx->dsp.stalled && !was_stalled)
        ctx->last_fault = SAFETY_ERR_DSP_STALL;
}

/*============= I2C ========================================================*/

void SAFETY_FeedI2C(SAFETY_Context *ctx, bool success)
{
    if(success)
    {
        ctx->i2c.consecutive_errors =   0;
        ctx->i2c.faulted            =   false;     /* clears on first success */
    }
    else
    {
        ctx->i2c.consecutive_errors++;
        if(ctx->i2c.consecutive_errors >= SAFETY_I2C_MAX_ERRORS &&
           !ctx->i2c.faulted)
        {
            ctx->i2c.faulted        =   true;
            ctx->last_fault         =   SAFETY_ERR_I2C_BUS;
        }
    }
}

/*============= RING (recoverable) ==================================*/
/**
 *  Trip condition (entry to fault):
 *      (current - baseline) > SAFETY_RING_OVERFLOW_LIMIT
 *  Recovery condition (exit from fault):
 *      no new overflow growth for SAFETY_RING_RECOVER_MS, then re-baseline.
 *
 *  This mirrors the hysteresis pattern already used by battery and thermal:
 *  the fault arms quickly, but only clears once the offending condition
 *  has been quiet for an explicit hold time. The baseline reset on
 *  recovery means a future burst is detected against a fresh window
 *  rather than against the all-time-cumulative count, which would
 *  otherwise stay above the threshold forever.
 */
void SAFETY_FeedRingOverflow(SAFETY_Context *ctx, uint32_t overflow_count)
{
    uint64_t now_us             =   safety_now_us();

    /* Track growth of the cumulative counter. */
    if(overflow_count > ctx->ring.overflow_count)
    {
        ctx->ring.last_growth_us = now_us;
    }
    ctx->ring.overflow_count    =   overflow_count;

    /* Delta against the current baseline. */
    uint32_t delta              =   (overflow_count >= ctx->ring.baseline_count)
                                     ? overflow_count - ctx->ring.baseline_count
                                     : 0;

    if(!ctx->ring.faulted)
    {
        if(delta > SAFETY_RING_OVERFLOW_LIMIT)
        {
            ctx->ring.faulted   =   true;
            ctx->last_fault     =   SAFETY_ERR_RING_OVF;
        }
    }
    else
    {
        /* In fault — clear once the SPSC ring has been quiet for
         * SAFETY_RING_RECOVER_MS. Re-baseline so subsequent bursts
         * trip cleanly without latching on the all-time count. */
        if(ctx->ring.last_growth_us > 0 &&
           (now_us - ctx->ring.last_growth_us) / 1000 > SAFETY_RING_RECOVER_MS)
        {
            ctx->ring.faulted        =   false;
            ctx->ring.baseline_count =   overflow_count;
        }
    }
}

/*============= THERMAL ====================================================*/

void SAFETY_FeedTemperature(SAFETY_Context *ctx, float temp_c)
{
    ctx->thermal.temperature_c  =   temp_c;
    ctx->thermal.warning        =   (temp_c > (float)SAFETY_THERMAL_WARN_C);

    if(!ctx->thermal.critical)
    {
        ctx->thermal.critical   =   (temp_c > (float)SAFETY_THERMAL_CRITICAL_C);
        if(ctx->thermal.critical)
            ctx->last_fault     =   SAFETY_ERR_THERMAL;
    }
    else if(temp_c < (float)SAFETY_THERMAL_RECOVER_C)
    {
        ctx->thermal.critical   =   false;
    }
}

/*============= DECISION (read-only) =======================================*/

bool SAFETY_CheckSystem(const SAFETY_Context *ctx)
{
    if(ctx->state != RADIO_RUNNING) return false;
    if(ctx->battery.critical)       return false;
    if(ctx->dsp.stalled)            return false;
    if(ctx->i2c.faulted)            return false;
    if(ctx->thermal.critical)       return false;
    if(ctx->ring.faulted)           return false;
    return true;
}

/*============= STATE TRANSITIONS ===================================*/
/**
 *  Single function that owns FSM transitions for non-radio-timing causes.
 *
 *  Entry to SAFE:
 *      If we're in RUNNING and any of {battery.critical, dsp.stalled,
 *      i2c.faulted, thermal.critical, ring.faulted} is true, transition
 *      to SAFE and record which.
 *
 *  Exit from SAFE:
 *      All five fault flags must be clear. Once they are, start the
 *      stable-clear timer; after SAFETY_SAFE_RECOVER_MS, transition out
 *      via RECOVERING (if the original cause was the radio link) or
 *      directly to RUNNING.
 */
void SAFETY_UpdateState(SAFETY_Context *ctx, uint64_t now_us)
{
    bool any_fault = ctx->battery.critical ||
                     ctx->dsp.stalled       ||
                     ctx->i2c.faulted       ||
                     ctx->thermal.critical  ||
                     ctx->ring.faulted;

    if(ctx->state == RADIO_RUNNING && any_fault)
    {
        /* Promote which one is "the" cause for telemetry */
        if(ctx->battery.critical)       ctx->last_fault = SAFETY_ERR_BATTERY;
        else if(ctx->thermal.critical)  ctx->last_fault = SAFETY_ERR_THERMAL;
        else if(ctx->dsp.stalled)       ctx->last_fault = SAFETY_ERR_DSP_STALL;
        else if(ctx->i2c.faulted)       ctx->last_fault = SAFETY_ERR_I2C_BUS;
        else if(ctx->ring.faulted)      ctx->last_fault = SAFETY_ERR_RING_OVF;

        ctx->state              =   RADIO_SAFE;
        ctx->safe_entry_us      =   now_us;
        ctx->safe_clear_since_us = 0;
        return;
    }

    if(ctx->state != RADIO_SAFE) return;

    /* In SAFE: try to recover */
    if(any_fault)
    {
        ctx->safe_clear_since_us = 0;
        return;
    }

    /* All flags clear — start or check the stable timer */
    if(ctx->safe_clear_since_us == 0)
    {
        ctx->safe_clear_since_us = now_us;
        return;
    }

    if((now_us - ctx->safe_clear_since_us) / 1000 < SAFETY_SAFE_RECOVER_MS)
        return;

    /* Stable for required hold time — exit SAFE */
    if(ctx->last_fault == SAFETY_ERR_RADIO)
    {
        ctx->state              =   RADIO_RECOVERING;
        ctx->recovery_cnt       =   0;
    }
    else
    {
        ctx->state              =   RADIO_RUNNING;
        ctx->last_fault         =   SAFETY_OK;
    }
    ctx->safe_clear_since_us    =   0;
}

/*============= AUTO-CLEAR =================================================*/

bool SAFETY_ShouldAutoClear(SAFETY_Context *ctx, uint64_t now_us)
{
    if(ctx->link.quality != LINK_GOOD) return false;
    if(ctx->link.good_since_us == 0)   return false;

    uint64_t good_ms = (now_us - ctx->link.good_since_us) / 1000;
    if(good_ms >= SAFETY_AUTO_CLEAR_MS)
    {
        ctx->link.good_since_us = now_us;
        return true;
    }
    return false;
}

/*============= STRINGS ====================================================*/

const char *SAFETY_StatusStr(SAFETY_Status s)
{
    switch(s)
    {
        case SAFETY_OK:             return "OK";
        case SAFETY_ERR_RADIO:      return "RADIO_LOST";
        case SAFETY_ERR_BATTERY:    return "BATT_CRITICAL";
        case SAFETY_ERR_DSP_STALL:  return "DSP_STALLED";
        case SAFETY_ERR_NRF_HW:     return "NRF_HW_FAIL";
        case SAFETY_ERR_I2C_BUS:    return "I2C_BUS_FAIL";
        case SAFETY_ERR_RING_OVF:   return "RING_OVERFLOW";
        case SAFETY_ERR_THERMAL:    return "OVERTEMP";
        default:                    return "UNKNOWN";
    }
}

const char *SAFETY_RadioStr(RADIO_State s)
{
    switch(s)
    {
        case RADIO_INIT:        return "INIT";
        case RADIO_RUNNING:     return "RUNNING";
        case RADIO_DEGRADED:    return "DEGRADED";
        case RADIO_RECOVERING:  return "RECOVERING";
        case RADIO_SAFE:        return "SAFE";
        default:                return "???";
    }
}

