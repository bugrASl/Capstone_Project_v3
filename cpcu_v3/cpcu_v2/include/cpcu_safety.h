/**
 *  @file   cpcu_safety.h
 *  @brief  Safety monitor API — 7 fault sources, FSM, tunable thresholds.
 *
 *  Caller contract (cpcu_io main loop, fixed order per iteration):
 *    1. SAFETY_FeedPacket       (on packet arrival)
 *    2. SAFETY_CheckTimeout     (every loop)
 *    3. SAFETY_CheckDSP         (every loop)
 *    4. SAFETY_FeedRingOverflow (every loop)
 *    5. SAFETY_FeedI2C          (on I2C activity)
 *    6. SAFETY_FeedTemperature  (every ~5 s)
 *    7. SAFETY_UpdateState      (every loop, after all feeds)
 */

#ifndef CPCU_SAFETY_H
#define CPCU_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "wireless_packet.h"

/*============= TUNABLE CONFIG =============================================*/

/* Radio Timing */
#define SAFETY_RADIO_TIMEOUT_MS         750
#define SAFETY_RADIO_SAFE_MS            1500
#define SAFETY_RADIO_BOOT_GRACE_MS      5000        /* cold-start
                                                       grace before radio
                                                       timeout fires. Distin-
                                                       guishes "no packet ever
                                                       received from this BSAU"
                                                       (initial sync) from
                                                       "packets stopped"
                                                       (genuine fault). See
                                                       docs/BOOT_AND_SYNC.md
                                                       for the full rationale. */
#define RECOVERY_PKT_COUNT              10

/* Link Quality */
#define SAFETY_LINK_WINDOW              1000
#define SAFETY_LINK_RETRY_GOOD          0.5f
#define SAFETY_LINK_RETRY_DEGRADED      3.0f
#define SAFETY_LINK_LOSS_GOOD           0.001f
#define SAFETY_LINK_LOSS_DEGRADED       0.05f

/* Battery (with hysteresis) */
#define SAFETY_VBAT_LOW_V               3.0f
#define SAFETY_VBAT_CRITICAL_V          2.7f
#define SAFETY_VBAT_RECOVER_V           3.0f
#define SAFETY_VBAT_DIVIDER             2.0f    /*  BSAU PB0 has a 2:1 resistor
                                                 *  divider so a 4.0 V battery
                                                 *  reads as 2.0 V at the ADC.
                                                 *  bsau_app.c sends the raw
                                                 *  post-divider 12-bit count
                                                 *  directly, so CPCU must
                                                 *  multiply by 2.0 here to
                                                 *  recover the battery V.
                                                 *  set this to 1.0 with
                                                 *  a doc-comment claiming the
                                                 *  BSAU firmware now corrects
                                                 *  — but BSAU_ADC_GetBattery()
                                                 *  in bsau_adc.c is unchanged
                                                 *  and passes the raw count
                                                 *  through, so 1.0 was a real
                                                 *  bug: every healthy 4.0 V
                                                 *  battery was being read as
                                                 *  2.00 V on the CPCU side
                                                 *  → battery.critical stuck
                                                 *  true → CheckSystem stuck
                                                 *  false → servos always
                                                 *  parked at neutral.
                                                 *  Restored to 2.0 (corrected)./

/* DSP */
#define SAFETY_DSP_STALL_MS             2000

/* I2C */
#define SAFETY_I2C_MAX_ERRORS           5

/* Ring (with recovery) */
#define SAFETY_RING_OVERFLOW_LIMIT      100         /* trip threshold (delta) */
#define SAFETY_RING_RECOVER_MS          5000        /* clear after this much
                                                       quiescence            */

/* Thermal (with hysteresis) */
#define SAFETY_THERMAL_WARN_C           75
#define SAFETY_THERMAL_CRITICAL_C       82
#define SAFETY_THERMAL_RECOVER_C        70

/* SAFE recovery */
#define SAFETY_SAFE_RECOVER_MS          3000

/* Auto-clear of cumulative diag counters */
#define SAFETY_AUTO_CLEAR_MS            300000

/*============= STATUS =====================================================*/

typedef enum
{
    SAFETY_OK = 0,
    SAFETY_ERR_RADIO,
    SAFETY_ERR_BATTERY,
    SAFETY_ERR_DSP_STALL,
    SAFETY_ERR_NRF_HW,
    SAFETY_ERR_I2C_BUS,
    SAFETY_ERR_RING_OVF,
    SAFETY_ERR_THERMAL,
} SAFETY_Status;

typedef enum
{
    RADIO_INIT = 0,
    RADIO_RUNNING,
    RADIO_DEGRADED,
    RADIO_RECOVERING,
    RADIO_SAFE,
} RADIO_State;

typedef enum
{
    LINK_GOOD = 0,
    LINK_DEGRADED,
    LINK_POOR,
} LINK_Quality;

typedef struct
{
    LINK_Quality    quality;
    uint32_t        w_packets;
    uint32_t        w_gaps;
    uint32_t        w_retry_sum;
    float           mean_retry;
    float           loss_rate;
    uint16_t        last_timestamp;
    bool            has_previous;
    uint64_t        good_since_us;
} LINK_Stats;

typedef struct
{
    uint16_t        raw;
    float           voltage;
    uint8_t         level;
    bool            critical;
} BATTERY_State;

typedef struct
{
    uint64_t        last_cmd_us;
    bool            stalled;
    bool            active;
} DSP_Health;

typedef struct
{
    uint32_t        consecutive_errors;
    bool            faulted;
} I2C_Health;

/**
 *  RING_Health.
 *      overflow_count   — last cumulative count we sampled (atomic from IPC).
 *      baseline_count   — count value at the last quiescent re-baseline.
 *                         The trip threshold is applied to (overflow_count
 *                         - baseline_count) so the FSM doesn't latch on a
 *                         monotonic counter.
 *      last_growth_us   — last time the cumulative count went up. Used by
 *                         the recovery side: faulted clears once
 *                         SAFETY_RING_RECOVER_MS pass with no new growth.
 *      faulted          — public flag consumed by SAFETY_CheckSystem and
 *                         SAFETY_UpdateState.
 */
typedef struct
{
    uint32_t        overflow_count;
    uint32_t        baseline_count;
    uint64_t        last_growth_us;
    bool            faulted;
} RING_Health;

typedef struct
{
    float           temperature_c;
    bool            warning;
    bool            critical;
} THERMAL_Health;

typedef struct
{
    RADIO_State     state;
    SAFETY_Status   last_fault;

    uint64_t        last_pkt_rcv_us;
    uint64_t        degraded_entry_us;
    uint32_t        recovery_cnt;
    uint8_t         expected_seq;
    bool            seq_init;

    /* cold-start grace tracking. boot_us is set by
     * SAFETY_Init to the moment the safety subsystem started.
     * first_packet_seen flips to true the first time
     * SAFETY_FeedPacket is invoked. SAFETY_CheckTimeout suppresses
     * the radio fault until either is true OR until the grace has
     * elapsed. See docs/BOOT_AND_SYNC.md. */
    uint64_t        boot_us;
    bool            first_packet_seen;
    bool            boot_grace_flushed;     /* link stats flushed after grace */

    LINK_Stats      link;
    BATTERY_State   battery;
    DSP_Health      dsp;
    I2C_Health      i2c;
    RING_Health     ring;
    THERMAL_Health  thermal;

    uint64_t        safe_clear_since_us;
    uint64_t        safe_entry_us;
} SAFETY_Context;

/*============= API ========================================================*/

void        SAFETY_Init(SAFETY_Context *ctx);

uint32_t    SAFETY_SeqGap(SAFETY_Context *ctx, uint8_t seq);
void        SAFETY_FeedPacket(SAFETY_Context *ctx, const WL_Packet *pkt, uint64_t now_us);
void        SAFETY_CheckTimeout(SAFETY_Context *ctx, uint64_t now_us);

void        SAFETY_FeedMotorCMD(SAFETY_Context *ctx, uint64_t now_us);
void        SAFETY_CheckDSP(SAFETY_Context *ctx, uint64_t now_us);

void        SAFETY_FeedI2C(SAFETY_Context *ctx, bool success);

/**
 *  API unchanged (still takes the cumulative atomic counter from
 *  ipc.diag->io_ring_overflows), but the recovery logic is now internal
 *  — the function uses clock_gettime() to maintain its own quiescence
 *  timer so cpcu_io and the safety testbench did not have to change.
 *  Call this every loop, even when no new overflows occurred.
 */
void        SAFETY_FeedRingOverflow(SAFETY_Context *ctx, uint32_t overflow_count);

void        SAFETY_FeedTemperature(SAFETY_Context *ctx, float temp_c);

bool        SAFETY_CheckSystem(const SAFETY_Context *ctx);

/* Single function that handles ALL FSM transitions.
 *  Call it once per loop after all the Feed/Check functions have
 *  refreshed the boolean fault flags.                                   */
void        SAFETY_UpdateState(SAFETY_Context *ctx, uint64_t now_us);

bool        SAFETY_ShouldAutoClear(SAFETY_Context *ctx, uint64_t now_us);

const char  *SAFETY_StatusStr(SAFETY_Status status);
const char  *SAFETY_RadioStr(RADIO_State state);

#ifdef __cplusplus
}
#endif

#endif /* CPCU_SAFETY_H */

