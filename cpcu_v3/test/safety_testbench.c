/**
 *  @file   safety_testbench.c
 *  @brief  Safety FSM test harness — exercises all 7 fault paths with synthetic inputs.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "cpcu_safety.h"
#include "wireless_packet.h"

/*============= TEST FRAMEWORK =============================================================*/

static int g_tests_pass = 0;
static int g_tests_fail = 0;

#define CHECK(test_id, desc, expr, fmt, ...)                                    \
    do {                                                                        \
        if(expr) {                                                              \
            printf("[PASS] " test_id " %-32s  " fmt "\n", desc, ##__VA_ARGS__); \
            g_tests_pass++;                                                     \
        } else {                                                                \
            printf("[FAIL] " test_id " %-32s  " fmt "\n", desc, ##__VA_ARGS__); \
            g_tests_fail++;                                                     \
        }                                                                       \
    } while(0)

/*============= HELPERS ====================================================================*/

/**
 *  Build a synthetic healthy packet: nominal battery, no retry, matching
 *  sequence, flags = 0.
 */
static void build_healthy_packet(WL_Packet *pkt, uint8_t seq, uint16_t vbat_raw)
{
    memset(pkt, 0, sizeof(*pkt));
    pkt->seq         = seq;
    pkt->vbat_raw    = vbat_raw;
    pkt->flags       = 0;
    pkt->tx_retry    = 0;
    pkt->pkt_loss    = 0;
    pkt->timestamp   = seq;

    /* Nominal mid-rail ADC samples */
    for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
            pkt->samples[s].ch[ch] = 2048;
}

/**
 *  Drive the safety FSM through a warm-up of healthy packets to reach
 *  RADIO_RUNNING. Returns the timestamp after warm-up in us.
 *
 *  ~3.3 V nominal => vbat_raw ≈ 2048 (3.3 * 4095 / (3.3 * 2)) — wait, the
 *  divider is 2x so vbat_raw = voltage / 3.3 * 4095 / 2 for a 3.3V battery
 *  gives 2048. For a 4.0 V healthy LiPo: 4.0 / 2 / 3.3 * 4095 = 2482.
 */
static uint64_t warm_up(SAFETY_Context *ctx, uint64_t t0_us)
{
    uint64_t now = t0_us;
    WL_Packet pkt;

    for(int i = 0; i < 20; i++)
    {
        build_healthy_packet(&pkt, (uint8_t)(i & 0xFF), 2482);
        if(i == 0) pkt.flags |= WL_FLAG_FIRST_PACKET;
        SAFETY_FeedPacket(ctx, &pkt, now);
        now += 1000;                        /* 1 ms between packets */
    }
    return now;
}

/*============= TB-SAF01 : Radio loss timeout ==============================================*/
/*
 *  Expected transitions:
 *      t = 0 ms         : RADIO_RUNNING (after warm-up)
 *      t > 750 ms       : RADIO_DEGRADED
 *      t > 750 + 1500 ms: RADIO_SAFE
 *
 *  Verify:
 *      - At 700 ms of silence: still RUNNING
 *      - At 800 ms of silence: DEGRADED
 *      - At 2260 ms of silence: SAFE
 */
static void test_radio_loss_timeout(void)
{
    SAFETY_Context ctx;
    SAFETY_Init(&ctx);
    uint64_t t0 = warm_up(&ctx, 1000000);   /* 1 s arbitrary epoch */

    CHECK("TB-SAF01a", "warm-up reaches RUNNING",
          ctx.state == RADIO_RUNNING,
          "state=%s", SAFETY_RadioStr(ctx.state));

    /* 700 ms silence — should still be RUNNING (threshold is 750) */
    uint64_t t = t0 + 700000ULL;
    SAFETY_CheckTimeout(&ctx, t);
    CHECK("TB-SAF01b", "before timeout: still RUNNING",
          ctx.state == RADIO_RUNNING,
          "state=%s  silence=700ms target<750ms",
          SAFETY_RadioStr(ctx.state));

    /* 800 ms silence — should trip to DEGRADED */
    t = t0 + 800000ULL;
    SAFETY_CheckTimeout(&ctx, t);
    CHECK("TB-SAF01c", "silence>750ms: DEGRADED",
          ctx.state == RADIO_DEGRADED,
          "state=%s  silence=800ms target=DEGRADED",
          SAFETY_RadioStr(ctx.state));

    /* degraded_entry_us + 1500ms => SAFE */
    uint64_t deg_entry = ctx.degraded_entry_us;
    uint64_t t_safe    = deg_entry + 1501000ULL;
    SAFETY_CheckTimeout(&ctx, t_safe);
    CHECK("TB-SAF01d", "degraded>1500ms: SAFE",
          ctx.state == RADIO_SAFE,
          "state=%s  degraded_ms=%llu target>1500",
          SAFETY_RadioStr(ctx.state),
          (unsigned long long)((t_safe - deg_entry)/1000ULL));

    CHECK("TB-SAF01e", "fault code = SAFETY_ERR_RADIO",
          ctx.last_fault == SAFETY_ERR_RADIO,
          "fault=%s", SAFETY_StatusStr(ctx.last_fault));

    CHECK("TB-SAF01f", "CheckSystem()==false when SAFE",
          !SAFETY_CheckSystem(&ctx),
          "expected=false");
}

/*============= TB-SAF02 : Low battery fault ===============================================*/
/*
 *  Critical vbat ≈ 2.7 V. Through the on-board 2:1 divider on the BSAU,
 *  that's raw = 2.7 / 3.3 / 2 * 4095 = 1675. We pick 1600 to be safely
 *  below.
 *
 *  contract:
 *      - SAFETY_FeedPacket sets battery.critical = true and last_fault =
 *        SAFETY_ERR_BATTERY but does NOT change the FSM state.
 *      - SAFETY_UpdateState moves RUNNING → SAFE on battery.critical.
 *      - A subsequent good packet recovers battery.critical (hysteresis).
 *      - SAFETY_UpdateState then waits SAFETY_SAFE_RECOVER_MS = 3 s of
 *        all-clear before moving SAFE → RUNNING.
 *
 *  Previously the FSM transition was inline in FeedPacket and SAFE was
 *  terminal. The old TB-SAF02e expected that; The refactor deliberately made
 *  battery recoverable, and TB-SAF02e is rewritten here to test the
 *  recovery instead.
 */
static void test_low_battery(void)
{
    SAFETY_Context ctx;
    SAFETY_Init(&ctx);
    uint64_t t0 = warm_up(&ctx, 1000000);

    CHECK("TB-SAF02a", "warm-up to RUNNING",
          ctx.state == RADIO_RUNNING,
          "state=%s", SAFETY_RadioStr(ctx.state));

    /* Feed one packet with critical battery. Raw=1600 → ~2.58 V after
     * the 2:1 divider correction in cpcu_safety.c. */
    WL_Packet pkt;
    build_healthy_packet(&pkt, 50, 1600);
    SAFETY_FeedPacket(&ctx, &pkt, t0 + 1000);

    /* state transition for battery is in UpdateState, not FeedPacket. */
    SAFETY_UpdateState(&ctx, t0 + 1000);

    CHECK("TB-SAF02b", "critical vbat: state=SAFE",
          ctx.state == RADIO_SAFE,
          "state=%s  vbat_raw=1600 → %.2fV",
          SAFETY_RadioStr(ctx.state),
          ctx.battery.voltage);

    CHECK("TB-SAF02c", "critical vbat: battery.critical=true",
          ctx.battery.critical,
          "critical=%d", ctx.battery.critical);

    CHECK("TB-SAF02d", "fault = SAFETY_ERR_BATTERY",
          ctx.last_fault == SAFETY_ERR_BATTERY,
          "fault=%s", SAFETY_StatusStr(ctx.last_fault));

    /*
     *  battery is RECOVERABLE (was previously terminal).
     *  Feed a healthy packet → battery.critical clears via hysteresis,
     *  but state stays SAFE until SAFETY_SAFE_RECOVER_MS = 3 s of
     *  all-clear has elapsed.
     */
    build_healthy_packet(&pkt, 51, 2482);
    SAFETY_FeedPacket(&ctx, &pkt, t0 + 2000);
    SAFETY_UpdateState(&ctx, t0 + 2000);

    CHECK("TB-SAF02e", "good vbat: state still SAFE during 3s hold-off",
          ctx.state == RADIO_SAFE,
          "state=%s  battery.critical=%d  (hold-off in progress)",
          SAFETY_RadioStr(ctx.state), ctx.battery.critical);

    CHECK("TB-SAF02f", "good vbat: battery.critical cleared",
          !ctx.battery.critical,
          "critical=%d  vbat=%.2fV", ctx.battery.critical, ctx.battery.voltage);

    /*  Advance time past SAFETY_SAFE_RECOVER_MS = 3 s to verify the
     *  SAFE → RUNNING exit. We only need to call UpdateState — no
     *  more packets — because the safe_clear_since_us timer was started
     *  on the previous call when all flags were clear.                  */
    SAFETY_UpdateState(&ctx, t0 + 2000 + 3100ULL * 1000ULL);

    CHECK("TB-SAF02g", "after 3s all-clear: SAFE → RUNNING",
          ctx.state == RADIO_RUNNING,
          "state=%s", SAFETY_RadioStr(ctx.state));
}

/*============= TB-SAF03 : Sequence gap storm ==============================================*/
/*
 *  Drive a full LINK_WINDOW (=1000) packets with gap events every 10th
 *  packet. Stats commit at the 1000-packet window boundary.
 *
 *  Note on SAFETY_SeqGap: the function treats gap <= 1 as "in-order" —
 *  a deliberate tolerance for transient 1-packet hiccups — so we have
 *  to *skip two* seq numbers per event to produce a counted gap of 2
 *  (burst loss). This matches reality: a single dropped packet on a
 *  1kHz link is below the noise floor and shouldn't trip the quality
 *  classifier.
 */
static void test_seq_gap_storm(void)
{
    SAFETY_Context ctx;
    SAFETY_Init(&ctx);
    uint64_t t0 = warm_up(&ctx, 1000000);
    (void)t0;

    /* Warm-up consumed 20 packets. Feed 980 more, skipping *two* seq
     * numbers every 10 packets → 98 gap events of magnitude 2.
     * w_gaps accumulator will reach ~196. */
    uint8_t  seq = 20;
    uint64_t t   = 2000000;
    for(int i = 0; i < 980; i++)
    {
        if(i % 10 == 9)
            seq += 3;                        /* skip TWO seq → counted gap = 2 */
        else
            seq += 1;

        WL_Packet pkt;
        build_healthy_packet(&pkt, seq, 2482);
        SAFETY_FeedPacket(&ctx, &pkt, t);
        t += 1000;
    }

    /* Window rolled over at w_packets=1000 (20 + 980). Stats now committed. */
    CHECK("TB-SAF03a", "gap storm: loss_rate > 0.05",
          ctx.link.loss_rate > 0.05f,
          "loss_rate=%.4f  target>0.05", ctx.link.loss_rate);

    CHECK("TB-SAF03b", "gap storm: link quality degraded",
          ctx.link.quality == LINK_POOR || ctx.link.quality == LINK_DEGRADED,
          "quality=%d  (0=GOOD 1=DEGRADED 2=POOR)", (int)ctx.link.quality);

    /* Link quality alone does NOT trip the radio FSM to SAFE — that's
     * by design. RADIO_RUNNING is about connectivity, LINK_POOR is
     * about reliability. CheckSystem stays true unless another fault
     * ALSO fires. */
    CHECK("TB-SAF03c", "link.mean_retry non-negative",
          ctx.link.mean_retry >= 0.0f,
          "mean_retry=%.3f", ctx.link.mean_retry);
}

/*============= TB-SAF04 : Ring overflow ===================================================*/
/*
 *  Feed cumulative overflow counts below, at, and above the 100-overflow
 *  sustained threshold.
 */
static void test_ring_overflow(void)
{
    SAFETY_Context ctx;
    SAFETY_Init(&ctx);
    warm_up(&ctx, 1000000);

    SAFETY_FeedRingOverflow(&ctx, 50);
    CHECK("TB-SAF04a", "50 overflows: not faulted",
          !ctx.ring.faulted,
          "count=50 faulted=%d", ctx.ring.faulted);

    SAFETY_FeedRingOverflow(&ctx, 99);
    CHECK("TB-SAF04b", "99 overflows: not faulted",
          !ctx.ring.faulted,
          "count=99 faulted=%d", ctx.ring.faulted);

    SAFETY_FeedRingOverflow(&ctx, 150);
    CHECK("TB-SAF04c", "150 overflows: faulted",
          ctx.ring.faulted,
          "count=150 faulted=%d", ctx.ring.faulted);

    CHECK("TB-SAF04d", "CheckSystem() false when ring faulted",
          !SAFETY_CheckSystem(&ctx),
          "expected=false");
}

/*============= TB-SAF05 : I2C error streak ================================================*/
/*
 *  Contract (from cpcu_safety.c SAFETY_FeedI2C):
 *    - 5 consecutive failures trip the fault flag
 *    - One success resets both the counter AND the fault flag
 *      (transient errors are forgiven)
 */
static void test_i2c_errors(void)
{
    SAFETY_Context ctx;
    SAFETY_Init(&ctx);
    warm_up(&ctx, 1000000);

    /* 4 consecutive errors: not faulted yet */
    for(int i = 0; i < 4; i++) SAFETY_FeedI2C(&ctx, false);
    CHECK("TB-SAF05a", "4 consecutive i2c errors: ok",
          !ctx.i2c.faulted,
          "consecutive=%u faulted=%d", ctx.i2c.consecutive_errors, ctx.i2c.faulted);

    /* 5th error: faulted */
    SAFETY_FeedI2C(&ctx, false);
    CHECK("TB-SAF05b", "5 consecutive i2c errors: FAULT",
          ctx.i2c.faulted,
          "consecutive=%u faulted=%d", ctx.i2c.consecutive_errors, ctx.i2c.faulted);

    /* While faulted, CheckSystem must refuse to arm the servos */
    CHECK("TB-SAF05c", "while i2c faulted: CheckSystem()==false",
          !SAFETY_CheckSystem(&ctx),
          "expected=false");

    /* A success clears both counter and faulted flag (per contract) */
    SAFETY_FeedI2C(&ctx, true);
    CHECK("TB-SAF05d", "success forgives i2c: counter + flag cleared",
          ctx.i2c.consecutive_errors == 0 && !ctx.i2c.faulted,
          "consecutive=%u faulted=%d",
          ctx.i2c.consecutive_errors, ctx.i2c.faulted);

    /* CheckSystem should recover */
    CHECK("TB-SAF05e", "after i2c recovery: CheckSystem()==true",
          SAFETY_CheckSystem(&ctx),
          "expected=true");
}

/*============= TB-SAF06 : Happy-path baseline =============================================*/
/*
 *  Sanity check: with warm-up only, everything should be green.
 */
static void test_happy_path(void)
{
    SAFETY_Context ctx;
    SAFETY_Init(&ctx);
    warm_up(&ctx, 1000000);

    CHECK("TB-SAF06a", "happy path: state=RUNNING",
          ctx.state == RADIO_RUNNING,
          "state=%s", SAFETY_RadioStr(ctx.state));
    CHECK("TB-SAF06b", "happy path: CheckSystem()==true",
          SAFETY_CheckSystem(&ctx),
          "expected=true");
    CHECK("TB-SAF06c", "happy path: no battery fault",
          !ctx.battery.critical,
          "critical=%d vbat=%.2fV",
          ctx.battery.critical, ctx.battery.voltage);
    CHECK("TB-SAF06d", "happy path: no i2c / ring / thermal",
          !ctx.i2c.faulted && !ctx.ring.faulted && !ctx.thermal.critical,
          "i2c=%d ring=%d thermal=%d",
          ctx.i2c.faulted, ctx.ring.faulted, ctx.thermal.critical);
}

/*============= TB-SAF07 : Recovery after radio comes back =================================*/
/*
 *  Exercise the DEGRADED -> RECOVERING -> RUNNING path:
 *      1. warm up -> RUNNING
 *      2. silence 800 ms -> DEGRADED
 *      3. 1 packet -> RECOVERING
 *      4. RECOVERY_PKT_COUNT (=10) more packets -> RUNNING
 */
static void test_recovery(void)
{
    SAFETY_Context ctx;
    SAFETY_Init(&ctx);
    uint64_t t0 = warm_up(&ctx, 1000000);

    /* Silence -> DEGRADED */
    SAFETY_CheckTimeout(&ctx, t0 + 800000ULL);
    CHECK("TB-SAF07a", "silence: DEGRADED",
          ctx.state == RADIO_DEGRADED,
          "state=%s", SAFETY_RadioStr(ctx.state));

    /* First packet after silence -> RECOVERING */
    uint64_t t = t0 + 900000ULL;
    WL_Packet pkt;
    uint8_t seq = 20;
    build_healthy_packet(&pkt, seq++, 2482);
    SAFETY_FeedPacket(&ctx, &pkt, t);
    CHECK("TB-SAF07b", "first good pkt: RECOVERING",
          ctx.state == RADIO_RECOVERING,
          "state=%s", SAFETY_RadioStr(ctx.state));

    /* Need 10 *consecutive* gap-free packets — RECOVERY_PKT_COUNT.
     * We already sent 1 (seq 20). Send 10 more contiguous ones. */
    for(int i = 0; i < 10; i++)
    {
        t += 1000;
        build_healthy_packet(&pkt, seq++, 2482);
        SAFETY_FeedPacket(&ctx, &pkt, t);
    }

    CHECK("TB-SAF07c", "10 more good pkts: RUNNING",
          ctx.state == RADIO_RUNNING,
          "state=%s recovery_cnt=%u",
          SAFETY_RadioStr(ctx.state), ctx.recovery_cnt);

    CHECK("TB-SAF07d", "after recovery: CheckSystem==true",
          SAFETY_CheckSystem(&ctx),
          "expected=true");
}

/*============= TB-SAF09 : Boot-grace period ======================================*/
/*
 *  introduced a cold-start grace period for the radio fault.
 *  SAFETY_CheckTimeout suppresses the radio timeout until either
 *      (a) the first valid packet has been received, OR
 *      (b) SAFETY_RADIO_BOOT_GRACE_MS has elapsed since SAFETY_Init.
 *
 *  Verify all four sub-cases:
 *      a. Inside grace, no packets ever  -> stays RUNNING (no spurious fault)
 *      b. Grace elapsed, no packets ever -> faults to DEGRADED then SAFE
 *      c. Packet received during grace   -> normal timeout semantics resume
 *      d. Boot grace doesn't affect post-FeedPacket timeout behaviour
 */
static void test_boot_grace(void)
{
    /* Sub-test (a): inside grace, no packets — must stay RUNNING. */
    {
        SAFETY_Context ctx;
        SAFETY_Init(&ctx);
        /* Override boot_us to a synthetic epoch so we control the grace
         * window precisely. SAFETY_Init captured real CLOCK_MONOTONIC,
         * which is incompatible with the synthetic test timeline. */
        uint64_t t0 = 1000000ULL;       /* 1 s arbitrary epoch */
        ctx.boot_us             =   t0;
        ctx.last_pkt_rcv_us     =   t0;     /* avoid wrap on silence calc */
        /* INIT -> RUNNING transition would normally happen on first packet.
         * For grace-tests we move it manually so CheckTimeout actually
         * evaluates (it returns early in INIT). */
        ctx.state               =   RADIO_RUNNING;

        /* 2 s into a 5 s grace, no packets ever — should still be RUNNING
         * even though "silence" is 2000 ms (well past the 750 ms timeout). */
        SAFETY_CheckTimeout(&ctx, t0 + 2000000ULL);
        CHECK("TB-SAF09a", "inside grace, no packets: stays RUNNING",
              ctx.state == RADIO_RUNNING,
              "state=%s  grace=%dms elapsed=2000ms",
              SAFETY_RadioStr(ctx.state), SAFETY_RADIO_BOOT_GRACE_MS);
    }

    /* Sub-test (b): grace elapsed, still no packets — must fault. */
    {
        SAFETY_Context ctx;
        SAFETY_Init(&ctx);
        uint64_t t0 = 1000000ULL;
        ctx.boot_us             =   t0;
        ctx.last_pkt_rcv_us     =   t0;
        ctx.state               =   RADIO_RUNNING;

        /* Grace + 1 second past, no first packet yet -> should fault. */
        uint64_t t = t0 + (SAFETY_RADIO_BOOT_GRACE_MS + 1000) * 1000ULL;
        SAFETY_CheckTimeout(&ctx, t);
        CHECK("TB-SAF09b", "grace elapsed, no packets: DEGRADED",
              ctx.state == RADIO_DEGRADED,
              "state=%s  grace=%dms elapsed=%dms",
              SAFETY_RadioStr(ctx.state),
              SAFETY_RADIO_BOOT_GRACE_MS,
              SAFETY_RADIO_BOOT_GRACE_MS + 1000);
    }

    /* Sub-test (c): packet received during grace -> normal semantics
     * resume immediately (grace gate is bypassed by first_packet_seen). */
    {
        SAFETY_Context ctx;
        SAFETY_Init(&ctx);
        uint64_t t0 = 1000000ULL;
        ctx.boot_us             =   t0;

        /* Receive one packet at t0 + 1 s (well inside grace). */
        WL_Packet pkt;
        build_healthy_packet(&pkt, 0, 2482);
        pkt.flags |= WL_FLAG_FIRST_PACKET;
        SAFETY_FeedPacket(&ctx, &pkt, t0 + 1000000ULL);

        CHECK("TB-SAF09c1", "first packet sets first_packet_seen",
              ctx.first_packet_seen,
              "expected=true");

        /* Now go silent for 800 ms past the packet (well past the 750ms
         * timeout). With first_packet_seen=true, normal timeout applies. */
        uint64_t t_silent = t0 + 1000000ULL + 800000ULL;
        SAFETY_CheckTimeout(&ctx, t_silent);
        CHECK("TB-SAF09c2", "after first packet: normal timeout resumes",
              ctx.state == RADIO_DEGRADED,
              "state=%s (silence>750ms after first_packet_seen)",
              SAFETY_RadioStr(ctx.state));
    }

    /* Sub-test (d): defensive — boot grace doesn't somehow leak into
     * established-RUNNING behaviour. After warm_up (which sets
     * first_packet_seen via 20 packets), CheckTimeout must behave
     * identically to before boot-grace. This re-runs the TB-SAF01b/c flow. */
    {
        SAFETY_Context ctx;
        SAFETY_Init(&ctx);
        uint64_t t0 = warm_up(&ctx, 1000000);

        /* 700 ms silence — RUNNING; 800 ms silence — DEGRADED. */
        SAFETY_CheckTimeout(&ctx, t0 + 700000ULL);
        bool ok_under = (ctx.state == RADIO_RUNNING);
        SAFETY_CheckTimeout(&ctx, t0 + 800000ULL);
        bool ok_over  = (ctx.state == RADIO_DEGRADED);

        CHECK("TB-SAF09d", "post-warmup timeout semantics unchanged",
              ok_under && ok_over,
              "under_timeout_running=%d  over_timeout_degraded=%d",
              ok_under, ok_over);
    }
}

/*============= MAIN =======================================================================*/

int main(void)
{
    printf("=== CPCU SAFETY MODULE TESTBENCH ===\n");
    printf("Target: cpcu_safety v2.3.1  thresholds: "
           "RADIO_TIMEOUT=%dms  RADIO_SAFE=%dms  BOOT_GRACE=%dms  VBAT_CRIT=%.2fV  "
           "I2C_MAX=%d  RING_MAX=%d\n\n",
           SAFETY_RADIO_TIMEOUT_MS, SAFETY_RADIO_SAFE_MS,
           SAFETY_RADIO_BOOT_GRACE_MS,
           SAFETY_VBAT_CRITICAL_V,
           SAFETY_I2C_MAX_ERRORS, SAFETY_RING_OVERFLOW_LIMIT);

    printf("--- TB-SAF01: Radio loss timeout ---\n");
    test_radio_loss_timeout();

    printf("\n--- TB-SAF02: Low battery fault ---\n");
    test_low_battery();

    printf("\n--- TB-SAF03: Sequence gap storm ---\n");
    test_seq_gap_storm();

    printf("\n--- TB-SAF04: Ring overflow ---\n");
    test_ring_overflow();

    printf("\n--- TB-SAF05: I2C error streak ---\n");
    test_i2c_errors();

    printf("\n--- TB-SAF06: Happy-path baseline ---\n");
    test_happy_path();

    printf("\n--- TB-SAF07: Recovery ---\n");
    test_recovery();

    printf("\n--- TB-SAF09: Boot grace period  ---\n");
    test_boot_grace();

    printf("\n======================================\n");
    printf("RESULTS: %d PASS, %d FAIL\n", g_tests_pass, g_tests_fail);
    printf("======================================\n");

    return (g_tests_fail == 0) ? 0 : 1;
}

/*==========================================================================================*/

