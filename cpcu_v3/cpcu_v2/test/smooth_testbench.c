/**
 *  @file   smooth_testbench.c
 *  @brief  Smoother test harness — trapezoidal motion, deadband, gravity compensation.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "cpcu_smooth.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(test_id, descr, cond, ...) do { \
    if(cond) { \
        g_pass++; \
        printf("[PASS] %-10s %s\n", test_id, descr); \
    } else { \
        g_fail++; \
        printf("[FAIL] %-10s %s  ", test_id, descr); \
        printf(__VA_ARGS__); \
        printf("\n"); \
    } \
} while(0)

/*============= TB-SMO01 : Init defaults ===================================*/

static void test_init_defaults(void)
{
    SMOOTH_Context s;
    SMOOTH_Init(&s, 1500);

    bool ok_curr = true, ok_targ = true, ok_set = true, ok_deadband = true;
    bool ok_ever = true, ok_lastw = true, ok_enabled = true;
    for(int i = 0; i < PCA_SERVO_COUNT; i++)
    {
        if(s.current[i] != 1500)              ok_curr = false;
        if(s.target[i]  != 1500)              ok_targ = false;
        if(!s.settled[i])                     ok_set = false;
        if(s.hold_deadband_us[i] != SMOOTH_DEFAULT_DEADBAND) ok_deadband = false;
        if(s.ever_written[i])                 ok_ever = false;
        if(s.last_written_us[i] != 0)         ok_lastw = false;
        if(!s.enabled[i])                     ok_enabled = false;
    }

    CHECK("TB-SMO01a", "Init: current[] == start_us",        ok_curr,  "");
    CHECK("TB-SMO01b", "Init: target[] == start_us",         ok_targ,  "");
    CHECK("TB-SMO01c", "Init: settled[] all true",           ok_set,   "");
    CHECK("TB-SMO01d", "Init: hold_deadband_us == default",  ok_deadband,
          "expected %d, got %u", SMOOTH_DEFAULT_DEADBAND, s.hold_deadband_us[0]);
    CHECK("TB-SMO01e", "Init: ever_written[] all false",     ok_ever,  "");
    CHECK("TB-SMO01f", "Init: last_written_us[] == 0",       ok_lastw, "");
    CHECK("TB-SMO01g", "Init: enabled[] all true",           ok_enabled, "");
}

/*============= TB-SMO02 : Trapezoidal motion ==============================*/

static void test_motion_settles(void)
{
    SMOOTH_Context s;
    SMOOTH_Init(&s, 1500);
    SMOOTH_SetTarget(&s, 0, 1900);              /* +400 us move */

    /* Default profile: vmax 2000 us/s, accel 8000 us/s². For a 400 us
     * move we expect a triangle: t_accel = sqrt(800/8000) = 0.316 s,
     * total ≈ 0.632 s. Plus settle margin -> 1 s budget at 50 Hz =
     * 50 ticks. */
    int ticks = 0;
    while(!s.settled[0] && ticks < 100)
    {
        SMOOTH_Update(&s, 20000);
        ticks++;
    }

    CHECK("TB-SMO02a", "settled within 1 s for 400 us move",
          s.settled[0] && ticks < 50,
          "ticks=%d settled=%d", ticks, s.settled[0]);
    CHECK("TB-SMO02b", "current converged on target",
          s.current[0] == 1900,
          "current=%u target=%u", s.current[0], s.target[0]);
    CHECK("TB-SMO02c", "velocity zeroed at settle",
          s.velocity_f[0] == 0.0f,
          "velocity_f=%f", s.velocity_f[0]);
}

/*============= TB-SMO03 : Deadband logic ==================================*/

static void test_deadband_holds(void)
{
    SMOOTH_Context s;
    SMOOTH_Init(&s, 1500);

    /* Pretend the consumer wrote 1500 once, marking the shadow. */
    SMOOTH_MarkWritten(&s, 0, 1500);

    /* Servo at rest, last_written = 1500, current = 1500.
     * ShouldWrite must be FALSE — already up to date. */
    CHECK("TB-SMO03a", "settled+matched: ShouldWrite=false",
          !SMOOTH_ShouldWrite(&s, 0),
          "expected false");

    /* Move target by 5 us (less than default 10 us deadband).
     * current[0] only changes after Update, but settled stays true if
     * the move is too small for the trapezoid to even start.
     * Force the situation: directly bump current and re-mark settled. */
    s.current[0] = 1505;
    s.settled[0] = true;
    CHECK("TB-SMO03b", "settled+within-deadband: ShouldWrite=false",
          !SMOOTH_ShouldWrite(&s, 0),
          "expected false (diff=5, deadband=10)");

    /* Bump current well past the deadband. */
    s.current[0] = 1520;
    s.settled[0] = true;
    CHECK("TB-SMO03c", "settled+outside-deadband: ShouldWrite=true",
          SMOOTH_ShouldWrite(&s, 0),
          "expected true (diff=20, deadband=10)");

    /* Motion in progress always writes regardless of deadband. */
    s.current[0] = 1505;
    s.settled[0] = false;
    CHECK("TB-SMO03d", "motion in progress: ShouldWrite=true",
          SMOOTH_ShouldWrite(&s, 0),
          "expected true (settled=false)");
}

/*============= TB-SMO04 : Deadband disabled ===============================*/

static void test_deadband_disabled(void)
{
    SMOOTH_Context s;
    SMOOTH_Init(&s, 1500);
    SMOOTH_SetDeadband(&s, 0, 0);
    SMOOTH_MarkWritten(&s, 0, 1500);

    /* Even though current matches last_written exactly, deadband=0
     * forces a write every tick. */
    CHECK("TB-SMO04a", "deadband=0: always writes (no jitter suppression)",
          SMOOTH_ShouldWrite(&s, 0),
          "expected true");
}

/*============= TB-SMO05 : Initial-write rule ==============================*/

static void test_initial_write(void)
{
    SMOOTH_Context s;
    SMOOTH_Init(&s, 1500);
    /* No SMOOTH_MarkWritten yet -> ever_written is false. ShouldWrite
     * must be true even though smoother is settled at 1500. */
    CHECK("TB-SMO05a", "ever_written=false: ShouldWrite=true (initial PWM)",
          SMOOTH_ShouldWrite(&s, 0),
          "expected true");

    SMOOTH_MarkWritten(&s, 0, 1500);
    CHECK("TB-SMO05b", "after first MarkWritten: ShouldWrite=false (settled)",
          !SMOOTH_ShouldWrite(&s, 0),
          "expected false");
}

/*============= TB-SMO06 : MarkWritten coherence ===========================*/

static void test_markwritten_coherence(void)
{
    SMOOTH_Context s;
    SMOOTH_Init(&s, 1500);
    SMOOTH_MarkWritten(&s, 2, 1700);

    CHECK("TB-SMO06a", "MarkWritten updates last_written_us",
          s.last_written_us[2] == 1700,
          "got %u", s.last_written_us[2]);
    CHECK("TB-SMO06b", "MarkWritten flips ever_written",
          s.ever_written[2],
          "got false");
    CHECK("TB-SMO06c", "MarkWritten on ch=2 doesn't touch ch=0",
          !s.ever_written[0] && s.last_written_us[0] == 0,
          "ch0 ever=%d last=%u", s.ever_written[0], s.last_written_us[0]);
}

/*============= TB-SMO07 : Snap preserves deadband state ===================*/

static void test_snap_preserves_deadband(void)
{
    SMOOTH_Context s;
    SMOOTH_Init(&s, 1500);
    SMOOTH_MarkWritten(&s, 0, 1500);
    SMOOTH_SetTarget(&s, 0, 1900);

    /* Mid-motion: settled becomes false, ever_written stays true. */
    SMOOTH_Update(&s, 20000);
    bool was_moving = !s.settled[0];

    /* Snap to target: zero velocity, settled=true. ever_written and
     * last_written must be UNCHANGED (Snap is internal, not a PCA write). */
    SMOOTH_Snap(&s);

    CHECK("TB-SMO07a", "after Update: smoother in motion",
          was_moving,
          "expected false-settled");
    CHECK("TB-SMO07b", "Snap zeroes velocity",
          s.velocity_f[0] == 0.0f,
          "got %f", s.velocity_f[0]);
    CHECK("TB-SMO07c", "Snap sets settled=true",
          s.settled[0],
          "got false");
    CHECK("TB-SMO07d", "Snap preserves ever_written",
          s.ever_written[0],
          "got false");
    CHECK("TB-SMO07e", "Snap preserves last_written_us",
          s.last_written_us[0] == 1500,
          "got %u (expected 1500)", s.last_written_us[0]);
    /* Now Snap moved current to target=1900. last_written is 1500.
     * Diff is 400 (>> deadband 10), so the next caller MUST write. */
    CHECK("TB-SMO07f", "post-Snap: ShouldWrite=true (current diverged from PCA)",
          SMOOTH_ShouldWrite(&s, 0),
          "expected true (current=%u last_written=%u)",
          s.current[0], s.last_written_us[0]);
}

/*============= TB-SMO08 : Out-of-range channel safety =====================*/

static void test_oor_channel_safety(void)
{
    SMOOTH_Context s;
    SMOOTH_Init(&s, 1500);

    /* These should not crash. Either silently no-op or return false. */
    SMOOTH_SetDeadband(&s, -1, 100);
    SMOOTH_SetDeadband(&s, PCA_SERVO_COUNT, 100);
    SMOOTH_MarkWritten(&s, -1, 1500);
    SMOOTH_MarkWritten(&s, PCA_SERVO_COUNT + 5, 1500);

    bool ok_neg = !SMOOTH_ShouldWrite(&s, -1);
    bool ok_oor = !SMOOTH_ShouldWrite(&s, PCA_SERVO_COUNT);

    CHECK("TB-SMO08a", "ShouldWrite(-1) = false (oor)",
          ok_neg, "expected false");
    CHECK("TB-SMO08b", "ShouldWrite(N) = false (oor)",
          ok_oor, "expected false");
}

/*============= MAIN =======================================================*/

int main(void)
{
    printf("=== CPCU SMOOTHER TESTBENCH ===\n");
    printf("Target: cpcu_smooth v2.1  defaults: "
           "deadband=%d us, vmax=%d us/s, amax=%d us/s²\n\n",
           SMOOTH_DEFAULT_DEADBAND,
           SMOOTH_DEFAULT_VELOCITY,
           SMOOTH_DEFAULT_ACCEL);

    printf("--- TB-SMO01: Init defaults ---\n");
    test_init_defaults();

    printf("\n--- TB-SMO02: Trapezoidal motion settles ---\n");
    test_motion_settles();

    printf("\n--- TB-SMO03: Deadband holds settled servos ---\n");
    test_deadband_holds();

    printf("\n--- TB-SMO04: Deadband disabled = always writes ---\n");
    test_deadband_disabled();

    printf("\n--- TB-SMO05: Initial-write rule ---\n");
    test_initial_write();

    printf("\n--- TB-SMO06: MarkWritten coherence ---\n");
    test_markwritten_coherence();

    printf("\n--- TB-SMO07: Snap preserves deadband state ---\n");
    test_snap_preserves_deadband();

    printf("\n--- TB-SMO08: Out-of-range channel safety ---\n");
    test_oor_channel_safety();

    printf("\n======================================\n");
    printf("RESULTS: %d PASS, %d FAIL\n", g_pass, g_fail);
    printf("======================================\n");

    return (g_fail == 0) ? 0 : 1;
}

