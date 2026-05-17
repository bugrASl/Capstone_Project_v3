/**
 *  @file       cpcu_io.c
 *  @brief      Core 3 — Real-time I/O controller
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.3.7
 *  @details    Deterministic loop:
 *                  1. Busy-poll NRF via spidev         -> read payload
 *                  2. WL_Unpack -> seq/safety/link     -> push to SPSC ring
 *                  3. Rate-limited (50 Hz) PCA servo update via I2C
 *                  4. Safety checks: radio, DSP, I2C, thermal, ring
 *                  5. Heartbeat to shared memory for watchdog
 *
 *              v2.3.7 changes:
 *                  - Gripper stall watchdog. When the smoother current
 *                    has been at servo_min[5] (within 5 us) for
 *                    grip_stall_recover_ms continuously, AND the target
 *                    is still asking it to stay there, retreat to
 *                    grip_touch_us. While active, incoming target[5]
 *                    is clamped at grip_touch_us as a floor. Clears on
 *                    target rising above touch+margin for 250 ms, or
 *                    on SAFE. The io_gripper_stalls counter in
 *                    IPC_Diagnostics tracks fires. Hardware-protection
 *                    backstop to dsp's soft-firm clamp; see SOFT_GRIP.md.
 *
 *              v2.3.6 changes:
 *                  - Smoother per-channel velocity / accel / deadband
 *                    are now consumed from IPC_RuntimeConfig at startup
 *                    AND on every config_seq change (i.e. after the
 *                    kernel reloads runtime.json on SIGHUP). The
 *                    apply_runtime_smoother_cfg helper calls SMOOTH_Set*
 *                    for each channel; zero values are skipped (mean
 *                    "use compile-time default"). The seq-compare
 *                    avoids redundant reapplies on the steady-state
 *                    50 Hz tick. See docs/SMOOTHER_TUNING.md.
 *
 *              v2.3.4 changes:
 *                  - Edit-mode handshake responder. When the TUI sets
 *                    ipc.ctrl->edit_mode_request = 1, cpcu_io overrides
 *                    incoming motor commands, parks the smoother at
 *                    neutral, and once SMOOTH_AllSettled() goes true
 *                    flips ipc.ctrl->edit_mode_active = 1 to tell the
 *                    TUI it's safe to edit. While active is 1, motor
 *                    commands from cpcu_dsp.py are silently ignored
 *                    (sticky-park). On request -> 0, active drops and
 *                    normal motor-cmd processing resumes. Safety FSM
 *                    has priority — any SAFE transition forces
 *                    edit_mode_active back to 0 unconditionally. Full
 *                    protocol in docs/EDIT_MODE.md.
 *
 *              v2.3.3 changes:
 *                  - Reads IPC_RuntimeConfig once per servo tick
 *                    (cfg_cache local). Per-servo bias offsets
 *                    (cfg_cache.servo_bias_us[]) are added to the
 *                    smoothed pulse-width before clamping to
 *                    compile-time hardware limits and writing the PCA.
 *                    Bias is signed (typically +/- 20 us) and is
 *                    intended for static gravity-sag compensation;
 *                    runtime tunable from JSON, picked up on next
 *                    SIGHUP-driven kernel reload. The bias-then-clamp
 *                    order means the runtime config can never escape
 *                    the compile-time safety envelope. See
 *                    docs/RUNTIME_CONFIG.md.
 *                  - SMOOTH_MarkWritten now records the BIASED pulse
 *                    width that actually went to the PCA, so the
 *                    deadband logic stays coherent against the true
 *                    hardware state.
 *
 *              v2.3.2 changes:
 *                  - Per-servo PCA writes are now gated on
 *                    SMOOTH_ShouldWrite() to suppress redundant refreshes
 *                    of settled servos. A servo holding its target within
 *                    its hold_deadband_us[] window stops being commanded,
 *                    which kills static jitter caused by the servo's
 *                    internal P controller fighting backlash and gravity
 *                    sag on every 50 Hz tick.
 *                  - SAFETY_FeedI2C is only invoked on ticks that
 *                    actually performed I/O, so a pure-deadband tick
 *                    (all servos settled, no writes) doesn't pollute
 *                    the I²C health counters.
 *                  - SMOOTH_MarkWritten called after each successful
 *                    write to keep the deadband shadow coherent.
 *                  - PCA_SetAllNeutral (SAFE-snap path) and PCA_AllOff
 *                    (I²C-streak path) update the shadow appropriately:
 *                    the SAFE path marks all written; the AllOff path
 *                    clears ever_written so the first post-recovery
 *                    write always goes through.
 *                  - See docs/JITTER_MITIGATION.md.
 *
 *              v2.3 changes:
 *                  - Now calls SAFETY_UpdateState() once per loop after the
 *                    Feed/Check calls. The v2.2 architectural change that
 *                    moved battery / thermal / i2c / ring transitions out
 *                    of FeedPacket and into UpdateState was never wired up
 *                    here; without this call the FSM would update boolean
 *                    flags but never move out of RUNNING for non-radio
 *                    faults. SAFETY_CheckSystem already gated on the flags
 *                    so servos were neutralised correctly, but the TUI
 *                    state indicator lied during a battery / thermal /
 *                    ring fault.
 *
 *              v2.2 changes:
 *                  - Uses cpcu_log LOG_* macros everywhere (no more printf)
 *                  - Supports --log flag for per-module CSV output
 *                  - On sustained SPI errors, now calls NRF_FlushRX / NRF_ClearIRQ /
 *                    NRF_PowerDown before re-init (cleaner recovery)
 *                  - Reports NRF_GetStatus + STATUS bits in 1 Hz telemetry
 *                  - Calls PCA_AllOff on graceful shutdown (instead of neutral)
 *                    so no current flows after exit
 *                  - Reports SMOOTH_AllSettled() in telemetry for motion tracking
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>

#include "nrf24l01_linux.h"
#include "wireless_packet.h"
#include "cpcu_ipc.h"
#include "cpcu_pca9685.h"
#include "cpcu_safety.h"
#include "cpcu_smooth.h"
#include "cpcu_log.h"
#include "cpcu_config.h"

/*============= CONFIG =====================================================================*/

#define SPI_DEVICE              "/dev/spidev0.0"
#define SPI_SPEED               8000000
#define I2C_DEVICE              "/dev/i2c-1"
#define PCA9685_ADDR            0x40
#define GPIO_CE                 25
#define NRF_CHANNEL             108
#define NRF_ADDRESS             {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}

#define NRF_INIT_RETRIES        3
#define NRF_INIT_POR_MS         200
#define NRF_REINIT_INTERVAL_US  3000000ULL      /* 3 s */
#define SERVO_INTERVAL_US       20000ULL        /* 50 Hz */
#define HEARTBEAT_INTERVAL_US   100000ULL       /* 100 ms */
#define DIAG_INTERVAL_US        1000000ULL      /* 1 s */
#define THERMAL_INTERVAL_US     5000000ULL      /* 5 s */

#define THERMAL_PATH            "/sys/class/thermal/thermal_zone0/temp"

/*============= TIMING =====================================================================*/

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

static void sleep_ms(uint32_t ms)
{
    struct timespec ts  =   { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/*============= THERMAL READ ===============================================================*/

static float read_cpu_temp(void)
{
    FILE *f                         =   fopen(THERMAL_PATH, "r");
    if(!f) return 0.0f;

    int millidegrees                =   0;
    if(fscanf(f, "%d", &millidegrees) != 1)
        millidegrees                =   0;
    fclose(f);

    return millidegrees / 1000.0f;
}

/*============= GLOBALS ====================================================================*/

static volatile sig_atomic_t g_run  =   1;

static void on_signal(int s)
{
    (void)s; g_run = 0;
}

/* v2.3.6: Apply per-channel smoother values from the runtime config.
 * Zero values mean "use compile-time default" — we skip those, leaving
 * whatever was set previously (typically the SMOOTH_DEFAULT_* values
 * from SMOOTH_Init). Non-zero values override per-channel.
 *
 * Called at startup and whenever cfg.config_seq changes (i.e. on
 * SIGHUP-driven kernel reloads). The setters are cheap — three
 * memory writes per channel — so calling this on every reload is
 * fine even though it's redundant when values haven't actually
 * changed for this channel. */
static void apply_runtime_smoother_cfg(SMOOTH_Context *smooth,
                                       const IPC_RuntimeConfig *cfg)
{
    for(int s = 0; s < PCA_SERVO_COUNT; s++)
    {
        if(cfg->smooth_velocity_us_per_s[s] != 0)
            SMOOTH_SetSpeed(smooth, s, cfg->smooth_velocity_us_per_s[s]);
        if(cfg->smooth_accel_us_per_s2[s] != 0)
            SMOOTH_SetAccel(smooth, s, cfg->smooth_accel_us_per_s2[s]);
        /* Deadband: zero IS a valid value (means "no deadband, write
         * every tick"), so we always apply rather than gate on != 0.
         * But the JSON loader's range check ensures 0..50, and the
         * default in CFG_Defaults is 10, so a fresh-loaded config
         * always has sensible deadband values. */
        SMOOTH_SetDeadband(smooth, s, cfg->smooth_deadband_us[s]);
    }
}

/*============= MAIN =======================================================================*/

int main(int argc, char *argv[])
{
    /* Bring up logging first so every step below is captured */
    Log_Init("IO", LOG_INFO);

    /* Parse CLI */
    bool enable_file_log  =   false;
    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--log") == 0)
            enable_file_log = true;
        else if(strcmp(argv[i], "--debug") == 0)
            Log_SetLevel(LOG_DEBUG);
        else if(strcmp(argv[i], "--trace") == 0)
            Log_SetLevel(LOG_TRACE);
    }
    if(enable_file_log)
    {
        Log_EnableFiles(LOG_DIR_DEFAULT);
        LOG_I("IO", "file logging enabled -> %s/log_*.csv", LOG_DIR_DEFAULT);
    }

    LOG_I("IO", "=== CPCU I/O Controller (Core 3) v2.3.7 ===");
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGHUP,  on_signal);  /* v3: safe shutdown on tmux kill */

    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Open GPIO chip */
    int gpio_fd =   open("/dev/gpiochip4", O_RDWR);
    if(gpio_fd < 0)
    {
        gpio_fd =   open("/dev/gpiochip0", O_RDWR);
    }
    if(gpio_fd < 0)
    {
        LOG_F("IO", "gpiochip open failed: %s", strerror(errno));
        Log_CloseFiles();
        return 1;
    }

    /* Open IPC */
    IPC_Context ipc;
    if(IPC_Open(&ipc) != 0)
    {
        LOG_F("IO", "IPC open failed — is cpcu_kernel running?");
        close(gpio_fd);
        Log_CloseFiles();
        return 1;
    }

    /*  Init NRF with retry */
    NRF_Handle nrf;
    const uint8_t addr[NRF_ADDR_WIDTH]  =   NRF_ADDRESS;
    NRF_Status nrf_ret                  =   NRF_ERR_NOT_DETECTED;
    bool nrf_ok                         =   false;

    for(int i = 0; i < NRF_INIT_RETRIES; i++)
    {
        if(i > 0)
        {
            sleep_ms(NRF_INIT_POR_MS);
        }
        nrf_ret =   NRF_Init(&nrf, SPI_DEVICE, SPI_SPEED,
                            gpio_fd, GPIO_CE,
                            NRF_CHANNEL, addr);
        if(nrf_ret == NRF_OK)
        {
            nrf_ok = true;
            /* Read back the config register as a post-init sanity check */
            uint8_t cfg     =   NRF_ReadReg(&nrf, 0x00);       /* CONFIG */
            uint8_t en_rxa  =   NRF_ReadReg(&nrf, 0x02);       /* EN_RXADDR */
            uint8_t rf_ch   =   NRF_ReadReg(&nrf, 0x05);       /* RF_CH */
            LOG_I("NRF", "init OK  CONFIG=0x%02X EN_RXADDR=0x%02X RF_CH=%u",
                  cfg, en_rxa, rf_ch);
            break;
        }

        LOG_W("NRF", "init attempt %d failed (%d)", i + 1, nrf_ret);
    }
    atomic_store(&ipc.diag->io_nrf_init_status, (uint32_t)nrf_ret);

    /* Init PCA9685 */
    PCA_Handle pca;
    PCA_Status pca_init_ret =   PCA_Init(&pca, I2C_DEVICE, PCA9685_ADDR);
    bool       pca_ok       =   (pca_init_ret == PCA_OK);
    if(pca_ok)
    {
        /* Read back MODE1 as a connectivity check */
        uint8_t mode1 = 0;
        if(PCA_ReadReg(&pca, PCA_REG_MODE1, &mode1) == PCA_OK)
            LOG_I("PCA", "init OK  addr=0x%02X prescale=%u MODE1=0x%02X",
                  pca.addr, pca.prescaler, mode1);
    }
    else
    {
        LOG_E("PCA", "init failed (%d)", pca_init_ret);
    }

    /* Init safety */
    SAFETY_Context safety;
    SAFETY_Init(&safety);

    /* Init servo smoother (start at neutral) */
    SMOOTH_Context smooth;
    SMOOTH_Init(&smooth, PCA_SERVO_NEUTRAL);

    /* Gripper (S5) bypasses the smoother — grip/release must be
     * immediate so objects don't slip during the ramp delay. */
    SMOOTH_SetEnabled(&smooth, 5, false);

    /* Signal ready */
    atomic_store(&ipc.ctrl->io_ready, 1);
    atomic_store(&ipc.ctrl->system_state, nrf_ok ? IPC_STATE_RUNNING : IPC_STATE_SAFE);
    LOG_I("IO", "ready (NRF=%s PCA=%s). Entering loop.",
          nrf_ok ? "OK" : "FAIL", pca_ok ? "OK" : "FAIL");

    /* Loop state */
    uint8_t     raw[NRF_PAYLOAD_SIZE];
    WL_Packet   pkt;
    uint16_t    servo_us[PCA_SERVO_COUNT];
    uint8_t     gesture_id;
    uint8_t     confidence;

    uint32_t    mcmd_ack    =   0;
    uint64_t    t_servo     =   now_us();
    uint64_t    t_hb        =   now_us();
    uint64_t    t_diag      =   now_us();
    uint64_t    t_thermal   =   now_us();
    uint64_t    t_reinit    =   0;
    uint32_t    i2c_err_streak  =   0;

    /* v2.3.3: runtime config snapshot. We read the IPC region into this
     * local copy at the top of every servo-tick (50 Hz), so updates from
     * cpcu_kernel's SIGHUP reload show up within ~20 ms and there's no
     * risk of a torn read mid-write. The seqlock retry logic is in
     * IPC_ReadRuntimeConfig itself.
     *
     * If the read fails (writer is mid-update or magic invalid), the
     * cache keeps its previous values — better to use slightly stale
     * config than zero-filled garbage. */
    IPC_RuntimeConfig cfg_cache;
    if(!IPC_ReadRuntimeConfig(&ipc, &cfg_cache))
    {
        /* Initial read failed — fall back to compile-time defaults so
         * we have *something* sane while the kernel re-tries. */
        CFG_Defaults(&cfg_cache);
        LOG_W("IO", "initial config read failed, using defaults");
    }

    /* v2.3.6: Apply runtime smoother config from the loaded snapshot.
     * Zero values mean "use compile-time default" — see cpcu_smooth.h
     * for SMOOTH_DEFAULT_VELOCITY/_ACCEL/_DEADBAND. We only call the
     * setters when the runtime value is non-zero, so a partly-filled
     * runtime.json (some servos tuned, others left at zero) does the
     * right thing without surprising the user.
     *
     * The seq counter we capture here lets us detect SIGHUP-driven
     * reloads without re-applying every tick — see the loop body. */
    uint32_t cfg_seq_seen = cfg_cache.config_seq;
    apply_runtime_smoother_cfg(&smooth, &cfg_cache);

    /* v2.3.9: gravity compensation for weight-bearing joints.
     * Loaded from runtime.json; falls back to hardcoded defaults for
     * S1 (gravity_dir=-1, 30%) and S2 (gravity_dir=+1, 30%). */
    for(int s = 0; s < PCA_SERVO_COUNT; s++)
    {
        if(cfg_cache.gravity_dir[s] != 0)
            SMOOTH_SetGravity(&smooth, s, (int8_t)cfg_cache.gravity_dir[s],
                              (float)cfg_cache.gravity_scale_pct[s] / 100.0f);
    }
    /* Defaults for S1/S2 if not in config */
    if(cfg_cache.gravity_dir[1] == 0)
        SMOOTH_SetGravity(&smooth, 1, -1, 0.30f);
    if(cfg_cache.gravity_dir[2] == 0)
        SMOOTH_SetGravity(&smooth, 2,  1, 0.30f);

    /* v2.3.7: gripper stall watchdog state. The watchdog fires when
     * the smoother current[5] has been at servo_min[5] (within a
     * small margin) for grip_stall_recover_ms continuously, AND the
     * smoother target is still asking it to stay there. The retreat
     * is to grip_touch_us; while active, incoming motor_cmd targets
     * for channel 5 are clamped to grip_touch_us so dsp can't keep
     * pinning us back to the floor. Cleared when target rises above
     * grip_touch_us + margin for 250 ms (debounce — avoids twitchy
     * re-engage on a single dropped tick).
     *
     * 0 in gripper_at_floor_since_us means "not currently at floor".
     * gripper_unstall_since_us tracks the debounce for clearing
     * stall_active. See SOFT_GRIP.md for the state diagram. */
    uint64_t gripper_at_floor_since_us = 0;
    uint64_t gripper_unstall_since_us  = 0;
    bool     gripper_stall_active       = false;

    /*---------------------------------------------------------------------------*/

    while(g_run)
    {
        uint64_t t  =   now_us();

        /* 1. NRF poll */
        if(nrf_ok && NRF_DataAvailable(&nrf))
        {
            while(NRF_ReadPayload(&nrf, raw) == NRF_OK)
            {
                WL_Unpack(raw, &pkt);

                uint32_t gap    =   SAFETY_SeqGap(&safety, pkt.seq);
                if(gap > 0)
                {
                    atomic_fetch_add(&ipc.diag->io_seq_gaps, gap);
                }

                SAFETY_FeedPacket(&safety, &pkt, t);
                    /* BSAU no longer samples battery — force-clear the
                     * critical flag so stale/zero vbat_raw doesn't trip
                     * SAFE and block the arm. */
                    safety.battery.critical = false;
                IPC_PushSensor(&ipc, &pkt, t);
                atomic_fetch_add(&ipc.diag->io_pkts_received, 1);
            }

            /* Belt-and-braces: clear RX_DR (bit 6) if it's still asserted.
             * NRF_ReadPayload already clears it, but this guards against
             * a stuck FIFO ISR from a spurious glitch. */
            uint8_t sr = NRF_GetStatus(&nrf);
            if(sr & 0x40)
                NRF_ClearIRQ(&nrf, 0x40);
        }

        /* 2. Safety: radio timeout */
        SAFETY_CheckTimeout(&safety, t);

        /* 3. Safety: DSP stall check */
        SAFETY_CheckDSP(&safety, t);

        /* 4. Safety: ring buffer overflow (v2.3 — recoverable) */
        SAFETY_FeedRingOverflow(&safety, atomic_load(&ipc.diag->io_ring_overflows));

        /* 5. Safety: drive non-radio FSM transitions (v2.3 — was dead in v2.2).
         * UpdateState owns the RUNNING <-> SAFE transitions for battery /
         * thermal / dsp / i2c / ring fault flags. Without this call the
         * boolean flags would update correctly (so SAFETY_CheckSystem()
         * already neutralises the servos), but the FSM state shown in the
         * TUI would stay RUNNING through any non-radio fault. */
        SAFETY_UpdateState(&safety, t);

        /* 6. Update system state in shared memory.
         * v2.3.9: DEGRADED and RECOVERING map to IPC_STATE_RUNNING (not INIT)
         * because the system IS receiving packets — it's just in a transitional
         * recovery phase. Only SAFE means servos are parked. INIT is reserved
         * for the genuine pre-first-packet state. */
        {
            uint32_t ipc_state;
            switch(safety.state)
            {
                case RADIO_SAFE:        ipc_state = IPC_STATE_SAFE;     break;
                case RADIO_INIT:        ipc_state = IPC_STATE_INIT;     break;
                default:                ipc_state = IPC_STATE_RUNNING;  break;
            }

            /* v2.3.9: track SAFE entries for diagnostics.
             * Increment counter on the RUNNING/RECOVERING → SAFE edge. */
            uint32_t prev_state = atomic_load(&ipc.ctrl->system_state);
            if(ipc_state == IPC_STATE_SAFE && prev_state != IPC_STATE_SAFE)
                atomic_fetch_add(&ipc.diag->io_safe_entries, 1);

            atomic_store(&ipc.ctrl->system_state, ipc_state);
        }

        /* 7. NRF re-init if failed (full clean-reset sequence) */
        if(!nrf_ok && (t - t_reinit) > NRF_REINIT_INTERVAL_US)
        {
            t_reinit    =   t;
            LOG_W("NRF", "attempting recovery...");

            /* If we still have an SPI handle, drain FIFOs and power down
             * cleanly before re-initing. Wires in NRF_FlushRX/TX/PowerDown
             * which were previously dead code. */
            NRF_FlushRX(&nrf);
            NRF_FlushTX(&nrf);
            NRF_ClearIRQ(&nrf, 0x70);       /* all IRQ flags */
            NRF_PowerDown(&nrf);
            sleep_ms(50);

            nrf_ret     =   NRF_Init(&nrf, SPI_DEVICE, SPI_SPEED,
                                    gpio_fd, GPIO_CE, NRF_CHANNEL, addr);

            atomic_store(&ipc.diag->io_nrf_init_status, (uint32_t)nrf_ret);
            if(nrf_ret == NRF_OK)
            {
                nrf_ok = true;
                LOG_I("NRF", "recovered");
            }
            else
            {
                LOG_E("NRF", "recovery failed (%d)", nrf_ret);
            }
        }

        /* 8. Servo update (50 Hz with smoothing) */
        if((t - t_servo) >= SERVO_INTERVAL_US)
        {
            uint32_t servo_dt   =   (uint32_t)(t - t_servo);
            t_servo =   t;

            if(SAFETY_CheckSystem(&safety) && pca_ok)
            {
                /* v2.3.4: Edit-mode handshake.
                 * If the TUI has requested edit mode, override any incoming
                 * motor command with neutral. Once the smoother has walked
                 * to neutral and settled, set edit_mode_active = 1 to tell
                 * the TUI it's safe to edit. While active, motor commands
                 * stay ignored (sticky-park). On exit, drop active and
                 * resume normal motor cmd processing. Safety FSM has
                 * priority — a fault forces edit mode off.
                 *
                 * This sits inside the safety-OK + pca-OK branch so faults
                 * naturally drop edit mode (the SAFE branch below will
                 * snap to neutral and clear active). */
                uint8_t edit_req = atomic_load_explicit(
                    &ipc.ctrl->edit_mode_request, memory_order_acquire);

                if(edit_req)
                {
                    /* Park at neutral. Don't touch IPC motor_cmd —
                     * cpcu_dsp.py is still publishing but we ignore it. */
                    uint16_t neutral_targets[PCA_SERVO_COUNT];
                    for(int s = 0; s < PCA_SERVO_COUNT; s++)
                        neutral_targets[s] = PCA_SERVO_NEUTRAL;
                    SMOOTH_SetAllTargets(&smooth, neutral_targets);

                    /* Once we've walked there, signal active. The TUI
                     * can then enable its editor and start writing
                     * runtime.json. We keep republishing this every
                     * tick (idempotent atomic store) so a TUI that
                     * connects late still sees the right state. */
                    uint8_t active_now = SMOOTH_AllSettled(&smooth) ? 1 : 0;
                    atomic_store_explicit(&ipc.ctrl->edit_mode_active,
                                          active_now, memory_order_release);
                }
                else
                {
                    /* Not in edit mode: clear active (idempotent),
                     * normal motor-cmd path. */
                    atomic_store_explicit(&ipc.ctrl->edit_mode_active,
                                          0, memory_order_release);

                    if(IPC_ReadMotorCmd(&ipc, servo_us, &gesture_id, &confidence, &mcmd_ack))
                    {
                        /* Notify safety: DSP is alive */
                        SAFETY_FeedMotorCMD(&safety, t);

                        /* Clamp targets to safe range, feed to smoother */
                        PCA_SafetyClamp(&pca, servo_us);
                        SMOOTH_SetAllTargets(&smooth, servo_us);
                    }
                }

                /* Advance smoother toward targets */
                SMOOTH_Update(&smooth, servo_dt);

                /* v2.3.7: gripper stall watchdog. Two-state machine.
                 *
                 * INACTIVE: monitor whether current[5] is at the floor
                 *   (within MARGIN_US). If yes for grip_stall_recover_ms
                 *   continuously, FIRE: snap target to grip_touch_us,
                 *   set active=true, increment counter.
                 *
                 * ACTIVE: clamp incoming target[5] to grip_touch_us as
                 *   a floor (doesn't constrain opening). When target
                 *   naturally rises above grip_touch_us + MARGIN_US for
                 *   grip_unstall_debounce_us, clear active.
                 *
                 * The stall is detected on the smoother's CURRENT, not
                 * its TARGET — the target may already have been clamped
                 * to grip_firm_us by dsp's soft policy. We care about
                 * whether the physical position has been pinned. */
                #define WD_MARGIN_US           5
                #define WD_UNSTALL_DEBOUNCE_US (250 * 1000ULL)
                {
                    uint16_t cur5 = smooth.current[5];
                    uint16_t tgt5 = smooth.target[5];
                    uint16_t floor_us = pca.servo_min[5];
                    uint16_t touch_us = cfg_cache.grip_touch_us
                                          ? cfg_cache.grip_touch_us : 1200;
                    uint64_t recover_us = (uint64_t)
                        (cfg_cache.grip_stall_recover_ms
                          ? cfg_cache.grip_stall_recover_ms : 2000) * 1000ULL;

                    bool at_floor = (cur5 <= floor_us + WD_MARGIN_US) &&
                                    (tgt5 <= floor_us + WD_MARGIN_US);

                    if(!gripper_stall_active)
                    {
                        if(at_floor)
                        {
                            if(gripper_at_floor_since_us == 0)
                                gripper_at_floor_since_us = t;
                            else if((t - gripper_at_floor_since_us) > recover_us)
                            {
                                /* FIRE: retreat. */
                                SMOOTH_SetTarget(&smooth, 5, touch_us);
                                gripper_stall_active = true;
                                gripper_unstall_since_us = 0;
                                atomic_fetch_add_explicit(
                                    &ipc.diag->io_gripper_stalls, 1u,
                                    memory_order_relaxed);
                                LOG_W("IO", "gripper stall watchdog fired -> "
                                            "retreat to %u us (was at floor "
                                            "%llu ms)",
                                      (unsigned)touch_us,
                                      (unsigned long long)
                                        ((t - gripper_at_floor_since_us) / 1000));
                            }
                        }
                        else
                        {
                            gripper_at_floor_since_us = 0;
                        }
                    }
                    else /* gripper_stall_active */
                    {
                        /* Override incoming target while active.
                         * SMOOTH_SetAllTargets called above may have
                         * just set target[5] back to whatever dsp is
                         * publishing. Re-clamp to touch as a floor. */
                        if(smooth.target[5] < touch_us)
                            SMOOTH_SetTarget(&smooth, 5, touch_us);

                        /* Recovery: target naturally above touch+margin
                         * for the debounce window. */
                        if(smooth.target[5] > touch_us + WD_MARGIN_US)
                        {
                            if(gripper_unstall_since_us == 0)
                                gripper_unstall_since_us = t;
                            else if((t - gripper_unstall_since_us)
                                     > WD_UNSTALL_DEBOUNCE_US)
                            {
                                gripper_stall_active = false;
                                gripper_at_floor_since_us = 0;
                                gripper_unstall_since_us  = 0;
                                LOG_I("IO", "gripper stall cleared");
                            }
                        }
                        else
                        {
                            gripper_unstall_since_us = 0;
                        }
                    }
                }
                #undef WD_MARGIN_US
                #undef WD_UNSTALL_DEBOUNCE_US

                /* v2.3.3: refresh runtime config snapshot. If the kernel
                 * just reloaded JSON (SIGHUP), this picks up the new
                 * values within one servo tick. Failed reads (writer
                 * mid-update) leave cfg_cache untouched. */
                IPC_ReadRuntimeConfig(&ipc, &cfg_cache);

                /* v2.3.6: detect SIGHUP-driven config changes and
                 * re-apply per-channel smoother values when seq has
                 * advanced. The seq-compare avoids reapplying every
                 * tick; on a typical session the apply runs once at
                 * boot and once per kill -HUP. */
                if(cfg_cache.config_seq != cfg_seq_seen)
                {
                    apply_runtime_smoother_cfg(&smooth, &cfg_cache);
                    cfg_seq_seen = cfg_cache.config_seq;
                    LOG_I("IO", "smoother config re-applied "
                                "(seq %u -> %u)",
                          (unsigned)cfg_seq_seen, (unsigned)cfg_cache.config_seq);
                }

                /* Write smoothed positions to servos (one I2C burst per
                 * channel). v2.3.2: gate each write on SMOOTH_ShouldWrite
                 * to suppress redundant refreshes of settled servos —
                 * see docs/JITTER_MITIGATION.md. v2.3.3: apply per-servo
                 * bias offset (gravity-sag compensation) to the smoothed
                 * value before clamping & write — see RUNTIME_CONFIG.md.
                 * Servos still in motion, or sitting outside the deadband
                 * from their last latched value, write every tick as before. */
                PCA_Status pca_ret  =   PCA_OK;
                bool       any_io   =   false;
                for(int s = 0; s < PCA_SERVO_COUNT; s++)
                {
                    if(!SMOOTH_ShouldWrite(&smooth, s))
                        continue;       /* deadband — let the servo coast */

                    /* v2.3.3: bias is signed (typ. +/- 20 us). Clamp the
                     * BIASED value to compile-time hardware limits — the
                     * runtime config can't escape the safety envelope. */
                    int32_t biased = (int32_t)smooth.current[s] +
                                     (int32_t)cfg_cache.servo_bias_us[s];
                    if(biased < (int32_t)pca.servo_min[s]) biased = pca.servo_min[s];
                    if(biased > (int32_t)pca.servo_max[s]) biased = pca.servo_max[s];
                    uint16_t pulse_us = (uint16_t)biased;

                    PCA_Status r    =   PCA_SetServo(&pca, (uint8_t)s, pulse_us);
                    if(r == PCA_OK)
                    {
                        SMOOTH_MarkWritten(&smooth, s, pulse_us);
                    }
                    else
                    {
                        pca_ret =   r;
                    }
                    any_io = true;
                }

                /* Only feed I2C health if we actually performed I/O this
                 * tick. A pure-deadband tick (all servos settled, no
                 * writes) shouldn't be miscounted as either a success
                 * or a failure. */
                if(any_io)
                {
                    SAFETY_FeedI2C(&safety, pca_ret == PCA_OK);

                    if(pca_ret == PCA_OK)   i2c_err_streak  =   0;
                    else                    i2c_err_streak++;
                }

                /* If I2C has been failing for over 500 ms (25 ticks @ 50 Hz),
                 * assume the bus is wedged and kill all outputs with PCA_AllOff.
                 * This wires in PCA_AllOff from cpcu_io (previously only used
                 * in pca_testbench). */
                if(i2c_err_streak >= 25)
                {
                    LOG_E("PCA", "I2C streak %u — forcing AllOff",
                          i2c_err_streak);
                    PCA_AllOff(&pca);
                    i2c_err_streak = 0;

                    /* v2.3.2: AllOff clears the PCA's PWM registers, so
                     * what was "last_written" no longer reflects hardware.
                     * Reset the deadband shadow so the first write after
                     * the bus recovers always goes through. */
                    for(int s = 0; s < PCA_SERVO_COUNT; s++)
                        smooth.ever_written[s] = false;
                }
            }
            else if(pca_ok)
            {
                /* Safety triggered -> instant snap to neutral */
                for(int s = 0; s < PCA_SERVO_COUNT; s++)
                    smooth.target[s] = PCA_SERVO_NEUTRAL;
                SMOOTH_Snap(&smooth);

                PCA_SetAllNeutral(&pca);

                /* v2.3.7: clear gripper stall watchdog so post-recovery
                 * doesn't have us still pinned at grip_touch_us. The
                 * arm just snapped to neutral; nothing's at the floor. */
                gripper_stall_active       = false;
                gripper_at_floor_since_us  = 0;
                gripper_unstall_since_us   = 0;

                /* v2.3.2: keep the deadband shadow coherent with what's
                 * actually latched in the PCA. Without this, the next
                 * SMOOTH_ShouldWrite would see ever_written=true with
                 * last_written != neutral, and write another redundant
                 * neutral on the very next tick. */
                for(int s = 0; s < PCA_SERVO_COUNT; s++)
                    SMOOTH_MarkWritten(&smooth, s, PCA_SERVO_NEUTRAL);

                /* v2.3.4: edit mode loses to safety. Clear active so the
                 * TUI's banner reverts to LOCKED on next render. The TUI
                 * must re-request after the system recovers. */
                atomic_store_explicit(&ipc.ctrl->edit_mode_active,
                                      0, memory_order_release);

                /* Still peek motor commands in SAFE to prevent DSP stall
                 * fault from blocking recovery. We discard the servo
                 * targets (arm stays at neutral) but tell the safety
                 * monitor that DSP is still publishing. Without this,
                 * the DSP stall timer fires 2s after SAFE entry and the
                 * system can never recover. */
                {
                    uint16_t _us[PCA_SERVO_COUNT];
                    uint8_t _g, _c; uint32_t _ack;
                    if(IPC_ReadMotorCmd(&ipc, _us, &_g, &_c, &_ack))
                        SAFETY_FeedMotorCMD(&safety, t);
                }
            }
        }

        /* 9. Thermal check */
        if((t - t_thermal) >= THERMAL_INTERVAL_US)
        {
            t_thermal   =   t;
            float temp  =   read_cpu_temp();
            SAFETY_FeedTemperature(&safety, temp);
        }

        /* 10. Heartbeat */
        if((t - t_hb) >= HEARTBEAT_INTERVAL_US)
        {
            t_hb    =   t;
            atomic_store(&ipc.ctrl->io_heartbeat_us, t);
        }

        /* 11. Diagnostics (1 Hz — this is also the only file-log pressure point) */
        if((t - t_diag) >= DIAG_INTERVAL_US)
        {
            t_diag      =   t;
            uint8_t sr  =   nrf_ok ? NRF_GetStatus(&nrf) : 0xFF;

            LOG_I("IO",
                  "pkts=%u gaps=%u ring=%u state=%s fault=%s "
                  "link=%d batt=%.2fV temp=%.1fC nrf_sr=0x%02X motion=%s",
                  atomic_load(&ipc.diag->io_pkts_received),
                  atomic_load(&ipc.diag->io_seq_gaps),
                  IPC_SensorCount(&ipc),
                  SAFETY_RadioStr(safety.state),
                  SAFETY_StatusStr(safety.last_fault),
                  safety.link.quality,
                  safety.battery.voltage,
                  safety.thermal.temperature_c,
                  sr,
                  SMOOTH_AllSettled(&smooth) ? "IDLE" : "MOVING");
        }
    }

    /* Cleanup */
    LOG_I("IO", "shutting down");

    if(pca_ok)
    {
        /* Bring servos to a safe neutral pose first (so the hand doesn't
         * fall into an awkward position), then PCA_AllOff disables PWM
         * entirely to cut torque. */
        PCA_SetAllNeutral(&pca);
        sleep_ms(300);
        PCA_AllOff(&pca);
        PCA_Close(&pca);
    }

    if(nrf_ok)
    {
        NRF_PowerDown(&nrf);
        NRF_Close(&nrf);
    }

    IPC_Close(&ipc);
    close(gpio_fd);
    Log_CloseFiles();
    return 0;
}
