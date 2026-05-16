/* ═══════════════════════════════════════════════════════════════════
 *  cpcu_ipc.h — v3.0 CHANGES
 *
 *  Two additions to the existing header:
 *    1. Repurpose IPC_MotorCommand._reserved[64] for smoother overrides
 *    2. Add IPC_LatencyTrace region after IPC_DspFiltered
 *
 *  IPC_VERSION bumps to 0x0300.
 * ═══════════════════════════════════════════════════════════════════ */

/* ── CHANGE 1: IPC_VERSION ──
 * Replace:  #define IPC_VERSION 0x0206
 * With: */
#define IPC_VERSION             0x0300

/* ── CHANGE 2: IPC_MotorCommand ──
 * Replace the _reserved[64] field in IPC_MotorCommand with: */

typedef struct __attribute__((aligned(64)))
{
    /* Cache line 0: servo targets (unchanged) */
    _Atomic uint32_t    seq;
    uint16_t            servo_us[IPC_NUM_SERVOS];
    uint8_t             gesture_id;
    uint8_t             confidence;
    uint16_t            _pad0;
    uint64_t            timestamp_us;
    uint8_t             _pad1[28];

    /* Cache line 1: per-gesture smoother overrides (NEW in v3.0).
     * Written by cpcu_dsp.py alongside servo targets. Zero = use
     * global runtime.json default. cpcu_io reads these and calls
     * SMOOTH_SetVelocity/SetAccel/SetEnabled per channel. */
    uint16_t            smooth_velocity_override[IPC_NUM_SERVOS]; /* 12 B */
    uint16_t            smooth_accel_override[IPC_NUM_SERVOS];    /* 12 B */
    uint8_t             snap_flags;     /* bitmask: bit N = servo N snaps */
    uint8_t             _pad2[39];      /* pad to 64 B */
} IPC_MotorCommand;

_Static_assert(sizeof(IPC_MotorCommand) == 128,
               "IPC_MotorCommand must be 128 bytes");


/* ── CHANGE 3: IPC_LatencyTrace (NEW) ──
 * Add this struct definition before the TOTAL SHM SIZE section. */

typedef struct __attribute__((aligned(64)))
{
    /* Written by cpcu_dsp.py (microseconds, updated per window) */
    _Atomic uint32_t    t_ring_dwell;       /* time data waited in ring */
    _Atomic uint32_t    t_dsp_compute;      /* filter + features + ML */
    _Atomic uint32_t    t_velocity;         /* integration + publish */
    _Atomic uint32_t    t_window_wait;      /* stride alignment wait */
    _Atomic uint32_t    t_hysteresis_ms;    /* votes × stride in ms */

    /* Written by cpcu_io.c */
    _Atomic uint32_t    t_smoother;         /* SMOOTH_Update + PCA write */

    /* Summary (computed by whoever wrote last) */
    _Atomic uint32_t    t_total_proc;       /* sum of processing stages */
    _Atomic uint32_t    seq;                /* monotonic, for TUI staleness */
} IPC_LatencyTrace;

_Static_assert(sizeof(IPC_LatencyTrace) == 64,
               "IPC_LatencyTrace must be 64 bytes");


/* ── CHANGE 4: IPC_SHM_SIZE ──
 * Add sizeof(IPC_LatencyTrace) to the total: */

#define IPC_SHM_SIZE    (\
        sizeof(IPC_ControlBlock)                            +\
        sizeof(IPC_SensorEntry) * IPC_SENSOR_RING_SIZE      +\
        sizeof(IPC_MotorCommand)                            +\
        sizeof(IPC_Diagnostics)                             +\
        sizeof(IPC_DSPExport)                               +\
        sizeof(IPC_RuntimeConfig)                           +\
        sizeof(IPC_ToolPresence)                            +\
        sizeof(IPC_DspFiltered)                             +\
        sizeof(IPC_LatencyTrace)                             \
        )


/* ── CHANGE 5: IPC_Context ──
 * Add the latency_trace pointer: */

typedef struct
{
    void                *base;
    IPC_ControlBlock    *ctrl;
    IPC_SensorEntry     *ring;
    IPC_MotorCommand    *motor;
    IPC_Diagnostics     *diag;
    IPC_DSPExport       *dsp_export;
    IPC_RuntimeConfig   *config;
    IPC_ToolPresence    *tool_presence;
    IPC_DspFiltered     *dsp_filtered;
    IPC_LatencyTrace    *latency;           /* NEW */
    int                 shm_fd;
} IPC_Context;


/* ── CHANGE 6: IPC_Create / IPC_Open ──
 * In cpcu_ipc.c, after assigning ctx->dsp_filtered, add:
 *
 *   ctx->latency = (IPC_LatencyTrace *)
 *       ((uint8_t *)ctx->dsp_filtered + sizeof(IPC_DspFiltered));
 *
 * And in IPC_Create, zero it:
 *   memset(ctx->latency, 0, sizeof(IPC_LatencyTrace));
 */
