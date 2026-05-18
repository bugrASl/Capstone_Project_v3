/**
 *  @file   cpcu_ipc.h
 *  @brief  IPC shared memory layout — control block, ring buffer, seqlock regions.
 *
 *  All inter-process communication through /dev/shm/cpcu_ipc.
 *
 *  Layout:
 *    Offset 0        192 B   IPC_ControlBlock (system state, heartbeats, edit-mode)
 *    Offset 192      64 KB   IPC_SensorEntry[1024] (SPSC ring buffer)
 *    Offset 65728    128 B   IPC_MotorCommand (seqlock-protected servo targets)
 *    Offset 65856    128 B   IPC_Diagnostics (atomic counters)
 *    Offset 65984    256 B   IPC_DSPExport (gesture, confidence, RMS)
 *    + IPC_RuntimeConfig, IPC_ToolPresence, IPC_DspFiltered
 */

#ifndef CPCU_IPC_H
#define CPCU_IPC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wireless_packet.h"

#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>    

/*============= CONSTANTS ==============================================================*/

#define IPC_SHM_NAME            "/cpcu_ipc"
#define IPC_SHM_PERMS           0644            /* Owner RW, Group/Others R */
#define IPC_MAGIC               0x494E4654UL    /* "INFT" - Infinitech */
#define IPC_VERSION             0x0300  /* v3.0 */          /* added IPC_ToolPresence + IPC_DspFiltered for web bridge */

#define IPC_SENSOR_RING_SIZE    1024
#define IPC_SENSOR_RING_MASK    (IPC_SENSOR_RING_SIZE - 1)
#define IPC_NUM_SERVOS          6

#define IPC_STATE_INIT          0
#define IPC_STATE_RUNNING       1
#define IPC_STATE_SAFE          2

/*============= CONTROL BLOCK ==========================================================*/

typedef struct __attribute__((aligned(64)))
{
    /* Cache line 0:    Header | System State */
    uint32_t            magic;
    uint16_t            version;
    _Atomic uint8_t     io_ready;
    _Atomic uint8_t     dsp_ready;
    _Atomic uint8_t     system_state;
    uint8_t             _pad0[3];
    _Atomic uint64_t    io_heartbeat_us;
    _Atomic uint32_t    motor_cmd_ack;

    /* Edit-mode handshake.
     *   request:  TUI -> world. 1 = "I want edit mode", 0 = "I want to exit".
     *   active:   io+dsp -> TUI. 1 = "we're parked, you may edit",
     *             0 = "we're not in edit mode" (either not requested, or
     *             still walking to neutral, or fault forced exit).
     *   request_us: when the request was raised, for TUI's 500 ms timeout.
     * 3 atomic bytes + 5 pad + 8 timestamp = 16 bytes, fits in the 36 B
     * reserve below. The TUI is the sole writer of request/request_us;
     * cpcu_io is the sole writer of active. cpcu_dsp.py reads request,
     * never writes.
     * See docs/EDIT_MODE.md for the full handshake protocol. */
    _Atomic uint8_t     edit_mode_request;
    _Atomic uint8_t     edit_mode_active;
    _Atomic uint8_t     edit_mode_dsp_ack;          /* dsp acks it saw the request */
    uint8_t             _pad_edit[5];
    _Atomic uint64_t    edit_mode_request_us;

    /* kernel pid published for the TUI live editor's Ctrl+S
     * handler — after CFG_PatchFile rewrites runtime.json, the TUI
     * sends SIGHUP to this pid so the kernel re-parses and republishes
     * IPC_RuntimeConfig. cpcu_kernel writes this once at startup;
     * other processes only read. 0 = not yet ready (TUI shows
     * "kernel not available, save deferred" if so). 4-byte uint32_t,
     * consumes 4 of the 12 reserved bytes. */
    _Atomic uint32_t    kernel_pid;

    uint8_t             _reserved0[8];

    /* Cache line 1:    Producer Index (Core 3 writes, Cores 1-2 read) */
    _Atomic uint32_t    sensor_head __attribute__((aligned(64)));
    uint8_t             _pad1[60];

    /* Cache line 2:    Consumer Index (Cores 1-2 write, Core 3 reads) */
    _Atomic uint32_t    sensor_tail __attribute__((aligned(64)));
    uint8_t             _pad2[60];
} IPC_ControlBlock;

_Static_assert( sizeof(IPC_ControlBlock) == 192, "IPC_ControlBlock must be 192 bytes (3 cache lines)" );

/*============= SENSOR RING ENTRY ======================================================*/

typedef struct __attribute__((aligned(64)))
{
    WL_SampleSet        samples[WL_SAMPLES_PER_PACKET];
    uint8_t             seq;
    uint8_t             flags;
    uint8_t             tx_retry;
    uint8_t             pkt_loss;
    uint16_t            timestamp;
    uint16_t            vbat_raw;
    uint64_t            rx_time_us;
    uint8_t             _pad[16];
} IPC_SensorEntry;

_Static_assert( sizeof(IPC_SensorEntry) == 64, "IPC_SensorEntry must be 64 bytes (1 cache line)" );
_Static_assert( (IPC_SENSOR_RING_SIZE & IPC_SENSOR_RING_MASK) == 0, "Ring size must be power of 2" );

/*============= MOTOR COMMAND ==========================================================*/

typedef struct __attribute__((aligned(64)))
{
    /* Cache line 0: */
    _Atomic uint32_t    seq;
    uint16_t            servo_us[IPC_NUM_SERVOS];
    uint8_t             gesture_id;
    uint8_t             confidence;
    uint16_t            _pad0;                                  
    uint64_t            timestamp_us;
    uint8_t             _pad1[28];

    /* Cache line 1:    Future Extension */
    uint8_t             _reserved[64];
} IPC_MotorCommand;

_Static_assert( sizeof(IPC_MotorCommand) == 128, "IPC_MotorCommand must be 128 bytes (2 cache lines)" );

/*============= DIAGNOSTICS ============================================================*/

typedef struct __attribute__((aligned(128)))
{
    /* Written by Core 3 only */
    _Atomic uint32_t    io_pkts_received;
    _Atomic uint32_t    io_pkts_dropped;
    _Atomic uint32_t    io_ring_overflows;
    _Atomic uint32_t    io_seq_gaps;
    _Atomic uint32_t    io_nrf_init_status;
    _Atomic uint32_t    io_safe_entries;
    _Atomic uint32_t    io_max_poll_us;
    
    /* Written by Core 1-2 only */
    _Atomic uint32_t    dsp_batches;
    _Atomic uint32_t    dsp_max_latency_us;
    _Atomic uint32_t    dsp_ring_underflows;
    _Atomic uint32_t    dsp_inferences;

    /* gripper stall watchdog. Incremented every time the
     * stall watchdog in cpcu_io.c retreats the gripper from its
     * mechanical floor after grip_stall_recover_ms continuous time
     * pinned there. See SOFT_GRIP.md. Read by the TUI's HEALTH page.
     * Allocated from the existing _reserved[5] pool — no layout
     * change, IPC_VERSION unchanged. */
    _Atomic uint32_t    io_gripper_stalls;

    uint32_t            _reserved[4];
} IPC_Diagnostics;

_Static_assert( sizeof(IPC_Diagnostics) == 128, "IPC_Diagnostics must be 128 bytes (2 cache lines)" );

/*============= DSP EXPORT (Python -> TUI) =============================================*/
/**
 *  @brief      Extra telemetry written by Python DSP, read by TUI.
 *  @details    Contains per-channel RMS, gesture name, per-class confidence,
 *              and inference timing — data the standard MotorCommand doesn't carry.
 */

#define IPC_MAX_GESTURE_NAME    16
#define IPC_MAX_CLASSES         10

typedef struct __attribute__((aligned(64)))
{
    float               channel_rms[WL_NUM_CHANNELS];           /* Filtered RMS per channel  */
    char                gesture_name[IPC_MAX_GESTURE_NAME];     /* "REST", "HAND SLOW" etc.  */
    float               class_confidence[IPC_MAX_CLASSES];      /* Per-class probability     */
    uint8_t             num_classes;
    uint8_t             active_class;
    uint16_t            _pad0;
    uint32_t            inference_time_us;                      /* Last inference wall time   */
    _Atomic uint32_t    update_seq;                             /* Bumped on each write       */
    uint8_t             _pad1[132];                             /* Pad to 256 bytes           */
} IPC_DSPExport;

_Static_assert( sizeof(IPC_DSPExport) == 256, "IPC_DSPExport must be 256 bytes (4 cache lines)" );

/*============= RUNTIME CONFIG ================================================*/
/*
 *  cpcu_kernel reads config/runtime.json at startup and on
 *  SIGHUP, populating this region. Other processes (cpcu_io, cpcu_dsp.py
 *  via Python's mmap) read it directly. Writes are seqlock-style: kernel
 *  bumps config_seq, writes, bumps again. Readers retry on torn reads.
 *
 *  All fields are EXPLICITLY-sized so the Python side can mmap & unpack
 *  with `struct` predictably. Pad to a fixed total to keep the IPC
 *  layout deterministic across schema additions across schema additions.
 *
 *  See docs/RUNTIME_CONFIG.md for the full schema and the
 *  consumer wiring.
 */

#define IPC_CFG_NUM_SERVOS          6
#define IPC_CFG_VALID_MAGIC         0x43464702      /* "CFG\x02" */

typedef struct __attribute__((aligned(64)))
{
    /* Header */
    _Atomic uint32_t    config_seq;                         /* seqlock: odd = mid-write */
    uint32_t            magic;                              /* IPC_CFG_VALID_MAGIC when populated */
    uint32_t            schema_version;                     /* must match expected */
    uint32_t            _pad_hdr;

    /* Servo limits (mirror of compile-time PCA_SERVO_MIN/MAX_US arrays).
     * These ARE runtime-tunable for calibration, but cpcu_io still
     * clamps to compile-time hardware limits as a final safety. */
    uint16_t            servo_min_us[IPC_CFG_NUM_SERVOS];
    uint16_t            servo_max_us[IPC_CFG_NUM_SERVOS];

    /* Per-servo gravity-sag bias offsets (signed us, applied AFTER
     * smoothing, BEFORE clamping). First consumer of the
     * runtime-config infrastructure. See JITTER_MITIGATION.md §6. */
    int16_t             servo_bias_us[IPC_CFG_NUM_SERVOS];

    /* Smoother per-servo overrides (zero = use compile-time default). */
    uint16_t            smooth_velocity_us_per_s[IPC_CFG_NUM_SERVOS];
    uint16_t            smooth_accel_us_per_s2[IPC_CFG_NUM_SERVOS];
    uint16_t            smooth_deadband_us[IPC_CFG_NUM_SERVOS];

    /* per-servo gravity compensation.
     *   gravity_dir:       -1 = gravity helps negative, +1 = positive, 0 = off.
     *   gravity_scale_pct: 10-100, velocity multiplier for gravity direction. */
    int16_t             gravity_dir[IPC_CFG_NUM_SERVOS];
    int16_t             gravity_scale_pct[IPC_CFG_NUM_SERVOS];

    /* Gesture velocities (us/s, signed) — gesture velocity consumer.
     * Indexed by [class_id][servo_id]. class_id 0 == rest. */
    int16_t             gesture_velocity[IPC_MAX_CLASSES][IPC_CFG_NUM_SERVOS];

    /* DSP/AI thresholds */
    uint8_t             interp_conf_floor_pct;              /* 0-100, default 40 */
    uint8_t             interp_conf_ceil_pct;               /* 0-100, default 85 */
    uint8_t             _reserved_hyst;                     /* v3: moved to gestures.json */
    uint8_t             _pad_dsp;
    uint16_t            grip_open_us;                       /* default 1700 */
    uint16_t            grip_touch_us;                      /* default 1200 */
    uint16_t            grip_firm_us;                       /* default 1100 */
    uint16_t            grip_stall_recover_ms;              /* default 2000 */

    /* Pad to a fixed size so future future additions don't change the
     * IPC layout binary-incompatibly. Reserve generously. */
    uint8_t             _reserved[256];
} IPC_RuntimeConfig;

_Static_assert(sizeof(IPC_RuntimeConfig) >= 512,
               "IPC_RuntimeConfig must be >= 512 bytes (8 cache lines minimum)");

/*============= IPC_ToolPresence ==============================================*/
/*  A small registry of "side tools" that opt into being visible to the
 *  web dashboard. Each tool that wants to publish state owns one slot
 *  identified by tool_id. Slots are not assigned dynamically — each
 *  tool hard-codes which slot it writes to (see IPC_TOOL_SLOT_* below)
 *  to avoid a hand-shake protocol. The web bridge reads all slots and
 *  shows whichever are alive.
 *
 *  ships only the *region* — the publisher patches in
 *  pca_testbench and signal_testbench are published by the
 *  web-bridge layered roll-out). Until then, all slots stay
 *  alive=0 and the dashboard's Tools tab shows "no side tools running".
 */
#define IPC_TOOL_PRESENCE_SLOTS    8           /* room for future tools */
#define IPC_TOOL_NAME_MAX          16
#define IPC_TOOL_PAYLOAD_BYTES     32          /* tool-specific opaque blob */

#define IPC_TOOL_SLOT_PCA          0           /* pca_testbench */
#define IPC_TOOL_SLOT_SIGNAL       1           /* signal_testbench */
/* slots 2..7 reserved for future tools */

typedef struct {
    _Atomic uint8_t     alive;                 /* 1 = tool is running */
    uint8_t             _pad0[7];
    _Atomic uint64_t    last_heartbeat_us;     /* tool's monotonic clock */
    char                tool_name[IPC_TOOL_NAME_MAX];   /* "pca_testbench" etc. */
    /* Tool-specific opaque payload. The dashboard JSON-serializer for
     * each tool slot interprets this field. For pca_testbench: selected
     * servo idx (1 B), current pulse_us (2 B), smoother enabled (1 B),
     * scratch. For signal_testbench: selected channel (1 B), buffered
     * RMS (4 B), drop counter (4 B). Anything else is the tool's call.
     * 32 bytes is enough for ~1-2 dozen small fields, more than any
     * dashboard widget will need to show. */
    uint8_t             payload[IPC_TOOL_PAYLOAD_BYTES];
    /* Total: 1 + 7 + 8 + 16 + 32 = 64 B = one cache line. */
} IPC_ToolSlot;

_Static_assert(sizeof(IPC_ToolSlot) == 64, "IPC_ToolSlot must be 64 bytes (1 cache line)");

typedef struct {
    IPC_ToolSlot        slot[IPC_TOOL_PRESENCE_SLOTS];
} IPC_ToolPresence;

_Static_assert(sizeof(IPC_ToolPresence) == 64 * IPC_TOOL_PRESENCE_SLOTS,
               "IPC_ToolPresence must be 8 * 64 bytes");

/*============= IPC_DspFiltered ==============================================*/
/*  Per-channel snapshot of the most recent post-bandpass+notch+envelope
 *  buffer that cpcu_dsp.py computed. The web bridge reads this for the
 *  "filtered" view in the Waves tab and (in the next turn) the Spectrum
 *  tab. Updated once per dsp window (50 ms stride at WINDOW_MS = 200,
 *  i.e. ~20 Hz refresh).
 *
 *  Layout: 8 channels × 200 samples float32 = 6400 B per buffer.
 *  Plus a seqlock-style sequence counter so the bridge gets a
 *  consistent snapshot.
 *
 *  Why 200 samples? cpcu_dsp.py's TARGET_FS_HZ = 200 and
 *  WINDOW_MS = 200 → 200 * 200 / 1000 = 40 samples per window. So
 *  200 samples = the last 5 windows (1 second of envelope history),
 *  which is plenty for the wave-tab plot and gives the FFT enough
 *  context (next turn).
 *
 *  Note: IPC_DSPExport (256 B) already publishes the *features* the
 *  classifier sees — RMS, MAV, etc. This new region publishes the
 *  *signal itself*, post-filter, for visualization. Different
 *  consumer, different region.
 */
#define IPC_DSPFILT_CHANNELS       8
#define IPC_DSPFILT_SAMPLES        200         /* 1 s @ 200 Hz */

typedef struct {
    _Atomic uint32_t    seq;                   /* odd = writer in progress */
    uint32_t            sample_rate_hz;        /* 200 — for the consumer */
    uint64_t            update_us;             /* monotonic time of last update */
    uint8_t             _pad0[16];             /* align channels[] to cache line */
    /* Row-major: [channel][sample]. Column-major would be marginally
     * faster for per-sample-across-channels reads, but the bridge
     * consumes channel by channel, so row-major matches access. */
    float               channel[IPC_DSPFILT_CHANNELS][IPC_DSPFILT_SAMPLES];
} IPC_DspFiltered;

/* 32 B header + 8 * 200 * 4 B = 6432 B. Pad to next 64 B boundary
 * so subsequent regions stay cache-aligned. */
_Static_assert(sizeof(IPC_DspFiltered) >= 32 + IPC_DSPFILT_CHANNELS * IPC_DSPFILT_SAMPLES * 4,
               "IPC_DspFiltered geometry sanity");

/*============= TOTAL SHM SIZE =========================================================*/

#define IPC_SHM_SIZE    (\
        sizeof(IPC_ControlBlock)                            +\
        sizeof(IPC_SensorEntry) *   IPC_SENSOR_RING_SIZE    +\
        sizeof(IPC_MotorCommand)                            +\
        sizeof(IPC_Diagnostics)                             +\
        sizeof(IPC_DSPExport)                               +\
        sizeof(IPC_RuntimeConfig)                           +\
        sizeof(IPC_ToolPresence)                            +\
        sizeof(IPC_DspFiltered)                              \
        )

/*============= CONTEXT HANDLE =========================================================*/

typedef struct
{
    void                *base;
    IPC_ControlBlock    *ctrl;
    IPC_SensorEntry     *ring;
    IPC_MotorCommand    *motor;
    IPC_Diagnostics     *diag;
    IPC_DSPExport       *dsp_export;
    IPC_RuntimeConfig   *config;            /* */
    IPC_ToolPresence    *tool_presence;     /* */
    IPC_DspFiltered     *dsp_filtered;      /* */
    int                 shm_fd;
} IPC_Context;

/*============= API ====================================================================*/

int         IPC_Create(IPC_Context *ctx);
int         IPC_Open(IPC_Context *ctx);
void        IPC_Close(IPC_Context *ctx);
void        IPC_Destroy(void);

/* Ring Buffer */
void        IPC_PushSensor(IPC_Context *ctx, const WL_Packet *pkt, uint64_t rx_time_us);
uint32_t    IPC_PopSensorBatch(IPC_Context *ctx, IPC_SensorEntry *out, uint32_t max_count);
uint32_t    IPC_SensorCount(IPC_Context *ctx);

/* Motor Command (SeqLock) */
void        IPC_WriteMotorCmd(IPC_Context *ctx, const uint16_t servo_us[IPC_NUM_SERVOS],
                              uint8_t gesture_id, uint8_t confidence, uint64_t timestamp_us);
bool        IPC_ReadMotorCmd(IPC_Context *ctx, uint16_t servo_us[IPC_NUM_SERVOS],
                             uint8_t *gesture_id, uint8_t *confidence, uint32_t *last_ack);

/* Runtime Config (SeqLock, current version).
 * Writers (cpcu_kernel only) call IPC_WriteRuntimeConfig. Readers
 * (cpcu_io, cpcu_dsp.py) call IPC_ReadRuntimeConfig — it copies the
 * full struct out under a torn-read-retry loop. The copy is cheap
 * (~512 bytes); readers should call this once per loop iteration
 * rather than holding pointers into shared memory across barriers. */
void        IPC_WriteRuntimeConfig(IPC_Context *ctx, const IPC_RuntimeConfig *src);
bool        IPC_ReadRuntimeConfig (IPC_Context *ctx, IPC_RuntimeConfig *dst);
uint32_t    IPC_RuntimeConfigSeq  (IPC_Context *ctx);    /* cheap polling check */

#ifdef __cplusplus
}
#endif

#endif  /* CPCU_IPC_H */                          

