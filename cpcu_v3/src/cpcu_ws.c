/**
 *  @file   cpcu_ws.c
 *  @brief  WebSocket bridge — read-only JSON dashboard over HTTP.
 *
 *  Maps /dev/shm/cpcu_ipc as a consumer (same as cpcu_tui), serves a
 *  single-page browser dashboard via Mongoose embedded HTTP/WS server.
 *
 *  Operation:
 *    1. Parse --bind URL and --static directory from command line.
 *    2. Open IPC shared memory (read-only consumer).
 *    3. Listen on HTTP; upgrade /ws path to WebSocket.
 *    4. Broadcast JSON state frame at 10 Hz (system status, gesture, diagnostics).
 *    5. Broadcast JSON wave frame at 20 Hz (raw envelope + filtered + raw-full FFT data).
 *    6. Serve static files (index.html) from the configured directory.
 *
 *  No command channel — browsers cannot affect the running system.
 *  Runs on Core 0 at default priority (not real-time).
 */

#include "cpcu_ipc.h"
#include "cpcu_json.h"
#include "wireless_packet.h"      /* WL_SampleSet, WL_SAMPLES_PER_PACKET, WL_NUM_CHANNELS */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdatomic.h>
#include <time.h>

#ifdef CPCU_WS_HAVE_MONGOOSE
#  include "mongoose.h"
#else
#  include "mongoose_stub.h"   /* parser-pleaser; produces a non-functional binary */
#endif

/*============= GLOBALS ===============================================================*/

static volatile sig_atomic_t g_run = 1;
static IPC_Context           g_ipc;

/* Bind URL passed on the command line. Default is loud-and-shared. */
static char                  g_bind_url[64] = "ws://0.0.0.0:8765";

/* Static directory passed on the command line; defaults to the
 * deployed location and falls back to a development-tree relative
 * path if that doesn't exist. */
static char                  g_static_dir[256] = "/opt/cpcu/ws_static";

/* Per-frame scratch buffers. State frame is small; wave frame is
 * larger because it carries 8 channels × ~50 samples per update. */
static char                  g_state_buf[2048];
static char                  g_wave_buf[32768];     /* doubled to fit raw_full */

static void on_sig(int s) { (void)s; g_run = 0; }

/*============= TIME UTILITIES ========================================================*/

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/*============= STATE FRAME (Overview tab + Edit-mode banner) =========================*/

static const char *state_name(uint8_t s)
{
    switch(s)
    {
        case 0: return "boot";
        case 1: return "running";
        case 2: return "safe";
        case 3: return "shutdown";
        default: return "?";
    }
}

/* Build/broadcast functions are declared __attribute__((unused)) so
 * the warning is suppressed when CPCU_WS_HAVE_MONGOOSE is undefined
 * (broadcast loop is compiled out, leaving these orphaned). The
 * functional binary uses them; the stub binary just doesn't. */
__attribute__((unused))
static void build_state_frame(void)
{
    JW jw;
    jw_init(&jw, g_state_buf, sizeof(g_state_buf));

    jw_obj_begin(&jw);
    jw_kv_str(&jw, "ch", "state");

    /* --- core control block --- */
    uint8_t  sys     = atomic_load(&g_ipc.ctrl->system_state);
    uint8_t  io_rdy  = atomic_load(&g_ipc.ctrl->io_ready);
    uint8_t  dsp_rdy = atomic_load(&g_ipc.ctrl->dsp_ready);
    uint64_t hb      = atomic_load(&g_ipc.ctrl->io_heartbeat_us);
    uint64_t now     = now_us();

    jw_kv_str (&jw, "system_state",       state_name(sys));
    jw_kv_int (&jw, "system_state_id",    (long long)sys);
    jw_kv_bool(&jw, "io_ready",           io_rdy != 0);
    jw_kv_bool(&jw, "dsp_ready",          dsp_rdy != 0);
    jw_kv_u64 (&jw, "io_heartbeat_age_us",
               (hb && now > hb) ? (now - hb) : 0);

    /* --- edit-mode banner state (display-only on web) --- */
    jw_kv_obj_begin(&jw, "edit_mode");
        jw_kv_bool(&jw, "request",  atomic_load(&g_ipc.ctrl->edit_mode_request) != 0);
        jw_kv_bool(&jw, "active",   atomic_load(&g_ipc.ctrl->edit_mode_active)  != 0);
        jw_kv_bool(&jw, "dsp_ack",  atomic_load(&g_ipc.ctrl->edit_mode_dsp_ack) != 0);
    jw_obj_end(&jw);

    /* --- DSP export: gesture, confidence, RMS per channel --- */
    /* IPC_DSPExport is published with a seqlock (update_seq); we tolerate
     * one read tear (re-read once) to get a consistent snapshot. */
    IPC_DSPExport snap;
    uint32_t s1, s2;
    int      tries = 0;
    do {
        s1 = atomic_load_explicit(&g_ipc.dsp_export->update_seq,
                                  memory_order_acquire);
        memcpy(&snap, g_ipc.dsp_export, sizeof(snap));
        s2 = atomic_load_explicit(&g_ipc.dsp_export->update_seq,
                                  memory_order_acquire);
    } while(s1 != s2 && ++tries < 4);

    jw_kv_obj_begin(&jw, "dsp");
        /* gesture_name is a fixed-size char[16]; ensure NUL termination. */
        char gname[IPC_MAX_GESTURE_NAME + 1];
        memcpy(gname, snap.gesture_name, IPC_MAX_GESTURE_NAME);
        gname[IPC_MAX_GESTURE_NAME] = '\0';
        jw_kv_str(&jw, "gesture",       gname);
        jw_kv_int(&jw, "active_class",  (long long)snap.active_class);
        jw_kv_int(&jw, "num_classes",   (long long)snap.num_classes);
        /* Top-1 confidence (the active class's prob) is the value the
         * dashboard displays as "confidence". Per-class is a separate
         * array. */
        float top_conf = (snap.active_class < snap.num_classes &&
                          snap.active_class < IPC_MAX_CLASSES)
                         ? snap.class_confidence[snap.active_class]
                         : 0.0f;
        jw_kv_f32(&jw, "confidence",    top_conf);
        jw_kv_arr_f32(&jw, "class_confidence",
                      snap.class_confidence,
                      snap.num_classes < IPC_MAX_CLASSES
                          ? snap.num_classes : IPC_MAX_CLASSES);
        jw_kv_arr_f32(&jw, "channel_rms",
                      snap.channel_rms, WL_NUM_CHANNELS);
        jw_kv_u32(&jw, "inference_us", snap.inference_time_us);
    jw_obj_end(&jw);

    /* --- diagnostics counters --- */
    jw_kv_obj_begin(&jw, "diag");
        jw_kv_u32(&jw, "io_pkts_received",  atomic_load(&g_ipc.diag->io_pkts_received));
        jw_kv_u32(&jw, "io_pkts_dropped",   atomic_load(&g_ipc.diag->io_pkts_dropped));
        jw_kv_u32(&jw, "io_ring_overflows", atomic_load(&g_ipc.diag->io_ring_overflows));
        jw_kv_u32(&jw, "io_seq_gaps",       atomic_load(&g_ipc.diag->io_seq_gaps));
        jw_kv_u32(&jw, "io_safe_entries",   atomic_load(&g_ipc.diag->io_safe_entries));
        jw_kv_u32(&jw, "io_gripper_stalls", atomic_load(&g_ipc.diag->io_gripper_stalls));
        jw_kv_u32(&jw, "dsp_inferences",    atomic_load(&g_ipc.diag->dsp_inferences));
        jw_kv_u32(&jw, "dsp_max_latency_us",atomic_load(&g_ipc.diag->dsp_max_latency_us));
    jw_obj_end(&jw);

    /* --- Tools — IPC_ToolPresence registry. Each slot a tool
     *     might be alive in. We emit only alive slots, with a freshness
     *     check on the heartbeat (>2 s old → treat as dead). The tool's
     *     32-byte payload is opaque from the bridge's POV; we surface
     *     it as a tool-specific subobject.
     *
     *     Slot 0 = pca_testbench:
     *       payload[0] = selected servo idx (0..5)
     *       payload[1..2] = current pulse_us (uint16 LE)
     *       payload[3] = smoother enabled (0/1)
     *       payload[4] = jog mode (0=normal, 1=hold-pose)
     *
     *     Slot 1 = signal_testbench:
     *       payload[0] = selected channel (0..7)
     *       payload[1..4] = RMS as uint32 LE
     *       payload[5..8] = drop counter as uint32 LE
     */
    jw_kv_arr_begin(&jw, "tools");
    for(int slot = 0; slot < IPC_TOOL_PRESENCE_SLOTS; slot++)
    {
        IPC_ToolSlot *t = &g_ipc.tool_presence->slot[slot];
        uint8_t alive = atomic_load_explicit(&t->alive, memory_order_acquire);
        if(!alive) continue;

        uint64_t hb = atomic_load_explicit(&t->last_heartbeat_us,
                                            memory_order_acquire);
        uint64_t age = (hb && now > hb) ? (now - hb) : 0;
        bool fresh = age < 2000000;       /* < 2 s old */

        /* NUL-terminate name */
        char tname[IPC_TOOL_NAME_MAX + 1];
        memcpy(tname, t->tool_name, IPC_TOOL_NAME_MAX);
        tname[IPC_TOOL_NAME_MAX] = '\0';

        jw_obj_begin(&jw);
            jw_kv_int(&jw, "slot",           slot);
            jw_kv_str(&jw, "name",           tname);
            jw_kv_bool(&jw, "fresh",         fresh);
            jw_kv_u64(&jw, "heartbeat_age_us", age);

            /* Per-slot payload decode. The bridge knows the layouts. */
            if(slot == IPC_TOOL_SLOT_PCA)
            {
                uint16_t pulse;
                memcpy(&pulse, &t->payload[1], 2);
                jw_kv_obj_begin(&jw, "state");
                    jw_kv_int(&jw, "servo_idx",      t->payload[0]);
                    jw_kv_int(&jw, "pulse_us",       pulse);
                    jw_kv_bool(&jw, "smoother_on",   t->payload[3] != 0);
                    jw_kv_int(&jw, "jog_mode",       t->payload[4]);
                jw_obj_end(&jw);
            }
            else if(slot == IPC_TOOL_SLOT_SIGNAL)
            {
                uint32_t amp_u32, drops;
                memcpy(&amp_u32, &t->payload[1], 4);
                memcpy(&drops,   &t->payload[5], 4);
                /* Amplitude payload is a uint32 holding a float bit
                 * pattern (signal_testbench writes Vpp into it). */
                float amp;
                memcpy(&amp, &amp_u32, 4);
                jw_kv_obj_begin(&jw, "state");
                    jw_kv_int(&jw, "channel",        t->payload[0]);
                    jw_kv_f32(&jw, "amplitude_vpp", amp);
                    jw_kv_u32(&jw, "drops",          drops);
                jw_obj_end(&jw);
            }
            else
            {
                /* Unknown slot — emit raw payload as int array so a
                 * future client that knows the layout can decode. */
                jw_kv_arr_begin(&jw, "payload");
                for(int i = 0; i < IPC_TOOL_PAYLOAD_BYTES; i++)
                    jw_int(&jw, t->payload[i]);
                jw_arr_end(&jw);
            }
        jw_obj_end(&jw);
    }
    jw_arr_end(&jw);

    /* --- bridge meta --- */
    jw_kv_obj_begin(&jw, "bridge");
        jw_kv_u64(&jw, "now_us",          now);
        jw_kv_u32(&jw, "ipc_version",     IPC_VERSION);
    jw_obj_end(&jw);

    jw_obj_end(&jw);
}

/*============= WAVE FRAME (Waves tab) ================================================*/

/*  Pre-filter (raw):
 *    Sourced from IPC_SensorEntry ring at 1 kpkt/s × 2 samples/pkt =
 *    2 kHz per channel. We don't ship 2 kHz — that's 16 KB/ch/s of
 *    raw 16-bit samples, ten of them is 1.3 MB/s. Way too much.
 *    Decimate by averaging to ~50 Hz envelope per channel. Each
 *    50 ms tick we emit 1 sample per channel, batched into 50-sample
 *    chunks broadcast at 20 Hz (= 1 second of trailing data per
 *    update for smooth scrolling).
 *
 *  Post-filter:
 *    Sourced from IPC_DspFiltered which dsp publishes once per
 *    window (~20 Hz). We just snapshot it.
 */

#define WAVE_RAW_DECIM_HZ      50          /* downsample target for raw envelope */
#define WAVE_RAW_BATCH_SAMPLES 50          /* one second of trailing envelope */
#define WAVE_RAW_FULL_FS_HZ    2000        /* native ADC rate  */
#define WAVE_RAW_FULL_SAMPLES  256         /* 128 ms @ 2 kHz — FFT window */

/* Per-channel rolling envelope buffer for the raw stream. Each entry
 * is the mean-absolute deviation over a 20 ms window (= 40 samples
 * @ 2 kHz). Updated whenever new ring data arrives. */
typedef struct {
    float    history[WAVE_RAW_BATCH_SAMPLES];
    int      head;            /* next write index */
    uint32_t last_seen_seq;   /* ring tail we last consumed */
} RawChEnv;

static RawChEnv g_raw_env[8];

/* rolling 256-sample raw window per channel for the Spectrum
 * tab's browser-side FFT. Stored as int16 (raw ADC value, no centering)
 * so the JSON serializer emits compact numbers. The browser does its
 * own DC-removal + Hann window + FFT. */
typedef struct {
    int16_t  history[WAVE_RAW_FULL_SAMPLES];
    int      head;            /* circular write index */
} RawChFull;

static RawChFull g_raw_full[8];

/* Drain the ring up to the current head. For each ~40-sample chunk of
 * a channel, compute MAV and push to that channel's history buffer. */
__attribute__((unused))
static void update_raw_envelope(void)
{
    uint32_t head = atomic_load_explicit(&g_ipc.ctrl->sensor_head,
                                         memory_order_acquire);
    uint32_t tail = atomic_load_explicit(&g_ipc.ctrl->sensor_tail,
                                         memory_order_relaxed);
    /* Only consume what's new since last call. We're a *third* reader
     * here (after dsp), so we must NOT advance the real tail. We keep
     * our own "private tail" in g_raw_env[*].last_seen_seq. */
    uint32_t our_tail = g_raw_env[0].last_seen_seq;

    /* If ring producer outran us by more than the ring size, snap to
     * the freshest data. */
    uint32_t avail = head - our_tail;
    (void)tail;
    if(avail > IPC_SENSOR_RING_SIZE)
    {
        our_tail = head - IPC_SENSOR_RING_SIZE;
        avail    = IPC_SENSOR_RING_SIZE;
    }
    if(avail == 0) return;

    /* Walk entries; each IPC_SensorEntry holds WL_SAMPLES_PER_PACKET
     * (2) sample sets across 8 channels. Accumulate absolute deviation
     * per channel, push when we've covered ~20 ms (= 40 samples). */
    static int   acc_count[8] = {0};
    static float acc_sum[8]   = {0};
    const int    DECIM_NSAMPLES = 2000 / WAVE_RAW_DECIM_HZ;  /* 40 */

    for(uint32_t i = 0; i < avail; i++)
    {
        uint32_t idx = (our_tail + i) & (IPC_SENSOR_RING_SIZE - 1);
        IPC_SensorEntry *e = &g_ipc.ring[idx];
        for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        {
            for(int ch = 0; ch < 8; ch++)
            {
                /* capture raw value (int16, no centering) into the
                 * full-resolution ring for browser-side FFT. */
                int16_t raw_v = (int16_t)e->samples[s].ch[ch];
                g_raw_full[ch].history[g_raw_full[ch].head] = raw_v;
                g_raw_full[ch].head = (g_raw_full[ch].head + 1)
                                       % WAVE_RAW_FULL_SAMPLES;

                /* Center on 2048 (mid-scale of 12-bit ADC) and accumulate
                 * absolute deviation. */
                int v = (int)raw_v - 2048;
                if(v < 0) v = -v;
                acc_sum[ch] += (float)v;
                acc_count[ch]++;
                if(acc_count[ch] >= DECIM_NSAMPLES)
                {
                    /* Normalize to a 0..1 envelope estimate. 12-bit
                     * full-scale half-range = 2048; divide. */
                    float env = (acc_sum[ch] / (float)acc_count[ch]) / 2048.0f;
                    g_raw_env[ch].history[g_raw_env[ch].head] = env;
                    g_raw_env[ch].head = (g_raw_env[ch].head + 1)
                                          % WAVE_RAW_BATCH_SAMPLES;
                    acc_sum[ch]   = 0;
                    acc_count[ch] = 0;
                }
            }
        }
    }
    for(int ch = 0; ch < 8; ch++) g_raw_env[ch].last_seen_seq = head;
}

__attribute__((unused))
static void build_wave_frame(void)
{
    update_raw_envelope();

    JW jw;
    jw_init(&jw, g_wave_buf, sizeof(g_wave_buf));

    jw_obj_begin(&jw);
    jw_kv_str(&jw, "ch", "waves");
    jw_kv_int(&jw, "raw_fs_hz",      WAVE_RAW_DECIM_HZ);
    jw_kv_int(&jw, "raw_n_samples",  WAVE_RAW_BATCH_SAMPLES);

    /* Raw channels — each as a flat array of WAVE_RAW_BATCH_SAMPLES
     * floats. We unwrap the ring buffer so the oldest sample is
     * first. */
    jw_kv_arr_begin(&jw, "raw");
    for(int ch = 0; ch < 8; ch++)
    {
        jw_arr_begin(&jw);
        int start = g_raw_env[ch].head;       /* head points at oldest after wrap */
        for(int i = 0; i < WAVE_RAW_BATCH_SAMPLES; i++)
        {
            int idx = (start + i) % WAVE_RAW_BATCH_SAMPLES;
            jw_f32(&jw, g_raw_env[ch].history[idx]);
        }
        jw_arr_end(&jw);
    }
    jw_arr_end(&jw);

    /* raw-full channels for the Spectrum tab's browser-side FFT.
     * 256 samples per channel @ 2 kHz = 128 ms window, ~7.8 Hz/bin.
     * Sent as int16 (raw 12-bit ADC counts). At 8 ch × 256 samples ×
     * up to 5 chars per int = ~10 KB per wave frame. Browser
     * DC-removes, Hann-windows, and FFTs. */
    jw_kv_int(&jw, "raw_full_fs_hz",      WAVE_RAW_FULL_FS_HZ);
    jw_kv_int(&jw, "raw_full_n_samples",  WAVE_RAW_FULL_SAMPLES);
    jw_kv_arr_begin(&jw, "raw_full");
    for(int ch = 0; ch < 8; ch++)
    {
        jw_arr_begin(&jw);
        int start = g_raw_full[ch].head;     /* head points at oldest after wrap */
        for(int i = 0; i < WAVE_RAW_FULL_SAMPLES; i++)
        {
            int idx = (start + i) % WAVE_RAW_FULL_SAMPLES;
            jw_int(&jw, (long long)g_raw_full[ch].history[idx]);
        }
        jw_arr_end(&jw);
    }
    jw_arr_end(&jw);

    /* Filtered channels — sourced from IPC_DspFiltered. seqlock-style
     * read: capture seq, copy, capture seq again; if they differ
     * (writer ran in the middle), accept the half-torn snapshot and
     * note it. The values are display-only floats; a tear is visually
     * negligible. */
    uint32_t s1 = atomic_load_explicit(&g_ipc.dsp_filtered->seq,
                                       memory_order_acquire);
    uint32_t fs = g_ipc.dsp_filtered->sample_rate_hz;
    uint64_t fu = g_ipc.dsp_filtered->update_us;

    jw_kv_int(&jw, "filtered_fs_hz",     (long long)fs);
    jw_kv_int(&jw, "filtered_n_samples", (long long)IPC_DSPFILT_SAMPLES);
    jw_kv_u64(&jw, "filtered_update_us", fu);
    jw_kv_bool(&jw, "filtered_present",  fs != 0);

    jw_kv_arr_begin(&jw, "filtered");
    for(int ch = 0; ch < 8; ch++)
    {
        jw_arr_begin(&jw);
        for(int i = 0; i < IPC_DSPFILT_SAMPLES; i++)
        {
            jw_f32(&jw, g_ipc.dsp_filtered->channel[ch][i]);
        }
        jw_arr_end(&jw);
    }
    jw_arr_end(&jw);
    (void)s1;   /* not strictly needed yet — dsp publisher will set this in Layer D */

    jw_obj_end(&jw);
}

/*============= MONGOOSE EVENT HANDLER ================================================*/
/*
 *  Two listeners share this handler:
 *    - HTTP requests on /            -> serve static/index.html
 *    - WS upgrade requests on /ws    -> add to broadcast set
 *
 *  Mongoose flags `is_websocket` on a connection after upgrade. The
 *  broadcast loop walks the connection list and sends to every
 *  is_websocket==1 connection. New connections get the next periodic
 *  frame; we don't backfill history.
 */
__attribute__((unused))
static void ev_handler(struct mg_connection *c, int ev, void *ev_data)
{
#ifdef CPCU_WS_HAVE_MONGOOSE
    if(ev == MG_EV_HTTP_MSG)
    {
        struct mg_http_message *hm = (struct mg_http_message *)ev_data;
        if(mg_match(hm->uri, mg_str("/ws"), NULL))
        {
            mg_ws_upgrade(c, hm, NULL);
            /* Welcome frame so the client knows it's connected even
             * before the next periodic broadcast fires. */
            const char *welcome =
                "{\"ch\":\"hello\",\"server\":\"cpcu_ws\","
                "\"version\":\"v2.4.0\",\"ipc_version\":518}";
            mg_ws_send(c, welcome, strlen(welcome), WEBSOCKET_OP_TEXT);
        }
        else
        {
            struct mg_http_serve_opts opts = { .root_dir = g_static_dir };
            mg_http_serve_dir(c, hm, &opts);
        }
    }
#else
    (void)c; (void)ev; (void)ev_data;
#endif
}

/*============= BROADCAST =============================================================*/

#ifdef CPCU_WS_HAVE_MONGOOSE
static void broadcast(struct mg_mgr *mgr, const char *buf, size_t len)
{
    int n_sent = 0;
    for(struct mg_connection *c = mgr->conns;
        c != NULL; c = c->next)
    {
        if(c->is_websocket && !c->is_closing)
        {
            mg_ws_send(c, buf, len, WEBSOCKET_OP_TEXT);
            n_sent++;
        }
    }
    (void)n_sent;
}
#endif

/*============= ARG PARSE =============================================================*/

static void usage(void)
{
    fprintf(stderr,
        "Usage: cpcu_ws [--bind URL] [--static DIR]\n"
        "  --bind URL    Bind URL for HTTP/WS listener\n"
        "                  default: ws://0.0.0.0:8765 (LAN-shared)\n"
        "                  use ws://127.0.0.1:8765 for loopback only\n"
        "  --static DIR  Static files directory (default /opt/cpcu/ws_static,\n"
        "                  falls back to ./web/static if missing)\n");
}

static void parse_args(int argc, char **argv)
{
    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--bind") == 0 && i + 1 < argc)
        {
            snprintf(g_bind_url, sizeof(g_bind_url), "%s", argv[++i]);
        }
        else if(strcmp(argv[i], "--static") == 0 && i + 1 < argc)
        {
            snprintf(g_static_dir, sizeof(g_static_dir), "%s", argv[++i]);
        }
        else if(strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            usage();
            exit(0);
        }
        else if(strcmp(argv[i], "--version") == 0 || strcmp(argv[i], "-v") == 0)
        {
#ifdef CPCU_WS_HAVE_MONGOOSE
            fprintf(stdout, "cpcu_ws v2.4.0 (Mongoose bridge)\n");
#else
            fprintf(stdout, "cpcu_ws v2.4.0 (BUILT WITHOUT MONGOOSE — stub)\n");
#endif
            exit(0);
        }
        else
        {
            fprintf(stderr, "[WS] unknown arg: %s\n", argv[i]);
            usage();
            exit(1);
        }
    }
    if(access(g_static_dir, F_OK) != 0)
    {
        const char *fallback = "web/static";
        if(access(fallback, F_OK) == 0)
        {
            snprintf(g_static_dir, sizeof(g_static_dir), "%s", fallback);
        }
    }
}

/*============= MAIN ==================================================================*/

int main(int argc, char **argv)
{
    parse_args(argc, argv);

    fprintf(stderr,
        "===========================================================\n"
        "  CPCU Dashboard — WebSocket bridge (cpcu_ws v2.4.0)\n"
        "===========================================================\n"
        "  bind     : %s\n"
        "  static   : %s\n",
        g_bind_url, g_static_dir);

    if(strstr(g_bind_url, "0.0.0.0") != NULL)
    {
        fprintf(stderr,
            "  WARNING: serving biosignals to your LAN.\n"
            "           Use --bind ws://127.0.0.1:8765 to restrict.\n");
    }
    fprintf(stderr,
        "===========================================================\n");

    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);

    if(IPC_Open(&g_ipc) != 0)
    {
        fprintf(stderr, "[WS] IPC_Open failed — is cpcu_kernel running?\n");
        return 1;
    }
    fprintf(stderr, "[WS] IPC mapped, IPC_VERSION=0x%04x\n",
            (unsigned)g_ipc.ctrl->version);

#ifdef CPCU_WS_HAVE_MONGOOSE
    struct mg_mgr mgr;
    mg_mgr_init(&mgr);

    /* Mongoose's URL grammar is "http://addr:port" for HTTP and
     * "ws://addr:port" alternates by upgrade. We use the http://
     * form for the listener since both protocols share it. */
    char http_url[64];
    if(strncmp(g_bind_url, "ws://", 5) == 0)
        snprintf(http_url, sizeof(http_url), "http://%s", g_bind_url + 5);
    else
        snprintf(http_url, sizeof(http_url), "%s", g_bind_url);

    if(mg_http_listen(&mgr, http_url, ev_handler, NULL) == NULL)
    {
        fprintf(stderr, "[WS] mg_http_listen failed for %s\n", http_url);
        IPC_Close(&g_ipc);
        return 1;
    }
    fprintf(stderr, "[WS] listening on %s\n", http_url);

    uint64_t last_state_us = 0;
    uint64_t last_wave_us  = 0;
    while(g_run)
    {
        mg_mgr_poll(&mgr, 25);   /* up to 25 ms in the event loop */
        uint64_t t = now_us();

        if(t - last_state_us >= 100000)   /* 10 Hz */
        {
            build_state_frame();
            broadcast(&mgr, g_state_buf, strlen(g_state_buf));
            last_state_us = t;
        }
        if(t - last_wave_us >= 50000)     /* 20 Hz */
        {
            build_wave_frame();
            broadcast(&mgr, g_wave_buf, strlen(g_wave_buf));
            last_wave_us = t;
        }
    }

    mg_mgr_free(&mgr);
#else
    fprintf(stderr, "[WS] BUILT WITHOUT MONGOOSE — this binary is a stub.\n");
    fprintf(stderr, "[WS] Run web/vendor/fetch.sh and rebuild.\n");
    /* Keep the IPC mapping open so smoke-tests can verify the rest of
     * the build path; sleep until killed. */
    while(g_run)
    {
        struct timespec ts = { 0, 250 * 1000 * 1000 };
        nanosleep(&ts, NULL);
    }
#endif

    IPC_Close(&g_ipc);
    fprintf(stderr, "[WS] exited cleanly\n");
    return 0;
}

