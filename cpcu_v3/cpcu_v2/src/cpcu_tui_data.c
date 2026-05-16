/**
 *  @file   cpcu_tui_data.c
 *  @brief  TUI data layer — demo signal synthesis, dataset capture, wave ring.
 *
 *  Generates synthetic EMG waveforms for demo mode (sine, chirp, multi-tone,
 *  noise-burst, AM, chirp-burst). Drains the IPC sensor ring into a rolling
 *  per-channel waveform buffer for the Waves page. Manages CSV file I/O for
 *  the Dataset capture page (start/stop/save/cancel).
 */

#include "cpcu_tui.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

/*============= DEMO STATE (PUBLIC) ========================================================*/

uint32_t  demo_pkts             =   0;
uint32_t  demo_gaps             =   0;
double    demo_phase            =   0.0;
uint32_t  demo_inf_count        =   0;
uint32_t  demo_fault_mask       =   FAULT_NONE;
uint64_t  demo_fault_onset_ms   =   0;
DemoWave  demo_wave             =   WAVE_SINE;
float     demo_freq_hz          =   100.0f;
uint8_t   demo_gesture          =   1;      /* HAND_SLOW */
uint8_t   demo_conf             =   94;

/*============= DEMO STATE (PRIVATE) =======================================================*/

static IPC_ControlBlock  demo_ctrl;
static IPC_SensorEntry   demo_ring[IPC_SENSOR_RING_SIZE];
static IPC_MotorCommand  demo_motor;
static IPC_Diagnostics   demo_diag;
static IPC_DSPExport     demo_dsp_export;

/*============= DATASET STATE (PUBLIC) =====================================================*/

DatasetState  ds_state          =   DS_IDLE;
DatasetMode   ds_mode           =   DS_MODE_FILTERED;
int           ds_label_idx      =   0;
char          ds_path[DATASET_PATH_MAX] = {0};
uint32_t      ds_samples        =   0;
uint32_t      ds_gaps           =   0;
uint32_t      ds_missed         =   0;
uint64_t      ds_start_ms       =   0;
uint64_t      ds_msg_until      =   0;

/*============= DATASET STATE (PRIVATE) ====================================================*/

static FILE        *ds_file         =   NULL;
static uint32_t     ds_last_head    =   0;
static uint8_t      ds_prev_seq     =   0;
static bool         ds_seq_seeded   =   false;

/*  One-pole IIR cascade. Crude but cheap alternative to scipy sosfilt.
 *  HP at 20 Hz (DC removal) + LP at 450 Hz (band limit). State is per
 *  channel — see ds_filter_step() below.                               */
typedef struct {
    double hp_prev_x;
    double hp_prev_y;
    double lp_prev_y;
} DatasetFilter;

#define DS_HP_ALPHA         0.9409      /* ~20 Hz HP at Fs=2 kHz   */
#define DS_LP_ALPHA         0.7520      /* ~450 Hz LP at Fs=2 kHz  */

static DatasetFilter ds_filter[WL_NUM_CHANNELS];

/*============= WAVEFORM BUFFER (PUBLIC) ===================================================*/

int   wave_sel_ch   =   0;
bool  wave_detail   =   false;

/*============= WAVEFORM BUFFER (PRIVATE) ==================================================*/

uint16_t wave_buf[WL_NUM_CHANNELS][WAVE_BUF_SIZE];   /* read by render */
uint32_t wave_count    =   0;                        /* read by render */
uint32_t wave_wr       =   0;                        /* read by render */
static uint32_t wave_last_head =   0;

/*  Forward-declared as extern so the render layer can read the buffers
 *  without us giving up the file-scope discipline elsewhere.            */

/*============= TIMING ====================================================================*/

uint64_t now_ms_wall(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000);
}

/*============= FAULT BANNER ==============================================================*/

const char *fault_banner(uint32_t mask)
{
    /* Single short label for the footer. Priority: radio > batt > others. */
    if(mask & FAULT_RADIO_FREEZE)   return "[INJ:RADIO_FREEZE]";
    if(mask & FAULT_BATT_LOW)       return "[INJ:BATT_LOW]";
    if(mask & FAULT_GAP_STORM)      return "[INJ:GAP_STORM]";
    if(mask & FAULT_RING_OVF)       return "[INJ:RING_OVF]";
    if(mask & FAULT_I2C_FAIL)       return "[INJ:I2C_FAIL]";
    return NULL;
}

/*============= DEMO INIT / TICK ==========================================================*/

void demo_init(IPC_Context *ipc)
{
    memset(&demo_ctrl, 0, sizeof(demo_ctrl));
    memset(demo_ring, 0, sizeof(demo_ring));
    memset(&demo_motor, 0, sizeof(demo_motor));
    memset(&demo_diag, 0, sizeof(demo_diag));
    memset(&demo_dsp_export, 0, sizeof(demo_dsp_export));

    demo_ctrl.magic         =   IPC_MAGIC;
    demo_ctrl.version       =   IPC_VERSION;
    atomic_store(&demo_ctrl.io_ready, 1);
    atomic_store(&demo_ctrl.dsp_ready, 1);
    atomic_store(&demo_ctrl.system_state, IPC_STATE_RUNNING);

    ipc->ctrl       =   &demo_ctrl;
    ipc->ring       =   demo_ring;
    ipc->motor      =   &demo_motor;
    ipc->diag       =   &demo_diag;
    ipc->dsp_export =   &demo_dsp_export;
}

/**
 *  Build one synthetic packet, pack it via WL_Pack, immediately unpack
 *  with WL_Unpack, and drop the rebuilt packet into the ring. This makes
 *  demo mode exercise the codec end-to-end (including WL_Pack, which is
 *  otherwise BSAU-only on the production path).
 */
static void demo_push_packet(IPC_Context *ipc, uint32_t p_idx)
{
    WL_Packet pkt;
    memset(&pkt, 0, sizeof(pkt));

    for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        {
            float phase_off = (float)ch * 0.2f;
            float v = demo_gen(demo_wave, (float)demo_phase, demo_freq_hz, phase_off);

            /* ADC quantisation noise. */
            float noise = ((float)(rand() % 5) - 2.0f) / 4095.0f * 3.3f;
            v += noise;
            if(v < 0.0f)  v = 0.0f;
            if(v > 3.3f)  v = 3.3f;
            pkt.samples[s].ch[ch] = (uint16_t)(v / 3.3f * 4095.0f);
        }
        demo_phase += 1.0 / 2000.0;   /* 2 kHz sample rate */
    }
    pkt.vbat_raw    =   4031;
    pkt.flags       =   0;
    pkt.seq         =   (uint8_t)((demo_pkts + p_idx) & 0xFF);
    pkt.tx_retry    =   0;
    pkt.pkt_loss    =   0;
    pkt.timestamp   =   (uint16_t)((demo_pkts + p_idx) & 0xFFFF);

    uint8_t raw[WL_PAYLOAD_SIZE];
    WL_Pack(&pkt, raw);
    WL_Packet rebuilt;
    WL_Unpack(raw, &rebuilt);

    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    IPC_SensorEntry *e = &ipc->ring[head & IPC_SENSOR_RING_MASK];
    memcpy(e->samples, rebuilt.samples, sizeof(e->samples));
    e->vbat_raw     =   rebuilt.vbat_raw;
    e->flags        =   rebuilt.flags;
    e->seq          =   rebuilt.seq;
    e->tx_retry     =   rebuilt.tx_retry;
    e->pkt_loss     =   rebuilt.pkt_loss;
    e->timestamp    =   rebuilt.timestamp;
    atomic_store(&ipc->ctrl->sensor_head, head + 1);
}

void demo_tick(IPC_Context *ipc)
{
    uint64_t now = now_ms_wall();

    bool radio_silent      = (demo_fault_mask & FAULT_RADIO_FREEZE) != 0;
    bool batt_low          = (demo_fault_mask & FAULT_BATT_LOW)     != 0;
    bool inject_gap_storm  = (demo_fault_mask & FAULT_GAP_STORM)    != 0;
    bool inject_ring_ovf   = (demo_fault_mask & FAULT_RING_OVF)     != 0;
    bool inject_i2c_fail   = (demo_fault_mask & FAULT_I2C_FAIL)     != 0;

    if(radio_silent)
    {
        uint64_t silence_ms = now - demo_fault_onset_ms;
        if(silence_ms > 2250)
            atomic_store(&ipc->ctrl->system_state, IPC_STATE_SAFE);
        else if(silence_ms > 750)
            atomic_store(&ipc->ctrl->system_state, IPC_STATE_INIT);
        /* No packets this tick. */
    }
    else
    {
        uint16_t vbat = batt_low ? 1600 : 4031;
        for(uint32_t p = 0; p < 100; p++)
        {
            demo_push_packet(ipc, p);
            uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
            if(head > 0) {
                IPC_SensorEntry *e = &ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK];
                e->vbat_raw = vbat;
            }
        }
        demo_pkts += 100;

        if(inject_gap_storm)
            demo_gaps += 10;

        if(batt_low || inject_i2c_fail || inject_ring_ovf)
            atomic_store(&ipc->ctrl->system_state, IPC_STATE_SAFE);
        else
            atomic_store(&ipc->ctrl->system_state, IPC_STATE_RUNNING);
    }

    demo_inf_count++;

    if(atomic_load(&ipc->ctrl->system_state) == IPC_STATE_RUNNING
       && demo_inf_count % 50 == 0)
        demo_gesture = (demo_gesture + 1) % 10;

    atomic_store(&ipc->diag->io_pkts_received, demo_pkts);
    atomic_store(&ipc->diag->io_seq_gaps, demo_gaps);
    atomic_store(&ipc->diag->io_nrf_init_status, 0);
    atomic_store(&ipc->diag->io_ring_overflows, inject_ring_ovf ? 150U : 0U);

    atomic_store(&ipc->diag->dsp_inferences, demo_inf_count);
    atomic_store(&ipc->diag->dsp_batches,    demo_inf_count * 2);
    atomic_store(&ipc->diag->dsp_max_latency_us, 3200);

    ipc->motor->gesture_id      =   demo_gesture;
    ipc->motor->confidence      =   demo_conf;
    for(int i = 0; i < IPC_NUM_SERVOS; i++)
        ipc->motor->servo_us[i] =   1500;

    atomic_store(&ipc->dsp_export->update_seq, demo_inf_count);
    ipc->dsp_export->num_classes       = 10;
    ipc->dsp_export->active_class      = demo_gesture;
    ipc->dsp_export->inference_time_us = 2800;
    snprintf((char *)ipc->dsp_export->gesture_name, IPC_MAX_GESTURE_NAME,
             "%s", CLS_NAMES[demo_gesture]);
    for(int c = 0; c < 10; c++)
        ipc->dsp_export->class_confidence[c] = (c == demo_gesture) ? 0.94f : 0.01f;
    for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        ipc->dsp_export->channel_rms[ch] = 0.28f + 0.04f * (ch % 3);
}

/*============= DATASET CAPTURE ===========================================================*/

/**
 *  Sanitise a label to a filesystem-safe stem. Must match the mapping in
 *  bsau_dataset_collector.py's sanitize_label() so a BSAU-side UART
 *  capture and a CPCU-side radio capture land on identical stems.
 */
static void ds_sanitize(const char *in, char *out, size_t outsz)
{
    size_t j = 0;
    for(size_t i = 0; in[i] && j + 4 < outsz; i++)
    {
        char c = in[i];
        if(c == '.')            { if(j + 1 < outsz) out[j++] = '_'; }
        else if(c == '<')       { if(j + 3 < outsz) { out[j++]='_'; out[j++]='L'; out[j++]='T'; } }
        else if(c == '>')       { if(j + 3 < outsz) { out[j++]='_'; out[j++]='G'; out[j++]='T'; } }
        else if(c == '=')       { if(j + 3 < outsz) { out[j++]='_'; out[j++]='E'; out[j++]='Q'; } }
        else if(isalnum((unsigned char)c) || c == '_' || c == '-')
                                { out[j++] = c; }
    }
    if(j == 0)
    {
        const char *fallback = "unlabeled";
        for(size_t k = 0; fallback[k] && j + 1 < outsz; k++) out[j++] = fallback[k];
    }
    out[j] = '\0';
}

/**
 *  Scan DATASET_OUT_DIR for files matching "{stem}_{N}_{mode}.csv" or
 *  legacy "{stem}_{N}.csv", return max N + 1.
 */
static int ds_next_index(const char *out_dir, const char *stem)
{
    DIR *d = opendir(out_dir);
    if(!d) return 0;

    size_t stem_len = strlen(stem);
    int max_n = -1;
    struct dirent *ent;
    while((ent = readdir(d)) != NULL)
    {
        const char *name = ent->d_name;
        if(strncmp(name, stem, stem_len) != 0) continue;
        if(name[stem_len] != '_') continue;

        const char *nstr = name + stem_len + 1;
        char *endp = NULL;
        long n = strtol(nstr, &endp, 10);
        if(endp == nstr) continue;

        /* Accept "{stem}_{N}.csv" (legacy) or "{stem}_{N}_{mode}.csv" */
        if(strcmp(endp, ".csv") == 0 ||
           strcmp(endp, "_filtered.csv") == 0 ||
           strcmp(endp, "_unfiltered.csv") == 0)
        {
            if(n > max_n) max_n = (int)n;
        }
    }
    closedir(d);
    return max_n + 1;
}

int ds_start_capture(IPC_Context *ipc)
{
    char stem[32];
    ds_sanitize(CLS_NAMES[ds_label_idx], stem, sizeof(stem));

    if(mkdir(DATASET_OUT_DIR, 0755) != 0 && errno != EEXIST)
    {
        /* Best-effort. The fopen() below will surface the failure. */
    }

    int idx = ds_next_index(DATASET_OUT_DIR, stem);
    const char *mode_str = (ds_mode == DS_MODE_FILTERED) ? "filtered" : "unfiltered";
    snprintf(ds_path, sizeof(ds_path), "%s/%s_%d_%s.csv",
             DATASET_OUT_DIR, stem, idx, mode_str);

    ds_file = fopen(ds_path, "w");
    if(!ds_file)
    {
        ds_path[0] = '\0';
        return -1;
    }

    setvbuf(ds_file, NULL, _IOLBF, 0);

    ds_samples      = 0;
    ds_gaps         = 0;
    ds_missed       = 0;
    ds_seq_seeded   = false;
    ds_start_ms     = now_ms_wall();
    ds_last_head    = atomic_load(&ipc->ctrl->sensor_head);

    memset(ds_filter, 0, sizeof(ds_filter));

    ds_state = DS_COLLECTING;
    return 0;
}

void ds_stop_capture(bool save)
{
    if(ds_file)
    {
        fflush(ds_file);
        fclose(ds_file);
        ds_file = NULL;
    }

    if(!save && ds_path[0])
    {
        (void)remove(ds_path);
    }

    ds_state     = save ? DS_SAVED : DS_CANCELLED;
    ds_msg_until = now_ms_wall() + 2000;
}

/**
 *  HP+LP cascade for FILTERED mode.
 */
static double ds_filter_step(DatasetFilter *f, double x)
{
    double y_hp = DS_HP_ALPHA * (f->hp_prev_y + x - f->hp_prev_x);
    f->hp_prev_x = x;
    f->hp_prev_y = y_hp;

    double y_lp = DS_LP_ALPHA * f->lp_prev_y + (1.0 - DS_LP_ALPHA) * y_hp;
    f->lp_prev_y = y_lp;

    return y_lp;
}

void ds_drain_ring_to_file(IPC_Context *ipc)
{
    if(ds_state != DS_COLLECTING || !ds_file) return;

    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    uint32_t new_entries = head - ds_last_head;

    if(new_entries == 0) return;

    if(new_entries > IPC_SENSOR_RING_SIZE)
    {
        ds_missed   += new_entries - IPC_SENSOR_RING_SIZE;
        new_entries  = IPC_SENSOR_RING_SIZE;
        ds_last_head = head - IPC_SENSOR_RING_SIZE;
    }

    char line[DATASET_LINE_MAX];

    for(uint32_t i = 0; i < new_entries; i++)
    {
        uint32_t idx = (ds_last_head + i) & IPC_SENSOR_RING_MASK;
        IPC_SensorEntry *e = &ipc->ring[idx];

        if(ds_seq_seeded)
        {
            uint8_t expected = (uint8_t)(ds_prev_seq + 1);
            if(e->seq != expected)
            {
                uint8_t delta = (uint8_t)(e->seq - expected);
                ds_gaps += delta;
            }
        }
        ds_prev_seq   = e->seq;
        ds_seq_seeded = true;

        for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        {
            int n = 0;
            if(ds_mode == DS_MODE_RAW)
            {
                n = snprintf(line, sizeof(line),
                             "%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                             (unsigned)e->samples[s].ch[0],
                             (unsigned)e->samples[s].ch[1],
                             (unsigned)e->samples[s].ch[2],
                             (unsigned)e->samples[s].ch[3],
                             (unsigned)e->samples[s].ch[4],
                             (unsigned)e->samples[s].ch[5],
                             (unsigned)e->samples[s].ch[6],
                             (unsigned)e->samples[s].ch[7]);
            }
            else
            {
                double v[WL_NUM_CHANNELS];
                for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
                {
                    double raw   = (double)e->samples[s].ch[ch];
                    double volts = raw * 3.3 / 4095.0 - 1.65;
                    v[ch]        = ds_filter_step(&ds_filter[ch], volts);
                }
                n = snprintf(line, sizeof(line),
                             "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",
                             v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
            }

            if(n > 0 && (size_t)n < sizeof(line))
            {
                if(fwrite(line, 1, (size_t)n, ds_file) == (size_t)n)
                    ds_samples++;
            }
        }
    }

    ds_last_head = head;
}

/*============= WAVEFORM RING =============================================================*/

void wave_peek_ring(IPC_Context *ipc)
{
    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    uint32_t new_entries = head - wave_last_head;
    if(new_entries == 0) return;
    if(new_entries > IPC_SENSOR_RING_SIZE) new_entries = IPC_SENSOR_RING_SIZE;

    uint32_t start = wave_last_head;
    for(uint32_t i = 0; i < new_entries; i++)
    {
        uint32_t idx = (start + i) & IPC_SENSOR_RING_MASK;
        IPC_SensorEntry *e = &ipc->ring[idx];

        for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
        {
            for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
                wave_buf[ch][wave_wr] = e->samples[s].ch[ch];

            wave_wr = (wave_wr + 1) % WAVE_BUF_SIZE;
            if(wave_count < WAVE_BUF_SIZE) wave_count++;
        }
    }
    wave_last_head = head;
}

/*==========================================================================================*/

