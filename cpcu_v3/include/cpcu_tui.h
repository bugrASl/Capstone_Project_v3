/**
 *  @file   cpcu_tui.h
 *  @brief  TUI shared header — page IDs, layout globals, render/data prototypes.
 */

#ifndef CPCU_TUI_H
#define CPCU_TUI_H

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>             /* memcpy used by tui_lat_u32 inline */

#include "cpcu_ipc.h"
#include "wireless_packet.h"
#include "demo_signals.h"           /* DemoWave enum + waveform helpers */

/*============= COLOR PAIRS ================================================================*/

#define CP_NORMAL       1
#define CP_GOOD         2
#define CP_WARN         3
#define CP_BAD          4
#define CP_CYAN         5
#define CP_DIM          6
#define CP_HEADER       7
#define CP_BAR_FILL     8
#define CP_BAR_EMPTY    9
#define CP_MAGENTA      10

/*============= LAYOUT =====================================================================*/

#define TUI_MIN_WIDTH       72
#define TUI_MIN_HEIGHT      24
#define REFRESH_US          100000      /* 10 Hz */

/* IO heartbeat thresholds (relative to cpcu_io's HEARTBEAT_INTERVAL_US =
 * 100 ms; see comment block in cpcu_tui_render.c). */
#define IO_HB_WARN_MS       200u
#define IO_HB_BAD_MS        500u

/* Layout globals — owned by cpcu_tui_render.c, refreshed by layout_update(). */
extern int  g_term_w;
extern int  g_term_h;
extern int  g_tui_w;
extern int  g_col_r;
extern int  g_bar_w;
extern int  g_slider_w;

/*============= SERVO CONFIG ===============================================================*/

#define SERVO_COUNT     PCA_SERVO_COUNT

extern const char     *SERVO_NAMES[];
extern const uint16_t  SERVO_MIN[];
extern const uint16_t  SERVO_MAX[];
extern const char     *CLS_NAMES[];

/*============= PAGES ======================================================================*/

typedef enum {
    PAGE_OVERVIEW = 0,
    PAGE_RADIO,
    PAGE_DSP,
    PAGE_WAVES,
    PAGE_HEALTH,
    PAGE_DATASET,
    PAGE_CONFIG,
    PAGE_COUNT
} Page;

extern const char *PAGE_TITLES[];

/*============= MAIN-MODULE STATE ==========================================================*/

extern volatile sig_atomic_t  g_run;
extern Page                   current_page;
extern bool                   demo_mode;
extern bool                   show_splash;

/*============= DEMO STATE / FAULT INJECTION ===============================================*/

typedef enum {
    FAULT_NONE          = 0,
    FAULT_RADIO_FREEZE  = 1 << 0,
    FAULT_BATT_LOW      = 1 << 1,
    FAULT_GAP_STORM     = 1 << 2,
    FAULT_RING_OVF      = 1 << 3,
    FAULT_I2C_FAIL      = 1 << 4,
} DemoFault;

/* Demo state — defined in cpcu_tui_data.c, read by render + main key handler. */
extern uint32_t  demo_pkts;
extern uint32_t  demo_gaps;
extern double    demo_phase;
extern uint32_t  demo_inf_count;
extern uint32_t  demo_fault_mask;
extern uint64_t  demo_fault_onset_ms;
extern DemoWave  demo_wave;
extern float     demo_freq_hz;
extern uint8_t   demo_gesture;
extern uint8_t   demo_conf;

/*============= DATASET CAPTURE ============================================================*/

#define DATASET_OUT_DIR     "./datasets"
#define DATASET_LABEL_COUNT 10
#define DATASET_PATH_MAX    256
#define DATASET_LINE_MAX    192

typedef enum {
    DS_IDLE = 0,
    DS_COLLECTING,
    DS_SAVED,
    DS_CANCELLED,
} DatasetState;

typedef enum {
    DS_MODE_FILTERED = 0,
    DS_MODE_RAW,
} DatasetMode;

/* Dataset state — defined in cpcu_tui_data.c, read by render + main key handler. */
extern DatasetState  ds_state;
extern DatasetMode   ds_mode;
extern int           ds_label_idx;
extern char          ds_path[DATASET_PATH_MAX];
extern uint32_t      ds_samples;
extern uint32_t      ds_gaps;
extern uint32_t      ds_missed;
extern uint64_t      ds_start_ms;
extern uint64_t      ds_msg_until;

/*============= WAVEFORM BUFFER ============================================================*/

#define WAVE_BUF_SIZE   512
#define WAVE_PLOT_H     8
#define WAVE_PLOT_H_BIG 14
#define WAVE_ADC_MAX    4095.0f

extern int   wave_sel_ch;
extern bool  wave_detail;

/* Owned by cpcu_tui_data.c, read-only from render. */
extern uint16_t  wave_buf[WL_NUM_CHANNELS][WAVE_BUF_SIZE];
extern uint32_t  wave_count;
extern uint32_t  wave_wr;

/*============= END-TO-END LATENCY ACCESSORS ===============================================*/
/*
 *  cpcu_dsp.py publishes per-tick latency measurements into the
 *  IPC_DSPExport.padding region. These offsets MUST stay in sync with
 *  cpcu_ipc_bridge.py's EXPORT_* constants:
 *      EXPORT_PKT_LATENCY  = 100   uint32  packet rx → servo write   (µs)
 *      EXPORT_SEQ_AGE      = 104   uint32  oldest seq age in window  (pkts)
 *      EXPORT_RING_DWELL   = 108   uint32  IPC ring drain time       (µs)
 *      EXPORT_DSP_US       = 112   uint32  DSP compute + inference   (µs)
 *      EXPORT_HYST_CONSEC  = 116   uint8   debounce counter
 *      EXPORT_HYST_NEEDED  = 117   uint8   debounce threshold
 *      EXPORT_HYST_TYPE    = 118   uint8   0=r2a, 1=a2r, 2=a2a
 *
 *  IPC_DSPExport's first 100 bytes are typed fields (channel_rms, gesture
 *  name, class_confidence, num_classes, active_class, inference_time_us,
 *  update_seq). Bytes 100-231 are _pad1[132]. We reach into the padding
 *  via a uint8_t* view + offset to stay within strict-aliasing rules.
 *
 *  All values are 0 until cpcu_dsp.py publishes for the first time, so
 *  callers should check (val != 0) before colouring as "valid".
 */
#define TUI_LAT_OFF_PKT_TO_SERVO    100
#define TUI_LAT_OFF_SEQ_AGE         104
#define TUI_LAT_OFF_RING_DWELL      108
#define TUI_LAT_OFF_DSP_COMPUTE     112
#define TUI_LAT_OFF_HYST_CONSEC     116
#define TUI_LAT_OFF_HYST_NEEDED     117
#define TUI_LAT_OFF_HYST_TYPE       118

static inline uint32_t tui_lat_u32(const IPC_DSPExport *e, size_t off)
{
    if(!e) return 0;
    uint32_t v;
    memcpy(&v, ((const uint8_t *)e) + off, sizeof(v));
    return v;
}

static inline uint32_t tui_lat_pkt_to_servo_us (const IPC_DSPExport *e)
{ return tui_lat_u32(e, TUI_LAT_OFF_PKT_TO_SERVO); }

static inline uint32_t tui_lat_seq_age         (const IPC_DSPExport *e)
{ return tui_lat_u32(e, TUI_LAT_OFF_SEQ_AGE); }

static inline uint32_t tui_lat_ring_dwell_us   (const IPC_DSPExport *e)
{ return tui_lat_u32(e, TUI_LAT_OFF_RING_DWELL); }

static inline uint32_t tui_lat_dsp_compute_us  (const IPC_DSPExport *e)
{ return tui_lat_u32(e, TUI_LAT_OFF_DSP_COMPUTE); }

/* Constants matching cpcu_dsp.py's hardware-latency budget (per-segment
 * datasheet/wall-clock estimates). These are the values shown in the
 * "constant" rows of the waterfall — they don't change per tick. */
#define TUI_LAT_ADC_PACK_US         226u    /* STM32 ADC + WL_Pack */
#define TUI_LAT_WIRELESS_US         332u    /* NRF24L01+ ESB TX+ACK */
#define TUI_LAT_SPI_UNPACK_US        36u    /* CPCU SPI rx + Unpack */
#define TUI_LAT_SMOOTHER_I2C_US     610u    /* SMOOTH_Update + I²C */
#define TUI_LAT_SERVO_MECH_US     15000u    /* SG90 mechanical */
#define TUI_LAT_TRANSPORT_US                                            \
        (TUI_LAT_ADC_PACK_US + TUI_LAT_WIRELESS_US + TUI_LAT_SPI_UNPACK_US)

/*============= IO HEARTBEAT THRESHOLDS — see top of file ==================================*/

/*============= API: data layer (cpcu_tui_data.c) ==========================================*/

void          demo_init               (IPC_Context *ipc);
void          demo_tick               (IPC_Context *ipc);
const char   *fault_banner            (uint32_t mask);
uint64_t      now_ms_wall             (void);

int           ds_start_capture        (IPC_Context *ipc);
void          ds_stop_capture         (bool save);
void          ds_drain_ring_to_file   (IPC_Context *ipc);

void          wave_peek_ring          (IPC_Context *ipc);

/*============= API: render layer (cpcu_tui_render.c) ======================================*/

void          layout_update           (void);

uint64_t      now_ms                  (void);

int           draw_header             (int r);
void          draw_footer             (int r);

void          draw_page_overview      (int r, IPC_Context *ipc,
                                       uint32_t pkt_rate, float loss_rate,
                                       uint32_t up_h, uint32_t up_m, uint32_t up_s);
void          draw_page_radio         (int r, IPC_Context *ipc,
                                       uint32_t pkt_rate, float loss_rate,
                                       uint32_t up_h, uint32_t up_m, uint32_t up_s);
void          draw_page_dsp           (int r, IPC_Context *ipc);
void          draw_page_waves         (int r, IPC_Context *ipc);
void          draw_page_health        (int r, IPC_Context *ipc,
                                       uint32_t pkt_rate, float loss_rate);
void          draw_page_dataset       (int r, IPC_Context *ipc);
void          draw_page_config        (int r, IPC_Context *ipc);

void          draw_waveform           (int row, int col, int width, int height,
                                       int ch_idx, int color_pair);

#endif /* CPCU_TUI_H */

