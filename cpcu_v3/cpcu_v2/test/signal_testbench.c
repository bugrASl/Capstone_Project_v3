/**
 *  @file   signal_testbench.c
 *  @brief  Signal integrity TUI — live 8-channel ADC plots with frequency/amplitude stats.
 */

#include <ncurses.h>
#include <locale.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdatomic.h>      /* IPC_ToolPresence publish */

#include "cpcu_ipc.h"
#include "demo_signals.h"

/*============= COLOR PAIRS ================================================================*/

#define CP_NORMAL       1
#define CP_GOOD         2
#define CP_WARN         3
#define CP_BAD          4
#define CP_CYAN         5
#define CP_DIM          6
#define CP_HEADER       7
#define CP_WAVE         8
#define CP_MAGENTA      9
#define CP_AXIS         10

/*============= LAYOUT (dynamic) ===========================================================*/

/* Minimum usable terminal size — below this we show a "too small" message */
#define TUI_W_MIN       60
#define TUI_H_MIN       18

/* Absolute cap on TUI width (so on ultra-wide terminals we don't waste space) */
#define TUI_W_MAX       140

#define REFRESH_US      50000       /* 20 Hz */

/* Globals recomputed every frame in layout_update() */
static int g_term_w         = 80;
static int g_term_h         = 24;
static int g_tui_w          = 80;   /* actual drawn width */
static int g_plot_w         = 60;   /* big-plot width */
static int g_plot_h         = 12;   /* big-plot height */
static int g_mini_w         = 30;   /* mini-plot width (per column) */
static int g_mini_h         = 4;    /* mini-plot height */
static int g_mini_col2_x    = 41;   /* x of 2nd column in all-channel view */

/*============= UNICODE BLOCK GLYPHS =======================================================*/

/*  Waveform rendering uses a line-trace renderer (see draw_waveform below)
 *  with 7-bit ASCII glyphs only — identical output on narrow libncurses and
 *  wide libncursesw, regardless of the user's LANG/LC_* settings.          */

/*============= SIGNAL ANALYSIS ============================================================*/

/* Rolling buffer: ~1 second of samples at 1 kHz (2 samples/packet * 1000 pkt/s) */
#define WAVE_BUF_SIZE   2048
#define ADC_VREF        3.3f
#define ADC_RESOLUTION  4095.0f

/* Goertzel frequency bins to test (Hz) — covers typical function generator range */
#define GOERTZEL_BINS   12
static const float GOERTZEL_FREQS[GOERTZEL_BINS] = {
    25.0f, 50.0f, 75.0f, 100.0f, 150.0f, 200.0f,
    250.0f, 300.0f, 400.0f, 500.0f, 750.0f, 1000.0f
};

/* Assumed sample rate — 2 samples per packet at 1 kHz packet rate */
#define SAMPLE_RATE     2000.0f

/*============= PER-CHANNEL STATE ==========================================================*/

typedef struct
{
    /* Rolling sample buffer (raw ADC 12-bit) */
    uint16_t    buf[WAVE_BUF_SIZE];
    uint32_t    write_idx;
    uint32_t    count;          /* Total samples seen (saturates at WAVE_BUF_SIZE) */

    /* Analysis results */
    float       dc_offset_v;    /* Mean voltage */
    float       vpp_v;          /* Peak-to-peak voltage */
    float       dominant_freq;  /* Goertzel dominant frequency (Hz) */
    float       dominant_mag;   /* Magnitude at dominant freq */
    float       snr_db;         /* Rough SNR: dominant bin power vs rest */
    uint16_t    adc_min;        /* Raw ADC min in buffer */
    uint16_t    adc_max;        /* Raw ADC max in buffer */
} ChannelState;

/*============= GLOBALS ====================================================================*/

static volatile sig_atomic_t g_run = 1;
static void on_sig(int s) { (void)s; g_run = 0; }

/* publish to IPC_ToolPresence slot 1 (signal_testbench).
 * The web bridge reads this and surfaces it on the dashboard's Tools
 * tab. Cheap — called once per main-loop iteration (~20 Hz). Empty
 * payload regions are zeroed so a partial write doesn't leak old
 * bytes. The companion call sigtb_tool_presence_clear() goes in
 * cleanup so the dashboard sees us go away when we exit cleanly. */
static void sigtb_tool_presence_publish(IPC_Context *ipc,
                                        int sel_ch,
                                        float rms,
                                        uint32_t drops)
{
    if(!ipc || !ipc->tool_presence) return;
    IPC_ToolSlot *t = &ipc->tool_presence->slot[IPC_TOOL_SLOT_SIGNAL];

    /* Name (16 B fixed). Written once is enough but it's cheap. */
    memset(t->tool_name, 0, IPC_TOOL_NAME_MAX);
    memcpy(t->tool_name, "signal_testbench",
           sizeof("signal_testbench") - 1 < IPC_TOOL_NAME_MAX
               ? sizeof("signal_testbench") - 1
               : IPC_TOOL_NAME_MAX);

    /* Payload (32 B). Layout per the bridge's decoder:
     *   [0]   selected channel (uint8)
     *   [1..4] RMS (float bits as uint32 LE)
     *   [5..8] drop counter (uint32 LE)
     *   [9..31] reserved, zeroed.
     */
    memset(t->payload, 0, IPC_TOOL_PAYLOAD_BYTES);
    t->payload[0] = (uint8_t)sel_ch;
    memcpy(&t->payload[1], &rms, 4);
    memcpy(&t->payload[5], &drops, 4);

    /* Heartbeat, then alive=1 last (release ordering so a reader that
     * sees alive=1 also sees the fresh name+payload+heartbeat). */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL +
                      (uint64_t)ts.tv_nsec / 1000ULL;
    atomic_store_explicit(&t->last_heartbeat_us, now_us,
                          memory_order_relaxed);
    atomic_store_explicit(&t->alive, 1, memory_order_release);
}

static void sigtb_tool_presence_clear(IPC_Context *ipc)
{
    if(!ipc || !ipc->tool_presence) return;
    IPC_ToolSlot *t = &ipc->tool_presence->slot[IPC_TOOL_SLOT_SIGNAL];
    atomic_store_explicit(&t->alive, 0, memory_order_release);
}

static ChannelState channels[WL_NUM_CHANNELS];
static int  selected_ch     = 0;    /* Currently displayed channel */
static bool view_all        = true; /* Show all 8 mini-plots vs one big plot */

/* Packet rate tracking */
static uint32_t prev_pkts   = 0;
static uint64_t prev_time   = 0;
static uint32_t pkt_rate    = 0;

/* Battery */
static float    batt_v      = 0.0f;
static uint16_t batt_raw    = 0;

/* Demo mode */
static bool     demo_mode   = false;
static uint32_t demo_pkt_count = 0;
static double   demo_phase  = 0.0;

/* Waveform + frequency selection (live-editable via hotkeys in --demo).
 *   w / W    Cycle SINE → SQUARE → TRI → SAW → NOISE → EMG → ECG → CHIRP
 *   [        Halve frequency  (minimum 10 Hz)
 *   ]        Double frequency (maximum 1000 Hz)
 */
static DemoWave demo_wave    = WAVE_SINE;
static float    demo_freq_hz = 100.0f;

/* Forward declaration (defined in SIGNAL ANALYSIS section below) */
static void channel_push_sample(ChannelState *ch, uint16_t sample);

/**
 *  Generate synthetic ADC samples as if a sine wave was connected.
 *  DC offset = 1.65V (mid-rail), amplitude per channel varies slightly.
 */
static void demo_generate_samples(void)
{
    double dt = 1.0 / SAMPLE_RATE;

    /* Generate 2 samples (one packet's worth) per call at ~1kHz packet rate */
    for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        {
            /* Selectable waveform via shared generator. Per-channel phase
             * offset so all 8 traces aren't perfectly identical. */
            float phase_off = (float)ch * 0.2f;
            float voltage   = demo_gen(demo_wave, (float)demo_phase,
                                       demo_freq_hz, phase_off);

            /* ADC quantisation noise (~2 LSB) */
            float noise = ((float)(rand() % 5) - 2.0f) / ADC_RESOLUTION * ADC_VREF;
            voltage += noise;

            /* Clamp to [0, 3.3V] and convert to 12-bit ADC */
            if(voltage < 0.0f) voltage = 0.0f;
            if(voltage > ADC_VREF) voltage = ADC_VREF;
            uint16_t adc_val = (uint16_t)(voltage / ADC_VREF * ADC_RESOLUTION);

            channel_push_sample(&channels[ch], adc_val);
        }
        demo_phase += dt;
    }

    demo_pkt_count++;

    /* Fake battery at 3.3V through divider */
    batt_raw    = 4031;
    batt_v      = 3.28f;
}

/*============= TIMING ====================================================================*/

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec) * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000);
}

/*============= LAYOUT =====================================================================*/

/**
 *  Recompute dynamic layout from current terminal size.
 *  Called once per frame — cheap.
 *
 *  Scaling rules:
 *      g_tui_w      = clamp(term_w, TUI_W_MIN, TUI_W_MAX)
 *      g_plot_w     = g_tui_w - 22       (leaves ~22 cols for analysis box)
 *      g_plot_h     = clamp(term_h - 9, 6, 16)
 *      g_mini_w     = (g_tui_w / 2) - 10  (two columns, small padding)
 *      g_mini_col2_x = g_tui_w / 2 + 1
 */
static void layout_update(void)
{
    getmaxyx(stdscr, g_term_h, g_term_w);

    /* Width */
    g_tui_w = g_term_w;
    if(g_tui_w < TUI_W_MIN) g_tui_w = TUI_W_MIN;
    if(g_tui_w > TUI_W_MAX) g_tui_w = TUI_W_MAX;

    /* Big plot */
    g_plot_w = g_tui_w - 22;
    if(g_plot_w < 30)  g_plot_w = 30;
    if(g_plot_w > 110) g_plot_w = 110;

    g_plot_h = g_term_h - 9;
    if(g_plot_h < 6)  g_plot_h = 6;
    if(g_plot_h > 16) g_plot_h = 16;

    /* Mini plots — 2 columns, 4 rows */
    g_mini_w = (g_tui_w / 2) - 10;
    if(g_mini_w < 20) g_mini_w = 20;
    if(g_mini_w > 60) g_mini_w = 60;

    g_mini_h = 4;
    g_mini_col2_x = g_tui_w / 2 + 1;
}

/*============= SIGNAL ANALYSIS FUNCTIONS ==================================================*/

static void channel_push_sample(ChannelState *ch, uint16_t sample)
{
    ch->buf[ch->write_idx] = sample;
    ch->write_idx = (ch->write_idx + 1) % WAVE_BUF_SIZE;
    if(ch->count < WAVE_BUF_SIZE) ch->count++;
}

/**
 *  Goertzel algorithm — compute magnitude at a single frequency bin.
 *  More efficient than FFT when you only need a few bins.
 */
static float goertzel_mag(const uint16_t *buf, uint32_t start, uint32_t N,
                          float target_freq, float sample_rate)
{
    float k     = (target_freq / sample_rate) * (float)N;
    float w     = (2.0f * (float)M_PI * k) / (float)N;
    float coeff = 2.0f * cosf(w);
    float s0    = 0.0f, s1 = 0.0f, s2 = 0.0f;

    for(uint32_t i = 0; i < N; i++)
    {
        uint32_t idx = (start + i) % WAVE_BUF_SIZE;
        float sample = (float)buf[idx] / ADC_RESOLUTION * ADC_VREF;
        s0 = sample + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    /* Magnitude */
    return sqrtf(s1 * s1 + s2 * s2 - coeff * s1 * s2);
}

/**
 *  Analyze a channel: compute min/max, DC offset, Vpp, dominant frequency.
 */
static void analyze_channel(ChannelState *ch)
{
    if(ch->count < 64) return;

    /* Min / max / DC */
    uint16_t mn = 0xFFFF, mx = 0;
    uint64_t sum = 0;
    uint32_t N      = (ch->count < WAVE_BUF_SIZE) ? ch->count : WAVE_BUF_SIZE;
    uint32_t start  = (ch->write_idx + WAVE_BUF_SIZE - N) % WAVE_BUF_SIZE;

    /* Scan min/max/mean over all N samples */
    for(uint32_t i = 0; i < N; i++)
    {
        uint32_t idx = (start + i) % WAVE_BUF_SIZE;
        uint16_t v   = ch->buf[idx];
        if(v < mn) mn = v;
        if(v > mx) mx = v;
        sum += v;
    }
    ch->adc_min     = mn;
    ch->adc_max     = mx;
    ch->dc_offset_v = (float)sum / (float)N / ADC_RESOLUTION * ADC_VREF;
    ch->vpp_v       = (float)(mx - mn) / ADC_RESOLUTION * ADC_VREF;

    /* Goertzel — find dominant frequency */
    uint32_t fft_N     = (N < 512) ? N : 512;
    uint32_t fft_start = (ch->write_idx + WAVE_BUF_SIZE - fft_N) % WAVE_BUF_SIZE;

    float max_mag   = 0.0f;
    float max_freq  = 0.0f;
    float total_pwr = 0.0f;

    for(int b = 0; b < GOERTZEL_BINS; b++)
    {
        float m = goertzel_mag(ch->buf, fft_start, fft_N,
                               GOERTZEL_FREQS[b], SAMPLE_RATE);
        float p = m * m;
        total_pwr += p;
        if(m > max_mag)
        {
            max_mag  = m;
            max_freq = GOERTZEL_FREQS[b];
        }
    }

    ch->dominant_freq = max_freq;
    ch->dominant_mag  = max_mag;

    /* SNR: dominant bin vs rest */
    float signal_pwr = max_mag * max_mag;
    float noise_pwr  = total_pwr - signal_pwr;
    if(noise_pwr < 1e-9f) noise_pwr = 1e-9f;
    ch->snr_db = 10.0f * log10f(signal_pwr / noise_pwr);
}

/*============= DRAWING HELPERS ============================================================*/

static void draw_hline(int row, int col, int len)
{
    mvhline(row, col, ACS_HLINE, len);
}

/**
 *  Draw an Unicode-block waveform plot from the channel's ring buffer.
 *
 *  Each character cell in the plot represents 8 vertical sub-pixels, giving
 *  (height * 8) effective vertical resolution. For each column we pick the
 *  cell that contains the sample, and within that cell render the lower-block
 *  glyph matching the sub-pixel remainder.
 */
static void draw_waveform(int row, int col, int width, int height,
                          ChannelState *ch, int color_pair)
{
    if(ch->count < 2) return;
    if(width < 4 || height < 2) return;

    /* Determine display range */
    uint16_t disp_min = ch->adc_min;
    uint16_t disp_max = ch->adc_max;
    if(disp_max <= disp_min) disp_max = disp_min + 1;

    /* How many samples to show — downsample if buffer has more than width */
    uint32_t avail  = (ch->count < WAVE_BUF_SIZE) ? ch->count : WAVE_BUF_SIZE;
    uint32_t show_n = (avail < (uint32_t)width) ? avail : (uint32_t)width;
    if(show_n < 2) return;

    /* Axis labels — dim */
    attron(COLOR_PAIR(CP_AXIS) | A_DIM);
    mvprintw(row, col - 1, "^");
    for(int y = 1; y < height; y++)
        mvprintw(row + y, col - 1, "|");
    mvhline(row + height, col, ACS_HLINE, width);
    mvprintw(row + height, col + width, ">");
    attroff(COLOR_PAIR(CP_AXIS) | A_DIM);

    /* Line-trace renderer — one glyph per column on the sample row, plus
     * diagonal/vertical connectors between samples at different rows. This
     * reads as a scope trace on both narrow libncurses and wide libncursesw
     * because every glyph is 7-bit ASCII.
     *
     * 3 sub-cells per row gives 3*height effective vertical resolution. */
    attron(COLOR_PAIR(color_pair));

    int total_sub = height * 3;
    int prev_row  = -1;

    for(uint32_t x = 0; x < show_n; x++)
    {
        uint32_t buf_offset = (avail * x) / show_n;
        uint32_t idx = (ch->write_idx + WAVE_BUF_SIZE - avail + buf_offset) % WAVE_BUF_SIZE;

        uint16_t val = ch->buf[idx];

        float frac = (float)(val - disp_min) / (float)(disp_max - disp_min);
        if(frac < 0.0f) frac = 0.0f;
        if(frac > 1.0f) frac = 1.0f;

        /* Sub position from the TOP of the plot. 0 = top, total_sub-1 = bottom */
        int sub = (int)((1.0f - frac) * (float)(total_sub - 1) + 0.5f);
        if(sub < 0)              sub = 0;
        if(sub > total_sub - 1)  sub = total_sub - 1;

        int y_row = sub / 3;               /* ncurses row (0 = top) */
        int y_rem = sub - y_row * 3;       /* 0 = upper, 2 = lower  */

        const char *pt = (y_rem == 0) ? "." :
                         (y_rem == 1) ? "-" :
                                        "o";

        mvprintw(row + y_row, col + (int)x, "%s", pt);

        /* Connector between samples at different rows */
        if(prev_row >= 0 && prev_row != y_row)
        {
            int step = (y_row > prev_row) ? 1 : -1;
            const char *conn = (step > 0) ? "\\" : "/";
            for(int r = prev_row + step; r != y_row; r += step)
                mvprintw(row + r, col + (int)x, "%s", conn);
        }

        prev_row = y_row;
    }

    attroff(COLOR_PAIR(color_pair));
}

/**
 *  Draw a compact mini-waveform for the all-channels overview.
 */
static void draw_mini_wave(int row, int col, int width, int height,
                           ChannelState *ch, bool is_selected)
{
    int cp = is_selected ? CP_MAGENTA : CP_WAVE;
    draw_waveform(row, col, width, height, ch, cp);
}

static void draw_analysis_box(int row, int col, int ch_idx)
{
    ChannelState *ch = &channels[ch_idx];

    mvprintw(row, col, "Dominant: ");
    attron(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
    printw("%6.0f Hz", ch->dominant_freq);
    attroff(COLOR_PAIR(CP_MAGENTA) | A_BOLD);

    mvprintw(row + 1, col, "Vpp:      ");
    attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
    printw("%5.3f V", ch->vpp_v);
    attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

    mvprintw(row + 2, col, "DC Offs:  ");
    attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
    printw("%5.3f V", ch->dc_offset_v);
    attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

    mvprintw(row + 3, col, "SNR:      ");
    int snr_cp = (ch->snr_db > 20.0f) ? CP_GOOD : (ch->snr_db > 10.0f) ? CP_WARN : CP_BAD;
    attron(COLOR_PAIR(snr_cp) | A_BOLD);
    printw("%5.1f dB", ch->snr_db);
    attroff(COLOR_PAIR(snr_cp) | A_BOLD);

    mvprintw(row + 4, col, "ADC:      ");
    attron(COLOR_PAIR(CP_DIM));
    printw("%u - %u", ch->adc_min, ch->adc_max);
    attroff(COLOR_PAIR(CP_DIM));
}

static void draw_too_small(void)
{
    erase();
    int cy = g_term_h / 2;
    int cx = g_term_w / 2 - 12;
    if(cx < 0) cx = 0;
    mvprintw(cy,     cx, "Terminal too small.");
    mvprintw(cy + 1, cx, "Need at least %dx%d.", TUI_W_MIN, TUI_H_MIN);
    mvprintw(cy + 2, cx, "Current: %dx%d", g_term_w, g_term_h);
    mvprintw(cy + 4, cx, "Press q to quit.");
    refresh();
}

static void draw_screen(IPC_Context *ipc, bool demo_mode_)
{
    erase();
    int r = 0;

    /* HEADER — full-width bar */
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    if(demo_mode_)
    {
        char hdr[160];
        snprintf(hdr, sizeof(hdr),
                 "  SIGNAL INTEGRITY TESTBENCH - v1.2 [DEMO %s %gHz]  (raw ADC, no DSP)",
                 demo_wave_label(demo_wave), (double)demo_freq_hz);
        mvprintw(r, 0, "%-*s", g_tui_w, hdr);
    }
    else
        mvprintw(r, 0, "%-*s", g_tui_w,
                 "  SIGNAL INTEGRITY TESTBENCH - InfiniTech CPCU v1.2  (raw ADC, no DSP)");
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
    r += 2;

    /* RADIO LINK STATUS */
    uint32_t pkts       = demo_mode_ ? demo_pkt_count : atomic_load(&ipc->diag->io_pkts_received);
    uint32_t gaps       = demo_mode_ ? 0 : atomic_load(&ipc->diag->io_seq_gaps);
    uint8_t  sys_state  = demo_mode_ ? IPC_STATE_RUNNING : atomic_load(&ipc->ctrl->system_state);
    uint8_t  io_rdy     = demo_mode_ ? 1 : atomic_load(&ipc->ctrl->io_ready);

    /* Stat row: positions scale to terminal width.
     * On a narrow (60-col) terminal we compress; on wide we space out. */
    int col_state = 1;
    int col_io    = 22;
    int col_rate  = 36;
    int col_gaps  = 52;
    int col_batt  = (g_tui_w > 75) ? 66 : 60;

    mvprintw(r, col_state, "State: ");
    int st_cp = (sys_state == IPC_STATE_RUNNING) ? CP_GOOD :
                (sys_state == IPC_STATE_SAFE) ? CP_BAD : CP_WARN;
    attron(COLOR_PAIR(st_cp) | A_BOLD);
    printw("%-8s", sys_state == IPC_STATE_RUNNING ? "RUNNING" :
                   sys_state == IPC_STATE_SAFE    ? "SAFE" : "INIT");
    attroff(COLOR_PAIR(st_cp) | A_BOLD);

    mvprintw(r, col_io, "IO: ");
    attron(COLOR_PAIR(io_rdy ? CP_GOOD : CP_BAD) | A_BOLD);
    printw("%s", io_rdy ? "READY" : "DOWN");
    attroff(COLOR_PAIR(io_rdy ? CP_GOOD : CP_BAD) | A_BOLD);

    mvprintw(r, col_rate, "Pkts/s: ");
    attron(COLOR_PAIR(pkt_rate > 900 ? CP_GOOD : pkt_rate > 0 ? CP_WARN : CP_BAD) | A_BOLD);
    printw("%-5u", pkt_rate);
    attroff(COLOR_PAIR(pkt_rate > 900 ? CP_GOOD : pkt_rate > 0 ? CP_WARN : CP_BAD) | A_BOLD);

    mvprintw(r, col_gaps, "Gaps: ");
    attron(COLOR_PAIR(gaps > 10 ? CP_WARN : CP_GOOD));
    printw("%-5u", gaps);
    attroff(COLOR_PAIR(gaps > 10 ? CP_WARN : CP_GOOD));

    mvprintw(r, col_batt, "Batt:");
    attron(COLOR_PAIR(batt_v > 3.0f ? CP_GOOD : CP_BAD) | A_BOLD);
    printw("%.2fV", batt_v);
    attroff(COLOR_PAIR(batt_v > 3.0f ? CP_GOOD : CP_BAD) | A_BOLD);
    r++;

    float loss_rate = (pkts > 0) ? (float)gaps / (float)pkts * 100.0f : 0.0f;
    mvprintw(r, 1, "Total: ");
    attron(COLOR_PAIR(CP_DIM));
    printw("%-8u", pkts);
    attroff(COLOR_PAIR(CP_DIM));
    mvprintw(r, col_io, "Loss: ");
    attron(COLOR_PAIR(loss_rate > 1.0f ? CP_BAD : loss_rate > 0.1f ? CP_WARN : CP_GOOD));
    printw("%.3f%%", loss_rate);
    attroff(COLOR_PAIR(loss_rate > 1.0f ? CP_BAD : loss_rate > 0.1f ? CP_WARN : CP_GOOD));

    mvprintw(r, col_gaps, "VBAT raw: ");
    attron(COLOR_PAIR(CP_DIM));
    printw("%u", batt_raw);
    attroff(COLOR_PAIR(CP_DIM));
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    if(view_all)
    {
        /* ═══ ALL-CHANNEL VIEW: 8 mini-plots (4 rows each, 2 columns) ═══ */
        for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        {
            int c_col   = (ch < 4) ? 1 : g_mini_col2_x;
            int c_row   = r + (ch % 4) * (g_mini_h + 2);
            bool is_sel = (ch == selected_ch);

            /* Channel label */
            int lbl_cp = is_sel ? CP_MAGENTA : CP_NORMAL;
            attron(COLOR_PAIR(lbl_cp) | (is_sel ? A_BOLD : 0));
            mvprintw(c_row, c_col, "%sch%d", is_sel ? ">" : " ", ch);
            attroff(COLOR_PAIR(lbl_cp) | (is_sel ? A_BOLD : 0));

            /* Compact stats next to label */
            attron(COLOR_PAIR(CP_DIM));
            printw(" %4.0fHz %5.3fVpp", channels[ch].dominant_freq, channels[ch].vpp_v);
            attroff(COLOR_PAIR(CP_DIM));

            /* Mini waveform */
            draw_mini_wave(c_row + 1, c_col + 1, g_mini_w, g_mini_h, &channels[ch], is_sel);
        }

        r += 4 * (g_mini_h + 2);
    }
    else
    {
        /* ═══ SINGLE-CHANNEL VIEW: big plot + full analysis ═══ */
        attron(A_BOLD);
        mvprintw(r, 1, "CHANNEL %d", selected_ch);
        attroff(A_BOLD);
        r++;

        /* Big waveform */
        draw_waveform(r, 3, g_plot_w, g_plot_h, &channels[selected_ch], CP_WAVE);

        /* Analysis box on the right — 4 cols padding after plot */
        draw_analysis_box(r, g_plot_w + 6, selected_ch);

        r += g_plot_h + 2;
    }

    /* KEYBINDINGS */
    draw_hline(r - 1, 0, g_tui_w);
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1,     "UP/DOWN  select ch    TAB  toggle all/single view");
    mvprintw(r + 1, 1, "q  quit   w:wave   [/]:freq   20 Hz refresh | reads IPC ring buffer");
    attroff(COLOR_PAIR(CP_DIM));

    /* Version tag bottom-right */
    if(g_tui_w > 72)
        mvprintw(r + 1, g_tui_w - 12, "v1.2");

    refresh();
}

/*============= MAIN =======================================================================*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);

    /* Check for --demo flag */
    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--demo") == 0 || strcmp(argv[i], "-d") == 0)
            demo_mode = true;
    }

    /* Init channel state */
    memset(channels, 0, sizeof(channels));
    srand(42);  /* Deterministic noise for demo */

    /* Open IPC (skip in demo mode) */
    IPC_Context ipc;
    memset(&ipc, 0, sizeof(ipc));

    if(!demo_mode)
    {
        if(IPC_Open(&ipc) != 0)
        {
            fprintf(stderr, "[SIG-TB] Cannot open shared memory. Is cpcu_kernel running?\n");
            fprintf(stderr, "  Try: ./signal_testbench --demo   (synthetic data, no hardware)\n");
            return 1;
        }
        printf("[SIG-TB] IPC opened. Starting TUI...\n");
    }
    else
    {
        printf("[SIG-TB] Demo mode - generating synthetic sine waves.\n");
    }

    /* Init ncurses */
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(0);

    if(has_colors())
    {
        start_color();
        use_default_colors();
        init_pair(CP_NORMAL,    COLOR_WHITE,    -1);
        init_pair(CP_GOOD,      COLOR_GREEN,    -1);
        init_pair(CP_WARN,      COLOR_YELLOW,   -1);
        init_pair(CP_BAD,       COLOR_RED,      -1);
        init_pair(CP_CYAN,      COLOR_CYAN,     -1);
        init_pair(CP_DIM,       COLOR_WHITE,    -1);
        init_pair(CP_HEADER,    COLOR_BLACK,    COLOR_CYAN);
        init_pair(CP_WAVE,      COLOR_GREEN,    -1);
        init_pair(CP_MAGENTA,   COLOR_MAGENTA,  -1);
        init_pair(CP_AXIS,      COLOR_WHITE,    -1);
    }

    uint32_t analysis_tick  = 0;

    /* ═══ MAIN LOOP ═══ */
    while(g_run)
    {
        /* Update dynamic layout from current terminal size */
        layout_update();

        if(demo_mode)
        {
            /* Generate ~50 packets per frame (simulates 1kHz at 20Hz refresh) */
            for(int p = 0; p < 50; p++)
                demo_generate_samples();

            pkt_rate = 1000;
        }
        else
        {
            /* 1. Drain ring buffer — consume all available sensor entries */
            IPC_SensorEntry batch[64];
            uint32_t got = IPC_PopSensorBatch(&ipc, batch, 64);

            for(uint32_t i = 0; i < got; i++)
            {
                IPC_SensorEntry *e = &batch[i];

                /* Push ADC samples into per-channel rolling buffers */
                for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
                {
                    for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
                    {
                        channel_push_sample(&channels[ch], e->samples[s].ch[ch]);
                    }
                }

                /* Battery: use latest packet's vbat */
                batt_raw    = e->vbat_raw;
                batt_v      = (float)batt_raw / ADC_RESOLUTION * ADC_VREF * 2.0f;
            }

            /* 2. Packet rate calculation */
            uint64_t t_now = now_ms();
            uint32_t pkts  = atomic_load(&ipc.diag->io_pkts_received);

            if(prev_time > 0 && t_now > prev_time)
            {
                uint32_t dt_ms = (uint32_t)(t_now - prev_time);
                if(dt_ms > 0)
                    pkt_rate = (pkts - prev_pkts) * 1000 / dt_ms;
            }
            prev_pkts = pkts;
            prev_time = t_now;
        }

        /* 3. Run signal analysis every ~250 ms (every 5th frame at 20 Hz) */
        analysis_tick++;
        if(analysis_tick >= 5)
        {
            analysis_tick = 0;
            for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
            {
                analyze_channel(&channels[ch]);
            }
        }

        /* 4. Draw — bail early if terminal is too small */
        if(g_term_w < TUI_W_MIN || g_term_h < TUI_H_MIN)
            draw_too_small();
        else
            draw_screen(&ipc, demo_mode);

        /* publish to IPC_ToolPresence so the web dashboard's
         * Tools tab can show that we're running. Skip in demo mode
         * (no live IPC). The amplitude_vpp value comes from the
         * channel's existing peak-to-peak voltage measurement. */
        if(!demo_mode)
        {
            uint32_t drops_now = atomic_load(&ipc.diag->io_pkts_dropped);
            sigtb_tool_presence_publish(&ipc, selected_ch,
                                        channels[selected_ch].vpp_v,
                                        drops_now);
        }

        /* 5. Handle input */
        int key = getch();
        switch(key)
        {
            case KEY_UP:
                selected_ch = (selected_ch - 1 + WL_NUM_CHANNELS) % WL_NUM_CHANNELS;
                break;
            case KEY_DOWN:
                selected_ch = (selected_ch + 1) % WL_NUM_CHANNELS;
                break;
            case '\t':      /* TAB toggles view mode */
            case '1':
                view_all = !view_all;
                break;

            /*-- Waveform selection --*/
            case 'w': case 'W':
                demo_wave = (DemoWave)(((int)demo_wave % 8) + 1);
                break;
            case '[':
                demo_freq_hz *= 0.5f;
                if(demo_freq_hz < 10.0f)   demo_freq_hz = 10.0f;
                break;
            case ']':
                demo_freq_hz *= 2.0f;
                if(demo_freq_hz > 1000.0f) demo_freq_hz = 1000.0f;
                break;

            case 'q':
            case 'Q':
                g_run = 0;
                break;
            default:
                break;
        }

        usleep(REFRESH_US);
    }

    /* Cleanup */
    /* clear our IPC_ToolPresence slot so the dashboard's
     * Tools tab notices us going away immediately, rather than
     * waiting for the 2-second heartbeat-stale timeout. */
    if(!demo_mode) sigtb_tool_presence_clear(&ipc);

    endwin();
    if(!demo_mode) IPC_Close(&ipc);

    printf("[SIG-TB] Exited cleanly.\n");
    return 0;
}

/*==========================================================================================*/

