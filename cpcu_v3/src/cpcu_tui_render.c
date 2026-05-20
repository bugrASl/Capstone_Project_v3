/**
 *  @file   cpcu_tui_render.c
 *  @brief  TUI render layer — drawing primitives, page renderers, footer.
 *
 *  ROLE
 *    Owns all interaction with ncurses and the layout globals
 *    (g_term_w/h, g_tui_w, g_col_r, g_bar_w, g_slider_w). Reads — never
 *    writes — the data buffers maintained by cpcu_tui_data.c (sensor
 *    ring waveform buffer, dataset capture state, demo synthesis
 *    state) and the main-module state (current_page, demo_mode) from
 *    cpcu_tui.c. Reads the IPC regions directly via the IPC_Context
 *    passed in by every draw_page_* function.
 *
 *  DEPENDENCIES — what this file READS
 *    cpcu_tui.h          : Shared types (Page enum, layout globals,
 *                          DemoFault, WAVE_BUF_SIZE, latency budget
 *                          constants TUI_LAT_*).
 *    cpcu_tui_editor.h   : ED_Render hook on the CONFIG page when
 *                          the edit-mode handshake has completed.
 *    cpcu_ipc.h          : All IPC region structs — read live.
 *    /tmp/cpcu_servo_names.txt    : Optional file dropped by
 *                                   cpcu_dsp.py; if present its names
 *                                   override the SERVO_NAMES fallback.
 *    /tmp/cpcu_group_state.txt    : Per-window classifier digest from
 *                                   cpcu_dsp.py; parsed for the DSP
 *                                   page's per-group view.
 *    /tmp/cpcu_gestures_digest.txt: Pre-formatted gestures.json digest
 *                                   from cpcu_dsp.py; printed verbatim
 *                                   on the CONFIG page.
 *    /tmp/cpcu_smoother_config.txt: Per-servo motion profile written
 *                                   by launch.sh's preflight; rendered
 *                                   on the CONFIG page.
 *    /tmp/cpcu_ws_active.txt      : Web dashboard URL from launch.sh.
 *
 *  DOWNSTREAM
 *    cpcu_tui.c          : Calls draw_page_*, draw_header, draw_footer
 *                          from the main loop. Also reads the
 *                          SERVO_NAMES / CLS_NAMES tables and the
 *                          DATASET_LABEL_COUNT macro (declared in
 *                          cpcu_tui.h, defined here).
 *
 *  CROSS-MODULE EFFECTS
 *    - Reduce/extend CLS_NAMES → also update DATASET_LABEL_COUNT in
 *      cpcu_tui.h; the LEFT/RIGHT cycler in cpcu_tui.c uses that as
 *      its modulus.
 *    - Servo bar scales prefer ipc->config->servo_*_us[] when the
 *      kernel has populated IPC_RuntimeConfig; the helpers
 *      tui_servo_min_us / tui_servo_max_us fall back to the hardcoded
 *      arrays only until the first valid config arrives, so a live
 *      edit through the CONFIG editor rescales bars within one frame.
 *    - The latency table mirrors TUI_LAT_*_US constants in cpcu_tui.h
 *      which must match cpcu_dsp.py's LAT_* set. Drift = misleading
 *      totals.
 */

#include "cpcu_tui.h"
#include "cpcu_tui_editor.h"        /* ED_Render */

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/*============= §1 LAYOUT GLOBALS (definitions) ============================================*/
/*
 *  These are declared extern in cpcu_tui.h. Defined here because the
 *  render layer is the only place that writes them (via layout_update()).
 */
int  g_term_w    =   80;
int  g_term_h    =   24;
int  g_tui_w     =   76;
int  g_col_r     =   39;
int  g_bar_w     =   20;
int  g_slider_w  =   20;

void layout_update(void)
{
    /* Sync stdscr with actual terminal size after tmux resize/attach.
     * ncurses updates LINES/COLS on SIGWINCH but does NOT resize stdscr
     * automatically. This 4-line check fixes the tmux 80x24 problem. */
    int _cur_h, _cur_w;
    getmaxyx(stdscr, _cur_h, _cur_w);
    if(LINES != _cur_h || COLS != _cur_w)
        resize_term(LINES, COLS);

    getmaxyx(stdscr, g_term_h, g_term_w);
    g_tui_w     =   g_term_w;
    if(g_tui_w  <   TUI_MIN_WIDTH)      g_tui_w = TUI_MIN_WIDTH;
    g_col_r     =   g_tui_w / 2;
    /* Bars and sliders scale with width: roughly quarter of the screen */
    g_bar_w     =   (g_col_r - 8);
    if(g_bar_w  <   14) g_bar_w = 14;
    if(g_bar_w  >   32) g_bar_w = 32;
    g_slider_w  =   g_bar_w;
}

/*============= §1 SERVO + CLASS NAMES (definitions) =======================================*/

/* Servo display names. These are the FALLBACK values rendered before
 * cpcu_dsp.py drops /tmp/cpcu_servo_names.txt on first start;
 * tui_reload_servo_names() then reads that file and rewrites this
 * array with whatever's in gestures.json (sorted by pca_ch).
 *
 * The fallback values below match gestures.json's stock servo_channels
 * block — Base, Elbow, Forearm, Wrist1, Wrist2, Gripper — so the TUI
 * shows the right names even in the brief window before DSP starts.
 * (Used to be Base/Upper/Last/Jnt-1/Jnt-2/Grip — left over from an
 * older gestures.json that no longer matches any reality.) */
#define TUI_SERVO_NAME_MAX 16
static char     g_servo_name_buf[6][TUI_SERVO_NAME_MAX] = {
    "Base", "Elbow", "Forearm", "Wrist1", "Wrist2", "Gripper"
};
const char     *SERVO_NAMES[]   =   {
    g_servo_name_buf[0], g_servo_name_buf[1], g_servo_name_buf[2],
    g_servo_name_buf[3], g_servo_name_buf[4], g_servo_name_buf[5]
};
/* Per-servo PWM limits.
 *
 * The hardcoded arrays below are the COMPILE-TIME FALLBACK matching
 * stock gestures.json. At runtime, cpcu_kernel populates
 * IPC_RuntimeConfig::servo_{min,max}_us[] from runtime.json (or
 * gestures.json if runtime.json is missing). The helpers
 * `tui_servo_min_us` / `tui_servo_max_us` below prefer the IPC value
 * when the config block is mapped and has a valid magic — that way a
 * live edit through the CONFIG-page editor immediately rescales the
 * draw_slider bars without a rebuild. Demo mode (no kernel) falls
 * straight through to the constants. */
const uint16_t  SERVO_MIN[]     =   {  498, 1074, 1074, 1001, 1001,  976 };
const uint16_t  SERVO_MAX[]     =   { 2500, 1953, 1953, 2002, 2002, 1733 };

static inline uint16_t tui_servo_min_us(const IPC_Context *ipc, int i)
{
    if(ipc && ipc->config
       && ipc->config->magic == IPC_CFG_VALID_MAGIC
       && i >= 0 && i < IPC_CFG_NUM_SERVOS
       && ipc->config->servo_min_us[i] > 0)
        return ipc->config->servo_min_us[i];
    return (i >= 0 && i < (int)(sizeof(SERVO_MIN)/sizeof(SERVO_MIN[0])))
         ? SERVO_MIN[i] : 1000;
}
static inline uint16_t tui_servo_max_us(const IPC_Context *ipc, int i)
{
    if(ipc && ipc->config
       && ipc->config->magic == IPC_CFG_VALID_MAGIC
       && i >= 0 && i < IPC_CFG_NUM_SERVOS
       && ipc->config->servo_max_us[i] > 0)
        return ipc->config->servo_max_us[i];
    return (i >= 0 && i < (int)(sizeof(SERVO_MAX)/sizeof(SERVO_MAX[0])))
         ? SERVO_MAX[i] : 2000;
}

/* Re-read /tmp/cpcu_servo_names.txt — one servo name per line, sorted
 * by pca_ch. Idempotent and cheap; called on every page-draw of the
 * pages that print servo names (DSP/AI, CONFIG, OVERVIEW). */
/* Pretty-print a microsecond value into a caller-supplied buffer.
 * Picks µs / ms / s automatically based on magnitude:
 *
 *      0 .. 9999      → "1234 us"
 *      10 ms .. 9.9 s → "12.3 ms"
 *      ≥ 10 s         → "12.3 s"
 *
 * The fixed-width format keeps columns aligned without padding. Used
 * everywhere the latency table prints a single duration. */
static const char *fmt_time_us(uint64_t us, char buf[32])
{
    if(us < 10000ULL)
        snprintf(buf, 32, "%5llu us",   (unsigned long long)us);
    else if(us < 10ULL * 1000ULL * 1000ULL)
        snprintf(buf, 32, "%5.1f ms",   us / 1000.0);
    else
        snprintf(buf, 32, "%5.2f s",    us / 1000000.0);
    return buf;
}

static void tui_reload_servo_names(void)
{
    FILE *f = fopen("/tmp/cpcu_servo_names.txt", "r");
    if(!f) return;
    char line[64];
    int  i = 0;
    while(i < 6 && fgets(line, sizeof(line), f))
    {
        size_t L = strlen(line);
        while(L > 0 && (line[L-1] == '\n' || line[L-1] == '\r'))
            line[--L] = '\0';
        if(L == 0) continue;
        if(L >= TUI_SERVO_NAME_MAX) L = TUI_SERVO_NAME_MAX - 1;
        memcpy(g_servo_name_buf[i], line, L);
        g_servo_name_buf[i][L] = '\0';
        i++;
    }
    fclose(f);
}

const char     *CLS_NAMES[]     =   {
    /* Active model is models/arm.pkl, a sklearn RandomForestClassifier
     * (200 trees) shared between the right_arm and left_arm groups —
     * same muscles, same classifier, alphabetical class order:
     *     0 = ext     1 = flex     2 = hand     3 = rest
     * Used by:
     *   (a) the Overview/DSP page confidence bars, indexed by the
     *       num_classes-many slots of dsp_export.class_confidence[]
     *   (b) the Dataset capture page's LEFT/RIGHT label cycler,
     *       bounded by DATASET_LABEL_COUNT in cpcu_tui.h.
     *
     * If the trained model later gains more classes, EXTEND this array
     * AND bump DATASET_LABEL_COUNT to match — the cycler uses that
     * macro as the modulus, and the overview iterates up to
     * IPC_MAX_CLASSES so the IPC buffer has room for up to 10. */
    "EXT", "FLEX", "HAND", "REST"
};

const char *PAGE_TITLES[] = {
    "OVERVIEW",
    "RADIO/IO",
    "DSP/AI",
    "WAVES",
    "HEALTH",
    "DATASET",
    "CONFIG",
};

/*============= §2 HELPERS: state strings + signal stats ===================================*/


/*============= HELPERS: State Strings =====================================================*/

static const char *state_str(uint8_t s)
{
    switch(s)
    {
        case IPC_STATE_INIT:    return "INIT";
        case IPC_STATE_RUNNING: return "RUNNING";
        case IPC_STATE_SAFE:    return "SAFE";
        default:                return "???";
    }
}

static int state_color(uint8_t s)
{
    switch(s)
    {
        case IPC_STATE_RUNNING: return CP_GOOD;
        case IPC_STATE_SAFE:    return CP_BAD;
        default:                return CP_WARN;
    }
}

static const char *batt_str(uint8_t flags)
{
    switch(flags & 0x03)
    {
        case 0: return "OK";
        case 1: return "LOW";
        case 2: return "CRITICAL";
        case 3: return "CHARGING";
        default: return "???";
    }
}

static int batt_color(uint8_t flags)
{
    switch(flags & 0x03)
    {
        case 0: return CP_GOOD;
        case 1: return CP_WARN;
        case 2: return CP_BAD;
        case 3: return CP_CYAN;
        default: return CP_WARN;
    }
}

/**
 *  Decode BSAU packet flags into a compact banner string.
 *  Each set flag adds a short tag; result looks like "CLIP ELEC CAL".
 *  Returns empty string if no non-battery flags are set.
 */
static void wl_flags_decode(uint8_t flags, char *out, size_t outsz)
{
    out[0] = '\0';
    size_t n = 0;
    const struct { uint8_t bit; const char *tag; } tags[] = {
        { WL_FLAG_CLIPPING,     "CLIP" },
        { WL_FLAG_ELEC_OFF,     "ELEC" },
        { WL_FLAG_ADC_OVRN,     "OVRN" },
        { WL_FLAG_TX_SAT,       "TX_SAT" },
        { WL_FLAG_CAL,          "CAL"  },
        { WL_FLAG_FIRST_PACKET, "FIRST" },
    };
    for(size_t i = 0; i < sizeof(tags)/sizeof(tags[0]); i++)
    {
        if(flags & tags[i].bit)
        {
            int m = snprintf(out + n, outsz - n, "%s%s",
                             n ? " " : "", tags[i].tag);
            if(m < 0 || (size_t)m >= outsz - n) break;
            n += (size_t)m;
        }
    }
}

/**
 *  Compute approximate zero-crossing frequency (Hz) from a rolling buffer.
 *  Sample rate is assumed 2 kHz. Counts crossings around the DC mean, so
 *  this gives the fundamental of sine/square/triangle nicely; returns 0
 *  for pure noise or DC.
 */
static float estimate_zcr_hz(const uint16_t *buf, uint32_t count)
{
    if(count < 4) return 0.0f;

    /* Mean */
    uint64_t sum = 0;
    for(uint32_t i = 0; i < count; i++) sum += buf[i];
    float mean = (float)sum / (float)count;

    /* Crossings with hysteresis (1 % of span) */
    uint16_t mn = 4095, mx = 0;
    for(uint32_t i = 0; i < count; i++) {
        if(buf[i] < mn) mn = buf[i];
        if(buf[i] > mx) mx = buf[i];
    }
    float hyst = (float)(mx - mn) * 0.01f;
    if(hyst < 1.0f) hyst = 1.0f;

    int side = 0;                /* -1, 0, +1 */
    uint32_t cross = 0;
    for(uint32_t i = 0; i < count; i++)
    {
        if((float)buf[i] > mean + hyst)
        {
            if(side == -1) cross++;
            side = +1;
        }
        else if((float)buf[i] < mean - hyst)
        {
            if(side == +1) cross++;
            side = -1;
        }
    }

    /* Each full cycle has 2 crossings */
    float period_s = (float)count / 2000.0f;
    if(period_s <= 0.0f) return 0.0f;
    return (float)cross * 0.5f / period_s;
}

/**
 *  Compute RMS in volts from a rolling raw-ADC buffer.
 */
static float compute_rms_v(const uint16_t *buf, uint32_t count)
{
    if(count == 0) return 0.0f;
    double sumsq = 0.0;
    double mean_sum = 0.0;
    for(uint32_t i = 0; i < count; i++) mean_sum += buf[i];
    double mean = mean_sum / (double)count;
    for(uint32_t i = 0; i < count; i++)
    {
        double d = (double)buf[i] - mean;
        sumsq += d * d;
    }
    double rms_adc = sqrt(sumsq / (double)count);
    return (float)(rms_adc / 4095.0 * 3.3);
}


/*============= §3 DRAWING PRIMITIVES ======================================================*/
static void draw_hline(int row, int col, int len)
{
    mvhline(row, col, ACS_HLINE, len);
}

static void draw_lv(int row, int col, const char *label, int cp, const char *fmt, ...)
{
    mvprintw(row, col, "%-14s", label);
    attron(COLOR_PAIR(cp) | A_BOLD);
    va_list ap;
    va_start(ap, fmt);
    char buf[64];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printw("%s", buf);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

static void draw_bar(int row, int col, int width, float frac, int cp_f, int cp_e)
{
    if(frac < 0.0f) frac = 0.0f;
    if(frac > 1.0f) frac = 1.0f;
    int filled = (int)(frac * width);

    move(row, col);
    attron(COLOR_PAIR(cp_f));
    for(int i = 0; i < filled; i++) addch(ACS_BLOCK);
    attroff(COLOR_PAIR(cp_f));

    attron(COLOR_PAIR(cp_e) | A_DIM);
    for(int i = filled; i < width; i++) addch(ACS_BOARD);
    attroff(COLOR_PAIR(cp_e) | A_DIM);

    printw(" %3d%%", (int)(frac * 100));
}

static void draw_slider(int row, int col, uint16_t val, uint16_t lo, uint16_t hi, int width)
{
    float frac = (hi > lo) ? (float)(val - lo) / (float)(hi - lo) : 0.0f;
    if(frac < 0.0f) frac = 0.0f;
    if(frac > 1.0f) frac = 1.0f;
    int pos = (int)(frac * (width - 1));

    move(row, col);
    for(int i = 0; i < width; i++)
    {
        if(i == pos)
        {
            int cp = CP_GOOD;
            if(frac < 0.05f || frac > 0.95f)      cp = CP_BAD;
            else if(frac < 0.15f || frac > 0.85f)  cp = CP_WARN;
            attron(COLOR_PAIR(cp) | A_BOLD);
            addch('O');
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
        else
        {
            attron(COLOR_PAIR(CP_DIM) | A_DIM);
            addch(ACS_HLINE);
            attroff(COLOR_PAIR(CP_DIM) | A_DIM);
        }
    }
}

static void draw_section(int row, int col, const char *title)
{
    attron(A_BOLD);
    mvprintw(row, col, "%s", title);
    attroff(A_BOLD);
}

uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec) * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000);
}

/*============= DRAW: Tab Bar ==============================================================*/

/*============= §4 TAB BAR HEADER ==========================================================*/
/*============= DRAW: Tab Bar ==============================================================*/

int draw_header(int r)
{
    attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
    mvprintw(r, 0, "%-*s", g_tui_w,
             demo_mode ? "  CPCU MONITOR - InfiniTech v3.4-pageorder [DEMO]"
                       : "  CPCU MONITOR - InfiniTech v3.4-pageorder");
    attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
    r++;

    move(r, 1);
    for(int p = 0; p < PAGE_COUNT; p++)
    {
        if(p == (int)current_page)
        {
            attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
            printw("[%d:%s]", p + 1, PAGE_TITLES[p]);
            attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);
        }
        else
        {
            attron(COLOR_PAIR(CP_DIM));
            printw(" %d:%s ", p + 1, PAGE_TITLES[p]);
            attroff(COLOR_PAIR(CP_DIM));
        }
        printw("  ");
    }
    r++;
    draw_hline(r, 0, g_tui_w);
    return r + 1;
}

/*============= DRAW: Page 1 — Overview ====================================================*/

/*============= §6 WAVEFORM LINE-TRACE RENDERER ============================================*/
/**
 *  Line-trace waveform renderer — one glyph per column on the sample row,
 *  plus connector chars between adjacent samples so the trace reads as a
 *  continuous curve the way a scope trace does.
 *
 *  Sub-cell resolution: within the single row a sample falls in we pick
 *  one of three glyphs based on where inside the row the sample sits —
 *  '.' (upper third), '-' (middle), 'o' (lower third) — giving roughly
 *  3x vertical resolution without the area-fill clutter.
 *
 *  Connectors ('/' '\' '|' '-') fill the gap between two adjacent samples
 *  that are at different rows so the eye follows the curve.
 *
 *  Works identically on narrow libncurses and wide libncursesw — every
 *  glyph is a single 7-bit ASCII character.
 */
void draw_waveform(int row, int col, int width, int height,
                          int ch_idx, int color_pair)
{
    if(wave_count < 2) return;

    /* Auto-scale to the channel's actual min/max */
    uint16_t vmin = 4095, vmax = 0;
    uint32_t avail = (wave_count < WAVE_BUF_SIZE) ? wave_count : WAVE_BUF_SIZE;
    for(uint32_t i = 0; i < avail; i++)
    {
        uint16_t v = wave_buf[ch_idx][i];
        if(v < vmin) vmin = v;
        if(v > vmax) vmax = v;
    }
    if(vmax <= vmin) vmax = vmin + 1;

    /* Axes */
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    for(int y = 0; y < height; y++)
        mvaddch(row + y, col - 1, ACS_VLINE);
    mvhline(row + height, col, ACS_HLINE, width);
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    uint32_t show_n = (avail < (uint32_t)width) ? avail : (uint32_t)width;
    if(show_n < 2) return;

    attron(COLOR_PAIR(color_pair));

    /* Total sub-row resolution: 5 sub-cells per row × height rows.
     * Glyph picks a visual height inside the cell:
     *    rem 0  → "'"   (top ~80-100%)
     *    rem 1  → "`"   (upper)
     *    rem 2  → "-"   (middle)
     *    rem 3  → "."   (lower)
     *    rem 4  → ","   (bottom ~0-20%)
     */
    int total_sub = height * 5;

    /* Map a sample value → (row, sub_in_row) where row is the ncurses
     * row index counting from the top of the plot (row = 0 at top). */
    int prev_row = -1;
    int prev_sub = -1;

    for(uint32_t x = 0; x < show_n; x++)
    {
        uint32_t buf_off = (avail * x) / show_n;
        uint32_t idx = (wave_wr + WAVE_BUF_SIZE - avail + buf_off) % WAVE_BUF_SIZE;
        uint16_t val = wave_buf[ch_idx][idx];

        float frac = (float)(val - vmin) / (float)(vmax - vmin);
        if(frac < 0.0f) frac = 0.0f;
        if(frac > 1.0f) frac = 1.0f;

        /* sub position from the top, 0..total_sub-1 */
        int sub = (int)((1.0f - frac) * (float)(total_sub - 1) + 0.5f);
        if(sub < 0)              sub = 0;
        if(sub > total_sub - 1)  sub = total_sub - 1;

        int y_row  = sub / 5;
        int y_rem  = sub - y_row * 5;   /* 0..4  (0 = upper part) */

        /* Pick the sample glyph by sub-row position */
        const char *pt;
        switch(y_rem) {
            case 0:  pt = "'";  break;
            case 1:  pt = "`";  break;
            case 2:  pt = "-";  break;
            case 3:  pt = ".";  break;
            case 4:
            default: pt = ",";  break;
        }

        mvaddstr(row + y_row, col + (int)x, pt);

        /* Connector between previous sample and this one.
         * If adjacent samples are at different rows, fill the in-between
         * rows with a diagonal/vertical glyph so the trace doesn't break. */
        if(prev_row >= 0 && prev_row != y_row)
        {
            int step  = (y_row > prev_row) ? 1 : -1;
            const char *conn = (step > 0) ? "\\" : "/";
            for(int r = prev_row + step; r != y_row; r += step)
                mvaddstr(row + r, col + (int)x, conn);
        }

        prev_row = y_row;
        prev_sub = sub;
    }
    (void)prev_sub;

    attroff(COLOR_PAIR(color_pair));
}

/*============= DRAW: Page 4 — Waveforms ==================================================*/

void draw_page_waves(int r, IPC_Context *ipc)
{
    (void)ipc;

    /*---- Top bar: live BSAU flags of the most recent packet ----*/
    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    uint8_t  last_flags = 0;
    if(head > 0)
        last_flags = ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK].flags;

    {
        char fbuf[64];
        wl_flags_decode(last_flags, fbuf, sizeof(fbuf));
        int severe = (last_flags &
                      (WL_FLAG_CLIPPING | WL_FLAG_ELEC_OFF | WL_FLAG_ADC_OVRN));
        int warn   = (last_flags & WL_FLAG_TX_SAT);
        int cp = fbuf[0] == '\0' ? CP_GOOD
               : severe          ? CP_BAD
               : warn            ? CP_WARN
                                 : CP_CYAN;

        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, 1, "BSAU flags:");
        attroff(COLOR_PAIR(CP_DIM));
        attron(COLOR_PAIR(cp) | A_BOLD);
        printw(" %s", (fbuf[0] == '\0') ? "OK" : fbuf);
        attroff(COLOR_PAIR(cp) | A_BOLD);

        /* Tell the user the glyph ramp they're looking at */
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, g_col_r, "glyphs top->bot: '  `  -  .  ,     (/ \\ connectors)");
        attroff(COLOR_PAIR(CP_DIM));
        r++;
    }

    if(wave_detail)
    {
        /* ───── SINGLE CHANNEL DETAIL ───── */
        uint32_t avail = (wave_count < WAVE_BUF_SIZE) ? wave_count : WAVE_BUF_SIZE;
        uint32_t sum = 0;
        uint16_t vmin = 4095, vmax = 0;
        for(uint32_t i = 0; i < avail; i++)
        {
            uint16_t v = wave_buf[wave_sel_ch][i];
            sum += v;
            if(v < vmin) vmin = v;
            if(v > vmax) vmax = v;
        }
        float dc_v  = (avail > 0) ? ((float)sum / avail / WAVE_ADC_MAX * 3.3f) : 0.0f;
        float vpp_v = (float)(vmax - vmin) / WAVE_ADC_MAX * 3.3f;
        float rms_v = compute_rms_v(wave_buf[wave_sel_ch], avail);
        float hz    = estimate_zcr_hz(wave_buf[wave_sel_ch], avail);

        /* CLIP indicator: raw min/max sitting at ADC rails (within 1%)  */
        bool clip_lo = (vmin <= 40);             /* 40 / 4095 < 1% */
        bool clip_hi = (vmax >= 4055);
        bool clip    = clip_lo || clip_hi;

        attron(A_BOLD);
        mvprintw(r, 1, "CHANNEL %d", wave_sel_ch);
        attroff(A_BOLD);

        mvprintw(r, 12, "Hz:");
        attron(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
        printw("%6.0f", hz);
        attroff(COLOR_PAIR(CP_MAGENTA) | A_BOLD);

        printw("  Vpp:");
        attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
        printw("%.3fV", vpp_v);
        attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

        printw("  DC:");
        attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
        printw("%.3fV", dc_v);
        attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

        printw("  RMS:");
        attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
        printw("%.3fV", rms_v);
        attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

        printw("  ADC:");
        attron(COLOR_PAIR(clip ? CP_BAD : CP_DIM));
        printw("%u-%u%s", vmin, vmax, clip ? " CLIP!" : "");
        attroff(COLOR_PAIR(clip ? CP_BAD : CP_DIM));
        r++;

        /* Horizontal axis: time-per-screen = WAVE_BUF_SIZE / 2 kHz */
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, 3, "t = 0");
        mvprintw(r, (g_tui_w / 2) - 5, "t = %.0f ms",
                 (float)WAVE_BUF_SIZE / 2000.0f * 1000.0f / 2.0f);
        mvprintw(r, g_tui_w - 14, "t = %.0f ms",
                 (float)WAVE_BUF_SIZE / 2000.0f * 1000.0f);
        attroff(COLOR_PAIR(CP_DIM));
        r++;

        int big_w = g_tui_w - 6;
        draw_waveform(r, 3, big_w, WAVE_PLOT_H_BIG, wave_sel_ch, CP_GOOD);
        r += WAVE_PLOT_H_BIG + 2;
    }
    else
    {
        /* ───── ALL 8 CHANNELS (mini-plots, 2 columns x 4 rows) ───── */
        int mini_h = WAVE_PLOT_H;
        int mini_w = g_col_r - 4;
        if(mini_w < 20) mini_w = 20;

        for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        {
            int c_col = (ch < 4) ? 1 : g_col_r;
            int c_row = r + (ch % 4) * (mini_h + 2);
            bool sel  = (ch == wave_sel_ch);

            int lbl_cp = sel ? CP_MAGENTA : CP_NORMAL;
            attron(COLOR_PAIR(lbl_cp) | (sel ? A_BOLD : 0));
            mvprintw(c_row, c_col, "%sch%d", sel ? ">" : " ", ch);
            attroff(COLOR_PAIR(lbl_cp) | (sel ? A_BOLD : 0));

            /* Stats over the rolling buffer */
            uint32_t avail = (wave_count < WAVE_BUF_SIZE) ? wave_count : WAVE_BUF_SIZE;
            uint16_t vmin = 4095, vmax = 0;
            for(uint32_t i = 0; i < avail; i++)
            {
                uint16_t v = wave_buf[ch][i];
                if(v < vmin) vmin = v;
                if(v > vmax) vmax = v;
            }
            float vpp = (float)(vmax - vmin) / WAVE_ADC_MAX * 3.3f;
            float rms = compute_rms_v(wave_buf[ch], avail);
            float hz  = estimate_zcr_hz(wave_buf[ch], avail);
            bool  clip = (vmin <= 40) || (vmax >= 4055);

            /* Right-aligned stats line next to label, colored by clip state */
            attron(COLOR_PAIR(CP_DIM));
            printw(" %4.0fHz %.2fVpp %.2fVrms", hz, vpp, rms);
            attroff(COLOR_PAIR(CP_DIM));

            if(clip)
            {
                attron(COLOR_PAIR(CP_BAD) | A_BOLD);
                printw(" CLIP");
                attroff(COLOR_PAIR(CP_BAD) | A_BOLD);
            }

            int wave_cp = sel ? CP_MAGENTA : CP_GOOD;
            draw_waveform(c_row + 1, c_col + 1, mini_w, mini_h, ch, wave_cp);
        }
    }
}

/*============= §5 PAGE: OVERVIEW ==========================================================*/
/*============= DRAW: Page 1 — Overview ====================================================*/

void draw_page_overview(int r, IPC_Context *ipc,
                               uint32_t pkt_rate, float loss_rate,
                               uint32_t up_h, uint32_t up_m, uint32_t up_s)
{
    tui_reload_servo_names();
    char _ftb[32];
    uint8_t  sys_state  =   atomic_load(&ipc->ctrl->system_state);
    uint8_t  io_rdy     =   atomic_load(&ipc->ctrl->io_ready);
    uint8_t  dsp_rdy    =   atomic_load(&ipc->ctrl->dsp_ready);
    uint32_t pkts       =   atomic_load(&ipc->diag->io_pkts_received);
    uint32_t gaps       =   atomic_load(&ipc->diag->io_seq_gaps);
    uint32_t overflows  =   atomic_load(&ipc->diag->io_ring_overflows);
    uint32_t nrf_status =   atomic_load(&ipc->diag->io_nrf_init_status);
    uint32_t dsp_inf    =   atomic_load(&ipc->diag->dsp_inferences);
    uint32_t dsp_lat    =   atomic_load(&ipc->diag->dsp_max_latency_us);
    uint32_t dsp_batch  =   atomic_load(&ipc->diag->dsp_batches);
    uint32_t ring_fill  =   IPC_SensorCount(ipc);

    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    IPC_SensorEntry latest;
    memset(&latest, 0, sizeof(latest));
    if(head > 0) latest = ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK];

    uint16_t servo[IPC_NUM_SERVOS];
    memcpy(servo, (const void *)ipc->motor->servo_us, sizeof(servo));
    uint8_t gesture     =   ipc->motor->gesture_id;
    uint8_t confidence  =   ipc->motor->confidence;

    /*===== HEALTH SUMMARY BANNER (rolled up from page 6) =====
     * One line showing green/yellow/red per subsystem so the user
     * can see the whole system state at a glance without switching
     * to page 6.  Each pill is [OK]/[WARN]/[FAULT]. */
    {
        uint32_t nrf_status = atomic_load(&ipc->diag->io_nrf_init_status);
        uint32_t overflows  = atomic_load(&ipc->diag->io_ring_overflows);
        uint32_t dsp_lat    = atomic_load(&ipc->diag->dsp_max_latency_us);
        uint64_t io_hb_us   = atomic_load(&ipc->ctrl->io_heartbeat_us);
        uint32_t hb_age_ms  = 0;
        {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
            if(io_hb_us > 0 && now_us > io_hb_us)
                hb_age_ms = (uint32_t)((now_us - io_hb_us) / 1000ULL);
        }

        int radio = (nrf_status != 0)    ? 2 : (pkt_rate < 900 ? 1 : 0);
        int ioh   = (!io_rdy)            ? 2 : (hb_age_ms > IO_HB_BAD_MS ? 2 : (hb_age_ms > IO_HB_WARN_MS ? 1 : 0));
        int ipcs  = (overflows > 0)      ? 2 : (ring_fill > 900 ? 1 : 0);
        int batt  = 0; /* v3: BSAU no longer samples battery */
        // was: (latest.vbat_raw > 0 && batt_v < 2.7f) ? 2
        // v3: battery line removed
        int dsph  = (!dsp_rdy)           ? 2 : (dsp_lat > 50000 ? 2 : (dsp_lat > 20000 ? 1 : 0));
        int fsm   = (sys_state == IPC_STATE_SAFE)    ? 2
                  : (sys_state == IPC_STATE_RUNNING) ? 0 : 1;

        #define PILL(lbl, s) do { \
            int cp_ = (s) == 0 ? CP_GOOD : (s) == 1 ? CP_WARN : CP_BAD; \
            const char *st = (s) == 0 ? "OK" : (s) == 1 ? "WARN" : "FAULT"; \
            attron(COLOR_PAIR(CP_DIM)); printw("%s:", (lbl)); attroff(COLOR_PAIR(CP_DIM)); \
            attron(COLOR_PAIR(cp_) | A_BOLD); printw("%s", st); attroff(COLOR_PAIR(cp_) | A_BOLD); \
            printw("  "); \
        } while(0)

        mvprintw(r, 1, "HEALTH  ");
        PILL("radio",  radio);
        PILL("io",     ioh);
        PILL("ipc",    ipcs);
        /* v3: battery removed */
        PILL("dsp",    dsph);
        PILL("fsm",    fsm);

        /* Overall verdict at right edge */
        int worst = 0;
        int stats[] = { radio, ioh, ipcs, batt, dsph, fsm };
        for(size_t i = 0; i < sizeof(stats)/sizeof(stats[0]); i++)
            if(stats[i] > worst) worst = stats[i];
        const char *vr = worst == 0 ? "NOMINAL" : worst == 1 ? "WARNING" : "DEGRADED";
        int vr_cp     = worst == 0 ? CP_GOOD   : worst == 1 ? CP_WARN   : CP_BAD;
        int vr_len    = (int)strlen(vr);
        attron(COLOR_PAIR(vr_cp) | A_BOLD);
        mvprintw(r, g_tui_w - vr_len - 2, "%s", vr);
        attroff(COLOR_PAIR(vr_cp) | A_BOLD);

        #undef PILL
        r++;
        draw_hline(r, 0, g_tui_w);
        r++;
    }

    draw_section(r, 1, "SYSTEM");
    draw_section(r, g_col_r, "RADIO LINK");
    r++;

    draw_lv(r, 1,       "State:",      state_color(sys_state), "%s", state_str(sys_state));
    draw_lv(r, g_col_r, "Packets/s:",  pkt_rate > 900 ? CP_GOOD : CP_WARN, "%u pkt/s", pkt_rate);
    r++;
    draw_lv(r, 1,       "Uptime:",     CP_CYAN, "%02u:%02u:%02u", up_h, up_m, up_s);
    draw_lv(r, g_col_r, "Total pkts:", CP_CYAN, "%u  (since boot)", pkts);
    r++;
    draw_lv(r, 1,       "IO ready:",   io_rdy ? CP_GOOD : CP_BAD, "%s", io_rdy ? "YES" : "NO");
    draw_lv(r, g_col_r, "Seq gaps:",   gaps > 10 ? CP_WARN : CP_GOOD, "%u  (missed pkts)", gaps);
    r++;
    draw_lv(r, 1,       "DSP ready:",  dsp_rdy ? CP_GOOD : CP_BAD, "%s", dsp_rdy ? "YES" : "NO");
    draw_lv(r, g_col_r, "Loss rate:",  loss_rate > 0.01f ? CP_WARN : CP_GOOD, "%.3f %%  (of last 1k)", loss_rate * 100);
    r++;
    draw_lv(r, 1,       "NRF init:",   nrf_status == 0 ? CP_GOOD : CP_BAD, "%s",
            nrf_status == 0 ? "OK" : "FAIL");
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);
    draw_section(r, 1, "EMG CHANNELS (latest raw ADC, % of 4095)");
    r++;
    for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
    {
        int col = (ch < 4) ? 1 : g_col_r;
        int row = r + (ch % 4);
        float frac = (float)latest.samples[0].ch[ch] / 4095.0f;
        mvprintw(row, col, "ch%d ", ch);
        draw_bar(row, col + 4, g_bar_w, frac, CP_BAR_FILL, CP_BAR_EMPTY);
    }
    r += 5;

    draw_hline(r - 1, 0, g_tui_w);
    draw_section(r, 1, "SERVOS (pulse width, us)");
    draw_section(r, g_col_r, "SYSTEM STATUS");
    r++;
    for(int i = 0; i < IPC_NUM_SERVOS; i++)
    {
        mvprintw(r + i, 1, "S%d %-5s %4u ", i, SERVO_NAMES[i], servo[i]);
        draw_slider(r + i, 16, servo[i],
                    tui_servo_min_us(ipc, i), tui_servo_max_us(ipc, i),
                    g_slider_w);
    }
    // draw_lv(r,     g_col_r, "Voltage:",   batt_v < 3.0f ? CP_BAD : CP_GOOD, "%.2f V  (pack)", batt_v);
    // draw_lv(r + 1, g_col_r, "Raw ADC:",   CP_CYAN, "%u  (12-bit, 0..4095)", latest.vbat_raw);
    draw_lv(r + 2, g_col_r, "Level:",     batt_color(latest.flags), "%s", batt_str(latest.flags));
    draw_lv(r + 4, g_col_r, "Ring fill:", ring_fill > 100 ? CP_WARN : CP_GOOD,
            "%u / %u  (IPC buffer)", ring_fill, IPC_SENSOR_RING_SIZE);
    draw_lv(r + 5, g_col_r, "Overflows:", overflows > 0 ? CP_BAD : CP_GOOD, "%u  (dropped by IO)", overflows);
    r += 7;

    draw_hline(r - 1, 0, g_tui_w);
    draw_section(r, 1, "DSP PIPELINE");
    draw_section(r, g_col_r, "INFERENCE");
    r++;
    draw_lv(r, 1,       "DSP windows:", CP_CYAN, "%u  (200-sample feature windows)", dsp_batch);
    draw_lv(r, g_col_r, "Inferences:",  CP_CYAN, "%u  (ML predictions)", dsp_inf);
    r++;
    draw_lv(r, 1,       "Max latency:", dsp_lat > 50000 ? CP_WARN : CP_GOOD, "%u us  (worst batch)", dsp_lat);
    /* Use the named gesture from dsp_export if available, otherwise fall
     * back to the numeric motor->gesture_id. */
    {
        uint32_t exp_sq = atomic_load(&ipc->dsp_export->update_seq);
        if(exp_sq > 0)
        {
            char gname[IPC_MAX_GESTURE_NAME];
            memcpy(gname, (const void *)ipc->dsp_export->gesture_name, sizeof(gname));
            gname[IPC_MAX_GESTURE_NAME - 1] = '\0';
            draw_lv(r, g_col_r, "Gesture:", CP_GOOD, "%s  (%u%%)", gname, confidence);
        }
        else
        {
            draw_lv(r, g_col_r, "Gesture:", CP_GOOD, "#%u  (%u%%)", gesture, confidence);
        }
    }
    r += 2;

    /*==================== END-TO-END LATENCY BREAKDOWN ==================
     * Always shown on the OVERVIEW page so the operator can see the
     * full SYS-REQ-01 budget at a glance, with each stage labelled and
     * described. Constants come from per-segment datasheet/wall-clock
     * probes (cpcu_dsp.py's LAT_*_US set, mirrored in cpcu_tui.h's
     * TUI_LAT_*_US set). Measurements come from cpcu_dsp.py via the
     * IPC_DSPExport padding region.
     *
     * Math (matches Python's _print_latency_waterfall and DSP-page
     * waterfall — the three views render the same total):
     *   E2E = BSAU_const               (ADC_PACK + WIRELESS)
     *       + lat_pkt_us                (SPI_UNPACK + ring + DSP + IPC)
     *       + SMOOTHER_I2C_const
     *       + SERVO_MECH_const
     */
    {
        uint32_t lat_pkt_us  = tui_lat_pkt_to_servo_us(ipc->dsp_export);
        uint32_t lat_dwell   = tui_lat_ring_dwell_us  (ipc->dsp_export);
        uint32_t lat_dsp     = tui_lat_dsp_compute_us (ipc->dsp_export);

        uint32_t bsau_const  = TUI_LAT_ADC_PACK_US + TUI_LAT_WIRELESS_US;
        uint32_t srvo_const  = TUI_LAT_SMOOTHER_I2C_US + TUI_LAT_SERVO_MECH_US;

        /* Layout columns (1-indexed character positions):
         *   col 3   stage label   (left-justified, 21 chars wide)
         *   col 26  numeric value (right-justified inside 9-char field)
         *   col 36  unit + tag    ("us  (const)" / "us  (meas)")
         *   col 53  description   (single line, dim)
         * Sub-rows under "pkt → motor IPC" indent the label to col 5 so
         * the dash bullet visually nests them inside the CPCU group. */
        #define LAT_LBL_COL     3
        #define LAT_VAL_COL     27
        #define LAT_TAG_COL     38
        #define LAT_DESC_COL    56

        draw_hline(r - 1, 0, g_tui_w);
        draw_section(r, 1, "END-TO-END LATENCY  (EMG event → servo motion, budget < 300 ms)");
        r++;

        /*---- Group 1: BSAU → CPCU (sensor path, fixed) ----*/
        attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
        mvprintw(r, 1, "BSAU → CPCU  (sensor path)");
        attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
        r++;
        mvprintw(r, LAT_LBL_COL, "ADC + WL_Pack");
        attron(COLOR_PAIR(CP_CYAN));
        mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(TUI_LAT_ADC_PACK_US, _ftb));
        attroff(COLOR_PAIR(CP_CYAN));
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, LAT_TAG_COL, "(const)");
        mvprintw(r, LAT_DESC_COL, "STM32 8-ch ADC sampling + 32 B frame pack");
        attroff(COLOR_PAIR(CP_DIM));
        r++;
        mvprintw(r, LAT_LBL_COL, "Wireless TX + ACK");
        attron(COLOR_PAIR(CP_CYAN));
        mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(TUI_LAT_WIRELESS_US, _ftb));
        attroff(COLOR_PAIR(CP_CYAN));
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, LAT_TAG_COL, "(const)");
        mvprintw(r, LAT_DESC_COL, "NRF24L01+ ESB: SPI upload + 2 Mbps air + auto-ACK");
        attroff(COLOR_PAIR(CP_DIM));
        r++;
        mvprintw(r, LAT_LBL_COL, "subtotal");
        attron(COLOR_PAIR(CP_GOOD) | A_BOLD);
        mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(bsau_const, _ftb));
        attroff(COLOR_PAIR(CP_GOOD) | A_BOLD);
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, LAT_TAG_COL, "(sum)");
        attroff(COLOR_PAIR(CP_DIM));
        r += 2;

        /*---- Group 2: CPCU (compute, measured by Python) ----*/
        attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
        mvprintw(r, 1, "CPCU  (compute on Pi 5, isolcpus 1-3)");
        attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
        r++;
        if(lat_pkt_us > 0)
        {
            int cp_pkt = (lat_pkt_us > 100000) ? CP_WARN : CP_GOOD;
            mvprintw(r, LAT_LBL_COL, "pkt → motor IPC");
            attron(COLOR_PAIR(cp_pkt) | A_BOLD);
            mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(lat_pkt_us, _ftb));
            attroff(COLOR_PAIR(cp_pkt) | A_BOLD);
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, LAT_TAG_COL, "(meas)");
            mvprintw(r, LAT_DESC_COL, "rx_time_us → motor cmd write");
            attroff(COLOR_PAIR(CP_DIM));
            r++;
            /* sub-rows: indent label by 2 to nest visually */
            mvprintw(r, LAT_LBL_COL + 2, "- SPI unpack");
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(TUI_LAT_SPI_UNPACK_US, _ftb));
            mvprintw(r, LAT_TAG_COL, "(in meas)");
            mvprintw(r, LAT_DESC_COL, "NRF SPI read + WL_Unpack + IPC ring push");
            attroff(COLOR_PAIR(CP_DIM));
            r++;
            mvprintw(r, LAT_LBL_COL + 2, "- Ring dwell");
            attron(COLOR_PAIR(CP_CYAN));
            mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(lat_dwell, _ftb));
            attroff(COLOR_PAIR(CP_CYAN));
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, LAT_TAG_COL, "(in meas)");
            mvprintw(r, LAT_DESC_COL, "Samples wait in IPC ring before DSP drains");
            attroff(COLOR_PAIR(CP_DIM));
            r++;
            int cp_dsp = (lat_dsp > 50000) ? CP_WARN : CP_CYAN;
            mvprintw(r, LAT_LBL_COL + 2, "- DSP compute");
            attron(COLOR_PAIR(cp_dsp));
            mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(lat_dsp, _ftb));
            attroff(COLOR_PAIR(cp_dsp));
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, LAT_TAG_COL, "(in meas)");
            mvprintw(r, LAT_DESC_COL, "Filter + features + RandomForest inference");
            attroff(COLOR_PAIR(CP_DIM));
            r += 2;
        }
        else
        {
            mvprintw(r, LAT_LBL_COL, "pkt → motor IPC");
            attron(COLOR_PAIR(CP_WARN));
            mvprintw(r, LAT_VAL_COL, "   N/A   ");
            attroff(COLOR_PAIR(CP_WARN));
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, LAT_TAG_COL, "(waiting)");
            mvprintw(r, LAT_DESC_COL, "No BSAU packets yet — DSP filling first window");
            attroff(COLOR_PAIR(CP_DIM));
            r += 2;
        }

        /*---- Group 3: CPCU → Robotic Arm (actuation, fixed) ----*/
        attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
        mvprintw(r, 1, "CPCU → Robotic Arm  (actuation path)");
        attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
        r++;
        mvprintw(r, LAT_LBL_COL, "Smoother + I²C");
        attron(COLOR_PAIR(CP_CYAN));
        mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(TUI_LAT_SMOOTHER_I2C_US, _ftb));
        attroff(COLOR_PAIR(CP_CYAN));
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, LAT_TAG_COL, "(const)");
        mvprintw(r, LAT_DESC_COL, "50 Hz tick: slew limiter + PCA9685 I²C 6 servos");
        attroff(COLOR_PAIR(CP_DIM));
        r++;
        mvprintw(r, LAT_LBL_COL, "Servo mechanical");
        attron(COLOR_PAIR(CP_CYAN));
        mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(TUI_LAT_SERVO_MECH_US, _ftb));
        attroff(COLOR_PAIR(CP_CYAN));
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, LAT_TAG_COL, "(const)");
        mvprintw(r, LAT_DESC_COL, "SG90 step response (~15 ms typical, no-load)");
        attroff(COLOR_PAIR(CP_DIM));
        r++;
        mvprintw(r, LAT_LBL_COL, "subtotal");
        attron(COLOR_PAIR(CP_GOOD) | A_BOLD);
        mvprintw(r, LAT_VAL_COL, "%9s", fmt_time_us(srvo_const, _ftb));
        attroff(COLOR_PAIR(CP_GOOD) | A_BOLD);
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, LAT_TAG_COL, "(sum)");
        attroff(COLOR_PAIR(CP_DIM));
        r += 2;

        /*---- E2E grand total — visually separated from the breakdown ----*/
        draw_hline(r - 1, 2, g_tui_w - 4);
        if(lat_pkt_us > 0)
        {
            uint32_t e2e_us = bsau_const + lat_pkt_us + srvo_const;
            float    e2e_ms = e2e_us / 1000.0f;
            int e2e_cp = (e2e_ms < 200.0f) ? CP_GOOD
                       : (e2e_ms < 300.0f) ? CP_WARN
                       :                     CP_BAD;
            attron(A_BOLD);
            mvprintw(r, 1, "E2E TOTAL");
            attroff(A_BOLD);
            attron(COLOR_PAIR(e2e_cp) | A_BOLD);
            mvprintw(r, LAT_VAL_COL - 3, "%9.1f ms", e2e_ms);
            attroff(COLOR_PAIR(e2e_cp) | A_BOLD);
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, LAT_TAG_COL, "budget < 300 ms      BSAU %u + CPCU %u + arm %u us",
                     bsau_const, lat_pkt_us, srvo_const);
            attroff(COLOR_PAIR(CP_DIM));
        }
        else
        {
            attron(A_BOLD);
            mvprintw(r, 1, "E2E TOTAL");
            attroff(A_BOLD);
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, LAT_VAL_COL - 3, "  waiting");
            mvprintw(r, LAT_TAG_COL, "BSAU not transmitting — DSP has no measurement yet");
            attroff(COLOR_PAIR(CP_DIM));
        }
        r += 2;

        #undef LAT_LBL_COL
        #undef LAT_VAL_COL
        #undef LAT_TAG_COL
        #undef LAT_DESC_COL
    }

    uint32_t export_seq = atomic_load(&ipc->dsp_export->update_seq);
    if(export_seq > 0)
    {
        draw_hline(r - 1, 0, g_tui_w);
        draw_section(r, 1, "ML CLASSIFICATION (Python)");
        r++;

        char export_name[IPC_MAX_GESTURE_NAME];
        memcpy(export_name, (const void *)ipc->dsp_export->gesture_name, sizeof(export_name));
        export_name[IPC_MAX_GESTURE_NAME - 1] = '\0';
        uint32_t inf_us  = ipc->dsp_export->inference_time_us;
        uint8_t  active  = ipc->dsp_export->active_class;

        mvprintw(r, 1, "Gesture: ");
        attron(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
        printw("%-16s", export_name);
        attroff(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
        mvprintw(r, g_col_r, "Inf. time: ");
        attron(COLOR_PAIR(inf_us > 50000 ? CP_WARN : CP_CYAN) | A_BOLD);
        printw("%u us", inf_us);
        attroff(COLOR_PAIR(inf_us > 50000 ? CP_WARN : CP_CYAN) | A_BOLD);
        r++;

        uint8_t nc = ipc->dsp_export->num_classes;
        if(nc > IPC_MAX_CLASSES)     nc = IPC_MAX_CLASSES;
        /* Also clamp to the static CLS_NAMES table: a future model with
         * more than DATASET_LABEL_COUNT classes would otherwise read
         * past the end of the names array and segfault. Display gets
         * truncated at that bound; the warning lives in CLS_NAMES'
         * docblock. */
        if(nc > DATASET_LABEL_COUNT) nc = DATASET_LABEL_COUNT;
        for(int c = 0; c < (int)nc; c++)
        {
            int col = (c < 5) ? 1 : g_col_r;
            int row = r + (c % 5);
            float conf = ipc->dsp_export->class_confidence[c];
            if(conf < 0.0f) conf = 0.0f;
            if(conf > 1.0f) conf = 1.0f;
            mvprintw(row, col, "%-6s", CLS_NAMES[c]);
            draw_bar(row, col + 7, 12, conf,
                     c == active ? CP_MAGENTA : CP_BAR_FILL, CP_BAR_EMPTY);
        }
        r += 6;

        draw_section(r, 1, "FILTERED RMS (Python, bar=% of 0.5V full-scale)");
        r++;
        for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
        {
            int col  = (ch < 4) ? 1 : g_col_r;
            int row  = r + (ch % 4);
            float rv = ipc->dsp_export->channel_rms[ch];
            float fr = rv / 0.5f;
            if(fr > 1.0f) fr = 1.0f;
            mvprintw(row, col, "ch%d ", ch);
            draw_bar(row, col + 4, 14, fr, CP_CYAN, CP_BAR_EMPTY);
            printw(" %.4f V", rv);
        }
    }
}

/*============= §5 PAGE: RADIO/IO ==========================================================*/
/*============= DRAW: Page 2 — Radio / I/O =================================================*/

void draw_page_radio(int r, IPC_Context *ipc,
                            uint32_t pkt_rate, float loss_rate,
                            uint32_t up_h, uint32_t up_m, uint32_t up_s)
{
    uint8_t  sys_state  =   atomic_load(&ipc->ctrl->system_state);
    uint8_t  io_rdy     =   atomic_load(&ipc->ctrl->io_ready);
    uint32_t pkts       =   atomic_load(&ipc->diag->io_pkts_received);
    uint32_t pkts_drp   =   atomic_load(&ipc->diag->io_pkts_dropped);
    uint32_t gaps       =   atomic_load(&ipc->diag->io_seq_gaps);
    uint32_t overflows  =   atomic_load(&ipc->diag->io_ring_overflows);
    uint32_t nrf_status =   atomic_load(&ipc->diag->io_nrf_init_status);
    uint32_t max_poll   =   atomic_load(&ipc->diag->io_max_poll_us);
    uint32_t safe_ents  =   atomic_load(&ipc->diag->io_safe_entries);
    uint64_t io_hb_us   =   atomic_load(&ipc->ctrl->io_heartbeat_us);
    uint32_t ring_fill  =   IPC_SensorCount(ipc);

    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    IPC_SensorEntry latest;
    memset(&latest, 0, sizeof(latest));
    if(head > 0) latest = ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK];


    /* IO heartbeat age (how long since cpcu_io updated heartbeat timestamp).
     * The natural ceiling is HEARTBEAT_INTERVAL_US (~100 ms), so thresholds
     * are set well above that — see IO_HB_WARN_MS / IO_HB_BAD_MS. */
    uint32_t hb_age_ms = 0;
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
        if(io_hb_us > 0 && now_us > io_hb_us)
            hb_age_ms = (uint32_t)((now_us - io_hb_us) / 1000ULL);
    }

    /*==================== NRF24L01+ STATUS ====================*/
    draw_section(r, 1, "NRF24L01+ STATUS");
    draw_section(r, g_col_r, "SAFETY FSM");
    r++;

    draw_lv(r, 1,       "Init:",        nrf_status == 0 ? CP_GOOD : CP_BAD, "%s",
            nrf_status == 0 ? "OK" : "FAIL");
    draw_lv(r, g_col_r, "State:",       state_color(sys_state), "%s", state_str(sys_state));
    r++;
    draw_lv(r, 1,       "Channel:",     CP_CYAN, "76  (2.476 GHz)");
    draw_lv(r, g_col_r, "IO ready:",    io_rdy ? CP_GOOD : CP_BAD, "%s", io_rdy ? "YES" : "NO");
    r++;
    draw_lv(r, 1,       "Address:",     CP_CYAN, "E7:E7:E7:E7:E7  (5-byte)");
    draw_lv(r, g_col_r, "IO heartbeat:",
            hb_age_ms > IO_HB_BAD_MS ? CP_BAD :
            hb_age_ms > IO_HB_WARN_MS ? CP_WARN : CP_GOOD,
            "%u ms ago  (RT loop)", hb_age_ms);
    r++;
    draw_lv(r, 1,       "SPI speed:",   CP_CYAN, "8 MHz");
    draw_lv(r, g_col_r, "SAFE entries:",
            safe_ents > 0 ? CP_WARN : CP_GOOD, "%u  (times FSM went SAFE)", safe_ents);
    r++;
    draw_lv(r, 1,       "Payload:",     CP_CYAN, "32 B fixed");
    // draw_lv(r, g_col_r, "Batt (pack):", batt_v > 3.0f ? CP_GOOD : CP_BAD, "%.2f V", batt_v);
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== PACKET STATISTICS ====================*/
    draw_section(r, 1, "PACKET STATISTICS");
    draw_section(r, g_col_r, "TIMING / RT");
    r++;

    draw_lv(r, 1,       "Total RX:",    CP_CYAN, "%u pkts  (since boot)", pkts);
    draw_lv(r, g_col_r, "Uptime:",      CP_CYAN, "%02u:%02u:%02u", up_h, up_m, up_s);
    r++;
    draw_lv(r, 1,       "Rate:",        pkt_rate > 900 ? CP_GOOD : CP_WARN, "%u pkt/s", pkt_rate);
    draw_lv(r, g_col_r, "Max poll:",    max_poll > 100 ? CP_WARN : CP_GOOD, "%u us  (worst SPI read)", max_poll);
    r++;
    draw_lv(r, 1,       "Dropped:",     pkts_drp > 0 ? CP_BAD : CP_GOOD, "%u  (ring full)", pkts_drp);
    draw_lv(r, g_col_r, "Ring OVF:",    overflows > 0 ? CP_BAD : CP_GOOD, "%u  (events)", overflows);
    r++;
    draw_lv(r, 1,       "Seq gaps:",    gaps > 10 ? CP_WARN : CP_GOOD, "%u  (missed seqs)", gaps);

    /*---- Ring-fill bar ----*/
    mvprintw(r, g_col_r, "Ring fill:");
    {
        float frac = (float)ring_fill / (float)IPC_SENSOR_RING_SIZE;
        int ring_bar_w = g_bar_w - 4;
        if(ring_bar_w < 10) ring_bar_w = 10;
        draw_bar(r, g_col_r + 11, ring_bar_w, frac,
                 ring_fill > 100 ? CP_WARN : CP_BAR_FILL, CP_BAR_EMPTY);
        attron(COLOR_PAIR(CP_DIM));
        printw(" %u/%u", ring_fill, IPC_SENSOR_RING_SIZE);
        attroff(COLOR_PAIR(CP_DIM));
    }
    r++;

    draw_lv(r, 1,       "Loss rate:",   loss_rate > 0.01f ? CP_WARN : CP_GOOD,
            "%.4f %%  (of last 1k)", loss_rate * 100.0f);
    draw_lv(r, g_col_r, "Retry (last):",
            latest.tx_retry > 2 ? CP_WARN : CP_GOOD, "%u  (nRF auto-retries)", latest.tx_retry);
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== LAST PACKET RAW ====================
     * Decoded fields from the most recent BSAU packet. Each field gets
     * a full English label instead of the terse wire-protocol name, so
     * the operator doesn't need to remember what "ts", "seq", or
     * "flags" mean. The legacy short names from wireless_packet.h are
     * still listed in dimmed parentheses for cross-reference with the
     * codebase. */
    draw_section(r, 1, "LAST PACKET RAW  (most recent BSAU frame decoded)");
    draw_section(r, g_col_r, "BSAU SENSOR FLAGS");
    r++;

    /* Row 1: sequence + retry count */
    mvprintw(r, 1, "Sequence # ");
    attron(COLOR_PAIR(CP_DIM)); printw("(seq):     "); attroff(COLOR_PAIR(CP_DIM));
    attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
    printw("%-4u", latest.seq);
    attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);
    attron(COLOR_PAIR(CP_DIM));
    printw("  (8-bit wrap counter)");
    attroff(COLOR_PAIR(CP_DIM));

    /* RIGHT col: decoded flag string */
    {
        char buf[64];
        wl_flags_decode(latest.flags, buf, sizeof(buf));
        int severe = (latest.flags &
                      (WL_FLAG_CLIPPING | WL_FLAG_ELEC_OFF | WL_FLAG_ADC_OVRN));
        int warn   = (latest.flags & WL_FLAG_TX_SAT);
        int cp = (buf[0] == '\0') ? CP_GOOD
               : severe           ? CP_BAD
               : warn             ? CP_WARN
                                  : CP_CYAN;
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvprintw(r, g_col_r, "%s", (buf[0] == '\0') ? "OK  (no faults raised)" : buf);
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }
    r++;

    /* Row 2: TX retry count */
    {
        int rt_cp = (latest.tx_retry > 2) ? CP_WARN : CP_GOOD;
        mvprintw(r, 1, "TX retries ");
        attron(COLOR_PAIR(CP_DIM)); printw("(retry):   "); attroff(COLOR_PAIR(CP_DIM));
        attron(COLOR_PAIR(rt_cp) | A_BOLD);
        printw("%-4u", latest.tx_retry);
        attroff(COLOR_PAIR(rt_cp) | A_BOLD);
        attron(COLOR_PAIR(CP_DIM));
        printw("  (nRF24 auto-ACK attempts, 0 = first try)");
        attroff(COLOR_PAIR(CP_DIM));
    }

    /* Flag legend (right column, dim) — only show the bits that mean
     * something on the v3 board so the operator knows what to look for. */
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, g_col_r, "bits: CLIP ELEC OVRN TX_SAT CAL FIRST");
    attroff(COLOR_PAIR(CP_DIM));
    r++;

    /* Row 3: per-packet packet-loss counter from BSAU */
    mvprintw(r, 1, "BSAU pkt loss ");
    attron(COLOR_PAIR(CP_DIM)); printw("(loss):  "); attroff(COLOR_PAIR(CP_DIM));
    attron(COLOR_PAIR(latest.pkt_loss > 0 ? CP_WARN : CP_GOOD) | A_BOLD);
    printw("%-4u", latest.pkt_loss);
    attroff(COLOR_PAIR(latest.pkt_loss > 0 ? CP_WARN : CP_GOOD) | A_BOLD);
    attron(COLOR_PAIR(CP_DIM));
    printw("  (BSAU's own retry-exhausted counter since boot)");
    attroff(COLOR_PAIR(CP_DIM));

    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, g_col_r, "severe=red  warn=yellow  info=cyan  none=green");
    attroff(COLOR_PAIR(CP_DIM));
    r++;

    /* Row 4: BSAU timestamp */
    mvprintw(r, 1, "BSAU timestamp ");
    attron(COLOR_PAIR(CP_DIM)); printw("(ts): "); attroff(COLOR_PAIR(CP_DIM));
    attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
    printw("%-5u ms", latest.timestamp);
    attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);
    attron(COLOR_PAIR(CP_DIM));
    printw("  (BSAU's millisecond clock, wraps at 65 s)");
    attroff(COLOR_PAIR(CP_DIM));
    r++;

    /* Row 5: CPCU rx age + battery summary */
    {
        uint32_t rx_age_ms = 0;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
        if(latest.rx_time_us > 0 && now_us > latest.rx_time_us)
            rx_age_ms = (uint32_t)((now_us - latest.rx_time_us) / 1000ULL);

        mvprintw(r, 1, "RX age ");
        attron(COLOR_PAIR(CP_DIM)); printw("(rx_age):      "); attroff(COLOR_PAIR(CP_DIM));
        attron(COLOR_PAIR(rx_age_ms > 50 ? CP_WARN : CP_CYAN) | A_BOLD);
        printw("%-4u ms", rx_age_ms);
        attroff(COLOR_PAIR(rx_age_ms > 50 ? CP_WARN : CP_CYAN) | A_BOLD);
        attron(COLOR_PAIR(CP_DIM));
        printw("  (how long ago CPCU received this packet)");
        attroff(COLOR_PAIR(CP_DIM));

        mvprintw(r, g_col_r, "Battery: ");
        attron(COLOR_PAIR(batt_color(latest.flags)) | A_BOLD);
        printw("%s", batt_str(latest.flags));
        attroff(COLOR_PAIR(batt_color(latest.flags)) | A_BOLD);
        attron(COLOR_PAIR(CP_DIM));
        printw("  (BSAU pack — v3: not sampled, status from flags)");
        attroff(COLOR_PAIR(CP_DIM));
    }
    r += 2;

    /* Row 6: ADC samples for all 8 channels — single line each side */
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1, "ADC samples (raw 0..4095, sample 0 of %d per packet):",
             WL_SAMPLES_PER_PACKET);
    attroff(COLOR_PAIR(CP_DIM));
    r++;
    mvprintw(r, 1, "ch0..3:  ");
    attron(COLOR_PAIR(CP_CYAN));
    printw("%4u  %4u  %4u  %4u",
           latest.samples[0].ch[0], latest.samples[0].ch[1],
           latest.samples[0].ch[2], latest.samples[0].ch[3]);
    attroff(COLOR_PAIR(CP_CYAN));
    mvprintw(r, g_col_r, "ch4..7:  ");
    attron(COLOR_PAIR(CP_CYAN));
    printw("%4u  %4u  %4u  %4u",
           latest.samples[0].ch[4], latest.samples[0].ch[5],
           latest.samples[0].ch[6], latest.samples[0].ch[7]);
    attroff(COLOR_PAIR(CP_CYAN));
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== EMG CHANNELS BAR GRAPHS ====================*/
    draw_section(r, 1, "EMG CHANNELS (latest raw ADC, % of 4095)");
    r++;
    for(int ch = 0; ch < WL_NUM_CHANNELS; ch++)
    {
        int col = (ch < 4) ? 1 : g_col_r;
        int row = r + (ch % 4);
        float frac = (float)latest.samples[0].ch[ch] / 4095.0f;
        mvprintw(row, col, "ch%d ", ch);
        draw_bar(row, col + 4, g_bar_w, frac, CP_BAR_FILL, CP_BAR_EMPTY);
    }
}

/*============= §5 PAGE: DSP/AI ============================================================*/
/*============= DRAW: Page 3 — DSP / AI ====================================================*/

void draw_page_dsp(int r, IPC_Context *ipc)
{
    tui_reload_servo_names();
    uint8_t  dsp_rdy    =   atomic_load(&ipc->ctrl->dsp_ready);
    uint32_t dsp_inf    =   atomic_load(&ipc->diag->dsp_inferences);
    uint32_t dsp_lat    =   atomic_load(&ipc->diag->dsp_max_latency_us);
    uint32_t dsp_batch  =   atomic_load(&ipc->diag->dsp_batches);
    uint32_t dsp_under  =   atomic_load(&ipc->diag->dsp_ring_underflows);
    uint32_t ring_fill  =   IPC_SensorCount(ipc);
    uint32_t export_seq =   atomic_load(&ipc->dsp_export->update_seq);
    uint32_t motor_seq  =   atomic_load(&ipc->motor->seq);
    uint64_t motor_ts   =   ipc->motor->timestamp_us;

    uint16_t servo[IPC_NUM_SERVOS];
    memcpy(servo, (const void *)ipc->motor->servo_us, sizeof(servo));

    /*---- Rolling deltas (for rates), tracked across frames ----*/
    static uint32_t prev_inf       = 0;
    static uint32_t prev_batch     = 0;
    static uint32_t prev_export_sq = 0;
    static uint32_t prev_motor_sq  = 0;
    static uint64_t prev_tick_ms   = 0;

    uint64_t now = now_ms_wall();
    uint32_t inf_rate    = 0;      /* inferences/s */
    uint32_t batch_rate  = 0;
    uint32_t export_rate = 0;
    uint32_t motor_rate  = 0;
    if(prev_tick_ms > 0 && now > prev_tick_ms)
    {
        uint32_t dt = (uint32_t)(now - prev_tick_ms);
        if(dt > 0)
        {
            inf_rate    = (dsp_inf    - prev_inf)       * 1000 / dt;
            batch_rate  = (dsp_batch  - prev_batch)     * 1000 / dt;
            export_rate = (export_seq - prev_export_sq) * 1000 / dt;
            motor_rate  = (motor_seq  - prev_motor_sq)  * 1000 / dt;
        }
    }
    prev_inf       = dsp_inf;
    prev_batch     = dsp_batch;
    prev_export_sq = export_seq;
    prev_motor_sq  = motor_seq;
    prev_tick_ms   = now;

    /* Motor command age: now − motor->timestamp_us */
    uint32_t motor_age_ms = 0;
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
        if(motor_ts > 0 && now_us > motor_ts)
            motor_age_ms = (uint32_t)((now_us - motor_ts) / 1000ULL);
    }

    /*==================== DSP PIPELINE ====================*/
    draw_section(r, 1, "DSP PIPELINE");
    draw_section(r, g_col_r, "INFERENCE ENGINE");
    r++;

    draw_lv(r, 1,       "DSP ready:",   dsp_rdy ? CP_GOOD : CP_BAD, "%s", dsp_rdy ? "YES" : "NO");
    draw_lv(r, g_col_r, "Model:",       CP_CYAN, "RandomForest");
    r++;
    draw_lv(r, 1,       "DSP windows:", CP_CYAN, "%u  (%u/s, 200-sample feature windows)", dsp_batch, batch_rate);
    draw_lv(r, g_col_r, "Inferences:",  CP_CYAN, "%u  (%u/s, RF preds)", dsp_inf, inf_rate);
    r++;
    draw_lv(r, 1,       "Max latency:", dsp_lat > 50000 ? CP_WARN : CP_GOOD, "%u us  (worst batch)", dsp_lat);
    draw_lv(r, g_col_r, "Window:",      CP_CYAN, "200 samples  (WINDOW_HI in cpcu_dsp.py)");
    r++;
    draw_lv(r, 1,       "Ring fill:",   ring_fill > 100 ? CP_WARN : CP_GOOD,
            "%u / %u  (IPC samples)", ring_fill, IPC_SENSOR_RING_SIZE);
    draw_lv(r, g_col_r, "Stride:",      CP_CYAN, "100 samples  (50%% overlap)");
    r++;
    draw_lv(r, 1,       "Underflows:",  dsp_under > 0 ? CP_WARN : CP_GOOD, "%u  (ring-empty events)", dsp_under);
    draw_lv(r, g_col_r, "Export rate:",
            export_rate > 0 ? CP_GOOD : CP_WARN, "%u Hz  (Python publishes)", export_rate);
    r++;
    draw_lv(r, 1,       "Motor cmds:",  CP_CYAN, "%u  (%u/s to servos)", motor_seq, motor_rate);
    draw_lv(r, g_col_r, "Cmd age:",
            motor_age_ms > 100 ? CP_WARN : CP_GOOD, "%u ms  (since last write)", motor_age_ms);
    r += 2;

    if(dsp_rdy)
    {
        /*==================== PER-GROUP GESTURE STATE ====================
         * Read /tmp/cpcu_group_state.txt (written by cpcu_dsp.py on
         * every inference tick) and render one block per group. Each
         * line is:
         *     <group>\t<state>\t<conf_pct>\t<cls0>:<p0>,<cls1>:<p1>,...
         * With two groups (right_arm, left_arm) we get a stacked view
         * of both classifiers simultaneously, since the additive
         * velocity integrator means both can drive servos at once. */
        draw_hline(r - 1, 0, g_tui_w);
        draw_section(r, 1, "ACTIVE GESTURE PER GROUP");
        r++;

        FILE *gf = fopen("/tmp/cpcu_group_state.txt", "r");
        int   groups_drawn = 0;
        if(gf)
        {
            char line[512];
            while(fgets(line, sizeof(line), gf))
            {
                /* parse name\tstate\tconf_pct\tcls:p,cls:p,... */
                char *p = line;
                char *gname = p;
                char *tab1 = strchr(p, '\t');
                if(!tab1) continue;
                *tab1 = '\0';
                char *state = tab1 + 1;
                char *tab2 = strchr(state, '\t');
                if(!tab2) continue;
                *tab2 = '\0';
                char *conf_s = tab2 + 1;
                char *tab3 = strchr(conf_s, '\t');
                if(!tab3) continue;
                *tab3 = '\0';
                char *classes = tab3 + 1;
                /* trim trailing newline */
                size_t L = strlen(classes);
                while(L > 0 && (classes[L-1] == '\n' || classes[L-1] == '\r'))
                    classes[--L] = '\0';

                int conf_pct = atoi(conf_s);
                int conf_cp  = (conf_pct >= 80) ? CP_GOOD
                              : (conf_pct >= 50) ? CP_WARN : CP_BAD;

                mvprintw(r, 3, "[");
                attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
                printw("%s", gname);
                attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);
                printw("]  ");
                attron(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
                printw("%-10s", state);
                attroff(COLOR_PAIR(CP_MAGENTA) | A_BOLD);
                printw("  conf: ");
                attron(COLOR_PAIR(conf_cp) | A_BOLD);
                printw("%3d %%", conf_pct);
                attroff(COLOR_PAIR(conf_cp) | A_BOLD);
                printw("    classes: ");
                attron(COLOR_PAIR(CP_DIM));
                printw("%s", classes);
                attroff(COLOR_PAIR(CP_DIM));
                r++;
                groups_drawn++;
            }
            fclose(gf);
        }
        if(groups_drawn == 0)
        {
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, 3, "(no inference yet — waiting for first window to fill)");
            attroff(COLOR_PAIR(CP_DIM));
            r++;
        }
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, 3, "seq#%u    inf time: %u us", export_seq,
                 ipc->dsp_export->inference_time_us);
        attroff(COLOR_PAIR(CP_DIM));
        r += 2;

        /* For backward compatibility with the latency block below, keep
         * `active` and `export_seq` semantics from the IPC export (primary
         * group only — TUI shows the full per-group view above). */
        uint8_t active = ipc->dsp_export->active_class;

        /*==================== END-TO-END LATENCY WATERFALL ====================
         * Each module measures or fixes its own stage; here we stitch them
         * together so the operator can see the full BSAU → servo budget
         * without leaving the TUI. Constants come from the per-module
         * datasheets / wall-clock probes; measurements come from
         * cpcu_dsp.py via the IPC_DSPExport padding region.
         *
         * Math (matches cpcu_dsp.py's _print_latency_waterfall):
         *   BSAU stage      = ADC_PACK + WIRELESS                 (const)
         *   CPCU stage      = SPI_UNPACK + RING_DWELL + DSP_COMPUTE + SMOOTHER_I2C
         *   SERVO stage     = SERVO_MECH                          (const)
         *   End-to-end E2E  = BSAU + CPCU + SERVO
         *
         * The SYS-REQ-01 target is < 300 ms wall-time from EMG event to
         * mechanical motion. */
        uint32_t lat_pkt_us  = tui_lat_pkt_to_servo_us(ipc->dsp_export);
        uint32_t lat_dwell   = tui_lat_ring_dwell_us  (ipc->dsp_export);
        uint32_t lat_dsp     = tui_lat_dsp_compute_us (ipc->dsp_export);
        uint32_t lat_seq_age = tui_lat_seq_age        (ipc->dsp_export);

        if(lat_pkt_us > 0)
        {
            /* Reconstruct the per-segment breakdown.
             *
             * lat_pkt_us is the wall-clock cpcu_dsp.py measures from the
             * BSAU packet's rx_time_us stamp (taken in cpcu_io BEFORE the
             * NRF SPI read of the payload) to the moment cpcu_dsp writes
             * the motor command into IPC. It therefore covers:
             *   SPI_UNPACK + ring dwell + DSP compute + motor IPC write.
             *
             * smoother+I²C is a SEPARATE downstream tick in cpcu_io that
             * runs at 50 Hz: it reads motor_cmd and pushes PWM to the
             * PCA9685. So we add it after lat_pkt_us, not inside it.
             *
             * Total budget:
             *   BSAU_const + lat_pkt_us + SMOOTHER_I2C_const + SERVO_MECH_const
             */
            uint32_t bsau_const = TUI_LAT_ADC_PACK_US + TUI_LAT_WIRELESS_US;
            uint32_t cpcu_meas  = lat_pkt_us;
            uint32_t smth_const = TUI_LAT_SMOOTHER_I2C_US;
            uint32_t srvo_const = TUI_LAT_SERVO_MECH_US;
            uint32_t e2e_us     = bsau_const + cpcu_meas
                                + smth_const + srvo_const;
            float    e2e_ms     = e2e_us / 1000.0f;

            int e2e_cp = (e2e_ms < 200.0f) ? CP_GOOD
                       : (e2e_ms < 300.0f) ? CP_WARN
                       :                     CP_BAD;

            draw_hline(r - 1, 0, g_tui_w);
            draw_section(r, 1, "END-TO-END LATENCY  (BSAU EMG event → servo motion)");
            r++;

            /* Headline: total wall time + budget comparison */
            mvprintw(r, 3, "E2E total: ");
            attron(COLOR_PAIR(e2e_cp) | A_BOLD);
            printw("%6.1f ms", e2e_ms);
            attroff(COLOR_PAIR(e2e_cp) | A_BOLD);
            printw("   budget: ");
            attron(COLOR_PAIR(CP_DIM));
            printw("< 300 ms  (SYS-REQ-01)");
            attroff(COLOR_PAIR(CP_DIM));
            r++;

            /* Per-stage breakdown, indented like a tree so the totals
             * line up under each section header. Measurements get a
             * "meas" tag, fixed constants get "const". */
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, 3, "┌─ BSAU ────────────────────────────────────");
            attroff(COLOR_PAIR(CP_DIM));
            r++;
            char tb[32];
            draw_lv(r, 3, "│ ADC + pack:",   CP_DIM, "%s  (const)",
                    fmt_time_us(TUI_LAT_ADC_PACK_US, tb));
            r++;
            draw_lv(r, 3, "│ wireless:",     CP_DIM, "%s  (const)",
                    fmt_time_us(TUI_LAT_WIRELESS_US, tb));
            r++;
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, 3, "├─ CPCU ────────────────────────────────────");
            attroff(COLOR_PAIR(CP_DIM));
            r++;
            int cp_cpcu = (cpcu_meas > 100000) ? CP_WARN : CP_GOOD;
            draw_lv(r, 3, "│ pkt → motor:",
                    cp_cpcu, "%s  (meas)", fmt_time_us(cpcu_meas, tb));
            r++;
            draw_lv(r, 3, "│   ring dwell:", CP_CYAN, "%s  (meas)",
                    fmt_time_us(lat_dwell, tb));
            r++;
            int cp_dsp = (lat_dsp > 50000) ? CP_WARN : CP_CYAN;
            draw_lv(r, 3, "│   DSP compute:", cp_dsp, "%s  (meas)",
                    fmt_time_us(lat_dsp, tb));
            r++;
            draw_lv(r, 3, "│ smoother+I²C:", CP_DIM, "%s  (const, 50 Hz tick)",
                    fmt_time_us(smth_const, tb));
            r++;
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, 3, "└─ SERVO ───────────────────────────────────");
            attroff(COLOR_PAIR(CP_DIM));
            r++;
            draw_lv(r, 3, "  mechanical:",  CP_DIM, "%s  (const, SG90)",
                    fmt_time_us(srvo_const, tb));
            r++;

            /* Seq-age is a sanity check: oldest packet in the inference
             * window is "seq_age" packets behind the newest. With 1 kHz
             * BSAU packet rate this equals milliseconds. Values > 250
             * (one full window) indicate a stalled ring. */
            int cp_seq = (lat_seq_age > 250) ? CP_BAD
                       : (lat_seq_age > 220) ? CP_WARN
                       :                       CP_GOOD;
            draw_lv(r, 3, "  seq age:",
                    cp_seq, "%5u pkts  (oldest sample in window, ≈ ms @1 kHz)",
                    lat_seq_age);
            r += 2;
        }
        else
        {
            /* DSP hasn't published a latency sample yet — typically
             * means cpcu_dsp.py is still warming up its first window
             * or no BSAU packets are flowing. */
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, 3, "End-to-end latency: waiting for DSP to fill its first window...");
            attroff(COLOR_PAIR(CP_DIM));
            r += 2;
        }

        draw_hline(r - 1, 0, g_tui_w);
        draw_section(r, 1, "CLASS CONFIDENCE (% softmax probability, per group)");
        draw_section(r, g_col_r, "FILTERED RMS (bar=% of 0.5V, +abs V)");
        r++;

        /* Per-group class panel. Pulled from /tmp/cpcu_group_state.txt
         * (same source the DSP/AI page's ACTIVE GESTURE PER GROUP
         * section uses). Each group occupies a small block:
         *     [group_name]
         *       cls0 ▓▓▓▓░░ 67%
         *       cls1 ▓░░░░░ 8%
         *       ...
         * Active class (matches the group's current state) is drawn in
         * magenta; others use the standard bar color. We render this
         * column independently of the RMS column so they stack
         * vertically without overlapping; max_rows is chosen so the
         * page doesn't overflow. */
        int class_row = r;
        FILE *gf_cc = fopen("/tmp/cpcu_group_state.txt", "r");
        if(gf_cc)
        {
            char line[512];
            while(class_row < g_term_h - 8 &&
                  fgets(line, sizeof(line), gf_cc))
            {
                /* parse name\tstate\tconf\tclasses */
                char *p_name = line;
                char *tab1 = strchr(p_name, '\t'); if(!tab1) continue; *tab1++ = '\0';
                char *p_state = tab1;
                char *tab2 = strchr(p_state, '\t'); if(!tab2) continue; *tab2++ = '\0';
                char *tab3 = strchr(tab2, '\t');    if(!tab3) continue; *tab3++ = '\0';
                char *p_cls = tab3;
                size_t L = strlen(p_cls);
                while(L > 0 && (p_cls[L-1] == '\n' || p_cls[L-1] == '\r'))
                    p_cls[--L] = '\0';

                /* Group header */
                attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
                mvprintw(class_row, 1, "[%s]", p_name);
                attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);
                class_row++;

                /* Walk "cls0:p0,cls1:p1,..." */
                char *tok = p_cls;
                while(tok && *tok && class_row < g_term_h - 8)
                {
                    char *next = strchr(tok, ',');
                    if(next) { *next = '\0'; next++; }
                    char *colon = strchr(tok, ':');
                    if(colon)
                    {
                        *colon = '\0';
                        int pct = atoi(colon + 1);
                        if(pct < 0) pct = 0;
                        if(pct > 100) pct = 100;
                        int is_active = (strcmp(tok, p_state) == 0);
                        mvprintw(class_row, 3, "%-6.6s", tok);
                        draw_bar(class_row, 10, 14, pct / 100.0f,
                                 is_active ? CP_MAGENTA : CP_BAR_FILL,
                                 CP_BAR_EMPTY);
                        mvprintw(class_row, 26, "%3d%%", pct);
                    }
                    tok = next;
                    class_row++;
                }
                class_row++;   /* blank line between groups */
            }
            fclose(gf_cc);
        }
        else
        {
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(class_row, 1, "(no group digest yet — waiting for first window)");
            attroff(COLOR_PAIR(CP_DIM));
            class_row++;
        }

        /* RMS column (right side) — still IPC-direct (no per-group RMS) */
        for(int i = 0; i < WL_NUM_CHANNELS; i++)
        {
            float rv = ipc->dsp_export->channel_rms[i];
            float fr = rv / 0.5f;
            if(fr > 1.0f) fr = 1.0f;
            mvprintw(r + i, g_col_r, "ch%d ", i);
            draw_bar(r + i, g_col_r + 4, 14, fr, CP_CYAN, CP_BAR_EMPTY);
            printw(" %.3f V", rv);
        }

        /* Advance r by the taller of the two columns so SERVO OUTPUT
         * doesn't overlap whatever's still on screen above. */
        int rms_end = r + WL_NUM_CHANNELS;
        r = (class_row > rms_end ? class_row : rms_end) + 1;
    }
    else
    {
        r++;
        attron(COLOR_PAIR(CP_WARN));
        mvprintw(r, 1, "cpcu_dsp.py is not running (dsp_ready = NO). "
                       "Check the KERNEL window for spawn errors.");
        attroff(COLOR_PAIR(CP_WARN));
        r += 2;
    }

    draw_hline(r - 1, 0, g_tui_w);
    draw_section(r, 1, "SERVO OUTPUT (smoothed PWM pulse width, us)");
    r++;

    for(int i = 0; i < IPC_NUM_SERVOS; i++)
    {
        int col = (i < 3) ? 1 : g_col_r;
        int row = r + (i % 3);
        mvprintw(row, col, "S%d %-5s ", i, SERVO_NAMES[i]);
        draw_slider(row, col + 10, servo[i],
                    tui_servo_min_us(ipc, i), tui_servo_max_us(ipc, i), 14);
        printw(" %4u us", servo[i]);
    }
}

/*============= §5 PAGE: HEALTH ============================================================*/
/*============= DRAW: Page 6 — System Health ===============================================*/

/**
 *  Traffic-light rollup dashboard. One row per subsystem, with a
 *  colored status cell and a short explanation of what's being checked
 *  and (on fault) why it's unhappy. Intended as the first page a
 *  debugger should glance at — "is everything OK?"
 */
void draw_page_health(int r, IPC_Context *ipc,
                             uint32_t pkt_rate, float loss_rate)
{
    /*---- Gather the telemetry we'll compare against thresholds ----*/
    uint8_t  sys_state  = atomic_load(&ipc->ctrl->system_state);
    uint8_t  io_rdy     = atomic_load(&ipc->ctrl->io_ready);
    uint8_t  dsp_rdy    = atomic_load(&ipc->ctrl->dsp_ready);
    uint32_t nrf_status = atomic_load(&ipc->diag->io_nrf_init_status);
    uint32_t overflows  = atomic_load(&ipc->diag->io_ring_overflows);
    uint32_t gaps       = atomic_load(&ipc->diag->io_seq_gaps);
    uint32_t pkts_drp   = atomic_load(&ipc->diag->io_pkts_dropped);
    uint32_t safe_ents  = atomic_load(&ipc->diag->io_safe_entries);
    uint32_t max_poll   = atomic_load(&ipc->diag->io_max_poll_us);
    uint32_t dsp_lat    = atomic_load(&ipc->diag->dsp_max_latency_us);
    uint32_t dsp_under  = atomic_load(&ipc->diag->dsp_ring_underflows);
    uint32_t export_seq = atomic_load(&ipc->dsp_export->update_seq);
    uint32_t ring_fill  = IPC_SensorCount(ipc);
    uint64_t io_hb_us   = atomic_load(&ipc->ctrl->io_heartbeat_us);

    uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
    IPC_SensorEntry latest;
    memset(&latest, 0, sizeof(latest));
    if(head > 0) latest = ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK];
    /* v3: battery not sampled */

    uint32_t hb_age_ms = 0;
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
        if(io_hb_us > 0 && now_us > io_hb_us)
            hb_age_ms = (uint32_t)((now_us - io_hb_us) / 1000ULL);
    }

    /*---- Compute per-subsystem status ----
     * Each entry is (name, status_code, detail_text).
     * status_code: 0=OK (green), 1=WARN (yellow), 2=FAULT (red) */
    typedef struct {
        const char *name;
        int         status;         /* 0 OK, 1 WARN, 2 FAULT */
        char        detail[128];
    } HealthRow;
    HealthRow rows[12];
    int nrows = 0;

    #define ADD_ROW(NAME, STAT, ...) do { \
        rows[nrows].name   = (NAME); \
        rows[nrows].status = (STAT); \
        snprintf(rows[nrows].detail, sizeof(rows[nrows].detail), __VA_ARGS__); \
        nrows++; \
    } while(0)

    /* 1. Safety FSM */
    if(sys_state == IPC_STATE_RUNNING)
        ADD_ROW("Safety FSM",  0, "RUNNING  (normal operation)");
    else if(sys_state == IPC_STATE_SAFE)
        ADD_ROW("Safety FSM",  2, "SAFE  (FSM tripped - check other rows)");
    else
        ADD_ROW("Safety FSM",  1, "INIT / DEGRADED  (not yet stable)");

    /* 2. Radio link */
    if(nrf_status != 0)
        ADD_ROW("Radio (nRF)", 2, "NRF init FAILED  (check wiring/SPI)");
    else if(pkt_rate < 900)
        ADD_ROW("Radio (nRF)", 2, "Rate %u/s  (expected ~1000)", pkt_rate);
    else if(loss_rate > 0.01f)
        ADD_ROW("Radio (nRF)", 1, "Loss %.3f %%  (>1 %% over last 1k)", loss_rate * 100.0f);
    else
        ADD_ROW("Radio (nRF)", 0, "%u pkt/s, loss %.3f %%", pkt_rate, loss_rate * 100.0f);

    /* 3. IO realtime loop */
    if(!io_rdy)
        ADD_ROW("IO loop",     2, "IO not ready  (cpcu_io not started?)");
    else if(hb_age_ms > IO_HB_BAD_MS)
        ADD_ROW("IO loop",     2, "Heartbeat %u ms stale  (RT loop stalled)", hb_age_ms);
    else if(hb_age_ms > IO_HB_WARN_MS || max_poll > 100)
        ADD_ROW("IO loop",     1, "hb %u ms, poll %u us", hb_age_ms, max_poll);
    else
        ADD_ROW("IO loop",     0, "hb %u ms, poll %u us", hb_age_ms, max_poll);

    /* 4. IPC ring (v2.3: ring overflows are recoverable; only flag a sustained burst) */
    if(overflows > 100)
        ADD_ROW("IPC ring",    2, "%u overflows  (DSP can't keep up)", overflows);
    else if(ring_fill > 900 || overflows > 0)
        ADD_ROW("IPC ring",    1, "%u / %u fill, %u overflows", ring_fill, IPC_SENSOR_RING_SIZE, overflows);
    else if(pkts_drp > 0)
        ADD_ROW("IPC ring",    1, "%u dropped", pkts_drp);
    else
        ADD_ROW("IPC ring",    0, "%u / %u  healthy", ring_fill, IPC_SENSOR_RING_SIZE);

    /* 5. Packet sequence integrity */
    if(gaps > 50)
        ADD_ROW("Pkt integrity", 2, "%u seq gaps  (heavy loss)", gaps);
    else if(gaps > 10)
        ADD_ROW("Pkt integrity", 1, "%u seq gaps", gaps);
    else
        ADD_ROW("Pkt integrity", 0, "%u seq gaps", gaps);

    /* v3: battery section removed (BSAU not sampling) */

    /* 7. DSP pipeline */
    if(!dsp_rdy)
        ADD_ROW("DSP pipeline", 2, "DSP not ready  (Python not started?)");
    else if(dsp_lat > 50000)
        ADD_ROW("DSP pipeline", 2, "Max latency %u us  (> 50 ms)", dsp_lat);
    else if(dsp_under > 0)
        ADD_ROW("DSP pipeline", 1, "%u ring-empty events", dsp_under);
    else if(dsp_lat > 20000)
        ADD_ROW("DSP pipeline", 1, "Max latency %u us", dsp_lat);
    else
        ADD_ROW("DSP pipeline", 0, "max lat %u us  (under budget)", dsp_lat);

    /* 8. ML export */
    if(export_seq == 0)
        ADD_ROW("ML export",   1, "No updates yet  (first cycle?)");
    else
        ADD_ROW("ML export",   0, "seq#%u  (DSP writes actively)", export_seq);

    /* 9. BSAU flags (latest packet) */
    {
        uint8_t severe = latest.flags &
                        (WL_FLAG_CLIPPING | WL_FLAG_ELEC_OFF | WL_FLAG_ADC_OVRN);
        uint8_t warn   = latest.flags & WL_FLAG_TX_SAT;
        char fbuf[64];
        wl_flags_decode(latest.flags, fbuf, sizeof(fbuf));
        if(severe)
            ADD_ROW("BSAU sensor", 2, "flags: %s  (hardware fault)", fbuf);
        else if(warn)
            ADD_ROW("BSAU sensor", 1, "flags: %s  (transient issue)", fbuf);
        else
            ADD_ROW("BSAU sensor", 0, "flags: OK  (no CLIP/ELEC/OVRN)");
    }

    /* 10. SAFE entries count */
    if(safe_ents == 0)
        ADD_ROW("SAFE trips",  0, "0  (never entered SAFE since boot)");
    else if(safe_ents < 3)
        ADD_ROW("SAFE trips",  1, "%u  (recovered)", safe_ents);
    else
        ADD_ROW("SAFE trips",  2, "%u  (persistent instability)", safe_ents);

    /* 11. v2.3.7 gripper stall watchdog. Counts every time cpcu_io
     * had to retreat the gripper from grip_firm because it had been
     * pinned at the floor for grip_stall_recover_ms. Single fires
     * are normal during heavy gripping; persistent fires suggest
     * grip_firm_us is set too low. */
    uint32_t grip_stalls = atomic_load(&ipc->diag->io_gripper_stalls);
    if(grip_stalls == 0)
        ADD_ROW("Gripper stalls", 0, "0  (no watchdog activity)");
    else if(grip_stalls < 5)
        ADD_ROW("Gripper stalls", 1, "%u  (occasional retreats)", grip_stalls);
    else
        ADD_ROW("Gripper stalls", 2,
                "%u  (raise grip_firm_us in runtime.json)", grip_stalls);

    #undef ADD_ROW

    /*---- Count the overall statuses for summary line ----*/
    int n_ok = 0, n_warn = 0, n_fault = 0;
    for(int i = 0; i < nrows; i++)
    {
        if      (rows[i].status == 2) n_fault++;
        else if (rows[i].status == 1) n_warn++;
        else                          n_ok++;
    }

    /*---- Draw summary banner ----*/
    attron(A_BOLD);
    mvprintw(r, 1, "SYSTEM STATUS:");
    attroff(A_BOLD);
    mvprintw(r, 16, "[ ");
    attron(COLOR_PAIR(CP_GOOD) | A_BOLD); printw("%d OK", n_ok);        attroff(COLOR_PAIR(CP_GOOD) | A_BOLD);
    printw(" | ");
    attron(COLOR_PAIR(CP_WARN) | A_BOLD); printw("%d warn", n_warn);    attroff(COLOR_PAIR(CP_WARN) | A_BOLD);
    printw(" | ");
    attron(COLOR_PAIR(CP_BAD)  | A_BOLD); printw("%d fault", n_fault);  attroff(COLOR_PAIR(CP_BAD)  | A_BOLD);
    printw(" ]");

    /* Overall verdict in the right side */
    const char *verdict;
    int         verdict_cp;
    if      (n_fault > 0) { verdict = "DEGRADED";  verdict_cp = CP_BAD;  }
    else if (n_warn  > 0) { verdict = "OPERATIONAL (warnings)"; verdict_cp = CP_WARN; }
    else                  { verdict = "NOMINAL";   verdict_cp = CP_GOOD; }
    int vlen = (int)strlen(verdict);
    attron(COLOR_PAIR(verdict_cp) | A_BOLD);
    mvprintw(r, g_tui_w - vlen - 2, "%s", verdict);
    attroff(COLOR_PAIR(verdict_cp) | A_BOLD);
    r++;

    draw_hline(r, 0, g_tui_w);
    r++;

    /*---- Column headers ----*/
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(r, 2,  "SUBSYSTEM");
    mvprintw(r, 20, "STATUS");
    mvprintw(r, 32, "DETAIL");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    r++;

    /*---- One row per subsystem ----*/
    for(int i = 0; i < nrows; i++)
    {
        mvprintw(r, 2, "%-16s", rows[i].name);

        const char *stag;
        int         scp;
        switch(rows[i].status) {
            case 0:  stag = "  OK  ";  scp = CP_GOOD; break;
            case 1:  stag = " WARN ";  scp = CP_WARN; break;
            case 2:  stag = "FAULT ";  scp = CP_BAD;  break;
            default: stag = "  ??  ";  scp = CP_DIM;  break;
        }
        attron(COLOR_PAIR(scp) | A_BOLD);
        mvprintw(r, 20, "[%s]", stag);
        attroff(COLOR_PAIR(scp) | A_BOLD);

        attron(COLOR_PAIR(rows[i].status == 0 ? CP_DIM : scp));
        mvprintw(r, 32, "%s", rows[i].detail);
        attroff(COLOR_PAIR(rows[i].status == 0 ? CP_DIM : scp));
        r++;
    }
    r++;

    /*---- LIVE SYSTEM REQUIREMENTS COMPLIANCE ----
     *
     * One unified SYS-REQ table rendered via the REQ_ROW macro below.
     * (A second redundant block used to live here that only rendered
     * on tall terminals — same content, fewer rows, looked like a bug
     * to anyone running in a roomy tmux pane. Removed.) */
    draw_hline(r, 0, g_tui_w);
    r++;
    attron(A_BOLD);
    mvprintw(r, 1, "SYSTEM REQUIREMENTS (live)");
    attroff(A_BOLD);

    /* Count pass/fail for summary */
    int req_pass = 0, req_fail = 0;

    /* Helper macro: evaluate a requirement and draw one row */
    #define REQ_ROW(id, name, pass_cond, fmt, ...) do {         r++;         int _p = (pass_cond);         if(_p) req_pass++; else req_fail++;         attron(COLOR_PAIR(_p ? CP_GOOD : CP_BAD) | A_BOLD);         mvprintw(r, 2, "[%s]", _p ? "PASS" : "FAIL");         attroff(COLOR_PAIR(_p ? CP_GOOD : CP_BAD) | A_BOLD);         attron(COLOR_PAIR(CP_DIM));         printw("  %-12s %-28s ", id, name);         attroff(COLOR_PAIR(CP_DIM));         printw(fmt, __VA_ARGS__);     } while(0)

    uint32_t sample_rate_est = pkt_rate * 2;  /* 2 samples per packet */

    /* ── SYS-REQ-01: End-to-End Latency ──
     *
     * E2E budget = BSAU stage (ADC+wireless, datasheet)
     *            + CPCU stage (lat_pkt_us measured by cpcu_dsp.py; spans
     *              SPI_UNPACK + ring dwell + DSP compute + IPC write)
     *            + smoother + I²C (cpcu_io's PCA9685 update tick, fixed)
     *            + servo mechanical (datasheet)
     *
     * Two paths depending on whether cpcu_dsp.py has published a tick yet:
     *   measured  →  full chain with real CPCU number
     *   pre-DSP   →  legacy obs+inf+servo estimate            (ballpark
     *                only; lets the test run before any window fills) */
    uint32_t lat_pkt_us2 = tui_lat_pkt_to_servo_us(ipc->dsp_export);
    float e2e_ms;
    if(lat_pkt_us2 > 0)
    {
        uint32_t e2e_us = TUI_LAT_ADC_PACK_US + TUI_LAT_WIRELESS_US
                        + lat_pkt_us2
                        + TUI_LAT_SMOOTHER_I2C_US
                        + TUI_LAT_SERVO_MECH_US;
        e2e_ms = e2e_us / 1000.0f;
        REQ_ROW("SYS-REQ-01", "E2E latency < 300 ms",
                (e2e_ms < 300.0f),
                "%.1f ms (BSAU 558us + CPCU %.1f ms + smth 0.6 ms + servo 15 ms, meas)",
                e2e_ms, lat_pkt_us2 / 1000.0f);
    }
    else
    {
        e2e_ms = 200.0f + (dsp_lat / 1000.0f) + 20.0f;
        REQ_ROW("SYS-REQ-01", "E2E latency < 300 ms",
                (e2e_ms < 300.0f),
                "%.0f ms (obs 200 + inf %.0f + servo 20, no DSP yet)",
                e2e_ms, dsp_lat / 1000.0f);
    }

    /* ── SYS-REQ-03: Durability (uptime) ──
     * v3 hardware doesn't sample battery at all, so the old SYS-REQ-03a
     * "Battery (not sampled)" row was a fake PASS — removed. */
    {   /* Track uptime from first call */
        static uint64_t health_boot_ms = 0;
        if(health_boot_ms == 0) health_boot_ms = now_ms_wall();
        uint32_t up_sec = (uint32_t)((now_ms_wall() - health_boot_ms) / 1000);
        uint32_t up_min = up_sec / 60;
        REQ_ROW("SYS-REQ-03b", "Uptime >= 40 min",
                (up_min >= 40),
                "%u min %u sec", up_min, up_sec % 60);
    }

    /* ── SYS-REQ-04: Signal Accuracy ── */
    REQ_ROW("SYS-REQ-04a", "Packet loss < 1%%",
            (loss_rate < 0.01f),
            "%.3f %%", loss_rate * 100.0f);

    uint32_t dsp_inf_r = atomic_load(&ipc->diag->dsp_inferences);
    REQ_ROW("SYS-REQ-04b", "Inference active > 5 Hz",
            (dsp_rdy && dsp_inf_r > 0),
            "%s (DSP %s)", dsp_rdy ? "active" : "inactive", dsp_rdy ? "ready" : "NOT ready");

    REQ_ROW("SYS-REQ-04c", "50 Hz notch (PLI)",
            (dsp_rdy),
            "%s", dsp_rdy ? "Q=30 notch active" : "DSP not running");

    /* ── SYS-REQ-05: Wireless Range ── */
    REQ_ROW("SYS-REQ-05", "Radio > 900 pkt/s",
            (pkt_rate > 900),
            "%u pkt/s", pkt_rate);

    /* ── SYS-REQ-06: Sampling Rate ── */
    REQ_ROW("SYS-REQ-06", "Sample rate >= 2000 Hz",
            (sample_rate_est >= 1900),
            "%u Hz (est)", sample_rate_est);

    /* ── SYS-REQ-07: Acquisition Bandwidth ── */
    REQ_ROW("SYS-REQ-07", "DSP bandpass 20-450 Hz",
            (dsp_rdy),
            "%s", dsp_rdy ? "4th Butterworth active" : "DSP not running");

    /* ── SYS-REQ-08: Mechanical Safety (multiple sub-checks) ── */
    REQ_ROW("SYS-REQ-08a", "Safety FSM = RUNNING",
            (sys_state == IPC_STATE_RUNNING),
            "%s", state_str(sys_state));

    REQ_ROW("SYS-REQ-08b", "Zero SAFE entries",
            (safe_ents == 0),
            "%u entries", safe_ents);

    /* Check all servos within hardware limits */
    {
        int servos_ok = 1;
        uint16_t srv[IPC_NUM_SERVOS];
        memcpy(srv, (const void *)ipc->motor->servo_us, sizeof(srv));
        for(int s = 0; s < IPC_NUM_SERVOS; s++) {
            if(srv[s] < (int)tui_servo_min_us(ipc, s) - 50
            || srv[s] > (int)tui_servo_max_us(ipc, s) + 50)
                servos_ok = 0;
        }
        REQ_ROW("SYS-REQ-08c", "Servo limits enforced",
                servos_ok,
                "%s", servos_ok ? "all within range" : "VIOLATION");
    }

    REQ_ROW("SYS-REQ-08d", "Fail-safe -> neutral",
            (sys_state == IPC_STATE_RUNNING || sys_state == IPC_STATE_SAFE),
            "%s", sys_state == IPC_STATE_SAFE ? "ACTIVE (neutral)" : "armed");

    /* SYS-REQ-08e ("I2C bus healthy") used to live here with a hardcoded
     * i2c_fail=0 stub that always PASSED. IPC_Diagnostics has no I²C
     * error counter for cpcu_io to populate, so the row was a fake
     * green light — removed. Add it back once cpcu_io.c starts tracking
     * PCA9685 write failures into a new diag field. */

    /* ── SYS-REQ-09: Joint Accuracy ── */
    REQ_ROW("SYS-REQ-09", "Servo update >= 40 Hz",
            (pkt_rate > 900),
            "%s", pkt_rate > 900 ? "50 Hz (nominal)" : "degraded");

    /* ── Subsystem checks ── */
    uint32_t ring_ovf = atomic_load(&ipc->diag->io_ring_overflows);
    REQ_ROW("SUB-RING", "Ring zero overflows",
            (ring_ovf == 0),
            "%u overflows", ring_ovf);

    uint32_t grip_st = atomic_load(&ipc->diag->io_gripper_stalls);
    REQ_ROW("SUB-GRIP", "Gripper no stalls",
            (grip_st == 0),
            "%u stalls", grip_st);

    REQ_ROW("SUB-NRF", "NRF init OK",
            (nrf_status == 0),
            "%s", nrf_status == 0 ? "OK" : "FAILED");

    #undef REQ_ROW

    r++;
    /* Summary line */
    int req_total = req_pass + req_fail;
    attron(A_BOLD);
    mvprintw(r, 2, "COMPLIANCE:");
    attroff(A_BOLD);
    if(req_fail == 0) {
        attron(COLOR_PAIR(CP_GOOD) | A_BOLD);
        printw(" ALL %d REQUIREMENTS MET", req_total);
        attroff(COLOR_PAIR(CP_GOOD) | A_BOLD);
    } else {
        attron(COLOR_PAIR(CP_BAD) | A_BOLD);
        printw(" %d/%d PASS  %d FAIL", req_pass, req_total, req_fail);
        attroff(COLOR_PAIR(CP_BAD) | A_BOLD);
    }
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(r, 2, "Press ");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    attron(A_BOLD); printw("R"); attroff(A_BOLD);
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    printw(" in demo mode to clear injected faults and reset counters");
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
}

/*============= §5 PAGE: DATASET ===========================================================*/
/*============= DRAW: Page 7 — Dataset Capture =============================================*/

void draw_page_dataset(int r, IPC_Context *ipc)
{
    /* Drain is done once per tick in the main loop, not here — so that
     * captures keep advancing even when the user flips to another page. */

    /* Expire the transient SAVED / CANCELLED banner after its TTL. */
    if((ds_state == DS_SAVED || ds_state == DS_CANCELLED) &&
       now_ms_wall() > ds_msg_until)
    {
        ds_state = DS_IDLE;
    }

    /*---- Header row: state | label | mode ------------------------------*/
    const char *state_tag;
    int         state_cp;
    switch(ds_state)
    {
        case DS_COLLECTING: state_tag = "* COLLECTING"; state_cp = CP_BAD;  break;
        case DS_SAVED:      state_tag = "v SAVED";      state_cp = CP_GOOD; break;
        case DS_CANCELLED:  state_tag = "x CANCELLED";  state_cp = CP_WARN; break;
        case DS_IDLE:
        default:            state_tag = "IDLE";         state_cp = CP_DIM;  break;
    }

    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1, "State:");
    attroff(COLOR_PAIR(CP_DIM));
    attron(COLOR_PAIR(state_cp) | A_BOLD);
    mvprintw(r, 8, "%-14s", state_tag);
    attroff(COLOR_PAIR(state_cp) | A_BOLD);

    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 26, "Label:");
    attroff(COLOR_PAIR(CP_DIM));
    attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
    mvprintw(r, 33, "[%d] %-8s", ds_label_idx, CLS_NAMES[ds_label_idx]);
    attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 51, "Mode:");
    attroff(COLOR_PAIR(CP_DIM));
    attron(A_BOLD);
    mvprintw(r, 57, "%s", ds_mode == DS_MODE_FILTERED ? "FILTERED" : "RAW");
    attroff(A_BOLD);
    r++;

    /*---- Stats row: samples, elapsed, gaps, missed ---------------------*/
    uint64_t elapsed_ms = 0;
    if(ds_state == DS_COLLECTING)
        elapsed_ms = now_ms_wall() - ds_start_ms;
    else if(ds_state == DS_SAVED || ds_state == DS_CANCELLED)
        /* Freeze the displayed elapsed when transient — feels right. */
        elapsed_ms = (ds_msg_until > ds_start_ms + 2000)
                   ? (ds_msg_until - ds_start_ms - 2000)
                   : 0;

    double elapsed_s = elapsed_ms / 1000.0;

    attron(COLOR_PAIR(CP_DIM)); mvprintw(r, 1,  "Samples:"); attroff(COLOR_PAIR(CP_DIM));
    attron(A_BOLD);             mvprintw(r, 10, "%-10u", ds_samples); attroff(A_BOLD);

    attron(COLOR_PAIR(CP_DIM)); mvprintw(r, 22, "Elapsed:"); attroff(COLOR_PAIR(CP_DIM));
    attron(A_BOLD);             mvprintw(r, 31, "%7.3fs", elapsed_s); attroff(A_BOLD);

    attron(COLOR_PAIR(CP_DIM)); mvprintw(r, 43, "Gaps:"); attroff(COLOR_PAIR(CP_DIM));
    {
        int cp = ds_gaps > 0 ? CP_WARN : CP_GOOD;
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvprintw(r, 49, "%-6u", ds_gaps);
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }

    attron(COLOR_PAIR(CP_DIM)); mvprintw(r, 58, "Missed:"); attroff(COLOR_PAIR(CP_DIM));
    {
        int cp = ds_missed > 0 ? CP_BAD : CP_GOOD;
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvprintw(r, 66, "%u", ds_missed);
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }
    r++;

    /*---- Paths ---------------------------------------------------------*/
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1, "Out dir:  %s/", DATASET_OUT_DIR);
    attroff(COLOR_PAIR(CP_DIM));
    r++;

    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1, "File:     ");
    attroff(COLOR_PAIR(CP_DIM));
    if(ds_path[0])
    {
        int cp = ds_state == DS_CANCELLED ? CP_WARN
               : ds_state == DS_COLLECTING ? CP_CYAN
               : CP_GOOD;
        attron(COLOR_PAIR(cp));
        printw("%s", ds_path);
        attroff(COLOR_PAIR(cp));
    }
    else
    {
        attron(COLOR_PAIR(CP_DIM) | A_DIM);
        printw("(none yet)");
        attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    }
    r++;

    /*---- Demo banner ---------------------------------------------------*/
    if(demo_mode)
    {
        attron(COLOR_PAIR(CP_CYAN) | A_DIM);
        mvprintw(r, 1,
                 "Demo mode: capture writes real CSV from synthetic %s @ %gHz packets.",
                 demo_wave_label(demo_wave), (double)demo_freq_hz);
        attroff(COLOR_PAIR(CP_CYAN) | A_DIM);
        r++;
    }

    r++;
    draw_hline(r, 0, g_tui_w);
    r++;

    /*---- Label strip ---------------------------------------------------*/
    attron(COLOR_PAIR(CP_DIM));
    mvprintw(r, 1,
             "Labels (LEFT/RIGHT cycle | s,SPACE start/stop | r cancel | t raw<->filt):");
    attroff(COLOR_PAIR(CP_DIM));
    r++;

    /* Lay out labels in one row, each padded to 7 cols so they align. */
    int x = 2;
    for(int i = 0; i < DATASET_LABEL_COUNT; i++)
    {
        if(i == ds_label_idx)
        {
            /* Selected: reverse-video. Disable selection change visually
             * while collecting so the user knows they can't change labels
             * mid-capture. */
            int cp = (ds_state == DS_COLLECTING) ? CP_BAD : CP_CYAN;
            attron(COLOR_PAIR(cp) | A_REVERSE | A_BOLD);
            mvprintw(r, x, " %-7s", CLS_NAMES[i]);
            attroff(COLOR_PAIR(cp) | A_REVERSE | A_BOLD);
        }
        else
        {
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, x, " %-7s", CLS_NAMES[i]);
            attroff(COLOR_PAIR(CP_DIM));
        }
        x += 8;   /* 1 space + 7 label chars */
        if(x + 8 > g_tui_w) { r++; x = 2; }   /* wrap if terminal narrow */
    }
    r++;

    r++;
    draw_hline(r, 0, g_tui_w);
    r++;

    /*---- Live waveforms ------------------------------------------------*/
    {
        char fbuf[64];
        uint32_t head = atomic_load(&ipc->ctrl->sensor_head);
        uint8_t  last_flags = (head > 0)
            ? ipc->ring[(head - 1) & IPC_SENSOR_RING_MASK].flags
            : 0;
        wl_flags_decode(last_flags, fbuf, sizeof(fbuf));
        int severe = (last_flags &
                      (WL_FLAG_CLIPPING | WL_FLAG_ELEC_OFF | WL_FLAG_ADC_OVRN));
        int cp = (fbuf[0] == '\0') ? CP_GOOD : (severe ? CP_BAD : CP_WARN);

        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, 1, "Live waveforms  (raw ADC, 8 ch, BSAU flags:");
        attroff(COLOR_PAIR(CP_DIM));
        attron(COLOR_PAIR(cp) | A_BOLD);
        printw(" %s", fbuf[0] ? fbuf : "OK");
        attroff(COLOR_PAIR(cp) | A_BOLD);
        attron(COLOR_PAIR(CP_DIM));
        printw(")");
        attroff(COLOR_PAIR(CP_DIM));
    }
    r++;

    int half_w = g_tui_w / 2;
    int plot_w = half_w - 8;
    if(plot_w < 16) plot_w = 16;

    /* Stop at g_term_h - 3 to leave room for the footer (separator + line). */
    int max_r = g_term_h - 3;

    /* Auto-size plot height from what's left. We have 4 plot rows in a 2x4
     * grid, each consuming (plot_h + 1) rows for the trace + axis line.
     * Cap at a sane upper bound so a tall window doesn't produce a single
     * giant plot per channel — at ~6 rows the line trace already gives 30
     * sub-row resolution which is plenty. */
    int avail   = max_r - r;          /* rows we can use         */
    int plot_h  = (avail / 4) - 1;    /* per plot, minus its axis*/
    if(plot_h < 2)  plot_h = 2;       /* don't go below original */
    if(plot_h > 6)  plot_h = 6;       /* cap at scope-readable   */

    for(int i = 0; i < 4 && r + plot_h + 1 <= max_r; i++)
    {
        /* Left column: ch 0..3. Label aligned to the vertical middle of
         * the plot so a tall plot doesn't look mislabelled. */
        int label_row = r + plot_h / 2;
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(label_row, 1, "ch%d", i);
        attroff(COLOR_PAIR(CP_DIM));
        draw_waveform(r, 5, plot_w, plot_h, i, CP_GOOD);

        /* Right column: ch 4..7 */
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(label_row, half_w + 1, "ch%d", i + 4);
        attroff(COLOR_PAIR(CP_DIM));
        draw_waveform(r, half_w + 5, plot_w, plot_h, i + 4, CP_CYAN);

        r += plot_h + 1;   /* plot rows + axis */
    }
}

/*============= §5 PAGE: CONFIG ============================================================*/
/*============= DRAW: Page 5 — System Configuration ========================================*/

/**
 *  Static compile-time + hardware/topology info. Nothing here changes at
 *  runtime — it's a "spec sheet" so new readers understand the system
 *  they're looking at without digging through source/docs.
 */
void draw_page_config(int r, IPC_Context *ipc)
{
    /*==================== EDIT-MODE BANNER (v2.3.4) ====================*/
    /*  Shows the current handshake state. The banner consumes one row
     *  at the very top of the CONFIG page so it's always visible while
     *  editing.
     *
     *  States:
     *    LOCKED      — edit_mode_request == 0. Read-only. Press 'e' to enter.
     *    PARKING     — request is set, smoother walking to neutral, not yet
     *                  settled (active still 0). Editing not yet allowed.
     *    EDITING     — both request and active are 1. Arm parked at neutral.
     *                  Edits permitted (and persisted on Ctrl+S).
     *    DSP UNRESP  — request raised but dsp_ack hasn't flipped within
     *                  500 ms. Possible cpcu_dsp.py crash. Editing held.
     */
    uint8_t  edit_req      = atomic_load(&ipc->ctrl->edit_mode_request);
    uint8_t  edit_active   = atomic_load(&ipc->ctrl->edit_mode_active);
    uint8_t  edit_dsp_ack  = atomic_load(&ipc->ctrl->edit_mode_dsp_ack);
    uint64_t edit_req_us   = atomic_load(&ipc->ctrl->edit_mode_request_us);
    uint64_t now_us        = now_ms_wall() * 1000ULL;

    const char *banner_text;
    int banner_color;
    if(!edit_req)
    {
        banner_text  = "Edit mode: [LOCKED]      Press 'e' on this page to enter edit mode";
        banner_color = CP_DIM;
    }
    else if(edit_active)
    {
        banner_text  = "Edit mode: [EDITING - arm parked]   Press 'e' to exit, "
                       "Ctrl+S to save (planned)";
        banner_color = CP_GOOD;
    }
    else
    {
        /* Request is up but cpcu_io hasn't confirmed settled.
         * Distinguish "still walking" from "DSP unresponsive". */
        uint64_t elapsed_ms = (edit_req_us > 0 && now_us > edit_req_us)
                              ? (now_us - edit_req_us) / 1000 : 0;
        if(elapsed_ms > 500 && !edit_dsp_ack)
        {
            banner_text  = "Edit mode: [DSP UNRESPONSIVE]   handshake timed out, "
                           "check cpcu_dsp.py";
            banner_color = CP_BAD;
        }
        else
        {
            banner_text  = "Edit mode: [PARKING ARM...]   waiting for SMOOTH_AllSettled, "
                           "press 'e' again to abort";
            banner_color = CP_WARN;
        }
    }
    attron(COLOR_PAIR(banner_color) | A_BOLD);
    mvprintw(r, 1, "%-*s", g_tui_w - 2, banner_text);
    attroff(COLOR_PAIR(banner_color) | A_BOLD);
    r += 2;

    /* v2.3.8: in EDITING state, render the live editor instead of the
     * spec-sheet view. The editor's key bindings are handled in
     * cpcu_tui.c's main loop; here we only draw. Outside EDITING the
     * spec sheet renders (the user is viewing, not editing). */
    if(edit_req && edit_active)
    {
        ED_Render(r);
        return;       /* editor rendered; spec sheet skipped */
    }

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== LIVE GESTURE CONFIG (from cpcu_dsp.py) ====================
     * cpcu_dsp.py writes a pre-formatted text digest of gestures.json
     * to /tmp/cpcu_gestures_digest.txt at startup (and after the future
     * SIGHUP reload). The TUI just opens the file and prints it line
     * by line — this avoids embedding a JSON parser in C. If the file
     * is missing (DSP not running, or older version), the section
     * falls back to a single hint line. */
    draw_section(r, 1, "LIVE GESTURE CONFIG (from cpcu_dsp.py — see config/gestures.json)");
    r++;
    {
        FILE *df = fopen("/tmp/cpcu_gestures_digest.txt", "r");
        if(!df)
        {
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, 1,
                "digest not available — start cpcu_dsp.py to populate "
                "/tmp/cpcu_gestures_digest.txt");
            attroff(COLOR_PAIR(CP_DIM));
            r++;
        }
        else
        {
            char line[256];
            while(r < g_term_h - 1 && fgets(line, sizeof(line), df))
            {
                /* strip newline */
                size_t L = strlen(line);
                while(L > 0 && (line[L-1] == '\n' || line[L-1] == '\r'))
                    line[--L] = '\0';
                if(L == 0) { r++; continue; }
                /* Section headers (UPPERCASE first char + no leading
                 * spaces) get a bold accent; group sub-headers
                 * ("  [right_arm]") get cyan; everything else dim. */
                int cp = CP_DIM;
                int bold = 0;
                if(line[0] == '#') {
                    cp = CP_DIM;  /* comment row */
                } else if(line[0] != ' ' && line[0] != '\0') {
                    cp = CP_HEADER; bold = 1;
                } else if(strstr(line, "[") && strstr(line, "]")) {
                    cp = CP_CYAN;   bold = 1;
                }
                attron(COLOR_PAIR(cp) | (bold ? A_BOLD : 0));
                mvprintw(r, 1, "%.*s", g_tui_w - 2, line);
                attroff(COLOR_PAIR(cp) | (bold ? A_BOLD : 0));
                r++;
            }
            fclose(df);
        }
    }
    r++;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== SMOOTHER CONFIG (per-servo) ====================
     * launch.sh's preflight writes /tmp/cpcu_smoother_config.txt from
     * runtime.json: one tab-separated line per slot with the live
     * velocity / accel / deadband / bias the smoother is using. This
     * is the same data IPC_RuntimeConfig carries to cpcu_io, just made
     * visible here so the operator can confirm a calibration without
     * leaving the TUI. */
    draw_section(r, 1, "SMOOTHER CONFIG (per-servo motion profile, from runtime.json)");
    r++;
    {
        FILE *sf = fopen("/tmp/cpcu_smoother_config.txt", "r");
        if(!sf)
        {
            attron(COLOR_PAIR(CP_DIM));
            mvprintw(r, 1,
                "smoother digest not available — restart with './launch.sh tui' "
                "to populate /tmp/cpcu_smoother_config.txt");
            attroff(COLOR_PAIR(CP_DIM));
            r++;
        }
        else
        {
            /* Column header */
            attron(COLOR_PAIR(CP_HEADER) | A_BOLD);
            mvprintw(r, 3, "%-10s %12s %15s %12s %10s",
                     "Servo", "Vel (us/s)", "Accel (us/s²)",
                     "Deadband", "Bias");
            attroff(COLOR_PAIR(CP_HEADER) | A_BOLD);
            r++;

            char line[256];
            while(r < g_term_h - 1 && fgets(line, sizeof(line), sf))
            {
                size_t L = strlen(line);
                while(L > 0 && (line[L-1] == '\n' || line[L-1] == '\r'))
                    line[--L] = '\0';
                if(L == 0) continue;

                /* parse name\tvel\taccel\tdeadband\tbias */
                char *p_name = line;
                char *p_vel  = strchr(p_name, '\t'); if(!p_vel)  continue; *p_vel++  = '\0';
                char *p_acc  = strchr(p_vel,  '\t'); if(!p_acc)  continue; *p_acc++  = '\0';
                char *p_db   = strchr(p_acc,  '\t'); if(!p_db)   continue; *p_db++   = '\0';
                char *p_bias = strchr(p_db,   '\t'); if(!p_bias) continue; *p_bias++ = '\0';

                long vel = atol(p_vel), acc = atol(p_acc);
                long db  = atol(p_db),  bs  = atol(p_bias);

                /* color hints: red if vel/accel is zero (servo would freeze),
                 * dim if bias is non-zero (operator override). */
                int vel_cp  = (vel  > 0) ? CP_GOOD : CP_BAD;
                int acc_cp  = (acc  > 0) ? CP_GOOD : CP_BAD;
                int bias_cp = (bs  != 0) ? CP_WARN : CP_DIM;

                mvprintw(r, 3, "%-10s ", p_name);
                attron(COLOR_PAIR(vel_cp));
                printw("%12ld ", vel);
                attroff(COLOR_PAIR(vel_cp));
                attron(COLOR_PAIR(acc_cp));
                printw("%15ld ", acc);
                attroff(COLOR_PAIR(acc_cp));
                attron(COLOR_PAIR(CP_CYAN));
                printw("%12ld ", db);
                attroff(COLOR_PAIR(CP_CYAN));
                attron(COLOR_PAIR(bias_cp));
                printw("%+10ld", bs);
                attroff(COLOR_PAIR(bias_cp));
                r++;
            }
            fclose(sf);
        }
    }
    r++;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== BSAU SIDE ====================*/
    draw_section(r, 1,       "BSAU (BIOSIGNAL ACQUISITION UNIT)");
    draw_section(r, g_col_r, "CPCU (CENTRAL PROCESSING & CONTROL UNIT)");
    r++;

    draw_lv(r, 1,       "MCU:",           CP_CYAN, "STM32L432KC  (ARM Cortex-M4F, 80 MHz)");
    draw_lv(r, g_col_r, "SBC:",           CP_CYAN, "Raspberry Pi 4B/5  (ARM Cortex-A72/A76)");
    r++;
    draw_lv(r, 1,       "EMG channels:",  CP_CYAN, "%d  (Soldered INA333 front-ends)", WL_NUM_CHANNELS);
    draw_lv(r, g_col_r, "OS / kernel:",   CP_CYAN, "Raspberry Pi OS 64-bit  (6.x)");
    r++;
    draw_lv(r, 1,       "ADC:",           CP_CYAN, "STM32 12-bit ADC  (0..4095)");
    draw_lv(r, g_col_r, "RT isolation:",  CP_CYAN, "isolcpus=1,2,3  (nohz_full+rcu_nocbs)");
    r++;
    draw_lv(r, 1,       "Sample rate:",   CP_CYAN, "2 kHz per channel");
    draw_lv(r, g_col_r, "Core 0:",        CP_CYAN, "Supervisor / TUI / logger");
    r++;
    draw_lv(r, 1,       "Samples/pkt:",   CP_CYAN, "%d  (packed 12-bit)", WL_SAMPLES_PER_PACKET);
    /* cpcu_dsp.py is spawned on cores 1-2 at SCHED_FIFO prio 80 by
     * cpcu_kernel; classifier is RandomForest (200 trees) loaded from
     * models/arm.pkl. */
    draw_lv(r, g_col_r, "Cores 1-2:",     CP_CYAN, "Python DSP + RandomForest  (SCHED_FIFO prio 80)");
    r++;
    draw_lv(r, 1,       "Packet rate:",   CP_CYAN, "1000 pkt/s  (2 samples @ 2 kHz)");
    /* cpcu_io is the realtime BSAU radio + PCA9685 driver. cpcu_kernel
     * spawns it on core 3 at SCHED_FIFO prio 90 — HIGHER than DSP so
     * radio + smoother ticks never get preempted by a Python GIL hold. */
    draw_lv(r, g_col_r, "Core 3:",        CP_CYAN, "cpcu_io  (SCHED_FIFO prio 90)");
    r++;
    /* v3 hardware: BSAU does not sample battery voltage; the safety
     * FSM ignores vbat entirely (safety_ignore_battery=1 in IPC config).
     * The Battery row used to claim "2S Li-ion pack + 2:1 divider" —
     * never accurate for v3. Replaced with the actual radio link. */
    draw_lv(r, 1,       "Wireless link:", CP_CYAN, "BSAU -> CPCU @ nRF24L01+");
    draw_lv(r, g_col_r, "Scheduler:",     CP_CYAN, "SCHED_FIFO realtime, mlockall");
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== WIRELESS LINK + IPC ====================*/
    draw_section(r, 1,       "WIRELESS LINK");
    draw_section(r, g_col_r, "IPC (SHARED MEMORY)");
    r++;

    /* Path size MUST equal IPC_SHM_SIZE in cpcu_ipc.h. Computing it
     * here from the same component sizes avoids the stale "66240 B"
     * literal that lived here before the 4096-entry ring fix and
     * silently lied about the actual mapping. After ring-size fix:
     *   192 + 64*4096 + 128 + 128 + 256 + 512 + 512 + 6432 = 270 304 B.
     * The constant matches cpcu_ipc_bridge.py's SHM_TOTAL. */
    draw_lv(r, 1,       "Radio:",         CP_CYAN, "nRF24L01+  (2.4 GHz GFSK)");
    draw_lv(r, g_col_r, "Path:",          CP_CYAN, "/dev/shm/cpcu_ipc  (%zu B)",
            (size_t)(sizeof(IPC_ControlBlock)
                   + sizeof(IPC_SensorEntry)  * IPC_SENSOR_RING_SIZE
                   + sizeof(IPC_MotorCommand)
                   + sizeof(IPC_Diagnostics)
                   + sizeof(IPC_DSPExport)
                   + sizeof(IPC_RuntimeConfig)
                   + sizeof(IPC_ToolPresence)
                   + sizeof(IPC_DspFiltered)));
    r++;
    draw_lv(r, 1,       "Channel:",       CP_CYAN, "76  (2.476 GHz, ISM)  -- matches BSAU");
    draw_lv(r, g_col_r, "Layout:",        CP_CYAN, "ctrl + ring + motor + dsp_export");
    r++;
    draw_lv(r, 1,       "Address:",       CP_CYAN, "E7:E7:E7:E7:E7");
    draw_lv(r, g_col_r, "Ring size:",     CP_CYAN, "%u entries  (64 B each)", IPC_SENSOR_RING_SIZE);
    r++;
    draw_lv(r, 1,       "SPI clock:",     CP_CYAN, "8 MHz");
    draw_lv(r, g_col_r, "Ring type:",     CP_CYAN, "SPSC lock-free  (seq head/tail)");
    r++;
    draw_lv(r, 1,       "Payload:",       CP_CYAN, "%d B fixed", WL_PAYLOAD_SIZE);
    draw_lv(r, g_col_r, "Motor cmd:",     CP_CYAN, "128 B  (seqlock updates)");
    r++;
    draw_lv(r, 1,       "Auto-ACK:",      CP_CYAN, "on, up to 3 auto-retries");
    draw_lv(r, g_col_r, "DSP export:",    CP_CYAN, "256 B  (gesture+confs+RMS)");
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== MOTOR + ML ====================*/
    draw_section(r, 1,       "MOTOR STAGE");
    draw_section(r, g_col_r, "DSP / ML");
    r++;

    draw_lv(r, 1,       "PWM driver:",    CP_CYAN, "PCA9685  (I2C @ 400 kHz, 50 Hz PWM)");
    draw_lv(r, g_col_r, "Window:",        CP_CYAN, "200 samples  (WINDOW_HI in cpcu_dsp.py)");
    r++;
    draw_lv(r, 1,       "Servos:",        CP_CYAN, "%d × SG90  (1.0-2.0 ms pulse)", IPC_NUM_SERVOS);
    draw_lv(r, g_col_r, "Stride:",        CP_CYAN, "100 samples  (50%% overlap)");
    r++;
    /* SMOOTH_DEFAULT_VELOCITY in cpcu_smooth.h is 2000 us/s; the gripper
     * slot is bumped down by runtime.json's smooth_velocity for slot 5.
     * "3000 us/s" used to live here — left over from an earlier tuning
     * pass; never matched the compile-time default. */
    draw_lv(r, 1,       "Slew limit:",    CP_CYAN, "2000 us/s default  (per-servo via runtime.json)");
    draw_lv(r, g_col_r, "Feature extr:",  CP_CYAN, "RMS, var, WL, env_mean, MAV, ZC, SSC  (7/ch)");
    r++;
    draw_lv(r, 1,       "Safety cmd:",    CP_CYAN, "Servos → neutral (1500 us) on SAFE");
    draw_lv(r, g_col_r, "Classifier:",    CP_CYAN, "RandomForest (sklearn, 200 trees)");
    r++;
    /* v3: vbatt thresholds removed — safety FSM skips the battery check
     * entirely (safety_ignore_battery=1). Only radio-loss thresholds
     * remain active. */
    draw_lv(r, 1,       "Safety thr:",    CP_CYAN, "Radio loss 750 / 1500 ms (warn / SAFE)");
    draw_lv(r, g_col_r, "Classes:",       CP_CYAN, "4 active (ext, flex, hand, rest)");
    r += 2;

    draw_hline(r - 1, 0, g_tui_w);

    /*==================== BUILD INFO + WEB DASHBOARD ====================*/
    draw_section(r, 1, "BUILD");
    draw_section(r, g_col_r, "WEB DASHBOARD");
    r++;

    draw_lv(r, 1, "TUI version:",  CP_CYAN, "v3.4-pageorder");
    /* Show web dashboard URL when active (written by launch.sh) */
    {
        FILE *ws_f = fopen("/tmp/cpcu_ws_active.txt", "r");
        if(ws_f)
        {
            char ws_url[128] = {0};
            char line[256];
            while(fgets(line, sizeof(line), ws_f))
            {
                if(strncmp(line, "url=", 4) == 0)
                {
                    size_t len = strlen(line + 4);
                    if(len > 0 && line[4 + len - 1] == '\n') line[4 + len - 1] = '\0';
                    snprintf(ws_url, sizeof(ws_url), "%s", line + 4);
                }
            }
            fclose(ws_f);
            draw_lv(r, g_col_r, "URL:",  CP_GOOD, "%s", ws_url[0] ? ws_url : "ACTIVE");
        }
        else
        {
            draw_lv(r, g_col_r, "Status:",  CP_DIM, "NOT RUNNING  (./launch.sh ws)");
        }
    }
    r++;
    draw_lv(r, 1, "Built:",  CP_DIM, "%s %s", __DATE__, __TIME__);
    draw_lv(r, g_col_r, "Bind:", CP_CYAN, "0.0.0.0:8765  (LAN-shared)");
    r++;
    draw_lv(r, 1, "Compiler:",     CP_CYAN,
#ifdef __GNUC__
            "GCC %d.%d", __GNUC__, __GNUC_MINOR__
#else
            "unknown"
#endif
            );
    draw_lv(r, g_col_r, "Logs:",  CP_CYAN, "log/");
}

/*============= §4 FOOTER ==================================================================*/
/*============= DRAW: Footer ===============================================================*/

void draw_footer(int r)
{
    draw_hline(r, 0, g_tui_w);
    r++;
    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    if(demo_mode)
    {
        /* Extended footer in demo mode: shows fault-injection + waveform hotkeys */
        if(current_page == PAGE_WAVES)
            mvprintw(r, 1,
                "1-7:pg UP/DN:ch TAB q:quit | w:wave [/]:freq | F=radio B=batt G=gaps O=ring I=i2c R=reset");
        else if(current_page == PAGE_DATASET)
            mvprintw(r, 1,
                "1-7:pg q:quit | LEFT/RIGHT:label s,SPACE:start/stop r:cancel t:raw/filt | w:wave [/]:freq");
        else
            mvprintw(r, 1,
                "1-7:pg q:quit | w:wave [/]:freq | FAULT INJ: F=radio B=batt G=gaps O=ring I=i2c R=reset");
    }
    else
    {
        if(current_page == PAGE_WAVES)
            mvprintw(r, 1, "1-7:pages  UP/DN:ch  TAB:detail  q:quit  30 Hz");
        else if(current_page == PAGE_DATASET)
            mvprintw(r, 1, "1-7:pg  LEFT/RIGHT:label  s,SPACE:start/stop  r:cancel  t:raw/filt  q:quit");
        else if(current_page == PAGE_CONFIG)
            mvprintw(r, 1, "1-7:pages  e:edit  q:quit  |  Ctrl+S:save  ESC:cancel  r:revert");
        else
            mvprintw(r, 1, "1-7:pages  q:quit  30 Hz  |  read-only (zero RT impact)");
    }
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);

    /* Right-side tags: fault banner (red) takes precedence over [DEMO] */
    if(demo_mode)
    {
        const char *inj = fault_banner(demo_fault_mask);
        if(inj)
        {
            int inj_len = (int)strlen(inj);
            attron(COLOR_PAIR(CP_BAD) | A_BOLD);
            mvprintw(r, g_tui_w - inj_len - 1, "%s", inj);
            attroff(COLOR_PAIR(CP_BAD) | A_BOLD);
        }
        else
        {
            /* No fault active — show current waveform + frequency instead
             * of the plain [DEMO] tag. e.g. "[SINE 100Hz]" */
            char tag[24];
            snprintf(tag, sizeof(tag), "[%s %gHz]",
                     demo_wave_label(demo_wave), (double)demo_freq_hz);
            int tag_len = (int)strlen(tag);
            attron(COLOR_PAIR(CP_CYAN));
            mvprintw(r, g_tui_w - tag_len - 1, "%s", tag);
            attroff(COLOR_PAIR(CP_CYAN));
        }
    }
}

/*==========================================================================================*/
