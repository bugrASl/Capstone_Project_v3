/**
 *  @file   cpcu_tui.c
 *  @brief  ncurses live dashboard — main loop, key dispatch, boot sequence.
 *
 *  7-page TUI for monitoring and configuring the running system:
 *    Page 1: Overview    Page 2: Radio/IO     Page 3: DSP/AI
 *    Page 4: Waves       Page 5: Health       Page 6: Dataset
 *    Page 7: Config (with live editor)
 *
 *  Reads IPC shared memory at ~10 Hz refresh. Supports demo mode with
 *  synthetic data for development without hardware.
 */

#include "cpcu_tui.h"
#include "cpcu_tui_editor.h"        /* live editor */

#include <locale.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*============= MAIN-MODULE STATE (definitions) ============================================*/
/*
 *  These are declared extern in cpcu_tui.h. Defined here because cpcu_tui.c
 *  is the program entry-point and conceptually owns the run/quit state and
 *  the active page selection. The render layer only reads them; the data
 *  layer doesn't touch them.
 */

volatile sig_atomic_t  g_run         =   1;

/* track whether ED_Init() has been called for the current
 * session of EDITING. Reset to false when leaving EDITING so re-entry
 * picks up fresh disk values (e.g. if pca_testbench saved between
 * sessions). */
static bool            g_ed_initialized_seen = false;
Page                   current_page  =   PAGE_OVERVIEW;
bool                   demo_mode     =   false;
bool                   show_splash   =   true;

static void on_sig(int s)
{
    (void)s;
    g_run = 0;
}

/*============= SPLASH ====================================================================*/

static void draw_splash(void)
{
    /*  ASCII-only block art — deliberately avoids Unicode box-drawing,
     *  which was reported as "garbled" on some SSH clients in prior
     *  versions. Each letter is a rigid 6-col × 8-row block separated
     *  by a 2-space gutter.                                               */
    static const char *art[] = {
        " ####   #####    ####   ##  ##",
        "##  ##  ##  ##  ##  ##  ##  ##",
        "##      ##  ##  ##      ##  ##",
        "##      #####   ##      ##  ##",
        "##      ##      ##      ##  ##",
        "##      ##      ##      ##  ##",
        "##  ##  ##      ##  ##  ##  ##",
        " ####   ##       ####    #### ",
    };
    const int lines = (int)(sizeof(art) / sizeof(art[0]));

    erase();

    /*  Vertical centring: art rows + 6 rows of text below.                */
    int total_h = lines + 6;
    int cy = (g_term_h - total_h) / 2;
    if(cy < 1) cy = 1;

    int art_w = (int)strlen(art[0]);
    int cx = (g_term_w - art_w) / 2;
    if(cx < 0) cx = 0;

    attron(COLOR_PAIR(CP_CYAN) | A_BOLD);
    for(int i = 0; i < lines; i++)
        mvprintw(cy + i, cx, "%s", art[i]);
    attroff(COLOR_PAIR(CP_CYAN) | A_BOLD);

    const char *t1 = "CPCU Monitor v3.4-pageorder  -  Prosthetic Hand Controller";
    const char *t2 = "EE493/494 Capstone Design Project";
    const char *t3 = "METU - 2026";
    const char *t4 = "Press any key to continue";

    attron(A_BOLD);
    mvprintw(cy + lines + 2, (g_term_w - (int)strlen(t1)) / 2, "%s", t1);
    attroff(A_BOLD);
    mvprintw(cy + lines + 3, (g_term_w - (int)strlen(t2)) / 2, "%s", t2);
    mvprintw(cy + lines + 4, (g_term_w - (int)strlen(t3)) / 2, "%s", t3);

    attron(COLOR_PAIR(CP_DIM) | A_DIM);
    mvprintw(cy + lines + 6, (g_term_w - (int)strlen(t4)) / 2, "%s", t4);
    attroff(COLOR_PAIR(CP_DIM) | A_DIM);
    refresh();
}

static void run_splash(void)
{
    layout_update();
    draw_splash();
    nodelay(stdscr, FALSE);
    timeout(-1);                        /* block until a key is pressed */
    getch();
    nodelay(stdscr, TRUE);
}

/*============= DEMO FULL RESET ===========================================================*/
/**
 *  Wipe demo fault flags AND zero the cumulative diagnostic counters so
 *  the UI snaps back to a clean-boot look. Triggered by the 'R' hotkey
 *  on any non-Dataset page (or on Dataset when not capturing).
 */
static void demo_full_reset(IPC_Context *ipc)
{
    demo_fault_mask     = FAULT_NONE;
    demo_fault_onset_ms = 0;
    demo_pkts           = 0;
    demo_gaps           = 0;
    demo_inf_count      = 0;
    demo_gesture        = 0;

    atomic_store(&ipc->ctrl->system_state,          IPC_STATE_RUNNING);
    atomic_store(&ipc->ctrl->io_ready,              1);
    atomic_store(&ipc->ctrl->dsp_ready,             1);
    atomic_store(&ipc->diag->io_pkts_received,      0);
    atomic_store(&ipc->diag->io_pkts_dropped,       0);
    atomic_store(&ipc->diag->io_seq_gaps,           0);
    atomic_store(&ipc->diag->io_ring_overflows,     0);
    atomic_store(&ipc->diag->io_safe_entries,       0);
    atomic_store(&ipc->diag->io_max_poll_us,        0);
    atomic_store(&ipc->diag->dsp_batches,           0);
    atomic_store(&ipc->diag->dsp_inferences,        0);
    atomic_store(&ipc->diag->dsp_max_latency_us,    0);
    atomic_store(&ipc->diag->dsp_ring_underflows,   0);
}

/*============= MAIN ======================================================================*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);

    /*-------------- CLI parsing --------------------------------------------------------------*/
    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--demo") == 0 || strcmp(argv[i], "-d") == 0)
            demo_mode = true;
        else if(strcmp(argv[i], "--no-splash") == 0)
            show_splash = false;
    }

    /*-------------- IPC bring-up -------------------------------------------------------------*/
    IPC_Context ipc;
    memset(&ipc, 0, sizeof(ipc));

    if(demo_mode)
    {
        demo_init(&ipc);
        printf("[TUI] Demo mode — synthetic data.\n");
    }
    else
    {
        if(IPC_Open(&ipc) != 0)
        {
            fprintf(stderr, "[TUI] Cannot open shared memory. Is cpcu_kernel running?\n");
            fprintf(stderr, "  Try: ./cpcu_tui --demo\n");
            return 1;
        }
    }

    /*-------------- ncurses bring-up ---------------------------------------------------------*/
    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

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
        init_pair(CP_BAR_FILL,  COLOR_GREEN,    -1);
        init_pair(CP_BAR_EMPTY, COLOR_WHITE,    -1);
        init_pair(CP_MAGENTA,   COLOR_MAGENTA,  -1);
    }

    if(show_splash)
        run_splash();

    /*-------------- Main loop ----------------------------------------------------------------*/
    uint32_t prev_pkts  =   0;
    uint64_t prev_time  =   0;
    uint64_t boot_time  =   now_ms();

    while(g_run)
    {
        layout_update();

        if(demo_mode) demo_tick(&ipc);

        /*  Live stats (used by overview + radio page draws). */
        uint32_t pkts       =   atomic_load(&ipc.diag->io_pkts_received);
        uint32_t gaps       =   atomic_load(&ipc.diag->io_seq_gaps);

        uint64_t t_now      =   now_ms();
        uint32_t pkt_rate   =   0;
        if(prev_time > 0 && t_now > prev_time)
        {
            uint32_t dt_ms = (uint32_t)(t_now - prev_time);
            if(dt_ms > 0)
                pkt_rate = (pkts - prev_pkts) * 1000 / dt_ms;
        }
        prev_pkts = pkts;
        prev_time = t_now;

        float    loss_rate  = (pkts > 0) ? (float)gaps / (float)pkts : 0.0f;
        uint32_t uptime_s   = (uint32_t)((t_now - boot_time) / 1000);
        uint32_t up_h       = uptime_s / 3600;
        uint32_t up_m       = (uptime_s / 60) % 60;
        uint32_t up_s       = uptime_s % 60;

        /*  Peek the ring (waveforms + dataset capture) every tick — both
         *  are page-independent so capture started on Dataset keeps
         *  running if the user flips to another page.                     */
        wave_peek_ring(&ipc);
        ds_drain_ring_to_file(&ipc);

        /*-------------- Render this frame --------------------------------------------------*/
        erase();
        int r = draw_header(0);

        switch(current_page)
        {
            case PAGE_OVERVIEW:
                draw_page_overview(r, &ipc, pkt_rate, loss_rate, up_h, up_m, up_s);
                break;
            case PAGE_RADIO:
                draw_page_radio(r, &ipc, pkt_rate, loss_rate, up_h, up_m, up_s);
                break;
            case PAGE_DSP:      draw_page_dsp(r, &ipc);                         break;
            case PAGE_WAVES:    draw_page_waves(r, &ipc);                       break;
            case PAGE_HEALTH:   draw_page_health(r, &ipc, pkt_rate, loss_rate); break;
            case PAGE_DATASET:  draw_page_dataset(r, &ipc);                     break;
            case PAGE_CONFIG:   draw_page_config(r, &ipc);                      break;
            default: break;
        }

        draw_footer(g_term_h - 2);
        refresh();

        /*-------------- Key dispatch -------------------------------------------------------*/
        int ch = getch();

        /* TUI live editor on CONFIG page.
         * When in EDITING state (handshake complete, arm parked), the
         * editor consumes nav keys (arrows, Enter, digits, Esc, Ctrl+S,
         * 'r'). Page-switch keys (1-7), 'e' (exit edit-mode), 'q' still
         * fall through to the global handler below. */
        if(current_page == PAGE_CONFIG && !demo_mode)
        {
            uint8_t edit_active = atomic_load_explicit(
                &ipc.ctrl->edit_mode_active, memory_order_acquire);
            if(edit_active)
            {
                /* Lazy init on first entry to EDITING. */
                if(!ED_GetField(0) || !g_ed_initialized_seen)
                {
                    ED_Init();
                    g_ed_initialized_seen = true;
                }
                if(ED_HandleKey(ch, &ipc))
                    goto after_dispatch;     /* editor consumed it */
            }
            else
            {
                /* Lost EDITING (e.g. user pressed 'e' to exit, or SAFE
                 * forced exit). Reset the lazy-init guard so re-entry
                 * starts fresh. Drafts are preserved across re-entry
                 * unless dsp/io reloaded the file in between (then
                 * ED_Init refreshes disk and clears dirty). */
                g_ed_initialized_seen = false;
            }
        }

        switch(ch)
        {
            case '1': current_page = PAGE_OVERVIEW; break;
            case '2': current_page = PAGE_RADIO;    break;
            case '3': current_page = PAGE_DSP;      break;
            case '4': current_page = PAGE_WAVES;    break;
            case '5': current_page = PAGE_HEALTH;   break;
            case '6': current_page = PAGE_DATASET;  break;
            case '7': current_page = PAGE_CONFIG;   break;

            /*-- Page 4 (Waves) navigation ------------------------------*/
            case KEY_UP:
                if(current_page == PAGE_WAVES)
                    wave_sel_ch = (wave_sel_ch - 1 + WL_NUM_CHANNELS) % WL_NUM_CHANNELS;
                break;
            case KEY_DOWN:
                if(current_page == PAGE_WAVES)
                    wave_sel_ch = (wave_sel_ch + 1) % WL_NUM_CHANNELS;
                break;
            case '\t':
                if(current_page == PAGE_WAVES)
                    wave_detail = !wave_detail;
                break;

            /*-- Page 6 (Dataset) controls ------------------------------*/
            case KEY_LEFT:
                if(current_page == PAGE_DATASET && ds_state != DS_COLLECTING)
                    ds_label_idx = (ds_label_idx - 1 + DATASET_LABEL_COUNT)
                                 % DATASET_LABEL_COUNT;
                break;
            case KEY_RIGHT:
                if(current_page == PAGE_DATASET && ds_state != DS_COLLECTING)
                    ds_label_idx = (ds_label_idx + 1) % DATASET_LABEL_COUNT;
                break;
            case 's': case 'S': case ' ':
                if(current_page == PAGE_DATASET)
                {
                    if(ds_state == DS_COLLECTING)        ds_stop_capture(true);
                    else if(ds_state == DS_IDLE)         (void)ds_start_capture(&ipc);
                    /* else: transient SAVED/CANCELLED banner — ignore */
                }
                break;
            case 't': case 'T':
                if(current_page == PAGE_DATASET && ds_state != DS_COLLECTING)
                    ds_mode = (ds_mode == DS_MODE_FILTERED)
                            ? DS_MODE_RAW : DS_MODE_FILTERED;
                break;

            /*-- Fault-injection hotkeys (demo mode only) ---------------*/
            /*  Single-bit XOR toggles share a small pattern. The 'F'
             *  case is special because flipping FAULT_RADIO_FREEZE on
             *  must also stamp the onset wall-clock time for the
             *  radio FSM.                                              */
            case 'f': case 'F':
                if(demo_mode)
                {
                    demo_fault_mask ^= FAULT_RADIO_FREEZE;
                    if(demo_fault_mask & FAULT_RADIO_FREEZE)
                        demo_fault_onset_ms = now_ms_wall();
                }
                break;
            case 'b': case 'B':
                if(demo_mode) { demo_fault_mask ^= FAULT_BATT_LOW;   } break;
            case 'g': case 'G':
                if(demo_mode) { demo_fault_mask ^= FAULT_GAP_STORM;  } break;
            case 'o': case 'O':
                if(demo_mode) { demo_fault_mask ^= FAULT_RING_OVF;   } break;
            case 'i': case 'I':
                if(demo_mode) { demo_fault_mask ^= FAULT_I2C_FAIL;   } break;

            case 'r': case 'R':
                /*  On Dataset page mid-capture: cancel-and-delete.
                 *  Everywhere else in demo mode: reset all faults. */
                if(current_page == PAGE_DATASET && ds_state == DS_COLLECTING)
                    ds_stop_capture(false);
                else if(demo_mode)
                    demo_full_reset(&ipc);
                break;

            /*-- Demo waveform selection (cpcu_tui --demo only) ---------*/
            case 'w': case 'W':
                /*  Cycle SINE(1) → ... → CHIRP(8) → SINE(1) */
                if(demo_mode)
                    demo_wave = (DemoWave)(((int)demo_wave % 8) + 1);
                break;
            case '[':
                if(demo_mode)
                {
                    demo_freq_hz *= 0.5f;
                    if(demo_freq_hz < 10.0f)   demo_freq_hz = 10.0f;
                }
                break;
            case ']':
                if(demo_mode)
                {
                    demo_freq_hz *= 2.0f;
                    if(demo_freq_hz > 1000.0f) demo_freq_hz = 1000.0f;
                }
                break;

            /*-- Page 7 (Config) edit-mode toggle ----------------*/
            /*  Pressing 'e' on the CONFIG page raises (or lowers) the
             *  edit_mode_request flag in IPC. cpcu_io watches that flag,
             *  parks the smoother at neutral, and once SMOOTH_AllSettled
             *  flips edit_mode_active to 1 — which the CONFIG renderer
             *  surfaces as a banner. The handshake protocol is fully
             *  documented in cpcu_v2/docs/EDIT_MODE.md.
             *
             *  Page-local: pressing 'e' from any other page does nothing
             *  (no global edit-mode action). The TUI's general principle
             *  is that destructive operations are scoped to the page
             *  that owns them. */
            case 'e': case 'E':
                if(current_page == PAGE_CONFIG)
                {
                    uint8_t cur = atomic_load_explicit(
                        &ipc.ctrl->edit_mode_request, memory_order_acquire);
                    uint8_t next = cur ? 0 : 1;
                    if(next)
                    {
                        atomic_store_explicit(
                            &ipc.ctrl->edit_mode_request_us,
                            now_ms_wall() * 1000ULL, memory_order_release);
                    }
                    atomic_store_explicit(
                        &ipc.ctrl->edit_mode_request, next,
                        memory_order_release);
                }
                break;

            case 'q': case 'Q': g_run = 0; break;
            default: break;
        }

after_dispatch:

        usleep(REFRESH_US);
    }

    /*-------------- Shutdown -----------------------------------------------------------------*/
    endwin();

    /*  Save anything that was being captured when the user quit — better
     *  than silently dropping the file. A partial capture is still useful
     *  if it ran for long enough.                                         */
    if(ds_state == DS_COLLECTING)
    {
        ds_stop_capture(true);
        printf("[TUI] Auto-saved in-progress capture: %s  (%u samples)\n",
               ds_path, ds_samples);
    }

    if(!demo_mode) IPC_Close(&ipc);
    printf("[TUI] Exited cleanly.\n");
    return 0;
}

/*==========================================================================================*/

