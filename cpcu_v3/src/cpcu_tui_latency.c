/* ═══════════════════════════════════════════════════════════════════
 *  cpcu_tui_render.c — v3.0 ADDITION
 *
 *  Add this function and call it from draw_page_dsp() (page 3).
 *  Insert the call after the existing inference rate / cmd age
 *  display, before the page footer.
 * ═══════════════════════════════════════════════════════════════════ */

/* draw a proportional bar: filled/total of width chars */
static void draw_latency_bar(int row, int col, uint32_t val_us,
                             uint32_t max_us, int width)
{
    int filled = (max_us > 0) ? (int)((float)val_us / max_us * width) : 0;
    if(filled > width) filled = width;
    for(int i = 0; i < width; i++)
    {
        if(i < filled)
            mvaddch(row, col + i, ACS_CKBOARD | COLOR_PAIR(CP_BAR_FILL));
        else
            mvaddch(row, col + i, ' ' | COLOR_PAIR(CP_BAR_EMPTY));
    }
}

static void draw_latency_breakdown(const IPC_Context *ipc, int start_row)
{
    /* read all latency fields (relaxed — display only) */
    uint32_t t_ring  = atomic_load(&ipc->latency->t_ring_dwell);
    uint32_t t_dsp   = atomic_load(&ipc->latency->t_dsp_compute);
    uint32_t t_vel   = atomic_load(&ipc->latency->t_velocity);
    uint32_t t_sm    = atomic_load(&ipc->latency->t_smoother);
    uint32_t t_win   = atomic_load(&ipc->latency->t_window_wait);
    uint32_t t_hyst  = atomic_load(&ipc->latency->t_hysteresis_ms);
    uint32_t t_proc  = t_ring + t_dsp + t_vel + t_sm;

    /* find max for bar scaling */
    uint32_t max_us = t_ring;
    if(t_dsp > max_us) max_us = t_dsp;
    if(t_vel > max_us) max_us = t_vel;
    if(t_sm  > max_us) max_us = t_sm;
    if(max_us == 0) max_us = 1;

    int r = start_row;
    int bar_w = 20;
    int val_col = 22;
    int bar_col = 38;

    draw_section(r, 1, "LATENCY BREAKDOWN (last window)");
    r++;
    draw_hline(r, 0, g_tui_w);
    r++;

    mvprintw(r, 1, "Ring dwell     :");
    mvprintw(r, val_col, "%6u us", t_ring);
    draw_latency_bar(r, bar_col, t_ring, max_us, bar_w);
    r++;

    mvprintw(r, 1, "DSP + ML       :");
    mvprintw(r, val_col, "%6u us", t_dsp);
    draw_latency_bar(r, bar_col, t_dsp, max_us, bar_w);
    r++;

    mvprintw(r, 1, "Velocity integ :");
    mvprintw(r, val_col, "%6u us", t_vel);
    draw_latency_bar(r, bar_col, t_vel, max_us, bar_w);
    r++;

    mvprintw(r, 1, "Smoother + I2C :");
    mvprintw(r, val_col, "%6u us", t_sm);
    draw_latency_bar(r, bar_col, t_sm, max_us, bar_w);
    r++;

    draw_hline(r, 0, g_tui_w);
    r++;

    mvprintw(r, 1, "Processing     :");
    mvprintw(r, val_col, "%6u us", t_proc);
    r++;

    mvprintw(r, 1, "Window wait    :");
    mvprintw(r, val_col, "%6u us  (avg)", t_win);
    r++;

    mvprintw(r, 1, "Hysteresis     :");
    mvprintw(r, val_col, "%6u ms  (%u votes)", t_hyst, t_hyst / 100);
    r++;

    draw_hline(r, 0, g_tui_w);
    r++;

    uint32_t e2e = t_proc / 1000 + t_win / 1000 + t_hyst;
    mvprintw(r, 1, "E2E estimate   :");
    attron(COLOR_PAIR(e2e > 300 ? CP_WARN : CP_GOOD));
    mvprintw(r, val_col, "%6u ms", e2e);
    attroff(COLOR_PAIR(e2e > 300 ? CP_WARN : CP_GOOD));
}

/* ── CALL SITE ──
 * In draw_page_dsp() (the page 3 renderer), find the end of the
 * existing content (after the inference/motor rates section) and add:
 *
 *   draw_latency_breakdown(&ipc, r);
 *
 * where 'r' is the current row counter.
 */
