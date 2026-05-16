/**
 *  @file   cpcu_tui_editor.c
 *  @brief  TUI live editor — in-system runtime config tuning on the CONFIG page.
 *
 *  Two-mode state machine (BROWSING / EDITING) with keyboard navigation.
 *  In EDITING mode, the arm is parked at neutral via the edit-mode handshake.
 *  On Ctrl+S: validates input, writes runtime.json via CFG_PatchFile(), then
 *  signals the kernel (SIGHUP) to reload and republish IPC_RuntimeConfig.
 */

#include "cpcu_tui_editor.h"
#include "cpcu_pca9685.h"           /* PCA_SERVO_COUNT */
#include "cpcu_log.h"

#include <ncurses.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#define ED_NCELLS_PER_SERVO     PCA_SERVO_COUNT     /* 6 */
#define ED_MAX_FIELDS           32

/*============= FIELD TABLE ================================================*/
/* The editor surfaces a curated subset of runtime.json. Per-servo
 * arrays come first (visually grouped), then scalar fields by topic.
 *
 * gesture_velocity is intentionally NOT in this table — it's a
 * string-keyed dict that only cpcu_dsp.py reads, and dsp loads it
 * once at startup. Editing it from the TUI without restarting dsp
 * would silently fail. Documented in TUI_EDITOR.md §6.
 *
 * Array is sized from the initializer (no fixed bound) so adding
 * a row only requires editing this list. ED_MAX_FIELDS is just a
 * safety cap on the patch-buffer allocation in ed_save().
 */
static ED_Field g_ed_fields[] = {
    /* SERVO LIMITS (per-servo, us) */
    { "servo_min_us",  "servo_min_us",  "us",      ED_KIND_U16, 6, 100, 2900, {0}, {0}, {0} },
    { "servo_max_us",  "servo_max_us",  "us",      ED_KIND_U16, 6, 100, 2900, {0}, {0}, {0} },
    { "servo_bias_us", "servo_bias_us", "us",      ED_KIND_I16, 6, -100, 100, {0}, {0}, {0} },
    /* SMOOTHER (per-servo) */
    { "smooth_velocity_us_per_s", "smooth_velocity",  "us/s",  ED_KIND_U16, 6, 100, 10000, {0}, {0}, {0} },
    { "smooth_accel_us_per_s2",   "smooth_accel",     "us/s2", ED_KIND_U16, 6, 500, 50000, {0}, {0}, {0} },
    { "smooth_deadband_us",       "smooth_deadband",  "us",    ED_KIND_U16, 6, 0,   50,    {0}, {0}, {0} },
    /* DSP THRESHOLDS (scalar) */
    { "interp_conf_floor_pct", "interp_floor_pct",  "%",  ED_KIND_U8, 1, 0,  100, {0}, {0}, {0} },
    { "interp_conf_ceil_pct",  "interp_ceil_pct",   "%",  ED_KIND_U8, 1, 0,  100, {0}, {0}, {0} },
    { "hysteresis_votes",      "hysteresis_votes",  "",   ED_KIND_U8, 1, 1,   20, {0}, {0}, {0} },
    /* GRIP LEVELS (scalar) */
    { "grip_open_us",          "grip_open_us",         "us", ED_KIND_U16, 1, 800,  2200,  {0}, {0}, {0} },
    { "grip_touch_us",         "grip_touch_us",        "us", ED_KIND_U16, 1, 800,  2200,  {0}, {0}, {0} },
    { "grip_firm_us",          "grip_firm_us",         "us", ED_KIND_U16, 1, 800,  2200,  {0}, {0}, {0} },
    { "grip_stall_recover_ms", "grip_stall_recover",   "ms", ED_KIND_U16, 1, 100,  30000, {0}, {0}, {0} },
};
static const int g_ed_field_count =
    (int)(sizeof(g_ed_fields) / sizeof(g_ed_fields[0]));

/* Safety: ed_save allocates patch buffers sized at ED_MAX_FIELDS.
 * If someone adds enough fields to exceed that, fail at compile time
 * rather than silently overflowing the stack. */
_Static_assert(sizeof(g_ed_fields) / sizeof(g_ed_fields[0]) <= ED_MAX_FIELDS,
               "g_ed_fields outgrew ED_MAX_FIELDS — bump it in cpcu_tui_editor.c");

static ED_State g_ed = {
    .initialized = false,
    .row = 0,
    .col = 0,
    .mode = ED_MODE_NAV,
    .entry_buf = {0},
    .entry_len = 0,
    .status_line = {0},
    .status_until_ms = 0,
    .field_count = 0,
};

/*============= UTILITIES ==================================================*/
static long now_wall_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (long)ts.tv_sec * 1000L + (long)(ts.tv_nsec / 1000000L);
}

static void ed_set_status(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_ed.status_line, sizeof(g_ed.status_line), fmt, ap);
    va_end(ap);
    g_ed.status_until_ms = now_wall_ms() + 4000;
}

/* Pull values for one field from the parsed runtime config. */
static void ed_field_load_from_cfg(ED_Field *f, const IPC_RuntimeConfig *cfg)
{
    if(strcmp(f->json_key, "servo_min_us") == 0)
        for(int i = 0; i < 6; i++) f->disk[i] = cfg->servo_min_us[i];
    else if(strcmp(f->json_key, "servo_max_us") == 0)
        for(int i = 0; i < 6; i++) f->disk[i] = cfg->servo_max_us[i];
    else if(strcmp(f->json_key, "servo_bias_us") == 0)
        for(int i = 0; i < 6; i++) f->disk[i] = cfg->servo_bias_us[i];
    else if(strcmp(f->json_key, "smooth_velocity_us_per_s") == 0)
        for(int i = 0; i < 6; i++) f->disk[i] = cfg->smooth_velocity_us_per_s[i];
    else if(strcmp(f->json_key, "smooth_accel_us_per_s2") == 0)
        for(int i = 0; i < 6; i++) f->disk[i] = cfg->smooth_accel_us_per_s2[i];
    else if(strcmp(f->json_key, "smooth_deadband_us") == 0)
        for(int i = 0; i < 6; i++) f->disk[i] = cfg->smooth_deadband_us[i];
    else if(strcmp(f->json_key, "interp_conf_floor_pct") == 0)
        f->disk[0] = cfg->interp_conf_floor_pct;
    else if(strcmp(f->json_key, "interp_conf_ceil_pct") == 0)
        f->disk[0] = cfg->interp_conf_ceil_pct;
    else if(strcmp(f->json_key, "hysteresis_votes") == 0)
        f->disk[0] = cfg->hysteresis_votes;
    else if(strcmp(f->json_key, "grip_open_us") == 0)
        f->disk[0] = cfg->grip_open_us;
    else if(strcmp(f->json_key, "grip_touch_us") == 0)
        f->disk[0] = cfg->grip_touch_us;
    else if(strcmp(f->json_key, "grip_firm_us") == 0)
        f->disk[0] = cfg->grip_firm_us;
    else if(strcmp(f->json_key, "grip_stall_recover_ms") == 0)
        f->disk[0] = cfg->grip_stall_recover_ms;
    /* draft mirrors disk on load; dirty array zeroed. */
    for(int i = 0; i < f->count; i++)
    {
        f->draft[i] = f->disk[i];
        f->dirty[i] = false;
    }
}

/*============= INIT =======================================================*/
bool ED_Init(void)
{
    g_ed.field_count = g_ed_field_count;

    /* Two-tier path search matches cpcu_kernel and pca_testbench. */
    const char *paths[] = {
        "/opt/cpcu/config.json",
        "config/runtime.json",
        NULL
    };
    IPC_RuntimeConfig cfg;
    char err[256] = {0};
    bool loaded = false;
    for(int p = 0; paths[p]; p++)
    {
        if(CFG_LoadFromFile(paths[p], &cfg, err, sizeof(err)) == CFG_OK)
        {
            for(int f = 0; f < g_ed_field_count; f++)
                ed_field_load_from_cfg(&g_ed_fields[f], &cfg);
            ed_set_status("loaded from %s", paths[p]);
            loaded = true;
            break;
        }
    }
    if(!loaded)
    {
        /* Fall back to compile-time defaults — editor still navigable
         * but Ctrl+S won't have a target file. */
        IPC_RuntimeConfig def;
        CFG_Defaults(&def);
        for(int f = 0; f < g_ed_field_count; f++)
            ed_field_load_from_cfg(&g_ed_fields[f], &def);
        ed_set_status("WARNING: no runtime.json found, defaults loaded "
                      "(Ctrl+S will fail until file exists)");
    }
    g_ed.initialized = true;
    g_ed.row = 0;
    g_ed.col = 0;
    g_ed.mode = ED_MODE_NAV;
    return loaded;
}

/*============= MUTATION ===================================================*/
/* Apply a new value to draft, mark dirty if it differs from disk,
 * clamp to range. Returns the (possibly clamped) value actually stored. */
static int ed_set_draft(ED_Field *f, int col, int new_val)
{
    if(new_val < f->range_min) new_val = f->range_min;
    if(new_val > f->range_max) new_val = f->range_max;
    f->draft[col] = new_val;
    f->dirty[col] = (f->draft[col] != f->disk[col]);
    return new_val;
}

void ED_RevertAll(void)
{
    int reverted = 0;
    for(int f = 0; f < g_ed_field_count; f++)
    {
        ED_Field *fld = &g_ed_fields[f];
        for(int i = 0; i < fld->count; i++)
        {
            if(fld->dirty[i])
            {
                fld->draft[i] = fld->disk[i];
                fld->dirty[i] = false;
                reverted++;
            }
        }
    }
    ed_set_status("reverted %d cell%s", reverted, reverted == 1 ? "" : "s");
}

bool ED_HasDirty(void)
{
    for(int f = 0; f < g_ed_field_count; f++)
        for(int i = 0; i < g_ed_fields[f].count; i++)
            if(g_ed_fields[f].dirty[i]) return true;
    return false;
}

int ED_DirtyCount(void)
{
    int n = 0;
    for(int f = 0; f < g_ed_field_count; f++)
        for(int i = 0; i < g_ed_fields[f].count; i++)
            if(g_ed_fields[f].dirty[i]) n++;
    return n;
}

/*============= SAVE =======================================================*/
/* Build a CFG_PatchEntry list from dirty fields. Patcher operates on
 * int16; we cast U16 down (safe, all our ranges fit) and U8 up. */
static bool ed_save(IPC_Context *ipc)
{
    /* Determine target file. Same two-tier search as ED_Init. */
    const char *target = NULL;
    if(access("/opt/cpcu/config.json", W_OK) == 0)
        target = "/opt/cpcu/config.json";
    else if(access("config/runtime.json", W_OK) == 0)
        target = "config/runtime.json";
    else
    {
        ed_set_status("SAVE FAILED: no writable runtime.json found");
        return false;
    }

    /* Each field that has *any* dirty cell becomes one patch entry —
     * CFG_PatchFile rewrites the whole array in place. We allocate
     * the int16 buffers on the stack; one per dirty field, so up to
     * g_ed_field_count entries × 6 cells = 13×6 = 78 int16s = 156 B.
     * Comfortably stack-able. */
    int16_t bufs[ED_MAX_FIELDS][ED_MAX_CELLS];
    CFG_PatchEntry patches[ED_MAX_FIELDS];
    int n_patches = 0;

    for(int f = 0; f < g_ed_field_count; f++)
    {
        ED_Field *fld = &g_ed_fields[f];
        bool any_dirty = false;
        for(int i = 0; i < fld->count; i++)
            if(fld->dirty[i]) { any_dirty = true; break; }
        if(!any_dirty) continue;

        for(int i = 0; i < fld->count; i++)
            bufs[n_patches][i] = (int16_t)fld->draft[i];
        patches[n_patches].key    = fld->json_key;
        patches[n_patches].values = bufs[n_patches];
        patches[n_patches].count  = (size_t)fld->count;
        n_patches++;
    }

    if(n_patches == 0)
    {
        ed_set_status("nothing to save (no dirty fields)");
        return true;
    }

    char err[256] = {0};
    CFG_Status st = CFG_PatchFile(target, patches, (size_t)n_patches,
                                  err, sizeof(err));
    if(st != CFG_OK)
    {
        ed_set_status("SAVE FAILED: %s (%s)", CFG_StatusStr(st), err);
        return false;
    }

    /* Promote draft -> disk and clear dirty. The kernel will SIGHUP-
     * reparse and republish IPC_RuntimeConfig within ~20 ms. */
    for(int f = 0; f < g_ed_field_count; f++)
    {
        ED_Field *fld = &g_ed_fields[f];
        for(int i = 0; i < fld->count; i++)
        {
            if(fld->dirty[i])
            {
                fld->disk[i] = fld->draft[i];
                fld->dirty[i] = false;
            }
        }
    }

    /* Send SIGHUP to cpcu_kernel so it re-parses the file. */
    uint32_t kpid = atomic_load(&ipc->ctrl->kernel_pid);
    if(kpid > 0)
    {
        /* Guard against kill(0, ...) which means "my process group" in
         * POSIX. We only want to signal the kernel, never ourselves. */
        if(kill((pid_t)kpid, SIGHUP) == 0)
        {
            ed_set_status("saved %d patch%s to %s -- SIGHUP'd kernel pid %u",
                          n_patches, n_patches == 1 ? "" : "es",
                          target, (unsigned)kpid);
        }
        else
        {
            ed_set_status("saved to %s but SIGHUP failed (errno %d) -- "
                          "kernel may need manual reload",
                          target, errno);
        }
    }
    else
    {
        ed_set_status("saved %d patch%s to %s; kernel_pid=0, run "
                      "`kill -HUP $(pgrep cpcu_kernel)` to reload",
                      n_patches, n_patches == 1 ? "" : "es", target);
    }
    return true;
}

/*============= ENTRY-MODE HELPERS =========================================*/
static void ed_entry_begin(void)
{
    g_ed.mode = ED_MODE_ENTRY;
    g_ed.entry_len = 0;
    g_ed.entry_buf[0] = '\0';
}

static void ed_entry_commit(void)
{
    /* Empty buffer = treat as Esc (no change). */
    if(g_ed.entry_len == 0)
    {
        g_ed.mode = ED_MODE_NAV;
        return;
    }
    int v = atoi(g_ed.entry_buf);
    ED_Field *f = &g_ed_fields[g_ed.row];
    int clamped = ed_set_draft(f, g_ed.col, v);
    if(clamped != v)
    {
        ed_set_status("'%s' clamped to range [%d..%d] -> %d",
                      g_ed.entry_buf, f->range_min, f->range_max, clamped);
    }
    g_ed.mode = ED_MODE_NAV;
}

static void ed_entry_cancel(void)
{
    g_ed.mode = ED_MODE_NAV;
    g_ed.entry_len = 0;
}

/*============= KEY HANDLER ================================================*/
bool ED_HandleKey(int ch, IPC_Context *ipc)
{
    if(!g_ed.initialized) return false;

    /* ENTRY mode: digits, sign, backspace, Enter, Esc — that's it. */
    if(g_ed.mode == ED_MODE_ENTRY)
    {
        ED_Field *f = &g_ed_fields[g_ed.row];
        if(ch == '\n' || ch == KEY_ENTER || ch == 13)
        {
            ed_entry_commit();
            return true;
        }
        if(ch == 27 /*Esc*/)
        {
            ed_entry_cancel();
            return true;
        }
        if(ch == KEY_BACKSPACE || ch == 127 || ch == 8)
        {
            if(g_ed.entry_len > 0)
            {
                g_ed.entry_len--;
                g_ed.entry_buf[g_ed.entry_len] = '\0';
            }
            return true;
        }
        /* Digit. Accept only if buffer has space. */
        if(isdigit(ch) && g_ed.entry_len < (int)sizeof(g_ed.entry_buf) - 1)
        {
            g_ed.entry_buf[g_ed.entry_len++] = (char)ch;
            g_ed.entry_buf[g_ed.entry_len] = '\0';
            return true;
        }
        /* Minus sign — only valid for I16 fields, only at position 0. */
        if(ch == '-' && f->kind == ED_KIND_I16 && g_ed.entry_len == 0)
        {
            g_ed.entry_buf[g_ed.entry_len++] = '-';
            g_ed.entry_buf[g_ed.entry_len] = '\0';
            return true;
        }
        /* Anything else: silently ignore so the user can't accidentally
         * navigate while editing. */
        return true;
    }

    /* NAV mode. */
    switch(ch)
    {
        case KEY_UP:
            if(g_ed.row > 0) g_ed.row--;
            /* Don't let col stay past the new row's count. */
            if(g_ed.col >= g_ed_fields[g_ed.row].count)
                g_ed.col = g_ed_fields[g_ed.row].count - 1;
            return true;
        case KEY_DOWN:
            if(g_ed.row < g_ed_field_count - 1) g_ed.row++;
            if(g_ed.col >= g_ed_fields[g_ed.row].count)
                g_ed.col = g_ed_fields[g_ed.row].count - 1;
            return true;
        case KEY_LEFT:
            if(g_ed.col > 0) g_ed.col--;
            return true;
        case KEY_RIGHT:
            if(g_ed.col < g_ed_fields[g_ed.row].count - 1) g_ed.col++;
            return true;
        case '\n':
        case KEY_ENTER:
        case 13:
            ed_entry_begin();
            return true;
        case 'r':
        case 'R':
            ED_RevertAll();
            return true;
        case 19:        /* Ctrl+S */
            ed_save(ipc);
            return true;
        default:
            return false;       /* let page handler see other keys (e, etc.) */
    }
}

/*============= RENDER =====================================================*/
static const char *SERVO_HEADERS[6] = { "S0", "S1", "S2", "S3", "S4", "S5" };

/* Color helpers — we expect CP_GOOD/CP_WARN/CP_BAD/CP_CYAN/CP_DIM to
 * be defined globally by the TUI's main color setup. Fall back to
 * default pair (0) if not. */
#ifndef CP_GOOD
#define CP_GOOD 2
#endif
#ifndef CP_WARN
#define CP_WARN 3
#endif
#ifndef CP_BAD
#define CP_BAD  4
#endif
#ifndef CP_CYAN
#define CP_CYAN 5
#endif
#ifndef CP_DIM
#define CP_DIM  6
#endif

int ED_Render(int r)
{
    /* Header row showing column labels for per-servo arrays. */
    attron(A_BOLD);
    mvprintw(r, 1, "%-22s", "FIELD");
    for(int c = 0; c < 6; c++)
    {
        mvprintw(r, 24 + c * 9, "%6s", SERVO_HEADERS[c]);
    }
    mvprintw(r, 24 + 6 * 9 + 2, "%-8s", "UNITS");
    attroff(A_BOLD);
    r++;
    mvhline(r, 0, ACS_HLINE, 24 + 6 * 9 + 14);
    r++;

    /* Field rows. */
    for(int f = 0; f < g_ed_field_count; f++)
    {
        ED_Field *fld = &g_ed_fields[f];
        bool selected_row = (f == g_ed.row);

        /* Label */
        if(selected_row && g_ed.mode == ED_MODE_NAV)
            attron(A_REVERSE);
        mvprintw(r, 1, "%-22s", fld->display_name);
        if(selected_row && g_ed.mode == ED_MODE_NAV)
            attroff(A_REVERSE);

        /* Cells */
        for(int c = 0; c < fld->count; c++)
        {
            bool selected_cell = selected_row && (c == g_ed.col);
            bool dirty = fld->dirty[c];

            int x = 24 + c * 9;
            char cellbuf[16];
            if(selected_cell && g_ed.mode == ED_MODE_ENTRY)
            {
                /* Show the entry buffer with a trailing underscore as
                 * cursor hint. Bound to 14 chars so the `_` and null
                 * always fit in the 16-byte cellbuf. */
                snprintf(cellbuf, sizeof(cellbuf), "%.14s_", g_ed.entry_buf);
            }
            else
            {
                snprintf(cellbuf, sizeof(cellbuf), "%d", fld->draft[c]);
            }

            int attr = 0;
            if(selected_cell && g_ed.mode == ED_MODE_ENTRY) attr |= A_REVERSE | A_BOLD;
            else if(selected_cell)                          attr |= A_REVERSE;
            else if(dirty)                                  attr |= A_BOLD;

            int color = dirty ? CP_WARN : CP_CYAN;

            attron(COLOR_PAIR(color) | attr);
            mvprintw(r, x, "%6s", cellbuf);
            attroff(COLOR_PAIR(color) | attr);

            /* Tiny dirty marker after the cell */
            if(dirty)
            {
                attron(COLOR_PAIR(CP_WARN));
                mvaddch(r, x + 7, '*');
                attroff(COLOR_PAIR(CP_WARN));
            }
        }

        /* Units column */
        attron(COLOR_PAIR(CP_DIM));
        mvprintw(r, 24 + 6 * 9 + 2, "%-8s", fld->units);
        attroff(COLOR_PAIR(CP_DIM));

        r++;
    }

    r++;
    mvhline(r - 1, 0, ACS_HLINE, 24 + 6 * 9 + 14);

    /* Footer: cursor pos, mode, dirty count. */
    int dirty = ED_DirtyCount();
    attron(COLOR_PAIR(CP_DIM));
    if(g_ed.mode == ED_MODE_ENTRY)
    {
        mvprintw(r, 1, "ENTRY  field=%s[%d]  buffer='%s'  Enter=commit  "
                       "Esc=cancel",
                 g_ed_fields[g_ed.row].display_name, g_ed.col,
                 g_ed.entry_buf);
    }
    else
    {
        mvprintw(r, 1, "NAV    arrows=move  Enter=edit  r=revert all  "
                       "Ctrl+S=save  dirty=%d", dirty);
    }
    attroff(COLOR_PAIR(CP_DIM));
    r++;

    /* Status line. */
    if(g_ed.status_line[0] && now_wall_ms() < g_ed.status_until_ms)
    {
        attron(COLOR_PAIR(CP_GOOD) | A_BOLD);
        mvprintw(r, 1, "%-78s", g_ed.status_line);
        attroff(COLOR_PAIR(CP_GOOD) | A_BOLD);
        r++;
    }
    else if(dirty > 0)
    {
        attron(COLOR_PAIR(CP_WARN));
        mvprintw(r, 1, "* %d unsaved change%s -- press Ctrl+S to commit, "
                       "r to revert *", dirty, dirty == 1 ? "" : "s");
        attroff(COLOR_PAIR(CP_WARN));
        r++;
    }

    return r;
}

/*============= ACCESSORS ==================================================*/
const ED_Field *ED_GetField(int idx)
{
    if(idx < 0 || idx >= g_ed_field_count) return NULL;
    return &g_ed_fields[idx];
}

int ED_GetFieldCount(void)
{
    return g_ed_field_count;
}

