/**
 *  @file   cpcu_tui_editor.h
 *  @brief  TUI live editor API — field model, key handler, save protocol.
 */

#ifndef CPCU_TUI_EDITOR_H
#define CPCU_TUI_EDITOR_H

#include "cpcu_ipc.h"
#include "cpcu_config.h"
#include <stdbool.h>

/*============= EDITOR FIELD MODEL =========================================*/
/*  Each editable runtime.json key is one ED_Field. Scalar fields have
 *  count=1; per-servo arrays have count=6. The kind matters for save
 *  (different IPC fields are different types) and for range checking.
 */
typedef enum {
    ED_KIND_U16 = 0,    /* unsigned 16-bit, range usually 100..50000 */
    ED_KIND_I16,        /* signed 16-bit, used for servo_bias_us only */
    ED_KIND_U8,         /* small unsigned, percentages and vote counts */
} ED_FieldKind;

#define ED_MAX_CELLS    6           /* max array length we edit (per-servo) */

typedef struct {
    const char    *json_key;        /* "servo_min_us" — the key CFG_PatchFile uses */
    const char    *display_name;    /* "servo_min_us" — what the TUI shows */
    const char    *units;           /* "us", "us/s", "%", "ms", or "" */
    ED_FieldKind   kind;
    int            count;           /* 1 for scalar, 6 for per-servo */
    int            range_min;       /* validated on entry, clamped on save */
    int            range_max;
    int            draft[ED_MAX_CELLS];
    int            disk[ED_MAX_CELLS];
    bool           dirty[ED_MAX_CELLS];
} ED_Field;

/*============= EDITOR STATE ===============================================*/
typedef enum {
    ED_MODE_NAV = 0,    /* arrows move cursor */
    ED_MODE_ENTRY,      /* digits/Backspace modify selected cell */
} ED_Mode;

typedef struct {
    bool        initialized;        /* loaded from disk yet? */
    int         row;                /* index into g_ed_fields */
    int         col;                /* 0..count-1 within current field */
    ED_Mode     mode;
    /* When in ENTRY mode, this buffer holds the typed string (so we can
     * show partial values like "172" while typing toward "1720"). */
    char        entry_buf[16];
    int         entry_len;
    /* Status line for Ctrl+S / r / errors. */
    char        status_line[256];
    long        status_until_ms;    /* status fades after this wall-time ms */
    int         field_count;        /* size of g_ed_fields, set on init */
} ED_State;

/*============= API ========================================================*/
/* Load draft + disk values from runtime.json. Idempotent — safe to
 * call repeatedly; only re-loads if path changes or after a save.
 * Returns true on success, false if config can't be loaded (editor
 * still works on defaults but Ctrl+S will fail). */
bool ED_Init(void);

/* Drop drafts back to disk values (the 'r' = revert key). */
void ED_RevertAll(void);

/* True if any field has a dirty cell. */
bool ED_HasDirty(void);

/* Number of dirty cells across all fields. */
int  ED_DirtyCount(void);

/* Handle a key while in EDITING state. Returns true if the editor
 * consumed the key (so the page handler shouldn't process it further).
 * Pass the cpcu_kernel pid (read from IPC) for Ctrl+S signaling. */
bool ED_HandleKey(int ch, IPC_Context *ipc);

/* Render the editor to the screen starting at row r. Returns the
 * row index after the editor (so the page handler can stack things
 * underneath, or just use as height hint). */
int  ED_Render(int r);

/* Public field-table accessor for tests + introspection. */
const ED_Field *ED_GetField(int idx);
int             ED_GetFieldCount(void);

#endif /* CPCU_TUI_EDITOR_H */

