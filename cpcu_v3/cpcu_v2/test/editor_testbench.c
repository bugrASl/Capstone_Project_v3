/**
 *  @file   editor_testbench.c
 *  @brief  TUI editor test harness — key dispatch, field navigation, save protocol.
 */

#include "cpcu_tui_editor.h"
#include "cpcu_ipc.h"
#include "cpcu_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>       /* mkdir */
#include <ncurses.h>        /* for KEY_UP, KEY_DOWN, etc. constants */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(NAME, COND, ...) do {                                 \
    if(COND) { printf("[PASS] %-12s ", NAME); printf(__VA_ARGS__); printf("\n"); g_pass++; } \
    else     { printf("[FAIL] %-12s ", NAME); printf(__VA_ARGS__); printf("\n"); g_fail++; } \
} while(0)

/* Minimal mock IPC context — kernel_pid=0 so save logs without trying to kill(). */
static IPC_ControlBlock  g_mock_ctrl = {0};
static IPC_Diagnostics   g_mock_diag = {0};
static IPC_Context       g_mock_ipc = {
    .ctrl  = &g_mock_ctrl,
    .diag  = &g_mock_diag,
};

/* Write a temp runtime.json so ED_Init has something to load. The
 * testbench keeps it under /tmp so we don't pollute the workspace.
 * Each call generates a fresh path — mkstemps requires the template
 * end in 6 'X' characters, so we reset the buffer here every time. */
static char g_tmp_path[64];
static char *write_runtime_json(void)
{
    /* mkstemps wants the suffix length. ".json" = 5 chars; before it
     * we need exactly 6 X's. Reset the template every call so a
     * subsequent invocation doesn't see the previous run's filled-in
     * pattern. */
    strcpy(g_tmp_path, "/tmp/editor_test_runtimeXXXXXX.json");
    int fd = mkstemps(g_tmp_path, 5);
    if(fd < 0) { perror("mkstemps"); exit(1); }
    const char *content =
        "{\n"
        "  \"schema_version\": 1,\n"
        "  \"servo_min_us\": [498, 1074, 1074, 1001, 1001, 976],\n"
        "  \"servo_max_us\": [2500, 1953, 1953, 2002, 2002, 1733],\n"
        "  \"servo_bias_us\": [0, 0, 0, 0, 0, 0],\n"
        "  \"smooth_velocity_us_per_s\": [2000, 2000, 2000, 2000, 2000, 1200],\n"
        "  \"smooth_accel_us_per_s2\":   [8000, 8000, 8000, 8000, 8000, 8000],\n"
        "  \"smooth_deadband_us\":       [10, 10, 10, 10, 10, 10],\n"
        "  \"interp_conf_floor_pct\": 40,\n"
        "  \"interp_conf_ceil_pct\":  85,\n"
        "  \"hysteresis_votes\": 3,\n"
        "  \"grip_open_us\":  1700,\n"
        "  \"grip_touch_us\": 1200,\n"
        "  \"grip_firm_us\":  1100,\n"
        "  \"grip_stall_recover_ms\": 2000\n"
        "}\n";
    write(fd, content, strlen(content));
    close(fd);
    return g_tmp_path;
}

/* Patch the editor's path search to find our temp file. The editor's
 * lookup order is /opt/cpcu/config.json first, then config/runtime.json
 * in the cwd. If /opt/cpcu/config.json exists (it does on a real Pi
 * after `./launch.sh setup`), the first path always wins, and our
 * symlink-in-cwd trick never gets a chance.
 *
 * Solution: temporarily rename /opt/cpcu/config.json to a sibling
 * .test_backup name for the duration of the test, then restore on
 * exit. Combined with a config/runtime.json symlink in the cwd
 * pointing at our temp file, this guarantees the editor reads what
 * the test wrote.
 */
static const char *g_opt_cfg_path        = "/opt/cpcu/config.json";
static const char *g_opt_cfg_backup_path = "/opt/cpcu/config.json.test_backup";
static int         g_opt_cfg_was_moved   = 0;

static void restore_opt_cpcu_config(void)
{
    if(g_opt_cfg_was_moved)
    {
        if(rename(g_opt_cfg_backup_path, g_opt_cfg_path) != 0)
            fprintf(stderr,
                "WARNING: couldn't restore %s from %s — please check manually\n",
                g_opt_cfg_path, g_opt_cfg_backup_path);
        g_opt_cfg_was_moved = 0;
    }
}

static void install_runtime_json(const char *src)
{
    /* Move /opt/cpcu/config.json out of the way if it exists. The
     * symlink target stays put — we only rename the symlink itself. */
    if(access(g_opt_cfg_path, F_OK) == 0 && !g_opt_cfg_was_moved)
    {
        if(rename(g_opt_cfg_path, g_opt_cfg_backup_path) == 0)
        {
            g_opt_cfg_was_moved = 1;
            atexit(restore_opt_cpcu_config);
        }
        else
        {
            fprintf(stderr,
                "WARNING: couldn't move %s aside (errno=%d). The editor will\n"
                "         load production config and tests may behave oddly.\n"
                "         Try running the testbench with sufficient permissions\n"
                "         on /opt/cpcu/, or remove the symlink temporarily.\n",
                g_opt_cfg_path, errno);
        }
    }

    /* Build a config/runtime.json in CWD as the second-tier lookup. */
    int rc = mkdir("config", 0755);
    (void)rc;       /* ok if exists */
    unlink("config/runtime.json");
    rc = symlink(src, "config/runtime.json");
    if(rc != 0) { perror("symlink"); exit(1); }
}

/*============= TB-ED01 : Init loads disk values ===========================*/

static void test_init_loads(void)
{
    printf("\n--- TB-ED01: ED_Init() loads disk values ---\n");
    install_runtime_json(write_runtime_json());
    bool ok = ED_Init();
    CHECK("ED01a", ok, "ED_Init returned true");
    CHECK("ED01b", ED_GetFieldCount() >= 13,
          "field count >= 13 (got %d)", ED_GetFieldCount());

    /* Find servo_min_us and check first cell */
    bool found = false;
    for(int i = 0; i < ED_GetFieldCount(); i++)
    {
        const ED_Field *f = ED_GetField(i);
        if(strcmp(f->json_key, "servo_min_us") == 0)
        {
            CHECK("ED01c", f->disk[0] == 498,
                  "servo_min_us[0] disk=%d expected 498", f->disk[0]);
            CHECK("ED01d", f->draft[0] == 498,
                  "draft mirrors disk on load: draft=%d", f->draft[0]);
            CHECK("ED01e", !f->dirty[0],
                  "fresh load has no dirty cells");
            found = true;
            break;
        }
    }
    CHECK("ED01f", found, "servo_min_us field present in table");
    CHECK("ED01g", ED_DirtyCount() == 0, "no dirty after init");
}

/*============= TB-ED02 : Navigation ========================================*/

static void test_navigation(void)
{
    printf("\n--- TB-ED02: NAV-mode arrow movement ---\n");
    ED_Init();
    /* Start at row 0, col 0. KEY_DOWN should move to row 1 col 0. */
    ED_HandleKey(KEY_DOWN, &g_mock_ipc);
    /* No public accessor for cursor — but we can infer indirectly:
     * pressing Enter on row 1 then a digit then Enter should mark
     * row-1 col-0 dirty, not row-0. */
    ED_HandleKey('\n', &g_mock_ipc);
    ED_HandleKey('1', &g_mock_ipc);
    ED_HandleKey('2', &g_mock_ipc);
    ED_HandleKey('3', &g_mock_ipc);
    ED_HandleKey('4', &g_mock_ipc);
    ED_HandleKey('\n', &g_mock_ipc);
    /* Field 0 should still be clean, field 1 should have dirty[0]. */
    const ED_Field *f0 = ED_GetField(0);
    const ED_Field *f1 = ED_GetField(1);
    CHECK("ED02a", !f0->dirty[0], "row-0 unchanged after navigating to row-1");
    /* The value 1234 may have been clamped; just check dirty. */
    CHECK("ED02b", f1->dirty[0], "row-1 col-0 became dirty");
    ED_RevertAll();
    CHECK("ED02c", ED_DirtyCount() == 0, "revert clears all dirty");
}

/*============= TB-ED03 : Range clamping ===================================*/

static void test_range_clamping(void)
{
    printf("\n--- TB-ED03: out-of-range entry gets clamped ---\n");
    ED_Init();
    /* Find servo_bias_us (range -100..+100) and try to set 9999. */
    int bias_idx = -1;
    for(int i = 0; i < ED_GetFieldCount(); i++)
        if(strcmp(ED_GetField(i)->json_key, "servo_bias_us") == 0)
        { bias_idx = i; break; }
    CHECK("ED03a", bias_idx >= 0, "found servo_bias_us at idx=%d", bias_idx);

    /* Navigate down to bias_idx */
    ED_Init();   /* reset cursor */
    for(int i = 0; i < bias_idx; i++)
        ED_HandleKey(KEY_DOWN, &g_mock_ipc);
    ED_HandleKey('\n', &g_mock_ipc);
    ED_HandleKey('9', &g_mock_ipc);
    ED_HandleKey('9', &g_mock_ipc);
    ED_HandleKey('9', &g_mock_ipc);
    ED_HandleKey('9', &g_mock_ipc);
    ED_HandleKey('\n', &g_mock_ipc);

    const ED_Field *bias = ED_GetField(bias_idx);
    CHECK("ED03b", bias->draft[0] == 100,
          "9999 clamped to range_max=100 (got %d)", bias->draft[0]);
    CHECK("ED03c", bias->dirty[0], "value differs from disk -> dirty");

    /* Negative side */
    ED_Init();
    for(int i = 0; i < bias_idx; i++)
        ED_HandleKey(KEY_DOWN, &g_mock_ipc);
    ED_HandleKey('\n', &g_mock_ipc);
    ED_HandleKey('-', &g_mock_ipc);
    ED_HandleKey('5', &g_mock_ipc);
    ED_HandleKey('0', &g_mock_ipc);
    ED_HandleKey('0', &g_mock_ipc);
    ED_HandleKey('\n', &g_mock_ipc);
    bias = ED_GetField(bias_idx);
    CHECK("ED03d", bias->draft[0] == -100,
          "-500 clamped to range_min=-100 (got %d)", bias->draft[0]);
}

/*============= TB-ED04 : Esc cancels entry =================================*/

static void test_esc_cancels(void)
{
    printf("\n--- TB-ED04: Esc cancels in-flight entry ---\n");
    ED_Init();
    int orig = ED_GetField(0)->draft[0];
    ED_HandleKey('\n', &g_mock_ipc);    /* enter ENTRY mode */
    ED_HandleKey('1', &g_mock_ipc);
    ED_HandleKey('2', &g_mock_ipc);
    ED_HandleKey('3', &g_mock_ipc);
    ED_HandleKey(27, &g_mock_ipc);      /* Esc */
    CHECK("ED04a", ED_GetField(0)->draft[0] == orig,
          "Esc preserved original value (still %d)", ED_GetField(0)->draft[0]);
    CHECK("ED04b", !ED_GetField(0)->dirty[0],
          "Esc didn't mark dirty");
}

/*============= TB-ED05 : Save round-trip ===================================*/

static void test_save_roundtrip(void)
{
    printf("\n--- TB-ED05: Ctrl+S round-trip via CFG_PatchFile ---\n");
    /* Fresh runtime.json so we know the starting state. */
    install_runtime_json(write_runtime_json());
    ED_Init();

    /* Find smooth_velocity_us_per_s and dirty cell 0. */
    int vel_idx = -1;
    for(int i = 0; i < ED_GetFieldCount(); i++)
        if(strcmp(ED_GetField(i)->json_key, "smooth_velocity_us_per_s") == 0)
        { vel_idx = i; break; }
    CHECK("ED05a", vel_idx >= 0, "found smooth_velocity field");

    /* Navigate down to the field. ED_Init resets cursor to (0, 0). */
    for(int i = 0; i < vel_idx; i++)
        ED_HandleKey(KEY_DOWN, &g_mock_ipc);
    /* Set cell 0 to 1500. */
    ED_HandleKey('\n', &g_mock_ipc);
    ED_HandleKey('1', &g_mock_ipc);
    ED_HandleKey('5', &g_mock_ipc);
    ED_HandleKey('0', &g_mock_ipc);
    ED_HandleKey('0', &g_mock_ipc);
    ED_HandleKey('\n', &g_mock_ipc);
    CHECK("ED05b", ED_GetField(vel_idx)->dirty[0],
          "vel[0] is dirty after entry");
    CHECK("ED05c", ED_GetField(vel_idx)->draft[0] == 1500,
          "vel[0] draft = 1500");

    /* Trigger save. kernel_pid in mock is 0, so SIGHUP is skipped
     * but the file write still happens. */
    ED_HandleKey(19 /* Ctrl+S */, &g_mock_ipc);
    CHECK("ED05d", !ED_GetField(vel_idx)->dirty[0],
          "save cleared dirty");
    CHECK("ED05e", ED_DirtyCount() == 0,
          "no dirty fields after save");

    /* Reload from disk and confirm the value persisted. We do this
     * by re-parsing the same file with CFG_LoadFromFile (the editor's
     * ED_Init also reloads, but bypassing it gives a more direct
     * round-trip check). */
    IPC_RuntimeConfig reloaded;
    char err[256] = {0};
    CFG_Status st = CFG_LoadFromFile("config/runtime.json", &reloaded,
                                     err, sizeof(err));
    CHECK("ED05f", st == CFG_OK, "reloaded file: %s",
          st == CFG_OK ? "OK" : err);
    CHECK("ED05g", reloaded.smooth_velocity_us_per_s[0] == 1500,
          "disk value after save = %u (wanted 1500)",
          reloaded.smooth_velocity_us_per_s[0]);

    /* Sanity: untouched cell preserved. Cell 5 was 1200 in our
     * template (gripper preset). After our save touching cell 0, the
     * patch rewrote the whole array, so cell 5 must remain 1200 from
     * draft[5] (which equaled disk[5] = 1200). */
    CHECK("ED05h", reloaded.smooth_velocity_us_per_s[5] == 1200,
          "untouched cell 5 = %u (wanted 1200)",
          reloaded.smooth_velocity_us_per_s[5]);
}

/*============= MAIN ========================================================*/

int main(void)
{
    printf("======================================\n");
    printf("  TB-ED — cpcu_tui_editor unit tests \n");
    printf("======================================\n");

    test_init_loads();
    test_navigation();
    test_range_clamping();
    test_esc_cancels();
    test_save_roundtrip();

    /* Cleanup */
    unlink(g_tmp_path);
    unlink("config/runtime.json");
    rmdir("config");

    printf("\n======================================\n");
    printf("RESULTS: %d PASS, %d FAIL\n", g_pass, g_fail);
    printf("======================================\n");
    return (g_fail == 0) ? 0 : 1;
}

