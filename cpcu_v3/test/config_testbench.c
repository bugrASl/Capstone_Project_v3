/**
 *  @file   config_testbench.c
 *  @brief  Config loader test harness — JSON parse, validation, defaults, patching.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include "cpcu_config.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(test_id, descr, cond, ...) do { \
    if(cond) { g_pass++; printf("[PASS] %-12s %s\n", test_id, descr); } \
    else     { g_fail++; printf("[FAIL] %-12s %s  ", test_id, descr); \
               printf(__VA_ARGS__); printf("\n"); } \
} while(0)

/* Write contents to a temp file. Return path (caller must free). */
static char *write_temp(const char *contents)
{
    static char path[256];
    snprintf(path, sizeof(path), "/tmp/cpcu_cfg_test_%d.json", (int)getpid());
    FILE *f = fopen(path, "w");
    if(!f) return NULL;
    fputs(contents, f);
    fclose(f);
    return path;
}

/*============= TB-CFG01 : Valid full file =================================*/

static void test_valid_full(void)
{
    const char *json =
        "{\n"
        "  \"schema_version\": 1,\n"
        "  \"servo_min_us\": [498, 1074, 1074, 1001, 1001, 976],\n"
        "  \"servo_max_us\": [2500, 1953, 1953, 2002, 2002, 1733],\n"
        "  \"servo_bias_us\": [0, 5, -3, 0, 0, 0],\n"
        "  \"smooth_velocity_us_per_s\": [2000,2000,2000,2000,2000,2000],\n"
        "  \"smooth_accel_us_per_s2\":   [8000,8000,8000,8000,8000,8000],\n"
        "  \"smooth_deadband_us\":       [10,10,10,10,10,10],\n"
        "  \"interp_conf_floor_pct\": 40,\n"
        "  \"interp_conf_ceil_pct\":  85,\n"
        "  \"hysteresis_votes\": 3,\n"
        "  \"grip_open_us\":   1700,\n"
        "  \"grip_touch_us\":  1200,\n"
        "  \"grip_firm_us\":   1100,\n"
        "  \"grip_stall_recover_ms\": 2000\n"
        "}\n";
    char *path = write_temp(json);
    IPC_RuntimeConfig cfg;
    char err[256] = {0};
    CFG_Status st = CFG_LoadFromFile(path, &cfg, err, sizeof(err));

    CHECK("TB-CFG01a", "valid file -> CFG_OK", st == CFG_OK,
          "got %s (err=%s)", CFG_StatusStr(st), err);
    CHECK("TB-CFG01b", "magic populated",
          cfg.magic == IPC_CFG_VALID_MAGIC, "got 0x%x", cfg.magic);
    CHECK("TB-CFG01c", "schema_version captured",
          cfg.schema_version == 1, "got %u", cfg.schema_version);
    CHECK("TB-CFG01d", "servo_min_us[0]=498",
          cfg.servo_min_us[0] == 498, "got %u", cfg.servo_min_us[0]);
    CHECK("TB-CFG01e", "servo_max_us[5]=1733",
          cfg.servo_max_us[5] == 1733, "got %u", cfg.servo_max_us[5]);
    CHECK("TB-CFG01f", "signed bias parsed",
          cfg.servo_bias_us[1] == 5 && cfg.servo_bias_us[2] == -3,
          "got [1]=%d [2]=%d", cfg.servo_bias_us[1], cfg.servo_bias_us[2]);
    CHECK("TB-CFG01g", "deadband[3]=10",
          cfg.smooth_deadband_us[3] == 10, "got %u", cfg.smooth_deadband_us[3]);
    CHECK("TB-CFG01h", "interp_conf_ceil_pct=85",
          cfg.interp_conf_ceil_pct == 85, "got %u", cfg.interp_conf_ceil_pct);
    CHECK("TB-CFG01i", "grip_firm_us=1100",
          cfg.grip_firm_us == 1100, "got %u", cfg.grip_firm_us);
    unlink(path);
}

/*============= TB-CFG02 : Defaults ========================================*/

static void test_defaults(void)
{
    IPC_RuntimeConfig cfg;
    CFG_Defaults(&cfg);
    CHECK("TB-CFG02a", "default magic set",
          cfg.magic == IPC_CFG_VALID_MAGIC, "");
    CHECK("TB-CFG02b", "default schema_version=1",
          cfg.schema_version == 1, "");
    CHECK("TB-CFG02c", "default servo_min_us[0]=498",
          cfg.servo_min_us[0] == 498, "got %u", cfg.servo_min_us[0]);
    CHECK("TB-CFG02d", "default deadband=10",
          cfg.smooth_deadband_us[2] == 10, "got %u", cfg.smooth_deadband_us[2]);
    CHECK("TB-CFG02e", "default conf floor < ceil",
          cfg.interp_conf_floor_pct < cfg.interp_conf_ceil_pct,
          "floor=%u ceil=%u", cfg.interp_conf_floor_pct, cfg.interp_conf_ceil_pct);
    CHECK("TB-CFG02f", "default grip_firm < grip_touch < grip_open",
          cfg.grip_firm_us < cfg.grip_touch_us &&
          cfg.grip_touch_us < cfg.grip_open_us,
          "firm=%u touch=%u open=%u",
          cfg.grip_firm_us, cfg.grip_touch_us, cfg.grip_open_us);
}

/*============= TB-CFG03 : Missing file ====================================*/

static void test_missing_file(void)
{
    IPC_RuntimeConfig cfg;
    char err[256] = {0};
    CFG_Status st = CFG_LoadFromFile("/tmp/this_file_does_not_exist_xyzzy.json",
                                     &cfg, err, sizeof(err));
    CHECK("TB-CFG03a", "missing file -> CFG_ERR_OPEN",
          st == CFG_ERR_OPEN, "got %s", CFG_StatusStr(st));
    CHECK("TB-CFG03b", "err message populated",
          strlen(err) > 0, "err='%s'", err);
}

/*============= TB-CFG04 : Wrong schema ====================================*/

static void test_wrong_schema(void)
{
    const char *json =
        "{\n"
        "  \"schema_version\": 99,\n"
        "  \"servo_min_us\": [498, 1074, 1074, 1001, 1001, 976],\n"
        "  \"servo_max_us\": [2500, 1953, 1953, 2002, 2002, 1733]\n"
        "}\n";
    char *path = write_temp(json);
    IPC_RuntimeConfig cfg;
    char err[256] = {0};
    CFG_Status st = CFG_LoadFromFile(path, &cfg, err, sizeof(err));
    CHECK("TB-CFG04a", "schema=99 -> CFG_ERR_SCHEMA",
          st == CFG_ERR_SCHEMA, "got %s", CFG_StatusStr(st));
    CHECK("TB-CFG04b", "err mentions schema",
          strstr(err, "schema_version") != NULL, "err='%s'", err);
    unlink(path);
}

/*============= TB-CFG05 : Out-of-range value ==============================*/

static void test_out_of_range(void)
{
    const char *json =
        "{\n"
        "  \"schema_version\": 1,\n"
        "  \"servo_min_us\": [498, 1074, 1074, 1001, 1001, 976],\n"
        "  \"servo_max_us\": [2500, 1953, 1953, 2002, 2002, 9999]\n"
        "}\n";
    char *path = write_temp(json);
    IPC_RuntimeConfig cfg;
    char err[256] = {0};
    CFG_Status st = CFG_LoadFromFile(path, &cfg, err, sizeof(err));
    CHECK("TB-CFG05a", "max[5]=9999 -> CFG_ERR_RANGE",
          st == CFG_ERR_RANGE, "got %s", CFG_StatusStr(st));
    unlink(path);
}

/*============= TB-CFG06 : min >= max ======================================*/

static void test_min_ge_max(void)
{
    const char *json =
        "{\n"
        "  \"schema_version\": 1,\n"
        "  \"servo_min_us\": [1500, 1074, 1074, 1001, 1001, 976],\n"
        "  \"servo_max_us\": [1499, 1953, 1953, 2002, 2002, 1733]\n"
        "}\n";
    char *path = write_temp(json);
    IPC_RuntimeConfig cfg;
    char err[256] = {0};
    CFG_Status st = CFG_LoadFromFile(path, &cfg, err, sizeof(err));
    CHECK("TB-CFG06a", "min>=max -> CFG_ERR_RANGE",
          st == CFG_ERR_RANGE, "got %s", CFG_StatusStr(st));
    CHECK("TB-CFG06b", "err mentions servo[0]",
          strstr(err, "servo[0]") != NULL, "err='%s'", err);
    unlink(path);
}

/*============= TB-CFG07 : Optional fields honoured ========================*/

static void test_optional_present(void)
{
    const char *json =
        "{\n"
        "  \"schema_version\": 1,\n"
        "  \"servo_min_us\": [498, 1074, 1074, 1001, 1001, 976],\n"
        "  \"servo_max_us\": [2500, 1953, 1953, 2002, 2002, 1733],\n"
        "  \"hysteresis_votes\": 5,\n"
        "  \"grip_firm_us\": 1050,\n"
        "  \"servo_bias_us\": [0, 8, -2, 0, 0, 0]\n"
        "}\n";
    char *path = write_temp(json);
    IPC_RuntimeConfig cfg;
    char err[256] = {0};
    CFG_Status st = CFG_LoadFromFile(path, &cfg, err, sizeof(err));
    CHECK("TB-CFG07a", "optional present -> CFG_OK",
          st == CFG_OK, "got %s (err=%s)", CFG_StatusStr(st), err);
    CHECK("TB-CFG07b", "hysteresis_votes=5",
          cfg.hysteresis_votes == 5, "got %u", cfg.hysteresis_votes);
    CHECK("TB-CFG07c", "grip_firm_us=1050",
          cfg.grip_firm_us == 1050, "got %u", cfg.grip_firm_us);
    CHECK("TB-CFG07d", "bias[1]=8",
          cfg.servo_bias_us[1] == 8, "got %d", cfg.servo_bias_us[1]);
    unlink(path);
}

/*============= TB-CFG08 : Optional fields default when absent =============*/

static void test_optional_absent(void)
{
    /* Minimal file: only the required fields. */
    const char *json =
        "{\n"
        "  \"schema_version\": 1,\n"
        "  \"servo_min_us\": [498, 1074, 1074, 1001, 1001, 976],\n"
        "  \"servo_max_us\": [2500, 1953, 1953, 2002, 2002, 1733]\n"
        "}\n";
    char *path = write_temp(json);
    IPC_RuntimeConfig cfg;
    char err[256] = {0};
    CFG_Status st = CFG_LoadFromFile(path, &cfg, err, sizeof(err));
    CHECK("TB-CFG08a", "minimal file -> CFG_OK",
          st == CFG_OK, "got %s (err=%s)", CFG_StatusStr(st), err);
    CHECK("TB-CFG08b", "default deadband applied",
          cfg.smooth_deadband_us[0] == 10, "got %u", cfg.smooth_deadband_us[0]);
    CHECK("TB-CFG08c", "default hysteresis_votes=3",
          cfg.hysteresis_votes == 3, "got %u", cfg.hysteresis_votes);
    CHECK("TB-CFG08d", "default grip_firm_us=1100",
          cfg.grip_firm_us == 1100, "got %u", cfg.grip_firm_us);
    unlink(path);
}

/*============= TB-CFG09 : CFG_PatchFile round-trip ========================*/

static void test_patch_round_trip(void)
{
    /* A file with several fields including one we don't touch
     * (gesture_velocity-like nested object). The patch must edit the
     * targeted arrays and leave the nested object alone. */
    const char *json =
        "{\n"
        "  \"schema_version\": 1,\n"
        "  \"servo_min_us\": [498, 1074, 1074, 1001, 1001, 976],\n"
        "  \"servo_max_us\": [2500, 1953, 1953, 2002, 2002, 1733],\n"
        "  \"servo_bias_us\": [0, 0, 0, 0, 0, 0],\n"
        "  \"gesture_velocity\": {\n"
        "      \"biceps_flex\": [0, 200, 0, 0, 0, 0]\n"
        "  }\n"
        "}\n";
    char *path = write_temp(json);

    int16_t new_min[6]  = { 510, 1080, 1080, 1010, 1010,  990 };
    int16_t new_max[6]  = {2490, 1940, 1940, 1990, 1990, 1720 };
    int16_t new_bias[6] = {  -3,    5,    0,    2,    0,   -1 };
    CFG_PatchEntry patches[] = {
        { "servo_min_us",  new_min,  6 },
        { "servo_max_us",  new_max,  6 },
        { "servo_bias_us", new_bias, 6 },
    };
    char err[256] = {0};
    CFG_Status st = CFG_PatchFile(path, patches, 3, err, sizeof(err));
    CHECK("TB-CFG09a", "patch returns CFG_OK",
          st == CFG_OK, "got %s (err=%s)", CFG_StatusStr(st), err);

    /* Reload and verify each patched field. */
    IPC_RuntimeConfig cfg;
    st = CFG_LoadFromFile(path, &cfg, err, sizeof(err));
    CHECK("TB-CFG09b", "patched file reloads",
          st == CFG_OK, "got %s (err=%s)", CFG_StatusStr(st), err);
    CHECK("TB-CFG09c", "servo_min_us[0] patched",
          cfg.servo_min_us[0] == 510, "got %u", cfg.servo_min_us[0]);
    CHECK("TB-CFG09d", "servo_min_us[5] patched",
          cfg.servo_min_us[5] == 990, "got %u", cfg.servo_min_us[5]);
    CHECK("TB-CFG09e", "servo_max_us[0] patched",
          cfg.servo_max_us[0] == 2490, "got %u", cfg.servo_max_us[0]);
    CHECK("TB-CFG09f", "servo_max_us[5] patched",
          cfg.servo_max_us[5] == 1720, "got %u", cfg.servo_max_us[5]);
    CHECK("TB-CFG09g", "servo_bias_us[0] patched (negative)",
          cfg.servo_bias_us[0] == -3, "got %d", cfg.servo_bias_us[0]);
    CHECK("TB-CFG09h", "servo_bias_us[1] patched",
          cfg.servo_bias_us[1] == 5, "got %d", cfg.servo_bias_us[1]);

    /* Verify gesture_velocity nested object survived untouched.
     * The C parser doesn't expose it (dsp owns it), so we read the
     * file as text and grep. */
    FILE *f = fopen(path, "r");
    CHECK("TB-CFG09i", "file still readable after patch", f != NULL, "");
    if(f)
    {
        char raw[8192] = {0};
        size_t n = fread(raw, 1, sizeof(raw)-1, f);
        fclose(f);
        raw[n] = '\0';
        bool found = strstr(raw, "biceps_flex") != NULL &&
                     strstr(raw, "[0, 200, 0, 0, 0, 0]") != NULL;
        CHECK("TB-CFG09j", "gesture_velocity preserved",
              found, "biceps_flex row missing after patch");
    }
    unlink(path);
}

/*============= TB-CFG10 : CFG_PatchFile error paths =======================*/

static void test_patch_errors(void)
{
    /* Missing key returns CFG_ERR_MISSING. */
    const char *json =
        "{\n"
        "  \"schema_version\": 1,\n"
        "  \"servo_min_us\": [498, 1074, 1074, 1001, 1001, 976],\n"
        "  \"servo_max_us\": [2500, 1953, 1953, 2002, 2002, 1733]\n"
        "}\n";
    char *path = write_temp(json);
    int16_t vals[6] = {0};
    CFG_PatchEntry patches[] = {
        { "no_such_key_anywhere", vals, 6 },
    };
    char err[256] = {0};
    CFG_Status st = CFG_PatchFile(path, patches, 1, err, sizeof(err));
    CHECK("TB-CFG10a", "missing key -> CFG_ERR_MISSING",
          st == CFG_ERR_MISSING, "got %s", CFG_StatusStr(st));
    CHECK("TB-CFG10b", "err mentions key",
          strstr(err, "no_such_key_anywhere") != NULL,
          "err='%s'", err);

    /* Missing file returns CFG_ERR_OPEN. */
    st = CFG_PatchFile("/tmp/nonexistent_xyzzy_pca.json",
                       patches, 1, err, sizeof(err));
    CHECK("TB-CFG10c", "missing file -> CFG_ERR_OPEN",
          st == CFG_ERR_OPEN, "got %s", CFG_StatusStr(st));
    unlink(path);
}

/*============= MAIN =======================================================*/

int main(void)
{
    printf("=== CPCU CONFIG LOADER TESTBENCH ===\n");
    printf("Target: cpcu_config v1.0  schema=%d\n\n",
           CPCU_CONFIG_SCHEMA_VERSION);

    printf("--- TB-CFG01: Valid full file ---\n");
    test_valid_full();
    printf("\n--- TB-CFG02: Defaults ---\n");
    test_defaults();
    printf("\n--- TB-CFG03: Missing file ---\n");
    test_missing_file();
    printf("\n--- TB-CFG04: Wrong schema_version ---\n");
    test_wrong_schema();
    printf("\n--- TB-CFG05: Out-of-range value ---\n");
    test_out_of_range();
    printf("\n--- TB-CFG06: min >= max sanity ---\n");
    test_min_ge_max();
    printf("\n--- TB-CFG07: Optional fields honoured ---\n");
    test_optional_present();
    printf("\n--- TB-CFG08: Optional fields default when absent ---\n");
    test_optional_absent();
    printf("\n--- TB-CFG09: CFG_PatchFile round-trip  ---\n");
    test_patch_round_trip();
    printf("\n--- TB-CFG10: CFG_PatchFile error paths  ---\n");
    test_patch_errors();

    printf("\n======================================\n");
    printf("RESULTS: %d PASS, %d FAIL\n", g_pass, g_fail);
    printf("======================================\n");
    return (g_fail == 0) ? 0 : 1;
}

