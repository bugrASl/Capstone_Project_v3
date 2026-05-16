/**
 *  @file   json_testbench.c
 *  @brief  JSON writer test harness — object/array nesting, type serialization.
 */

#include "cpcu_json.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_EQ(name, got, want) do {                                \
    if(strcmp(got, want) == 0) {                                       \
        printf("[PASS] %-12s\n", name);                                \
        g_pass++;                                                      \
    } else {                                                           \
        printf("[FAIL] %-12s\n  got:  %s\n  want: %s\n",                \
               name, got, want);                                        \
        g_fail++;                                                       \
    }                                                                  \
} while(0)

static void t_empty_object(void)
{
    char buf[64]; JW jw; jw_init(&jw, buf, sizeof(buf));
    jw_obj_begin(&jw); jw_obj_end(&jw);
    EXPECT_EQ("JSON01", buf, "{}");
}

static void t_simple_object(void)
{
    char buf[256]; JW jw; jw_init(&jw, buf, sizeof(buf));
    jw_obj_begin(&jw);
      jw_kv_int(&jw, "n", 42);
      jw_kv_str(&jw, "g", "rest");
      jw_kv_bool(&jw, "ok", true);
    jw_obj_end(&jw);
    EXPECT_EQ("JSON02", buf, "{\"n\":42,\"g\":\"rest\",\"ok\":true}");
}

static void t_array_of_floats(void)
{
    char buf[256]; JW jw; jw_init(&jw, buf, sizeof(buf));
    float v[] = { 1.0f, 2.5f, -3.25f };
    jw_obj_begin(&jw);
      jw_kv_arr_f32(&jw, "rms", v, 3);
    jw_obj_end(&jw);
    EXPECT_EQ("JSON03", buf, "{\"rms\":[1,2.5,-3.25]}");
}

static void t_nested(void)
{
    char buf[256]; JW jw; jw_init(&jw, buf, sizeof(buf));
    jw_obj_begin(&jw);
      jw_kv_obj_begin(&jw, "nested");
        jw_kv_int(&jw, "a", 1);
        jw_kv_int(&jw, "b", 2);
      jw_obj_end(&jw);
    jw_obj_end(&jw);
    EXPECT_EQ("JSON04", buf, "{\"nested\":{\"a\":1,\"b\":2}}");
}

static void t_string_escape(void)
{
    char buf[256]; JW jw; jw_init(&jw, buf, sizeof(buf));
    jw_obj_begin(&jw);
      jw_kv_str(&jw, "msg", "say \"hi\"\nthere");
    jw_obj_end(&jw);
    EXPECT_EQ("JSON05", buf, "{\"msg\":\"say \\\"hi\\\"\\nthere\"}");
}

static void t_nan_inf(void)
{
    char buf[256]; JW jw; jw_init(&jw, buf, sizeof(buf));
    jw_obj_begin(&jw);
      jw_kv_f32(&jw, "x", NAN);
      jw_kv_f32(&jw, "y", INFINITY);
      jw_kv_f32(&jw, "z", 1.5f);
    jw_obj_end(&jw);
    EXPECT_EQ("JSON06", buf, "{\"x\":null,\"y\":null,\"z\":1.5}");
}

static void t_overflow(void)
{
    char buf[8]; JW jw; jw_init(&jw, buf, sizeof(buf));
    jw_obj_begin(&jw);
      jw_kv_str(&jw, "verylongkey", "verylongvalue");
    jw_obj_end(&jw);
    if(jw.overflow) {
        printf("[PASS] JSON07     overflow flag set as expected\n");
        g_pass++;
    } else {
        printf("[FAIL] JSON07     overflow flag NOT set on small buffer\n");
        g_fail++;
    }
}

int main(void)
{
    printf("======================================\n");
    printf("  TB-JSON — cpcu_json unit tests \n");
    printf("======================================\n");
    t_empty_object();
    t_simple_object();
    t_array_of_floats();
    t_nested();
    t_string_escape();
    t_nan_inf();
    t_overflow();
    printf("\n======================================\n");
    printf("RESULTS: %d PASS, %d FAIL\n", g_pass, g_fail);
    printf("======================================\n");
    return g_fail == 0 ? 0 : 1;
}

