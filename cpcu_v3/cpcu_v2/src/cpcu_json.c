/**
 *  @file   cpcu_json.c
 *  @brief  Streaming JSON writer — builds JSON strings into a fixed buffer.
 *
 *  Used by cpcu_ws.c to serialize IPC state into JSON frames without
 *  dynamic allocation. Supports objects, arrays, strings, integers,
 *  floats, booleans, and uint32/uint64 types.
 */

#include "cpcu_json.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

void jw_init(JW *jw, char *buf, size_t cap)
{
    jw->buf      = buf;
    jw->cap      = cap;
    jw->len      = 0;
    jw->overflow = false;
    jw->depth    = 0;
    if(cap > 0) buf[0] = '\0';
}

/* Append a chunk; on overflow, set the flag and stop. We never write
 * past cap-1 (we always reserve a byte for the terminating NUL). */
static void jw_raw(JW *jw, const char *s, size_t n)
{
    if(jw->overflow) return;
    if(jw->len + n + 1 > jw->cap)
    {
        jw->overflow = true;
        return;
    }
    memcpy(jw->buf + jw->len, s, n);
    jw->len += n;
    jw->buf[jw->len] = '\0';
}

static void jw_chr(JW *jw, char c) { jw_raw(jw, &c, 1); }

/* Insert a comma if we're not the first child of the enclosing
 * container; toggle the first-flag once we've written. */
static void jw_sep(JW *jw)
{
    if(jw->depth <= 0) return;
    bool *first = &jw->first_at_depth[jw->depth - 1];
    if(*first) { *first = false; }
    else       { jw_chr(jw, ','); }
}

static void jw_emit_str(JW *jw, const char *s)
{
    jw_chr(jw, '"');
    if(s)
    {
        for(; *s; s++)
        {
            char c = *s;
            switch(c)
            {
                case '"':  jw_raw(jw, "\\\"", 2); break;
                case '\\': jw_raw(jw, "\\\\", 2); break;
                case '\b': jw_raw(jw, "\\b",  2); break;
                case '\f': jw_raw(jw, "\\f",  2); break;
                case '\n': jw_raw(jw, "\\n",  2); break;
                case '\r': jw_raw(jw, "\\r",  2); break;
                case '\t': jw_raw(jw, "\\t",  2); break;
                default:
                    if((unsigned char)c < 0x20)
                    {
                        char esc[8];
                        int  n = snprintf(esc, sizeof(esc), "\\u%04x", c);
                        if(n > 0) jw_raw(jw, esc, (size_t)n);
                    }
                    else
                    {
                        jw_chr(jw, c);
                    }
                    break;
            }
        }
    }
    jw_chr(jw, '"');
}

static void jw_push(JW *jw)
{
    if(jw->depth < JW_MAX_DEPTH)
    {
        jw->first_at_depth[jw->depth] = true;
        jw->depth++;
    }
    else
    {
        jw->overflow = true;
    }
}

static void jw_pop(JW *jw)
{
    if(jw->depth > 0) jw->depth--;
}

void jw_obj_begin(JW *jw) { jw_sep(jw); jw_chr(jw, '{'); jw_push(jw); }
void jw_obj_end  (JW *jw) { jw_pop(jw); jw_chr(jw, '}'); }
void jw_arr_begin(JW *jw) { jw_sep(jw); jw_chr(jw, '['); jw_push(jw); }
void jw_arr_end  (JW *jw) { jw_pop(jw); jw_chr(jw, ']'); }

void jw_int(JW *jw, long long v)
{
    char tmp[32];
    int  n = snprintf(tmp, sizeof(tmp), "%lld", v);
    if(n > 0) { jw_sep(jw); jw_raw(jw, tmp, (size_t)n); }
}

void jw_u32(JW *jw, uint32_t v)
{
    char tmp[16];
    int  n = snprintf(tmp, sizeof(tmp), "%u", (unsigned)v);
    if(n > 0) { jw_sep(jw); jw_raw(jw, tmp, (size_t)n); }
}

void jw_f32(JW *jw, float v)
{
    char tmp[24];
    int  n;
    /* JSON has no NaN / Inf — emit null instead. The browser dashboard
     * treats null as "missing sample" which is the right behaviour for
     * a filter that hasn't started producing yet. */
    if(isnan(v) || isinf(v))
    {
        jw_sep(jw);
        jw_raw(jw, "null", 4);
        return;
    }
    n = snprintf(tmp, sizeof(tmp), "%.6g", (double)v);
    if(n > 0) { jw_sep(jw); jw_raw(jw, tmp, (size_t)n); }
}

void jw_bool(JW *jw, bool v)
{
    jw_sep(jw);
    if(v) jw_raw(jw, "true",  4);
    else  jw_raw(jw, "false", 5);
}

void jw_str(JW *jw, const char *s)
{
    jw_sep(jw);
    jw_emit_str(jw, s);
}

/* key+value helpers. Key is always a string, then ':', then the value. */
static void jw_emit_key(JW *jw, const char *k)
{
    jw_sep(jw);
    jw_emit_str(jw, k);
    jw_chr(jw, ':');
}

void jw_kv_int(JW *jw, const char *k, long long v)
{
    jw_emit_key(jw, k);
    char tmp[32];
    int n = snprintf(tmp, sizeof(tmp), "%lld", v);
    if(n > 0) jw_raw(jw, tmp, (size_t)n);
}

void jw_kv_u32(JW *jw, const char *k, uint32_t v)
{
    jw_emit_key(jw, k);
    char tmp[16];
    int n = snprintf(tmp, sizeof(tmp), "%u", (unsigned)v);
    if(n > 0) jw_raw(jw, tmp, (size_t)n);
}

void jw_kv_u64(JW *jw, const char *k, uint64_t v)
{
    jw_emit_key(jw, k);
    char tmp[32];
    int n = snprintf(tmp, sizeof(tmp), "%llu", (unsigned long long)v);
    if(n > 0) jw_raw(jw, tmp, (size_t)n);
}

void jw_kv_f32(JW *jw, const char *k, float v)
{
    jw_emit_key(jw, k);
    if(isnan(v) || isinf(v)) { jw_raw(jw, "null", 4); return; }
    char tmp[24];
    int n = snprintf(tmp, sizeof(tmp), "%.6g", (double)v);
    if(n > 0) jw_raw(jw, tmp, (size_t)n);
}

void jw_kv_str(JW *jw, const char *k, const char *v)
{
    jw_emit_key(jw, k);
    jw_emit_str(jw, v);
}

void jw_kv_bool(JW *jw, const char *k, bool v)
{
    jw_emit_key(jw, k);
    jw_raw(jw, v ? "true" : "false", v ? 4 : 5);
}

void jw_kv_arr_f32(JW *jw, const char *k, const float *vals, size_t n)
{
    jw_emit_key(jw, k);
    jw_chr(jw, '[');
    jw_push(jw);
    for(size_t i = 0; i < n; i++) jw_f32(jw, vals[i]);
    jw_pop(jw);
    jw_chr(jw, ']');
}

void jw_kv_arr_u16(JW *jw, const char *k, const uint16_t *vals, size_t n)
{
    jw_emit_key(jw, k);
    jw_chr(jw, '[');
    jw_push(jw);
    for(size_t i = 0; i < n; i++) jw_u32(jw, (uint32_t)vals[i]);
    jw_pop(jw);
    jw_chr(jw, ']');
}

void jw_kv_arr_i16(JW *jw, const char *k, const int16_t *vals, size_t n)
{
    jw_emit_key(jw, k);
    jw_chr(jw, '[');
    jw_push(jw);
    for(size_t i = 0; i < n; i++) jw_int(jw, (long long)vals[i]);
    jw_pop(jw);
    jw_chr(jw, ']');
}

void jw_kv_arr_u32(JW *jw, const char *k, const uint32_t *vals, size_t n)
{
    jw_emit_key(jw, k);
    jw_chr(jw, '[');
    jw_push(jw);
    for(size_t i = 0; i < n; i++) jw_u32(jw, vals[i]);
    jw_pop(jw);
    jw_chr(jw, ']');
}

void jw_kv_obj_begin(JW *jw, const char *k)
{
    jw_emit_key(jw, k);
    jw_chr(jw, '{');
    jw_push(jw);
}

void jw_kv_arr_begin(JW *jw, const char *k)
{
    jw_emit_key(jw, k);
    jw_chr(jw, '[');
    jw_push(jw);
}

