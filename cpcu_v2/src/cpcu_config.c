/**
 *  @file   cpcu_config.c
 *  @brief  Runtime configuration loader — parses JSON into IPC_RuntimeConfig.
 *
 *  Reads config/runtime.json, validates all fields against allowed ranges,
 *  and populates an IPC_RuntimeConfig struct. Provides CFG_Defaults() for
 *  compile-time fallback values. CFG_PatchFile() supports single-field
 *  updates used by the TUI live editor's Ctrl+S save path.
 */

#include "cpcu_config.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <unistd.h>          /* unlink, getpid, rename, ssize_t */
#include <stdbool.h>

/*============= COMPILE-TIME DEFAULTS ======================================*/
/*
 *  These mirror the values in cpcu_pca9685.h, cpcu_smooth.h, and the
 *  draft GESTURE_BEHAVIOR map for future. If runtime.json is absent on
 *  a fresh install, the kernel refuses to start and points the user at
 *  cpcu_v2/config/runtime.json.example. CFG_Defaults is for tests and
 *  configure.sh --reset only.
 */

static const uint16_t DEFAULT_SERVO_MIN_US[IPC_CFG_NUM_SERVOS] =
    {  498, 1074, 1074, 1001, 1001,  976 };
static const uint16_t DEFAULT_SERVO_MAX_US[IPC_CFG_NUM_SERVOS] =
    { 2500, 1953, 1953, 2002, 2002, 1733 };

void CFG_Defaults(IPC_RuntimeConfig *out)
{
    memset(out, 0, sizeof(*out));
    out->magic           = IPC_CFG_VALID_MAGIC;
    out->schema_version  = CPCU_CONFIG_SCHEMA_VERSION;

    for(int i = 0; i < IPC_CFG_NUM_SERVOS; i++)
    {
        out->servo_min_us[i]              = DEFAULT_SERVO_MIN_US[i];
        out->servo_max_us[i]              = DEFAULT_SERVO_MAX_US[i];
        out->servo_bias_us[i]             = 0;
        out->smooth_velocity_us_per_s[i]  = 2000;       /* SMOOTH_DEFAULT_VELOCITY */
        out->smooth_accel_us_per_s2[i]    = 8000;       /* SMOOTH_DEFAULT_ACCEL */
        out->smooth_deadband_us[i]        = 10;         /* SMOOTH_DEFAULT_DEADBAND */
        out->gravity_dir[i]              = 0;
        out->gravity_scale_pct[i]        = 100;
    }

    /* Gesture velocities: zero everywhere by default (i.e. rest = freeze).
     * will populate non-rest classes from runtime.json. */
    /* (memset above already zeroed out->gesture_velocity[][]). */

    out->interp_conf_floor_pct  = 40;       /* 0.40 */
    out->interp_conf_ceil_pct   = 85;       /* 0.85 */
    out->hysteresis_votes       = 3;
    out->grip_open_us           = 1700;
    out->grip_touch_us          = 1200;
    out->grip_firm_us           = 1100;
    out->grip_stall_recover_ms  = 2000;
}

const char *CFG_StatusStr(CFG_Status s)
{
    switch(s)
    {
        case CFG_OK:           return "OK";
        case CFG_ERR_OPEN:     return "file open failed";
        case CFG_ERR_PARSE:    return "JSON parse error";
        case CFG_ERR_SCHEMA:   return "schema_version mismatch";
        case CFG_ERR_RANGE:    return "value out of range";
        case CFG_ERR_MISSING:  return "required field missing";
        case CFG_ERR_INTERNAL: return "internal loader error";
        default:               return "unknown";
    }
}

/*============= MINIMAL JSON TOKENIZER =====================================*/
/*
 *  We don't build a real AST — we walk the file linearly and match
 *  expected keys in order. This works because runtime.json has a fixed
 *  shape that we control. The "find-by-key" function lets us skip
 *  whitespace and comments while looking for the next quoted string
 *  matching `key`.
 *
 *  This is NOT a robust parser. It assumes the file was produced by
 *  configure.sh or hand-edited carefully. If you need full JSON, swap
 *  this for jansson later.
 */

typedef struct
{
    const char *src;
    size_t      pos;
    size_t      len;
    char       *err;
    size_t      err_sz;
} JC;

static void jc_set_err(JC *j, const char *fmt, ...)
{
    if(j->err == NULL || j->err_sz == 0) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(j->err, j->err_sz, fmt, ap);
    va_end(ap);
}

static void jc_skip_ws(JC *j)
{
    while(j->pos < j->len)
    {
        char c = j->src[j->pos];
        if(c == ' ' || c == '\t' || c == '\n' || c == '\r')
            j->pos++;
        else if(c == '/' && j->pos + 1 < j->len && j->src[j->pos+1] == '/')
        {
            /* line comment — JSON doesn't allow this, but be lenient */
            while(j->pos < j->len && j->src[j->pos] != '\n') j->pos++;
        }
        else
            break;
    }
}

/* Look ahead for `"<key>"` at the JSON root level (depth==1, since the
 * outer `{` puts us at depth 1). On success, advances j->pos past the
 * colon. */
static bool jc_find_key(JC *j, const char *key)
{
    size_t klen = strlen(key);
    int depth = 0;
    while(j->pos < j->len)
    {
        jc_skip_ws(j);
        if(j->pos >= j->len) break;
        char c = j->src[j->pos];
        if(c == '{' || c == '[') { depth++; j->pos++; continue; }
        if(c == '}' || c == ']')
        {
            if(depth == 0) return false;
            depth--; j->pos++; continue;
        }
        if(c == '"')
        {
            /* Read string content — could be a key or a value. */
            size_t start = j->pos + 1;
            j->pos++;
            while(j->pos < j->len && j->src[j->pos] != '"') j->pos++;
            if(j->pos >= j->len) { jc_set_err(j, "unterminated string"); return false; }
            size_t slen = j->pos - start;
            j->pos++;       /* past closing quote */

            /* Is this a key (followed by ':')? */
            jc_skip_ws(j);
            if(j->pos < j->len && j->src[j->pos] == ':')
            {
                /* Root-level keys are at depth 1 (inside the document
                 * object). Nested object keys would be at depth >= 2. */
                if(depth == 1 && slen == klen &&
                   memcmp(j->src + start, key, klen) == 0)
                {
                    j->pos++;       /* past ':' */
                    return true;
                }
                j->pos++;            /* skip ':' */
            }
            continue;
        }
        j->pos++;
    }
    return false;
}

/* Read a signed integer at the current position. Skips ws first. */
static bool jc_read_int(JC *j, long *out)
{
    jc_skip_ws(j);
    if(j->pos >= j->len) return false;
    char *endp = NULL;
    errno = 0;
    long v = strtol(j->src + j->pos, &endp, 10);
    if(endp == j->src + j->pos || errno != 0) return false;
    j->pos = endp - j->src;
    *out = v;
    return true;
}

/* Read an array of integers into a dst[] of size n. Each element must
 * be present. Reads `[ N, N, N ... ]`. */
static bool jc_read_int_array(JC *j, long *dst, int n)
{
    jc_skip_ws(j);
    if(j->pos >= j->len || j->src[j->pos] != '[') return false;
    j->pos++;
    for(int i = 0; i < n; i++)
    {
        if(!jc_read_int(j, &dst[i])) return false;
        jc_skip_ws(j);
        if(i < n - 1)
        {
            if(j->pos >= j->len || j->src[j->pos] != ',') return false;
            j->pos++;
        }
    }
    jc_skip_ws(j);
    if(j->pos >= j->len || j->src[j->pos] != ']') return false;
    j->pos++;
    return true;
}

/*============= FIELD READERS ==============================================*/
/*
 *  Each helper takes (parser, output, key_path), returns true on success.
 *  Caller decides whether absence is an error or just a "use default".
 *  Per design rules, top-level mandatory fields use REQUIRE; per-field
 *  servo bias / per-class velocities use OPTIONAL with defaulted values
 *  so a JSON without the future gesture map still works for current.
 */

#define REQUIRE(parser_call, errmsg)                                    \
    do {                                                                \
        if(!(parser_call))                                              \
        {                                                               \
            jc_set_err(j, errmsg);                                      \
            return CFG_ERR_PARSE;                                       \
        }                                                               \
    } while(0)

static bool read_u16_array(JC *j, const char *key, uint16_t *dst, int n,
                           uint16_t lo, uint16_t hi)
{
    JC saved = *j; j->pos = 0;
    if(!jc_find_key(j, key)) { *j = saved; return false; }
    long tmp[16];
    if(n > 16) return false;        /* programmer error */
    if(!jc_read_int_array(j, tmp, n)) return false;
    for(int i = 0; i < n; i++)
    {
        if(tmp[i] < (long)lo || tmp[i] > (long)hi)
        {
            jc_set_err(j, "%s[%d] = %ld out of range [%u..%u]",
                       key, i, tmp[i], lo, hi);
            return false;
        }
        dst[i] = (uint16_t)tmp[i];
    }
    return true;
}

static bool read_i16_array(JC *j, const char *key, int16_t *dst, int n,
                           int16_t lo, int16_t hi)
{
    JC saved = *j; j->pos = 0;
    if(!jc_find_key(j, key)) { *j = saved; return false; }
    long tmp[16];
    if(n > 16) return false;
    if(!jc_read_int_array(j, tmp, n)) return false;
    for(int i = 0; i < n; i++)
    {
        if(tmp[i] < (long)lo || tmp[i] > (long)hi)
        {
            jc_set_err(j, "%s[%d] = %ld out of range [%d..%d]",
                       key, i, tmp[i], lo, hi);
            return false;
        }
        dst[i] = (int16_t)tmp[i];
    }
    return true;
}

static bool read_uint(JC *j, const char *key, long *dst, long lo, long hi)
{
    JC saved = *j; j->pos = 0;
    if(!jc_find_key(j, key)) { *j = saved; return false; }
    long tmp;
    if(!jc_read_int(j, &tmp)) return false;
    if(tmp < lo || tmp > hi)
    {
        jc_set_err(j, "%s = %ld out of range [%ld..%ld]", key, tmp, lo, hi);
        return false;
    }
    *dst = tmp;
    return true;
}

/*============= TOP-LEVEL LOAD =============================================*/

CFG_Status CFG_LoadFromFile(const char *path, IPC_RuntimeConfig *out,
                            char *err_msg, size_t err_msg_sz)
{
    /* Start from defaults so non-required fields have sensible values
     * even if absent from runtime.json. */
    CFG_Defaults(out);

    FILE *f = fopen(path, "rb");
    if(!f)
    {
        if(err_msg)
            snprintf(err_msg, err_msg_sz,
                     "fopen(%s) failed: %s", path, strerror(errno));
        return CFG_ERR_OPEN;
    }

    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if(fsz < 0 || fsz > 65536)      /* 64 KB hard cap */
    {
        fclose(f);
        if(err_msg) snprintf(err_msg, err_msg_sz,
                             "%s: bad size %ld (limit 64 KB)", path, fsz);
        return CFG_ERR_PARSE;
    }

    char *buf = malloc((size_t)fsz + 1);
    if(!buf) { fclose(f); return CFG_ERR_INTERNAL; }
    size_t rd = fread(buf, 1, (size_t)fsz, f);
    fclose(f);
    if(rd != (size_t)fsz)
    {
        free(buf);
        if(err_msg) snprintf(err_msg, err_msg_sz, "%s: short read", path);
        return CFG_ERR_PARSE;
    }
    buf[fsz] = '\0';

    JC jp = { .src = buf, .pos = 0, .len = (size_t)fsz,
              .err = err_msg, .err_sz = err_msg_sz };
    JC *j = &jp;

    /* Schema version is mandatory. */
    long sv;
    if(!read_uint(j, "schema_version", &sv, 1, 1024))
    {
        free(buf);
        return CFG_ERR_MISSING;
    }
    if(sv != CPCU_CONFIG_SCHEMA_VERSION)
    {
        if(err_msg)
            snprintf(err_msg, err_msg_sz,
                     "schema_version=%ld but expected %d. "
                     "Run scripts/configure.sh --reset to regenerate.",
                     sv, CPCU_CONFIG_SCHEMA_VERSION);
        free(buf);
        return CFG_ERR_SCHEMA;
    }
    out->schema_version = (uint32_t)sv;

    /* Servo limits — mandatory. */
    if(!read_u16_array(j, "servo_min_us", out->servo_min_us,
                       IPC_CFG_NUM_SERVOS, 400, 2600))
    { free(buf); return CFG_ERR_RANGE; }
    if(!read_u16_array(j, "servo_max_us", out->servo_max_us,
                       IPC_CFG_NUM_SERVOS, 400, 2600))
    { free(buf); return CFG_ERR_RANGE; }
    /* Sanity: min < max for every channel. */
    for(int i = 0; i < IPC_CFG_NUM_SERVOS; i++)
    {
        if(out->servo_min_us[i] >= out->servo_max_us[i])
        {
            if(err_msg) snprintf(err_msg, err_msg_sz,
                "servo[%d]: min=%u >= max=%u", i,
                out->servo_min_us[i], out->servo_max_us[i]);
            free(buf);
            return CFG_ERR_RANGE;
        }
    }

    /* Optional fields — keep defaults if absent. */
    (void)read_i16_array(j, "servo_bias_us", out->servo_bias_us,
                         IPC_CFG_NUM_SERVOS, -100, 100);
    (void)read_u16_array(j, "smooth_velocity_us_per_s",
                         out->smooth_velocity_us_per_s, IPC_CFG_NUM_SERVOS,
                         100, 10000);
    (void)read_u16_array(j, "smooth_accel_us_per_s2",
                         out->smooth_accel_us_per_s2, IPC_CFG_NUM_SERVOS,
                         500, 50000);
    (void)read_u16_array(j, "smooth_deadband_us",
                         out->smooth_deadband_us, IPC_CFG_NUM_SERVOS,
                         0, 50);

    /* gravity compensation */
    (void)read_i16_array(j, "gravity_dir",
                         out->gravity_dir, IPC_CFG_NUM_SERVOS,
                         -1, 1);
    (void)read_i16_array(j, "gravity_scale_pct",
                         out->gravity_scale_pct, IPC_CFG_NUM_SERVOS,
                         10, 100);

    long tmp;
    if(read_uint(j, "interp_conf_floor_pct", &tmp, 0, 100))
        out->interp_conf_floor_pct = (uint8_t)tmp;
    if(read_uint(j, "interp_conf_ceil_pct",  &tmp, 0, 100))
        out->interp_conf_ceil_pct  = (uint8_t)tmp;
    if(out->interp_conf_floor_pct >= out->interp_conf_ceil_pct)
    {
        if(err_msg) snprintf(err_msg, err_msg_sz,
            "interp_conf_floor_pct (%u) >= ceil_pct (%u)",
            out->interp_conf_floor_pct, out->interp_conf_ceil_pct);
        free(buf);
        return CFG_ERR_RANGE;
    }
    if(read_uint(j, "hysteresis_votes", &tmp, 1, 20))
        out->hysteresis_votes = (uint8_t)tmp;

    if(read_uint(j, "grip_open_us",  &tmp, 800, 2200))
        out->grip_open_us  = (uint16_t)tmp;
    if(read_uint(j, "grip_touch_us", &tmp, 800, 2200))
        out->grip_touch_us = (uint16_t)tmp;
    if(read_uint(j, "grip_firm_us",  &tmp, 800, 2200))
        out->grip_firm_us  = (uint16_t)tmp;
    if(read_uint(j, "grip_stall_recover_ms", &tmp, 100, 30000))
        out->grip_stall_recover_ms = (uint16_t)tmp;

    /* gesture_velocity: 2-D array, one row per class. Optional —
     * absence means "all rest, no motion". future is the consumer. */
    /* Skipped for current — adds complexity not yet exercised. The
     * existing default (zero-filled) is correct for "freeze on every
     * gesture", which is the safe pre-velocity-mode behaviour. */

    out->magic = IPC_CFG_VALID_MAGIC;

    free(buf);
    return CFG_OK;
}

/*============= TARGETED PATCH WRITER =====================*/
/*
 *  Surgical text-level edit of a single JSON file. We load the whole
 *  file, locate each target key's value array, and splice in new
 *  contents. Other content (comments, ordering, whitespace, fields
 *  we don't know about) is preserved.
 *
 *  The key location uses the same depth-aware walker as jc_find_key,
 *  reused via a small wrapper. We work on a growing dynamic buffer
 *  rather than a fixed array because each splice changes the length.
 */

/* Find the position of "<key>" at depth 1 in `src`, return the index
 * of the colon following it, or (size_t)-1 if not found. Reuses the
 * same logic as jc_find_key but doesn't depend on the JC parser
 * state. */
static size_t pf_find_key_colon(const char *src, size_t len, const char *key)
{
    size_t klen = strlen(key);
    size_t i    = 0;
    int    depth = 0;
    bool   in_str = false;
    while(i < len)
    {
        char c = src[i];
        if(in_str)
        {
            if(c == '\\' && i + 1 < len) { i += 2; continue; }
            if(c == '"') in_str = false;
            i++;
            continue;
        }
        if(c == '/' && i + 1 < len && src[i+1] == '/')
        {
            while(i < len && src[i] != '\n') i++;
            continue;
        }
        if(c == '{' || c == '[') { depth++; i++; continue; }
        if(c == '}' || c == ']') { depth--; i++; continue; }
        if(c == '"')
        {
            size_t s = i + 1;
            i++;
            while(i < len && src[i] != '"')
            {
                if(src[i] == '\\' && i + 1 < len) i += 2;
                else i++;
            }
            if(i >= len) return (size_t)-1;
            size_t slen = i - s;
            i++;        /* past closing quote */

            /* Skip whitespace, then check for ':' */
            size_t j = i;
            while(j < len && (src[j]==' '||src[j]=='\t'||src[j]=='\n'||src[j]=='\r')) j++;
            if(j < len && src[j] == ':')
            {
                if(depth == 1 && slen == klen &&
                   memcmp(src + s, key, klen) == 0)
                {
                    return j;       /* index of the ':' */
                }
                i = j + 1;
                continue;
            }
            continue;
        }
        i++;
    }
    return (size_t)-1;
}

/* Find the matching ']' for the '[' at position `open_pos`. Returns
 * the index of the matching ']', or (size_t)-1 on imbalance. */
static size_t pf_find_array_end(const char *src, size_t len, size_t open_pos)
{
    int depth = 0;
    bool in_str = false;
    for(size_t i = open_pos; i < len; i++)
    {
        char c = src[i];
        if(in_str)
        {
            if(c == '\\' && i + 1 < len) { i++; continue; }
            if(c == '"') in_str = false;
            continue;
        }
        if(c == '"') { in_str = true; continue; }
        if(c == '[') depth++;
        else if(c == ']')
        {
            depth--;
            if(depth == 0) return i;
        }
    }
    return (size_t)-1;
}

/* Build the replacement array text "[ N, N, N, ... ]" into `out`,
 * which must have enough room (caller sizes generously). */
static void pf_format_array(char *out, size_t out_sz,
                            const int16_t *vals, size_t n)
{
    size_t pos = 0;
    int written = snprintf(out + pos, out_sz - pos, "[ ");
    if(written > 0) pos += (size_t)written;
    for(size_t i = 0; i < n && pos < out_sz; i++)
    {
        written = snprintf(out + pos, out_sz - pos,
                           "%d%s", (int)vals[i], (i + 1 < n) ? ", " : " ");
        if(written > 0) pos += (size_t)written;
    }
    if(pos < out_sz)
    {
        snprintf(out + pos, out_sz - pos, "]");
    }
}

CFG_Status CFG_PatchFile(const char *path,
                         const CFG_PatchEntry *entries, size_t n_entries,
                         char *err_msg, size_t err_msg_sz)
{
    /* Load entire file into memory. */
    FILE *f = fopen(path, "rb");
    if(!f)
    {
        if(err_msg) snprintf(err_msg, err_msg_sz,
                             "fopen(%s) failed: %s", path, strerror(errno));
        return CFG_ERR_OPEN;
    }
    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if(fsz < 0 || fsz > 65536)
    {
        fclose(f);
        if(err_msg) snprintf(err_msg, err_msg_sz, "%s: bad size %ld", path, fsz);
        return CFG_ERR_PARSE;
    }

    /* Allocate generously — every patch can grow the file by some
     * margin. 4× plus 4 KB headroom is safely more than enough. */
    size_t buf_cap = (size_t)fsz * 4 + 4096;
    char *buf = malloc(buf_cap);
    if(!buf) { fclose(f); return CFG_ERR_INTERNAL; }
    size_t buf_len = (size_t)fsz;
    if(fread(buf, 1, buf_len, f) != buf_len)
    {
        free(buf); fclose(f);
        if(err_msg) snprintf(err_msg, err_msg_sz, "%s: short read", path);
        return CFG_ERR_PARSE;
    }
    fclose(f);

    /* Apply each patch in turn. We must re-locate the key after every
     * splice because positions shift. */
    for(size_t e = 0; e < n_entries; e++)
    {
        const CFG_PatchEntry *p = &entries[e];

        /* Format the new array text. Worst case: 6 ints + brackets +
         * commas + sign + 5-digit value each ~= 80 chars. Pad to 256. */
        char new_arr[256];
        pf_format_array(new_arr, sizeof(new_arr), p->values, p->count);
        size_t new_arr_len = strlen(new_arr);

        size_t colon = pf_find_key_colon(buf, buf_len, p->key);
        if(colon == (size_t)-1)
        {
            if(err_msg) snprintf(err_msg, err_msg_sz,
                                 "key '%s' not found in %s", p->key, path);
            free(buf);
            return CFG_ERR_MISSING;
        }

        /* Skip whitespace after ':' to find '[' */
        size_t open_pos = colon + 1;
        while(open_pos < buf_len &&
              (buf[open_pos]==' '||buf[open_pos]=='\t'||
               buf[open_pos]=='\n'||buf[open_pos]=='\r'))
            open_pos++;
        if(open_pos >= buf_len || buf[open_pos] != '[')
        {
            if(err_msg) snprintf(err_msg, err_msg_sz,
                                 "key '%s' is not an array (got '%c')",
                                 p->key,
                                 open_pos < buf_len ? buf[open_pos] : '?');
            free(buf);
            return CFG_ERR_PARSE;
        }
        size_t close_pos = pf_find_array_end(buf, buf_len, open_pos);
        if(close_pos == (size_t)-1)
        {
            if(err_msg) snprintf(err_msg, err_msg_sz,
                                 "key '%s': unterminated array", p->key);
            free(buf);
            return CFG_ERR_PARSE;
        }

        /* Splice: replace buf[open_pos..close_pos] with new_arr. */
        size_t old_len = close_pos - open_pos + 1;
        ssize_t delta = (ssize_t)new_arr_len - (ssize_t)old_len;

        if(buf_len + (size_t)((delta > 0) ? delta : 0) >= buf_cap)
        {
            /* Grow if necessary. */
            size_t want = buf_len + (size_t)delta + 4096;
            char *bigger = realloc(buf, want);
            if(!bigger) { free(buf); return CFG_ERR_INTERNAL; }
            buf = bigger;
            buf_cap = want;
        }

        if(delta != 0)
        {
            memmove(buf + open_pos + new_arr_len,
                    buf + close_pos + 1,
                    buf_len - close_pos - 1);
            buf_len += (size_t)delta;
        }
        memcpy(buf + open_pos, new_arr, new_arr_len);
    }

    /* Write to <path>.tmp atomically, then rename. */
    char tmp_path[1024];
    int tw = snprintf(tmp_path, sizeof(tmp_path), "%s.cfgtmp.%d",
                      path, (int)getpid());
    if(tw < 0 || tw >= (int)sizeof(tmp_path))
    {
        free(buf);
        if(err_msg) snprintf(err_msg, err_msg_sz, "tmp path overflow");
        return CFG_ERR_INTERNAL;
    }

    FILE *out = fopen(tmp_path, "wb");
    if(!out)
    {
        if(err_msg) snprintf(err_msg, err_msg_sz,
                             "fopen(%s) for write failed: %s",
                             tmp_path, strerror(errno));
        free(buf);
        return CFG_ERR_OPEN;
    }
    size_t wrote = fwrite(buf, 1, buf_len, out);
    int    fcr   = fclose(out);
    free(buf);
    if(wrote != buf_len || fcr != 0)
    {
        unlink(tmp_path);
        if(err_msg) snprintf(err_msg, err_msg_sz,
                             "write to %s failed", tmp_path);
        return CFG_ERR_OPEN;
    }

    if(rename(tmp_path, path) != 0)
    {
        if(err_msg) snprintf(err_msg, err_msg_sz,
                             "rename(%s -> %s) failed: %s",
                             tmp_path, path, strerror(errno));
        unlink(tmp_path);
        return CFG_ERR_OPEN;
    }
    return CFG_OK;
}

