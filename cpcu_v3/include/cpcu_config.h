/**
 *  @file   cpcu_config.h
 *  @brief  Runtime config API — load JSON, validate, patch, provide defaults.
 */

#ifndef CPCU_CONFIG_H
#define CPCU_CONFIG_H

#include <stddef.h>

#include "cpcu_ipc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CPCU_CONFIG_SCHEMA_VERSION  1

/*  Return codes for CFG_LoadFromFile. */
typedef enum
{
    CFG_OK = 0,
    CFG_ERR_OPEN,           /* fopen failed, file missing/unreadable */
    CFG_ERR_PARSE,          /* JSON syntax error */
    CFG_ERR_SCHEMA,         /* schema_version mismatch */
    CFG_ERR_RANGE,          /* a value was out of its sane range */
    CFG_ERR_MISSING,        /* a required field was absent */
    CFG_ERR_INTERNAL        /* something went wrong inside the loader */
} CFG_Status;

/*  Parse JSON file into runtime config struct.
 *      path        absolute path to runtime.json
 *      out         destination — zeroed before use
 *      err_msg     buffer for human-readable error message, may be NULL
 *      err_msg_sz  size of err_msg buffer
 *  Returns CFG_OK on success. On any error the caller should LOG_E and
 *  refuse to start. */
CFG_Status  CFG_LoadFromFile(const char *path, IPC_RuntimeConfig *out,
                             char *err_msg, size_t err_msg_sz);

/*  Populate out with compile-time defaults (factory configuration).
 *  Used by tests and by the configure.sh "reset" path; cpcu_kernel
 *  itself never falls back to this on production startup. */
void        CFG_Defaults(IPC_RuntimeConfig *out);

/*  Convert a CFG_Status to a static string (for logging). */
const char *CFG_StatusStr(CFG_Status s);

/*============= TARGETED PATCH WRITER =====================*/
/*
 *  Surgical JSON edit: rewrites only the listed keys and preserves
 *  every other field (including gesture_velocity, which dsp owns and
 *  the C parser ignores). Used by pca_testbench to persist the
 *  servo_min_us / servo_max_us / servo_bias_us values the user
 *  discovered by physically jogging the arm.
 *
 *  Atomic via tmpfile + rename(2). On failure, leaves the original
 *  file untouched.
 *
 *  Limitations:
 *    - Only patches int16 array fields (the ones pca_testbench cares
 *      about). Scalar / string / object fields are not supported.
 *    - The target key must already exist in the file as a flat array.
 *      If absent, returns CFG_ERR_MISSING.
 *    - Comment-keys ('// foo': '...') are preserved verbatim.
 *
 *  See cpcu_v2/docs/RUNTIME_CONFIG.md §10 for the round-trip workflow.
 */

typedef struct {
    const char    *key;             /* JSON key, e.g. "servo_min_us" */
    const int16_t *values;          /* signed for bias compatibility */
    size_t         count;           /* number of array elements */
} CFG_PatchEntry;

CFG_Status  CFG_PatchFile(const char *path,
                          const CFG_PatchEntry *entries, size_t n_entries,
                          char *err_msg, size_t err_msg_sz);

#ifdef __cplusplus
}
#endif

#endif  /* CPCU_CONFIG_H */

