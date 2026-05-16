/**
 *  @file   cpcu_log.h
 *  @brief  Logging API — LOG_I/W/E/D/F macros with module tag and CSV sinks.
 */

#ifndef CPCU_LOG_H
#define CPCU_LOG_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/types.h>

/*============= LOG LEVELS ============================================================*/

typedef enum {
    LOG_TRACE   =   0,
    LOG_DEBUG   =   1,
    LOG_INFO    =   2,
    LOG_WARN    =   3,
    LOG_ERROR   =   4,
    LOG_FATAL   =   5,
} LogLevel;

/*============= ANSI COLORS ===========================================================*/

#define ANSI_RESET      "\033[0m"
#define ANSI_DIM        "\033[2m"
#define ANSI_CYAN       "\033[36m"
#define ANSI_GREEN      "\033[32m"
#define ANSI_YELLOW     "\033[33m"
#define ANSI_RED        "\033[31m"
#define ANSI_BOLD_RED   "\033[1;31m"
#define ANSI_WHITE      "\033[37m"
#define ANSI_BOLD       "\033[1m"

/*============= FILE SINK =============================================================*/

#define LOG_MAX_MODULES     16
#define LOG_MODULE_NAMELEN  8
#define LOG_DIR_DEFAULT     "/var/log/cpcu"

typedef struct
{
    char    name[LOG_MODULE_NAMELEN];
    FILE   *fp;
} LogFileSink;

/*============= GLOBAL STATE ==========================================================*/

extern  LogLevel        g_log_level;
extern  const char     *g_log_proc;
extern  int             g_log_color;
extern  struct timespec g_log_boot;

extern  bool            g_log_to_file;
extern  char            g_log_dir[128];
extern  LogFileSink     g_log_sinks[LOG_MAX_MODULES];
extern  int             g_log_sink_count;

/*============= INIT ==================================================================*/

static inline void Log_Init(const char *proc_name, LogLevel level)
{
    g_log_proc  =   proc_name;
    g_log_level =   level;
    g_log_color =   isatty(STDERR_FILENO);
    clock_gettime(CLOCK_MONOTONIC, &g_log_boot);
}

static inline void Log_SetLevel(LogLevel level)
{
    g_log_level =   level;
}

/*============= FILE-SINK API =========================================================*/

/**
 *  Enable CSV file logging. Writes to <dir>/log_{module}.csv, one file
 *  per module name (lowercased). Directory must already exist (launch.sh
 *  does `mkdir -p /var/log/cpcu`).
 */
static inline void Log_EnableFiles(const char *dir)
{
    g_log_to_file =   true;
    if(dir != NULL && dir[0] != '\0')
    {
        size_t n  =   strlen(dir);
        if(n >= sizeof(g_log_dir)) n = sizeof(g_log_dir) - 1;
        memcpy(g_log_dir, dir, n);
        g_log_dir[n] = '\0';
    }
    else
    {
        snprintf(g_log_dir, sizeof(g_log_dir), "%s", LOG_DIR_DEFAULT);
    }
    /* Best-effort mkdir; launch.sh usually did this already */
    mkdir(g_log_dir, 0755);
}

/**
 *  Close all open file sinks. Safe to call during cleanup.
 */
static inline void Log_CloseFiles(void)
{
    for(int i = 0; i < g_log_sink_count; i++)
    {
        if(g_log_sinks[i].fp != NULL)
        {
            fclose(g_log_sinks[i].fp);
            g_log_sinks[i].fp = NULL;
        }
    }
    g_log_sink_count    =   0;
    g_log_to_file       =   false;
}

/**
 *  Find or lazily create a file sink for the given module name.
 *  Returns NULL if the module table is full or fopen() fails.
 */
static inline FILE *log_get_sink(const char *module)
{
    /* Search existing */
    for(int i = 0; i < g_log_sink_count; i++)
    {
        if(strncmp(g_log_sinks[i].name, module, LOG_MODULE_NAMELEN) == 0)
            return g_log_sinks[i].fp;
    }

    if(g_log_sink_count >= LOG_MAX_MODULES)  return NULL;

    /* Build path: <dir>/log_<module-lower>.csv */
    char path[192];
    int  written = snprintf(path, sizeof(path), "%s/log_", g_log_dir);
    if(written < 0 || written >= (int)sizeof(path)) return NULL;

    int p = written;
    for(int i = 0; module[i] != '\0' && p < (int)sizeof(path) - 5; i++, p++)
    {
        char c = module[i];
        if(c >= 'A' && c <= 'Z')  c = (char)(c + 32);
        path[p] = c;
    }
    path[p++] = '.';
    path[p++] = 'c';
    path[p++] = 's';
    path[p++] = 'v';
    path[p]   = '\0';

    FILE *fp = fopen(path, "a");
    if(fp == NULL) return NULL;

    /* If the file is brand-new (size 0) write a CSV header */
    fseek(fp, 0, SEEK_END);
    if(ftell(fp) == 0)
        fprintf(fp, "timestamp_s,timestamp_us,proc,level,message\n");

    /* Register the sink */
    LogFileSink *s = &g_log_sinks[g_log_sink_count];
    size_t nm = strlen(module);
    if(nm >= LOG_MODULE_NAMELEN) nm = LOG_MODULE_NAMELEN - 1;
    memcpy(s->name, module, nm);
    s->name[nm] = '\0';
    s->fp       = fp;
    g_log_sink_count++;
    return fp;
}

/*============= TIMESTAMP =============================================================*/

static inline uint64_t log_timestamp_us(void)
{
    struct  timespec    now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return ( (uint64_t)((now.tv_sec - g_log_boot.tv_sec) * 1000000ULL)
           + (uint64_t)((now.tv_nsec - g_log_boot.tv_nsec) / 1000ULL) );
}

/*============= LEVEL STRINGS =========================================================*/

static inline const char *log_level_str(LogLevel level)
{
    switch(level)
    {
        case LOG_TRACE:     return  "TRACE";
        case LOG_DEBUG:     return  "DEBUG";
        case LOG_INFO:      return  "INFO ";
        case LOG_WARN:      return  "WARN ";
        case LOG_ERROR:     return  "ERROR";
        case LOG_FATAL:     return  "FATAL";
        default:            return  "?????";
    }
}

static inline const char *log_level_color(LogLevel level)
{
    switch(level)
    {
        case LOG_TRACE:     return  ANSI_DIM;
        case LOG_DEBUG:     return  ANSI_CYAN;
        case LOG_INFO:      return  ANSI_GREEN;
        case LOG_WARN:      return  ANSI_YELLOW;
        case LOG_ERROR:     return  ANSI_RED;
        case LOG_FATAL:     return  ANSI_BOLD_RED;
        default:            return  ANSI_RESET;
    }
}

/*============= LOG MACRO =============================================================*/

#define LOG(level, module, fmt, ...) \
    do \
    {\
        if( (level) >= g_log_level )\
        {\
            uint64_t _ts    =   log_timestamp_us();\
            uint32_t _ts_s  =   (uint32_t)(_ts / 1000000ULL);\
            uint32_t _ts_us =   (uint32_t)(_ts % 1000000ULL);\
            if(g_log_color)\
            {\
                fprintf(stderr,\
                        ANSI_DIM "[%06u.%06u]" ANSI_RESET " "\
                        ANSI_BOLD "%-4s" ANSI_RESET " " ANSI_DIM "|" ANSI_RESET " "\
                        ANSI_WHITE "%-4s" ANSI_RESET " " ANSI_DIM "|" ANSI_RESET " "\
                        "%s%-5s" ANSI_RESET " " ANSI_DIM "|" ANSI_RESET " "\
                        fmt "\n",\
                        _ts_s, _ts_us, g_log_proc, module,\
                        log_level_color(level), log_level_str(level),\
                        ##__VA_ARGS__);\
            }\
            else\
            {\
                fprintf(stderr,\
                        "[%06u.%06u] %-4s | %-4s | %-5s | " fmt "\n",\
                        _ts_s, _ts_us, g_log_proc, module,\
                        log_level_str(level),\
                        ##__VA_ARGS__);\
            }\
            if(g_log_to_file)\
            {\
                FILE *_sink = log_get_sink(module);\
                if(_sink != NULL)\
                {\
                    fprintf(_sink, "%06u,%06u,%s,%s,\"" fmt "\"\n",\
                            _ts_s, _ts_us, g_log_proc,\
                            log_level_str(level),\
                            ##__VA_ARGS__);\
                    fflush(_sink);\
                }\
            }\
        }\
    }\
    while(0)

/*============= LOG ALIASES ===========================================================*/

#define LOG_T(module, fmt, ...) LOG(LOG_TRACE,  module, fmt,    ##__VA_ARGS__)
#define LOG_D(module, fmt, ...) LOG(LOG_DEBUG,  module, fmt,    ##__VA_ARGS__)
#define LOG_I(module, fmt, ...) LOG(LOG_INFO,   module, fmt,    ##__VA_ARGS__)
#define LOG_W(module, fmt, ...) LOG(LOG_WARN,   module, fmt,    ##__VA_ARGS__)
#define LOG_E(module, fmt, ...) LOG(LOG_ERROR,  module, fmt,    ##__VA_ARGS__)
#define LOG_F(module, fmt, ...) LOG(LOG_FATAL,  module, fmt,    ##__VA_ARGS__)

#endif /* CPCU_LOG_H */

