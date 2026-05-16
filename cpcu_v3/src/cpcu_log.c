/**
 *  @file   cpcu_log.c
 *  @brief  Structured logging — colored stderr + optional per-module CSV files.
 *
 *  Provides LOG_I/W/E/D/F macros with module tag, timestamp, and color.
 *  When file logging is enabled (--log flag), each module writes to a
 *  separate CSV file in /var/log/cpcu/ for post-session analysis.
 */

#include "cpcu_log.h"
#include <stdbool.h>    /* defensive — already pulled in by cpcu_log.h,
                           but explicit here so an older header still
                           compiles past the bool declaration below.      */

/*============= CORE STATE =========================================================*/

LogLevel        g_log_level =   LOG_INFO;
const char     *g_log_proc  =   "????";
int             g_log_color =   0;
struct timespec g_log_boot  =   {0, 0};

/*============= FILE-SINK STATE ====================================================*/

bool            g_log_to_file       =   false;
char            g_log_dir[128]      =   LOG_DIR_DEFAULT;
LogFileSink     g_log_sinks[LOG_MAX_MODULES] = {{{0}, NULL}};
int             g_log_sink_count    =   0;

