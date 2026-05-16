/**
 *  @file   cpcu_kernel.c
 *  @brief  Core 0 supervisor — creates IPC, spawns child processes, monitors health.
 *
 *  Responsibilities:
 *    1. Create /dev/shm/cpcu_ipc shared memory region.
 *    2. Load runtime.json into IPC_RuntimeConfig.
 *    3. Fork+exec cpcu_io (Core 3, SCHED_FIFO 90).
 *    4. Fork+exec cpcu_dsp.py (Cores 1-2, SCHED_FIFO 80).
 *    5. Monitor heartbeats; restart dead children.
 *    6. Pet /dev/watchdog (15 s hardware timeout).
 *    7. Reload config on SIGHUP without restarting children.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sched.h>
#include <linux/watchdog.h>

#include "cpcu_ipc.h"
#include "cpcu_log.h"
#include "cpcu_config.h"

/*============= CONFIGURATION ==============================================================*/

#define HB_TIMEOUT_MS           2000            /* Heartbeat stale -> kill + respawn */
#define WDG_PET_S               5               /* Pet /dev/watchdog every N seconds */
#define LOG_S                   5               /* Telemetry print every N seconds */
#define READY_TIMEOUT_S         10              /* Max wait for child ready flag */

/* Paths */
#define CPCU_IO_BIN             "./cpcu_io"
#define CPCU_DSP_SCRIPT         "/opt/cpcu/python/cpcu_dsp.py"
#define CPCU_DSP_SCRIPT_ALT     "./cpcu_dsp.py"
#define PYTHON3_BIN             "/usr/bin/python3"

/* runtime config path. The default points to the symlinked
 * system path that setup_pi.sh creates (-> repo's cpcu_v2/config/runtime.json).
 * Override with --config <path> for testing. */
#define CONFIG_PATH_DEFAULT     "/opt/cpcu/config.json"
#define CONFIG_PATH_FALLBACK    "config/runtime.json"

/*============= TIMING =====================================================================*/

static volatile sig_atomic_t g_run          =   1;
static volatile sig_atomic_t g_reload_cfg   =   0;      /* */

static void on_sig(int s)
{
    (void)s; g_run = 0;
}

static void on_sighup(int s)
{
    (void)s; g_reload_cfg = 1;
}

/* Load runtime.json and publish to IPC.
 * Returns 0 on success, non-zero on parse failure (caller decides
 * whether to abort startup or keep the previous config). */
static int kern_load_config(IPC_Context *ipc, const char *path)
{
    IPC_RuntimeConfig cfg;
    char err[256] = {0};
    CFG_Status st = CFG_LoadFromFile(path, &cfg, err, sizeof(err));
    if(st != CFG_OK)
    {
        LOG_E("KERN", "config load failed: %s (%s) — %s",
              CFG_StatusStr(st), path, err);
        return -1;
    }
    IPC_WriteRuntimeConfig(ipc, &cfg);
    LOG_I("KERN", "runtime config loaded from %s (schema=%u, seq=%u)",
          path, cfg.schema_version, IPC_RuntimeConfigSeq(ipc));
    return 0;
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

/*============= GLOBALS ====================================================================*/

static bool g_forward_log_flag = false;

/*============= PROCESS SPAWNING ===========================================================*/
/**
 *  Spawn a C binary with CPU affinity and real-time priority.
 *
 *  Uses sched_setscheduler() and sched_setaffinity() in the
 *  parent process (cpcu_kernel) BEFORE exec, instead of going through
 *  `taskset → chrt → bin`. The reason is capability inheritance.
 *
 *  Linux file capabilities (set with `setcap cap_sys_nice+ep
 *  /opt/cpcu/bin/cpcu_kernel`) are dropped across exec() unless the
 *  caps are explicitly inheritable AND in the ambient set. The default
 *  +ep mode is effective+permitted only, so `taskset` runs without
 *  CAP_SYS_NICE, and `chrt`'s eventual sched_setscheduler() call gets
 *  EPERM. That's the root cause of the long-running "cpcu_io fails to
 *  achieve RT priority" issue.
 *
 *  The fix: do the schedAffinity work in the parent, where CAP_SYS_NICE
 *  is still in effect (no exec boundary has been crossed yet). Then
 *  exec() the binary directly. The scheduling attributes carry through
 *  exec because they're per-task, not per-binary.
 *
 *  If --log was passed to the kernel, pass it through to the child so
 *  both cpcu_kernel and cpcu_io write CSV logs in /var/log/cpcu/.
 */

static int parse_cpu_list(const char *cores, cpu_set_t *set)
{
    CPU_ZERO(set);
    const char *p                   =   cores;
    while(*p)
    {
        char *end                   =   NULL;
        long start                  =   strtol(p, &end, 10);
        if(end == p) return -1;
        long stop                   =   start;
        if(*end == '-')
        {
            p                       =   end + 1;
            stop                    =   strtol(p, &end, 10);
            if(end == p) return -1;
        }
        for(long c = start; c <= stop; c++)
        {
            if(c < 0 || c >= CPU_SETSIZE) return -1;
            CPU_SET((int)c, set);
        }
        if(*end == ',') { p = end + 1; continue; }
        if(*end == '\0') break;
        return -1;
    }
    return 0;
}

static pid_t spawn_native(const char *label, const char *bin,
                           const char *cores, int prio)
{
    pid_t pid                       =   fork();
    if(pid == 0)
    {
        /* === Child: parent's caps still in effect until exec() === */

        /* 1. Pin to CPU set */
        cpu_set_t set;
        if(parse_cpu_list(cores, &set) != 0)
        {
            fprintf(stderr, "[KERNEL] %s: bad cpu list '%s'\n", label, cores);
            _exit(1);
        }
        if(sched_setaffinity(0, sizeof(set), &set) != 0)
        {
            fprintf(stderr, "[KERNEL] %s: sched_setaffinity(%s) failed: %s\n",
                    label, cores, strerror(errno));
            /* Non-fatal: the kernel will run on whatever CPUs are available */
        }

        /* 2. Set SCHED_FIFO priority. This is the key step that needs
         *    CAP_SYS_NICE — and because we haven't exec'd yet, the
         *    parent's file caps are still our caps. */
        struct sched_param sp        =   { .sched_priority = prio };
        if(sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        {
            fprintf(stderr, "[KERNEL] %s: sched_setscheduler(FIFO,%d) failed: %s\n",
                    label, prio, strerror(errno));
            /* Hard fail — RT priority is required for the IO process to
             * meet its 1 ms deadline */
            _exit(2);
        }

        /* 3. Now exec the actual binary. Scheduling attributes are
         *    per-task and survive exec; file caps don't matter on the
         *    child binary. */
        if(g_forward_log_flag)
        {
            execl(bin, bin, "--log", NULL);
        }
        else
        {
            execl(bin, bin, NULL);
        }
        /* execl returned -> error */
        fprintf(stderr, "[KERNEL] exec %s failed: %s\n", label, strerror(errno));
        _exit(1);
    }

    if(pid > 0)
        LOG_I("KERN", "spawned %s pid=%d cores=%s prio=%d", label, pid, cores, prio);
    else
        LOG_E("KERN", "fork %s failed: %s", label, strerror(errno));

    return pid;
}

/**
 *  Spawn a Python script with CPU affinity and real-time priority.
 */

static pid_t spawn_python(const char *label, const char *script,
                           const char *cores, int prio)
{
    /* Check if script exists at primary path, fall back to alt */
    const char *actual_script       =   script;
    if(access(script, F_OK) != 0)
    {
        actual_script               =   CPCU_DSP_SCRIPT_ALT;
        if(access(actual_script, F_OK) != 0)
        {
            LOG_E("KERN", "%s: script not found at %s or %s",
                  label, script, CPCU_DSP_SCRIPT_ALT);
            return -1;
        }
    }

    pid_t pid                       =   fork();
    if(pid == 0)
    {
        /* Same parent-side scheduling setup as spawn_native — see the
         * detailed comment there. The python binary itself doesn't need
         * any caps; the scheduling attributes carry through exec. */
        cpu_set_t set;
        if(parse_cpu_list(cores, &set) != 0)
        {
            fprintf(stderr, "[KERNEL] %s: bad cpu list '%s'\n", label, cores);
            _exit(1);
        }
        if(sched_setaffinity(0, sizeof(set), &set) != 0)
        {
            fprintf(stderr, "[KERNEL] %s: sched_setaffinity(%s) failed: %s\n",
                    label, cores, strerror(errno));
        }
        struct sched_param sp        =   { .sched_priority = prio };
        if(sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        {
            fprintf(stderr, "[KERNEL] %s: sched_setscheduler(FIFO,%d) failed: %s\n",
                    label, prio, strerror(errno));
            _exit(2);
        }

        execl(PYTHON3_BIN, PYTHON3_BIN, actual_script, NULL);
        fprintf(stderr, "[KERNEL] exec %s failed: %s\n", label, strerror(errno));
        _exit(1);
    }

    if(pid > 0)
        LOG_I("KERN", "spawned %s pid=%d cores=%s prio=%d script=%s",
              label, pid, cores, prio, actual_script);
    else
        LOG_E("KERN", "fork %s failed: %s", label, strerror(errno));

    return pid;
}

/*============= READY WAIT =================================================================*/

static int wait_ready(const char *label, _Atomic uint8_t *flag, int timeout_s)
{
    for(int i = 0; i < timeout_s * 10; i++)
    {
        if(atomic_load(flag))
        {
            LOG_I("KERN", "%s ready", label);
            return 1;
        }
        usleep(100000);
    }

    LOG_W("KERN", "%s not ready within %ds", label, timeout_s);
    return 0;
}

/*============= MAIN =======================================================================*/

int main(int argc, char *argv[])
{
    Log_Init("KERN", LOG_INFO);

    const char *config_path = CONFIG_PATH_DEFAULT;

    /* Parse CLI */
    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--log") == 0)
        {
            g_forward_log_flag = true;
        }
        else if(strcmp(argv[i], "--debug") == 0)
        {
            Log_SetLevel(LOG_DEBUG);
        }
        else if(strcmp(argv[i], "--config") == 0 && i + 1 < argc)
        {
            config_path = argv[++i];
        }
    }
    if(g_forward_log_flag)
    {
        Log_EnableFiles(LOG_DIR_DEFAULT);
        LOG_I("KERN", "file logging enabled -> %s/log_*.csv", LOG_DIR_DEFAULT);
    }

    LOG_I("KERN", "=== CPCU Kernel Supervisor (Core 0) v2.3.8 ===");
    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);
    signal(SIGHUP,  on_sighup);                 /* */

    /* Create shared memory */
    IPC_Context ipc;
    if(IPC_Create(&ipc) != 0)
    {
        LOG_F("KERN", "IPC_Create failed");
        Log_CloseFiles();
        return 1;
    }

    /* publish our pid so the TUI live editor can SIGHUP us
     * after a Ctrl+S commit. The TUI sees kernel_pid == 0 until
     * IPC_Create returned, so we publish immediately after. */
    atomic_store(&ipc.ctrl->kernel_pid, (uint32_t)getpid());

    /* load runtime config BEFORE spawning children, so they
     * see a populated config region from their first IPC read. Try
     * the user-supplied path first; if that fails AND it was the
     * default symlink, try the in-repo path as a graceful fallback
     * for development workflows where setup_pi.sh hasn't run. If
     * BOTH fail we refuse to start — no silent defaults. */
    int cfg_ret = kern_load_config(&ipc, config_path);
    if(cfg_ret != 0 &&
       strcmp(config_path, CONFIG_PATH_DEFAULT) == 0)
    {
        LOG_W("KERN", "default config path failed, trying %s",
              CONFIG_PATH_FALLBACK);
        cfg_ret = kern_load_config(&ipc, CONFIG_PATH_FALLBACK);
    }
    if(cfg_ret != 0)
    {
        LOG_F("KERN", "no usable runtime config — refusing to start.");
        LOG_F("KERN", "  To recover: run './launch.sh configure --reset --runtime'");
        LOG_F("KERN", "  Or restore from git: 'git checkout HEAD -- config/runtime.json'");
        LOG_F("KERN", "  See cpcu_v2/docs/CONFIGURATION.md for the full schema.");
        Log_CloseFiles();
        return 1;
    }

    /* Hardware watchdog (systemd usually holds it — EBUSY is non-fatal) */
    int wdg                         =   open("/dev/watchdog", O_WRONLY);
    if(wdg >= 0)
    {
        int timeout                 =   15;
        if(ioctl(wdg, WDIOC_SETTIMEOUT, &timeout) < 0)
        {
            LOG_W("WDG", "WDIOC_SETTIMEOUT failed: %s", strerror(errno));
        }
        LOG_I("WDG", "enabled (%ds timeout)", timeout);
    }
    else if(errno == EBUSY)
    {
        LOG_I("WDG", "held by another process (likely systemd) — skipping");
    }
    else
    {
        LOG_I("WDG", "not available (non-fatal): %s", strerror(errno));
    }

    /* Spawn cpcu_io on Core 3 */
    pid_t io_pid                    =   spawn_native("cpcu_io", CPCU_IO_BIN, "3", 90);
    if(io_pid < 0)
    {
        IPC_Close(&ipc);
        IPC_Destroy();
        Log_CloseFiles();
        return 1;
    }

    if(!wait_ready("cpcu_io", &ipc.ctrl->io_ready, READY_TIMEOUT_S))
    {
        LOG_F("KERN", "cpcu_io failed to become ready");
        kill(io_pid, SIGTERM);
        waitpid(io_pid, NULL, 0);
        IPC_Close(&ipc);
        IPC_Destroy();
        Log_CloseFiles();
        return 1;
    }

    /* Spawn cpcu_dsp.py on Cores 1-2 */
    pid_t dsp_pid                   =   spawn_python("cpcu_dsp", CPCU_DSP_SCRIPT, "1,2", 80);
    if(dsp_pid > 0)
        wait_ready("cpcu_dsp", &ipc.ctrl->dsp_ready, READY_TIMEOUT_S);

    LOG_I("KERN", "all processes up. Monitoring...");

    /* Monitoring loop */
    time_t t_log                    =   time(NULL);
    time_t t_pet                    =   time(NULL);

    while(g_run)
    {
        sleep(1);
        time_t now                  =   time(NULL);

        /* SIGHUP reload. Re-parse the same path, republish to
         * IPC. On parse failure, KEEP the previous config (writers
         * already populated IPC at startup) and log loudly. We don't
         * want a typo in runtime.json to take down a running session. */
        if(g_reload_cfg)
        {
            g_reload_cfg            =   0;
            LOG_I("KERN", "SIGHUP — reloading runtime config from %s",
                  config_path);
            int rr                  =   kern_load_config(&ipc, config_path);
            if(rr != 0)
            {
                LOG_E("KERN", "SIGHUP reload failed — keeping previous config");
            }
        }

        /* Check if children died */
        int st;
        pid_t dead                  =   waitpid(-1, &st, WNOHANG);

        if(dead == io_pid)
        {
            LOG_W("KERN", "cpcu_io died (status=%d) — restarting", st);
            io_pid                  =   spawn_native("cpcu_io", CPCU_IO_BIN, "3", 90);
        }

        if(dead == dsp_pid)
        {
            LOG_W("KERN", "cpcu_dsp died (status=%d) — restarting", st);
            dsp_pid                 =   spawn_python("cpcu_dsp", CPCU_DSP_SCRIPT, "1,2", 80);
        }

        /* Heartbeat check (cpcu_io writes every 100ms) */
        uint64_t hb                 =   atomic_load(&ipc.ctrl->io_heartbeat_us);
        if(hb > 0 && (now_us() - hb) / 1000 > HB_TIMEOUT_MS)
        {
            LOG_E("KERN", "cpcu_io heartbeat stale (%llu ms) — killing",
                  (unsigned long long)((now_us() - hb) / 1000));
            kill(io_pid, SIGKILL);
            waitpid(io_pid, NULL, 0);
            io_pid                  =   spawn_native("cpcu_io", CPCU_IO_BIN, "3", 90);
        }

        /* Telemetry */
        if(now - t_log >= LOG_S)
        {
            t_log                   =   now;
            LOG_I("KERN",
                  "state=%u pkts=%u gaps=%u ovf=%u inf=%u maxlat=%u "
                  "ring=%u io_pid=%d dsp_pid=%d",
                  atomic_load(&ipc.ctrl->system_state),
                  atomic_load(&ipc.diag->io_pkts_received),
                  atomic_load(&ipc.diag->io_seq_gaps),
                  atomic_load(&ipc.diag->io_ring_overflows),
                  atomic_load(&ipc.diag->dsp_inferences),
                  atomic_load(&ipc.diag->dsp_max_latency_us),
                  IPC_SensorCount(&ipc),
                  io_pid, dsp_pid);
        }

        /* Pet watchdog */
        if(wdg >= 0 && now - t_pet >= WDG_PET_S)
        {
            t_pet                   =   now;
            write(wdg, "V", 1);
        }
    }

    /* Shutdown */
    LOG_I("KERN", "shutting down");
    if(io_pid > 0)  { kill(io_pid, SIGTERM);    waitpid(io_pid, NULL, 0);   }
    if(dsp_pid > 0) { kill(dsp_pid, SIGTERM);   waitpid(dsp_pid, NULL, 0);  }
    IPC_Close(&ipc);
    IPC_Destroy();
    if(wdg >= 0) close(wdg);
    Log_CloseFiles();

    return 0;
}

