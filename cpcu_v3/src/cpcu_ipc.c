/**
 *  @file   cpcu_ipc.c
 *  @brief  POSIX shared memory IPC — create, open, close, ring buffer, seqlock.
 *
 *  Manages a single /dev/shm/cpcu_ipc region containing:
 *    - Control block (192 B): system state, heartbeats, edit-mode flags.
 *    - Sensor ring (64 KB): lock-free SPSC ring buffer, 1024 entries.
 *    - Motor command (128 B): seqlock-protected servo targets.
 *    - Diagnostics (128 B): atomic counters per subsystem.
 *    - DSP export (256 B): gesture name, confidence, RMS per channel.
 *    - Runtime config: servo limits, smoother params, grip thresholds.
 *    - Tool presence (512 B): side-tool heartbeat registry for dashboard.
 *    - DSP filtered (6.4 KB): post-filter envelope for web visualization.
 */

#include "cpcu_ipc.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

/*============= INTERNAL: Sub-pointers =============================================================*/

static inline void ipc_map_ptrs(IPC_Context *ctx)
{
    uint8_t *b  =   (uint8_t *)ctx->base;
    size_t  off =   0;

    ctx->ctrl       =   (IPC_ControlBlock *)(b + off);
    off            +=   sizeof(IPC_ControlBlock);
    ctx->ring       =   (IPC_SensorEntry *)(b + off);
    off            +=   sizeof(IPC_SensorEntry) * IPC_SENSOR_RING_SIZE;
    ctx->motor      =   (IPC_MotorCommand *)(b + off);
    off            +=   sizeof(IPC_MotorCommand);
    ctx->diag       =   (IPC_Diagnostics *)(b + off);
    off            +=   sizeof(IPC_Diagnostics);
    ctx->dsp_export =   (IPC_DSPExport *)(b + off);
    off            +=   sizeof(IPC_DSPExport);
    ctx->config     =   (IPC_RuntimeConfig *)(b + off);     /* */
    off            +=   sizeof(IPC_RuntimeConfig);
    ctx->tool_presence = (IPC_ToolPresence *)(b + off);     /* */
    off            +=   sizeof(IPC_ToolPresence);
    ctx->dsp_filtered  = (IPC_DspFiltered *)(b + off);      /* */
}

/*============= IPC_Create =========================================================================*/

int IPC_Create(IPC_Context *ctx)
{
    shm_unlink(IPC_SHM_NAME);
    int shm_fd  =   shm_open(IPC_SHM_NAME, O_CREAT | O_RDWR, IPC_SHM_PERMS);
    if(shm_fd < 0)
    {
        perror("[IPC] shm_open");
        return -1;
    }

    if( ftruncate(shm_fd, IPC_SHM_SIZE) < 0 )
    {
        perror("[IPC] ftruncate");
        close(shm_fd);
        return -1;
    }

    /* MAP_POPULATE: pre-fault pages for mlockall compatibility */
    void *p     =   mmap(NULL, IPC_SHM_SIZE,
                         PROT_READ | PROT_WRITE,
                         MAP_SHARED | MAP_POPULATE,
                         shm_fd, 0);
    if(p == MAP_FAILED)
    {
        perror("[IPC] mmap");
        close(shm_fd);
        return -1;
    }

    memset(p, 0, IPC_SHM_SIZE);

    ctx->base           =   p;
    ctx->shm_fd         =   shm_fd;
    ipc_map_ptrs(ctx);

    ctx->ctrl->magic    =   IPC_MAGIC;
    ctx->ctrl->version  =   IPC_VERSION;
    atomic_store(&ctx->ctrl->system_state, IPC_STATE_INIT);
    atomic_store(&ctx->ctrl->sensor_head, 0);
    atomic_store(&ctx->ctrl->sensor_tail, 0);
    atomic_store(&ctx->motor->seq, 0);          /* SeqLock starts at 0 (even = stable) */

    printf("[IPC] Created %s: %zu bytes (ctrl=%zu | ring=%zu | motor=%zu | diag=%zu | export=%zu)\n",
            IPC_SHM_NAME, (size_t)(IPC_SHM_SIZE),
            sizeof(IPC_ControlBlock),
            sizeof(IPC_SensorEntry) * IPC_SENSOR_RING_SIZE,
            sizeof(IPC_MotorCommand),
            sizeof(IPC_Diagnostics),
            sizeof(IPC_DSPExport));
    
    return 0;
}

/*============= IPC_Open ===========================================================================*/

int IPC_Open(IPC_Context *ctx)
{
    int fd  =   shm_open(IPC_SHM_NAME, O_RDWR, 0);
    if(fd < 0)
    {
        perror("[IPC] shm_open");
        return -1;
    }

    void *p =   mmap(NULL, IPC_SHM_SIZE,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_POPULATE,
                     fd, 0);
    if(p == MAP_FAILED)
    {
        perror("[IPC] mmap");
        close(fd);
        return -1;
    }

    ctx->base   =   p;
    ctx->shm_fd =   fd;
    ipc_map_ptrs(ctx);

    if(ctx->ctrl->magic != IPC_MAGIC)
    {
        fprintf(stderr, "[IPC] Bad magic: got 0x%08X | expected 0x%08X\n",
                ctx->ctrl->magic, (uint32_t)IPC_MAGIC);
        munmap(p, IPC_SHM_SIZE);
        close(fd);
        return -1;
    }

    printf("[IPC] Opened %s: magic=0x%08X ver=0x%04X\n",
            IPC_SHM_NAME, ctx->ctrl->magic, ctx->ctrl->version);

    return 0;
}

/*============= IPC_Close / IPC_Destroy ============================================================*/

void IPC_Close(IPC_Context *ctx)
{
    if(ctx->base)       
    { 
        munmap(ctx->base, IPC_SHM_SIZE);  
        ctx->base   =   NULL; 
    }
    if(ctx->shm_fd >= 0)
    { 
        close(ctx->shm_fd);               
        ctx->shm_fd =   -1;   
    }
}

void IPC_Destroy(void)
{
    shm_unlink(IPC_SHM_NAME);
}

/*============= Ring Buffer: Push (Core 3 — sole producer) =========================================*/

void IPC_PushSensor(IPC_Context *ctx, const WL_Packet *pkt, uint64_t rx_time_us)
{
    uint32_t head       =   atomic_load_explicit(&ctx->ctrl->sensor_head, memory_order_relaxed);
    IPC_SensorEntry *e  =   &ctx->ring[head & IPC_SENSOR_RING_MASK];

    for(int s = 0; s < WL_SAMPLES_PER_PACKET; s++)
    {
        e->samples[s]   =   pkt->samples[s];
    }

    e->seq              =   pkt->seq;
    e->flags            =   pkt->flags;
    e->tx_retry         =   pkt->tx_retry;
    e->pkt_loss         =   pkt->pkt_loss;
    e->timestamp        =   pkt->timestamp;
    e->vbat_raw         =   pkt->vbat_raw;
    e->rx_time_us       =   rx_time_us;

    /* RELEASE: data visible before head increment */
    atomic_store_explicit(&ctx->ctrl->sensor_head, head + 1, memory_order_release);
}

/*============= Ring Buffer: Pop Batch (Cores 1-2 — sole consumer) =================================*/

uint32_t IPC_PopSensorBatch(IPC_Context *ctx, IPC_SensorEntry *out, uint32_t max_count)
{
    uint32_t tail   =   atomic_load_explicit(&ctx->ctrl->sensor_tail, memory_order_relaxed);
    uint32_t head   =   atomic_load_explicit(&ctx->ctrl->sensor_head, memory_order_acquire);
    uint32_t avail  =   head - tail;

    if(avail == 0)
    {
        return 0;
    }

    /* Overflow: producer lapped us — skip stale entries */
    if(avail > IPC_SENSOR_RING_SIZE)
    {
        uint32_t lost   =   avail - IPC_SENSOR_RING_SIZE;
        tail           +=   lost;
        avail           =   IPC_SENSOR_RING_SIZE;
        atomic_fetch_add(&ctx->diag->io_ring_overflows, lost);
    }

    uint32_t n  =   (avail < max_count) ? avail : max_count;
    for(uint32_t i = 0; i < n; i++)
    {
        out[i]  =   ctx->ring[(tail + i) & IPC_SENSOR_RING_MASK];
    }

    /* RELEASE: signal that we've consumed these entries */
    atomic_store_explicit(&ctx->ctrl->sensor_tail, tail + n, memory_order_release);

    return n;
}

/*============= Ring Buffer: Count =================================================================*/

uint32_t IPC_SensorCount(IPC_Context *ctx)
{
    uint32_t head   =   atomic_load(&ctx->ctrl->sensor_head);
    uint32_t tail   =   atomic_load(&ctx->ctrl->sensor_tail);
    uint32_t diff   =   head - tail;

    return (diff <= IPC_SENSOR_RING_SIZE) ? diff : IPC_SENSOR_RING_SIZE;
}

/*============= Motor Command: Write (SeqLock — Cores 1-2 writer) ==================================*/
/**
 *  @details    SeqLock write protocol:
 *              1. seq++    (even -> odd: write in progress)
 *              2. Write all data fields
 *              3. seq++    (odd -> even: write complete)
 */

void IPC_WriteMotorCmd(IPC_Context *ctx, const uint16_t servo_us[IPC_NUM_SERVOS],
                       uint8_t gesture_id, uint8_t confidence, uint64_t timestamp_us)
{
    /* Step 1: seq -> odd (write-in-progress) */
    atomic_fetch_add_explicit(&ctx->motor->seq, 1, memory_order_release);

    /* Step 2: write data */
    memcpy((void *)ctx->motor->servo_us, servo_us, sizeof(uint16_t) * IPC_NUM_SERVOS);
    ctx->motor->gesture_id      =   gesture_id;
    ctx->motor->confidence      =   confidence;
    ctx->motor->timestamp_us    =   timestamp_us;

    /* Step 3: seq -> even (write-complete, data consistent) */
    atomic_fetch_add_explicit(&ctx->motor->seq, 1, memory_order_release);
}

/*============= Motor Command: Read (SeqLock — Core 3 reader) ======================================*/
/**
 *  @details    Spins up to 4 times if writer is active.
 *              Returns false if no new data or all attempts failed.
 */

bool IPC_ReadMotorCmd(IPC_Context *ctx, uint16_t servo_us[IPC_NUM_SERVOS],
                      uint8_t *gesture_id, uint8_t *confidence, uint32_t *last_ack)
{
    for(int attempt = 0; attempt < 4; attempt++)
    {
        /* Step 1: read seq — must be EVEN and different from last ack */
        uint32_t s1     =   atomic_load_explicit(&ctx->motor->seq, memory_order_acquire);

        if(s1 == *last_ack)
        {
            return false;           /* No new data since last read */
        }

        if(s1 & 1)
        {
            continue;               /* Writer is mid-update, retry */
        }

        /* Step 2: read data */
        memcpy(servo_us, (const void *)ctx->motor->servo_us, sizeof(uint16_t) * IPC_NUM_SERVOS);
        *gesture_id     =   ctx->motor->gesture_id;
        *confidence     =   ctx->motor->confidence;

        /* Step 3: re-read seq — if unchanged, data is consistent */
        uint32_t s2     =   atomic_load_explicit(&ctx->motor->seq, memory_order_acquire);

        if(s1 == s2)
        {
            *last_ack   =   s2;
            return true;            /* Consistent snapshot obtained */
        }

        /* seq changed during read — data may be torn, retry */
    }

    return false;   /* Failed after 4 attempts */
}

/*============= Runtime Config (SeqLock) ===================================================*/
/**
 *  Writer (cpcu_kernel only) populates the entire IPC_RuntimeConfig
 *  in one go. The seqlock pattern is identical to MotorCmd but the
 *  payload is bigger (~512 bytes), so reads are slightly more
 *  expensive — fine because configuration is consulted once per
 *  loop iteration, not in tight inner loops.
 */

void IPC_WriteRuntimeConfig(IPC_Context *ctx, const IPC_RuntimeConfig *src)
{
    /* Step 1: seq -> odd */
    atomic_fetch_add_explicit(&ctx->config->config_seq, 1, memory_order_release);

    /* Step 2: copy payload (skip the seqlock header so we don't
     * clobber config_seq itself) */
    uint8_t *dst_bytes = (uint8_t *)ctx->config + sizeof(uint32_t);
    const uint8_t *src_bytes = (const uint8_t *)src + sizeof(uint32_t);
    memcpy(dst_bytes, src_bytes, sizeof(IPC_RuntimeConfig) - sizeof(uint32_t));

    /* Step 3: seq -> even */
    atomic_fetch_add_explicit(&ctx->config->config_seq, 1, memory_order_release);
}

bool IPC_ReadRuntimeConfig(IPC_Context *ctx, IPC_RuntimeConfig *dst)
{
    for(int attempt = 0; attempt < 4; attempt++)
    {
        uint32_t s1 = atomic_load_explicit(&ctx->config->config_seq,
                                           memory_order_acquire);
        if(s1 & 1) continue;                /* writer mid-update */

        memcpy(dst, ctx->config, sizeof(IPC_RuntimeConfig));

        uint32_t s2 = atomic_load_explicit(&ctx->config->config_seq,
                                           memory_order_acquire);
        if(s1 == s2)
        {
            /* Snapshot consistent. Surface the seq we read into the
             * destination so the caller can compare across reads. */
            dst->config_seq = s2;
            return dst->magic == IPC_CFG_VALID_MAGIC;
        }
    }
    return false;
}

uint32_t IPC_RuntimeConfigSeq(IPC_Context *ctx)
{
    return atomic_load_explicit(&ctx->config->config_seq, memory_order_acquire);
}

/*==================================================================================================*/

