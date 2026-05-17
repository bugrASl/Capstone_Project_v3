#!/usr/bin/env python3
"""
cpcu_ipc_bridge.py — Python interface to /dev/shm/cpcu_ipc

Mirrors cpcu_ipc.h current binary layout exactly.
Uses mmap + struct for zero-copy shared memory access.

Author: bugrASl
Date:   April 2026

CRITICAL: If you change ANY struct size or field order in cpcu_ipc.h,
          you MUST update the offsets here AND run test_ipc_offsets.py.
"""

import mmap
import os
import struct
import time
import numpy as np

# ══════════════════════════════════════════════════════════════════════
#  CONSTANTS — must match cpcu_ipc.h exactly
# ══════════════════════════════════════════════════════════════════════

IPC_SHM_PATH            =   "/dev/shm/cpcu_ipc"
IPC_MAGIC               =   0x494E4654
IPC_VERSION             =   0x0300          # IPC_ToolPresence + IPC_DspFiltered

RING_SIZE               =   1024
RING_MASK               =   RING_SIZE - 1
NUM_CHANNELS            =   8
SAMPLES_PER_PKT         =   2
NUM_SERVOS              =   6
MAX_GESTURE_NAME        =   16
MAX_CLASSES             =   10

# ══════════════════════════════════════════════════════════════════════
#  STRUCT SIZES — from _Static_assert in cpcu_ipc.h
# ══════════════════════════════════════════════════════════════════════

SZ_CTRL                 =   192         # 3 cache lines
SZ_ENTRY                =   64          # 1 cache line
SZ_MOTOR                =   128         # 2 cache lines
SZ_DIAG                 =   128         # 2 cache lines
SZ_EXPORT               =   256         # 4 cache lines
SZ_RING                 =   SZ_ENTRY * RING_SIZE    # 65536
SZ_CONFIG               =   512         # IPC_RuntimeConfig
SZ_TOOL_PRESENCE        =   512         # 8 slots × 64 B
SZ_DSP_FILTERED         =   6432        # 32 B header + 8 ch × 200 samples × 4 B

# ══════════════════════════════════════════════════════════════════════
#  SECTION OFFSETS — sequential in shared memory
# ══════════════════════════════════════════════════════════════════════

OFF_CTRL                =   0
OFF_RING                =   OFF_CTRL + SZ_CTRL                  # 192
OFF_MOTOR               =   OFF_RING + SZ_RING                  # 65728
OFF_DIAG                =   OFF_MOTOR + SZ_MOTOR                # 65856
OFF_EXPORT              =   OFF_DIAG + SZ_DIAG                  # 65984
OFF_CONFIG              =   OFF_EXPORT + SZ_EXPORT              # 66240
OFF_TOOL_PRESENCE       =   OFF_CONFIG + SZ_CONFIG              # 66752
OFF_DSP_FILTERED        =   OFF_TOOL_PRESENCE + SZ_TOOL_PRESENCE # 67264
SHM_TOTAL               =   OFF_DSP_FILTERED + SZ_DSP_FILTERED   # 73696

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_ControlBlock (192 bytes)
# ══════════════════════════════════════════════════════════════════════
#  Cache line 0 (0-63): header
#    magic(4) + version(2) + io_ready(1) + dsp_ready(1)
#    + system_state(1) + pad(3) + heartbeat(8) + motor_cmd_ack(4) + reserved(36)
#  Cache line 1 (64-127): sensor_head(4) + pad(60)
#  Cache line 2 (128-191): sensor_tail(4) + pad(60)

CTRL_MAGIC              =   0
CTRL_VERSION            =   4
CTRL_IO_READY           =   6
CTRL_DSP_READY          =   7
CTRL_STATE              =   8
CTRL_HEARTBEAT          =   12          # uint64
CTRL_MOTOR_ACK          =   20          # uint32

# edit-mode handshake bytes (live in the cache-line 0 reserve region)
CTRL_EDIT_REQUEST       =   24          # uint8 — TUI -> world
CTRL_EDIT_ACTIVE        =   25          # uint8 — io  -> TUI
CTRL_EDIT_DSP_ACK       =   26          # uint8 — dsp -> TUI (ack flag)
CTRL_EDIT_REQUEST_US    =   32          # uint64 (5B pad at 27..31 for align)

CTRL_HEAD               =   64          # cache line 1
CTRL_TAIL               =   128         # cache line 2

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_SensorEntry (64 bytes)
# ══════════════════════════════════════════════════════════════════════
#  samples[0]: 8 x uint16 = 16 bytes (offset 0)
#  samples[1]: 8 x uint16 = 16 bytes (offset 16)
#  seq(1) flags(1) tx_retry(1) pkt_loss(1) timestamp(2) vbat_raw(2) rx_time(8) pad(16)

ENTRY_SAMPLES           =   0           # 32 bytes total
ENTRY_SEQ               =   32
ENTRY_FLAGS             =   33
ENTRY_RETRY             =   34
ENTRY_LOSS              =   35
ENTRY_TIMESTAMP         =   36
ENTRY_VBAT              =   38
ENTRY_RXTIME            =   40

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_MotorCommand (128 bytes)
# ══════════════════════════════════════════════════════════════════════
#  seq(4) + servo_us[6](12) + gesture_id(1) + confidence(1) + pad(2)
#  + timestamp_us(8) + pad(28) + reserved(64)

MOTOR_SEQ               =   0
MOTOR_SERVO             =   4           # 6 x uint16
MOTOR_GESTURE           =   16
MOTOR_CONF              =   17
MOTOR_TIMESTAMP         =   20          # uint64 (after 2 bytes pad at 18)

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_Diagnostics (128 bytes)
# ══════════════════════════════════════════════════════════════════════

DIAG_PKTS               =   0
DIAG_DROPPED            =   4
DIAG_OVERFLOWS          =   8
DIAG_GAPS               =   12
DIAG_NRF_STATUS         =   16
DIAG_SAFE               =   20
DIAG_MAXPOLL            =   24
DIAG_BATCHES            =   28
DIAG_MAXLAT             =   32
DIAG_UNDERFLOWS         =   36
DIAG_INFERENCES         =   40

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_DSPExport (256 bytes)
# ══════════════════════════════════════════════════════════════════════
#  channel_rms[8](32) + gesture_name[16](16) + class_confidence[10](40)
#  + num_classes(1) + active_class(1) + pad(2) + inference_time_us(4)
#  + update_seq(4) + pad(132)

EXPORT_RMS              =   0           # 8 x float32 = 32 bytes
EXPORT_NAME             =   32          # 16 bytes
EXPORT_CONFIDENCE       =   48          # 10 x float32 = 40 bytes
EXPORT_NUM_CLASSES      =   88
EXPORT_ACTIVE_CLASS     =   89
EXPORT_INF_TIME         =   92          # uint32
EXPORT_UPDATE_SEQ       =   96          # uint32

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_DspFiltered
# ══════════════════════════════════════════════════════════════════════
#  Layout (matches cpcu_ipc.h):
#    seq(4) + sample_rate_hz(4) + update_us(8) + _pad0(16) = 32 B header
#    channel[8][200] of float32 = 6400 B payload
#  Total: 6432 B.

DSPFILT_SEQ             =   0           # uint32, odd = writer in progress
DSPFILT_SAMPLE_RATE     =   4           # uint32 (Hz, e.g. 200)
DSPFILT_UPDATE_US       =   8           # uint64 (monotonic time)
DSPFILT_HEADER_BYTES    =   32          # data starts here
DSPFILT_NUM_CHANNELS    =   8
DSPFILT_NUM_SAMPLES     =   200         # 1 s of envelope @ 200 Hz
DSPFILT_BYTES_PER_CH    =   DSPFILT_NUM_SAMPLES * 4

# ══════════════════════════════════════════════════════════════════════
#  FIELD OFFSETS within IPC_ToolPresence slot
# ══════════════════════════════════════════════════════════════════════
#  Slot layout (64 B):
#    alive(1) + _pad0(7) + last_heartbeat_us(8) + tool_name[16]
#    + payload[32]

TOOL_SLOT_BYTES         =   64
TOOL_SLOT_ALIVE         =   0
TOOL_SLOT_HEARTBEAT_US  =   8
TOOL_SLOT_NAME          =   16          # 16 bytes
TOOL_SLOT_PAYLOAD       =   32          # 32 bytes

# ══════════════════════════════════════════════════════════════════════
#  IPC STATE CONSTANTS
# ══════════════════════════════════════════════════════════════════════

IPC_STATE_INIT          =   0
IPC_STATE_RUNNING       =   1
IPC_STATE_SAFE          =   2


class IPCBridge:
    """
    Read/write interface to CPCU shared memory.
    
    Usage:
        ipc = IPCBridge()
        ipc.set_dsp_ready()
        entries = ipc.pop_sensor_batch(100)
        ipc.write_motor_cmd(servo_us, gesture_id, confidence)
        ipc.close()
    """

    def __init__(self, path=IPC_SHM_PATH):
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"Shared memory not found: {path}\n"
                f"Is cpcu_kernel running? (creates /dev/shm/cpcu_ipc)"
            )
        fd                  =   os.open(path, os.O_RDWR)
        self.mm             =   mmap.mmap(fd, SHM_TOTAL, mmap.MAP_SHARED)
        os.close(fd)
        self._verify_magic()

    def _verify_magic(self):
        magic               =   self._r32(OFF_CTRL + CTRL_MAGIC)
        version             =   self._r16(OFF_CTRL + CTRL_VERSION)
        if magic != IPC_MAGIC:
            raise RuntimeError(
                f"IPC magic mismatch: got 0x{magic:08X}, expected 0x{IPC_MAGIC:08X}"
            )
        if version != IPC_VERSION:
            print(f"[IPC] WARNING: version mismatch: got 0x{version:04X}, expected 0x{IPC_VERSION:04X}")

    # ── Primitive read/write helpers ──

    def _r8(self, off):         return self.mm[off]
    def _w8(self, off, val):    self.mm[off] = val & 0xFF
    def _r16(self, off):        return struct.unpack_from('<H', self.mm, off)[0]
    def _w16(self, off, val):   struct.pack_into('<H', self.mm, off, val)
    def _r32(self, off):        return struct.unpack_from('<I', self.mm, off)[0]
    def _w32(self, off, val):   struct.pack_into('<I', self.mm, off, val)
    def _r64(self, off):        return struct.unpack_from('<Q', self.mm, off)[0]
    def _w64(self, off, val):   struct.pack_into('<Q', self.mm, off, val)
    def _rf32(self, off):       return struct.unpack_from('<f', self.mm, off)[0]
    def _wf32(self, off, val):  struct.pack_into('<f', self.mm, off, val)

    # ══════════════════════════════════════════════════════════════════
    #  CONTROL BLOCK
    # ══════════════════════════════════════════════════════════════════

    def read_system_state(self):    return self._r8(OFF_CTRL + CTRL_STATE)
    def read_io_ready(self):        return self._r8(OFF_CTRL + CTRL_IO_READY)
    def read_dsp_ready(self):       return self._r8(OFF_CTRL + CTRL_DSP_READY)
    def set_dsp_ready(self):        self._w8(OFF_CTRL + CTRL_DSP_READY, 1)
    def read_heartbeat(self):       return self._r64(OFF_CTRL + CTRL_HEARTBEAT)

    # Edit-mode handshake helpers.
    def read_edit_request(self):    return self._r8(OFF_CTRL + CTRL_EDIT_REQUEST)
    def read_edit_active(self):     return self._r8(OFF_CTRL + CTRL_EDIT_ACTIVE)
    def write_edit_request(self, v):
        # TUI is the sole writer of request + request_us.
        self._w8(OFF_CTRL + CTRL_EDIT_REQUEST, v)
        # Stamp request time so the TUI can implement its 500 ms timeout.
        if v:
            import time as _t
            self._w64(OFF_CTRL + CTRL_EDIT_REQUEST_US,
                      int(_t.monotonic_ns() // 1000))
    def read_edit_dsp_ack(self):    return self._r8(OFF_CTRL + CTRL_EDIT_DSP_ACK)
    def write_edit_dsp_ack(self, v):
        # cpcu_dsp.py acknowledges that it has seen the request and stopped
        # publishing motor commands. The TUI doesn't gate edit-active on
        # this — cpcu_io's SMOOTH_AllSettled is the authoritative gate —
        # but the ack is exposed in the diagnostic banner.
        self._w8(OFF_CTRL + CTRL_EDIT_DSP_ACK, v)

    def _read_head(self):           return self._r32(OFF_CTRL + CTRL_HEAD)
    def _read_tail(self):           return self._r32(OFF_CTRL + CTRL_TAIL)
    def _write_tail(self, val):     self._w32(OFF_CTRL + CTRL_TAIL, val)

    # ══════════════════════════════════════════════════════════════════
    #  RING BUFFER CONSUMER
    # ══════════════════════════════════════════════════════════════════

    def pop_sensor_batch(self, max_count=100):
        """
        Pop up to max_count entries from the SPSC ring buffer.
        
        Returns a dict with numpy arrays for efficient batch processing:
            {
                'count':     int,
                'samples':   np.ndarray (n, 2, 8) uint16,
                'seq':       np.ndarray (n,) uint8,
                'flags':     np.ndarray (n,) uint8,
                'tx_retry':  np.ndarray (n,) uint8,
                'pkt_loss':  np.ndarray (n,) uint8,
                'timestamp': np.ndarray (n,) uint16,
                'vbat_raw':  np.ndarray (n,) uint16,
            }
        
        WARNING: Only ONE process may call this (sole SPSC consumer).
        """
        tail                =   self._read_tail()
        head                =   self._read_head()
        avail               =   (head - tail) & 0xFFFFFFFF      # unsigned wrap

        if avail == 0:
            return {'count': 0}

        # Overflow detection: producer lapped us
        if avail > RING_SIZE:
            lost            =   avail - RING_SIZE
            tail           +=   lost
            avail           =   RING_SIZE

        n                   =   min(avail, max_count)

        # Pre-allocate numpy arrays for the whole batch
        samples             =   np.zeros((n, SAMPLES_PER_PKT, NUM_CHANNELS), dtype=np.uint16)
        seq_arr             =   np.zeros(n, dtype=np.uint8)
        flags_arr           =   np.zeros(n, dtype=np.uint8)
        retry_arr           =   np.zeros(n, dtype=np.uint8)
        loss_arr            =   np.zeros(n, dtype=np.uint8)
        ts_arr              =   np.zeros(n, dtype=np.uint16)
        vbat_arr            =   np.zeros(n, dtype=np.uint16)

        for i in range(n):
            idx             =   (tail + i) & RING_MASK
            base            =   OFF_RING + idx * SZ_ENTRY

            # Read 2 sample sets (each: 8 x uint16 = 16 bytes)
            for s in range(SAMPLES_PER_PKT):
                off         =   base + ENTRY_SAMPLES + s * NUM_CHANNELS * 2
                samples[i, s]   =   np.frombuffer(
                    self.mm, dtype='<u2', count=NUM_CHANNELS, offset=off
                ).copy()

            seq_arr[i]      =   self.mm[base + ENTRY_SEQ]
            flags_arr[i]    =   self.mm[base + ENTRY_FLAGS]
            retry_arr[i]    =   self.mm[base + ENTRY_RETRY]
            loss_arr[i]     =   self.mm[base + ENTRY_LOSS]
            ts_arr[i]       =   self._r16(base + ENTRY_TIMESTAMP)
            vbat_arr[i]     =   self._r16(base + ENTRY_VBAT)

        # Advance tail (we are the sole consumer)
        self._write_tail(tail + n)

        return {
            'count':        n,
            'samples':      samples,
            'seq':          seq_arr,
            'flags':        flags_arr,
            'tx_retry':     retry_arr,
            'pkt_loss':     loss_arr,
            'timestamp':    ts_arr,
            'vbat_raw':     vbat_arr,
        }

    def sensor_count(self):
        head    =   self._read_head()
        tail    =   self._read_tail()
        diff    =   (head - tail) & 0xFFFFFFFF
        return min(diff, RING_SIZE)

    # ══════════════════════════════════════════════════════════════════
    #  MOTOR COMMAND (SeqLock Writer)
    # ══════════════════════════════════════════════════════════════════

    def write_motor_cmd(self, servo_us, gesture_id, confidence):
        """
        Write motor command using SeqLock protocol.
        
        servo_us:       list/array of NUM_SERVOS uint16 pulse widths
        gesture_id:     uint8 classified gesture (0-9)
        confidence:     uint8 confidence percentage (0-100)
        """
        base                =   OFF_MOTOR

        # Step 1: seq -> odd (write in progress)
        seq                 =   self._r32(base + MOTOR_SEQ)
        self._w32(base + MOTOR_SEQ, seq + 1)

        # Step 2: write data
        for i in range(NUM_SERVOS):
            self._w16(base + MOTOR_SERVO + i * 2, int(servo_us[i]))
        self._w8(base + MOTOR_GESTURE, int(gesture_id))
        self._w8(base + MOTOR_CONF, int(confidence))
        ts                  =   int(time.monotonic() * 1_000_000) & 0xFFFFFFFFFFFFFFFF
        self._w64(base + MOTOR_TIMESTAMP, ts)

        # Step 3: seq -> even (write complete)
        self._w32(base + MOTOR_SEQ, seq + 2)

    # ══════════════════════════════════════════════════════════════════
    #  DIAGNOSTICS (atomic-ish increments)
    # ══════════════════════════════════════════════════════════════════

    def inc_dsp_inferences(self):
        off                 =   OFF_DIAG + DIAG_INFERENCES
        self._w32(off, self._r32(off) + 1)

    def update_dsp_max_latency(self, lat_us):
        off                 =   OFF_DIAG + DIAG_MAXLAT
        if lat_us > self._r32(off):
            self._w32(off, int(lat_us))

    def inc_dsp_batches(self, n):
        off                 =   OFF_DIAG + DIAG_BATCHES
        self._w32(off, self._r32(off) + n)

    def read_diagnostics(self):
        """Read all diagnostic counters as a dict."""
        b                   =   OFF_DIAG
        return {
            'io_pkts_received':     self._r32(b + DIAG_PKTS),
            'io_pkts_dropped':      self._r32(b + DIAG_DROPPED),
            'io_ring_overflows':    self._r32(b + DIAG_OVERFLOWS),
            'io_seq_gaps':          self._r32(b + DIAG_GAPS),
            'io_nrf_init_status':   self._r32(b + DIAG_NRF_STATUS),
            'dsp_batches':          self._r32(b + DIAG_BATCHES),
            'dsp_max_latency_us':   self._r32(b + DIAG_MAXLAT),
            'dsp_inferences':       self._r32(b + DIAG_INFERENCES),
        }

    # ══════════════════════════════════════════════════════════════════
    #  DSP EXPORT (Python -> TUI)
    # ══════════════════════════════════════════════════════════════════

    def write_dsp_export(self, channel_rms, gesture_name, class_confidence,
                         active_class, inference_time_us):
        """
        Write DSP telemetry for the TUI to display.
        
        channel_rms:        list/array of 8 floats (per-channel RMS)
        gesture_name:       string, max 15 chars (null-terminated)
        class_confidence:   list/array of up to 10 floats (per-class probability)
        active_class:       int (0-9)
        inference_time_us:  int (microseconds)
        """
        b                   =   OFF_EXPORT

        # Channel RMS: 8 x float32
        for i in range(min(len(channel_rms), NUM_CHANNELS)):
            self._wf32(b + EXPORT_RMS + i * 4, float(channel_rms[i]))

        # Gesture name: null-terminated string
        name_bytes          =   gesture_name[:MAX_GESTURE_NAME - 1].encode('ascii', errors='replace')
        name_bytes         +=   b'\x00' * (MAX_GESTURE_NAME - len(name_bytes))
        self.mm[b + EXPORT_NAME : b + EXPORT_NAME + MAX_GESTURE_NAME] = name_bytes

        # Class confidence: up to 10 x float32
        nc                  =   min(len(class_confidence), MAX_CLASSES)
        for i in range(nc):
            self._wf32(b + EXPORT_CONFIDENCE + i * 4, float(class_confidence[i]))

        self._w8(b + EXPORT_NUM_CLASSES, nc)
        self._w8(b + EXPORT_ACTIVE_CLASS, int(active_class) & 0xFF)
        self._w32(b + EXPORT_INF_TIME, int(inference_time_us))

        # Bump update sequence
        seq                 =   self._r32(b + EXPORT_UPDATE_SEQ)
        self._w32(b + EXPORT_UPDATE_SEQ, seq + 1)

    # ══════════════════════════════════════════════════════════════════
    #  DSP FILTERED BUFFER  ── post-bandpass+notch+envelope
    # ══════════════════════════════════════════════════════════════════

    def write_dsp_filtered_window(self, ch_idx, samples_lo, sample_rate_hz=200):
        """
        Append a freshly-computed window's envelope into channel ch_idx's
        rolling buffer in IPC_DspFiltered. The buffer holds 200 samples
        (1 s of envelope at 200 Hz). New samples shift older ones left.

        ch_idx:           0..7  (channel index)
        samples_lo:       numpy array, post-decimation/filter envelope.
                          Typically 40 samples (one window) but any length
                          <= 200 works.
        sample_rate_hz:   200 (TARGET_FS_HZ); sets the meta field on first call.

        seqlock-style write: bump seq to odd (writer in progress), write
        payload, bump again to even. Bridge readers tolerate one tear by
        re-reading. Each channel is written independently — channels can
        tear relative to each other but for visualization that's fine.

        This method is called from cpcu_dsp.py's per-window loop. Cost
        is dominated by the np.copy + struct.pack_into for ~40 floats =
        a few microseconds, negligible relative to the 50 ms stride.
        """
        import numpy as _np
        import struct as _struct

        if ch_idx < 0 or ch_idx >= DSPFILT_NUM_CHANNELS:
            return

        n_new                       =   min(len(samples_lo), DSPFILT_NUM_SAMPLES)
        if n_new <= 0: return

        # Channel base address
        ch_base                     =   (OFF_DSP_FILTERED + DSPFILT_HEADER_BYTES
                                          + ch_idx * DSPFILT_BYTES_PER_CH)

        # Read existing 200 samples (bytes), shift left by n_new, write
        # the new tail. We avoid a full numpy round-trip — just bytewise
        # memmove via a temporary buffer. For 200 floats this is 800 B
        # which is tiny.
        old_bytes                   =   bytes(self.mm[ch_base : ch_base + DSPFILT_BYTES_PER_CH])
        # New layout: drop n_new samples from the front, append n_new from samples_lo
        kept                        =   old_bytes[n_new * 4:]
        new_tail                    =   _np.asarray(samples_lo[-n_new:], dtype='<f4').tobytes()
        merged                      =   kept + new_tail
        # Length sanity (must equal DSPFILT_BYTES_PER_CH)
        if len(merged) != DSPFILT_BYTES_PER_CH:
            # Should never happen but defensively pad/truncate
            if len(merged) < DSPFILT_BYTES_PER_CH:
                merged             +=   b'\x00' * (DSPFILT_BYTES_PER_CH - len(merged))
            else:
                merged              =   merged[:DSPFILT_BYTES_PER_CH]

        # seqlock: bump to odd, write, bump to even
        seq                         =   self._r32(OFF_DSP_FILTERED + DSPFILT_SEQ)
        self._w32(OFF_DSP_FILTERED + DSPFILT_SEQ, seq | 1)        # odd
        self.mm[ch_base : ch_base + DSPFILT_BYTES_PER_CH] = merged
        # Channel-0 writer also stamps the meta fields. Other channels
        # share them — they're identical for all channels in our setup.
        if ch_idx == 0:
            self._w32(OFF_DSP_FILTERED + DSPFILT_SAMPLE_RATE, int(sample_rate_hz))
            import time as _t
            self._w64(OFF_DSP_FILTERED + DSPFILT_UPDATE_US,
                      int(_t.monotonic_ns() // 1000))
        self._w32(OFF_DSP_FILTERED + DSPFILT_SEQ, (seq | 1) + 1)  # even

    # ══════════════════════════════════════════════════════════════════
    #  CLEANUP
    # ══════════════════════════════════════════════════════════════════

    def close(self):
        if self.mm:
            self.mm.close()
            self.mm         =   None

    # ══════════════════════════════════════════════════════════════════
    #  Convenience readers for system_test.py
    # ══════════════════════════════════════════════════════════════════

    def read_magic(self):           return self._r32(OFF_CTRL + CTRL_MAGIC)
    def read_version(self):         return self._r16(OFF_CTRL + CTRL_VERSION)
    def read_heartbeat_us(self):    return self._r64(OFF_CTRL + CTRL_HEARTBEAT)
    def read_sensor_head(self):     return self._r32(OFF_CTRL + 64)   # cache line 1
    def read_sensor_tail(self):     return self._r32(OFF_CTRL + 128)  # cache line 2

    def read_diag_pkts_received(self):  return self._r32(OFF_DIAG + DIAG_PKTS)
    def read_diag_seq_gaps(self):       return self._r32(OFF_DIAG + DIAG_GAPS)
    def read_diag_safe_entries(self):   return self._r32(OFF_DIAG + DIAG_SAFE)
    def read_diag_ring_overflows(self): return self._r32(OFF_DIAG + DIAG_OVERFLOWS)
    def read_diag_dsp_inferences(self): return self._r32(OFF_DIAG + DIAG_INFERENCES)
    def read_diag_dsp_max_latency_us(self): return self._r32(OFF_DIAG + DIAG_MAXLAT)

    def read_motor_cmd_servos(self):
        """Read the 6 servo pulse widths from the motor command region."""
        base = OFF_MOTOR + 4  # skip seq field
        servos = []
        for i in range(6):
            servos.append(self._r16(base + i * 2))
        return servos

    def read_sensor_entry(self, idx):
        """Read one sensor ring entry by index. Returns dict with vbat_raw."""
        off = OFF_RING + (idx & RING_MASK) * SZ_ENTRY
        # vbat_raw is at offset 38 within the entry (after samples+seq+flags+retry+loss+timestamp)
        # samples: 2 × 8ch × 2B = 32B, seq(1), flags(1), retry(1), loss(1), timestamp(2)
        vbat_raw = self._r16(off + 38)
        return {'vbat_raw': vbat_raw}

        return struct.unpack_from('<H', self.mm, off)[0]


# ══════════════════════════════════════════════════════════════════════
#  v3.0 ADDITIONS — latency trace, gesture name read, smoother overrides
# ══════════════════════════════════════════════════════════════════════

# IPC_LatencyTrace offsets (appended after IPC_DspFiltered)
SZ_LATENCY              = 64
LAT_RING_DWELL          = 0
LAT_DSP_COMPUTE         = 4
LAT_VELOCITY            = 8
LAT_WINDOW_WAIT         = 12
LAT_HYSTERESIS_MS       = 16
LAT_SMOOTHER            = 20
LAT_TOTAL_PROC          = 24
LAT_SEQ                 = 28

# Motor command smoother override offsets (in _reserved region)
MOTOR_SMOOTH_VEL_OVERRIDE = 64
MOTOR_SMOOTH_ACC_OVERRIDE = 76
MOTOR_SNAP_FLAGS          = 88

def _ipc_read_dsp_gesture_name(mm, off_export, export_name_off=32):
    """Read gesture name string from DSP export region."""
    base = off_export + export_name_off
    raw = mm[base:base + 16]
    end = raw.find(b"\x00")
    if end >= 0:
        raw = raw[:end]
    return raw.decode("ascii", errors="replace").strip()
