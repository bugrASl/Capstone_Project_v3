"""
cpcu_ipc_bridge.py — v3.0 ADDITIONS

Add these constants and methods to the existing IPCBridge class.
"""

# ══════════════════════════════════════════════════════════════════════
#  NEW CONSTANTS — add to the constants section
# ══════════════════════════════════════════════════════════════════════

# IPC_LatencyTrace sits after IPC_DspFiltered
SZ_LATENCY              =   64
OFF_LATENCY             =   OFF_DSPFILT + SZ_DSPFILT  # add to offset chain

# Field offsets within IPC_LatencyTrace (64 bytes)
LAT_RING_DWELL          =   0       # uint32
LAT_DSP_COMPUTE         =   4       # uint32
LAT_VELOCITY            =   8       # uint32
LAT_WINDOW_WAIT         =   12      # uint32
LAT_HYSTERESIS_MS       =   16      # uint32
LAT_SMOOTHER            =   20      # uint32
LAT_TOTAL_PROC          =   24      # uint32
LAT_SEQ                 =   28      # uint32

# IPC_MotorCommand smoother override offsets (in _reserved region, byte 64+)
MOTOR_SMOOTH_VEL_OVERRIDE   =   64      # 6 × uint16 = 12 B
MOTOR_SMOOTH_ACC_OVERRIDE   =   76      # 6 × uint16 = 12 B
MOTOR_SNAP_FLAGS            =   88      # uint8 bitmask


# ══════════════════════════════════════════════════════════════════════
#  NEW METHODS — add to the IPCBridge class
# ══════════════════════════════════════════════════════════════════════

def read_dsp_gesture_name(self):
    """Read current gesture name from IPC_DSPExport. Returns string."""
    base = OFF_EXPORT + EXPORT_NAME  # 32 bytes into DSPExport
    raw = self._mm[base:base + 16]
    # null-terminated ASCII
    end = raw.find(b'\x00')
    if end >= 0:
        raw = raw[:end]
    try:
        return raw.decode('ascii', errors='replace').strip()
    except Exception:
        return ""

def read_latency_trace(self):
    """Read all latency fields. Returns dict of uint32 values in µs."""
    base = OFF_LATENCY
    return {
        'ring_dwell_us':    self._r32(base + LAT_RING_DWELL),
        'dsp_compute_us':   self._r32(base + LAT_DSP_COMPUTE),
        'velocity_us':      self._r32(base + LAT_VELOCITY),
        'window_wait_us':   self._r32(base + LAT_WINDOW_WAIT),
        'hysteresis_ms':    self._r32(base + LAT_HYSTERESIS_MS),
        'smoother_us':      self._r32(base + LAT_SMOOTHER),
        'total_proc_us':    self._r32(base + LAT_TOTAL_PROC),
        'seq':              self._r32(base + LAT_SEQ),
    }

def write_latency_dsp(self, ring_dwell_us, dsp_compute_us,
                      velocity_us, window_wait_us, hysteresis_ms):
    """Write DSP-side latency fields to shared memory."""
    base = OFF_LATENCY
    self._w32(base + LAT_RING_DWELL,    ring_dwell_us)
    self._w32(base + LAT_DSP_COMPUTE,   dsp_compute_us)
    self._w32(base + LAT_VELOCITY,      velocity_us)
    self._w32(base + LAT_WINDOW_WAIT,   window_wait_us)
    self._w32(base + LAT_HYSTERESIS_MS, hysteresis_ms)
    total = ring_dwell_us + dsp_compute_us + velocity_us
    self._w32(base + LAT_TOTAL_PROC, total)
    # bump seq
    seq = self._r32(base + LAT_SEQ)
    self._w32(base + LAT_SEQ, seq + 1)

def write_motor_cmd_v3(self, servo_us, gesture_id, confidence,
                       smooth_vel=None, smooth_acc=None, snap_flags=0):
    """Extended motor command with smoother overrides.
    smooth_vel/acc: list of 6 uint16, 0 = use global default.
    snap_flags: bitmask, bit N = servo N bypasses smoother."""
    # write base motor command (existing method)
    self.write_motor_cmd(servo_us, gesture_id, confidence)

    # write overrides into the _reserved region
    base = OFF_MOTOR
    if smooth_vel:
        for i in range(min(6, len(smooth_vel))):
            self._w16(base + MOTOR_SMOOTH_VEL_OVERRIDE + i*2,
                      smooth_vel[i])
    if smooth_acc:
        for i in range(min(6, len(smooth_acc))):
            self._w16(base + MOTOR_SMOOTH_ACC_OVERRIDE + i*2,
                      smooth_acc[i])
    self._w8(base + MOTOR_SNAP_FLAGS, snap_flags & 0xFF)


# ══════════════════════════════════════════════════════════════════════
#  UPDATE IPC_SHM_SIZE
# ══════════════════════════════════════════════════════════════════════
#
# In the constants section, update:
#   SZ_SHM = OFF_LATENCY + SZ_LATENCY
#
# And update the mmap size in __init__:
#   self._mm = mmap.mmap(fd, SZ_SHM, ...)
