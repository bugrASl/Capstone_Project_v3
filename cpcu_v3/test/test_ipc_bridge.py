#!/usr/bin/env python3
"""
test_ipc_bridge.py — Validate Python IPC bridge against C struct layout.

Run this ALONGSIDE a running cpcu_kernel (which creates the shared memory).
It reads known fields written by C code and verifies the Python offsets match.

Also tests: SeqLock write/read, ring buffer indexing, DSP export.

Usage:
    # Terminal 1: start kernel (creates shm)
    ./cpcu_kernel

    # Terminal 2: run this test
    python3 test_ipc_bridge.py

Author: bugrASl
Date:   April 2026
"""

import struct
import sys
import os
import time
import mmap

# Add both this directory (test/) and the python module dir to sys.path
# so `import cpcu_ipc_bridge` works regardless of cwd. The restructure moved Python
# modules from scripts/ to python/; we add both so this test works on
# either layout.
_TEST_DIR    =   os.path.dirname(os.path.abspath(__file__))
_PYTHON_DIR  =   os.path.normpath(os.path.join(_TEST_DIR, "..", "python"))
_SCRIPTS_DIR =   os.path.normpath(os.path.join(_TEST_DIR, "..", "scripts"))
for _p in (_TEST_DIR, _PYTHON_DIR, _SCRIPTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from cpcu_ipc_bridge import (
    IPCBridge, IPC_MAGIC, IPC_VERSION, SHM_TOTAL,
    OFF_CTRL, OFF_RING, OFF_MOTOR, OFF_DIAG, OFF_EXPORT,
    OFF_CONFIG, OFF_TOOL_PRESENCE, OFF_DSP_FILTERED,
    SZ_CTRL, SZ_ENTRY, SZ_MOTOR, SZ_DIAG, SZ_EXPORT, SZ_RING,
    SZ_CONFIG, SZ_TOOL_PRESENCE, SZ_DSP_FILTERED,
    CTRL_MAGIC, CTRL_VERSION, CTRL_HEAD, CTRL_TAIL,
    CTRL_DSP_READY, CTRL_STATE,
    MOTOR_SEQ, MOTOR_SERVO, MOTOR_GESTURE, MOTOR_CONF,
    RING_SIZE, NUM_SERVOS, NUM_CHANNELS, SAMPLES_PER_PKT,
    IPC_STATE_INIT, IPC_STATE_RUNNING,
)

g_pass                  =   0
g_fail                  =   0

def ASSERT(cond, msg):
    global g_pass, g_fail
    if cond:
        print(f"  [PASS] {msg}")
        g_pass         +=   1
    else:
        print(f"  [FAIL] {msg}")
        g_fail         +=   1


def test_struct_sizes():
    """Verify struct sizes match C _Static_assert values."""
    print("\n--- Struct Size Validation ---")
    ASSERT(SZ_CTRL == 192,      f"ControlBlock: {SZ_CTRL} == 192")
    ASSERT(SZ_ENTRY == 64,      f"SensorEntry: {SZ_ENTRY} == 64")
    ASSERT(SZ_MOTOR == 128,     f"MotorCommand: {SZ_MOTOR} == 128")
    ASSERT(SZ_DIAG == 128,      f"Diagnostics: {SZ_DIAG} == 128")
    ASSERT(SZ_EXPORT == 256,    f"DSPExport: {SZ_EXPORT} == 256")


def test_section_offsets():
    """Verify section offsets are sequential and non-overlapping.
       Now includes CONFIG, TOOL_PRESENCE
, and DSP_FILTERED sections."""
    print("\n--- Section Offset Validation ---")
    ASSERT(OFF_CTRL          == 0,                                      f"CTRL at 0")
    ASSERT(OFF_RING          == 192,                                    f"RING at 192")
    ASSERT(OFF_MOTOR         == 192 + 64 * 1024,                        f"MOTOR at {OFF_MOTOR}")
    ASSERT(OFF_DIAG          == OFF_MOTOR + 128,                        f"DIAG at {OFF_DIAG}")
    ASSERT(OFF_EXPORT        == OFF_DIAG + 128,                         f"EXPORT at {OFF_EXPORT}")
    ASSERT(OFF_CONFIG        == OFF_EXPORT + SZ_EXPORT,                 f"CONFIG at {OFF_CONFIG}")
    ASSERT(OFF_TOOL_PRESENCE == OFF_CONFIG + SZ_CONFIG,                 f"TOOL_PRESENCE at {OFF_TOOL_PRESENCE}")
    ASSERT(OFF_DSP_FILTERED  == OFF_TOOL_PRESENCE + SZ_TOOL_PRESENCE,   f"DSP_FILTERED at {OFF_DSP_FILTERED}")
    ASSERT(SHM_TOTAL         == OFF_DSP_FILTERED + SZ_DSP_FILTERED,     f"Total SHM = {SHM_TOTAL}")


def test_magic_and_version():
    """Read magic and version from live shared memory."""
    print("\n--- Magic/Version Read Test ---")
    try:
        ipc             =   IPCBridge()
        magic           =   ipc._r32(OFF_CTRL + CTRL_MAGIC)
        version         =   ipc._r16(OFF_CTRL + CTRL_VERSION)
        ASSERT(magic == IPC_MAGIC,      f"magic=0x{magic:08X} == 0x{IPC_MAGIC:08X}")
        ASSERT(version == IPC_VERSION,  f"version=0x{version:04X} == 0x{IPC_VERSION:04X}")
        ipc.close()
    except FileNotFoundError:
        print("  [SKIP] Shared memory not found (cpcu_kernel not running)")
        return


def test_dsp_ready_flag():
    """Write dsp_ready and read it back."""
    print("\n--- DSP Ready Flag ---")
    try:
        ipc             =   IPCBridge()
        ipc.set_dsp_ready()
        val             =   ipc._r8(OFF_CTRL + CTRL_DSP_READY)
        ASSERT(val == 1, f"dsp_ready = {val} (expected 1)")
        # Reset for clean state
        ipc._w8(OFF_CTRL + CTRL_DSP_READY, 0)
        ipc.close()
    except FileNotFoundError:
        print("  [SKIP] Shared memory not found")


def test_motor_cmd_seqlock():
    """Write motor command and verify fields."""
    print("\n--- Motor Command SeqLock ---")
    try:
        ipc             =   IPCBridge()
        
        servo_us        =   [1500, 1200, 1800, 1600, 1400, 2000]
        ipc.write_motor_cmd(servo_us, gesture_id=3, confidence=95)

        # Read back raw values
        seq             =   ipc._r32(OFF_MOTOR + MOTOR_SEQ)
        ASSERT(seq % 2 == 0, f"SeqLock seq={seq} is even (write complete)")
        
        for i in range(NUM_SERVOS):
            val         =   ipc._r16(OFF_MOTOR + MOTOR_SERVO + i * 2)
            ASSERT(val == servo_us[i], f"servo[{i}]={val} == {servo_us[i]}")
        
        gesture         =   ipc._r8(OFF_MOTOR + MOTOR_GESTURE)
        conf            =   ipc._r8(OFF_MOTOR + MOTOR_CONF)
        ASSERT(gesture == 3, f"gesture_id={gesture} == 3")
        ASSERT(conf == 95,   f"confidence={conf} == 95")
        
        ipc.close()
    except FileNotFoundError:
        print("  [SKIP] Shared memory not found")


def test_ring_head_tail():
    """Verify ring buffer head/tail field alignment."""
    print("\n--- Ring Buffer Head/Tail Alignment ---")
    
    # Head should be at byte 64 within ControlBlock (cache line 1)
    ASSERT(CTRL_HEAD == 64,     f"CTRL_HEAD={CTRL_HEAD} == 64 (cache line 1)")
    ASSERT(CTRL_TAIL == 128,    f"CTRL_TAIL={CTRL_TAIL} == 128 (cache line 2)")
    
    # Verify no false sharing: head and tail are on different cache lines
    head_cacheline      =   CTRL_HEAD // 64
    tail_cacheline      =   CTRL_TAIL // 64
    ASSERT(head_cacheline != tail_cacheline,
           f"head(CL{head_cacheline}) != tail(CL{tail_cacheline}) — no false sharing")


def test_dsp_export():
    """Write DSP export and read back."""
    print("\n--- DSP Export ---")
    try:
        ipc             =   IPCBridge()
        
        rms             =   [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
        proba           =   [0.01, 0.02, 0.85, 0.03, 0.01, 0.02, 0.01, 0.02, 0.02, 0.01]
        
        ipc.write_dsp_export(
            channel_rms         =   rms,
            gesture_name        =   "HAND HARD",
            class_confidence    =   proba,
            active_class        =   2,
            inference_time_us   =   12345,
        )
        
        # Read back
        from cpcu_ipc_bridge import (OFF_EXPORT, EXPORT_RMS, EXPORT_NAME,
                                      EXPORT_ACTIVE_CLASS, EXPORT_INF_TIME, EXPORT_UPDATE_SEQ)
        
        rms_back        =   ipc._rf32(OFF_EXPORT + EXPORT_RMS)
        ASSERT(abs(rms_back - 0.1) < 0.001, f"channel_rms[0]={rms_back:.3f} == 0.1")
        
        active          =   ipc._r8(OFF_EXPORT + EXPORT_ACTIVE_CLASS)
        ASSERT(active == 2, f"active_class={active} == 2")
        
        inf_time        =   ipc._r32(OFF_EXPORT + EXPORT_INF_TIME)
        ASSERT(inf_time == 12345, f"inference_time={inf_time} == 12345")
        
        update_seq      =   ipc._r32(OFF_EXPORT + EXPORT_UPDATE_SEQ)
        ASSERT(update_seq > 0, f"update_seq={update_seq} > 0")
        
        ipc.close()
    except FileNotFoundError:
        print("  [SKIP] Shared memory not found")


def test_pop_empty_ring():
    """Pop from empty ring should return count=0."""
    print("\n--- Pop Empty Ring ---")
    try:
        ipc             =   IPCBridge()
        batch           =   ipc.pop_sensor_batch(10)
        ASSERT(batch['count'] == 0, f"empty ring: count={batch['count']} == 0")
        ipc.close()
    except FileNotFoundError:
        print("  [SKIP] Shared memory not found")


# ══════════════════════════════════════════════════════════════════════

def main():
    print("=== IPC Bridge Test Suite ===")
    
    test_struct_sizes()
    test_section_offsets()
    test_ring_head_tail()
    test_magic_and_version()
    test_dsp_ready_flag()
    test_motor_cmd_seqlock()
    test_dsp_export()
    test_pop_empty_ring()

    print(f"\n{'=' * 40}")
    print(f"  RESULTS: {g_pass} PASS, {g_fail} FAIL")
    print(f"{'=' * 40}")
    
    return 1 if g_fail > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
