#!/usr/bin/env python3
"""
test_dsp_pipeline.py — DSP pipeline validation against cpcu_dsp.py
                       Current version.

@author bugrASl

Verifies that the Python DSP module exports the correct public API and
that its filter / feature primitives produce sensible outputs for known
analytical inputs. Runs in isolation: no shared memory, no model file,
no Pi hardware.

What this test covers:

    1. Public API surface check — every name the rest of the system
       imports from cpcu_dsp must exist with the right signature.
    2. butter_bandpass — passes a 100 Hz sine through a 20-450 Hz
       bandpass and checks the amplitude is preserved.
    3. notch_filter   — feeds a 50 Hz sine and checks attenuation.
    4. envelope       — feeds a 100 Hz burst and checks the envelope
       tracks its amplitude.
    5. extract_features — confirms output is a list of 4 floats matching
       the team's [rms, var, wl, env_mean] order, and that values change
       monotonically with input amplitude.
    6. process_window — feeds a 400-sample @ 2 kHz buffer and checks
       the full pipeline returns (cleaned, env, features) with the
       right shapes.
    7. GESTURE_SERVO_MAP — every entry is a 6-tuple of valid servo µs.
    8. Constants — INPUT_FS_HZ=2000, TARGET_FS_HZ=200, DECIMATE_FACTOR=10,
       WINDOW_SAMPLES_HI=400, NUM_SERVOS=6.

History:
    Replaces the legacy test which imported from a
    long-removed `get_features` API and expected a 7-feature vector
    [mav, rms, wl, zc, ssc, var, log_det]. The team's training pipeline
    settled on 4 features [rms, var, wl, env_mean] (see feature_ex.py
    in their training repo); this test was not updated for that change
    until now. The legacy test is preserved alongside this file as
    test_dsp_pipeline_legacy for for reference.
"""

import os
import sys
import numpy as np

# Add the directory holding cpcu_dsp.py to import path. The restructure moved Python
# modules from scripts/ to python/; we add both so this test works on
# either layout.
HERE                =   os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'python'))    # current layout
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))   # legacy fallback

g_pass              =   0
g_fail              =   0


def ASSERT(condition, msg):
    """Print a PASS/FAIL line and tally the global counters."""
    global g_pass, g_fail
    if condition:
        print(f"  [PASS] {msg}")
        g_pass         +=   1
    else:
        print(f"  [FAIL] {msg}")
        g_fail         +=   1


def TEST_OK(msg):
    print(f"  [OK]   {msg}")


# ══════════════════════════════════════════════════════════════════════
#  Test 1 — Public API surface
# ══════════════════════════════════════════════════════════════════════

def test_api_surface():
    """Every name the rest of the codebase imports from cpcu_dsp must
    exist. If this group fails, every later group will too."""
    print("\n--- TB-DSP01: Public API surface ---")

    import cpcu_dsp

    # Functions
    for name in ("butter_bandpass", "notch_filter", "envelope",
                 "extract_features", "process_window", "main"):
        ASSERT(hasattr(cpcu_dsp, name),
               f"cpcu_dsp.{name} is exported")

    # Constants the rest of the system relies on
    for name in ("INPUT_FS_HZ", "TARGET_FS_HZ", "DECIMATE_FACTOR",
                 "WINDOW_SAMPLES_HI", "NUM_SERVOS",
                 "ACTIVE_CHANNELS", "GESTURE_SERVO_MAP"):
        ASSERT(hasattr(cpcu_dsp, name),
               f"cpcu_dsp.{name} is exported")


# ══════════════════════════════════════════════════════════════════════
#  Test 2 — butter_bandpass
# ══════════════════════════════════════════════════════════════════════

def test_bandpass():
    """100 Hz sine through 20-450 Hz BP should pass with ~unity gain."""
    print("\n--- TB-DSP02: butter_bandpass ---")

    from cpcu_dsp import butter_bandpass

    fs                  =   2000
    t                   =   np.arange(2 * fs) / fs        # 2 s @ 2 kHz
    sig                 =   0.5 * np.sin(2 * np.pi * 100 * t)

    out                 =   butter_bandpass(sig, 20.0, 450.0, fs)

    # Compare RMS — filtfilt is zero-phase so amplitude is preserved
    rms_in              =   float(np.sqrt(np.mean(sig ** 2)))
    rms_out             =   float(np.sqrt(np.mean(out ** 2)))
    ratio               =   rms_out / rms_in

    ASSERT(0.9 < ratio < 1.1,
           f"in-band RMS preserved: ratio={ratio:.3f} ∈ (0.9, 1.1)")


def test_bandpass_rejects_dc():
    """A pure-DC input should be killed by the high-pass cutoff."""
    print("\n--- TB-DSP03: bandpass kills DC ---")

    from cpcu_dsp import butter_bandpass

    fs                  =   2000
    sig                 =   np.full(2 * fs, 1.65)

    out                 =   butter_bandpass(sig, 20.0, 450.0, fs)

    rms_out             =   float(np.sqrt(np.mean(out ** 2)))
    ASSERT(rms_out < 0.05,
           f"DC rejected: out RMS={rms_out:.4f} < 0.05")


# ══════════════════════════════════════════════════════════════════════
#  Test 3 — notch_filter
# ══════════════════════════════════════════════════════════════════════

def test_notch():
    """50 Hz sine through a 50 Hz notch should be heavily attenuated."""
    print("\n--- TB-DSP04: notch_filter ---")

    from cpcu_dsp import notch_filter

    fs                  =   2000
    t                   =   np.arange(2 * fs) / fs
    sig_50              =   0.5 * np.sin(2 * np.pi * 50 * t)
    sig_200             =   0.5 * np.sin(2 * np.pi * 200 * t)

    out_50              =   notch_filter(sig_50, 50.0, fs)
    out_200             =   notch_filter(sig_200, 50.0, fs)

    rms_50_in           =   float(np.sqrt(np.mean(sig_50 ** 2)))
    rms_50_out          =   float(np.sqrt(np.mean(out_50 ** 2)))
    rms_200_in          =   float(np.sqrt(np.mean(sig_200 ** 2)))
    rms_200_out         =   float(np.sqrt(np.mean(out_200 ** 2)))

    atten_50            =   rms_50_out / rms_50_in
    atten_200           =   rms_200_out / rms_200_in

    ASSERT(atten_50 < 0.15,
           f"50 Hz attenuated: ratio={atten_50:.3f} < 0.15 "
           f"(Q=30 notch gives ~-18 dB)")
    ASSERT(atten_200 > 0.9,
           f"200 Hz preserved: ratio={atten_200:.3f} > 0.9")


# ══════════════════════════════════════════════════════════════════════
#  Test 4 — envelope
# ══════════════════════════════════════════════════════════════════════

def test_envelope():
    """The envelope of a 100 Hz, 0.5-amplitude sine should approach
    the rectified-mean (= 2·A/π ≈ 0.318) once the LPF settles."""
    print("\n--- TB-DSP05: envelope ---")

    from cpcu_dsp import envelope

    fs                  =   200                           # team's downsampled rate
    t                   =   np.arange(2 * fs) / fs
    sig                 =   0.5 * np.sin(2 * np.pi * 30 * t)

    env                 =   envelope(sig, fs, cutoff=3.0)

    # Settled portion (skip the first 200 ms transient)
    env_settled         =   env[fs // 5:]
    mean_env            =   float(np.mean(env_settled))

    # Theoretical mean of |sin| = 2A/π ≈ 0.318
    ASSERT(0.25 < mean_env < 0.40,
           f"envelope mean ≈ rectified-mean: {mean_env:.4f} ∈ (0.25, 0.40)")


# ══════════════════════════════════════════════════════════════════════
#  Test 5 — extract_features
# ══════════════════════════════════════════════════════════════════════

def test_extract_features():
    """Features must be 4 floats in order [rms, var, wl, env_mean]."""
    print("\n--- TB-DSP06: extract_features ---")

    from cpcu_dsp import extract_features

    # Clean sine, plus its rectified-mean envelope as the "env" arg.
    fs                  =   200
    t                   =   np.arange(40) / fs            # one window
    clean               =   0.5 * np.sin(2 * np.pi * 30 * t)
    env                 =   np.abs(clean)                 # cheap stand-in

    feat                =   extract_features(clean, env)

    ASSERT(isinstance(feat, list),
           f"return type is list (got {type(feat).__name__})")
    ASSERT(len(feat) == 4,
           f"feature count = {len(feat)} (expected 4)")
    rms, var, wl, em    =   feat

    ASSERT(0.30 < rms < 0.40,
           f"rms = {rms:.4f} ≈ 0.354 (sine amplitude 0.5)")
    ASSERT(var > 0,
           f"var = {var:.6f} > 0")
    ASSERT(wl > 0,
           f"wl = {wl:.6f} > 0")
    ASSERT(em > 0,
           f"env_mean = {em:.6f} > 0")


def test_features_scale_with_amplitude():
    """Doubling input amplitude should ~double RMS."""
    print("\n--- TB-DSP07: feature scaling ---")

    from cpcu_dsp import extract_features

    fs                  =   200
    t                   =   np.arange(40) / fs

    sig1                =   0.5 * np.sin(2 * np.pi * 30 * t)
    sig2                =   1.0 * np.sin(2 * np.pi * 30 * t)

    feat1               =   extract_features(sig1, np.abs(sig1))
    feat2               =   extract_features(sig2, np.abs(sig2))

    rms_ratio           =   feat2[0] / feat1[0] if feat1[0] > 0 else 0

    ASSERT(1.8 < rms_ratio < 2.2,
           f"rms doubles with amplitude: ratio={rms_ratio:.3f} ≈ 2.0")


# ══════════════════════════════════════════════════════════════════════
#  Test 6 — process_window (full pipeline pass)
# ══════════════════════════════════════════════════════════════════════

def test_process_window():
    """One 400-sample @ 2 kHz window through the full pipeline must
    return (cleaned, env, features) with the right shapes."""
    print("\n--- TB-DSP08: process_window ---")

    from cpcu_dsp import process_window, WINDOW_SAMPLES_HI, WINDOW_SAMPLES_LO

    fs                  =   2000
    t                   =   np.arange(WINDOW_SAMPLES_HI) / fs
    # 100 Hz band-of-interest tone + 50 Hz mains we want notched out
    sig                 =   (0.4 * np.sin(2 * np.pi * 100 * t) +
                             0.3 * np.sin(2 * np.pi *  50 * t))

    cleaned, env, feat  =   process_window(sig)

    ASSERT(len(cleaned) == WINDOW_SAMPLES_LO,
           f"cleaned length = {len(cleaned)} == WINDOW_SAMPLES_LO ({WINDOW_SAMPLES_LO})")
    ASSERT(len(env) == WINDOW_SAMPLES_LO,
           f"env length = {len(env)} == WINDOW_SAMPLES_LO ({WINDOW_SAMPLES_LO})")
    ASSERT(len(feat) == 4,
           f"feature count = {len(feat)} (expected 4)")

    # The 50 Hz line should be largely gone after the notch.
    # Compare RMS of the cleaned signal to the input — expect attenuation
    # because we stripped 50 Hz energy.
    rms_in              =   float(np.sqrt(np.mean(sig ** 2)))
    rms_out             =   float(np.sqrt(np.mean(cleaned ** 2)))
    ASSERT(rms_out < rms_in,
           f"cleaned RMS {rms_out:.3f} < input RMS {rms_in:.3f} (50 Hz removed)")


# ══════════════════════════════════════════════════════════════════════
#  Test 7 — GESTURE_SERVO_MAP
# ══════════════════════════════════════════════════════════════════════

def test_gesture_servo_map():
    """Every entry must be a 6-tuple of valid servo microseconds."""
    print("\n--- TB-DSP09: GESTURE_SERVO_MAP ---")

    from cpcu_dsp import GESTURE_SERVO_MAP, NUM_SERVOS

    ASSERT(len(GESTURE_SERVO_MAP) >= 1,
           f"map has {len(GESTURE_SERVO_MAP)} entries (>= 1 required)")
    ASSERT("rest" in GESTURE_SERVO_MAP,
           "'rest' is in the map (always-safe fallback)")

    for label, servos in GESTURE_SERVO_MAP.items():
        ASSERT(len(servos) == NUM_SERVOS,
               f"{label}: {len(servos)} servos == NUM_SERVOS ({NUM_SERVOS})")
        for i, us in enumerate(servos):
            ASSERT(500 <= us <= 2500,
                   f"{label} servo[{i}] = {us} µs ∈ [500, 2500]")


# ══════════════════════════════════════════════════════════════════════
#  Test 8 — Constants
# ══════════════════════════════════════════════════════════════════════

def test_constants():
    """Pin down the constants that other parts of the system depend on."""
    print("\n--- TB-DSP10: constants ---")

    from cpcu_dsp import (INPUT_FS_HZ, TARGET_FS_HZ, DECIMATE_FACTOR,
                          WINDOW_SAMPLES_HI, WINDOW_SAMPLES_LO, NUM_SERVOS)

    ASSERT(INPUT_FS_HZ == 2000,
           f"INPUT_FS_HZ = {INPUT_FS_HZ} (must match BSAU 2 kHz scan rate)")
    ASSERT(TARGET_FS_HZ == 200,
           f"TARGET_FS_HZ = {TARGET_FS_HZ} (must match training rate)")
    ASSERT(DECIMATE_FACTOR == INPUT_FS_HZ // TARGET_FS_HZ,
           f"DECIMATE_FACTOR = {DECIMATE_FACTOR} (= INPUT/TARGET)")
    ASSERT(WINDOW_SAMPLES_HI == 400,
           f"WINDOW_SAMPLES_HI = {WINDOW_SAMPLES_HI} (= 200ms @ 2kHz)")
    ASSERT(WINDOW_SAMPLES_LO == 40,
           f"WINDOW_SAMPLES_LO = {WINDOW_SAMPLES_LO} (= 200ms @ 200Hz)")
    ASSERT(NUM_SERVOS == 6,
           f"NUM_SERVOS = {NUM_SERVOS} (must match IPC_NUM_SERVOS)")


# ══════════════════════════════════════════════════════════════════════
#  TB-DSP11 — Velocity-mode runtime config loader
# ══════════════════════════════════════════════════════════════════════

def test_runtime_config_loader_defaults():
    """When runtime.json is absent everywhere, loader returns defaults
    without crashing."""
    print("\n--- TB-DSP11: runtime config loader (defaults) ---")
    import cpcu_dsp
    floor, ceil_, votes, beh, _ = cpcu_dsp.load_dsp_runtime_config(
        ["rest", "biceps_flex"], path="/tmp/this_does_not_exist_xyzzy.json")
    ASSERT(floor == cpcu_dsp.INTERP_CONF_FLOOR,
           f"floor defaulted to {floor} (expected {cpcu_dsp.INTERP_CONF_FLOOR})")
    ASSERT(ceil_ == cpcu_dsp.INTERP_CONF_CEIL,
           f"ceil defaulted to {ceil_}")
    ASSERT(votes == cpcu_dsp.CONFIRMATION_THRESHOLD,
           f"votes defaulted to {votes}")
    ASSERT("rest" in beh and beh["rest"]["mode"] == "freeze",
           "rest is freeze by default")
    ASSERT("biceps_flex" in beh and beh["biceps_flex"]["mode"] == "freeze",
           "unknown class defaults to freeze")


def test_runtime_config_loader_velocity():
    """When runtime.json specifies gesture_velocity, it's parsed correctly."""
    print("\n--- TB-DSP12: runtime config loader (velocity) ---")
    import cpcu_dsp, tempfile, os
    sample = """
    {
      "schema_version": 1,
      "servo_min_us": [498, 1074, 1074, 1001, 1001, 976],
      "servo_max_us": [2500, 1953, 1953, 2002, 2002, 1733],
      "interp_conf_floor_pct": 25,
      "interp_conf_ceil_pct":  90,
      "hysteresis_votes": 4,
      "gesture_velocity": {
          "biceps_flex": [0, 250, 0, 0, 0, 0],
          "hand_flex":   [-100, 0, 0, 100, 100, 100]
      }
    }
    """
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write(sample); path = f.name
    try:
        floor, ceil_, votes, beh, _ = cpcu_dsp.load_dsp_runtime_config(
            ["rest", "biceps_flex", "hand_flex"], path=path)
        ASSERT(abs(floor - 0.25) < 1e-9, f"floor parsed as {floor}")
        ASSERT(abs(ceil_ - 0.90) < 1e-9, f"ceil parsed as {ceil_}")
        ASSERT(votes == 4, f"votes parsed as {votes}")
        ASSERT(beh["biceps_flex"]["mode"] == "velocity", "biceps_flex velocity")
        ASSERT(beh["biceps_flex"]["rate"][1] == 250, "biceps rate[1]")
        ASSERT(beh["hand_flex"]["mode"] == "velocity", "hand_flex velocity")
        ASSERT(beh["hand_flex"]["rate"][0] == -100, "negative rate parsed")
    finally:
        os.unlink(path)


def test_runtime_config_loader_clamping():
    """Out-of-range rates get clamped, not silently accepted or rejected."""
    print("\n--- TB-DSP13: runtime config loader (clamping) ---")
    import cpcu_dsp, tempfile, os
    sample = """
    {
      "schema_version": 1,
      "servo_min_us": [498, 1074, 1074, 1001, 1001, 976],
      "servo_max_us": [2500, 1953, 1953, 2002, 2002, 1733],
      "gesture_velocity": {
          "biceps_flex": [99999, -99999, 0, 0, 0, 0]
      }
    }
    """
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write(sample); path = f.name
    try:
        _, _, _, beh, _ = cpcu_dsp.load_dsp_runtime_config(
            ["rest", "biceps_flex"], path=path)
        ASSERT(beh["biceps_flex"]["rate"][0] == 5000,
               f"99999 clamped to 5000, got {beh['biceps_flex']['rate'][0]}")
        ASSERT(beh["biceps_flex"]["rate"][1] == -5000,
               f"-99999 clamped to -5000, got {beh['biceps_flex']['rate'][1]}")
    finally:
        os.unlink(path)


def test_runtime_config_loader_unknown_class():
    """gesture_velocity entries for classes not in model.classes_ are
    ignored without raising."""
    print("\n--- TB-DSP14: runtime config loader (unknown class) ---")
    import cpcu_dsp, tempfile, os
    sample = """
    {
      "schema_version": 1,
      "servo_min_us": [498, 1074, 1074, 1001, 1001, 976],
      "servo_max_us": [2500, 1953, 1953, 2002, 2002, 1733],
      "gesture_velocity": {
          "biceps_flex":         [0, 200, 0, 0, 0, 0],
          "this_class_doesnt_exist": [0, 100, 0, 0, 0, 0]
      }
    }
    """
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write(sample); path = f.name
    try:
        _, _, _, beh, _ = cpcu_dsp.load_dsp_runtime_config(
            ["rest", "biceps_flex"], path=path)
        ASSERT("biceps_flex" in beh and beh["biceps_flex"]["mode"] == "velocity",
               "known class loaded")
        ASSERT("this_class_doesnt_exist" not in beh,
               "unknown class silently dropped")
    finally:
        os.unlink(path)


def test_runtime_config_loader_invariant_violation():
    """floor >= ceil triggers a fallback to defaults rather than a
    division-by-zero in the integrator."""
    print("\n--- TB-DSP15: runtime config (invariant violation) ---")
    import cpcu_dsp, tempfile, os
    sample = """
    {
      "schema_version": 1,
      "servo_min_us": [498, 1074, 1074, 1001, 1001, 976],
      "servo_max_us": [2500, 1953, 1953, 2002, 2002, 1733],
      "interp_conf_floor_pct": 80,
      "interp_conf_ceil_pct":  50
    }
    """
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write(sample); path = f.name
    try:
        floor, ceil_, _, _, _ = cpcu_dsp.load_dsp_runtime_config(["rest"], path=path)
        ASSERT(floor < ceil_,
               f"loader rejected invariant violation: floor={floor} ceil={ceil_}")
    finally:
        os.unlink(path)


def test_runtime_config_jsonc_comments():
    """// line comments and trailing commas (JSONC) are tolerated."""
    print("\n--- TB-DSP16: runtime config (JSONC tolerance) ---")
    import cpcu_dsp, tempfile, os
    sample = """
    {
      // schema and limits
      "schema_version": 1,
      "servo_min_us": [498, 1074, 1074, 1001, 1001, 976], // mech min
      "servo_max_us": [2500, 1953, 1953, 2002, 2002, 1733],
      // hysteresis_votes left at default
      "gesture_velocity": {
          "biceps_flex": [0, 200, 0, 0, 0, 0],   // close elbow
      },
    }
    """
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write(sample); path = f.name
    try:
        _, _, _, beh, _ = cpcu_dsp.load_dsp_runtime_config(
            ["rest", "biceps_flex"], path=path)
        ASSERT(beh.get("biceps_flex", {}).get("mode") == "velocity",
               "JSONC parsed and velocity row applied")
    finally:
        os.unlink(path)


# ══════════════════════════════════════════════════════════════════════
#  TB-DSP17 — Soft-grip clamp
# ══════════════════════════════════════════════════════════════════════

def test_grip_firm_us_loaded():
    """grip_firm_us is parsed from runtime.json into the loader's
    5th return value, defaulting to 1100 when absent or invalid."""
    print("\n--- TB-DSP17: grip_firm_us loader ---")
    import cpcu_dsp, tempfile, os

    # Default when absent
    sample_default = """
    {
      "schema_version": 1,
      "servo_min_us": [498, 1074, 1074, 1001, 1001, 976],
      "servo_max_us": [2500, 1953, 1953, 2002, 2002, 1733]
    }
    """
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write(sample_default); path = f.name
    try:
        _, _, _, _, gf = cpcu_dsp.load_dsp_runtime_config(["rest"], path=path)
        ASSERT(gf == cpcu_dsp.GRIP_FIRM_US_DEFAULT,
               f"absent grip_firm_us defaults to {gf}")
    finally:
        os.unlink(path)

    # Honored when present
    sample_present = """
    {
      "schema_version": 1,
      "servo_min_us": [498, 1074, 1074, 1001, 1001, 976],
      "servo_max_us": [2500, 1953, 1953, 2002, 2002, 1733],
      "grip_firm_us": 1150
    }
    """
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write(sample_present); path = f.name
    try:
        _, _, _, _, gf = cpcu_dsp.load_dsp_runtime_config(["rest"], path=path)
        ASSERT(gf == 1150, f"present grip_firm_us parsed as {gf}")
    finally:
        os.unlink(path)

    # Out of range falls back to default
    sample_bad = """
    {
      "schema_version": 1,
      "servo_min_us": [498, 1074, 1074, 1001, 1001, 976],
      "servo_max_us": [2500, 1953, 1953, 2002, 2002, 1733],
      "grip_firm_us": 99999
    }
    """
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write(sample_bad); path = f.name
    try:
        _, _, _, _, gf = cpcu_dsp.load_dsp_runtime_config(["rest"], path=path)
        ASSERT(gf == cpcu_dsp.GRIP_FIRM_US_DEFAULT,
               f"out-of-range grip_firm_us rejected, got {gf}")
    finally:
        os.unlink(path)


# ══════════════════════════════════════════════════════════════════════
#  Driver
# ══════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("  TB-DSP — cpcu_dsp.py pipeline validation")
    print("=" * 60)

    test_api_surface()
    test_bandpass()
    test_bandpass_rejects_dc()
    test_notch()
    test_envelope()
    test_extract_features()
    test_features_scale_with_amplitude()
    test_process_window()
    test_gesture_servo_map()
    test_constants()
    test_runtime_config_loader_defaults()
    test_runtime_config_loader_velocity()
    test_runtime_config_loader_clamping()
    test_runtime_config_loader_unknown_class()
    test_runtime_config_loader_invariant_violation()
    test_runtime_config_jsonc_comments()
    test_grip_firm_us_loaded()

    print("\n" + "=" * 60)
    print(f"  RESULTS: {g_pass} PASS, {g_fail} FAIL")
    print("=" * 60)

    return 0 if g_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
