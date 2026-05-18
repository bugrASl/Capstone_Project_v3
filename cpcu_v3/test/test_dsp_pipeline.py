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
    7. gestures.json — valid structure, rest exists, servo refs valid.
    8. Constants — INPUT_FS_HZ=1000, TARGET_FS_HZ=200, DECIMATE_FACTOR=5,
       WINDOW_HI=400, NUM_SERVOS=6.

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
                 "WINDOW_HI", "NUM_SERVOS"):
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

    from cpcu_dsp import process_window, WINDOW_HI, WINDOW_LO

    fs                  =   2000
    t                   =   np.arange(WINDOW_HI) / fs
    # 100 Hz band-of-interest tone + 50 Hz mains we want notched out
    sig                 =   (0.4 * np.sin(2 * np.pi * 100 * t) +
                             0.3 * np.sin(2 * np.pi *  50 * t))

    cleaned, env, feat  =   process_window(sig)

    ASSERT(len(cleaned) == WINDOW_LO,
           f"cleaned length = {len(cleaned)} == WINDOW_LO ({WINDOW_LO})")
    ASSERT(len(env) == WINDOW_LO,
           f"env length = {len(env)} == WINDOW_LO ({WINDOW_LO})")
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
    """v3: validate gestures.json structure (replaces old GESTURE_SERVO_MAP test)."""
    print("\n--- TB-DSP09: gestures.json validation ---")
    import json, os
    gs_path = os.path.join(os.path.dirname(__file__), "..", "config", "gestures.json")
    ASSERT(os.path.exists(gs_path), f"gestures.json exists at {gs_path}")
    with open(gs_path) as f:
        gs = json.load(f)
    gestures = gs.get("gestures", {})
    servos = gs.get("servo_channels", {})
    ASSERT(len(gestures) >= 1, f"has {len(gestures)} gestures (>= 1)")
    ASSERT("rest" in gestures, "'rest' gesture exists")
    ASSERT(gestures["rest"]["mode"] == "freeze", "rest is freeze mode")
    for gname, gdef in gestures.items():
        for sname in gdef.get("channels", {}):
            ASSERT(sname in servos,
                   f"{gname} references valid servo '{sname}'")


# ══════════════════════════════════════════════════════════════════════
#  Test 8 — Constants
# ══════════════════════════════════════════════════════════════════════

def test_constants():
    """Pin down the constants that other parts of the system depend on."""
    print("\n--- TB-DSP10: constants ---")

    from cpcu_dsp import (INPUT_FS_HZ, TARGET_FS_HZ, DECIMATE_FACTOR,
                          WINDOW_HI, WINDOW_LO, NUM_SERVOS)

    ASSERT(INPUT_FS_HZ == 1000,
           f"INPUT_FS_HZ = {INPUT_FS_HZ} (must match BSAU 1 kHz packet rate)")
    ASSERT(TARGET_FS_HZ == 200,
           f"TARGET_FS_HZ = {TARGET_FS_HZ} (must match training rate)")
    ASSERT(DECIMATE_FACTOR == INPUT_FS_HZ // TARGET_FS_HZ,
           f"DECIMATE_FACTOR = {DECIMATE_FACTOR} (= INPUT/TARGET)")
    ASSERT(WINDOW_HI == 200,
           f"WINDOW_HI = {WINDOW_HI} (= 200ms @ 2kHz)")
    ASSERT(WINDOW_LO == 40,
           f"WINDOW_LO = {WINDOW_LO} (= 200ms @ 200Hz)")
    ASSERT(NUM_SERVOS == 6,
           f"NUM_SERVOS = {NUM_SERVOS} (must match IPC_NUM_SERVOS)")


# ══════════════════════════════════════════════════════════════════════
#  TB-DSP11 — Velocity-mode runtime config loader
# ══════════════════════════════════════════════════════════════════════

def test_runtime_config_loader_defaults():
    """v3: load_gestures returns defaults when gestures.json missing."""
    print("\n--- TB-DSP11: config loader (defaults) ---")
    import cpcu_dsp
    gestures, channels, conf, hyst = cpcu_dsp.load_gestures("/tmp/nonexistent.json")
    ASSERT("rest" in gestures, "default has rest gesture")
    ASSERT(gestures["rest"]["mode"] == "freeze", "rest defaults to freeze")
    ASSERT(len(channels) > 0, f"default channels: {channels}")
    ASSERT(conf["curve"] == "quadratic", f"default curve: {conf['curve']}")
    ASSERT(hyst["rest_to_active"] > 0, f"default hysteresis: {hyst}")


def test_runtime_loader():
    """v3: load_runtime returns defaults when runtime.json missing."""
    print("\n--- TB-DSP12: runtime loader (defaults) ---")
    import cpcu_dsp
    _, grip = cpcu_dsp.load_runtime("/tmp/nonexistent.json")
    ASSERT(800 <= grip <= 2000, f"default grip_firm={grip} in valid range")


def test_gestures_json_valid():
    """v3: actual gestures.json loads without error."""
    print("\n--- TB-DSP13: gestures.json loads ---")
    import cpcu_dsp, os
    gs_path = os.path.join(os.path.dirname(__file__), "..", "config", "gestures.json")
    gestures, channels, conf, hyst = cpcu_dsp.load_gestures(gs_path)
    ASSERT(len(gestures) >= 2, f"has {len(gestures)} gestures")
    ASSERT("rest" in gestures, "has rest")
    ASSERT(len(channels) >= 1, f"has {len(channels)} EMG channels")
    for gname, gdef in gestures.items():
        ASSERT("mode" in gdef, f"{gname} has mode")
        ASSERT("_rates" in gdef, f"{gname} has resolved _rates")
    ASSERT(conf["floor_pct"] < conf["ceil_pct"], "floor < ceil")
    ASSERT(hyst["active_to_active"] >= hyst["active_to_rest"],
           "active→active votes >= active→rest")


def test_confidence_scale():
    """v3: confidence_scale quadratic curve."""
    print("\n--- TB-DSP14: confidence curve ---")
    from cpcu_dsp import confidence_scale
    ASSERT(confidence_scale(0.0, 0.4, 0.85, "quadratic") == 0.0, "below floor = 0")
    ASSERT(confidence_scale(1.0, 0.4, 0.85, "quadratic") == 1.0, "above ceil = 1")
    mid = confidence_scale(0.625, 0.4, 0.85, "quadratic")  # midpoint
    ASSERT(0.2 < mid < 0.3, f"midpoint = {mid:.3f} (quadratic < linear 0.5)")
    lin = confidence_scale(0.625, 0.4, 0.85, "linear")
    ASSERT(0.45 < lin < 0.55, f"linear midpoint = {lin:.3f}")


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
    test_runtime_loader()
    test_gestures_json_valid()
    test_confidence_scale()

    print("\n" + "=" * 60)
    print(f"  RESULTS: {g_pass} PASS, {g_fail} FAIL")
    print("=" * 60)

    return 0 if g_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
