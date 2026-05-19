#!/usr/bin/env python3
"""cpcu_dsp.py — live DSP + ML inference pipeline for the CPCU.

Reads EMG from the IPC ring, filters, extracts features, runs ML inference
for one or more gesture groups in parallel, integrates servo targets via
velocity mode, and publishes motor commands.

Config sources (priority order):
  1. config/gestures.json     — gesture_groups + servo_channels (schema v5)
  2. models/velocity_map.json — operator-calibrated rates (overrides)
  3. config/runtime.json      — hardware tuning (smoother, grip, bias)
  4. models/*.pkl             — trained ML model + scaler (per group)

Top-level structure (read this first):
  - CONFIG LOADERS:   load_gestures, load_velocity_map, load_runtime, discover_model
  - DSP FILTERS:      butter_bandpass, notch_filter, envelope, extract_features,
                      process_window  (per-channel pipeline, ports the team's code)
  - GROUP STATE:      GroupState  (one instance per gesture_group in gestures.json)
  - HELPERS:          confidence_scale, _resolve_gesture_rates, UART debug
  - INFERENCE LOOP:   run_inference  (delegates to small helper functions)
  - CALIBRATION:      run_calibrate  (record rest, compute thresholds)
  - ENTRY:            main
"""
import argparse
import glob
import json
import os
import re
import signal
import sys
import time
from collections import deque

import numpy as np
from scipy.signal import butter, decimate, filtfilt, iirnotch

from cpcu_ipc_bridge import IPCBridge

# ══════════════════════════════════════════════════════════════════════
#  PATHS / CONSTANTS
# ══════════════════════════════════════════════════════════════════════

REPO_ROOT       = os.environ.get(
    "CPCU_ROOT", os.path.dirname(os.path.abspath(__file__)) + "/..")
GESTURES_PATH   = os.path.join(REPO_ROOT, "config", "gestures.json")
RUNTIME_PATH    = os.path.join(REPO_ROOT, "config", "runtime.json")
MODEL_DIR       = os.environ.get(
    "CPCU_MODEL_DIR", os.path.join(REPO_ROOT, "models"))
INSTALLED_MODEL = "/opt/cpcu/models"
INSTALLED_CFG   = "/opt/cpcu/config.json"
THRESHOLDS_PATH = os.path.join(MODEL_DIR, "noise_thresholds.json")

# Sampling rates / window sizing
INPUT_FS_HZ     = 1000                              # BSAU 1 kHz packet rate
TARGET_FS_HZ    = 200                               # after decimation
DECIMATE_FACTOR = INPUT_FS_HZ // TARGET_FS_HZ       # 5
WINDOW_MS       = 200
STRIDE_MS       = 100
WINDOW_HI       = INPUT_FS_HZ  * WINDOW_MS // 1000  # 200 samples @ 1 kHz
STRIDE_HI       = INPUT_FS_HZ  * STRIDE_MS // 1000  # 100 samples @ 1 kHz
WINDOW_LO       = TARGET_FS_HZ * WINDOW_MS // 1000  # 40  samples @ 200 Hz
BUFFER_SIZE     = WINDOW_HI * 4                     # 800 samples ring

# Hardware constants (matched to BSAU + cpcu_io)
ADC_MIDRAIL     = 2048
NUM_SERVOS      = 6
NUM_EMG_CH      = 8
SERVO_NEUTRAL   = 1500
DRAIN_PERIOD_S  = 0.020                             # main loop period
DRAIN_BATCH     = 200                               # max packets per drain
PROB_THRESH     = 0.65                              # min SVM prob to consider

# Default servo limit arrays — overwritten by load_gestures()
SERVO_MIN_US    = [498, 1074, 1074, 1001, 1001,  976]
SERVO_MAX_US    = [2500, 1953, 1953, 2002, 2002, 1733]

# Hardware-latency constants (from datasheets / wall-clock probes) — used
# in the periodic waterfall print. These MUST match cpcu_tui.h's
# TUI_LAT_* set so the two views report the same E2E budget.
#
# Stage timing model:
#   BSAU side: ADC sampling + WL_Pack + NRF wireless air time + ACK.
#              All fixed (datasheet) — we can't measure these from CPCU.
#   CPCU side: measured per-tick by run_inference as `lat_pkt_us` =
#              motor IPC write time − rx_time_us. Since rx_time_us is
#              stamped in cpcu_io BEFORE the NRF SPI read, this single
#              measurement covers SPI_UNPACK + ring dwell + DSP compute
#              + motor IPC write. Don't double-count its sub-stages
#              when computing the E2E total.
#   Smoother/I²C: cpcu_io runs a separate 50 Hz tick that reads
#              motor_cmd and pushes PWM to PCA9685. Fixed by I²C bus
#              clock × 6 servos.
#   Servo: SG90 mechanical step response, ~15 ms typical.
LAT_ADC_PACK_US     = 226   # STM32 ADC 6ch×2kHz + firmware pack
LAT_WIRELESS_US     = 332   # NRF24L01+ ESB: SPI upload + air + ACK
LAT_SPI_UNPACK_US   = 36    # CPCU NRF SPI read + WL_Unpack + IPC push
                            # (informational only — included in lat_pkt_us)
LAT_SMOOTHER_I2C_US = 610   # SMOOTH_Update + PCA9685 I²C 6 servos
LAT_SERVO_MECH_US   = 15000 # SG90 mechanical response (~15 ms typical)
LAT_TRANSPORT_US    = LAT_ADC_PACK_US + LAT_WIRELESS_US + LAT_SPI_UNPACK_US


# ══════════════════════════════════════════════════════════════════════
#  CONFIG LOADERS
# ══════════════════════════════════════════════════════════════════════

def _resolve_gesture_rates(gestures, name_to_idx):
    """Convert servo-name references inside a `gestures` dict to indexed arrays.

    Each gesture's ``channels`` dict uses servo NAMES (e.g. "Elbow") so JSON
    stays human-readable. The integration loop wants index arrays for speed,
    so we pre-compute and cache them as ``_rates`` / ``_snap`` / ``_smooth_v``
    / ``_smooth_a`` on the gesture definition itself.
    """
    for gname, gdef in gestures.items():
        raw_ch     = gdef.get("channels", {})
        rates      = [0]     * NUM_SERVOS
        snap_flags = [False] * NUM_SERVOS
        smoother_v = [0]     * NUM_SERVOS
        smoother_a = [0]     * NUM_SERVOS
        for sname, chdef in raw_ch.items():
            idx = name_to_idx.get(sname)
            if idx is None:
                print(f"[DSP] unknown servo '{sname}' in gesture '{gname}'",
                      flush=True)
                continue
            rates[idx]      = chdef.get("rate_us_s", 0)
            snap_flags[idx] = chdef.get("snap", False)
            so = gdef.get("smoother_override", {}).get(sname, {})
            smoother_v[idx] = so.get("velocity_us_s", 0)
            smoother_a[idx] = so.get("accel_us_s2", 0)
        gdef["_rates"]    = rates
        gdef["_snap"]     = snap_flags
        gdef["_smooth_v"] = smoother_v
        gdef["_smooth_a"] = smoother_a


def load_gestures(path=GESTURES_PATH):
    """Load gestures.json.

    Returns ``(groups_list, servo_channels_dict)`` where each group dict has:
        name          group identifier (e.g. "right_arm")
        gestures      dict of gestures, with _rates already resolved
        emg_channels  list of active EMG channel indices (e.g. [0,1,2])
        confidence    {"curve", "floor_pct", "ceil_pct"}
        hysteresis    {"rest_to_active", "active_to_rest", "active_to_active"}
        model_path    path string

    Supports schema v5 (gesture_groups) and v4 (flat gestures, single group)
    for backwards compatibility.
    """
    # Default hysteresis matches GroupState defaults (a2r=0 for instant
    # stop, r2a=a2a=3 for debounced activation).
    default_conf  = {"curve": "quadratic", "floor_pct": 40, "ceil_pct": 85}
    default_hyst  = {"rest_to_active":   3,
                     "active_to_rest":   0,
                     "active_to_active": 3}

    # Default two-arm layout used when gestures.json is missing entirely.
    # Mirrors the user's mechanical convention:
    #   right arm channels 0,1,2 → Forearm/Biceps/Triceps R
    #     - hand   (right forearm)  → Gripper OPEN
    #     - flex   (right biceps)   → arm UP
    #     - ext    (right triceps)  → arm DOWN
    #   left arm channels 3,4,5 → Forearm/Biceps/Triceps L
    #     - hand   (left forearm)   → Gripper CLOSE
    #     - flex   (left biceps)    → arm RIGHT
    #     - ext    (left triceps)   → arm LEFT
    # `channels` is intentionally empty here — the real per-servo rate
    # mappings come from config/gestures.json. These defaults exist
    # only so a totally empty install can still boot the DSP for sanity
    # checks; they're overwritten the moment a real config loads.
    default_groups = [
        {
            "name":         "right_arm",
            "gestures": {
                "rest": {"mode": "freeze", "channels": {}},
                "hand": {"mode": "velocity", "channels": {}},
                "flex": {"mode": "velocity", "channels": {}},
                "ext":  {"mode": "velocity", "channels": {}},
            },
            "emg_channels": [0, 1, 2],
            "confidence":   default_conf,
            "hysteresis":   default_hyst,
            "model_path":   "models/right_arm.pkl",
        },
        {
            "name":         "left_arm",
            "gestures": {
                "rest": {"mode": "freeze", "channels": {}},
                "hand": {"mode": "velocity", "channels": {}},
                "flex": {"mode": "velocity", "channels": {}},
                "ext":  {"mode": "velocity", "channels": {}},
            },
            "emg_channels": [3, 4, 5],
            "confidence":   default_conf,
            "hysteresis":   default_hyst,
            "model_path":   "models/left_arm.pkl",
        },
    ]

    try:
        with open(path) as f:
            gs = json.load(f)
    except Exception as e:
        print(f"[DSP] gestures.json load failed: {e}, using defaults",
              flush=True)
        return default_groups, {}

    # Servo limits — flat, single arm
    servo_ch    = gs.get("servo_channels", {})
    name_to_idx = {n: d["pca_ch"] for n, d in servo_ch.items()}
    _update_servo_limits(servo_ch)

    # Build the list of group dicts (schema v5 first, v4 fallback)
    groups = _build_group_list(gs, name_to_idx, default_conf, default_hyst)

    if not groups:
        groups = default_groups

    # Log summary
    print(f"[DSP] {len(groups)} gesture group(s):", flush=True)
    for g in groups:
        print(f"[DSP]   {g['name']}: {list(g['gestures'].keys())} "
              f"emg={g['emg_channels']} model={g['model_path']}", flush=True)
    print(f"[DSP] servo limits: min={SERVO_MIN_US} max={SERVO_MAX_US}",
          flush=True)
    return groups, servo_ch


def _update_servo_limits(servo_ch):
    """Refresh the module-global SERVO_MIN_US / SERVO_MAX_US from a
    servo_channels dict. PCA channel number drives the array index."""
    global SERVO_MIN_US, SERVO_MAX_US
    servo_list = sorted(servo_ch.items(), key=lambda x: x[1].get("pca_ch", 0))
    if not servo_list:
        return
    SERVO_MIN_US = [s[1].get("min_us", 500)  for s in servo_list[:NUM_SERVOS]]
    SERVO_MAX_US = [s[1].get("max_us", 2500) for s in servo_list[:NUM_SERVOS]]
    while len(SERVO_MIN_US) < NUM_SERVOS: SERVO_MIN_US.append(500)
    while len(SERVO_MAX_US) < NUM_SERVOS: SERVO_MAX_US.append(2500)


def _build_group_list(gs, name_to_idx, default_conf, default_hyst):
    """Parse either schema-v5 (gesture_groups) or schema-v4 (flat gestures)
    into a uniform list-of-dicts. Each entry is fully self-contained so the
    inference loop doesn't need to know which schema was on disk."""
    groups = []
    gg     = gs.get("gesture_groups")
    if gg:
        for gname, gdef in gg.items():
            gestures = gdef.get("gestures", {"rest": {"mode": "freeze"}})
            _resolve_gesture_rates(gestures, name_to_idx)
            groups.append({
                "name":         gname,
                "gestures":     gestures,
                "emg_channels": gdef.get("emg_channels", {})
                                    .get("active", [0, 1, 2]),
                "confidence":   gdef.get("confidence", default_conf),
                "hysteresis":   gdef.get("hysteresis", default_hyst),
                "model_path":   gdef.get("model_path", ""),
            })
    elif "gestures" in gs:
        # v4 backward compatibility: wrap the flat dict as one synthetic group
        gestures = gs["gestures"]
        _resolve_gesture_rates(gestures, name_to_idx)
        groups.append({
            "name":         "gesture_0",
            "gestures":     gestures,
            "emg_channels": gs.get("emg_channels", {})
                                .get("active", [0, 1, 2]),
            "confidence":   gs.get("confidence",  default_conf),
            "hysteresis":   gs.get("hysteresis",  default_hyst),
            "model_path":   gs.get("model_path", ""),
        })
    return groups


def load_velocity_map(operator="default"):
    """Operator-calibrated rate overrides from velocity_map[_OP].json.

    Returns ``{gesture_name: {servo_name: rate_us_s}}`` or ``{}`` if no
    calibrated map is on disk. The bare template (no ``calibrated_at``
    field) is treated as absent."""
    if operator == "default":
        path = os.path.join(MODEL_DIR, "velocity_map.json")
    else:
        path = os.path.join(MODEL_DIR, f"velocity_map_{operator}.json")
    try:
        with open(path) as f:
            vm = json.load(f)
        if not vm.get("calibrated_at"):
            return {}
        result = {}
        for gname, servos in vm.get("gesture_levels", {}).items():
            result[gname] = {sname: sdef.get("rate_us_s", 0)
                             for sname, sdef in servos.items()}
        print(f"[DSP] velocity_map ({operator}): {list(result.keys())}",
              flush=True)
        return result
    except (OSError, ValueError):
        return {}


def load_runtime(path=None):
    """Read grip_firm_us from runtime.json (or installed copy).

    Tolerates the project's comment + trailing-comma JSON dialect by
    stripping ``//`` comments and orphan commas before json.loads."""
    grip_firm  = 1100
    candidates = [path] if path else [INSTALLED_CFG, RUNTIME_PATH]
    for p in candidates:
        if not p:
            continue
        try:
            with open(p) as f:
                text = f.read()
            text = re.sub(r'//[^\n]*',     '',    text)
            text = re.sub(r',(\s*[}\]])',  r'\1', text)
            raw  = json.loads(text)
            if "grip_firm_us" in raw:
                v = int(raw["grip_firm_us"])
                if 800 <= v <= 2200:
                    grip_firm = v
            print(f"[DSP] runtime: grip_firm={grip_firm}", flush=True)
            return grip_firm
        except Exception as e:
            print(f"[DSP] runtime {p}: {e}", flush=True)
    return grip_firm


def discover_model(model_path=""):
    """Load an ML model from a hint path or by scanning models/ dirs.

    Returns ``(model, scaler)`` on success or ``(None, None)`` if joblib
    is missing or no usable .pkl was found. Accepts a dict ``{model,
    scaler}`` only — that's the format the AI team committed to in §9 of
    SESSION_HANDOFF.md."""
    try:
        import joblib
    except ImportError:
        print("[DSP] joblib missing — feature-only mode", flush=True)
        return None, None

    # 1. explicit path (CLI / per-group hint)
    if model_path:
        candidates = [model_path]
        if not os.path.isabs(model_path):
            candidates.append(os.path.join(REPO_ROOT, model_path))
            candidates.append(os.path.join("/opt/cpcu", model_path))
        for p in candidates:
            try:
                cp = joblib.load(p)
                if isinstance(cp, dict) and "model" in cp and "scaler" in cp:
                    print(f"[DSP] model: {p}", flush=True)
                    return cp["model"], cp["scaler"]
            except Exception:
                pass

    # 2. scan default model dirs (used when group has no model_path)
    for d in [MODEL_DIR, INSTALLED_MODEL]:
        for pkl in sorted(glob.glob(os.path.join(d, "*.pkl"))):
            try:
                cp = joblib.load(pkl)
                if isinstance(cp, dict) and "model" in cp and "scaler" in cp:
                    print(f"[DSP] model: {pkl}", flush=True)
                    return cp["model"], cp["scaler"]
            except Exception as e:
                print(f"[DSP] {pkl}: {e}", flush=True)

    print("[DSP] no model found — feature-only mode", flush=True)
    return None, None


# ══════════════════════════════════════════════════════════════════════
#  CONFIG DIGEST (for the C TUI to read without parsing JSON itself)
# ══════════════════════════════════════════════════════════════════════
#
# The CONFIG page in cpcu_tui_render.c can't easily parse gestures.json
# itself (no embedded JSON parser), so we serialise a human-readable
# digest here whenever cpcu_dsp.py boots. The TUI then fopen()s the
# digest and prints it line-by-line — no IPC schema bumps required.
#
# Path: /tmp/cpcu_gestures_digest.txt   (re-written on every spawn)

GESTURES_DIGEST_PATH = "/tmp/cpcu_gestures_digest.txt"


def _write_gestures_digest(group_states, servo_channels):
    """Dump a pretty-printed summary of the running config to a text
    file the TUI can read. One section per:
        - servo channels (sorted by pca_ch)
        - each gesture group (EMG, model, conf curve, hysteresis,
          per-gesture servo mapping with rate + snap + grip_firm)
    Failures are non-fatal: the TUI simply won't show the section."""
    try:
        lines = []
        lines.append("# auto-generated by cpcu_dsp.py — do not edit")
        lines.append("")
        # Servos
        lines.append("SERVOS (sorted by PCA channel)")
        sc = sorted(servo_channels.items(),
                    key=lambda x: x[1].get("pca_ch", 0))
        for name, sd in sc:
            lines.append(
                f"  PCA{sd.get('pca_ch','?'):>2}  {name:<10}  "
                f"min={sd.get('min_us','?'):>4}  "
                f"max={sd.get('max_us','?'):>4}  "
                f"neutral={sd.get('neutral_us','?'):>4}  us")
        lines.append("")
        # Groups
        lines.append(f"GESTURE GROUPS ({len(group_states)} active)")
        for gst in group_states:
            lines.append("")
            lines.append(f"  [{gst.name}]")
            lines.append(f"    EMG channels:  {gst.emg_channels}")
            lines.append(f"    Model:         {gst.model_path or '(unset)'}"
                         f"   inference={'ON' if gst.inference_on else 'OFF'}")
            lines.append(f"    Confidence:    curve={gst.conf_curve}  "
                         f"floor={int(gst.conf_floor*100)}%  "
                         f"ceil={int(gst.conf_ceil*100)}%")
            a2r_tag = " (INSTANT)" if gst.hyst_a2r == 0 else ""
            lines.append(f"    Hysteresis:    "
                         f"r→a={gst.hyst_r2a}  "
                         f"a→r={gst.hyst_a2r}{a2r_tag}  "
                         f"a→a={gst.hyst_a2a}")
            lines.append(f"    Gestures ({len(gst.gestures)}):")
            for gname, gd in gst.gestures.items():
                mode     = gd.get("mode", "?")
                channels = gd.get("channels", {})  # servo-name keyed dict
                if not channels:
                    summary = "(freeze)" if mode == "freeze" else "—"
                else:
                    parts = []
                    for sn, chdef in channels.items():
                        r     = chdef.get("rate_us_s", 0)
                        snap  = " SNAP" if chdef.get("snap") else ""
                        arrow = "↑" if r > 0 else ("↓" if r < 0 else "·")
                        parts.append(f"{sn} {arrow}{abs(r)}{snap}")
                    summary = ", ".join(parts)
                firm = gd.get("grip_firm_us")
                if firm:
                    summary += f"  firm={firm}us"
                lines.append(f"      {gname:<8} [{mode:<8}]  {summary}")
        lines.append("")
        with open(GESTURES_DIGEST_PATH, "w") as f:
            f.write("\n".join(lines) + "\n")
        print(f"[DSP] config digest -> {GESTURES_DIGEST_PATH}", flush=True)
    except Exception as e:
        print(f"[DSP] digest write failed: {e}", flush=True)


# ══════════════════════════════════════════════════════════════════════
#  DSP FILTERS — direct port of the AI team's pipeline
# ══════════════════════════════════════════════════════════════════════

def butter_bandpass(data, low, high, fs, order=4):
    """Bandpass with end-stop clamping to keep the filter well-defined when
    the requested high frequency approaches Nyquist."""
    nyq = 0.5 * fs
    if high >= nyq:
        high = nyq * 0.95
    b, a = butter(order, [low/nyq, high/nyq], btype='band')
    return filtfilt(b, a, data)


def notch_filter(data, f0, fs, q=30.0):
    """50/100/200 Hz mains-harmonic notch. No-op above Nyquist."""
    nyq = 0.5 * fs
    w0  = f0 / nyq
    if w0 >= 1:
        return data
    b, a = iirnotch(w0, q)
    return filtfilt(b, a, data)


def envelope(data, fs, cutoff=3.0):
    """Full-wave rectify + low-pass to recover the EMG amplitude envelope."""
    nyq  = 0.5 * fs
    b, a = butter(4, cutoff/nyq, btype='low')
    return filtfilt(b, a, np.abs(data))


#: Number of features the per-channel extractor emits. Kept as a single
#: module-level constant so every place that needs the count (model
#: validation, IPC export sizing, log lines) reads the same value. If
#: you add a new feature inside extract_features(), bump this and
#: retrain the model — the scaler's n_features_in_ must match
#: NUM_FEATURES_PER_CHANNEL * num_channels_in_group.
NUM_FEATURES_PER_CHANNEL = 7


def extract_features(clean, env):
    """Per-channel feature vector. Mirrors predictX.py exactly so the
    Pi-side pipeline matches the desktop training pipeline byte-for-byte.

    Returns a 7-element list in this fixed order:
        [0] rms       Root-mean-square of cleaned signal (energy proxy)
        [1] var       Variance of cleaned signal (spread)
        [2] wl        Waveform length: sum |diff| / N (complexity)
        [3] env_mean  Mean of low-passed envelope (slow activation level)
        [4] mav       Mean absolute value (alt. energy proxy, robust)
        [5] zc        Zero-crossings (sign changes — frequency content)
        [6] ssc       Slope sign changes (sign flips of derivative —
                      catches local maxima/minima, related to second
                      derivative crossings)
    """
    eps      = 1e-8
    rms      = float(np.sqrt(np.mean(clean ** 2) + eps))
    var      = float(np.var(clean))
    wl       = float(np.sum(np.abs(np.diff(clean))) / (len(clean) + eps))
    env_mean = float(np.mean(env))
    mav      = float(np.mean(np.abs(clean)))
    # zero-crossings: count sign changes of the cleaned signal
    zc       = int(np.sum(np.diff(np.sign(clean)) != 0))
    # slope-sign changes: sign changes of the first derivative
    diff_sig = np.diff(clean)
    ssc      = int(np.sum(np.diff(np.sign(diff_sig)) != 0))
    return [rms, var, wl, env_mean, mav, float(zc), float(ssc)]


def process_window(window_hi):
    """Full per-channel pipeline on a WINDOW_HI-sample window at INPUT_FS_HZ.

    DC remove → 20-450 Hz bandpass → 50/100/200 Hz notches → decimate to
    TARGET_FS_HZ → envelope → features. Returns ``(cleaned, env, feats)``."""
    centered = window_hi - np.mean(window_hi)
    bp       = butter_bandpass(centered, 20.0, 450.0, INPUT_FS_HZ)
    n50      = notch_filter(bp,   50.0,  INPUT_FS_HZ)
    n100     = notch_filter(n50,  100.0, INPUT_FS_HZ)
    n200     = notch_filter(n100, 200.0, INPUT_FS_HZ)
    cleaned  = decimate(n200, DECIMATE_FACTOR, zero_phase=True)
    env      = envelope(cleaned, TARGET_FS_HZ, cutoff=3.0)
    return cleaned, env, extract_features(cleaned, env)


# ══════════════════════════════════════════════════════════════════════
#  CONFIDENCE CURVE
# ══════════════════════════════════════════════════════════════════════

def confidence_scale(conf_frac, floor, ceil, curve="quadratic"):
    """Map SVM probability into a velocity scale [0, 1].

    * ``"quadratic"`` — slow start, fast finish (the default; gives clean
      gesture entry without overshoot).
    * ``"linear"``    — proportional to confidence above the floor.
    * ``"none"``      — always 1.0 (servos run at full rate_us_s as long as
      the gesture is detected; demo/teaching mode).
    """
    if curve == "none":
        return 1.0
    if conf_frac <= floor:
        return 0.0
    if conf_frac >= ceil:
        return 1.0
    t = (conf_frac - floor) / (ceil - floor)
    if curve == "quadratic":
        return t * t
    return t


# ══════════════════════════════════════════════════════════════════════
#  UART DEBUG (optional)
# ══════════════════════════════════════════════════════════════════════

UART_PORT = os.environ.get("CPCU_UART_DEBUG", "")
_uart     = None


def _uart_init():
    """Open the debug serial port if CPCU_UART_DEBUG was set by launch.sh."""
    global _uart
    if not UART_PORT:
        return
    try:
        import serial
        _uart = serial.Serial(UART_PORT, 115200, timeout=0)
        print(f"[DSP] UART debug → {UART_PORT}", flush=True)
    except Exception as e:
        print(f"[DSP] UART: {e}", flush=True)


def _uart_send(gesture, conf, feats):
    """Emit ``ts,gesture,conf,f0,f1,...\\n`` for an attached USB-UART logger."""
    if _uart is None:
        return
    try:
        ts  = int(time.time() * 1000)
        csv = ",".join(f"{v:.6f}" for v in feats)
        _uart.write(f"{ts},{gesture},{conf:.3f},{csv}\n".encode())
    except Exception:
        pass


# ══════════════════════════════════════════════════════════════════════
#  PER-GROUP STATE
# ══════════════════════════════════════════════════════════════════════

class GroupState:
    """All per-classifier runtime state.

    There is one ``GroupState`` per gesture group in gestures.json. The
    inference loop iterates these in order and each contributes its own
    velocity vector into the shared ``current_target[]`` array. Group 0 is
    designated "primary" for IPC export (the TUI only shows one gesture
    label at a time, so we pick group 0 to broadcast).

    The class owns:
      - identity:       name, gestures dict (with _rates resolved)
      - signal source:  emg_channels (list of channel indices)
      - rolling buffer: ``buffers[i]`` is a deque per active EMG channel
      - ML:             model + scaler + ``inference_on`` flag
      - confidence:     curve + floor + ceil  (resolved to 0..1 fractions)
      - hysteresis:     r2a / a2r / a2a thresholds + a single counter +
                        ``current_state`` string label
      - per-tick:       conf_pct, class_conf list, last_active class index
    """

    def __init__(self, group_def, prob_thresh=PROB_THRESH):
        self.name         = group_def["name"]
        self.gestures     = group_def["gestures"]
        self.emg_channels = group_def["emg_channels"]
        self.model_path   = group_def.get("model_path", "")
        self.prob_thresh  = prob_thresh

        # confidence
        cc                = group_def.get("confidence", {})
        self.conf_curve   = cc.get("curve", "quadratic")
        self.conf_floor   = cc.get("floor_pct", 40) / 100.0
        self.conf_ceil    = cc.get("ceil_pct",  85) / 100.0
        if self.conf_floor >= self.conf_ceil:
            self.conf_floor, self.conf_ceil = 0.40, 0.85

        # Hysteresis — debounce ML predictions to prevent jitter.
        # Defaults tuned from bench testing:
        #   r2a = 3  (rest → active): need 3 consecutive votes for a
        #            new gesture to defeat resting state. Filters out
        #            single-frame misclassifications.
        #   a2r = 0  (active → rest): INSTANT. The moment the model
        #            says "rest", we stop driving the arm. Critical
        #            for operator safety — any lag here means the arm
        #            keeps moving after the user relaxes.
        #   a2a = 3  (active → active): 3 votes to switch between
        #            active gestures (e.g. flex → ext). Same rationale
        #            as r2a, with the added bonus of preventing
        #            antagonist-muscle co-contraction from causing
        #            rapid label flips.
        hy                = group_def.get("hysteresis", {})
        self.hyst_r2a     = hy.get("rest_to_active",   3)
        self.hyst_a2r     = hy.get("active_to_rest",   0)
        self.hyst_a2a     = hy.get("active_to_active", 3)

        # rolling buffers — one deque per active EMG channel
        self.buffers      = [deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
                             for _ in self.emg_channels]

        # ML model — populated by attach_model()
        self.model        = None
        self.scaler       = None
        self.inference_on = False

        # per-tick scratch
        self.current_state = "rest"
        self.consec_count  = 0
        self.last_active   = 0
        self.conf_pct      = 0
        self.class_conf    = []

    def attach_model(self):
        """Try to load self.model_path. Falls back to default scan."""
        self.model, self.scaler = discover_model(self.model_path)
        self.inference_on       = (self.model is not None)
        if self.inference_on:
            classes  = [str(c) for c in self.model.classes_]
            n_feat   = self.scaler.n_features_in_
            expected = len(self.emg_channels) * NUM_FEATURES_PER_CHANNEL
            if n_feat != expected:
                print(f"[DSP] {self.name}: model expects {n_feat} features, "
                      f"pipeline produces {expected} "
                      f"({len(self.emg_channels)} ch × "
                      f"{NUM_FEATURES_PER_CHANNEL} feat/ch)", flush=True)
            for cls in classes:
                if cls not in self.gestures:
                    print(f"[DSP] {self.name}: model class '{cls}' not in "
                          f"gestures — will fall back to neutral", flush=True)
            print(f"[DSP] {self.name}: classes={classes}", flush=True)

    def ingest_sample(self, ei, si, samples):
        """Append one BSAU sample-pair entry into the per-channel deques."""
        for bi, ch in enumerate(self.emg_channels):
            self.buffers[bi].append(int(samples[ei, si, ch]) - ADC_MIDRAIL)

    def reset_to_rest(self):
        """Force the group back to its rest state (used in SAFE / edit mode)."""
        self.current_state = "rest"
        self.consec_count  = 0


# ══════════════════════════════════════════════════════════════════════
#  INFERENCE — modular helpers
# ══════════════════════════════════════════════════════════════════════

def _apply_velocity_overrides(group_states, vel_overrides, servo_channels):
    """Patch gesture _rates with operator-calibrated values from
    velocity_map.json. No-op when the operator has not run ./launch.sh
    calibrate."""
    if not vel_overrides:
        return
    name_to_idx = {n: d["pca_ch"] for n, d in servo_channels.items()}
    for gst in group_states:
        for gname, overrides in vel_overrides.items():
            if gname not in gst.gestures:
                continue
            for sname, rate in overrides.items():
                idx = name_to_idx.get(sname)
                if idx is not None and "_rates" in gst.gestures[gname]:
                    gst.gestures[gname]["_rates"][idx] = rate


def _resolve_grip_firm(group_states, grip_firm_default):
    """Cache the effective grip_firm_us per gesture. Falls back to the
    runtime.json value when a gesture doesn't override it."""
    for gst in group_states:
        for gdef in gst.gestures.values():
            gdef["_grip_firm"] = gdef.get("grip_firm_us", grip_firm_default)


def _drain_ring_into_groups(ipc, group_states, rx_times, seq_history):
    """Pull a batch off the IPC ring and distribute each sample-pair to every
    group's per-channel buffers.

    Returns ``(added_samples, drain_seconds)``. ``added_samples`` is the
    number of decimation-input samples that landed in the group buffers
    (each BSAU packet contributes 2 samples, since BSAU sends 2 samples per
    1 kHz packet)."""
    t_start = time.monotonic()
    batch   = ipc.pop_sensor_batch(DRAIN_BATCH)
    n       = batch.get('count', 0)
    if n == 0:
        return 0, time.monotonic() - t_start

    ipc.inc_dsp_batches(n)
    samples   = batch['samples']
    batch_seq = batch.get('seq')
    batch_rx  = batch.get('rx_time_us')

    for ei in range(n):
        for si in range(2):                                    # 2 samples / pkt
            for gst in group_states:
                gst.ingest_sample(ei, si, samples)
        # track packet rx_time + seq for latency measurement
        if batch_rx is not None and batch_rx[ei] > 0:
            rx_times.append(int(batch_rx[ei]))
            rx_times.append(int(batch_rx[ei]))
        if batch_seq is not None:
            seq_history.append(int(batch_seq[ei]))
            seq_history.append(int(batch_seq[ei]))

    return n * 2, time.monotonic() - t_start


def _features_and_inference(gst, rms_8ch, ipc):
    """Process the most recent window for one group: per-channel filter +
    feature extract, then SVM inference + hysteresis state-machine update.

    Sets ``gst.conf_pct`` / ``gst.class_conf`` / ``gst.last_active``."""
    features_flat = []
    for bi, ch in enumerate(gst.emg_channels):
        w  = np.array(list(gst.buffers[bi])[-WINDOW_HI:], dtype=np.float64)
        _cl, ev, feats = process_window(w)
        features_flat.extend(feats)
        rms_8ch[ch] = feats[0]
        try:
            ipc.write_dsp_filtered_window(ch, ev,
                                          sample_rate_hz=TARGET_FS_HZ)
        except Exception:
            pass

    # Reset per-tick scratch in case inference is off / fails
    gst.conf_pct   = 0
    gst.class_conf = []

    if not gst.inference_on:
        return

    X     = np.asarray(features_flat, dtype=np.float64).reshape(1, -1)
    Xs    = gst.scaler.transform(X)
    probs = gst.model.predict_proba(Xs)[0]
    ai    = int(np.argmax(probs))
    label = str(gst.model.classes_[ai])
    conf  = float(probs[ai])

    # Hysteresis state-machine
    _update_hysteresis(gst, label, conf)

    gst.last_active = ai
    gst.class_conf  = list(probs)
    gst.conf_pct    = int(round(conf * 100.0))


def _update_hysteresis(gst, label, conf):
    """Advance the group's debounce counter. A confident new label is only
    accepted once it has held for the configured number of windows.

    Three thresholds, picked by the transition type:
      r→a  rest → any active gesture
      a→r  active → rest
      a→a  active → another active gesture
    """
    if conf <= gst.prob_thresh or label == gst.current_state:
        gst.consec_count = 0
        return
    if gst.current_state == "rest":
        needed = gst.hyst_r2a
    elif label == "rest":
        needed = gst.hyst_a2r
    else:
        needed = gst.hyst_a2a
    gst.consec_count += 1
    if gst.consec_count >= needed:
        gst.current_state = label
        gst.consec_count  = 0


def _publish_primary_state(ipc, gst):
    """Mirror group-0 hysteresis state into the IPC export so cpcu_tui can
    render the debounce progress bar. Only group 0 is published — the TUI
    has one gesture slot."""
    needed = gst.hyst_r2a if gst.current_state == "rest" else gst.hyst_a2a
    htype  = 0            if gst.current_state == "rest" else 2
    ipc.write_hysteresis_state(gst.consec_count, needed, htype)


def _integrate_velocity(group_states, current_target, dt,
                        grip_firm_default):
    """Combine per-group velocity contributions into ``current_target``.

    Each group contributes additively. Servos a group doesn't reference
    (rate == 0 in its current gesture) get no contribution from that group.

    Special cases:
      * ALL groups at rest  → snap to neutral (no integration)
      * gesture mode "freeze" → that group contributes nothing
      * gripper (servo index 5) — clamp from below by the gesture's
        ``_grip_firm`` value so it never goes slack mid-grasp.
    """
    if all(gst.current_state == "rest" for gst in group_states):
        return [SERVO_NEUTRAL] * NUM_SERVOS

    for gst in group_states:
        gdef = gst.gestures.get(gst.current_state)
        if gdef is None or gdef.get("mode") == "freeze":
            continue

        scale     = confidence_scale(gst.conf_pct / 100.0,
                                     gst.conf_floor, gst.conf_ceil,
                                     gst.conf_curve)
        rates     = gdef.get("_rates",     [0]*NUM_SERVOS)
        grip_firm = gdef.get("_grip_firm", grip_firm_default)

        for s in range(NUM_SERVOS):
            if rates[s] == 0:
                continue
            nv = current_target[s] + rates[s] * dt * scale
            nv = max(SERVO_MIN_US[s], min(SERVO_MAX_US[s], nv))
            if s == 5 and nv < grip_firm:           # gripper safety floor
                nv = grip_firm
            current_target[s] = nv

    return current_target


def _handle_edit_mode(ipc, group_states, edit_mode):
    """The TUI's "live edit" mode parks the arm at neutral and waits for the
    user to finish editing config. We respond with an acknowledgement back
    on the IPC handshake. Returns ``(new_edit_mode, snap_neutral)``."""
    edit_req = ipc.read_edit_request()
    if edit_req:
        if not edit_mode:
            for gst in group_states:
                gst.reset_to_rest()
            ipc.write_edit_dsp_ack(1)
            return True, True
        return True, False

    if edit_mode:
        ipc.write_edit_dsp_ack(0)
        return False, False
    return False, False


def _handle_safe_state(ipc, group_states, last_safe):
    """If the safety FSM has transitioned into SAFE, snap groups back to
    rest and request a neutral target. Returns the new ``last_safe`` flag
    plus a ``snap_neutral`` request bit."""
    sys_state = ipc.read_system_state()
    in_safe   = (sys_state == 2)
    snap      = False
    if in_safe and not last_safe:
        for gst in group_states:
            gst.reset_to_rest()
        snap = True
    return in_safe, snap


def _measure_pkt_latency(rx_times, seq_history, t_drain_s, t_dsp_s, ipc,
                         lat_accum):
    """Compute pkt→servo / ring-dwell / dsp-compute / seq-age latencies and
    publish them. Also accumulates samples for the 5 s periodic waterfall.
    Returns the sequence-age estimate (used by the verbose print path)."""
    rx_list  = list(rx_times)
    seq_list = list(seq_history)
    if len(rx_list) < WINDOW_HI:
        return 0, 0.0
    oldest_rx = rx_list[-WINDOW_HI]
    if oldest_rx <= 0:
        return 0, 0.0

    t_servo_us       = int(time.monotonic() * 1e6)
    pkt_to_servo_us  = t_servo_us - oldest_rx
    ring_dwell_us    = int(t_drain_s * 1e6)
    oldest_seq       = seq_list[-WINDOW_HI] & 0xFF
    newest_seq       = seq_list[-1]         & 0xFF
    seq_age          = (newest_seq - oldest_seq) & 0xFF

    ipc.write_latency_pkt(
        pkt_to_servo_us = pkt_to_servo_us,
        ring_dwell_us   = ring_dwell_us,
        dsp_compute_us  = int(t_dsp_s * 1e6),
        seq_age         = seq_age,
    )
    lat_accum['pkt'].append(pkt_to_servo_us / 1000.0)
    lat_accum['seq'].append(seq_age)
    lat_accum['dsp'].append(t_dsp_s   * 1e3)
    lat_accum['drn'].append(t_drain_s * 1e3)
    return seq_age, pkt_to_servo_us / 1000.0


def _print_latency_waterfall(group_states, inferences, lat_accum):
    """Periodic 5 s ASCII waterfall: which stage dominates end-to-end
    latency. Constants are best-case datasheet values; measurements use
    actual ring drain, inference, and pkt→motor times for the past 5 s.

    Math (matches the C TUI's draw_page_dsp):
        BSAU stage   = LAT_ADC_PACK_US + LAT_WIRELESS_US        (const)
        CPCU stage   = lat_pkt_us (measured)                    (already
                       includes SPI_UNPACK + ring dwell + DSP compute +
                       motor-cmd IPC write, since rx_time_us is stamped
                       in cpcu_io BEFORE the NRF SPI read of the payload)
        SMOOTHER+I²C = LAT_SMOOTHER_I2C_US                      (const,
                       cpcu_io's separate 50 Hz PCA9685 update tick)
        SERVO        = LAT_SERVO_MECH_US                        (const)

    The breakdown rows below the CPCU header (ring dwell, DSP compute,
    SPI+unpack) are shown as informational sub-components of the
    measured pkt→motor wall-clock — they're NOT re-added to the total.
    """
    grp_summary = [(g.name, g.current_state, g.conf_pct)
                   for g in group_states]
    print(f"[DSP] ── latency waterfall ───────────────────",       flush=True)
    print(f"[DSP]   groups={grp_summary}  inf={inferences}",       flush=True)
    print(f"[DSP]   ┌─ BSAU ─────────────────────────────",        flush=True)
    print(f"[DSP]   │ ADC + pack:      {LAT_ADC_PACK_US:>6} µs  (const)",
          flush=True)
    print(f"[DSP]   │ wireless TX+ACK: {LAT_WIRELESS_US:>6} µs  (const)",
          flush=True)
    print(f"[DSP]   ├─ CPCU (pkt → motor IPC, all measured) ────", flush=True)
    print(f"[DSP]   │   incl. SPI+unpack ({LAT_SPI_UNPACK_US} µs const inside)",
          flush=True)
    if lat_accum['drn']:
        dravg = sum(lat_accum['drn']) / len(lat_accum['drn'])
        print(f"[DSP]   │   ring dwell:  {dravg*1000:>8.0f} µs  (meas, sub)",
              flush=True)
    if lat_accum['dsp']:
        davg = sum(lat_accum['dsp']) / len(lat_accum['dsp'])
        dmax = max(lat_accum['dsp'])
        print(f"[DSP]   │   DSP compute: {davg*1000:>8.0f} µs  "
              f"(meas, sub, max {dmax*1000:.0f})", flush=True)
    print(f"[DSP]   │ smoother+I²C:   {LAT_SMOOTHER_I2C_US:>6} µs  "
          f"(const, 50 Hz tick)", flush=True)
    print(f"[DSP]   ├─ SERVO ────────────────────────────",        flush=True)
    print(f"[DSP]   │ mechanical:    {LAT_SERVO_MECH_US:>8} µs  (const)",
          flush=True)
    print(f"[DSP]   └─ TOTALS ───────────────────────────",        flush=True)
    if lat_accum['pkt']:
        s          = sorted(lat_accum['pkt'])
        avg        = sum(s) / len(s)
        p95        = s[int(len(s) * 0.95)] if len(s) > 1 else s[0]
        cpcu_avg   = avg * 1000                                    # µs
        bsau_const = LAT_ADC_PACK_US + LAT_WIRELESS_US             # 558 µs
        e2e_total  = (bsau_const + cpcu_avg
                      + LAT_SMOOTHER_I2C_US + LAT_SERVO_MECH_US)
        print(f"[DSP]   │ BSAU pre-CPCU:{bsau_const:>8.0f} µs  (const)",
              flush=True)
        print(f"[DSP]   │ CPCU pkt→mot: {cpcu_avg:>8.0f} µs  "
              f"avg ({avg:.1f}ms)", flush=True)
        print(f"[DSP]   │               {p95*1000:>8.0f} µs  "
              f"p95 ({p95:.1f}ms)", flush=True)
        print(f"[DSP]   │ E2E (BSAU→servo):{e2e_total:>7.0f} µs  "
              f"({e2e_total/1000:.1f}ms)  budget < 300 ms", flush=True)
    if lat_accum['seq']:
        avg_seq = sum(lat_accum['seq']) / len(lat_accum['seq'])
        print(f"[DSP]   │ seq age:     {avg_seq:>8.0f} pkts "
              f"({avg_seq:.0f}ms @1kHz)", flush=True)
    print(f"[DSP] ──────────────────────────────────────",         flush=True)
    for k in lat_accum:
        lat_accum[k].clear()


# ══════════════════════════════════════════════════════════════════════
#  MAIN INFERENCE LOOP
# ══════════════════════════════════════════════════════════════════════

def run_inference(verbose=False, operator="default"):
    """Drain → window → infer → integrate → publish, forever.

    Top-level flow per ~20 ms tick:
      1. _drain_ring_into_groups           push BSAU packets into deques
      2. windowing check                   STRIDE_HI new samples + WINDOW_HI total
      3. per-group _features_and_inference filter + features + SVM + hysteresis
      4. _handle_edit_mode / _handle_safe_state
      5. _integrate_velocity               combine all groups → current_target
      6. publish motor_cmd + dsp_export    via IPC
      7. _measure_pkt_latency              update waterfall accumulators
      8. every 5 s: _print_latency_waterfall
    """
    # ── load config ──
    groups, servo_channels = load_gestures()
    grip_firm              = load_runtime()
    vel_overrides          = load_velocity_map(operator)

    # ── build per-group state + attach models ──
    group_states = [GroupState(g) for g in groups]
    for gst in group_states:
        gst.attach_model()

    _apply_velocity_overrides(group_states, vel_overrides, servo_channels)
    _resolve_grip_firm(group_states, grip_firm)

    # ── publish a pre-formatted digest for the TUI ──
    # The C TUI doesn't ship a JSON parser, so instead of teaching it
    # to read gestures.json directly we drop a plain-text digest here
    # that draw_page_config() can `fopen` and print line-by-line. The
    # digest is rewritten on every cpcu_dsp.py launch (and would also
    # be rewritten on SIGHUP if the kernel propagated SIGHUP to us).
    _write_gestures_digest(group_states, servo_channels)

    # ── IPC ──
    ipc = IPCBridge()
    ipc.set_dsp_ready()
    print(f"[DSP] ready. groups={len(group_states)} "
          f"operator={operator}", flush=True)
    _uart_init()

    # ── shared transient state ──
    rx_times       = deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
    seq_history    = deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
    samples_since_window = 0
    total_samples  = 0
    current_target = [SERVO_NEUTRAL] * NUM_SERVOS
    last_int_t     = time.monotonic()
    last_safe      = False
    edit_mode      = False
    inferences     = 0
    last_report    = time.monotonic()
    lat_accum      = {'pkt': [], 'seq': [], 'dsp': [], 'drn': []}

    # graceful shutdown on SIGINT / SIGTERM
    running = [True]
    def _stop(sig, _): running[0] = False
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT,  _stop)

    while running[0]:
        t0 = time.monotonic()

        # 1. drain
        added, t_drain_s = _drain_ring_into_groups(
            ipc, group_states, rx_times, seq_history)
        samples_since_window += added
        total_samples        += added

        # 2. window readiness
        if (samples_since_window >= STRIDE_HI
                and total_samples >= WINDOW_HI):
            samples_since_window = 0
            t_dsp_start          = time.monotonic()

            # 3. per-group features + inference + hysteresis
            rms_8ch = [0.0] * NUM_EMG_CH
            for gi, gst in enumerate(group_states):
                _features_and_inference(gst, rms_8ch, ipc)
                if gi == 0:
                    _publish_primary_state(ipc, gst)

            primary = group_states[0]
            _uart_send(primary.current_state,
                       primary.conf_pct / 100.0, [])

            # 4. publish DSP export
            t_dsp_s = time.monotonic() - t_dsp_start
            ipc.update_dsp_max_latency(int(t_dsp_s * 1e6))
            ipc.write_dsp_export(
                channel_rms       = rms_8ch,
                gesture_name      = primary.current_state,
                class_confidence  = primary.class_conf,
                active_class      = primary.last_active,
                inference_time_us = int(t_dsp_s * 1e6),
            )

            # 5. edit-mode + SAFE handling
            edit_mode, snap_edit = _handle_edit_mode(
                ipc, group_states, edit_mode)
            snap_safe            = False
            if edit_mode:
                # while editing, freeze the integrator and park the arm
                current_target = [SERVO_NEUTRAL] * NUM_SERVOS
                last_int_t     = time.monotonic()
            else:
                last_safe, snap_safe = _handle_safe_state(
                    ipc, group_states, last_safe)
                if snap_safe:
                    current_target = [SERVO_NEUTRAL] * NUM_SERVOS

                # 6. integrate
                now = time.monotonic()
                dt  = now - last_int_t
                last_int_t = now
                if dt > 0.5: dt = 0.0          # stall guard
                current_target = _integrate_velocity(
                    group_states, current_target, dt, grip_firm)

                servo_us = [int(round(v)) for v in current_target]
                ipc.write_motor_cmd(servo_us,
                                    primary.last_active,
                                    primary.conf_pct)

            # 7. latency measurement
            seq_age, pkt_ms = _measure_pkt_latency(
                rx_times, seq_history, t_drain_s, t_dsp_s, ipc, lat_accum)

            ipc.inc_dsp_inferences()
            inferences += 1

            if verbose:
                print(f"[DSP] {primary.current_state} "
                      f"{primary.conf_pct}%  pkt→servo={pkt_ms:.1f}ms  "
                      f"drain={t_drain_s*1e3:.1f}ms  "
                      f"dsp={t_dsp_s*1e3:.1f}ms  seq_age={seq_age}",
                      flush=True)

        # 8. periodic waterfall (5 s)
        if t0 - last_report >= 5.0:
            _print_latency_waterfall(group_states, inferences, lat_accum)
            last_report = t0

        # 9. sleep the remainder of the period
        elapsed = time.monotonic() - t0
        if elapsed < DRAIN_PERIOD_S:
            time.sleep(DRAIN_PERIOD_S - elapsed)

    print("[DSP] shutdown", flush=True)
    ipc.close()
    return 0


# ══════════════════════════════════════════════════════════════════════
#  CALIBRATION MODE
# ══════════════════════════════════════════════════════════════════════

def run_calibrate(seconds):
    """Record ``seconds`` of rest-state EMG, compute 3·std per channel,
    save to models/noise_thresholds.json. Used by ``./launch.sh calibrate
    --rest-only``."""
    groups, _ = load_gestures()
    active_channels = groups[0]["emg_channels"] if groups else [0, 1, 2]
    num_ch          = len(active_channels)
    labels          = [f"s{i+1}" for i in range(num_ch)]

    ipc = IPCBridge()
    print(f"[CAL] recording {seconds}s of rest...", flush=True)

    bufs  = [deque(maxlen=int(INPUT_FS_HZ * (seconds + 1)))
             for _ in range(num_ch)]
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        batch = ipc.pop_sensor_batch(DRAIN_BATCH)
        n     = batch.get('count', 0)
        if n > 0:
            samples = batch['samples']
            for ei in range(n):
                for si in range(2):
                    for bi, ch in enumerate(active_channels):
                        bufs[bi].append(int(samples[ei, si, ch]) - ADC_MIDRAIL)
        time.sleep(DRAIN_PERIOD_S)

    thresholds = {}
    for bi, label in enumerate(labels):
        sig_arr = np.array(bufs[bi], dtype=np.float64)
        if len(sig_arr) < WINDOW_HI:
            thresholds[label] = 50.0
            continue
        sig_lo   = decimate(sig_arr, DECIMATE_FACTOR, zero_phase=True)
        centered = sig_lo - np.mean(sig_lo)
        bp       = butter_bandpass(centered, 20.0, 450.0, TARGET_FS_HZ)
        cleaned  = notch_filter(bp, 50.0, TARGET_FS_HZ)
        thr      = float(np.std(cleaned) * 3.0)
        thresholds[label] = thr
        print(f"[CAL] {label} (ch{active_channels[bi]}): "
              f"std={np.std(cleaned):.2f} thr={thr:.2f}", flush=True)

    os.makedirs(MODEL_DIR, exist_ok=True)
    with open(THRESHOLDS_PATH, 'w') as f:
        json.dump(thresholds, f, indent=4)
    print(f"[CAL] saved: {THRESHOLDS_PATH}", flush=True)
    ipc.close()
    return 0


# ══════════════════════════════════════════════════════════════════════
#  ENTRY
# ══════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="CPCU DSP + ML pipeline")
    ap.add_argument('--calibrate', type=float, metavar='SEC',
                    help="Record N seconds of rest, save thresholds, exit")
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--operator', default='default',
                    help="Load velocity_map_<name>.json profile")
    args = ap.parse_args()

    if args.calibrate is not None:
        return run_calibrate(args.calibrate)
    return run_inference(verbose=args.verbose, operator=args.operator)


if __name__ == "__main__":
    sys.exit(main())
