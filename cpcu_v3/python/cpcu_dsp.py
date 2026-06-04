#!/usr/bin/env python3
"""cpcu_dsp.py — live DSP + ML inference pipeline for the CPCU.

ROLE
    Runs pinned to CPU cores 1-2 at SCHED_FIFO priority 80, spawned
    by cpcu_kernel. Drains the IPC sensor ring written by cpcu_io,
    applies a per-channel filter cascade (bandpass + notches +
    envelope), extracts 7 features per channel, runs ML inference for
    one or more gesture groups in parallel, and writes the integrated
    servo targets back into the IPC motor command region. cpcu_io
    then drives the PCA9685 from those targets at 50 Hz.

    The classifier loaded from models/arm.pkl is a sklearn
    RandomForestClassifier (200 trees) with 4 alphabetical classes
    (ext, flex, hand, rest) shared between the right_arm and left_arm
    groups — same muscles, same model.

DEPENDENCIES — what this script READS
    config/gestures.json     : Gesture groups (right_arm, left_arm),
                               EMG channel assignments, servo channel
                               map, per-group confidence + hysteresis.
                               Schema v5.
    models/arm.pkl           : Trained RandomForest + StandardScaler.
                               Loaded once at startup; same model
                               serves both arm groups.
    models/velocity_map_<op>.json : Per-operator velocity preferences
                               written by cpcu_calibrate.py. Falls
                               back to velocity_map.json.
    models/dynamic_noise_thresholds[_<op>].json : Per-muscle envelope
                               floor for the noise-gate. Aleyna's
                               canonical file is the default; an
                               operator-specific file overrides if
                               present. Never overwritten by the
                               --calibrate flow on this side.
    config/runtime.json      : (via IPC_RuntimeConfig from kernel)
                               servo limits, smoother params,
                               gravity-comp bias, grip thresholds.
    /dev/shm/cpcu_ipc        : Sensor ring, motor command, dsp_export,
                               dsp_filtered — through cpcu_ipc_bridge.
    /dev/ttyAMA0 (optional)  : 1 kHz raw-sample UART debug stream when
                               CPCU_UART_DEBUG points to a tty.
                               This is the ONLY UART writer in the
                               system (cpcu_io's was retired).

DOWNSTREAM — what reads what this script PRODUCES
    cpcu_io                  : Motor command (servo_us[6], gesture_id,
                               confidence_pct, latency_ack).
    cpcu_tui                 : dsp_export, dsp_filtered, plus three
                               /tmp digest files:
                                 /tmp/cpcu_servo_names.txt
                                 /tmp/cpcu_group_state.txt
                                 /tmp/cpcu_gestures_digest.txt
    cpcu_ws                  : same dsp_export + dsp_filtered; forwards
                               groups[] from cpcu_group_state.txt.
    monitor.py / predictX.py : raw 6-channel UART stream at 1 kHz when
                               CPCU_UART_DEBUG is set. Format matches
                               predictX.py's FS=1000 expectation.

CROSS-MODULE EFFECTS
    - Adding a new class to arm.pkl: also extend CLS_NAMES and bump
      DATASET_LABEL_COUNT in cpcu_tui.h, otherwise the overview
      class-bar loop displays "EXT/FLEX/HAND/REST" past the trained
      set.
    - Changing WINDOW_HI, STRIDE_HI, or filter cutoffs: the TUI's
      DSP-page and CONFIG-page reference card hardcode the same
      numbers; update those strings in cpcu_tui_render.c.
    - Changing the per-window window/stride math also changes the
      dsp_filtered publish rate which the web dashboard documents.
    - The model was trained against the filter chain in
      process_window(); modifying that chain INVALIDATES the model
      and requires retraining. predictX.py uses the same chain.

CONFIG SOURCE PRIORITY (highest first)
    1. config/gestures.json     — gesture_groups + servo_channels
    2. models/velocity_map.json — operator-calibrated rates
    3. config/runtime.json      — hardware tuning
    4. models/*.pkl             — trained model + scaler per group

CLI
    --calibrate SEC            Record N seconds of rest, write the
                               operator's dynamic_noise_thresholds_<op>.json
                               (Aleyna's canonical file untouched).
    --verbose                  Per-window inference log.
    --operator NAME            Load operator-specific profile
                               (velocity_map_<NAME>.json and
                               dynamic_noise_thresholds_<NAME>.json
                               when present). Defaults to
                               $CPCU_OPERATOR or "default".
"""
import argparse
import glob
import json
import os
from collections import deque
from concurrent.futures import ThreadPoolExecutor
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

# Per-muscle envelope-floor thresholds produced by the AI team's
# proccess.py at training time. Same muscle order on BOTH arms (each
# arm has 4 channels: Hand, Biceps, Triceps, Shoulder), so a single
# 4-element threshold vector serves both groups:
#   s1 → Hand    (ch0 right / ch4 left)
#   s2 → Biceps  (ch1 right / ch5 left)
#   s3 → Triceps (ch2 right / ch6 left)
#   s4 → Shoulder/Trapezius (ch3 right / ch7 left)
#
# If a window's MEAN envelope on every channel of a group is BELOW
# its corresponding threshold, the muscle is not contracting and we
# short-circuit the classifier to "rest" — no ML call, no jitter.
# This is exactly what the training pipeline does to label rest data;
# applying it at runtime keeps train/test conditions aligned.
#
# OPERATOR-SPECIFIC OVERRIDE:
#   The canonical file dynamic_noise_thresholds.json is what Aleyna's
#   training pipeline produced and is the default for every operator.
#   Recording new rest data with `./launch.sh calibrate --operator
#   <name>` writes dynamic_noise_thresholds_<name>.json without
#   touching the canonical file. At runtime the operator's file wins
#   if present, otherwise we fall back to Aleyna's. This is per
#   operator's request — never overwrite the training-time defaults.
DYNAMIC_THR_DEFAULT_PATH = os.path.join(MODEL_DIR, "dynamic_noise_thresholds.json")

def _dynamic_thr_path(operator):
    """Path to the rest-noise threshold file for the given operator.

    'default' → Aleyna's canonical file. Any other name → operator
    -specific file, even if it doesn't yet exist (caller falls back)."""
    if not operator or operator == "default":
        return DYNAMIC_THR_DEFAULT_PATH
    return os.path.join(MODEL_DIR, f"dynamic_noise_thresholds_{operator}.json")

_dyn_thresh = None              # cached threshold list (variable length)
_dyn_thresh_op = None           # which operator the cache is for

# Muscle-name labels for the print line, in s1..sN order. Truncated
# to match the actual loaded threshold count so the log mirrors the
# data exactly. Same order as the per-arm emg_channels block.
_DYN_THR_KEYS  = ("s1", "s2", "s3", "s4")
_DYN_THR_NAMES = ("hand", "biceps", "triceps", "shoulder")


def _load_dynamic_thresholds(operator="default"):
    """Read s1/s2/s3/s4 → [hand, biceps, triceps, shoulder] thresholds.

    Length-flexible: returns however many of s1..s4 are actually
    present in the file (in order, stopping at the first missing
    key). The caller (`_features_and_inference`) length-checks
    against the group's channel count, so a 3-key file disables the
    gate for 4-channel groups but still works for legacy 3-channel
    setups.

    Lookup order (first hit wins):
      1. dynamic_noise_thresholds_<operator>.json  (operator-specific)
      2. dynamic_noise_thresholds.json             (Aleyna's canonical)

    Returns the threshold list, or None when neither file exists or
    is malformed (DSP then falls through to the plain classifier,
    same as before). Cached per-operator so repeated calls are free."""
    global _dyn_thresh, _dyn_thresh_op
    if _dyn_thresh is not None and _dyn_thresh_op == operator:
        return _dyn_thresh

    candidates = []
    if operator and operator != "default":
        candidates.append(_dynamic_thr_path(operator))
    candidates.append(DYNAMIC_THR_DEFAULT_PATH)

    for path in candidates:
        try:
            with open(path) as f:
                d = json.load(f)
            thr = []
            for key in _DYN_THR_KEYS:
                if key not in d:
                    break          # stop at first missing key, preserve order
                thr.append(float(d[key]))
            _dyn_thresh    = thr
            _dyn_thresh_op = operator
            tag = "operator" if path != DYNAMIC_THR_DEFAULT_PATH else "default(Aleyna)"
            summary = "  ".join(f"{n}={v:.1f}"
                                for n, v in zip(_DYN_THR_NAMES, thr))
            print(f"[DSP] loaded noise-floor gates [{tag}] {summary}  "
                  f"({os.path.basename(path)}, n={len(thr)})",
                  flush=True)
            return _dyn_thresh
        except FileNotFoundError:
            continue
        except Exception as e:
            print(f"[DSP] noise-floor file {path}: {e}", flush=True)
            continue

    print("[DSP] no dynamic_noise_thresholds*.json found "
          "(falls through to plain classifier)", flush=True)
    _dyn_thresh    = []         # mark "tried, none available"
    _dyn_thresh_op = operator
    return _dyn_thresh

# Sampling rates / window sizing
INPUT_FS_HZ     = 1000                              # BSAU 1 kHz packet rate
TARGET_FS_HZ    = 200                               # after decimation
DECIMATE_FACTOR = INPUT_FS_HZ // TARGET_FS_HZ       # 5
WINDOW_MS       = 200
# STRIDE_MS controls the inference rate: a new window emerges every
# STRIDE_MS and triggers one inference per gesture_group (2 groups =
# 2 sequential ML predictions). Each prediction takes ~50-60 ms on
# Cortex-A76, so a stride < (groups × pred_ms) backs up the IPC ring.
# 200 ms gives ≥80 ms headroom for two-group inference; reduces
# command rate to 5 Hz, well above human reaction time (~250 ms).
# STRIDE_MS controls the inference rate. With parallel two-group
# inference (see ThreadPoolExecutor below) the per-stride cost is the
# MAX of group inference times, not the sum — so 100 ms gives a 40 ms
# safety margin on top of a 50–60 ms sklearn predict_proba. If the
# operator sets CPCU_DSP_SERIAL=1 the inference becomes sequential
# (~114 ms) and STRIDE_MS=100 will back the ring up — bump to 200 in
# that case (this constant is read once at startup; restart the DSP
# to change it).
STRIDE_MS       = 100
WINDOW_HI       = INPUT_FS_HZ  * WINDOW_MS // 1000  # 200 samples @ 1 kHz
STRIDE_HI       = INPUT_FS_HZ  * STRIDE_MS // 1000  # 200 samples @ 1 kHz
WINDOW_LO       = TARGET_FS_HZ * WINDOW_MS // 1000  # 40  samples @ 200 Hz
BUFFER_SIZE     = WINDOW_HI * 8                     # 1600-sample ring (more slack)

# Hardware constants (matched to BSAU + cpcu_io)
ADC_MIDRAIL     = 2048
# Spike-filter constants — mirrored from predictX.py. ADC samples
# outside this range or differing from the previous valid sample by
# more than ADC_MAX_DELTA are treated as wireless-link garbage and
# replaced with the last good value. Keeps the feature vector clean
# so the trained model sees the same kind of signal it learned on.
ADC_MIN_VALID   = 10
ADC_MAX_VALID   = 4095
ADC_MAX_DELTA   = 1300
NUM_SERVOS      = 6
NUM_EMG_CH      = 8
SERVO_NEUTRAL   = 1500
DRAIN_PERIOD_S  = 0.010                             # main loop period (was 0.020)
DRAIN_BATCH     = 512                               # max packets per drain (was 200)
"""
PROB_THRESH gates which model predictions are even ALLOWED to count
toward the hysteresis vote. A value of 0.65 was too high for the
current 4-class model: predictions routinely hover at 30-50%
confidence (especially when the operator's electrodes aren't on
the exact training-time positions), and every below-threshold vote
was silently dropped — making the system feel unresponsive even
though the model was producing reasonable outputs.

Default 0.40 means: as long as the winning class is at least 1.6×
random (25% for 4 classes), the vote counts. Override via env:
    CPCU_PROB_THRESH=0.30 ./launch.sh tui
"""
PROB_THRESH     = float(os.environ.get("CPCU_PROB_THRESH", "0.40"))

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
                "rest":  {"mode": "freeze",   "channels": {}},
                "hand":  {"mode": "velocity", "channels": {}},
                "flex":  {"mode": "velocity", "channels": {}},
                "ext":   {"mode": "velocity", "channels": {}},
                "wrist": {"mode": "velocity", "channels": {}},
            },
            "emg_channels": [0, 1, 2, 3],
            "confidence":   default_conf,
            "hysteresis":   default_hyst,
            "model_path":   "models/arm.pkl",
        },
        {
            "name":         "left_arm",
            "gestures": {
                "rest":  {"mode": "freeze",   "channels": {}},
                "hand":  {"mode": "velocity", "channels": {}},
                "flex":  {"mode": "velocity", "channels": {}},
                "ext":   {"mode": "velocity", "channels": {}},
                "wrist": {"mode": "velocity", "channels": {}},
            },
            "emg_channels": [4, 5, 6, 7],
            "confidence":   default_conf,
            "hysteresis":   default_hyst,
            "model_path":   "models/arm.pkl",
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
    # Build the logical-slot mapping. Internally we use 0..NUM_SERVOS-1
    # as the index into all 6-element arrays (rates, motor_cmd, etc).
    # The slot order is the gestures.json servo_channels declaration
    # order, sorted by pca_ch ascending — so the slot index is stable
    # regardless of what physical PCA9685 channel each servo is wired
    # to. Physical channel routing happens entirely in cpcu_io.c via
    # IPC_RuntimeConfig::servo_pca_ch, which launch.sh keeps in sync.
    sorted_servos = sorted(servo_ch.items(),
                           key=lambda x: x[1].get("pca_ch", 0))
    name_to_idx = {n: slot for slot, (n, _d) in enumerate(sorted_servos)}
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

    # Cross-check trace: for every group/gesture, print which servo
    # slots the rates[] resolves to. This catches name typos in
    # gestures.json (e.g. "Elbo" → silently dropped) and confirms the
    # slot ordering matches expectations after a pca_ch re-wire.
    slot_to_name = {slot: name for name, slot in name_to_idx.items()}
    print("[DSP] gesture → servo trace:", flush=True)
    for g in groups:
        print(f"[DSP]   [{g['name']}]", flush=True)
        for gname, gdef in g['gestures'].items():
            rates = gdef.get('_rates', [0] * NUM_SERVOS)
            non_zero = [(slot, rates[slot]) for slot in range(NUM_SERVOS) if rates[slot] != 0]
            if non_zero:
                desc = ", ".join(f"{slot_to_name.get(s, '?')}(slot {s}) {'+' if r > 0 else ''}{r}"
                                 for s, r in non_zero)
                print(f"[DSP]     {gname:<10} → {desc}", flush=True)
            else:
                mode = gdef.get('mode', '?')
                print(f"[DSP]     {gname:<10} → ({mode})", flush=True)

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
    inference loop doesn't need to know which schema was on disk.

    ``class_remap`` translates model output class names → gesture names
    in this group's ``gestures`` block. Use case: a model trained with
    label ``trap`` (trapezius muscle) drives a gesture called ``wrist``
    (the robot action that muscle triggers). The model's class set is
    {ext, flex, hand, rest, trap}; gestures.json has {rest, hand, flex,
    ext, wrist}. The default ``{"trap": "wrist"}`` makes that work
    without renaming either side. Configurable per group so the two
    arms can map differently if one day they use different models."""
    DEFAULT_CLASS_REMAP = {"trap": "wrist"}
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
                "class_remap":  gdef.get("class_remap", DEFAULT_CLASS_REMAP),
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
            "class_remap":  gs.get("class_remap", DEFAULT_CLASS_REMAP),
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
SERVO_NAMES_PATH     = "/tmp/cpcu_servo_names.txt"
EMG_NAMES_PATH       = "/tmp/cpcu_emg_names.txt"
GROUP_STATE_PATH     = "/tmp/cpcu_group_state.txt"


def _write_servo_names(servo_channels):
    """Publish servo names (sorted by pca_ch, one per line) for the C
    TUI to read instead of using compile-time SERVO_NAMES[]."""
    try:
        sc = sorted(servo_channels.items(),
                    key=lambda x: x[1].get("pca_ch", 0))
        with open(SERVO_NAMES_PATH, "w") as f:
            for name, _sd in sc[:NUM_SERVOS]:
                f.write(name + "\n")
    except Exception:
        pass


def _write_emg_names(path=GESTURES_PATH):
    """Publish per-channel muscle names (one per line, channel-index
    order ch0..ch7) for the C TUI to read instead of falling back to
    the compile-time defaults baked into g_emg_name_buf in
    cpcu_tui_render.c.

    Reads gestures.json directly (cheap — runs once at DSP startup
    and on `./launch.sh reload`) so we don't have to plumb the name
    strings through load_gestures()'s parsed return shape. Walks every
    gesture group's emg_channels.{active, names} arrays and indexes
    into a flat NUM_EMG_CH-element output list by absolute channel
    number. Channels not claimed by any group keep a "chN" placeholder
    so the file always has exactly NUM_EMG_CH lines (the TUI reader
    stops at NUM_EMG_CH anyway, but the placeholder makes the layout
    obvious when the file is inspected by hand)."""
    names = [f"ch{i}" for i in range(NUM_EMG_CH)]
    try:
        with open(path) as f:
            gs = json.load(f)
        groups = (gs.get("gesture_groups") or {}).values()
        for gdef in groups:
            ec     = gdef.get("emg_channels", {})
            active = ec.get("active", [])
            ns     = ec.get("names",  [])
            for slot, ch in enumerate(active):
                if 0 <= ch < NUM_EMG_CH and slot < len(ns) and ns[slot]:
                    names[ch] = ns[slot]
        with open(EMG_NAMES_PATH, "w") as f:
            for n in names:
                f.write(n + "\n")
    except Exception:
        pass


def write_group_state_digest(group_states):
    """Publish one line per group with current state + top-class confs
    for the TUI's DSP/AI page. Called every inference tick (cheap —
    writes <500 bytes to /tmp on a tmpfs).

    Format per line:
        <group_name>\\t<current_state>\\t<conf_pct>\\t<cls0>:<p0>,<cls1>:<p1>,...
    """
    try:
        lines = []
        for gst in group_states:
            classes = ([str(c) for c in gst.model.classes_]
                       if gst.model is not None else [])
            confs   = gst.class_conf or []
            pairs   = ",".join(
                f"{classes[i] if i < len(classes) else f'c{i}'}:{int(confs[i]*100)}"
                for i in range(len(confs)))
            lines.append(f"{gst.name}\t{gst.current_state}\t"
                         f"{gst.conf_pct}\t{pairs}")
        with open(GROUP_STATE_PATH, "w") as f:
            f.write("\n".join(lines) + "\n")
    except Exception:
        pass


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
    """Map classifier probability into a velocity scale [0, 1].

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
#  UART STREAMING — for the AI team's host-side monitor.py
# ══════════════════════════════════════════════════════════════════════
#
# Two payload formats are multiplexed on the same /dev/ttyAMA0 link at
# 921600 baud, distinguished by a single leading character:
#
#   1. RAW-SAMPLE LINE  (no prefix, just CSV ints, 8 fields)
#         "<ch0>,<ch1>,<ch2>,<ch3>,<ch4>,<ch5>,<ch6>,<ch7>\n"
#      Emitted at 1 kHz — one line per BSAU sample-pair entry, taking
#      the first of the two oversamples in each packet. Matches the
#      predictX.py / monitor.py expectation (FS=1000).
#
#   2. PREDICTION LINE  (prefix '#', for sanity / debug overlay)
#         "#pred,<ts_ms>,<group>,<gesture>,<conf>\n"
#      Emitted at the inference rate (10 Hz). monitor.py's parser
#      treats any line starting with '#' as metadata (skips CSV parse).
#
# Budget: 8 channels × ~5 chars × 1000 lines/s ≈ 40 kB/s, comfortably
# under 921600 baud's 92 kB/s usable bandwidth (~43% utilisation).
#
# BACKPRESSURE — CRITICAL
#   The host monitor.py briefly stops reading every 100 ms to run its
#   own inference. If the Pi-side serial write blocks during that gap,
#   it blocks the ENTIRE DSP main loop (drain stops, IPC ring backs up,
#   next drain emits a catch-up burst → host sees "stops then continues"
#   pattern). We avoid that with two layers of protection:
#     (1) Serial(write_timeout=0.010)  — write attempts time out after
#         10 ms (longer than one drain period, shorter than two), so a
#         genuinely-stalled host never freezes inference.
#     (2) `out_waiting` check          — if more than UART_BACKLOG_BYTES
#         are already queued in the OS buffer, drop this whole batch
#         silently. Better to lose one 10-line burst than to pile up.
#   Both layers also protect against the rare case of an OS scheduler
#   hiccup that pauses the tty kernel thread briefly.

UART_PORT           = os.environ.get("CPCU_UART_DEBUG", "")
UART_BAUD           = int(os.environ.get("CPCU_UART_BAUD", "921600"))
UART_WRITE_TIMEOUT  = 0.010   # 10 ms — see backpressure docblock above
UART_BACKLOG_BYTES  = 8192    # ~200 ms of UART data; drop new batches above this
_uart               = None


def _uart_init():
    """Open the debug serial port if CPCU_UART_DEBUG was set by launch.sh.

    Uses a short write_timeout (10 ms — longer than one DRAIN_PERIOD_S
    so well-behaved hosts never hit it, shorter than two so a genuinely
    stalled host doesn't freeze the inference loop). See backpressure
    docblock above."""
    global _uart
    if not UART_PORT:
        return
    try:
        import serial
        _uart = serial.Serial(UART_PORT, UART_BAUD,
                              timeout=0,
                              write_timeout=UART_WRITE_TIMEOUT)
        print(f"[DSP] UART stream → {UART_PORT} @ {UART_BAUD} baud "
              f"(write_timeout={UART_WRITE_TIMEOUT*1000:.0f}ms, "
              f"backlog_drop={UART_BACKLOG_BYTES}B)", flush=True)
    except Exception as e:
        print(f"[DSP] UART: {e}", flush=True)


_uart_last_good = [2048] * NUM_EMG_CH   # persistent baseline used by UART filter
_uart_drops     = {"samples": 0, "predictions": 0}   # diagnostics


def _uart_stream_samples(samples_batch):
    """Stream raw ADC samples line-by-line in predictX-compatible CSV.

    ``samples_batch`` is the 3-D BSAU array shape (n_entries, 2, NUM_EMG_CH)
    that cpcu_io drained from the ring. BSAU packs two 2 kHz samples
    into each 1 kHz packet. We emit ONLY the FIRST sample per packet,
    giving a clean 1 kHz UART stream — matches predictX.py /
    monitor.py's FS=1000 expectation and matches the legacy "BSAU UART
    at 1 kHz" pipeline the team has always used. Emits ALL NUM_EMG_CH
    (8) channels in order: ch0..ch7 = R_Hand, R_Biceps, R_Triceps,
    R_Shoulder, L_Hand, L_Biceps, L_Triceps, L_Shoulder.

    BACKPRESSURE: drops the entire batch silently if the host can't
    keep up (out_waiting > UART_BACKLOG_BYTES). Better to lose one
    10-line burst than block the DSP main loop. See the module-level
    backpressure docblock for the full rationale.

    Applies the SAME spike filter as `ingest_sample()` so the host
    monitor.py doesn't see all-zero packets from wireless reacquisition
    glitches. Without this filter the host display showed sudden
    drops to 0 even though Pi-side inference was already substituting
    the same values away."""
    if _uart is None or samples_batch is None:
        return

    # Drop early if the OS buffer is already pressured — never let the
    # write() below block the main loop.
    try:
        if _uart.out_waiting > UART_BACKLOG_BYTES:
            _uart_drops["samples"] += int(samples_batch.shape[0])
            return
    except Exception:
        pass

    try:
        n_e = samples_batch.shape[0]
        buf = []
        # si=0 only: one sample per packet → 1 kHz output rate.
        for ei in range(n_e):
            row = samples_batch[ei, 0]
            filtered = [0] * NUM_EMG_CH
            for ch in range(NUM_EMG_CH):
                raw = int(row[ch])
                lv  = _uart_last_good[ch]
                if raw < ADC_MIN_VALID or raw > ADC_MAX_VALID \
                   or abs(raw - lv) > ADC_MAX_DELTA:
                    raw = lv
                _uart_last_good[ch] = raw
                filtered[ch] = raw
            buf.append(",".join(str(v) for v in filtered))
        if buf:
            _uart.write(("\n".join(buf) + "\n").encode())
    except Exception:
        # SerialTimeoutException (host stalled longer than write_timeout)
        # or any OSError on a transient USB-UART hiccup — count and move
        # on. Never let UART errors stop inference.
        _uart_drops["samples"] += int(samples_batch.shape[0])


def _uart_send_prediction(group_name, gesture, conf, classes=None, probs=None):
    """Emit a prediction summary for monitor.py's overlay, equivalent
    to what predictX.py prints. Format:

        #pred,<ts_ms>,<group>,<top_gesture>,<top_conf>,cls0:p0,cls1:p1,...

    The leading ``#`` makes monitor.py's CSV parser skip it (it's not
    8 ints), so the host can either:
      * ignore prediction lines and re-run inference locally
      * parse them as a debug overlay showing what the Pi predicted
    `classes` is the list of class names (model.classes_) and `probs`
    is the corresponding softmax vector. Both optional; if absent we
    fall back to the old two-field format.

    Same backpressure protection as _uart_stream_samples — drops the
    prediction if the host's serial buffer is full instead of blocking."""
    if _uart is None:
        return
    try:
        if _uart.out_waiting > UART_BACKLOG_BYTES:
            _uart_drops["predictions"] += 1
            return
    except Exception:
        pass
    try:
        ts = int(time.time() * 1000)
        line = f"#pred,{ts},{group_name},{gesture},{conf:.3f}"
        if classes is not None and probs is not None:
            tail = ",".join(f"{c}:{float(p):.3f}"
                            for c, p in zip(classes, probs))
            line = f"{line},{tail}"
        _uart.write((line + "\n").encode())
    except Exception:
        _uart_drops["predictions"] += 1


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

    def __init__(self, group_def, prob_thresh=None, operator="default"):
        self.name         = group_def["name"]
        self.gestures     = group_def["gestures"]
        self.emg_channels = group_def["emg_channels"]
        self.model_path   = group_def.get("model_path", "")
        # Each group reads the same operator-scoped noise threshold file.
        # Stored on the group so _features_and_inference can call
        # _load_dynamic_thresholds(self.operator) without taking a
        # second argument.
        self.operator     = operator
        # Model output class → gestures.json key remap. Empty dict
        # means "use raw class names". Default (set by _build_group_list)
        # maps the trapezius/shoulder model class to the wrist gesture
        # because the model was trained with muscle-name labels but
        # gestures.json uses robot-action names.
        self.class_remap  = dict(group_def.get("class_remap", {}))

        # Resolve prob_thresh precedence:
        #   1. Explicit arg (used by tests)
        #   2. CPCU_PROB_THRESH env (live override without editing files)
        #   3. gestures.json::gesture_groups.<group>.confidence.floor_pct
        #   4. Module-level PROB_THRESH default
        # gestures.json's existing `confidence.floor_pct` (default 40)
        # IS the vote-gate — it was being read for the confidence
        # curve but not for the hysteresis prob_thresh. Wire them
        # together so editing floor_pct actually changes responsiveness.
        cc_floor_pct = group_def.get("confidence", {}).get("floor_pct")
        env_override = os.environ.get("CPCU_PROB_THRESH")
        if prob_thresh is not None:
            self.prob_thresh = prob_thresh
        elif env_override:
            try:    self.prob_thresh = float(env_override)
            except: self.prob_thresh = PROB_THRESH
        elif cc_floor_pct is not None:
            self.prob_thresh = float(cc_floor_pct) / 100.0
        else:
            self.prob_thresh = PROB_THRESH

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
        # last accepted raw sample per channel — used by ingest_sample's
        # spike validator to substitute for out-of-range / Δ>MAX values
        self._last_raw    = [ADC_MIDRAIL] * len(self.emg_channels)

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
                # A class is "OK" if it's directly in gestures OR the
                # remap routes it to a gesture that is. Without this
                # remap check the model's "trap" class always
                # produces a confusing warning for setups that map
                # trap → wrist via class_remap.
                gesture_key = self.class_remap.get(cls, cls)
                if gesture_key not in self.gestures:
                    print(f"[DSP] {self.name}: model class '{cls}' "
                          f"(→ '{gesture_key}') not in gestures — "
                          f"will fall back to neutral", flush=True)
            if self.class_remap:
                print(f"[DSP] {self.name}: class_remap={self.class_remap}",
                      flush=True)
            print(f"[DSP] {self.name}: classes={classes}", flush=True)

    def ingest_sample(self, ei, si, samples):
        """Append one BSAU sample-pair into the per-channel deques.
        Mirrors predictX.py's validate_sample: drops impossibly fast
        sample-to-sample deltas (wireless dropouts that decode to
        garbage ADC values otherwise pollute the feature vector and
        bias the classifier toward whichever class was most common in
        training). The threshold MAX_DELTA=1300 matches what the
        operator used to train the model — keep them in lockstep."""
        for bi, ch in enumerate(self.emg_channels):
            raw = int(samples[ei, si, ch])
            # 12-bit ADC; reject out-of-range (0 = no signal, 4095 = clip)
            if raw < ADC_MIN_VALID or raw > ADC_MAX_VALID:
                raw = self._last_raw[bi]
            elif abs(raw - self._last_raw[bi]) > ADC_MAX_DELTA:
                raw = self._last_raw[bi]
            self._last_raw[bi] = raw
            self.buffers[bi].append(raw - ADC_MIDRAIL)

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
    # name → slot index (sorted by pca_ch ascending) — same scheme as
    # load_gestures. Without this fix the override used the raw pca_ch
    # value as an array index, which crashes / writes the wrong slot
    # whenever the operator wires servos to non-default channels.
    sorted_servos = sorted(servo_channels.items(),
                           key=lambda x: x[1].get("pca_ch", 0))
    name_to_idx = {n: slot for slot, (n, _d) in enumerate(sorted_servos)}
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

    # Mirror the raw 2 kHz sample stream to the AI team's UART
    # so monitor.py on the host PC sees the exact same data the
    # filter+ML pipeline does. No-op when CPCU_UART_DEBUG is unset.
    _uart_stream_samples(samples[:n])

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
    feature extract, then RandomForest inference + hysteresis state-machine update.

    Sets ``gst.conf_pct`` / ``gst.class_conf`` / ``gst.last_active``."""
    features_flat = []
    env_means     = []        # one per channel — used by the noise-floor gate
    for bi, ch in enumerate(gst.emg_channels):
        w  = np.array(list(gst.buffers[bi])[-WINDOW_HI:], dtype=np.float64)
        _cl, ev, feats = process_window(w)
        features_flat.extend(feats)
        env_means.append(float(np.mean(np.abs(ev))))
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

    # Noise-floor gate: when EVERY channel's mean envelope is below the
    # AI team's training-time threshold, we KNOW the muscle isn't
    # contracting — running the classifier here would just produce a
    # noisy guess and pollute the hysteresis history. Short-circuit
    # straight to "rest". Disabled if thresholds aren't loaded yet
    # (dynamic_noise_thresholds.json missing), in which case behaviour
    # is unchanged from before.
    thr = _load_dynamic_thresholds(gst.operator)
    if thr and len(env_means) == len(thr):
        if all(em < t for em, t in zip(env_means, thr)):
            # Force rest. Synthesise a "100% rest" probability vector
            # for the TUI / UART overlay so the operator sees what's
            # happening, but skip the actual model call.
            classes = (list(gst.model.classes_)
                       if gst.model is not None else ["rest"])
            try:
                rest_idx = classes.index("rest")
            except ValueError:
                rest_idx = 0
            probs = [0.0] * len(classes)
            probs[rest_idx] = 1.0
            _update_hysteresis(gst, "rest", 1.0)
            gst.last_active = rest_idx
            gst.class_conf  = probs
            gst.conf_pct    = 100
            return

    X     = np.asarray(features_flat, dtype=np.float64).reshape(1, -1)
    Xs    = gst.scaler.transform(X)
    probs = gst.model.predict_proba(Xs)[0]
    ai    = int(np.argmax(probs))
    raw_label = str(gst.model.classes_[ai])
    # class_remap: model's training labels may not match gesture keys
    # (e.g. model emits "trap" for trapezius/shoulder activation but
    # gestures.json calls that gesture "wrist"). Remap before the
    # hysteresis state machine so current_state always holds a
    # gesture-key value that _integrate_velocity can look up.
    label = gst.class_remap.get(raw_label, raw_label)
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
      a→r  active → rest                  (handled by the rest fast-path
                                           below; effectively immediate)
      a→a  active → another active gesture

    REST FAST-PATH (predict_ch4.py-style):
      When the classifier's argmax is "rest" the new state is committed
      IMMEDIATELY, regardless of confidence and regardless of the
      hyst_a2r vote count. Rationale: rest is the SAFE state — a false
      "rest" merely halts motion, while a false NOT-rest keeps the arm
      moving past the operator's intent. The AI team's reference
      predict_ch4.py does this unconditionally and the user reports that
      script returning to rest cleanly; without this fast-path, our
      conf_floor (default 25%) was rejecting low-confidence rest votes
      and leaving current_state stuck on the prior active gesture even
      after the operator relaxed.

    Counter-reset semantics for the non-rest path:
      * label == current_state → counter at 0; we're already there.
      * conf  <= prob_thresh   → REJECT this vote but DO NOT zero the
        counter. The old code reset on every below-threshold sample,
        which meant a noisy stretch of [55%, 38%, 60%, 42%, 70%]
        predictions never accumulated to a transition even though the
        intent was clearly there. With this fix the 38% and 42% reads
        are simply skipped and the 55%, 60%, 70% reads count toward
        ``needed``.
    """
    # Already in this state — nothing to vote for.
    if label == gst.current_state:
        gst.consec_count = 0
        return

    # ── Rest fast-path: predict_ch4.py-style unconditional commit ────
    # Any rest argmax flips the state to "rest" and clears the vote
    # counter. Confidence is intentionally NOT checked here; see the
    # rationale block above. Note hyst_a2r is therefore essentially
    # dead code — it survives in the JSON schema for backward compat
    # but no longer gates the transition.
    if label == "rest":
        gst.current_state = "rest"
        gst.consec_count  = 0
        return

    # ── Non-rest transitions: confidence-gated + vote-counted ────────
    # Low-confidence prediction: skip without punishing prior good votes.
    if conf <= gst.prob_thresh:
        return

    if gst.current_state == "rest":
        needed = gst.hyst_r2a
    else:
        needed = gst.hyst_a2a    # active → another active
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
      * ALL groups at rest  → HOLD the last commanded position (no
        integration, no snap). This mirrors a real prosthetic: when
        the operator stops contracting, the arm stays where it is.
        Use SAFE state (radio lost, vbat critical, etc.) to force a
        neutral snap — see _handle_safe_state.
      * gesture mode "freeze" → that group contributes nothing
      * gripper (servo index 5) — clamp from below by the gesture's
        ``_grip_firm`` value so it never goes slack mid-grasp.
      * Gripper with ``snap=true`` — bypass rate integration entirely
        and pin the target to ``grip_firm`` while this gesture is
        active. The downstream cpcu_smooth smoother still applies the
        runtime velocity/accel caps, so the actual mechanical step
        per PWM tick stays bounded — this is "snap to target at
        smoother-max speed", not a true PCA9685 teleport. To get a
        true teleport, IO would need a per-servo snap IPC bit; the
        smoother-bounded version below is enough for the firm-grip
        use case and keeps the existing safety guarantees intact.
        ``snap=true`` on non-Gripper servos is silently ignored —
        only Gripper has a well-defined "snap target" (grip_firm).
    """
    if all(gst.current_state == "rest" for gst in group_states):
        return current_target          # hold pose, don't snap

    for gst in group_states:
        gdef = gst.gestures.get(gst.current_state)
        if gdef is None or gdef.get("mode") == "freeze":
            continue

        scale     = confidence_scale(gst.conf_pct / 100.0,
                                     gst.conf_floor, gst.conf_ceil,
                                     gst.conf_curve)
        rates     = gdef.get("_rates",     [0]*NUM_SERVOS)
        snaps     = gdef.get("_snap",      [False]*NUM_SERVOS)
        grip_firm = gdef.get("_grip_firm", grip_firm_default)

        for s in range(NUM_SERVOS):
            # Gripper snap path: jump target to grip_firm immediately.
            # Smoother in cpcu_io will still ramp at v_max — no
            # PCA-level teleport. Skip the rate integration entirely
            # so a coexisting non-zero rate doesn't fight the snap.
            if snaps[s] and s == 5:
                current_target[s] = float(grip_firm)
                continue
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
    # UART backpressure: how many sample-pair entries / predictions we
    # had to drop because the host wasn't keeping up. Steady-state
    # should be 0; non-zero values indicate the host monitor.py is
    # falling behind (CPU overloaded, ttyUSB renegotiating, etc.).
    if _uart is not None:
        ds  = _uart_drops["samples"]
        dp  = _uart_drops["predictions"]
        if ds or dp:
            print(f"[DSP]   │ UART drops:   {ds:>8d} samples, "
                  f"{dp} predictions (host backpressure)", flush=True)
            _uart_drops["samples"]     = 0
            _uart_drops["predictions"] = 0
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
      3. per-group _features_and_inference filter + features + RF + hysteresis
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
    group_states = [GroupState(g, operator=operator) for g in groups]
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
    _write_servo_names(servo_channels)
    _write_emg_names()

    # ── IPC ──
    ipc = IPCBridge()
    ipc.set_dsp_ready()
    # Drop an initial group-state digest BEFORE any windows fill, so the
    # TUI's DSP/AI page can show the list of groups (each in 'rest' at
    # 0%) the moment DSP is ready, instead of "(no inference yet)". The
    # real inference loop overwrites this on the first window.
    write_group_state_digest(group_states)
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

    # Parallel inference executor. Each gesture group is fully
    # independent — disjoint EMG channels, separate model/scaler,
    # separate hysteresis state — so two-group inference can run on
    # two threads. sklearn's predict_proba releases the GIL during the
    # heavy numpy work, so threading actually helps (not just an
    # illusion of speedup). One worker per group, executor lives for
    # the lifetime of the process to avoid per-iteration thread
    # spawn cost. Set CPCU_DSP_SERIAL=1 to disable and fall back to
    # sequential — useful when debugging or on single-core hardware.
    use_parallel = (len(group_states) > 1
                    and os.environ.get("CPCU_DSP_SERIAL", "0") != "1")
    pool = (ThreadPoolExecutor(max_workers=len(group_states),
                                thread_name_prefix="dsp_grp")
            if use_parallel else None)
    if pool:
        print(f"[DSP] parallel inference: {len(group_states)} workers",
              flush=True)
    else:
        print("[DSP] sequential inference (single group or CPCU_DSP_SERIAL=1)",
              flush=True)

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

            # 3. per-group features + inference + hysteresis. Runs in
            # parallel when the executor is available; sequential
            # fallback when there's only one group or the operator
            # opted out. We always wait for ALL groups to finish before
            # publishing — the IPC export and motor command depend on
            # every group's state being current.
            rms_8ch = [0.0] * NUM_EMG_CH
            if pool is not None:
                futs = [pool.submit(_features_and_inference, gst, rms_8ch, ipc)
                        for gst in group_states]
                # block here — total wait ≈ max(group_inference_times)
                # rather than the sequential sum
                for f in futs:
                    f.result()
            else:
                for gst in group_states:
                    _features_and_inference(gst, rms_8ch, ipc)
            # Primary state publish (group 0 is the primary). Done
            # AFTER both groups finish so the IPC export reflects a
            # consistent snapshot.
            _publish_primary_state(ipc, group_states[0])

            # Publish per-group state for the TUI's DSP/AI page. The
            # IPC export only carries one group's prediction (the
            # primary), so we drop a tiny /tmp file with all of them.
            write_group_state_digest(group_states)

            # UART prediction lines — one per group, with the full class
            # probability vector. This mirrors what predictX.py prints
            # to the host so the AI team can A/B compare on-Pi inference
            # against their host-side classifier without leaving the
            # CSV stream behind.
            for gst in group_states:
                classes = (list(gst.model.classes_)
                           if gst.model is not None else None)
                _uart_send_prediction(gst.name,
                                      gst.current_state,
                                      gst.conf_pct / 100.0,
                                      classes=classes,
                                      probs=gst.class_conf or None)
            primary = group_states[0]

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
    if pool is not None:
        # Drain in-flight futures + release worker threads cleanly.
        # cancel_futures requires Python 3.9+; the project targets the
        # Pi's bundled 3.11 so no compat guard needed.
        pool.shutdown(wait=True, cancel_futures=True)
    ipc.close()
    return 0


# ══════════════════════════════════════════════════════════════════════
#  CALIBRATION MODE
# ══════════════════════════════════════════════════════════════════════

def run_calibrate(seconds, operator="default"):
    """Record ``seconds`` of rest-state EMG, compute per-muscle envelope
    floor, write the operator's noise-threshold file.

    File written:
        models/dynamic_noise_thresholds_<operator>.json   (when operator != default)
        models/dynamic_noise_thresholds_default.json      (when operator == default)

    The canonical models/dynamic_noise_thresholds.json — Aleyna's
    training-time calibration — is NEVER overwritten by this command,
    by design. At runtime _load_dynamic_thresholds(<operator>) prefers
    the operator-specific file but falls back to Aleyna's, so a new
    operator can record their own profile without touching the default.

    The threshold value uses the SAME quantity the runtime gate checks
    against in _features_and_inference: ``mean(|envelope|)`` of the
    cleaned, decimated, low-passed signal. Old code stored ``3*std(cleaned)``
    which is in different units than what the gate compares — leftover
    from when this file was diagnostic-only. Switching now yields a
    threshold that's directly comparable to the runtime env_means,
    matching the units of Aleyna's s1/s2/s3 values.

    Use a generous safety factor (mean + 3*std of |envelope|) so the
    threshold sits clearly ABOVE typical rest noise — anything below
    that is genuinely quiet.
    """
    out_path = _dynamic_thr_path(operator)
    # Guard: never overwrite Aleyna's canonical file. If the operator
    # name reduces to "default" we still write to a distinct file.
    if out_path == DYNAMIC_THR_DEFAULT_PATH:
        out_path = os.path.join(MODEL_DIR,
                                "dynamic_noise_thresholds_default.json")

    print(f"[CAL] operator={operator}  output={out_path}", flush=True)
    print(f"[CAL] canonical Aleyna file is left untouched.", flush=True)

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
            thresholds[label] = 50.0       # safe fallback if no data
            continue
        # Mirror _features_and_inference's pipeline so the threshold
        # has the SAME units as the env_means it will be compared
        # against at inference time.
        sig_lo   = decimate(sig_arr, DECIMATE_FACTOR, zero_phase=True)
        centered = sig_lo - np.mean(sig_lo)
        bp       = butter_bandpass(centered, 20.0, 450.0, TARGET_FS_HZ)
        n50      = notch_filter(bp,   50.0,  TARGET_FS_HZ)
        n100     = notch_filter(n50,  100.0, TARGET_FS_HZ)
        n200     = notch_filter(n100, 200.0, TARGET_FS_HZ)
        env      = envelope(n200, TARGET_FS_HZ, cutoff=3.0)
        env_abs  = np.abs(env)
        # threshold = upper bound of rest envelope; mean + 3 std
        # gives a one-sided ~99.7% rest gate, well-clear of typical
        # rest noise but below any genuine muscle activation.
        thr      = float(np.mean(env_abs) + 3.0 * np.std(env_abs))
        thresholds[label] = thr
        print(f"[CAL] {label} (ch{active_channels[bi]}): "
              f"env_mean={np.mean(env_abs):.2f} "
              f"env_std={np.std(env_abs):.2f} thr={thr:.2f}",
              flush=True)

    os.makedirs(MODEL_DIR, exist_ok=True)
    with open(out_path, 'w') as f:
        json.dump(thresholds, f, indent=4)
    print(f"[CAL] saved: {out_path}", flush=True)
    print(f"[CAL] to use it:  ./launch.sh tui --operator {operator}", flush=True)
    # Drop the in-process cache so a subsequent run_inference picks up
    # the freshly-written values without restarting the DSP.
    global _dyn_thresh, _dyn_thresh_op
    _dyn_thresh    = None
    _dyn_thresh_op = None
    ipc.close()
    return 0


# ══════════════════════════════════════════════════════════════════════
#  ENTRY
# ══════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="CPCU DSP + ML pipeline")
    ap.add_argument('--calibrate', type=float, metavar='SEC',
                    help="Record N seconds of rest, write the operator's "
                         "rest-noise file, exit. Aleyna's canonical "
                         "dynamic_noise_thresholds.json is never touched.")
    ap.add_argument('--verbose', action='store_true')
    # Operator defaults to the CPCU_OPERATOR env var if set (launch.sh
    # exports it for every command), then to "default" (Aleyna's
    # noise file + bare velocity_map.json). This is what wires
    # `./launch.sh tui --operator alice` into the noise-floor lookup
    # — the kernel exec'd cpcu_dsp.py without --operator, but the env
    # var carries through fork() so we still see the choice.
    ap.add_argument('--operator',
                    default=os.environ.get('CPCU_OPERATOR', 'default'),
                    help="Operator profile name. Loads "
                         "velocity_map_<name>.json and "
                         "dynamic_noise_thresholds_<name>.json when present.")
    args = ap.parse_args()

    if args.calibrate is not None:
        return run_calibrate(args.calibrate, operator=args.operator)
    return run_inference(verbose=args.verbose, operator=args.operator)


if __name__ == "__main__":
    sys.exit(main())
