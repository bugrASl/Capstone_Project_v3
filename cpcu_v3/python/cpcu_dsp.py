#!/usr/bin/env python3
"""cpcu_dsp.py — live DSP + ML inference pipeline for the CPCU.

Reads EMG from IPC ring, filters, extracts features, runs ML inference,
integrates servo targets via velocity mode, publishes motor commands.

Config sources (priority order):
  1. config/gestures.json    — gesture definitions, EMG channel map, confidence curve
  2. models/velocity_map.json — operator-calibrated rates (overrides gestures.json rates)
  3. config/runtime.json     — hardware tuning (smoother, grip, bias)
  4. models/*.pkl            — trained ML model + scaler
"""
import argparse
import json
import glob
import os
import re
import signal
import sys
import time
from collections import deque

import numpy as np
from scipy.signal import butter, filtfilt, iirnotch, decimate

from cpcu_ipc_bridge import IPCBridge

# ── paths ──
REPO_ROOT       = os.environ.get("CPCU_ROOT",
                    os.path.dirname(os.path.abspath(__file__)) + "/..")
GESTURES_PATH   = os.path.join(REPO_ROOT, "config", "gestures.json")
RUNTIME_PATH    = os.path.join(REPO_ROOT, "config", "runtime.json")
MODEL_DIR       = os.environ.get("CPCU_MODEL_DIR",
                    os.path.join(REPO_ROOT, "models"))
INSTALLED_MODEL = "/opt/cpcu/models"
INSTALLED_CFG   = "/opt/cpcu/config.json"
THRESHOLDS_PATH = os.path.join(MODEL_DIR, "noise_thresholds.json")

# ── sampling ──
INPUT_FS_HZ     = 1000    # matches BSAU 1kHz packet rate (1 sample per pkt after averaging)
TARGET_FS_HZ    = 200
DECIMATE_FACTOR = INPUT_FS_HZ // TARGET_FS_HZ  # 5
WINDOW_MS       = 200
STRIDE_MS       = 100
WINDOW_HI       = INPUT_FS_HZ  * WINDOW_MS // 1000   # 200  (200ms @ 1kHz)
STRIDE_HI       = INPUT_FS_HZ  * STRIDE_MS // 1000   # 100  (100ms stride @ 1kHz)
WINDOW_LO       = TARGET_FS_HZ * WINDOW_MS // 1000   # 40
BUFFER_SIZE     = WINDOW_HI * 4                       # 800

# ── defaults (overridden by config files) ──
ADC_MIDRAIL     = 2048
NUM_SERVOS      = 6
SERVO_NEUTRAL   = 1500
DRAIN_PERIOD_S  = 0.020
DRAIN_BATCH     = 200
PROB_THRESH     = 0.65

# hardware latency constants (from datasheets, not measured)
LAT_ADC_PACK_US     = 226   # STM32 ADC 6ch×2kHz + firmware pack
LAT_WIRELESS_US     = 332   # NRF24L01+ ESB: SPI upload + air + ACK
LAT_SPI_UNPACK_US   = 36    # CPCU NRF SPI read + WL_Unpack + IPC push
LAT_SMOOTHER_I2C_US = 610   # SMOOTH_Update + PCA9685 I²C 6 servos
LAT_SERVO_MECH_US   = 15000 # SG90 mechanical response (~15ms typical)
LAT_TRANSPORT_US    = LAT_ADC_PACK_US + LAT_WIRELESS_US + LAT_SPI_UNPACK_US  # 594µs

SERVO_MIN_US    = [498, 1074, 1074, 1001, 1001, 976]
SERVO_MAX_US    = [2500, 1953, 1953, 2002, 2002, 1733]


# ══════════════════════════════════════════════════════════════════════
#  CONFIG LOADERS
# ══════════════════════════════════════════════════════════════════════

def _resolve_gesture_rates(gestures, name_to_idx):
    """Convert servo name references to index-based rate arrays."""
    for gname, gdef in gestures.items():
        raw_ch = gdef.get("channels", {})
        rates = [0] * NUM_SERVOS
        snap_flags = [False] * NUM_SERVOS
        smoother_v = [0] * NUM_SERVOS
        smoother_a = [0] * NUM_SERVOS
        for sname, chdef in raw_ch.items():
            idx = name_to_idx.get(sname)
            if idx is None:
                print(f"[DSP] unknown servo '{sname}'", flush=True)
                continue
            rates[idx] = chdef.get("rate_us_s", 0)
            snap_flags[idx] = chdef.get("snap", False)
            so = gdef.get("smoother_override", {}).get(sname, {})
            smoother_v[idx] = so.get("velocity_us_s", 0)
            smoother_a[idx] = so.get("accel_us_s2", 0)
        gdef["_rates"] = rates
        gdef["_snap"] = snap_flags
        gdef["_smooth_v"] = smoother_v
        gdef["_smooth_a"] = smoother_a


def load_gestures(path=GESTURES_PATH):
    """Load gestures.json. Returns (groups_list, servo_channels_dict).

    Each group in groups_list is a dict:
      { "name": "gesture_0",
        "gestures": {...},       # with _rates resolved
        "emg_channels": [0,1,2],
        "confidence": {...},
        "hysteresis": {...},
        "model_path": "models/x.pkl" }

    Supports schema v5 (gesture_groups) and v4 (flat gestures, single group).
    """
    default_conf = {"curve": "quadratic", "floor_pct": 40, "ceil_pct": 85}
    default_hyst = {"rest_to_active": 4, "active_to_rest": 2, "active_to_active": 6}
    default_group = {
        "name": "gesture_0",
        "gestures": {"rest": {"mode": "freeze", "channels": {}}},
        "emg_channels": [0, 1, 2],
        "confidence": default_conf,
        "hysteresis": default_hyst,
        "model_path": "models/model.pkl",
    }
    try:
        with open(path) as f:
            gs = json.load(f)

        servo_ch = gs.get("servo_channels", {})
        name_to_idx = {n: d["pca_ch"] for n, d in servo_ch.items()}

        # build servo limit arrays
        servo_list = sorted(servo_ch.items(), key=lambda x: x[1].get("pca_ch", 0))
        global SERVO_MIN_US, SERVO_MAX_US
        if servo_list:
            SERVO_MIN_US = [s[1].get("min_us", 500) for s in servo_list[:NUM_SERVOS]]
            SERVO_MAX_US = [s[1].get("max_us", 2500) for s in servo_list[:NUM_SERVOS]]
            while len(SERVO_MIN_US) < NUM_SERVOS: SERVO_MIN_US.append(500)
            while len(SERVO_MAX_US) < NUM_SERVOS: SERVO_MAX_US.append(2500)

        groups = []
        gg = gs.get("gesture_groups")
        if gg:
            # schema v5: multi-group
            for gname, gdef in gg.items():
                gestures = gdef.get("gestures", {"rest": {"mode": "freeze"}})
                _resolve_gesture_rates(gestures, name_to_idx)
                groups.append({
                    "name": gname,
                    "gestures": gestures,
                    "emg_channels": gdef.get("emg_channels", {}).get("active", [0,1,2]),
                    "confidence": gdef.get("confidence", default_conf),
                    "hysteresis": gdef.get("hysteresis", default_hyst),
                    "model_path": gdef.get("model_path", ""),
                })
        elif "gestures" in gs:
            # schema v4 backward compat: single group
            gestures = gs["gestures"]
            _resolve_gesture_rates(gestures, name_to_idx)
            groups.append({
                "name": "gesture_0",
                "gestures": gestures,
                "emg_channels": gs.get("emg_channels", {}).get("active", [0,1,2]),
                "confidence": gs.get("confidence", default_conf),
                "hysteresis": gs.get("hysteresis", default_hyst),
                "model_path": gs.get("model_path", ""),
            })

        if not groups:
            groups.append(default_group)

        print(f"[DSP] {len(groups)} gesture group(s):", flush=True)
        for g in groups:
            print(f"[DSP]   {g['name']}: {list(g['gestures'].keys())} "
                  f"emg={g['emg_channels']} model={g['model_path']}", flush=True)
        print(f"[DSP] servo limits: min={SERVO_MIN_US} max={SERVO_MAX_US}", flush=True)
        return groups, servo_ch
    except Exception as e:
        print(f"[DSP] gestures.json load failed: {e}, using defaults", flush=True)
        return [default_group], {}


def load_velocity_map(operator="default"):
    """Load operator-calibrated velocity overrides from velocity_map.json.
    Returns {gesture: {servo_idx: rate_us_s}} or empty dict."""
    if operator == "default":
        path = os.path.join(MODEL_DIR, "velocity_map.json")
    else:
        path = os.path.join(MODEL_DIR, f"velocity_map_{operator}.json")
    try:
        with open(path) as f:
            vm = json.load(f)
        if not vm.get("calibrated_at"):
            return {}  # template, not calibrated
        result = {}
        for gname, servos in vm.get("gesture_levels", {}).items():
            result[gname] = {}
            for sname, sdef in servos.items():
                result[gname][sname] = sdef.get("rate_us_s", 0)
        print(f"[DSP] velocity_map ({operator}): {list(result.keys())}", flush=True)
        return result
    except (OSError, ValueError):
        return {}


def load_runtime(path=None):
    """Load hardware tuning from runtime.json.
    Returns grip_firm_us."""

    grip_firm = 1100
    candidates = [path] if path else [INSTALLED_CFG, RUNTIME_PATH]
    for p in candidates:
        if not p:
            continue
        try:
            with open(p) as f:
                text = f.read()
            # strip // comment lines (legacy format tolerance)
            text = re.sub(r'//[^\n]*', '', text)
            text = re.sub(r',(\s*[}\]])', r'\1', text)
            raw = json.loads(text)
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
    """Find and load ML model. Returns (model, scaler) or (None, None).
    If model_path is given, try it first. Then MODEL_DIR/*.pkl, INSTALLED_MODEL/*.pkl."""
    try:
        import joblib
    except ImportError:
        print("[DSP] joblib missing — feature-only mode", flush=True)
        return None, None

    # try specific path first
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

    search_dirs = [MODEL_DIR, INSTALLED_MODEL]
    for d in search_dirs:
        # combined .pkl (dict with model+scaler)
        for pkl in sorted(glob.glob(os.path.join(d, "*.pkl"))):
            try:
                cp = joblib.load(pkl)
                if isinstance(cp, dict) and "model" in cp and "scaler" in cp:
                    print(f"[DSP] model: {pkl}", flush=True)
                    return cp["model"], cp["scaler"]
            except Exception as e:
                print(f"[DSP] {pkl}: {e}", flush=True)


    print(f"[DSP] no model found — feature-only mode", flush=True)
    return None, None


# ══════════════════════════════════════════════════════════════════════
#  DSP FILTERS — direct ports of the team's pipeline
# ══════════════════════════════════════════════════════════════════════

def butter_bandpass(data, low, high, fs, order=4):
    nyq = 0.5 * fs
    if high >= nyq:
        high = nyq * 0.95
    b, a = butter(order, [low/nyq, high/nyq], btype='band')
    return filtfilt(b, a, data)

def notch_filter(data, f0, fs, q=30.0):
    nyq = 0.5 * fs
    w0 = f0 / nyq
    if w0 >= 1:
        return data
    b, a = iirnotch(w0, q)
    return filtfilt(b, a, data)

def envelope(data, fs, cutoff=3.0):
    nyq = 0.5 * fs
    b, a = butter(4, cutoff/nyq, btype='low')
    return filtfilt(b, a, np.abs(data))

def extract_features(clean, env):
    """4 features per channel: RMS, VAR, WL, ENV-mean."""
    eps = 1e-8
    rms = float(np.sqrt(np.mean(clean ** 2) + eps))
    var = float(np.var(clean))
    wl  = float(np.sum(np.abs(np.diff(clean))) / len(clean))
    em  = float(np.mean(env))
    return [rms, var, wl, em]

def process_window(window_hi):
    """Full pipeline on a 400-sample @ 2 kHz window.
    DC remove → bandpass 20-450 Hz → notch 50/100/200 Hz → decimate → envelope."""
    centered = window_hi - np.mean(window_hi)
    bp   = butter_bandpass(centered, 20.0, 450.0, INPUT_FS_HZ)
    n50  = notch_filter(bp, 50.0, INPUT_FS_HZ)
    n100 = notch_filter(n50, 100.0, INPUT_FS_HZ)
    n200 = notch_filter(n100, 200.0, INPUT_FS_HZ)
    cleaned = decimate(n200, DECIMATE_FACTOR, zero_phase=True)
    env = envelope(cleaned, TARGET_FS_HZ, cutoff=3.0)
    return cleaned, env, extract_features(cleaned, env)


# ══════════════════════════════════════════════════════════════════════
#  CONFIDENCE CURVE
# ══════════════════════════════════════════════════════════════════════

def confidence_scale(conf_frac, floor, ceil, curve="quadratic"):
    """Map SVM confidence [0,1] to velocity scale [0,1].
    quadratic: slow start, fast finish. linear: proportional. none: always 1.0."""
    if curve == "none":
        return 1.0
    if conf_frac <= floor:
        return 0.0
    if conf_frac >= ceil:
        return 1.0
    t = (conf_frac - floor) / (ceil - floor)
    if curve == "quadratic":
        return t * t
    return t  # linear fallback


# ══════════════════════════════════════════════════════════════════════
#  UART DEBUG (optional)
# ══════════════════════════════════════════════════════════════════════

UART_PORT = os.environ.get("CPCU_UART_DEBUG", "")
_uart = None

def _uart_init():
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
    if _uart is None:
        return
    try:
        ts = int(time.time() * 1000)
        csv = ",".join(f"{v:.6f}" for v in feats)
        _uart.write(f"{ts},{gesture},{conf:.3f},{csv}\n".encode())
    except Exception:
        pass


# ══════════════════════════════════════════════════════════════════════
#  MAIN INFERENCE LOOP
# ══════════════════════════════════════════════════════════════════════

def run_inference(verbose=False, operator="default"):
    # ── load config ──
    gestures, active_channels, conf_cfg, hyst_cfg = load_gestures()
    num_ch = len(active_channels)
    grip_firm = load_runtime()
    hyst_r2a = hyst_cfg.get("rest_to_active", 4)
    hyst_a2r = hyst_cfg.get("active_to_rest", 2)
    hyst_a2a = hyst_cfg.get("active_to_active", 6)
    vel_overrides = load_velocity_map(operator)

    # apply velocity_map overrides to gesture rates
    servo_channels = {}
    try:
        with open(GESTURES_PATH) as f:
            gs_raw = json.load(f)
        servo_channels = gs_raw.get("servo_channels", {})
    except Exception:
        pass
    name_to_idx = {n: d["pca_ch"] for n, d in servo_channels.items()}

    for gname, overrides in vel_overrides.items():
        if gname not in gestures:
            continue
        for sname, rate in overrides.items():
            idx = name_to_idx.get(sname)
            if idx is not None and "_rates" in gestures[gname]:
                gestures[gname]["_rates"][idx] = rate

    # per-gesture grip_firm override
    for gname, gdef in gestures.items():
        if "grip_firm_us" in gdef:
            gdef["_grip_firm"] = gdef["grip_firm_us"]
        else:
            gdef["_grip_firm"] = grip_firm

    # confidence curve
    conf_floor = conf_cfg.get("floor_pct", 40) / 100.0
    conf_ceil  = conf_cfg.get("ceil_pct", 85) / 100.0
    conf_curve = conf_cfg.get("curve", "quadratic")
    if conf_floor >= conf_ceil:
        conf_floor, conf_ceil = 0.40, 0.85

    # ── load model ──
    model, scaler = discover_model()
    inference_on = model is not None
    if inference_on:
        classes = [str(c) for c in model.classes_]
        n_feat = scaler.n_features_in_
        expected = num_ch * 4
        if n_feat != expected:
            print(f"[DSP] WARNING: model expects {n_feat} features, "
                  f"pipeline produces {expected}", flush=True)
        # cross-check gesture names
        for cls in classes:
            if cls not in gestures:
                print(f"[DSP] model class '{cls}' not in gestures.json "
                      f"— will fall back to neutral", flush=True)
        print(f"[DSP] model classes: {classes}", flush=True)

    # ── IPC ──
    ipc = IPCBridge()
    ipc.set_dsp_ready()
    print(f"[DSP] ready. channels={active_channels} "
          f"curve={conf_curve} hyst=r2a:{hyst_r2a}/a2r:{hyst_a2r}/a2a:{hyst_a2a}", flush=True)

    _uart_init()

    # ── shared state (all channels) ──
    raw_buffers = {ch: deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE) for ch in all_channels}
    rx_times = deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
    seq_history = deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
    samples_since_window = 0
    total_samples = 0
    current_state = "rest"
    consec_count = 0
    last_active = 0
    current_target = [SERVO_NEUTRAL] * NUM_SERVOS
    last_int_t = time.monotonic()
    last_safe = False
    edit_mode = False
    inferences = 0
    last_report = time.monotonic()
    lat_pkt_samples = []   # pkt→servo ms per window
    lat_seq_samples = []   # seq age per window
    lat_dsp_samples = []   # dsp+ml ms per window
    lat_drain_samples = [] # ring drain ms per window

    running = [True]
    def stop(sig, _):
        running[0] = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    while running[0]:
        t0 = time.monotonic()

        # ── 1. drain ring ──
        t_drain_start = time.monotonic()
        batch = ipc.pop_sensor_batch(DRAIN_BATCH)
        n = batch.get('count', 0)
        if n > 0:
            ipc.inc_dsp_batches(n)
            samples = batch['samples']
            batch_seq = batch.get('seq')
            batch_rx = batch.get('rx_time_us')
            for ei in range(n):
                for si in range(2):
                    for bi, ch in enumerate(active_channels):
                        buffers[bi].append(
                            int(samples[ei, si, ch]) - ADC_MIDRAIL)
                # track rx_time and seq per sample pair (2 samples per packet)
                if batch_rx is not None and batch_rx[ei] > 0:
                    rx_times.append(int(batch_rx[ei]))
                    rx_times.append(int(batch_rx[ei]))  # 2 samples per pkt
                if batch_seq is not None:
                    seq_history.append(int(batch_seq[ei]))
                    seq_history.append(int(batch_seq[ei]))
            added = n * 2
            samples_since_window += added
            total_samples += added
        t_drain = time.monotonic() - t_drain_start

        # ── 2. check window readiness ──
        ready = (samples_since_window >= STRIDE_HI
                 and total_samples >= WINDOW_HI)

        if ready:
            t_dsp_start = time.monotonic()
            samples_since_window = 0

            # ── 3+4. per-group: feature extraction + inference + debounce ──
            rms_8ch = [0.0] * 8
            primary_state = "rest"
            primary_conf_pct = 0
            primary_class_conf = []
            primary_active = 0

            for gi, gst in enumerate(group_states):
                features_flat = []
                for bi, ch in enumerate(gst.emg_channels):
                    w = np.array(list(gst.buffers[bi])[-WINDOW_HI:], dtype=np.float64)
                    _cl, _ev, feats = process_window(w)
                    features_flat.extend(feats)
                    rms_8ch[ch] = feats[0]
                    try:
                        ipc.write_dsp_filtered_window(ch, _ev,
                                                      sample_rate_hz=TARGET_FS_HZ)
                    except Exception:
                        pass

                # inference for this group
                gst.conf_pct = 0
                gst.class_conf = []
                if gst.inference_on:
                    X = np.asarray(features_flat, dtype=np.float64).reshape(1, -1)
                    Xs = gst.scaler.transform(X)
                    probs = gst.model.predict_proba(Xs)[0]
                    ai = int(np.argmax(probs))
                    label = str(gst.model.classes_[ai])
                    conf = float(probs[ai])

                    if conf > PROB_THRESH and label != gst.current_state:
                        if gst.current_state == "rest":
                            needed = gst.hyst_r2a
                            htype_id = 0
                        elif label == "rest":
                            needed = gst.hyst_a2r
                            htype_id = 1
                        else:
                            needed = gst.hyst_a2a
                            htype_id = 2
                        gst.consec_count += 1
                        if gst.consec_count >= needed:
                            gst.current_state = label
                            gst.consec_count = 0
                    else:
                        gst.consec_count = 0

                    gst.last_active = ai
                    gst.class_conf = list(probs)
                    gst.conf_pct = int(round(conf * 100.0))

                # first group is "primary" for IPC export
                if gi == 0:
                    primary_state = gst.current_state
                    primary_conf_pct = gst.conf_pct
                    primary_class_conf = gst.class_conf
                    primary_active = gst.last_active
                    ipc.write_hysteresis_state(gst.consec_count,
                        gst.hyst_r2a if gst.current_state == "rest" else gst.hyst_a2a,
                        0 if gst.current_state == "rest" else 2)

            _uart_send(primary_state, primary_conf_pct / 100.0, [])

            # ── 5. publish dsp export ──
            inf_us = int((time.monotonic() - t_dsp_start) * 1e6)
            ipc.update_dsp_max_latency(inf_us)
            ipc.write_dsp_export(
                channel_rms=rms_8ch,
                gesture_name=primary_state,
                class_confidence=primary_class_conf,
                active_class=primary_active,
                inference_time_us=inf_us,
            )

            # ── 6. edit-mode handshake ──
            edit_req = ipc.read_edit_request()
            if edit_req:
                if not edit_mode:
                    for gst in group_states:
                        gst.current_state = "rest"
                        gst.consec_count = 0
                    current_target = [SERVO_NEUTRAL] * NUM_SERVOS
                    ipc.write_edit_dsp_ack(1)
                    edit_mode = True
                last_int_t = time.monotonic()
            else:
                if edit_mode:
                    ipc.write_edit_dsp_ack(0)
                    edit_mode = False

                # fault recovery: snap to neutral if SAFE was entered
                sys_state = ipc.read_system_state()
                in_safe = (sys_state == 2)
                if in_safe and not last_safe:
                    current_target = [SERVO_NEUTRAL] * NUM_SERVOS
                    for gst in group_states:
                        gst.current_state = "rest"
                        gst.consec_count = 0
                last_safe = in_safe

                # ── 7. velocity integration (all groups combined) ──
                t_vel_start = time.monotonic()
                now = time.monotonic()
                dt = now - last_int_t
                last_int_t = now
                if dt > 0.5:
                    dt = 0.0  # stall guard

                # each group contributes to current_target independently
                any_rest_all = all(gst.current_state == "rest" for gst in group_states)
                if any_rest_all:
                    current_target = [SERVO_NEUTRAL] * NUM_SERVOS
                else:
                    for gst in group_states:
                        gdef = gst.gestures.get(gst.current_state)
                        if gdef is None:
                            continue
                        if gdef.get("mode") == "freeze":
                            continue  # freeze = no contribution
                        # velocity mode
                        conf_frac = gst.conf_pct / 100.0
                        scale = confidence_scale(conf_frac, gst.conf_floor,
                                                 gst.conf_ceil, gst.conf_curve)
                        rates = gdef.get("_rates", [0]*NUM_SERVOS)
                        gf = gdef.get("_grip_firm", grip_firm)
                        for s in range(NUM_SERVOS):
                            if rates[s] == 0:
                                continue  # this group doesn't control this servo
                            delta = rates[s] * dt * scale
                            nv = current_target[s] + delta
                            nv = max(SERVO_MIN_US[s], min(SERVO_MAX_US[s], nv))
                            if s == 5 and nv < gf:
                                nv = gf
                            current_target[s] = nv

                servo_us = [int(round(v)) for v in current_target]
                ipc.write_motor_cmd(servo_us, primary_active, primary_conf_pct)
                t_vel = time.monotonic() - t_vel_start

            # ── 7b. packet-based latency measurement ──
            t_servo_us = int(time.monotonic() * 1e6)
            rx_list = list(rx_times)
            seq_list = list(seq_history)
            if len(rx_list) >= WINDOW_HI:
                oldest_rx = rx_list[-WINDOW_HI]
                newest_rx = rx_list[-1]
                if oldest_rx > 0:
                    pkt_to_servo_us = t_servo_us - oldest_rx
                    ring_dwell_us = int(t_drain * 1e6)
                    oldest_seq = seq_list[-WINDOW_HI] & 0xFF
                    newest_seq = seq_list[-1] & 0xFF
                    seq_age = (newest_seq - oldest_seq) & 0xFF
                    ipc.write_latency_pkt(
                        pkt_to_servo_us=pkt_to_servo_us,
                        ring_dwell_us=ring_dwell_us,
                        dsp_compute_us=int(t_dsp * 1e6),
                        seq_age=seq_age,
                    )
                    # accumulate for periodic report
                    lat_pkt_samples.append(pkt_to_servo_us / 1000.0)
                    lat_seq_samples.append(seq_age)
                    lat_dsp_samples.append(t_dsp * 1e3)
                    lat_drain_samples.append(t_drain * 1e3)

            ipc.inc_dsp_inferences()
            inferences += 1

            if verbose:
                pkt_ms = pkt_to_servo_us / 1000.0 if 'pkt_to_servo_us' in dir() else 0
                print(f"[DSP] {current_state} {conf_pct}%  "
                      f"pkt→servo={pkt_ms:.1f}ms  "
                      f"drain={t_drain*1e3:.1f}ms  "
                      f"dsp={t_dsp*1e3:.1f}ms  "
                      f"seq_age={seq_age if 'seq_age' in dir() else '?'}",
                      flush=True)

        # ── 8. periodic report (every 5s) ──
        if t0 - last_report >= 5.0:
            print(f"[DSP] ── latency waterfall ───────────────────", flush=True)
            print(f"[DSP]   groups={[(g.name,g.current_state,g.conf_pct) for g in group_states]}  inf={inferences}", flush=True)
            print(f"[DSP]   ┌─ BSAU ─────────────────────────────", flush=True)
            print(f"[DSP]   │ ADC + pack:      {LAT_ADC_PACK_US:>6} µs  (const)", flush=True)
            print(f"[DSP]   │ wireless TX+ACK: {LAT_WIRELESS_US:>6} µs  (const)", flush=True)
            print(f"[DSP]   ├─ CPCU ─────────────────────────────", flush=True)
            print(f"[DSP]   │ SPI + unpack:    {LAT_SPI_UNPACK_US:>6} µs  (const)", flush=True)
            if lat_drain_samples:
                dravg = sum(lat_drain_samples) / len(lat_drain_samples)
                print(f"[DSP]   │ ring dwell:    {dravg*1000:>8.0f} µs  (meas)", flush=True)
            if lat_dsp_samples:
                davg = sum(lat_dsp_samples) / len(lat_dsp_samples)
                dmax = max(lat_dsp_samples)
                print(f"[DSP]   │ DSP compute:   {davg*1000:>8.0f} µs  (meas, max {dmax*1000:.0f})", flush=True)
            print(f"[DSP]   │ smoother+I²C:   {LAT_SMOOTHER_I2C_US:>6} µs  (const)", flush=True)
            print(f"[DSP]   ├─ SERVO ────────────────────────────", flush=True)
            print(f"[DSP]   │ mechanical:    {LAT_SERVO_MECH_US:>8} µs  (const)", flush=True)
            print(f"[DSP]   └─ TOTALS ───────────────────────────", flush=True)
            if lat_pkt_samples:
                s = sorted(lat_pkt_samples)
                avg = sum(s) / len(s)
                p95 = s[int(len(s) * 0.95)] if len(s) > 1 else s[0]
                cpcu_avg = avg * 1000  # µs
                bsau_total = LAT_TRANSPORT_US + cpcu_avg + LAT_SMOOTHER_I2C_US + LAT_SERVO_MECH_US
                print(f"[DSP]   │ CPCU pkt→srv:  {cpcu_avg:>8.0f} µs  avg ({avg:.1f}ms)", flush=True)
                print(f"[DSP]   │                {p95*1000:>8.0f} µs  p95 ({p95:.1f}ms)", flush=True)
                print(f"[DSP]   │ BSAU→servo:   {bsau_total:>8.0f} µs  ({bsau_total/1000:.1f}ms)", flush=True)
            if lat_seq_samples:
                avg_seq = sum(lat_seq_samples) / len(lat_seq_samples)
                print(f"[DSP]   │ seq age:     {avg_seq:>8.0f} pkts ({avg_seq:.0f}ms @1kHz)", flush=True)
            print(f"[DSP] ──────────────────────────────────────", flush=True)
            # reset accumulators
            lat_pkt_samples.clear()
            lat_seq_samples.clear()
            lat_dsp_samples.clear()
            lat_drain_samples.clear()
            last_report = t0

        # ── 9. sleep remainder ──
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
    """Collect rest-state samples, compute 3*std per channel, save thresholds."""
    groups, _ = load_gestures()
    active_channels = groups[0]["emg_channels"] if groups else [0,1,2]
    num_ch = len(active_channels)
    labels = [f"s{i+1}" for i in range(num_ch)]

    ipc = IPCBridge()
    print(f"[CAL] recording {seconds}s of rest...", flush=True)

    bufs = [deque(maxlen=int(INPUT_FS_HZ * (seconds + 1))) for _ in range(num_ch)]
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        batch = ipc.pop_sensor_batch(DRAIN_BATCH)
        n = batch.get('count', 0)
        if n > 0:
            samples = batch['samples']
            for ei in range(n):
                for si in range(2):
                    for bi, ch in enumerate(active_channels):
                        bufs[bi].append(int(samples[ei, si, ch]) - ADC_MIDRAIL)
        time.sleep(DRAIN_PERIOD_S)

    thresholds = {}
    for bi, label in enumerate(labels):
        sig = np.array(bufs[bi], dtype=np.float64)
        if len(sig) < WINDOW_HI:
            thresholds[label] = 50.0
            continue
        sig_lo = decimate(sig, DECIMATE_FACTOR, zero_phase=True)
        centered = sig_lo - np.mean(sig_lo)
        bp = butter_bandpass(centered, 20.0, 450.0, TARGET_FS_HZ)
        cleaned = notch_filter(bp, 50.0, TARGET_FS_HZ)
        thr = float(np.std(cleaned) * 3.0)
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
