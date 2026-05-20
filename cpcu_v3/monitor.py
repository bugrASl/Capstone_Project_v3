#!/usr/bin/env python3
"""monitor.py — CPCU live UART dashboard.

Six EMG waveforms (left column, ch1-3 stacked then ch4-6 stacked) and
two classifier panels (right column): ARM 1 (R, ch1-3) and ARM 2 (L,
ch4-6). Each panel shows host-side inference + Pi-side overlay parsed
from "#pred,..." UART lines.

Pi side streams:
    raw CSV     "v0,v1,v2,v3,v4,v5\\n"                      @ 1 kHz
    predictions "#pred,ts,group,gesture,conf,cls:p,..."     @ 5 Hz/group

CLI:
    --port /dev/ttyUSB0   (or COMx on Windows)
    --baud 921600
    --model models/arm.pkl
"""
import argparse
import sys
import threading
import warnings
from collections import deque

import numpy as np
import serial
import joblib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.signal import butter, filtfilt, iirnotch

warnings.filterwarnings("ignore", message="X does not have valid feature names")
try:
    from sklearn.exceptions import InconsistentVersionWarning
    warnings.filterwarnings("ignore", category=InconsistentVersionWarning)
except ImportError:
    pass

# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser()
    default_port = "COM5" if sys.platform.startswith("win") else "/dev/ttyUSB0"
    p.add_argument("--port",  default=default_port)
    p.add_argument("--baud",  type=int, default=921600)
    p.add_argument("--model", default="models/arm.pkl",
                   help="path to arm.pkl (used for BOTH groups since they "
                        "share the same Hand/Biceps/Triceps muscle set)")
    return p.parse_args()


args = parse_args()

# ─────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────
FS                = 1000
WINDOW_SIZE       = 200
PREDICT_EVERY     = 100
CHANNELS_EXPECTED = 6
PLOT_WINDOW       = 1000
SELECTED_CHANNELS = [1, 2, 3, 4, 5, 6]
NUM_CHANNELS      = len(SELECTED_CHANNELS)

MAX_ADC_VALUE = 4095
MIN_ADC_VALUE = 10
MAX_DELTA     = 1300

PLOT_DECIMATION  = 2
ANIM_INTERVAL_MS = 25
SERIAL_POLL_MS   = 1
PLOT_POINTS      = PLOT_WINDOW // PLOT_DECIMATION

# ─────────────────────────────────────────────────────────────────────
# FILTER COEFFICIENTS (computed once)
# ─────────────────────────────────────────────────────────────────────
NYQ            = 0.5 * FS
BP_B,   BP_A   = butter(4, [20 / NYQ, min(450 / NYQ, 0.95)], btype="band")
N50_B,  N50_A  = iirnotch(50  / NYQ, 30.0)
N100_B, N100_A = iirnotch(100 / NYQ, 30.0)
N200_B, N200_A = iirnotch(200 / NYQ, 30.0)
LP_B,   LP_A   = butter(4, 3.0 / NYQ, btype="low")

# ─────────────────────────────────────────────────────────────────────
# MODELS — same arm.pkl drives both groups (shared muscle set)
# ─────────────────────────────────────────────────────────────────────
def load_model(path):
    try:
        ckpt = joblib.load(path)
        print(f"[monitor] loaded model: {path}")
        return ckpt["model"], ckpt["scaler"]
    except Exception as e:
        print(f"[monitor] failed to load {path}: {e}")
        sys.exit(1)


model_R, scaler_R = load_model(args.model)
model_L, scaler_L = load_model(args.model)
CLASSES_R = list(model_R.classes_)
CLASSES_L = list(model_L.classes_)

# Print the exact class order so we can verify it matches what the
# AI team trained on. If the model says "wrong gesture" (e.g. you
# flex biceps and it shows hand), the order here may not match what
# you expect — check against your training script.
print(f"[monitor] ARM 1 (R) classes (in model order): {CLASSES_R}")
print(f"[monitor] ARM 2 (L) classes (in model order): {CLASSES_L}")
print(f"[monitor] ch1-3 fed to ARM 1, ch4-6 fed to ARM 2")
print(f"[monitor] If predictions seem swapped — your model's training "
      f"channel order may differ from BSAU's. Edit SELECTED_CHANNELS.")

CLASS_COLORS = {
    "rest": "#d4edda",
    "hand": "#ffe4b5",
    "flex": "#add8e6",
    "ext":  "#ffcccb",
}

# ─────────────────────────────────────────────────────────────────────
# RING BUFFER + READER THREAD
# ─────────────────────────────────────────────────────────────────────
ring_buffer       = np.full((NUM_CHANNELS, PLOT_WINDOW), 2048.0, dtype=np.float32)
ring_idx          = 0
ring_lock         = threading.Lock()
last_valid        = np.full(NUM_CHANNELS, 2048, dtype=np.int32)
new_samples_count = 0

pi_predictions = {}   # {group: {"gesture", "conf", "classes": {name: prob}}}

try:
    ser = serial.Serial(args.port, args.baud, timeout=0)
    ser.flushInput()
    print(f"[monitor] listening on {args.port} @ {args.baud} baud")
except Exception as e:
    print(f"[monitor] serial open failed: {e}")
    sys.exit(1)

stop_event = threading.Event()


def _parse_pred_line(line):
    try:
        parts = line[len("#pred,"):].split(",")
        if len(parts) < 4:
            return
        _ts, group, gesture, conf = parts[0], parts[1], parts[2], parts[3]
        entry = {"gesture": gesture, "conf": float(conf), "classes": {}}
        for tail in parts[4:]:
            if ":" not in tail:
                continue
            cname, cprob = tail.split(":", 1)
            try:
                entry["classes"][cname] = float(cprob)
            except ValueError:
                pass
        pi_predictions[group] = entry
    except Exception:
        pass


def serial_reader():
    global ring_idx, new_samples_count
    leftover = b""
    sel      = SELECTED_CHANNELS
    while not stop_event.is_set():
        try:
            n = ser.in_waiting
            if n == 0:
                stop_event.wait(SERIAL_POLL_MS / 1000.0)
                continue
            data = leftover + ser.read(n)
            if b"\n" not in data:
                leftover = data
                continue
            chunks   = data.split(b"\n")
            leftover = chunks[-1]
            for raw in chunks[:-1]:
                if not raw or len(raw) < 4:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("#pred,"):
                    _parse_pred_line(line)
                    continue
                if line.startswith("#"):
                    continue
                parts = line.split(",")
                if len(parts) < CHANNELS_EXPECTED:
                    continue
                # Parse + validate ATOMICALLY: build new values into a
                # scratch list, only commit to last_valid + ring buffer
                # if every channel parsed cleanly. Avoids two bugs:
                #   1. np.empty leaves uninitialized garbage in the
                #      array — a malformed line that aborts mid-loop
                #      used to push random memory to the ring (the
                #      "drops to 0" you see in the waveforms).
                #   2. last_valid got partially poisoned even on a
                #      bad line, shifting the spike-filter baseline.
                new_vals = [None] * NUM_CHANNELS
                ok = True
                for i, ch in enumerate(sel):
                    try:
                        v = int(parts[ch - 1])
                    except (ValueError, IndexError):
                        ok = False; break
                    lv = last_valid[i]
                    if v < MIN_ADC_VALUE or v > MAX_ADC_VALUE \
                       or abs(v - lv) > MAX_DELTA:
                        v = int(lv)   # hold last good value
                    new_vals[i] = v
                if not ok:
                    continue
                # Commit
                for i in range(NUM_CHANNELS):
                    last_valid[i] = new_vals[i]
                with ring_lock:
                    for i in range(NUM_CHANNELS):
                        ring_buffer[i, ring_idx] = new_vals[i]
                    ring_idx          = (ring_idx + 1) % PLOT_WINDOW
                    new_samples_count += 1
        except Exception:
            stop_event.wait(0.01)


reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()


# ─────────────────────────────────────────────────────────────────────
# DSP + FEATURES
# ─────────────────────────────────────────────────────────────────────
def process_live_signal(data):
    sig = data - np.mean(data)
    sig = filtfilt(BP_B,   BP_A,   sig)
    sig = filtfilt(N50_B,  N50_A,  sig)
    sig = filtfilt(N100_B, N100_A, sig)
    sig = filtfilt(N200_B, N200_A, sig)
    env = filtfilt(LP_B,   LP_A,   np.abs(sig))
    return sig, env


def extract_features(clean_win, env_win):
    eps    = 1e-8
    abs_c  = np.abs(clean_win)
    diff_c = np.diff(clean_win)
    signs  = np.sign(clean_win)
    return [
        float(np.sqrt(np.mean(clean_win * clean_win) + eps)),
        float(np.var(clean_win)),
        float(np.sum(np.abs(diff_c)) / (len(clean_win) + eps)),
        float(np.mean(env_win)),
        float(np.mean(abs_c)),
        int  (np.sum(np.diff(signs) != 0)),
        int  (np.sum(np.diff(np.sign(diff_c)) != 0)),
    ]


def get_snapshot():
    with ring_lock:
        idx  = ring_idx
        snap = np.concatenate((ring_buffer[:, idx:], ring_buffer[:, :idx]), axis=1)
    return snap


# ─────────────────────────────────────────────────────────────────────
# PLOT
# ─────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 9))
fig.suptitle("CPCU UART Monitor — dual-arm classifier (R: ch1-3, L: ch4-6)",
             fontsize=14)

sig_axes = [fig.add_subplot(3, 3, slot) for slot in (1, 4, 7, 2, 5, 8)]
PLOT_COLORS = ["blue", "red", "green", "orange", "purple", "brown"]
LABELS      = ["R_Hand (ch1)", "R_Biceps (ch2)", "R_Triceps (ch3)",
               "L_Hand (ch4)", "L_Biceps (ch5)", "L_Triceps (ch6)"]
x_axis      = np.arange(PLOT_POINTS)
initial_y   = np.full(PLOT_POINTS, 2048.0)
lines = []
for i, ax in enumerate(sig_axes):
    line, = ax.plot(x_axis, initial_y, color=PLOT_COLORS[i],
                    label=LABELS[i], animated=True, linewidth=0.9)
    ax.set_ylim(0, 4500)
    ax.set_xlim(0, PLOT_POINTS - 1)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    lines.append(line)

ax_R = fig.add_subplot(2, 3, 3)
ax_L = fig.add_subplot(2, 3, 6)
for ax in (ax_R, ax_L):
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")


def setup_status_panel(ax, title, classes):
    ax.text(0.5, 0.96, title, ha="center", va="top",
            fontsize=13, fontweight="bold", transform=ax.transAxes)
    host_state = ax.text(0.5, 0.85, "HOST: —", ha="center", va="top",
                         fontsize=16, fontweight="bold",
                         transform=ax.transAxes, animated=True,
                         bbox=dict(facecolor="white", edgecolor="black",
                                   boxstyle="round,pad=0.4"))
    host_conf  = ax.text(0.5, 0.68, "Confidence: --", ha="center", va="top",
                         fontsize=11, transform=ax.transAxes, animated=True)
    pi_state   = ax.text(0.5, 0.58, "Pi: —", ha="center", va="top",
                         fontsize=10, color="#555555",
                         transform=ax.transAxes, animated=True,
                         family="monospace")
    bar_axes = []
    for i, cls in enumerate(classes):
        y = 0.46 - i * 0.10
        ax.text(0.05, y + 0.03, cls.upper(), fontsize=10,
                transform=ax.transAxes, va="center")
        bg = plt.Rectangle((0.05, y - 0.02), 0.90, 0.07,
                           transform=ax.transAxes, color="#e0e0e0", zorder=1)
        fg = plt.Rectangle((0.05, y - 0.02), 0.0, 0.07,
                           transform=ax.transAxes, color="steelblue",
                           zorder=2, animated=True)
        ax.add_patch(bg); ax.add_patch(fg)
        bar_axes.append(fg)
    return host_state, host_conf, pi_state, bar_axes


hs_R, hc_R, pi_R, bars_R = setup_status_panel(ax_R, "ARM 1 — RIGHT (ch1-3)", CLASSES_R)
hs_L, hc_L, pi_L, bars_L = setup_status_panel(ax_L, "ARM 2 — LEFT  (ch4-6)", CLASSES_L)

animated = (list(lines)
            + [hs_R, hc_R, pi_R, hs_L, hc_L, pi_L]
            + bars_R + bars_L)


hist_R = deque(maxlen=3)
hist_L = deque(maxlen=3)
stable_R = "WAITING..."
stable_L = "WAITING..."


def smooth(raw, history, current):
    history.append(raw)
    if raw == "rest":
        history.clear()
        history.extend(["rest", "rest", "rest"])
        return "rest"
    if len(history) == 3 and len(set(history)) == 1:
        return raw
    return current


def update_panel(group_name, hs, hc, pi_t, bars, conf, probs, classes, stable):
    bg = CLASS_COLORS.get(stable, "yellow")
    hs.set_text(f"HOST: {stable.upper()}")
    hs.get_bbox_patch().set_facecolor(bg)
    hc.set_text(f"Confidence: {conf:.1%}")
    for i, cls in enumerate(classes):
        bars[i].set_width(0.90 * probs[i])
        bars[i].set_facecolor(bg if cls == stable else "steelblue")
    p = pi_predictions.get(group_name)
    if p:
        cls_str = " ".join(f"{c}:{int(v*100):02d}"
                           for c, v in p["classes"].items())
        pi_t.set_text(f"Pi → {p['gesture'].upper()} {int(p['conf']*100)}%\n[{cls_str}]")
    else:
        pi_t.set_text("Pi: (no #pred yet)")


def update(frame):
    global new_samples_count, stable_R, stable_L

    snap = get_snapshot()
    display = snap[:, ::PLOT_DECIMATION] if PLOT_DECIMATION > 1 else snap
    for i, line in enumerate(lines):
        line.set_ydata(display[i])

    with ring_lock:
        do_predict = new_samples_count >= PREDICT_EVERY
        if do_predict:
            new_samples_count = 0

    if do_predict:
        win = snap[:, -WINDOW_SIZE:] - 2048.0

        # Right arm — ch1-3
        feats_R = []
        for i in range(3):
            cl, ev = process_live_signal(win[i])
            feats_R.extend(extract_features(cl, ev))
        fR      = np.asarray(feats_R, dtype=np.float32).reshape(1, -1)
        probs_R = model_R.predict_proba(scaler_R.transform(fR))[0]
        raw_R   = CLASSES_R[int(probs_R.argmax())]
        conf_R  = float(probs_R.max())
        stable_R = smooth(raw_R, hist_R, stable_R)
        update_panel("right_arm", hs_R, hc_R, pi_R, bars_R,
                     conf_R, probs_R, CLASSES_R, stable_R)

        # Left arm — ch4-6
        feats_L = []
        for i in range(3, 6):
            cl, ev = process_live_signal(win[i])
            feats_L.extend(extract_features(cl, ev))
        fL      = np.asarray(feats_L, dtype=np.float32).reshape(1, -1)
        probs_L = model_L.predict_proba(scaler_L.transform(fL))[0]
        raw_L   = CLASSES_L[int(probs_L.argmax())]
        conf_L  = float(probs_L.max())
        stable_L = smooth(raw_L, hist_L, stable_L)
        update_panel("left_arm", hs_L, hc_L, pi_L, bars_L,
                     conf_L, probs_L, CLASSES_L, stable_L)

    return animated


anim = animation.FuncAnimation(fig, update,
                               interval=ANIM_INTERVAL_MS,
                               blit=True,
                               cache_frame_data=False)

try:
    plt.tight_layout()
    plt.show()
finally:
    stop_event.set()
    reader_thread.join(timeout=1.0)
    ser.close()
