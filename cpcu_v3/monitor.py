#!/usr/bin/env python3
"""
monitor.py — Host-side EMG monitor for the CPCU UART stream.

This is a direct port of the AI team's predictX.py, adapted to read the
6-channel raw ADC stream that cpcu_dsp.py mirrors out of /dev/ttyAMA0
at 921600 baud.  The parser, filter chain, feature extractor, model
invocation, plotting layout, and debounce logic are identical to
predictX.py — only the serial source and channel count change.

Wire-up:
    Pi GPIO14 TX  ->  USB-UART adapter RX
    Pi GND        ->  USB-UART adapter GND
    Adapter VBUS  ->  host PC USB

Usage:
    python3 monitor.py --port /dev/ttyUSB0
    python3 monitor.py --port COM7        --channels 1,2,3   # right arm
    python3 monitor.py --port /dev/ttyUSB0 --channels 4,5,6  # left arm

The stream multiplexes:
    CSV ints:    "ch0,ch1,ch2,ch3,ch4,ch5\\n"   (2 kHz, raw samples)
    Prediction:  "#pred,ts_ms,group,gesture,conf,cls0:p0,cls1:p1,...\\n"
                 (~5 Hz, per group, comment-prefixed so the CSV
                 parser skips it. The optional class:prob tail gives
                 the full softmax vector — same data predictX.py
                 plotted from local inference.)

Lines starting with '#' are skipped by the CSV parser, so the
prediction line is purely informational — this script re-runs
inference locally from the raw stream to give the AI team a
ground-truth view independent of what the Pi computed.
"""
import argparse
import sys
import warnings
from collections import deque

import joblib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial
from scipy.signal import butter, filtfilt, iirnotch

warnings.filterwarnings("ignore", message="X does not have valid feature names")


# ============================================================================
# CLI / CONFIG
# ============================================================================
def parse_args():
    p = argparse.ArgumentParser(description="CPCU UART monitor")
    p.add_argument("--port",     default="/dev/ttyUSB0",
                   help="serial port (default /dev/ttyUSB0; Windows: COM7 etc.)")
    p.add_argument("--baud",     default=921600,  type=int)
    p.add_argument("--channels", default="1,2,3",
                   help="1-indexed channels to display+classify (e.g. '1,2,3' "
                        "for right arm, '4,5,6' for left arm)")
    p.add_argument("--model",    default="x.pkl",
                   help="trained .pkl file ({'model','scaler'} dict)")
    return p.parse_args()


ARGS              = parse_args()
FS                = 1000                   # CPCU streams effectively 1 kHz to host
WINDOW_SIZE       = 200                    # 200 ms window for inference
PREDICT_EVERY     = 100                    # predict every 100 new samples
CHANNELS_STREAMED = 6                      # CPCU emits 6 columns per line
SELECTED_CHANNELS = [int(c) for c in ARGS.channels.split(",")]
NUM_CHANNELS      = len(SELECTED_CHANNELS)
PLOT_WINDOW       = 1000                   # show 1 s of data per channel

# Hardware spike rejection (same thresholds as predictX.py)
MAX_ADC_VALUE = 4095
MIN_ADC_VALUE = 10
MAX_DELTA     = 1300


# ============================================================================
# MODEL
# ============================================================================
try:
    checkpoint = joblib.load(ARGS.model)
    model      = checkpoint["model"]
    scaler     = checkpoint["scaler"]
    print(f"[monitor] model loaded: {ARGS.model}")
except Exception as e:
    print(f"[monitor] error loading {ARGS.model}: {e}")
    sys.exit(1)

CLASSES      = list(model.classes_)
CLASS_COLORS = dict(zip(CLASSES,
    ["#d4edda", "#ffcccb", "#add8e6", "#ffe4b5", "#e6e6fa"]))


# ============================================================================
# SERIAL
# ============================================================================
try:
    ser = serial.Serial(ARGS.port, ARGS.baud, timeout=0.01)
    ser.flushInput()
    print(f"[monitor] listening on {ARGS.port} @ {ARGS.baud} baud")
except Exception as e:
    print(f"[monitor] serial error: {e}")
    sys.exit(1)


# ============================================================================
# DSP — identical to predictX.py
# ============================================================================
def validate_sample(cur, last):
    """Hardware spike reject: clamp out-of-range / huge-delta samples."""
    if cur < MIN_ADC_VALUE or cur > MAX_ADC_VALUE: return last
    if abs(cur - last) > MAX_DELTA:                return last
    return cur


def butter_bandpass_filter(data, lo, hi, fs, order=4):
    nyq = 0.5 * fs
    if hi >= nyq: hi = nyq * 0.95
    b, a = butter(order, [lo/nyq, hi/nyq], btype="band")
    return filtfilt(b, a, data)


def notch_filter(data, f0, fs, q=30.0):
    nyq = 0.5 * fs
    w0  = f0 / nyq
    if w0 >= 1: return data
    b, a = iirnotch(w0, q)
    return filtfilt(b, a, data)


def lowpass_envelope(data, cutoff, fs, order=4):
    b, a = butter(order, cutoff/(0.5*fs), btype="low")
    return filtfilt(b, a, data)


def process_live(data):
    """DC-center -> 20-450 BP -> 50/100/200 notches -> envelope."""
    sig    = data - np.mean(data)
    sig    = butter_bandpass_filter(sig, 20, 450, FS)
    sig    = notch_filter(sig, 50,  FS)
    sig    = notch_filter(sig, 100, FS)
    sig    = notch_filter(sig, 200, FS)
    env    = lowpass_envelope(np.abs(sig), 3.0, FS)
    return sig, env


def extract_features(clean, env):
    """7 features per channel — must match cpcu_dsp.py NUM_FEATURES_PER_CHANNEL."""
    eps      = 1e-8
    rms      = np.sqrt(np.mean(clean**2) + eps)
    var      = np.var(clean)
    wl       = np.sum(np.abs(np.diff(clean))) / (len(clean) + eps)
    env_mean = np.mean(env)
    mav      = np.mean(np.abs(clean))
    zc       = np.sum(np.diff(np.sign(clean))           != 0)
    ssc      = np.sum(np.diff(np.sign(np.diff(clean))) != 0)
    return [rms, var, wl, env_mean, mav, zc, ssc]


# ============================================================================
# GUI
# ============================================================================
buffers    = [deque([2048]*PLOT_WINDOW, maxlen=PLOT_WINDOW)
              for _ in range(NUM_CHANNELS)]
last_valid = [2048] * NUM_CHANNELS

fig = plt.figure(figsize=(12, 8))
fig.suptitle(f"CPCU UART Monitor — channels {SELECTED_CHANNELS}", fontsize=14)

# left column: raw waveforms
axes = [fig.add_subplot(NUM_CHANNELS, 2, 2*i + 1) for i in range(NUM_CHANNELS)]
COLORS = ["blue", "red", "green", "orange", "purple", "brown"]
lines  = []
for i, ax in enumerate(axes):
    ln, = ax.plot(buffers[i], color=COLORS[i % len(COLORS)],
                  label=f"CH{SELECTED_CHANNELS[i]}")
    ax.set_ylim(0, 4500); ax.legend(loc="upper right"); ax.grid(alpha=0.3)
    lines.append(ln)

# right column: state + class confidences
ax_state = fig.add_subplot(1, 2, 2)
ax_state.set_xlim(0, 1); ax_state.set_ylim(0, 1); ax_state.axis("off")

state_text = ax_state.text(0.5, 0.82, "WAITING...", ha="center", va="center",
    fontsize=24, fontweight="bold", transform=ax_state.transAxes,
    bbox=dict(facecolor="white", edgecolor="black", boxstyle="round,pad=0.5"))
conf_text  = ax_state.text(0.5, 0.66, "Confidence: --", ha="center",
    va="center", fontsize=14, transform=ax_state.transAxes)
ax_state.text(0.5, 0.95, "Host inference (this PC's model)",
              ha="center", va="top", fontsize=11, color="#222",
              fontweight="bold", transform=ax_state.transAxes)

bar_axes = []
for i, cls in enumerate(CLASSES):
    y = 0.50 - i*0.12
    ax_state.text(0.05, y+0.03, cls.upper(), fontsize=11,
                  transform=ax_state.transAxes, va="center")
    bg = plt.Rectangle((0.05, y-0.02), 0.90, 0.08, transform=ax_state.transAxes,
                       color="#e0e0e0", zorder=1)
    fg = plt.Rectangle((0.05, y-0.02), 0.00, 0.08, transform=ax_state.transAxes,
                       color="steelblue", zorder=2)
    ax_state.add_patch(bg); ax_state.add_patch(fg)
    bar_axes.append(fg)

# ─────────────────────────────────────────────────────────────────────
# Pi-side group panels — one stack per group, rendered from #pred
# lines. Layout: stacked below the host-inference panel, sharing the
# right column. Each group keeps its own dict of {class: Rectangle}
# so the update loop can set widths in O(1) without rebuilding the
# patches every frame.
# ─────────────────────────────────────────────────────────────────────
PI_GROUPS = ["right_arm", "left_arm"]
pi_panels = {}        # group_name → dict of widgets we mutate per frame

def build_pi_panel(group_name, y_top):
    """Render a small classifier-bar panel under the host one. y_top is
    the upper edge (in axes coords); each panel is 0.30 tall."""
    title = ax_state.text(0.05, y_top, f"Pi [{group_name}]",
                          fontsize=11, fontweight="bold",
                          color="#444", transform=ax_state.transAxes,
                          va="top")
    state = ax_state.text(0.50, y_top - 0.04,
                          "—", ha="center", va="top",
                          fontsize=14, fontweight="bold",
                          color="#222", transform=ax_state.transAxes)
    bars = {}
    labels = {}
    n = max(1, len(CLASSES))
    bar_h  = 0.04
    bar_gap = 0.005
    start_y = y_top - 0.10
    for i, cls in enumerate(CLASSES):
        y = start_y - i * (bar_h + bar_gap)
        labels[cls] = ax_state.text(0.05, y + bar_h/2, cls.upper(),
                                     fontsize=9, va="center",
                                     transform=ax_state.transAxes)
        bg = plt.Rectangle((0.30, y), 0.65, bar_h,
                           transform=ax_state.transAxes,
                           color="#e8e8e8", zorder=1)
        fg = plt.Rectangle((0.30, y), 0.0,  bar_h,
                           transform=ax_state.transAxes,
                           color="#888", zorder=2)
        ax_state.add_patch(bg); ax_state.add_patch(fg)
        bars[cls] = fg
    return {"title": title, "state": state, "bars": bars, "labels": labels}

# Stacked vertically — host panel keeps the top, Pi panels below.
# y_top values picked so the three panels share the right column
# without overlap. If you ever switch to a single-group setup, just
# pop one entry from PI_GROUPS.
pi_panels["right_arm"] = build_pi_panel("right_arm", y_top=0.46)
pi_panels["left_arm"]  = build_pi_panel("left_arm",  y_top=0.21)


# ============================================================================
# MAIN LOOP
# ============================================================================
new_samples       = 0
pred_history      = deque(maxlen=3)
stable_prediction = "WAITING..."

# Per-group Pi-side predictions parsed from "#pred,..." UART lines.
# Shape: {group_name: {"gesture": str, "conf": float, "classes": {name: prob}}}
# Refreshed every time a new "#pred,..." line arrives (~5 Hz per group).
pi_predictions = {}

def _parse_pred_line(line):
    """Parse a "#pred,ts,group,gesture,conf,cls:p,cls:p,..." line into
    the pi_predictions dict. Silently drops malformed lines."""
    try:
        parts = line[len("#pred,"):].split(",")
        if len(parts) < 4:
            return
        _ts, group, gesture, conf = parts[0], parts[1], parts[2], parts[3]
        entry = {
            "gesture": gesture,
            "conf":    float(conf),
            "classes": {},
        }
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


def update(_frame):
    """Drain serial, plot, predict, and update widgets."""
    global new_samples, stable_prediction, last_valid
    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("#pred,"):
                _parse_pred_line(line)
                continue
            if line.startswith("#"):                 # other comment lines
                continue
            parts = line.split(",")
            if len(parts) < CHANNELS_STREAMED:
                continue
            try:
                raw = [int(parts[ch - 1]) for ch in SELECTED_CHANNELS]
            except ValueError:
                continue
            vals = [validate_sample(raw[i], last_valid[i])
                    for i in range(NUM_CHANNELS)]
            last_valid = vals
            for i in range(NUM_CHANNELS):
                buffers[i].append(vals[i])
            new_samples += 1

        for i, ln in enumerate(lines):
            ln.set_ydata(buffers[i])

        if new_samples >= PREDICT_EVERY:
            new_samples = 0
            feats = []
            for i in range(NUM_CHANNELS):
                win = np.array(list(buffers[i])[-WINDOW_SIZE:]) - 2048
                clean, env = process_live(win)
                feats.extend(extract_features(clean, env))

            X     = np.array(feats).reshape(1, -1)
            probs = model.predict_proba(scaler.transform(X))[0]
            top   = int(np.argmax(probs))
            raw_p = CLASSES[top]
            conf  = float(probs[top])

            # Debounce (same logic as predictX.py)
            pred_history.append(raw_p)
            if raw_p == "rest":
                stable_prediction = "rest"
                pred_history.clear()
                pred_history.extend(["rest"]*3)
            elif len(pred_history) == 3 and len(set(pred_history)) == 1:
                stable_prediction = raw_p

            bg = CLASS_COLORS.get(stable_prediction, "yellow")
            state_text.set_text(stable_prediction.upper())
            state_text.set_bbox(dict(facecolor=bg, edgecolor="black",
                                     boxstyle="round,pad=0.5"))
            conf_text.set_text(f"Confidence: {conf:.1%}")

        # Update Pi-side group panels from latest #pred lines. Each
        # group has its own four bars + a top-line state label. If
        # there's no data yet, we just show "—".
        for grp_name, panel in pi_panels.items():
            p = pi_predictions.get(grp_name)
            if not p:
                panel["state"].set_text("—")
                for cls, bar in panel["bars"].items():
                    bar.set_width(0.0)
                continue
            panel["state"].set_text(
                f"{p['gesture'].upper()}  {int(p['conf']*100)}%")
            for cls, bar in panel["bars"].items():
                v = p["classes"].get(cls, 0.0)
                if v < 0.0: v = 0.0
                if v > 1.0: v = 1.0
                bar.set_width(0.65 * v)
                # highlight the winning bar
                bar.set_facecolor("#2c7a2c" if cls == p["gesture"] else "#888")
            for i, cls in enumerate(CLASSES):
                bar_axes[i].set_width(0.90 * probs[i])
                bar_axes[i].set_facecolor(
                    bg if cls == stable_prediction else "steelblue")
    except Exception:
        pass
    return lines


ani = animation.FuncAnimation(fig, update, interval=20, blit=False,
                              cache_frame_data=False)
plt.tight_layout()
try:
    plt.show()
finally:
    ser.close()
