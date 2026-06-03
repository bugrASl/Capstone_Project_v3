#!/usr/bin/env python3
"""monitor.py — CPCU host-side UART dashboard.

ROLE
    Host-PC visualisation + verification tool for the BSAU/CPCU
    stack. Reads the 1 kHz raw EMG stream from the Pi over UART,
    plots six waveforms, and runs the SAME RandomForest classifier
    the Pi runs (models/arm.pkl) so the operator can compare host
    inference against the Pi's inference on identical samples. The
    Pi side overlays its own predictions via `#pred,...` lines.

DEPENDENCIES — what this script READS
    Pi side (cpcu_dsp.py) producing two interleaved UART streams:
        raw CSV     "v0,v1,v2,v3,v4,v5\\n"                    @ 1 kHz
        predictions "#pred,ts,group,gesture,conf,cls:p,..."   per window
    models/arm.pkl                                            joblib-saved
                                                              {"model","scaler"}

CROSS-MODULE EFFECTS
    - Filter cascade below MIRRORS cpcu_dsp.py::process_window
      (decimate -> bandpass 20-450 Hz -> notch 50/100/200 Hz ->
      envelope LP 3 Hz). Edit cutoffs in one without the other and
      the host/Pi predictions stop agreeing.
    - SELECTED_CHANNELS = [1..6] mirrors the BSAU board's first
      six ADCs (ch7/ch8 unused on v3 BSAU). Re-mapping requires a
      matching change in BSAU firmware and in gestures.json's
      emg_channels assignment.
    - The classifier reads `model.classes_` for ordering, so a
      retrain that produces a different class set (count or names)
      is picked up automatically here — but the four class colours
      in CLASS_COLORS below may need a new entry.

CLI
    --port /dev/ttyUSB0     (Windows default: COM5)
    --baud 921600
    --model models/arm.pkl
"""
import argparse
import sys
import threading
import time
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

# ── Serial reader pacing ─────────────────────────────────────────────
# Process at most MAX_BYTES_PER_ITER bytes per reader-loop iteration.
# Holding the GIL longer than this blocks matplotlib's redraw and
# makes the window look frozen. At 921600 baud (~92 KB/s) and 30 ms
# animation interval, ~4 KB per iteration is plenty.
MAX_BYTES_PER_ITER = 4096
# If OS buffer holds more than DRAIN_THRESHOLD bytes, the link
# clearly stalled and recovered with a burst. The ring is only 1 s
# of data anyway — drop the oldest bytes instead of trying to read
# them all (which would just freeze the UI for hundreds of ms).
DRAIN_THRESHOLD    = 16384

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
# MODEL — same arm.pkl drives both groups (shared muscle set).
# Loaded ONCE; aliases below let the inference block address the two
# arms symmetrically without duplicating ~5 MB of trees in memory.
# ─────────────────────────────────────────────────────────────────────
def load_model(path):
    try:
        ckpt = joblib.load(path)
        print(f"[monitor] loaded model: {path}")
        return ckpt["model"], ckpt["scaler"]
    except Exception as e:
        print(f"[monitor] failed to load {path}: {e}")
        sys.exit(1)


model, scaler = load_model(args.model)
model_R, scaler_R = model, scaler
model_L, scaler_L = model, scaler
CLASSES_R = list(model.classes_)
CLASSES_L = list(model.classes_)

# Print the exact class order so we can verify it matches what the
# AI team trained on. If the model says "wrong gesture" (e.g. you
# flex biceps and it shows hand), the order here may not match what
# you expect — check against your training script.
print(f"[monitor] classes (in model order): {CLASSES_R}")
print(f"[monitor] ch1-3 → ARM 1 (R)   ch4-6 → ARM 2 (L)")
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
pi_pred_ts     = {}   # {group: monotonic time of last #pred line}
pi_pred_lock   = threading.Lock()   # guards both dicts above
PI_STALE_SEC   = 2.0  # show "stream stalled" if no #pred for this long

# ── Reader-side stats for the on-screen status line ──────────────────
# Updated by serial_reader, read by the matplotlib update() callback.
# Both happen in the same process so a normal threading.Lock is fine.
# Mutating the inner dict is atomic enough under the GIL for our use
# (we never iterate it from the consumer side without holding the
# lock), but the lock keeps it future-proof if more fields are added.
reader_stats = {
    'total_samples':  0,      # cumulative valid samples committed
    'last_sample_ts': 0.0,    # monotonic time of last successful commit
    'dropped_bytes':  0,      # bytes drained from the OS buffer to avoid hang
    'last_error':     '',     # short string of last serial exception
    'last_error_ts':  0.0,    # monotonic time it happened
}
reader_stats_lock = threading.Lock()
DATA_STALE_SEC = 0.5         # raw stream is "live" if a sample arrived this recently

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
        # Write both dicts atomically. The reader thread is the sole
        # writer; the animate-thread (which calls get_pi_pred) holds
        # the same lock for an O(1) snapshot, so the inner classes
        # dict never gets iterated while it's being mutated.
        with pi_pred_lock:
            pi_predictions[group] = entry
            pi_pred_ts[group]     = time.monotonic()
    except Exception:
        pass


def get_pi_pred(group):
    """Snapshot a group's last prediction (returns (entry|None, stale_bool))."""
    with pi_pred_lock:
        entry = pi_predictions.get(group)
        ts    = pi_pred_ts.get(group, 0.0)
    if not entry:
        return None, True
    return entry, (time.monotonic() - ts > PI_STALE_SEC)


def serial_reader():
    global ring_idx, new_samples_count
    leftover       = b""
    sel            = SELECTED_CHANNELS
    last_err_print = 0.0
    while not stop_event.is_set():
        try:
            n = ser.in_waiting
            if n == 0:
                stop_event.wait(SERIAL_POLL_MS / 1000.0)
                continue
            # ── Bounded read ─────────────────────────────────────────
            # On flaky USB-UART links (Windows, FTDI, CP210x with power
            # save), in_waiting occasionally spikes to tens of kilobytes
            # all at once after a stall. Processing the whole backlog
            # in one loop iteration starves matplotlib of GIL time and
            # makes the window appear frozen ("hangs like disconnected").
            #
            # We cap each read at MAX_BYTES_PER_ITER. If the OS buffer
            # holds more than that, we yield back to the main loop and
            # come back for the rest on the next tick — animation gets
            # to redraw between iterations.
            if n > MAX_BYTES_PER_ITER:
                # Also: if the backlog is enormous (> DRAIN_THRESHOLD),
                # there's no point reading the oldest bytes — the ring
                # only holds PLOT_WINDOW samples (1 second @ 1 kHz) and
                # we'd just overwrite what we just read. Drop the old
                # bytes, keep the recent tail. Bump the dropped counter
                # so the dashboard shows when this happens.
                if n > DRAIN_THRESHOLD:
                    ser.read(n - DRAIN_THRESHOLD)  # discard oldest
                    with reader_stats_lock:
                        reader_stats['dropped_bytes'] += (n - DRAIN_THRESHOLD)
                    n = DRAIN_THRESHOLD
                n = MAX_BYTES_PER_ITER
            data = leftover + ser.read(n)
            if b"\n" not in data:
                leftover = data
                continue
            chunks   = data.split(b"\n")
            leftover = chunks[-1]
            # Batch all parsed samples into a local list; acquire
            # ring_lock ONCE for the whole batch instead of once per
            # sample. Cuts lock-acquire overhead from ~1000/s to
            # ~50-100/s while reading the same data.
            batch = []
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
                # Parse + validate atomically — see notes in earlier
                # version: the goal is "never partially-commit". A
                # malformed line that aborts mid-parse must NOT poison
                # last_valid or push half-junk to the ring.
                new_vals = [None] * NUM_CHANNELS
                ok = True
                for i, ch in enumerate(sel):
                    try:
                        v = int(parts[ch - 1])
                    except (ValueError, IndexError):
                        ok = False; break
                    lv = last_valid[i]
                    if v < MIN_ADC_VALUE or v > MAX_ADC_VALUE:
                        # Out of ADC range → bad sample, hold last.
                        v = int(lv)
                    elif abs(v - lv) > MAX_DELTA:
                        # Spike filter. Hold last value but ALSO bump
                        # last_valid HALFWAY toward the new sample, so
                        # if the underlying signal genuinely shifted
                        # (e.g., operator just flexed hard) we don't
                        # get permanently stuck at the old baseline.
                        v = int(lv + (v - lv) // 2)
                    new_vals[i] = v
                if not ok:
                    continue
                # Commit to last_valid here (single-threaded path).
                for i in range(NUM_CHANNELS):
                    last_valid[i] = new_vals[i]
                batch.append(new_vals)
            # ── Single ring-lock acquire for the whole batch ─────────
            if batch:
                with ring_lock:
                    for new_vals in batch:
                        for i in range(NUM_CHANNELS):
                            ring_buffer[i, ring_idx] = new_vals[i]
                        ring_idx           = (ring_idx + 1) % PLOT_WINDOW
                        new_samples_count += 1
                # Stats outside the ring lock so animate thread isn't
                # waiting on it.
                with reader_stats_lock:
                    reader_stats['total_samples'] += len(batch)
                    reader_stats['last_sample_ts'] = time.monotonic()
        except Exception as e:
            # Surface USB-disconnect / permission / decode errors. The
            # dashboard's status line also displays this; stderr keeps
            # the historical log.
            now = time.monotonic()
            if now - last_err_print > 2.0:
                print(f"[monitor] serial error: {e}", file=sys.stderr)
                last_err_print = now
            with reader_stats_lock:
                reader_stats['last_error']    = str(e)[:60]
                reader_stats['last_error_ts'] = time.monotonic()
            stop_event.wait(0.05)


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
fig = plt.figure(figsize=(16, 9), facecolor="#fafafa")
fig.suptitle("CPCU UART Monitor — dual-arm classifier  (R: ch1-3   L: ch4-6)",
             fontsize=15, fontweight="bold", color="#222222")

# Top-of-figure status line. Shows live data rate, time since the last
# sample, dropped-bytes counter, and the most recent serial error.
# Whenever this turns red, the dashboard is "frozen" because the data
# isn't flowing — never again silently confused about why a plot is
# flat. Updated by update() from reader_stats.
#
# IMPORTANT — must live on an AXES, not on the figure itself.
# matplotlib >= 3.7's blit machinery does {a.axes for a in artists}
# and calls ax._get_view() on each. A fig.text(...) artist has
# axes=None, which crashes blit with:
#   AttributeError: 'NoneType' object has no attribute '_get_view'
# So we add a thin invisible axes at the top strip just for hosting
# this text. The position is outside the GridSpec — figure-absolute
# coordinates, sits between the suptitle (~y=0.98) and the plots
# (gridspec top=0.92).
_status_ax = fig.add_axes([0.05, 0.945, 0.90, 0.025], frameon=False)
_status_ax.set_xticks([]); _status_ax.set_yticks([])
_status_ax.set_xlim(0, 1); _status_ax.set_ylim(0, 1)
_status_ax.axis("off")
status_line = _status_ax.text(0.5, 0.5, "starting up…",
                              ha="center", va="center",
                              fontsize=10, color="#444444",
                              family="monospace", animated=True)

# ── Unified layout via GridSpec ──────────────────────────────────────
# The old layout mixed `fig.add_subplot(3, 3, ...)` (for EMG) with
# `fig.add_subplot(2, 3, ...)` (for the status panels). Mixing row
# counts that aren't multiples (3 vs 2) makes tight_layout silently
# no-op with the warning:
#   "tight_layout not applied: number of rows in subplot specifications
#    must be multiples of one another."
# When that happens matplotlib falls back to rcParams defaults, and on
# some window sizes / DPI scales the EMG subplots get squeezed off the
# visible area.
#
# Fix: a single 6-row × 3-col grid. EMG plots span 2 rows each (so
# they're the same visible size as before), status panels span 3 rows
# each. Same grid → tight_layout works → layout is deterministic.
from matplotlib.gridspec import GridSpec
gs = GridSpec(6, 3, figure=fig,
              left=0.06, right=0.97, top=0.92, bottom=0.06,
              wspace=0.25, hspace=0.55)

# EMG waveform grid: left two columns × 3 stacked plots (each 2 rows
# of the 6-row grid). Order matches LABELS below.
sig_axes = [
    fig.add_subplot(gs[0:2, 0]),   # R_Hand
    fig.add_subplot(gs[2:4, 0]),   # R_Biceps
    fig.add_subplot(gs[4:6, 0]),   # R_Triceps
    fig.add_subplot(gs[0:2, 1]),   # L_Hand
    fig.add_subplot(gs[2:4, 1]),   # L_Biceps
    fig.add_subplot(gs[4:6, 1]),   # L_Triceps
]
PLOT_COLORS = ["#2266cc", "#cc3333", "#2a8a2a",
               "#e08a1f", "#7a3fb8", "#8a5a2a"]
LABELS      = ["R_Hand (ch1)",   "R_Biceps (ch2)", "R_Triceps (ch3)",
               "L_Hand (ch4)",   "L_Biceps (ch5)", "L_Triceps (ch6)"]
x_axis      = np.arange(PLOT_POINTS)
initial_y   = np.full(PLOT_POINTS, 2048.0)
lines = []
for i, ax in enumerate(sig_axes):
    # Centre baseline at 2048 (12-bit ADC midpoint) — gives the eye a
    # reference for "rest" vs "active" without staring at the y-tick.
    # zorder=0 keeps it BEHIND the data line.
    ax.axhline(2048, color="#cccccc", linewidth=0.8,
               linestyle="--", zorder=0)
    # animated=True is REQUIRED for FuncAnimation(blit=True): the line
    # is excluded from the cached static background and re-blitted on
    # every frame from update()'s return value. Without it, the line
    # ends up in the static cache and set_ydata changes never repaint.
    line, = ax.plot(x_axis, initial_y, color=PLOT_COLORS[i],
                    animated=True, linewidth=1.1, zorder=2)
    ax.set_ylim(0, 4500)
    ax.set_xlim(0, PLOT_POINTS - 1)
    ax.set_title(LABELS[i], fontsize=10, fontweight="bold",
                 color=PLOT_COLORS[i], loc="left", pad=3)
    ax.tick_params(axis="both", labelsize=7, colors="#666666")
    ax.grid(True, alpha=0.25)
    for spine in ax.spines.values():
        spine.set_color("#cccccc")
    lines.append(line)

# Status panels: right column, two stacked panels each 3 rows tall.
ax_R = fig.add_subplot(gs[0:3, 2])
ax_L = fig.add_subplot(gs[3:6, 2])
for ax in (ax_R, ax_L):
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")


# ── Status-panel layout constants (axes-relative units) ──────────────
# These were inline magic numbers in the old setup; pulling them out
# here makes a future redesign one place to edit instead of two.
PANEL_BORDER_PAD   = 0.02     # margin between panel content and ax edge
TITLE_Y            = 0.93     # panel title baseline
HOST_PILL_Y        = 0.78     # big HOST: PRED pill (center)
HOST_CONF_Y        = 0.64     # confidence text (center)
PI_OVERLAY_Y       = 0.55     # Pi-side mirror text (center)
BAR_ROW_TOP_Y      = 0.42     # FIRST class-row center
BAR_ROW_DY         = 0.09     # vertical spacing between class rows
BAR_HEIGHT         = 0.055    # bar rect height
LABEL_X            = 0.07     # class label left edge
BAR_X              = 0.30     # bar column left edge
BAR_W              = 0.50     # bar column width
PCT_X              = 0.92     # percentage right edge (right-aligned)

BAR_BG_COLOR       = "#dddddd"
BAR_INACTIVE_COLOR = "#9aa6b2"
PANEL_BORDER_COLOR = "#888888"


def setup_status_panel(ax, title, classes):
    # ── Visible panel frame ──
    border = plt.Rectangle((PANEL_BORDER_PAD, PANEL_BORDER_PAD),
                           1.0 - 2 * PANEL_BORDER_PAD,
                           1.0 - 2 * PANEL_BORDER_PAD,
                           transform=ax.transAxes,
                           fill=False,
                           edgecolor=PANEL_BORDER_COLOR,
                           linewidth=1.2, zorder=0)
    ax.add_patch(border)

    # ── Panel title ──
    ax.text(0.5, TITLE_Y, title,
            ha="center", va="center",
            fontsize=12, fontweight="bold",
            color="#222222",
            transform=ax.transAxes)

    # ── Big HOST pill ──
    host_state = ax.text(0.5, HOST_PILL_Y, "HOST: —",
                         ha="center", va="center",
                         fontsize=15, fontweight="bold",
                         transform=ax.transAxes, animated=True,
                         bbox=dict(facecolor="white",
                                   edgecolor="#444444",
                                   linewidth=1.0,
                                   boxstyle="round,pad=0.5"))

    host_conf = ax.text(0.5, HOST_CONF_Y, "Confidence: --",
                        ha="center", va="center",
                        fontsize=10, color="#333333",
                        transform=ax.transAxes, animated=True)

    pi_state = ax.text(0.5, PI_OVERLAY_Y, "Pi: (no #pred yet)",
                       ha="center", va="center",
                       fontsize=9, color="#777777",
                       family="monospace",
                       transform=ax.transAxes, animated=True)

    # ── Class bars: aligned row layout ──
    #   LABEL (left)  |  BAR (middle)  |  PCT (right)
    # All three placed at the SAME y_center so they read as one row.
    bar_axes = []
    pct_axes = []
    for i, cls in enumerate(classes):
        y_center = BAR_ROW_TOP_Y - i * BAR_ROW_DY
        y_bar    = y_center - BAR_HEIGHT / 2

        # Label (left column, fixed-width-ish via monospace)
        ax.text(LABEL_X, y_center, cls.upper(),
                fontsize=10, fontweight="bold",
                ha="left", va="center",
                color="#222222",
                family="monospace",
                transform=ax.transAxes)

        # Bar background
        bg = plt.Rectangle((BAR_X, y_bar), BAR_W, BAR_HEIGHT,
                           transform=ax.transAxes,
                           color=BAR_BG_COLOR, zorder=1)
        # Bar foreground (animated, width = BAR_W * prob)
        fg = plt.Rectangle((BAR_X, y_bar), 0.0, BAR_HEIGHT,
                           transform=ax.transAxes,
                           color=BAR_INACTIVE_COLOR,
                           zorder=2, animated=True)
        ax.add_patch(bg); ax.add_patch(fg)
        bar_axes.append(fg)

        # Percentage (right column, right-aligned)
        pct = ax.text(PCT_X, y_center, "  0%",
                      ha="right", va="center",
                      fontsize=10, color="#333333",
                      family="monospace",
                      transform=ax.transAxes, animated=True)
        pct_axes.append(pct)

    return host_state, host_conf, pi_state, bar_axes, pct_axes


# Single space in the title — proportional font means the old double
# space didn't actually align with the line above. tight_layout +
# ha="center" handles centering.
hs_R, hc_R, pi_R, bars_R, pcts_R = setup_status_panel(
    ax_R, "ARM 1 — RIGHT (ch1-3)", CLASSES_R)
hs_L, hc_L, pi_L, bars_L, pcts_L = setup_status_panel(
    ax_L, "ARM 2 — LEFT (ch4-6)",  CLASSES_L)

animated = (list(lines)
            + [status_line, hs_R, hc_R, pi_R, hs_L, hc_L, pi_L]
            + bars_R + bars_L
            + pcts_R + pcts_L)


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


def update_panel(group_name, hs, hc, pi_t, bars, pcts,
                 conf, probs, classes, stable):
    bg_color = CLASS_COLORS.get(stable, "#ffe066")

    # Big HOST pill (top of panel)
    hs.set_text(f"HOST: {stable.upper()}")
    hs.get_bbox_patch().set_facecolor(bg_color)

    hc.set_text(f"Confidence: {conf:.1%}")

    # Each class row: bar width = probability, label colour =
    # highlight if active, percentage shown numerically.
    for i, cls in enumerate(classes):
        bars[i].set_width(BAR_W * probs[i])
        bars[i].set_facecolor(bg_color if cls == stable
                              else BAR_INACTIVE_COLOR)
        pcts[i].set_text(f"{int(probs[i] * 100):3d}%")

    # Pi-side overlay. get_pi_pred returns (entry, stale_flag) under
    # the lock; we never iterate p["classes"] while the reader thread
    # might be mutating it.
    p, stale = get_pi_pred(group_name)
    if p and not stale:
        cls_str = " ".join(f"{c}:{int(v * 100):02d}"
                           for c, v in p["classes"].items())
        # Two lines: headline (gesture + conf) above the per-class
        # breakdown. Single-line tended to overflow the panel width
        # on long class lists.
        pi_t.set_text(f"Pi → {p['gesture'].upper()} "
                      f"{int(p['conf'] * 100)}%\n"
                      f"[{cls_str}]")
        pi_t.set_color("#444444")
    elif p and stale:
        pi_t.set_text("Pi: (stream stalled)")
        pi_t.set_color("#cc4444")
    else:
        pi_t.set_text("Pi: (no #pred yet)")
        pi_t.set_color("#999999")


# Rolling 1-second sample-rate tracker. We snapshot total_samples
# once per second and report the delta as "samples/s". Window kept
# short so the readout reacts within ~1 s of a stall.
_rate_window_ts      = [time.monotonic()]
_rate_window_samples = [0]
_RATE_WINDOW_LEN     = 5      # 5 × ~1 s = 5 s of history

def _compute_status_text():
    """Build the top-of-figure status string from reader_stats."""
    now = time.monotonic()
    with reader_stats_lock:
        total      = reader_stats['total_samples']
        last_ts    = reader_stats['last_sample_ts']
        dropped    = reader_stats['dropped_bytes']
        last_err   = reader_stats['last_error']
        last_err_t = reader_stats['last_error_ts']
    # Rolling samples/sec
    if now - _rate_window_ts[-1] >= 1.0:
        _rate_window_ts.append(now)
        _rate_window_samples.append(total)
        if len(_rate_window_ts) > _RATE_WINDOW_LEN:
            _rate_window_ts.pop(0)
            _rate_window_samples.pop(0)
    if len(_rate_window_samples) >= 2:
        d_samples = _rate_window_samples[-1] - _rate_window_samples[0]
        d_time    = _rate_window_ts[-1]      - _rate_window_ts[0]
        rate_hz   = (d_samples / d_time) if d_time > 0 else 0.0
    else:
        rate_hz = 0.0

    if last_ts == 0.0:
        age_s   = float('inf')
        age_str = "  no data yet"
    else:
        age_s   = now - last_ts
        age_str = f"  age:{age_s*1000:5.0f}ms"

    rate_str    = f"rate:{rate_hz:4.0f} Hz"
    dropped_str = f"  dropped:{dropped // 1024:4d} KB"
    err_str     = (f"  err: {last_err}"
                   if last_err and now - last_err_t < 5.0 else "")

    # Colour rule: red if no recent data, amber if dropped bytes
    # recently, otherwise grey.
    if age_s > DATA_STALE_SEC:
        color = "#cc4444"     # stalled
    elif dropped > 0 and rate_hz > 100:
        color = "#cc8800"     # data flowing but we had to drain
    else:
        color = "#444444"     # healthy

    return f"{rate_str}{age_str}{dropped_str}{err_str}", color


def update(frame):
    global new_samples_count, stable_R, stable_L

    snap = get_snapshot()
    display = snap[:, ::PLOT_DECIMATION] if PLOT_DECIMATION > 1 else snap
    for i, line in enumerate(lines):
        line.set_ydata(display[i])

    # Update the top-of-figure status line every frame.
    txt, col = _compute_status_text()
    status_line.set_text(txt)
    status_line.set_color(col)

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
        update_panel("right_arm", hs_R, hc_R, pi_R, bars_R, pcts_R,
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
        update_panel("left_arm", hs_L, hc_L, pi_L, bars_L, pcts_L,
                     conf_L, probs_L, CLASSES_L, stable_L)

    return animated


anim = animation.FuncAnimation(fig, update,
                               interval=ANIM_INTERVAL_MS,
                               blit=True,
                               cache_frame_data=False)

try:
    # GridSpec already sets explicit margins; we don't need (and don't
    # want) tight_layout, which used to silently no-op with a warning
    # because the old layout mixed 3-row and 2-row subplot grids.
    plt.show()
finally:
    stop_event.set()
    reader_thread.join(timeout=1.0)
    ser.close()
