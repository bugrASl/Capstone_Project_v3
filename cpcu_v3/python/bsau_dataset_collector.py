#!/usr/bin/env python3
"""
bsau_dataset_collector.py -- Collect BSAU UART dataset files.

Pairs with BSAU_MODE_DATASET firmware. The BSAU transmits one CSV line
per millisecond on USART1 at 921600 baud, 8N1:

    c0,c1,c2,c3,c4,c5,c6,c7\r\n

where c0..c7 are the raw 12-bit ADC readings of scan 0 from each packet
(i.e. 1000 samples/s/channel). This script writes them to a file named
{label}_{N}.csv, auto-incrementing N so nothing overwrites.

The CPCU-side TUI produces a structurally identical file (same 8 columns,
no metadata, no header) from the over-the-air path, with optional
Butterworth bandpass + 50 Hz notch pre-applied. A matching pair of files
lets the DSP/AI team compare raw-from-wire vs filtered-from-CPCU without
reconciling schemas.

Usage:
    python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label rest
    python3 bsau_dataset_collector.py --port COM10 --label biceps_flex --output ./datasets
    python3 bsau_dataset_collector.py --port /dev/ttyACM0 --label rest --live   # + matplotlib

    Ctrl+C stops collection. The file is closed cleanly and a summary
    prints (duration, line count, gap count, line rate).

Requirements:
    pip install pyserial
    pip install matplotlib    # only if you pass --live
"""

import argparse
import os
import re
import serial
import signal
import sys
import time
from pathlib import Path

DEFAULT_BAUD = 921600
EXPECTED_COLS = 8
# At 1 kHz CSV rate, expect ~1000 lines/sec. Anything below 800 for more
# than a second is a sign the UART is throttled (radio interference,
# buffer backpressure from a slow collector).
UNDERRUN_THRESHOLD = 800


# --- Helpers ------------------------------------------------------------

def next_file_index(output_dir: Path, label: str) -> int:
    """Scan output_dir for {label}_*.csv files and return next available N."""
    pattern = re.compile(rf"^{re.escape(label)}_(\d+)\.csv$")
    max_seen = -1
    if output_dir.exists():
        for entry in output_dir.iterdir():
            m = pattern.match(entry.name)
            if m:
                n = int(m.group(1))
                if n > max_seen:
                    max_seen = n
    return max_seen + 1


def sanitize_label(label: str) -> str:
    """Match the CPCU-side label sanitiser so filenames line up."""
    out = []
    for c in label:
        if c == ".":
            out.append("_")
        elif c == "<":
            out.append("_LT")
        elif c == ">":
            out.append("_GT")
        elif c == "=":
            out.append("_EQ")
        elif c.isalnum() or c == "_" or c == "-":
            out.append(c)
    return "".join(out) or "unlabeled"


class LiveViewer:
    """Optional matplotlib 8-channel rolling scope. Disabled unless --live."""

    def __init__(self, window_size=1000):
        import matplotlib.pyplot as plt
        from collections import deque

        self.plt = plt
        self.deques = [deque([0] * window_size, maxlen=window_size) for _ in range(8)]
        self.fig, self.axes = plt.subplots(4, 2, figsize=(10, 8))
        self.axes = self.axes.flatten()
        self.lines = []
        for i, ax in enumerate(self.axes):
            (line,) = ax.plot(self.deques[i])
            ax.set_ylim(0, 4095)
            ax.set_title(f"ch{i}", fontsize=8)
            ax.tick_params(labelsize=6)
            self.lines.append(line)
        self.fig.suptitle("BSAU live (raw ADC, 1 kHz)", fontsize=12)
        plt.tight_layout()
        plt.show(block=False)
        self._last_draw = 0.0

    def push(self, channels):
        for i, v in enumerate(channels):
            self.deques[i].append(v)

    def maybe_redraw(self):
        now = time.monotonic()
        if now - self._last_draw < 0.05:   # 20 Hz max
            return
        self._last_draw = now
        for line, dq in zip(self.lines, self.deques):
            line.set_ydata(dq)
        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            pass   # window probably closed; ignore, keep collecting


# --- Main collector loop -------------------------------------------------

def collect(port: str, baud: int, out_path: Path, live: bool):
    """
    Open the serial port, stream CSV lines to out_path until SIGINT.
    Returns a dict with duration, line count, gap count, throughput.
    """
    t_open = time.monotonic()

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        print(f"[collector] FATAL: cannot open {port}: {e}", file=sys.stderr)
        sys.exit(2)

    # Drain whatever was queued before we started. The firmware has been
    # streaming since boot, so the first few lines will be stale.
    time.sleep(0.1)
    ser.reset_input_buffer()

    running = {"ok": True}

    def _on_sig(signum, frame):
        running["ok"] = False
    signal.signal(signal.SIGINT, _on_sig)

    viewer = LiveViewer() if live else None

    out_path.parent.mkdir(parents=True, exist_ok=True)
    total_lines = 0
    bad_lines = 0
    t_start = time.monotonic()
    t_last_print = t_start

    print(f"[collector] Writing to {out_path}")
    print("[collector] Press Ctrl+C to stop.")

    with open(out_path, "w", newline="") as f:
        while running["ok"]:
            try:
                line = ser.readline().decode("ascii", errors="replace").strip()
            except Exception as e:
                print(f"[collector] serial read failed: {e}", file=sys.stderr)
                break
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != EXPECTED_COLS:
                bad_lines += 1
                continue

            try:
                vals = [int(p) for p in parts]
            except ValueError:
                bad_lines += 1
                continue

            # Validate each value is a plausible 12-bit ADC reading.
            # A non-integer sneaking through would already have raised
            # above; what we catch here is partial lines caused by a
            # UART frame break or sudden buffer flush.
            if any(v < 0 or v > 4095 for v in vals):
                bad_lines += 1
                continue

            # Write the line verbatim (same format as incoming) so the
            # CPCU file and this file are byte-compatible for any loader
            # that takes 8 comma-separated ints.
            f.write(line + "\r\n")
            total_lines += 1

            if viewer is not None:
                viewer.push(vals)
                viewer.maybe_redraw()

            now = time.monotonic()
            if now - t_last_print >= 1.0:
                elapsed = now - t_start
                rate = total_lines / elapsed if elapsed > 0 else 0
                warn = " [UNDERRUN]" if rate < UNDERRUN_THRESHOLD and elapsed > 2 else ""
                print(f"\r[collector] lines={total_lines} rate={rate:6.1f}/s "
                      f"bad={bad_lines}{warn}", end="", flush=True)
                t_last_print = now

        # Flush everything before the OS kills us.
        try:
            f.flush()
            os.fsync(f.fileno())
        except Exception:
            pass

    ser.close()
    t_end = time.monotonic()
    elapsed = t_end - t_start
    rate = total_lines / elapsed if elapsed > 0 else 0.0
    print()   # newline after the progress line

    return {
        "duration_s": elapsed,
        "lines":      total_lines,
        "bad_lines":  bad_lines,
        "rate_hz":    rate,
        "port_open":  t_end - t_open,
    }


# --- Entry point --------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Collect BSAU UART dataset (BSAU_MODE_DATASET firmware).",
        epilog="Each invocation produces ONE file. Re-run with a new label "
               "or a new capture by interrupting and restarting.",
    )
    ap.add_argument("--port",    required=True, help="serial port (COM10 / /dev/ttyACM0)")
    ap.add_argument("--baud",    type=int, default=DEFAULT_BAUD,
                    help=f"baud rate (default {DEFAULT_BAUD})")
    ap.add_argument("--label",   required=True,
                    help="gesture label, e.g. rest, biceps_flex, hand_open. "
                         "Becomes the filename stem.")
    ap.add_argument("--output",  default="./datasets",
                    help="output directory (default ./datasets)")
    ap.add_argument("--live",    action="store_true",
                    help="open a live 8-channel matplotlib scope (requires matplotlib)")
    args = ap.parse_args()

    label = sanitize_label(args.label)
    if label != args.label:
        print(f"[collector] label sanitised: '{args.label}' -> '{label}'")

    out_dir = Path(args.output).expanduser().resolve()
    idx = next_file_index(out_dir, label)
    out_file = out_dir / f"{label}_{idx}.csv"

    print(f"[collector] port     = {args.port}")
    print(f"[collector] baud     = {args.baud}")
    print(f"[collector] label    = {label}")
    print(f"[collector] out file = {out_file}")
    if args.live:
        print(f"[collector] live view enabled (slows collection ~5-10%)")

    summary = collect(args.port, args.baud, out_file, args.live)

    print()
    print(f"[collector] === SUMMARY ===")
    print(f"[collector] file      : {out_file}")
    print(f"[collector] duration  : {summary['duration_s']:.2f} s")
    print(f"[collector] lines     : {summary['lines']}")
    print(f"[collector] bad lines : {summary['bad_lines']}")
    print(f"[collector] rate      : {summary['rate_hz']:.1f} lines/s "
          f"(expected ~1000)")

    if summary["rate_hz"] < UNDERRUN_THRESHOLD and summary["duration_s"] > 2:
        print(f"[collector] WARN: throughput below {UNDERRUN_THRESHOLD} lines/s "
              f"-> UART underrun (firmware behind, or host I/O slow).")
    if summary["bad_lines"] > 0:
        frac = summary["bad_lines"] / max(1, summary["bad_lines"] + summary["lines"])
        print(f"[collector] WARN: {summary['bad_lines']} bad/partial lines "
              f"({frac*100:.3f}%) - check baud and cable integrity.")


if __name__ == "__main__":
    main()
