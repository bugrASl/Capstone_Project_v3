#!/usr/bin/env python3
"""uart_monitor.py — receive CPCU debug data on host PC via USB-UART adapter.

Reads CSV lines from the Pi's UART and displays them as a live table.
Run on your laptop/PC, not on the Pi.

Format from cpcu_dsp.py:
    timestamp_ms, gesture, confidence, rms0, var0, wl0, env0, rms1, ...

Usage:
    python3 uart_monitor.py --port /dev/ttyUSB0
    python3 uart_monitor.py --port COM3
    python3 uart_monitor.py --port /dev/ttyUSB0 --log data.csv
    python3 uart_monitor.py --port /dev/ttyUSB0 --raw
"""
import argparse
import signal
import sys
import time

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description="CPCU UART debug monitor")
    ap.add_argument("--port", required=True, help="Serial port (COM3, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", help="Save raw CSV to file")
    ap.add_argument("--raw", action="store_true", help="Print raw lines (no formatting)")
    args = ap.parse_args()

    running = True
    def stop(sig, _):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Can't open {args.port}: {e}")
        sys.exit(1)

    print(f"Listening on {args.port} @ {args.baud}...")
    print("Press Ctrl+C to stop.\n")

    logfile = open(args.log, 'w') if args.log else None
    count = 0
    last_gesture = ""

    try:
        while running:
            line = ser.readline()
            if not line:
                continue

            try:
                text = line.decode('ascii', errors='replace').strip()
            except Exception:
                continue

            if not text:
                continue

            if logfile:
                logfile.write(text + '\n')

            count += 1

            if args.raw:
                print(text)
                continue

            # parse CSV: ts,gesture,confidence,f0,f1,...
            parts = text.split(',')
            if len(parts) < 3:
                print(f"[?] {text}")
                continue

            ts = parts[0]
            gesture = parts[1]
            conf = parts[2]

            # detect transition
            marker = ""
            if gesture != last_gesture:
                marker = " ◄ TRANSITION"
                last_gesture = gesture

            # compact display
            feats = parts[3:7] if len(parts) > 6 else parts[3:]
            feat_str = " ".join(f"{float(f):8.4f}" for f in feats)

            sys.stdout.write(
                f"\r  [{count:6d}] {gesture:20s} conf={conf:>5s}  "
                f"rms=[{feat_str}]{marker}       "
            )
            sys.stdout.flush()

            if marker:
                print()  # newline on transition

    finally:
        ser.close()
        if logfile:
            logfile.close()
            print(f"\nSaved {count} lines to {args.log}")
        print(f"\nReceived {count} lines.")


if __name__ == "__main__":
    main()
