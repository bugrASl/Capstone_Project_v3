#!/usr/bin/env python3
"""cpcu_calibrate.py — interactive 0-10 velocity preference tuning.

Mapping: rate = base_rate × (level / 5)²
    0  → 0%       (stopped)
    3  → 36%      (slow precision)
    5  → 100%     (base rate)
    7  → 196%     (fast)
    10 → 400%     (maximum)

Called by: ./launch.sh calibrate (via scripts/calibrate.sh)
"""
import argparse
import json
import os
import sys
from datetime import datetime


def level_to_rate(base, level):
    """Convert 0-10 scale to µs/s. Preserves sign of base."""
    sign = 1 if base >= 0 else -1
    return int(sign * abs(base) * (level / 5.0) ** 2)


def draw_bar(level, w=20):
    filled = int(w * level / 10)
    return f"[{'▓' * filled}{'░' * (w - filled)}] {level}/10"


def tune_one(gesture, servo, base):
    """Interactive prompt for one gesture+servo pair. Returns (level, rate)."""
    print(f"\n  Gesture: \"{gesture}\"  Servo: {servo}  (base={base} µs/s)")
    while True:
        try:
            raw = input("  Level 0-10 [5]: ").strip()
            level = 5 if raw == '' else int(raw)
            if 0 <= level <= 10:
                break
            print("  Range: 0-10")
        except ValueError:
            print("  Enter a number.")
        except (EOFError, KeyboardInterrupt):
            return None, None
    rate = level_to_rate(base, level)
    print(f"  {draw_bar(level)} → {rate} µs/s")
    return level, rate


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--gestures", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--operator", default="default")
    args = p.parse_args()

    with open(args.gestures) as f:
        gs = json.load(f)

    results = {}
    for gname, gdef in gs.get("gestures", {}).items():
        if gdef.get("mode") != "velocity":
            continue
        channels = gdef.get("channels", {})
        if not channels:
            continue
        results[gname] = {}
        for sname, chdef in channels.items():
            base = chdef.get("rate_us_s", 200)
            level, rate = tune_one(gname, sname, base)
            if level is None:
                return 1
            results[gname][sname] = {"level": level, "rate_us_s": rate}

    # build output, preserve existing rest thresholds if present
    out = {
        "operator": args.operator,
        "calibrated_at": datetime.now().isoformat(timespec="seconds"),
        "gesture_levels": results,
        "confidence_curve": {
            "floor_pct": gs.get("confidence", {}).get("floor_pct", 40),
            "ceil_pct": gs.get("confidence", {}).get("ceil_pct", 85)
        }
    }
    if os.path.exists(args.output):
        try:
            with open(args.output) as f:
                old = json.load(f)
            if "rest_thresholds" in old:
                out["rest_thresholds"] = old["rest_thresholds"]
        except Exception:
            pass

    with open(args.output, 'w') as f:
        json.dump(out, f, indent=4)
    print(f"\n  Saved: {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
