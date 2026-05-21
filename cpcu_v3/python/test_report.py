#!/usr/bin/env python3
"""test_report.py — comprehensive documentation-grade system test.

ROLE
    Produces a defence-ready test report for the CPCU prosthetic-arm
    system by running three measurement phases live against the
    running kernel, then writing the consolidated numbers to a
    timestamped Markdown + JSON report under log/.

    Unlike test/system_test.py (which is the quick PASS/FAIL
    SYS-REQ checker for CI), this script is operator-interactive
    and runs ~15 minutes by default. The kernel + DSP must already
    be up — start them with `./launch.sh tui` in a separate
    terminal before launching this.

PHASES
    Phase 1 — System stability (50 s default)
        5 × 10 s captures. For each window:
          - Mean / max packet-to-servo latency (ms)
          - Packet rate (pkt/s)
          - Packet loss rate (%, derived from io_seq_gaps)
          - Sequence gap count
        The operator should remain at rest during Phase 1 so the
        per-channel envelope RMS we capture here is also usable as
        the SNR noise floor for Phase 3.

    Phase 2 — Per-gesture max-confidence (long, 5×10s prep + 10s
              active per gesture)
        For every gesture in every gesture group from gestures.json:
          - 10 s prep countdown ("Get ready to perform <GESTURE>")
          - 10 s active recording window (operator activates the
            muscle group; the script records the MAX of the
            corresponding class probability seen in that window)
        5 reps per gesture. Reports min/mean/max/std across reps.

    Phase 3 — Per-channel SNR
        Uses rest-period envelope RMS from Phase 1 as the noise
        floor, and the peak envelope RMS observed during each
        gesture's active windows in Phase 2 as the signal. Reports:
          SNR_dB = 20 * log10(signal_rms_peak / noise_rms_mean)
        per (group, gesture, channel).

DEPENDENCIES
    cpcu_kernel + cpcu_dsp running (writes IPC + group state file).
    config/gestures.json    : gesture group + class enumeration.
    /tmp/cpcu_group_state.txt : per-group state digest written by
                                cpcu_dsp.py every inference tick.
    python/cpcu_ipc_bridge.py : SHM mapping.

OUTPUTS
    log/test_report_<YYYYmmdd_HHMMSS>.md  — defence-ready Markdown
    log/test_report_<YYYYmmdd_HHMMSS>.json — machine-readable data

EXIT CODES
    0 — completed (no requirement asserted; this is a measurement
        tool, not a pass/fail gate)
    1 — operator aborted with Ctrl+C
    2 — system not ready (kernel/dsp not running, IPC not mapped)

CLI
    --phase1-runs N        Phase 1 reps                 (default 5)
    --phase1-duration SEC  Phase 1 each-run duration    (default 10)
    --break SEC            Break between phases         (default 30)
    --reps N               Phase 2 reps per gesture     (default 5)
    --prep SEC             Phase 2 prep time per rep    (default 10)
    --active SEC           Phase 2 active recording sec (default 10)
    --no-phase2            Skip per-gesture testing
    --no-snr               Skip SNR calculation
    --gestures NAMES       Comma-list of gestures to test (default: all)
    --groups NAMES         Comma-list of groups to test (default: all)
    --output-dir DIR       Where to write reports       (default log/)
    --quiet                Less verbose console output
"""

import argparse
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime


# ─────────────────────────────────────────────────────────────────────
#  PATH BOOTSTRAP
# ─────────────────────────────────────────────────────────────────────
HERE = os.path.dirname(os.path.abspath(__file__))
# Try dev-tree paths first, fall back to installed.
for p in (os.path.join(HERE, '..', 'python'),
          os.path.join(HERE, 'python'),
          '/opt/cpcu/python'):
    if os.path.isdir(p):
        sys.path.insert(0, p)

try:
    from cpcu_ipc_bridge import IPCBridge
except ImportError as e:
    print(f"ERROR: cannot import cpcu_ipc_bridge ({e}).")
    print("       Run from the repo root or after `./launch.sh build`.")
    sys.exit(2)


# ─────────────────────────────────────────────────────────────────────
#  CONFIG LOOKUP
# ─────────────────────────────────────────────────────────────────────
def find_gestures_json():
    for p in (os.path.join(HERE, '..', 'config', 'gestures.json'),
              os.path.join(HERE, 'config', 'gestures.json'),
              '/opt/cpcu/config/gestures.json'):
        if os.path.isfile(p):
            return p
    return None


def load_gestures():
    path = find_gestures_json()
    if not path:
        return {}
    try:
        with open(path) as f:
            data = json.load(f)
        return data.get('gesture_groups', {})
    except Exception as e:
        print(f"WARNING: couldn't load gestures.json ({e}); using empty set.")
        return {}


# ─────────────────────────────────────────────────────────────────────
#  GROUP STATE DIGEST  (one line per group, written by cpcu_dsp.py)
# ─────────────────────────────────────────────────────────────────────
GROUP_STATE_PATH = "/tmp/cpcu_group_state.txt"


def read_group_state():
    """Return {group_name: {'state': str, 'conf_pct': int,
                            'classes': {name: pct, ...}}}."""
    out = {}
    try:
        with open(GROUP_STATE_PATH) as f:
            for ln in f:
                ln = ln.rstrip('\n')
                if not ln:
                    continue
                parts = ln.split('\t')
                if len(parts) < 4:
                    continue
                name, state, conf_pct, classes_str = parts[:4]
                cls_dict = {}
                for tok in classes_str.split(','):
                    if ':' in tok:
                        k, v = tok.split(':', 1)
                        try:
                            cls_dict[k] = int(v)
                        except ValueError:
                            pass
                try:
                    conf = int(conf_pct)
                except ValueError:
                    conf = 0
                out[name] = {'state': state, 'conf_pct': conf,
                             'classes': cls_dict}
    except FileNotFoundError:
        pass
    except Exception:
        # corrupted file is fine — caller treats empty dict as
        # "DSP hasn't written yet"
        pass
    return out


# ─────────────────────────────────────────────────────────────────────
#  PRESENTATION HELPERS
# ─────────────────────────────────────────────────────────────────────
ANSI = {
    'reset':  "\033[0m",
    'bold':   "\033[1m",
    'dim':    "\033[2m",
    'red':    "\033[31m",
    'green':  "\033[32m",
    'yellow': "\033[33m",
    'blue':   "\033[34m",
    'cyan':   "\033[36m",
    'gray':   "\033[90m",
}


def c(text, color):
    return f"{ANSI.get(color, '')}{text}{ANSI['reset']}"


def banner(text, color='cyan'):
    bar = "═" * (len(text) + 4)
    print()
    print(c(bar, color))
    print(c(f"  {text}  ", color))
    print(c(bar, color))


def countdown(seconds, prompt, quiet=False):
    """Print a one-line countdown that updates in place. Returns
    cleanly even if user runs without a TTY (quiet mode just prints
    start/end)."""
    if quiet:
        print(f"  [{seconds}s] {prompt}")
        time.sleep(seconds)
        return
    start = time.monotonic()
    while True:
        elapsed   = time.monotonic() - start
        remaining = seconds - elapsed
        if remaining <= 0:
            print(f"\r  {' '*60}\r", end='', flush=True)
            return
        # Live update every 0.2 s
        print(f"\r  {c(f'[{int(math.ceil(remaining)):3d}s]', 'yellow')} "
              f"{prompt}",
              end='', flush=True)
        time.sleep(0.2)


# ─────────────────────────────────────────────────────────────────────
#  PHASE 1 — System stability
# ─────────────────────────────────────────────────────────────────────
def capture_window(ipc, duration_s, sample_interval_s=0.05):
    """Capture latency / packet / channel-RMS samples for one window.
    Returns dict of measurements + lists.
    """
    out = {
        'latency_ms':       [],
        'seq_age_pkts':     [],
        'channel_rms':      [[] for _ in range(8)],
    }

    pkts_start  = ipc.read_diag_pkts_received()
    gaps_start  = ipc.read_diag_seq_gaps()
    infer_start = ipc.read_diag_dsp_inferences()
    ovf_start   = ipc.read_diag_ring_overflows()

    t0 = time.monotonic()
    tick_count = 0
    while time.monotonic() - t0 < duration_s:
        time.sleep(sample_interval_s)
        try:
            lat = ipc.read_latency_pkt()
            if lat['pkt_to_servo_us'] > 0:
                out['latency_ms'].append(lat['pkt_to_servo_us'] / 1000.0)
            if lat['seq_age'] > 0:
                out['seq_age_pkts'].append(lat['seq_age'])
        except Exception:
            pass
        # Per-channel envelope RMS (snapshot from DSPExport).
        try:
            for ch in range(8):
                v = ipc.read_dsp_channel_rms(ch) \
                    if hasattr(ipc, 'read_dsp_channel_rms') else None
                if v is None:
                    # Fallback — try the raw read pattern.
                    v = _read_channel_rms_fallback(ipc, ch)
                if v is not None and v > 0:
                    out['channel_rms'][ch].append(float(v))
        except Exception:
            pass
        tick_count += 1

    pkts_end  = ipc.read_diag_pkts_received()
    gaps_end  = ipc.read_diag_seq_gaps()
    infer_end = ipc.read_diag_dsp_inferences()
    ovf_end   = ipc.read_diag_ring_overflows()

    elapsed = time.monotonic() - t0
    pkts    = pkts_end  - pkts_start
    gaps    = gaps_end  - gaps_start
    infers  = infer_end - infer_start

    return {
        'duration_s':        elapsed,
        'pkts':              pkts,
        'gaps':              gaps,
        'inferences':        infers,
        'ring_overflows':    ovf_end - ovf_start,
        'pkt_rate_hz':       pkts / elapsed if elapsed > 0 else 0,
        'pkt_loss_pct':      (100.0 * gaps / pkts) if pkts > 0 else 0.0,
        'infer_rate_hz':     infers / elapsed if elapsed > 0 else 0,
        'latency_ms':        out['latency_ms'],
        'seq_age_pkts':      out['seq_age_pkts'],
        'channel_rms':       out['channel_rms'],
    }


def _read_channel_rms_fallback(ipc, ch):
    """Read channel_rms[ch] from DSPExport via direct offset arithmetic.
    The IPCBridge may not expose a typed helper — this gets us the
    same bytes from the same offset."""
    try:
        # DSPExport region begins at OFF_EXPORT in cpcu_ipc_bridge.
        # channel_rms is the first field (float32 × 8).
        import struct
        # Look up offsets defensively — they were imported at module
        # load. If absent, we just give up.
        from cpcu_ipc_bridge import OFF_EXPORT
        b = ipc.mm[OFF_EXPORT + ch * 4 : OFF_EXPORT + (ch + 1) * 4]
        return struct.unpack('<f', b)[0]
    except Exception:
        return None


def summarise_window(name, w):
    """Format a single window's stats for terminal output."""
    lat = w['latency_ms']
    if lat:
        lat_mean = statistics.mean(lat)
        lat_max  = max(lat)
        lat_min  = min(lat)
    else:
        lat_mean = lat_max = lat_min = 0.0
    return {
        'name':         name,
        'duration_s':   w['duration_s'],
        'pkt_rate_hz':  w['pkt_rate_hz'],
        'pkt_loss_pct': w['pkt_loss_pct'],
        'gaps':         w['gaps'],
        'lat_mean_ms':  lat_mean,
        'lat_max_ms':   lat_max,
        'lat_min_ms':   lat_min,
        'infer_rate':   w['infer_rate_hz'],
        'overflows':    w['ring_overflows'],
        'channel_rms':  w['channel_rms'],
    }


def run_phase1(ipc, n_runs, duration, quiet=False):
    banner(f"PHASE 1 — System stability ({n_runs}× {duration}s)", 'cyan')
    print()
    print("  Operator: please remain AT REST for this phase.")
    print("  This captures latency, packet loss, data rate, and the")
    print("  noise floor used for the SNR calculation later.")
    print()

    rest_rms_acc = [[] for _ in range(8)]
    runs = []

    for i in range(1, n_runs + 1):
        print(c(f"  Run {i}/{n_runs} ", 'bold') +
              c(f"(capturing {duration}s)", 'gray'))
        w = capture_window(ipc, duration)
        s = summarise_window(f"run{i}", w)
        runs.append(s)
        # Accumulate envelope RMS for rest baseline.
        for ch in range(8):
            rest_rms_acc[ch].extend(w['channel_rms'][ch])
        print(f"    pkt_rate={s['pkt_rate_hz']:6.1f} pkt/s   "
              f"loss={s['pkt_loss_pct']:5.2f}%   "
              f"gaps={s['gaps']:4d}   "
              f"lat_max={s['lat_max_ms']:5.1f} ms   "
              f"lat_mean={s['lat_mean_ms']:5.1f} ms")
        if i < n_runs:
            time.sleep(0.5)

    # Aggregate
    agg = {
        'pkt_rate_hz_mean':  statistics.mean(r['pkt_rate_hz'] for r in runs),
        'pkt_loss_pct_mean': statistics.mean(r['pkt_loss_pct'] for r in runs),
        'pkt_loss_pct_max':  max(r['pkt_loss_pct'] for r in runs),
        'lat_max_ms_max':    max(r['lat_max_ms']  for r in runs),
        'lat_mean_ms_mean':  statistics.mean(r['lat_mean_ms'] for r in runs),
        'gaps_total':        sum(r['gaps']        for r in runs),
        'overflows_total':   sum(r['overflows']   for r in runs),
        'infer_rate_mean':   statistics.mean(r['infer_rate'] for r in runs),
    }
    print()
    print(c("  Aggregate over all runs:", 'bold'))
    print(f"    Packet rate (mean) : {agg['pkt_rate_hz_mean']:7.1f} pkt/s")
    print(f"    Packet loss (mean) : {agg['pkt_loss_pct_mean']:7.2f} %")
    print(f"    Packet loss (worst): {agg['pkt_loss_pct_max']:7.2f} %")
    print(f"    Latency max (worst): {agg['lat_max_ms_max']:7.1f} ms")
    print(f"    Latency mean (avg) : {agg['lat_mean_ms_mean']:7.1f} ms")
    print(f"    Seq gaps (total)   : {agg['gaps_total']:7d}")
    print(f"    Ring overflows     : {agg['overflows_total']:7d}")
    print(f"    Inference rate     : {agg['infer_rate_mean']:7.1f} Hz")

    # Rest-RMS baseline for SNR (per channel: mean of all collected
    # samples — gives a stable noise-floor estimate)
    baseline = []
    for ch in range(8):
        if rest_rms_acc[ch]:
            baseline.append(statistics.mean(rest_rms_acc[ch]))
        else:
            baseline.append(0.0)

    return {'runs': runs, 'aggregate': agg, 'rest_rms_baseline': baseline}


# ─────────────────────────────────────────────────────────────────────
#  PHASE 2 — Per-gesture confidence
# ─────────────────────────────────────────────────────────────────────
def run_phase2(ipc, groups, reps, prep_s, active_s,
               gesture_filter=None, group_filter=None, quiet=False):
    banner(f"PHASE 2 — Per-gesture confidence ({reps} reps each)", 'cyan')
    print()
    print(f"  For each gesture: {prep_s}s prep -> {active_s}s active recording.")
    print(f"  Activate the target muscle group cleanly during the")
    print(f"  recording window. The script logs the MAX confidence")
    print(f"  reached for the matching class.")
    print()

    results = {}   # results[group][gesture] = list of rep dicts

    for group_name, group in groups.items():
        if group_filter and group_name not in group_filter:
            continue
        print(c(f"\n  ── Group: {group_name} ──", 'blue'))
        gestures = group.get('gestures', {})
        for gname in gestures:
            if gesture_filter and gname not in gesture_filter:
                continue
            print(c(f"\n  Gesture: {group_name}/{gname}", 'bold'))
            per_rep = []
            for r in range(1, reps + 1):
                # ── Prep ──
                countdown(prep_s,
                          c(f"Rep {r}/{reps}: GET READY to perform "
                            f"{group_name}/{gname.upper()}", 'yellow'),
                          quiet)
                # ── Active ──
                print(c(f"  [REC]  rep {r}/{reps}  performing "
                        f"{gname.upper()}  ...", 'green'),
                      flush=True)
                rep = capture_gesture_rep(ipc, group_name, gname,
                                          active_s, quiet)
                per_rep.append(rep)
                print(f"         max_conf={rep['max_conf_pct']:3d}%   "
                      f"mean_conf={rep['mean_conf_pct']:5.1f}%   "
                      f"samples={rep['n_samples']}   "
                      f"ch_rms_peak={max(rep['ch_rms_peak'] or [0]):.4f}")
            results.setdefault(group_name, {})[gname] = per_rep

    # Per-gesture summary
    print()
    print(c("  Per-gesture summary:", 'bold'))
    for group_name, by_g in results.items():
        for gname, reps_list in by_g.items():
            maxes = [r['max_conf_pct'] for r in reps_list]
            if maxes:
                print(f"    {group_name}/{gname:6s}  "
                      f"max-conf  min={min(maxes):3d}%  "
                      f"mean={statistics.mean(maxes):5.1f}%  "
                      f"max={max(maxes):3d}%  "
                      f"stdev={statistics.pstdev(maxes):4.1f}")

    return results


def capture_gesture_rep(ipc, group_name, gesture_name, active_s, quiet):
    """Capture one active window. Polls /tmp/cpcu_group_state.txt at
    20 Hz (50 ms) and records the per-window MAX confidence for the
    target class. Also captures peak channel-RMS per channel.
    """
    confs       = []     # int %s for the target class
    ch_rms_peak = [0.0] * 8
    t0          = time.monotonic()
    last_print  = 0.0
    while time.monotonic() - t0 < active_s:
        time.sleep(0.05)
        # Per-class confidence from the group-state digest
        state = read_group_state()
        g     = state.get(group_name)
        if g:
            cls = g['classes']
            # Match by exact class name (case-insensitive fallback)
            v = cls.get(gesture_name)
            if v is None:
                # cpcu_dsp may write lowercase class names; the
                # gesture_name in gestures.json is also lowercase
                # but just in case, try case-insensitive
                for k, vv in cls.items():
                    if k.lower() == gesture_name.lower():
                        v = vv
                        break
            if v is not None:
                confs.append(int(v))
        # Peak channel RMS
        for ch in range(8):
            try:
                v = ipc.read_dsp_channel_rms(ch) \
                    if hasattr(ipc, 'read_dsp_channel_rms') else \
                    _read_channel_rms_fallback(ipc, ch)
                if v is not None and v > ch_rms_peak[ch]:
                    ch_rms_peak[ch] = float(v)
            except Exception:
                pass
        # Live readout (overwrite same line) every 0.5 s
        if not quiet and time.monotonic() - last_print > 0.5:
            now_conf = confs[-1] if confs else 0
            elapsed  = time.monotonic() - t0
            remain   = active_s - elapsed
            bar = ('█' * int(now_conf / 5)).ljust(20, '░')
            print(f"\r         {c(f'[{int(math.ceil(remain)):2d}s]', 'green')} "
                  f"live conf: {now_conf:3d}%  |{bar}|",
                  end='', flush=True)
            last_print = time.monotonic()
    if not quiet:
        print(f"\r         {' '*60}\r", end='', flush=True)

    if confs:
        return {
            'n_samples':     len(confs),
            'max_conf_pct':  max(confs),
            'mean_conf_pct': statistics.mean(confs),
            'min_conf_pct':  min(confs),
            'ch_rms_peak':   ch_rms_peak,
        }
    else:
        return {
            'n_samples':     0,
            'max_conf_pct':  0,
            'mean_conf_pct': 0.0,
            'min_conf_pct':  0,
            'ch_rms_peak':   ch_rms_peak,
        }


# ─────────────────────────────────────────────────────────────────────
#  PHASE 3 — Per-channel SNR
# ─────────────────────────────────────────────────────────────────────
def compute_snr(phase1, phase2):
    """SNR_dB = 20 * log10(signal_rms_peak / noise_rms_mean).

    Noise floor = mean per-channel envelope RMS during Phase 1
                  (operator at rest). One value per channel.
    Signal     = peak per-channel envelope RMS observed during each
                  active window of each gesture rep. Take the MEAN
                  of those peaks across the 5 reps (so a single
                  noisy rep doesn't dominate).
    """
    baseline = phase1['rest_rms_baseline']        # [8]
    snr = {}      # snr[group][gesture] = list[8] of dB (or None per ch)
    for group_name, by_g in phase2.items():
        snr[group_name] = {}
        for gname, reps_list in by_g.items():
            per_ch_peaks = [[] for _ in range(8)]
            for rep in reps_list:
                for ch, peak in enumerate(rep.get('ch_rms_peak', [])):
                    if peak > 0:
                        per_ch_peaks[ch].append(peak)
            per_ch_snr = []
            for ch in range(8):
                noise = baseline[ch]
                if noise <= 0 or not per_ch_peaks[ch]:
                    per_ch_snr.append(None)
                    continue
                signal = statistics.mean(per_ch_peaks[ch])
                if signal <= noise:
                    per_ch_snr.append(0.0)
                else:
                    per_ch_snr.append(20.0 * math.log10(signal / noise))
            snr[group_name][gname] = per_ch_snr
    return snr


def print_snr_table(snr, baseline, channel_names):
    banner("PHASE 3 — Per-channel SNR", 'cyan')
    print()
    print("  SNR_dB = 20 * log10(signal_RMS_peak / noise_RMS_mean).")
    print("  Noise floor = mean envelope RMS during Phase 1 (rest).")
    print("  Signal      = mean of per-rep peak envelope RMS in Phase 2.")
    print()
    print(c("  Noise floor (rest baseline) per channel:", 'bold'))
    for ch, v in enumerate(baseline):
        nm = channel_names[ch] if ch < len(channel_names) else f"ch{ch}"
        if v > 0:
            print(f"    ch{ch} ({nm:10s})  RMS = {v:.5f}")
        else:
            print(f"    ch{ch} ({nm:10s})  RMS = (no data)")
    print()
    print(c("  Per-gesture SNR (dB) per channel:", 'bold'))
    if not snr:
        print("    (no gesture data — Phase 2 may have been skipped)")
        return
    # Header
    hdr_chs = "  ".join(f"ch{ch:>1}" for ch in range(8))
    print(f"    {'group/gesture':<20s}  {hdr_chs}")
    print(f"    {'-' * 20}  {'-' * (4*8 + 2*7)}")
    for group_name, by_g in snr.items():
        for gname, vals in by_g.items():
            row = []
            for v in vals:
                row.append(f"{v:5.1f}" if v is not None else "  --")
            print(f"    {group_name+'/'+gname:<20s}  {'  '.join(row)}")
    print()
    print(c("  Interpretation:", 'gray'))
    print(c("    < 6 dB    : muscle barely above noise — unreliable.", 'gray'))
    print(c("    6-12 dB   : weak but classifiable signal.", 'gray'))
    print(c("    12-20 dB  : healthy EMG.", 'gray'))
    print(c("    > 20 dB   : strong activation; very reliable.", 'gray'))


# ─────────────────────────────────────────────────────────────────────
#  REPORT WRITER (Markdown + JSON)
# ─────────────────────────────────────────────────────────────────────
def write_markdown_report(path, phase1, phase2, snr,
                          channel_names, args, gestures_meta):
    with open(path, 'w') as f:
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        f.write(f"# CPCU System Test Report\n\n")
        f.write(f"**Generated:** {ts}  \n")
        f.write(f"**Host:** {os.uname().nodename}  \n")
        f.write(f"**Operator:** {os.environ.get('CPCU_OPERATOR', 'default')}  \n")
        f.write(f"**Test mode:** test-report (comprehensive)  \n")

        # ── Phase 1
        f.write("\n## Phase 1 — System Stability\n\n")
        f.write(f"Captured {len(phase1['runs'])} runs of "
                f"{args.phase1_duration} s each.\n\n")
        f.write("| Run | Pkt rate (pkt/s) | Pkt loss (%) | Seq gaps | "
                "Lat min (ms) | Lat mean (ms) | Lat max (ms) | Inferences |\n")
        f.write("|-----|------------------:|--------------:|----------:|"
                "--------------:|---------------:|--------------:|------------:|\n")
        for r in phase1['runs']:
            f.write(f"| {r['name']} | {r['pkt_rate_hz']:.1f} | "
                    f"{r['pkt_loss_pct']:.2f} | {r['gaps']} | "
                    f"{r['lat_min_ms']:.1f} | {r['lat_mean_ms']:.1f} | "
                    f"{r['lat_max_ms']:.1f} | {r['infer_rate']*r['duration_s']:.0f} |\n")
        agg = phase1['aggregate']
        f.write("\n**Aggregate:**\n\n")
        f.write(f"- Mean packet rate: **{agg['pkt_rate_hz_mean']:.1f} pkt/s**\n")
        f.write(f"- Mean packet loss: **{agg['pkt_loss_pct_mean']:.2f}%** "
                f"(worst single run: {agg['pkt_loss_pct_max']:.2f}%)\n")
        f.write(f"- Mean latency: **{agg['lat_mean_ms_mean']:.1f} ms** "
                f"(worst single observation: {agg['lat_max_ms_max']:.1f} ms)\n")
        f.write(f"- Total sequence gaps: **{agg['gaps_total']}**\n")
        f.write(f"- Total ring overflows: **{agg['overflows_total']}**\n")
        f.write(f"- Mean inference rate: **{agg['infer_rate_mean']:.1f} Hz**\n")

        # ── Phase 2
        f.write("\n## Phase 2 — Per-Gesture Confidence\n\n")
        if phase2:
            f.write(f"Reps per gesture: **{args.reps}**, "
                    f"prep: **{args.prep} s**, "
                    f"active recording: **{args.active} s**.\n\n")
            for group_name, by_g in phase2.items():
                f.write(f"\n### Group `{group_name}`\n\n")
                f.write("| Gesture | Min conf (%) | Mean conf (%) | "
                        "Max conf (%) | StDev | Reps |\n")
                f.write("|---------|--------------:|---------------:|"
                        "--------------:|------:|------:|\n")
                for gname, reps_list in by_g.items():
                    maxes = [r['max_conf_pct'] for r in reps_list]
                    if not maxes:
                        continue
                    f.write(f"| {gname} | {min(maxes)} | "
                            f"{statistics.mean(maxes):.1f} | "
                            f"{max(maxes)} | "
                            f"{statistics.pstdev(maxes):.1f} | "
                            f"{len(maxes)} |\n")
                # Per-rep detail
                f.write("\n<details><summary>Per-rep detail</summary>\n\n")
                for gname, reps_list in by_g.items():
                    f.write(f"\n**{gname}**\n\n")
                    f.write("| Rep | n_samples | min | mean | max |\n")
                    f.write("|-----|----------:|----:|-----:|----:|\n")
                    for i, r in enumerate(reps_list, start=1):
                        f.write(f"| {i} | {r['n_samples']} | "
                                f"{r['min_conf_pct']} | "
                                f"{r['mean_conf_pct']:.1f} | "
                                f"{r['max_conf_pct']} |\n")
                f.write("\n</details>\n")
        else:
            f.write("_Skipped via `--no-phase2`._\n")

        # ── Phase 3
        f.write("\n## Phase 3 — Per-Channel SNR\n\n")
        if snr:
            f.write("**SNR_dB = 20 × log10(signal_RMS_peak / noise_RMS_mean)**\n\n")
            f.write("Noise floor = mean envelope RMS during Phase 1 (operator at rest).  \n")
            f.write("Signal = mean across reps of per-rep peak envelope RMS during Phase 2 actives.\n\n")
            # Baseline table
            f.write("**Rest-baseline RMS per channel:**\n\n")
            f.write("| Channel | Name | RMS |\n|---|---|---:|\n")
            for ch, v in enumerate(phase1['rest_rms_baseline']):
                nm = channel_names[ch] if ch < len(channel_names) else f"ch{ch}"
                f.write(f"| ch{ch} | {nm} | {v:.5f} |\n")
            # SNR table
            f.write("\n**SNR per gesture per channel (dB):**\n\n")
            chs = " | ".join(f"ch{ch}" for ch in range(8))
            seps = "---:|" * 8
            f.write(f"| Group/Gesture | {chs} |\n")
            f.write(f"|---|{seps}\n")
            for group_name, by_g in snr.items():
                for gname, vals in by_g.items():
                    row = " | ".join(
                        f"{v:.1f}" if v is not None else "—"
                        for v in vals)
                    f.write(f"| {group_name}/{gname} | {row} |\n")
            f.write("\n**Interpretation:**\n\n")
            f.write("- `< 6 dB`: muscle barely above noise — unreliable.\n")
            f.write("- `6–12 dB`: weak but classifiable signal.\n")
            f.write("- `12–20 dB`: healthy EMG.\n")
            f.write("- `> 20 dB`: strong activation; very reliable.\n")
        else:
            f.write("_Skipped or unavailable._\n")

        # ── Settings appendix
        f.write("\n## Test Settings\n\n")
        f.write("```\n")
        for k, v in sorted(vars(args).items()):
            f.write(f"  {k:25s} {v}\n")
        f.write("```\n")


def write_json_report(path, phase1, phase2, snr, args):
    payload = {
        'timestamp':     datetime.now().isoformat(timespec='seconds'),
        'host':          os.uname().nodename,
        'operator':      os.environ.get('CPCU_OPERATOR', 'default'),
        'settings':      vars(args),
        'phase1': {
            'runs':       [
                {k: v for k, v in r.items() if k != 'channel_rms'}
                for r in phase1['runs']
            ],
            'aggregate':  phase1['aggregate'],
            'rest_rms_baseline': phase1['rest_rms_baseline'],
        },
        'phase2':        phase2,
        'snr_db':        snr,
    }
    with open(path, 'w') as f:
        json.dump(payload, f, indent=2, default=float)


# ─────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="Comprehensive CPCU system test report.",
        formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--phase1-runs',     type=int,   default=5)
    p.add_argument('--phase1-duration', type=int,   default=10)
    p.add_argument('--break',           type=int,   default=30,
                   dest='break_s')
    p.add_argument('--reps',            type=int,   default=5)
    p.add_argument('--prep',            type=int,   default=10)
    p.add_argument('--active',          type=int,   default=10)
    p.add_argument('--no-phase2',       action='store_true')
    p.add_argument('--no-snr',          action='store_true')
    p.add_argument('--gestures',        type=str,   default='',
                   help="comma-list of gesture names to test (default: all)")
    p.add_argument('--groups',          type=str,   default='',
                   help="comma-list of groups to test (default: all)")
    p.add_argument('--output-dir',      type=str,   default='')
    p.add_argument('--quiet',           action='store_true')
    return p.parse_args()


def main():
    args = parse_args()

    # IPC ready?
    try:
        ipc = IPCBridge()
    except Exception as e:
        print(c(f"ERROR: cannot map IPC ({e}).", 'red'))
        print("       The kernel must be running. Start it with:")
        print("         ./launch.sh tui   (in a separate terminal)")
        sys.exit(2)

    # Friendly readiness check
    try:
        st = ipc.read_system_state()
        if st != 1:
            print(c(f"  warning: system_state={st} (expected 1=RUNNING). "
                    f"Press Ctrl+C to abort, or any sample now.", 'yellow'))
    except Exception:
        pass

    groups   = load_gestures()
    if not groups and not args.no_phase2:
        print(c("WARNING: gestures.json had no gesture_groups; "
                "Phase 2 skipped.", 'yellow'))
    gesture_filter = set(args.gestures.split(',')) if args.gestures else None
    group_filter   = set(args.groups.split(','))   if args.groups   else None

    # Channel names from gestures.json (per arm), best-effort.
    channel_names = []
    for gname, group in groups.items():
        for ch in group.get('emg_channels', {}).get('active', []):
            # Use the names array if present, otherwise "chN".
            names = group.get('emg_channels', {}).get('names', [])
            idx = group.get('emg_channels', {}).get('active', []).index(ch)
            label = names[idx] if idx < len(names) else f"ch{ch}"
            while len(channel_names) <= ch:
                channel_names.append('')
            channel_names[ch] = label
    while len(channel_names) < 8:
        channel_names.append(f"ch{len(channel_names)}")

    banner("CPCU TEST REPORT — comprehensive", 'bold')
    print()
    print(f"  Reports go to: {args.output_dir or 'log/'}/")
    print(f"  Operator     : {os.environ.get('CPCU_OPERATOR', 'default')}")
    print(f"  Estimated total time: "
          f"~{args.phase1_runs*args.phase1_duration + args.break_s + (0 if args.no_phase2 else len(groups)*sum(len(g.get('gestures',{})) for g in groups.values())*args.reps*(args.prep+args.active)//max(1,len(groups)))}s")
    print()

    try:
        # Phase 1
        phase1 = run_phase1(ipc, args.phase1_runs,
                            args.phase1_duration, quiet=args.quiet)

        # Break
        if not args.no_phase2:
            banner(f"BREAK — {args.break_s} s to prepare for gesture tests",
                   'yellow')
            countdown(args.break_s,
                      "stretch, hydrate, get into position",
                      quiet=args.quiet)

        # Phase 2
        if args.no_phase2:
            phase2 = {}
        else:
            phase2 = run_phase2(ipc, groups, args.reps, args.prep,
                                args.active, gesture_filter,
                                group_filter, quiet=args.quiet)

        # Phase 3
        snr = {} if args.no_snr else compute_snr(phase1, phase2)
        if not args.no_snr:
            print_snr_table(snr, phase1['rest_rms_baseline'], channel_names)

    except KeyboardInterrupt:
        print()
        print(c("ABORTED by operator (Ctrl+C).", 'yellow'))
        sys.exit(1)

    # ── Write report files ──
    out_dir = args.output_dir or os.path.join(HERE, '..', 'log')
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    md_path   = os.path.join(out_dir, f"test_report_{stamp}.md")
    json_path = os.path.join(out_dir, f"test_report_{stamp}.json")

    try:
        write_markdown_report(md_path, phase1, phase2, snr,
                              channel_names, args, groups)
        print()
        print(c(f"  ✓ Markdown report saved: {md_path}", 'green'))
    except Exception as e:
        print(c(f"  WARNING: failed to write Markdown ({e})", 'red'))

    try:
        write_json_report(json_path, phase1, phase2, snr, args)
        print(c(f"  ✓ JSON report saved    : {json_path}", 'green'))
    except Exception as e:
        print(c(f"  WARNING: failed to write JSON ({e})", 'red'))

    print()
    sys.exit(0)


if __name__ == "__main__":
    main()
