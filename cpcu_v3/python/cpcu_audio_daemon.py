#!/usr/bin/env python3
"""cpcu_audio_daemon.py — priority-based audio feedback.

Priority levels:
  1 = gesture transitions (highest — operator needs immediate feedback)
  2 = failures (safe state, link lost, I²C error)
  3 = system events (startup, shutdown, config reload)

Higher priority interrupts lower. Same priority queues FIFO.
"""
import heapq
import json
import os
import signal
import subprocess
import sys
import time
import wave

import numpy as np

REPO = os.environ.get("CPCU_ROOT",
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GESTURES_PATH = os.path.join(REPO, "config", "gestures.json")
AUDIO_DIR = os.path.join(REPO, "config", "audio_cues")
sys.path.insert(0, os.path.join(REPO, "python"))

POLL_HZ = 20
POLL_INTERVAL = 1.0 / POLL_HZ


def generate_tone(freq_hz, dur_ms, path):
    """Synthesize sine tone .wav."""
    sr = 22050
    n = int(sr * dur_ms / 1000)
    t = np.linspace(0, dur_ms / 1000, n, False)
    fade = min(int(sr * 0.005), n // 4)
    env = np.ones(n)
    if fade > 0:
        env[:fade] = np.linspace(0, 1, fade)
        env[-fade:] = np.linspace(1, 0, fade)
    data = (np.sin(2 * np.pi * freq_hz * t) * env * 16000).astype(np.int16)
    with wave.open(path, 'w') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(data.tobytes())


def set_volume(pct):
    try:
        subprocess.run(["amixer", "set", "PCM", f"{pct}%"],
                       capture_output=True, timeout=2)
    except Exception:
        pass


def load_config():
    """Load gestures.json. Returns (mode, gesture_cues, event_cues, volume)."""
    try:
        with open(GESTURES_PATH) as f:
            gs = json.load(f)
    except Exception as e:
        print(f"[AUDIO] config: {e}", flush=True)
        return "off", {}, {}, 80

    mode = gs.get("audio_mode", "off")
    volume = gs.get("audio_volume_pct", 80)
    os.makedirs(AUDIO_DIR, exist_ok=True)

    def resolve_cue(audio_def, name):
        """Resolve voice or freq cue path."""
        if mode == "voice":
            vname = audio_def.get("voice")
            if vname:
                wav = os.path.join(AUDIO_DIR, f"{vname}.wav")
                if os.path.exists(wav):
                    return wav
        fhz = audio_def.get("freq_hz", 0)
        fms = audio_def.get("freq_ms", 80)
        if fhz > 0:
            tp = os.path.join(AUDIO_DIR, f"_gen_{name}_{fhz}hz.wav")
            try:
                generate_tone(fhz, fms, tp)
                return tp
            except Exception:
                pass
        return None

    # gesture cues (priority 1)
    gesture_cues = {}
    for gname, gdef in gs.get("gestures", {}).items():
        audio = gdef.get("audio", {})
        path = resolve_cue(audio, gname)
        if path:
            gesture_cues[gname] = path

    # event cues (priority from JSON, default 3)
    event_cues = {}
    for ename, edef in gs.get("audio_events", {}).items():
        if ename.startswith("_"):
            continue
        path = resolve_cue(edef, ename)
        if path:
            event_cues[ename] = {
                "path": path,
                "priority": edef.get("priority", 3),
            }

    print(f"[AUDIO] mode={mode} vol={volume}% "
          f"gestures={len(gesture_cues)} events={len(event_cues)}", flush=True)
    return mode, gesture_cues, event_cues, volume


class AudioQueue:
    """Priority queue for audio playback. Lower number = higher priority."""

    def __init__(self):
        self._heap = []
        self._seq = 0
        self._playing = None  # (priority, subprocess)
        self._playing_prio = 99

    def enqueue(self, priority, path):
        heapq.heappush(self._heap, (priority, self._seq, path))
        self._seq += 1

    def tick(self):
        """Call at POLL_HZ. Manages playback."""
        # check if current sound finished
        if self._playing and self._playing.poll() is not None:
            self._playing = None
            self._playing_prio = 99

        if not self._heap:
            return

        # peek at next queued sound
        next_prio = self._heap[0][0]

        # play if nothing playing, or if higher priority interrupts
        if self._playing is None or next_prio < self._playing_prio:
            if self._playing and next_prio < self._playing_prio:
                self._playing.terminate()
                self._playing = None

            prio, _, path = heapq.heappop(self._heap)
            try:
                self._playing = subprocess.Popen(
                    ["aplay", "-q", path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self._playing_prio = prio
            except FileNotFoundError:
                self._playing = None


class EventDetector:
    """Polls IPC and detects state transitions."""

    def __init__(self, ipc):
        self.ipc = ipc
        self.gesture = ""
        self.sys_state = -1
        self.io_ready = False
        self.dsp_ready = False
        self.safe_count = 0
        self.overflow_count = 0
        self.last_pkt_time = 0
        self.link_ok = True
        self.started = False

    def poll(self):
        """Returns list of (event_name, priority) for events detected this tick."""
        events = []

        # gesture transition (priority 1)
        try:
            name = self.ipc.read_dsp_gesture_name()
            if name and name != self.gesture:
                events.append(("gesture:" + name, 1))
                self.gesture = name
        except Exception:
            pass

        # system state (priority 2 for faults, 3 for normal)
        try:
            state = self.ipc.read_system_state()
            if state != self.sys_state:
                old = self.sys_state
                self.sys_state = state
                if state == 1 and old <= 0:
                    events.append(("system_ready", 3))
                elif state == 2:
                    events.append(("safe_state_entered", 2))
                elif state == 1 and old == 2:
                    events.append(("safe_state_cleared", 2))
        except Exception:
            pass

        # IO ready
        try:
            io = bool(self.ipc.read_io_ready())
            if io and not self.io_ready:
                events.append(("io_ready", 3))
            self.io_ready = io
        except Exception:
            pass

        # DSP ready
        try:
            dsp = bool(self.ipc.read_dsp_ready())
            if dsp and not self.dsp_ready:
                events.append(("dsp_ready", 3))
            self.dsp_ready = dsp
        except Exception:
            pass

        # safe state counter (ring overflow / repeated faults)
        try:
            sc = self.ipc.read_diag_safe_entries()
            if sc > self.safe_count + 1:
                events.append(("safe_state_entered", 2))
            self.safe_count = sc
        except Exception:
            pass

        # ring overflow
        try:
            oc = self.ipc.read_diag_ring_overflows()
            if oc > self.overflow_count:
                events.append(("ring_overflow", 2))
            self.overflow_count = oc
        except Exception:
            pass

        # link lost detection (no new packets for >1s)
        try:
            pkts = self.ipc.read_diag_pkts_received()
            now = time.monotonic()
            if pkts > 0:
                if self.last_pkt_time > 0 and (now - self.last_pkt_time) > 1.0:
                    if not self.link_ok:
                        events.append(("link_recovered", 2))
                        self.link_ok = True
                self.last_pkt_time = now
            elif self.last_pkt_time > 0 and (now - self.last_pkt_time) > 1.5:
                if self.link_ok:
                    events.append(("link_lost", 2))
                    self.link_ok = False
        except Exception:
            pass

        # startup event (once)
        if not self.started and self.io_ready and self.dsp_ready:
            events.append(("audio_started", 3))
            self.started = True

        return events


def main():
    running = True
    def stop(sig, _):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    # verify aplay
    try:
        subprocess.run(["aplay", "--version"], capture_output=True, timeout=2)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print("[AUDIO] aplay not found.", flush=True)
        return

    mode, gesture_cues, event_cues, volume = load_config()
    if mode == "off":
        print("[AUDIO] mode=off, exiting.", flush=True)
        return

    set_volume(volume)

    try:
        from cpcu_ipc_bridge import IPCBridge
        ipc = IPCBridge()
    except Exception as e:
        print(f"[AUDIO] IPC: {e}", flush=True)
        return

    queue = AudioQueue()
    detector = EventDetector(ipc)

    print(f"[AUDIO] running. gestures={list(gesture_cues.keys())} "
          f"events={list(event_cues.keys())}", flush=True)

    while running:
        t0 = time.monotonic()

        # detect events
        for event_name, priority in detector.poll():
            # gesture event
            if event_name.startswith("gesture:"):
                gname = event_name[8:]
                if gname in gesture_cues:
                    queue.enqueue(1, gesture_cues[gname])
            # system/failure event
            elif event_name in event_cues:
                ec = event_cues[event_name]
                queue.enqueue(ec["priority"], ec["path"])

        # process queue
        queue.tick()

        # sleep remainder
        elapsed = time.monotonic() - t0
        if elapsed < POLL_INTERVAL:
            time.sleep(POLL_INTERVAL - elapsed)

    # shutdown sound
    if "system_shutdown" in event_cues:
        try:
            subprocess.run(["aplay", "-q", event_cues["system_shutdown"]["path"]],
                           timeout=2)
        except Exception:
            pass

    print("[AUDIO] stopped.", flush=True)


if __name__ == "__main__":
    main()
