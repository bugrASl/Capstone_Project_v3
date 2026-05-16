#!/usr/bin/env python3
"""cpcu_audio_daemon.py — gesture transition audio via MAX98357A I2S DAC.

Hardware: MAX98357A → 8Ω speaker (5 wires, no extra components).
Output:   aplay through ALSA → I2S → MAX98357A → speaker.

Modes (gestures.json "audio_mode"):
  "off"   — silent
  "voice" — spoken word .wav files (espeak-ng generated)
  "freq"  — synthesized tones at per-gesture Hz/ms

Each gesture carries both:
  "audio": { "voice": "voice_flex", "freq_hz": 440, "freq_ms": 80 }

Called by: ./launch.sh tui --audio
"""
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


def generate_tone(freq_hz, dur_ms, path):
    """Synthesize a sine tone .wav file."""
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
    """Set ALSA PCM volume (0-100)."""
    try:
        subprocess.run(["amixer", "set", "PCM", f"{pct}%"],
                       capture_output=True, timeout=2)
    except Exception:
        pass


def load_config():
    """Read gestures.json. Returns (mode, voice_map, freq_map, volume)."""
    try:
        with open(GESTURES_PATH) as f:
            gs = json.load(f)
    except Exception as e:
        print(f"[AUDIO] gestures.json: {e}", flush=True)
        return "off", {}, {}, 80

    mode = gs.get("audio_mode", "off")
    volume = gs.get("audio_volume_pct", 80)
    voice_map = {}
    freq_map = {}

    os.makedirs(AUDIO_DIR, exist_ok=True)

    for gname, gdef in gs.get("gestures", {}).items():
        audio = gdef.get("audio", {})

        # voice cue
        vname = audio.get("voice")
        if vname:
            wav = os.path.join(AUDIO_DIR, f"{vname}.wav")
            if os.path.exists(wav):
                voice_map[gname] = wav

        # freq cue (generate on the fly)
        fhz = audio.get("freq_hz")
        fms = audio.get("freq_ms", 80)
        if fhz and fhz > 0:
            tone_path = os.path.join(AUDIO_DIR, f"_gen_{gname}_{fhz}hz.wav")
            try:
                generate_tone(fhz, fms, tone_path)
                freq_map[gname] = tone_path
            except Exception as e:
                print(f"[AUDIO] tone {gname}: {e}", flush=True)

    print(f"[AUDIO] mode={mode} vol={volume}% "
          f"voice={len(voice_map)} freq={len(freq_map)}", flush=True)
    return mode, voice_map, freq_map, volume


def play(path):
    """Non-blocking aplay."""
    try:
        subprocess.Popen(["aplay", "-q", path],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        pass


def check_audio_hw():
    """Verify ALSA output is available."""
    try:
        r = subprocess.run(["aplay", "-l"], capture_output=True,
                           text=True, timeout=3)
        if "hifiberry" in r.stdout.lower() or "sndrpi" in r.stdout.lower():
            print("[AUDIO] I2S DAC detected (MAX98357A).", flush=True)
            return True
        if "usb" in r.stdout.lower():
            print("[AUDIO] USB audio detected.", flush=True)
            return True
        if r.stdout.strip():
            print("[AUDIO] ALSA output found.", flush=True)
            return True
        print("[AUDIO] no ALSA output — check wiring or run "
              "./launch.sh setup-audio", flush=True)
        return False
    except Exception:
        print("[AUDIO] aplay not found.", flush=True)
        return False


def main():
    running = True
    def stop(sig, _):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    if not check_audio_hw():
        return

    mode, voice_map, freq_map, volume = load_config()
    if mode == "off":
        print("[AUDIO] mode=off, exiting.", flush=True)
        return

    set_volume(volume)

    cue_map = voice_map if mode == "voice" else freq_map
    if not cue_map:
        print(f"[AUDIO] no {mode} cues found. "
              f"Run ./launch.sh generate-cues", flush=True)
        return

    try:
        from cpcu_ipc_bridge import IPCBridge
        ipc = IPCBridge()
    except Exception as e:
        print(f"[AUDIO] IPC: {e}", flush=True)
        return

    print(f"[AUDIO] active ({mode}): {list(cue_map.keys())}", flush=True)
    last = ""

    while running:
        try:
            name = ipc.read_dsp_gesture_name()
            if name and name != last:
                if name in cue_map:
                    play(cue_map[name])
                last = name
        except Exception:
            pass
        time.sleep(0.05)

    print("[AUDIO] stopped.", flush=True)


if __name__ == "__main__":
    main()
