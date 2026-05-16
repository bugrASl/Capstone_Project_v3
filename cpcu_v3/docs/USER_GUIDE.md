# InfiniTech CPCU v3.0 — Complete User Guide

## Table of Contents

1. [System Overview](#1-system-overview)
2. [First-Time Setup](#2-first-time-setup)
3. [Running the System](#3-running-the-system)
4. [Configuration: gestures.json](#4-configuration)
5. [Motor Management](#5-motor-management)
6. [EMG Channel Management](#6-emg-channel-management)
7. [Gesture Management](#7-gesture-management)
8. [Audio Feedback](#8-audio-feedback)
9. [Calibration](#9-calibration)
10. [Model Training Workflow](#10-model-training-workflow)
11. [TUI & Web Dashboard](#11-tui-and-web-dashboard)
12. [UART Debug Output](#12-uart-debug-output)
13. [Troubleshooting](#13-troubleshooting)
14. [Complete Command Reference](#14-complete-command-reference)

---

## 1. System Overview

InfiniTech is a prosthetic arm controlled by surface EMG signals. The signal
path is: **muscle contraction → EMG electrodes → BSAU (STM32) → NRF wireless →
CPCU (Raspberry Pi 5) → PCA9685 servo driver → robotic arm**.

The CPCU is where all software runs. It has four processes:

- **cpcu_kernel** — supervisor, spawns and monitors everything
- **cpcu_io** — reads NRF packets, drives PCA9685 servos, runs safety FSM
- **cpcu_dsp.py** — DSP filtering, ML inference, velocity integration
- **cpcu_audio_daemon.py** — plays voice/tone cues on gesture transitions

All configuration lives in one file: `config/gestures.json`. Every change
you make through `./launch.sh` commands modifies this file. The TUI, web
dashboard, and all processes read from it dynamically.

---

## 2. First-Time Setup

### 2.1 Pi Setup

```bash
./launch.sh setup
```

Installs dependencies, configures kernel isolation for real-time cores,
enables SPI and I2C, sets up permissions.

### 2.2 Audio Hardware (PCM5102A + PAM8403)

Wire the components (no resistors, no capacitors needed):

```
Pi 5                PCM5102A              PAM8403           Speaker
────                ────────              ───────           ───────
GPIO18 (pin 12) →   BCK
GPIO19 (pin 35) →   LCK
GPIO21 (pin 40) →   DIN
3.3V   (pin  1) →   VIN
GND    (pin  6) →   GND
                    SCK → GND (TIE!)
                    3.5mm out        →    L-IN / GND
5V     (pin  4)                      →    VCC
GND    (pin  6)                      →    GND
                                          L+ / L-       →   + / −
```

**Important:** The PCM5102A's SCK pin must be connected to GND. This tells
the chip to generate its own clock. Without this, no sound.

Then run:

```bash
./launch.sh setup-audio    # enables I2S overlay, installs espeak-ng
# reboot when prompted
./launch.sh generate-cues  # creates voice .wav files for all gestures
./launch.sh audio test     # verify you hear "flex" from the speaker
```

### 2.3 Build

```bash
./launch.sh build
./launch.sh check          # verify everything is ready
```

### 2.4 First Run

```bash
./launch.sh tui --audio    # dashboard + voice feedback
```

---

## 3. Running the System

### 3.1 Start

```bash
./launch.sh tui                           # TUI dashboard only
./launch.sh tui --audio                   # + voice/tone feedback
./launch.sh tui --with-ws                 # + web dashboard
./launch.sh tui --audio --with-ws         # + both
./launch.sh tui --operator bugraaslan     # with operator velocity profile
```

### 3.2 Stop

```bash
./launch.sh stop
```

### 3.3 Re-attach (if SSH disconnects)

```bash
./launch.sh attach
```

### 3.4 Apply Config Changes Without Full Restart

```bash
./launch.sh reload                # reload everything
./launch.sh reload --dsp          # restart DSP only (gesture/model changes)
./launch.sh reload --audio        # restart audio only (mode/volume changes)
```

---

## 4. Configuration

All configuration is in `config/gestures.json`. You never edit this file
manually — all changes go through `./launch.sh` commands.

To see the current complete configuration:

```bash
./launch.sh show-config
```

This prints every motor, every channel, every gesture, every audio cue, and
all tuning parameters in one view.

### 4.1 What's in gestures.json

| Section | What it controls |
|---|---|
| `servo_channels` | Motor names, PCA channel assignment, pulse limits |
| `emg_channels` | Which ADC channels are active, their muscle names |
| `gestures` | Gesture names, mode, which motors each gesture moves |
| `audio_mode` | "off", "voice", or "freq" |
| `audio_events` | System event sounds (start, stop, fault, etc.) |
| `confidence` | ML confidence curve (quadratic/linear, floor, ceiling) |
| `hysteresis` | Debounce votes for gesture transitions |
| `model_path` | Which .pkl model file to use |

### 4.2 Dynamic Adaptation

The TUI and web dashboard read motor names, gesture names, and channel
names from gestures.json at startup and on every reload. When you rename
a motor from "Gripper" to "Claw", the TUI shows "Claw" after
`./launch.sh reload`. No code changes, no recompilation.

---

## 5. Motor Management

### 5.1 View Current Motors

```bash
./launch.sh show-config
```

Shows each motor's name, PCA channel, and pulse width range.

### 5.2 Add a New Motor

```bash
./launch.sh add-motor Thumb 6
```

Adds motor "Thumb" on PCA9685 channel 6 with default limits
(500-2500 µs, neutral 1500 µs). Then tune the limits:

```bash
./launch.sh edit-motor Thumb
# prompts for min_us, max_us, neutral_us
```

### 5.3 Rename a Motor

```bash
./launch.sh rename-motor Gripper Claw
```

Renames everywhere — the servo_channels entry AND every gesture that
references "Gripper" now references "Claw" instead.

### 5.4 Edit Motor Limits

```bash
./launch.sh edit-motor Elbow
# Current: min=1074 max=1953 neutral=1500
# min_us [1074]: 1100
# max_us [1953]: 1900
# neutral_us [1500]: 1500
```

---

## 6. EMG Channel Management

### 6.1 View Current Channels

```bash
./launch.sh show-config
```

Shows which BSAU ADC channels are active and their muscle names.

### 6.2 Change Active Channels

When you add more electrodes to the BSAU board:

```bash
./launch.sh set-channels 0 1 2 3 4
# prompts for muscle name per channel:
#   Channel 0: Forearm
#   Channel 1: Biceps
#   Channel 2: Triceps
#   Channel 3: Deltoid
#   Channel 4: Flexor
```

The script updates gestures.json and checks if the current ML model
matches the new feature count. If it doesn't match, it tells you
to retrain.

### 6.3 Channel Progression

| Phase | Command | Then |
|---|---|---|
| 3-channel | `set-channels 0 1 2` | Use model_3ch.pkl |
| 5-channel | `set-channels 0 1 2 3 4` | Retrain, use model_5ch.pkl |
| 8-channel | `set-channels 0 1 2 3 4 5 6 7` | Retrain, use model_8ch.pkl |

After changing channels, update the model path:

```bash
# (edit gestures.json model_path, or wait for the set-channels
# script to tell you the next steps)
./launch.sh reload
```

---

## 7. Gesture Management

### 7.1 View Gestures

```bash
./launch.sh show-config
```

### 7.2 Add a New Gesture

```bash
./launch.sh add-gesture
```

Interactive wizard:

```
  Gesture name: pinch
  Mode: [1] velocity  [2] freeze
  Which servos? Wrist1 Gripper
  Wrist1 rate (µs/s): 150
  Gripper rate: -300
  Gripper snap? y
  → Voice cue generated: voice_pinch.wav
  → Tone: 770 Hz (auto-picked)
  ✓ Added 'pinch' to gestures.json
```

After adding, you need to:
1. Record training data: `./launch.sh collect`
2. Retrain the ML model with the new class
3. `./launch.sh reload`

### 7.3 Edit a Gesture's Servo Mapping

```bash
./launch.sh edit-gesture flex
# Shows current mapping, lets you reassign motors and rates
```

### 7.4 Rename a Gesture

```bash
./launch.sh rename-gesture hand grip
```

**Warning:** The gesture name must match the ML model's class name.
If you rename in gestures.json, you must also retrain the model with
the same class name, or the DSP won't find a mapping.

### 7.5 Remove a Gesture

```bash
./launch.sh remove-gesture biceps
```

The "rest" gesture cannot be removed.

### 7.6 Gesture Modes

- **velocity**: Motors move continuously while the gesture is held.
  Speed scales with ML confidence (quadratic curve).
- **freeze**: All motors hold their current position (used for "rest").

### 7.7 Hysteresis (Transition Stability)

The system requires multiple consecutive ML predictions of the same
class before switching gestures. This prevents jittery transitions.

Settings in gestures.json:

```json
"hysteresis": {
    "rest_to_active": 4,    // 4 votes to leave rest
    "active_to_rest": 2,    // 2 votes to return to rest
    "active_to_active": 6   // 6 votes to switch between active gestures
}
```

If the arm is switching gestures too fast, increase `active_to_active`.
If it's slow to respond, decrease `rest_to_active`.

---

## 8. Audio Feedback

### 8.1 Three Modes

```bash
./launch.sh audio voice    # spoken words: "flex", "grip", "extend"
./launch.sh audio freq     # synthesized tones: 440Hz, 550Hz, etc.
./launch.sh audio off      # silent
```

Apply after changing:

```bash
./launch.sh reload --audio
```

### 8.2 Volume

```bash
./launch.sh audio volume 60     # set to 60%
./launch.sh audio volume        # show current
```

### 8.3 Test

```bash
./launch.sh audio test          # plays a sample sound
```

### 8.4 Generate/Regenerate Voice Cues

After adding new gestures or renaming existing ones:

```bash
./launch.sh generate-cues
```

Reads all gesture names from gestures.json and generates spoken
`.wav` files using espeak-ng. Also reads `audio_events` and
generates system event voice cues.

### 8.5 Gesture Audio

Each gesture has both a voice and a frequency tone defined:

```json
"flex": {
    "audio": { "voice": "voice_flex", "freq_hz": 440, "freq_ms": 80 }
}
```

The `audio_mode` setting selects which one plays. When you
`add-gesture`, both are created automatically.

### 8.6 System Event Audio

System events can also play sounds:

```json
"audio_events": {
    "system_start":   { "voice": "voice_system_start",   "freq_hz": 880 },
    "safe_state":     { "voice": "voice_safe_state",     "freq_hz": 200 },
    "link_lost":      { "voice": "voice_link_lost",      "freq_hz": 300 },
    "low_battery":    { "voice": "voice_low_battery",    "freq_hz": 250 }
}
```

These are played by the audio daemon when the safety FSM
transitions. After adding/editing events, run:

```bash
./launch.sh generate-cues
./launch.sh reload --audio
```

### 8.7 Hardware

PCM5102A I2S DAC + PAM8403 Class-D amplifier + 1W 8Ω speaker.
No resistors, no capacitors. See Section 2.2 for wiring.

---

## 9. Calibration

### 9.1 Gripper Firmness

```bash
./launch.sh grip-tune
```

Walks you through finding the optimal grip pulse width by stepping
the servo and asking when the grip is firm vs. stalling. Also lets
you choose gripper close speed (slow/medium/fast/snap).

### 9.2 Operator Velocity Preference (0-10 Scale)

```bash
./launch.sh calibrate
```

Two phases:

1. **Rest calibration**: Record 10s of muscle silence to establish
   the noise floor.
2. **Velocity tuning**: For each gesture, engage it and pick a
   comfort level on a 0-10 scale (0=stop, 5=normal, 10=max speed).

The mapping is quadratic: level 3 = 36% speed, level 7 = 196% speed.
This gives precise control at low levels and fast motion at high levels.

### 9.3 Per-Operator Profiles

```bash
./launch.sh calibrate --operator ali
./launch.sh tui --operator ali --audio
```

Each operator's calibration is saved separately. Switch between
operators by name.

### 9.4 Partial Calibration

```bash
./launch.sh calibrate --rest-only     # noise floor only
./launch.sh calibrate --vel-only      # velocity preference only
```

---

## 10. Model Training Workflow

### 10.1 Collect Training Data

```bash
./launch.sh collect
```

Records labelled EMG data to `datasets/`. The operator performs
each gesture when prompted. Capture at least 30 seconds per
gesture class.

### 10.2 Retrain

Training happens externally (on a PC with Python + scikit-learn).
Use the collected CSV files to train a RandomForest or SVM:

```python
# Example training script (run on PC, not Pi)
import joblib
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import StandardScaler

# load data, extract features...
scaler = StandardScaler().fit(X_train)
model = RandomForestClassifier(n_estimators=100).fit(
    scaler.transform(X_train), y_train)

# save combined checkpoint
joblib.dump({"model": model, "scaler": scaler}, "model_5ch.pkl")
```

### 10.3 Deploy

```bash
# copy model to Pi
scp model_5ch.pkl pi@<ip>:cpcu_v2/models/

# set it as active
./launch.sh set-model models/model_5ch.pkl

# apply
./launch.sh reload
```

Running `set-model` with no arguments lists all available `.pkl` files:

```bash
./launch.sh set-model
#   Available models:
#     aleynask.pkl
#     model_5ch.pkl
```

The DSP will log on startup:

```
[DSP] model: models/model_5ch.pkl
[DSP] model classes: ['rest', 'flex', 'ext', 'hand']
```

Verify the classes match your gestures.json gesture names.

---

## 11. TUI and Web Dashboard

### 11.1 TUI Pages

Navigate with number keys (1-7):

| Page | Content |
|---|---|
| 1 | System overview: state, uptime, link quality |
| 2 | Servo positions: live pulse widths, safety status |
| 3 | DSP/AI: gesture, confidence, latency breakdown |
| 4 | Signal: per-channel RMS, envelope waveforms |
| 5 | Radio: packet rate, loss, retransmit stats |
| 6 | Config: runtime parameter viewer |
| 7 | Editor: live parameter tuning |

### 11.2 Dynamic Names

Motor names on Page 2 come from `gestures.json → servo_channels`.
Gesture names on Page 3 come from `gestures.json → gestures`.
Channel names on Page 4 come from `gestures.json → emg_channels`.

When you rename anything via `./launch.sh`, the TUI picks up the
new names after `./launch.sh reload`.

### 11.3 Web Dashboard

```bash
./launch.sh tui --with-ws
```

When the web dashboard starts, the terminal prints:

```
  ══════════════════════════════════════════════
  WEB DASHBOARD
  Same network:  http://192.168.1.42:8765
  Remote (SSH):  ssh -L 8765:localhost:8765 pi@192.168.1.42
                 then http://localhost:8765
  ══════════════════════════════════════════════
```

If your PC is on the same network as the Pi, open the first URL
directly in your browser.

If you're connecting remotely (e.g., from the university network
to the Pi over SSH), run the SSH tunnel command on your PC first:

```bash
# on your PC (replace with your Pi's IP)
ssh -L 8765:localhost:8765 pi@144.122.177.115
```

Then open `http://localhost:8765` in your browser. The tunnel
forwards your local port 8765 to the Pi's port 8765.

### 11.4 Battery Display

Battery monitoring has been removed from the TUI. The BSAU no
longer samples the battery voltage (the field exists in the
wireless packet for backward compatibility but reads zero).
The health banner no longer shows a battery pill, and the battery
section on page 1 is replaced with system status info.

---

## 12. UART Debug Output

### 12.1 What It Does

When enabled, the DSP pipeline sends a CSV line over the Pi's
UART (GPIO14 TX) at every inference window (~10 Hz):

```
timestamp_ms, gesture, confidence, rms0, var0, wl0, env0, rms1, ...
```

This lets you capture live classification data on a host PC for
analysis, debugging, or recording without affecting the Pi's
performance.

### 12.2 Hardware

You need a USB-to-UART adapter (CP2102, CH340, or FTDI — the same
type you use for the BSAU). No extra components.

```
Pi 5 GPIO14 TX (pin  8)  →  Adapter RX
Pi 5 GPIO15 RX (pin 10)  →  Adapter TX  (optional, for bidirectional)
Pi 5 GND       (pin 14)  →  Adapter GND

Do NOT connect adapter VCC to Pi.
Baud: 115200 8N1.
```

### 12.3 Pi Setup (one-time)

```bash
./launch.sh setup-uart
# reboot when prompted
```

This enables the UART hardware and removes the Linux serial
console so the port is free for your data.

### 12.4 Usage

```bash
./launch.sh tui --uart                    # UART debug only
./launch.sh tui --uart --audio            # + audio feedback
./launch.sh tui --uart --audio --with-ws  # everything
```

### 12.5 Host PC

On your PC, read the data with any of these:

```bash
# simple terminal
screen /dev/ttyUSB0 115200              # Linux
screen /dev/tty.usbserial* 115200      # macOS
# Windows: PuTTY → Serial → COM port → 115200

# or the included monitor with live formatting
python3 scripts/uart_monitor.py --port /dev/ttyUSB0

# with logging to file
python3 scripts/uart_monitor.py --port COM3 --log session.csv

# raw output (for piping to other tools)
python3 scripts/uart_monitor.py --port /dev/ttyUSB0 --raw
```

The monitor shows a live updating line with gesture name,
confidence, and feature values, with a `◄ TRANSITION` marker
on gesture changes.

---

## 13. Troubleshooting

### No audio output

```bash
./launch.sh audio test
```

If no sound:
1. Check wiring (PCM5102A SCK must be tied to GND)
2. `aplay -l` — should show the I2S device
3. `./launch.sh setup-audio` — re-run setup
4. Reboot if I2S overlay was just added

### Model mismatch after channel change

```
[DSP] WARNING: model expects 12 features, pipeline produces 20
```

The model doesn't match the channel count. Retrain with the
current channel configuration.

### Gesture switching too fast (jittery)

Increase hysteresis in gestures.json:

```json
"hysteresis": { "active_to_active": 8 }
```

Then `./launch.sh reload`.

### Gripper doesn't hold objects

```bash
./launch.sh grip-tune
```

Lower the `grip_firm_us` value and increase gripper speed.

### System enters SAFE state unexpectedly

Check radio link. The safety watchdog triggers after 750ms
without a valid NRF packet. Verify BSAU is powered and in range.

### DSP can't find model

```
[DSP] no model found — feature-only mode
```

Check `gestures.json → model_path`. The path is relative to
the repo root. Verify the file exists:

```bash
ls -la models/*.pkl
```

---

## 14. Complete Command Reference

```
═══════════════════════════════════════════════════════════════
 ./launch.sh <command> [options]
═══════════════════════════════════════════════════════════════

 SETUP
   setup                              Pi one-time configuration
   setup-audio                        I2S DAC + speaker setup
   setup-uart                         UART debug output setup
   build                              Compile and install
   check                              Verify readiness

 RUNNING
   tui                                Dashboard
   tui --audio                        + voice/tone feedback
   tui --uart                         + UART debug to host PC
   tui --with-ws                      + web dashboard
   tui --audio --uart --with-ws       All features enabled
   tui --operator NAME                Operator velocity profile
   ws                                 Web dashboard only
   kernel                             Kernel only (for systemd)
   collect                            Dataset capture mode
   menu                               Interactive mode picker
   attach                             Re-attach tmux session
   stop                               Stop all processes

 TESTING
   test-sw                            Software tests (no hardware)
   test-ipc                           + IPC validation
   test-hw                            + hardware probes
   test-pca                           Interactive servo check
   test-signal                        Live signal integrity
   test-signal-demo                   Synthetic signal demo
   test-safety-demo                   Fault injection demo
   test-system                        Full system verification

 EMG CHANNELS
   set-channels 0 1 2                 Set active channels
   set-channels 0 1 2 3 4             (prompts for muscle names)
   set-channels 0 1 2 3 4 5 6 7       (validates model match)

 SERVO MOTORS
   add-motor NAME PCA_CH              Add motor on PCA channel
   edit-motor NAME                    Edit limits (min/max/neutral)
   rename-motor OLD NEW               Rename (updates all refs)

 GESTURES
   add-gesture                        Guided wizard (+ audio)
   edit-gesture NAME                  Change servo mapping
   rename-gesture OLD NEW             Rename gesture
   remove-gesture NAME                Delete gesture

 CALIBRATION
   grip-tune                          Gripper firmness wizard
   calibrate                          Rest noise + velocity (0-10)
   calibrate --operator NAME          Per-operator profile
   calibrate --rest-only              Noise floor only
   calibrate --vel-only               Velocity only

 AUDIO (PCM5102A + PAM8403 + 1W 8Ω speaker)
   audio                              Show audio configuration
   audio off                          Disable audio feedback
   audio voice                        Spoken word mode
   audio freq                         Frequency tone mode
   audio volume N                     Set volume (0-100%)
   audio test                         Play test sound
   generate-cues                      Generate voice .wav files

 MODEL
   set-model PATH                     Set active ML model (.pkl)
   set-model                          List available models

 CONFIGURATION
   show-config                        Print full system config
   show-gestures                      Same as show-config
   configure                          Compile-time settings
   configure --show                   Show compile values
   configure --reset                  Restore defaults

 RELOAD (apply changes without full restart)
   reload                             All (runtime + DSP + audio)
   reload --dsp                       DSP pipeline only
   reload --audio                     Audio daemon only

 UART DEBUG
   setup-uart                         Enable UART on Pi 5
   tui --uart                         Run with UART debug
   Host: uart_monitor.py --port PORT  Receive on host PC

 SERVICES (auto-start at boot)
   install-service                    Kernel systemd unit
   install-ws-service                 Web dashboard service
   grant-caps                         RT capabilities
═══════════════════════════════════════════════════════════════
```

---

*InfiniTech — Design and Implementation of a Bioelectrical Signal-Based
Control System for a Prosthetic Limb. METU EE Senior Design, 2025-2026.*
