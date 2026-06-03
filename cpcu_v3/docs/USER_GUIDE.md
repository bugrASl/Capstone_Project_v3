# InfiniTech CPCU v3.0 — Complete User Guide

## Table of Contents

1. [System Overview](#1-system-overview)
2. [First-Time Setup](#2-first-time-setup)
3. [Running the System](#3-running-the-system)
4. [Configuration](#4-configuration)
5. [Motor Management](#5-motor-management)
6. [EMG Channel Management](#6-emg-channel-management)
7. [Gesture Management](#7-gesture-management)
8. [Audio Feedback](#8-audio-feedback)
9. [Calibration](#9-calibration)
10. [Model Training Workflow](#10-model-training-workflow)
11. [TUI & Web Dashboard](#11-tui-and-web-dashboard)
12. [Host-Side EMG Monitor (UART)](#12-host-side-emg-monitor-uart)
13. [Performance Tuning](#13-performance-tuning)
14. [Troubleshooting](#14-troubleshooting)
15. [Complete Command Reference](#15-complete-command-reference)

---

## 1. System Overview

InfiniTech is a prosthetic arm controlled by surface EMG signals. The
signal path is: **muscle contraction → EMG electrodes → BSAU (STM32) →
NRF24L01+ wireless → CPCU (Raspberry Pi 5) → PCA9685 servo driver → 6
servos**.

The CPCU runs four processes, each pinned to its own real-time CPU core:

- **cpcu_kernel** (Core 0) — supervisor; spawns processes, owns the
  safety FSM, watchdog, configuration, audio daemon, TUI, and
  WebSocket dashboard.
- **cpcu_io** (Core 3, SCHED_FIFO 90) — polls the NRF radio, drives the
  PCA9685 over I²C, applies the smoother (independent velocity/accel
  safety caps).
- **cpcu_dsp.py** (Cores 1+2, SCHED_FIFO 80) — bandpass + notch +
  envelope filtering, feature extraction, RandomForest inference,
  hysteresis, gesture→velocity integration.
- **cpcu_audio_daemon.py** — plays voice or tone cues on gesture
  transitions and safety events.

All inter-process traffic goes through a single 4 MB shared-memory
region at `/dev/shm/cpcu_ipc` using lock-free SPSC rings — no
mutexes anywhere in the hot path.

### 1.1 Dual-arm classification

The DSP runs **per-arm classifiers** in parallel. Each arm is a
*gesture group* with its own EMG channel set, its own model file, and
its own gesture-to-servo mapping. v3 ships with two groups:

| Group | EMG channels | Muscles | Drives |
|---|---|---|---|
| `right_arm` | 0, 1, 2, 3 | R_Hand, R_Biceps, R_Triceps, R_Shoulder | Forearm + Elbow + Wrist1 + Gripper (close) |
| `left_arm`  | 4, 5, 6, 7 | L_Hand, L_Biceps, L_Triceps, L_Shoulder | Base + Wrist1 + Gripper (open) |

Both groups share `models/arm.pkl` — a single classifier trained on
4-channel × 5-class data (rest, hand, flex, ext, wrist).

### 1.2 Configuration files

Two files in `config/`:

| File | What it controls |
|---|---|
| `gestures.json` | Servo channel map, EMG channel assignment, gesture definitions, confidence curves, hysteresis, audio cues |
| `runtime.json` | Smoother caps (velocity, accel, deadband), gripper firmware (firm/touch/open µs), gravity compensation, safety thresholds |

The TUI editor (page 7) modifies `runtime.json` live. Most other
configuration lives in `gestures.json` and is edited through
`./launch.sh` commands. Both files are read on `./launch.sh reload`
without restarting the kernel.

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
./launch.sh tui --uart                    # + UART debug stream to host PC
./launch.sh tui --audio --with-ws --uart  # everything
./launch.sh tui --operator bugraaslan     # with per-operator velocity profile
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

All configuration lives in two files under `config/`. You normally
never hand-edit them — `./launch.sh` commands cover the common cases
and the TUI editor (page 7) lets you tune runtime parameters live.

To see the current complete configuration:

```bash
./launch.sh show-config
```

This prints every motor, every channel, every gesture, every audio
cue, and all tuning parameters in one view.

### 4.1 `config/gestures.json` — application-level config

| Section | What it controls |
|---|---|
| `servo_channels` | Motor names, PCA channel assignment, pulse limits |
| `audio_mode` / `audio_volume_pct` | Audio output mode and volume |
| `audio_events` | System event sounds (start, stop, fault, etc.) |
| `gesture_groups` | Per-arm config: emg_channels, model_path, confidence curve, hysteresis votes, gestures |

Inside each gesture group:

| Field | Meaning |
|---|---|
| `emg_channels.active` | Absolute ADC channel indices for this arm |
| `emg_channels.names`  | Human-readable muscle name per channel |
| `model_path` | Which `.pkl` model file to use |
| `confidence` | Curve type (quadratic/linear/none), floor_pct, ceil_pct |
| `hysteresis` | Vote counts for state transitions |
| `gestures` | Per-gesture: mode (velocity/freeze), per-servo `rate_us_s`, optional `snap`, audio cue |

### 4.2 `config/runtime.json` — low-level tuning

| Field | Meaning |
|---|---|
| `servo_min_us` / `servo_max_us` / `servo_bias_us` | Per-servo pulse limits + trim |
| `servo_pca_ch` | Logical slot → PCA9685 channel (non-contiguous: 0, 2, 4, 7, 11, 15) |
| `smooth_velocity` | Max position rate per servo (µs/s) — independent safety cap |
| `smooth_accel`    | Max acceleration per servo (µs/s²) |
| `smooth_deadband` | Hold-pose deadband (µs); below this the smoother considers itself settled |
| `interp_conf_floor_pct` / `interp_conf_ceil_pct` | Default confidence curve boundaries |
| `grip_firm_us` / `grip_touch_us` / `grip_open_us` | Soft-grip thresholds |
| `grip_stall_recover_ms` | Timeout before stall watchdog retreats |
| `safety_ignore_battery` | Set to 1 for bench tests without battery sensing |
| `gravity_dir` / `gravity_scale_pct` | Per-servo gravity compensation (default off) |

The smoother caps in `runtime.json` are **independent safety limits**.
Even if a gesture's `rate_us_s` would command a large step, the
smoother won't let any servo move faster than `smooth_velocity[s]`
or accelerate harder than `smooth_accel[s]`.

### 4.3 Dynamic adaptation

The TUI and web dashboard read motor names, gesture names, and
channel names from `gestures.json` at startup and on every reload.
Rename a motor from "Gripper" to "Claw" and the TUI shows "Claw"
after `./launch.sh reload`. No code changes, no recompilation.

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

v3 hardware has eight EMG channels (INA-style instrumentation amps,
one per electrode), split four-per-arm between two gesture groups.

### 6.1 Current channel layout

| Channel | Operator's muscle | Gesture group | Drives gesture class |
|:---:|---|---|---|
| 0 | R_Hand     | right_arm | hand  → Gripper close (snap) |
| 1 | R_Biceps   | right_arm | flex  → Forearm + Elbow flexion |
| 2 | R_Triceps  | right_arm | ext   → Forearm + Elbow extension |
| 3 | R_Shoulder | right_arm | wrist → Wrist1 rotation (+) |
| 4 | L_Hand     | left_arm  | hand  → Gripper open (soft) |
| 5 | L_Biceps   | left_arm  | flex  → Base rotate + |
| 6 | L_Triceps  | left_arm  | ext   → Base rotate − |
| 7 | L_Shoulder | left_arm  | wrist → Wrist1 rotation (−) |

### 6.2 Naming convention

Three distinct names live near each EMG channel — don't confuse them:

| Concept | Example | Where it appears |
|---|---|---|
| **Muscle name** | `R_Shoulder` | EMG electrode (electrode sits on this muscle) |
| **Gesture name** | `wrist` | Classifier output class (robot action the gesture triggers) |
| **Servo name** | `Wrist1` | Mechanical joint the gesture's `channels` block drives |

So `ch3 (R_Shoulder) → wrist gesture → Wrist1 servo`. The muscle
generates the signal; the gesture is the classifier's decision; the
servo is the actuator.

### 6.3 View current channels

```bash
./launch.sh show-config
```

Shows per-arm `emg_channels.active` and `emg_channels.names`.

### 6.4 Change active channels

If you re-wire the BSAU board (e.g., add electrodes, or swap muscle
placement):

```bash
./launch.sh set-channels 0 1 2 3 4 5 6 7
# Prompts for muscle name per channel:
#   Channel 0: R_Hand
#   Channel 1: R_Biceps
#   ...
```

The script updates `gestures.json` and warns if the current ML model's
expected feature count doesn't match the new channel count
(4 channels × 7 features = 28-feature input per arm in the current
build).

### 6.5 Re-assigning channels between arms

If you want to give a different arm an extra channel, edit
`gestures.json::gesture_groups.<arm>.emg_channels.active` directly
and re-run training. The DSP, TUI, and web dashboard pick up the new
assignment on `./launch.sh reload`.

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
  Position increment per DSP tick (10 ms) is
  `rate_us_s × dt × confidence_scale`. Confidence scaling uses the
  group's `confidence.curve` (quadratic by default) between
  `floor_pct` and `ceil_pct` — below floor the servo doesn't move,
  above ceil it moves at full `rate_us_s`.
- **freeze**: All servos hold their current position. Used for
  `rest`. If every group is in rest, the arm holds its last commanded
  pose — it does NOT snap to neutral. Use SAFE state (radio loss,
  watchdog timeout) to force a neutral snap.

### 7.7 Snap behaviour on the Gripper

A gesture can include `"snap": true` on the Gripper to bypass rate
integration:

```json
"hand": {
    "channels": {
        "Gripper": { "rate_us_s": -400, "snap": true }
    }
}
```

When `snap=true` on the Gripper, the DSP pins the gripper's target
directly to `grip_firm_us` (from `runtime.json` or the per-gesture
override) while the gesture is active. The downstream smoother still
applies its velocity/accel caps, so the actual closing time is
`|neutral - grip_firm| / smooth_velocity[Gripper]` ≈ 130 ms (not a
hardware-level teleport).

`snap=true` on any non-Gripper servo is currently ignored — only the
Gripper has a well-defined "snap target" (`grip_firm_us`). The
non-snap path (rate integration) still works normally for any servo.

v3 default: `right_arm.hand` uses `snap=true` (firm-grip closing);
`left_arm.hand` uses `snap=false` (soft-grip opening at rate 600 µs/s).

### 7.8 Hysteresis (transition stability)

The classifier votes every 100 ms. Hysteresis requires N consecutive
votes for the same class before committing to a transition:

```json
"hysteresis": {
    "rest_to_active": 1,   // votes to leave rest
    "active_to_rest": 0,   // votes to return to rest
    "active_to_active": 1  // votes to switch between active gestures
}
```

v3 defaults are aggressive (`1, 0, 1`) — the classifier commits on
first crossing. Raise `active_to_active` to 3 or 4 if gesture
switching is too jittery; raise `rest_to_active` if the system
triggers spuriously at rest.

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

Records labelled EMG data to `datasets/`. The operator performs each
gesture when prompted. Capture at least 30 seconds of clean data per
gesture class per arm. v3's classifier sees four channels per arm
and outputs one of five classes: `rest, hand, flex, ext, wrist`.

### 10.2 Retrain

Training happens externally (on a PC with Python + scikit-learn). Use
the collected CSV files to extract features and train a
RandomForest. Feature extractor produces 7 time-domain features per
channel (RMS, MAV, ZC, WL, IAV, MNF, MDF), so the per-arm input is
4 × 7 = 28-dimensional:

```python
# Example training script (run on PC, not Pi)
import joblib
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import StandardScaler

# load data, extract features (28 features/arm × N windows)
scaler = StandardScaler().fit(X_train)
model  = RandomForestClassifier(n_estimators=100,
                                max_depth=12,
                                min_samples_split=5,
                                random_state=42)
model.fit(scaler.transform(X_train), y_train)

# save combined checkpoint
joblib.dump({"model": model, "scaler": scaler}, "arm.pkl")
```

The single `arm.pkl` is used for both `right_arm` and `left_arm`
classifiers (they share the model — same muscle types, just mirrored).

### 10.3 Deploy

```bash
# copy model to Pi
scp arm.pkl pi@<ip>:Capstone_Project_v3/models/

# set it as active
./launch.sh set-model models/arm.pkl

# apply
./launch.sh reload
```

Running `set-model` with no arguments lists all available `.pkl` files:

```bash
./launch.sh set-model
#   Available models:
#     arm.pkl
#     arm_2025_05_30.pkl  (your backup checkpoints)
```

The DSP logs on startup:

```
[DSP] right_arm: model classes: ['rest', 'flex', 'ext', 'hand', 'wrist']
[DSP] left_arm:  model classes: ['rest', 'flex', 'ext', 'hand', 'wrist']
```

Verify the classes match your gesture group names. If they don't,
the DSP falls back to "feature-only mode" and no inference runs.

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

## 12. Host-Side EMG Monitor (UART)

### 12.1 What it does

When the Pi is launched with `--uart`, `cpcu_io` streams raw 8-channel
EMG samples plus per-arm classification state at 1 kHz over the Pi's
hardware UART (GPIO 14 TX). A host-side Python script
(`python/monitor.py`) reads the stream over USB-UART and renders a
live matplotlib dashboard with:

* 8 channel waveforms (4 right-arm + 4 left-arm, colour-coded)
* per-arm class-probability bars for `rest, hand, flex, ext, wrist`
* live status line at the top of the figure showing
  `rate: ## Hz   age: ## ms   dropped: ## KB   err: ...`
* red flash if the UART link has stalled, amber while it drains a backlog

### 12.2 Wiring

Use any USB-to-UART adapter (CP2102, CH340, FTDI). No extra parts.

```
Pi 5 GPIO14 TX (pin  8)  →  Adapter RX
Pi 5 GPIO15 RX (pin 10)  →  Adapter TX  (optional)
Pi 5 GND       (pin 14)  →  Adapter GND

Do NOT connect adapter VCC to the Pi.
Baud: 921600 8N1.
```

### 12.3 Pi setup (one-time)

```bash
./launch.sh setup-uart
# reboot when prompted
```

This enables the UART hardware and removes the Linux serial console
so the port is free for our data stream.

### 12.4 Run the Pi

```bash
./launch.sh tui --uart                    # UART only
./launch.sh tui --uart --audio            # + audio feedback
./launch.sh tui --uart --audio --with-ws  # all features
```

Without `--uart`, no UART data is generated — the host monitor will
sit at the red "rate 0 Hz, no data yet" status line.

### 12.5 Run the host monitor

On the host PC:

```bash
# from the cloned repo
python3 python/monitor.py \
    --port /dev/ttyUSB0 \
    --baud 921600 \
    --model models/arm.pkl
```

Flags:

| Flag | Default | Meaning |
|---|---|---|
| `--port` | required | Serial device (`/dev/ttyUSB0`, `COM5`, etc.) |
| `--baud` | 921600 | UART baud rate (must match Pi side) |
| `--model` | (none) | Load `arm.pkl` for on-host re-classification (independent of Pi DSP) |
| `--log FILE` | (none) | Append a CSV of every sample to FILE |

The matplotlib window stays open until you close it; the status line
turns grey when data is flowing healthily.

### 12.6 What's in the stream

Each line over UART is a 30-byte packet: 8 channels × 16 bits + a
2-byte tail with the right_arm and left_arm class index. The host
parses it once per ms. The bandwidth is ~30 KB/s, which is 33% of
the 921 600 baud capacity — plenty of headroom for jitter.

---

## 13. Performance Tuning

The smoother caps in `runtime.json` and the confidence curve in each
gesture group are the two main tuning knobs for "how snappy does it
feel". This section is the cheat sheet.

### 13.1 The signal chain in one line

```
classifier confidence × confidence_scale × rate_us_s × dt
   = position step per DSP tick (10 ms)
       → smoother caps it at smooth_velocity[s] per IO tick (20 ms)
           → PCA9685 writes at 50 Hz
```

So `rate_us_s` is the *desired* speed; `smooth_velocity` is the
*allowed* speed. They should usually be set with
`rate_us_s ≤ smooth_velocity[s]` — otherwise the smoother just clips
your gesture rate and the operator can't feel the difference between
"moderate" and "fast" gestures.

### 13.2 Confidence curve

```json
"confidence": { "curve": "quadratic", "floor_pct": 25, "ceil_pct": 70 }
```

Quadratic curve at intermediate confidence:

| Conf | scale (floor=25, ceil=70) |
|---:|---:|
| 25 % | 0.00 |
| 40 % | 0.111 |
| 50 % | 0.309 |
| 60 % | 0.605 |
| 70 % | 1.000 |

* **Raise `floor_pct`** if rest is being misclassified into a low-confidence active gesture (false positives).
* **Lower `floor_pct`** if hands feel sluggish at moderate effort.
* **Lower `ceil_pct`** if the operator can't hit 85%+ confidence and gestures never reach full rate.
* **Switch `curve` to `linear`** if the quadratic dead zone near floor is too aggressive — gives `(conf − floor)/(ceil − floor)` directly.

### 13.3 Smoother caps (per-servo, runtime.json)

```json
"smooth_velocity":  [3000, 3000, 3000, 3000, 3000, 1500],
"smooth_accel":     [30000, 15000, 15000, 30000, 30000, 15000],
"smooth_deadband":  [4, 4, 4, 4, 4, 4]
```

* **`smooth_velocity[s]`** caps absolute speed. SG90 mechanical max is
  ~6666 µs/s; we cap at 45% (3000 µs/s) for gearbox margin. Gripper
  at 1500 µs/s for delicate grasping.
* **`smooth_accel[s]`** caps how quickly velocity can change. Heavy
  joints (Elbow, Forearm, Gripper) use 15000 µs/s² for gentler ramp;
  light joints (Base, Wrist1, Wrist2) use 30000 for snappier motion.
  If a joint is *straining* (audible buzz, slow start), lower its
  accel.
* **`smooth_deadband[s]`** is the hold-pose threshold. 4 µs ≈ 1
  PCA9685 LSB; below this the smoother considers itself settled.
  Raising it to 6–8 µs reduces servo chatter at the cost of slightly
  more position quantization.

### 13.4 Per-gesture rate

`rate_us_s` (in each gesture's `channels` block) is how fast the
servo moves at full confidence:

```json
"flex": {
    "channels": {
        "Forearm": { "rate_us_s": -500 },
        "Elbow":   { "rate_us_s":  500 }
    }
}
```

500 µs/s is moderate; 1000+ feels aggressive on light joints. Set
each per gesture by feel — too slow and the operator gets bored, too
fast and they overshoot.

### 13.5 Quick recipes

| Symptom | Try |
|---|---|
| Motors strain / buzz on start | `smooth_accel` for that servo → 15000 |
| Gestures don't engage until 80% confidence | `confidence.ceil_pct` → 70 |
| Gripper closes too slowly | `smooth_velocity[5]` → 2500, or use `snap=true` |
| Servo chatters when held idle | `smooth_deadband` → 6–8 |
| Position drifts at low effort | `confidence.floor_pct` → 35 (raise threshold) |

---

## 14. Troubleshooting

### No audio output

```bash
./launch.sh audio test
```

If no sound:

1. Check wiring (PCM5102A SCK must be tied to GND)
2. `aplay -l` — should show the I²S device
3. `./launch.sh setup-audio` — re-run setup
4. Reboot if I²S overlay was just added

### Motors strain / buzz audibly during motion

Three usual suspects, in order of likelihood:

1. **Mechanical end-stop**: `runtime.json::servo_min_us` /
   `servo_max_us` exceed the joint's mechanical range. The servo
   keeps pushing into a hard stop and stalls. Measure the actual
   mechanical range with `./launch.sh test-pca` (interactive servo
   calibrator) and tighten the limits with ±30 µs margin.
2. **Power supply sag**: SG90s draw up to 700 mA stall current. If
   the 5 V rail droops, multiple servos starve each other. Run them
   off a separate 5 V / 3 A SMPS with common ground to the Pi.
3. **Acceleration too aggressive**: drop `smooth_accel` for the
   straining joint from 30000 to 15000 µs/s² in `runtime.json`.

### Model mismatch after channel change

```
[DSP] WARNING: model expects 21 features, pipeline produces 28
```

The model was trained on 3 channels (3×7=21 features); your current
config uses 4 (4×7=28). Retrain with `./launch.sh collect` and a fresh
4-channel training session.

### Gestures don't engage until very strong contraction

The confidence floor is too high. Drop
`gesture_groups.<arm>.confidence.floor_pct` from 40 → 25 in
`gestures.json` and `./launch.sh reload`.

### Gesture switching jittery

```json
"hysteresis": { "active_to_active": 4 }
```

Raises the vote threshold for class switches. Then
`./launch.sh reload`.

### Gripper doesn't hold objects

```bash
./launch.sh grip-tune
```

Lowers `grip_firm_us` (firmer pinch) and/or `smooth_velocity[5]`
(faster close). If you want the gripper to clamp shut at full speed,
also set `"snap": true` on the gripper inside the `hand` gesture.

### System enters SAFE state unexpectedly

The safety FSM enters SAFE when the radio is silent for more than
200 ms. Check:

* BSAU is powered (battery not depleted)
* BSAU is in range
* No Wi-Fi co-channel interference (nRF24L01 channel 76 is
  2.476 GHz; Wi-Fi 802.11g channel 13 is 2.472 GHz — close enough
  that a loud router can interfere)
* `io_seq_gaps` in the TUI diagnostics page — non-zero means packets
  are arriving but with gaps; large `io_safe_entries` means the
  watchdog is repeatedly triggering

### DSP can't find model

```
[DSP] no model found — feature-only mode
```

Check `gestures.json → gesture_groups.<arm>.model_path`. The path is
relative to the repo root. Verify:

```bash
ls -la models/*.pkl
```

If empty, you need to train and deploy a model — see §10.

---

## 15. Complete Command Reference

```
═══════════════════════════════════════════════════════════════
 ./launch.sh <command> [options]
═══════════════════════════════════════════════════════════════

 SETUP
   setup                              Pi one-time configuration
   setup-audio                        I²S DAC + speaker setup
   setup-uart                         UART debug output setup
   build                              Compile and install
   check                              Verify readiness

 RUNNING
   tui                                Dashboard
   tui --audio                        + voice/tone feedback
   tui --uart                         + UART debug to host PC
   tui --with-ws                      + web dashboard
   tui --audio --uart --with-ws       all features enabled
   tui --operator NAME                operator velocity profile
   ws                                 web dashboard only
   kernel                             kernel only (for systemd)
   collect                            dataset capture mode
   menu                               interactive mode picker
   attach                             re-attach tmux session
   stop                               stop all processes

 TESTING
   test-sw                            software tests (no hardware)
   test-ipc                           + IPC validation
   test-hw                            + hardware probes
   test-pca                           interactive servo calibrator
   test-signal                        live signal integrity
   test-signal-demo                   synthetic signal demo
   test-safety-demo                   fault-injection demo
   test-system                        quick PASS/FAIL SYS-REQ check
   test-report                        comprehensive defence-grade
                                       test (5-15 min, writes
                                       Markdown + JSON to log/)

 EMG CHANNELS
   set-channels 0 1 2 3 4 5 6 7       set active channels
                                       (prompts for muscle names)

 SERVO MOTORS
   add-motor NAME PCA_CH              add motor on PCA channel
   edit-motor NAME                    edit limits (min/max/neutral)
   rename-motor OLD NEW               rename (updates all refs)

 GESTURES
   add-gesture                        guided wizard (+ audio)
   edit-gesture NAME                  change servo mapping
   rename-gesture OLD NEW             rename gesture
   remove-gesture NAME                delete gesture

 CALIBRATION
   grip-tune                          gripper firmness wizard
   calibrate                          rest noise + velocity (0-10)
   calibrate --operator NAME          per-operator profile
   calibrate --rest-only              noise floor only
   calibrate --vel-only               velocity only

 AUDIO (PCM5102A + PAM8403 + 1 W 8 Ω speaker)
   audio                              show audio configuration
   audio off                          disable audio feedback
   audio voice                        spoken-word mode
   audio freq                         frequency tone mode
   audio volume N                     set volume (0-100%)
   audio test                         play test sound
   generate-cues                      generate voice .wav files

 MODEL
   set-model PATH                     set active ML model (.pkl)
   set-model                          list available models

 CONFIGURATION
   show-config                        print full system config
   show-gestures                      same as show-config
   configure                          compile-time settings
   configure --show                   show compile values
   configure --reset                  restore defaults

 RELOAD (apply changes without full restart)
   reload                             all (runtime + DSP + audio)
   reload --dsp                       DSP pipeline only
   reload --audio                     audio daemon only

 UART DEBUG
   setup-uart                         enable UART on Pi 5
   tui --uart                         run with UART debug
   Host: python/monitor.py --port P   receive on host PC

 SERVICES (auto-start at boot)
   install-service                    kernel systemd unit
   install-ws-service                 web dashboard service
   grant-caps                         RT capabilities
═══════════════════════════════════════════════════════════════
```

---

*InfiniTech — Design and Implementation of a Bioelectrical Signal-Based
Control System for a Prosthetic Limb. METU EE Senior Design, 2025–2026.*
