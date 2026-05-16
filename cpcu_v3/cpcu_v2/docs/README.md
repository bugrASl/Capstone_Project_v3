# CPCU — Central Processing & Control Unit

**The receiver half of the InfiniTech prosthetic hand system.** A Raspberry Pi 5
that receives wireless EMG packets from the BSAU, runs a real-time DSP + ML
pipeline, and commands 6 servo motors via a PCA9685 PWM driver.

> **First time?** Read `docs/USER_GUIDE.md` — it covers setup through operation.
> This README is the quick reference.

---

## Signal Chain

```
  Forearm (EMG)
       │
       ▼
  ┌─────────────┐    2.4 GHz     ┌──────────────┐    I²C     ┌──────────┐
  │    BSAU     │───────────────▶│     CPCU     │──────────▶│  6 Servos │
  │ STM32L432KC │  1000 pkt/s    │  Raspberry   │  50 Hz    │  MG995 +  │
  │ 8-ch sEMG   │  32 B/pkt      │  Pi 5        │  PCA9685  │  SG90     │
  └─────────────┘                └──────────────┘           └──────────┘
```

**Pipeline:** NRF24L01+ → SPI → Unpack → SPSC Ring → DSP + ML → Smoother → PCA9685 → Servos

## Key Numbers

| Metric | Value |
|--------|-------|
| End-to-end latency | ≤ 51 ms worst, 26 ms typical |
| Wireless input rate | 1000 pkt/s sustained |
| Inference rate | 10 Hz (70% headroom) |
| Safety fault sources | 7, all individually recoverable |
| IPC shared memory | 66 KB at `/dev/shm/cpcu_ipc` |
| Servo update rate | 50 Hz with trapezoidal smoothing |

---

## Quick Start

```bash
# One-time Pi setup (installs deps, enables SPI/I2C, isolates cores)
./launch.sh setup

# Build everything
./launch.sh build

# Verify readiness
./launch.sh check

# Run software-only tests (233 PASS, no hardware needed)
./launch.sh test-sw

# Run with the TUI dashboard
./launch.sh tui

# Run with web dashboard (friends can watch at http://<pi-ip>:8765)
./launch.sh tui --with-ws

# System-level requirements verification (needs BSAU transmitting)
./launch.sh test-system

# Stop
./launch.sh stop
```

`./launch.sh help` lists all commands. `./launch.sh help <cmd>` gives detail.

---

## Core Allocation

| Core | Process | Scheduler | Role |
|------|---------|-----------|------|
| **0** | `cpcu_kernel` + `cpcu_tui` + Linux | CFS | Supervisor, watchdog, TUI, SSH |
| **1–2** | `cpcu_dsp.py` | SCHED_FIFO 80, isolated | DSP filtering + ML inference |
| **3** | `cpcu_io` | SCHED_FIFO 90, isolated | NRF SPI + safety + PCA9685 servo |

Cores 1–3 isolated via `isolcpus=1,2,3 nohz_full=1,2,3` (applied by `setup_pi.sh`).

---

## IPC Layout (`/dev/shm/cpcu_ipc`)

| Region | Size | Mechanism | Direction |
|--------|------|-----------|-----------|
| Control block | 192 B | Atomic flags | Kernel ↔ All |
| Sensor ring | 64 KB | Lock-free SPSC | IO → DSP + TUI |
| Motor command | 128 B | SeqLock | DSP → IO |
| Diagnostics | 128 B | Atomic counters | All → Kernel/TUI |
| DSP export | 256 B | Atomic snapshot | DSP → TUI |
| Runtime config | 512+ B | SeqLock | Kernel → IO/DSP |
| Tool presence | 512 B | Per-slot heartbeat | Tools → WS bridge |
| DSP filtered | 6.4 KB | SeqLock snapshot | DSP → WS bridge |

---

## Safety Monitor

Seven fault sources, all recoverable with hysteresis:

| Source | Threshold | Recovery |
|--------|-----------|----------|
| Radio timeout | 750 ms silence | 10 good packets |
| Battery | ≤ 2.7 V critical | Voltage ≥ 3.0 V |
| DSP stall | No motor cmd for 2 s | Next command received |
| I²C bus | 5 consecutive failures | Next success |
| Thermal | CPU > 82°C | CPU < 70°C |
| Ring overflow | 100 overflows (delta) | 5 s quiescence |
| NRF hardware | SPI register mismatch | Re-init success |

Cold-start grace: 5-second radio timeout suppression on boot (BSAU and CPCU can power on in any order).

---

## DSP / ML Pipeline

Runs on Cores 1–2 in Python:

1. Pop sensor batch from SPSC ring (up to 100 entries).
2. Convert 12-bit ADC → voltage, subtract 1.65 V DC bias.
3. Bandpass filter 20–450 Hz (4th-order Butterworth).
4. 50 Hz notch filter (Q = 30).
5. Sliding window: 200 ms window, 50 ms stride.
6. Extract 7 features × N channels (MAV, RMS, WL, ZC, SSC, VAR, LOG_DET).
7. StandardScaler + RandomForest inference (100 trees).
8. Confidence-scaled velocity integration (gesture → servo rate).
9. Publish motor command via SeqLock.

Velocity mode: holding a gesture integrates servo position over time (graded control). Releasing snaps to "rest" which holds position.

---

## Servo Control

- **Trapezoidal smoother:** acceleration-limited ramp to target, per-servo configurable velocity/accel.
- **Hold-pose deadband:** suppresses redundant PCA writes when settled (kills static jitter).
- **Gravity compensation:** asymmetric velocity bias for weight-bearing joints.
- **Gripper stall watchdog:** if gripper is pinned at mechanical floor for 2 s, retreats to safe position.
- **Runtime tuning:** edit `config/runtime.json` or use the TUI live editor (`./launch.sh tui` → page 7 → `e`).

---

## Logging

Every `./launch.sh` command creates per-process log files under `cpcu_v2/log/`:

```
log/
├── log_kernel_tui_20260511_143022.txt
├── log_tui_tui_20260511_143022.txt
├── log_ws_tui_20260511_143022.txt     (if --with-ws)
├── log_kernel_ws_20260511_150000.txt   (from ./launch.sh ws)
└── log_signal_signal_20260511_160000.txt
```

Format: `log_{process}_{launch-mode}_{timestamp}.txt`. Each file captures the full stdout+stderr of that process for the session.

---

## Dataset Collection

```bash
./launch.sh collect    # guided workflow
```

Files land in `cpcu_v2/datasets/` with the naming convention:

```
datasets/
├── REST_0_filtered.csv
├── REST_1_unfiltered.csv
├── H_OPN_0_filtered.csv
├── BICEP_0_filtered.csv
└── ...
```

Format: `{gesture}_{index}_{filtered|unfiltered}.csv`. Toggle raw/filtered with `t` in the Dataset page.

---

## Web Dashboard

Browser-accessible at `http://<pi-ip>:8765`. Read-only, multi-viewer.

```bash
./launch.sh ws    # fully self-configuring: builds, vendors, starts
```

The command handles everything automatically (setup, Mongoose download, build, kernel start) and prints the URL in a prominent box. Connection info is also shown on the TUI's CONFIG page (page 7) so you always know how to reach the dashboard.

| Tab | Content |
|-----|---------|
| Overview | System state, gesture + confidence, per-channel RMS, diagnostics |
| Waves | 8-channel rolling raw + filtered envelopes |
| Spectrum | Per-channel 256-pt FFT + waterfall (browser-side) |
| Tools | Live tool presence (pca_testbench, signal_testbench) |

---

## File Map

```
cpcu_v2/
├── launch.sh              ← Unified entry point (all commands)
├── CMakeLists.txt         ← Build system
├── config/runtime.json    ← Runtime configuration
├── src/                   ← C production code (16 files)
├── include/               ← C headers (14 files)
├── test/                  ← Testbenches + Python tests (12 files)
├── python/                ← DSP pipeline + IPC bridge (4 files)
├── web/static/            ← Dashboard HTML/CSS/JS
├── web/vendor/            ← Mongoose (fetched on demand)
├── scripts/               ← Shell helpers (setup, configure, run_tests)
└── docs/                  ← Architecture, testing, user guide, web dashboard
    └── diagrams/          ← System block diagrams (SVG)
```

---

## System Requirements Verification

```bash
# Start the system first
./launch.sh tui

# In the SHELL window (Ctrl-b 1), run:
./launch.sh test-system              # 10-second default capture
./launch.sh test-system --duration 30  # longer for stability testing
./launch.sh test-system --json         # JSON output for CI
```

The test monitors live IPC data and verifies all SYS-REQ thresholds:

| Requirement | Target | Measured from |
|-------------|--------|---------------|
| SYS-REQ-01: End-to-end latency | < 300 ms | Pipeline analysis (observation + inference + servo) |
| SYS-REQ-03: Battery | > 2.7 V (not critical) | Live battery voltage from BSAU packets |
| SYS-REQ-04: Signal fidelity | Packet loss < 1% | Sequence gap counter vs received packets |
| SYS-REQ-05: Wireless link | > 900 pkt/s | Packet rate over capture window |
| SYS-REQ-06: Sampling rate | ≥ 2000 Hz | Packet rate × 2 samples/pkt |
| SYS-REQ-08: Safety | State = RUNNING, zero faults | Safety FSM state + SAFE entry counter |
| SYS-REQ-09: Servo accuracy | Update rate ≥ 40 Hz | PCA9685 write cadence |

Plus subsystem checks: IPC version, IO/DSP alive, ring health, CPU temperature, inference latency P50, sequence gap rate. Reports saved to `cpcu_v2/log/system_test_*.txt`.

---

## Documentation

| Doc | Content |
|-----|---------|
| `docs/USER_GUIDE.md` | Setup → build → test → operate + TUI editor reference |
| `docs/ARCHITECTURE.md` | Core allocation, IPC, safety FSM, smoother, velocity mode, soft-grip, jitter |
| `docs/TESTING.md` | Phase-by-phase test guide (233 PASS expected) |
| `docs/WEB_DASHBOARD.md` | Web bridge architecture, JSON protocol, deployment |
| `docs/CONFIGURATION.md` | Runtime config schema (keep your existing copy) |

---

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| Processing unit | Raspberry Pi 5 (≥ 1 GB) | — |
| Radio | NRF24L01+ | SPI0 @ 8 MHz + GPIO 25 (CE) |
| Servo driver | PCA9685 | I²C1 @ 400 kHz, addr 0x40 |
| Servos | 3× MG995 + 3× SG90 | PCA9685 channels 0–5 |
| PSU (Pi) | USB-C 27 W | — |
| PSU (servos) | 6 V / 3 A separate | **Common GND only** |
