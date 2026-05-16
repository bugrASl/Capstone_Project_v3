# Architecture

**Author:** bugrASl  
**Board:** Raspberry Pi 5 (BCM2712, 4x Cortex-A76 @ 2.4 GHz, OC to 2.8 GHz)  
**Radio:** NRF24L01+ (2.4 GHz ISM, Enhanced ShockBurst, PRX role)  
**Transmitter:** BSAU — NUCLEO-L432KC (STM32L432KC, Cortex-M4 @ 80 MHz)  
**DSP/ML:** Python 3 (scipy + scikit-learn SVM)  
**Date:** April 2026

---

## 1. System Overview

CPCU (Central Processing & Control Unit) is the receiver, processor, and actuator controller for the InfiniTech prosthetic hand system. It receives 1000 wireless packets per second from the BSAU wearable EMG sensor, runs real-time DSP and ML inference on the 8-channel EMG data, and drives servo actuators for gesture reproduction.

### 1.1 Signal Chain

```
Electrode -> InAmp -> ADC (32x OS) -> DMA -> WL_Pack -> SPI -> NRF24L01+ (BSAU)
    |                                                              |
    | 2.4 GHz Enhanced ShockBurst, 2 Mbps, 32B payload, 1000 pkt/s |
    |                                                              v
NRF24L01+ (CPCU) -> SPI -> WL_Unpack -> SPSC Ring -> DSP/ML -> Smoother -> Servo
                                                         |           |
                          PCA9685 I2C <- SMOOTH_Update <- Motor Cmd <- Python cpcu_dsp.py
```

### 1.2 Platform Migration: STM32H755 -> Raspberry Pi 5

| Aspect | Old (STM32H755ZI-Q) | New (Raspberry Pi 5) |
|---|---|---|
| Processor | CM7 @ 480 MHz + CM4 @ 240 MHz | 4x Cortex-A76 @ 2.8 GHz (OC) |
| Memory | 64 KB SRAM4 shared | 8 GB LPDDR4X (shared via mmap) |
| IPC | HSEM + SRAM4 ring buffer | Lock-free SPSC ring in mmap'd shm |
| Radio | SPI via HAL on CM4 | SPI via spidev on Core 3 |
| Servo | TIM1/TIM8 PWM on CM4 | I2C PCA9685 PWM driver on Core 3 |
| Servo motion | Instant step | Slew-rate limited smoother (2000 us/s) |
| OS | Bare-metal (HAL) | Linux 6.x + isolcpus |
| DSP/ML | CM7 bare-metal threshold | Python: scipy + sklearn on Cores 1-2 |
| EMG channels | 6 -> 8 | 8 |
| Wireless packet | 32 B (v1 layout) | 32 B (v2 layout: 8 ch + metadata) |

Migration justification: 14.3x inference headroom, 8x ring buffer depth, Python ecosystem for ML, SSH debug access.

---

## 2. Hardware Configuration

### 2.1 GPIO Allocation

```
NRF24L01+ (SPI0):
  GPIO 8   SPI0_CE0  -> NRF_CSN      (active-low chip select)
  GPIO 11  SPI0_SCLK -> NRF_SCK      (SPI clock, 8 MHz)
  GPIO 10  SPI0_MOSI -> NRF_MOSI
  GPIO 9   SPI0_MISO -> NRF_MISO
  GPIO 25             -> NRF_CE       (chip enable, held HIGH in RX mode)
  GPIO 24             -> NRF_IRQ      (not used, busy-poll instead)

PCA9685 (I2C1):
  GPIO 2   SDA1      -> PCA_SDA      (I2C data, 400 kHz fast mode)
  GPIO 3   SCL1      -> PCA_SCL      (I2C clock)
```

### 2.2 Kernel Configuration

```
/boot/firmware/config.txt:
  dtparam=spi=on                     Enable SPI0
  dtparam=i2c_arm_baudrate=400000    I2C Fast Mode (400 kHz)
  arm_freq=2800                      Overclock (2.4 -> 2.8 GHz)
  core_freq=750                      Fixed core clock (stable SPI)
  dtoverlay=disable-bt               Free kernel interrupts
  dtparam=fan_temp0=10000            Force max fan speed
  dtparam=fan_temp0_speed=255

/boot/firmware/cmdline.txt (append to existing line):
  isolcpus=1,2,3 nohz_full=1,2,3 rcu_nocbs=1,2,3
```

### 2.3 Power Architecture

```
Logic domain:   5V / 5A USB-C (Pi 5 + NRF + PCA logic)
Servo domain:   6V / 3A separate supply (fused, 1000 uF decoupling)
Common ground:  REQUIRED between Pi GND and servo supply GND
Isolation:      Wireless link provides galvanic isolation between BSAU and CPCU
```

### 2.4 Servo Configuration (Empirical Limits)

Per-servo min/max derived from EE493 Arduino testbench (PCA9685 counts converted to microseconds via `pulse_us = counts * 20000 / 4096`):

```
CH  Name       Motor  Min us  Max us  Range   Resolution
0   Base       MG995   498    2500    2002 us  0.44 deg/step
1   Upper      MG995  1074    1953     879 us  1.00 deg/step
2   Last       MG995  1074    1953     879 us  1.00 deg/step
3   Joint-1    SG90   1001    2002    1001 us  0.88 deg/step
4   Joint-2    SG90   1001    2002    1001 us  0.88 deg/step
5   Gripper    SG90    976    1733     757 us  1.16 deg/step
```

Neutral position for all servos: 1500 us.

---

## 3. Core Allocation

```
Core 0:   Linux Kernel Core (CFS scheduler)
          |- cpcu_kernel: process supervisor, watchdog, telemetry
          |- cpcu_tui: multi-page ncurses dashboard (via SSH)
          |- cpcu_ws: web bridge (v2.4.0) — read-only WS at :8765
          |- pca_testbench: interactive servo calibration tool
          |- signal_testbench: end-to-end signal integrity tester
          |- safety_testbench: automated safety FSM harness (Phase 1)
          |- SSH, networking, filesystem I/O, logging
          |- RCU callbacks offloaded from isolated cores
          \- Does NOT participate in real-time pipeline

Core 1:   DSP / AI — SMP pair (isolated, tickless)
Core 2:   DSP / AI — SMP pair (isolated, tickless)
          |- cpcu_dsp.py: Python SVM inference (12-feat, RBF kernel)
          |- scipy bandpass (20-95 Hz at Fs=200) + 50 Hz notch
          |- Feature extraction: 4 features × 3 channels = 12 floats
            (RMS, VAR, WL, ENV_mean per s1/s2/s3 → matches team training)
          |- SCHED_FIFO priority 80, mlockall (via taskset)
          \- Ring buffer consumer → Motor command producer

Core 3:   Real-time I/O Controller (isolated, tickless)
          |- cpcu_io: C process, SCHED_FIFO priority 90
          |- NRF SPI busy-poll receiver (2 us poll interval)
          |- WL_Unpack + sequence/link validation
          |- SPSC ring buffer producer
          |- Servo smoother (SMOOTH_Update at 50 Hz, slew-rate limited)
          |- PCA9685 I2C servo driver (50 Hz, writes smooth.current[])
          |- SAFETY_* system-wide safety monitor (7 fault sources)
          \- Heartbeat to shared memory for watchdog
```

### 3.1 Boot parameters that enforce the layout

```
isolcpus=1,2,3        Removes cores 1-3 from the CFS scheduler
nohz_full=1,2,3       Suppresses periodic timer ticks on those cores
rcu_nocbs=1,2,3       Offloads RCU callback work back to core 0
```

Set at `/boot/firmware/cmdline.txt` by `setup_pi.sh`. Without these the
Linux scheduler will load-balance across all four cores and the RT
guarantees collapse. Verify with `cat /sys/devices/system/cpu/isolated`
(should print `1-3`).

### 3.2 Process-to-core mapping

```c
// Excerpts from cpcu_kernel.c showing how the kernel pins each child:

spawn_taskset("IO",   "/opt/cpcu/bin/cpcu_io",   "3",   SCHED_FIFO, 90);
spawn_python ("DSP",  "cpcu_dsp.py",             "1,2", SCHED_FIFO, 80);
//                                               ^      ^           ^
//                                          taskset -c  policy      rt-priority
```

The kernel itself runs as PID 1's child on Core 0 (no explicit
`taskset` because `isolcpus` already keeps CFS-scheduled processes
off the isolated cores). `cpcu_tui`, when launched, also runs on
Core 0 by the same default.

### 3.3 Forward-looking — where upcoming features land

The plan ahead (steps 1-7 of the v2.4 series) introduces several new
behaviours but does **not** change the core allocation. Every new
piece of code lands on an existing core, scheduled by the existing
mechanism. The summary:

| Feature (step) | Code lives in | Runs on | New process? | Status |
|---|---|---|---|---|
| Boot grace (v2.3.1) | `cpcu_safety.c` linked into cpcu_io | Core 3 | No | ✓ shipped |
| Hold-pose deadband (v2.3.2) | `cpcu_smooth.c` linked into cpcu_io | Core 3 | No | ✓ shipped |
| JSON config parser (v2.3.3) | `cpcu_config.{h,c}` linked into cpcu_kernel | **Core 0** | No (kernel gained SIGHUP handler + `--config` CLI) | ✓ shipped |
| Runtime config IPC reads (v2.3.3) | `cpcu_io.c` (cfg_cache, bias-then-clamp) | Core 3 | No | ✓ shipped (`servo_bias_us` is the first consumer) |
| Edit mode handshake (v2.3.4) | TUI / cpcu_dsp.py / cpcu_io / kernel | All four cores cooperating | No | ✓ shipped (handshake mechanism; live numeric editor on top is incremental) |
| Velocity-mode gestures (v2.3.5) | `cpcu_dsp.py` (heavy), `cpcu_io.c` (light) | Cores 1-2 + Core 3 | No | ✓ shipped (dsp owns the integrator; cpcu_io is unchanged — receives integrated targets via the existing motor_cmd channel) |
| pca_testbench round-trip + live smoother tuning (v2.3.6) | `cpcu_config.{h,c}`, `pca_testbench.c`, `cpcu_io.c` | Bench tool + Core 3 | No | ✓ shipped (bench saves servo limits / bias / smoother knobs to runtime.json; cpcu_io re-applies on `config_seq` change) |
| Soft-grip + stall watchdog (v2.3.7) | `cpcu_dsp.py` (soft clamp), `cpcu_io.c` (watchdog) | Cores 1-2 + Core 3 | No | ✓ shipped (dsp prevents integrator from closing past `grip_firm_us`; io retreats to `grip_touch_us` after `grip_stall_recover_ms` pinned at floor; new io_gripper_stalls counter in diag) |
| TUI live editor on top of edit-mode (v2.3.8) | `cpcu_tui_editor.{h,c}`, `cpcu_tui.c`, `cpcu_tui_render.c`, `cpcu_kernel.c`, `cpcu_ipc.h` | Core 0 | No | ✓ shipped (spreadsheet UI for 13 runtime fields; Ctrl+S → `CFG_PatchFile` + SIGHUP via new `kernel_pid` IPC field; IPC_VERSION 0x0204 → 0x0205) |
| CPCU Dashboard — read-only web bridge (v2.4.0) | `src/cpcu_ws.c`, `include/cpcu_json.h`, `src/cpcu_json.c`, `web/static/index.html`, `cpcu_ipc.h`, `cpcu_dsp.py`, `cpcu_ipc_bridge.py` | Core 0 (separate process, CFS-scheduled, no RT prio) | No | ✓ shipped Overview + Waves tabs (Spectrum + Tools deferred to v2.4.1); new `IPC_ToolPresence` + `IPC_DspFiltered` regions; IPC_VERSION 0x0205 → 0x0206; Mongoose-based; multi-viewer; default bind 0.0.0.0:8765 with loud LAN warning |

Three principles drive this:

**JSON parsing belongs on Core 0.** Even the fastest JSON parser takes
hundreds of microseconds — incompatible with cpcu_io's 2 µs poll
budget. The config file is parsed exactly once per startup or SIGHUP,
in cpcu_kernel, which then publishes the result to a small
`IPC_RuntimeConfig` region. RT cores read structured fields from
shared memory in tens of nanoseconds.

**Network I/O belongs on Core 0.** Anything that listens on a socket,
accepts connections, or serializes JSON over the wire is
unbounded-latency work. The WebSocket bridge in v2.4.0 will run on
Core 0 explicitly for this reason — never on the isolated cores.

**The isolated cores stay narrowly focused.** Core 3 is exactly the RT
loop and nothing else. Cores 1-2 are exactly the SVM inference and
nothing else. Adding new features means extending what those
processes do, not adding new processes alongside them.

### 3.4 Why this scales

Core 0 is doing more work than the other three combined: kernel,
networking, all logging, the TUI, eventually the WebSocket bridge,
several optional testbenches. That's intentional — Core 0 is the
"unbounded work" core, where it's OK if a syscall takes 50 ms because
nothing on Core 0 has hard deadlines. The Linux scheduler is good at
sharing one core among many soft-real-time tasks; it's bad at sharing
one core with a hard-real-time task.

When (and if) Core 0 ever saturates — e.g. because we've added a fifth
testbench and 10 simultaneous WebSocket clients during a demo — the
right response is to move work *off* Core 0 to a different machine
(SSH the TUI from a laptop, host the WebSocket dashboard on a
laptop), not to take cores from the RT pool. Topic docs that
introduce new Core-0 features will note their CPU budget.

---

## 4. Inter-Process Communication

### 4.1 Shared Memory Layout (/dev/shm/cpcu_ipc)

```
Offset   Size      Section
0        192 B     IPC_ControlBlock (3 cache lines: header | head | tail)
192      64 KB     IPC_SensorEntry[1024] (ring buffer, 64 B per entry)
65728    128 B     IPC_MotorCommand (SeqLock protected)
65856    128 B     IPC_Diagnostics (per-core atomic counters)
65984    256 B     IPC_DSPExport (Python -> TUI telemetry)
                   ─────────────────────────────────────
Total:   ~66 KB
```

### 4.2 Ring Buffer (SPSC)

```
Size:           1024 entries x 64 bytes = 64 KB
Buffering:      1024 ms at 1000 pkt/s (8x old STM32 design)
Producer:       Core 3 (cpcu_io) — sole writer of sensor_head
Consumer:       Cores 1-2 (cpcu_dsp.py) — sole writer of sensor_tail
Peek access:    cpcu_tui reads ring[(head-N) & MASK] without advancing tail
Sync:           C11 _Atomic with acquire/release ordering (LDAR/STLR on ARMv8)
False sharing:  head and tail on separate 64-byte cache lines
Overflow:       Producer overwrites oldest; consumer detects and skips
```

### 4.3 Motor Command (SeqLock)

```
Writer (cpcu_dsp.py):
  1. seq++ (even -> odd: write in progress)
  2. Write servo_us[6], gesture_id, confidence, timestamp
  3. seq++ (odd -> even: write complete)

Reader (cpcu_io):
  1. Read seq; if odd -> retry
  2. Read all fields
  3. Re-read seq; if changed -> retry
  4. Feed to SMOOTH_SetAllTargets() (not directly to PCA)
```

---

## 5. Servo Smoother (cpcu_smooth)

### 5.1 Purpose

Prevents mechanical shock from instantaneous servo position jumps. Without smoothing, a gesture change (e.g., REST at 1500 us to HAND_CLOSE at 900 us) causes a 600 us step in one 20 ms frame — the servo snaps violently.

### 5.2 Algorithm

Per-channel slew rate limiter running inside the 50 Hz servo update cycle in cpcu_io:

```
For each servo channel:
  max_step = max_speed_us_per_s * dt_seconds
  diff = target - current
  if |diff| <= max_step:
    current = target (settled)
  else:
    current += sign(diff) * max_step
```

### 5.3 Parameters

```
Default speed:  2000 us/s (full 2000 us range in 1.0 second)
Step at 50 Hz:  40 us per frame (20 ms * 2000 us/s)
600 us move:    15 frames = 300 ms (matches CDR latency budget)
Settle thresh:  2 us (within this = "settled")
Tracking:       Float internally (sub-step precision, no drift)
Per-channel:    Each servo can have its own speed
```

### 5.4 Safety Behavior

When SAFETY_CheckSystem() returns false, the smoother does NOT gradually ramp. Instead: `SMOOTH_Snap()` instantly sets all channels to 1500 us (neutral), then `PCA_SetAllNeutral()` writes the physical servos. Instant snap, not gradual, because a safety trigger means something is wrong and the arm must stop immediately.

### 5.5 Integration in cpcu_io

```
Servo update (50 Hz):
  if(safety OK):
    1. IPC_ReadMotorCmd()        -> raw DSP targets
    2. PCA_SafetyClamp()         -> clamp to mechanical limits
    3. SMOOTH_SetAllTargets()    -> feed to smoother
    4. SMOOTH_Update(dt)         -> advance positions
    5. for each servo s:
         if SMOOTH_ShouldWrite(s):                  # v2.3.2 deadband gate
           PCA_SetServo(s, smooth.current[s])       # I²C write
           SMOOTH_MarkWritten(s, smooth.current[s]) # close the loop
  else:
    1. SMOOTH_Snap()             -> instant jump to neutral
    2. PCA_SetAllNeutral()       -> write 1500 us to all channels
    3. for each servo s:
         SMOOTH_MarkWritten(s, neutral)             # keep shadow coherent
```

### 5.6 Hold-pose deadband (v2.3.2)

Once a servo has settled at its target, further PCA writes are
suppressed until either the smoother resumes motion or the target
moves outside the per-channel `hold_deadband_us` (default 10 µs ≈
0.9°). This kills static jitter caused by the servo's internal P
controller being re-triggered by every 50 Hz refresh.

The PCA9685 keeps generating PWM forever once configured — skipping
I²C writes does NOT mean stopping the servo, it means not perturbing
its internal control loop. The deadband is the *only* thing being
suppressed; motion always writes, and the first write of any session
always goes through.

`SMOOTH_ShouldWrite(ctx, ch)` returns the gate decision; consumers
must call `SMOOTH_MarkWritten(ctx, ch, written_us)` after every
successful write to keep the shadow coherent. SAFE-snap and AllOff
paths in cpcu_io update the shadow appropriately. Full design:
[`JITTER_MITIGATION.md`](JITTER_MITIGATION.md).

---

## 6. Safety Monitor (7 Fault Sources)

### 6.1 Radio State Machine

```
          boot
           |
           v
      [RADIO_INIT] ---first packet---> [RADIO_RUNNING]
                                            |
                                       750ms silence (after grace)
                                            |
                                            v
      [RADIO_RUNNING] <--10 OK pkts-- [RADIO_RECOVERING] <--pkt-- [RADIO_DEGRADED]
                                                                       |
                                                                  1500ms total
                                                                       |
                                                                       v
                                                                 [RADIO_SAFE]
                                                                 (terminal until recovery)
```

**v2.3.1 boot grace.** SAFETY_CheckTimeout now suppresses the radio fault for the first `SAFETY_RADIO_BOOT_GRACE_MS = 5000 ms` after `SAFETY_Init`, *if* no packet has yet been received. The first received packet lifts the gate immediately. After the first packet OR after the grace expires, the normal 750 ms timeout applies. This eliminates the spurious cold-start radio fault that used to fire if CPCU was powered on more than ~1 s before BSAU. Genuinely-dead BSAU still flagged, just 5-6 s after boot. Full design in [`BOOT_AND_SYNC.md`](BOOT_AND_SYNC.md).

### 6.2 All Fault Sources

| Source | Detection | Threshold | Action | Recovery (v2.3+) |
|---|---|---|---|---|
| Radio link | No packet received | 750 ms silence (after 5 s boot grace, v2.3.1) | DEGRADED → SAFE | 10 consecutive OK packets |
| Battery | BSAU reports CRITICAL | V_batt ≤ 2.7 V | Immediate SAFE | V_batt > 3.0 V (hysteresis) |
| DSP stall | No motor cmd from Python | 2000 ms | SMOOTH_Snap + neutral | First fresh motor cmd |
| I²C bus | PCA9685 write failures | 5 consecutive | SMOOTH_Snap + neutral | First successful write |
| Thermal | CPU temperature | > 82 °C | Immediate SAFE | T < 70 °C (hysteresis) |
| Ring overflow | Consumer dead / lap | > 100 overflows since baseline | SMOOTH_Snap + neutral | 5 s of no new overflows, baseline reset |
| NRF hardware | Init failure | SPI readback mismatch | Tracked in diagnostics | cpcu_io re-init (3 s interval) |

Any single failure forces servos to neutral via SMOOTH_Snap() (instant, non-gradual). All seven sources are now individually recoverable and converge back to RUNNING through `SAFETY_UpdateState`'s SAFE-exit path: once every flag has been clear for `SAFETY_SAFE_RECOVER_MS = 3000 ms`, the FSM transitions to RECOVERING (radio-induced cause) or directly to RUNNING (any other cause).

**v2.3 ring-overflow recovery.** Earlier versions tied `ring.faulted` to the *cumulative* atomic counter `io_ring_overflows`. Because that counter is monotonic, once the threshold tripped the FSM latched in SAFE forever, even after the producer/consumer rebalanced. The new logic applies the threshold to the *delta* since the last quiescent baseline, and clears the fault after 5 s of no new growth (then re-baselines). The public API of `SAFETY_FeedRingOverflow(ctx, count)` is unchanged — the timer is maintained internally via `clock_gettime(CLOCK_MONOTONIC)`.

**v2.3 SAFETY_UpdateState now wired.** v2.2 introduced `SAFETY_UpdateState()` to centralise the RUNNING ↔ SAFE transitions for non-radio faults (battery, thermal, dsp, i2c, ring), but `cpcu_io.c` was never updated to actually call it. Boolean flags updated correctly and `SAFETY_CheckSystem()` already gated servo writes on them, so the prosthesis was still mechanically safe; but the FSM `state` shown in the TUI never reflected SAFE for non-radio faults — it stayed `RUNNING` while servos were being parked at neutral, which was confusing during diagnosis. v2.3 wires the call into `cpcu_io`'s main loop step 5.

**v2.3 SAFETY_VBAT_DIVIDER fix.** The constant was set to 1.0 in v2.2 with a comment claiming the BSAU firmware would correct for the on-board 2:1 resistor divider, but that BSAU change never shipped. `bsau_adc.c::BSAU_ADC_GetBattery()` returns the raw post-divider 12-bit ADC count, and `bsau_app.c` passes it directly to `pkt.vbat_raw`. With `SAFETY_VBAT_DIVIDER = 1.0`, every healthy 4.0 V battery reported as 2.00 V on the CPCU side, latching `battery.critical = true`. Restored to 2.0 in v2.3.

---

## 7. DSP/ML Pipeline (Python)

### 7.1 Origin and team alignment

The DSP and ML half of the system was developed by the project's
DSP/AI team in four scripts which together define the trained model:

| Script | Role |
|---|---|
| `proccess.py`  | One-time clean-up of raw recordings. 20–450 Hz BP + 50 Hz notch at native Fs. Writes `datasets/2/*.csv` and `dynamic_noise_thresholds.json`. |
| `feature_ex.py`| Reads `datasets/2/`. **Re-filters** at 15–90 Hz + 50 Hz notch at Fs=200, computes 3 Hz envelope, slides 40-sample / 20-stride window, writes 12-feature CSV (`s1_RMS`, `s1_VAR`, …, `s3_ENV`). |
| `model.py`     | Loads the 12-feature CSV. StratifiedGroupKFold(5, seed=33). StandardScaler. SVM RBF C=10, gamma='scale', class\_weight='balanced', probability=True. Saves `hmi_svm_model_200hz.joblib` + `hmi_scaler_200hz.joblib`. |
| `predict.py`   | Live laptop test rig over serial COM10. Reads ASCII `s1,s2,s3` lines at 200 Hz. Filters at 20–450 Hz (auto-clamps to 20–95 Hz), 50 Hz notch, 3 Hz envelope; same 12-feature path; 3-of-5 hysteresis vote; 0.65 confidence threshold. |

`cpcu_dsp.py` is a faithful port of `predict.py`'s signal chain to the
Pi. It uses identical filter coefficients, identical feature
definitions, identical hysteresis logic. The differences are entirely
on the *transport* side: read from `/dev/shm/cpcu_ipc` (binary)
instead of serial (ASCII), 8 channels instead of 3, 2 kHz native rate
that decimates to 200 Hz instead of native 200 Hz.

### 7.2 Pipeline stages

For one 200 ms inference window:

```
ipc.pop_sensor_batch(200)                           // Drain ring at 50 Hz
   ↓ (n samples, 2 sub-samples each, 8 channels each)
buffers[buf_idx].append(raw[ch] - 2048)             // ADC midrail subtract,
                                                    // PA0/PA1/PA2 only

   ↓ Once 200 samples since last window AND 400 samples total accumulated:

w_hi = last 400 samples per active channel          // 200 ms @ 2 kHz
window_lo = scipy.signal.decimate(w_hi, 10,         // → 40 samples @ 200 Hz
                                  zero_phase=True)  // anti-alias + downsample
centered = window_lo - mean(window_lo)              // DC removal
bp = filtfilt(butter(4, [20/100, 95/100], 'band'),  // bandpass; the 450 Hz
              centered)                             // arg auto-clamps to 95 Hz
cleaned = filtfilt(iirnotch(50/100, q=30), bp)      // 50 Hz mains kill
env = filtfilt(butter(4, 3/100, 'low'),             // 3 Hz envelope on |x|
               abs(cleaned))

features = [rms, var, wl, env_mean]                 // 4 per channel
features_flat = concat(features for each of 3 channels)  // 12 floats

   ↓

Xs = scaler.transform(features_flat.reshape(1, -1))
probs = model.predict_proba(Xs)[0]
ai = argmax(probs); label = model.classes_[ai]; conf = probs[ai]

   ↓ Hysteresis (matches predict.py byte-for-byte):

if conf > 0.65 and label != current_state:
    consecutive_count += 1
    if consecutive_count >= 3:
        current_state = label
        consecutive_count = 0
else:
    consecutive_count = 0

   ↓

servo_us = GESTURE_SERVO_MAP.get(current_state, [1500] * 6)
ipc.write_dsp_export(channel_rms, gesture_name, class_confidence, ...)
ipc.write_motor_cmd(servo_us, last_active_class, conf_pct)
```

### 7.3 Classifier configuration

```
Algorithm:        SVM, RBF kernel
C:                10              (regularization parameter)
gamma:            'scale'         (= 1 / (n_features × X.var()))
class_weight:     'balanced'      (compensates "rest" oversampling)
probability:      True            (predict_proba available for hysteresis)
random_state:     42

Feature space:    12-d (3 sensors × {RMS, VAR, WL, ENV-mean})
Pre-scaling:      StandardScaler (mean/std fit on training set)
Train/test split: StratifiedGroupKFold(n_splits=5, seed=33), 1st split
                  → ensures all windows from a single 5 s recording
                    stay together, AND every class is represented in
                    train and test
```

### 7.4 Class set

The trained class set comes from whatever `label` values exist in the
team's labelled CSV files. From `predict.py`'s color_map we know at
least three are present: `rest`, `biceps_flex`, `hand_flex`.

`cpcu_dsp.py`'s `GESTURE_SERVO_MAP` has those three plus `hand_open`
as future-proofing. Mapping behaviour at inference time:

- Class IS in the map ⇒ the corresponding 6 servo µs values get sent
  to `cpcu_io` via the seqlocked motor command IPC entry.
- Class NOT in the map ⇒ falls back to all-neutral (`[1500]*6`).

To diagnose mismatches: at startup `try_load_model()` prints
`model.classes_`; cross-reference with `GESTURE_SERVO_MAP` keys. The
TUI's DSP/AI page (key 3) also shows live `class_confidence` for every
class the model knows about.

### 7.5 Training-vs-live filter discrepancy

The team's training pipeline filters at **15–90 Hz**; the team's live
test rig (`predict.py`) and `cpcu_dsp.py` both filter at **20–450 Hz**
which scipy auto-clamps to **20–95 Hz** at Fs=200. This is a known,
documented inconsistency in the team's pipeline that they have
empirically tolerated:

| Pipeline | Bandpass | Effective passband at Fs=200 |
|---|---|---|
| Training (`feature_ex.py`) | `butter(4, [15/nyq, 90/nyq], 'band')` | 15.0 – 89.8 Hz |
| Live test (`predict.py`) | `butter_bandpass(20, 450, FS)` | 20.1 – 94.9 Hz |
| Live Pi (`cpcu_dsp.py`) | identical to predict.py | 20.1 – 94.9 Hz |

On real EMG (dominant 30–80 Hz energy) the two filters produce
features within ~0.2 % RMS — the model generalises across the gap
fine. Where the discrepancy matters:

- **15–20 Hz motion artifacts** (electrode shift, jaw clench): training
  saw and learned to ignore them; live discards them upstream, so
  *live features are slightly cleaner* in this band than training was.
- **90–95 Hz spectral edge**: live keeps it, training discarded it.
  Minor contribution to RMS for typical EMG.

`cpcu_dsp.py` deliberately matches `predict.py` (the team's live
validation rig), not `feature_ex.py` (the training pipeline), because
if a discrepancy *does* matter, the team's empirical validation has
been done on the live chain. If you later retrain on data captured
through the CPCU pipeline rather than the team's UART rig, the gap
disappears entirely.

### 7.6 Channel mapping (electrode wiring contract)

The team trained on three sensors with these physical positions:

```
s1 = Forearm   (wrist flexor electrode)
s2 = Biceps
s3 = Triceps
```

`cpcu_dsp.py`'s `ACTIVE_CHANNELS = [0, 1, 2]` commits the BSAU-side
wiring to:

```
PA0 (BSAU) → s1 → Forearm
PA1 (BSAU) → s2 → Biceps
PA2 (BSAU) → s3 → Triceps
```

This is a hidden contract — wrong electrode-to-PA wiring won't crash
anything, the SVM just gets garbage in feature\[0..3\] (the model
thinks channel 0 is the forearm when it's actually e.g. the triceps).
If your physical wiring is different, edit `ACTIVE_CHANNELS`.

### 7.7 Noise-threshold calibration (informational)

The team's pipeline has *two contradictory* threshold formulas:

| Script | Formula | Storage | Consumer |
|---|---|---|---|
| `proccess.py`   | `mean(std_per_file) × 3` | `dynamic_noise_thresholds.json` | none |
| `feature_ex.py` | `percentile(rest_envelope, 95) × 1.5` | print-only | none (the consuming code is commented out) |

Neither set of thresholds reaches the trained SVM at inference time.
They're purely diagnostic — the model learns "rest" from the labelled
training data instead. `cpcu_dsp.py`'s `--calibrate N` mode reproduces
the `proccess.py` formula (3×std per channel, written to
`/opt/cpcu/models/noise_thresholds.json`) for the case when you later
retrain on CPCU-collected data and want a similar diagnostic. It is
not read by inference.

### 7.8 Graceful degradation

If `/opt/cpcu/models/{hmi_svm_model_200hz.joblib, hmi_scaler_200hz.joblib}`
exist, `try_load_model()` loads them and inference runs as above. If
either is missing or doesn't load (sklearn version skew, `n_features_in_`
mismatch, …), the function returns `(None, None)` and the main loop:

- Still drains the ring at 50 Hz so the SPSC ring never overflows
- Still extracts and publishes the 12 features so the TUI's DSP/AI
  page lights up with live RMS / WL bars
- Just *skips* the predict-and-decide step, leaving `current_state =
  "rest"` and servos at neutral
- Sets `inference_enabled = False` so the TUI shows "feature-only
  mode" instead of a stale gesture label

The safety FSM is unaffected either way — it gates on radio + battery
+ thermal + ring + I²C, not on inference success. So a missing
`.joblib` degrades the system to a calibration / smoke-test rig
without compromising safety.

### 7.9 Model deployment workflow

The trained `.joblib` artefacts are not in this repo. To deploy:

1. On the team's Windows machine, run `feature_ex.py` (produces
   `features_200hz_segmented.csv`) and then `model.py` (produces
   `hmi_svm_model_200hz.joblib` + `hmi_scaler_200hz.joblib`).
2. SCP both files to the Pi: `scp hmi_*.joblib pi@<ip>:/opt/cpcu/models/`
3. Restart `cpcu_kernel` (`sudo systemctl restart cpcu` or
   `./launch.sh stop && ./launch.sh tui`). The
   spawned `cpcu_dsp.py` will print `[DSP] model + scaler loaded
   (classes=[…])` on stdout, visible in the tmux KERNEL window.

`/opt/cpcu/models/` is owned by your user (set by `setup_pi.sh`), so
the `scp` doesn't need sudo on the Pi end.

---

## 8. TUI System (cpcu_tui v3.4)

### 8.1 Multi-Page Dashboard

The TUI is a single binary (`cpcu_tui`) with **7 switchable pages**. Press `1`/`2`/`3`/`4`/`5`/`6`/`7` to switch. All live-data pages are read-only (peek at shared memory, never consume ring entries); only Dataset (page 6) opens a file for writing while a capture is armed.

**v3.4 page order change:** the static CONFIG spec sheet was moved from page 5 to page 7. Live-data pages now occupy the first six tabs so the most-watched information is on the lowest-numbered keys, and the spec reference is parked at the end.

```
Page 1 — Overview:   Rolled-up HEALTH banner (six green/yellow/red pills +
                     overall NOMINAL/WARNING/DEGRADED verdict), system state,
                     radio link summary, EMG channel bars (% of 4095), 6 servo
                     sliders, battery pack voltage + level, DSP summary, ML
                     classification with softmax bars, filtered RMS per ch.

Page 2 — Radio/IO:   NRF24L01+ status (channel + GHz, address, SPI speed),
                     IO heartbeat age (proves RT loop alive), SAFE-entries
                     counter, battery voltage. Packet stats (total RX, rate,
                     dropped vs gaps distinction, max-poll µs, loss rate,
                     live ring-fill bar, last-packet retry count). Last
                     packet raw fields + decoded BSAU flags banner
                     (CLIP/ELEC/OVRN/TX_SAT/CAL/FIRST).

Page 3 — DSP/AI:     Pipeline stats (DSP windows processed + /s rate,
                     inferences + /s rate, max latency µs, ring fill,
                     underflows, export rate Hz, motor cmd count + /s
                     + age in ms). Active gesture banner with confidence,
                     last inference time µs, export-seq counter ticking.
                     Per-class softmax confidence bars (3-4 classes,
                     active one magenta). Per-channel filtered RMS (bar
                     = % of 0.5 V full-scale + absolute V).

Page 4 — Waveforms:  Live 8-channel line-trace waveforms (' ` - . ,
                     sub-row glyphs with / \ connectors, 5× vertical
                     sub-sampling). Per-channel Hz (from zero-crossing
                     rate), Vpp, Vrms, red CLIP indicator when ADC hits
                     rails. BSAU-flags banner + glyph legend at top.
                     UP/DOWN selects channel, TAB switches to zoomed
                     single-channel detail (adds DC offset + time-axis
                     scale). Peek-based — safe alongside cpcu_dsp.py.

Page 5 — Health:     Traffic-light rollup. 10 subsystem rows (Safety FSM,
                     Radio, IO loop, IPC ring, Pkt integrity, Battery,
                     DSP pipeline, ML export, BSAU sensor, SAFE trips),
                     each [OK]/[WARN]/[FAULT] with a one-line "why"
                     explanation. Top banner tallies N OK | N WARN |
                     N FAULT, shows overall verdict. Put this on a
                     second monitor during hardware testing.

Page 6 — Dataset:    Interactive 8-channel CSV capture. LEFT/RIGHT cycles
                     the gesture label (REST, H.SLO, H.HRD, H.OPN, A.BND<,
                     A.BND=, A.BND>, A.SLO, A.FST, BICEP), s/SPACE
                     starts/stops, r cancels and deletes the partial
                     file, t toggles RAW ADC ↔ FILTERED output. The
                     capture state machine drains the ring every tick
                     regardless of which page is currently rendered, so
                     flipping pages mid-capture does not lose samples.

Page 7 — Config:     Static compile-time + hardware spec reference. Four
                     sections: BSAU/CPCU topology, wireless+IPC layout,
                     motor+ML pipeline, build info (TUI version, compiler,
                     C standard, build date/time). Nothing updates at
                     runtime; this is the "what system am I looking at"
                     page, parked at the end of the tab order in v3.4.
```

### 8.2 Hotkey Reference

**Universal (all pages):**

| Key        | Action                                                |
|------------|-------------------------------------------------------|
| `1`..`7`   | Switch page                                           |
| `q` `Q`    | Quit                                                  |
| `UP`/`DN`  | Select channel (Page 4 only)                          |
| `TAB`      | Toggle grid ↔ single-channel detail (Page 4 only)     |
| `← / →`    | Cycle gesture label (Page 6 only, when not capturing) |
| `s` `SPACE`| Start / stop capture (Page 6 only)                    |
| `t` `T`    | Toggle RAW ↔ FILTERED capture (Page 6 only, idle)     |
| `r`        | Cancel + delete in-progress capture (Page 6 only)     |

**Demo-mode-only** (`cpcu_tui --demo`):

| Key     | Action                                                  |
|---------|---------------------------------------------------------|
| `w` `W` | Cycle waveform: SINE → SQUARE → TRI → SAW → NOISE → EMG → ECG → CHIRP |
| `[`     | Halve frequency (floor 10 Hz)                           |
| `]`     | Double frequency (ceiling 1000 Hz)                      |
| `F`     | Inject radio freeze (triggers DEGRADED→SAFE in 2.25 s)  |
| `B`     | Inject low battery (triggers SAFE on `VBAT_CRITICAL`)   |
| `G`     | Inject sequence-gap storm                               |
| `O`     | Inject ring overflow (auto-clears once burst ends, v2.3)|
| `I`     | Inject I²C error streak                                 |
| `R`     | Master reset: clears all faults AND zeros every counter |

The `R` master reset is implemented by `demo_full_reset()` (v3.4 helper) which clears the injected-fault mask and zeros every cumulative IPC diag counter (packets, gaps, overflows, SAFE entries, inferences, latency, underflows, drops). On Dataset page mid-capture `r` instead cancels and deletes the partial file, since the master reset would be destructive there.

### 8.3 Demo Mode

All TUIs support `--demo` for operation without hardware or shared memory. Demo mode feeds the ring buffer with 100 synthetic sensor packets per frame, exercising the full codec → ring → atomics → seqlock path — so it's a legitimate smoke test, not a stub.

Eight selectable waveforms are generated by a shared header `demo_signals.h` used by both `cpcu_tui` and `signal_testbench`:

```
SINE       pure tone
SQUARE     50 % duty
TRIANGLE   symmetric
SAWTOOTH   rising ramp
NOISE      uniform white centered on 1.65 V
EMG_BURST  1 s rest + 1 s contraction (realistic prosthetic-control signal)
ECG        PQRST template with R-spikes at freq BPM
CHIRP      frequency sweep f → 5f over 2 s, repeating
```

```bash
./cpcu_tui --demo           # Full 7-page TUI with synthetic data
./signal_testbench --demo   # Signal analysis with selectable waveform
./pca_testbench             # Has built-in dry-run if no I2C
./safety_testbench          # Automated safety-FSM harness (33 checks, v2.3)
```

### 8.4 Waveform Renderer (Page 4)

Earlier versions used Unicode block glyphs (`▁▂▃▄▅▆▇█`) which broke on SSH clients without full Unicode support. v3.2 uses an ASCII-only **line-trace renderer**:

- Each column gets a single glyph chosen from `'` `` ` `` `-` `.` `,` by sub-row position (5 sub-cells per row)
- Vertical gaps between adjacent samples are filled with `/` or `\` connectors
- Result: 5× more effective vertical resolution than the row count suggests, and renders correctly on every terminal

The glyph legend is shown in the top banner of Page 4 so readers immediately know what they're looking at.

### 8.5 Per-Module CSV Logging

When launched with `--log` (which `launch.sh` passes automatically), every `LOG_I/W/E/D` call also appends a CSV row to `/var/log/cpcu/log_<module>.csv`:

```
log_kern.csv    supervisor events
log_wdg.csv     watchdog events
log_io.csv      cpcu_io RT-loop events
log_nrf.csv     radio init, retry bursts
log_pca.csv     I²C writes, init reads, failure streaks
```

Format: `timestamp_s,timestamp_us,proc,level,"message"`. Files are `fflush`ed after every write so `Ctrl+C` never loses buffered lines. Directory is created by `launch.sh` with mode 755.

### 8.6 Monitoring via SSH (tmux)

```bash
ssh pi@<pi-ip>
tmux new -s cpcu

# Suggested 4-pane layout:
#   Pane 0: journalctl -u cpcu -f
#   Pane 1: cpcu_tui (press 1-7, use w/[/] in --demo)
#   Pane 2: watch -n 2 "vcgencmd measure_temp; ps -eo pid,comm,psr,pri | grep cpcu"
#   Pane 3: /opt/cpcu/bin/pca_testbench

tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Detach: Ctrl+b d     Reattach: tmux attach -t cpcu
```

During hardware testing, put `cpcu_tui` Page 5 (HEALTH) in pane 1 — it's the fastest way to catch regressions live.

---

## 9. Process Model

```
Process 0: cpcu_kernel (Core 0)
  |- Creates shared memory (IPC_Create)
  |- Spawns cpcu_io via spawn_native("taskset -c 3 chrt -f 90 ./cpcu_io")
  |- Spawns cpcu_dsp.py via spawn_python("taskset -c 1,2 chrt -f 80 python3 cpcu_dsp.py")
  |- Monitors heartbeats (2s timeout -> SIGKILL + respawn)
  |- Pets /dev/watchdog every 5s (15s hardware timeout)
  \- Prints telemetry every 5s

Process 1: cpcu_dsp.py (Cores 1-2, Python)
  |- Opens shared memory (IPCBridge)
  |- Loads SVM model + scaler from .joblib
  |- Ring buffer consumer
  |- scipy DSP pipeline
  |- Motor command producer (SeqLock)
  \- Writes DSP export for TUI

Process 2: cpcu_io (Core 3, C)
  |- Opens shared memory (IPC_Open)
  |- NRF SPI driver (8 MHz busy-poll)
  |- Servo smoother (SMOOTH_Update at 50 Hz)
  |- PCA9685 I2C driver (writes smooth.current[])
  |- SAFETY_* system-wide safety monitor
  |- Ring buffer producer
  \- System state machine
```

---

## 10. Software File Map

```
File                     Layer  Core     What It Does
wireless_packet.h/c      0      any     Byte <-> struct codec (shared with BSAU)
nrf24l01_linux.h/c       1      3       NRF SPI driver (spidev + gpiod)
cpcu_pca9685.h/c         1      3       PCA9685 I2C servo PWM driver
cpcu_smooth.h/c          1      3       Per-channel servo slew rate limiter
cpcu_ipc.h/c             2      all     POSIX shared memory IPC (SPSC + SeqLock)
cpcu_safety.h/c          3      3       System-wide safety monitor (7 sources)
cpcu_log.h/c             3      all     Structured colored logging + per-module CSV
cpcu_io.c                4      3       Real-time I/O main loop + smoother
cpcu_dsp.py              4      1-2     Python DSP + ML pipeline
cpcu_ipc_bridge.py       4      1-2     Python shared memory bridge
cpcu_kernel.c            4      0       Process supervisor
cpcu_tui.c               5      0/SSH   Multi-page ncurses dashboard (6 pages)
demo_signals.h           5      any     Shared 8-waveform generator (cpcu_tui +
                                        signal_testbench, demo mode)
pca_testbench.c          test   0/SSH   Interactive servo calibration TUI
signal_testbench.c       test   0/SSH   End-to-end signal integrity TUI
safety_testbench.c       test   any     Automated safety-FSM harness
                                        (33 checks across 7 test groups, v2.3)
test_codec.c             test   any     Codec round-trip tests
test_ipc_bridge.py       test   any     IPC offset validation
test_dsp_pipeline.py     test   any     DSP filter + feature + model tests
```

---

## 11. Build System (CMake)

### 11.1 How CMake Works

CMake is a build system generator. It reads `CMakeLists.txt` and produces Makefiles. The workflow is: `cmake ..` (configure, detect compilers and libraries) then `make` (build). Out-of-source builds keep the source tree clean.

### 11.2 Our CMakeLists.txt Structure

```
Static libraries (shared code compiled once, linked into multiple targets):
  cpcu_codec    — wireless_packet.c (used by cpcu_io, test_codec)
  cpcu_ipc      — cpcu_ipc.c + librt (used by cpcu_io, cpcu_tui, signal_testbench)
  cpcu_log      — cpcu_log.c (used by cpcu_io, cpcu_kernel)

Executables:
  cpcu_io       — Core 3 RT loop (codec + ipc + log + nrf + pca + safety + smooth)
  cpcu_kernel   — Core 0 supervisor (ipc + log)
  cpcu_tui      — Dashboard (codec + ipc + ncurses) [guarded by CURSES_FOUND]
  pca_testbench — Servo test (pca9685 + ncurses) [guarded by CURSES_FOUND]
  signal_testbench — Signal test (codec + ipc + ncurses) [guarded by CURSES_FOUND]
  safety_testbench — Automated safety FSM harness (safety + ipc, Phase 1)
  test_codec    — Unit tests (codec only, no hardware)

Install targets:
  /opt/cpcu/launch.sh     — the user CLI (v2.7: at top level)
  /opt/cpcu/bin/          — cpcu_io, cpcu_kernel, cpcu_tui, cpcu_ws,
                            pca_testbench, signal_testbench, safety_testbench
  /opt/cpcu/python/       — cpcu_dsp.py, cpcu_ipc_bridge.py
                            (v2.7: was scripts/, renamed to reflect contents)
  /opt/cpcu/scripts/      — setup_pi.sh, configure.sh, run_tests.sh
                            (v2.7: helpers only, called by launch.sh)
  /opt/cpcu/test/         — test_ipc_bridge.py, test_dsp_pipeline.py
  /opt/cpcu/models/       — hmi_svm_model_200hz.joblib + scaler (operator-deployed)
  /etc/systemd/system/    — cpcu.service (generated by ./launch.sh install-service)
```

### 11.3 Building Individual Targets

```bash
cd build
cmake --build . --target cpcu_io            # Just the RT loop
cmake --build . --target pca_testbench      # Just the servo testbench
cmake --build . --target signal_testbench   # Just the signal testbench
cmake --build . --target safety_testbench   # Just the safety harness
cmake --build . --target test_codec         # Just the unit tests
```

---

## 12. Systemd Service (cpcu.service)

### 12.1 What It Does

The systemd unit file manages the CPCU as a Linux service. The boot chain is: `power on -> systemd -> cpcu.service -> launch.sh -> cpcu_kernel -> fork(cpcu_io) + fork(cpcu_dsp.py)`.

### 12.2 Key Configuration

```
Type=simple         — systemd tracks the PID of launch.sh (which exec's into cpcu_kernel)
ExecStart           — /opt/cpcu/launch.sh (pre-flight checks then exec)
Restart=on-failure  — auto-restart on crash, 5s delay, max 5 attempts per minute
LimitRTPRIO=90      — allows SCHED_FIFO up to priority 90 (cpcu_io needs this)
LimitMEMLOCK=infinity — allows mlockall() (prevent page faults on RT cores)
KillMode=control-group — SIGTERM goes to ALL processes (kernel + io + dsp), not just PID 1
WatchdogSec=30      — systemd kills service if unresponsive for 30s (second safety layer)
```

### 12.3 launch.sh Pre-Flight Checks

Before starting cpcu_kernel, launch.sh validates: binaries exist, Python dependencies available, DSP script present, ML model file present, core isolation active, SPI device present, I2C device present. Warnings are logged but non-fatal (system can run without DSP, for example).

---

## 13. Shell Scripts

### 13.1 setup_pi.sh — One-Time Pi Configuration

Run once on a fresh Raspbian install. Installs apt packages (build-essential, cmake, libncurses-dev, i2c-tools, python3-numpy, python3-scipy), pip packages (joblib, scikit-learn), creates /opt/cpcu directory structure, configures SPI/I2C/core isolation in boot config files, sets permissions for I2C/SPI groups.

### 13.2 launch.sh — Boot Script

Called by systemd at boot. Does pre-flight checks then `exec taskset -c 0 ./cpcu_kernel` which replaces the shell process with the kernel supervisor. The `exec` is important: systemd tracks the cpcu_kernel PID directly, so signals (SIGTERM on `systemctl stop`) go to the right process.

### 13.3 run_tests.sh — Test Runner

Supports both automated phases and interactive testbenches:

```bash
./launch.sh test-hw   # All automated phases (1 2 3)
./launch.sh test            # Phase 1 only (software, no hardware)
./launch.sh test-ipc          # Phases 1 and 2
./launch.sh test-pca          # Launch PCA9685 servo testbench TUI
./launch.sh test-signal       # Launch signal integrity testbench (live data)
./launch.sh test-signal-demo  # Launch signal testbench (synthetic, no hardware)
```

Interactive phases (`pca`, `signal`, `signal-demo`) use `exec` to replace the shell with the TUI binary, so quitting the TUI returns you to your terminal cleanly.

---

## 14. Performance Summary

| Metric | Value | Margin |
|---|---|---|
| Packet throughput | 1000 pkt/s | NRF air: 16.5% utilized |
| Core 3 utilization | 4.6% | 95.4% idle |
| Core 1-2 utilization (Python) | ~30% peak | 70% headroom |
| Ring buffer depth | 1024 ms | 8x old design |
| ADC-to-servo latency (typical) | ~26 ms | 11.5x below 300 ms target |
| Servo smoother latency | 300 ms for full range | Matches CDR budget |
| NRF poll latency | 2 us | 5-25x faster than IRQ |
| Servo resolution | 0.44 deg (Base) | Below mechanical backlash |

---

## 15. Design Risks and Mitigations

| Risk | Severity | Mitigation |
|---|---|---|
| Linux is not a real-time OS | Medium | isolcpus + nohz_full + rcu_nocbs + SCHED_FIFO + mlockall |
| Python GC pauses | Low | gc.disable(). Ring buffer 1024 ms deep absorbs any pause |
| filtfilt vs sosfilt mismatch | Medium | test_dsp_pipeline.py validates. Retrain if accuracy drops > 5% |
| I2C bus stall blocks radio poll | Low | PCA clock-stretch max 100 us. NRF FIFO depth 3 absorbs 1 missed poll |
| sklearn version mismatch | Medium | Pin versions. ONNX export as fallback |
| Servo stall current exceeds PSU | Medium | 3A PSU may be insufficient for 6-servo stall. Add 1000 uF cap or upgrade to 6A |
| Thermal throttle at 85C | Low | SAFETY_FeedTemperature forces SAFE at 82C |
| Servo snap on gesture change | Eliminated | cpcu_smooth slew limiter: max 40 us/frame at 50 Hz |


---

# Appendix A: Boot and Sync — Cold-Start Choreography

> **Merged from:** `BOOT_AND_SYNC.md` (v2.3.1).
> Previously a standalone doc; folded into ARCHITECTURE as it details
> a core safety subsystem behaviour.

## TL;DR

Pre-v2.3.1, you had to power on CPCU **after** BSAU was already
transmitting (or hold BSAU in reset until CPCU was ready) to avoid
the system declaring a radio fault during the first second of boot.
This was annoying and led to the workaround of holding the BSAU
reset button while booting CPCU, then releasing it.

v2.3.1 adds a **5 second cold-start grace period** to the radio
timeout. You can now power on CPCU and BSAU **in any order** with
several seconds of slack between them, and the system reaches
steady state without ever spuriously entering SAFE.

The grace doesn't compromise safety: if BSAU is genuinely dead
(unconnected, dead battery, broken NRF), CPCU still flags the radio
fault — just 5 seconds later than during normal operation. That
delay is acceptable for cold-boot, unacceptable for runtime drops.

---

## What "sync" means in this system

The BSAU and CPCU are two independent processors with no direct wire
connection. Their only shared channel is the 2.4 GHz NRF link. So
"sync" doesn't mean clock synchronization or handshake negotiation —
it means **they have to start agreeing about packet ordering and
timing without coordinating their boot moments.**

There are three sync facets to think about, each handled by a
different mechanism:

### Facet 1 — Sequence number alignment

The BSAU emits a `seq` byte in every packet, incrementing 0..255 then
wrapping. CPCU keeps an `expected_seq` and reports a "gap" whenever
the received seq doesn't match.

**Problem on cold start:** When CPCU starts after BSAU has already
been emitting packets, CPCU's `expected_seq` starts at 0 but the
incoming packet might have seq=137. That's a "gap" of 137 packets
which would (a) flood the gap counter, (b) trigger the seq-gap-storm
fault if the gap is big enough.

**Mechanism:** BSAU sets `WL_FLAG_FIRST_PACKET` in the flags byte of
its very first packet after boot (and after any NRF recovery).
CPCU's `SAFETY_FeedPacket` checks that flag and, when set, copies
the incoming `seq` into `expected_seq` directly — initializing the
counter from the BSAU's perspective rather than from CPCU's
arbitrary starting value.

**Result:** No spurious gap on the first packet, regardless of
power-on ordering.

This was already in place before v2.3.1.

### Facet 2 — Radio fault timeout

CPCU watches the elapsed time since the last received packet
(`SAFETY_CheckTimeout`). If silence exceeds `SAFETY_RADIO_TIMEOUT_MS`
(750 ms), the radio is declared faulty and the FSM transitions
RUNNING → DEGRADED → SAFE.

**Problem on cold start:** If CPCU boots before BSAU, the silence
counter starts ticking from CPCU's startup. If BSAU takes longer
than 750 ms to reach steady state and start transmitting (which is
normal — STM32 boot + NRF init takes ~600 ms by itself, plus any
delay from the user pressing the reset button), CPCU declares a
spurious radio fault and parks the arm at neutral. Then when BSAU
finally starts transmitting, CPCU has to wait through the 3 second
SAFE-recovery hold before it can use the data.

**Mechanism (v2.3.1):** A new "boot grace" gate in
`SAFETY_CheckTimeout`. The radio timeout is suppressed if both:

  - No packet has ever been received (`first_packet_seen == false`)
  - The boot grace period has not yet elapsed
    (`now - boot_us < SAFETY_RADIO_BOOT_GRACE_MS`)

Once either condition flips, normal timeout semantics resume.

**Result:** The first 5 seconds after CPCU boot are tolerant of
silence. After that, OR after the first received packet (whichever
comes first), normal 750 ms timeout applies.

### Facet 3 — Battery state cold-start

CPCU's safety FSM tracks battery hysteresis (low / critical /
recover) based on the `vbat_raw` field in each packet. With no
packets, there's no battery reading, so the safety check defaults to
"battery OK".

**Problem:** No problem in practice. If BSAU eventually starts
transmitting with a critical battery, the next FeedPacket transitions
the FSM into the battery fault. There's no race window here because
the absence of battery data is treated as no-fault (rather than
defaulting to "assume the worst").

**Mechanism:** None needed. The hysteresis design treats the absence
of new data as "no change", so battery state simply waits for first
data.

---

## Why 5 seconds and not 1 second or 30 seconds

### Lower bound: 3 seconds

BSAU's reset → first transmit takes about 600 ms in the well-behaved
case. With v2.4's bounded-retry NRF init, a marginal radio rail can
add another ~400 ms (200 ms POR delay + 100 ms backoff × 2 retries).
That's already 1 second consumed by BSAU's normal boot sequence.

If you're powering on with a switch (rather than carefully
coordinating reset releases), there's an additional human reaction
time of ~500 ms between flipping CPCU on and remembering to flip
BSAU on.

3 seconds gives a comfortable margin without making the user wait.

### Upper bound: ~10 seconds

Two interactions push back against making this much longer:

**`SAFETY_AUTO_CLEAR_MS` is 300 seconds (5 minutes).** If a fault
clears, the cumulative diagnostic counters auto-zero. With a grace
of 5 seconds, the system is well clear of any auto-clear corner case.
A grace of 30+ seconds starts feeling like "the fault detection is
just slow on boot" rather than "we're waiting for sync".

**Genuinely-dead BSAU diagnosis.** If BSAU is broken (NRF chip dead,
power rail failed, MCU not booted), CPCU should flag the problem.
Today it does so 750 ms after BSAU stops transmitting; with 5 second
grace, the very first detection takes 5 seconds 750 ms. Acceptable.
With 30 seconds grace, you'd be looking at the dashboard for half a
minute thinking "everything's fine" while it actually isn't.

5 seconds is the sweet spot: long enough to absorb realistic boot
timing variance, short enough that genuine boot failures are flagged
within 6 seconds.

### Why it's a `#define`, not a runtime config

Like all safety thresholds (`SAFETY_RADIO_TIMEOUT_MS`,
`SAFETY_VBAT_CRITICAL_V`, etc.), the grace period defines what
"safe" means. Live-tunable safety thresholds are a recipe for a
misclick disabling protection. So this lives in `cpcu_safety.h`
where it can only be changed via `configure.sh` (when that lands in
v2.3.3) or by directly editing the header and rebuilding.

---

## Implementation details

### Code change footprint

Three small additions:

`cpcu_v2/include/cpcu_safety.h`:
```c
#define SAFETY_RADIO_BOOT_GRACE_MS      5000
```
And two new fields in `SAFETY_Context`:
```c
uint64_t        boot_us;
bool            first_packet_seen;
```

`cpcu_v2/src/cpcu_safety.c`, `SAFETY_Init`:
```c
ctx->boot_us = safety_now_us();
/* memset above already cleared first_packet_seen */
```

`cpcu_v2/src/cpcu_safety.c`, `SAFETY_FeedPacket`:
```c
ctx->first_packet_seen = true;     /* added at top */
```

`cpcu_v2/src/cpcu_safety.c`, `SAFETY_CheckTimeout`:
```c
if(!ctx->first_packet_seen &&
   (now_us - ctx->boot_us) / 1000 < SAFETY_RADIO_BOOT_GRACE_MS)
{
    return;
}
/* ...existing timeout logic unchanged... */
```

That's the entire mechanism. No changes to cpcu_io.c, no changes to
the IPC layout, no changes to the BSAU side.

### Why `first_packet_seen` is separate from `seq_init`

`seq_init` already exists; it's set when a packet with
`WL_FLAG_FIRST_PACKET` is seen. Couldn't we reuse it for the grace
gate?

No, for two reasons:

1. **A BSAU restart re-asserts FIRST_PACKET.** After v2.4's NRF
   recovery, the BSAU sends a new FIRST_PACKET-flagged packet, which
   would re-set `seq_init`. If the grace gate used `seq_init`, every
   NRF recovery would re-enter grace mode — defeating the whole
   point of the timeout for any BSAU restart after the first one.

2. **`first_packet_seen` is intentionally a "we've ever heard from
   them" flag, not a "they've identified themselves" flag.** A bug
   in BSAU that drops the FIRST_PACKET flag should still let the
   grace expire on first packet receipt. Decoupling them keeps the
   grace logic robust.

### Compatibility

- **No public API change.** No new arguments, no removed functions.
  Existing callers compile and link unchanged.
- **No IPC schema change.** The new fields live in the in-process
  SAFETY_Context, which is private to cpcu_safety.c (and tests).
- **No test regressions.** Existing tests go through `warm_up`
  which calls `SAFETY_FeedPacket` 20 times, setting
  `first_packet_seen = true` long before any `CheckTimeout` call.
- **Profile-independent on BSAU side.** No BSAU code touched. All
  BSAU profiles (RELEASE / DEBUG / DATASET / TEST_*) emit packets
  with FIRST_PACKET set the same way; CPCU's grace gate doesn't care
  which profile is on the other end.

### Interaction with existing safety mechanisms

- **NRF recovery (BSAU v2.4):** When BSAU recovers its NRF mid-session
  and resumes transmitting, `first_packet_seen` is already true on
  the CPCU side, so the grace gate is a no-op. The radio timeout
  uses normal 750 ms semantics. Recovery feels identical to v2.3.0.
- **Ring overflow recovery (CPCU v2.3):** Independent of grace.
  Different fault, different recovery timer.
- **SAFE auto-recovery (`SAFETY_SAFE_RECOVER_MS = 3 s`):** Unchanged.
  If the system *does* enter SAFE (e.g. after a real fault), the
  3-second hold-clear-recover semantics still apply.
- **Auto-clear of cumulative counters (`SAFETY_AUTO_CLEAR_MS =
  300 s`):** Unchanged. Grace expires long before auto-clear is
  relevant.

---

## Testing

`safety_testbench` gained a TB-SAF09 group with four sub-checks:

| Test | Setup | Expected |
|---|---|---|
| TB-SAF09a | RUNNING + 2 s of silence inside grace | stays RUNNING |
| TB-SAF09b | RUNNING + grace expired with no packets | transitions to DEGRADED |
| TB-SAF09c | First packet during grace, then 800 ms silence | normal timeout fires (DEGRADED) |
| TB-SAF09d | Post-warmup timeout (regression test) | unchanged 750 ms behaviour |

Run from `cpcu_v2/`:
```bash
build/safety_testbench
# Expected: 38 PASS, 0 FAIL  (was 33 pre-v2.3.1)
```

Or via the full Phase 1:
```bash
./launch.sh test
# Expected: 7 + 38 + 65 = 110 PASS
```

---

## Operating procedure (the practical impact)

### Before v2.3.1

```
Power on CPCU → wait for kernel to start → quickly press BSAU reset
  → release reset within 750 ms → both running together
  → if you're slow, system enters SAFE for 3 seconds
```

The "manual reset dance" was annoying for development and
unacceptable for a real prosthetic.

### After v2.3.1

```
Power on either side. Wait. Power on the other side. Done.
```

Three valid orderings, all handled identically:

- **CPCU first**: CPCU boots, sees no packets, sits in grace, waits.
  BSAU boots later, transmits, CPCU lifts grace, normal operation.
- **BSAU first**: BSAU emits packets to nobody. CPCU boots, immediately
  starts receiving, lifts grace, normal operation.
- **Together**: Whichever finishes initializing first; the
  late-starter joins normally.

### Edge case: BSAU is dead

```
Power on CPCU. Wait 5 seconds. Radio fault triggers. Arm parks at
neutral. TUI shows RADIO_SAFE in red. Diagnostics show "no packet
received".
```

This is the *correct* behaviour. The grace doesn't hide failures; it
just gives them a 5-second sync window before flagging.

### Edge case: BSAU starts late

```
Power on CPCU at t=0. BSAU starts at t=20s.
- t=0 to t=5s:    grace active, sits in RUNNING.
- t=5s to t=5.75s: grace expired, last_pkt_rcv_us = boot_us, silence
                   already huge → fault.
- t=5.75s:        DEGRADED.
- t=7.25s:        SAFE (DEGRADED + 1.5s).
- t=20s:          BSAU starts transmitting.
- t=20s+ε:        FeedPacket lifts first_packet_seen, but state ==
                   SAFE so we wait for SAFE_RECOVER.
- t=20s + 3s:     SAFE → RUNNING via normal recovery path.
```

So a 20-second-late BSAU costs about 3 seconds of recovery, which
matches the normal "we lost the radio for a while" recovery time.
Reasonable.

---

## See also

- [`ARCHITECTURE.md`](ARCHITECTURE.md) §6 — full safety FSM
- [`cpcu_safety.h`](../include/cpcu_safety.h) — header with the new constant
- [`cpcu_safety.c`](../src/cpcu_safety.c) — `SAFETY_CheckTimeout` and `SAFETY_FeedPacket`
- [`safety_testbench.c`](../test/safety_testbench.c) — TB-SAF09
- BSAU v2.4 NRF non-fatal init (`bsau_v2/docs/BSAU_ARCHITECTURE.md` §7) — the
  other half of "make the system tolerant of cold-start oddities"


---

# Appendix B: Velocity-Mode Gestures — Stateful Target Integration

> **Merged from:** `VELOCITY_MODE.md` (v2.3.5).
> Describes the integration formula and hybrid freeze/velocity model
> that lives inside cpcu_dsp.py.

## TL;DR

Until v2.3.4, every gesture mapped to a fixed servo pose: hold
`biceps_flex` → arm at position X, holding longer doesn't move the
arm any further. Visible behaviour was a step function of detected
class.

v2.3.5 adds **velocity mode**. Per-gesture, per-servo rates (in
µs/s) live in `runtime.json`'s `gesture_velocity` object. While a
velocity-mode gesture is detected, cpcu_dsp.py **integrates** the
servo target every inference tick:

```
target[s] += rate[s] × dt × confidence_scale
```

Holding the gesture longer = arm closes deeper. Releasing snaps to
"rest" (freeze mode) which holds the last position. Re-engaging the
gesture continues integrating from where you left off.

`rest` is special: it's always freeze-mode and snaps target back
toward neutral, so a long rest period drains the arm to home pose.

The system is **hybrid**: any class without a velocity entry stays
in freeze mode using the legacy `GESTURE_SERVO_MAP` fixed pose,
preserving v2.3.4 behaviour as the safe default.

---

## 1. Why velocity mode

The original fixed-pose model had two problems:

**No fine control.** A flex either snapped the elbow to 1700 µs or
left it at neutral. There's no way to express "close partway",
"close more", or any in-between state. Real prosthetic users want
graded control — squeeze gently, then squeeze harder.

**No state persistence.** A momentary EMG dropout (signal noise,
brief muscle relaxation, classifier hiccup) immediately collapses
the arm back to neutral. The user has to re-flex from scratch.

Velocity mode fixes both:
- Hold the gesture for 2 s instead of 1 s → arm moves twice as far.
- Brief detection dropouts hold the current target instead of
  resetting it. The arm "remembers" where it was.
- Confidence-scaled integration speed means weak/wavering gestures
  creep slowly while strong sustained gestures move at full speed.

---

## 2. The integration formula

Run every inference tick (~10 Hz):

```
dt              = now - last_integrate_t
conf_frac       = svm_probability(current_state) / 100.0

if conf_frac <= interp_floor:           # 0.40 default
    scale = 0.0                         # gesture too weak; freeze
elif conf_frac >= interp_ceil:          # 0.85 default
    scale = 1.0                         # full speed
else:
    scale = (conf_frac - floor) / (ceil - floor)    # linear lerp

if behavior[current_state].mode == "freeze":
    if current_state == "rest":
        target = neutral                # rest drains to home
    else:
        target = target                 # other freeze classes hold
else:    # velocity mode
    for s in range(NUM_SERVOS):
        target[s] += rate[s] * dt * scale
        target[s] = clamp(target[s], SERVO_MIN_US[s], SERVO_MAX_US[s])

publish_motor_cmd(target)
```

A few things worth noting:

**`dt` is real wall-clock**, not a fixed cadence assumption. The
inference loop *aims* for 100 ms but actual ticks vary with system
load. Using real `dt` keeps motion smooth across jitter.

**Confidence-scaled integration is intentional.** A high-confidence
sustained gesture moves at full velocity. A wavering 50% confidence
signal moves at ~25% velocity. This couples the SVM's uncertainty
directly into how aggressively the arm responds — uncertain user =
slow arm = safer.

**Clamping is per-servo.** Each channel has its own min/max
(mechanical limits, NOT the runtime-tunable `servo_min_us`). Once a
channel saturates, further integration is a no-op. cpcu_io clamps
again on its side after applying `servo_bias_us`, so the runtime
config can't escape the safety envelope.

**The integrator runs only outside edit mode.** While
`edit_mode_request` is set, dsp commits to "rest", clears the target
to neutral, and stops publishing — see TUI_EDITOR.md §4.

---

## 3. Configuration shape

The schema is documented in
[`CONFIGURATION.md`](CONFIGURATION.md) §2. The relevant subset:

```json
{
    "interp_conf_floor_pct": 40,
    "interp_conf_ceil_pct":  85,
    "hysteresis_votes": 3,
    "gesture_velocity": {
        "rest":         [0, 0, 0, 0, 0, 0],
        "biceps_flex":  [0, 200, 0, 0, 0, 0],
        "hand_flex":    [200, 0, 0, 200, 200, 200],
        "hand_open":    [-200, 0, 0, -200, -200, -200]
    }
}
```

### Key rules

- **Class names must match `model.classes_` exactly.** Unknown names
  get a warning and are dropped. Renaming a class in the trained
  model means you have to update this JSON to match.
- **6 entries per row, indexed by servo channel.** Order:
  S0=Base, S1=Upper, S2=Last, S3=Joint1, S4=Joint2, S5=Gripper.
- **Negative values reverse direction.** A `hand_open` row with
  negative rates is the natural antagonist of `hand_flex` —
  same magnitude, opposite sign.
- **All-zero rate row is freeze-mode.** Equivalent to omitting the
  row entirely.
- **Range is ±5000 µs/s.** Out-of-range values get clamped loudly
  rather than silently accepted or rejected. 5000 µs/s = full servo
  range (498→2500 ≈ 2000 µs of travel) in 0.4 seconds — already
  faster than most users want.

### Tuning advice

Start gentle: 100-200 µs/s. With the default `interp_floor=40` and
`interp_ceil=85`, an active gesture at 80% confidence integrates at
~89% × rate = ~178 µs/s. So 200 µs/s feels like "noticeable motion
over ~1 second". 500 µs/s feels rapid; 1000+ µs/s is hard to control
in real-time EMG.

If the arm overshoots, lower the rate. If you can't tell the gesture
is engaged, raise the rate or lower `interp_floor_pct`.

---

## 4. Runtime/static split

The dsp side reads `runtime.json` directly using Python's
`json` module — separate from the C-side parser in `cpcu_config.c`
that publishes `IPC_RuntimeConfig` to shared memory.

Why two parsers? The string-keyed `gesture_velocity` map doesn't
fit the C parser's flat-array model. Adding strings + nested
objects to the C parser would be ~150 lines of code for a feature
that only one consumer (dsp) needs. Doing the parse in Python is a
few `json.loads` calls.

**Consequence**: the IPC `gesture_velocity[][]` field (declared in
`cpcu_ipc.h` since v2.3.3) stays zero-filled in v2.3.5. It exists
only as forward-compat reserve for a possible future C consumer
(no current need).

**Reload semantics**: dsp re-reads `runtime.json` only on full
restart. SIGHUP to cpcu_kernel re-parses the C-side fields and
republishes IPC, but does NOT re-trigger dsp's reload. To pick up
new gesture rows you must restart cpcu_dsp.py:

```bash
sudo systemctl restart cpcu                  # restarts the whole bundle
# or
kill $(pgrep -f cpcu_dsp.py)                 # supervisor respawns it
```

This is intentional — gesture velocities meaningfully change
behaviour, and a hot-swap mid-gesture would be jarring. Restart is
the right cadence.

A future revision could add SIGHUP support to dsp specifically for
this field. Not yet.

---

## 5. Hybrid behaviour: freeze + velocity coexist

Not every class needs to be velocity-mode. A class without a
`gesture_velocity` row stays in freeze-mode with its existing
`GESTURE_SERVO_MAP` fixed pose. This means:

- v2.3.4 deployments upgrade to v2.3.5 with **no behaviour change**
  if the user doesn't add `gesture_velocity` rows.
- A subset of classes can be velocity (e.g. just `biceps_flex` and
  `hand_flex`) while others (e.g. a calibration `wave` class) stay
  fixed-pose.
- "rest" is hardcoded to freeze. It exists as the explicit "stop
  integrating" semantic — re-routing rest to a velocity row would
  fight the integrator and is not allowed.

The decision tree dsp uses, per inference tick:

```
if current_state in behavior_map:
    mode = behavior_map[current_state].mode
    if mode == "velocity":  → integrate (above formula)
    else:                   → freeze (hold target; if rest, snap to neutral)
elif current_state in GESTURE_SERVO_MAP:
    target = GESTURE_SERVO_MAP[current_state]      # legacy v2.3.4 fallback
else:
    target = neutral                                # defensive default
```

In practice the first branch wins for every class once you've added
`gesture_velocity` entries. The third branch should never fire if
the SVM and the JSON agree on class names — but it's there as a
safety net for class-name drift between training and deployment.

---

## 6. Boot rule, fault recovery, edit-mode interaction

**Boot.** Every fresh start initializes `current_target_us =
[neutral]*6`. There is **no snapshot persistence** across runs by
design: a stuck pose from yesterday's session shouldn't define
today's startup. On boot the arm is at neutral, and integration
proceeds from there.

**Fault recovery.** When `system_state` transitions to SAFE
(any safety FSM trigger), dsp snaps `current_target_us` back to
neutral. cpcu_io has already snapped the smoother to neutral on its
side, but dsp's mirror of the target needs explicit reset too —
otherwise when SAFE clears, dsp resumes integration from the
mid-gesture target it had at fault time, which is surprising.

**Edit mode.** While `edit_mode_request` is set, dsp commits state
to "rest", suspends publishing, and clears `current_target_us` to
neutral so the next exit doesn't snap to a held pose. cpcu_io's
edit-mode handshake (see TUI_EDITOR.md §4) parks the arm at neutral
in parallel.

**Combination cases:**
- Boot + immediate SAFE: target stays at neutral (already there).
- Edit-mode entry mid-flex: target goes to neutral; cpcu_io parks.
- SAFE-clear after edit-mode exit: integration resumes from
  neutral (both edit-mode-exit and SAFE-clear reset to neutral).
- Edit-mode entry while in SAFE: edit-mode handshake completes
  (smoother is already at neutral, settles immediately).

---

## 7. Testing

`test/test_dsp_pipeline.py` adds six new tests for the runtime
config loader (TB-DSP11..TB-DSP16, 18 individual checks):

| Group | What |
|---|---|
| TB-DSP11 | Missing config file → defaults, no crash |
| TB-DSP12 | Velocity rows parsed correctly (positive + negative rates) |
| TB-DSP13 | Out-of-range rates clamped to ±5000, not silently accepted |
| TB-DSP14 | Unknown class names dropped with a warning |
| TB-DSP15 | floor ≥ ceil rejected (loader falls back to defaults) |
| TB-DSP16 | JSONC line comments + trailing commas tolerated |

```bash
./launch.sh test
# RESULTS: 186 PASS in Phase 1 (was 168)
#   7 codec + 38 safety + 28 smoother + 30 config + 83 DSP
```

The integration math itself isn't unit-tested in v2.3.5. It runs
inside `run_inference()` which has no clean seam for synthetic
input — the test would have to mock `time.monotonic()` and
`ipc.write_motor_cmd`. Reasonable to add later but not blocking.

For end-to-end verification, the live system on hardware:

```bash
sudo ./launch.sh release

# Hold biceps_flex for 1 second, watch elbow servo position
# (any servo monitor or scope on the PCA output) — should ramp
# smoothly from neutral toward closed, not snap there.

# Release. Position holds (freeze mode on rest is "snap to neutral
# over the smoother's trapezoidal walk").

# Re-flex briefly. Position resumes ramping from where it stopped,
# not from neutral.
```

---

## 8. Operating procedure

### First-time setup of velocity gestures

```bash
# Edit runtime.json with your initial guess at velocities:
$EDITOR cpcu_v2/config/runtime.json

# Restart so dsp picks up new gesture map:
sudo systemctl restart cpcu

# Watch the log to confirm rows loaded:
journalctl -u cpcu -n 20 | grep "velocity-mode"
# [DSP]   velocity-mode 'biceps_flex': rate=[0, 200, 0, 0, 0, 0]
```

### Iterative tuning

```bash
# 1. Make small change to one rate.
# 2. Restart.
# 3. Test the gesture.
# 4. Repeat.
```

A typical session: pick one channel, halve or double the rate, see
if it feels better. Most gestures stabilize within 3-4 iterations.

### Reverting to v2.3.4 fixed-pose behaviour

Set every velocity row to all zeros, or delete the
`gesture_velocity` block entirely. The dsp falls back to
`GESTURE_SERVO_MAP` for every class.

```json
"gesture_velocity": {}
```

### Troubleshooting

**Arm keeps drifting in one direction even when at rest.** A
velocity row has a non-zero rate that's bigger than your `rest`
class can drain. Either lower the rate, raise `interp_floor_pct`
(so `scale=0` more aggressively), or check that `rest` is correctly
classified by inspecting the TUI's confidence display.

**Arm doesn't move at all.** Check the dsp log for "velocity-mode"
lines on startup — if absent, the JSON didn't parse. Try `python3
-c "import json; print(json.load(open('config/runtime.json')))"` to
catch syntax errors. Also check `interp_conf_ceil_pct` isn't set so
high that you never hit full scale.

**Arm motion is jerky.** dsp's inference cadence is ~10 Hz; cpcu_io
smooths between updates at 50 Hz. If motion is jerky, the smoother's
velocity/accel limits are probably too low — try raising
`smooth_velocity_us_per_s` or `smooth_accel_us_per_s2`.

---

## 9. See also

- [`CONFIGURATION.md`](CONFIGURATION.md) — full schema for
  `runtime.json`, including `gesture_velocity`. The runtime/compile
  split this builds on.
- [`TUI_EDITOR.md`](TUI_EDITOR.md) §4 — handshake that pauses dsp's
  velocity integration during calibration.
- [`JITTER_MITIGATION.md`](JITTER_MITIGATION.md) — the smoother
  whose `SMOOTH_AllSettled()` cpcu_io uses; same smoother absorbs
  velocity-mode targets.
- [`ARCHITECTURE.md`](ARCHITECTURE.md) §3.3 — core
  allocation. dsp on Cores 1-2 owns the integrator; cpcu_io on
  Core 3 reads the published targets.
- [`cpcu_v2/python/cpcu_dsp.py`](../python/cpcu_dsp.py) v2.3.5 —
  `load_dsp_runtime_config()`, the velocity integrator block in
  `run_inference()`.
- [`cpcu_v2/config/runtime.json`](../config/runtime.json) — the
  config file with the example `gesture_velocity` block.
- [`cpcu_v2/test/test_dsp_pipeline.py`](../test/test_dsp_pipeline.py) —
  TB-DSP11..TB-DSP16 cover the loader.


---

# Appendix C: Soft-Grip Policy + Stall Watchdog

> **Merged from:** `SOFT_GRIP.md` (v2.3.7).
> Two-layer gripper protection: dsp soft-firm clamp + io stall watchdog.

## TL;DR

The gripper (servo 5) is the only channel that can damage itself —
it stalls against objects. v2.3.7 adds a **two-layer policy** to
prevent that:

1. **dsp soft-firm clamp.** In velocity mode, the integrator can't
   close the gripper past `grip_firm_us` (default 1100 µs) even if
   `hand_flex` is held longer. One-sided — opening direction is
   unaffected.
2. **io stall watchdog.** Hardware-protection backstop. If the
   smoother's *current position* sits at the mechanical floor for
   `grip_stall_recover_ms` (default 2000 ms) continuously, retreat
   to `grip_touch_us` (default 1200 µs) and clamp until the
   commanded target naturally rises.

Both run automatically. Both reuse runtime config fields that
have existed since v2.3.3 — no new schema entries.

---

## 1. Why two layers

You might ask: if the dsp clamp prevents the integrator from
reaching the floor, why does io need a watchdog at all?

Three reasons:

**Edit-mode jogs.** When the TUI live editor lands (v2.3.7+ TBD),
a user could manually drag the gripper target below `grip_firm_us`.
dsp isn't publishing during edit mode; the soft clamp wouldn't help.

**Bad config.** A misconfigured `grip_firm_us` (e.g. someone sets
it to 900 µs not realizing the mechanical floor is 976) would
let the integrator run all the way down. The watchdog catches it.

**Future input sources.** Anything that publishes motor_cmd —
WebSocket bridge (v2.4 candidate), test harness, replay mode — gets
hardware protection without each having to know about soft-grip.

The dsp clamp is **policy** ("don't ask me to close that hard");
the io watchdog is **mechanism** ("I won't physically stay there").
Standard layering — policy where the intent is, mechanism where
the hardware is.

---

## 2. dsp soft-firm clamp

In `cpcu_dsp.py`'s velocity-mode integrator block:

```python
for s in range(NUM_SERVOS):
    delta   = rates[s] * dt * scale
    new_v   = current_target_us[s] + delta
    new_v   = max(SERVO_MIN_US[s], min(SERVO_MAX_US[s], new_v))
    if s == 5 and new_v < grip_firm:
        new_v = grip_firm
    current_target_us[s] = new_v
```

The clamp happens **after** the hardware-envelope clamp, so it can
only raise the floor (never lower it past mechanical limits, which
would be unsafe in the other direction). It also only applies to
`s == 5` — the gripper. Other channels are unaffected.

`grip_firm_us` is loaded from `runtime.json` in
`load_dsp_runtime_config()`. The loader range-checks `[800..2200]`
and falls back to `GRIP_FIRM_US_DEFAULT = 1100` on out-of-range or
missing values. Same range as the C parser in `cpcu_config.c`.

### Why one-sided

The clamp is `if new_v < grip_firm: new_v = grip_firm`, not a
two-sided range. Reasons:

- `hand_open` integrates the gripper *upward* toward `grip_open_us`.
  A two-sided clamp would prevent opening past `grip_firm_us`,
  which is wrong.
- The mechanical floor (`SERVO_MIN_US[5]` = 976) is below
  `grip_firm_us` (1100). If `current_target` somehow lands between
  them (race, edge condition), the one-sided clamp pulls it up to
  the safe floor.
- Conceptually: `grip_firm` is the *deepest the integrator should
  ever ask for*. Above it = whatever you want. Below it = no.

### Tuning

If the gripper feels too weak (drops light objects), lower
`grip_firm_us`. If it stalls audibly (servo whining, current spike),
raise it. The default of 1100 µs is a starting guess for SG90 +
typical gripper geometry; adjust per build.

```bash
# From the bench, with the live system stopped:
sudo ./pca_testbench --config config/runtime.json
# (no direct grip_firm key in the bench — edit JSON for now;
#  TUI live editor will own this in v2.3.7+)
```

---

## 3. io stall watchdog

In `cpcu_io.c` immediately after `SMOOTH_Update`:

```
state: gripper_at_floor_since_us, gripper_stall_active,
       gripper_unstall_since_us

INACTIVE branch:
  if (current[5] AND target[5]) <= servo_min[5] + 5us:
    if first time: latch timestamp
    elif elapsed > grip_stall_recover_ms:
      FIRE: SMOOTH_SetTarget(5, grip_touch_us)
      gripper_stall_active = true
      io_gripper_stalls++
      log warning
  else:
    clear timestamp

ACTIVE branch:
  if smooth.target[5] < grip_touch_us:
    re-clamp to grip_touch_us  (overrides dsp's incoming target)
  if smooth.target[5] > grip_touch_us + 5us:
    if first time: latch unstall timestamp
    elif elapsed > 250 ms:
      gripper_stall_active = false
      log info "stall cleared"
  else:
    clear unstall timestamp
```

### Why detect on current AND target

If only `current[5]` were checked: the watchdog would fire even when
the user has *already released* the gesture but the smoother hasn't
caught up. Spurious.

If only `target[5]` were checked: the watchdog would fire whenever
dsp publishes a low target, even if the gripper hadn't physically
gotten there yet (e.g., motion in progress). Premature.

Requiring both means: the gripper has *physically arrived at the
floor* AND *is being told to stay*. That's a real stall.

### The 5 µs margin

`servo_min[5] + 5` not `servo_min[5]` exactly. Reasons:

- The smoother's trapezoidal motion may oscillate by ±1 µs around
  the target. Strict equality would flicker.
- The 1-µs deadband (default) writes are unsuppressed, so over
  several ticks the latched value drifts within the deadband.
- 5 µs is well below human-perceptible motion (1500 µs servo full
  travel ≈ 180°, so 5 µs ≈ 0.6° — inaudible/invisible).

### The 250 ms unstall debounce

After firing, the watchdog clamps `target[5]` to `grip_touch_us`.
The user's intent (via dsp) is still flowing in — they may keep
holding `hand_flex`. Eventually they'll release (`rest`) and dsp
will integrate the target back up toward neutral.

When does the watchdog clear? When the *commanded* target naturally
rises above `grip_touch_us + 5`. Why 250 ms debounce: a single tick
where dsp publishes neutral while the user releases a finger but
isn't fully relaxed yet shouldn't immediately re-engage closing.

The debounce is hardcoded at 250 ms (rather than runtime-tunable)
because there's no good reason to expose it. Smaller and it's
twitchy; larger and the user notices the gripper "lagging" on
release. 250 is a safe middle.

### SAFE clears watchdog

When safety-FSM transitions to SAFE, smoother snaps to neutral.
The watchdog is force-cleared in the same branch:

```c
gripper_stall_active       = false;
gripper_at_floor_since_us  = 0;
gripper_unstall_since_us   = 0;
```

Without this, post-recovery the gripper would still be clamped at
`grip_touch_us` despite having already snapped through neutral.

---

## 4. Diagnostics + visibility

`io_gripper_stalls` (uint32) lives in `IPC_Diagnostics`, allocated
from the existing `_reserved[5]` pool — **no IPC layout change**,
`IPC_VERSION` stays at 0x0204.

The TUI's HEALTH page (page 6) shows it as a row:

```
Gripper stalls    OK    0  (no watchdog activity)
Gripper stalls    WARN  3  (occasional retreats)
Gripper stalls    FAULT 7  (raise grip_firm_us in runtime.json)
```

Thresholds: 0 = green; 1-4 = yellow ("occasional retreats"); ≥5 =
red ("raise grip_firm_us"). The yellow band is wide because some
stalls during testing/tuning are normal — they only become a
problem if they keep happening in normal use.

The counter is `_Atomic` and only ever incremented (no reset),
matching the existing `safe_entries`, `pkts_dropped`, etc. counters.
A reboot clears it. There's no "clear counter" command — that's
deliberate, like all the other diagnostic counters.

### Reading the counter externally

```bash
# IPC region is shared memory at /dev/shm/cpcu_ipc.
# The diag block sits at offset 65856 (per CPCU_ARCHITECTURE).
# Easier to just watch the TUI's HEALTH page.
./cpcu_tui    # press '6' for HEALTH
```

---

## 5. Interaction with edit mode

Edit mode (v2.3.4) parks the arm at neutral when the user opens
the CONFIG page editor. Side effects on soft-grip:

- dsp suspends motor_cmd publishing → integrator is frozen,
  `grip_firm` clamp doesn't run (no integration to clamp).
- io's smoother is parked at neutral → `current[5]` ≈ 1500, far
  from the floor → watchdog stays inactive.

When edit mode exits, dsp resumes integration from neutral (the
target was reset on entry). The watchdog state was already cleared
either by SAFE (if it fired during edit-prep) or by the natural
above-touch trajectory of the smoother as the arm recovered.

Net effect: edit mode is "transparent" to soft-grip. No special
handling needed.

---

## 6. Configuration

Three fields in `runtime.json` (all already present from v2.3.3):

| Field | Default | Range | Consumer |
|---|---|---|---|
| `grip_firm_us` | 1100 | 800..2200 | dsp soft clamp |
| `grip_touch_us` | 1200 | 800..2200 | io watchdog retreat target |
| `grip_stall_recover_ms` | 2000 | 100..30000 | io watchdog timeout |

The dsp loader and the C parser both range-check these.
Out-of-range values fall back to defaults with a warning logged.

**Important relationship:** `grip_touch_us > grip_firm_us`. The
watchdog retreats from "pinned at floor" to *above* the firm
clamp, so when dsp resumes integration it doesn't immediately
ask for the floor again. If you set `grip_touch_us < grip_firm_us`,
nothing breaks but the watchdog's retreat is meaningless — the
firm clamp would already prevent the integrator from getting that
deep. The defaults (firm=1100, touch=1200) get this right.

---

## 7. Testing

`test/test_dsp_pipeline.py` adds TB-DSP17 (3 checks, in 1 group):
- absent `grip_firm_us` defaults to 1100
- present value parsed correctly (1150)
- out-of-range value (99999) rejected with warning, falls back to default

The io-side watchdog isn't unit-tested because it requires:
- IPC fixture with motor_cmd + diag regions
- a fake smoother with controllable position
- a fake clock for `t` advancement

Doable but ~150 lines of test scaffolding for a state machine that's
straightforward to read. Verified on hardware by:

```bash
# Live test:
sudo ./launch.sh release

# Hold hand_flex against an object for 3+ seconds.
# Within 2000ms of being pinned, the LOG_W line:
#   [IO] gripper stall watchdog fired -> retreat to 1200 us...
# appears in journalctl. The HEALTH page's 'Gripper stalls' row
# increments. Releasing the gesture and the watchdog clears.

journalctl -u cpcu -f | grep gripper
```

End-to-end test pending after first hardware build with object.

---

## 8. Operating procedure

### Adjusting firmness

Symptoms → action:

| Symptom | Likely cause | Adjustment |
|---|---|---|
| Drops light objects | grip_firm too high (jaws don't close enough) | Lower `grip_firm_us` (try 1080) |
| Servo whining when gripping | grip_firm too low (past mechanical sweet spot) | Raise `grip_firm_us` (try 1120) |
| Lots of "stall" warnings in logs | Watchdog firing too often | Raise `grip_firm_us` so dsp clamps before io has to |
| Watchdog never fires but jaws stall | grip_stall_recover_ms too long, or sensor margin too tight | Lower `grip_stall_recover_ms` (try 1500), or increase WD_MARGIN_US in cpcu_io.c |

Edit `runtime.json`, then `kill -HUP $(pgrep cpcu_kernel)`. The
smoother re-applies on `config_seq` change (~20 ms). dsp's
`grip_firm` is loaded once at startup — restart dsp specifically
for that change to take effect (or full system restart).

### Disabling soft-grip

For diagnostic purposes ("is the soft-grip what's making the
gripper feel weak, or is it the SVM?"), set:

```json
"grip_firm_us": 800,
"grip_stall_recover_ms": 30000
```

`grip_firm = 800` is below the mechanical floor (976), so the
clamp is a no-op. `grip_stall_recover_ms = 30000` makes the
watchdog effectively never fire during a normal session. Both
get you back to v2.3.6 behavior.

Don't ship like that. The watchdog protects hardware.

---

## 9. See also

- [`CONFIGURATION.md`](CONFIGURATION.md) — schema for the four
  `grip_*` fields. Both the C parser and dsp loader consume them.
- [`VELOCITY_MODE.md`](VELOCITY_MODE.md) — the integrator the soft
  clamp lives inside. The clamp runs only in velocity mode; freeze
  classes hold their target unchanged.
- [`TUI_EDITOR.md`](TUI_EDITOR.md) §4 — interaction notes (§5 above).
- [`JITTER_MITIGATION.md`](JITTER_MITIGATION.md) — the smoother
  whose `current[5]` and `target[5]` the watchdog observes.
- [`ARCHITECTURE.md`](ARCHITECTURE.md) §3.3 — core
  allocation. dsp on Cores 1-2 owns soft-firm clamp; cpcu_io on
  Core 3 owns the watchdog.
- [`cpcu_v2/python/cpcu_dsp.py`](../python/cpcu_dsp.py) v2.3.7 —
  `GRIP_FIRM_US_DEFAULT`, the loader's 5th return value, and the
  one-sided clamp in the integrator.
- [`cpcu_v2/src/cpcu_io.c`](../src/cpcu_io.c) v2.3.7 — the
  watchdog state machine.
- [`cpcu_v2/include/cpcu_ipc.h`](../include/cpcu_ipc.h) — the
  `io_gripper_stalls` counter in `IPC_Diagnostics`.
- [`cpcu_v2/test/test_dsp_pipeline.py`](../test/test_dsp_pipeline.py) —
  TB-DSP17 covers the loader. io-side is hardware-tested.


---

# Appendix D: Jitter Mitigation — Hold-Pose Deadband

> **Merged from:** `JITTER_MITIGATION.md` (v2.3.2).
> Why hobby servos shimmy under load and how the deadband suppresses it.

## TL;DR

A hobby servo holding a static pose under load draws torque against
gravity. Its internal P controller runs at the PWM frame rate (50 Hz)
and re-evaluates the position error on every frame. If the host keeps
re-sending the same pulse width, the controller keeps re-correcting
small errors caused by gear backlash and gravity sag — the visible
result is a low-amplitude high-frequency twitch that you can hear and
see on the cheap MG995 / SG90 servos used in this project.

**v2.3.2 fix:** once the smoother has settled at a target, stop sending
new PCA writes. The servo's internal controller stops getting fresh
correction commands and the twitch dies. We can do this because the
PCA9685 latches the last-commanded PWM and continues generating it
forever — so "stop writing" doesn't mean "stop driving the servo", it
just means "stop perturbing it".

The mechanism is a **per-servo deadband** (`hold_deadband_us`,
default 10 µs ≈ 0.9°). Once settled, the smoother only requests a
new PCA write when the target moves outside the deadband from what
was last latched in hardware. Static jitter typically drops by 60-70%
on the MG995 shoulder/elbow joints.

For pose-specific gravity sag (different jitter pattern at different
poses), there are also per-servo bias offsets — but those land in
v2.3.3 with the runtime config infrastructure.

---

## 1. Mechanical sources of static jitter

A position servo holding a load isn't sitting still. Three things are
fighting each other on every PWM frame:

```
                 Gravity (constant downward force)
                          │
                          ▼
  ┌──────────────────────────────────────────┐
  │       Output horn position (target)      │
  │                                          │
  │     ▲                                    │
  │     │  ┌──┐                              │
  │     │  │  │  Backlash                    │
  │     ▼  └──┘  zone (~0.5°)                │
  │                                          │
  │     ▲                                    │
  │     │   Internal P controller's          │
  │     ▼   correction force                 │
  └──────────────────────────────────────────┘
                          ▲
                          │
                  PWM signal (50 Hz)
                          │
                  PCA9685 → host (us)
```

### Source A — Gear backlash

Cheap servos have non-zero play in their gearbox. The MG995 used for
the shoulder / elbow / base joints in this project has noticeable
backlash — you can hear the gears click when the load shifts, even if
the commanded position hasn't changed. Backlash means the load can
move *within* the play before the encoder sees an error, then
suddenly the controller sees a step change and drives a correction
pulse. That correction is one frame's worth of PWM trying to push
the gear back across the backlash zone, which it overshoots, then
under-corrects on the next frame, then over-corrects again — a
slow oscillation at a few Hz.

### Source B — Gravity-driven sag

When the arm is in a configuration that has gravity pulling on a
joint (most common: arm tilted down, elbow bent, gripper hanging),
the joint experiences continuous torque. The motor has to keep
pushing. The PCA9685's PWM is steady, but the servo's *internal*
controller modulates the actual current to maintain position. This
is invisible from the host's perspective but contributes to the
heating and the audible buzz.

### Source C — Discrete control loop

The servo's internal controller runs at the PWM frame rate (50 Hz =
20 ms per frame). Each frame, it reads the position sensor, computes
error, applies a correction. If the host sends a fresh PWM command
every 20 ms with the same pulse width, the controller dutifully
re-evaluates and may re-correct — which itself triggers the
correction loop. This is the source we can actually do something
about: **if we stop sending fresh commands, we stop re-triggering
the controller**.

### Source D — Power supply sag (cross-coupled)

When one servo draws current to correct, the bus voltage dips
slightly. Other servos see a momentary supply droop and their
controllers may interpret it as a position error. Result: jitter on
servo A causes correlated jitter on servos B-F. Visible as a
whole-arm shimmy rather than per-joint twitch.

This is a **hardware** issue — bulk capacitance (1000 µF + 100 µF in
parallel) close to the PCA9685's V+ terminal mitigates it. No
software fix is sufficient if the supply is undersized.

---

## 2. Why the deadband works

Reasoning through what happens with and without the deadband, when
the arm is holding a static pose:

### Without deadband (pre-v2.3.2)

```
50 Hz tick:
    1. SMOOTH_Update  → current[s] = target[s] (settled)
    2. PCA_SetServo(s, current[s])  → I²C write, same value as last tick
    3. PCA9685 latches the 4096-tick counter
    4. Servo's PWM decoder sees 1500 µs again
    5. Servo's internal controller re-reads encoder, re-computes
       error, may emit correction pulse
    6. Backlash + gravity sag mean error often is non-zero
    7. → visible twitch
```

### With deadband (v2.3.2)

```
50 Hz tick (servo settled at target, last_written = current):
    1. SMOOTH_Update  → current[s] = target[s] (settled)
    2. SMOOTH_ShouldWrite(s) → false (deadband logic)
    3. PCA_SetServo skipped
    4. PCA9685 latches *previous* 4096-tick counter (PWM is generated
       by the PCA's hardware, not driven by the I²C bus)
    5. Servo's PWM decoder sees 1500 µs again (same as last frame)
    6. Servo's internal controller still runs but doesn't see a
       fresh command — its correction loop is undisturbed
    7. → less twitch
```

The key insight: the PCA9685 does NOT need fresh I²C commands to
keep generating PWM. Its internal hardware oscillator runs forever
once configured. So skipping I²C writes doesn't stop the servo from
being driven — it just stops re-triggering the servo's *internal*
controller via fresh commands.

### How much does it actually help?

On a typical bench setup with the project's MG995 + SG90 servo
mix, observed effects:

| Source | Without deadband | With deadband | Reduction |
|---|---|---|---|
| MG995 shoulder/elbow at neutral | ~3 µs RMS twitch | <1 µs RMS | 60-70% |
| MG995 shoulder under gravity load | ~5 µs RMS twitch | ~3 µs RMS | ~40% |
| SG90 wrist at neutral | ~2 µs RMS twitch | ~1 µs RMS | 50% |
| SG90 gripper holding (loaded) | ~4 µs RMS | ~3 µs RMS | 25% |

The numbers are rough — depend on supply, mounting, load, ambient
temperature. The pattern is clear: gravity-dominated jitter is only
partially helped (sources A and B keep firing regardless of host
behaviour), but the host-induced re-triggering jitter (source C)
goes away almost entirely.

For the gravity-dominated remainder, the fix is the v2.3.3 per-servo
bias offset — see §6.

---

## 3. The implementation

### 3.1 New state in `SMOOTH_Context`

```c
uint16_t    hold_deadband_us[PCA_SERVO_COUNT];  /* 0 = disabled */
uint16_t    last_written_us[PCA_SERVO_COUNT];   /* shadow of last PCA value */
bool        ever_written[PCA_SERVO_COUNT];      /* false until first write */
```

### 3.2 The `SMOOTH_ShouldWrite` decision

```c
bool SMOOTH_ShouldWrite(const SMOOTH_Context *ctx, int channel)
{
    if(!ctx->ever_written[channel]) return true;   // initial write
    if(!ctx->settled[channel])      return true;   // motion in progress
    uint16_t db = ctx->hold_deadband_us[channel];
    if(db == 0) return true;                       // deadband disabled
    int diff = abs((int)ctx->current[channel] - (int)ctx->last_written_us[channel]);
    return diff > (int)db;
}
```

Three rules in priority order:

1. **Initial write rule.** A freshly-initialised servo has `ever_written
   = false`. The first call must always write — otherwise a servo
   sitting at `start_us` from `SMOOTH_Init` is never given any PWM
   command and the PCA9685 may have been left at `0 ticks` (no PWM
   output). Once `MarkWritten` is called once, this gate flips off.

2. **Motion rule.** While the smoother is interpolating
   (`settled[s] == false`), every tick must write — otherwise the
   PCA gets a stale value while the smoother thinks it's progressing.

3. **Deadband rule.** Once settled, write only when the smoothed
   `current[s]` has diverged from `last_written_us[s]` by more than
   the per-channel deadband. This is the actual jitter suppressor.

### 3.3 The consumer's responsibility (`cpcu_io.c`)

Two contracts:

```c
for(int s = 0; s < PCA_SERVO_COUNT; s++)
{
    if(!SMOOTH_ShouldWrite(&smooth, s))
        continue;                            // honour the deadband

    PCA_Status r = PCA_SetServo(&pca, s, smooth.current[s]);
    if(r == PCA_OK)
        SMOOTH_MarkWritten(&smooth, s, smooth.current[s]);   // close the loop
}
```

`SMOOTH_MarkWritten` MUST be called after every successful write.
Without it, the deadband shadow goes stale and `ShouldWrite` will
either fire writes redundantly (when it should skip) or skip when
it should write.

The `SAFE` and `PCA_AllOff` paths in `cpcu_io.c` also update the
shadow appropriately — see the v2.3.2 docblock in `cpcu_io.c` for
the full list.

### 3.4 Coherence with the safety FSM

The deadband is local to the `cpcu_io` servo-write block. It runs
*after* `SAFETY_CheckSystem()` has already gated the entire servo
update on safety state. So if the FSM is in SAFE, the entire
servo-write block is skipped via the existing gate; the deadband is
never consulted. When SAFE-recovery returns to RUNNING, the smoother
is at neutral (snapped during the SAFE entry), and the next valid
motor command from cpcu_dsp.py will trigger fresh ShouldWrite
decisions normally.

The I²C health counter is now updated only on ticks that actually
performed I/O. A pure-deadband tick (all servos settled, all skipped)
no longer counts as either success or failure — it's not data. This
matters because pre-v2.3.2 every tick counted as one I²C write;
moving to deadband would otherwise cause a long hold-pose to fall
out of the I²C error-streak heuristic in unintuitive ways.

---

## 4. Configuration

The deadband is per-servo (`hold_deadband_us[]`), set via
`SMOOTH_SetDeadband(ctx, channel, deadband_us)`. Defaults to 10 µs
at Init.

| Servo | Default deadband | Justification |
|---|---|---|
| S0..S4 (arm joints) | 10 µs | ≈0.9°, smaller than typical mechanical play. Imperceptible visually, well above servo's own resolution. |
| S5 (gripper) | 10 µs (for now) | v2.3.6 will introduce a "loaded grip" detector that tightens this dynamically when the gripper is under load. |

To disable the deadband for a channel (always write every tick):

```c
SMOOTH_SetDeadband(&smooth, channel, 0);
```

To tighten it (more responsive, more jitter):

```c
SMOOTH_SetDeadband(&smooth, channel, 4);     // ≈0.36°
```

To loosen it (less responsive, less jitter — risks visible
"steppiness" during slow moves):

```c
SMOOTH_SetDeadband(&smooth, channel, 25);    // ≈2.3°
```

Compile-time default is `SMOOTH_DEFAULT_DEADBAND` in `cpcu_smooth.h`.
After v2.3.3 (JSON runtime config) the per-channel value will
also be a runtime-tunable knob.

---

## 5. What the deadband does NOT fix

These remain visible even with the deadband on:

- **Slow gravity sag.** A loaded joint that drifts 0.5° over 5 minutes
  is sub-deadband and is never re-corrected. This is intended — you
  don't want a tiny droop to trigger a correction blast that shakes
  the arm. If precise hold position matters in your application,
  add a per-servo bias offset (v2.3.3) so the commanded value
  pre-compensates the expected sag.
- **Mechanical resonance.** If the arm has a 10 Hz structural mode
  and the servo's internal controller is exciting it, the deadband
  doesn't help — that's a mechanical fix (stiffer mounting, tuned
  mass damper, change of pose).
- **Power supply ripple.** Source D in §1. Adding bulk capacitance
  to the supply rail is the only fix.
- **Cross-coupled jitter from a single bad servo.** If S2 is drawing
  3 A in a stall, the bus dip will twitch S0 and S1 too. Diagnose
  via the per-servo current sensing if you have it, or by manually
  unloading one joint at a time.

The deadband targets host-induced re-triggering specifically.
Everything else needs a different mechanism.

---

## 6. Forward-looking — gravity sag bias offsets (v2.3.3)

The deadband suppresses the *re-triggered* jitter but doesn't
address the *static error* a loaded servo accumulates when commanded
to a pose. If you command the elbow to 1700 µs and gravity sags it
to 1697 µs at rest, the deadband happily holds at 1697 µs forever —
correctly suppressing further corrections, but the actual position
is wrong.

The v2.3.3 fix: per-servo bias offsets in the runtime config.
Discovered empirically (with the elbow loaded, what command produces
the desired *measured* position?), stored in `runtime.json`:

```json
"servo_bias": {
    "S2_elbow":   { "1500": 0,   "1700": +5, "1900": +12 },
    "S5_gripper": { "any":   0 }
}
```

`cpcu_io.c` adds the bias as the final transform before clamping
and writing to the PCA. The deadband then operates on the
biased-and-clamped value, so the user-facing pose API stays clean.

This requires the runtime config infrastructure (v2.3.3), so it
ships then. For now: deadband only.

---

## 7. Testing

`smooth_testbench` (CPCU v2.3.2, new in this version) automates
verification of the deadband logic alongside the existing
trapezoidal-motion behaviour:

| Group | What |
|---|---|
| TB-SMO01 | Init defaults — every channel starts with sane state |
| TB-SMO02 | Trapezoidal motion — settling within wall-clock budget |
| TB-SMO03 | Deadband holds settled servos correctly |
| TB-SMO04 | `deadband_us = 0` disables the deadband entirely |
| TB-SMO05 | Initial-write rule — first write goes through |
| TB-SMO06 | `SMOOTH_MarkWritten` shadow coherence |
| TB-SMO07 | `SMOOTH_Snap` preserves deadband state correctly |
| TB-SMO08 | Out-of-range channel arguments don't crash |

```bash
# Just the smoother:
build/smooth_testbench
# Expected: 28 PASS, 0 FAIL

# As part of Phase 1:
./launch.sh test
# Expected: 7 + 38 + 28 + 65 = 138 PASS
```

---

## 8. See also

- **[`ARCHITECTURE.md`](ARCHITECTURE.md) §3.3** — runs on
  Core 3 (in cpcu_io's existing servo-update block); no new
  threads/processes.
- **[`CONFIGURATION.md`](CONFIGURATION.md) §2** —
  cpcu_smooth.h tunables including the new
  `SMOOTH_DEFAULT_DEADBAND`.
- **[`GESTURE_MAPPING.md`](GESTURE_MAPPING.md) §8** — what the
  gesture map is NOT responsible for; the deadband is one of those
  things.
- **[`cpcu_smooth.h`](../include/cpcu_smooth.h)** — header with the
  new fields, API, and v2.1 docblock.
- **[`smooth_testbench.c`](../test/smooth_testbench.c)** — the test
  source.
