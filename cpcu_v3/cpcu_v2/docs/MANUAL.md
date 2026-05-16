# InfiniTech CPCU — Developer Manual

This is the internal developer guide. It explains how every module
works, why it was designed that way, and how to modify it.

---

## 1. Data Flow

```
Muscle → Electrode → BSAU ADC (2kHz) → NRF TX → [air] →
NRF RX → cpcu_io (SPI read) → IPC Ring Buffer →
cpcu_dsp.py (filter → features → ML → velocity) → IPC MotorCmd →
cpcu_io (smoother → PCA9685 I²C) → Servo PWM → Arm moves
```

Every module is a stage in this pipeline. They communicate only
through shared memory (IPC). No module calls another directly.

---

## 2. Process Architecture

Four processes, pinned to specific CPU cores:

| Process | Core | Language | Role |
|---|---|---|---|
| `cpcu_kernel` | 0 | C | Supervisor. Spawns/monitors all others. |
| `cpcu_io` | 3 | C | Real-time I/O. NRF read, servo write, safety. |
| `cpcu_dsp.py` | 1-2 | Python | DSP filtering, ML inference, velocity control. |
| `cpcu_audio_daemon.py` | 0 | Python | Audio feedback (optional). |

`cpcu_kernel` forks `cpcu_io` and `cpcu_dsp.py`. If either dies,
kernel respawns it within 2 seconds.

---

## 3. IPC Shared Memory Layout

All inter-process communication goes through a single POSIX shared
memory region (`/dev/shm/cpcu_ipc`). No sockets, no pipes, no files.

```
Offset  Size    Region              Writer → Reader
─────────────────────────────────────────────────
0       64B     ControlBlock        kernel ↔ all
64      64KB    SensorRing[1024]    io → dsp
~64KB   128B    MotorCommand        dsp → io
+128    128B    Diagnostics         io → tui
+128    256B    DSPExport           dsp → tui/web
+256    128B    RuntimeConfig       kernel → io
+128    64B     ToolPresence        kernel → all
+64     var     DspFiltered         dsp → tui
+var    64B     LatencyTrace        dsp+io → tui
```

Key design rules:
- Each struct is cache-line aligned (64B) to avoid false sharing.
- Writers use `atomic_store`, readers use `atomic_load`.
- The sensor ring is SPSC (single-producer single-consumer) with
  acquire/release ordering — no locks needed.
- MotorCommand uses a sequence number for latest-wins semantics.

**To add a new IPC region:**
1. Define the struct in `include/cpcu_ipc.h` with `__attribute__((aligned(64)))`.
2. Add it to `IPC_SHM_SIZE`.
3. Add the pointer to `IPC_Context`.
4. Assign it in `IPC_Create()` and `IPC_Open()` in `src/cpcu_ipc.c`.
5. Add read/write methods in `python/cpcu_ipc_bridge.py`.

---

## 4. C Modules

### 4.1 cpcu_kernel.c (495 lines)

**Purpose:** Supervisor process. Creates IPC, forks children, monitors
heartbeats, handles SIGHUP for config reload.

**Key functions:**
- `main()` — create IPC, fork io + dsp, enter monitor loop.
- `fork_io()` — exec `cpcu_io` with RT priority on core 3.
- `fork_dsp()` — exec `python3 cpcu_dsp.py` on cores 1-2.
- SIGHUP handler — re-reads `runtime.json`, writes to `IPC_RuntimeConfig`.

**To edit:** You rarely touch this. Add new child processes here if
needed (e.g., the audio daemon could be managed by kernel instead
of tmux).

### 4.2 cpcu_io.c (871 lines)

**Purpose:** The real-time core. Runs at 1kHz polling the NRF radio,
50Hz writing servos, monitoring safety.

**Main loop structure (simplified):**
```
while(running) {
    t = monotonic_us();
    // 1. Poll NRF for new packets (1kHz)
    if(NRF_DataReady()) {
        packet = NRF_ReadPayload();
        IPC_PushSensor(packet);
        SAFETY_FeedPacket(packet);
    }
    // 2. Read motor command from DSP (50Hz)
    if(time_for_servo_update) {
        servo_us = IPC_ReadMotorCmd();
        SMOOTH_SetAllTargets(servo_us);
        // apply smoother overrides + snap flags (v3)
        SMOOTH_Update(dt);
        for(each servo) PCA_SetServo(smoothed_value);
    }
    // 3. Safety checks
    SAFETY_CheckSystem();  // watchdog, thermal, I2C health
}
// Cleanup: neutral → wait → alloff → close
```

**To edit:** The most common change is adding a new data source to
the safety monitor or changing the servo update rate. The main loop
order matters — NRF polling must happen before servo writes to
minimize latency.

### 4.3 cpcu_smooth.c (312 lines)

**Purpose:** Trapezoidal velocity profile smoother. Prevents servos
from snapping to targets — they accelerate, cruise, decelerate.

**Key API:**
- `SMOOTH_SetTarget(ch, target_us)` — set where the servo should go.
- `SMOOTH_Update(dt_us)` — advance one tick. Returns smoothed positions.
- `SMOOTH_SetVelocity(ch, us_per_s)` — max speed.
- `SMOOTH_SetAccel(ch, us_per_s2)` — ramp rate.
- `SMOOTH_SetEnabled(ch, bool)` — false = bypass (snap mode for gripper).
- `SMOOTH_ShouldWrite(ch)` — deadband check to avoid redundant I²C writes.

**To edit:** The profile is per-channel. If you want a different
curve (e.g., S-curve instead of trapezoidal), replace the `Update`
function — the rest of the API stays the same.

### 4.4 cpcu_safety.c (510 lines)

**Purpose:** 4-state safety FSM: INIT → RUNNING → DEGRADED → SAFE.

**Triggers:**
- Radio timeout (750ms no packets) → DEGRADED → SAFE.
- I²C bus failure (25 consecutive errors) → PCA_AllOff.
- DSP stall (2s no motor commands) → DEGRADED.
- Thermal (>85°C) → SAFE.

**To edit:** Add new fault sources by calling `SAFETY_Feed*()` from
`cpcu_io.c`. The FSM transitions are in `SAFETY_CheckSystem()`.

### 4.5 cpcu_pca9685.c (242 lines)

**Purpose:** I²C driver for PCA9685 16-channel PWM controller.

**Key API:**
- `PCA_Init(addr, freq_hz)` — configure at 50Hz for servos.
- `PCA_SetServo(ch, pulse_us)` — write one servo.
- `PCA_SetAllNeutral()` — all servos to 1500µs.
- `PCA_AllOff()` — disable all PWM outputs (servos go limp).

**To edit:** The prescale formula is `round(25MHz / (4096 * freq)) - 1`.
If you change servo frequency from 50Hz, update this.

### 4.6 cpcu_tui_render.c (2131 lines)

**Purpose:** ncurses TUI dashboard. 7 pages of live data.

**Structure:** Each page is a `draw_page_*()` function. They read
from IPC and render. The main loop in `cpcu_tui.c` calls the active
page's draw function at 10Hz.

**To edit:** To add a new page, write a `draw_page_foo()` function,
add it to the page array in `cpcu_tui.c`, increment `NUM_PAGES`.
All data comes from IPC — never call hardware directly from TUI.

### 4.7 cpcu_config.c (697 lines)

**Purpose:** Reads `runtime.json` and writes values to
`IPC_RuntimeConfig`. The kernel calls this on startup and on SIGHUP.

**To edit:** To add a new tunable parameter, add the field to
`IPC_RuntimeConfig` in `cpcu_ipc.h`, parse it in `cpcu_config.c`,
and read it in whichever module needs it.

### 4.8 cpcu_ws.c (657 lines)

**Purpose:** WebSocket server using Mongoose. Serializes IPC data
to JSON and pushes to connected browsers at 10Hz.

**To edit:** The JSON serialization is in `ws_broadcast()`. Add new
fields by reading from IPC and appending to the JSON object.

---

## 5. Python Modules

### 5.1 cpcu_dsp.py (617 lines)

**Purpose:** The brain. Filters EMG, extracts features, runs ML
inference, integrates servo velocities.

**Structure:**
```python
# Config loaders
load_gestures()     # reads gestures.json → gesture defs, channels, hysteresis
load_velocity_map() # reads velocity_map.json → operator calibration
load_runtime()      # reads runtime.json → grip limits
discover_model()    # finds .pkl model file

# DSP functions
butter_bandpass()   # 4th order Butterworth 20-450Hz
notch_filter()      # 50Hz mains rejection
envelope()          # low-pass rectified signal
extract_features()  # RMS, VAR, WL, ENV per channel
process_window()    # full pipeline on one 200ms window

# Confidence curve
confidence_scale()  # maps ML probability to velocity scale (quadratic)

# Main loop
run_inference()     # drain ring → window → features → predict → integrate → publish
```

**Key design:** The velocity integration is the core control mechanism.
Each gesture defines rates per servo. Every inference window:
```python
delta = rate * dt * confidence_scale(probability)
target[servo] += delta
```
This means: stronger muscle contraction → higher probability →
faster servo movement. Release muscle → probability drops → servo
stops. The quadratic curve makes this feel natural.

**To edit:** To change the feature set, modify `extract_features()`
and retrain the model. To add a new filter stage, insert it in
`process_window()`. To change the control law, modify the velocity
integration block in `run_inference()`.

### 5.2 cpcu_ipc_bridge.py (590 lines)

**Purpose:** Python interface to the C shared memory layout.
Mirrors every struct from `cpcu_ipc.h` with manual offset math.

**Key rule:** Offsets must exactly match the C structs. If you change
a struct in `cpcu_ipc.h`, you must update the corresponding offsets
in this file. There is no auto-generation — it's manual and fragile.

**To edit:** When adding a new IPC field, calculate the byte offset
from the base of the region. Use `struct.pack_into` / `unpack_from`
for reads and writes. Test with `test_ipc_bridge.py`.

### 5.3 cpcu_calibrate.py (101 lines)

**Purpose:** Interactive 0-10 velocity preference tuning.

**The mapping:** `rate = base_rate × (level / 5)²`. This is quadratic:
level 3 gives 36% of base speed, level 7 gives 196%.

### 5.4 cpcu_audio_daemon.py (184 lines)

**Purpose:** Watches IPC gesture transitions, plays audio cues.

**Modes:** "voice" plays `.wav` files via `aplay`. "freq" generates
sine tones in-memory from `freq_hz`/`freq_ms` in `gestures.json`.
Polls IPC at 20Hz — fast enough for transitions, cheap enough to
not affect RT performance.

---

## 6. Configuration System

```
gestures.json ─── THE source of truth
├── servo_channels     (motor names, PCA mapping, limits)
├── emg_channels       (active ADC channels, muscle names)
├── gestures           (name → mode + servos + audio)
├── audio_mode/volume  (off/voice/freq)
├── audio_events       (system event sounds)
├── confidence         (curve type, floor, ceiling)
├── hysteresis         (asymmetric transition votes)
└── model_path         (which .pkl to load)

runtime.json ─── hardware tuning only
├── servo limits       (min/max/bias per channel)
├── smoother defaults  (velocity, accel, deadband)
├── gravity comp       (direction, scale per channel)
└── grip parameters    (open, touch, firm, stall timeout)
```

**Design rule:** If it's about WHAT the system does (gestures, audio,
channels), it goes in `gestures.json`. If it's about HOW the hardware
behaves (servo limits, smoother speed), it goes in `runtime.json`.

**Every ./launch.sh command modifies gestures.json through Python
one-liners** — never by hand-editing. This ensures the JSON stays
valid and cross-references stay consistent (e.g., renaming a motor
updates all gesture references automatically).

---

## 7. Adding a New Feature (Step by Step)

### Example: Add a current sensor for grip force feedback

1. **Hardware:** Wire INA219 current sensor on I²C bus (same as PCA9685,
   different address).

2. **C driver:** Create `src/cpcu_ina219.c` + `include/cpcu_ina219.h`.
   Model after `cpcu_pca9685.c` — init, read, close. Keep it pure I²C,
   no IPC knowledge.

3. **IPC region:** Add `IPC_GripForce` struct to `cpcu_ipc.h` (aligned 64B).
   Update `IPC_SHM_SIZE`, `IPC_Context`. Assign pointer in `cpcu_ipc.c`.

4. **IO integration:** In `cpcu_io.c`, read INA219 in the servo update
   block (50Hz). Write current reading to `IPC_GripForce`. Use the
   reading to detect object contact / stall.

5. **Python bridge:** Add offset constants and `read_grip_force()` to
   `cpcu_ipc_bridge.py`.

6. **DSP usage:** In `cpcu_dsp.py`, read grip force from IPC. Use it
   to modulate gripper velocity — slow down as force increases.

7. **TUI display:** Add `draw_grip_force()` to `cpcu_tui_render.c`.

8. **Config:** Add `grip_force_threshold` to `runtime.json` and parse
   it in `cpcu_config.c`.

9. **CMakeLists.txt:** Add `src/cpcu_ina219.c` to the build.

10. **Test:** Add `test/ina219_testbench.c` to verify I²C communication.

---

## 8. Module Design Rules

1. **No module calls another directly.** All communication through IPC.
   This means any module can be restarted independently.

2. **C modules are single-threaded.** Concurrency comes from multi-process
   on isolated cores, not from threads. No mutexes anywhere.

3. **Python talks to C only through shared memory.** No RPC, no sockets
   between C and Python. The IPC bridge is the only bridge.

4. **Config flows one direction:** JSON file → kernel → IPC → consumers.
   No module writes config back. The kernel is the sole config distributor.

5. **Safety has priority over everything.** The safety FSM in `cpcu_io.c`
   can override any motor command. No module can bypass safety.

6. **The smoother sits between DSP and hardware.** DSP writes targets,
   smoother profiles them, IO writes the smoothed values to PCA.
   DSP never writes to PCA directly.

7. **gestures.json is the single source of truth.** Motor names, gesture
   definitions, channel mappings, audio cues — all in one file. The
   TUI, web, DSP, and audio daemon all read from it.

---

## 9. File Quick Reference

| File | Lines | One-line purpose |
|---|---|---|
| `src/cpcu_kernel.c` | 495 | Supervisor: fork, monitor, SIGHUP reload |
| `src/cpcu_io.c` | 871 | RT loop: NRF poll, servo write, safety FSM |
| `src/cpcu_smooth.c` | 312 | Trapezoidal velocity profiler |
| `src/cpcu_safety.c` | 510 | 4-state safety FSM |
| `src/cpcu_pca9685.c` | 242 | PCA9685 I²C servo driver |
| `src/cpcu_config.c` | 697 | JSON config → IPC_RuntimeConfig |
| `src/cpcu_ipc.c` | 356 | Shared memory create/open/close |
| `src/cpcu_tui.c` | 458 | ncurses main loop + page dispatch |
| `src/cpcu_tui_render.c` | 2131 | All 7 TUI pages |
| `src/cpcu_tui_data.c` | 500 | Dataset recording from TUI |
| `src/cpcu_tui_editor.c` | 594 | Live parameter editing in TUI |
| `src/cpcu_ws.c` | 657 | WebSocket server (Mongoose) |
| `src/cpcu_json.c` | 261 | Minimal JSON parser for C |
| `src/cpcu_log.c` | 28 | File + stderr logging |
| `include/cpcu_ipc.h` | 410 | All IPC struct definitions |
| `include/cpcu_smooth.h` | 134 | Smoother API |
| `include/cpcu_safety.h` | 266 | Safety FSM API |
| `nrf/nrf24l01.c` | 430 | NRF24L01+ radio driver |
| `nrf/nrf24l01_linux.c` | 335 | Linux SPI HAL for NRF |
| `wire/wireless_packet.c` | 79 | 32-byte packet codec |
| `python/cpcu_dsp.py` | 617 | DSP + ML + velocity control |
| `python/cpcu_ipc_bridge.py` | 590 | Python ↔ C shared memory |
| `python/cpcu_calibrate.py` | 101 | 0-10 velocity tuning |
| `python/cpcu_audio_daemon.py` | 184 | Gesture audio feedback |
| `scripts/launch.sh` | 2301 | All commands (single entry point) |
| `config/gestures.json` | 76 | Gesture/motor/channel/audio config |
| `config/runtime.json` | 71 | Hardware tuning parameters |
