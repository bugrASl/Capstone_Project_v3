# Web Dashboard — Read-Only Multi-Viewer Bridge

**Audience:** anyone running the system, anyone sharing live demos
with advisors/teammates, anyone debugging "the dashboard says it's
disconnected."

## DEPENDENCIES

| File / region | What this dashboard relies on |
|---|---|
| `/dev/shm/cpcu_ipc` | Read-only mmap of the kernel's full IPC layout. |
| `cpcu_ipc.h` | Struct definitions; IPC_VERSION is the schema-binding contract. |
| `cpcu_ws.c` | Process that maps the SHM and emits the wire format. |
| `cpcu_dsp.py` | Writes `/tmp/cpcu_group_state.txt` (per-group classifier digest). |
| `mongoose` (vendored) | HTTP/WS event loop. Falls back to a stub when missing. |
| `config/gestures.json` | Indirect — its emg_channels names + servo limits show up via IPC_RuntimeConfig. |

## CROSS-MODULE EFFECTS

- Schema bumps in `cpcu_ipc.h` (new region, struct grows, IPC_VERSION
  changes) → rebuild `cpcu_ws` and verify `index.html` still consumes
  every field it reads.
- New tab on `index.html` → add a matching block in
  `build_state_frame()` (state-tab data) or `build_wave_frame()`
  (wave-tab data) in `cpcu_ws.c`.
- New testbench claims an `IPC_ToolPresence` slot → add a payload
  decoder in `build_state_frame()`'s `tools[]` loop in `cpcu_ws.c`,
  and a card renderer in `index.html`.

---

## TL;DR

A separate process called `cpcu_ws` runs alongside `cpcu_kernel` on the
Pi. It maps `/dev/shm/cpcu_ipc` read-only (same way `cpcu_tui` does),
listens on HTTP+WebSocket at `:8765`, and serves a single-page browser
dashboard called the **CPCU Dashboard**.

```
┌──────────────┐     mmap RO      ┌─────────────────┐
│ /dev/shm/    │──────────────────▶│   cpcu_ws       │
│  cpcu_ipc    │                  │   (Mongoose)    │
└──────────────┘                  │                 │
                                   │  10 Hz state    │
                                   │  20 Hz waves    │
                                   └────────┬────────┘
                                            │ WebSocket
                                            ▼
                                   ┌─────────────────┐
                                   │  Browser tabs:  │
                                   │  Arm            │
                                   │  EMG            │
                                   │  Diagnostics    │
                                   └─────────────────┘
```

The dashboard is **read-only**. There is no command channel from the
browser to the running system. The browser cannot enter edit mode,
save config, drive servos, or affect anything else. If you want to
tune values, use the TUI live editor or `pca_testbench` at the bench.

The bridge supports **multiple simultaneous viewers**. Teammates on
the same LAN can each open the dashboard URL; each connection gets
its own broadcast. Default bind is `0.0.0.0:8765` so the Pi accepts
connections from any device on its network. A loud warning prints at
startup so you don't accidentally serve biosignals to a public
network.

---

## 1. Why this exists

Three real problems the TUI alone doesn't solve:

1. **Demos.** Pointing a projector at a 24-line ncurses terminal
   doesn't sell the project. A browser tab on the demo screen does.
2. **Tuning while wearing the device.** Once the prosthetic is on
   your arm, the keyboard becomes hard to reach. A phone or laptop
   in the other hand showing live diagnostics is far more usable
   than craning over the Pi.
3. **Multiple viewers during development.** Friends, advisors, and
   the capstone committee can watch the live system from their own
   laptops without taking turns at the Pi.

---

## 2. Architecture decisions

### Separate process, not a thread inside cpcu_kernel

The web bridge is a separate binary that opens the IPC region as a
*third* consumer (after cpcu_io and cpcu_tui). Reasons:

- **Fault isolation.** A bug in the bridge can't crash the realtime
  control loop. Mongoose is a well-tested library but it's still
  thousands of lines of someone else's code.
- **Independent deployment.** You can install or replace the bridge
  without rebuilding the kernel binary. systemd treats it as its
  own unit (`cpcu_ws.service`).
- **Matches the existing pattern.** cpcu_tui is also an IPC
  consumer running as a separate process; the bridge fits the same
  shape.

The trade-off is that the bridge has to read the same IPC layout the
kernel publishes, which means schema changes need to be coordinated
across both binaries (caught by `IPC_VERSION`).

### Mongoose for the HTTP/WS server

Mongoose is a single-file C library (one `.c` + one `.h`, about
10K LOC). It's small enough to vendor into our tree without dragging
in a third-party build system. The `web/vendor/` directory has a
`fetch.sh` that pulls a pinned version (7.14) from GitHub and a
README explaining the licensing (dual GPLv2/commercial; we use GPLv2
for academic work, which fits a senior capstone).

Mongoose was chosen over libwebsockets (heavier API, would need
`apt install`) and over hand-rolling RFC 6455 (fiddly, not worth
the effort).

The build is graceful in offline environments: if `mongoose.{c,h}`
isn't present, CMake builds `cpcu_ws` as a stub binary that prints
"run vendor/fetch.sh and rebuild." The rest of the system builds
unchanged. So a fresh git clone on a network-isolated box still
configures cleanly.

### Hand-rolled JSON serializer

`include/cpcu_json.h` + `src/cpcu_json.c` is a tiny stream-style writer (~250 LOC,
7 unit tests). Reasons for not vendoring cJSON or similar:

- **Symmetry** with the existing hand-rolled JSON parser in
  `cpcu_config.c`. Same project, same style.
- **Minimal surface area.** We only emit JSON, never parse it.
  Writing is much simpler than parsing.
- **No malloc.** The serializer takes a caller-provided buffer and
  sets an `overflow` flag if it doesn't fit. No allocations in the
  realtime path.

The writer supports objects, arrays, strings (with escaping), ints
(int/uint32/uint64), floats (NaN/Inf become `null` per the JSON
spec), booleans, and convenience helpers for arrays of primitives.

### Broadcast cadence

Two channels at different rates:

| Channel  | Rate   | Frame size | Use |
|----------|--------|------------|-----|
| `state`  | 10 Hz  | ~1.5 KB    | Overview tab — gesture, edit-mode banner, diagnostics |
| `waves`  | 20 Hz  | ~6 KB      | Waves tab — raw envelope + filtered envelope |

Per-client bandwidth: `(1.5 + 6) × 10 + 6 × 10 ≈ 135 KB/s`. Eight
simultaneous clients = 1 MB/s. Well within Pi 5 LAN throughput.

CPU on Core 0: serialization is the dominant cost. Measured ~50 µs
per state frame and ~200 µs per wave frame on a Pi 4 (less on Pi 5).
At 10 + 20 frame/s × 8 clients ≈ 36 ms/s = 3.6% of one core.

---

## 3. The IPC layout

The bridge mmaps the full `/dev/shm/cpcu_ipc` block. Region sizes
(matching `cpcu_ipc.h` and `cpcu_ipc_bridge.py::SHM_TOTAL`):

```
+----------------------+
| IPC_ControlBlock     |     192 B
| IPC_SensorEntry ring |  262 144 B   (4096 entries × 64 B)
| IPC_MotorCommand     |     128 B
| IPC_Diagnostics      |     128 B
| IPC_DSPExport        |     256 B
| IPC_RuntimeConfig    |     512 B
| IPC_ToolPresence     |     512 B    (8 slots × 64 B)
| IPC_DspFiltered      |   6 432 B    (32 B header + 8×200 floats)
+----------------------+
Total                  : 270 304 B
```

The Python bridge mmaps the same total (`SHM_TOTAL` in
`cpcu_ipc_bridge.py`). Any drift between the C `IPC_SHM_SIZE` and the
Python `SHM_TOTAL` is caught at runtime by the size-guard in
`IPCBridge.__init__` and aborts the DSP process loudly.

### `IPC_ToolPresence`

A small registry: 8 slots × 64 B each. Each tool that wants to be
visible writes its own slot (alive flag, heartbeat timestamp, name,
32-byte tool-specific payload). The bridge reads all slots and
forwards alive+fresh entries in the state frame's `tools[]` array.

Currently **signal_testbench** writes slot 1 on every main-loop
iteration (~20 Hz) and clears `alive=0` on clean exit. Payload layout:

```
payload[0]    = uint8   selected channel (0..7)
payload[1..4] = float32 amplitude (Vpp from the channel's analyzer)
payload[5..8] = uint32  io_pkts_dropped counter (latest snapshot)
```

**pca_testbench is intentionally NOT a publisher** even though slot 0
is reserved for it. The reason: pca_testbench and cpcu_kernel are
mutually exclusive (they both want to drive the I2C bus), so
`launch.sh` stops the kernel before starting pca_testbench. But if
cpcu_kernel is stopped, the bridge has nothing to read and is stopped
too. So the configuration "pca_testbench is running and visible on the
dashboard" is unreachable. Slot 0 stays reserved in case some future
bench tool needs it.

### `IPC_DspFiltered` (populated by `cpcu_dsp.py`)

Holds the most recent 200 samples (= 1 second @ 200 Hz) of the
post-filter envelope per channel. Updated by `cpcu_dsp.py` on every
processing window. With `WINDOW_MS = 200` and `STRIDE_MS = 100`, this
produces roughly one update every 100 ms (~10 Hz publish rate).

Layout: 32 B header (`seq`, `sample_rate_hz`, `update_us`, padding)
plus 8 channels × 200 samples × 4 B = 6400 B. Total 6432 B.

The Python bridge has a `write_dsp_filtered_window(ch_idx, samples_lo)`
method which appends a window's envelope to channel `ch_idx`'s rolling
buffer (shift-left by N, write new tail), with seqlock-style sequence
bumping so readers can detect torn writes.

### Filter chain producing the data

```
2 kHz input ──┐
              ▼
        decimate ×5   (cpcu_dsp.py: DECIMATE_FACTOR = INPUT_FS_HZ / TARGET_FS_HZ)
              ▼
      DC removal (subtract mean of the window)
              ▼
   bandpass 20-450 Hz  (Butterworth, scipy auto-clamps the upper
                        cutoff against Nyquist)
              ▼
        notch 50 Hz  (Q=30) → 100 Hz → 200 Hz
              ▼
   envelope = LP 3 Hz on |signal|   ◄── what gets published
              ▼
       feature extraction (RMS, var, WL, env_mean, MAV, ZC, SSC)
```

The bridge publishes the **envelope**, not the post-bandpass signal.
The envelope shows muscle activation patterns at small plot sizes;
the bandpass-only signal oscillates within the envelope and is too
high-frequency for the dashboard's wave plot.

For future spectrum / FFT views, the option chosen was to ship raw
12-bit ADC windows in the `raw_full` field and let the browser
compute its own FFT — that way DSP stays unchanged and the FFT cost
doesn't scale with viewer count.

---

## 4. Wire format

### State frame (Overview tab)

```json
{
  "ch": "state",
  "system_state": "running",
  "system_state_id": 1,
  "io_ready": true,
  "dsp_ready": true,
  "io_heartbeat_age_us": 980,
  "edit_mode": { "request": false, "active": false, "dsp_ack": false },
  "dsp": {
    "gesture": "rest",
    "active_class": 3,
    "num_classes": 4,
    "confidence": 0.91,
    "class_confidence": [0.02, 0.04, 0.03, 0.91],
    "channel_rms": [0.012, 0.018, ...],
    "inference_us": 142,
    "groups": [
      {"name":"right_arm","state":"rest","confidence":91,"classes":{"rest":91,"hand":4,"flex":3,"ext":2}},
      {"name":"left_arm","state":"rest","confidence":88,"classes":{"rest":88,"hand":6,"flex":4,"ext":2}}
    ]
  },
  "motor": {
    "servo_us": [1500, 1500, 1500, 1500, 1500, 1500],
    "gesture_id": 3,
    "confidence_pct": 91
  },
  "runtime_config": {
    "servo_min_us": [498, 1074, 1074, 1001, 1001, 976],
    "servo_max_us": [2500, 1953, 1953, 2002, 2002, 1733],
    "config_seq": 7
  },
  "diag": {
    "io_pkts_received":  124350,
    "io_pkts_dropped":   12,
    "io_ring_overflows": 0,
    "io_seq_gaps":       3,
    "io_safe_entries":   0,
    "io_gripper_stalls": 0,
    "dsp_inferences":    24870,
    "dsp_max_latency_us": 312
  },
  "hysteresis": { "consec": 2, "needed": 3, "type": 0, "type_name": "rest_to_active" },
  "latency":    { "pkt_to_servo_us": 35200, "seq_age": 18, "dsp_compute_us": 22400 },
  "bridge":     { "now_us": 4587000123, "ipc_version": 518 }
}
```

The active model has 4 classes (`ext`, `flex`, `hand`, `rest` —
alphabetical because sklearn sorts string labels at fit time). The
`groups[]` array carries the per-group classifier digest that
`cpcu_dsp.py` writes to `/tmp/cpcu_group_state.txt`; it lets the
dashboard render right_arm and left_arm side by side without needing
two IPC export blocks. The `runtime_config` block is forwarded from
`IPC_RuntimeConfig` and lets the JS rescale servo bars against the
operator's live calibration.

### Wave frame (Waves tab + Spectrum tab)

```json
{
  "ch": "waves",
  "raw_fs_hz": 50,
  "raw_n_samples": 50,
  "raw": [[ch0...], [ch1...], ..., [ch7...]],
  "raw_full_fs_hz": 2000,
  "raw_full_n_samples": 256,
  "raw_full": [[ch0_int16s...], ..., [ch7_int16s...]],
  "filtered_fs_hz": 200,
  "filtered_n_samples": 200,
  "filtered_update_us": 4587000123,
  "filtered_present": true,
  "filtered": [[ch0...], [ch1...], ..., [ch7...]]
}
```

The Waves-tab path (`raw`, `filtered`) and the Spectrum-tab path
(`raw_full`, 2 kHz raw 12-bit ADC values in 128 ms windows) are both
emitted on the wave frame, even though the shipped `index.html`
doesn't yet render Spectrum. `raw_full` adds ~10 KB per wave frame
at 20 Hz = ~200 KB/s per client, comfortably within Pi 5 LAN
throughput — the cost is paid whether anyone's looking or not, but
that's a fixed budget per client.

If `filtered_present` is false, the dsp publisher hasn't started yet —
either dsp isn't running, or it's running a build that predates the
`IPC_DspFiltered` region.

### Hello frame

Sent once on WS connect. Carries the bridge's compile-time
`IPC_VERSION` so the client can fail loudly if it expects a different
schema than the server's mapping:

```json
{ "ch": "hello", "server": "cpcu_ws", "ipc_version": 518 }
```

---

## 5. Deployment — three modes

### A. Local development (single user, loopback only)

```bash
cd cpcu_v2
build/cpcu_ws --bind ws://127.0.0.1:8765 --static web/static
```

Browse to `http://localhost:8765`. Useful when working at the Pi's
desktop without sharing.

### B. LAN-shared (the default — your friends watch)

```bash
build/cpcu_ws
# default: --bind ws://0.0.0.0:8765 --static /opt/cpcu/ws_static
# falls back to ./web/static if /opt/cpcu/ws_static doesn't exist
```

Loud startup banner says `WARNING: serving biosignals to your LAN`.
Tell your friends the Pi's IP (run `hostname -I`) and have them open
`http://<that-ip>:8765` in their browsers.

#### Friendlier URLs via mDNS (`cpcu.local:8765`)

Raspberry Pi OS ships with Avahi installed and running by default,
which means **the Pi already advertises itself as `<hostname>.local`**
on any network that supports mDNS / Bonjour (most home, dorm, and
campus networks do; some enterprise/guest networks block multicast).

Two small pieces of setup get your friends to a friendly URL:

1. **Set the Pi's hostname to `cpcu`** (or whatever you want to
   appear in `<hostname>.local`):

   ```bash
   sudo hostnamectl set-hostname cpcu
   sudo systemctl restart avahi-daemon
   ```

2. **Confirm it resolves.** From your laptop, on the same network:

   ```bash
   ping cpcu.local
   # or just open http://cpcu.local:8765/ in a browser
   ```

That's it — no extra service file needed. macOS, iOS, and modern
Linux desktops resolve `.local` natively. Windows resolves it if
either Bonjour Print Services or the Windows mDNS support is
installed (Windows 10+ resolves it natively from a recent update).

If `.local` doesn't resolve, your network probably blocks mDNS
multicast — fall back to the IP address. No bridge code change
helps with that; it's a network issue.

> **Note.** A formal `_http._tcp` Avahi service-publication file at
> `/etc/avahi/services/cpcu_ws.service` would make the dashboard
> appear in service-discovery apps (Bonjour Browser, `avahi-browse`)
> but doesn't change what works in a browser. We didn't add one
> because it's pure cosmetic for our use case. If you want it, the
> file is one paragraph of XML; see the Avahi docs.

### C. Public internet via SSH tunnel

```bash
# from your laptop
ssh -L 8765:localhost:8765 pi@your.pi.address
# then open http://localhost:8765 in your browser
```

This works without making the Pi internet-reachable. The tunnel
encrypts traffic; the bridge itself stays on loopback. Good for
remote demos to advisors who aren't on your LAN.

### Systemd

```bash
scripts/launch.sh install-ws-service
sudo systemctl start cpcu_ws
sudo systemctl status cpcu_ws
journalctl -u cpcu_ws -f
```

Generates `/etc/systemd/system/cpcu_ws.service` with:
- `After=cpcu.service network.target`
- `Requires=cpcu.service` (so the kernel must be up first; otherwise
  IPC_Open fails and the bridge waits)
- `Restart=on-failure`

---

## 6. Browser dashboard tabs

The shipped `index.html` exposes three tabs. The wire format
emitted by `cpcu_ws.c` carries enough information for additional
tabs (raw-full FFT data and a ToolPresence registry are already in
the payload); those would be UI additions, not bridge changes. See
the "Future work" section below.

### Arm

- Connection pill in the header: green = WS connected, red = retrying.
- System-state pill: RUNNING / SAFE / INIT.
- 3D arm canvas (three.js): six-DOF anthropomorphic arm with a
  scissor-jaw gripper. Each joint reads its angle from the
  corresponding `motor.servo_us[]` slot, mapped through the SV[]
  fallback table — which itself is **rescaled live** from
  `runtime_config.servo_min_us[]` / `servo_max_us[]` whenever the
  kernel publishes a fresh `IPC_RuntimeConfig`. Calibration done in
  the TUI live editor is reflected on the dashboard within one state
  frame (~100 ms).
- Overlay: current gesture name (large), top-1 confidence percentage,
  per-group cards (state + per-class confidence histogram for
  right_arm and left_arm).
- Sidebar: all-gesture class panel (one block per gesture group with
  rest/hand/flex/ext bars), hysteresis gate (state + vote counter),
  servo position slider per joint.

### EMG

- One card per channel (8 total).
- Channel names map to gestures.json:
  - ch0..ch2 = R_Hand, R_Biceps, R_Triceps (right_arm group)
  - ch3..ch5 = L_Hand, L_Biceps, L_Triceps (left_arm group)
  - ch6, ch7 = unused on the v3 BSAU board
- Each card shows the channel's rolling RMS as a horizontal bar.
- Updated at the 10 Hz state-frame cadence — sufficient for muscle
  activation visualisation, well under the actual sampling rate.

### Diagnostics

- **System state** rows: state, io_ready, dsp_ready, active gesture,
  confidence percentage.
- **Packet statistics**: packets received, sequence gaps, ring
  overflows, sequence age.
- **Latency breakdown**:
  - BSAU stage (datasheet, constants): ADC+pack (226 µs) + wireless
    (332 µs) = **558 µs**.
  - CPCU stage (measured by `cpcu_dsp.py`, exported via
    `IPC_DSPExport._pad1[]`): SPI read+unpack, ring dwell, DSP
    compute, inference worst-case.
  - Servo stage: smoother+I²C (~610 µs) + mechanical (~15 ms).
  - Totals: `pkt→servo_us` (CPCU only) and the full BSAU→CPCU→servo
    chain, colour-coded against the 300 ms SYS-REQ-01 budget.
- **Safety**: SAFE entries, gripper stalls, hysteresis type and votes.
- **DSP pipeline**: total inferences, gesture classes count, active
  class, per-class confidences.

---

## 7. Security stance

This is honestly modest:

- **No authentication.** Anyone who can reach `:8765` on the Pi can
  watch.
- **No encryption.** Plain HTTP/WS. A snoop on the LAN sees the
  EMG stream.
- **Read-only.** No control surface — the worst a malicious viewer
  can do is flood with subscriptions or watch your gesture stream.
- **LAN is the trust boundary.** The default bind is `0.0.0.0` so a
  capstone demo "just works" but the startup banner is loud about
  what's being served.

For a senior capstone on a private network, this is fine. If this
project ever moves to a hospital network or public-internet access:

- Stand up `nginx` reverse-proxy with HTTP basic auth and Let's
  Encrypt TLS in front of the bridge.
- Or, simpler: bind to `127.0.0.1` and require an SSH tunnel.

Both are out of scope today but documented as future work.

---

## 8. Troubleshooting

**"Mongoose NOT vendored — cpcu_ws will build as a stub"** in the
CMake output: run `web/vendor/fetch.sh` and re-run
`cmake -S . -B build`. The fetch script needs internet access (it
pulls from GitHub).

**`[WS] IPC_Open failed — is cpcu_kernel running?`**: the kernel
isn't up. Start it first (`scripts/launch.sh kernel` or
`sudo systemctl start cpcu`) and the bridge will succeed on retry.

**Browser shows "filtered stream not yet published"**: cpcu_dsp.py
isn't publishing into `IPC_DspFiltered`. Either dsp isn't running, or
it's running a build from before the dsp_filtered region existed.
Confirm with `journalctl -u cpcu | grep DSP`; the dsp process logs
the IPC version it sees on startup, which must match the bridge's.

**Multiple viewers connect but only one updates**: this would be a
bug; `cpcu_ws.c::broadcast()` walks every connection in the manager's
list. Check that all clients are `is_websocket==1` (i.e., the upgrade
succeeded). If you're seeing this, file an issue.

**Dashboard shows "disconnected — retrying" forever**: the WS handshake
is failing. Probably wrong port (`:8765` is the default), or the bridge
isn't running, or a firewall is blocking inbound. `curl -v
http://<pi-ip>:8765/` should return the index.html — if it doesn't,
the bridge isn't reachable from where you're testing.

**Page loads but no data flows**: the kernel and bridge are both up,
but dsp is feature-only mode (model files missing). The Overview tab
still shows live state and per-channel RMS; only the gesture name and
confidence are absent. Install a trained `.pkl` model and
run `./launch.sh set-model` to enable inference.

---

## 9. Current scope and future work

**Currently shipped:**

- 3-tab dashboard (Arm, EMG, Diagnostics) consuming the state frame.
- Live servo limits via `runtime_config` so calibration done in the
  TUI is reflected on the dashboard within a frame.
- mDNS / `cpcu.local:8765` resolution (uses the Pi's built-in Avahi;
  `sudo hostnamectl set-hostname cpcu`).
- `IPC_ToolPresence` slot 1 is populated by `signal_testbench` on
  every loop iteration (~20 Hz). The bridge forwards the registry
  as the state frame's `tools[]` array — nothing in the shipped
  `index.html` renders it yet (the data is there for whoever wires
  it up).
- The wave frame already carries `raw_full` (8 ch × 256 int16
  samples), enough for a browser-side FFT / spectrum tab when one
  is added.

**Deliberately out:**

| Item | Why |
|---|---|
| Spectrum / waterfall tab | Wire format ready (`raw_full`), JS not yet written. Would be a UI-only addition. |
| Tools tab on `index.html` | Same — `tools[]` array is already in the state frame; renderer not implemented. |
| pca_testbench publisher to slot 0 | `pca_testbench` is mutually exclusive with `cpcu_kernel` (both want the I²C bus). Because the bridge requires the kernel to be up, the configuration "pca_testbench running and visible on the dashboard" is unreachable. Slot 0 stays reserved. |
| Audio sonification of EMG | Not load-bearing for the defence. |
| Browser-side editing or commands | Explicit "no" — read-only by design. |
| TLS / authentication | LAN is the trust boundary (see §7). For public-internet access, document SSH tunnel; `nginx` front-proxy is the path forward if it matters. |
| Formal `_http._tcp` Avahi service-publication file | Pure cosmetic; `<hostname>.local` already resolves on networks that pass mDNS, no extra file needed. |

---

## 10. Files

| File | Purpose |
|------|---------|
| `./src/cpcu_ws.c` | Bridge process: IPC mapping, broadcast loop, JSON builders, Mongoose handler. Emits state frame (10 Hz) and wave frame (20 Hz), the latter including `raw_full` so browsers can compute their own FFT. |
| `./include/cpcu_json.h` + `./src/cpcu_json.c` | Hand-rolled JSON writer (no-malloc, caller-supplied buffer). |
| `./web/static/index.html` | Single-page browser dashboard (Arm / EMG / Diagnostics tabs). |
| `./web/vendor/README.md` + `fetch.sh` + `mongoose_stub.h` | Mongoose vendoring + offline-build stub. |
| `./include/cpcu_ipc.h` | All IPC region declarations; `IPC_VERSION` is the schema-binding contract. |
| `./src/cpcu_ipc.c` | Region pointer wiring in `ipc_map_ptrs`. |
| `./python/cpcu_dsp.py` | Per-window publish to `IPC_DspFiltered`; writes `/tmp/cpcu_group_state.txt` for the per-group view. |
| `./python/cpcu_ipc_bridge.py` | Python mmap of the same SHM; `write_dsp_filtered_window` helper. |
| `./scripts/launch.sh` | `ws` and `install-ws-service` subcommands. |
| `./test/signal_testbench.c` | Publishes to `IPC_ToolPresence` slot 1 — example of the tool-publisher pattern. |
| `./test/json_testbench.c` | Unit tests for the JSON serializer. |
| `./CMakeLists.txt` | `cpcu_ws` and `json_testbench` targets; mongoose presence detection. |

---

## 11. See also

- [`ARCHITECTURE.md`](ARCHITECTURE.md) — overall layering;
  the bridge is a Core 0 IPC consumer like the TUI.
- [`TUI_EDITOR.md`](TUI_EDITOR.md) — the dashboard's edit-mode banner
  reflects the same handshake state the TUI's editor uses; the
  actual editing only happens in the TUI.
- [`CONFIGURATION.md`](CONFIGURATION.md) — where the values
  visible in the dashboard come from.
