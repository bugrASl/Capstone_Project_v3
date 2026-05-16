# Web Dashboard — Read-Only Multi-Viewer Bridge

**Author:** bugrASl
**Date:** April 2026
**Version:** v2.4.0 (introduced)
**Last updated:** v2.4.1 (Spectrum + Tools + mDNS guidance)
**Audience:** anyone running the system, anyone planning to share live
demos with friends/advisors, anyone debugging "the dashboard says it's
disconnected."

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
                                   │  Overview       │
                                   │  Waves          │
                                   │  Spectrum       │
                                   │  Tools          │
                                   └─────────────────┘
```

The dashboard is **read-only**. There is no command channel from the
browser to the running system. The browser cannot enter edit mode,
save config, drive servos, or affect anything else. If you want to
tune values, use the TUI (which has the full v2.3.8 live editor) or
pca_testbench at the bench.

The bridge supports **multiple simultaneous viewers**. Friends on the
same LAN can each open the dashboard URL in their browser; each
connection gets its own broadcast. Default bind is `0.0.0.0:8765` so
the Pi accepts connections from any device on its network. A loud
warning prints at startup so you don't accidentally serve biosignals
to a public network.

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

## 3. The IPC layout in v2.4.0

The C side bumps `IPC_VERSION` from `0x0205` (v2.3.8) to **`0x0206`**.

Two new regions appear at the end of the SHM block:

```
... existing layout (192 + 65536 + 128 + 128 + 256 + 512 = 66752 B) ...
+----------------------+
| IPC_ToolPresence     |   8 slots × 64 B = 512 B
+----------------------+
| IPC_DspFiltered      |   32 B header + 8×200 floats = 6432 B
+----------------------+
```

Total SHM size grows from `66240` to `73696` bytes. This is reflected
in both the C macro `IPC_SHM_SIZE` and the Python bridge's
`SHM_TOTAL` constant (which `cpcu_dsp.py` mmaps).

### `IPC_ToolPresence` (declared in v2.4.0; partially populated in v2.4.1)

A small registry: 8 slots × 64 B each. Each tool that wants to be
visible writes its own slot (alive flag, heartbeat timestamp, name,
32-byte tool-specific payload). The bridge reads all slots and shows
whichever are alive on the Tools tab.

In v2.4.1, **signal_testbench** writes slot 1 on every main-loop
iteration (~20 Hz) and clears alive=0 on clean exit. Payload layout:

```
payload[0]    = uint8   selected channel (0..7)
payload[1..4] = float32 amplitude (Vpp from the channel's analyzer)
payload[5..8] = uint32  io_pkts_dropped counter (latest snapshot)
```

**pca_testbench is intentionally NOT a publisher** even though slot 0
is reserved for it. The reason: pca_testbench and cpcu_kernel are
mutually exclusive (they both want to drive the I2C bus), so launch.sh
stops the kernel before starting pca_testbench. But if cpcu_kernel
is stopped, the bridge has nothing to read and is stopped too. So
the configuration "pca_testbench is running and visible on the
dashboard" is unreachable. We left slot 0 reserved in case some
future bench tool needs it.

### `IPC_DspFiltered` (populated by cpcu_dsp.py)

Holds the most recent 200 samples (= 1 second @ 200 Hz) of the
post-filter envelope per channel. Updated by `cpcu_dsp.py` on every
processing window (~5 Hz with WINDOW_MS=200, STRIDE_MS=50).

Layout: 32 B header (seq, sample_rate_hz, update_us, padding) plus
8 channels × 200 samples × 4 B = 6400 B. Total 6432 B.

The Python bridge has a new method `write_dsp_filtered_window(ch_idx,
samples_lo)` which appends a window's envelope to channel ch_idx's
rolling buffer (shift-left by N, write new tail), with seqlock-style
sequence bumping so readers can detect torn writes.

### Filter chain producing the data

```
2 kHz input ──┐
              ▼
        decimate ×10  (200 Hz)
              ▼
      DC removal (subtract mean)
              ▼
   bandpass 20-95 Hz  (Butterworth, scipy auto-clamps from 450 to Nyquist*0.95)
              ▼
        notch 50 Hz  (Q=30)
              ▼
   envelope = LP 3 Hz on |signal|  ◄── what gets published
              ▼
       feature extraction (RMS/MAV/...)
```

The bridge publishes the **envelope**, not the post-bandpass signal.
The envelope is what shows muscle activation patterns at small plot
sizes; the bandpass-only signal oscillates within the envelope and
is too high-frequency for the dashboard's wave plot.

In v2.4.1 we wanted FFT for the Spectrum tab, which needs the
*signal* — not the envelope. We considered three options:

1. Have dsp publish a new `pre_envelope` channel.
2. Have dsp publish the bandpass-only intermediate.
3. Have the bridge ship raw 12-bit ADC windows and let the browser
   do its own filtering / FFT.

Option 3 won. The raw ring buffer is already what the bridge reads
for the `raw` envelope; sending a wider window of those same samples
costs no extra IPC region. The browser handles DC removal, Hann
windowing, and the FFT itself. dsp stays unchanged. This adds the
`raw_full` field to the wave frame (8 ch × 256 int16 samples per
frame, ~10 KB at 20 Hz = 200 KB/s per client).

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
    "active_class": 0,
    "num_classes": 3,
    "confidence": 0.91,
    "class_confidence": [0.91, 0.05, 0.04],
    "channel_rms": [0.012, 0.018, ...],
    "inference_us": 142
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
  "bridge": { "now_us": 4587000123, "ipc_version": 518 }
}
```

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

The Waves tab consumes `raw` (50 Hz envelope) and `filtered` (200 Hz
envelope). The Spectrum tab (added in v2.4.1) consumes `raw_full`
(2 kHz raw 12-bit ADC values, 128 ms windows) and computes its own
FFT in-browser. `raw_full` adds ~10 KB per wave frame at 20 Hz =
200 KB/s per client, comfortably within Pi 5 LAN throughput.

If `filtered_present` is false, the dsp publisher hasn't started yet —
either dsp isn't running, or it's running an old build that doesn't
know about `IPC_DspFiltered`.

### Hello frame

Sent once on WS connect:

```json
{ "ch": "hello", "server": "cpcu_ws", "version": "v2.4.0", "ipc_version": 518 }
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

### Overview (live in v2.4.0)

- Connection pill: green/yellow/red
- Edit-mode banner: LOCKED / EDITING / PARKING / DSP UNRESPONSIVE
- System: state, io_ready, dsp_ready, heartbeat age
- Current gesture: large name + confidence percentage
- Per-class confidence bars (the active class is highlighted)
- Per-channel RMS bars (relative to peak)
- Diagnostics counters (rx/dropped/ring/seq_gaps/safe/gripper_stalls/dsp_*)

### Waves (live in v2.4.0)

- Toggle: raw / filtered / both
- 8 channels stacked vertically, each track ~60 px tall
- Raw stream: 50 Hz envelope from `IPC_SensorEntry` ring (decimated)
- Filtered stream: 200 Hz envelope from `IPC_DspFiltered`
- Both streams plotted as line traces, raw in blue, filtered in green
- Auto-rescaling per track within [0..1] envelope range

### Spectrum (live in v2.4.1)

Per-channel selector buttons (ch0..ch7) at top. Two side-by-side
panels:

- **Left: static spectrum.** 256-pt FFT of the most recent 128 ms
  raw window for the selected channel. Hann window, DC removed.
  Resolution ≈ 7.8 Hz/bin. A vertical reference line marks 50 Hz so
  mains-hum spikes are obvious. Auto-normalized to the latest peak;
  read it as relative shape, not absolute volts.
- **Right: waterfall.** ~30 seconds of trailing FFT history for the
  selected channel, viridis-colored. Newest at the top. Brighter =
  more energy. Bands across time = persistent feature (good or bad);
  intermittent flicker = loose contact or radio glitches; rising
  noise floor = degrading link.

The browser computes the FFT itself with a small radix-2 Cooley-Tukey
implementation (~50 LOC of JS). The bridge ships the raw-full window
in the wave frame as `raw_full` (8 channels × 256 int16 samples,
~10 KB per frame at 20 Hz = 200 KB/s per client). Server-side FFT
was rejected: per-frame cost would scale with client count and the
browser does it for free.

**Why not waterfall *in addition to* the existing wave plots?** The
Waves tab is the *envelope* view (slow muscle activation patterns,
post-3-Hz-LP). The Spectrum tab is the *spectral content* view (raw
signal, full bandwidth up to 1 kHz). Both views are useful for
different reasons; one doesn't replace the other.

### Tools (live in v2.4.1, partial)

Reads the `IPC_ToolPresence` registry every state frame and lists
tools that are alive (`alive=1`) and fresh (heartbeat < 2 s old).
Each tool gets a card with:

- Status dot (green = fresh, red = stale)
- Tool name and slot number
- Heartbeat age in milliseconds
- Per-tool decoded state (channel, amplitude, drops, etc.)

In v2.4.1, only **signal_testbench (slot 1)** publishes to this
registry. The pca_testbench publisher is intentionally deferred —
see §9 for the rationale.

If no tools are running, the panel says "no tools running" (which
is the most common state — most of the time you only have
cpcu_kernel running, and the dashboard is reading from it).

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

Both are out of scope for v2.4.0 but documented as future work.

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
it's running an older build (pre-v2.4.0). Confirm with
`journalctl -u cpcu | grep DSP` — the v2.4.0 dsp logs the IPC version
on startup.

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
confidence are absent. Install `models/svm.joblib` and
`models/scaler.joblib` to enable inference.

---

## 9. v2.4.1 status — what shipped, what's still out

### Shipped in v2.4.1

| Feature | Status |
|---|---|
| Spectrum tab — per-channel selector, browser FFT, waterfall | ✓ shipped |
| signal_testbench publisher to `IPC_ToolPresence` slot 1 | ✓ shipped |
| Tools tab — reads `IPC_ToolPresence`, shows alive tools | ✓ shipped |
| mDNS / `cpcu.local:8765` resolution | ✓ documented (uses Pi's built-in Avahi; `sudo hostnamectl set-hostname cpcu`) |

### Still out — and why

| Out-of-scope | Why |
|---|---|
| pca_testbench publisher | The tool is mutually exclusive with cpcu_kernel (both want the I2C bus). Since the bridge requires cpcu_kernel running, the configuration "pca_testbench is running and visible on the dashboard" is unreachable. Slot 0 stays reserved. |
| Audio sonification of EMG (Web Audio API) | Cute, not load-bearing for the defense |
| 3D arm visualization | Needs geometry; a botched 3D viz looks worse than no viz. Targeted for v2.5. |
| Browser-side editing or commands | Explicit "no" — read-only by design |
| TLS / authentication | LAN is the trust boundary (see §7). For public-internet access, document SSH tunnel; nginx-front-proxy is the path forward if/when it matters. |
| Formal `_http._tcp` Avahi service-publication file | Pure cosmetic; `<hostname>.local` already resolves on networks that pass mDNS, no extra file needed |

---

## 10. Files

| File | Purpose |
|------|---------|
| `cpcu_v2/src/cpcu_ws.c` | Bridge process: IPC mapping, broadcast loop, JSON builders, Mongoose handler. v2.4.1 added `raw_full` field for browser FFT and `tools` array reading `IPC_ToolPresence`. |
| `cpcu_v2/include/cpcu_json.h` + `cpcu_v2/src/cpcu_json.c` | Hand-rolled JSON writer |
| `cpcu_v2/web/static/index.html` | Single-page browser dashboard. v2.4.1 added Spectrum tab (FFT + waterfall) and Tools tab. |
| `cpcu_v2/web/vendor/README.md` + `fetch.sh` + `mongoose_stub.h` | Mongoose vendoring |
| `cpcu_v2/include/cpcu_ipc.h` | Region declarations (IPC_ToolPresence, IPC_DspFiltered); IPC_VERSION 0x0206 |
| `cpcu_v2/src/cpcu_ipc.c` | New region pointers wired in `ipc_map_ptrs` |
| `cpcu_v2/python/cpcu_dsp.py` | Per-window publish to `IPC_DspFiltered` |
| `cpcu_v2/python/cpcu_ipc_bridge.py` | Region offsets + `write_dsp_filtered_window` method |
| `cpcu_v2/scripts/launch.sh` | `ws` and `install-ws-service` modes |
| `cpcu_v2/test/signal_testbench.c` | v2.4.1: publishes to `IPC_ToolPresence` slot 1 each loop iteration |
| `cpcu_v2/test/json_testbench.c` | 7 unit tests for the JSON serializer |
| `cpcu_v2/CMakeLists.txt` | `cpcu_ws` and `json_testbench` targets; mongoose presence detection |

---

## 11. See also

- [`ARCHITECTURE.md`](ARCHITECTURE.md) — overall layering;
  the bridge is a Core 0 IPC consumer like the TUI.
- [`TUI_EDITOR.md`](TUI_EDITOR.md) §4 — the dashboard's edit-mode banner
  reflects the v2.3.4 handshake state.
- [`TUI_EDITOR.md`](TUI_EDITOR.md) — what edit mode is *for*. The
  dashboard shows when someone is editing; the actual editing
  happens in the TUI.
- [`CONFIGURATION.md`](CONFIGURATION.md) — where the values
  visible in the dashboard come from.
