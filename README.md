# nx-tracker-osc (`nxosc`)

Read the **Waves Nx Head Tracker** (NXIMU010) over Bluetooth LE on Linux
(BlueZ), decode head orientation, and re-emit it as OSC compatible with the
**IEM SceneRotator** (IEM Plugin Suite). Built as a standalone crate, kept
modular (`ble` / `decode` / `osc`) for later integration into the
[Omniphony](https://github.com/mgth/Omniphony) spatial renderer. Licensed
GPLv3 (`GPL-3.0-only`), matching Omniphony.

> Status: **Phase 3** done — `run` streams decoded orientation over OSC
> (IEM SceneRotator or a Sensors2OSC-style feed for Omniphony), with software
> recenter, smoothing, rate-limit and auto-reconnect.

## Roadmap

| Phase | Command | State |
|-------|---------|-------|
| 0 | skeleton + CLI (`scan`/`raw`/`run`) | done |
| 1 | `raw` — connect, start, dump frames + live candidate interpretations + CSV | done |
| 2 | `decode` — `decode(&[u8]) -> Orientation` (quaternion + yaw/pitch/roll), unit-tested on captured frames | done |
| 3 | `run` — stream orientation as OSC (SceneRotator + Sensors2OSC/Omniphony), recenter/smoothing/rate-limit | done |

## Payload layout (`a015`), determined empirically

10-byte notification:

| bytes | meaning |
|-------|---------|
| 0..2  | `x` — `int16` LE, Q1.14 (value / 16384) |
| 2..4  | `y` — `int16` LE, Q1.14 |
| 4..6  | `z` — `int16` LE, Q1.14 |
| 6..8  | `w` — `int16` LE, Q1.14 |
| 8..10 | constant `00 03` (footer/flags) — ignored |

`(w, x, y, z)` is a **unit quaternion** (measured norm = 1.0000, sd 2e-5 across
all captures). Device body axes match the ADM convention used by Omniphony:
X = pitch, Y = roll, Z = yaw. yaw/pitch/roll are extracted for the intrinsic
Z-X-Y sequence (inverse of `HeadPose::from_euler_deg`). Near pitch = ±90° the
Euler triple hits gimbal lock; the quaternion stays valid.

## Bluetooth prerequisites (BlueZ)

`nxosc` talks to the system BlueZ daemon over D-Bus; it does **not** pair the
device itself. Pair and trust the tracker once with `bluetoothctl`:

```sh
rfkill unblock bluetooth          # if the adapter is soft-blocked
bluetoothctl
[bluetooth]# power on
[bluetooth]# scan on
# ... wait until a device named like "nx tracker" appears, note its MAC ...
[bluetooth]# pair  AA:BB:CC:DD:EE:FF
[bluetooth]# trust AA:BB:CC:DD:EE:FF
[bluetooth]# scan off
[bluetooth]# quit
```

Notes:
- The tracker advertises a name containing **"nx tracker"** (case-insensitive).
- `trust` lets it reconnect automatically after sleep — required for the
  auto-reconnect behaviour of `raw`/`run`.
- Permissions: using the BlueZ D-Bus API normally needs **no** extra
  privileges or group membership. If `scan`/`raw` reports no adapter, check
  `systemctl status bluetooth` and `bluetoothctl show` (adapter `Powered: yes`).
- If the device is busy/connected elsewhere (e.g. the macOS NXOSC app or a
  phone), disconnect it there first.

## Build

```sh
cargo build --release    # binary at target/release/nxosc
```

Linux build dependency: the BlueZ backend talks D-Bus; ensure a D-Bus
development setup is present if the build complains (`pkg-config`, `dbus`).

## Usage

```sh
# 1. Find the tracker (Nx devices are listed first and marked).
nxosc scan
nxosc scan --secs 8

# 2. Capture (Phase 1). Move your head on ONE axis at a time and watch which
#    rows/columns develop a wide RANGE — those slots encode that axis.
nxosc raw                                  # discover by name, live range table
nxosc raw --address AA:BB:CC:DD:EE:FF       # force a MAC
nxosc raw --csv capture.csv                 # also log every raw frame for offline analysis
nxosc raw --stream                          # one block per packet instead of the live table
nxosc raw --decode                          # also show decoded quaternion + yaw/pitch/roll (Phase 2)
nxosc raw --csv yaw.csv                     # suggested: one CSV per axis (yaw/pitch/roll/still)

# verbosity
nxosc -v raw           # debug logs   (RUST_LOG overrides, e.g. RUST_LOG=trace)
```

### What `raw` shows

For each notification it tracks four competing readings of the payload —
`i16 LE`, `i16 BE`, `f32 LE`, `f32 BE` — and, per slot, the live value plus the
**min / max / range** observed since start, with a bar scaled within each
group. A slot whose bar grows when you yaw (and stays flat otherwise) is a yaw
component; flat constant bytes are header; a slot that only ever increases is a
packet counter. `--csv` always records every frame as `ts_us,len,hex` for
offline analysis.

### `run` — stream orientation as OSC (Phase 3)

```sh
# IEM SceneRotator (default profile). yaw/pitch/roll in degrees:
nxosc run --address AA:BB:CC:DD:EE:FF
nxosc run --mode quaternion                  # send /SceneRotator/quaternions w x y z instead
nxosc run --osc-target 127.0.0.1:9000

# Omniphony / Sensors2OSC emulation (test directly in Omniphony):
nxosc run --profile omniphony --osc-address /gamerotationvector --osc-target 127.0.0.1:9000
```

OSC wire formats (verified against IEM docs / a reference head-tracker script
and Omniphony's own parser):

| profile | mode | message |
|---------|------|---------|
| `scenerotator` | `ypr` (default) | `/SceneRotator/ypr <yaw> <pitch> <roll>` (degrees) |
| `scenerotator` | `quaternion` | `/SceneRotator/quaternions <w> <x> <y> <z>` |
| `omniphony` | `quaternion` (default) | `<--osc-address> <x> <y> <z> <w>` (Android order; Omniphony reads `from_quat(w,x,y,z)`) |
| `omniphony` | `ypr` | `<--osc-address> <yaw> <pitch> <roll>` (requires Omniphony `format = euler`) |

To test in Omniphony, set `render.binaural.head_tracking.osc_address` to the
same value as `--osc-address` (and `format` to `auto`/`quat` for quaternion).

Extras:
- **Recenter**: press **Enter** while `run` is active to set "forward = current
  heading" (yaw-only; pitch/roll stay gravity-referenced). `--recenter-on-start`
  zeroes at launch.
- `--smoothing <0..1>`: exponential quaternion smoothing (0 = instant).
- `--max-hz <n>`: cap the OSC send rate (default 60).
- Auto-reconnects when the tracker sleeps/wakes.

### Device exploration (`gatt` / `probe`)

```sh
# Map the device: connect WITHOUT sending start, print every service +
# characteristic and read the readable ones (battery, firmware, manufacturer…).
nxosc gatt

# Experiment on the a011 command characteristic and measure the a015 rate.
nxosc probe --rate 100      # write [100 u32 LE, 0x01] -> expect ~100 Hz
nxosc probe --stop          # write [0x32,0,0,0,0x00]  -> expect the stream to stop
nxosc probe --cmd "32 00 00 00 01"   # write arbitrary bytes to a011
nxosc probe --sweep 50:100:5         # one connection; find where the rate "steps up"
```

`probe --sweep` takes a comma list ("50,60,70") or a range ("lo:hi:step") and
writes each rate then measures on the **same** connection — avoiding the
reconnect churn that can wedge the BLE link / send the tracker to sleep.

The start command is **`[rate (u32 LE), enable (u8)]`** — `0x32` = 50, and the
stream runs at ~50 Hz. Measured behaviour (`probe`):

| requested | measured | |
|-----------|----------|--|
| 50 | ~50 Hz | default, honored |
| 75 / 100 / 200 / 400 | ~98–100 Hz | saturates at the BLE link ceiling (~100 Hz) |
| 30 / 10 | erratic (49 / 3.7 Hz) | values below 50 are unreliable |

So the useful range is **50–100 Hz**; ~100 Hz halves the head-tracking sampling
latency. The 5th byte is **not** a clean enable (writing `0` did not stop the
stream). `gatt` is read-only; `probe` only ever *writes* to `a011` — no other
characteristic is touched, so a DFU/firmware service (if present) is safe.

Both `raw` and `run` accept **`--rate <hz>`** (default 50) to request a faster
stream, e.g. `nxosc run --rate 100 …` for lower-latency head tracking.

GATT map (from `nxosc gatt`): Device Information reports firmware `v100`,
hardware `v4.4`, software `A v1.30 B v1.13`, "Waves Audio"; a Battery service;
and vendor characteristics including `a018` (the writable device name) and two
further vendor services (`a030`, `a050`) of unknown purpose — `a030` looks like
a DFU control point, so it is left untouched.

### Capture protocol for Phase 2

To pin down the layout, capture a few short, labelled runs and send the CSVs:

1. `still.csv` — tracker flat and motionless (baseline / constant bytes).
2. `yaw.csv` — slow full left/right rotation only.
3. `pitch.csv` — slow nod up/down only.
4. `roll.csv` — slow tilt left/right only.

## License

GPL-3.0-only. See [LICENSE](LICENSE).
