# Dual-IMU Knee Goniometer

Two Arduino Nano 33 BLE Rev2 boards (one on the thigh, one on the shank) measure
knee flexion angle. Each board runs a 6-DOF orientation filter; the **shank
board (peripheral)** streams its state to the **thigh board (central)** over a wired UART
link, and the central merges both segments and streams them to a PC, where
`knee_collector_uart.py` computes and logs the knee angle.

The design goal is a **placement-independent** angle: you should not have to mount
the boards in a precise, repeatable orientation.

---

## Hardware & wiring

- 2× **Arduino Nano 33 BLE Rev2** (onboard **BMI270** accel+gyro; BMM150
  magnetometer is present but **unused**).
- UART between the boards (`Serial1`, **115200 baud**), **both directions wired**:
  the peripheral streams data to the central, and the central sends a keepalive to the
  peripheral so it only runs during a collection (see "Collect-on-demand" below):

  ```
  Peripheral TX (D1)  ->  Central RX (D0)     # shank data stream
  Central TX (D1) ->  Peripheral RX (D0)      # keepalive / start-stop
  Peripheral GND     <->  Central GND
  ```
- Central connects to the PC over USB.
- Mount convention used in testing: board long axis along the limb long axis,
  USB port toward the hip, one board above and one below the knee. Exact rotation
  / position does **not** need to match between boards (see "How it works").

---

## Quick start

1. **Flash** `central_imu/central_imu.ino` to the thigh board and
   `peripheral_imu/peripheral_imu.ino` to the shank board (Arduino IDE, board = "Arduino
   Nano 33 BLE", library **Arduino_BMI270_BMM150**). Confirm each reports
   *Done uploading*.
2. On power-up / reset, **keep both boards still for ~4 s** — they measure gyro
   bias then. The central prints `# central gyro bias dps: ...`.
3. **Sanity check** the link and signal:
   ```
   python knee_collector_uart.py --port /dev/ttyACM0 --monitor
   ```
   `thigh` / `shank` are the gyro-fused gravity tilts the angle uses; `accel` is
   a filter-free raw-accel cross-check; `rel` is the yaw-prone relative-quaternion
   angle (drift probe); `valid%` / `rtt` show link health.
4. **Collect**:
   ```
   python knee_collector_uart.py --port /dev/ttyACM0
   ```
   Calibration is three phases, each paced to you (not a stopwatch):
   1. **ZERO** — stand with the leg **straight** and hold **still**. It waits for a
      genuinely quiet window (~5 s) before capturing the zero, so a little sway
      can't bias it.
   2. **HIP** — keep the **knee locked straight** and swing the whole leg from the
      **hip**, forward and back. This learns the thigh's forward axis on its own,
      and (knee locked → knee should read ~0) doubles as a check on hip-into-knee
      leakage.
   3. **KNEE** — **sit down, hold the thigh still**, and bend/straighten the
      **knee**. This learns the shank's forward axis on its own.

   Each sweep advances as soon as the segment has covered enough range, and any
   poor calibration (out-of-plane swing, too little range, thigh not held still)
   is flagged. The signed knee angle is then printed live and logged to
   `knee_log.csv`.

   Separating the hip and knee sweeps is deliberate: a single combined
   sit-to-stand makes both segments move together, so neither forward axis is
   clean — the main reason hip motion leaks into the knee reading.

`--selftest` (no hardware) runs the full math test suite.

---

## Live GUI (`knee_gui.py`)

For a testing / proof-of-concept session there is a Tkinter + Matplotlib front
end over the same angle math. It is the easiest way to run a collection:

```
pip install -r requirements.txt
python knee_gui.py                 # scan for the central port and go
python knee_gui.py --simulate      # no hardware: synthetic flexing-knee source
python knee_gui.py --selftest      # headless: sample-gate + source logic
```

What it adds over the CLI:

- **Port scan** — probes each serial port for valid `D` lines and picks the
  central automatically. (Only the central is on USB; a dead peripheral link shows up
  as a low shank-valid %, not a second port.)
- **Consistent 50 Hz** — the fixed 50 Hz device stream is resampled onto a
  fixed 20 ms grid, so both the CSV and the plots are a clean 50 Hz record
  regardless of source jitter.
- **Obvious calibration** — a colour-coded banner drives the phases: amber
  **ZERO** (hold straight & still, with a live "how quiet" readout) → amber
  **HIP** (knee locked, swing from the hip) → amber **KNEE** (thigh still, bend
  the knee), each showing live progress toward its target → green **RUNNING**.
  The zero is gated on stillness and each sweep on range covered, so it paces to
  the user and flags a poor calibration instead of trusting it.
- **Plots** — knee angle (primary), the two segment inclinations it is built
  from, and the link RTT. While collecting they show a rolling window; when you
  **Stop collecting** they switch to the **entire session** (not just the last
  window) and a pan/zoom/save toolbar becomes usable so you can inspect the
  frozen trace (Home returns to the full-session view).
- **Dropout mode (switchable live)** —
  **Fill** forward-fills short gaps and draws a continuous line;
  **Gap** keeps only real samples and draws discrete points, so dropouts appear
  as visible gaps. The CSV records which samples were real in either mode.
- **Session-based collect / stop / reset** — each **Calibrate & collect** starts
  a fresh session: a new timestamped CSV, a cleared plot, and calibration from
  scratch. **Stop collecting** ends it, freezes the display on the last data,
  and returns to idle. Collecting again is a clean reset — you recalibrate, and
  a new file is written. (There is no pause/resume: the display only rolls while
  a session is actually collecting.)
- **Errors** — a red banner names the exact fault (no data / wrong firmware /
  dead peripheral link), using the same diagnosis as the CLI.
- **Auto-save** — each session auto-saves to `knee_YYYYMMDD_HHMMSS.csv`;
  **Save copy…** relocates the current one. A crash never loses a session.

The CSV is a superset of the CLI log (adds wall-clock and session time, the two
inclinations, and the `phase` / `fill_mode` columns), so existing analysis still
reads it.

---

## How it works (the math)

Each IMU estimates orientation with a **6-DOF Mahony filter** (accelerometer +
gyroscope, no magnetometer — robust to nearby metal). There is **one** angle
method, chosen for reliability across both slow and brisk motion.

### Gravity-referenced sagittal inclinometer

The reliable, observable quantity in a 6-DOF filter is the **direction of gravity
in each board's own frame**: the accelerometer pins it (so it doesn't drift in
tilt) and it's invariant to heading (so the 6-DOF yaw, which *does* drift, never
enters). We read that gravity direction from the **fused quaternion** — so the
gyro carries it smoothly through fast motion instead of the reading collapsing
whenever the limb accelerates — then build a signed segment inclination and take
the difference. Here is the full derivation.

Notation: quaternions are `(w, x, y, z)`, Hamilton convention; `q*` is the
conjugate; world "up" is `ẑ = (0, 0, 1)`. Each board `i ∈ {thigh, shank}`
reports a unit orientation quaternion `q_i` (board → world).

**1. Gravity in the board frame.** Rotate world-up into the board's own frame
with the inverse orientation, and keep the vector part:

```
g_i = vec( q_i* ⊗ (0, ẑ) ⊗ q_i ),   then normalize      # unit vector
```

`g_i` is the direction of gravity as the board feels it. It uses only the tilt
part of `q_i`, so it is **drift-free** (the accelerometer fixes tilt) and
**yaw-invariant** (rotating `q_i` about vertical leaves `g_i` unchanged). Code:
`gravity_in_board()`.

**2. Zero reference `d_i` (stillness-gated static hold).** With the leg held
straight and still, average `g_i` over the hold window. `d_i` is the segment's
long axis as seen in the board frame — the direction gravity points at 0°:

```
d_i = normalize( Σ g_i(t)  over the still hold )         # code: average_gravity()
```

The zero is captured only from a **genuinely quiet** window — the code waits until
the spread of `g_i` over the trailing `CAL_SECONDS` stays under `ZERO_STILL_TOL_DEG`
for both segments (`gravity_spread_deg()`), falling back after `ZERO_MAX_WAIT`.
Averaging longer doesn't help once a slow postural drift dominates sensor noise;
waiting for stillness does.

**3. Forward direction `f_i` (per-segment functional sweep).** As the segment
flexes by angle `t`, `g_i = cos(t) d_i + sin(t) f_i`, so the part of `g_i`
**perpendicular to `d_i`** is `sin(t) f_i` — every such sample lies on the **line
spanned by `f_i`**. So `f_i` is the dominant principal direction of the
perpendicular components (a 2×2 eigenproblem in the plane perp to `d_i`), and the
secondary spread measures how far the motion strayed out of plane:

```
g⊥ = g_i − (g_i · d_i) d_i                                # in-plane component
f_i        = dominant eigenvector of Σ g⊥ g⊥ᵀ            # code: plane_forward()
planarity  = 1 − λ2/λ1     (1.0 = perfectly planar)
```

This replaces the old sign-aligned sum (which anchored on one noisy sample) with a
principled fit and a free **quality score**: a sweep with `planarity < PLANARITY_MIN`
is flagged rather than trusted. Samples tilting less than `SWEEP_MIN_ANGLE_DEG` are
ignored so noise around the zero can't define the axis. `d_i` and `f_i` are
orthonormal and span that segment's **sagittal plane** in its own frame; if a
segment barely moved, `f_i` is undefined and it contributes 0.

The two forwards are learned from **separate** motions — `f_thigh` from a
knee-locked hip swing, `f_shank` from a thigh-fixed knee flex — so the two
segments never move together and contaminate each other's axis. Because the knee
is locked during the hip swing, its angle should stay ~0 throughout; the code
measures the largest residual (`hip_lock_residual_deg()`) and warns past
`HIP_KNEE_RESIDUAL_WARN_DEG`, surfacing exactly the out-of-plane hip motion that
leaks into the knee.

**4. Signed segment inclination.** The tilt of the segment is the angle of `g_i`
within the `(d_i, f_i)` plane — the four-quadrant angle from the zero axis `d_i`
toward forward `f_i`:

```
incl_i = atan2( g_i · f_i , g_i · d_i )   [rad]           # code: sagittal_inclination()
```

At the zero pose `g_i = d_i`, so `g_i · f_i = 0`, `g_i · d_i = 1`, and
`incl_i = 0`. `atan2` gives a continuous signed angle through the full range
(flexion positive, hyperextension negative), with no ±90° wrap.

**5. Knee angle.** The joint angle is the difference of the two segment
inclinations:

```
knee = incl_thigh − incl_shank        (converted to degrees)   # code: gravity_knee_angle()
```

*Why the difference is placement-independent (sketch):* for planar flexion the
segment rotates about a fixed axis, so `g_i(t)` traces a circular arc that lies
entirely in one plane of the board frame — and a rigid mount `B_i` maps that plane
to a fixed plane spanned by `d_i, f_i`. `incl_i` reads the arc angle inside that
plane, and the fixed mount rotation only rotated the plane, not the angle within
it. So any constant mount `B_i` cancels exactly (the `--selftest` proves this
against random `B_thigh`, `B_shank`).

Because every quantity is an angle between two vectors in the **same** board
frame, the result is immune to:
- **Position** on the limb (orientation sensors don't measure location);
- **Constant mounting rotation**, incl. **rotation of the board about the leg's
  long axis** (any fixed mount rotation cancels);
- **6-DOF yaw drift** (gravity-in-board is invariant to heading about vertical).

Validated in `--selftest` against random mounts, a turning shared heading,
independent per-board yaw drift (exact recovery), a full flex-to-130°-and-back
sweep (the signed angle retraces identically — no stuck zeros on return), and the
**separated hip/knee calibration** (learning each forward axis from an isolated
motion still recovers the true knee, and a pure hip motion reads ~0).

**Trade-off:** gravity gives only **2 of the 3 rotational DOF** — it is blind to
rotation about the vertical (gravity) axis. So this method measures the
**sagittal-plane** component of the angle — ideal for upright knee flexion
(standing ROM, gait, sit-to-stand). It under-reads motion well out of the vertical
plane (e.g. lying down with large hip rotation). Note this is an *observability*
limit, not a mounting one: a fixed mount tilt cancels for planar motion (step 5),
but tilt combined with out-of-sagittal-plane motion steers part of the true joint
rotation into the unobservable yaw direction, where gravity cannot see it — the
one case a gravity-only method fundamentally cannot recover.

This is why **hip motion can still leak into the knee** even after calibration: a
real hip movement carries some abduction and thigh axial rotation, and axial
rotation of the thigh is exactly that unobservable DOF. The separated calibration
and the planarity/hip-lock checks **minimize and surface** this leakage (learn
each forward axis from a clean, isolated, in-plane motion; warn when a sweep or the
locked hip swing goes out of plane), but they cannot remove it entirely —
eliminating it would require magnetometer-aided heading or a hinge-axis
constraint, a larger change than this proof of concept.

### What "0°" means (relative, not absolute)

The angle is **relative to the pose held during the static zero** — whatever
posture you hold at calibration becomes 0°, regardless of the true anatomical
geometry. A leg that is anatomically straight but sits at a residual angle
(recurvatum, or a patient who can't fully extend) still reads **0°** here. Every
reading is therefore a **change from the zeroing pose**, not an absolute
femur-vs-tibia angle.

This is exactly right for what a session actually measures — **ROM (max − min),
rep counting, gait, and movement quality** are all difference-based, so the
reference cancels and the choice of zero doesn't matter. What it does **not**
give:

- an **absolute clinical angle** (e.g. "flexion contracture of 10°"), which is a
  statement about the zero itself;
- comparability **across sessions** zeroed on different poses, since each
  session's 0° may correspond to a different real angle.

Recovering a true anatomical zero would require an **external reference** — a
manual goniometer on the leg, or a fixture that physically defines full
extension. The two IMUs alone cannot observe it: the method is deliberately blind
to how each board is mounted relative to the bone (that blindness is exactly what
makes it placement-independent), so there is no anatomical landmark in the data
for it to find absolute zero from. For this proof of concept we accept the
relative zero and rely on the difference-based readings above.

### Why the quaternion (not the raw accelerometer)

Gravity-in-board is taken from the Mahony quaternion, which fuses accel + gyro,
rather than from the bare accelerometer. During any brisk movement the raw
accelerometer measures gravity **plus linear acceleration**, so its direction is
momentarily wrong — the cause of erratic readings (and apparent zeros) right after
swinging back from a high angle. The fused quaternion lets the **gyro carry** the
fast part while the accelerometer keeps the tilt drift-free, so the angle stays
smooth *and* drift-free. The firmware trusts the accelerometer with a **soft
gate** (`ACC_TRUST_FULL_G` → `ACC_TRUST_ZERO_G`): full weight when near-static,
ramping to zero as `|accel|` leaves 1 g — always applying *some* correction, so
the estimate never runs fully open-loop and then snaps back.

(The raw accelerometer is still streamed and shown as a filter-free cross-check in
`--monitor`.)

---

## Commands

```
python knee_collector_uart.py --port PORT [options]

  --monitor                 live bring-up check (gyro-fused vs raw-accel tilts)
  --raw                     dump raw serial lines with field counts, then exit
  --cal-seconds N           length of the STILL window required to zero
                            (stillness-gated, not a countdown; default 5)
  --sweep-seconds N         per-sweep timeout; each of the hip and knee sweeps is
                            motion-gated and normally advances earlier (default 12)
  --out FILE                CSV path (default knee_log.csv)
  --selftest                run the math self-tests (no hardware)
```

CSV columns: `t_thigh_us, thigh_qw..qz, shank_qw..qz, knee_angle_deg, status,
rtt_us` (status ∈ `zeroing / hip / knee / valid / filled / missing`).

---

## Data format / wire protocol

**Peripheral → central stream (binary, 30 bytes/packet, fixed 50 Hz):**
```
[0]      0xAA header
[1..16]  float q0..q3   (little-endian, w,x,y,z)
[17..28] float ax,ay,az (little-endian, g, handedness-corrected)
[29]     XOR checksum of bytes [1..28]
```
**Collect-on-demand & rate.** The boards run their IMUs only while a collection is
active, so nothing runs unobserved. "Active" = the central's **USB port is open** (the
collector, GUI, or even the Serial Monitor holds it) — no PC-side command needed. On
start the central calibrates + seeds fresh and forwards a **keepalive** to the peripheral
(over `Central TX -> Peripheral RX`); the peripheral calibrates and streams while the keepalive
arrives and idles when it stops (port closed / central reset). Both boards emit at a
fixed **50 Hz** (down from ~104 Hz) for generous link + USB timing margin; the peripheral
also self-heals a bad low-rate sensor start by re-initializing (no manual reset). LED:
central lit = collecting; peripheral fast blink = active, slow blink = idle, solid = sensor
stalled, 3 flashes = boot.

The shank board **streams** its packets while active; it is not polled. The central is
a passive listener on the data line: each loop it drains its UART buffer, **resyncs on the `0xAA`
header**, validates the checksum, and keeps the freshest complete packet — it never
blocks on the peripheral. A lost/extra byte fails one checksum and resyncs on the next
header, so a glitch costs one packet, never a lasting desync. The central timestamps
each packet on its own clock when parsed; if the freshest packet is older than
`SHANK_STALE_US` (30 ms) the shank is reported invalid (zeros) for the collector to
forward-fill. The line's last field is that packet's **age** in µs — small is
healthy, large (or a run of invalids) means the peripheral has stalled.

**Central → PC line (text, 18 fields):**
```
D,t_thigh_us,tw,tx,ty,tz,tax,tay,taz,t_shank_recv_us,sw,sx,sy,sz,sax,say,saz,age_us
```
When the freshest shank packet is stale/absent, `t_shank_recv_us` and the shank
fields are `0`; the collector marks such samples invalid (and short gaps are
forward-filled). `age_us` is the freshest shank packet's age (the CSV keeps the
`rtt_us` column name for continuity — same units, a link-health number).

---

## Findings / debugging log

The path to a stable signal turned up several non-obvious issues, all worth
recording:

1. **The Arduino library returns a left-handed sensor frame.**
   `Arduino_BMI270_BMM150` maps `x=-sensor.y, y=-sensor.x, z=sensor.z` for both
   accel and gyro — **determinant −1, a reflection**. A quaternion filter needs a
   right-handed frame (angular velocity is an axial vector), so the gyro's
   rotation sense was mirrored relative to the accelerometer and the filter
   wandered. **Fix:** negate `x` of accel *and* gyro to restore det +1.

2. **Startup gyro-bias calibration can cause runaway.** Averaging the gyro at
   boot to subtract bias is only valid if the board is still; if it's moving, the
   subtracted "bias" is a large false rate that can exceed the filter's
   correction gain and make the estimate diverge. **Fix:** discard the measured
   bias if any axis exceeds `BIAS_SANITY_DPS` (3 °/s), and keep the board still
   at startup.

3. **6-DOF has no shared heading.** Each board's yaw is unobservable and drifts
   independently, so the *relative-quaternion* angle drifts even on a rigid,
   stationary rig. The **gravity-referenced** method avoids this because
   gravity-in-board is yaw-invariant. (A rigid-ruler test — both boards on one
   ruler — was the key diagnostic: the true relative angle must be constant, so
   any drift is pure estimation error.)

4. **"Gravity from the quaternion" inherits a bad quaternion.** *While the filter
   was still wandering* (before finding #1 was fixed), both paths failed together,
   and reading gravity straight from the **raw accelerometer** was the decisive
   static fix. Once the handedness reflection was corrected the quaternion tracks
   properly, and the raw accelerometer's own weakness (finding #5) made it the
   worse default — see finding #7.

5. **Fast motion corrupts the accel-as-gravity assumption.** During any brisk
   movement the accelerometer reads gravity **plus linear acceleration**, so its
   direction is momentarily wrong. In the filter this is handled by **accel
   trust gating**; a bare-accelerometer angle has no such protection and reads
   erratically (apparent zeros/overshoot) right after swinging back from a high
   angle.

6. **Shank dropouts that grow over a session — traced in three steps.** The symptom
   was shank samples going invalid more and more as a session ran, with the link
   round-trip climbing alongside. Diagnosing it took three passes, each of which
   ruled something out:

   - **Framing desync.** The board-to-board reply was read as a fixed 30 bytes with
     no header resync, so one lost/extra byte on the async link shifted every
     following packet by one, and the misalignment was self-sustaining. **Fix:**
     resync on the `0xAA` header + validate the checksum, so a glitch costs one
     packet, not a run. Also returned `Serial1` from `460800` to **115200** for ~4×
     the per-byte timing margin (a 30-byte packet at 104 Hz needs only ~25 kbaud, so
     the lower rate costs nothing and drifts far less as the boards self-heat). An
     earlier pass had *raised* the rate to `460800` to shrink time-on-wire, which
     masked the desync but added the thermal fragility.
   - **Too-tight response window.** With framing fixed, `valid%` rose to ~93% but
     the remaining drops showed the request round-trip hitting the *full* timeout —
     the peripheral producing nothing in time, not corrupt bytes. Root cause: the peripheral
     only answered a poll between chunks of its own loop, and the mbed RTOS adds
     sporadic multi-ms stalls, so a reply could arrive after any fixed deadline.
   - **The real fix — stream instead of poll.** Widening the deadline is an arms
     race (and every stall stalls the central). Instead the peripheral now **streams** its
     packet continuously and the central reads the freshest one already in its UART
     buffer, never blocking. A peripheral stall no longer drops a sample — it just ages
     the newest packet; if that age exceeds `SHANK_STALE_US` (30 ms) the sample is
     marked invalid and the collector forward-fills it. This decouples the central's
     rate from the peripheral's latency entirely. Line field 18 is now the packet **age**
     (`age_us`); a large age or a run of invalids localizes a stalled/dead peripheral.

7. **Consolidated to one reliable method: gravity-in-board from the fused
   quaternion.** The project had accumulated four selectable behaviors
   (`--method gravity|quat` × `--gravity-source accel|quat`) that masked each
   other's failure modes. With the filter now correct (finding #1), the
   quaternion-sourced gravity direction is both **smooth through fast motion**
   (gyro carries) and **drift-free in tilt** (accel corrects) — strictly better
   than the raw-accel default, whose motion corruption (finding #5) produced the
   erratic post-flexion readings. The alternate modes and the yaw-prone
   relative-quaternion swing-twist method were removed. The hard accel gate was
   also replaced with a **soft gate** (`ACC_TRUST_FULL_G` 0.10 g →
   `ACC_TRUST_ZERO_G` 0.60 g): trust ramps down with `|accel|` instead of cutting
   off, so the filter is always partly corrected during motion and never runs
   open-loop and then snaps back on the way out.

Validation so far: on a rigid ruler, thigh and shank tilts agree; placing the
shank board at a right angle to the thigh board reads ~90°.

> Note: an earlier exploration added a full **magnetometer (9-DOF)** path with
> runtime PC-side calibration and an automatic BMM150→BMI270 axis solver. It was
> removed in favor of the simpler, metal-robust 6-DOF + gravity approach, but it
> lives in the branch history if absolute-heading stability is ever needed.

---

## Known limitations & next steps

- **Sagittal plane only** — see the trade-off above.
- **Validate against a protractor** — tape both boards across a hinge, zero
  straight, sweep the range, and check the reading at known angles.
- **Fast dynamic capture** — the gyro-fused gravity direction carries through
  motion; if very fast reps still lag or overshoot, tune the filter (`TWO_KP`)
  and the soft gate (`ACC_TRUST_FULL_G` / `ACC_TRUST_ZERO_G`) on both boards.
- **Per-session re-zero** handles any slow accelerometer/bias offset; hold the
  leg straight and still at the start of each capture.
- **Dropouts** — watch the run summary (`valid / filled / missing`); if the link
  is flaky over long wires, drop `Serial1` to 230400 on both boards.

---

## Files

```
central_imu/central_imu.ino   thigh board: 6-DOF filter, reads peripheral stream, streams to PC
peripheral_imu/peripheral_imu.ino     shank board: 6-DOF filter, streams packets over UART
knee_collector_uart.py      PC collector: calibration, angle math, CSV, --selftest
```
