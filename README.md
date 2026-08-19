# Dual-IMU Knee Goniometer

Two Arduino Nano 33 BLE Rev2 boards (one on the thigh, one on the shank) measure
knee flexion angle. Each board runs a 6-DOF orientation filter; the **shank
board (slave)** answers the **thigh board (master)** over a wired UART link, and
the master streams both segments' data to a PC, where `knee_collector_uart.py`
computes and logs the knee angle.

The design goal is a **placement-independent** angle: you should not have to mount
the boards in a precise, repeatable orientation.

---

## Hardware & wiring

- 2× **Arduino Nano 33 BLE Rev2** (onboard **BMI270** accel+gyro; BMM150
  magnetometer is present but **unused**).
- UART between the boards (`Serial1`, **460800 baud**):

  ```
  Master TX (D1) ->  Slave RX (D0)
  Master RX (D0) <-  Slave TX (D1)
  Master GND    <->  Slave GND
  ```
- Master connects to the PC over USB.
- Mount convention used in testing: board long axis along the limb long axis,
  USB port toward the hip, one board above and one below the knee. Exact rotation
  / position does **not** need to match between boards (see "How it works").

---

## Quick start

1. **Flash** `master_imu/master_imu.ino` to the thigh board and
   `slave_imu/slave_imu.ino` to the shank board (Arduino IDE, board = "Arduino
   Nano 33 BLE", library **Arduino_BMI270_BMM150**). Confirm each reports
   *Done uploading*.
2. On power-up / reset, **keep both boards still for ~4 s** — they measure gyro
   bias then. The master prints `# master gyro bias dps: ...`.
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
   - Hold the leg **straight and still** (~2 s) to zero.
   - Do a few slow reps that bend **both knee and hip** (sit-to-stands /
     marching) so each board tilts enough to learn its "forward" direction.
   - The signed knee angle is printed live and logged to `knee_log.csv`.

`--selftest` (no hardware) runs the full math test suite.

---

## Live GUI (`knee_gui.py`)

For a testing / proof-of-concept session there is a Tkinter + Matplotlib front
end over the same angle math. It is the easiest way to run a collection:

```
pip install -r requirements.txt
python knee_gui.py                 # scan for the master port and go
python knee_gui.py --simulate      # no hardware: synthetic flexing-knee source
python knee_gui.py --selftest      # headless: sample-gate + source logic
```

What it adds over the CLI:

- **Port scan** — probes each serial port for valid `D` lines and picks the
  master automatically. (Only the master is on USB; a dead slave link shows up
  as a low shank-valid %, not a second port.)
- **Consistent 100 Hz** — the jittery ~104 Hz device stream is resampled onto a
  fixed 10 ms grid, so both the CSV and the plots are a clean 100 Hz record.
- **Obvious calibration** — a colour-coded banner drives the phases with a live
  countdown: amber **ZEROING** (hold straight & still) → amber **SWEEP** (bend
  knee + hip, with a live shank-tilt readout) → green **RUNNING**.
- **Plots** — knee angle (primary), the two segment inclinations it is built
  from, and the link RTT. While collecting they show a rolling window; when you
  **Stop collecting** they auto-fit the whole session and a pan/zoom/save
  toolbar becomes usable so you can inspect the frozen trace (Home returns to
  the full-session view).
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
  dead slave link), using the same diagnosis as the CLI.
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

**2. Zero reference `d_i` (static-hold calibration).** With the leg held straight
and still, average `g_i` over the hold window. `d_i` is the segment's long axis
as seen in the board frame — the direction gravity points at 0°:

```
d_i = normalize( Σ g_i(t)  over the still hold )         # code: average_gravity()
```

**3. Forward direction `f_i` (functional-sweep calibration).** During the slow
reps, the part of `g_i` **perpendicular to `d_i`** is the direction the segment
tilts toward as it flexes. Remove the `d_i` component, sign-align the samples (the
limb sweeps consistently one way from extension), and sum:

```
g⊥ = g_i − (g_i · d_i) d_i                                # in-plane component
f_i = normalize( Σ sign(g⊥ · ref) · g⊥ )                  # code: estimate_forward()
```

Samples that tilt less than `SWEEP_MIN_ANGLE_DEG` are ignored so IMU noise around
the zero pose can't define the direction. `d_i` and `f_i` are orthonormal by
construction and together span that segment's **sagittal plane** in its own frame.
If a segment barely moved, `f_i` is undefined and that segment contributes 0.

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
independent per-board yaw drift (exact recovery), and a full flex-to-130°-and-back
sweep (the signed angle retraces identically — no stuck zeros on return).

**Trade-off:** gravity gives only **2 of the 3 rotational DOF** — it is blind to
rotation about the vertical (gravity) axis. So this method measures the
**sagittal-plane** component of the angle — ideal for upright knee flexion
(standing ROM, gait, sit-to-stand). It under-reads motion well out of the vertical
plane (e.g. lying down with large hip rotation). Note this is an *observability*
limit, not a mounting one: a fixed mount tilt cancels for planar motion (step 5),
but tilt combined with out-of-sagittal-plane motion steers part of the true joint
rotation into the unobservable yaw direction, where gravity cannot see it — the
one case a gravity-only method fundamentally cannot recover.

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
  --cal-seconds N           straight-and-still zero hold (default 2)
  --sweep-seconds N         calibration-motion window (default 6)
  --out FILE                CSV path (default knee_log.csv)
  --selftest                run the math self-tests (no hardware)
```

CSV columns: `t_thigh_us, thigh_qw..qz, shank_qw..qz, knee_angle_deg, status,
rtt_us` (status ∈ `zeroing / sweep / valid / filled / missing`).

---

## Data format / wire protocol

**Slave → master reply (binary, 30 bytes):**
```
[0]      0xAA header
[1..16]  float q0..q3   (little-endian, w,x,y,z)
[17..28] float ax,ay,az (little-endian, g, handedness-corrected)
[29]     XOR checksum of bytes [1..28]
```
Master requests with a single `'R'` byte and reads exactly 30 bytes within
`SLAVE_TIMEOUT_US` (8 ms); the slave sample is bracketed at `(t_req+t_resp)/2`.

**Master → PC line (text, 18 fields):**
```
D,t_thigh_us,tw,tx,ty,tz,tax,tay,taz,t_shank_mid_us,sw,sx,sy,sz,sax,say,saz,rtt_us
```
On a bad/missing slave reply, the shank fields and midpoint are `0`; the collector
marks such samples invalid (and short gaps are forward-filled).

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

6. **UART timeouts.** The 30-byte reply is ~0.65 ms at 460800 baud (was ~2.7 ms
   at 115200, which crowded the 8 ms budget and caused dropped "invalid" shank
   samples). Raising `Serial1` to **460800** on both boards restored margin.

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
master_imu/master_imu.ino   thigh board: 6-DOF filter, polls slave, streams to PC
slave_imu/slave_imu.ino     shank board: 6-DOF filter, answers 'R' over UART
knee_collector_uart.py      PC collector: calibration, angle math, CSV, --selftest
```
