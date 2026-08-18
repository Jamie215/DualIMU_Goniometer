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
   `thigh` / `shank` are filter-free raw-accel tilts (rock-steady when still);
   `rel` is the filter-based relative angle; `valid%` / `rtt` show link health.
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

## How it works (the math)

Each IMU estimates orientation with a **6-DOF Mahony filter** (accelerometer +
gyroscope, no magnetometer — robust to nearby metal). The knee angle is derived
from the **relative** geometry of the two segments, which is what makes it
placement-independent.

### Default method: gravity-referenced sagittal inclinometer

The reliable, drift-free quantity in 6-DOF is the **direction of gravity in each
board's own frame**. From it we build a signed segment inclination and take the
difference:

```
d_i    = gravity direction in board i at the straight-and-still zero pose
f_i    = each segment's "forward", learned from the calibration motion
incl_i = atan2(g_i · f_i, g_i · d_i)          # signed tilt in the sagittal plane
knee   = incl_thigh − incl_shank
```

This is immune to:
- **Position** on the limb (orientation sensors don't measure location);
- **Constant mounting rotation**, incl. **rotation of the board about the leg's
  long axis** (the angle is between two vectors in the same board frame, so any
  fixed mount rotation cancels);
- **6-DOF yaw drift** (gravity-in-board is invariant to heading about vertical).

Validated in `--selftest` against random mounts, a turning shared heading, and
independent per-board yaw drift (exact recovery).

**Trade-off:** it measures the **sagittal-plane** component of the angle — ideal
for upright knee flexion (standing ROM, gait, sit-to-stand). It under-reads
motion well out of the vertical plane (e.g. lying down with large hip rotation).

### Gravity source: raw accel vs. quaternion

`--gravity-source` chooses where the gravity direction comes from:
- **`accel` (default)** — the **raw accelerometer**, bypassing the filter
  entirely. Drift-free and filter-free; a stationary rig reads a flat line. Best
  for slow / quasi-static motion; noisier during fast limb acceleration.
- **`quat`** — gravity from the orientation quaternion (uses the filter). Smooth
  through fast motion because the gyro carries it, and still yaw-immune. Use for
  dynamic capture.

### Alternate method: `--method quat`

Relative-quaternion **swing-twist** about a flexion axis learned during the
sweep. General (any plane) but partly exposed to 6-DOF yaw drift; kept for
comparison.

---

## Commands

```
python knee_collector_uart.py --port PORT [options]

  --monitor                 live bring-up check (raw-accel tilts vs filter angle)
  --raw                     dump raw serial lines with field counts, then exit
  --method gravity|quat     angle method (default: gravity)
  --gravity-source accel|quat   gravity source for the gravity method (default: accel)
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

4. **"Gravity from the quaternion" inherits a bad quaternion.** When the filter
   was wandering, *both* angle methods failed together. Reading gravity straight
   from the **raw accelerometer** (`--gravity-source accel`) bypasses the filter
   and gives a rock-steady static reading — the decisive fix.

5. **Fast motion corrupts the accel-as-gravity assumption.** Added
   **accel-magnitude gating**: the filter only applies the accel correction when
   `|accel|` is within `ACC_GATE_G` (0.15 g) of gravity, letting the gyro carry
   through quick movement (`--gravity-source quat`).

6. **UART timeouts.** The 30-byte reply is ~0.65 ms at 460800 baud (was ~2.7 ms
   at 115200, which crowded the 8 ms budget and caused dropped "invalid" shank
   samples). Raising `Serial1` to **460800** on both boards restored margin.

Validation so far: on a rigid ruler, thigh and shank tilts agree; placing the
shank board at a right angle to the thigh board reads ~90°.

> Note: an earlier exploration added a full **magnetometer (9-DOF)** path with
> runtime PC-side calibration and an automatic BMM150→BMI270 axis solver. It was
> removed in favor of the simpler, metal-robust 6-DOF + gravity approach, but it
> lives in the branch history if absolute-heading stability is ever needed.

---

## Known limitations & next steps

- **Sagittal plane only** (gravity method) — see the trade-off above.
- **Validate against a protractor** — tape both boards across a hinge, zero
  straight, sweep the range, and check the reading at known angles.
- **Slow residual yaw drift** remains in the quaternion path (bias instability);
  the gravity method is unaffected. Per-session re-zero handles it.
- **Fast dynamic capture** — compare `--gravity-source accel` vs `quat`; if `quat`
  needs it, tune `TWO_KP` / `ACC_GATE_G`.
- **Dropouts** — watch the run summary (`valid / filled / missing`); if the link
  is flaky over long wires, drop `Serial1` to 230400 on both boards.

---

## Files

```
master_imu/master_imu.ino   thigh board: 6-DOF filter, polls slave, streams to PC
slave_imu/slave_imu.ino     shank board: 6-DOF filter, answers 'R' over UART
knee_collector_uart.py      PC collector: calibration, angle math, CSV, --selftest
```
