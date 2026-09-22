# Areas to Explore

A running log of research directions for the dual-IMU knee goniometer. Each area
records the motivation, what it would enable and how, the reasoning/derivation
behind it, the design trade-offs, and a status/recommendation. Add new areas as
sections below; keep the same structure so entries stay comparable.

> Companion to `README.md` ("How it works", "Findings / debugging log",
> "Known limitations & next steps"). Where this doc references code, function
> names are in `knee_collector_uart.py` unless noted.

---

## Area 1 — Magnetometer fusion: accuracy, and absolute vs. relative angle

**Status:** *Investigated (analysis only). Recommendation: do not add for the
current sagittal use case; pursue only if the requirement shifts to 3-D /
out-of-plane kinematics. For an absolute (anatomical) angle, pursue functional
calibration instead — see Area 2.*

### 1.1 Motivation / the proposal

The boards (Arduino Nano 33 BLE Rev2) carry a BMM150 magnetometer that is
currently **unused**. A proposal was raised: use the magnetometer to (a) improve
accuracy, and/or (b) recover an **absolute** joint angle rather than the current
**relative** one. The intuition offered was: "the magnetometer gives the vector
to north, so we can use it as a reference to compute the absolute angle between
the segments."

This section captures the analysis of whether that holds.

### 1.2 Current state (baseline this is measured against)

- Fusion is **6-DOF** (accel + gyro), per-board Mahony filter. No magnetometer —
  chosen deliberately for robustness to nearby metal.
- Angle method: **gravity-referenced sagittal inclinometer**. Gravity-in-board
  `g_i` is read from the fused quaternion (`gravity_in_board()`), a static hold
  defines the per-board zero axis `d_i` (`average_gravity()`), a functional sweep
  defines forward `f_i` (`estimate_forward()`), and the signed inclination
  `incl_i = atan2(g_i·f_i, g_i·d_i)` (`sagittal_inclination()`) is differenced
  across segments (`gravity_knee_angle()`).
- The result is **relative to the pose held at calibration** and **sagittal-plane
  only** (gravity gives 2 of 3 rotational DOF; it is blind to rotation about
  vertical).
- History: a full **9-DOF magnetometer path** existed and was removed
  (`146cc14` added it; `0dd3a93` stripped it back). Per the strip-back commit it
  was cut to reach a clean, verifiable baseline — *"nothing about the mag
  machinery was ever confirmed on hardware"* — **not** because mag fusion was
  measured and found worse. A `mag_calibrate.ino` with hard/soft-iron calibration
  and an automatic BMM150→BMI270 axis solver lives in that history (`87096d9`,
  `d79e51a`, `9cf21cb`).

### 1.3 What a magnetometer actually measures

- It reports the **local magnetic field as a 3-D vector in the sensor's own
  frame** — the magnetic analogue of how the accelerometer reports gravity in its
  own frame. **Rotating the chip changes the reading**; that change *is* the
  orientation information. (It is *not* orientation-invariant — a common
  misconception.)
- The field points to **magnetic** north (declination offset from true north) and
  **dips into the ground** (magnetic inclination, ~60–70° below horizontal at mid
  latitudes). It is not a clean horizontal "north arrow"; a compass hides this by
  using only the horizontal component.
- The **angle between the magnetometer and gravity vectors** is orientation-
  invariant, but it only equals `90° − dip` — a constant of your **location**, not
  of the leg. So "angle between mag and gravity" carries no joint information.

### 1.4 The key framing: two jobs — a *scale* and a *zero*

Measuring any angle needs two independent things:

1. **The scale** — how far apart two directions are (the geometry / "ruler").
2. **The zero** — which configuration counts as 0° (the "datum" / baseline).

Board-mounted inertial/magnetic sensors are good at the **scale**. The **zero**
is a separate problem, and it is an *anatomy* problem, not a sensor problem. This
split is the crux of the whole magnetometer question.

### 1.5 What the magnetometer enables, and how (the scale)

- **Resolves heading (yaw).** Gravity fixes tilt (2 DOF) but leaves rotation about
  vertical unobservable and drifting independently per board (README finding #3).
  The magnetometer's horizontal component pins that last axis.
- With gravity **and** magnetometer, each board's **full 3-D orientation in the
  Earth frame** is determined (two non-parallel reference vectors → full rotation;
  TRIAD / Wahba's problem). Call these `R_thigh`, `R_shank` (board→world).
- The **relative orientation** between the boards then follows, absolutely and
  drift-free, all 3 DOF:
  `R_rel = R_thigh⁻¹ · R_shank`.
- **Net enablement:** the magnetometer's genuine, *unique* contribution is the
  **out-of-sagittal-plane / full-3-D** component of the angle. For the in-plane
  (sagittal) angle, gravity alone already suffices, because both boards share the
  same "down"; the magnetometer adds nothing there.

### 1.6 Absolute vs. relative angle — why the magnetometer does not close it

The joint angle is between the **bones**, not the **boards**. Each board sits on
skin at an unknown mounting rotation `M_i` (board→bone). The bone-to-bone angle is:

```
joint = M_thigh⁻¹ · R_rel · M_shank
```

The magnetometer nails `R_rel`. It contributes **nothing** to `M_thigh`,
`M_shank`, which are unknown and unobservable from accel+gyro+mag alone.

- **Two meanings of "absolute" that get conflated:**
  1. *Absolute heading* (Earth-referenced yaw) — the magnetometer provides this.
  2. *Absolute anatomical/clinical zero* (true femur–tibia angle, e.g. flexion
     contracture) — the magnetometer does **not** provide this.
- **Contracture ambiguity (the decisive case):** two participants can hold their
  segments in identical orientations — identical accel *and* magnetometer readings
  — while one is a healthy leg posed at 10° and the other is a 10° contracture at
  best-effort straight. Same data, opposite clinical truth ⇒ **no function of the
  sensor data can recover the contracture.** The deciding information (where each
  person's true straight sits) is anatomical and never enters the sensors.
- **"Straight" is not a direction in the room.** A joint angle is invariant to how
  the person is oriented; the magnetometer's only added information *is* room
  orientation — the one thing a joint angle does not depend on. So it is
  structurally incapable of supplying the zero.

**Why the current method is relative (mechanism):** the "relative" character comes
from the **static zero-hold**, not from any quaternion algebra. Each inclination is
measured from `d_i` (gravity direction during the held pose), so the held pose
becomes 0° and the unknown mounting is absorbed into `d_i` and cancels. The
between-segment **difference** additionally cancels a shared heading. (Note the
conjugate in `gravity_in_board()` rotates world-up *into each board's frame* — a
per-board operation to extract gravity-in-board — it does not cancel one board
against the other. The pure relative-quaternion method `q_thigh* ⊗ q_shank` was
removed for yaw drift.)

### 1.7 Design choices / trade-offs

- **The fundamental trade:** you either **pin** the board→bone mounting (via
  calibration → absolute zero, but placement precision now matters) or **cancel**
  it (re-zero each session → relative zero, robust to placement). The current
  design deliberately *cancels* — that is exactly what makes it
  placement-independent. The magnetometer sits on **neither** branch of this
  trade; it never touches the zero.
- **Magnetic-disturbance fragility (the main cost of adding it):** the BMM150 is
  corrupted by hard-iron/soft-iron effects and any nearby ferromagnetic material
  or current — table/chair legs, treadmills, wheelchairs, rebar floors, the LiPo,
  the USB cable, the other board. Indoor clinical/gym settings are magnetically
  dirty. Adding the magnetometer re-introduces precisely the fragility 6-DOF was
  chosen to avoid.
- **Calibration burden:** per-board hard/soft-iron calibration (figure-8), with
  recalibration as conditions drift.
- **Accuracy verdict for the target use (sagittal ROM, gait, sit-to-stand):**
  expected gain ≈ **zero**, because the sagittal reading is already yaw-invariant
  and drift-free in the DOF that matter.

### 1.8 Concrete next experiments (if this area is revived)

1. **Field-quality gate (cheapest, do first):** log raw `mx,my,mz` for a normal
   session *in the actual test environment* and plot field magnitude
   `√(mx²+my²+mz²)`. Roughly constant (~25–65 µT) as the limb sweeps ⇒ environment
   clean enough for fusion; large swings ⇒ locally distorted, fusion would inject
   error. This decides the question empirically for the real setting.
2. **A/B in 3-D:** revive the old 9-DOF branch behind a flag and compare against
   6-DOF specifically on out-of-plane motions, not sagittal flexion.
3. Only pursue integration if (1) passes and a 3-D requirement actually exists.

### 1.9 Recommendation

- **Sagittal knee flexion (current scope):** do **not** add the magnetometer — no
  accuracy gain, and it undoes metal-robustness.
- **True 3-D kinematics (out-of-plane):** the magnetometer is the principled way
  to get the 3rd DOF; revive the old branch, but run the field-quality gate first
  and budget for calibration.
- **Absolute anatomical angle (contracture, cross-session comparability):** not a
  magnetometer problem — see Area 2.

---

## Area 2 — Absolute (anatomical) angle via functional / anatomical calibration

**Status:** *Identified as the correct path for an absolute angle. Not yet
prototyped.*

### 2.1 Motivation

The current angle is relative to the calibration pose, so it cannot report an
absolute clinical angle (e.g. a flexion contracture: "best-effort straight" is
really 10° of flexion) or be compared across sessions zeroed on different poses.
Per Area 1, this gap is anatomical and cannot be closed by the magnetometer; it
requires a reference that **observes the anatomy**.

### 2.2 Candidate approaches

- **Functional joint-axis calibration:** have the subject perform prescribed pure
  knee-flexion reps. The flexion axis appears as the dominant, consistent rotation
  axis in each board's frame (observable from the **gyro**), which pins the
  board→bone relationship `M_i`. Needs **no magnetometer**. This is how
  research-grade IMU systems claim anatomical angles.
- **Known-pose / external reference:** physically define true full extension once
  (straight-leg fixture, or a manual goniometer / imaging reading) and anchor the
  anatomical zero to it.

### 2.3 Trade-offs

- Buys an absolute, cross-session-comparable, contracture-capable angle.
- Costs placement/calibration discipline (the "pin the mounting" branch of the
  Area 1 trade) and a defined, repeatable calibration protocol per subject.
- Sensitivity to how faithfully the calibration motion isolates the flexion axis;
  needs validation against a manual goniometer / gold standard.

### 2.4 Next steps

- Prototype a functional-axis estimator from a flexion-only capture and compare
  its inferred zero against a manually measured angle at known poses.
- Decide whether the clinical use actually needs absolute angles, or whether the
  difference-based measures (ROM, rep counting, gait, movement quality) that the
  relative angle already serves are sufficient.

---

## How to add an area

Copy the Area template: **Status → Motivation → What it enables & how →
Reasoning/derivation → Design trade-offs → Next experiments → Recommendation.**
Keep claims tied to code (function names, commit hashes) and to `README.md`
sections where relevant.
