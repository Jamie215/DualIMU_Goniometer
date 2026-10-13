---
title: "Dual-IMU Knee Goniometer — Project Handover"
subtitle: "State at transition, design decisions and their rationale, and directions to explore"
---

*Drafted in October 2026, prepared by Jamie Kim*

# About this document

This is the handover record for the dual-IMU knee goniometer proof of concept
(August–October 2026). It is composed of the following:

- **Handover summary (§1):** the state of the project at transition, open
  issues, and what to do next. Read this first.
- **Design decisions (§2–§11):** what was decided, why, and which alternatives
  were considered.
- **Areas to explore (Appendices A–D):** longer analyses of directions that were
  weighed but not pursued yet.

Companion documents:

- `docs/SOP_Build_and_Use.docx` (`.md` mirror) — how to build and operate the
  prototype and the interface.
- `README.md` — the technical reference (math, wire protocol, commands).
- `analysis/static_test/Static_Stability_Test_Report.docx` — the Preliminary
  Stationary Test Report, with `FINDINGS.md` as its detailed working record.

# 1. State at transition (October 2026)

## 1.1 Current state

- **Working bench prototype.** Two Arduino Nano 33 BLE Rev2 boards (thigh =
  central, shank = peripheral) in 3D printed cases (`DualIMU_Goniometer.stl`),
  joined by soldered UART wires, each powered over USB; the central streams to a
  PC at 50 Hz.
- **Interface built for testing.** Live Graphical User Interface (GUI) with
  guided calibration and auto-saved sessions; CLI collector; self-tests pass.
  The GUI is released as a Windows program, `KneeGoniometerGUI.exe`
  ([Link](https://github.com/Jamie215/DualIMU_Goniometer/releases)).
- **Preliminary stationary testing done.** ≈0.01–0.02° noise, ≈ −0.001 °/min
  drift on healthy hardware; 45°/90° rigs read 45.0°/89.0° (preliminary; §9.1).
- **Standard Operating Procedure (SOP) developed**
  (`docs/SOP_Build_and_Use.docx`): build, setup, sessions and troubleshooting
  for operators; Python setup and releasing a new GUI version are in its
  appendices.

## 1.2 Open issues

1. **Faulty board.** One board's BMI270 fails after 20–40 min (§9.1). It must be
   identified and replaced. It should be the board without the red wire soldered
   to the 3.3 V pin; confirm before replacing it.
2. **Raw test data not preserved.** The stationary-test CSVs and diagnostic logs
   were not archived (`*.csv` / `*.log` are git-ignored intentionally during
   development phase). Plan data storage once testing moves beyond the
   preliminary stage.
3. **Analysis scripts are working scripts.** `analysis/static_test/stats.py`,
   `plot.py` and `calibration.py` were written ad hoc for this dataset (they
   read every CSV in the current folder). `plot_report.py`, `plot_session2.py`
   and `plot_angle_rig.py` produced the report figures and expect specific file
   names.
4. **Peripheral-only fault guard.** The sensor-fault guard runs on the
   peripheral only; the central has no equivalent (§7.2).
5. **Angle log resolution.** Angles are written to 0.01°, which limits how low
   a noise floor can be measured.

## 1.3 Recommended next steps

1. Replace the faulty board; repeat a one-hour stationary run on two healthy
   boards for a clean baseline.
2. Sweep **both** nodes during calibration so the angle uses both sensors (§5.3)
3. Try the strap placement in the SOP on a person and refine it if the boards
   shift during movement.
4. Controlled accuracy test: repeated 0°/45°/90° placements on a precise rig or
   against a goniometer.
5. Dynamic range-of-motion testing (tune `TWO_KP` and the soft accel gate, §4.4,
   if fast reps lag or overshoot).
6. Consider the longer-term directions in §10 and Appendices A–D (battery +
   BLE for an untethered device; absolute angle via functional calibration).

## 1.4 Documents & files reference

| Document | Use it for |
|---|---|
| `docs/SOP_Build_and_Use.docx` (master; `.md` mirror) | Building the device and running a session |
| `docs/Project_Handover.md` (+ `.docx`) | This document: state, design rationale, future directions |
| `analysis/static_test/Static_Stability_Test_Report.docx` | Preliminary Stationary Test Report |
| `analysis/static_test/FINDINGS.md` | Detailed working record behind the report |
| `README.md` | Technical reference: math, wire protocol, commands |

| File / folder | Contents |
|---|---|
| `central_imu/central_imu.ino` | Thigh board firmware |
| `peripheral_imu/peripheral_imu.ino` | Shank board firmware |
| `knee_collector_uart.py` | Angle math, calibration, CLI collector, `--selftest` |
| `knee_gui.py` | Live GUI (uses `knee_collector_uart.py`) |
| `requirements.txt` | Python dependencies |
| `DualIMU_Goniometer.stl` | 3D printable board case (base + lid) |
| `analysis/static_test/` | Stationary test scripts, figures, report |
| `analysis/diagnostics/parse_diag.py` | Diagnostic log → tables |
| `docs/` | SOP, this document, and its figures (`docs/images/handover/`) |
| `docs/build_docs.sh` | Regenerates this document's `.docx` from its Markdown |
| `.github/workflows/release.yml` | Builds `KneeGoniometerGUI.exe` on a `v*` tag |

# 2. Problem and goals

The goal is a low-cost wearable that measures **knee flexion angle** from two
inertial sensors, one above and one below the knee.

| Goal | Explanation |
|---|---|
| Placement independence | The boards should not need to be mounted at a precise or repeatable orientation on the limb. |
| Robust signal | The collected signals should be stable at rest, smooth through motion. There should be no drift in the angle. |
| Simple design | Prefer the smallest system as a proof of concept that can be easily built and tested |
| Usable in testing sessions | A GUI that drives calibration, shows live data, and saves every session automatically. |

Scope: **sagittal-plane** knee flexion (standing ROM, gait, sit-to-stand),
relative to a calibration pose. Absolute anatomical angles and 3-D kinematics
are out of scope (see §5.5 and §10).

# 3. System architecture

Figure 1 shows the prototype on the leg and Figure 2 the path a sample takes
from the sensors to the CSV file.

![Figure 1. The prototype on the leg (side view). Each board sits in a printed case, held by an elastic strap with its USB port toward the hip.](images/handover/fig1_setup.png)

![Figure 2. Data flow for one sample, and the shank packet sent over the wire.](images/handover/fig2_dataflow.png)

## 3.1 Two-board central/peripheral topology

- **Decision:** The thigh board (*central*) runs its own filter, receives the
  shank board's (*peripheral*) orientation over a wire, merges both and sends
  one line per sample to the PC. The PC computes the angle.
- **Why:** Only one board needs a PC connection; both segments arrive already
  time-aligned on the central's clock; the angle math stays in Python, where it
  is easy to test and change without reflashing.
- **Alternatives:** Each board on its own USB port (needs PC-side time alignment
  of two streams); computing the angle on the board (harder to test and iterate).

## 3.2 Hardware

- **Decision:** Arduino Nano 33 BLE Rev2, using the onboard **BMI270**
  accelerometer + gyroscope. The onboard **BMM150 magnetometer is not used**
  (§4.1).
- **Why:** Off-the-shelf, inexpensive, IMU on board, Arduino tooling, and a BLE
  radio for a future wireless version.

## 3.3 Physical build

- **Decision:** Printed cases and elastic straps; the boards are joined by three
  soldered wires (two UART lines + ground). The central is powered and read over
  one USB cable to the PC; the peripheral has its own USB power.
- **Why:** Fastest route to a working, reliable bench prototype.
- **Trade-offs:** The subject is tethered to the PC and to power, and a wire runs
  across the knee. Battery power and wireless links are future work (Appendix C).

# 4. Sensor fusion (on each board)

## 4.1 6-DOF Mahony filter, no magnetometer

- **Decision:** Each board estimates its orientation with a 6-DOF Mahony filter
  (accelerometer + gyroscope), `TWO_KP = 1.0`, `TWO_KI = 0.04`.
- **Why:** The angle method (§5) only needs the direction of gravity in each
  board's frame, which 6-DOF provides without drift. A magnetometer is disturbed
  by nearby metal and needs per-board calibration.
- **Alternatives:** A 9-DOF path with magnetometer calibration was built and
  then removed to get back to a clean, verifiable baseline; it was never
  confirmed on hardware. Appendix A concludes it adds nothing for sagittal
  angles. The code is recoverable from history if 3-D is ever needed.

## 4.2 Fixing the library's left-handed axes

- **Decision:** Negate the x axis of both accelerometer and gyroscope after
  reading them. **Any new firmware must keep this.**
- **Why:** `Arduino_BMI270_BMM150` returns `x=-sensor.y, y=-sensor.x, z=sensor.z`,
  a mirrored (left-handed) frame. A quaternion filter needs a right-handed frame;
  without the fix the filter wandered.

## 4.3 Gyro bias calibration with a sanity gate

- **Decision:** When a collection starts, average ~300 gyro samples (up to 4 s)
  and subtract the result as bias, but **discard it** if any axis exceeds 3 °/s.
- **Why:** Bias is the main drift source in a 6-DOF filter. If the board moves
  during calibration, the "bias" is a false rate that can make the filter
  diverge, so zero is safer.

## 4.4 Soft accelerometer-trust gate

- **Decision:** Weight the accelerometer correction by how close `|accel|` is to
  1 g: full trust within 0.10 g, ramping to none at 0.60 g.
- **Why:** During brisk motion the accelerometer reads gravity *plus* linear
  acceleration. A hard on/off gate let the filter run gyro-only through a fast
  move, drift, then snap back. A soft ramp always applies some correction.
- **Status:** Not yet tuned on dynamic data.

# 5. Knee-angle method (PC side)

## 5.1 Gravity-referenced sagittal inclinometer

![Figure 3. How the knee angle is computed. Each segment's inclination is its tilt from the zero pose, measured from gravity in its own board; the knee angle is their difference.](images/handover/fig3_angle.png)

- **Decision:** For each board, take the direction of gravity in the board's own
  frame (from the fused quaternion), express it as a signed inclination in that
  segment's sagittal plane, and define the knee angle as
  `incl_thigh − incl_shank` (Figure 3).
- **Why:** Gravity-in-board is drift-free in tilt and **unaffected by heading**,
  so the 6-DOF yaw drift never enters. Any constant mounting rotation cancels,
  which gives placement independence.
- **Alternatives:** (a) Relative quaternion between the boards — rejected because
  each board's heading drifts independently in 6-DOF, so the angle drifted even
  with both boards on a rigid ruler. (b) Four selectable angle modes —
  consolidated to one so that modes could not mask each other's failures.

## 5.2 Gravity from the fused quaternion, not the raw accelerometer

- **Decision:** Read gravity from the filter's quaternion; keep the raw
  accelerometer only as a filter-free cross-check (still streamed and logged).
- **Why:** The raw accelerometer is corrupted during motion; the fused quaternion
  lets the gyro carry fast movement while the accelerometer keeps tilt drift-free.

## 5.3 Two-step calibration: static zero + functional sweep

![Figure 4. A session in the GUI, from connecting to stopping.](images/handover/fig4_session.png)

- **Decision:** Each session starts with (Figure 4):
  1. **Zeroing** (2 s): leg straight and still; the average gravity direction
     becomes each segment's 0° axis.
  2. **Sweep** (6 s): a few slow movements bending knee and hip; the direction
     of tilt becomes each segment's forward axis.
- **Why:** These two axes define each segment's sagittal plane in its own board
  frame, without knowing how the board is mounted.
- **Detail:** A segment that does not tilt at least 5° during the sweep
  contributes 0 (treated as fixed). In all bench tests only the shank was swept,
  so the angle came from the shank alone; both-node sweeps are a required next
  step (§1.3).

## 5.4 Signed, continuous angle

- **Decision:** `incl = atan2(g·f, g·d)`: flexion positive, hyperextension
  negative, continuous through the full range with no ±90° wrap.
- **Why:** Distinguishes flexion from hyperextension and retraces identically on
  the way back from high flexion (checked in `--selftest` with a 0→130°→0 sweep).

## 5.5 Relative zero, not an absolute anatomical angle

- **Decision:** Accept that 0° is "the pose held during zeroing".
- **Why:** ROM, rep counting, gait and movement quality are difference-based, so
  the zero cancels. An absolute angle (e.g. a flexion contracture) needs an
  external reference that observes the anatomy; the sensors cannot supply it,
  and a magnetometer does not help (Appendix A).
- **Alternatives:** Functional joint-axis calibration or a known-pose fixture
  (Appendix B) — identified, not prototyped.

## 5.6 Accepted limitation: sagittal plane only

Gravity gives 2 of 3 rotational degrees of freedom; rotation about vertical is
unobservable. The method therefore under-reads motion well out of the vertical
plane (e.g. lying down with large hip rotation). This is accepted for upright
knee flexion.

# 6. Board-to-board link and data protocol

## 6.1 Wired UART at 115 200 baud

- **Decision:** UART at 115 200 baud, wired both ways: shank data to the
  central, keepalive to the peripheral (Figure 2).
- **Why:** Simple and deterministic. A faster rate (460 800) was tried; it hid a
  framing bug and was more sensitive to clock drift as the boards warmed up.
  115 200 is far above the ~15 kbit/s needed.
- **Alternatives:** BLE (Appendix C) — deferred.

## 6.2 Framed binary packet with resync and checksum

- **Decision:** 30-byte packet: header byte, quaternion, raw accelerometer and an
  XOR checksum (Figure 2). The central resyncs on the header and drops packets
  that fail the checksum.
- **Why:** The original fixed-length read had no resync, so one lost byte shifted
  every following packet. Now a glitch costs one packet.

## 6.3 Peripheral streams; central never waits

- **Decision:** The peripheral **streams** packets at 50 Hz; the central keeps
  the newest valid one. If it is older than 30 ms the shank is reported invalid
  and the PC forward-fills it (§8.3).
- **Why:** Polling the peripheral lost samples whenever it stalled for a few ms,
  and longer waits only stalled the central. Streaming decouples the two boards.

## 6.4 Collect-on-demand

- **Decision:** Both boards run their IMUs only while the central's USB port is
  open. On open, the central calibrates its gyro and sends keepalives every
  200 ms; the peripheral goes idle if none arrives for 1 s.
- **Why:** Nothing runs unobserved, and each session starts with a fresh gyro
  calibration on a still board (Figure 4).

## 6.5 Fixed 50 Hz output, one write per line

- **Decision:** Both boards send at a fixed 50 Hz; the central builds each PC
  line in memory and sends it with **one** write. The filters still run at the
  sensor's native rate.
- **Why:** 50 Hz is ample for knee ROM and leaves timing margin. Printing each
  line piece by piece took ~11.7 ms and held the real rate to ~43.5 Hz; one write
  takes ~1–1.8 ms and restored a true 50 Hz.

## 6.6 Diagnostics channel

- **Decision:** Both boards report a health summary once a second (on by
  default). The GUI saves these lines to `knee_diag_<timestamp>.log`, and
  `analysis/diagnostics/parse_diag.py` turns them into tables.
- **Why:** Needed to find the sample-rate shortfall and the faulty sensor.
- **Constraint:** The diagnostic frame layout must match in both sketches; flash
  both boards from the same commit.

# 7. Robustness and fault handling

## 7.1 Peripheral self-healing

- **Low-rate start:** if fewer than 30 samples arrive in 1 s, re-initialise the
  sensor (the BMI270 occasionally starts at a wrong ~10 Hz rate).
- **Stalled sensor:** if no sample arrives for 250 ms, the LED goes solid and
  the sensor is re-initialised every 500 ms.
- **IMU failure at boot:** fast-blink for ~3 s, then reboot.

## 7.2 Sensor-fault guard

- **Decision:** On the peripheral, reject a sample if the read took over 50 ms,
  `|accel|` is below 0.25 g or above 4 g, or the gyro reads over 1000 °/s while
  the board is otherwise near 1 g. Rejected samples are sent as a gap, not a
  wrong angle. If no good sample arrives for 5 s, the board reboots.
- **Why:** In endurance runs one board's BMI270 began returning garbage after
  20–38 min and then stopped; the guard turns wrong angles into visible gaps.

## 7.3 LED indicators

| Board | Pattern | Meaning |
|---|---|---|
| Central | On | Collecting (USB port open) |
| Central | Off | Idle |
| Peripheral | 3 quick flashes | Boot |
| Peripheral | Slow blink | Idle, waiting for keepalive |
| Peripheral | Fast blink (~3 Hz) | Active, streaming |
| Peripheral | Solid on | Sensor stalled (re-init firing) |
| Peripheral | Off / frozen | Unpowered or hard fault |

# 8. PC software

## 8.1 CLI collector plus a live GUI

- **Decision:** `knee_collector_uart.py` holds the math, calibration, CSV logic
  and `--selftest`; `knee_gui.py` reuses it and adds port scanning, a
  colour-coded calibration banner, live plots, and auto-saved sessions.
- **Why:** One tested implementation of the math; a GUI makes test sessions
  repeatable for non-developers.

## 8.2 Sessions and CSV rows

- **Decision:** Each "Calibrate & collect" starts a new session with a fresh
  calibration and a new timestamped CSV; "Stop collecting" freezes the plot on
  the whole session (Figure 4). Exactly one row is written per line received from
  the central, timed by the board's clock.
- **Why:** Keeps one calibration per file. The GUI used to write rows on its own
  20 ms timer, which ticks unevenly on Windows: ~20 % of rows repeated a sample
  and others were skipped.

## 8.3 Dropouts: forward-fill vs gap

- **Decision:** Short shank gaps (up to 10 samples) are forward-filled; longer
  gaps are marked missing. Every CSV row records its status
  (`valid / filled / missing`).
- **Why:** Keeps plots readable without hiding where data was real.

## 8.4 Clear error diagnosis

When no valid data arrives, the CLI and GUI name the failing link: nothing on
the port, wrong or old firmware, or a dead peripheral.

## 8.5 Windows executable, released on version tags

- **Decision:** Ship the GUI as a single Windows program,
  `KneeGoniometerGUI.exe`, built by a GitHub Actions workflow. Pushing a `v*`
  tag runs the self-test, builds the exe and publishes it on the Releases page
  (steps: README "Releasing an updated GUI", SOP Appendix B).
- **Why:** Operators should not need to install Python. Building from a tag ties
  each exe to a known commit, and the self-test stops a broken GUI from being
  released.
- **Alternatives:** Running from source (too much setup for operators); building
  the exe by hand (not repeatable, easy to ship uncommitted changes).
- **Trade-offs:** Windows only (macOS/Linux use the Python version); the exe is
  unsigned, so Windows SmartScreen warns on first run.

# 9. Testing and validation

## 9.1 Preliminary stationary testing (Sept 2026)

Full results are in the Preliminary Stationary Test Report.

| Test | Result |
|---|---|
| Stability (6 × ~5 min holds) | ~0.01–0.02° noise; no noise growth over 0.1–60 s |
| Endurance (24–60 min) | Stable for 33 min on healthy hardware; ≈ −0.001 °/min drift |
| 45° / 90° rigs (preliminary) | Read 45.0° and 89.0° |
| Sample rate | ~40 Hz found; fixed to a true 50 Hz (§6.5, §8.2) |
| Data loss in long runs | Traced to a **faulty BMI270 on one board**; the other ran > 1 h clean in either role |

Methods worth keeping:

- **Exclude the first 20 s** of each session from stationary analysis
  (calibration and settling).
- **Board-swap test** to separate a hardware fault from a firmware fault — the
  fault moved with the board.
- **Raw accelerometer as an independent check** of physical movement; it showed
  that one session's creep was the rig settling, not sensor drift.

## 9.2 Earlier bench checks

A rigid-ruler test (both boards on one ruler, so the true angle is constant) was
the key diagnostic for drift; a right-angle placement read ~90°. The math is
covered by `knee_collector_uart.py --selftest` and `knee_gui.py --selftest`.

# 10. Directions considered but not pursued

| Area | Recommendation |
|---|---|
| A. Magnetometer fusion | Do not add for sagittal angles; only for a true 3-D requirement, after checking the magnetic environment. |
| B. Absolute anatomical angle | Functional joint-axis calibration or a known-pose reference; not prototyped. |
| C. BLE (wireless) links | Start with the board-to-PC link; needs source timestamps and batteries. Main benefit is safety / REB approval. |
| D. More nodes / joints | Needs node IDs, a generalised data model and cross-node time sync; BLE suits a star topology. |

# 11. Timeline of major milestones

| Date (2026) | Milestone | Commits |
|---|---|---|
| Aug 14 | Initial UART master/slave prototype (pitch-based complementary filter) | `09f3557`, `4b2fd42` |
| Aug 17 | Quaternion, placement-independent angle; 9-DOF magnetometer path tried | `557ba29`, `146cc14` |
| Aug 18 | Strip back to clean 6-DOF; handedness fix; gyro bias; gravity-referenced angle | `0dd3a93`, `6f1bdc7`, `276d9dc` |
| Aug 19 | One angle method; live GUI | `4effe4c`, `9ef5550` |
| Aug 20 | Link fixes (resync, stream not poll); collect-on-demand; 50 Hz | `c767bc7`, `7197327`, `86efe6d` |
| Sep 21–22 | Central/peripheral rename; research directions documented | `d9ccd14`, `170580c` |
| Sep 23–24 | Stationary testing; diagnostics; rate fix; sensor-fault guard | `99a2446`, `72943ba`, `e02ae6d`, `8a9aaf4` |
| Sep–Oct | Preliminary Stationary Test Report | `70815cf`, `38724bf` |
| Oct 9–13 | Handover docs: SOP, design decisions, handover summary | `d0dd9be`, `82835c7` |
| Oct 13 | Windows exe build workflow; release v1.0.0; case STL added | `572b164`, `a789fd6` |

# Appendix A — Magnetometer fusion: accuracy, and absolute vs. relative angle

**Status:** Investigated (analysis only). Do not add for the current sagittal
use case; pursue only if the requirement shifts to 3-D / out-of-plane
kinematics. For an absolute (anatomical) angle, see Appendix B.

## A.1 The proposal

The boards carry a BMM150 magnetometer that is currently unused (§4.1). The
proposal was to use it to (a) improve accuracy, and/or (b) recover an
**absolute** joint angle rather than the current **relative** one, on the
intuition that "the magnetometer gives the vector to north, so we can use it as
a reference to compute the absolute angle between the segments."

## A.2 What a magnetometer actually measures

- The **local magnetic field as a 3-D vector in the sensor's own frame**, just
  as the accelerometer reports gravity in its own frame. Rotating the chip
  changes the reading; that change *is* the orientation information.
- The field points to **magnetic** north and **dips into the ground** (~60–70°
  below horizontal at mid latitudes). It is not a clean horizontal "north
  arrow"; a compass hides this by using only the horizontal component.
- The angle between the magnetic and gravity vectors is fixed by the
  **location** (`90° − dip`), not by the leg, so it carries no joint information.

## A.3 Two jobs: a *scale* and a *zero*

Measuring any angle needs two independent things:

1. **The scale** — how far apart two directions are.
2. **The zero** — which configuration counts as 0°.

Board-mounted sensors are good at the scale. The zero is an *anatomy* problem,
not a sensor problem. This split is the crux of the magnetometer question.

## A.4 What the magnetometer adds (the scale)

- **Heading.** Gravity fixes tilt (2 degrees of freedom) but leaves rotation
  about vertical unobservable and drifting independently on each board. The
  magnetometer pins that last axis.
- With gravity **and** the magnetic field, each board's full 3-D orientation is
  known, and so is the full, drift-free relative orientation between the boards
  (`R_rel = R_thigh⁻¹ · R_shank`).
- **Net:** its only unique contribution is the **out-of-sagittal-plane** part of
  the angle. For the sagittal angle, gravity alone already suffices because both
  boards share the same "down".

## A.5 Why it does not give an absolute angle

The joint angle is between the **bones**, not the **boards**. Each board sits on
skin at an unknown mounting rotation `M_i` (board → bone):

```
joint = M_thigh⁻¹ · R_rel · M_shank
```

The magnetometer nails `R_rel` but contributes nothing to `M_thigh` or
`M_shank`.

- **Two meanings of "absolute":** absolute *heading* (Earth-referenced yaw),
  which the magnetometer provides; and an absolute *anatomical zero* (the true
  femur–tibia angle, e.g. a flexion contracture), which it does not.
- **The decisive case:** two people can hold their segments in identical
  orientations, with identical accelerometer *and* magnetometer readings, while
  one is a healthy leg posed at 10° and the other a 10° contracture at
  best-effort straight. Same data, opposite clinical truth, so no function of the
  sensor data can recover the contracture.
- **"Straight" is not a direction in the room.** A joint angle does not depend on
  how the person faces; room orientation is the only thing the magnetometer adds.

The current method is relative because of the **static zero hold** (§5.3): the
held pose becomes 0°, and the unknown mounting is absorbed into it and cancels.

## A.6 Trade-offs

- **The fundamental trade:** either **pin** the board → bone mounting
  (calibration gives an absolute zero, but placement precision matters) or
  **cancel** it (re-zero each session: relative zero, robust to placement). The
  current design cancels, which is what makes it placement-independent. The
  magnetometer is on neither branch.
- **Magnetic disturbance:** table and chair legs, treadmills, wheelchairs, rebar
  floors, batteries, the USB cable and the other board all distort the field.
  Indoor clinical and gym settings are magnetically dirty.
- **Calibration burden:** per-board hard/soft-iron calibration (figure-8 motion),
  repeated as conditions change.
- **Accuracy gain for sagittal ROM, gait, sit-to-stand:** about zero.

## A.7 Next experiments (if revived)

1. **Field-quality check (do first):** log raw magnetometer values during a
   normal session *in the real test environment* and plot the field magnitude.
   Roughly constant (~25–65 µT) as the limb moves means the environment is clean
   enough; large swings mean fusion would add error.
2. **3-D comparison:** revive the old 9-DOF code behind a flag and compare it
   with 6-DOF on out-of-plane motions, not sagittal flexion.
3. Integrate only if (1) passes and a 3-D requirement actually exists.

# Appendix B — Absolute (anatomical) angle via functional calibration

**Status:** Identified as the correct path for an absolute angle. Not yet
prototyped.

## B.1 Motivation

The current angle is relative to the calibration pose (§5.5), so it cannot
report an absolute clinical angle (e.g. a flexion contracture) or be compared
across sessions zeroed on different poses. Per Appendix A, this needs a
reference that **observes the anatomy**.

## B.2 Candidate approaches

- **Functional joint-axis calibration:** the subject performs prescribed pure
  knee-flexion reps. The flexion axis shows up in the gyro as the dominant,
  consistent rotation axis in each board's frame, which pins the board → bone
  mounting. Needs no magnetometer; this is how research-grade IMU systems report
  anatomical angles.
- **Known-pose / external reference:** define true full extension once (a
  straight-leg fixture, or a manual goniometer or imaging reading) and anchor the
  anatomical zero to it.

## B.3 Trade-offs

- Gains an absolute, cross-session-comparable, contracture-capable angle.
- Costs placement and calibration discipline (the "pin" branch in A.6) and a
  repeatable calibration protocol per subject.
- Depends on how well the calibration motion isolates the flexion axis; needs
  validation against a goniometer or other gold standard.

## B.4 Next steps

- Prototype a functional-axis estimator from a flexion-only capture and compare
  its zero against a manually measured angle at known poses.
- Decide whether the clinical use needs absolute angles at all, or whether the
  difference-based measures (ROM, rep counting, gait, movement quality) are
  sufficient.

# Appendix C — Bluetooth (BLE) links: going wireless

**Status:** Idea raised; not investigated in depth.

## C.1 Motivation

The boards' nRF52840 chips already have BLE radios, unused today. Two links could
go wireless (Figure 2):

- **Link A — peripheral ↔ central** (board to board): today the wired UART,
  including the wire across the knee.
- **Link B — central ↔ PC** (board to PC): today USB serial.

**Main driver: safety and REB approval.** Removing wires — especially one across
the joint or tethering the subject to a PC — reduces trip, entanglement and
electrical-safety concerns. It also frees gait and sit-to-stand movements.

## C.2 How

- ArduinoBLE on the boards (packets as BLE notifications); a BLE client such as
  Python `bleak` replaces pyserial on the PC. Either link can be cut on its own.
- **Link B first:** it removes the PC tether without touching the board-to-board
  data path. Link A removes the wire that crosses the knee.

## C.3 Things to watch

- **Dropouts:** the existing gap handling (§6.3, §8.3) helps, but BLE adds its
  own loss and reconnection behaviour.
- **Timing:** BLE delivers in connection intervals (≥ 7.5 ms) with jitter, so
  samples should be **timestamped at the source** rather than on receipt.
- **Throughput:** 50 Hz is feasible but with less headroom than UART; may need a
  larger MTU, several samples per notification, or a lower rate.
- **Framing gets simpler:** each notification is already a frame, so the header
  and resync logic (§6.2) is not needed; keep the checksum.
- **Power:** wireless means a battery per board (also a magnetic disturbance if
  the magnetometer is ever revived).
- **2.4 GHz congestion** and pairing/reconnection robustness.

## C.4 Next steps

- Prototype one link at a time, starting with Link B. Measure rate, dropout and
  timing jitter against the wired baseline, and compare angle traces.
- Decide per link whether the safety benefit outweighs the added variability for
  the intended study.

# Appendix D — Scaling to multiple nodes / multiple joints

**Status:** Idea raised; not investigated in depth.

## D.1 Motivation

Today one pair (thigh + shank) measures one joint. Studies may want a
**kinematic chain** (thigh + shank + foot for hip / knee / ankle), **bilateral**
capture, or a fuller lower-body montage. Two topologies were raised:

- **Star:** several sensor boards stream to one hub, which forwards a single
  stream to the PC.
- **Independent pairs:** several two-board units (today's firmware), each
  measuring one joint, combined on the PC.

## D.2 Key design considerations

- **Wiring pushes toward BLE (Appendix C).** The board has only one hardware
  UART, so a wired star does not fan out; a BLE hub can hold several
  connections.
- **Node addressing:** packets need a node ID, and the line format and CSV must
  generalise from fixed thigh/shank columns to N segments and M joints.
- **Time alignment** is the biggest correctness risk: angles between segments
  need time-aligned samples, so source timestamps and a shared clock across nodes
  are required.
- **Bandwidth:** N nodes × 50 Hz share one link budget, which may cap the rate or
  node count.
- **Calibration** grows with each segment, so the protocol and UI must scale.

## D.3 Topology trade-off

| Topology | For | Against |
|---|---|---|
| Star | One stream and one time base at the hub; simple PC side | Hub's link budget is the ceiling; single point of failure |
| Independent pairs | Modular; reuses current firmware | PC must time-align several streams; more radios / USB ports and hardware |

## D.4 Next steps

- Define a node-ID packet and a generalised data model (N segments, M joints)
  before choosing a topology.
- Prototype cross-node time sync and measure residual skew against the accuracy
  the study needs.
- Choose by study scale: a BLE star for a compact multi-segment montage;
  independent pairs when modularity matters more.
