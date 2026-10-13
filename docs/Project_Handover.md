---
title: "Dual-IMU Knee Goniometer — Project Handover"
subtitle: "State at transition, design decisions and their rationale, and directions to explore"
---

# About this document

This is the handover record for the dual-IMU knee goniometer proof of concept
(August–October 2026). It brings together, in one place:

- **Part 1 — Handover summary (§1):** the state of the project at transition,
  open issues, and what to do next. Read this first.
- **Part 2 — Design decisions (§2–§11):** what was decided, why, which
  alternatives were considered, and where the evidence lives.
- **Part 3 — Areas to explore (Appendices A–D):** longer analyses of directions
  that were weighed but not pursued yet.

Companion documents:

- `docs/SOP_Build_and_Use.docx` (`.md` mirror) — how to build and operate the device.
- `README.md` — the technical reference (math, wire protocol, commands).
- `analysis/static_test/Static_Stability_Test_Report.docx` — the Preliminary
  Stationary Test Report, with `FINDINGS.md` as its detailed working record.

Each design decision follows the same layout: **Decision**, **Why**,
**Alternatives considered**, **Evidence** (commits / documents), **Status**.
Commit hashes refer to this repository's history. Where code is referenced,
function names are in `knee_collector_uart.py` unless noted.

# Part 1 — Handover summary

# 1. State at transition (October 2026)

## 1.1 Current state

- **Working bench prototype.** Two Arduino Nano 33 BLE Rev2 boards (thigh =
  central, shank = peripheral) in 3-D printed cases (`DualIMU_Goniometer.stl`),
  joined by soldered UART wires, each powered over USB; the central streams to a
  PC at a true 50 Hz.
- **Software complete for PoC testing.** Live GUI with guided calibration and
  auto-saved sessions; CLI collector; self-tests pass. The GUI is released as a
  Windows program, `KneeGoniometerGUI.exe` (v1.0.0, Releases page).
- **SOP finalised** (`docs/SOP_Build_and_Use.docx`, version 1.0): build, setup,
  sessions and troubleshooting for operators; Python setup and releasing a new
  GUI version are in its appendices.
- **Preliminary stationary testing done.** ≈0.01–0.02° noise, ≈ −0.001 °/min
  drift on healthy hardware; 45°/90° rigs read 45.0°/89.0° (preliminary; §9.1).
- **Not yet done:** dynamic / on-body testing, both-node calibration, a
  controlled accuracy test (§9.3).

## 1.2 Open issues

1. **Faulty board.** One board's BMI270 fails after 20–40 min (§9.1). It must be
   identified, labelled and replaced. It should be the board without the red
   wire soldered to the 3.3 V pin; confirm before replacing it.
2. **Raw test data not preserved.** The stationary-test CSVs and diagnostic
   logs were not archived (`*.csv` / `*.log` are git-ignored), so the figures in
   the report cannot be regenerated. Plan data storage once testing moves
   beyond the preliminary stage.
3. **Analysis scripts are working scripts.** `analysis/static_test/stats.py`,
   `plot.py` and `calibration.py` were written ad hoc for this dataset (they
   read every CSV in the current folder). `plot_report.py`, `plot_session2.py`
   and `plot_angle_rig.py` produced the report figures and expect specific file
   names.
4. **Peripheral-only fault guard.** The sensor-fault guard runs on the
   peripheral only; the central has no equivalent (§7.2).
5. **Angle log resolution.** Angles are written to 0.01°, which limits how low
   a noise floor can be measured.

## 1.3 Recommended next steps (in order)

1. Replace the faulty board; repeat a one-hour stationary run on two healthy
   boards for a clean baseline.
2. Sweep **both** nodes during calibration so the angle uses both sensors
   (§5.3); fix the boards and cables firmly.
3. Try the strap placement in the SOP on a person and refine it if the boards
   shift during movement.
4. Controlled accuracy test: repeated 0°/45°/90° placements on a precise rig or
   against a goniometer.
5. Dynamic range-of-motion testing (tune `TWO_KP` and the soft accel gate, §4.4,
   if fast reps lag or overshoot).
6. Consider the longer-term directions in §10 and Appendices A–D (battery +
   BLE for an untethered device; absolute angle via functional calibration).

## 1.4 Where things are

| Document | Use it for |
|---|---|
| `docs/SOP_Build_and_Use.docx` (master; `.md` mirror) | Building the device and running a session |
| `docs/Project_Handover.md` (+ `.docx`) | This document: state, design rationale, future directions |
| `analysis/static_test/Static_Stability_Test_Report.docx` | Preliminary Stationary Test Report |
| `analysis/static_test/FINDINGS.md` | Detailed working record behind the report |
| `README.md` | Technical reference: math, wire protocol, commands |

```
central_imu/central_imu.ino          thigh board firmware
peripheral_imu/peripheral_imu.ino    shank board firmware
knee_collector_uart.py               angle math, calibration, CLI collector, --selftest
knee_gui.py                          live GUI (uses knee_collector_uart.py)
requirements.txt                     Python dependencies
DualIMU_Goniometer.stl               3-D printable board case (base + lid)
analysis/static_test/                stationary test scripts, figures, report
analysis/diagnostics/parse_diag.py   diagnostic log -> tables
docs/                                SOP and this handover document
docs/build_docs.sh                   regenerates this document's .docx from its Markdown
.github/workflows/release.yml        builds KneeGoniometerGUI.exe (Windows GUI) on a v* tag
```

## 1.5 Housekeeping

- No licence file — decide whether one is needed before sharing the code.
- **GUI release:** `v1.0.0` is published with `KneeGoniometerGUI.exe` on the
  Releases page (tag on `main` at the handover). To publish a changed GUI, follow
  "Releasing an updated GUI" in the README (also SOP Appendix B): merge to `main`,
  push the next version tag, and the release workflow builds and attaches the exe
  (§8.6).
- **Documents:** the Word and Markdown copies of each document share a name.
  `Project_Handover.md` is the master for this document; regenerate its Word copy
  after editing with `bash docs/build_docs.sh`. The SOP is the other way round:
  edit `SOP_Build_and_Use.docx` in Word and refresh its `.md` mirror (command at
  the top of the mirror).

# Part 2 — Design decisions

# 2. Problem and goals

The goal is a low-cost wearable that measures **knee flexion angle** from two
inertial sensors, one above and one below the knee.

| Goal | What it means in practice |
|---|---|
| Placement independence | The boards should not need to be mounted at a precise or repeatable orientation on the limb. |
| Robust signal | Stable at rest, smooth through motion, no drift in the angle. |
| Simple, verifiable PoC | Prefer the smallest system that can be checked on the bench over more capable but unverified machinery. |
| Usable in testing sessions | A GUI that drives calibration, shows live data, and saves every session automatically. |

Scope: **sagittal-plane** knee flexion (standing ROM, gait, sit-to-stand),
relative to a calibration pose. Absolute anatomical angles and 3-D kinematics
are out of scope (see §5.5 and §10).

# 3. System architecture

```
 shank board (peripheral)  --UART-->  thigh board (central)  --USB serial-->  PC
   BMI270, Mahony filter   <--keep--    BMI270, Mahony filter                 knee_gui.py / knee_collector_uart.py
                              alive                                           (calibration, angle, CSV)
```

## 3.1 Two-board central/peripheral topology

- **Decision:** The thigh board (*central*) runs its own filter, receives the
  shank board's (*peripheral*) state over a wire, merges both and sends one
  line per sample to the PC. The PC computes the angle.
- **Why:** Only one board needs a PC connection; both segments arrive already
  time-aligned on the central's clock; the angle math stays in Python where it
  is easy to test and change without reflashing.
- **Alternatives:** Each board on its own USB port (needs PC-side time
  alignment of two streams); computing the angle on the board (harder to
  test and iterate).
- **Evidence:** `README.md` (Hardware & wiring); `d9ccd14` renamed
  master/slave → central/peripheral.
- **Status:** In use.

## 3.2 Hardware: Arduino Nano 33 BLE Rev2

- **Decision:** Two Arduino Nano 33 BLE Rev2 boards, using the onboard
  **BMI270** accelerometer + gyroscope. The onboard **BMM150 magnetometer is
  not used.**
- **Why:** Off-the-shelf, inexpensive, IMU on board, Arduino tooling, BLE radio
  available for a future wireless version.
- **Status:** In use. One of the two boards has a faulty BMI270 (see §9.1).

## 3.3 Physical build as it stands

- **Decision:** Each board sits in a custom 3-D printed case
  (`DualIMU_Goniometer.stl`, base + lid) and is held on the leg with an elastic
  strap. The boards are joined by **soldered wires** (two UART lines + ground).
  **Each board is powered by its own external USB supply**, and the central is
  connected to the PC over USB serial for data.
- **Why:** Fastest route to a working, reliable bench prototype.
- **Trade-offs:** The subject is tethered to the PC and to power; a wire runs
  across the knee. Battery power and wireless links are future work (§10,
  Appendix C).
- **Evidence:** `a789fd6` (case STL); SOP §§2–3 and 6 (materials, build, placing the sensors).
- **Status:** Bench prototype. The strap placement in the SOP has not yet been
  tried on a person.

# 4. Sensor fusion (on each board)

## 4.1 6-DOF Mahony filter, no magnetometer

- **Decision:** Each board estimates its orientation with a 6-DOF Mahony
  filter (accelerometer + gyroscope), `TWO_KP = 1.0`, `TWO_KI = 0.04`.
- **Why:** The angle method (§5) only needs the direction of gravity in each
  board's frame, which 6-DOF provides without drift. A magnetometer is
  disturbed by nearby metal (furniture, treadmills, the USB cable, the other
  board) and needs per-board hard/soft-iron calibration.
- **Alternatives:** A 9-DOF path with runtime magnetometer calibration and an
  automatic BMM150→BMI270 axis solver was built (`146cc14`, `87096d9`,
  `d79e51a`, `9cf21cb`) and then **removed** (`0dd3a93`) to get back to a clean,
  verifiable baseline — it was never confirmed on hardware, not measured and
  found worse. The analysis in Appendix A concludes it adds nothing for
  sagittal angles.
- **Status:** In use. 9-DOF code is recoverable from history if 3-D is ever needed.

## 4.2 Fixing the library's left-handed axes

- **Decision:** Negate the x axis of both accelerometer and gyroscope after
  reading them.
- **Why:** `Arduino_BMI270_BMM150` returns `x=-sensor.y, y=-sensor.x, z=sensor.z`
  — a reflection (determinant −1). A quaternion filter needs a right-handed
  frame; without the fix the gyro's rotation sense was mirrored and the filter
  wandered.
- **Evidence:** `6f1bdc7`; README finding #1; comment block at the top of both sketches.
- **Status:** In use. **Any new firmware must keep this.**

## 4.3 Gyro bias calibration with a sanity gate

- **Decision:** At the start of each collection, average ~300 gyro samples
  (up to 4 s) and subtract the result as bias, but **discard it** if any axis
  exceeds `BIAS_SANITY_DPS` = 3 °/s.
- **Why:** Bias is the main drift source in a 6-DOF filter. If the board is
  moving during calibration, the "bias" is a false rate that can make the
  filter diverge, so it is safer to use zero.
- **Evidence:** `fd0d53b`, `6f1bdc7`; README finding #2.
- **Status:** In use. Calibration happens when a collection starts (USB port
  opened), not at power-up (`86efe6d`).

## 4.4 Soft accelerometer-trust gate

- **Decision:** Weight the accelerometer correction by how close `|accel|` is
  to 1 g: full trust within 0.10 g (`ACC_TRUST_FULL_G`), ramping to none at
  0.60 g (`ACC_TRUST_ZERO_G`).
- **Why:** During brisk motion the accelerometer reads gravity *plus* linear
  acceleration. A hard on/off gate let the filter run gyro-only through a fast
  move, drift, then snap back — the "wrong reading right after returning from
  a big angle" symptom. A soft ramp always applies some correction.
- **Evidence:** `fd0fbaa` (hard gate), `4effe4c` (soft gate); README finding #7.
- **Status:** In use; not yet tuned on dynamic data.

# 5. Knee-angle method (PC side)

## 5.1 Gravity-referenced sagittal inclinometer

- **Decision:** For each board, take the direction of gravity in the board's
  own frame (from the fused quaternion), express it as a signed inclination in
  that segment's sagittal plane, and define the knee angle as
  `incl_thigh − incl_shank`.
- **Why:** Gravity-in-board is drift-free in tilt (the accelerometer pins it)
  and **invariant to heading**, so the 6-DOF yaw drift never enters. Any
  constant mounting rotation cancels, which gives placement independence.
- **Alternatives:** (a) Relative quaternion `q_thigh* ⊗ q_shank` with
  swing–twist decomposition — rejected because each board's yaw drifts
  independently in 6-DOF, so the angle drifted even on a rigid ruler
  (README finding #3). (b) Four selectable modes (`--method` ×
  `--gravity-source`) — consolidated to one to stop modes masking each other's
  failures (`4effe4c`).
- **Evidence:** `276d9dc`, `4effe4c`, `9f5c8c9`; README "How it works";
  `--selftest` proves cancellation against random mounts and per-board yaw drift.
- **Status:** In use. Code: `gravity_in_board()`, `sagittal_inclination()`,
  `gravity_knee_angle()`.

## 5.2 Gravity from the fused quaternion, not the raw accelerometer

- **Decision:** Read gravity from the Mahony quaternion; keep the raw
  accelerometer only as a filter-free cross-check.
- **Why:** The raw accelerometer is corrupted during motion; the fused
  quaternion lets the gyro carry fast movement while the accelerometer keeps
  tilt drift-free. (Raw accelerometer was briefly the default while the
  handedness bug made the quaternion wrong — README finding #4.)
- **Status:** In use. Raw accelerometer values are still streamed and logged
  (useful for checking physical movement, as in the stationary report).

## 5.3 Two-step calibration: static zero + functional sweep

- **Decision:** Each session starts with
  1. **Zeroing** (`CAL_SECONDS` = 2 s): leg straight and still; the average
     gravity direction becomes each segment's 0° axis `d_i`.
  2. **Sweep** (`SWEEP_SECONDS` = 6 s): a few slow movements bending knee and
     hip; the in-plane tilt direction becomes each segment's forward axis `f_i`.
- **Why:** These two vectors define each segment's sagittal plane in its own
  board frame without knowing how the board is mounted.
- **Detail:** Samples tilting less than `SWEEP_MIN_ANGLE_DEG` = 5° are ignored
  so noise cannot define the direction. **A segment that does not tilt ≥ 5°
  during the sweep contributes 0** (treated as fixed).
- **Consequence observed in testing:** In all bench tests only the shank node
  was swept, so the angle came from the shank alone. Both-node sweeps are a
  required next step (Stationary Test Report, Limitations).
- **Status:** In use.

## 5.4 Signed, continuous angle

- **Decision:** `incl = atan2(g·f, g·d)`: flexion positive, hyperextension
  negative, continuous through the full range with no ±90° wrap.
- **Why:** A signed four-quadrant angle distinguishes flexion from
  hyperextension and retraces identically on the way back from high flexion
  (verified in `--selftest` with a 0→130°→0 sweep, no stuck zeros on return).

## 5.5 Relative zero, not an absolute anatomical angle

- **Decision:** Accept that 0° is "the pose held during zeroing".
- **Why:** ROM, rep counting, gait and movement quality are difference-based,
  so the zero cancels. An absolute angle (e.g. a flexion contracture) needs an
  external reference that observes the anatomy; the sensors cannot supply it,
  and a magnetometer does not help (Appendix A).
- **Alternatives:** Functional joint-axis calibration or a known-pose fixture
  (Appendix B) — identified, not prototyped.
- **Evidence:** `91bc803`; README "What 0° means"; Appendices A–B.

## 5.6 Accepted limitation: sagittal plane only

Gravity gives 2 of 3 rotational degrees of freedom; rotation about vertical is
unobservable. The method therefore under-reads motion well out of the vertical
plane (e.g. lying down with large hip rotation). This is an observability
limit, accepted for upright knee flexion.

# 6. Board-to-board link and data protocol

## 6.1 Wired UART at 115 200 baud

- **Decision:** `Serial1` UART at 115 200 baud, both directions wired
  (peripheral TX → central RX for data; central TX → peripheral RX for the
  keepalive), common ground.
- **Why:** Simple and deterministic. The rate was briefly raised to 460 800 to
  shorten time on the wire, which masked a framing bug and added sensitivity to
  clock drift as boards warm up; 115 200 gives ~4× the per-byte timing margin
  and is far above the ~15 kbit/s needed.
- **Alternatives:** BLE (Appendix C) — deferred.
- **Evidence:** `fd0fbaa`, `c767bc7`; README finding #6.

## 6.2 Framed binary packet with resync and checksum

- **Decision:** 30-byte packet: `0xAA` header, quaternion (4 floats),
  raw accel (3 floats), XOR checksum. The central resyncs on the header and
  drops packets that fail the checksum.
- **Why:** The original fixed-length read had no resync, so one lost byte
  shifted every following packet. Now a glitch costs one packet.
- **Evidence:** `c767bc7`.

## 6.3 Peripheral streams; central never waits

- **Decision:** The peripheral **streams** packets at 50 Hz; the central
  drains its UART buffer each loop and keeps the freshest valid packet. If that
  packet is older than `SHANK_STALE_US` = 30 ms the shank is reported invalid
  (zeros) and the PC forward-fills it. The packet age is the line's last field.
- **Why:** Polling with a deadline lost samples whenever the peripheral's RTOS
  stalled for a few ms; widening the deadline only stalled the central. Streaming
  decouples the two boards.
- **Evidence:** `3b97e85`, `7197327`, `4fc3a07`, `c111974`; README finding #6.

## 6.4 Collect-on-demand

- **Decision:** Both boards run their IMUs only while the central's USB port
  is open. On open, the central calibrates and sends `S` keepalives every
  200 ms; the peripheral idles if none arrives for 1 s, or on `X`.
- **Why:** Nothing runs unobserved; each session starts with a fresh bias
  calibration on a still board, rather than relying on a boot-time one.
- **Evidence:** `86efe6d`.

## 6.5 Fixed 50 Hz output, one write per line

- **Decision:** Both boards emit at a fixed 50 Hz (`EMIT_PERIOD_US` = 20 000)
  on a fixed schedule; the central builds each PC line in memory and sends it
  with **one** write. Filters still run at the sensor's native rate.
- **Why:** ~104 Hz left little timing margin and 50 Hz is ample for knee ROM.
  Testing showed the central actually ran at ~43.5 Hz because it printed each
  line as ~36 blocking prints (~11.7 ms); the one-write line takes ~1–1.8 ms
  and restored a true 50 Hz.
- **Evidence:** `8211f4a`, `5c8f67d`, `e02ae6d`; Stationary Test Report, Appendix.

## 6.6 Diagnostics channel

- **Decision:** With `DIAG 1` (the current default in both sketches) the
  peripheral sends a once-a-second health frame (`0xAB` header) that the central
  relays as `# PDIAG …`, and the central adds `# CDIAG …`. The GUI saves all `#`
  lines to `knee_diag_<timestamp>.log`; `analysis/diagnostics/parse_diag.py`
  turns them into tables.
- **Why:** Needed to find the sample-rate shortfall and the faulty sensor.
- **Constraint:** The `PDiag` struct must match byte for byte in both sketches;
  flash both boards from the same commit.
- **Evidence:** `72943ba`.

# 7. Robustness and fault handling

## 7.1 Peripheral self-healing

- **Low-rate start:** if fewer than 30 samples arrive in 1 s, re-initialise the
  sensor (the BMI270 occasionally starts at a wrong ~10 Hz rate) — `4036498`.
- **Data-ready stall:** if no sample for 250 ms, LED solid and re-init every
  500 ms — `5f8aaff`.
- **IMU init failure at boot:** fast-blink ~3 s, then reboot.

## 7.2 Sensor-fault guard

- **Decision:** On the peripheral, reject a sample if the read took > 50 ms,
  `|accel|` is < 0.25 g or > 4 g, or the gyro reads > 1000 °/s while `|accel|`
  is within 0.15 g of 1 g. Rejected samples are sent as an all-zero quaternion
  (a gap, not a wrong angle). Each re-init resets the filter's integral term.
  If no good sample arrives for 5 s, the board reboots.
- **Why:** In endurance runs one board's BMI270 began returning garbage after
  20–38 min and then stopped entirely; this turns wrong angles into visible gaps.
- **Note:** Only the peripheral has this guard. The central does not (open
  issue, §1.2).
- **Evidence:** `e02ae6d`; Stationary Test Report, "Cause: one faulty board".

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
  and `--selftest`; `knee_gui.py` (Tkinter + Matplotlib) reuses it and adds port
  scanning, a colour-coded calibration banner, live plots, and auto-saved sessions.
- **Why:** One tested implementation of the math; a GUI makes test sessions
  repeatable for non-developers.
- **Evidence:** `9ef5550`, `d15c4db`, `1a79b4d`, `a0e852a`.

## 8.2 Session-based collect / stop

- **Decision:** Each "Calibrate & collect" starts a new session with fresh
  calibration and a new timestamped CSV; "Stop collecting" freezes the plot on
  the full session for inspection. No pause/resume.
- **Why:** Avoids mixing calibrations in one file; makes every file self-contained.

## 8.3 One CSV row per received sample

- **Decision:** Write exactly one row per line from the central, timed by the
  board's clock (`t_session_s`), plus the PC receive time (`t_wall_iso`).
- **Why:** The GUI used to write rows on its own 20 ms timer, which ticks
  unevenly on Windows: ~20 % of rows repeated a sample and others were skipped.
- **Evidence:** `8a9aaf4`; Stationary Test Report, Appendix.

## 8.4 Dropouts: forward-fill vs gap

- **Decision:** Short gaps (≤ `MAX_FILL` = 10 samples) are forward-filled;
  longer gaps are marked missing. The CSV records the status of every sample
  (`valid / filled / missing`), and the GUI can display either a filled line or
  discrete points.
- **Why:** Keeps plots readable without hiding where data was real.

## 8.5 Clear error diagnosis

`diagnose_stream()` names the failing link: no data on the port, wrong/old
firmware (not an 18-field `D` line), or a dead peripheral (all shank samples
invalid). The CLI and GUI share it.

## 8.6 Windows executable, released on version tags

- **Decision:** Ship the GUI to operators as a single Windows program,
  `KneeGoniometerGUI.exe`, built with PyInstaller (one file, windowed) by a
  GitHub Actions workflow. Pushing a `v*` tag runs the self-test, builds the exe
  and publishes it on the Releases page.
- **Why:** People collecting data should not need to install Python or use a
  terminal. Building in CI from a tag ties each exe to a known commit and keeps
  the build repeatable; the self-test step stops a broken GUI from being released.
- **Alternatives:** Running from source for everyone (too much setup for
  operators); building the exe by hand on a developer's PC (not repeatable, easy
  to ship uncommitted changes).
- **Trade-offs:** Windows only (macOS/Linux use the Python version); the exe is
  unsigned, so Windows SmartScreen warns on first run; recordings are saved next
  to the exe, so it must live in a writable folder.
- **Evidence:** `572b164` (`.github/workflows/release.yml`); release `v1.0.0`.
- **Status:** In use. How to publish a new version: README, "Releasing an
  updated GUI" (also SOP Appendix B).

# 9. Testing and validation

## 9.1 Preliminary stationary testing (Sept 2026)

Full results: `analysis/static_test/Static_Stability_Test_Report.docx`.

| Test | Result |
|---|---|
| Stability (6 × ~5 min holds) | ~0.01–0.02° noise; no noise growth (Allan deviation flat 0.1–60 s) |
| Endurance (24–60 min) | Stable for 33 min on healthy hardware; ≈ −0.001 °/min drift |
| 45° / 90° rigs (preliminary) | Read 45.0° and 89.0° |
| Sample rate | ~40 Hz found; fixed to a true 50 Hz |
| Data loss in long runs | Traced to a **faulty BMI270 on one board**; the other ran > 1 h clean in either role |

Key decisions from testing:

- **Exclude the first 20 s** of each session from stationary analysis
  (calibration and settling).
- **Board-swap test** to separate a hardware fault from a firmware fault — the
  fault moved with the board.
- **Raw accelerometer as an independent check** of physical movement, which
  showed that Session 2's creep was the rig settling, not sensor drift.

## 9.2 Earlier bench checks

Rigid-ruler test (both boards on one ruler, true relative angle constant) was
the key diagnostic for drift; the right-angle placement read ~90°. Math is
covered by `knee_collector_uart.py --selftest` and `knee_gui.py --selftest`.

## 9.3 Not yet tested

Both-node calibration; dynamic movement; on-body use; controlled accuracy
against a goniometer or precise rig; noise below the 0.01° log resolution.

# 10. Directions considered but not pursued

Each is analysed in its own appendix (Part 3).

| Area | Recommendation |
|---|---|
| A. Magnetometer fusion | Do not add for sagittal angles; only for a true 3-D requirement, after checking the magnetic environment. |
| B. Absolute anatomical angle | Functional joint-axis calibration or a known-pose reference; not prototyped. |
| C. BLE (wireless) links | Start with the board-to-PC link; needs source timestamps and batteries. Main benefit is safety / REB approval. |
| D. More nodes / joints | Needs node IDs, a generalised data model and cross-node time sync; BLE suits a star topology. |

To add an area, append an appendix with the same layout: **Status → Motivation →
What it enables & how → Reasoning → Design trade-offs → Next experiments →
Recommendation**, tie claims to code (function names, commit hashes) and README
sections, and add a row to the table above.

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
| Oct 9–13 | Handover docs: SOP v1.0, design decisions, handover summary | `d0dd9be`, `82835c7` |
| Oct 13 | Windows exe build workflow; release v1.0.0; case STL added | `572b164`, `a789fd6` |

# Part 3 — Areas to explore

# Appendix A — Magnetometer fusion: accuracy, and absolute vs. relative angle

**Status:** *Investigated (analysis only). Recommendation: do not add for the
current sagittal use case; pursue only if the requirement shifts to 3-D /
out-of-plane kinematics. For an absolute (anatomical) angle, pursue functional
calibration instead — see Appendix B.*

## A.1 Motivation / the proposal

The boards (Arduino Nano 33 BLE Rev2) carry a BMM150 magnetometer that is
currently **unused**. A proposal was raised: use the magnetometer to (a) improve
accuracy, and/or (b) recover an **absolute** joint angle rather than the current
**relative** one. The intuition offered was: "the magnetometer gives the vector
to north, so we can use it as a reference to compute the absolute angle between
the segments."

This appendix captures the analysis of whether that holds.

## A.2 Current state (baseline this is measured against)

- Fusion is **6-DOF** (accel + gyro), per-board Mahony filter (§4.1). No
  magnetometer — chosen deliberately for robustness to nearby metal.
- Angle method: **gravity-referenced sagittal inclinometer** (§5). Gravity-in-board
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

## A.3 What a magnetometer actually measures

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

## A.4 The key framing: two jobs — a *scale* and a *zero*

Measuring any angle needs two independent things:

1. **The scale** — how far apart two directions are (the geometry / "ruler").
2. **The zero** — which configuration counts as 0° (the "datum" / baseline).

Board-mounted inertial/magnetic sensors are good at the **scale**. The **zero**
is a separate problem, and it is an *anatomy* problem, not a sensor problem. This
split is the crux of the whole magnetometer question.

## A.5 What the magnetometer enables, and how (the scale)

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

## A.6 Absolute vs. relative angle — why the magnetometer does not close it

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

## A.7 Design choices / trade-offs

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

## A.8 Concrete next experiments (if this area is revived)

1. **Field-quality gate (cheapest, do first):** log raw `mx,my,mz` for a normal
   session *in the actual test environment* and plot field magnitude
   `√(mx²+my²+mz²)`. Roughly constant (~25–65 µT) as the limb sweeps ⇒ environment
   clean enough for fusion; large swings ⇒ locally distorted, fusion would inject
   error. This decides the question empirically for the real setting.
2. **A/B in 3-D:** revive the old 9-DOF branch behind a flag and compare against
   6-DOF specifically on out-of-plane motions, not sagittal flexion.
3. Only pursue integration if (1) passes and a 3-D requirement actually exists.

## A.9 Recommendation

- **Sagittal knee flexion (current scope):** do **not** add the magnetometer — no
  accuracy gain, and it undoes metal-robustness.
- **True 3-D kinematics (out-of-plane):** the magnetometer is the principled way
  to get the 3rd DOF; revive the old branch, but run the field-quality gate first
  and budget for calibration.
- **Absolute anatomical angle (contracture, cross-session comparability):** not a
  magnetometer problem — see Appendix B.

# Appendix B — Absolute (anatomical) angle via functional / anatomical calibration

**Status:** *Identified as the correct path for an absolute angle. Not yet
prototyped.*

## B.1 Motivation

The current angle is relative to the calibration pose (§5.5), so it cannot report
an absolute clinical angle (e.g. a flexion contracture: "best-effort straight" is
really 10° of flexion) or be compared across sessions zeroed on different poses.
Per Appendix A, this gap is anatomical and cannot be closed by the magnetometer;
it requires a reference that **observes the anatomy**.

## B.2 Candidate approaches

- **Functional joint-axis calibration:** have the subject perform prescribed pure
  knee-flexion reps. The flexion axis appears as the dominant, consistent rotation
  axis in each board's frame (observable from the **gyro**), which pins the
  board→bone relationship `M_i`. Needs **no magnetometer**. This is how
  research-grade IMU systems claim anatomical angles.
- **Known-pose / external reference:** physically define true full extension once
  (straight-leg fixture, or a manual goniometer / imaging reading) and anchor the
  anatomical zero to it.

## B.3 Trade-offs

- Buys an absolute, cross-session-comparable, contracture-capable angle.
- Costs placement/calibration discipline (the "pin the mounting" branch of the
  trade in A.7) and a defined, repeatable calibration protocol per subject.
- Sensitivity to how faithfully the calibration motion isolates the flexion axis;
  needs validation against a manual goniometer / gold standard.

## B.4 Next steps

- Prototype a functional-axis estimator from a flexion-only capture and compare
  its inferred zero against a manually measured angle at known poses.
- Decide whether the clinical use actually needs absolute angles, or whether the
  difference-based measures (ROM, rep counting, gait, movement quality) that the
  relative angle already serves are sufficient.

# Appendix C — Bluetooth (BLE) links: going wireless

**Status:** *Idea raised; not investigated in depth. Captured for later.*

## C.1 Motivation

Replace the current wired links with Bluetooth Low Energy. Both boards are
Arduino Nano 33 BLE Rev2 (nRF52840), so BLE radios are already on board and
unused for data today. Two links are in scope (using the terms as raised;
endpoints spelled out to avoid ambiguity):

- **Link A — Peripheral ↔ Central** (board-to-board; *"intra-node"*). Currently
  wired UART on `Serial1` @ 115200, both directions (shank data stream + central
  keepalive; §6.1).
- **Link B — Central ↔ Data-collector PC** (board-to-PC; *"inter-node"*).
  Currently USB serial (the text `D,...` line protocol).

**Primary driver: REB / safety.** Removing wires — especially any wire running
across the joint or tethering the subject to a PC / mains — reduces trip,
entanglement, and electrical-safety concerns, which is easier to justify to a
Research Ethics Board. Secondary benefits: better wearability and ecological
validity (freer gait / sit-to-stand), no bench tether.

## C.2 What it enables, and how (brief)

- A fully untethered subject; either link can be cut independently.
- **How:** ArduinoBLE on the boards (BLE GATT notifications carry the packets);
  on the PC side, replace pyserial with a BLE client (e.g. Python `bleak`). The
  central can act as BLE central toward the peripheral and BLE peripheral toward
  the PC simultaneously, or the peripheral can advertise/notify directly.
- **Link B is likely the higher REB value for the lower risk** (removes the PC /
  mains tether without touching the cross-body data path), so it is the natural
  first target; Link A removes the wire that actually crosses the knee.

## C.3 Known drawbacks (already anticipated) and things to watch

- **Packet loss / dropouts** — the existing validity + forward-fill design
  (`SHANK_STALE_US`, `age_us`; §6.3, §8.4) already handles gaps, but BLE adds its
  own loss and reconnection modes.
- **Timing / contaminated samples** — BLE quantizes to the connection interval
  (≥ 7.5 ms) with jitter, so *when* a sample arrives is no longer clean. Mitigate
  by **timestamping at the source** (have the sampler stamp its own sample time
  and send it) rather than on receipt as today, decoupling angle timing from link
  jitter.
- **Rate / throughput** — 50 Hz (20 ms) is feasible within BLE connection
  intervals but with less headroom than UART; may need a larger ATT MTU, batching
  several samples per notification, or a modest rate drop. Validate against the
  wired baseline.
- **Framing (a simplification, actually)** — BLE notifications are inherently
  framed per-notification, so the `0xAA` header / byte-resync logic UART needed
  (§6.2; README finding #6) is unnecessary on a BLE link; keep the checksum as a
  cheap integrity check.
- **Power** — going wireless implies battery power, adding a LiPo per board
  (interacts with Appendix A: a battery near the board is also a magnetic
  disturbance source if the magnetometer is ever revived).
- **2.4 GHz congestion / coexistence** and pairing/reconnection robustness.

## C.4 Next steps

- Prototype one link at a time (start with Link B). Measure achievable rate,
  dropout %, and timing jitter vs. the wired baseline, and A/B the resulting angle
  traces to confirm the signal is not degraded beyond what the analysis needs.
- Decide per link whether the REB / wearability benefit outweighs the added
  variability for the intended study.

# Appendix D — Scaling to multiple nodes / multiple joints

**Status:** *Idea raised; not investigated in depth. Captured for later.*

## D.1 Motivation

Today the system is one pair (thigh + shank) measuring one joint (knee). Many
studies want more: a **kinematic chain** on one limb (thigh + shank + foot →
hip / knee / ankle), **bilateral** capture (both knees), or a fuller lower-body
montage. Two topologies were raised:

- **A. Star — multiple peripherals → one central.** Several sensor boards stream
  to a single hub, which aggregates and forwards one stream to the PC.
- **B. Multiple independent central–peripheral pairs.** Several 2-board units,
  each measuring its own joint (reusing today's firmware), aggregated at the PC.

## D.2 What it enables

- Multi-joint / multi-segment angles from one capture (e.g. hip–knee–ankle),
  bilateral symmetry measures, richer gait analysis.
- With more segments, *between-segment* angles beyond a single pair become
  possible (any two instrumented segments), subject to the same relative-vs-
  absolute limits in Appendices A–B.

## D.3 How / key design considerations (brief)

- **Wiring pushes toward BLE (ties to Appendix C).** The Nano 33 BLE exposes only
  one hardware UART (`Serial1`), so a *wired* star to one central does not fan out
  cleanly. A BLE central, by contrast, can hold **multiple** peripheral
  connections — so topology A is most natural over BLE.
- **Node addressing.** The packet needs a **node-ID** field (and the `D,...` line
  / CSV must generalize from fixed thigh/shank columns to N segments / M joints).
- **Time alignment becomes critical.** Any angle between two segments needs
  time-aligned samples. With several asynchronous streams, receipt-time stamping
  is not enough — move to **source timestamps + a common time base / clock sync**
  across nodes. This is the single biggest correctness risk when scaling.
- **Bandwidth / rate scaling.** N nodes × 50 Hz × packet size shares one link
  budget (a BLE central shares connection intervals across peripherals); may cap
  the achievable per-node rate or node count.
- **Calibration scales linearly.** Each segment needs its own zero + sweep, so the
  calibration protocol and UI must scale (more steps, clearer subject guidance).

## D.4 Topology trade-off

- **A. Star (many peripherals → one central):** single aggregated stream, one
  time base at the hub, simpler PC-side ingest — but the hub's link budget /
  connection count is the ceiling and it is a single point of failure.
- **B. Independent pairs:** modular and reuses current firmware per pair; each pair
  is isolated — but the **PC** must ingest and time-align multiple streams, and it
  needs more radios / USB ports and duplicates central hardware.

## D.5 Next steps

- Define a node-ID'd packet + generalized collector data model (N segments,
  M joints) before committing to a topology.
- Prototype cross-node time sync and quantify residual skew vs. the angle accuracy
  the study needs.
- Pick topology by study scale: a BLE star (A) for a compact multi-segment montage
  on one operator; independent pairs (B) when modularity / isolation matters more.
