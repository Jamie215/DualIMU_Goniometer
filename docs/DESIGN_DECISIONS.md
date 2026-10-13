---
title: "Dual-IMU Knee Goniometer — Design Decisions"
subtitle: "Compilation of the design choices, their rationale, and what was done"
---

# About this document

This document collects, in one place, the design decisions made during the
dual-IMU knee goniometer proof of concept (August–October 2026): what was
decided, why, which alternatives were considered, and where the evidence
lives. It is a companion to:

- `README.md` — the technical reference (math, wire protocol, commands).
- `docs/SOP_Build_and_Use.md` — how to build and operate the device.
- `docs/HANDOVER.md` — current state, open issues and next steps.
- `docs/areas_to_explore.md` — longer analyses of directions not pursued yet.
- `analysis/static_test/Static_Stability_Test_Report.docx` — the Preliminary
  Stationary Test Report, with `FINDINGS.md` as its detailed working record.

Each decision follows the same layout: **Decision**, **Why**,
**Alternatives considered**, **Evidence** (commits / documents), **Status**.
Commit hashes refer to this repository's history.

# 1. Problem and goals

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
are out of scope (see §4.5 and §9).

# 2. System architecture

```
 shank board (peripheral)  --UART-->  thigh board (central)  --USB serial-->  PC
   BMI270, Mahony filter   <--keep--    BMI270, Mahony filter                 knee_gui.py / knee_collector_uart.py
                              alive                                           (calibration, angle, CSV)
```

## 2.1 Two-board central/peripheral topology

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

## 2.2 Hardware: Arduino Nano 33 BLE Rev2

- **Decision:** Two Arduino Nano 33 BLE Rev2 boards, using the onboard
  **BMI270** accelerometer + gyroscope. The onboard **BMM150 magnetometer is
  not used.**
- **Why:** Off-the-shelf, inexpensive, IMU on board, Arduino tooling, BLE radio
  available for a future wireless version.
- **Status:** In use. One of the two boards has a faulty BMI270 (see §8.1).

## 2.3 Physical build as it stands

- **Decision:** Each board sits in a basic case. The boards are joined by
  **soldered wires** (two UART lines + ground). **Each board is powered by its
  own external USB supply**, and the central is connected to the PC over USB
  serial for data.
- **Why:** Fastest route to a working, reliable bench prototype.
- **Trade-offs:** The subject is tethered to the PC and to power; a wire runs
  across the knee. Battery power and wireless links are future work (§9).
- **Status:** Bench prototype. No strap or on-body mounting has been finalised.

# 3. Sensor fusion (on each board)

## 3.1 6-DOF Mahony filter, no magnetometer

- **Decision:** Each board estimates its orientation with a 6-DOF Mahony
  filter (accelerometer + gyroscope), `TWO_KP = 1.0`, `TWO_KI = 0.04`.
- **Why:** The angle method (§4) only needs the direction of gravity in each
  board's frame, which 6-DOF provides without drift. A magnetometer is
  disturbed by nearby metal (furniture, treadmills, the USB cable, the other
  board) and needs per-board hard/soft-iron calibration.
- **Alternatives:** A 9-DOF path with runtime magnetometer calibration and an
  automatic BMM150→BMI270 axis solver was built (`146cc14`, `87096d9`,
  `d79e51a`, `9cf21cb`) and then **removed** (`0dd3a93`) to get back to a clean,
  verifiable baseline — it was never confirmed on hardware, not measured and
  found worse. Analysis in `docs/areas_to_explore.md` Area 1 concludes it adds
  nothing for sagittal angles.
- **Status:** In use. 9-DOF code is recoverable from history if 3-D is ever needed.

## 3.2 Fixing the library's left-handed axes

- **Decision:** Negate the x axis of both accelerometer and gyroscope after
  reading them.
- **Why:** `Arduino_BMI270_BMM150` returns `x=-sensor.y, y=-sensor.x, z=sensor.z`
  — a reflection (determinant −1). A quaternion filter needs a right-handed
  frame; without the fix the gyro's rotation sense was mirrored and the filter
  wandered.
- **Evidence:** `6f1bdc7`; README finding #1; comment block at the top of both sketches.
- **Status:** In use. **Any new firmware must keep this.**

## 3.3 Gyro bias calibration with a sanity gate

- **Decision:** At the start of each collection, average ~300 gyro samples
  (up to 4 s) and subtract the result as bias, but **discard it** if any axis
  exceeds `BIAS_SANITY_DPS` = 3 °/s.
- **Why:** Bias is the main drift source in a 6-DOF filter. If the board is
  moving during calibration, the "bias" is a false rate that can make the
  filter diverge, so it is safer to use zero.
- **Evidence:** `fd0d53b`, `6f1bdc7`; README finding #2.
- **Status:** In use. Calibration happens when a collection starts (USB port
  opened), not at power-up (`86efe6d`).

## 3.4 Soft accelerometer-trust gate

- **Decision:** Weight the accelerometer correction by how close `|accel|` is
  to 1 g: full trust within 0.10 g (`ACC_TRUST_FULL_G`), ramping to none at
  0.60 g (`ACC_TRUST_ZERO_G`).
- **Why:** During brisk motion the accelerometer reads gravity *plus* linear
  acceleration. A hard on/off gate let the filter run gyro-only through a fast
  move, drift, then snap back — the "wrong reading right after returning from
  a big angle" symptom. A soft ramp always applies some correction.
- **Evidence:** `fd0fbaa` (hard gate), `4effe4c` (soft gate); README finding #7.
- **Status:** In use; not yet tuned on dynamic data.

# 4. Knee-angle method (PC side)

## 4.1 Gravity-referenced sagittal inclinometer

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
  `gravity_knee_angle()` in `knee_collector_uart.py`.

## 4.2 Gravity from the fused quaternion, not the raw accelerometer

- **Decision:** Read gravity from the Mahony quaternion; keep the raw
  accelerometer only as a filter-free cross-check.
- **Why:** The raw accelerometer is corrupted during motion; the fused
  quaternion lets the gyro carry fast movement while the accelerometer keeps
  tilt drift-free. (Raw accelerometer was briefly the default while the
  handedness bug made the quaternion wrong — README finding #4.)
- **Status:** In use. Raw accelerometer values are still streamed and logged
  (useful for checking physical movement, as in the stationary report).

## 4.3 Two-step calibration: static zero + functional sweep

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

## 4.4 Signed, continuous angle

- **Decision:** `incl = atan2(g·f, g·d)`: flexion positive, hyperextension
  negative, continuous through the full range with no ±90° wrap.
- **Why:** A signed four-quadrant angle distinguishes flexion from
  hyperextension and retraces identically on the way back from high flexion
  (verified in `--selftest` with a 0→130°→0 sweep, no stuck zeros on return).

## 4.5 Relative zero, not an absolute anatomical angle

- **Decision:** Accept that 0° is "the pose held during zeroing".
- **Why:** ROM, rep counting, gait and movement quality are difference-based,
  so the zero cancels. An absolute angle (e.g. a flexion contracture) needs an
  external reference that observes the anatomy; the sensors cannot supply it,
  and a magnetometer does not help (Area 1).
- **Alternatives:** Functional joint-axis calibration or a known-pose fixture
  (Area 2) — identified, not prototyped.
- **Evidence:** `91bc803`; README "What 0° means"; `docs/areas_to_explore.md` Areas 1–2.

## 4.6 Accepted limitation: sagittal plane only

Gravity gives 2 of 3 rotational degrees of freedom; rotation about vertical is
unobservable. The method therefore under-reads motion well out of the vertical
plane (e.g. lying down with large hip rotation). This is an observability
limit, accepted for upright knee flexion.

# 5. Board-to-board link and data protocol

## 5.1 Wired UART at 115 200 baud

- **Decision:** `Serial1` UART at 115 200 baud, both directions wired
  (peripheral TX → central RX for data; central TX → peripheral RX for the
  keepalive), common ground.
- **Why:** Simple and deterministic. The rate was briefly raised to 460 800 to
  shorten time on the wire, which masked a framing bug and added sensitivity to
  clock drift as boards warm up; 115 200 gives ~4× the per-byte timing margin
  and is far above the ~15 kbit/s needed.
- **Alternatives:** BLE (Area 3) — deferred.
- **Evidence:** `fd0fbaa`, `c767bc7`; README finding #6.

## 5.2 Framed binary packet with resync and checksum

- **Decision:** 30-byte packet: `0xAA` header, quaternion (4 floats),
  raw accel (3 floats), XOR checksum. The central resyncs on the header and
  drops packets that fail the checksum.
- **Why:** The original fixed-length read had no resync, so one lost byte
  shifted every following packet. Now a glitch costs one packet.
- **Evidence:** `c767bc7`.

## 5.3 Peripheral streams; central never waits

- **Decision:** The peripheral **streams** packets at 50 Hz; the central
  drains its UART buffer each loop and keeps the freshest valid packet. If that
  packet is older than `SHANK_STALE_US` = 30 ms the shank is reported invalid
  (zeros) and the PC forward-fills it. The packet age is the line's last field.
- **Why:** Polling with a deadline lost samples whenever the peripheral's RTOS
  stalled for a few ms; widening the deadline only stalled the central. Streaming
  decouples the two boards.
- **Evidence:** `3b97e85`, `7197327`, `4fc3a07`, `c111974`; README finding #6.

## 5.4 Collect-on-demand

- **Decision:** Both boards run their IMUs only while the central's USB port
  is open. On open, the central calibrates and sends `S` keepalives every
  200 ms; the peripheral idles if none arrives for 1 s, or on `X`.
- **Why:** Nothing runs unobserved; each session starts with a fresh bias
  calibration on a still board, rather than relying on a boot-time one.
- **Evidence:** `86efe6d`.

## 5.5 Fixed 50 Hz output, one write per line

- **Decision:** Both boards emit at a fixed 50 Hz (`EMIT_PERIOD_US` = 20 000)
  on a fixed schedule; the central builds each PC line in memory and sends it
  with **one** write. Filters still run at the sensor's native rate.
- **Why:** ~104 Hz left little timing margin and 50 Hz is ample for knee ROM.
  Testing showed the central actually ran at ~43.5 Hz because it printed each
  line as ~36 blocking prints (~11.7 ms); the one-write line takes ~1–1.8 ms
  and restored a true 50 Hz.
- **Evidence:** `8211f4a`, `5c8f67d`, `e02ae6d`; Stationary Test Report, Appendix.

## 5.6 Diagnostics channel

- **Decision:** With `DIAG 1` (the current default in both sketches) the
  peripheral sends a once-a-second health frame (`0xAB` header) that the central
  relays as `# PDIAG …`, and the central adds `# CDIAG …`. The GUI saves all `#`
  lines to `knee_diag_<timestamp>.log`; `analysis/diagnostics/parse_diag.py`
  turns them into tables.
- **Why:** Needed to find the sample-rate shortfall and the faulty sensor.
- **Constraint:** The `PDiag` struct must match byte for byte in both sketches;
  flash both boards from the same commit.
- **Evidence:** `72943ba`.

# 6. Robustness and fault handling

## 6.1 Peripheral self-healing

- **Low-rate start:** if fewer than 30 samples arrive in 1 s, re-initialise the
  sensor (the BMI270 occasionally starts at a wrong ~10 Hz rate) — `4036498`.
- **Data-ready stall:** if no sample for 250 ms, LED solid and re-init every
  500 ms — `5f8aaff`.
- **IMU init failure at boot:** fast-blink ~3 s, then reboot.

## 6.2 Sensor-fault guard

- **Decision:** On the peripheral, reject a sample if the read took > 50 ms,
  `|accel|` is < 0.25 g or > 4 g, or the gyro reads > 1000 °/s while `|accel|`
  is within 0.15 g of 1 g. Rejected samples are sent as an all-zero quaternion
  (a gap, not a wrong angle). Each re-init resets the filter's integral term.
  If no good sample arrives for 5 s, the board reboots.
- **Why:** In endurance runs one board's BMI270 began returning garbage after
  20–38 min and then stopped entirely; this turns wrong angles into visible gaps.
- **Note:** Only the peripheral has this guard. The central does not.
- **Evidence:** `e02ae6d`; Stationary Test Report, "Cause: one faulty board".

## 6.3 LED indicators

| Board | Pattern | Meaning |
|---|---|---|
| Central | On | Collecting (USB port open) |
| Central | Off | Idle |
| Peripheral | 3 quick flashes | Boot |
| Peripheral | Slow blink | Idle, waiting for keepalive |
| Peripheral | Fast blink (~3 Hz) | Active, streaming |
| Peripheral | Solid on | Sensor stalled (re-init firing) |
| Peripheral | Off / frozen | Unpowered or hard fault |

# 7. PC software

## 7.1 CLI collector plus a live GUI

- **Decision:** `knee_collector_uart.py` holds the math, calibration, CSV logic
  and `--selftest`; `knee_gui.py` (Tkinter + Matplotlib) reuses it and adds port
  scanning, a colour-coded calibration banner, live plots, and auto-saved sessions.
- **Why:** One tested implementation of the math; a GUI makes test sessions
  repeatable for non-developers.
- **Evidence:** `9ef5550`, `d15c4db`, `1a79b4d`, `a0e852a`.

## 7.2 Session-based collect / stop

- **Decision:** Each "Calibrate & collect" starts a new session with fresh
  calibration and a new timestamped CSV; "Stop collecting" freezes the plot on
  the full session for inspection. No pause/resume.
- **Why:** Avoids mixing calibrations in one file; makes every file self-contained.

## 7.3 One CSV row per received sample

- **Decision:** Write exactly one row per line from the central, timed by the
  board's clock (`t_session_s`), plus the PC receive time (`t_wall_iso`).
- **Why:** The GUI used to write rows on its own 20 ms timer, which ticks
  unevenly on Windows: ~20 % of rows repeated a sample and others were skipped.
- **Evidence:** `8a9aaf4`; Stationary Test Report, Appendix.

## 7.4 Dropouts: forward-fill vs gap

- **Decision:** Short gaps (≤ `MAX_FILL` = 10 samples) are forward-filled;
  longer gaps are marked missing. The CSV records the status of every sample
  (`valid / filled / missing`), and the GUI can display either a filled line or
  discrete points.
- **Why:** Keeps plots readable without hiding where data was real.

## 7.5 Clear error diagnosis

`diagnose_stream()` names the failing link: no data on the port, wrong/old
firmware (not an 18-field `D` line), or a dead peripheral (all shank samples
invalid). The CLI and GUI share it.

## 7.6 Windows executable, released on version tags

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

# 8. Testing and validation

## 8.1 Preliminary stationary testing (Sept 2026)

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

## 8.2 Earlier bench checks

Rigid-ruler test (both boards on one ruler, true relative angle constant) was
the key diagnostic for drift; the right-angle placement read ~90°. Math is
covered by `knee_collector_uart.py --selftest` and `knee_gui.py --selftest`.

## 8.3 Not yet tested

Both-node calibration; dynamic movement; on-body use; controlled accuracy
against a goniometer or precise rig; noise below the 0.01° log resolution.

# 9. Directions considered but not pursued

Detailed in `docs/areas_to_explore.md`.

| Area | Recommendation |
|---|---|
| 1. Magnetometer fusion | Do not add for sagittal angles; only for a true 3-D requirement, after checking the magnetic environment. |
| 2. Absolute anatomical angle | Functional joint-axis calibration or a known-pose reference; not prototyped. |
| 3. BLE (wireless) links | Start with the board-to-PC link; needs source timestamps and batteries. Main benefit is safety / REB approval. |
| 4. More nodes / joints | Needs node IDs, a generalised data model and cross-node time sync; BLE suits a star topology. |

# 10. Timeline of major milestones

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
| Oct 9–13 | Handover docs: SOP, design decisions, handover summary | this branch |
| Oct 13 | Windows exe build workflow; release v1.0.0 | `572b164` |
