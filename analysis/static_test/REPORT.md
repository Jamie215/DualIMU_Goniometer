# Dual-IMU Goniometer — Static Test Report

**Test dates:** 2026-09-23 and 2026-09-24 · **Hardware:** 2 × Arduino Nano 33 BLE Rev2 (BMI270), wired UART link · **Logger:** `knee_gui.py`

> Detailed analysis, per-run tables and method notes are in [FINDINGS.md](FINDINGS.md).

## Purpose of this document

This document records the static testing of the dual-IMU goniometer prototype:
what was tested, how, what the results showed, which problems turned up along the
way and how they were addressed. It closes with conclusions and recommended next
steps for anyone continuing the work.

## What we are testing

- **Stability:** if nothing moves, does the measured angle also stay unchanged?
  This covers short-term jitter, slow drift over minutes to an hour, and data
  quality over long sessions.
- **Preliminary angle check:** do fixed 45° and 90° rigs read close to nominal?
  The rigs weren't precise enough for a formal accuracy figure, so this part is a
  preliminary inspection only.

## Setup

| | Session 1 (runs 1–3) | Session 2 (runs 4–6) | Long runs (3 runs) | Angle rig (45°, 90°) |
|---|---|---|---|---|
| Date | 2026-09-23 | 2026-09-24 | 2026-09-24 | 2026-09-24 |
| Thigh node | Taped to the desk | Taped to a raised rig | Taped to the rig | On the flat upper segment |
| Shank node | Taped to a ruler | Taped to a ruler, ~1 cm lower, ~5 cm apart | Taped to a ruler | On the angled lower segment |
| Duration | ~5 min per run | ~5–5.5 min per run | 25–65 min, unattended | ~20–25 s hold each |

All nodes were powered over USB.

**Procedure (every run):**
1. **Zeroing, 2 s:** both nodes held still. This pose becomes 0°.
2. **Calibration sweep, 6 s:** the shank node rotated and returned, so the
   software learns the direction it bends in.
3. **Hold:** nothing touched. For the angle rig, the shank segment was set at the
   rig angle and held.

The first 20 s (calibration plus setting the node down) are excluded from the
stability analysis.

## Results

### The reading stays still when nothing moves

![Change in angle during each 5-minute hold](report_stability.png)

*Change in the measured angle during each hold (5 s rolling average), relative
to the first 10 s of the hold.*

| Run | Resting angle | Jitter (SD) | Change over the hold | Drift |
|---|---|---|---|---|
| 1 | −0.30° | 0.015° | −0.06° | −0.005 °/min |
| 2 | −0.02° | 0.011° | −0.03° | −0.0003 °/min |
| 3 | +0.03° | 0.011° | −0.01° | +0.001 °/min |
| 4 | +6.38° | 0.016°\* | +0.16° | +0.033 °/min |
| 5 | −1.27° | 0.047°\* | +0.49° (two steps) | +0.082 °/min |
| 6 | −0.84° | 0.012°\* | +0.11° | +0.019 °/min |
| 1-hour hold | +0.23° | 0.018° | ~0.06° over 57 min | −0.001 °/min |

\*Session 2 SD with the slow trend removed, so it measures jitter rather than the creep.

- **Session 1 (flat desk):** the reading moved by 0.06° at most over 5 minutes,
  and the drift direction flips between runs. That's noise, not drift.
- **Session 2 (raised rig):** the reading crept by 0.1–0.5°, and run 5 has two
  sudden steps. The desk node, which isn't part of the angle calculation, moved at
  the same time, so the creep most likely comes from the rig or wires settling.
  In a later run, the raw accelerometer confirmed that a similar creep was the
  ruler physically tilting.
- **1-hour hold** (from the board-swap run, where the angle came from the healthy
  board): 57 minutes at 0.23° with 0.018° jitter. The 5 s averages stayed within
  a 0.1° range. This is the longest clean static record.
- **Resting angle** is where the node came to rest after the calibration sweep,
  relative to where it was zeroed. It reflects placement, and it stays constant
  during the hold.

### Noise does not grow with time

Allan deviation shows how much averages over a time window τ differ from one
window to the next. If it stays flat or falls as τ grows, there's no drift at
those time scales.

| Averaging window τ | Session 1 | Session 2 |
|---|---|---|
| 0.1 s | 0.003° | 0.003° |
| 1 s | 0.005° | 0.005–0.007° |
| 60 s | 0.004–0.007° | — (rig creep dominates) |

Session 1 is flat from 10 s to 60 s, and the 1-hour hold drifted only
−0.001 °/min, so no sensor drift shows up at any time scale tested.

### Preliminary angle check: 45° and 90° rigs

| Rig | Reading (steady part) | Diff from nominal | Jitter (SD) | Raw-accelerometer check |
|---|---|---|---|---|
| 45° | 45.0° | 0.0° | 0.034° | 45.2° |
| 90° | 89.0° | 1.0° | 0.093° | 89.0° |

The GUI shows these as negative angles because of the bending direction the
calibration learned.

- **The shank node's tilt is measured correctly.** The raw accelerometer, which
  doesn't depend on the filter or the calibration, agrees with the reported
  angle within about 0.2°. So the calibration learned the right bending
  direction, and a 90° hold is as steady as a 0° hold.
- **Why this is preliminary:**
  - **One placement each.**
  - **The reference (thigh) node shifted 2.7° and 4.5° during placement,**
    probably pulled by its cable. The software ignores that node because it
    didn't tilt during calibration. If the upper segment stayed flat, as
    intended, the readings above stand. If the segment itself moved, the true
    angle between the segments could differ by up to ~2–3°.
  - The data suggests the board shifted on its segment rather than the segment
    rotating, since it tilted about as much across the bending plane as along
    it. But this can't be confirmed.

### Other checks

- **Heading drift doesn't affect the angle.** Each sensor's heading (compass
  direction) drifted by 0.2–5 °/min, as expected without a magnetometer, and the
  angle ignored it in every run. That's by design.
- **Calibration copes with an imperfect setup.** In session 2 the ruler node was
  zeroed 4–15° off level, and the sweep was shorter (~55°) and in the opposite
  direction. Calibration still worked and the reading was just as steady.
- **Link reliability:** 100 % valid samples in all six short runs. The long runs
  had no checksum failures while the boards were healthy, apart from 2 at start-up
  in one run.

## Issues found

### 1. Sample rate was ~40 Hz, not 50 Hz — fixed

| | Before | After the fixes |
|---|---|---|
| Central board's cycle | ~23 ms (~43.5 Hz) | 20.0 ms (50 Hz) |
| Printing each data line | ~11.7 ms (~36 separate prints) | ~1.0–1.8 ms (one write) |
| Unique samples in the CSV | ~40.5 per second | ~50 per second |
| CSV rows repeating the previous sample | 19–20 % | 0 % |

**Cause.** Two separate problems:
- **The central board ran slow.** Each output cycle spent ~10.7 ms reading its
  IMU and ~11.7 ms printing the data line as ~36 separate blocking prints.
- **The GUI wrote on its own timer.** It wrote a row every 20 ms whatever had
  arrived, and on Windows that timer ticks unevenly, so rows repeated some
  samples and skipped others.

**Fixes.**
- The central now builds each line in memory and sends it with one write.
- The GUI now writes exactly one row per sample received, timed by the board's
  own clock.

Both were confirmed in the hour-long run: 50 rows/s, 0 duplicates, rows 19.9 ms
apart.

**Effect on these results: none.** With duplicates removed, every run's mean, SD
and drift are identical to within 0.0002°. The fix matters for movement data.

### 2. Poor data quality in long sessions: traced to one faulty board

In long sessions, data quality from one node would break down after 20–47 minutes.
The evidence points to a hardware fault on one of the two Arduino boards as the
most likely cause.

**What happened.** On that board, individual sensor reads started **hanging for
~190–200 ms** and returning garbage while the board sat still: accelerometer up to
4.4 g, gyro up to 2400 °/s. This made the reported orientation spin and the angle
swing by tens of degrees. A few minutes later the sensor **stopped producing data
entirely**. Restarting the sensor, or rebooting the board, didn't bring it back.

| Recording | That board's role | Sensor trouble started | Sensor stopped |
|---|---|---|---|
| No-motion test | Shank (ruler) | before 26 min | ~26 min |
| Long run 1 | Shank | 20.4 min | 26.3 min |
| Long run 2 | Shank | 24.8 min (abrupt) | 24.8 min; still dead after the automatic reboot |
| Long run 3, **roles swapped** | Thigh (central) | 6.5 min (missed updates); 37.8 min (hung reads) | 47.3 min |

**How it was traced.**
- **Swapping the boards' firmware roles moved the fault with the board.** In long
  run 3, the board that had been failing as the shank node failed again as the
  thigh node. The other board ran the shank role for over an hour with 0 bad
  readings. Across all runs, the other board never had a sensor problem in either
  role.
- **Ruled out:**
  - **The link:** no checksum failures before the fault in any run (apart from 2
    at start-up in one).
  - **Power loss or a board reset:** its uptime counter never restarted by itself.
  - **Calibration:** unrelated to how or whether the node moved.
  - **The firmware:** both firmware roles failed on this board, and neither failed
    on the other.
- Its cable, USB port and position stayed with it during the swap, so those
  weren't separately excluded. A faulty sensor chip or joint on the board itself
  remains the most likely explanation.

**Mitigation added along the way.** The shank firmware now rejects implausible
sensor samples and sends them as gaps instead of wrong angles. It also resets its
filter after each sensor restart, and reboots the board if the sensor produces
nothing usable for 5 s. In long run 2 the guard caught the bad readings and the
reboot fired, but the sensor stayed dead. That's further evidence the fault was in
the hardware.

## Limitations

- **Only one node was exercised in the angle.** The angle is the difference
  between the two nodes' tilts, but the software only uses a node's tilt if it
  moved at least 5° during the calibration sweep. In every test only the shank
  node was swept, so the thigh node was treated as fixed and the angle came from
  the shank node alone. The thigh node's own data shows it was just as stable, but
  its contribution to the angle wasn't tested.
- **"Valid %" only reflects the shank node's data.** A failure of the thigh node
  doesn't show up in the valid % or, with the calibration above, in the angle. In
  long run 3 the failing board was the thigh node, and validity stayed ~100 % while
  its sensor died. It only showed in the diagnostic log and the CSV's
  `thigh_ax..az` columns.
- **Accuracy is preliminary only.** The 45°/90° check was one placement each, and
  the reference node shifted 2.7–4.5° during placement.
- **Jitter is limited by the log format.** Angles are logged to 0.01°, so the
  ~0.01° jitter is an upper bound; the true sensor noise is probably lower.
- **Rig vs. sensor.** Where the rig moved (session 2), its movement couldn't be
  separated from sensor drift at the time. The GUI now logs the raw accelerometer,
  which does that; it confirmed a later creep was physical.

## Conclusions and next steps

**What was tested.** Six ~5-minute static holds on two setups (a flat desk and a
raised rig), three long unattended sessions of 25–65 minutes with diagnostic
firmware, a board-swap session, and a preliminary check on 45° and 90° rigs.

**What the tests showed.**
- On healthy hardware the angle is very stable at rest:
  - Jitter is about 0.01–0.02°.
  - Drift was undetectable over 5 minutes and −0.001 °/min over a clean
    57-minute hold (about 0.06° in total).
  - Heading drift in the sensors didn't leak into the angle.
- Calibration worked even with a tilted zero pose and a short, reversed sweep.
- At the 45° and 90° rigs the shank node read 45.0° and 89.0°, and its
  filter-free accelerometer agreed within ~0.2°. This is encouraging, but only a
  preliminary result.

**What was found and addressed.**
- **Sample rate:** the system ran at ~40 Hz instead of 50 Hz, with ~20 % duplicate
  CSV rows. Two fixes brought it to a true 50 Hz with 0 duplicates:
  - The central board now sends each line in one write.
  - The GUI now writes one row per received sample, timed by the board's clock.
- **Long-session data loss:** traced to a faulty sensor on one Arduino board.
  Firmware now limits the damage by turning bad samples into gaps and rebooting
  the node, but it can't repair the hardware.
- **Diagnostics added:**
  - Once-a-second health reports from both boards, saved to a log.
  - Raw accelerometer columns in the CSV, which separate physical movement from
    filter behaviour.

**Recommended next steps.** Anyone continuing this work should first replace the
faulty board and repeat a one-hour static run on two healthy boards, to get a
clean baseline. From there:
- Sweep **both** nodes during calibration so the angle uses both sensors.
- Fix the reference node and its cable firmly so it can't shift.
- If a more precise rig is available, repeat the 0°, 45° and 90° placements
  several times each to turn the angle check into an accuracy figure.
- Log angles to more decimal places to measure the true noise floor.
- Move on to dynamic (movement) testing, now that the sample rate and CSV timing
  are sound.

## Files

| File | Contents |
|---|---|
| [FINDINGS.md](FINDINGS.md) | Full analysis and per-run detail |
| `plot_report.py` | Recreates the figure above (needs the six CSVs in this folder) |
| `Static_Stability_Test_Report.docx` | Word version of this report, generated from this file by `build_report_docx.js` |
| `stats.py`, `calibration.py`, `plot.py`, `plot_session2.py` | Detailed statistics and plots |
