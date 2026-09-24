# Dual-IMU Goniometer — Static Stability Test Report

**Test dates:** 2026-09-23 and 2026-09-24 · **Hardware:** 2 × Arduino Nano 33 BLE Rev2 (BMI270), wired UART link · **Logger:** `knee_gui.py`

> Detailed analysis, per-run tables and method notes are in [FINDINGS.md](FINDINGS.md).
> This report summarises what the tests showed.

## Summary

With nothing moving, the goniometer's reading stays still. Jitter is about
**0.01°**, and on a stable setup the reading changed by **less than 0.1° over
5 minutes**, with no detectable drift. Every sample in all six runs was valid.

Testing also turned up three problems to fix before dynamic (movement) testing:

1. **The sample rate is ~40 Hz, not the configured 50 Hz**, and ~20 % of CSV rows
   are duplicates. The static results are unaffected.
2. **The ruler-side node's sensor fails after ~20 minutes of running.** Its reads
   start hanging and returning garbage, and a few minutes later it stops producing
   data. Seen twice, both times with the board untouched.
3. **Only one of the two sensors was exercised** by these tests, because of how
   calibration works when one node never tilts.

## Purpose

To check **stability, not accuracy**: if nothing moves, does the measured angle
also stay unchanged? No reference angle was used, so absolute accuracy is out of
scope.

## Setup

| | Session 1 (runs 1–3) | Session 2 (runs 4–6) |
|---|---|---|
| Date | 2026-09-23 | 2026-09-24 |
| Rig | Both nodes on a flat desk | Raised rig: desk node ~1 cm higher, ~5 cm apart, roughly in line |
| Thigh node | Taped to the desk | Taped to the rig |
| Shank node | Taped to a ruler | Taped to a ruler |
| Firmware | Original | Emit-timer change (no effect, see issue 1) |
| Duration | ~5 min per run | ~5–5.5 min per run |

**Procedure (every run):**
1. **Zeroing, 2 s:** both nodes held still. This pose becomes 0°.
2. **Calibration sweep, 6 s:** the ruler rotated ~90° and returned, so the software
   learns the direction it bends in.
3. **Hold, ~5 min:** nothing touched.

The first 20 s (calibration plus setting the ruler down) are excluded from the analysis.

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

\*Session 2 SD with the slow trend removed, so it measures jitter rather than the creep.

- **Session 1 (flat desk):** the reading moved by 0.06° at most over 5 minutes,
  and the drift direction flips between runs. That's noise, not drift.
- **Session 2 (raised rig):** the reading crept by 0.1–0.5°. Run 5 has two sudden
  steps. The desk node, which isn't part of the angle calculation, moved at the
  same time, so the creep most likely comes from the rig or wires settling rather
  than the sensor. Short-term jitter was the same as in session 1.
- **Resting angle** is where the ruler came to rest after the calibration sweep,
  relative to where it was zeroed. It reflects placement, and it stays constant
  during the hold.

### Noise does not grow with time

Allan deviation shows how much averages over a time window τ differ from one
window to the next. If it stayed flat or fell as τ grew, there's no drift at
those time scales.

| Averaging window τ | Session 1 | Session 2 |
|---|---|---|
| 0.1 s | 0.003° | 0.003° |
| 1 s | 0.005° | 0.005–0.007° |
| 60 s | 0.004–0.007° | — (rig creep dominates) |

Session 1 is flat from 10 s to 60 s, so no drift appears over these time scales.

### Other checks

- **Heading drift doesn't affect the angle.** Each sensor's heading (compass
  direction) drifted by 0.2–5 °/min, as expected without a magnetometer, and the
  angle ignored it in both sessions. That's by design.
- **Calibration copes with an imperfect setup.** In session 2 the ruler node was
  zeroed 4–15° off level, and the sweep was shorter (~55°) and in the opposite
  direction. Calibration still worked and the reading was just as steady.
- **Link reliability:** 100 % valid samples in all six runs, no dropouts.
- **Longest clean hold so far:** the first ~18 minutes of the 40-minute run held
  at 0.80° with 0.016° jitter and −0.001 °/min drift, before the sensor fault
  (issue 2) began.

## Issues found

### 1. Sample rate is ~40 Hz, not 50 Hz

| | Rate |
|---|---|
| Configured | 50 Hz |
| Produced by the central board | ~43.5 Hz (one sample every ~23 ms) |
| Unique samples in the CSV | ~40.5 Hz |
| CSV rows repeating the previous sample | 19–20 % |

**Cause.** The central board takes ~23 ms per output cycle instead of 20 ms. The
GUI writes a row every 20 ms whatever it has received, so when no new sample has
arrived it writes the previous one again. Repeated rows share the board's
timestamp (`t_thigh_us`), which confirms they are copies of the same measurement.

**Effect on these results: none.** With duplicates removed, every run's mean, SD
and drift are identical to within 0.0002°. For movement data it would matter:
repeated values, dropped samples, and row times up to ~20 ms off.

**Measured cause.** The diagnostic build timed each ~23 ms cycle: ~10.7 ms
reading the thigh IMU and ~11.7 ms printing the data line as ~36 separate
prints. Sending a longer line as one write took ~0.7 ms.

**Status.**
- A first firmware fix to the output timer had no effect, which showed the
  problem is inside each output cycle.
- **Fix made, awaiting a test run:** the central now builds each data line in
  memory and sends it with one write, which should bring the cycle to ~11–12 ms
  and the rate to a true 50 Hz.
- Still planned: write one CSV row per received sample, so duplicates can't happen.

### 2. The ruler-side node's sensor fails after ~20 minutes

Seen in two recordings with the boards untouched: a 1-minute no-motion
recording, and a ~40-minute unattended run with the diagnostic build (both
boards on USB power). The diagnostic run shows the sequence:

| Board uptime | What the ruler node reported |
|---|---|
| 0–20 min | Healthy: 100 % valid, jitter 0.016°, drift −0.001 °/min |
| 20.4 min | Individual sensor reads start **hanging for ~190 ms** and returning garbage (accelerometer up to 4.4 g, two axes stuck at −1 count) |
| 21–26 min | Hung reads more frequent; gyro reads 1500–2400 °/s while still, so the orientation spins and the angle swings by tens of degrees |
| 26.3 min | The sensor **stops producing data**. Each restart attempt reports success but takes ~1.9 s, and no sample ever follows |
| 26–41 min | Board still running and talking to the central; its sensor silent |

- **Not the link:** 0 checksum failures and 0 corrupted bytes for the whole run,
  and the ruler board's health reports kept arriving.
- **Not a board reset or power loss:** its uptime counter never restarted.
- **Not calibration:** the fault is between the ruler board's processor and its
  own sensor. The desk node's sensor stayed perfect for all 41 minutes.

The likely cause is a hardware problem on that board (the sensor, its
connections, or heat), but a firmware cause isn't ruled out yet. Swapping the
two boards' roles will tell: if the fault stays with the same physical board,
it's that board.

**Firmware protection (made, awaiting a test run):** the ruler board now
rejects implausible samples and sends them as gaps instead of wrong angles,
resets its filter after each sensor restart, and reboots itself if its sensor
produces nothing usable for 5 s.

### 3. Only the ruler node was tested

The angle is the difference between the two nodes' tilts, but the software only
uses a node's tilt if it moved at least 5° during the calibration sweep. The desk
node never did, so it was treated as fixed and the angle came from the ruler node
alone. Its own data shows it was just as stable (jitter ~0.01°, no drift), but a
future test should tilt both nodes during calibration.

## Limitations

- **Stability only, not accuracy.** There was no reference angle, and resting
  angles were at most 6.4°, too small to reveal scale errors.
- **Jitter is limited by the log format.** Angles are logged to 0.01°, so the
  ~0.01° jitter is an upper bound; the true sensor noise is probably lower.
- **Mostly 5-minute holds.** One clean 18-minute stretch exists; longer drift
  isn't measured yet because of the sensor fault.
- **Rig vs. sensor.** Where the rig moved (session 2), its movement can't yet be
  separated from sensor drift. The GUI now logs the raw accelerometer, which
  will allow that.

## Next steps

| Step | Status |
|---|---|
| Run the diagnostic build past the ~15 min mark | Done (~40 min run) |
| Log the raw accelerometer in the CSV | Done (`knee_gui.py`) |
| Speed up the output cycle (one write per line) | Done: flash both boards and test |
| Guard the ruler node against a failing sensor | Done: flash both boards and test |
| Swap the boards' roles to see if the fault follows the hardware | Next test |
| Write one CSV row per received sample | To do |
| Log angles with more decimal places | To do |
| Stiffen the raised rig (clamp the ruler, fix both nodes rigidly) | To do |
| Tilt both nodes during calibration | Next test |
| Static test at known angles (30°, 45°, 90°) for accuracy | Next test |
| Longer static run (30–60 min) | Next test |

## Files

| File | Contents |
|---|---|
| [FINDINGS.md](FINDINGS.md) | Full analysis and per-run detail |
| `plot_report.py` | Recreates the figure above (needs the six CSVs in this folder) |
| `Static_Stability_Test_Report.docx` | Word version of this report (rebuild with `build_report_docx.js`) |
| `stats.py`, `calibration.py`, `plot.py`, `plot_session2.py` | Detailed statistics and plots |
