# Static stability test — findings

**Date collected:** 2026-09-23  **Logger:** `knee_gui.py` (CSV session output)
**Files:** `knee_static_1.csv`, `knee_static_2.csv`, `knee_static_3.csv` (≈5 min each)

## Headline

At rest the prototype is **very stable**: jitter is ~0.01° SD, drift is effectively
zero (≤0.005°/min), and there were no dropouts. Between runs the resting value
differed by up to 0.33°, which is most likely how the ruler was set back down,
not sensor error. **Caveat:** in this setup only the ruler-side (shank) node
contributes to the angle — see [§4.4](#44-only-the-shank-node-was-actually-tested).

## 1. Setup

- **Thigh node** taped to the desk (never moves).
- **Shank node** taped to a ruler lying flat on the desk.
- Calibration (same for every run, see `knee_gui.py` / `knee_collector_uart.py`):
  1. **Zeroing** (0–2 s): both nodes flat and still → each node's zero gravity
     direction `d_i`.
  2. **Sweep** (2–8 s): ruler rotated ~90° and returned → each node's in-plane
     forward axis `f_i` (only learned if the node tilts > 5°).
  3. **Running** (≥ 8 s): angle = `incl_thigh − incl_shank`.
- Ruler then left flat on the desk for the remainder of the 5 min.

The two nodes are not exactly coplanar (the ruler raises one), but the angle is
relative to the zeroing pose, so that constant offset is calibrated out.

## 2. Method

- **Trim:** analysis uses `t_session_s ≥ 20 s`, excluding the calibration and the
  return from the sweep (see §4.3 for whether 20 s is enough).
- **Metrics per run:**
  - **Mean** — offset from the calibrated zero (return-to-zero error).
  - **SD** — short-term jitter.
  - **Min–max range** — worst-case spread.
  - **Linear drift** — slope of a least-squares line (°/min) and total change over
    the window.
  - **Detrended SD** — noise after removing the drift line.
  - **30 s block means / SDs** — low-frequency wander.
  - **Allan deviation** — how noise averages down across time scales
    (computed on de-duplicated source samples, see §4.6).
  - **Settling time** — last time the angle was more than 0.05° / 0.1° away from the
    steady-state mean.
  - **Link / timing health** — `status`, `rtt_us` (shank packet age), row spacing,
    duplicated `t_thigh_us`.

Reproduce with `stats.py` and `plot.py` in this folder (place the three CSVs next
to them; needs `numpy pandas matplotlib`).

## 3. Results

### 3.1 Steady-state stability (t ≥ 20 s)

| Metric | Run 1 | Run 2 | Run 3 |
|---|---|---|---|
| Samples | 15 038 | 15 099 | 15 039 |
| Mean (offset) | −0.298° | −0.020° | +0.031° |
| SD | 0.015° | 0.011° | 0.011° |
| Min / max | −0.34° / −0.16° | −0.05° / +0.02° | −0.01° / +0.08° |
| 2.5th–97.5th percentile | −0.32° … −0.26° | −0.04° … 0.00° | +0.01° … +0.05° |
| Drift slope | −0.0054 °/min | −0.0003 °/min | +0.0013 °/min |
| Total drift over window | −0.027° | −0.002° | +0.007° |
| Detrended SD | 0.013° | 0.011° | 0.011° |
| 30 s block SD (typical) | 0.004–0.011°* | 0.005–0.012° | 0.005–0.012° |
| Spread of 30 s block means | −0.27 … −0.31° | −0.01 … −0.04° | +0.02 … +0.04° |

\*Run 1's first block (20–50 s) had SD 0.024° because of the slow settling in §4.3.

**Across runs:** SD of the three run means = 0.18°, range = 0.33° (run 1 is the
outlier; runs 2 and 3 agree within 0.05°).

### 3.2 Allan deviation

| Averaging time τ | Run 1 | Run 2 | Run 3 |
|---|---|---|---|
| 0.1 s | 0.0027° | 0.0027° | 0.0027° |
| 1 s | 0.0048° | 0.0045° | 0.0045° |
| 10 s | 0.0065° | 0.0063° | 0.0049° |
| 30 s | 0.0061° | 0.0045° | 0.0053° |
| 60 s | 0.0070° | 0.0038° | 0.0056° |

Flat at ~0.005–0.007° from 10 to 60 s: no random-walk / bias-instability growth is
visible at these time scales. (The rise from 0.1 s to 1 s mostly reflects
the 0.01° output quantisation — see §4.1.)

### 3.3 Settling after calibration

| | Run 1 | Run 2 | Run 3 |
|---|---|---|---|
| Within 0.1° of final mean after | 20.5 s | 14.3 s | 16.2 s |
| Within 0.05° of final mean after | 23.1 s | 14.5 s | 16.2 s |

### 3.4 Link and timing

| | Run 1 | Run 2 | Run 3 |
|---|---|---|---|
| Valid samples (running phase) | 100 % | 100 % | 100 % |
| Shank packet age ≤ 100 µs | 63 % | 63 % | 63 % |
| Shank packet age 5–15 ms | 34 % | 36 % | 37 % |
| Shank packet age > 15 ms (≈22 ms) | 2.4 % | 1.4 % | 0.5 % |
| Unique source samples / s | 40.5 | 40.5 | 40.2 |
| Duplicate rows (same `t_thigh_us`) | 19 % | 19 % | 20 % |

![Static test plots](static_test_plots.png)

*Top: return from the 90° sweep at the start of the running phase. Middle: raw
steady-state signal. Bottom: 5 s rolling mean.*

## 4. Observations

### 4.1 Noise is at the logging resolution
`knee_angle_deg` is written with 2 decimals and ~90 % of consecutive samples are
identical; the quaternion columns (4 decimals) resolve only ~0.01° as well. The
measured SD of ~0.011° is therefore an **upper bound** — the true sensor noise is
probably lower. Log more decimals (e.g. `.4f` for the angle, `.6f` for quaternions)
to measure the real noise floor.

### 4.2 No meaningful drift
Linear drift over ~5 min is ≤ 0.03° in every run, and 30 s block means stay within
±0.02° of each other once settled. Over the tested duration the gravity-referenced
angle does not drift.

### 4.3 Settling time: 20 s trim is borderline
The ruler was still returning from 90° when the running phase began at t = 8 s
(readings of −20° to −47°), so the first seconds of "running" are motion, not
static data. After the return:
- Run 2 settled by ~14.5 s, run 3 by ~16 s. Run 3 sat at ≈ +0.5° from 12–16 s and
  then stepped down — looks like the ruler/tape shifting, not the filter.
- **Run 1 kept creeping ≈ 0.07° from 20 s to ~50 s** (5 s means: −0.23 → −0.29°)
  before levelling off.

Recommendation: trim ~40 s, or better, trim with a rule — start the analysis window
once the 5 s rolling mean stays within 0.05° of the final value.

### 4.4 Only the shank node was actually tested
`incl_thigh_deg` is exactly 0.00 for every running-phase sample. The desk node
never tilted > 5° during the sweep, so `estimate_forward` returned `None` and the
code treated the thigh as fixed (knee = −shank inclination). The reported angle
therefore reflects **only the ruler node's** gravity tilt; the desk node's noise
and drift are not in these numbers. To characterise the full two-sensor difference,
tilt both nodes during the sweep.

### 4.5 Offset vs. true error
Run 1's −0.30° offset is ~20× its SD, so it is a real difference in resting pose,
not noise. Most likely the ruler came back to a slightly different position (tape
compliance, ruler edge, desk contact). Without an independent reference this cannot
be separated from sensor return-to-zero error — a fixture with a hard stop would
allow that.

### 4.6 Heading (yaw) drift does not leak into the angle
Yaw from the raw quaternions drifts ~5 °/min on the thigh node and 1.7–3.5 °/min on
the shank node (expected for 6-DOF fusion with no magnetometer), and the two nodes
drift at different rates. The knee angle is flat regardless — this directly
confirms the method's yaw-invariance by design.

### 4.7 Timing
- No dropouts or forward-filled samples in any run.
- The CSV is written on a 50 Hz grid, but new thigh samples arrive every ~22.9 ms
  (~43.6 Hz; ~40.5 unique/s), so ~19 % of rows repeat the previous sample.
  **De-duplicate on `t_thigh_us` before any spectral / Allan analysis.**
- Shank packet age is mostly fresh (µs) with a small tail at one packet period
  (~22 ms).
- Board clock vs PC wall clock differed by ~0.22 % (300.07 s vs 300.73 s). Irrelevant
  here, but matters for long synchronised recordings.

## 5. Summary numbers for reporting

| Quantity | Value |
|---|---|
| Static jitter (SD) | 0.011–0.015° (upper bound; logging-limited) |
| Drift | ≤ 0.005 °/min (≤ 0.03° over 5 min) |
| Allan deviation, τ = 1 s / 60 s | ≈ 0.005° / ≈ 0.004–0.007° |
| Run-to-run repeatability of resting value | 0.33° range (SD 0.18°) |
| Settling after calibration movement | 14–23 s to within 0.05° |
| Data validity | 100 %, no dropouts |

## 6. Suggested next tests

1. **Static at known non-zero angles** (e.g. 30°, 45°, 90° on a fixture) — a zero
   test cannot reveal scale-factor or cross-axis error.
2. **Tilt both nodes during calibration** so the thigh node contributes to the angle.
3. **Repeated calibrations without disturbing the setup** — separates calibration
   repeatability from repositioning error.
4. **Higher-precision logging** (§4.1) to measure the true noise floor.
5. **Longer static run** (30–60 min) to check drift and Allan deviation at longer τ.
