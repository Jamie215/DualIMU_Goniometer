# Calibration Redesign — Separated Hip + Knee Protocol

**Status:** implemented on `claude/gracious-shannon-ypu8le` (separated hip/knee
protocol; old combined single-sweep path removed; zero hold 5 s, sweep timeout
12 s per user request). The section below is the original proposal, kept as the
design record; the "Parameters" table reflects the shipped defaults.
**Branch:** `claude/gracious-shannon-ypu8le`
**Scope:** calibration only. The runtime angle math (`gravity_knee_angle`) and the
serial/CSV formats are **unchanged**, so this is a low-risk change to *how* the
per-segment axes `d_i`, `f_i` are learned — not what they feed.

---

## 1. Problem

Three complaints, one root cause each.

### 1a. Hip movement leaks into the knee angle
`knee = incl_thigh − incl_shank` should cancel hip flexion (it tilts the thigh,
which is captured by `incl_thigh` and subtracted). It doesn't, because:

- **Sagittal-projection blindness.** `incl_i = atan2(g·f_i, g·d_i)`
  (`knee_collector_uart.py:225`) reads only the tilt *inside the learned (d,f)
  plane*. It is invariant to world-yaw and to a fixed mount, but **not** to real
  out-of-plane hip motion — abduction and especially thigh internal/external
  (axial) rotation. Axial thigh rotation swings gravity's in-plane component off
  `f_thigh`, so `incl_thigh` mis-reads and the error lands straight in the knee
  difference. This is the *observability* limit the README already notes
  (README lines 204–212); two gravity-only IMUs cannot fully remove it.
- **`f_thigh` is badly conditioned.** `estimate_forward`
  (`knee_collector_uart.py:200`) sign-aligns every sample to the *first* one past
  5° and sums. The thigh tilts less, over a curved noisy arc, during a combined
  sit-to-stand, so `f_thigh` anchors to a near-threshold noisy sample and comes
  out tilted. A tilted `f_thigh` re-projects hip motion into the knee.
- **The two forwards are learned independently** and are not guaranteed coplanar
  or consistently signed, so common (hip) motion never perfectly cancels.

### 1b. Inconsistent session-to-session
Calibration is **timer-driven, not motion-driven** (`CAL_SECONDS=2.0`,
`SWEEP_SECONDS=6.0`, `knee_collector_uart.py:93`):

- The zero countdown starts the instant valid data flows (`knee_gui.py:340`) with
  **no stillness check** — residual motion contaminates `d_i`, which shifts every
  downstream reading.
- The 6 s sweep fires on the clock whether or not a full-range rep happened. A
  partial/slow rep → thin, noisy `f_i` → a different result every session.
- No per-segment *coverage* or *planarity* quality gate beyond the 5 % per-sample
  filter, so a bad calibration passes silently.

### 1c. Too fast / not tailored to sit-to-stand
Fixed 2 s + 6 s windows don't match a real sit-to-stand tempo and give no
coaching, so people rush and under-move.

---

## 2. Goals / non-goals

**Goals**
- Isolate the two segments' forward axes with **separate, clean motions** so hip
  and knee calibration stop contaminating each other.
- Make calibration **motion-gated and stillness-gated** so it paces to the user
  and produces the same result every session.
- **Detect and surface** bad calibration (out-of-plane motion, insufficient
  range, residual hip→knee coupling) instead of silently accepting it.
- Keep the runtime angle math, serial protocol, and CSV columns unchanged.

**Non-goals (call out explicitly)**
- Fully removing hip *axial-rotation* leakage — unobservable with gravity-only
  IMUs. Would need magnetometer-aided heading or a hinge-axis constraint (a
  separate, larger change). We minimize and *measure* it here.
- Absolute anatomical zero (still relative to the held pose, as today).

---

## 3. New protocol (four phases)

```
WAIT → ZERO (still-gated) → HIP (learn f_thigh) → KNEE (learn f_shank) → RUN
```

### Phase 1 — ZERO (stillness-gated)
- **Instruction:** "Leg straight and still."
- Accept the zero only once a sliding window of `ZERO_STILL_SECONDS` has angular
  spread below `ZERO_STILL_TOL_DEG` for **both** segments; until then the banner
  stays "hold still" and the timer does not "count down," it *waits for quiet*.
- `d_thigh`, `d_shank` = `average_gravity()` over the accepted still window.
- Fallback: after `ZERO_MAX_WAIT` accept the quietest window seen and warn.

### Phase 2 — HIP (learn `f_thigh`)
- **Instruction:** "Keep the knee locked straight — swing the whole leg from the
  hip, forward and back." (Standing on the other leg, or lying on your side.)
- Because the knee is locked, thigh and shank tilt *together*; only the **thigh**
  forward is taken here.
- Advance when thigh coverage ≥ `HIP_MIN_COVERAGE_DEG` (and a min sample count),
  or on `PHASE_MAX_SECONDS` timeout (then warn if under target).
- `f_thigh = plane_forward(g_thigh_samples, d_thigh)` (PCA — §4).
- **Validation (this is the direct answer to complaint 1a):** with the knee
  locked, the knee reading should stay ≈ 0 the whole swing. We compute it live
  and, if it strays past `HIP_KNEE_RESIDUAL_WARN_DEG`, warn "leg not swinging in
  a straight plane — that's the hip motion that leaks into the knee; swing
  straighter." This both improves *and* explains the calibration to the user.

### Phase 3 — KNEE (learn `f_shank`)
- **Instruction:** "Sit down. Keep the thigh still — bend and straighten the knee."
- Only the **shank** forward is taken here (thigh should be ~still; if the thigh
  moves more than `KNEE_THIGH_STILL_TOL_DEG`, warn).
- Advance when shank coverage ≥ `KNEE_MIN_COVERAGE_DEG`, or on timeout + warn.
- `f_shank = plane_forward(g_shank_samples, d_shank)`.

### Plane / sign consistency
- Each `f_i` sign is fixed to the direction the segment actually tilted during
  its own sweep (mean of the in-plane component over the high-coverage samples),
  preserving **today's polarity** (flexion sign as asserted by `--selftest`).
- `f_thigh` from the hip phase and `f_shank` from the knee phase are checked for
  coplanarity of their sagittal planes; a large mismatch (segments moved in
  different planes) is warned, since that is exactly what breaks cancellation.

---

## 4. Robust forward-axis estimate (`plane_forward`)

For pure planar flexion by angle θ, `g = cosθ·d + sinθ·f`, so the in-plane part
`g⊥ = g − (g·d)d = sinθ·f` lies on the **line spanned by `f`**. The dominant
principal direction of the `g⊥` samples *is* `f`; the secondary spread measures
out-of-plane motion. This replaces the fragile sign-aligned sum with a principled
fit and yields a free quality score.

**Stdlib-only implementation (no numpy in the collector):** build an orthonormal
2-D basis `{e1, e2}` in the plane ⊥ `d`, project each `g⊥` into 2-D, and do a
closed-form **2×2 symmetric eigensolve** of the 2×2 scatter matrix:

```
For each g with |g⊥| ≥ sin(SWEEP_MIN_ANGLE_DEG):  p = (g⊥·e1, g⊥·e2)
  Sxx += px·px;  Sxy += px·py;  Syy += py·py
λ1,λ2 = eigenvalues of [[Sxx,Sxy],[Sxy,Syy]]     (λ1 ≥ λ2, closed form)
v1     = dominant eigenvector (2-D) → f = v1.x·e1 + v1.y·e2   (unit)
planarity = 1 − λ2/λ1        (1.0 = perfectly planar)
sign(f)   = align to mean g⊥ over the samples (preserve current polarity)
return f, planarity, coverage_deg    # or (None, …) if no sample cleared the gate
```

- `planarity < PLANARITY_MIN` → warn (out-of-plane / axial motion during that
  phase — the leak source), calibration still usable but flagged.
- Same call replaces both `estimate_forward` uses; the existing `--selftest`
  planar/mount/yaw asserts must still pass (PCA recovers the same `f` for clean
  planar motion), plus new asserts for planarity and noise robustness.

---

## 5. Parameters (proposed defaults, all overridable)

| Constant | Default | Meaning |
|---|---|---|
| `CAL_SECONDS` | 5.0 | quiet window required to accept the zero |
| `ZERO_STILL_TOL_DEG` | 1.0 | max gravity-direction spread counted as "still" |
| `ZERO_MAX_WAIT` | 15.0 | fallback: accept quietest window, warn |
| `HIP_MIN_COVERAGE_DEG` | 25.0 | thigh tilt needed to learn `f_thigh` |
| `KNEE_MIN_COVERAGE_DEG` | 60.0 | shank tilt needed to learn `f_shank` |
| `SWEEP_MIN_SECONDS` | 3.0 | don't advance a sweep before this even if covered |
| `SWEEP_SECONDS` | 12.0 | per-sweep timeout (advance + warn if short) |
| `SWEEP_MIN_ANGLE_DEG` | 5.0 | per-sample in-plane gate (unchanged) |
| `PLANARITY_MIN` | 0.90 | below → out-of-plane warning |
| `HIP_KNEE_RESIDUAL_WARN_DEG` | 8.0 | hip-phase knee drift → coupling warning |
| `KNEE_THIGH_STILL_TOL_DEG` | 10.0 | thigh drift during knee phase → warning |

---

## 6. Files touched

- **`knee_collector_uart.py`**
  - New `plane_forward()` (§4); keep `estimate_forward` as a thin wrapper or
    retire it (self-test decides).
  - New stillness gate + coverage/planarity helpers.
  - Rework `run()` state machine: `zeroing → hip → knee → running` with
    motion/stillness gating and the hip-phase knee-residual validation.
  - CLI: keep `--cal-seconds` (now zero max-wait); replace `--sweep-seconds` with
    `--hip-cov` / `--knee-cov` (+ `--phase-timeout`); optional `--combined` flag
    to keep the old single-sweep path for A/B testing.
  - Extend `_self_test()` (§7).
- **`knee_gui.py`**
  - `Collector.run()` state machine mirrors the same four phases (rename the
    single `sweep` into `hip` + `knee`).
  - `PHASE_STYLE`, `Sampler.ACTIVE`, phase strings, banner coaching text, and the
    live preview (show coverage progress + planarity, not a stopwatch).
- **`README.md`**
  - Rewrite the calibration walkthrough (lines ~53–54, 82–84) and the algorithm
    sections (§2–§3, lines ~143–164); update CLI flag table (~266–267); expand
    the trade-off note with the hip-phase validation and the honest
    axial-rotation limit.

---

## 7. Test plan (`--selftest`, still stdlib-only)

1. **`plane_forward` recovers `f`** for clean planar motion under random mounts /
   shared heading / yaw drift (reuse the existing generator at
   `knee_collector_uart.py:392`) and reports `planarity ≈ 1`.
2. **Planarity flags noise:** add out-of-plane jitter → `f` still ~recovered,
   `planarity` drops below `PLANARITY_MIN`.
3. **Stillness gate:** a synthetic still window is accepted; a drifting one is
   rejected until it settles.
4. **Coverage gating:** advance triggers at the threshold, times out below it.
5. **End-to-end separated calibration:** synthesize a knee-locked hip swing and a
   thigh-fixed knee flex, learn `f_thigh`/`f_shank` from them separately, then
   assert recovered knee angle matches ground truth across the existing test
   poses (and that a pure hip motion reads ≈ 0 knee — the complaint, proven).

---

## 8. Rollout

1. Land `plane_forward` + stillness/coverage helpers with self-tests (no behavior
   change to `run()` yet) — provable in isolation.
2. Switch `run()` and the GUI to the four-phase machine.
3. Update README + CLI help.
4. `--selftest` green; optional bench check against a protractor hinge
   (README "Validate against a protractor").

---

## 9. Open questions for you

1. **Posture for the hip phase** — standing leg-swing, or lying on your side?
   Lying removes balance as a confound and gives a cleaner plane; standing is
   more like real use. Default proposed: allow either, instruct standing.
2. **Keep the old combined sweep** behind `--combined` for comparison, or remove
   it outright?
3. **Threshold defaults** in §5 — comfortable, or do you want them tuned to a
   specific population (e.g. limited-ROM patients need lower coverage targets)?
