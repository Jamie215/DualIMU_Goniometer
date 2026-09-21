#!/usr/bin/env python3
"""
KNEE ANGLE COLLECTOR  [UART topology, gravity-referenced sagittal angle]

Only the CENTRAL is read via Serial. Its stream carries, for BOTH segments (thigh
+ shank), a 6-DOF orientation quaternion and the raw accelerometer vector, merged
on the central's clock. There is ONE angle method (chosen for reliability):

  GRAVITY-REFERENCED SAGITTAL INCLINOMETER.
  The drift-free, observable quantity in a 6-DOF (accel+gyro) filter is the
  direction of gravity in each board's own frame -- it is pinned by the
  accelerometer (so it does not drift in tilt) and yaw-invariant (so the 6-DOF
  heading, which DOES drift, never enters). We read that gravity direction from
  the fused QUATERNION -- not the bare accelerometer -- so the gyro carries it
  smoothly through fast motion instead of the reading collapsing whenever the
  limb accelerates. From it we build each segment's signed sagittal tilt and take
  the difference:
      d_i    = gravity-in-board at the straight-and-still zero pose (segment axis)
      f_i    = each segment's in-plane "forward", learned from the calibration sweep
      incl_i = atan2(g_i . f_i, g_i . d_i)      (signed tilt in the sagittal plane)
      knee   = incl_thigh - incl_shank

Because every quantity is an angle between two vectors in the SAME board frame,
the result is independent of how each board is strapped on (position on the limb,
constant mounting rotation, rotation about the leg's long axis all cancel), as
long as each board is rigidly fixed to its segment. It measures the sagittal
component of the joint angle -- ideal for upright knee flexion (standing ROM,
gait, sit-to-stand).

Calibration is three-phase (all handled here, no reflashing needed). The two
sweeps are SEPARATED on purpose: learning each segment's forward axis from a
motion that isolates it keeps hip and knee calibration from contaminating each
other (a combined sit-to-stand makes both tilt together, so neither forward axis
is clean -- the main reason hip motion leaks into the knee reading).
  1. STATIC ZERO -- hold the leg straight and STILL. We wait for a genuinely quiet
     window (stillness-gated, not just timed) and average each segment's
     gravity-in-board direction to get its zero axis d_i.
  2. HIP SWEEP -- knee locked straight, swing the whole leg from the hip. Both
     boards tilt together; we learn the THIGH forward axis f_thigh from this, and
     (since the knee is locked) the knee reading should stay ~0 -- a built-in
     check that surfaces the very out-of-plane hip motion that causes leakage.
  3. KNEE SWEEP -- sit down, hold the thigh still, bend/straighten the knee. We
     learn the SHANK forward axis f_shank from this. A segment that stays still
     just contributes ~0 to the angle.
Each forward axis is fit as the dominant principal direction of the in-plane
gravity trajectory (robust to noise), which also yields a planarity quality score
that flags an out-of-plane sweep instead of trusting a mis-aimed axis.

What "0 deg" means (IMPORTANT):
  The angle is RELATIVE to the pose held during the static zero -- whatever
  posture you hold becomes 0 deg, regardless of the true anatomical geometry. A
  leg that is anatomically straight but sits at a residual angle (recurvatum, a
  patient who can't fully extend) still reads 0 deg here. So every reading is a
  CHANGE FROM the zeroing pose, not an absolute femur-vs-tibia angle. This is
  correct for ROM (max - min), rep counting, and movement quality -- all
  difference-based, so the reference cancels -- but the readings are NOT absolute
  clinical angles, and are not comparable across sessions that were zeroed on
  different poses. Recovering the true anatomical zero would require an external
  reference (a manual goniometer on the leg, or a fixture that defines true
  extension); the two IMUs alone cannot observe it, because the method is
  deliberately blind to how each board is mounted relative to the bone.

Dropout handling:
  - A sample is INVALID if the shank timestamp is 0 (timeout) OR the shank
    quaternion is all zeros (a packet that carried no real data). A real unit
    quaternion is never all-zero, so that's a safe "no data" sentinel.
  - Short dropouts are FORWARD-FILLED; a sustained run (> MAX_FILL) is MISSING.
  - Data-quality stats are tracked and each row is flagged so nothing is hidden.

Fusion is 6-DOF (accel + gyro), no magnetometer -- calibration-free and robust
to nearby metal.

Central line format (18 fields):
  D,<t_thigh_us>,<tw>,<tx>,<ty>,<tz>,<tax>,<tay>,<taz>,
    <t_shank_recv_us>,<sw>,<sx>,<sy>,<sz>,<sax>,<say>,<saz>,<age_us>
The shank board streams continuously; the central reports its freshest packet and,
in the last field, that packet's AGE in us (parsed below into 'rtt' for continuity
with older logs -- same units, a link-health number). A stale/absent shank makes
t_shank_recv and the shank quaternion 0, i.e. an invalid sample.

Usage:
  python knee_collector_uart.py --port /dev/ttyACM0            # collect
  python knee_collector_uart.py --port /dev/ttyACM0 --monitor  # bring-up check
  python knee_collector_uart.py --port /dev/ttyACM0 --raw      # inspect stream
  python knee_collector_uart.py --selftest
"""

import argparse
import csv
import math
import time

try:
    import serial
except ImportError:
    serial = None

# How many consecutive dropped samples we're willing to forward-fill before we
# stop trusting the held value and mark the stretch as missing. At the fixed 50 Hz
# stream, 10 samples is ~200 ms.
MAX_FILL = 10

# --- Calibration timing / gating ------------------------------------------- #
# The zero is STILLNESS-gated, not just timed: we require a quiet window this long
# (a settling drift dwarfs sensor noise, so waiting for quiet beats averaging
# longer). CAL_SECONDS is the length of the still window we must observe.
CAL_SECONDS = 5.0
# Max gravity-direction spread (deg) across the window that still counts as "still".
ZERO_STILL_TOL_DEG = 1.0
# Give up waiting for perfect quiet after this; accept the quietest window, warn.
ZERO_MAX_WAIT = 15.0

# The two functional sweeps (hip, then knee) are MOTION-gated: each advances as
# soon as its segment has covered enough range (so the user sets the pace and is
# never cut off mid-rep), bounded below by SWEEP_MIN_SECONDS (don't jump ahead on
# a twitch) and above by SWEEP_SECONDS (advance anyway, warn if under-covered).
SWEEP_SECONDS = 12.0
SWEEP_MIN_SECONDS = 3.0
HIP_MIN_COVERAGE_DEG = 25.0     # thigh tilt needed to learn f_thigh (hip swing)
KNEE_MIN_COVERAGE_DEG = 60.0    # shank tilt needed to learn f_shank (knee flex)

# A segment must tilt more than this during the sweep for its motion to define
# the forward direction, so IMU noise around the zero pose doesn't set it.
SWEEP_MIN_ANGLE_DEG = 5.0

# Below this planarity score (1.0 = perfectly planar motion) the sweep wandered
# out of the sagittal plane -- the exact source of hip-into-knee leakage -- so we
# flag it rather than silently trusting a mis-aimed forward axis.
PLANARITY_MIN = 0.90
# With the knee locked during the hip swing, the knee angle should stay ~0; a
# larger residual means out-of-plane hip motion the method can't fully cancel.
HIP_KNEE_RESIDUAL_WARN_DEG = 8.0
# During the knee sweep the thigh should stay put; warn if it drifts past this.
KNEE_THIGH_STILL_TOL_DEG = 10.0

# --------------------------------------------------------------------------- #
# Small vector / quaternion helpers (w, x, y, z), stdlib-only.
# --------------------------------------------------------------------------- #
def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm3(a):
    return math.sqrt(_dot3(a, a))


def _normalize3(a):
    n = _norm3(a)
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_canonical(q):
    """Force w >= 0 so the quaternion represents the shortest-arc rotation."""
    if q[0] < 0.0:
        return (-q[0], -q[1], -q[2], -q[3])
    return q


def q_from_axis_angle(axis, angle_rad):
    ax, ay, az = _normalize3(axis)
    h = angle_rad * 0.5
    s = math.sin(h)
    return (math.cos(h), ax * s, ay * s, az * s)


def total_angle_deg(q):
    """Unsigned total rotation angle of a quaternion, in degrees. Used by the
    --monitor drift diagnostic (relative-quaternion angle)."""
    q = q_canonical(q_normalize(q))
    vn = _norm3((q[1], q[2], q[3]))
    return math.degrees(2.0 * math.atan2(vn, q[0]))


def average_quaternion(qs):
    """Mean of near-identical quaternions: sign-align to a reference, sum,
    normalize. Adequate for the small spread of a held-still window."""
    if not qs:
        return None
    ref = qs[0]
    acc = [0.0, 0.0, 0.0, 0.0]
    for q in qs:
        d = q[0] * ref[0] + q[1] * ref[1] + q[2] * ref[2] + q[3] * ref[3]
        s = -1.0 if d < 0.0 else 1.0
        for i in range(4):
            acc[i] += s * q[i]
    return q_normalize(tuple(acc))


def relative_quaternion(thigh_q, shank_q):
    return q_mul(q_conj(thigh_q), shank_q)


# --------------------------------------------------------------------------- #
# Gravity-referenced (heading-immune) angle.  See the module docstring for the
# geometry. d_i is the zero-pose gravity direction (the segment's long axis),
# f_i the learned in-plane forward; the signed sagittal tilt is atan2(g.f, g.d)
# and the knee is the difference of the two segments' tilts.
# --------------------------------------------------------------------------- #
def gravity_in_board(q):
    """Unit gravity direction expressed in the board's own frame (world up
    rotated by the inverse orientation). Drift-free: invariant to yaw."""
    qc = q_conj(q_normalize(q))
    r = q_mul(q_mul(qc, (0.0, 0.0, 0.0, 1.0)), q_conj(qc))
    return _normalize3((r[1], r[2], r[3]))


def _perp(g, d):
    gd = _dot3(g, d)
    return (g[0] - gd * d[0], g[1] - gd * d[1], g[2] - gd * d[2])


def _basis_perp(d):
    """Two orthonormal vectors spanning the plane perpendicular to unit vector d.
    Seeds off d's smallest component for numerical stability."""
    ax, ay, az = abs(d[0]), abs(d[1]), abs(d[2])
    if ax <= ay and ax <= az:
        seed = (1.0, 0.0, 0.0)
    elif ay <= az:
        seed = (0.0, 1.0, 0.0)
    else:
        seed = (0.0, 0.0, 1.0)
    sd = _dot3(seed, d)
    e1 = _normalize3((seed[0] - sd * d[0], seed[1] - sd * d[1], seed[2] - sd * d[2]))
    e2 = _normalize3((d[1] * e1[2] - d[2] * e1[1],
                      d[2] * e1[0] - d[0] * e1[2],
                      d[0] * e1[1] - d[1] * e1[0]))
    return e1, e2


def plane_forward(g_list, d, min_tilt_deg=SWEEP_MIN_ANGLE_DEG):
    """Learn the segment's in-plane 'anterior' direction f, robustly.

    For planar flexion by angle t, g = cos(t) d + sin(t) f, so the part of gravity
    perpendicular to d is sin(t) f -- every such sample lies on the LINE spanned by
    f. So f is the dominant principal direction of the perpendicular components (a
    2x2 eigenproblem in the plane perp to d), and the SECONDARY spread measures how
    far the motion strayed out of plane. This replaces the old sign-aligned sum
    (which anchored on one noisy sample) and yields a free quality score.

    Returns (f, planarity, coverage_deg):
      f            -- unit forward vector, or None if the segment barely moved
      planarity    -- 1 - lambda2/lambda1 in [0,1]; 1.0 = perfectly planar motion
      coverage_deg -- largest tilt away from d among the samples used
    """
    if d is None:
        return None, 0.0, 0.0
    e1, e2 = _basis_perp(d)
    thr = math.sin(math.radians(min_tilt_deg))
    sxx = sxy = syy = 0.0
    mx = my = 0.0
    n = 0
    coverage = 0.0
    for g in g_list:
        r = _perp(g, d)
        rn = _norm3(r)
        if rn < thr:
            continue
        px = _dot3(r, e1); py = _dot3(r, e2)
        sxx += px * px; sxy += px * py; syy += py * py
        mx += px; my += py
        n += 1
        cov = math.degrees(math.asin(max(-1.0, min(1.0, rn))))
        if cov > coverage:
            coverage = cov
    if n == 0:
        return None, 0.0, 0.0
    # eigenvalues of the symmetric 2x2 scatter [[sxx,sxy],[sxy,syy]]
    tr = sxx + syy
    root = math.sqrt(max(0.0, tr * tr / 4.0 - (sxx * syy - sxy * sxy)))
    lam1 = tr / 2.0 + root
    lam2 = tr / 2.0 - root
    # dominant eigenvector (2D): (lam1 - syy, sxy), or the bigger diagonal axis
    if abs(sxy) > 1e-12:
        vx, vy = lam1 - syy, sxy
    else:
        vx, vy = (1.0, 0.0) if sxx >= syy else (0.0, 1.0)
    vn = math.hypot(vx, vy)
    if vn == 0.0:
        return None, 0.0, coverage
    vx /= vn; vy /= vn
    # sign: align to the mean in-plane direction (preserves the old polarity, where
    # 'forward' is the way the segment actually tilted from extension).
    if vx * mx + vy * my < 0.0:
        vx, vy = -vx, -vy
    f = _normalize3((vx * e1[0] + vy * e2[0],
                     vx * e1[1] + vy * e2[1],
                     vx * e1[2] + vy * e2[2]))
    planarity = 1.0 - (lam2 / lam1 if lam1 > 0.0 else 1.0)
    return f, planarity, coverage


def estimate_forward(g_list, d, min_tilt_deg=SWEEP_MIN_ANGLE_DEG):
    """Backward-compatible wrapper: just the forward axis from plane_forward()."""
    return plane_forward(g_list, d, min_tilt_deg)[0]


def gravity_spread_deg(g_list):
    """Largest angular deviation (deg) of the samples from their mean direction --
    the stillness measure that gates the zero window."""
    m = average_gravity(g_list)
    if m is None:
        return 0.0
    worst = 0.0
    for g in g_list:
        c = max(-1.0, min(1.0, _dot3(g, m)))
        a = math.degrees(math.acos(c))
        if a > worst:
            worst = a
    return worst


def sagittal_inclination(g, d, f):
    """Signed tilt (deg) of gravity g from the zero direction d, in the (d,f)
    plane. If f is None the segment is treated as still (0)."""
    if f is None:
        return 0.0
    return math.degrees(math.atan2(_dot3(g, f), _dot3(g, d)))


def gravity_knee_angle(thigh_q, shank_q, d_thigh, f_thigh, d_shank, f_shank):
    gt = gravity_in_board(thigh_q)
    gs = gravity_in_board(shank_q)
    return (sagittal_inclination(gt, d_thigh, f_thigh)
            - sagittal_inclination(gs, d_shank, f_shank))


def average_gravity(g_list):
    """Mean gravity-in-board direction over a still window, normalized."""
    if not g_list:
        return None
    acc = [0.0, 0.0, 0.0]
    for g in g_list:
        acc[0] += g[0]; acc[1] += g[1]; acc[2] += g[2]
    return _normalize3(tuple(acc))


# --------------------------------------------------------------------------- #
# Serial line parsing / validity
# --------------------------------------------------------------------------- #
def parse_line(line):
    p = line.strip().split(',')
    if not p or p[0] != 'D':
        return None
    try:
        if len(p) == 18:                 # quaternion + raw accel per segment
            return {
                't_thigh': int(p[1]),
                'thigh_q': (float(p[2]), float(p[3]), float(p[4]), float(p[5])),
                'thigh_a': (float(p[6]), float(p[7]), float(p[8])),
                't_shank_mid': int(p[9]),
                'shank_q': (float(p[10]), float(p[11]), float(p[12]), float(p[13])),
                'shank_a': (float(p[14]), float(p[15]), float(p[16])),
                'rtt': int(p[17]),
            }
        if len(p) == 12:                 # legacy: quaternion only (no accel)
            return {
                't_thigh': int(p[1]),
                'thigh_q': (float(p[2]), float(p[3]), float(p[4]), float(p[5])),
                'thigh_a': None,
                't_shank_mid': int(p[6]),
                'shank_q': (float(p[7]), float(p[8]), float(p[9]), float(p[10])),
                'shank_a': None,
                'rtt': int(p[11]),
            }
        return None
    except ValueError:
        return None


def gravity_from_quat(rec, seg):
    """Gravity direction in board frame from the fused orientation quaternion --
    the single source the angle method uses (gyro-fused: smooth through motion,
    drift-free in tilt)."""
    return gravity_in_board(rec[seg + '_q'])


def gravity_from_accel(rec, seg):
    """Gravity direction straight from the raw accelerometer (filter-free). Only
    used by --monitor as a static cross-check; falls back to the quaternion when
    a legacy stream carries no accel."""
    a = rec.get(seg + '_a')
    if a is not None:
        return _normalize3(a)
    return gravity_in_board(rec[seg + '_q'])


def is_valid(rec):
    """Valid only if the shank sample has a real timestamp AND the shank
    quaternion isn't all zeros (the no-data sentinel)."""
    if rec['t_shank_mid'] == 0:
        return False
    if all(c == 0.0 for c in rec['shank_q']):
        return False
    return True


class DropoutHandler:
    """Turns a stream of (valid/invalid) angle samples into a continuous output
    using forward-fill for short gaps, with stats and per-sample status."""
    def __init__(self, max_fill=MAX_FILL):
        self.max_fill = max_fill
        self.last_good = None
        self.consecutive_bad = 0
        self.n_valid = 0
        self.n_filled = 0
        self.n_missing = 0

    def process(self, valid, angle):
        """Returns (output_angle_or_None, status_string)."""
        if valid:
            self.last_good = angle
            self.consecutive_bad = 0
            self.n_valid += 1
            return angle, 'valid'
        self.consecutive_bad += 1
        if self.last_good is not None and self.consecutive_bad <= self.max_fill:
            self.n_filled += 1
            return self.last_good, 'filled'
        self.n_missing += 1
        return None, 'missing'

    def summary(self):
        total = self.n_valid + self.n_filled + self.n_missing
        if total == 0:
            return "no samples"
        return (f"valid={self.n_valid} ({100*self.n_valid/total:.1f}%)  "
                f"filled={self.n_filled} ({100*self.n_filled/total:.1f}%)  "
                f"missing={self.n_missing} ({100*self.n_missing/total:.1f}%)")


def _incl_from_zero(g, d):
    """Unsigned tilt (deg) of gravity g away from its zero direction d."""
    c = max(-1.0, min(1.0, _dot3(g, d)))
    return math.degrees(math.acos(c))


def _self_test():
    print("Running self-test...")

    assert is_valid({'t_shank_mid': 100, 'shank_q': (1.0, 0.0, 0.0, 0.0)})
    assert not is_valid({'t_shank_mid': 0, 'shank_q': (1.0, 0.0, 0.0, 0.0)})
    assert not is_valid({'t_shank_mid': 100, 'shank_q': (0.0, 0.0, 0.0, 0.0)})
    print("  validity logic OK")

    h = DropoutHandler(max_fill=3)
    assert h.process(True, 10.0) == (10.0, 'valid')
    assert h.process(False, None) == (10.0, 'filled')
    assert h.process(False, None) == (10.0, 'filled')
    assert h.process(False, None) == (10.0, 'filled')
    out, status = h.process(False, None)
    assert out is None and status == 'missing', (out, status)
    assert h.process(True, 20.0) == (20.0, 'valid')
    print("  forward-fill + missing logic OK")

    h2 = DropoutHandler()
    out, status = h2.process(False, None)
    assert out is None and status == 'missing'
    print("  handles leading invalid sample OK")

    # --- quaternion algebra (used by the monitor drift diagnostic) ---
    zx = (0.0, 0.0, 1.0)
    q45 = q_from_axis_angle(zx, math.radians(45))
    assert abs(total_angle_deg(q45) - 45.0) < 1e-6
    assert abs(total_angle_deg(q_mul(q45, q45)) - 90.0) < 1e-6
    assert total_angle_deg(q_mul(q45, q_conj(q45))) < 1e-6
    print("  quaternion algebra OK")

    # --- static average of near-identical quaternions ---
    base = q_from_axis_angle((0.1, 0.2, 0.97), math.radians(15))
    jitter = [q_mul(base, q_from_axis_angle((1.0, 0.0, 0.0), math.radians(d)))
              for d in (-0.5, 0.0, 0.5)]
    avg = average_quaternion(jitter)
    assert total_angle_deg(q_mul(q_conj(avg), base)) < 1.0
    print("  static-average calibration OK")

    # --- gravity-referenced angle: recover the true knee despite arbitrary
    #     mounts (B_t, B_s), a shared heading, and independent yaw drift. This is
    #     the whole method, so it is the placement-independence proof too. ---
    def _bq(psi, p, B):   # board->world: Rz(psi) . Ry(p) . B
        return q_mul(q_from_axis_angle((0, 0, 1), psi),
                     q_mul(q_from_axis_angle((0, 1, 0), p), B))
    B_t = q_from_axis_angle((0.4, -0.6, 0.7), math.radians(50))   # arbitrary mounts
    B_s = q_from_axis_angle((-0.3, 0.8, 0.2), math.radians(80))
    psi = 1.1                                                      # shared heading
    gt0 = [gravity_in_board(_bq(psi, math.radians(a), B_t)) for a in range(0, 46, 3)]
    gs0 = [gravity_in_board(_bq(psi, math.radians(a), B_s)) for a in range(0, 46, 3)]
    d_t = average_gravity([gt0[0]]); d_s = average_gravity([gs0[0]])
    f_t, plt, _ = plane_forward(gt0, d_t); f_s, pls, _ = plane_forward(gs0, d_s)
    assert f_t is not None and f_s is not None
    assert plt > 0.99 and pls > 0.99, (plt, pls)   # clean planar sweep -> planarity ~1
    # estimate_forward wrapper returns just the axis
    assert estimate_forward(gt0, d_t) == f_t
    for pth, psh in ((10, 40), (55, 15), (0, 0), (30, 70)):
        qt = _bq(psi, math.radians(pth), B_t)
        qs = _bq(psi, math.radians(psh), B_s)
        knee = gravity_knee_angle(qt, qs, d_t, f_t, d_s, f_s)
        assert abs(knee - (pth - psh)) < 1e-6, (pth, psh, knee)
    # independent per-board yaw drift must not change the reading (yaw-immune)
    for pth, psh in ((25, 60),):
        qt = _bq(psi + 0.9, math.radians(pth), B_t)   # thigh yaw drifted +0.9 rad
        qs = _bq(psi - 0.5, math.radians(psh), B_s)   # shank yaw drifted -0.5 rad
        knee = gravity_knee_angle(qt, qs, d_t, f_t, d_s, f_s)
        assert abs(knee - (pth - psh)) < 1e-6, (pth, psh, knee)
    # a still segment (no forward learned) simply contributes 0
    assert sagittal_inclination((0.1, 0.0, 0.99), d_t, None) == 0.0
    print("  gravity-referenced angle OK (placement- and yaw-independent)")

    # --- return-from-high-angle monotonicity: sweeping the shank up to 130 deg
    #     and back must trace the same signed angle both ways (no stuck 0s). ---
    seq = list(range(0, 131, 10)) + list(range(120, -1, -10))
    for a in seq:
        qs = _bq(psi, math.radians(a), B_s)
        knee = gravity_knee_angle(_bq(psi, 0.0, B_t), qs, d_t, f_t, d_s, f_s)
        assert abs(knee - (0 - a)) < 1e-6, (a, knee)
    print("  return-from-high-angle sweep OK")

    # --- separated hip/knee calibration: learn each forward axis from an
    #     ISOLATED motion (hip swing for the thigh, knee flex for the shank) and
    #     still recover the true knee. A pure hip motion must read ~0 knee -- the
    #     leak the redesign targets. ---
    hip_t = [gravity_in_board(_bq(psi, math.radians(h), B_t)) for h in range(0, 31, 3)]
    hip_s = [gravity_in_board(_bq(psi, math.radians(h), B_s)) for h in range(0, 31, 3)]
    knee_s = [gravity_in_board(_bq(psi, math.radians(k), B_s)) for k in range(0, 71, 5)]
    fh, plh, covh = plane_forward(hip_t, d_t)     # thigh forward from the hip swing
    fk, plk, covk = plane_forward(knee_s, d_s)    # shank forward from the knee flex
    assert fh is not None and fk is not None
    assert plh > 0.99 and plk > 0.99, (plh, plk)
    assert covh > 25.0 and covk > 60.0, (covh, covk)
    for hh in (5, 15, 25):                         # pure hip motion -> ~0 knee
        qt = _bq(psi, math.radians(hh), B_t); qs = _bq(psi, math.radians(hh), B_s)
        assert abs(gravity_knee_angle(qt, qs, d_t, fh, d_s, fk)) < 1e-6, hh
    for pth, psh in ((0, 40), (20, 60), (30, 30)):  # true knee still recovered
        qt = _bq(psi, math.radians(pth), B_t); qs = _bq(psi, math.radians(psh), B_s)
        assert abs(gravity_knee_angle(qt, qs, d_t, fh, d_s, fk) - (pth - psh)) < 1e-6
    assert hip_lock_residual_deg(hip_t, hip_s, d_t, fh, d_s, fk) < 1e-4
    print("  separated hip/knee calibration OK (pure hip -> ~0 knee)")

    # --- planarity score: a clean sagittal sweep scores ~1; an out-of-plane one
    #     (wobbling off the sagittal plane) scores lower, so it can be flagged. ---
    e1s, e2s = _basis_perp(d_s)
    def _perp_sample(t_deg, w_deg):
        t = math.radians(t_deg); w = math.radians(w_deg)
        ux = math.cos(w) * e1s[0] + math.sin(w) * e2s[0]
        uy = math.cos(w) * e1s[1] + math.sin(w) * e2s[1]
        uz = math.cos(w) * e1s[2] + math.sin(w) * e2s[2]
        c = math.cos(t); s = math.sin(t)
        return _normalize3((c * d_s[0] + s * ux, c * d_s[1] + s * uy, c * d_s[2] + s * uz))
    planar = [_perp_sample(a, 0.0) for a in range(5, 61, 3)]
    outp = [_perp_sample(a, 25.0 * math.sin(a)) for a in range(5, 61, 3)]
    _, pl_planar, _ = plane_forward(planar, d_s)
    _, pl_out, _ = plane_forward(outp, d_s)
    assert pl_planar > 0.99, pl_planar
    assert pl_out < 0.97, pl_out
    print("  planarity quality score OK")

    # --- stillness gate: a held-still window reads ~0 spread; a moving one large. ---
    still = [gravity_in_board(_bq(psi, 0.0, B_t)) for _ in range(20)]
    assert gravity_spread_deg(still) < 1e-6
    moving = [gravity_in_board(_bq(psi, math.radians(a), B_t)) for a in range(0, 20, 2)]
    assert gravity_spread_deg(moving) > 5.0
    print("  stillness gate OK")

    # --- parsing: 18-field (accel) and 12-field (legacy) lines ---
    r18 = parse_line("D,1,1,0,0,0,0.1,0.0,0.98,2,1,0,0,0,-0.2,0.0,0.97,300")
    assert r18 and r18['thigh_a'] == (0.1, 0.0, 0.98) and r18['shank_a'] == (-0.2, 0.0, 0.97)
    r12 = parse_line("D,1,1,0,0,0,2,1,0,0,0,300")
    assert r12 and r12['thigh_a'] is None
    gq = gravity_from_quat(r18, 'thigh')
    assert abs(_norm3(gq) - 1.0) < 1e-9
    ga = gravity_from_accel(r18, 'thigh')             # raw accel (monitor path)
    assert abs(_norm3(ga) - 1.0) < 1e-9
    ga12 = gravity_from_accel(r12, 'thigh')           # falls back to quat
    assert abs(_dot3(ga12, (0.0, 0.0, 1.0)) - 1.0) < 1e-6
    print("  parsing + gravity sources OK")

    # --- stream diagnosis: the three failure modes + the healthy case ---
    assert diagnose_stream(0, 0, 0, '', 'PORT') and 'No data' in \
        diagnose_stream(0, 0, 0, '', 'PORT')
    assert 'fields' in diagnose_stream(5, 0, 0, 'X,1,2,3', 'PORT')
    assert 'peripheral' in diagnose_stream(5, 5, 0, 'D,...', 'PORT')
    assert diagnose_stream(5, 5, 3, 'D,...', 'PORT', need_valid=1) is None
    print("  stream diagnosis OK")

    print(f"  example summary line: {h.summary()}")
    print("Self-test PASSED.\n")


def diagnose_stream(seen, parsed, valid_count, last_line, port, need_valid=1):
    """Turn raw-stream tallies into a one-line explanation of why healthy VALID
    'D' lines aren't arriving, or None once at least `need_valid` have been seen.

    Shared by the CLI's wait_for_stream and the GUI's link-error banner so both
    say the same thing. The three failure modes map to the three points in the
    chain: nothing on the wire, wrong/old firmware, or a dead peripheral link.
        seen    -- non-empty lines received
        parsed  -- lines that parsed as a 'D' record
        valid_count -- parsed records whose shank sample was real
        last_line   -- most recent non-empty raw line (for the field-count hint)
    """
    if valid_count >= need_valid:
        return None
    if seen == 0:
        return (f"No data on {port}. Is the central board plugged in and running? "
                f"(use --raw to inspect the raw stream)")
    if parsed == 0:
        n = len(last_line.split(','))
        return (f"Receiving data, but it isn't an 18-field 'D' line (last line had "
                f"{n} fields). Reflash BOTH boards with the current firmware. "
                f"Last line: {last_line!r}")
    return ("Central is streaming, but every shank quaternion is the zero sentinel "
            "-> no data from the peripheral. Check that the peripheral is powered and its LED "
            "is a fast blink (active); a slow idle blink means it isn't getting the "
            "central's keepalive -- verify BOTH UART wires (Peripheral TX->Central RX and "
            "Central TX->Peripheral RX) and GND between the boards.")


def wait_for_stream(ser, port, need_valid=5, warn_every=3.0):
    """Block until the central is sending healthy, parseable, VALID D lines.

    This is what stops the collector from silently sitting at 'zeroing' forever:
    the calibration timer only advances on parseable lines, so if the firmware,
    the format, or the peripheral link is wrong we'd otherwise hang with no clue. Here
    we watch the raw stream and, every few seconds, say exactly what's wrong via
    diagnose_stream()."""
    ser.reset_input_buffer()   # drop stale buffered bytes so timing starts clean
    seen = parsed = valid = 0
    last_line = ''
    last_warn = time.time()
    while valid < need_valid:
        line = ser.readline().decode('ascii', 'ignore').strip()
        if line.startswith('#'):
            # Surface the central's banners (firmware id, "calibrating, hold still",
            # gyro bias). Seeing them means the board is alive and starting a
            # collection, so reset the no-data timer instead of warning spuriously.
            print(f"  {line}")
            last_warn = time.time()
            continue
        if line:
            seen += 1
            last_line = line
            rec = parse_line(line)
            if rec is not None:
                parsed += 1
                if is_valid(rec):
                    valid += 1

        if time.time() - last_warn >= warn_every and valid < need_valid:
            msg = diagnose_stream(seen, parsed, valid, last_line, port, need_valid)
            if msg:
                print(f"  ...{msg}")
            last_warn = time.time()
    print("Data OK.")


def _write_row(writer, rec, angle, status, valid):
    """One CSV row in the fixed schema, shared by every phase."""
    tq = rec['thigh_q']
    sq = rec['shank_q']
    writer.writerow([
        rec['t_thigh'],
        f"{tq[0]:.4f}", f"{tq[1]:.4f}", f"{tq[2]:.4f}", f"{tq[3]:.4f}",
        f"{sq[0]:.4f}" if valid else '',
        f"{sq[1]:.4f}" if valid else '',
        f"{sq[2]:.4f}" if valid else '',
        f"{sq[3]:.4f}" if valid else '',
        f"{angle:.2f}" if angle is not None else '',
        status,
        rec['rtt'],
    ])


def hip_lock_residual_deg(hip_gt, hip_gs, d_thigh, f_thigh, d_shank, f_shank):
    """Largest |knee| over the knee-locked hip swing. With the knee locked the
    joint is straight the whole time, so this should be ~0; a large value is the
    out-of-plane hip motion the gravity method cannot fully cancel -- exactly the
    leakage the user sees. Returns None if either forward axis is missing."""
    if f_thigh is None or f_shank is None:
        return None
    worst = 0.0
    for gt, gs in zip(hip_gt, hip_gs):
        k = abs(sagittal_inclination(gt, d_thigh, f_thigh)
                - sagittal_inclination(gs, d_shank, f_shank))
        if k > worst:
            worst = k
    return worst


def calibration_notes(pl_thigh, cov_thigh, f_thigh,
                      pl_shank, cov_shank, f_shank,
                      hip_residual, thigh_drift):
    """Human-readable warnings about calibration quality, shared by CLI and GUI so
    both say the same thing. Empty list == a clean calibration."""
    notes = []
    if f_shank is None:
        notes.append("shank barely moved during the knee sweep -- redo with a fuller bend")
    elif cov_shank < KNEE_MIN_COVERAGE_DEG:
        notes.append(f"knee sweep only reached {cov_shank:.0f} deg "
                     f"(< {KNEE_MIN_COVERAGE_DEG:.0f}); bend further next time")
    if f_thigh is None:
        notes.append("thigh didn't tilt during the hip sweep -- treating it as fixed "
                     "(knee = shank tilt)")
    elif cov_thigh < HIP_MIN_COVERAGE_DEG:
        notes.append(f"hip sweep only reached {cov_thigh:.0f} deg "
                     f"(< {HIP_MIN_COVERAGE_DEG:.0f}); swing further next time")
    if f_thigh is not None and pl_thigh < PLANARITY_MIN:
        notes.append(f"thigh sweep drifted out of plane (planarity {pl_thigh:.2f}); "
                     f"swing straight in the sagittal plane")
    if f_shank is not None and pl_shank < PLANARITY_MIN:
        notes.append(f"shank sweep drifted out of plane (planarity {pl_shank:.2f})")
    if hip_residual is not None and hip_residual > HIP_KNEE_RESIDUAL_WARN_DEG:
        notes.append(f"knee drifted {hip_residual:.0f} deg during the locked hip swing "
                     f"-- residual hip-to-knee coupling; keep the knee locked and swing "
                     f"straighter")
    if thigh_drift is not None and thigh_drift > KNEE_THIGH_STILL_TOL_DEG:
        notes.append(f"thigh moved {thigh_drift:.0f} deg during the knee sweep -- "
                     f"keep it still")
    return notes


def _zero_phase(ser, writer, still_seconds,
                tol_deg=ZERO_STILL_TOL_DEG, max_wait=ZERO_MAX_WAIT):
    """Phase 1: read until a genuinely STILL window of length still_seconds is seen
    for both segments (or max_wait elapses), then average it into d_thigh, d_shank.
    Waiting for quiet -- rather than just averaging longer -- is what stops a slow
    postural drift from biasing the zero."""
    win = []                       # trailing (t, g_thigh, g_shank) within the window
    start = time.time()
    spread_t = spread_s = 0.0
    while True:
        rec = parse_line(ser.readline().decode('ascii', 'ignore'))
        if rec is None:
            continue
        now = time.time()
        valid = is_valid(rec)
        _write_row(writer, rec, None, 'zeroing', valid)
        if valid:
            win.append((now, gravity_from_quat(rec, 'thigh'),
                        gravity_from_quat(rec, 'shank')))
            win[:] = [w for w in win if now - w[0] <= still_seconds]
            spread_t = gravity_spread_deg([w[1] for w in win])
            spread_s = gravity_spread_deg([w[2] for w in win])
            covered = (now - win[0][0]) >= still_seconds * 0.98
            print(f"\r  holding still... spread thigh {spread_t:4.1f} / shank "
                  f"{spread_s:4.1f} deg (need < {tol_deg:.1f})   ", end='')
            if covered and spread_t <= tol_deg and spread_s <= tol_deg:
                print(f"\n  Zero captured ({len(win)} still samples).")
                return (average_gravity([w[1] for w in win]),
                        average_gravity([w[2] for w in win]))
        if now - start >= max_wait:
            if win:
                print(f"\n  WARNING: never fully settled; using best {len(win)} samples "
                      f"(spread up to {max(spread_t, spread_s):.1f} deg).")
                return (average_gravity([w[1] for w in win]),
                        average_gravity([w[2] for w in win]))
            print("\n  WARNING: no valid samples during zero; still waiting...")
            start = now


def _sweep_phase(ser, writer, learn_seg, d_thigh, d_shank,
                 min_cov_deg, max_seconds, status, min_seconds=SWEEP_MIN_SECONDS):
    """Phases 2/3: motion-gated sweep. Read while the user moves `learn_seg`;
    advance once that segment has covered min_cov_deg (and min_seconds have passed)
    or max_seconds elapses. Returns (f, planarity, coverage, gt_samples, gs_samples).
    The stored per-sample gravities let the caller validate the hip lock and the
    thigh-still assumption afterward."""
    d_learn = d_thigh if learn_seg == 'thigh' else d_shank
    gt = []; gs = []
    start = time.time()
    coverage = 0.0
    while True:
        rec = parse_line(ser.readline().decode('ascii', 'ignore'))
        if rec is None:
            continue
        elapsed = time.time() - start
        valid = is_valid(rec)
        preview = None
        if valid:
            g_t = gravity_from_quat(rec, 'thigh')
            g_s = gravity_from_quat(rec, 'shank')
            gt.append(g_t); gs.append(g_s)
            cov = _incl_from_zero(g_t if learn_seg == 'thigh' else g_s, d_learn)
            coverage = max(coverage, cov)
            preview = cov
        _write_row(writer, rec, preview, status, valid)
        pct = min(100.0, 100.0 * coverage / min_cov_deg)
        print(f"\r  {status} sweep: {coverage:4.0f}/{min_cov_deg:.0f} deg "
              f"[{pct:3.0f}%]  {elapsed:4.1f}s   ", end='')
        if elapsed >= min_seconds and coverage >= min_cov_deg:
            break
        if elapsed >= max_seconds:
            break
    samples = gt if learn_seg == 'thigh' else gs
    f, planarity, cov = plane_forward(samples, d_learn)
    print()
    return f, planarity, cov, gt, gs


def run(port, out_path, baud=115200,
        cal_seconds=CAL_SECONDS, sweep_seconds=SWEEP_SECONDS, raw=False):
    if serial is None:
        raise RuntimeError("pyserial not installed. pip install pyserial")

    ser = serial.Serial(port, baud, timeout=1)

    if raw:
        print(f"RAW mode: dumping serial lines from {port}. Ctrl-C to stop.")
        try:
            while True:
                line = ser.readline().decode('ascii', 'ignore').rstrip()
                if line:
                    print(f"[{len(line.split(','))} fields] {line!r}")
        except KeyboardInterrupt:
            print("\nStopping.")
        finally:
            ser.close()
        return

    outfile = open(out_path, 'w', newline='')
    writer = csv.writer(outfile)
    writer.writerow(['t_thigh_us',
                     'thigh_qw', 'thigh_qx', 'thigh_qy', 'thigh_qz',
                     'shank_qw', 'shank_qx', 'shank_qy', 'shank_qz',
                     'knee_angle_deg', 'status', 'rtt_us'])

    handler = DropoutHandler()

    # Don't start calibrating until real data is flowing, otherwise a phase can
    # advance before the user is ready (or hang invisibly on a bad link).
    # wait_for_stream diagnoses no-data / wrong-firmware / dead-peripheral.
    print(f"Waiting for data on {port} ... (gravity-referenced angle, gyro-fused)")
    wait_for_stream(ser, port)

    try:
        # Phase 1: stillness-gated zero.
        print(f"\nZERO -- stand with the leg STRAIGHT and hold STILL "
              f"(need {cal_seconds:.0f}s of quiet)...")
        d_thigh, d_shank = _zero_phase(ser, writer, cal_seconds)

        # Phase 2: hip sweep -> learn the thigh forward axis. Knee stays locked, so
        # the joint is straight and we can validate the reading afterward.
        print("\nHIP -- keep the knee LOCKED straight, swing the whole leg from the "
              "hip (forward and back)...")
        f_thigh, pl_thigh, cov_thigh, hip_gt, hip_gs = _sweep_phase(
            ser, writer, 'thigh', d_thigh, d_shank, HIP_MIN_COVERAGE_DEG,
            sweep_seconds, status='hip')

        # Phase 3: knee sweep -> learn the shank forward axis. Thigh held still.
        print("\nKNEE -- sit down, keep the THIGH still, bend and straighten the "
              "knee...")
        f_shank, pl_shank, cov_shank, knee_gt, knee_gs = _sweep_phase(
            ser, writer, 'shank', d_thigh, d_shank, KNEE_MIN_COVERAGE_DEG,
            sweep_seconds, status='knee')

        # Validate + report quality.
        hip_residual = hip_lock_residual_deg(hip_gt, hip_gs, d_thigh, f_thigh,
                                             d_shank, f_shank)
        thigh_drift = gravity_spread_deg(knee_gt) if knee_gt else None
        notes = calibration_notes(pl_thigh, cov_thigh, f_thigh,
                                  pl_shank, cov_shank, f_shank,
                                  hip_residual, thigh_drift)
        for note in notes:
            print(f"  ! {note}")
        if hip_residual is not None:
            print(f"  hip-lock knee residual: {hip_residual:.1f} deg "
                  f"(lower is better; < {HIP_KNEE_RESIDUAL_WARN_DEG:.0f} is good)")
        print("\nCalibrated (gravity-referenced, drift-free). Reporting knee angle. "
              "Ctrl-C to stop.")

        # Phase 4: run. Drift-free sagittal knee = thigh incl - shank incl.
        while True:
            rec = parse_line(ser.readline().decode('ascii', 'ignore'))
            if rec is None:
                continue
            valid = is_valid(rec)
            angle = None
            if valid:
                angle = gravity_knee_angle(rec['thigh_q'], rec['shank_q'],
                                           d_thigh, f_thigh, d_shank, f_shank)
            angle, status = handler.process(valid, angle)
            _write_row(writer, rec, angle, status, valid)
            disp = f"{angle:6.1f}" if angle is not None else "  --  "
            print(f"\rknee: {disp} deg  [{status:7s}]  rtt:{rec['rtt']:5d}us   ", end='')
    except KeyboardInterrupt:
        print("\nStopping.")
        print("Data quality (run phase): " + handler.summary())
    finally:
        outfile.close()
        ser.close()


def monitor(port, baud=115200, zero_seconds=1.5):
    """Bring-up diagnostic: zero once on a brief still hold, then show each
    segment's gyro-fused gravity inclination (the value the angle method uses)
    next to a filter-free raw-accel tilt and the yaw-prone relative-quaternion
    angle. thigh/shank should track; if 'rel' climbs while they sit still, that's
    the 6-DOF yaw drift the gravity method avoids. rtt/valid% localize a bad link."""
    if serial is None:
        raise RuntimeError("pyserial not installed. pip install pyserial")
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Monitor on {port}. Hold the leg STILL to zero...")
    wait_for_stream(ser, port)

    zero_samples = []; gz_t = []; gz_s = []; az_t = []; az_s = []
    start = time.time()
    while time.time() - start < zero_seconds:
        rec = parse_line(ser.readline().decode('ascii', 'ignore'))
        if rec and is_valid(rec):
            zero_samples.append(relative_quaternion(rec['thigh_q'], rec['shank_q']))
            gz_t.append(gravity_from_quat(rec, 'thigh'))
            gz_s.append(gravity_from_quat(rec, 'shank'))
            az_t.append(gravity_from_accel(rec, 'thigh'))
            az_s.append(gravity_from_accel(rec, 'shank'))
    q_rel0 = average_quaternion(zero_samples) or (1.0, 0.0, 0.0, 0.0)
    d_t = average_gravity(gz_t) or (0.0, 0.0, 1.0)
    d_s = average_gravity(gz_s) or (0.0, 0.0, 1.0)
    da_t = average_gravity(az_t) or (0.0, 0.0, 1.0)
    da_s = average_gravity(az_s) or (0.0, 0.0, 1.0)
    print(f"Zeroed on {len(zero_samples)} samples. thigh/shank = gyro-fused gravity "
          "tilts (what the angle uses); accel = filter-free raw-accel tilt "
          "cross-check; rel = relative-quaternion angle (drift probe). Ctrl-C to stop.")

    n = nbad = 0
    prev_t_thigh = None
    try:
        while True:
            rec = parse_line(ser.readline().decode('ascii', 'ignore'))
            if rec is None:
                continue
            n += 1
            # Central loop period, from ITS own clock (t_thigh_us). This localizes a
            # stall: if the shank goes stale AND this jumps to seconds, the CENTRAL
            # froze (e.g. USB print blocking on a slow host) -- both boards' data
            # freeze together. If this stays ~10 ms while the shank is stale, the
            # central is running fine and the PERIPHERAL went silent (reset / power / sensor).
            dthigh_ms = 0.0
            if prev_t_thigh is not None:
                dthigh_ms = ((rec['t_thigh'] - prev_t_thigh) & 0xFFFFFFFF) / 1000.0
            prev_t_thigh = rec['t_thigh']
            if not is_valid(rec):
                nbad += 1
                # Field 18 is now the freshest shank packet's AGE (us): the peripheral
                # streams and the central reports how old its newest packet is.
                # Invalid means nothing fresh enough this cycle -- age 0 = no packet
                # yet (link/power/wiring), else the peripheral has stalled past the
                # firmware's stale window.
                cause = 'no packet yet' if rec['rtt'] == 0 else 'peripheral stalled'
                print(f"\r  [shank INVALID: {cause:14s}]  valid:{100*(n-nbad)/n:4.0f}%  "
                      f"age:{rec['rtt']:8d}us  central dt:{dthigh_ms:7.1f}ms      ", end='')
                continue
            it = _incl_from_zero(gravity_from_quat(rec, 'thigh'), d_t)
            is_ = _incl_from_zero(gravity_from_quat(rec, 'shank'), d_s)
            at = _incl_from_zero(gravity_from_accel(rec, 'thigh'), da_t)
            as_ = _incl_from_zero(gravity_from_accel(rec, 'shank'), da_s)
            rel = total_angle_deg(q_mul(q_conj(q_rel0),
                                        relative_quaternion(rec['thigh_q'], rec['shank_q'])))
            print(f"\r  thigh:{it:5.1f} shank:{is_:5.1f}  accel(t/s):{at:5.1f}/{as_:5.1f}"
                  f"  rel:{rel:5.1f}  valid:{100*(n-nbad)/n:4.0f}%  age:{rec['rtt']:5d}us"
                  f"  central dt:{dthigh_ms:5.1f}ms  ", end='')
    except KeyboardInterrupt:
        print("\nMonitor stopped.")
    finally:
        ser.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--port')
    ap.add_argument('--out', default='knee_log.csv')
    ap.add_argument('--cal-seconds', type=float, default=CAL_SECONDS,
                    help='length of the STILL window required to capture the zero '
                         '(stillness-gated, not a fixed countdown)')
    ap.add_argument('--sweep-seconds', type=float, default=SWEEP_SECONDS,
                    help='per-sweep timeout; each of the hip and knee sweeps is '
                         'motion-gated and normally advances earlier, on coverage')
    ap.add_argument('--raw', action='store_true',
                    help='dump raw serial lines (with field count) and exit; '
                         'use this to check the central output format')
    ap.add_argument('--monitor', action='store_true',
                    help='live bring-up check: gyro-fused gravity tilts vs a '
                         'filter-free accel tilt and the yaw-prone quaternion angle')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()

    if args.selftest or (not args.port):
        _self_test()
        if not args.selftest:
            print("Provide --port to collect live data.")
    elif args.monitor:
        monitor(args.port)
    else:
        run(args.port, args.out, raw=args.raw,
            cal_seconds=args.cal_seconds, sweep_seconds=args.sweep_seconds)
