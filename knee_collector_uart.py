#!/usr/bin/env python3
"""
KNEE ANGLE COLLECTOR  [UART topology, quaternion / placement-independent]

Only the MASTER is read via Serial. Its stream carries the orientation QUATERNION
of both segments (thigh + shank), merged on the master's clock. The knee angle is
derived from the RELATIVE rotation between the two segments, which makes the result
independent of how the boards are strapped on:

  Let q_thigh, q_shank be the two segment quaternions. The relative rotation is
      q_rel = conj(q_thigh) . q_shank
  Any constant sensor-to-segment misalignment B turns the true joint delta into
  B^-1 . q_delta . B -- a similarity transform, which PRESERVES the rotation angle.
  So the flexion angle we read out does not depend on sensor placement, as long as
  each board is rigidly fixed to its segment.

Calibration is two-phase (both handled here, no reflashing needed):

  1. STATIC ZERO  -- hold full extension for ~CAL_SECONDS. We average q_rel over
     the window to get the reference q_rel0 (0 deg baseline).
  2. FUNCTIONAL SWEEP -- do a few slow flexion reps for ~SWEEP_SECONDS. We watch
     the relative rotation and learn the real flexion axis from the motion, then
     report a SIGNED angle by swing-twist decomposition about that axis (off-axis
     motion is rejected, flexion vs hyperextension is distinguished).

Dropout handling is unchanged from the pitch-based version:
  - A sample is INVALID if the shank timestamp is 0 (timeout) OR the shank
    quaternion is all zeros (a packet that carried no real data). A real unit
    quaternion is never all-zero, so that's a safe "no data" sentinel.
  - Short dropouts are FORWARD-FILLED; a sustained run (> MAX_FILL) is MISSING.
  - Data-quality stats are tracked and each row is flagged so nothing is hidden.

Fusion is 6-DOF (accel + gyro), no magnetometer -- calibration-free and robust
to nearby metal.

Two angle methods (--method):
  gravity (default) -- each segment is a DRIFT-FREE sagittal inclinometer built
    from the gravity direction in its own frame (immune to 6-DOF yaw drift, to
    constant mounting, and to board rotation about the leg's long axis). The knee
    is the difference of the two inclinations. Best for upright sagittal flexion;
    it measures the sagittal-plane component of the angle.
  quat -- relative-quaternion swing-twist about a learned flexion axis. General
    (any plane) but partly exposed to yaw drift in 6-DOF.

--gravity-source picks where the gravity method reads gravity:
  accel (default) -- the RAW accelerometer, bypassing the Mahony filter entirely.
    Drift-free and filter-free: a stationary rig reads a flat line. Noisier during
    fast limb acceleration (fine for slow/quasi-static knee motion).
  quat -- gravity derived from the orientation quaternion (uses the filter).

Calibration (both methods): stand STRAIGHT and still to zero, then do a few slow
reps that bend BOTH knee and hip (sit-to-stands / marching) so each segment tilts
enough to learn its axis. A segment that stays still just contributes ~0.

Master line format (12 fields):
  D,<t_thigh_us>,<tw>,<tx>,<ty>,<tz>,<t_shank_mid_us>,<sw>,<sx>,<sy>,<sz>,<rtt_us>

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
# stop trusting the held value and mark the stretch as missing. At 104 Hz, 10
# samples is ~100 ms.
MAX_FILL = 10

# Calibration windows (seconds): hold full extension, then flexion reps.
CAL_SECONDS = 2.0
SWEEP_SECONDS = 6.0

# A sweep delta must exceed this rotation to contribute to the axis estimate,
# so IMU noise around the zero pose doesn't define the flexion direction.
SWEEP_MIN_ANGLE_DEG = 5.0

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
    """Unsigned total rotation angle of a quaternion, in degrees."""
    q = q_canonical(q_normalize(q))
    vn = _norm3((q[1], q[2], q[3]))
    return math.degrees(2.0 * math.atan2(vn, q[0]))


def signed_flexion_deg(q_delta, axis):
    """Signed rotation of q_delta about `axis` (swing-twist twist angle), degrees.

    Positive means rotation along +axis. Off-axis (swing) components are ignored,
    which is what lets a hinge-joint angle survive out-of-plane wobble."""
    q = q_canonical(q_normalize(q_delta))
    proj = _dot3((q[1], q[2], q[3]), _normalize3(axis))
    return math.degrees(2.0 * math.atan2(proj, q[0]))


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


def estimate_flexion_axis(deltas, min_angle_deg=SWEEP_MIN_ANGLE_DEG):
    """Learn the flexion axis from a set of relative-rotation deltas.

    Each qualifying delta contributes a rotation vector (axis * angle); we
    sign-align them (a hinge sweeps consistently one way from extension) and sum.
    The normalized sum is the flexion axis, oriented so that the flexion done
    during the sweep reads as POSITIVE. Returns None if there wasn't enough
    motion to be trustworthy."""
    ref = None
    acc = [0.0, 0.0, 0.0]
    n = 0
    for q in deltas:
        q = q_canonical(q_normalize(q))
        v = (q[1], q[2], q[3])
        vn = _norm3(v)
        if vn == 0.0:
            continue
        angle = 2.0 * math.atan2(vn, q[0])   # >= 0 (canonical)
        if math.degrees(angle) < min_angle_deg:
            continue
        axis = (v[0] / vn, v[1] / vn, v[2] / vn)
        r = (axis[0] * angle, axis[1] * angle, axis[2] * angle)
        if ref is None:
            ref = axis
        if _dot3(r, ref) < 0.0:
            r = (-r[0], -r[1], -r[2])
        acc[0] += r[0]; acc[1] += r[1]; acc[2] += r[2]
        n += 1
    if n == 0:
        return None
    return _normalize3(tuple(acc))


def relative_quaternion(thigh_q, shank_q):
    return q_mul(q_conj(thigh_q), shank_q)


# --------------------------------------------------------------------------- #
# Gravity-referenced (heading-immune) angle.
#
# In 6-DOF, gravity pins each board's tilt but not its heading, so the relative
# quaternion carries yaw drift. The gravity DIRECTION in each board's own frame
# is drift-free (invariant to yaw about vertical). We turn each segment into a
# drift-free sagittal inclinometer and take the difference:
#   d_i  = gravity-in-board at the zero pose (the segment's long axis)
#   f_i  = "anterior" in-plane direction, learned from the calibration motion
#   incl_i = atan2(g.f_i, g.d_i)            (signed tilt in the sagittal plane)
#   knee  = incl_thigh - incl_shank
# This cancels constant mounting (angles between board-frame vectors), position,
# and heading drift. It measures the sagittal-plane component -- ideal for
# upright knee flexion. Validated against random mounts + yaw drift.
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


def estimate_forward(g_list, d, min_tilt_deg=SWEEP_MIN_ANGLE_DEG):
    """Learn the segment's in-plane 'anterior' direction from the tilt it shows
    during the calibration motion: the (perpendicular-to-d) component of gravity,
    sign-aligned and summed. Returns a unit vector, or None if the segment barely
    moved (then that segment contributes ~0 to the angle anyway)."""
    ref = None
    acc = [0.0, 0.0, 0.0]
    n = 0
    thr = math.sin(math.radians(min_tilt_deg))
    for g in g_list:
        r = _perp(g, d)
        rn = _norm3(r)
        if rn < thr:
            continue
        u = (r[0] / rn, r[1] / rn, r[2] / rn)
        if ref is None:
            ref = u
        s = 1.0 if _dot3(u, ref) >= 0.0 else -1.0
        acc[0] += s * r[0]; acc[1] += s * r[1]; acc[2] += s * r[2]
        n += 1
    if n == 0:
        return None
    return _normalize3(tuple(acc))


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
        if len(p) == 18:                 # new: quaternion + raw accel per segment
            return {
                't_thigh': int(p[1]),
                'thigh_q': (float(p[2]), float(p[3]), float(p[4]), float(p[5])),
                'thigh_a': (float(p[6]), float(p[7]), float(p[8])),
                't_shank_mid': int(p[9]),
                'shank_q': (float(p[10]), float(p[11]), float(p[12]), float(p[13])),
                'shank_a': (float(p[14]), float(p[15]), float(p[16])),
                'rtt': int(p[17]),
            }
        if len(p) == 12:                 # old: quaternion only (no accel)
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


def gravity_dir(rec, seg, source):
    """Unit gravity direction in board frame for segment 'thigh'/'shank'.
    source='accel' reads the raw accelerometer (drift-free, filter-free);
    source='quat' derives it from the orientation quaternion."""
    if source == 'accel' and rec.get(seg + '_a') is not None:
        return _normalize3(rec[seg + '_a'])
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

    # --- quaternion algebra ---
    zx = (0.0, 0.0, 1.0)   # arbitrary hinge axis for the tests
    q45 = q_from_axis_angle(zx, math.radians(45))
    assert abs(total_angle_deg(q45) - 45.0) < 1e-6
    assert abs(total_angle_deg(q_mul(q45, q45)) - 90.0) < 1e-6
    # conj round-trip -> identity -> 0 deg
    assert total_angle_deg(q_mul(q45, q_conj(q45))) < 1e-6
    print("  quaternion algebra OK")

    # --- signed swing-twist about a known axis ---
    q60 = q_from_axis_angle(zx, math.radians(60))
    assert abs(signed_flexion_deg(q60, zx) - 60.0) < 1e-6
    assert abs(signed_flexion_deg(q_conj(q60), zx) + 60.0) < 1e-6  # opposite dir
    # off-axis wobble should barely move the reported flexion
    wobble = q_from_axis_angle((1.0, 0.0, 0.0), math.radians(4))
    noisy = q_mul(q60, wobble)
    assert abs(signed_flexion_deg(noisy, zx) - 60.0) < 2.0
    print("  signed swing-twist OK")

    # --- PLACEMENT INDEPENDENCE: a constant misalignment B must not change the
    #     recovered flexion magnitude (angle is invariant under conjugation). ---
    B = q_from_axis_angle((0.3, -0.7, 0.5), math.radians(37))   # arbitrary mount
    q_joint = q_from_axis_angle(zx, math.radians(72))
    q_mounted = q_mul(q_mul(q_conj(B), q_joint), B)             # B^-1 . joint . B
    assert abs(total_angle_deg(q_mounted) - 72.0) < 1e-6
    # and the axis rotates by B, so swing-twist about the rotated axis recovers it
    rot_axis = q_mul(q_mul(q_conj(B), (0.0,) + zx), B)          # rotate axis by B^-1
    assert abs(signed_flexion_deg(q_mounted, (rot_axis[1], rot_axis[2], rot_axis[3]))
               - 72.0) < 1e-4
    print("  placement independence OK")

    # --- static average of near-identical quaternions ---
    base = q_from_axis_angle((0.1, 0.2, 0.97), math.radians(15))
    jitter = [q_mul(base, q_from_axis_angle((1.0, 0.0, 0.0), math.radians(d)))
              for d in (-0.5, 0.0, 0.5)]
    avg = average_quaternion(jitter)
    assert total_angle_deg(q_mul(q_conj(avg), base)) < 1.0
    print("  static-average calibration OK")

    # --- functional sweep: learn the flexion axis from motion ---
    true_axis = _normalize3((0.2, 0.9, -0.3))
    sweep = [q_from_axis_angle(true_axis, math.radians(a))
             for a in (10, 25, 40, 55, 40, 25, 10)]
    est = estimate_flexion_axis(sweep)
    assert est is not None and abs(abs(_dot3(est, true_axis)) - 1.0) < 1e-3
    # a signed angle taken about the learned axis matches the true rotation
    probe = q_from_axis_angle(true_axis, math.radians(48))
    assert abs(signed_flexion_deg(probe, est) - 48.0) < 1e-2
    # too little motion -> no trustworthy axis
    assert estimate_flexion_axis(
        [q_from_axis_angle(true_axis, math.radians(1))]) is None
    print("  functional-sweep axis estimation OK")

    a = signed_flexion_deg(q_from_axis_angle(zx, math.radians(30)), zx)
    assert abs(a - 30.0) < 1e-6
    print("  end-to-end angle math OK")

    # --- gravity-referenced angle: recover true knee despite mounts + heading ---
    def _bq(psi, p, B):   # board->world: Rz(psi) . Ry(p) . B
        return q_mul(q_from_axis_angle((0, 0, 1), psi),
                     q_mul(q_from_axis_angle((0, 1, 0), p), B))
    B_t = q_from_axis_angle((0.4, -0.6, 0.7), math.radians(50))   # arbitrary mounts
    B_s = q_from_axis_angle((-0.3, 0.8, 0.2), math.radians(80))
    psi = 1.1                                                      # shared heading
    # calibration: both segments tilt forward
    gt0 = [gravity_in_board(_bq(psi, math.radians(a), B_t)) for a in range(0, 46, 3)]
    gs0 = [gravity_in_board(_bq(psi, math.radians(a), B_s)) for a in range(0, 46, 3)]
    d_t = average_gravity([gt0[0]]); d_s = average_gravity([gs0[0]])
    f_t = estimate_forward(gt0, d_t); f_s = estimate_forward(gs0, d_s)
    assert f_t is not None and f_s is not None
    for pth, psh in ((10, 40), (55, 15), (0, 0), (30, 70)):
        qt = _bq(psi, math.radians(pth), B_t)
        qs = _bq(psi, math.radians(psh), B_s)
        knee = gravity_knee_angle(qt, qs, d_t, f_t, d_s, f_s)
        assert abs(knee - (pth - psh)) < 1e-6, (pth, psh, knee)
    # a still segment (no forward learned) simply contributes 0
    assert sagittal_inclination((0.1, 0.0, 0.99), d_t, None) == 0.0
    print("  gravity-referenced angle OK")

    # --- parsing: 18-field (accel) and 12-field (legacy) lines, gravity_dir ---
    r18 = parse_line("D,1,1,0,0,0,0.1,0.0,0.98,2,1,0,0,0,-0.2,0.0,0.97,300")
    assert r18 and r18['thigh_a'] == (0.1, 0.0, 0.98) and r18['shank_a'] == (-0.2, 0.0, 0.97)
    r12 = parse_line("D,1,1,0,0,0,2,1,0,0,0,300")
    assert r12 and r12['thigh_a'] is None
    ga = gravity_dir(r18, 'thigh', 'accel')      # from raw accel
    assert abs(_norm3(ga) - 1.0) < 1e-9
    gq = gravity_dir(r12, 'thigh', 'accel')      # falls back to quat when no accel
    assert abs(_dot3(gq, (0.0, 0.0, 1.0)) - 1.0) < 1e-6
    print("  accel/quat parsing + gravity source OK")

    print(f"  example summary line: {h.summary()}")
    print("Self-test PASSED.\n")


def wait_for_stream(ser, port, need_valid=5, warn_every=3.0):
    """Block until the master is sending healthy, parseable, VALID D lines.

    This is what stops the collector from silently sitting at 'zeroing' forever:
    the calibration timer only advances on parseable lines, so if the firmware,
    the format, or the slave link is wrong we'd otherwise hang with no clue. Here
    we watch the raw stream and, every few seconds, say exactly what's wrong:
    no bytes at all, bytes that don't parse (wrong/old firmware), or D lines whose
    shank quaternion is always zero (slave link down)."""
    ser.reset_input_buffer()   # drop stale buffered bytes so timing starts clean
    seen = parsed = valid = 0
    last_line = ''
    last_warn = time.time()
    while valid < need_valid:
        line = ser.readline().decode('ascii', 'ignore').strip()
        if line:
            seen += 1
            last_line = line
            rec = parse_line(line)
            if rec is not None:
                parsed += 1
                if is_valid(rec):
                    valid += 1

        if time.time() - last_warn >= warn_every and valid < need_valid:
            if seen == 0:
                print(f"  ...no data on {port}. Is the master board plugged in and "
                      f"running? (run with --raw to inspect the raw stream)")
            elif parsed == 0:
                n = len(last_line.split(','))
                print(f"  ...receiving data, but it isn't a 12-field 'D' line "
                      f"(last line had {n} fields). Reflash BOTH boards with the "
                      f"quaternion firmware. Last line: {last_line!r}")
            else:
                print("  ...master is streaming, but every shank quaternion is the "
                      "zero sentinel -> the slave isn't replying. Check the UART "
                      "wiring/GND between the two boards and that the slave is powered.")
            last_warn = time.time()
    print("Data OK.")


def run(port, out_path, baud=115200,
        cal_seconds=CAL_SECONDS, sweep_seconds=SWEEP_SECONDS, raw=False,
        method='gravity', grav_source='accel'):
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
    q_rel0 = None          # quat method: static-zero reference
    flex_axis = None       # quat method: learned flexion axis
    zero_done = False
    axis_done = False
    zero_samples = []
    sweep_deltas = []
    # gravity method: per-board zero direction and learned forward axis
    d_thigh = d_shank = f_thigh = f_shank = None
    gz_thigh = []; gz_shank = []      # gravity-in-board during zeroing
    gs_thigh = []; gs_shank = []      # gravity-in-board during sweep

    t_zero_end = cal_seconds
    t_sweep_end = cal_seconds + sweep_seconds

    # Don't start the calibration clock until real data is flowing, otherwise the
    # zero window can elapse before the user is ready (or hang invisibly on a bad
    # link). wait_for_stream diagnoses no-data / wrong-firmware / dead-slave.
    src = grav_source if method == 'gravity' else 'n/a'
    print(f"Waiting for data on {port} ... (method: {method}, gravity from: {src})")
    wait_for_stream(ser, port)
    start = time.time()
    print(f"Stand with the leg STRAIGHT and STILL ~{cal_seconds:.0f} s (zeroing)...")

    try:
        while True:
            line = ser.readline().decode('ascii', 'ignore')
            rec = parse_line(line)
            if rec is None:
                continue

            elapsed = time.time() - start
            valid = is_valid(rec)
            q_rel = relative_quaternion(rec['thigh_q'], rec['shank_q']) if valid else None

            angle = None
            if not zero_done:
                # Phase 1: straight-and-still hold. Capture the zero references.
                if valid:
                    zero_samples.append(q_rel)
                    gz_thigh.append(gravity_dir(rec, 'thigh', grav_source))
                    gz_shank.append(gravity_dir(rec, 'shank', grav_source))
                status = 'zeroing'
                if elapsed >= t_zero_end:
                    q_rel0 = average_quaternion(zero_samples) or (1.0, 0.0, 0.0, 0.0)
                    d_thigh = average_gravity(gz_thigh)
                    d_shank = average_gravity(gz_shank)
                    zero_done = True
                    n = len(zero_samples)
                    if n == 0:
                        print("\nWARNING: no valid samples during hold.")
                    print(f"\nZero captured ({n} samples). Now do a few slow reps that "
                          f"bend BOTH the knee and hip (e.g. sit-to-stands / marching) "
                          f"for ~{sweep_seconds:.0f} s...")

            elif not axis_done:
                # Phase 2: calibration motion. Learn the flexion axis (quat) and
                # each segment's forward direction (gravity). Preview live angle.
                status = 'sweep'
                if valid:
                    q_delta = q_mul(q_conj(q_rel0), q_rel)
                    sweep_deltas.append(q_delta)
                    gs_thigh.append(gravity_dir(rec, 'thigh', grav_source))
                    gs_shank.append(gravity_dir(rec, 'shank', grav_source))
                    angle = total_angle_deg(q_delta)
                if elapsed >= t_sweep_end:
                    flex_axis = estimate_flexion_axis(sweep_deltas)
                    f_thigh = estimate_forward(gs_thigh, d_thigh)
                    f_shank = estimate_forward(gs_shank, d_shank)
                    axis_done = True
                    if method == 'gravity':
                        if f_shank is None:
                            print("\nWARNING: the shank barely moved during calibration "
                                  "-- redo with a fuller range. Reporting anyway.")
                        elif f_thigh is None:
                            print("\nNote: thigh didn't tilt during calibration; treating "
                                  "it as fixed (knee = shank inclination). Ctrl-C to stop.")
                        else:
                            print("\nCalibrated (gravity, drift-free). Reporting knee "
                                  "angle. Ctrl-C to stop.")
                    elif flex_axis is not None:
                        print("\nFlexion axis learned. Reporting SIGNED knee angle. "
                              "Ctrl-C to stop.")
                    else:
                        print("\nWARNING: not enough sweep motion to learn an axis; "
                              "reporting UNSIGNED angle magnitude. Ctrl-C to stop.")

            else:
                # Phase 3: run.
                if valid:
                    if method == 'gravity':
                        # Drift-free sagittal knee = thigh inclination - shank incl.
                        gt = gravity_dir(rec, 'thigh', grav_source)
                        gs = gravity_dir(rec, 'shank', grav_source)
                        angle = (sagittal_inclination(gt, d_thigh, f_thigh)
                                 - sagittal_inclination(gs, d_shank, f_shank))
                    elif flex_axis is not None:
                        angle = signed_flexion_deg(q_mul(q_conj(q_rel0), q_rel), flex_axis)
                    else:
                        angle = total_angle_deg(q_mul(q_conj(q_rel0), q_rel))
                angle, status = handler.process(valid, angle)

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

            disp = f"{angle:6.1f}" if angle is not None else "  --  "
            print(f"\rknee: {disp} deg  [{status:7s}]  rtt:{rec['rtt']:5d}us   ", end='')
    except KeyboardInterrupt:
        print("\nStopping.")
        print("Data quality (run phase): " + handler.summary())
    finally:
        outfile.close()
        ser.close()


def _incl_from_zero(g, d):
    """Unsigned tilt (deg) of gravity g away from its zero direction d."""
    c = max(-1.0, min(1.0, _dot3(g, d)))
    return math.degrees(math.acos(c))


def monitor(port, baud=115200, zero_seconds=1.5):
    """Bring-up diagnostic: zero once on a brief still hold, then show each
    segment's DRIFT-FREE gravity inclination and the yaw-prone relative-quaternion
    angle side by side. If 'rel' climbs at rest while thigh/shank stay put, that's
    the 6-DOF yaw drift the gravity method avoids. rtt/valid% localize a bad link."""
    if serial is None:
        raise RuntimeError("pyserial not installed. pip install pyserial")
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Monitor on {port}. Hold the leg STILL to zero...")
    wait_for_stream(ser, port)

    zero_samples = []; gz_t = []; gz_s = []
    start = time.time()
    while time.time() - start < zero_seconds:
        rec = parse_line(ser.readline().decode('ascii', 'ignore'))
        if rec and is_valid(rec):
            zero_samples.append(relative_quaternion(rec['thigh_q'], rec['shank_q']))
            gz_t.append(gravity_dir(rec, 'thigh', 'accel'))
            gz_s.append(gravity_dir(rec, 'shank', 'accel'))
    q_rel0 = average_quaternion(zero_samples) or (1.0, 0.0, 0.0, 0.0)
    d_t = average_gravity(gz_t) or (0.0, 0.0, 1.0)
    d_s = average_gravity(gz_s) or (0.0, 0.0, 1.0)
    print(f"Zeroed on {len(zero_samples)} samples. thigh/shank = raw-accel tilts "
          "(filter-free, should be ROCK-STABLE when still); rel = quaternion angle "
          "(uses the filter). Ctrl-C to stop.")

    n = nbad = 0
    try:
        while True:
            rec = parse_line(ser.readline().decode('ascii', 'ignore'))
            if rec is None:
                continue
            n += 1
            if not is_valid(rec):
                nbad += 1
                print(f"\r  [shank INVALID]  valid:{100*(n-nbad)/n:4.0f}%  "
                      f"rtt:{rec['rtt']:5d}us      ", end='')
                continue
            it = _incl_from_zero(gravity_dir(rec, 'thigh', 'accel'), d_t)
            is_ = _incl_from_zero(gravity_dir(rec, 'shank', 'accel'), d_s)
            rel = total_angle_deg(q_mul(q_conj(q_rel0),
                                        relative_quaternion(rec['thigh_q'], rec['shank_q'])))
            print(f"\r  thigh:{it:5.1f}  shank:{is_:5.1f}  rel:{rel:5.1f}  "
                  f"valid:{100*(n-nbad)/n:4.0f}%  rtt:{rec['rtt']:5d}us   ", end='')
    except KeyboardInterrupt:
        print("\nMonitor stopped.")
    finally:
        ser.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--port')
    ap.add_argument('--out', default='knee_log.csv')
    ap.add_argument('--cal-seconds', type=float, default=CAL_SECONDS,
                    help='full-extension hold time for zeroing')
    ap.add_argument('--sweep-seconds', type=float, default=SWEEP_SECONDS,
                    help='flexion-sweep time for learning the flexion axis')
    ap.add_argument('--raw', action='store_true',
                    help='dump raw serial lines (with field count) and exit; '
                         'use this to check the master output format')
    ap.add_argument('--monitor', action='store_true',
                    help='live bring-up check: shows drift-free gravity tilts vs the '
                         'yaw-prone quaternion angle (no sweep/CSV)')
    ap.add_argument('--method', choices=['gravity', 'quat'], default='gravity',
                    help='gravity = drift-free sagittal inclinometer (default); '
                         'quat = relative-quaternion swing-twist')
    ap.add_argument('--gravity-source', choices=['accel', 'quat'], default='accel',
                    help='where the gravity method reads gravity: accel = raw '
                         'accelerometer (drift-free, bypasses the filter; default); '
                         'quat = derived from the orientation quaternion')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()

    if args.selftest or (not args.port):
        _self_test()
        if not args.selftest:
            print("Provide --port to collect live data.")
    elif args.monitor:
        monitor(args.port)
    else:
        run(args.port, args.out, raw=args.raw, method=args.method,
            grav_source=args.gravity_source,
            cal_seconds=args.cal_seconds, sweep_seconds=args.sweep_seconds)
