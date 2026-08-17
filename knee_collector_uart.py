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

Magnetometer (9-DOF) is optional and CONFIGURED AT RUNTIME -- no reflashing:
  - `--calibrate-mag both` streams raw accel+mag from each board (the slave via
    the master's relay), computes hard-/soft-iron offsets AND the BMM150->BMI270
    axis alignment (24-candidate search), saves them to mag_cal.json, and pushes
    them to the boards. Calibration is one-time per board -- it does NOT change
    when you move a board on the limb (placement is handled by the zero+sweep
    above), so you never redo it just because the Arduino moved.
  - On every collection run the stored calibration is pushed to both boards at
    startup. With a calibration present the boards fuse the magnetometer (9-DOF);
    with --no-mag, or no calibration on file, they run 6-DOF (accel+gyro).

Master line format (12 fields):
  D,<t_thigh_us>,<tw>,<tx>,<ty>,<tz>,<t_shank_mid_us>,<sw>,<sx>,<sy>,<sz>,<rtt_us>

Usage:
  python knee_collector_uart.py --port /dev/ttyACM0
  python knee_collector_uart.py --port /dev/ttyACM0 --calibrate-mag both
  python knee_collector_uart.py --port /dev/ttyACM0 --no-mag
  python knee_collector_uart.py --selftest
"""

import argparse
import csv
import json
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

# Magnetometer calibration: where the persisted constants live, and how long to
# tumble each board while collecting raw accel+mag.
DEFAULT_CAL_FILE = 'mag_cal.json'
MAG_TUMBLE_SECONDS = 40.0

# The 24 proper signed-axis permutations, as (ordering, parity). Used to solve
# the BMM150->BMI270 axis alignment (see solve_axis_alignment).
_AXIS_PERMS = [(0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)]
_AXIS_PAR   = [1, -1, -1, 1, 1, -1]


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
# Serial line parsing / validity
# --------------------------------------------------------------------------- #
def parse_line(line):
    p = line.strip().split(',')
    if len(p) != 12 or p[0] != 'D':
        return None
    try:
        return {
            't_thigh': int(p[1]),
            'thigh_q': (float(p[2]), float(p[3]), float(p[4]), float(p[5])),
            't_shank_mid': int(p[6]),
            'shank_q': (float(p[7]), float(p[8]), float(p[9]), float(p[10])),
            'rtt': int(p[11]),
        }
    except ValueError:
        return None


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

    _self_test_mag()

    print(f"  example summary line: {h.summary()}")
    print("Self-test PASSED.\n")


def _qrot(q, v):
    """Rotate 3-vector v by quaternion q."""
    r = q_mul(q_mul(q, (0.0,) + tuple(v)), q_conj(q))
    return (r[1], r[2], r[3])


def _self_test_mag():
    import random
    random.seed(7)

    # Ground truth: a proper-rotation axis map (mag-frame -> imu-frame) and a
    # hard-iron offset. Synthesize a tumble and check the solvers recover both.
    true_perm, true_sign = (1, 2, 0), (1, -1, -1)
    assert _AXIS_PAR[_AXIS_PERMS.index(true_perm)] * true_sign[0] * true_sign[1] * true_sign[2] == 1
    hard_iron = (14.0, -6.0, 22.0)
    g = (0.0, 0.0, -9.81)
    B = (20.0, -5.0, -40.0)

    samples = []
    for _ in range(500):
        axis = _normalize3((random.gauss(0, 1), random.gauss(0, 1), random.gauss(0, 1)))
        R = q_from_axis_angle(axis, random.uniform(0, 2 * math.pi))
        a_imu = _qrot(R, g)
        m_imu = _qrot(R, B)
        # sensor mag-frame = P^-1 . imu: mag[perm[k]] = sign[k] * m_imu[k]
        m_mag = [0.0, 0.0, 0.0]
        for k in range(3):
            m_mag[true_perm[k]] = true_sign[k] * m_imu[k]
        m_raw = (m_mag[0] + hard_iron[0], m_mag[1] + hard_iron[1], m_mag[2] + hard_iron[2])
        samples.append((a_imu[0], a_imu[1], a_imu[2], m_raw[0], m_raw[1], m_raw[2]))

    hs = solve_hard_soft_iron([(s[3], s[4], s[5]) for s in samples])
    assert hs is not None
    bias, scale = hs
    assert all(abs(bias[k] - hard_iron[k]) < 3.0 for k in range(3)), bias
    assert all(abs(scale[k] - 1.0) < 0.2 for k in range(3)), scale
    print("  hard/soft-iron recovery OK")

    axis = solve_axis_alignment(samples, bias, scale)
    assert axis is not None
    perm, sign, best_std, next_std = axis
    assert perm == true_perm and sign == true_sign, (perm, sign)
    assert next_std / best_std > 3.0, (best_std, next_std)   # clean separation
    print("  axis-alignment recovery OK")

    # round-trip through JSON persistence
    import tempfile, os
    cfg = {'master': {'use_mag': True, 'bias': bias, 'scale': scale,
                      'perm': list(perm), 'sign': list(sign)}}
    fd, path = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    try:
        save_mag_cal(path, cfg)
        assert load_mag_cal(path)['master']['perm'] == list(perm)
    finally:
        os.remove(path)
    assert load_mag_cal('/nonexistent/does-not-exist.json') == {}
    print("  mag-cal persistence OK")


# --------------------------------------------------------------------------- #
# Magnetometer calibration (runtime, PC-side, persisted to mag_cal.json).
#
# The firmware never solves anything: it streams raw accel+mag on request and
# accepts a config push. All the math lives here, so re-calibrating a board is
# just re-running --calibrate-mag, never a reflash.
# --------------------------------------------------------------------------- #
def solve_hard_soft_iron(mag_xyz):
    """Hard-/soft-iron from the min/max bounding box of a tumble.

    mag_xyz: list of (mx,my,mz). Returns (bias[3], scale[3]) or None if any axis
    barely moved (not enough rotation to trust)."""
    if not mag_xyz:
        return None
    mins = [min(s[k] for s in mag_xyz) for k in range(3)]
    maxs = [max(s[k] for s in mag_xyz) for k in range(3)]
    bias = [(maxs[k] + mins[k]) * 0.5 for k in range(3)]
    chord = [(maxs[k] - mins[k]) * 0.5 for k in range(3)]
    if min(chord) < 1.0:
        return None
    avg = sum(chord) / 3.0
    scale = [avg / c for c in chord]
    return bias, scale


def solve_axis_alignment(samples, bias, scale):
    """Find the BMM150->BMI270 axis mapping from synchronized accel+mag samples.

    The angle between gravity and the magnetic field is fixed, so accel_hat . mag_hat
    is constant across orientations ONLY when both are in the same frame. We search
    the 24 proper signed-axis permutations for the one minimizing the variance of
    that dot product (mag is calibrated in the raw frame first, matching the firmware
    order). samples: list of (ax,ay,az,mx,my,mz). Returns
    (perm, sign, best_std, next_std) or None."""
    best_var, best_perm, best_sign = float('inf'), None, None
    next_var = float('inf')
    for pi, perm in enumerate(_AXIS_PERMS):
        for smask in range(8):
            s = (-1 if smask & 1 else 1, -1 if smask & 2 else 1, -1 if smask & 4 else 1)
            if _AXIS_PAR[pi] * s[0] * s[1] * s[2] != 1:
                continue
            su = sq = 0.0
            cnt = 0
            for (ax, ay, az, mx, my, mz) in samples:
                an = math.sqrt(ax * ax + ay * ay + az * az)
                if an < 1e-6:
                    continue
                mc = ((mx - bias[0]) * scale[0],
                      (my - bias[1]) * scale[1],
                      (mz - bias[2]) * scale[2])
                mn = math.sqrt(mc[0] * mc[0] + mc[1] * mc[1] + mc[2] * mc[2])
                if mn < 1e-6:
                    continue
                pm = (s[0] * mc[perm[0]], s[1] * mc[perm[1]], s[2] * mc[perm[2]])
                d = (ax * pm[0] + ay * pm[1] + az * pm[2]) / (an * mn)
                su += d
                sq += d * d
                cnt += 1
            if cnt < 50:
                continue
            var = (sq - su * su / cnt) / cnt
            if var < best_var:
                next_var, best_var, best_perm, best_sign = best_var, var, perm, s
            elif var < next_var:
                next_var = var
    if best_perm is None:
        return None
    best_std = math.sqrt(max(best_var, 0.0))
    next_std = math.sqrt(max(next_var, 0.0)) if next_var != float('inf') else float('inf')
    return best_perm, best_sign, best_std, next_std


def load_mag_cal(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (FileNotFoundError, ValueError):
        return {}


def save_mag_cal(path, data):
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)


def _readline(ser):
    return ser.readline().decode('ascii', 'ignore').strip()


def firmware_supports_config(ser, timeout=1.5):
    """PING the master; return True if it answers PONG (runtime-config firmware)."""
    ser.reset_input_buffer()
    ser.write(b'PING\n')
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _readline(ser).startswith('PONG'):
            return True
    return False


def push_config(ser, who, cfg, timeout=2.0):
    """Send one board's config; 'who' is 'M' (master) or 'S' (slave, relayed)."""
    b, s, p, sg = cfg['bias'], cfg['scale'], cfg['perm'], cfg['sign']
    line = ("CFG,%s,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%d,%d,%d,%d,%d,%d\n" %
            (who, 1 if cfg.get('use_mag') else 0,
             b[0], b[1], b[2], s[0], s[1], s[2],
             p[0], p[1], p[2], sg[0], sg[1], sg[2]))
    ser.write(line.encode())
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = _readline(ser)
        if r.startswith('OK,' + who):
            return True
        if r.startswith('ERR,' + who):
            return False
    return False


def stream_raw(ser, who, seconds):
    """Command the board into raw-streaming mode and collect (ax,ay,az,mx,my,mz)."""
    ser.reset_input_buffer()
    ser.write(("STREAM,%s,1\n" % who).encode())
    prefix = 'RM,' if who == 'M' else 'RS,'
    samples = []
    start = time.time()
    last = 0.0
    while time.time() - start < seconds:
        line = _readline(ser)
        if line.startswith(prefix):
            parts = line.split(',')
            if len(parts) == 7:
                try:
                    samples.append(tuple(float(x) for x in parts[1:7]))
                except ValueError:
                    pass
        if time.time() - last > 1.0:
            last = time.time()
            print("\r  %2ds left, samples=%d   " %
                  (int(seconds - (time.time() - start)), len(samples)), end='')
    ser.write(("STREAM,%s,0\n" % who).encode())
    print()
    return samples


def calibrate_mag(port, roles, cal_file=DEFAULT_CAL_FILE, baud=115200,
                  seconds=MAG_TUMBLE_SECONDS):
    """Interactive per-board magnetometer calibration; persists to cal_file."""
    if serial is None:
        raise RuntimeError("pyserial not installed. pip install pyserial")
    ser = serial.Serial(port, baud, timeout=1)
    try:
        if not firmware_supports_config(ser):
            print("This firmware doesn't answer PING -> it predates runtime config. "
                  "Reflash the boards with the current master/slave sketches first.")
            return
        data = load_mag_cal(cal_file)
        for role in roles:
            who = 'M' if role == 'master' else 'S'
            print(f"\n=== Calibrating {role} magnetometer ===")
            input(f"Keep the {role} board clear of nearby metal. Press ENTER, then "
                  f"SLOWLY tumble it through EVERY orientation (all faces up and down) "
                  f"for ~{seconds:.0f}s...")
            samples = stream_raw(ser, who, seconds)
            if len(samples) < 100:
                print(f"  Only {len(samples)} samples - is the {role} board streaming? "
                      "Skipped.")
                continue
            hs = solve_hard_soft_iron([(s[3], s[4], s[5]) for s in samples])
            if hs is None:
                print("  An axis barely moved (not enough rotation). Skipped; re-run.")
                continue
            bias, scale = hs
            axis = solve_axis_alignment(samples, bias, scale)
            if axis is None:
                print("  Axis solve failed. Skipped.")
                continue
            perm, sign, best_std, next_std = axis
            ratio = (next_std / best_std) if best_std > 0 else float('inf')
            cfg = {'use_mag': True, 'bias': bias, 'scale': scale,
                   'perm': list(perm), 'sign': list(sign),
                   'axis_std': best_std, 'sep_ratio': ratio}
            data[role] = cfg
            save_mag_cal(cal_file, data)
            print("  bias  = [%.2f, %.2f, %.2f] uT" % tuple(bias))
            print("  scale = [%.3f, %.3f, %.3f]" % tuple(scale))
            print("  axis: perm=%s sign=%s   separation=%.1fx" %
                  (tuple(perm), tuple(sign), ratio))
            if ratio < 1.3:
                print("  WARNING: weak separation -> tumble was too fast/incomplete. "
                      "Re-run this board.")
            pushed = push_config(ser, who, cfg)
            print(f"  saved to {cal_file}; pushed to board: {'OK' if pushed else 'FAILED'}")
    finally:
        ser.close()


def apply_startup_config(ser, cal_file, use_mag):
    """At the start of a run, push each board's stored calibration (or force
    6-DOF). Skips silently if the firmware predates runtime config."""
    if not firmware_supports_config(ser):
        print("  (firmware predates runtime config; using its compiled-in fusion)")
        return
    data = load_mag_cal(cal_file)
    for role, who in (('master', 'M'), ('slave', 'S')):
        cfg = data.get(role)
        if use_mag and cfg:
            c = dict(cfg)
            c['use_mag'] = True
            ok = push_config(ser, who, c)
            print(f"  {role}: 9-DOF {'applied' if ok else 'PUSH FAILED'} "
                  f"(axis separation {cfg.get('sep_ratio', 0):.1f}x)")
        else:
            off = {'use_mag': False, 'bias': [0, 0, 0], 'scale': [1, 1, 1],
                   'perm': [0, 1, 2], 'sign': [1, 1, 1]}
            push_config(ser, who, off)
            if use_mag and not cfg:
                print(f"  {role}: no calibration on file -> 6-DOF. "
                      "Run --calibrate-mag to enable 9-DOF.")
            else:
                print(f"  {role}: 6-DOF (accel+gyro).")


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
        cal_file=DEFAULT_CAL_FILE, use_mag=True):
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

    # Push stored magnetometer calibration (or force 6-DOF) before collecting.
    print(f"Configuring fusion ({'9-DOF if calibrated' if use_mag else '6-DOF forced'})...")
    apply_startup_config(ser, cal_file, use_mag)

    handler = DropoutHandler()
    q_rel0 = None          # static-zero reference
    flex_axis = None       # learned flexion axis
    zero_done = False
    axis_done = False
    zero_samples = []
    sweep_deltas = []

    t_zero_end = cal_seconds
    t_sweep_end = cal_seconds + sweep_seconds

    # Don't start the calibration clock until real data is flowing, otherwise the
    # zero window can elapse before the user is ready (or hang invisibly on a bad
    # link). wait_for_stream diagnoses no-data / wrong-firmware / dead-slave.
    print(f"Waiting for data on {port} ...")
    wait_for_stream(ser, port)
    start = time.time()
    print(f"Hold FULL EXTENSION ~{cal_seconds:.0f} s (zeroing)...")

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
                # Phase 1: hold full extension, average q_rel.
                if valid:
                    zero_samples.append(q_rel)
                status = 'zeroing'
                if elapsed >= t_zero_end:
                    q_rel0 = average_quaternion(zero_samples)
                    zero_done = True
                    if q_rel0 is not None:
                        print(f"\nZero captured (mean of {len(zero_samples)} samples). "
                              f"Now do slow knee FLEXION reps for ~{sweep_seconds:.0f} s...")
                    else:
                        q_rel0 = (1.0, 0.0, 0.0, 0.0)
                        print("\nWARNING: no valid samples during hold; zero=identity. "
                              f"Now do slow knee FLEXION reps for ~{sweep_seconds:.0f} s...")

            elif not axis_done:
                # Phase 2: functional sweep, learn the flexion axis. Show an
                # unsigned preview of the total delta so the user can see motion.
                status = 'sweep'
                if valid:
                    q_delta = q_mul(q_conj(q_rel0), q_rel)
                    sweep_deltas.append(q_delta)
                    angle = total_angle_deg(q_delta)
                if elapsed >= t_sweep_end:
                    flex_axis = estimate_flexion_axis(sweep_deltas)
                    axis_done = True
                    if flex_axis is not None:
                        print("\nFlexion axis learned. Reporting SIGNED knee angle. "
                              "Ctrl-C to stop.")
                    else:
                        print("\nWARNING: not enough sweep motion to learn an axis; "
                              "reporting UNSIGNED angle magnitude. Ctrl-C to stop.")

            else:
                # Phase 3: run. Signed angle about the learned axis (or unsigned
                # magnitude if the sweep didn't yield a trustworthy axis).
                if valid:
                    q_delta = q_mul(q_conj(q_rel0), q_rel)
                    if flex_axis is not None:
                        angle = signed_flexion_deg(q_delta, flex_axis)
                    else:
                        angle = total_angle_deg(q_delta)
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
    ap.add_argument('--calibrate-mag', choices=['master', 'slave', 'both'],
                    help='run magnetometer calibration for the given board(s), '
                         'save to the cal file, then exit')
    ap.add_argument('--cal-file', default=DEFAULT_CAL_FILE,
                    help='magnetometer calibration JSON (default mag_cal.json)')
    ap.add_argument('--no-mag', action='store_true',
                    help='force 6-DOF (accel+gyro) even if a calibration exists')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()

    if args.selftest or (not args.port):
        _self_test()
        if not args.selftest:
            print("Provide --port to collect live data.")
    elif args.calibrate_mag:
        roles = ['master', 'slave'] if args.calibrate_mag == 'both' else [args.calibrate_mag]
        calibrate_mag(args.port, roles, cal_file=args.cal_file)
    else:
        run(args.port, args.out, raw=args.raw,
            cal_seconds=args.cal_seconds, sweep_seconds=args.sweep_seconds,
            cal_file=args.cal_file, use_mag=not args.no_mag)
