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
to nearby metal. Yaw can drift slowly, but the relative delta-angle is largely
immune to it, and the static zero re-references each session.

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


def monitor(port, baud=115200, zero_seconds=1.5):
    """Bring-up diagnostic: zero once on a brief still hold, then print the raw
    RELATIVE angle live. No sweep, no CSV. Lets you see immediately whether the
    sensor data tracks the knee (problem is calibration) or not (problem is
    firmware/link/filter). rtt and valid% localize a bad slave link."""
    if serial is None:
        raise RuntimeError("pyserial not installed. pip install pyserial")
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Monitor on {port}. Hold the leg STILL to zero...")
    wait_for_stream(ser, port)

    zero_samples = []
    start = time.time()
    while time.time() - start < zero_seconds:
        rec = parse_line(ser.readline().decode('ascii', 'ignore'))
        if rec and is_valid(rec):
            zero_samples.append(relative_quaternion(rec['thigh_q'], rec['shank_q']))
    q_rel0 = average_quaternion(zero_samples) or (1.0, 0.0, 0.0, 0.0)
    print(f"Zeroed on {len(zero_samples)} samples. Now MOVE the knee "
          "(should read ~0 at rest, climb with flexion). Ctrl-C to stop.")

    n = nbad = 0
    try:
        while True:
            rec = parse_line(ser.readline().decode('ascii', 'ignore'))
            if rec is None:
                continue
            n += 1
            if not is_valid(rec):
                nbad += 1
                print(f"\r  angle:   --    [shank INVALID]  valid:{100*(n-nbad)/n:4.0f}%  "
                      f"rtt:{rec['rtt']:5d}us   ", end='')
                continue
            q_delta = q_mul(q_conj(q_rel0),
                            relative_quaternion(rec['thigh_q'], rec['shank_q']))
            ang = total_angle_deg(q_delta)
            print(f"\r  relative angle: {ang:6.1f} deg   valid:{100*(n-nbad)/n:4.0f}%  "
                  f"rtt:{rec['rtt']:5d}us   ", end='')
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
                    help='live bring-up check: zero on the first still second, then '
                         'print the raw relative angle continuously (no sweep/CSV)')
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
