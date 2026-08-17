#!/usr/bin/env python3
"""
KNEE ANGLE COLLECTOR  [UART topology, with dropout handling]

Only the MASTER is read via Serial. Its stream carries both segments, merged on the
master's clock. It also handles invalid samples:

  - A sample is INVALID if the shank timestamp is 0 (timeout) OR both shank
    pitch and roll are exactly 0.00 (a packet that carried no real data). Real
    orientations are essentially never exactly 0.00, so exact zeros are a safe
    sentinel for "no valid data".
  - Short dropouts are FORWARD-FILLED: the last good knee angle is held so the
    trace stays continuous (a few ms gap at 104 Hz is imperceptible).
  - A sustained run of dropouts (> MAX_FILL samples) is flagged as MISSING
    rather than holding a stale value for too long.
  - Data-quality stats (valid / filled / missing counts) are tracked and printed,
    and each logged row is flagged so nothing is hidden.

The zero-offset (full-extension baseline) is captured by AVERAGING the shank-thigh
pitch difference over the first ~2 s hold window, rather than trusting a single
sample, so a lone noisy reading can't skew the whole calibration. The roll channels
from both segments are logged alongside pitch for out-of-plane diagnostics (the knee
angle itself is pitch-based).

Master line format:
  D,<t_thigh_us>,<thigh_pitch>,<thigh_roll>,<t_shank_mid_us>,<shank_pitch>,<shank_roll>,<rtt_us>

Usage:
  python knee_collector_uart.py --port /dev/ttyACM0
  python knee_collector_uart.py --selftest
"""

import argparse
import csv
import time

try:
    import serial
except ImportError:
    serial = None

# How many consecutive dropped samples we're willing to forward-fill before we
# stop trusting the held value and mark the stretch as missing. At 104 Hz, 10
# samples is ~100 ms.
MAX_FILL = 10

# How long to hold full extension while averaging the zero-offset baseline.
CAL_SECONDS = 2.0


def parse_line(line):
    p = line.strip().split(',')
    if len(p) != 8 or p[0] != 'D':
        return None
    try:
        return {
            't_thigh': int(p[1]),
            'thigh_pitch': float(p[2]),
            'thigh_roll': float(p[3]),
            't_shank_mid': int(p[4]),
            'shank_pitch': float(p[5]),
            'shank_roll': float(p[6]),
            'rtt': int(p[7]),
        }
    except ValueError:
        return None


def is_valid(rec):
    """Valid only if the shank sample has a real timestamp AND the shank
    orientation isn't exactly zero (the no-data sentinel)."""
    if rec['t_shank_mid'] == 0:
        return False
    if rec['shank_pitch'] == 0.0 and rec['shank_roll'] == 0.0:
        return False
    return True


def knee_angle(thigh_pitch, shank_pitch, zero_offset):
    return (shank_pitch - thigh_pitch) - zero_offset


def compute_zero_offset(cal_samples):
    """Mean shank-thigh pitch difference collected during the hold window.

    Averaging over the whole window makes the full-extension baseline robust to
    a single noisy sample. Returns 0.0 if no valid samples were captured (the
    caller warns in that case)."""
    if not cal_samples:
        return 0.0
    return sum(cal_samples) / len(cal_samples)


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

    assert is_valid({'t_shank_mid': 100, 'shank_pitch': 70.0, 'shank_roll': -2.0})
    assert not is_valid({'t_shank_mid': 0, 'shank_pitch': 70.0, 'shank_roll': -2.0})
    assert not is_valid({'t_shank_mid': 100, 'shank_pitch': 0.0, 'shank_roll': 0.0})
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

    a = knee_angle(10.0, 70.0, 5.0)
    assert abs(a - 55.0) < 1e-9
    print("  angle math OK")

    assert compute_zero_offset([]) == 0.0
    assert abs(compute_zero_offset([5.0, 5.0, 5.0]) - 5.0) < 1e-9
    assert abs(compute_zero_offset([4.0, 6.0]) - 5.0) < 1e-9
    print("  zero-offset averaging OK")

    print(f"  example summary line: {h.summary()}")
    print("Self-test PASSED.\n")


def run(port, out_path, baud=115200):
    if serial is None:
        raise RuntimeError("pyserial not installed. pip install pyserial")

    ser = serial.Serial(port, baud, timeout=1)
    outfile = open(out_path, 'w', newline='')
    writer = csv.writer(outfile)
    writer.writerow(['t_thigh_us', 'thigh_pitch', 'thigh_roll',
                     'shank_pitch', 'shank_roll',
                     'knee_angle_deg', 'status', 'rtt_us'])

    handler = DropoutHandler()
    zero_offset = 0.0
    captured_zero = False
    cal_samples = []
    start = time.time()
    print(f"Collecting. Hold FULL EXTENSION ~{CAL_SECONDS:.0f} s to set zero, "
          "then move. Ctrl-C to stop.")

    try:
        while True:
            line = ser.readline().decode('ascii', 'ignore')
            rec = parse_line(line)
            if rec is None:
                continue

            valid = is_valid(rec)

            # Average the shank-thigh difference over the hold window, then lock
            # in the baseline once the window has elapsed.
            if not captured_zero:
                if valid:
                    cal_samples.append(rec['shank_pitch'] - rec['thigh_pitch'])
                if time.time() - start > CAL_SECONDS:
                    zero_offset = compute_zero_offset(cal_samples)
                    captured_zero = True
                    if cal_samples:
                        print(f"Zero captured: {zero_offset:.2f} deg "
                              f"(mean of {len(cal_samples)} samples)")
                    else:
                        print("WARNING: no valid samples during hold; "
                              "zero_offset=0.00 deg")

            angle = None
            if valid:
                angle = knee_angle(rec['thigh_pitch'], rec['shank_pitch'], zero_offset)

            out_angle, status = handler.process(valid, angle)

            writer.writerow([
                rec['t_thigh'],
                f"{rec['thigh_pitch']:.2f}",
                f"{rec['thigh_roll']:.2f}",
                f"{rec['shank_pitch']:.2f}" if valid else '',
                f"{rec['shank_roll']:.2f}" if valid else '',
                f"{out_angle:.2f}" if out_angle is not None else '',
                status,
                rec['rtt'],
            ])

            disp = f"{out_angle:6.1f}" if out_angle is not None else "  --  "
            print(f"\rknee: {disp} deg  [{status:7s}]  rtt:{rec['rtt']:5d}us   ", end='')
    except KeyboardInterrupt:
        print("\nStopping.")
        print("Data quality: " + handler.summary())
    finally:
        outfile.close()
        ser.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--port')
    ap.add_argument('--out', default='knee_log.csv')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()

    if args.selftest or not args.port:
        _self_test()
        if not args.selftest:
            print("Provide --port to collect live data.")
    else:
        run(args.port, args.out)
