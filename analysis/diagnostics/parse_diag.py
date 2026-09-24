#!/usr/bin/env python3
"""
Turn a knee_diag_<timestamp>.log (saved by knee_gui.py) into tables.

Each '# CDIAG ...' / '# PDIAG ...' line is a once-a-second key=value health
report from the central / peripheral (see the DIAG section of each .ino). This
writes <log>_cdiag.csv and <log>_pdiag.csv and prints a per-minute summary of
the fields that matter most for the sample-rate and dropout questions.

Usage:
  python parse_diag.py knee_diag_20260924_101500.log
"""
import sys

import pandas as pd


def parse(path):
    rows = {'CDIAG': [], 'PDIAG': []}
    with open(path) as f:
        for line in f:
            parts = line.split()
            # "<iso time> # CDIAG k=v k=v ..."
            if len(parts) < 3 or parts[1] != '#' or parts[2] not in rows:
                continue
            rec = {'t_wall': parts[0]}
            for kv in parts[3:]:
                k, _, v = kv.partition('=')
                try:
                    rec[k] = int(v)
                except ValueError:
                    rec[k] = v
            rows[parts[2]].append(rec)
    out = {}
    for kind, r in rows.items():
        df = pd.DataFrame(r)
        if not df.empty:
            df['t_wall'] = pd.to_datetime(df['t_wall'])
            df['t_min'] = (df['t_wall'] - df['t_wall'].iloc[0]).dt.total_seconds() / 60
        out[kind] = df
    return out['CDIAG'], out['PDIAG']


def summary(df, cols, label):
    if df.empty:
        print(f"\n{label}: no lines")
        return
    have = [c for c in cols if c in df]
    g = df.groupby(df['t_min'].astype(int))
    print(f"\n{label} per minute (max of each 1 s report; emits/pkts/frames_ok = mean per second):")
    agg = g[have].max()
    for c in ('emits', 'pkts', 'frames_ok'):
        if c in have:
            agg[c] = g[c].mean().round(1)
    print(agg.to_string())


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    log = sys.argv[1]
    c, p = parse(log)
    stem = log.rsplit('.', 1)[0]
    if not c.empty:
        c.to_csv(stem + '_cdiag.csv', index=False)
    if not p.empty:
        p.to_csv(stem + '_pdiag.csv', index=False)
    summary(c, ['emits', 'emit_avg_us', 'emit_max_us', 'imu_max_us', 'imu_miss',
                'print_avg_us', 'print_max_us', 'diag_write_us', 'loop_gap_max_us',
                'frames_ok', 'cs_fail', 'skipped_bytes', 'rx_max', 'stale'], 'CENTRAL')
    summary(p, ['pkts', 'keepalives', 'rate_reinits', 'stall_reinits', 'begin_fails',
                'last_reinit_ms', 'send_late_max_us', 'loop_max_us', 'gyro_max_dps',
                'acc_min_mg', 'acc_max_mg', 'integ_dps10', 'bias_ok'], 'PERIPHERAL')
