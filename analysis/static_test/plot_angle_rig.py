#!/usr/bin/env python3
"""Plots for the preliminary 45°/90° angle-rig check.

Place knee_45_mount.csv and knee_90_mount.csv next to this script and run it.
Writes angle_rig_plots.png (reported angle vs accelerometer) and
angle_rig_nodes.png (thigh node vs shank node tilt).
Top row: the whole recording (zeroing, calibration sweep, placement on the rig,
hold). Bottom row: the steady part of the hold, zoomed around the nominal angle.
Each panel shows the reported knee angle and, as a filter-free cross-check, the
shank node's tilt computed straight from its raw accelerometer with the same
zero pose and bending direction. Angles are shown as positive magnitudes (the
GUI reports them as negative because of the bending direction calibration
learned).
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from knee_collector_uart import average_gravity, estimate_forward, gravity_in_board, sagittal_inclination

RIGS = [('knee_45_mount.csv', 45, (34, 43)), ('knee_90_mount.csv', 90, (20, 39))]
ANGLE, ACCEL = '#2a78d6', '#eb6834'            # categorical slots 1-2
INK, INK2, GRID, SURF, BAND = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb', '#ecebe7'


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def load(path):
    d = pd.read_csv(path)
    acc = ['shank_ax', 'shank_ay', 'shank_az']
    quat = ['shank_qw', 'shank_qx', 'shank_qy', 'shank_qz']
    z = d[d.phase == 'zeroing'][acc].dropna().values
    sw = d[d.phase == 'sweep'][quat].dropna().values
    # zero pose from the raw accelerometer; bending direction from the calibration
    # sweep, learned the same way the software does (from the fused orientation)
    d0 = average_gravity([tuple(unit(a)) for a in z])
    dq = average_gravity([gravity_in_board(tuple(q)) for q in d[d.phase == 'zeroing'][quat].dropna().values])
    f0 = estimate_forward([gravity_in_board(tuple(q)) for q in sw], dq)
    tilt = [sagittal_inclination(tuple(unit(a)), d0, f0) if np.all(np.isfinite(a)) else np.nan
            for a in d[acc].values]
    d['accel_tilt'] = np.abs(tilt)
    d['angle'] = d.knee_angle_deg.abs()
    return d


plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                     'xtick.color': INK2, 'ytick.color': INK2, 'text.color': INK})
fig, axes = plt.subplots(2, 2, figsize=(11, 7), facecolor=SURF,
                         gridspec_kw={'height_ratios': [1.1, 1]})
for col, (path, nominal, (t0, t1)) in enumerate(RIGS):
    d = load(path)
    run = d[d.phase == 'running']
    steady = d[(d.t_session_s >= t0) & (d.t_session_s < t1)]
    for row, ax in enumerate(axes[:, col]):
        ax.set_facecolor(SURF)
        src = d if row == 0 else steady
        ax.plot(src.t_session_s, src.accel_tilt, color=ACCEL, lw=1.5, label='Raw accelerometer')
        r = run if row == 0 else steady
        ax.plot(r.t_session_s, r.angle, color=ANGLE, lw=2, label='Reported angle')
        ax.axhline(nominal, color=INK2, lw=1, ls='--')
        ax.grid(axis='y', color=GRID, lw=0.8)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        ax.set_xlabel('Time (s)')
        if row == 0:
            ax.axvspan(t0, t1, color=BAND, zorder=0)
            ax.axvspan(0, 8, color=BAND, alpha=0.5, zorder=0)
            ax.text(4, nominal + 6, 'zero +\nsweep', ha='center', va='bottom', color=INK2, fontsize=8)
            ax.text((t0 + t1) / 2, nominal + 6, 'steady\nwindow', ha='center', va='bottom', color=INK2, fontsize=8)
            ax.set_title(f'{nominal}° rig · whole recording', loc='left', fontsize=11)
            ax.set_ylim(-5, nominal + 22)
            ax.legend(frameon=False, loc='lower right', fontsize=9)
        else:
            m, sd = steady.angle.mean(), steady.angle.std()
            am = steady.accel_tilt.mean()
            ax.set_title(f'{nominal}° rig · steady window ({t0}–{t1} s)', loc='left', fontsize=11)
            ax.set_ylim(nominal - 2, nominal + 1.5)
            ax.text(0.02, 0.95, f'reported {m:.2f}° (SD {sd:.3f}°)   accelerometer {am:.2f}°   nominal {nominal}°',
                    transform=ax.transAxes, va='top', color=INK2, fontsize=9,
                    bbox=dict(facecolor=SURF, edgecolor='none', pad=2))
        ax.text(ax.get_xlim()[1], nominal, ' nominal', va='center', ha='left', color=INK2, fontsize=8, clip_on=False)
    axes[0, col].set_xlim(0, d.t_session_s.max())
    axes[1, col].set_xlim(t0, t1)
axes[0, 0].set_ylabel('Angle (°)')
axes[1, 0].set_ylabel('Angle (°)')
fig.tight_layout()
fig.savefig('angle_rig_plots.png', dpi=130, facecolor=SURF)
print('wrote angle_rig_plots.png')


# --------------------------------------------------------------------------- #
# Second figure: thigh node vs shank node tilt. The software treats the thigh
# node as fixed here (it didn't tilt during calibration, so incl_thigh is 0), so
# each node's tilt is taken from its own raw accelerometer, as the angle between
# gravity now and gravity during the zeroing hold (a magnitude: it includes any
# tilt across the bending plane as well as along it).
# --------------------------------------------------------------------------- #
SHANK, THIGH = '#2a78d6', '#1baf7a'           # categorical slots 1 and 3


def tilt_from_zero(d, cols):
    z = unit(d[d.phase == 'zeroing'][cols].dropna().mean().values)
    a = d[cols].values
    n = np.linalg.norm(a, axis=1, keepdims=True)
    cosang = np.clip((a / n) @ z, -1, 1)
    return np.degrees(np.arccos(cosang))


fig, axes = plt.subplots(2, 2, figsize=(11, 7), facecolor=SURF,
                         gridspec_kw={'height_ratios': [1.1, 1]})
for col, (path, nominal, (t0, t1)) in enumerate(RIGS):
    d = pd.read_csv(path)
    d['shank_tilt'] = tilt_from_zero(d, ['shank_ax', 'shank_ay', 'shank_az'])
    d['thigh_tilt'] = tilt_from_zero(d, ['thigh_ax', 'thigh_ay', 'thigh_az'])
    steady = d[(d.t_session_s >= t0) & (d.t_session_s < t1)]
    top, bot = axes[0, col], axes[1, col]
    for ax in (top, bot):
        ax.set_facecolor(SURF)
        ax.axvspan(t0, t1, color=BAND, zorder=0)
        ax.axvspan(0, 8, color=BAND, alpha=0.5, zorder=0)
        ax.grid(axis='y', color=GRID, lw=0.8)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        ax.set_xlim(0, d.t_session_s.max())
        ax.set_xlabel('Time (s)')
    # top: both nodes on one scale
    top.plot(d.t_session_s, d.shank_tilt, color=SHANK, lw=1.8, label='Shank node (angled segment)')
    top.plot(d.t_session_s, d.thigh_tilt, color=THIGH, lw=1.8, label='Thigh node (flat segment)')
    top.axhline(nominal, color=INK2, lw=1, ls='--')
    top.text(top.get_xlim()[1], nominal, ' nominal', va='center', ha='left', color=INK2, fontsize=8, clip_on=False)
    top.text(4, nominal + 6, 'zero +\nsweep', ha='center', va='bottom', color=INK2, fontsize=8)
    top.text((t0 + t1) / 2, nominal + 6, 'steady\nwindow', ha='center', va='bottom', color=INK2, fontsize=8)
    top.set_ylim(-5, nominal + 22)
    top.set_title(f'{nominal}° rig · tilt of each node from its zero pose', loc='left', fontsize=11)
    top.legend(frameon=False, loc='center right', fontsize=9)
    # bottom: thigh node zoomed
    bot.plot(d.t_session_s, d.thigh_tilt, color=THIGH, lw=1.8)
    bot.set_ylim(0, 6)
    bot.set_title(f'{nominal}° rig · thigh node only (zoomed)', loc='left', fontsize=11)
    bot.text(0.02, 0.95, f'steady window: shank {steady.shank_tilt.mean():.2f}°, thigh {steady.thigh_tilt.mean():.2f}°',
             transform=bot.transAxes, va='top', color=INK2, fontsize=9,
             bbox=dict(facecolor=SURF, edgecolor='none', pad=2))
axes[0, 0].set_ylabel('Tilt from zero pose (°)')
axes[1, 0].set_ylabel('Tilt from zero pose (°)')
fig.tight_layout()
fig.savefig('angle_rig_nodes.png', dpi=130, facecolor=SURF)
print('wrote angle_rig_nodes.png')
