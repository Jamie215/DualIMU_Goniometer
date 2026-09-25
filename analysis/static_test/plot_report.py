#!/usr/bin/env python3
"""Figure 1 of Static_Stability_Test_Report.docx: change in knee angle during each static hold.

Place knee_static_1.csv ... knee_static_6.csv next to this script and run it.
Each trace is the 5 s rolling mean of the angle, relative to the mean of the
first 10 s of the hold (t >= 20 s), so every run starts at 0 and the y-axis
shows how much the reading moved while nothing was moving.
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd

SERIES = ['#2a78d6', '#eb6834', '#1baf7a']      # categorical slots 1-3, fixed order
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
SESSIONS = [('Session 1 · flat desk', [1, 2, 3]),
            ('Session 2 · raised rig', [4, 5, 6])]

plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                     'xtick.color': INK2, 'ytick.color': INK2, 'text.color': INK})
fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True, facecolor=SURF)
for ax, (title, runs) in zip(axes, SESSIONS):
    ax.set_facecolor(SURF)
    ends = []
    for color, run in zip(SERIES, runs):
        d = pd.read_csv(f'knee_static_{run}.csv').drop_duplicates('t_thigh_us')
        s = d[d.t_session_s >= 20]
        base = s.knee_angle_deg[s.t_session_s < 30].mean()
        y = (s.knee_angle_deg - base).rolling(200, center=True, min_periods=50).mean()
        t = (s.t_session_s - 20) / 60
        ax.plot(t, y, color=color, lw=2, label=f'Run {run}')
        ends.append([y.dropna().iloc[-1], t.iloc[-1], run])
    # direct labels at line ends, nudged apart so close finishes don't collide
    ends.sort()
    for i in range(1, len(ends)):
        ends[i][0] = max(ends[i][0], ends[i - 1][0] + 0.045)
    for yv, tv, run in ends:
        ax.annotate(f'Run {run}', (tv, yv), xytext=(6, 0), textcoords='offset points',
                    va='center', color=INK2, fontsize=9)
    ax.axhline(0, color=INK2, lw=0.8)
    ax.set_title(title, loc='left', fontsize=11, color=INK)
    ax.set_xlabel('Time into hold (min)')
    ax.set_xlim(0, 5.9)
    ax.grid(axis='y', color=GRID, lw=0.8)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False, loc='upper left', fontsize=9)
axes[0].set_ylabel('Change in angle (°)')
axes[0].set_ylim(-0.2, 0.6)
fig.tight_layout()
fig.savefig('report_stability.png', dpi=130, facecolor=SURF)
