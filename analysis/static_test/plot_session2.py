import sys,os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import pandas as pd, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from knee_collector_uart import gravity_in_board, average_gravity, _incl_from_zero
fig,ax=plt.subplots(3,2,figsize=(14,10))
for k,i in enumerate([4,5,6]):
    d=pd.read_csv(f'knee_static_{i}.csv')
    ax[k,0].plot(d.t_session_s,d.knee_angle_deg,lw=.6); ax[k,0].set_title(f'run {i}: knee angle (full)'); ax[k,0].set_ylim(-20,20)
    g=np.array([gravity_in_board(q) for q in d[['thigh_qw','thigh_qx','thigh_qy','thigh_qz']].values])
    z=np.array(average_gravity(list(g[(d.phase=='zeroing').values])))
    dv=np.degrees(g-z)
    ax[k,1].plot(d.t_session_s,dv[:,0],lw=.6,label='desk node x'); ax[k,1].plot(d.t_session_s,dv[:,1],lw=.6,label='desk node y')
    s=d[d.t_session_s>=20]; ax[k,1].plot(s.t_session_s,s.knee_angle_deg-s.knee_angle_deg.iloc[:500].mean(),lw=.6,label='knee angle − start')
    ax[k,1].set_ylim(-1,1); ax[k,1].legend(fontsize=7); ax[k,1].set_title(f'run {i}: desk node tilt from zero (deg) & angle change')
    for a in ax[k]: a.grid(alpha=.3)
plt.tight_layout(); plt.savefig('session2_plots.png',dpi=90)
for i in [4,5,6]:
    d=pd.read_csv(f'knee_static_{i}.csv')
    print(i, d.t_wall_iso.iloc[0], d.t_wall_iso.iloc[-1], 'board t0 s', d.t_thigh_us.iloc[0]/1e6)
    for T in [6,8,9,10,12,15,20]:
        x=d[(d.t_session_s>=T)&(d.t_session_s<T+1)].knee_angle_deg; print(f'   t={T} {x.mean():.2f}')
