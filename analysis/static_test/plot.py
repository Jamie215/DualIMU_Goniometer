import pandas as pd, numpy as np, glob, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
U='./'
files=sorted(glob.glob(U+'*.csv'),key=lambda f:f[-5])
fig,ax=plt.subplots(3,1,figsize=(11,10))
for i,f in enumerate(files):
    d=pd.read_csv(f); r=d[d.phase=='running']
    ax[0].plot(r.t_session_s,r.knee_angle_deg,lw=.8,label=f'run {i+1}')
    s=d[d.t_session_s>=20]
    ax[1].plot(s.t_session_s,s.knee_angle_deg,lw=.6,label=f'run {i+1}')
    ax[2].plot(s.t_session_s,s.knee_angle_deg.rolling(250,center=True).mean(),label=f'run {i+1} (5 s mean)')
    x=d[(d.t_session_s>=20)&(d.t_session_s<70)]
    print(i+1, x.groupby((x.t_session_s//5)*5).knee_angle_deg.mean().round(3).tolist())
ax[0].set_xlim(8,40); ax[0].set_title('Running phase start (return from 90° sweep)'); 
ax[1].set_title('Steady state, t ≥ 20 s (raw)'); ax[2].set_title('Steady state, 5 s rolling mean')
for a in ax: a.set_ylabel('knee angle (deg)'); a.legend(); a.grid(alpha=.3)
ax[2].set_xlabel('t_session (s)'); plt.tight_layout(); plt.savefig('static_test_plots.png',dpi=110)
