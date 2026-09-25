import pandas as pd, numpy as np, glob
U='./'
files=sorted(glob.glob(U+'*.csv'),key=lambda f:f[-5])
for f in files:
    d=pd.read_csv(f); name=f.split('-')[-1]
    print('=====',name, 'rows',len(d))
    print(d.groupby('phase')['t_session_s'].agg(['min','max','count']))
    print('status counts',d.status.value_counts().to_dict())
    r=d[d.phase=='running']
    print('running start',r.t_session_s.min())
    # show first 20s of angle
    for T in [8,9,10,12,15,20,25]:
        x=d[(d.t_session_s>=T)&(d.t_session_s<T+1)].knee_angle_deg
        print(f'  t={T}: mean {x.mean():.2f} sd {x.std():.3f}')
    s=d[d.t_session_s>=20].copy()
    a=s.knee_angle_deg.dropna(); t=s.loc[a.index,'t_session_s']
    print('incl_thigh unique', s.incl_thigh_deg.dropna().unique()[:5])
    print(f'N {len(a)} mean {a.mean():.3f} sd {a.std():.3f} min {a.min():.2f} max {a.max():.2f} p2.5 {a.quantile(.025):.2f} p97.5 {a.quantile(.975):.2f}')
    sl=np.polyfit(t,a,1)[0]; print(f'drift slope {sl*60:.4f} deg/min, total {sl*(t.max()-t.min()):.3f} deg')
    det=a-np.polyval(np.polyfit(t,a,1),t); print(f'detrended sd {det.std():.3f}')
    dd=np.diff(a.values); print(f'sample-to-sample diff sd {dd.std():.4f}; frac zero diff {(dd==0).mean():.2f}')
    # 30s block means
    b=((t-20)//30).astype(int); print('30s block means', a.groupby(b).mean().round(2).tolist())
    print('30s block sd', a.groupby(b).std().round(3).tolist())
    # yaw drift from quats
    def yaw(w,x,y,z): return np.degrees(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    yt=np.unwrap(np.radians(yaw(s.thigh_qw,s.thigh_qx,s.thigh_qy,s.thigh_qz))); ys=np.unwrap(np.radians(yaw(s.shank_qw,s.shank_qx,s.shank_qy,s.shank_qz)))
    tt=s.t_session_s
    print(f'yaw drift thigh {np.degrees(np.polyfit(tt,yt,1)[0])*60:.2f} deg/min, shank {np.degrees(np.polyfit(tt,ys,1)[0])*60:.2f} deg/min')
    dt=np.diff(s.t_session_s); print(f'dt median {np.median(dt)*1000:.1f}ms, max {dt.max()*1000:.0f}ms, >40ms: {(dt>0.04).sum()}')
    tu=s.t_thigh_us.values; print('repeated t_thigh_us frac', (np.diff(tu)==0).mean().round(3))
    print('rtt median',s.rtt_us.median(),'max',s.rtt_us.max(), 'p99', s.rtt_us.quantile(.99))
    print('resolution: unique angle vals', a.nunique())
import pandas as pd, numpy as np, glob
U='./'
files=sorted(glob.glob(U+'*.csv'),key=lambda f:f[-5])
for f in files:
    d=pd.read_csv(f); s=d[d.t_session_s>=20]
    print('==',f[-5])
    r=s.rtt_us; print('rtt buckets', pd.cut(r,[-1,100,1000,5000,15000,1e6]).value_counts().sort_index().to_dict())
    tu=s.t_thigh_us.values; nd=np.diff(tu); nd=nd[nd>0]
    print('thigh source period us median',np.median(nd), 'unique source samples/s', len(np.unique(tu))/(s.t_session_s.max()-s.t_session_s.min()))
    # clock comparison
    wall=pd.to_datetime(s.t_wall_iso); el=(wall.iloc[-1]-wall.iloc[0]).total_seconds()
    print('wall elapsed',el,'session elapsed',s.t_session_s.iloc[-1]-s.t_session_s.iloc[0],'board elapsed',(tu[-1]-tu[0])/1e6)
    # dedupe to source samples & allan dev
    u=s.drop_duplicates('t_thigh_us'); a=u.knee_angle_deg.values; fs=len(u)/(u.t_session_s.max()-u.t_session_s.min())
    out=[]
    for tau in [0.1,1,10,30,60]:
        m=int(round(tau*fs)); n=len(a)//m
        if n<3: continue
        bm=a[:n*m].reshape(n,m).mean(1); out.append((tau, np.sqrt(0.5*np.mean(np.diff(bm)**2))))
    print('Allan dev', [(t, round(v,4)) for t,v in out])
    # settling: first time after 10s angle stays within 0.05 of final mean
    fin=s.knee_angle_deg.mean(); x=d[d.t_session_s>=8]
    bad=x[(x.knee_angle_deg-fin).abs()>0.05]
    print('settled within 0.05 deg of final mean after t=',bad.t_session_s.max())
    bad=x[(x.knee_angle_deg-fin).abs()>0.1]; print('within 0.1 after', bad.t_session_s.max())
    # sweep peak angle: incl from zero not logged; use shank quats vs zero
