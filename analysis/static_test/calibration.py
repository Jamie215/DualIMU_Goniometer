import sys; import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import pandas as pd, numpy as np, glob, math
from knee_collector_uart import gravity_in_board, average_gravity, estimate_forward, sagittal_inclination, _incl_from_zero
U='./'
files=sorted(glob.glob(U+'*.csv'),key=lambda f:f[-5])
def G(r,seg): return gravity_in_board((r[seg+'_qw'],r[seg+'_qx'],r[seg+'_qy'],r[seg+'_qz']))
def ang(a,b): return math.degrees(math.acos(max(-1,min(1,np.dot(a,b)))))
D=[];F=[];DT=[];ENDs=[];ENDt=[]
for i,f in enumerate(files):
    d=pd.read_csv(f)
    z=d[d.phase=='zeroing']; sw=d[d.phase=='sweep']
    dt=average_gravity([G(r,'thigh') for _,r in z.iterrows()]); ds=average_gravity([G(r,'shank') for _,r in z.iterrows()])
    gs=[G(r,'shank') for _,r in sw.iterrows()]; fs=estimate_forward(gs,ds)
    ft=estimate_forward([G(r,'thigh') for _,r in sw.iterrows()],dt)
    sweep=[_incl_from_zero(g,ds) for g in gs]
    thsw=[_incl_from_zero(G(r,'thigh'),dt) for _,r in sw.iterrows()]
    run=d[d.phase=='running']
    thrun=[_incl_from_zero(G(r,'thigh'),dt) for _,r in run.iloc[::10].iterrows()]
    print(f'run{i+1}: d_shank={np.round(ds,4)} d_thigh={np.round(dt,4)} f_shank={np.round(fs,3) if fs else None} f_thigh={ft}')
    print(f'   sweep max shank tilt {max(sweep):.1f} deg; thigh max tilt during sweep {max(thsw):.3f}; thigh tilt during run max {max(thrun):.3f} mean {np.mean(thrun):.3f}')
    # zero pose noise: shank tilt SD within zeroing
    zs=[_incl_from_zero(G(r,'shank'),ds) for _,r in z.iterrows()]; print(f'   zeroing window: shank dev from d mean {np.mean(zs):.3f} max {max(zs):.3f}')
    s=d[d.t_session_s>=50]; p=np.polyfit(s.t_session_s,s.knee_angle_deg,1)[0]*60; print(f'   slope t>=50: {p:.4f} deg/min')
    D.append(ds);DT.append(dt);F.append(fs)
    e=d.tail(250); ENDs.append(average_gravity([G(r,'shank') for _,r in e.iterrows()])); ENDt.append(average_gravity([G(r,'thigh') for _,r in e.iterrows()]))
for i in range(3):
  for j in range(i+1,3):
    print(f'run{i+1} vs run{j+1}: d_shank diff {ang(D[i],D[j]):.3f} deg, d_thigh diff {ang(DT[i],DT[j]):.3f}, f_shank diff {ang(F[i],F[j]):.2f}')
for i in range(2):
    print(f'end run{i+1} -> zero run{i+2}: shank {ang(ENDs[i],D[i+1]):.3f} deg, thigh {ang(ENDt[i],DT[i+1]):.3f}')
    # signed: sagittal incl of next zero wrt this run's frame
    print('   signed shank incl of next zero in run frame', round(sagittal_inclination(D[i+1],D[i],F[i]),3), ' end-of-run incl', round(sagittal_inclination(ENDs[i],D[i],F[i]),3))

# --- desk (thigh) node stability, relative to its own zero ---
for i,f in enumerate(files):
    d=pd.read_csv(f).drop_duplicates('t_thigh_us')
    g=np.array([gravity_in_board(q) for q in d[['thigh_qw','thigh_qx','thigh_qy','thigh_qz']].values])
    z=average_gravity(list(g[(d.phase=='zeroing').values]))
    # signed components along board x,y deviation from zero (small-angle, deg)
    dev=np.degrees(g-np.array(z))[:, :2]
    tilt=np.array([_incl_from_zero(x,z) for x in g]); t=d.t_session_s.values
    m=t>=20
    print(f'run{i+1}: thigh tilt t>=20 mean {tilt[m].mean():.3f} sd {tilt[m].std():.3f} max {tilt[m].max():.3f} at t={t[m][tilt[m].argmax()]:.1f}; x-comp sd {dev[m,0].std():.4f} slope {np.polyfit(t[m],dev[m,0],1)[0]*60:.4f}/min, y-comp sd {dev[m,1].std():.4f} slope {np.polyfit(t[m],dev[m,1],1)[0]*60:.4f}/min')
    for a,b in [(0,2),(2,8),(8,12),(12,20)]:
        k=(t>=a)&(t<b); print(f'    t {a}-{b}: thigh tilt max {tilt[k].max():.3f}')
