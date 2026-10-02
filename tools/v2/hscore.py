import numpy as np, pandas as pd, emu
from emu import score, ring
s_old,X,Y,H=ring()
def evaluate(assets, out_csv, lag_p=0.0):
    r=pd.read_csv(f'{assets}/ring.csv'); o=pd.read_csv(out_csv); t0=emu.t0
    o=o[o.kind!=3]; t=o.t.values-t0; s=o.s.values-o.v.values*lag_p
    L=float(open(f'{assets}/meta.yaml').read().split('ring_len_m:')[1].split()[0])
    s=np.mod(s,L)
    sold=np.interp(s,r.s_m.values,s_old)
    x=np.interp(sold,s_old,X); y=np.interp(sold,s_old,Y); z=np.interp(s,r.s_m.values,r.h_m.values)-3.0
    # base_link offset: harness s is master-antenna arc? -> we fed base_link arcs (off_m added), so no extra
    rm,mx,n,zr,e,te=score(t,x,y,z); return rm,mx,zr,o.k.values[-1]
