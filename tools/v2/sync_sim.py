import sys, numpy as np
sys.path.insert(0,'repo')
from tools.eval.offline_checker import ApproximateTime
D=np.load('bag.npz')
Rf=D['localization_kinematic_state']; F=D['vehicle_front_bogie_velocity']; B=D['vehicle_rear_bogie_velocity']
def score(sol_arr, sol_stamp, sol_val, ref_arr=None, slop=0.05):
    ref_arr = Rf[:,0] if ref_arr is None else ref_arr
    ev=[(a,0,s,v) for a,s,v in zip(ref_arr,Rf[:,1],Rf[:,9])]+[(a,1,s,v) for a,s,v in zip(sol_arr,sol_stamp,sol_val)]
    ev.sort(key=lambda e:(e[0],e[1]))
    sy=ApproximateTime(100,slop)
    for a,i,s,v in ev: sy.add(i,s,v)
    e=np.array([so[1]-re[1] for re,so in sy.pairs]); return np.sqrt(np.mean(e**2)), len(e), e.mean()
if __name__=='__main__':
    rear={round(s,6):(a,v) for a,s,v in B}
    rows=[(max(fa,rear[round(s,6)][0]),s,(fv+rear[round(s,6)][1])/7.2) for fa,s,fv in F if round(s,6) in rear]
    rows=np.array(rows)
    print('ref arrival - stamp: median',np.median(Rf[:,0]-Rf[:,1]),'p99',np.percentile(Rf[:,0]-Rf[:,1],99))
    print('stamp-order ideal  ', score(rows[:,1]+0.105, rows[:,1]+0.105, rows[:,2]))
    print('realistic arrival  ', score(rows[:,0]+0.001, rows[:,1]+0.105, rows[:,2]))
