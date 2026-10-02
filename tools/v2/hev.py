# events for harness: start (master or both) and RTK master burst snaps, in a given assets param
import sys, numpy as np, pandas as pd
from emu import bursts
D=np.load('bag.npz'); t0=D['localization_kinematic_state'][0,1]
def make(assets, start='master', out='hev.csv', off_m=9.873, off_r=2.563, vertex=False, corr=True):
    r_old=pd.read_csv('repo/railbreak_backup_odometry/assets/ring.csv'); r=pd.read_csv(f'{assets}/ring.csv')
    conv=lambda so: np.interp(np.round(so) if vertex else so, r_old.s_m.values, r.s_m.values)
    Mg=np.load('sensing_gnss_master_fix.npy'); Rv=np.load('sensing_gnss_rover_fix.npy')
    F=D['vehicle_front_bogie_velocity'];B=D['vehicle_rear_bogie_velocity'];C=D['vehicle_driver_position_cmd']
    ev=np.r_[np.c_[F[:,1],np.zeros(len(F)),F[:,2]],np.c_[B[:,1],np.ones(len(B)),B[:,2]],np.c_[C[:,1],2*np.ones(len(C)),C[:,2]]]
    ev=np.c_[ev,np.zeros(len(ev))]
    Mb=bursts(Mg); Rb=bursts(Rv)
    fr=Mb[0][Mb[0][:,0]<=Mb[0][0,0]+3]; s0=np.median(conv(fr[:,1]))+off_m  # median arc ~ median latlon
    if start=='both':
        rr=Rb[0][Rb[0][:,0]<=Rb[0][0,0]+3]; s0=0.5*(s0+np.median(conv(rr[:,1]))-off_r)
    tinit=t0+max(fr[-1,0],0)
    extra=[[tinit-1e-6,4,s0,0]]
    if corr:
      for b in Mb[1:]:
        b=b[b[:,3]>=2]
        if len(b)==0: continue
        i=len(b)//2; tm=b[i,0]; sm=conv(b[i,1])+off_m
        # median arc of the burst at median time; applied 2 s after the last fix
        extra.append([t0+b[-1,0]+2.0,3,np.median(conv(b[:,1])+off_m - 0)+0*sm, t0+np.median(b[:,0])])
    ev=np.r_[ev,np.array(extra)]
    ev=ev[np.lexsort((ev[:,1],ev[:,0]))]
    np.savetxt(out,ev,delimiter=',',fmt=['%.9f','%d','%.6f','%.9f'],header='t,kind,value,aux',comments='')
if __name__=='__main__': make(sys.argv[1], sys.argv[2] if len(sys.argv)>2 else 'master')
