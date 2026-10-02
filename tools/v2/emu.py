from geo import *
r=pd.read_csv('repo/railbreak_backup_odometry/assets/ring.csv'); st_true=np.load('strue.npy')
s_old,X,Y,H=ring()
Rf=D['localization_kinematic_state']; t0=Rf[0,1]; tr=Rf[:,1]-t0
tr_,sr=np.load('sr.npy').T
F=D['vehicle_front_bogie_velocity']; B=D['vehicle_rear_bogie_velocity']; C=D['vehicle_driver_position_cmd']
tf=F[:,1]-t0; vm=(F[:,2]+np.interp(F[:,1],B[:,1],B[:,2]))/7.2
W=np.r_[0,np.cumsum(np.diff(tf)*(vm[1:]+vm[:-1])/2)]
kap=np.load('kappa.npy')
Mg=np.load('sensing_gnss_master_fix.npy'); Rv=np.load('sensing_gnss_rover_fix.npy')
def bursts(G,gap=2.0):
    g=np.r_[0,np.where(np.diff(G[:,0])>gap)[0]+1,len(G)]
    return [G[a:b] for a,b in zip(g[:-1],g[1:])]
def run(Pk=0.0008,vertex=False,rtk_only=True,param='old',k=0.999464,off_m=9.873,off_r=2.563,start='master',win=3.0,corr=True,lag_p=0.0,kappa_c=0.0,online_k=False,verbose=False,fade=0.0,start_win_all=False):
    S = s_old if param=='old' else st_true
    conv=lambda so: np.interp(so,s_old,S)   # old-param s -> param s
    # output stamps: all bogie + cmd stamps
    tout=np.unique(np.r_[tf, C[:,1]-t0]); tout=tout[tout>=tf[0]]
    # wheel odometer with curvature correction in param space: ds = k*(1+c|kappa(s)|) dW ; do numerically on wheel grid
    q=(lambda a: np.round(a)) if vertex else (lambda a: a)
    Mb=bursts(Mg); Rb=bursts(Rv)
    # start
    first=Mb[0]; fr=first[first[:,0]<=first[0,0]+win]
    sm0=conv(q(np.median(fr[:,1])))+off_m
    if start=='both':
        rr=Rb[0]; rr=rr[rr[:,0]<=rr[0,0]+win]; sr0=conv(q(np.median(rr[:,1])))-off_r; s0=(sm0+sr0)/2
    else: s0=sm0
    # anchors list (time, s_base meas) for later bursts
    anchors=[]
    for b in Mb[1:]:
        if rtk_only: b=b[b[:,3]>=2]
        if len(b)==0: continue
        anchors.append((b[0,0],b[:,0],conv(q(b[:,1]))+off_m))
    # integrate along wheel samples
    s=np.zeros(len(tf)); s[0]=s0 - k*0  # W at tf[0]=0
    kk=k; ai=0; last_anchor=(tf[0],s0,W[0]); pending=0.0
    kap_s=lambda x: np.interp(x,kap[:,0] if param!='old' else np.interp(kap[:,0],st_true,s_old),np.abs(kap[:,1]))
    # anchor at start: time of first sample assumed tf0; s0 measured during window while (likely) stationary
    shift=0.0
    for i in range(1,len(tf)):
        dW=W[i]-W[i-1]
        ds=kk*dW*(1+kappa_c*kap_s(s[i-1]))
        step=0.0
        if fade>0 and pending!=0.0:
            step=np.clip(pending, -abs(pending)*0+-1e9, 1e9)
            q=min(1.0,(tf[i]-tf[i-1])/fade); step=pending*q if abs(pending)>1e-6 else pending
            pending-=step
        s[i]=s[i-1]+ds+step
        if corr and ai<len(anchors) and tf[i]>=anchors[ai][1][-1]:
            tb,tg,sg=anchors[ai]
            est=np.interp(tg,tf[:i+1],s[:i+1])
            # pending fade not included in est; fine
            innov=np.median(sg-est)
            if online_k:
                tm=np.median(tg); sgm=np.median(sg-np.interp(tg,tf,W)*0)  # placeholder
                # GNSS-only scale: arc between two RTK bursts / wheel integral between them
                Wm=np.interp(tg,tf,W); sg0=np.median(sg-Wm*kk)+np.median(Wm)*kk
                Wmed=np.median(Wm)
                dWw=Wmed-last_anchor[2]
                if dWw>online_k:
                    km=(sg0-last_anchor[1])/dWw
                    sm=0.4*np.sqrt(2)/dWw; sp=Pk
                    w=sp**2/(sp**2+sm**2); kk=kk+w*(km-kk)
            
            if fade>0: pending+=innov
            else: s[i]+=innov
            if verbose: print(f'  anchor t={tb:.0f} innov {innov:+.2f} k={kk:.5f}')
            Wm_=np.interp(tg,tf,W); last_anchor=(np.median(tg),np.median(sg-Wm_*kk)+np.median(Wm_)*kk,np.median(Wm_))
            ai+=1
    so=np.interp(tout-lag_p,tf,s)
    # to MGRS: param s -> old s -> X,Y,H
    sold=np.interp(so,S,s_old)
    x=np.interp(sold,s_old,X); y=np.interp(sold,s_old,Y); z=np.interp(sold,s_old,H)-3.0
    return tout,x,y,z,so
def score(tout,x,y,z):
    # nearest output for each ref within 0.05
    idx=np.searchsorted(tout,tr); idx=np.clip(idx,1,len(tout)-1)
    j=np.where(np.abs(tout[idx]-tr)<np.abs(tout[idx-1]-tr),idx,idx-1)
    ok=np.abs(tout[j]-tr)<=0.05
    ex=x[j]-Rf[:,2]; ey=y[j]-Rf[:,3]; ez=z[j]-Rf[:,4]; e=np.sqrt(ex**2+ey**2+ez**2)
    e=e[ok]; return np.sqrt(np.mean(e**2)), e.max(), ok.sum(), np.sqrt(np.mean(ez[ok]**2)), e, tr[ok]
